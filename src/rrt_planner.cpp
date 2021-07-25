#include "rrt_planners/rrt_planner.hpp"


namespace PathPlanners
{
//*****************************************************************
// 				ThetaStar Algorithm Class Definitions
//*****************************************************************

// Default constructor
RRTPlanner::RRTPlanner()
{
}

// Constructor with arguments
RRTPlanner::RRTPlanner(std::string plannerType, std::string frame_id, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_, 
			float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, ros::NodeHandlePtr nh_, double goal_gap_m_
			, double distance_obstacle_ugv_, double distance_obstacle_uav_)
{
	// Call to initialization
	init(plannerType, frame_id, ws_x_max_, ws_y_max_, ws_z_max_, ws_x_min_, ws_y_min_, ws_z_min_, step_, h_inflation_, v_inflation_, goal_weight_, z_weight_cost_, z_not_inflate_, nh_ ,goal_gap_m_, debug_rrt, distance_obstacle_ugv_, distance_obstacle_uav_);
}

// Initialization: creates the occupancy matrix (discrete nodes) from the bounding box sizes, resolution, inflation and optimization arguments
void RRTPlanner::init(std::string plannerType, std::string frame_id_, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_,
				   float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, ros::NodeHandlePtr nh_, 
				   double goal_gap_m_, bool debug_rrt_, double distance_obstacle_ugv_, double distance_obstacle_uav_)
{
	// Pointer to the nodeHandler
	nh = nh_;

	planner_type = plannerType;
	// Not target initially
	disc_initial = NULL;
	disc_final = NULL;
	minR=0;
	// by default ~not timeout
	timeout = 100;
	// Init asymetric and inflated occupancy matrix
	ws_x_max = ((ws_x_max_ / step_) + 1);
	ws_y_max = ((ws_y_max_ / step_) + 1);
	ws_z_max = ((ws_z_max_ / step_) + 1);
	ws_x_min = ((ws_x_min_ / step_) - 1);
	ws_y_min = ((ws_y_min_ / step_) - 1);
	ws_z_min = ((ws_z_min_ / step_) - 1);

	frame_id = frame_id_;
	step = step_;
	step_inv = 1.0 / step_;
	h_inflation = (int)(h_inflation_ / step_);
	v_inflation = (int)(v_inflation_ / step_);
	ws_x_max_inflated = (ws_x_max + 2 * h_inflation);
	ws_y_max_inflated = (ws_y_max + 2 * h_inflation);
	ws_z_max_inflated = (ws_z_max + 2 * v_inflation);
	ws_x_min_inflated = (ws_x_min - 2 * h_inflation);
	ws_y_min_inflated = (ws_y_min - 2 * h_inflation);
	ws_z_min_inflated = (ws_z_min - 2 * v_inflation);
	matrix_size = (abs(ws_x_max_inflated) - ws_x_min_inflated + 1) * (abs(ws_y_max_inflated) - ws_y_min_inflated + 1) * (abs(ws_z_max_inflated) - ws_z_min_inflated + 1);
	discrete_world.resize(matrix_size);
	printf("Global Planner %s Node : Occupancy Matrix has %d nodes [%lu MB]\n", plannerType.c_str(), matrix_size, (uint_fast32_t)(matrix_size * sizeof(RRTNode)) / (1024 * 1024));
	Lx = ws_x_max_inflated - ws_x_min_inflated + 1;
	Ly = ws_y_max_inflated - ws_y_min_inflated + 1;
	Lz = ws_z_max_inflated - ws_z_min_inflated + 1;
	Lx_inv = 1.0 / Lx;
	Ly_inv = 1.0 / Ly;
	Lz_inv = 1.0 / Lz;
	// Optimization
	goal_weight = goal_weight_;
	z_weight_cost = z_weight_cost_;
	// Floor condition
	z_not_inflate = z_not_inflate_;

	goal_gap_m = goal_gap_m_;

	debug_rrt = debug_rrt_;
	
	markers_debug = false;
	nodes_marker_debug = true;

	distance_obstacle_ugv = distance_obstacle_ugv_;
	distance_obstacle_uav = distance_obstacle_uav_;

	lines_ugv_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("path_ugv_rrt_star", 2, true);
	lines_uav_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("path_uav_rrt_star", 2, true);
	occupancy_marker_pub_ = nh->advertise<PointCloud>("vis_marker_occupancy", 1, true);
	goal_point_pub_ = nh->advertise<visualization_msgs::Marker>("goal_point", 1, true);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marsupial", 1000, true);
	rand_point_pub_ = nh->advertise<visualization_msgs::MarkerArray>("rand_point", 2, true);

	if (nodes_marker_debug){
		tree_rrt_star_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("tree_rrt_star_ugv", 2, true);
		tree_rrt_star_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("tree_rrt_star_uav", 2, true);
		take_off_nodes_pub_ = nh->advertise<visualization_msgs::MarkerArray>("take_off_nodes_rrt_star", 2, true);
	}
	if (markers_debug){
		one_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("one_catenaty", 1000, true);
		points_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("points_marker", 10, true);
		new_point_pub_ = nh->advertise<visualization_msgs::MarkerArray>("new_point", 2, true);
		nearest_point_pub_ = nh->advertise<visualization_msgs::MarkerArray>("nearest_point", 2, true);
		reel1_point_pub_ = nh->advertise<visualization_msgs::Marker>("reel1_point", 1, true);
		reel2_point_pub_ = nh->advertise<visualization_msgs::Marker>("reel2_point", 1, true);
		all_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("all_catenaries_rrt", 10000, true);
	}
	
}

RRTPlanner::~RRTPlanner()
{
//   clearStatus(); 
}

int RRTPlanner::computeTreeCoupled()
{
    std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;
	printf("RRTPlanner::computeTreeCoupled -->  STARTING --> star_point_ugv[%.2f %.2f %.2f]  goal_point=[%.2f %.2f %.2f] \n\n",
	initial_position_ugv.x, initial_position_ugv.y, initial_position_ugv.z, final_position.x, final_position.y, final_position.z);    

	v_nodes_kdtree.clear();
	v_ugv_nodes_kdtree.clear();

	saveNode(disc_initial,true);

	count_graph = 0;
	if(nodes_marker_debug)
		rrtgm.getGraphMarker(disc_initial, count_graph, tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_);			// Incremental algorithm --> the graph is generated in each calculation
	if (markers_debug)
		rrtgm.getTakeOffNodesMarker(take_off_nodes, take_off_nodes_pub_);

	updateKdtreeNode(*disc_initial);

 	double ret_val = -1.0; 
    int count = 0; 
	
    while (count < n_iter)  // n_iter Max. number of nodes to expand for each round
    {
	  	count++;
		// printf("__________________________________  RRTPlanner::computeTreeCoupled: STARTING WHILE LOOP[%i]  _________________________________\n",count);
		RRTNode q_rand = getRandomNode();	// get a vector with one element in case of coupled configuration
		
		if (debug_rrt)
			printf("q_rand = [%f %f %f / %f %f %f]\n",q_rand.point.x*step,q_rand.point.y*step,q_rand.point.z*step,q_rand.point_uav.x*step,q_rand.point_uav.y*step,q_rand.point_uav.z*step);

		extendGraph(q_rand);
		if ((take_off_nodes.size() > 0) && got_to_goal>0){
			printf("RRTPlanner::computeTreeCoupled -->  breaking while for in iteration=%i",count);    
			break;
		}
	}

	if (take_off_nodes.size() > 0){
		rrt_path = getPath(); 
		printf("RRTPlanner::computeTreeCoupled -->  finded path for Coupled Marsupial Configuration-->  path size: %lu , number iteration: %i , take off nodes: %lu \n\n",
		rrt_path.size(), count + 1, take_off_nodes.size()); 
		int i_=0;   
		for (auto pt_: rrt_path){
			i_++;
			printf("point_path[%i/%lu] : ugv=[%f %f %f]  uav=[%f %f %f]  cost=[%f]\n", i_, rrt_path.size()
			,pt_->point.x*step, pt_->point.y*step, pt_->point.z*step, pt_->point_uav.x*step, pt_->point_uav.y*step, pt_->point_uav.z*step, pt_->cost);
		}
		rrtgm.getPathMarker(rrt_path, lines_ugv_marker_pub_, lines_uav_marker_pub_);
		ret_val = rrt_path.size();
	}
	else
		printf("RRTPlanner::computeTreeCoupled -->  could't find path for Coupled Marsupial Configuration-->  number iteration: %lu \n\n", nodes_tree.size());    

  	std::cout << "Explored Graph Nodes Numbers: " << nodes_tree.size() <<std::endl;
  	std::cout << "Explored Graph Nodes Numbers to Take Off: " << take_off_nodes.size() <<std::endl;
	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;

  return ret_val; 
}

int RRTPlanner::computeTreesIndependent()
{
	clearStatus();
	rrtgm.clearMarkers(tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_, take_off_nodes_pub_, lines_ugv_marker_pub_, lines_uav_marker_pub_);

	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;
	printf("RRTPlanner::computeTreesIndependent -->  STARTING --> star_point_ugv[%.2f %.2f %.2f]  goal_point=[%.2f %.2f %.2f] \n", initial_position_ugv.x, initial_position_ugv.y, initial_position_ugv.z, final_position.x, final_position.y, final_position.z);  

	publishOccupationMarkersMap();
	float yaw_ = getYawFromQuaternion(*disc_initial,false);
	setInitialCostGoal(disc_final);
	
	// printf("disc_initial [%f %f %f /%f %f %f] yaw=[%f] \n",disc_initial->point.x*step,disc_initial->point.y*step,disc_initial->point.z*step, disc_initial->point_uav.x*step,disc_initial->point_uav.y*step,disc_initial->point_uav.z*step, yaw_);
	
	if(!saveNode(disc_initial,true)){
		ROS_ERROR("RRTPlanner::computeTreesIndependent --> Not posible to get catenary in initial node");
		return 0;
	}
	
	rrtgm.goalPointMarker(final_position, goal_point_pub_);
	
	count_graph = 0;
	if(nodes_marker_debug)
		rrtgm.getGraphMarker(disc_initial, count_graph, tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_);

	updateKdtreeNode(*disc_initial);
	updateKdtreeUGV(*disc_initial);

 	double ret_val = -1.0; 
    int count = 0; 
	int count_loop = 0; 
    
	while (count < n_iter) { // n_iter Max. number of nodes to expand for each round
      	
		printf("\t\t-----  Planner (%s) :: computeTreeIndependent: iter=[%i/%i] , loop=[%i/%i] -----\n",planner_type.c_str(), count+1, n_iter, count_loop+1, n_loop);
		RRTNode q_rand;
		
		if ((count%samp_goal_rate)!=0){
			q_rand = getRandomNode();	
		}
		else{
			q_rand = getRandomNode(true);	
		}
		if (debug_rrt)
			printf(" q_rand = [%f %f %f / %f %f %f] \n",q_rand.point.x*step,q_rand.point.y*step,q_rand.point.z*step,q_rand.point_uav.x*step,q_rand.point_uav.y*step,q_rand.point_uav.z*step);
		
		if (markers_debug)
			rrtgm.randNodeMarker(q_rand, rand_point_pub_);
		rrtgm.randNodeMarker(q_rand, rand_point_pub_);

		
		extendGraph(q_rand);

		count++;

		if ( ( (got_to_goal>0) && (planner_type == "rrt") ) || ( (got_to_goal>0) && (planner_type == "rrt_star") && (count == n_iter) ) ){
			printf("\nRRTStar::computeTreesIndependent -->  finded goal for Coupled Marsupial Configuration.\n")	; 
			rrt_path = getPath(); 
			printf("RRTPlanner::computeTreesIndependent -->  finded path for Coupled Marsupial Configuration--> (path size: %lu , number iteration: %i) : \n",rrt_path.size(), count + 1); 
			if (planner_type == "rrt_star")
				printf("RRTPlanner::computeTreesIndependent -->  number of goals finded: %i\n",got_to_goal); 
			int i_=0;   
			printf("\tPrinting the Path Nodes obteinded through planner (%s) : \n",planner_type.c_str());
			for (auto pt_: rrt_path){
				i_++;
				printf("\tpoint_path[%i/%lu] :  ugv=[%.4f %.4f %.4f / %.4f %.4f %.4f %.4f]  uav=[%.4f %.4f %.4f / %.4f %.4f %.4f %.4f]  length_catenary=%f    cost=%f\n", i_, rrt_path.size(),
				pt_->point.x*step, pt_->point.y*step, pt_->point.z*step, pt_->rot_ugv.x, pt_->rot_ugv.y, pt_->rot_ugv.z, pt_->rot_ugv.w, 
				pt_->point_uav.x*step, pt_->point_uav.y*step, pt_->point_uav.z*step, pt_->rot_uav.x, pt_->rot_uav.y, pt_->rot_uav.z, pt_->rot_uav.w, pt_->length_cat, pt_->cost);
			}
			rrtgm.getPathMarker(rrt_path, lines_ugv_marker_pub_, lines_uav_marker_pub_);
			rrtgm.getCatenaryPathMarker(rrt_path, catenary_marker_pub_);
			ret_val = rrt_path.size();
			break;
		}
		
		else if (count >= n_iter){
			count_loop++;
			count = 0;
			if (count_loop > n_loop-1){
				printf("RRTPlanner::computeTreesIndependent -->  could't find path for Coupled Marsupial Configuration-->  number iteration: %lu \n\n", nodes_tree.size());    
				ret_val = 0;
				break;
			}
			else
				printf("\n\t\t       Planner (%s) :: computeTreeIndependent: Starting new Loop      \n\n",planner_type.c_str());
		}
	}

  	std::cout << "Explored Graph Nodes Numbers: " << nodes_tree.size() <<std::endl;
	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;

	if (markers_debug)
		rrtgm.getAllCatenaryMarker(nodes_tree, all_catenary_marker_pub_);

	ros::Duration(5.0).sleep();
	rrtgm.clearCatenaryMarker(catenary_marker_pub_);

  	return ret_val; 
}

bool RRTPlanner::extendGraph(const RRTNode q_rand_)
{ 
	if(is_coupled){
		RRTNode* new_node = new RRTNode();
		RRTNode q_new ;			//Take the new node value before to save it as a node in the list

		RRTNode* q_nearest = getNearestNode(q_rand_); 
		
		q_new = steering(*q_nearest, q_rand_, step_steer);
		
		RRTNode *q_min;
		if (checkNodeFeasibility(q_new,false)){
			if (obstacleFreeBetweenNodes(*q_nearest, q_new)){
				q_min = q_nearest;
			}
			else{
				ROS_INFO_COND(debug_rrt, PRINTF_RED"  RRTPlanner::extendGraph : Not Obstacle Free between q_new = [%f %f %f] and q_nearest =[%f %f %f]", q_new.point.x*step, q_new.point.y*step, q_new.point.z*step, q_nearest->point.x*step, q_nearest->point.y*step, q_nearest->point.z*step);
				return false;
			}
		}
		else{
			ROS_INFO_COND(debug_rrt, PRINTF_RED"  RRTPlanner::extendGraph : Not Feasible to extend point q_new = [%f %f %f]",q_new.point.x*step, q_new.point.y*step, q_new.point.z*step);
			return false;		
		}

		std::vector<int> v_near_nodes = getNearNodes(q_new, radius_near_nodes) ;
		q_new.parentNode = q_min;
		getParamsNode(q_new);
		updateKdtreeNode(q_new); //KdTree is updated after get Near Nodes because to not take it own node as a near node
		// I . Open near nodes and connected with minimum accumulated
		if (planner_type == "rrt_star"){
			for (size_t i = 0 ; i < v_near_nodes.size(); i++){
				for (auto nt_:nodes_tree) {
					if (nt_->id == v_near_nodes[i] ){
						if (obstacleFreeBetweenNodes(*nt_, q_new)){
							double C_ = nt_->cost + costBetweenNodes(*nt_,q_new);
							if (C_ < q_new.cost){
								q_min = nt_;
							}
						}
						else{
							// ROS_ERROR("RRTPlanner::extendGraph -->  exist collision between one of <X_near node> and <X_new node> !!");
						}
					}
				}
			}
		}
		q_new.parentNode = q_min;
		getParamsNode(q_new);
		// double C_new_ = q_new.cost;
		
		// II . Rewire Proccess 
		if (planner_type == "rrt_star"){
			for (size_t i = 0 ; i < v_near_nodes.size(); i++){
				for (auto nt_:nodes_tree) {
					if (nt_->id == v_near_nodes[i]  && nt_->id != q_min->id ){
						if (obstacleFreeBetweenNodes(*nt_, q_new)){
							if( nt_->cost > (q_new.cost + costBetweenNodes(q_new, *nt_)) ){
								*nt_->parentNode = q_new;
								nt_->cost = q_new.cost+costBetweenNodes(q_new, *nt_);
							}
						}
					}
				}
			}
		}
		*new_node = q_new;
		saveNode(new_node);
		
		if(nodes_marker_debug){
			count_graph++;
			rrtgm.getGraphMarker(new_node, count_graph, tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_);
		}
		if (markers_debug)
			rrtgm.getTakeOffNodesMarker(take_off_nodes, take_off_nodes_pub_);
		
		isGoal(q_new);
		
		return true;
	}
	else{
		RRTNode* new_node = new RRTNode();
		RRTNode q_new;	//Take the new node value before to save it as a node in the list

		RRTNode* q_nearest = getNearestNode(q_rand_); 

		if (debug_rrt) 
			printf(" q_nearest = [%f %f %f / %f %f %f] \n", q_nearest->point.x*step,q_nearest->point.y*step,q_nearest->point.z*step, q_nearest->point_uav.x*step,q_nearest->point_uav.y*step,q_nearest->point_uav.z*step);
		/********************* To graph Catenary Node *********************/
		// std::vector<geometry_msgs::Point> _points_cat_;
		// _points_cat_.clear();
		// geometry_msgs::Point _p_reel_, _p_uav_;
		// CatenarySolver _cSolver_;
		// _cSolver_.setMaxNumIterations(100);
		// _p_reel_ = getReelNode(*q_nearest);
		// _p_uav_.x = q_nearest->point_uav.x*step; 
		// _p_uav_.y = q_nearest->point_uav.y*step; 
		// _p_uav_.z = q_nearest->point_uav.z*step;
		// double _l_cat_ = q_nearest->length_cat;
		// _cSolver_.solve(_p_reel_.x, _p_reel_.y, _p_reel_.z, _p_uav_.x, _p_uav_.y, _p_uav_.z, _l_cat_, _points_cat_);
		// rrtgm.getCatenaryMarker(_points_cat_, one_catenary_marker_pub_);
		// rrtgm.markerPoints(catenary_marker, _points_cat_, 10.0, 2.0, catenary_marker_pub_);
		// std::string y_ ;
		// std::cin >> y_ ;
		// std::cout << "Continue DO-WHILE loop : " << y_ << std::endl;
	
		/******************************************************************/

		q_new = steering(*q_nearest, q_rand_, step_steer);
		q_new.parentNode = q_nearest;
		getParamsNode(q_new);
		
		if (debug_rrt)
			printf(" q_new = [%f %f %f / %f %f %f] q_new.catenary=[%s] cost=[%f] \n", q_new.point.x*step,q_new.point.y*step,q_new .point.z*step,q_new.point_uav.x*step,q_new.point_uav.y*step,q_new.point_uav.z*step,q_new.catenary ? "true" : "false",q_new.cost);
		
		

		if (!q_new.catenary){
			ROS_INFO( PRINTF_GREEN"  RRTPlanner::extendGraph : Not Catenary for new point q_new=[%f %f %f / %f %f %f]  q_nearest=[%f %f %f / %f %f %f]",
				q_new.point.x*step, q_new.point.y*step, q_new.point.z*step, q_new.point_uav.x*step, q_new.point_uav.y*step, q_new.point_uav.z*step,
				q_nearest->point.x*step, q_nearest->point.y*step, q_nearest->point.z*step, q_nearest->point_uav.x*step, q_nearest->point_uav.y*step, q_nearest->point_uav.z*step);
			return false;
		}

		RRTNode *q_min;
		// if (checkUGVFeasibility(q_new,false) && checkNodeFeasibility(q_new,true)){
		bool part_1 = checkNodeFeasibility(q_new,false) ; 
		bool part_2 = checkNodeFeasibility(q_new,true) ;
		if (part_1 && part_2){
			if (obstacleFreeBetweenNodes(*q_nearest, q_new)){
				q_min = q_nearest;
			}
			else{
				ROS_INFO(PRINTF_BLUE"  RRTPlanner::extendGraph : Not Obstacle Free q_new=[%f %f %f / %f %f %f]  q_nearest=[%f %f %f / %f %f %f]",
				q_new.point.x*step, q_new.point.y*step, q_new.point.z*step, q_new.point_uav.x*step, q_new.point_uav.y*step, q_new.point_uav.z*step,
				q_nearest->point.x*step, q_nearest->point.y*step, q_nearest->point.z*step, q_nearest->point_uav.x*step, q_nearest->point_uav.y*step, q_nearest->point_uav.z*step);
				return false;
			}
		}
		else{
			ROS_INFO(PRINTF_CYAN"  part_1=[%s] part_2=[%s]", part_1? "true" : "false" , part_2? "true" : "false");
			ROS_INFO(PRINTF_CYAN"  RRTPlanner::extendGraph : Not Feasible to extend point q_new=[%f %f %f / %f %f %f]  q_nearest=[%f %f %f / %f %f %f]", 
			q_new.point.x*step, q_new.point.y*step, q_new.point.z*step, q_new.point_uav.x*step, q_new.point_uav.y*step, q_new.point_uav.z*step,
			q_nearest->point.x*step, q_nearest->point.y*step, q_nearest->point.z*step, q_nearest->point_uav.x*step, q_nearest->point_uav.y*step, q_nearest->point_uav.z*step);
			return false;		
		}
		if (debug_rrt)
			printf(" q_min = [%f %f %f / %f %f %f] \n", q_min->point.x*step,q_min->point.y*step,q_min->point.z*step,q_min->point_uav.x*step,q_min->point_uav.y*step,q_min->point_uav.z*step);


		std::vector<int> v_near_nodes = getNearNodes(q_new, radius_near_nodes) ;
		updateKdtreeNode(q_new); 	//KdTree is updated after get Near Nodes because to not take it own node as a near node
		updateKdtreeUGV(q_new);
		bool new_parentNode_ = false;
		// I . Open near nodes and connected with minimum accumulated for UGV
		if (planner_type == "rrt_star"){
			for (size_t i = 0 ; i < v_near_nodes.size(); i++){
				for (auto nt_:nodes_tree) {
					if(nt_->id_uav == v_near_nodes[i] && nt_->id_uav != q_nearest->id_uav){
							double C_ = nt_->cost + costBetweenNodes(*nt_,q_new);   
							if (C_ < q_new.cost){
								if (obstacleFreeBetweenNodes(*nt_, q_new)){
									q_min = nt_;
									new_parentNode_ = true;
								}
							}
						break;
					}
				}
			}
		}
		if (new_parentNode_ ){
			q_new.parentNode = q_min;
			updateParamsNode(q_new);
		}

		*new_node = q_new;

		// II . Rewire Proccess for UGV
		if (planner_type == "rrt_star"){
			for (size_t i = 0 ; i < v_near_nodes.size(); i++){
				for (auto nt_:nodes_tree) {
					if(nt_->id_uav == v_near_nodes[i] && nt_->id_uav != q_min->id_uav){
						if( nt_->cost > (new_node->cost + costBetweenNodes(*new_node, *nt_)) ){
							if (obstacleFreeBetweenNodes(*nt_, *new_node)){
								nt_->parentNode = new_node;
								nt_->cost = new_node->cost + costBetweenNodes(*new_node, *nt_);
							}
						}
					}
				}
			}
		}
		
		int got_goal_aux_ = got_to_goal; 
		// printf ("got_goal_aux_ = %i ",got_goal_aux_);
		isGoal(q_new);
		// printf (" got_to_goal = %i \n",got_to_goal);
		if (got_to_goal != got_goal_aux_){
			saveNode(new_node);
			if(new_node->cost < disc_goal->cost){
				ROS_ERROR("new_node->point : [%f %f %f]",new_node->point.x*step, new_node->point.y*step, new_node->point.z*step);
				disc_goal = new_node;
			}
		}	
		else{
			saveNode(new_node);
		}
		
		if(nodes_marker_debug){
			count_graph++;
			rrtgm.getGraphMarker(new_node, count_graph, tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_);
		}

		return true;
	}
}

RRTNode RRTPlanner::getRandomNode(bool go_to_goal_) 
{
	// Random numbers
    std::random_device rd;   // obtain a random number from hardware
  	std::mt19937 eng(rd());  // seed the generator
	int max_ugv = (int)v_points_ws_ugv.size() -1;
  	std::uniform_int_distribution<int> distr_ugv(0, max_ugv);  // define the range
  	std::uniform_int_distribution<int> distr_x_uav(ws_x_min, ws_x_max);  // define the range
  	std::uniform_int_distribution<int> distr_y_uav(ws_y_min, ws_y_max);  // define the range
  	std::uniform_int_distribution<int> distr_z_uav(ws_z_min+(0.0+0.6*step_inv), ws_z_max);  // define the range 1.0+0.6*step_inv

	RRTNode randomState_;
	bool finded_node = false;
	bool catenary_state = false;
		
	// Get random position for UAV
	if (!is_coupled){
		if (!go_to_goal_){
			// printf("Random node Not Random position go_to_goal\n");
			do{
				randomState_.point_uav.x = distr_x_uav(eng);
				randomState_.point_uav.y = distr_y_uav(eng);
				randomState_.point_uav.z = distr_z_uav(eng);
				finded_node = checkNodeFeasibility(randomState_,true);

			}while(finded_node == false);
		}
		else{
			// printf("Ramdon node Go to go_to_goal\n");
			do{
				randomState_.point_uav.x = disc_final->point.x;
				randomState_.point_uav.y = disc_final->point.y;
				randomState_.point_uav.z = disc_final->point.z;
				finded_node = checkNodeFeasibility(randomState_,true);
			}while(finded_node == false);
		}
	}
	// Get random position for UGV
	if (!go_to_goal_){
		do{
			int num_rand = distr_ugv(eng);
			randomState_.point.x = v_points_ws_ugv[num_rand].x*step_inv;
			randomState_.point.y = v_points_ws_ugv[num_rand].y*step_inv;
			randomState_.point.z = v_points_ws_ugv[num_rand].z*step_inv;  
			// if (v_points_ws_ugv[num_rand].z *step_inv > disc_initial->point.z + 0.2*step_inv)
				// continue;
			if (v_points_ws_ugv[num_rand].z *step_inv < 1)
				finded_node = checkUGVFeasibility(randomState_,false); 
			else 	
				finded_node = checkUGVFeasibility(randomState_,true); 
		}while(finded_node == false);
		
	}
	else{ // Instead to go to goal, UGV fallow UAV
		if(sample_mode == 0){
			randomState_.point.x = disc_final->point.x;
			randomState_.point.y = disc_final->point.y;
			randomState_.point.z = disc_initial->point.z;   
		}
		else if(sample_mode == 1){
			// randomState_.point.x = randomState_.point_uav.x;
			// randomState_.point.y = randomState_.point_uav.y;
			// randomState_.point.z = disc_initial->point.z;   
			Eigen::Vector3d p_node_, trav_point_ugv_;
			p_node_.x() = randomState_.point_uav.x;
			p_node_.y() = randomState_.point_uav.y;
			p_node_.z() = randomState_.point_uav.z;
			trav_point_ugv_ = nn_trav_ugv.nearestObstacleMarsupial(nn_trav_ugv.kdtree, p_node_, nn_trav_ugv.obs_points);
			randomState_.point.x = trav_point_ugv_.x();
			randomState_.point.y = trav_point_ugv_.y();
			randomState_.point.z = trav_point_ugv_.z(); 
		}
	}
	return randomState_;
}

RRTNode* RRTPlanner::getNearestNode(const RRTNode q_rand_) 
{
  	RRTNode* q_nearest_; 

	double p_ugv_x_, p_ugv_y_, p_ugv_z_, p_uav_x_, p_uav_y_, p_uav_z_; 
	double d_ugv_ , d_uav_, l_cat_;
	double k0_ ,k1_, k2_ ;
	k0_ = 10.0; // 5.0
    k1_ = 10.0;  // 15.0
	k2_ = 5.0;  // 5.0

	double cost_nearest_ = 10000000;
	for (auto nt_:nodes_tree) {
		p_ugv_x_ = q_rand_.point.x * step - nt_->point.x * step;
		p_ugv_y_ = q_rand_.point.y * step - nt_->point.y * step;
		p_ugv_z_ = q_rand_.point.z * step - nt_->point.z * step;
		p_uav_x_ = q_rand_.point_uav.x * step - nt_->point_uav.x * step;
		p_uav_y_ = q_rand_.point_uav.y * step - nt_->point_uav.y * step;
		p_uav_z_ = q_rand_.point_uav.z * step - nt_->point_uav.z * step;

		d_ugv_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
		d_uav_ =  sqrt((p_uav_x_*p_uav_x_) + (p_uav_y_*p_uav_y_) + (p_uav_z_*p_uav_z_));
		l_cat_ = nt_->length_cat;

		// double cost_ = k0_ * d_ugv_ + k1_ * d_uav_ + k2_ * l_cat_;
		double cost_ = k0_ * d_ugv_ + k1_ * d_uav_ ;
		if(cost_nearest_ > cost_){
			q_nearest_ = nt_;
			cost_nearest_ = cost_;
		}
	}
	return q_nearest_;
}

RRTNode RRTPlanner::steering(const RRTNode &q_nearest_, const RRTNode &q_rand_, float factor_steer_)	
{
	RRTNode q_new_;
	float x_rand_ugv, y_rand_ugv, z_rand_ugv; 
	float x_nearest_ugv, y_nearest_ugv, z_nearest_ugv;
	float dir_ugv_x, dir_ugv_y, dir_ugv_z, uni_ugv_x, uni_ugv_y, uni_ugv_z; 
	float x_ugv_, y_ugv_, z_ugv_;
	float dist_nearest_rand;

	x_rand_ugv = q_rand_.point.x * step; 
	y_rand_ugv = q_rand_.point.y * step; 
	z_rand_ugv = q_rand_.point.z * step;
	x_nearest_ugv = q_nearest_.point.x * step; 
	y_nearest_ugv = q_nearest_.point.y * step; 
	z_nearest_ugv = q_nearest_.point.z * step;

	//Get the unitary vector from nearest to rand direction
	dir_ugv_x = x_rand_ugv - x_nearest_ugv;
	dir_ugv_y = y_rand_ugv - y_nearest_ugv; 
	dir_ugv_z = z_rand_ugv - z_nearest_ugv;

	dist_nearest_rand = sqrt(dir_ugv_x*dir_ugv_x + dir_ugv_y*dir_ugv_y+ dir_ugv_z*dir_ugv_z );

	// To avoid den = 0
	if (dist_nearest_rand < 0.0001){
		x_ugv_ = x_nearest_ugv; 
		y_ugv_ = y_nearest_ugv; 
		z_ugv_ = z_nearest_ugv; 
	}
	else{
		uni_ugv_x = dir_ugv_x/ dist_nearest_rand;
		uni_ugv_y = dir_ugv_y/ dist_nearest_rand;
		uni_ugv_z = dir_ugv_z/ dist_nearest_rand;
		// Move in direction nearest to rand with magnitude proporcional to factor_steer_
		x_ugv_ = (x_nearest_ugv + uni_ugv_x * factor_steer_); 
		y_ugv_ = (y_nearest_ugv + uni_ugv_y * factor_steer_); 
		z_ugv_ = (z_nearest_ugv + uni_ugv_z * factor_steer_); 
	}

	// Get steer point
	if ( (pow(x_ugv_ - x_nearest_ugv,2) + pow(y_ugv_ - y_nearest_ugv,2) + pow(z_ugv_ - z_nearest_ugv,2)) > 
		(pow(dir_ugv_x,2) + pow(dir_ugv_y,2) + pow(dir_ugv_z,2)) ){
		q_new_.point.x = q_rand_.point.x ; 
		q_new_.point.y = q_rand_.point.y ; 
		q_new_.point.z = q_rand_.point.z;
		// return q_new_;
	}
	else{
		Eigen::Vector3d p_node_, trav_point_ugv_, trav_point_ugv_aux;
		p_node_.x() = x_ugv_;
		p_node_.y() = y_ugv_;
		p_node_.z() = z_ugv_;
		// printf("   p_node_=[%f %f %f] dist_nearest_rand= %f \n",p_node_.x(),p_node_.y(),p_node_.z(), dist_nearest_rand);
		trav_point_ugv_aux = nn_trav_ugv.nearestObstacleMarsupial(nn_trav_ugv.kdtree, p_node_, nn_trav_ugv.obs_points);
		// printf("   trav_point_aux_=[%f %f %f]\n",trav_point_ugv_aux.x(),trav_point_ugv_aux.y(),trav_point_ugv_aux.z());
		trav_point_ugv_ = nn_trav_ugv.nearestTraversabilityMarsupial(nn_trav_ugv.kdtree, trav_point_ugv_aux, nn_trav_ugv.obs_points, 0.25);
		// printf("   trav_point_ugv_=[%f %f %f]\n",trav_point_ugv_.x(),trav_point_ugv_.y(),trav_point_ugv_.z());
		

		// z_ugv_ = (z_nearest_ugv); 
			
		//Check if the steering UGV position is in the air, in case is true, we keep as ugv_point the q_nearest_ugv position and we only steer in q_new_uav
		// bool node_on_air_ = false;
		// // if(z_ugv_ > 0.2 && isUGVInside(x_ugv_*step_inv, y_ugv_*step_inv, z_ugv_*step_inv)){
		// if(trav_point_ugv_.z() > 0.2 && isUGVInside(trav_point_ugv_.x()*step_inv, trav_point_ugv_.y()*step_inv, trav_point_ugv_.z()*step_inv)){
		// 	RRTNode n_air_;
		// 	n_air_.point.x = x_ugv_ * step_inv;
		// 	n_air_.point.y = y_ugv_ * step_inv;
		// 	n_air_.point.z = z_ugv_ * step_inv;
		// 	if(!isOccupied(n_air_)){
		// 		node_on_air_ = true;
		// 		ROS_ERROR("new node steering to the air , keeping the point q_nearest_ugv as q_new_ugv");
		// 	}
		// }

		// else if(node_on_air_){
		// 	q_new_.point.x = q_nearest_.point.x ; 
		// 	q_new_.point.y = q_nearest_.point.y ; 
		// 	q_new_.point.z = q_nearest_.point.z;
		// }
		// else{
		// 	q_new_.point.x = trav_point_ugv_.x() * step_inv; 
		// 	q_new_.point.y = trav_point_ugv_.y() * step_inv; 
		// 	q_new_.point.z = trav_point_ugv_.z() * step_inv;
		// }
		
		// double check_in_air = sqrt(pow(trav_point_ugv_aux.x()-p_node_.x(),2)+pow(trav_point_ugv_aux.y()-p_node_.y(),2)+pow(trav_point_ugv_aux.z()-p_node_.z(),2));
		// if (check_in_air > 0.20){
		// 	q_new_.point.x = q_nearest_.point.x ; 
		//  	q_new_.point.y = q_nearest_.point.y ; 
		//  	q_new_.point.z = q_nearest_.point.z;
		// 	ROS_ERROR("   Position in the air, not posible to steer [%f %f %f]",q_new_.point.x*step,q_new_.point.y*step,q_new_.point.z*step);
		// }
		// else{
		q_new_.point.x = trav_point_ugv_.x() * step_inv; 
		q_new_.point.y = trav_point_ugv_.y() * step_inv; 
		q_new_.point.z = trav_point_ugv_.z() * step_inv;
		// ROS_WARN("   Found node UGV [%f %f %f]",trav_point_ugv_.x(),trav_point_ugv_.y(),trav_point_ugv_.z());
		// }
	}

	// Here return the position of UGV in case in coupled mode or continue to get the position of UAV to return node
	if(is_coupled)
		return q_new_;
	else{
		float x_rand_uav, y_rand_uav, z_rand_uav; 
		float x_nearest_uav, y_nearest_uav, z_nearest_uav;
		float dir_uav_x, dir_uav_y, dir_uav_z, uni_uav_x, uni_uav_y, uni_uav_z; 
		float x_uav_, y_uav_, z_uav_;

		x_rand_uav = q_rand_.point_uav.x * step; 
		y_rand_uav = q_rand_.point_uav.y * step; 
		z_rand_uav = q_rand_.point_uav.z * step;
		x_nearest_uav = q_nearest_.point_uav.x * step; 
		y_nearest_uav = q_nearest_.point_uav.y * step; 
		z_nearest_uav = q_nearest_.point_uav.z * step;
			
		//Get the unitary vector from nearest to rand direction
		dir_uav_x = x_rand_uav - x_nearest_uav;
		dir_uav_y = y_rand_uav - y_nearest_uav; 
		dir_uav_z = z_rand_uav - z_nearest_uav;
			
		dist_nearest_rand = sqrt(dir_uav_x*dir_uav_x + dir_uav_y*dir_uav_y + dir_uav_z*dir_uav_z);

		// To avoid den = 0
		if (dist_nearest_rand < 0.0001){
			x_uav_ = x_nearest_uav; 
			y_uav_ = y_nearest_uav; 
			z_uav_ = z_nearest_uav; 
		}
		else{
			uni_uav_x = dir_uav_x/ dist_nearest_rand;
			uni_uav_y = dir_uav_y/ dist_nearest_rand;
			uni_uav_z = dir_uav_z/ dist_nearest_rand;
			// Move in direction nearest to rand with magnitude proporcional to factor_steer_
			x_uav_ = (x_nearest_uav + uni_uav_x * factor_steer_); 
			y_uav_ = (y_nearest_uav + uni_uav_y * factor_steer_); 
			z_uav_ = (z_nearest_uav + uni_uav_z * factor_steer_); 
		}


		if((pow(x_uav_ - x_nearest_uav,2)+pow(y_uav_ - y_nearest_uav,2)+pow(z_uav_ - z_nearest_uav,2)) > 
			(pow(x_rand_uav - x_nearest_uav,2)+pow(y_rand_uav - y_nearest_uav,2)+pow(z_rand_uav - z_nearest_uav,2)) ){
			q_new_.point_uav.x = q_rand_.point_uav.x ; 
			q_new_.point_uav.y = q_rand_.point_uav.y ; 
			q_new_.point_uav.z = q_rand_.point_uav.z;
			q_new_.rot_uav.x = 0.0;
			q_new_.rot_uav.y = 0.0;
			q_new_.rot_uav.z = 0.0;
			q_new_.rot_uav.w = 1.0;
		}
		else{
			q_new_.point_uav.x = x_uav_ * step_inv; 
			q_new_.point_uav.y = y_uav_ * step_inv; 
			q_new_.point_uav.z = z_uav_ * step_inv;
			q_new_.rot_uav.x = 0.0;
			q_new_.rot_uav.y = 0.0;
			q_new_.rot_uav.z = 0.0;
			q_new_.rot_uav.w = 1.0;
		}
			
		// Following lines keep Get random sample for only UAV and nearest UGV position for UAV
		bool catenary_able_;
		if(sample_mode == 1){ 
			std::vector<float> nearest_ugv_to_uav_  = getNearestUGVNode(q_new_);
			do_steer_ugv = true;
			RRTNode q_;
			q_.point.x = (int)nearest_ugv_to_uav_[0];
			q_.point.y = (int)nearest_ugv_to_uav_[1];
			q_.point.z = (int)nearest_ugv_to_uav_[2];
			q_.rot_ugv.x = nearest_ugv_to_uav_[3];
			q_.rot_ugv.y = nearest_ugv_to_uav_[4];
			q_.rot_ugv.z = nearest_ugv_to_uav_[5];
			q_.rot_ugv.w = nearest_ugv_to_uav_[6];
			q_.point_uav.x = q_new_.point_uav.x;
			q_.point_uav.y = q_new_.point_uav.y;
			q_.point_uav.z = q_new_.point_uav.z;
			q_.rot_uav.x =  q_new_.rot_uav.x; 
			q_.rot_uav.y =  q_new_.rot_uav.y; 
			q_.rot_uav.z =  q_new_.rot_uav.z;
			q_.rot_uav.w =  q_new_.rot_uav.w;
			catenary_able_= checkCatenary(q_, 2);
			if(catenary_able_ && q_.length_cat < min_dist_for_steer_ugv){
				q_new_ = q_;
				do_steer_ugv = false;
				ROS_WARN("New position UGV under UAV position  q_.length_cat = %f", q_.length_cat);
			} 
			else
				ROS_ERROR("Not posible to steer with mode=1 , using random node");
		}
		return q_new_;
	}
}

bool RRTPlanner::obstacleFreeBetweenNodes(const RRTNode q_nearest_,const RRTNode q_new_)
{
	geometry_msgs::Point  point_nearest_uav_ , point_new_uav_,p_reel_nearest_, p_reel_new_;
	std::vector<geometry_msgs::Point> points_cat_nearest_, points_cat_new_;
	points_cat_nearest_.clear();
	points_cat_new_.clear();

	point_new_uav_.x = q_new_.point_uav.x * step; 
	point_new_uav_.y = q_new_.point_uav.y * step; 
	point_new_uav_.z = q_new_.point_uav.z * step;

	point_nearest_uav_.x = q_nearest_.point_uav.x * step; 
	point_nearest_uav_.y = q_nearest_.point_uav.y * step; 
	point_nearest_uav_.z = q_nearest_.point_uav.z * step;

	//Get points for both catenary New_node and Nearest_node
	CatenarySolver cSolver_;
	cSolver_.setMaxNumIterations(100);

	p_reel_new_ = getReelNode(q_new_);
	double l_cat_new_ = q_new_.length_cat;
	cSolver_.solve(p_reel_new_.x, p_reel_new_.y, p_reel_new_.z, point_new_uav_.x, point_new_uav_.y, point_new_uav_.z, l_cat_new_, points_cat_new_);
	// rrtgm.reelPointMarker1(p_reel_new_, reel1_point_pub_);

	p_reel_nearest_ = getReelNode(q_nearest_);
	double l_cat_nearest_ = q_nearest_.length_cat;
	cSolver_.solve(p_reel_nearest_.x, p_reel_nearest_.y, p_reel_nearest_.z, point_nearest_uav_.x, point_nearest_uav_.y, point_nearest_uav_.z, l_cat_nearest_, points_cat_nearest_);
	// rrtgm.reelPointMarker2(p_reel_nearest_, reel2_point_pub_);

	double r_;

	std::vector<geometry_msgs::Point> points_obstacles_;
	geometry_msgs::Point point_obs_;
	points_obstacles_.clear();
	// printf("points_cat_new_.size()=%lu points_cat_nearest_.size() = %lu\n",points_cat_new_.size(), points_cat_nearest_.size());
	if(points_cat_new_.size() > points_cat_nearest_.size()){
		r_ = (double)(points_cat_nearest_.size()-1.0)/(double)(points_cat_new_.size()-1.0);
		for(size_t i = 0 ; i < points_cat_new_.size() ; i++){
			int k_ = (int)(round(r_ * (i)));
			//Get the unitary vector from nearest to rand direction
			float dir_x_ = points_cat_nearest_[k_].x - points_cat_new_[i].x;
			float dir_y_ = points_cat_nearest_[k_].y - points_cat_new_[i].y; 
			float dir_z_ = points_cat_nearest_[k_].z - points_cat_new_[i].z;
			float dist_nearest_new_ = sqrt(dir_x_*dir_x_ + dir_y_*dir_y_ + dir_z_*dir_z_);
			float uni_x_ = dir_x_/ dist_nearest_new_;
			float uni_y_ = dir_y_/ dist_nearest_new_;
			float uni_z_ = dir_z_/ dist_nearest_new_;
			int j_ = 0;
			RRTNode check_point_;
			do{
				check_point_.point.x = (points_cat_new_[i].x + uni_x_ * step * (double)j_)*step_inv; 
				check_point_.point.y = (points_cat_new_[i].y + uni_y_ * step * (double)j_)*step_inv; 
				check_point_.point.z = (points_cat_new_[i].z + uni_z_ * step * (double)j_)*step_inv; 
				point_obs_.x = (points_cat_new_[i].x + uni_x_ * step * (double)j_);
				point_obs_.y = (points_cat_new_[i].y + uni_y_ * step * (double)j_);
				point_obs_.z = (points_cat_new_[i].z + uni_z_ * step * (double)j_);
				j_++;
				//To graph point looking for obstacle//
				points_obstacles_.push_back(point_obs_);
				// printf("A: check_point_=[%i %i %i]\n",check_point_.point.x,check_point_.point.y,check_point_.point.z);
				// rrtgm.getPointsObsMarker(points_obstacles_, points_marker_pub_);		
				// std::string y_ ;
				// std::cin >> y_ ;
				// std::cout << "Continue DO-WHILE loop : " << y_ << std::endl;
				///////////////////////////////////////	
				if (!checkNodeFeasibility(check_point_,false)){
					// ROS_ERROR("A: THERE IS A OBSTACLE BETWEEN CATENARY OF Q_NEW and Q_NEAREST");
					return false;
				}
				
			}while ( (  pow(point_obs_.x - points_cat_nearest_[k_].x,2) + 
						pow(point_obs_.y - points_cat_nearest_[k_].y,2) + 
						pow(point_obs_.z - points_cat_nearest_[k_].z,2)) > step*step);
		}
	}
	else if(points_cat_new_.size() < points_cat_nearest_.size()){
		r_ = (double)(points_cat_new_.size()-1.0)/(double)(points_cat_nearest_.size()-1.0);
		for(size_t i = 0 ; i < points_cat_nearest_.size() ; i++){
			int k_ = (int)(round(r_ * (i)));
			//Get the unitary vector from nearest to rand direction
			float dir_x_ = points_cat_nearest_[i].x - points_cat_new_[k_].x;
			float dir_y_ = points_cat_nearest_[i].y - points_cat_new_[k_].y; 
			float dir_z_ = points_cat_nearest_[i].z - points_cat_new_[k_].z;
			float dist_nearest_new_ = sqrt(dir_x_*dir_x_ + dir_y_*dir_y_ + dir_z_*dir_z_);
			float uni_x_ = dir_x_/ dist_nearest_new_;
			float uni_y_ = dir_y_/ dist_nearest_new_;
			float uni_z_ = dir_z_/ dist_nearest_new_;
			int j_ = 0;
			RRTNode check_point_;
			do{
				check_point_.point.x = (points_cat_new_[k_].x + uni_x_ * step * (double)j_)*step_inv; 
				check_point_.point.y = (points_cat_new_[k_].y + uni_y_ * step * (double)j_)*step_inv; 
				check_point_.point.z = (points_cat_new_[k_].z + uni_z_ * step * (double)j_)*step_inv; 
				point_obs_.x = (points_cat_new_[k_].x + uni_x_ * step * (double)j_);
				point_obs_.y = (points_cat_new_[k_].y + uni_y_ * step * (double)j_);
				point_obs_.z = (points_cat_new_[k_].z + uni_z_ * step * (double)j_);
				j_++;
				//To graph point looking for obstacle//
				points_obstacles_.push_back(point_obs_);
				// rrttgm.getPointsObsMarker(points_obstacles_, points_marker_pub_);		
				if (!checkNodeFeasibility(check_point_,false)){
					// ROS_ERROR("B: THERE IS A OBSTACLE BETWEEN CATENARY OF Q_NEW and Q_NEAREST");
					return false;
				}
			}while ( (  pow(point_obs_.x - points_cat_nearest_[i].x,2) + 
						pow(point_obs_.y - points_cat_nearest_[i].y,2) + 
						pow(point_obs_.z - points_cat_nearest_[i].z,2)) > step*step);
		}
	}
	else{
		for(size_t i = 0 ; i < points_cat_nearest_.size() ; i++){
			//Get the unitary vector from nearest to rand direction
			float dir_x_ = points_cat_nearest_[i].x - points_cat_new_[i].x;
			float dir_y_ = points_cat_nearest_[i].y - points_cat_new_[i].y; 
			float dir_z_ = points_cat_nearest_[i].z - points_cat_new_[i].z;
			float dist_nearest_new_ = sqrt(dir_x_*dir_x_ + dir_y_*dir_y_ + dir_z_*dir_z_);
			float uni_x_ = dir_x_/ dist_nearest_new_;
			float uni_y_ = dir_y_/ dist_nearest_new_;
			float uni_z_ = dir_z_/ dist_nearest_new_;
			int j_ = 0;
			RRTNode check_point_;
			do{
				check_point_.point.x = (points_cat_new_[i].x + uni_x_ * step * (double)j_)*step_inv; 
				check_point_.point.y = (points_cat_new_[i].y + uni_y_ * step * (double)j_)*step_inv; 
				check_point_.point.z = (points_cat_new_[i].z + uni_z_ * step * (double)j_)*step_inv; 	
				point_obs_.x = (points_cat_new_[i].x + uni_x_ * step * (double)j_);
				point_obs_.y = (points_cat_new_[i].y + uni_y_ * step * (double)j_);
				point_obs_.z = (points_cat_new_[i].z + uni_z_ * step * (double)j_);
				j_++;
				//To graph point looking for obstacle//
				points_obstacles_.push_back(point_obs_);
				// printf("C: check_point_=[%i %i %i]",check_point_.point.x,check_point_.point.y,check_point_.point.z);		
				if (!checkNodeFeasibility(check_point_,false)){
					// ROS_ERROR("C: THERE IS A OBSTACLE BETWEEN CATENARY OF Q_NEW and Q_NEAREST");
					return false;
				}
			}while ( (  pow(point_obs_.x - points_cat_nearest_[i].x,2) + 
						pow(point_obs_.y - points_cat_nearest_[i].y,2) + 
						pow(point_obs_.z - points_cat_nearest_[i].z,2)) > step*step);
		}
	}	
	
	return true;	
}

std::vector<int> RRTPlanner::getNearNodes(const RRTNode &q_new_, double radius_) 
{
	std::vector<int> v_q_near_;
	std::vector<int> values_;
	v_q_near_.clear();
	values_.clear();

	point_t pt_;
	pt_ = {(float)q_new_.point_uav.x, (float)q_new_.point_uav.y, (float)q_new_.point_uav.z};
		
	int r_ = radius_*(int) step_inv;
	KDTree tree(v_nodes_kdtree);
	auto nears_ = tree.neighborhood_points(pt_, r_);
	for (auto nt_ : nears_) {
		values_.clear();
		for (int v_ : nt_) {
			values_.push_back(v_);
		}
		int x_uav_ = values_[0]; 
		int y_uav_ = values_[1];
		int z_uav_ = values_[2];
		int id_uav_ = getWorldIndex(x_uav_, y_uav_, z_uav_);
		v_q_near_.push_back(id_uav_);
	}

	return v_q_near_;
}

std::vector<float> RRTPlanner::getNearestUGVNode(const RRTNode &q_new_)  
{
	std::vector<float> ret_;
	ret_.clear();

	point_t pt_;
	pt_ = {(float)q_new_.point_uav.x, (float)q_new_.point_uav.y, (float)q_new_.point_uav.z, q_new_.rot_ugv.x, q_new_.rot_ugv.y, q_new_.rot_ugv.z, q_new_.rot_ugv.w};
		
	KDTree treeugv(v_ugv_nodes_kdtree);
	point_t nearest_ = treeugv.nearest_point(pt_,3);

    float x_ugv_to_uav_ = nearest_[0]; 
    float y_ugv_to_uav_ = nearest_[1];
    float z_ugv_to_uav_ = nearest_[2];
    float x_rot_ugv_ = nearest_[3];
    float y_rot_ugv_ = nearest_[4];
    float z_rot_ugv_ = nearest_[5];
    float w_rot_ugv_ = nearest_[6];
	// int id_ugv_ = getWorldIndex(x_ugv_to_uav_, y_ugv_to_uav_, z_ugv_to_uav_);
    ret_.push_back(x_ugv_to_uav_);
    ret_.push_back(y_ugv_to_uav_);
    ret_.push_back(z_ugv_to_uav_);
    ret_.push_back(x_rot_ugv_);
    ret_.push_back(y_rot_ugv_);
    ret_.push_back(z_rot_ugv_);
    ret_.push_back(w_rot_ugv_);

	// return id_ugv_;
	return ret_;
}

void RRTPlanner::getOrientation(RRTNode &n_ , RRTNode p_, bool is_uav_)
{
	float yaw_;
	tf::Quaternion q;

	if (!is_uav_ && do_steer_ugv){
		yaw_ = atan2(n_.point.y - p_.point.y, n_.point.x - p_.point.x);
		q.setRPY(0.0, 0.0, yaw_);
		n_.rot_ugv.x = q.x();
		n_.rot_ugv.y = q.y();
		n_.rot_ugv.z = q.z();
		n_.rot_ugv.w = q.w();
	}
	else if (!is_uav_ && !do_steer_ugv){
		n_.rot_ugv.x = p_.rot_ugv.x;
		n_.rot_ugv.y = p_.rot_ugv.y;
		n_.rot_ugv.z = p_.rot_ugv.z;
		n_.rot_ugv.w = p_.rot_ugv.w;
	}
	else if(is_uav_){
		yaw_ = atan2(n_.point_uav.y - p_.point_uav.y, n_.point_uav.x - p_.point_uav.x);
		q.setRPY(0.0, 0.0, yaw_);
		n_.rot_uav.x = q.x();
		n_.rot_uav.y = q.y();
		n_.rot_uav.z = q.z();
		n_.rot_uav.w = q.w();
	}
}

float RRTPlanner::getYawFromQuaternion(RRTNode n_, bool is_uav_)
{
	double r_, p_, y_;

	if (!is_uav_){
		tf::Quaternion q(n_.rot_ugv.x, n_.rot_ugv.y, n_.rot_ugv.z, n_.rot_ugv.w);
		tf::Matrix3x3 M(q);	
		M.getRPY(r_, p_, y_);
	}
	else{
		tf::Quaternion q(n_.rot_uav.x, n_.rot_uav.y, n_.rot_uav.z, n_.rot_uav.w);
		tf::Matrix3x3 M(q);	
		M.getRPY(r_, p_, y_);
	}

	return y_;
}

double RRTPlanner::costNode(const RRTNode q_new_)
{
	double cost_;
	double k0_, k1_, k2_, k3_, k4_; 
	double F0_, F1_, F2_, F3_;
	double r_security_ugv_ = 0.7;

	k0_ = 10.0;	// For ugv : 20.0 
	k1_ = 10.0;	// For uav: 5.0
	k2_ = 0.0;	// Diff length Cat
	k3_ = 0.0;	// Length Cat
	k4_ = 1.0;

	if(is_coupled){
		double p_ugv_x_ = q_new_.point.x * step - q_new_.parentNode->point.x * step;
		double p_ugv_y_ = q_new_.point.y * step - q_new_.parentNode->point.y * step;
		double p_ugv_z_ = q_new_.point.z * step - q_new_.parentNode->point.z * step;
		double p_uav_x_ = q_new_.point_uav.x * step - q_new_.parentNode->point_uav.x * step;
		double p_uav_y_ = q_new_.point_uav.y * step - q_new_.parentNode->point_uav.y * step;
		double p_uav_z_ = q_new_.point_uav.z * step - q_new_.parentNode->point_uav.z * step;

		double F0_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
		double F1_ =  sqrt((p_uav_x_*p_uav_x_) + (p_uav_y_*p_uav_y_) + (p_uav_z_*p_uav_z_));
								
		cost_ = k0_ * F0_ + k1_ * F1_ + q_new_.parentNode->cost;
	}
	else{
		double p_ugv_x_ = q_new_.point.x * step - q_new_.parentNode->point.x * step;
		double p_ugv_y_ = q_new_.point.y * step - q_new_.parentNode->point.y * step;
		double p_ugv_z_ = q_new_.point.z * step - q_new_.parentNode->point.z * step;

		double p_uav_x_ = q_new_.point_uav.x * step - q_new_.parentNode->point_uav.x * step;
		double p_uav_y_ = q_new_.point_uav.y * step - q_new_.parentNode->point_uav.y * step;
		double p_uav_z_ = q_new_.point_uav.z * step - q_new_.parentNode->point_uav.z * step;

		F0_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
		F1_ =  sqrt((p_uav_x_*p_uav_x_) + (p_uav_y_*p_uav_y_) + (p_uav_z_*p_uav_z_));
		F2_ = q_new_.length_cat - q_new_.parentNode->length_cat;
		F3_ = q_new_.length_cat;
		
		//exp(r_security_ugv_ - 1.0*q_new_.min_dist_obs_uav); 
		//exp(r_security_ugv_ - 1.0*q_new_.min_dist_obs_ugv); 
		//(exp(r_security_cat_ - 1.0*q_near_.min_dist_obs_cat)
		
		cost_ = k0_ * F0_ +  k1_ * F1_ + k2_ * F2_ + k3_ * F3_ + k4_* q_new_.parentNode->cost;
	}

	return cost_;
}

double RRTPlanner::costBetweenNodes(const RRTNode q_near_, const RRTNode q_new_)
{
	double cost_;
	double r_security_cat_ = 0.1;
	
	if(is_coupled){
		double p_ugv_x_ = q_new_.point.x * step - q_near_.point.x * step;
		double p_ugv_y_ = q_new_.point.y * step - q_near_.point.y * step;
		double p_ugv_z_ = q_new_.point.z * step - q_near_.point.z * step;
		
		double k1_ = 1.0;
		double dist_ugv_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
		cost_ = k1_ * dist_ugv_ ;
	}
	else{
		double k1_, k2_;

		double p_ugv_x_ = q_new_.point.x * step - q_near_.point.x * step;
		double p_ugv_y_ = q_new_.point.y * step - q_near_.point.y * step;
		double p_ugv_z_ = q_new_.point.z * step - q_near_.point.z * step;

		double p_uav_x_ = q_new_.point_uav.x * step - q_near_.point_uav.x * step;
		double p_uav_y_ = q_new_.point_uav.y * step - q_near_.point_uav.y * step;
		double p_uav_z_ = q_new_.point_uav.z * step - q_near_.point_uav.z * step;

		// k1_ = 20.0;
		// k2_ = 5.0;
		k1_ = 10.0;
		k2_ = 10.0;
		double dist_ugv_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_x_*p_ugv_x_) + (p_ugv_z_*p_ugv_z_));
		double dist_uav_ =  sqrt((p_uav_x_*p_uav_x_) + (p_uav_x_*p_uav_x_) + (p_uav_z_*p_uav_z_));
		
		cost_ = k1_ * dist_ugv_ + k2_ * dist_uav_;
	}

	return cost_;
}

void RRTPlanner::getParamsNode(RRTNode &node_, bool is_init_)
{
 	double Cat1_, Cat2_;
	Cat1_ = 5.0;
	Cat2_ = 20.0;
	double r_security_cat_ = 0.1;
	
	Eigen::Vector3d point_node_, obs_near_ugv_;
	point_node_.x() = node_.point.x * step;
	point_node_.y() = node_.point.y * step;
	point_node_.z() = node_.point.z * step;
	obs_near_ugv_ = nn_obs_ugv.nearestObstacleMarsupial(nn_obs_ugv.kdtree, point_node_, nn_obs_ugv.obs_points);
	double dist_obs_ugv = (point_node_ - obs_near_ugv_).norm();

	Eigen::Vector3d obs_near_uav_;
	point_node_.x() = node_.point_uav.x * step;
	point_node_.y() = node_.point_uav.y * step;
	point_node_.z() = node_.point_uav.z * step;
	obs_near_uav_ = nn_obs_uav.nearestObstacleMarsupial(nn_obs_uav.kdtree, point_node_, nn_obs_uav.obs_points);
	double dist_obs_uav = (point_node_ - obs_near_uav_).norm();

	node_.id = getWorldIndex(node_.point.x, node_.point.y, node_.point.z);
	node_.min_dist_obs_ugv = dist_obs_ugv;
	node_.min_dist_obs_uav = dist_obs_uav;

	if (!is_init_)
		getOrientation(node_, *(node_.parentNode), false);

	if(is_coupled){
		checkCatenary(node_, 1);
		node_.id_uav = -1.0;
		if (node_.catenary)
			node_.cost_takeoff = Cat1_* node_.length_cat + Cat2_ * exp(r_security_cat_ - 1.0*node_.min_dist_obs_cat);
		else
			node_.cost_takeoff = -1.0;
	}
	else{
		checkCatenary(node_, 2);
		node_.id_uav = getWorldIndex(node_.point_uav.x, node_.point_uav.y, node_.point_uav.z);
		node_.cost_takeoff = -1.0;
	}

	if (!is_init_){
		node_.cost = costNode(node_);
	}
}

void RRTPlanner::updateParamsNode(RRTNode &node_)
{
	node_.cost = costNode(node_);
}

bool RRTPlanner::checkUGVFeasibility(const RRTNode pf_, bool ugv_above_z_)
{
	bool ret;

	if(ugv_above_z_){
		if (isUGVInside(pf_.point.x,pf_.point.y,pf_.point.z)){
			if (isUGVOccupied(pf_)){ 
				ret = true;		
				// ROS_ERROR("Cell able for UGV");
			}
			else{
				ret = false;
				// ROS_INFO("Cell not able for UGV");
			}
		}
		else
			ret = false;
	}	
	else{
		if (isUGVOccupied(pf_)){ 
			ret = false;		
			// ROS_ERROR("Cell not able for UGV");
		}
		else{
			ret = true;
			// ROS_INFO("Cell able for UGV");
		}
	}
	return ret;	
}


// bool RRTPlanner::checkNodeFeasibility(const RRTNode pf_ , bool check_uav_)
// {
// 	bool ret;
// 	if(check_uav_==false){
// 		if (isInside(pf_.point.x,pf_.point.y,pf_.point.z) ){
// 			if (isOccupied(pf_))
// 				ret = false;
// 			else
// 				ret = true;		
// 		}
// 		else
// 			ret = false;
// 		return ret;
// 	}
// 	else{
// 		if (isInside(pf_.point_uav.x,pf_.point_uav.y,pf_.point_uav.z)){
// 			if (isOccupied(pf_, check_uav_))
// 				ret = false;
// 			else
// 				ret = true;		
// 		}
// 		else
// 			ret = false;
// 		return ret;
// 	}	
// }

bool RRTPlanner::checkNodeFeasibility(const RRTNode pf_ , bool check_uav_)
{
	bool ret;

	if(check_uav_==false){
		// if (isInside(pf_.point.x,pf_.point.y,pf_.point.z) ){
			Eigen::Vector3d obs_to_ugv, pos_ugv;
			
			pos_ugv.x() =pf_.point.x * step ;
			pos_ugv.y() =pf_.point.y * step ; 
			pos_ugv.z() =pf_.point.z * step ; 
			obs_to_ugv = nn_obs_ugv.nearestObstacleVertex(nn_obs_ugv.kdtree, pos_ugv, nn_obs_ugv.obs_points);
			double d_ = sqrt(pow(obs_to_ugv.x()-pos_ugv.x(),2) + pow(obs_to_ugv.y()-pos_ugv.y(),2) + pow(obs_to_ugv.z()-pos_ugv.z(),2));
			// printf(" pos[%f %f %f] obs[%f %f %f] d_=%f  bound_dist_obs_ugv=%f"\n,
			// 		pos_ugv.x(),pos_ugv.y(),pos_ugv.z(),obs_to_ugv.x(),obs_to_ugv.y(),obs_to_ugv.z(),d_ , distance_obstacle_ugv);
			if (d_ > distance_obstacle_ugv)
				ret = true;
			else
				ret = false;		
		// }
		// else
		// 	ret = false;
		
		return ret;
	}
	else{
		if (isInside(pf_.point_uav.x,pf_.point_uav.y,pf_.point_uav.z)){
			Eigen::Vector3d obs_to_uav, pos_uav;
			
			pos_uav.x() =pf_.point_uav.x * step ;
			pos_uav.y() =pf_.point_uav.y * step ; 
			pos_uav.z() =pf_.point_uav.z * step ; 
			obs_to_uav = nn_obs_uav.nearestObstacleVertex(nn_obs_uav.kdtree, pos_uav, nn_obs_uav.obs_points);
			double d_ = sqrt(pow(obs_to_uav.x()-pos_uav.x(),2) + pow(obs_to_uav.y()-pos_uav.y(),2) + pow(obs_to_uav.z()-pos_uav.z(),2));
			// printf("pos_uav=[%f %f %f] obs_to_uav=[%f %f %f] d_=[%f] distance_obstacle_uav=[%f]\n",pos_uav.x(),pos_uav.y(),pos_uav.z(),
			// 																				obs_to_uav.x(),obs_to_uav.y(),obs_to_uav.z(), 
			// 																				d_,	distance_obstacle_uav);
			if (d_ > distance_obstacle_uav)
				ret = true;
			else
				ret = false;
		}
		else
			ret = false;
		
		return ret;
	}	
}

bool RRTPlanner::checkPointsCatenaryFeasibility(const geometry_msgs::Point pf_)
{
	bool ret;
	RRTNode n_;
	n_.point.x = pf_.x*step_inv;
	n_.point.y = pf_.y*step_inv;
	n_.point.z = pf_.z*step_inv;
		if (isInside(n_.point.x,n_.point.y,n_.point.z) ){
			if (isOccupied(n_)){ //For UGV the cell must to be occupied because is moving through the sampled map
				ret = false;
				// ROS_ERROR("Cell not able for Catenary");
			}
			else{
				ret = true;		
				// ROS_INFO("Cell able for Catenary");
			}
		}
		else
			ret = false;
		
		return ret;

	}

bool RRTPlanner::checkCatenary(RRTNode &q_init_, int mode_)
{
	// mode 1: UGV-Goal  ,  mode 2: UGV-UAV
	geometry_msgs::Point p_reel_, p_final_;
	CatenarySolver cSolver_;
	std::vector<geometry_msgs::Point> points_catenary_;
	cSolver_.setMaxNumIterations(200);
	p_reel_ = getReelNode(q_init_);
	
	if(mode_ == 1){ 	
		p_final_.x = disc_final->point.x * step ;
		p_final_.y = disc_final->point.y * step ; 
		p_final_.z = disc_final->point.z * step ; 
	}
	else if(mode_ == 2){ 
		p_final_.x = q_init_.point_uav.x * step ;	
		p_final_.y = q_init_.point_uav.y * step ;   
		p_final_.z = q_init_.point_uav.z * step ;   
	}
	double dist_init_final_ = sqrt(pow(p_reel_.x - p_final_.x,2) + pow(p_reel_.y - p_final_.y,2) + pow(p_reel_.z - p_final_.z,2));
	double delta_ = 0.0;	//Initial Value
	bool check_catenary = true;
	bool founded_catenary = false;
	bool increase_catenary;
	double length_catenary_;
	int n_points_cat_dis_;
	double security_dis_ca_ = 0.1;
	
	do{
		increase_catenary = false;
		points_catenary_.clear();
		length_catenary_ = dist_init_final_* (1.001 + delta_);
		if (length_catenary_ > length_tether_max){
			check_catenary = false;
			// ROS_ERROR("L_cat_max < L_cat");
			break;
		}
		cSolver_.solve(p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z, length_catenary_, points_catenary_);
		// rrtgm.getCatenaryMarker(points_catenary_, one_catenary_marker_pub_);
		double d_min_point_cat = 100000;
		if (points_catenary_.size() > 5){
			n_points_cat_dis_ = ceil(1.5*ceil(length_catenary_)); // parameter to ignore collsion points in the begining and in the end of catenary
			if (n_points_cat_dis_ < 5)
				n_points_cat_dis_ = 5;
			for (size_t i = 0 ; i < points_catenary_.size() ; i++){
				geometry_msgs::Point point_cat;
				Eigen::Vector3d p_in_cat_, obs_to_cat_;
				if (points_catenary_[i].z < ws_z_min*step + ((1*step)+security_dis_ca_)){
					check_catenary = false;
					// ROS_ERROR("CATENARIA < z_min");
					break;
				}
				if ((i > n_points_cat_dis_ ) && (i < points_catenary_.size()-n_points_cat_dis_/2)){
					p_in_cat_.x() = points_catenary_[i].x;
					p_in_cat_.y() = points_catenary_[i].y;
					p_in_cat_.z() = points_catenary_[i].z;
					obs_to_cat_ = nn_obs_uav.nearestObstacleVertex(nn_obs_uav.kdtree, p_in_cat_, nn_obs_uav.obs_points);
					double dist_cat_obs = (p_in_cat_ - obs_to_cat_).norm();
					if (d_min_point_cat > dist_cat_obs){
						q_init_.min_dist_obs_cat = dist_cat_obs;
						d_min_point_cat = dist_cat_obs;
					}
					if (dist_cat_obs < security_dis_ca_){
						// ROS_ERROR("CATENARIA POINT VERY NEAR TO OBSTACLE , p=[%f %f %f]",points_catenary_[i].x, points_catenary_[i].y, points_catenary_[i].z);
						delta_ = delta_ + 0.005;
						increase_catenary = true;
						break;
					}
				}
				point_cat.x = points_catenary_[i].x;
				point_cat.y = points_catenary_[i].y;
				point_cat.z = points_catenary_[i].z;
				//TODO: Fallowing lines are not consider because checkPointsCatenaryFeasibility doesn't allow to get point near obstacle because of octomap
				// if (!checkPointsCatenaryFeasibility(point_cat)){
				// 	delta_ = delta_ + 0.005;
				// 	increase_catenary = true;
				// 	ROS_ERROR("Not Feaseble point of catenary p=[%f %f %f]",point_cat.x, point_cat.y, point_cat.z);
				// 	break;
				// }
			}
			if (check_catenary && !increase_catenary){
				founded_catenary = true;
				check_catenary = false;
				q_init_.length_cat = length_catenary_;
				// ROS_INFO("Founded Catenary !!!");
			}
		}
		else{
			check_catenary = false;
			// ROS_ERROR("RRTPlanner::checkCatenary: points_catenary_.size<= 5 ,  q_init=[%f %f %f] q_final=[%f %f %f]",p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z);
		}
	}while (check_catenary);
	//In case not feasible to find catenary
	if (!founded_catenary ){
		q_init_.length_cat = -1.0;	
		q_init_.min_dist_obs_cat = -1.0;
		// ROS_WARN("RRTPlanner::checkCatenary: Can't find catenary for node q_init=[%f %f %f] q_final=[%f %f %f]",p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z);
	}
	q_init_.catenary = founded_catenary;
	return founded_catenary;
}

geometry_msgs::Point RRTPlanner::getReelNode(const RRTNode node_)
{
	geometry_msgs::Point pos_reel;
	float yaw_ugv;

	yaw_ugv = getYawFromQuaternion(node_,false);
	double lengt_vec =  sqrt(pos_reel_ugv.x*pos_reel_ugv.x + pos_reel_ugv.y*pos_reel_ugv.y);
	pos_reel.x = node_.point.x*step + lengt_vec *cos(yaw_ugv); 
	pos_reel.y = node_.point.y*step + lengt_vec *sin(yaw_ugv);
	pos_reel.z = node_.point.z*step + pos_reel_ugv.z ;

	// printf("pos_reel = [%f %f %f] yaw = %f\n",pos_reel.x,pos_reel.y,pos_reel.z,yaw_ugv);

	return pos_reel;
}

void RRTPlanner::updateKdtreeNode(const RRTNode ukT_)
{
	point_t pt_;

	if (is_coupled){
		pt_ = {(float)ukT_.point.x, (float)ukT_.point.y, (float)ukT_.point.z};
		v_nodes_kdtree.push_back(pt_);
	}
	else{
		pt_ = {(float)ukT_.point_uav.x, (float)ukT_.point_uav.y, (float)ukT_.point_uav.z};
		v_nodes_kdtree.push_back(pt_);
	}
}

void RRTPlanner::updateKdtreeUGV(const RRTNode ukT_)
{
	point_t pt_;

	pt_ = {(float)ukT_.point.x, (float)ukT_.point.y, (float)ukT_.point.z,ukT_.rot_ugv.x, ukT_.rot_ugv.y, ukT_.rot_ugv.z, ukT_.rot_ugv.w};
	v_ugv_nodes_kdtree.push_back(pt_);
}

void RRTPlanner::readPointCloudTraversabilityMapUGV(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_trav_ugv.setInput(*msg);
	ROS_INFO_COND(debug_rrt, PRINTF_BLUE "RRTPlanner Planner: Receiving point cloud map to create Kdtree for Traversability UGV");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg,*cloud_in);

	ROS_INFO(PRINTF_BLUE"RRTPlanner::readPointCloudTraversabilityMapUGV  size point cloud = [%lu]",cloud_in->size());
	geometry_msgs::Point point_;
	for (size_t i = 0 ; i < cloud_in->size() ; i ++){
		point_.x = cloud_in->points[i].x;
		point_.y = cloud_in->points[i].y;
		point_.z = cloud_in->points[i].z;
		v_points_ws_ugv.push_back(point_);
	}
	ROS_INFO(PRINTF_BLUE"RRTPlanner::readPointCloudTraversabilityMapUGV  size v_points_ws_ugv = [%lu]",v_points_ws_ugv.size());
}

void RRTPlanner::readPointCloudMapForUGV(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_obs_ugv.setInput(*msg);
	ROS_INFO_COND(debug_rrt, PRINTF_BLUE "RRTPlanner Planner: Receiving point cloud map to create Kdtree for UGV Obstacles");
}

void RRTPlanner::readPointCloudMapForUAV(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_obs_uav.setInput(*msg);
	ROS_INFO_COND(debug_rrt, PRINTF_BLUE "RRTPlanner Planner: Receiving point cloud map to create Kdtree for UAV Obstacles");
}

bool RRTPlanner::saveNode(RRTNode* sn_, bool is_init_)
{
	if(is_init_){
		getParamsNode(*sn_,is_init_);
		if (!sn_->catenary)
			return false;
	}
	if (sn_->catenary && is_coupled)
		saveTakeOffNode(sn_);
	nodes_tree.push_back(sn_);

	return true;
}

inline void RRTPlanner::saveTakeOffNode(RRTNode* ston_)
{
	take_off_nodes.push_back(ston_); 
}

inline void RRTPlanner::clearStatus()
{
  	nodes_tree.clear();
  	take_off_nodes.clear();
	rrt_path.clear();

  	got_to_goal = 0;
	v_nodes_kdtree.clear();
	v_ugv_nodes_kdtree.clear();
	length_catenary.clear();

	rrtgm.clearMarkers(tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_, take_off_nodes_pub_, lines_ugv_marker_pub_, lines_uav_marker_pub_);
}

std::list<RRTNode*> RRTPlanner::getPath()
{
	std::list<RRTNode*> path_;
	RRTNode* current_node;
	
	if(is_coupled){
		int count = 0;
		if (is_coupled){
			double cost_ = 100000000;
			for(auto ton_:take_off_nodes){
				count++;
				if (ton_->cost_takeoff < cost_ && ton_->cost_takeoff > 0.1){
					cost_ = ton_->cost_takeoff;
					current_node = ton_;
				} 
			}
			path_.push_front(current_node);
			count = 0;
			while (current_node->parentNode != NULL){ 
				current_node = current_node->parentNode;
				path_.push_front(current_node);
				count++;
			}
		}
	}
	else{
		current_node = disc_goal;
		path_.push_front(current_node);
		length_catenary.push_back(current_node->length_cat);
		while (current_node->parentNode != NULL){ 
			current_node = current_node->parentNode;
			path_.push_front(current_node);
			length_catenary.push_back(current_node->length_cat);
		}
	}
	
	return path_;
}

void RRTPlanner::isGoal(const RRTNode st_) 
{
	geometry_msgs::Vector3 point_;

	if (is_coupled){
		point_.x = st_.point.x;
		point_.y = st_.point.y;
		point_.z = st_.point.z;

		double dist_goal_ = sqrt(pow(point_.x*step - final_position.x,2) + 
	  							 pow(point_.y*step - final_position.y,2) +
					 			 pow(point_.z*step - final_position.z,2) );
		if (dist_goal_ < goal_gap_m)
			got_to_goal = got_to_goal + 1;
		else
			got_to_goal = got_to_goal + 0;	
	}
	else{
		point_.x = st_.point_uav.x;
		point_.y = st_.point_uav.y;
		point_.z = st_.point_uav.z;

		double dist_goal_ = sqrt(pow(point_.x*step - final_position.x,2) + 
	  							 pow(point_.y*step - final_position.y,2) +
					 			 pow(point_.z*step - final_position.z,2) );
	
		// printf("dist_goal = %f/%f \n",dist_goal_, goal_gap_m);

		if (dist_goal_ < goal_gap_m){
			got_to_goal = got_to_goal + 1;
		}
		else
			got_to_goal = got_to_goal + 0;	
	}
}

bool RRTPlanner::getTrajectory(Trajectory &trajectory)
{
	trajectory_msgs::MultiDOFJointTrajectoryPoint traj_marsupial_;

	traj_marsupial_.transforms.resize(2);
	traj_marsupial_.velocities.resize(2);
	traj_marsupial_.accelerations.resize(2);

	for(auto nt_ : rrt_path){
		traj_marsupial_.transforms[0].translation.x = nt_->point.x*step;
		traj_marsupial_.transforms[0].translation.y = nt_->point.y*step;
		traj_marsupial_.transforms[0].translation.z = nt_->point.z*step;
		traj_marsupial_.transforms[0].rotation.x = nt_->rot_ugv.x;
		traj_marsupial_.transforms[0].rotation.y = nt_->rot_ugv.y;
		traj_marsupial_.transforms[0].rotation.z = nt_->rot_ugv.z;
		traj_marsupial_.transforms[0].rotation.w = nt_->rot_ugv.w;
		traj_marsupial_.velocities[0].linear.x = 0.0;
		traj_marsupial_.velocities[0].linear.y = 0.0;
		traj_marsupial_.velocities[0].linear.z = 0.0;
		traj_marsupial_.accelerations[0].linear.x = 0.0;
		traj_marsupial_.accelerations[0].linear.y = 0.0;
		traj_marsupial_.accelerations[0].linear.z = 0.0;
		traj_marsupial_.transforms[1].translation.x = nt_->point_uav.x*step;
		traj_marsupial_.transforms[1].translation.y = nt_->point_uav.y*step;
		traj_marsupial_.transforms[1].translation.z = nt_->point_uav.z*step;
		traj_marsupial_.transforms[1].rotation.x = nt_->rot_uav.x;
		traj_marsupial_.transforms[1].rotation.y = nt_->rot_uav.y;
		traj_marsupial_.transforms[1].rotation.z = nt_->rot_uav.z;
		traj_marsupial_.transforms[1].rotation.w = nt_->rot_uav.w;
		traj_marsupial_.velocities[1].linear.x = 0.0;
		traj_marsupial_.velocities[1].linear.y = 0.0;
		traj_marsupial_.velocities[1].linear.z = 0.0;
		traj_marsupial_.accelerations[1].linear.x = 0.0;
		traj_marsupial_.accelerations[1].linear.y = 0.0;
		traj_marsupial_.accelerations[1].linear.z = 0.0;
		traj_marsupial_.time_from_start = ros::Duration(0.5);
		
		trajectory.points.push_back(traj_marsupial_);
	}
	return true;
}

void RRTPlanner::configRRTParameters(double _l_m, geometry_msgs::Vector3 _p_reel , geometry_msgs::Vector3 _p_ugv, geometry_msgs::Quaternion _r_ugv, 
									bool coupled_, int n_iter_ , int n_loop_, double r_nn_, double s_s_, int s_g_r_, int sample_m_, bool do_s_ugv_, double min_l_steer_ugv_)
{
	length_tether_max = _l_m;
	pos_reel_ugv.x = _p_reel.x;
	pos_reel_ugv.y = _p_reel.y;
	pos_reel_ugv.z = _p_reel.z;
	pos_tf_ugv.x = _p_ugv.x;
	pos_tf_ugv.y = _p_ugv.y;
	pos_tf_ugv.z = _p_ugv.z;
	rot_tf_ugv.x = _r_ugv.x;
	rot_tf_ugv.y = _r_ugv.y;
	rot_tf_ugv.z = _r_ugv.z;
	rot_tf_ugv.w = _r_ugv.w;
	is_coupled = coupled_;
	n_iter = n_iter_;
	n_loop = n_loop_;
	radius_near_nodes = r_nn_;
	step_steer = s_s_;
	samp_goal_rate = s_g_r_;

	sample_mode = sample_m_; 
	do_steer_ugv = do_s_ugv_;

	min_dist_for_steer_ugv = min_l_steer_ugv_;

	rrtgm.configGraphMarkers(frame_id, step, is_coupled, n_iter, pos_reel_ugv);
}

bool RRTPlanner::setInitialPositionCoupled(DiscretePosition p_)
{
	if (isUGVInside(p_.x, p_.y, p_.z))
	{
		RRTNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(p_.x, p_.y, p_.z)];

		if (initialNodeInWorld->node == NULL)
		{
			initialNodeInWorld->node = new RRTNode();
			initialNodeInWorld->node->point.x = p_.x;
			initialNodeInWorld->node->point.y = p_.y;
			initialNodeInWorld->node->point.z = p_.z;

			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
		}
		disc_initial = initialNodeInWorld->node;

		initial_position_ugv.x = p_.x * step;
		initial_position_ugv.y = p_.y * step;
		initial_position_ugv.z = p_.z * step;

		disc_initial->point = p_;
		disc_initial->parentNode = NULL;
		disc_initial->cost = 0.0;
		disc_initial->length_cat = -1.0;
		disc_initial->min_dist_obs_cat = -1.0;
		disc_initial->min_dist_obs_ugv = -1.0;

		return true;
	}
	else
	{
		//~ std::cerr << "ThetaStar: Initial point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
		disc_initial = NULL;
		return false;
	}
}

bool RRTPlanner::setInitialPositionIndependent(DiscretePosition p1_, DiscretePosition p2_)
{
	if (isUGVInside(p1_.x, p1_.y, p1_.z) && isInside(p2_.x, p2_.y, p2_.z))
	{
		RRTNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(p1_.x, p1_.y, p1_.z)];

		if (initialNodeInWorld->node == NULL)
		{
			initialNodeInWorld->node = new RRTNode();
			initialNodeInWorld->node->point.x = p1_.x;
			initialNodeInWorld->node->point.y = p1_.y;
			initialNodeInWorld->node->point.z = p1_.z;
			initialNodeInWorld->node->rot_ugv.x = rot_tf_ugv.x;
			initialNodeInWorld->node->rot_ugv.y = rot_tf_ugv.y;
			initialNodeInWorld->node->rot_ugv.z = rot_tf_ugv.z;
			initialNodeInWorld->node->rot_ugv.w = rot_tf_ugv.w;

			initialNodeInWorld->node->point_uav.x = p2_.x;
			initialNodeInWorld->node->point_uav.y = p2_.y;
			initialNodeInWorld->node->point_uav.z = p2_.z;
			initialNodeInWorld->node->rot_uav.x = rot_tf_ugv.x;
			initialNodeInWorld->node->rot_uav.y = rot_tf_ugv.y;
			initialNodeInWorld->node->rot_uav.z = rot_tf_ugv.z;
			initialNodeInWorld->node->rot_uav.w = rot_tf_ugv.w;
			
			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
		}
		disc_initial = initialNodeInWorld->node;

		initial_position_ugv.x = p1_.x * step;
		initial_position_ugv.y = p1_.y * step;
		initial_position_ugv.z = p1_.z * step;

		initial_position_uav.x = p2_.x * step;
		initial_position_uav.y = p2_.y * step;
		initial_position_uav.z = p2_.z * step;

		disc_initial->point = p1_;
		disc_initial->point_uav = p2_;
		disc_initial->parentNode = NULL;
		disc_initial->cost = 0.0;
		disc_initial->length_cat = -1.0;
		disc_initial->min_dist_obs_cat = -1.0;
		disc_initial->min_dist_obs_ugv = -1.0;
		ROS_INFO(PRINTF_BLUE "RRTPlanner::setInitialPositionIndependent -->  disc_initial [%f %f %f /%f %f %f]",disc_initial->point.x*step,disc_initial->point.y*step,disc_initial->point.z*step,
												disc_initial->point_uav.x*step,disc_initial->point_uav.y*step,disc_initial->point_uav.z*step);

		return true;
	}
	else
	{
		//~ std::cerr << "ThetaStar: Initial point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
		disc_initial = NULL;
		return false;
	}
}

bool RRTPlanner::setFinalPosition(DiscretePosition p_)
{
	if (isInside(p_.x, p_.y, p_.z))
	{
		RRTNodeLink3D *finalNodeInWorld = &discrete_world[getWorldIndex(p_.x, p_.y, p_.z)];

		if (finalNodeInWorld->node == NULL)
		{
			finalNodeInWorld->node = new RRTNode();
			finalNodeInWorld->node->point.x = p_.x;
			finalNodeInWorld->node->point.x = p_.y;
			finalNodeInWorld->node->point.x = p_.z;

			finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
		}
		disc_final = finalNodeInWorld->node;

		final_position.x = p_.x * step;
		final_position.y = p_.y * step;
		final_position.z = p_.z * step;
		disc_final->point = p_;

		ROS_INFO(PRINTF_BLUE "RRTPlanner::setFinalPosition -->  disc_final [%f %f %f /%f %f %f]",disc_final->point.x*step,disc_final->point.y*step,disc_final->point.z*step,
												disc_final->point_uav.x*step,disc_final->point_uav.y*step,disc_final->point_uav.z*step);

		return true;
	}
	else
	{
		//~ std::cerr << "ThetaStar: Final point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
		disc_final = NULL;
		return false;
	}
}

inline void RRTPlanner::setInitialCostGoal(RRTNode* p_)
{
	RRTNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(p_->point.x, p_->point.y, p_->point.z)];

		if (initialNodeInWorld->node == NULL)
		{
			initialNodeInWorld->node = new RRTNode();
			initialNodeInWorld->node->point.x = p_->point.x;
			initialNodeInWorld->node->point.y = p_->point.y;
			initialNodeInWorld->node->point.z = p_->point.z;

			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
		}
		disc_goal = initialNodeInWorld->node;

		disc_goal->point.x = p_->point.x;
		disc_goal->point.y = p_->point.y;
		disc_goal->point.z = p_->point.z;
		disc_goal->parentNode = NULL;
		disc_goal->cost = 1000000000;
		disc_goal->length_cat = -1.0;
		disc_goal->min_dist_obs_cat = -1.0;
		disc_goal->min_dist_obs_ugv = -1.0;
}

bool RRTPlanner::isInitialPositionUGVOccupied()
{

	if (isUGVOccupied(*disc_initial))
		return true;
	else
		return false;
}

bool RRTPlanner::isInitialPositionUAVOccupied()
{
	if (isOccupied(*disc_initial, true))
		return true;
	else
		return false;
}

bool RRTPlanner::isFinalPositionOccupied()
{
	if (isOccupied(*disc_final))
		return true;
	else
		return false;
}

bool RRTPlanner::isOccupied(RRTNode n_, bool check_uav_)
{
	if(check_uav_==false){
		return !discrete_world[getWorldIndex(n_.point.x, n_.point.y, n_.point.z)].notOccupied;
	}
	else
		return !discrete_world[getWorldIndex(n_.point_uav.x, n_.point_uav.y, n_.point_uav.z)].notOccupied;
	
}

bool RRTPlanner::isUGVOccupied(RRTNode n_)
{
	RRTNode n_z_displace_;
	n_z_displace_.point.x = n_.point.x;
	n_z_displace_.point.y = n_.point.y;
	n_z_displace_.point.z = n_.point.z + (v_inflation + step_inv);

	return !discrete_world[getWorldIndex(n_z_displace_.point.x, n_z_displace_.point.y, n_z_displace_.point.z)].notOccupied;
}

void RRTPlanner::publishOccupationMarkersMap()
{
	markerRviz.header.frame_id = frame_id;
	markerRviz.header.stamp = ros::Time();
	markerRviz.ns = "debug";
	markerRviz.id = 66;
	markerRviz.type = RVizMarker::CUBE_LIST;
	markerRviz.action = RVizMarker::ADD;
	markerRviz.pose.orientation.w = 1.0;
	markerRviz.scale.x = 1.0 * step;
	markerRviz.scale.y = 1.0 * step;
	markerRviz.scale.z = 1.0 * step;
	markerRviz.color.a = 1.0;
	markerRviz.color.r = 0.0;
	markerRviz.color.g = 1.0;
	markerRviz.color.b = 0.0;

	occupancy_marker.clear();
	occupancy_marker.header.frame_id = frame_id; // "world";
	for (int i = ws_x_min_inflated; i <= ws_x_max_inflated; i++)
		for (int j = ws_y_min_inflated; j <= ws_y_max_inflated; j++)
			for (int k = ws_z_min_inflated; k <= ws_z_max_inflated; k++)
			{
				unsigned int matrixIndex = getWorldIndex(i, j, k);

				if (!discrete_world[matrixIndex].notOccupied)
				{
					//~ geometry_msgs::Point point;
					pcl::PointXYZ point;
					point.x = i * step;
					point.y = j * step;
					point.z = k * step;
					occupancy_marker.push_back(point);
				}
			}

	occupancy_marker_pub_.publish(occupancy_marker);
}


void RRTPlanner::updateMap(octomap_msgs::OctomapConstPtr message)
{
	// Clear current map in the discrete occupancy
	clearMap();

	// Read occupation data from the octomap_server
	//octomap_msgs::binaryMsgToMap(message));
	map = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*message);
	/*
     * Update discrete world with the read octomap data
     */

	// Occupied and not occupied points, only for debug
	u_int64_t occupied_leafs = 0, free_leafs = 0;
	int nit = 0;

	// Read from first to the last leaf of the tree set its xyz (discretized) if it is occupied into the occupancy matrix

	if (map->begin_leafs() != NULL)
	{
		for (octomap::OcTree::leaf_iterator it = map->begin_leafs(), end = map->end_leafs(); it != end; ++it)
		{
			if (map->isNodeOccupied(*it))
			{
				// Get occupied cells
				float x_w = it.getX();
				float y_w = it.getY();
				float z_w = it.getZ();

				// Exact discretization
				int x_ = (int)(x_w * step_inv);
				int y_ = (int)(y_w * step_inv);
				int z_ = (int)(z_w * step_inv);

				// Set as occupied in the discrete matrix
				//ROS_INFO("[%.2f, %.2f, %.2f] step_inv: %.2f", x_w, y_w, z_w, step_inv);
				//ROS_INFO("Disc [%d, %d, %d]", x_, y_, z_);
				//usleep(5e4);
				if (isInside(x_, y_, z_))
				{
					++nit;
					//ROS_INFO(PRINTF_RED"[%.2f, %.2f, %.2f] step_inv: %.2f", x_w, y_w, z_w, step_inv);
					//ROS_INFO(PRINTF_RED"Disc [%d, %d, %d]", x_, y_, z_);
					//usleep(5e4);
					unsigned int world_index_ = getWorldIndex(x_, y_, z_);
					discrete_world[world_index_].notOccupied = false;
					//ROS_INFO(PRINTF_RED"Inside: [%.2f, %.2f, %.2f]", x_w,y_w, z_w);
					//
					// Inflates nodes
					if (h_inflation >= step || v_inflation >= step)
					{
						if (z_w > z_not_inflate)
							//inflateNodeAsCube(x_, y_, z_);
							//inflateNodeAsCylinder(x_, y_, z_);
							inflateNodeAsXyRectangle(x_, y_, z_);

						else
							inflateNodeAsCylinder(x_, y_, z_);
					}
				}

#ifdef PRINT_OCTREE_STATS
				occupied_leafs++;
#endif
			}
			else
			{
#ifdef PRINT_OCTREE_STATS
				free_leafs++;
#endif
			}
		}
	}

#ifdef PRINT_OCTREE_STATS
	std::cout << "Occupied cells: " << occupied_leafs << " NO occupied cells: " << free_leafs << "It counter: " << nit << std::endl;
	ROS_INFO("Occupied cells: %d\t No occupied cells: %d", occupied_leafs, free_leafs);
#endif
}

void RRTPlanner::updateMap(PointCloud cloud)
{
	/*
     * Update discrete world with the Point Cloud = ocuppieds cells
     */
	clearMap();
	double dist=0;
	
	BOOST_FOREACH (const pcl::PointXYZ &p, cloud.points)
	{
		// Get occupied points
		//const pcl::PointXYZ &p = cloud.points[it];
		float x_w = p.x;
		float y_w = p.y;
		float z_w = p.z;

		// Exact discretization
		int x_ = (int)(x_w * step_inv);
		int y_ = (int)(y_w * step_inv);
		int z_ = (int)(z_w * step_inv);

		if (isInside(x_, y_, z_))
		{
			unsigned int world_index_ = getWorldIndex(x_, y_, z_);
			dist=sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
			if(dist < minR){
				discrete_world[world_index_].notOccupied = true;
				continue;
			}
			else
				discrete_world[world_index_].notOccupied = false;
			
			// Inflates nodes
			if (h_inflation * step >= step || v_inflation * step >= step)
			{
				if (z_w > z_not_inflate)
				{
					inflateNodeAsCube(x_, y_, z_);
					//inflateNodeAsCylinder(x_, y_, z_);
					// inflateNodeAsXyRectangle(x_, y_, z_);
				}
				else
				{
					//inflateNodeAsXyRectangle(x_, y_, z_);
					inflateNodeAsCube(x_, y_, z_);
				}
			}
		}
	}
}

void RRTPlanner::updateMap(const PointCloud::ConstPtr &map)
{
	/*
     * Update discrete world with the Point Cloud = ocuppieds cells
     */
	BOOST_FOREACH (const pcl::PointXYZ &p, map->points)
	{
		// Get occupied points
		//const pcl::PointXYZ &p = cloud->points[it];
		float x_w = p.x;
		float y_w = p.y;
		float z_w = p.z;

		// Exact discretization
		int x_ = (int)(x_w * step_inv);
		int y_ = (int)(y_w * step_inv);
		int z_ = (int)(z_w * step_inv);

		if (isInside(x_, y_, z_))
		{
			unsigned int world_index_ = getWorldIndex(x_, y_, z_);
			discrete_world[world_index_].notOccupied = false;

			// Inflates nodes
			if (h_inflation * step >= step || v_inflation * step >= step)
			{
				if (z_w > z_not_inflate)
				{
					inflateNodeAsCube(x_, y_, z_);
					//inflateNodeAsCylinder(x_, y_, z_);
					// inflateNodeAsXyRectangle(x_, y_, z_);
				}
				else
				{
					inflateNodeAsCube(x_, y_, z_);
				}
			}
		}
	}
}

void RRTPlanner::clearMap()
{
	for (int i = 0; i < matrix_size; i++)
	{
		discrete_world[i].notOccupied = true;
	}
}

DiscretePosition RRTPlanner::discretizePosition(Vector3 p)
{
	DiscretePosition res;

	res.x = p.x * step_inv;
	res.y = p.y * step_inv;
	res.z = p.z * step_inv;

	return res;
}

void RRTPlanner::setTimeOut(int sec)
{
	timeout = sec;
}

} //namespace



