#include "rrt_star_planners/RRT_Star.hpp"


namespace PathPlanners
{
//*****************************************************************
// 				ThetaStar Algorithm Class Definitions
//*****************************************************************

// Default constructor
RRTStar::RRTStar()
{
}

// Constructor with arguments
RRTStar::RRTStar(std::string plannerName, std::string frame_id, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_, float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, ros::NodeHandlePtr nh_, double goal_gap_m_)
{
	// Call to initialization
	init(plannerName, frame_id, ws_x_max_, ws_y_max_, ws_z_max_, ws_x_min_, ws_y_min_, ws_z_min_, step_, h_inflation_, v_inflation_, goal_weight_, z_weight_cost_, z_not_inflate_, nh_ ,goal_gap_m_);
}

// Initialization: creates the occupancy matrix (discrete nodes) from the bounding box sizes, resolution, inflation and optimization arguments
void RRTStar::init(std::string plannerName, std::string frame_id_, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_,
					   float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, ros::NodeHandlePtr nh_, double goal_gap_m_)
{
	// Pointer to the nodeHandler
	nh = nh_;

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
	printf("RRTStar (%s): Occupancy Matrix has %d nodes [%lu MB]\n", plannerName.c_str(), matrix_size, (uint_fast32_t)(matrix_size * sizeof(RRTNode)) / (1024 * 1024));
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
	
	debug = true;


	occupancy_marker_pub_ = nh->advertise<PointCloud>("vis_marker_occupancy", 1, true);

	tree_rrt_star_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("tree_rrt_star_ugv", 2, true);
	tree_rrt_star_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("tree_rrt_star_uav", 2, true);
	take_off_nodes_pub_ = nh->advertise<visualization_msgs::MarkerArray>("take_off_nodes_rrt_star", 2, true);
	lines_ugv_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("path_ugv_rrt_star", 2, true);
	lines_uav_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("path_uav_rrt_star", 2, true);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marsupial", 1000, true);
	all_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("all_catenaries_rrt", 10000, true);
	goal_point_pub_ = nh->advertise<visualization_msgs::Marker>("goal_point", 1, true);
	rand_point_pub_ = nh->advertise<visualization_msgs::MarkerArray>("rand_point", 2, true);
	one_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("one_catenaty", 1000, true);
	points_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("points_marker", 10, true);
}

RRTStar::~RRTStar()
{
//   clearNodes(); 
}

int RRTStar::computeTreeCoupled()
{
    std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;
	printf("RRTStar::computeTreeCoupled -->  STARTING --> star_point_ugv[%.2f %.2f %.2f]  goal_point=[%.2f %.2f %.2f] \n\n",
	initial_position_ugv.x, initial_position_ugv.y, initial_position_ugv.z, final_position.x, final_position.y, final_position.z);    

  	// clearNodes(); 
	// clearMarkers();
	v_nodes_kdtree.clear();

	saveNode(disc_initial,true);
	getGraphMarker();			// Incremental algorithm --> the graph is generated in each calculation
	getTakeOffNodesMarker();
	updateKdtree(*disc_initial);

 	double ret_val = -1.0; 
    int count = 0; 
	
    while (count < n_iter)  // n_iter Max. number of nodes to expand for each round
    {
	  	count++;
		// printf("__________________________________  RRTStar::computeTreeCoupled: STARTING WHILE LOOP[%i]  _________________________________\n",count);
		RRTNode q_rand = getRandomNode();	// get a vector with one element in case of coupled configuration
		
		printf("q_rand = [%f %f %f / %f %f %f]\n",q_rand.point.x*step,q_rand.point.y*step,q_rand.point.z*step,q_rand.point_uav.x*step,q_rand.point_uav.y*step,q_rand.point_uav.z*step);

		extendGraph(q_rand);
		if ((take_off_nodes.size() > 0) && got_to_goal){
			printf("RRTStar::computeTreeCoupled -->  breaking while for in iteration=%i",count);    
			break;
		}
	}

	if (take_off_nodes.size() > 0){
		rrt_path = getPath(); 
		printf("RRTStar::computeTreeCoupled -->  finded path for Coupled Marsupial Configuration-->  path size: %lu , number iteration: %i , take off nodes: %lu \n\n",
		rrt_path.size(), count + 1, take_off_nodes.size()); 
		int i_=0;   
		for (auto pt_: rrt_path){
			i_++;
			printf("point_path[%i/%lu] : ugv=[%f %f %f]  uav=[%f %f %f]  cost=[%f]\n", i_, rrt_path.size()
			,pt_->point.x*step, pt_->point.y*step, pt_->point.z*step, pt_->point_uav.x*step, pt_->point_uav.y*step, pt_->point_uav.z*step, pt_->cost);
		}
		getPathMarker(rrt_path);
		ret_val = rrt_path.size();
	}
	else
		printf("RRTStar::computeTreeCoupled -->  could't find path for Coupled Marsupial Configuration-->  number iteration: %lu \n\n", nodes_tree.size());    

  	std::cout << "Explored Graph Nodes Numbers: " << nodes_tree.size() <<std::endl;
  	std::cout << "Explored Graph Nodes Numbers to Take Off: " << take_off_nodes.size() <<std::endl;
	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;

  return ret_val; 
}

int RRTStar::computeTreesIndependent()
{
    std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;
	printf("RRTStar::computeTreesIndependent -->  STARTING --> star_point_ugv[%.2f %.2f %.2f]  goal_point=[%.2f %.2f %.2f] \n\n",
	initial_position_ugv.x, initial_position_ugv.y, initial_position_ugv.z, final_position.x, final_position.y, final_position.z);    

	publishOccupationMarkersMap();

	v_nodes_kdtree.clear();
	printf("disc_initial [%f %f %f /%f %f %f]\n",disc_initial->point.x*step,disc_initial->point.y*step,disc_initial->point.z*step, disc_initial->point_uav.x*step,disc_initial->point_uav.y*step,disc_initial->point_uav.z*step);
	
	if(!saveNode(disc_initial,true)){
		ROS_ERROR("RRTStar::computeTreesIndependent --> Not posible to get catenary in initial node");
		return 0;
	}
	goalPointMarker();
	getGraphMarker();			
	updateKdtree(*disc_initial);

 	double ret_val = -1.0; 
    int count = 0; 
    
	while (count < n_iter && !got_to_goal) { // n_iter Max. number of nodes to expand for each round
      	
		printf("__________________________________  RRTStar::computeTreeIndependent: STARTING WHILE LOOP[%i]  _________________________________\n",count);
		RRTNode q_rand;
		
		if ((count%samp_goal_rate)!=0){
			q_rand = getRandomNode();	
		}
		else{
			q_rand = getRandomNode(true);	
		}
		printf(" q_rand = [%f %f %f / %f %f %f] \n",q_rand.point.x*step,q_rand.point.y*step,q_rand.point.z*step,q_rand.point_uav.x*step,q_rand.point_uav.y*step,q_rand.point_uav.z*step);
		randPointMarker(q_rand);
		
		extendGraph(q_rand);

		count++;

		// std::string y_ ;
		// std::cin >> y_ ;
		// std::cout << "Continue to next iteration : " << y_ << std::endl;
	}

	if (got_to_goal){
		printf("\nRRTStar::computeTreesIndependent -->  finded goal for Coupled Marsupial Configuration\n")	; 
		rrt_path = getPath(); 
		printf("RRTStar::computeTreesIndependent -->  finded path for Coupled Marsupial Configuration-->  path size: %lu , number iteration: %i \n\n",rrt_path.size(), count + 1); 
		int i_=0;   
		for (auto pt_: rrt_path){
			i_++;
			printf("point_path[%i/%lu] :  ugv=[%f %f %f]  uav=[%f %f %f]  length_catenary=%f    cost=%f\n", i_, rrt_path.size(),
			pt_->point.x*step, pt_->point.y*step, pt_->point.z*step,pt_->point_uav.x*step, pt_->point_uav.y*step, pt_->point_uav.z*step, pt_->length_cat, pt_->cost);
		}
		getPathMarker(rrt_path);
		getCatenaryPathMarker(rrt_path);
		ret_val = rrt_path.size();
	}
	else{
		printf("RRTStar::computeTreesIndependent -->  could't find path for Coupled Marsupial Configuration-->  number iteration: %lu \n\n", nodes_tree.size());    
		ret_val = 0;
	}

  	std::cout << "Explored Graph Nodes Numbers: " << nodes_tree.size() <<std::endl;
	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;

	// if (print_all_catenary)
	getAllCatenaryMarker();

	

  return ret_val; 
}

bool RRTStar::extendGraph(const RRTNode q_rand_)
{ 
	if(is_coupled){
		RRTNode* new_node = new RRTNode();
		RRTNode q_new ;			//Take the new node value before to save it as a node in the list

		RRTNode* q_nearest = getNearestNode(q_rand_); 
		
		 
		
		q_new = steering(*q_nearest, q_rand_, step_steer);
		
		RRTNode *q_min;
		if (checkNodeFeasibility(q_new,false)){
			if (obstacleFree(*q_nearest, q_new)){
				q_min = q_nearest;
			}
			else{
				ROS_ERROR("RRTStar::extendGraph : Not Obstacle Free between q_new = [%f %f %f] and q_nearest =[%f %f %f]", q_new.point.x*step, q_new.point.y*step, q_new.point.z*step, q_nearest->point.x*step, q_nearest->point.y*step, q_nearest->point.z*step);
				return false;
			}
		}
		else{
			ROS_ERROR("RRTStar::extendGraph : Not Feasible to extend point q_new = [%f %f %f]",q_new.point.x*step, q_new.point.y*step, q_new.point.z*step);
			return false;		
		}

		std::vector<int> v_near_nodes = getNearNodes(q_new, radius_near_nodes) ;
		q_new.parentNode = q_min;
		getParamsNode(q_new);
		updateKdtree(q_new); //KdTree is updated after get Near Nodes because to not take it own node as a near node
		
		// I . Open near nodes and connected with minimum accumulated
		for (size_t i = 0 ; i < v_near_nodes.size(); i++){
			for (auto nt_:nodes_tree) {
				if (nt_->id == v_near_nodes[i] ){
					if (obstacleFree(*nt_, q_new)){
						double C_ = nt_->cost + costBetweenNodes(*nt_,q_new);
						if (C_ < q_new.cost){
							q_min = nt_;
						}
					}
					else{
						// ROS_ERROR("RRTStar::extendGraph -->  exist collision between one of <X_near node> and <X_new node> !!");
					}
				}
			}
		}
		q_new.parentNode = q_min;
		getParamsNode(q_new);
		// double C_new_ = q_new.cost;
		
		// II . Rewire Proccess 
		for (size_t i = 0 ; i < v_near_nodes.size(); i++){
			for (auto nt_:nodes_tree) {
				if (nt_->id == v_near_nodes[i]  && nt_->id != q_min->id ){
					if (obstacleFree(*nt_, q_new)){
						if( nt_->cost > (q_new.cost + costBetweenNodes(q_new, *nt_)) ){
							*nt_->parentNode = q_new;
							nt_->cost = q_new.cost+costBetweenNodes(q_new, *nt_);
						}
					}
				}
			}
		}

		*new_node = q_new;
		saveNode(new_node);
		getGraphMarker();
		getTakeOffNodesMarker();
		isGoal(q_new);
		
		return true;
	}
	else{
		RRTNode* new_node = new RRTNode();
		RRTNode q_new;	//Take the new node value before to save it as a node in the list

		RRTNode* q_nearest = getNearestNode(q_rand_);  
		printf(" q_nearest = [%f %f %f / %f %f %f] \n",
		q_nearest->point.x*step,q_nearest->point.y*step,q_nearest->point.z*step,q_nearest->point_uav.x*step,q_nearest->point_uav.y*step,q_nearest->point_uav.z*step);
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
		// getCatenaryMarker(_points_cat_);
		/******************************************************************/


		q_new = steering(*q_nearest, q_rand_, step_steer);
		q_new.parentNode = q_nearest;
		getParamsNode(q_new);
		printf(" q_new = [%f %f %f / %f %f %f] \n",
		q_new.point.x*step,q_new.point.y*step,q_new .point.z*step,q_new.point_uav.x*step,q_new.point_uav.y*step,q_new.point_uav.z*step);
		if (q_new.catenary == false){
			ROS_ERROR("RRTStar::extendGraph : Not Catenary for new point q_new = [%f %f %f]", q_new.point.x*step, q_new.point.y*step, q_new.point.z*step);
			return false;
		}
		/********************* To graph Catenary Node *********************/
		// std::vector<geometry_msgs::Point> points_cat_;
		// points_cat_.clear();
		// geometry_msgs::Point p_reel_, p_uav_;
		// CatenarySolver cSolver_;
		// cSolver_.setMaxNumIterations(100);
		// p_reel_ = getReelNode(q_new);
		// p_uav_.x = q_new.point_uav.x*step; 
		// p_uav_.y = q_new.point_uav.y*step; 
		// p_uav_.z = q_new.point_uav.z*step;
		// double l_cat_ = q_new.length_cat;
		// cSolver_.solve(p_reel_.x, p_reel_.y, p_reel_.z, p_uav_.x, p_uav_.y, p_uav_.z, l_cat_, points_cat_);
		// getCatenaryMarker(points_cat_);
		/******************************************************************/


		RRTNode *q_min;
		if (checkUGVFeasibility(q_new,false) && checkNodeFeasibility(q_new,true)){
			if (obstacleFree(*q_nearest, q_new)){
				q_min = q_nearest;
			}
			else{
				ROS_ERROR("RRTStar::extendGraph : Not Obstacle Free q_new=[%f %f %f]  q_nearest=[%f %f %f]",
				q_new.point.x*step, q_new.point.y*step, q_new.point.z*step, q_nearest->point.x*step, q_nearest->point.y*step, q_nearest->point.z*step);
				return false;
			}
		}
		else{
			ROS_ERROR("RRTStar::extendGraph : Not Feasible to extend point q_new = [%f %f %f]", q_new.point.x*step, q_new.point.y*step, q_new.point.z*step);
			return false;		
		}
		printf("q_min 1 = [%f %f %f / %f %f %f] \n",
		q_min->point.x*step,q_min->point.y*step,q_min->point.z*step,q_min->point_uav.x*step,q_min->point_uav.y*step,q_min->point_uav.z*step);


		std::vector<int> v_near_nodes = getNearNodes(q_new, radius_near_nodes) ;
		updateKdtree(q_new); 	//KdTree is updated after get Near Nodes because to not take it own node as a near node
		// printf("Kdtree : v_near_nodes.size=[%lu] v_nodes_kdtree=[%lu] \n",v_near_nodes.size(),v_nodes_kdtree.size());
		// // I . Open near nodes and connected with minimum accumulated for UGV
		for (size_t i = 0 ; i < v_near_nodes.size(); i++){
			// printf("For nearesNode [%lu/%lu] \n",i,v_near_nodes.size());
			for (auto nt_:nodes_tree) {
				// printf("nt_->id_uav [%i] v_near_nodes[i]=%i \n",nt_->id_uav , v_near_nodes[i]);
				if(nt_->id_uav == v_near_nodes[i] && nt_->id_uav != q_nearest->id_uav){
					// printf("get it !!\n");
					if (obstacleFree(*nt_, q_new)){
						double C_ = nt_->cost + costBetweenNodes(*nt_,q_new);   
						// printf("Cost = [%f] , nt->cost = %f , costBetweenNodes= %f \n",C_, nt_->cost, costBetweenNodes(*nt_,q_new));
						if (C_ < q_new.cost){
							q_min = nt_;
						}
					}
					break;
				}
			}
		}
		q_new.parentNode = q_min;
		getParamsNode(q_new);
		*new_node = q_new;

		/////////// TO CHECK PROCCESS //////////		
		// std::string y_ ;
		// std::cin >> y_ ;
		// std::cout << "Pressed key : " << y_ << std::endl;	
		////////////////////////////////////////		

		// ROS_ERROR("ENTER 1-1 , node [%f %f %f /%f %f %f] !!",new_node->point.x*step, new_node->point.y*step, new_node->point.z*step,
															// new_node->point_uav.x*step, new_node->point_uav.y*step, new_node->point_uav.z*step);
		// ROS_ERROR("ENTER 1-2 , node_parent [%f %f %f /%f %f %f] !!",new_node->parentNode->point.x*step, new_node->parentNode->point.y*step, new_node->parentNode->point.z*step, 
																	// new_node->parentNode->point_uav.x*step, new_node->parentNode->point_uav.y*step, new_node->parentNode->point_uav.z*step);
		// std::cout << "new_node->parentNode : " << new_node->parentNode << std::endl;
		// II . Rewire Proccess for UGV
		for (size_t i = 0 ; i < v_near_nodes.size(); i++){
			for (auto nt_:nodes_tree) {
				if(nt_->id_uav == v_near_nodes[i] && nt_->id_uav != q_min->id_uav){
					if (obstacleFree(*nt_, *new_node)){
						if( nt_->cost > (new_node->cost + costBetweenNodes(*new_node, *nt_)) ){
							nt_->parentNode = new_node;
							nt_->cost = new_node->cost + costBetweenNodes(*new_node, *nt_);
							// ROS_ERROR("ENTER 2-1 , node [%f %f %f /%f %f %f] !!",nt_->point.x*step, nt_->point.y*step, nt_->point.z*step,
							// 		nt_->point_uav.x*step, nt_->point_uav.y*step, nt_->point_uav.z*step);
							// ROS_ERROR("ENTER 2-2 , node_parent [%f %f %f /%f %f %f] !!",nt_->parentNode->point.x*step, nt_->parentNode->point.y*step, nt_->parentNode->point.z*step, 
							// 		nt_->parentNode->point_uav.x*step, nt_->parentNode->point_uav.y*step, nt_->parentNode->point_uav.z*step);
							// std::cout << " nt_ : " << nt_ << " ,  nt_->parentNode : " << nt_->parentNode << std::endl;
						}
						else{
							// ROS_ERROR("RRTStar::extendGraph -->  Not posible Rewire node [%f %f %f /%f %f %f] !!",nt_->point.x*step, nt_->point.y*step, nt_->point.z*step,
							// nt_->point_uav.x*step, nt_->point_uav.y*step, nt_->point_uav.z*step);
						}
					}
					else{
						// ROS_ERROR("RRTStar::extendGraph -->  Not posile Rewire node, Not obstacle Free between Q_new and Q_near!!");
					}
				}
			}
		}
		// ROS_ERROR("ENTER 3-1 , node [%f %f %f /%f %f %f] !!",new_node->point.x*step, new_node->point.y*step, new_node->point.z*step,
															// new_node->point_uav.x*step, new_node->point_uav.y*step, new_node->point_uav.z*step);
		// ROS_ERROR("ENTER 3-2 , node_parent [%f %f %f /%f %f %f] !!",new_node->parentNode->point.x*step, new_node->parentNode->point.y*step, new_node->parentNode->point.z*step, 
																	// new_node->parentNode->point_uav.x*step, new_node->parentNode->point_uav.y*step, new_node->parentNode->point_uav.z*step);
		// std::cout << "new_node->parentNode : " << new_node->parentNode << std::endl;
		
		
		getGraphMarker();
		getTakeOffNodesMarker();
		isGoal(q_new);
		if (got_to_goal){
			disc_goal = new_node;
			saveNode(disc_goal);
		}	
		else{
			saveNode(new_node);
		}

		return true;
	}
}

// RRTNode RRTStar::getRandomNode(bool go_to_goal_) 
// {
// 	// Random numbers
//     std::random_device rd;   // obtain a random number from hardware
//   	std::mt19937 eng(rd());  // seed the generator
//   	std::uniform_int_distribution<int> distr_x_ugv(ws_x_min, ws_x_max);  // define the range
//   	std::uniform_int_distribution<int> distr_y_ugv(ws_y_min, ws_y_max);  // define the range
//   	// std::uniform_int_distribution<int> distr_z_ugv(ws_z_min, ws_z_max);  // define the range
//   	std::uniform_int_distribution<int> distr_x_uav(ws_x_min, ws_x_max);  // define the range
//   	std::uniform_int_distribution<int> distr_y_uav(ws_y_min, ws_y_max);  // define the range
//   	std::uniform_int_distribution<int> distr_z_uav(ws_z_min+0.5*step_inv, ws_z_max);  // define the range
// 	RRTNode randomState_;
// 	bool finded_node = false;
// 	bool catenary_state = false;
// 	int x_ugv_, y_ugv_, z_ugv_,x_uav_, y_uav_, z_uav_;
// 		do{
// 			randomState_.point.x = distr_x_ugv(eng);
// 			randomState_.point.y = distr_y_ugv(eng);
// 			randomState_.point.z = (pos_tf_ugv.z + 0.2)*step_inv;
// 			finded_node = checkNodeFeasibility(randomState_,false);
// 		}while(finded_node == false);
// 	if (is_coupled){
// 		return randomState_;
// 	}
// 	else{
// 		if (!go_to_goal_){
// 			do{
// 				randomState_.point_uav.x = distr_x_uav(eng);
// 				randomState_.point_uav.y = distr_y_uav(eng);
// 				randomState_.point_uav.z = distr_z_uav(eng);
// 				finded_node = checkNodeFeasibility(randomState_,true);
// 				randomState_.parentNode = disc_initial;
// 				getParamsNode(randomState_,false);
// 				catenary_state = randomState_.catenary;
// 			}while(finded_node == false || catenary_state==false);
// 		}
// 		else{
// 			do{
// 				randomState_.point_uav.x = disc_final->point.x;
// 				randomState_.point_uav.y = disc_final->point.y;
// 				randomState_.point_uav.z = disc_final->point.z;
// 				finded_node = checkNodeFeasibility(randomState_,true);
// 				// randomState_.parentNode = disc_initial;
// 				// getParamsNode(randomState_,false);
// 				// catenary_state = randomState_.catenary;
// 				catenary_state = checkCatenary(randomState_,1);
// 				if (!catenary_state){
// 					randomState_.point.x = distr_x_ugv(eng);
// 					randomState_.point.y = distr_y_ugv(eng);
// 					randomState_.point.z = (pos_tf_ugv.z + 0.2)*step_inv;
// 					printf("Looking for new UGV position\n");
// 				}
// 			}while(finded_node == false || catenary_state==false);			
// 		}
//   	return randomState_;
// 	}
// }

RRTNode RRTStar::getRandomNode(bool go_to_goal_) 
{
	// Random numbers
    std::random_device rd;   // obtain a random number from hardware
  	std::mt19937 eng(rd());  // seed the generator
	int max_ugv = (int)v_points_ws_ugv.size() -1;
  	std::uniform_int_distribution<int> distr_ugv(0, max_ugv);  // define the range
  	std::uniform_int_distribution<int> distr_x_uav(ws_x_min, ws_x_max);  // define the range
  	std::uniform_int_distribution<int> distr_y_uav(ws_y_min, ws_y_max);  // define the range
  	std::uniform_int_distribution<int> distr_z_uav(ws_z_min+(1+0.6*step_inv), ws_z_max);  // define the range

	RRTNode randomState_;
	bool finded_node = false;
	bool catenary_state = false;
		
		// if (!go_to_goal_){
			do{
				int num_rand = distr_ugv(eng);
				randomState_.point.x = v_points_ws_ugv[num_rand].x*step_inv;
				randomState_.point.y = v_points_ws_ugv[num_rand].y*step_inv;
				randomState_.point.z = v_points_ws_ugv[num_rand].z*step_inv;  
				if (v_points_ws_ugv[num_rand].z *step_inv > disc_initial->point.z + 0.2*step_inv)
					continue;

				if (v_points_ws_ugv[num_rand].z *step_inv < 1)
					finded_node = checkUGVFeasibility(randomState_,false); 
				else 	
					finded_node = checkUGVFeasibility(randomState_,true); 
			}while(finded_node == false);
		// }
		// else{
		// 		randomState_.point.x = disc_final->point.x;
		// 		randomState_.point.y = disc_final->point.y;
		// 		randomState_.point.z = disc_initial->point.z;   
		// }

	if (is_coupled){
		return randomState_;
	}
	else{
		if (!go_to_goal_){
			do{
				randomState_.point_uav.x = distr_x_uav(eng);
				randomState_.point_uav.y = distr_y_uav(eng);
				randomState_.point_uav.z = distr_z_uav(eng);
				finded_node = checkNodeFeasibility(randomState_,true);
			}while(finded_node == false);
		}
		else{
			do{
				randomState_.point_uav.x = disc_final->point.x;
				randomState_.point_uav.y = disc_final->point.y;
				randomState_.point_uav.z = disc_final->point.z;
				finded_node = checkNodeFeasibility(randomState_,true);
			}while(finded_node == false);
		}
  	return randomState_;
	}
}

// RRTNode* RRTStar::getNearestNode(const RRTNode q_rand_) 
// {
//   	RRTNode q_nearest_; 
// 	Eigen::Vector3d Vqn_ugv_, Vqn_uav_, ret_;
// 	int id_node_near_;
// 	if(is_coupled){ 
// 		Vqn_ugv_.x() = q_rand_.point.x;
// 		Vqn_ugv_.y() = q_rand_.point.y;
// 		Vqn_ugv_.z() = q_rand_.point.z;
// 		ret_ = near_neighbor_nodes_ugv.nearestObstacleVertex(near_neighbor_nodes_ugv.kdtree, Vqn_ugv_ ,near_neighbor_nodes_ugv.obs_points);
// 			for (auto nt_:nodes_tree) {
// 				if (nt_->id == id_node_near_){
// 					q_nearest_ = *nt_;
// 					break;
// 			}
// 		}
// 	}
// 	else{
// 		int p_ugv_x_, p_ugv_y_, p_ugv_z_, p_uav_x_, p_uav_y_, p_uav_z_;
// 		p_ugv_x_ = q_rand_.point.x;
// 		p_ugv_y_ = q_rand_.point.y;
// 		p_ugv_z_ = q_rand_.point.z;
// 		p_uav_x_ = q_rand_.point_uav.x;
// 		p_uav_y_ = q_rand_.point_uav.y;
// 		p_uav_z_ = q_rand_.point_uav.z;
// 		point_t pt_;
// 		pt_ = {p_ugv_x_, p_ugv_y_, p_ugv_z_, p_uav_x_, p_uav_y_, p_uav_z_};
// 		KDTree trees(v_nodes_kdtree);
// 		auto res = trees.nearest_point(pt_);
// 		// v_nodes_kdtree.push_back(pt_);
// 		int i_= 0;
// 		std::vector<int> values_;
// 		values_.clear();
// 		for (int b : res) {
// 			values_.push_back(b);
// 			i_++;
// 		}
// 		int x_ugv_ = values_[0];
// 		int y_ugv_ = values_[1];
// 		int z_ugv_ = values_[2];
// 		int x_uav_ = values_[3];
// 		int y_uav_ = values_[4];
// 		int z_uav_ = values_[5];
// 		int id_ugv_ = getWorldIndex(x_ugv_, y_ugv_, z_ugv_);
// 		int id_uav_ = getWorldIndex(x_uav_, y_uav_, z_uav_);
// 		for(auto nt_: nodes_tree){
// 			if(nt_->id == id_ugv_ && nt_->id_uav == id_uav_){
// 				q_nearest_ = *nt_;
// 				break;
// 			}
// 		}
// 	}
// 	RRTNode *q_n_ = new RRTNode(q_nearest_); 
// 	return q_n_;	
// }

// RRTNode* RRTStar::getNearestNode(const RRTNode q_rand_) 
// {
//   	RRTNode q_nearest_; 
// 	Eigen::Vector3d Vqn_ugv_, Vqn_uav_, ret_;
// 	int point_ugv_x_, point_ugv_y_, point_ugv_z_, point_uav_x_, point_uav_y_, point_uav_z_;
// 	int id_node_near_;
// 	if(is_coupled){ 
// 		Vqn_ugv_.x() = q_rand_.point.x;
// 		Vqn_ugv_.y() = q_rand_.point.y;
// 		Vqn_ugv_.z() = q_rand_.point.z;
// 		ret_ = near_neighbor_nodes_ugv.nearestObstacleVertex(near_neighbor_nodes_ugv.kdtree, Vqn_ugv_ ,near_neighbor_nodes_ugv.obs_points);
// 		point_ugv_x_ = ret_.x();  
// 		point_ugv_y_ = ret_.y();  
// 		point_ugv_z_ = ret_.z();  
// 		id_node_near_ = getWorldIndex(point_ugv_x_, point_ugv_y_, point_ugv_z_);
// 			for (auto nt_:nodes_tree) {
// 				if (nt_->id == id_node_near_){
// 					q_nearest_ = *nt_;
// 					break;
// 			}
// 		}
// 	}
// 	else{
// 		RRTNode node_;
// 		Vqn_ugv_.x() = q_rand_.point.x;
// 		Vqn_ugv_.y() = q_rand_.point.y;
// 		Vqn_ugv_.z() = q_rand_.point.z;
// 		ret_ = near_neighbor_nodes_ugv.nearestObstacleVertex(near_neighbor_nodes_ugv.kdtree, Vqn_ugv_ ,near_neighbor_nodes_ugv.obs_points);
// 		point_ugv_x_ = ret_.x();  
// 		point_ugv_y_ = ret_.y();  
// 		point_ugv_z_ = ret_.z();  
// 		id_node_near_ = getWorldIndex(point_ugv_x_, point_ugv_y_, point_ugv_z_);
// 			for (auto nt_:nodes_tree) {
// 				if (nt_->id == id_node_near_){
// 					node_.point.x = nt_->point.x;
// 					node_.point.y = nt_->point.y;
// 					node_.point.z = nt_->point.z;
// 					break;
// 			}
// 		}
// 		Vqn_uav_.x() = q_rand_.point_uav.x;
// 		Vqn_uav_.y() = q_rand_.point_uav.y;
// 		Vqn_uav_.z() = q_rand_.point_uav.z;
// 		ret_ = near_neighbor_nodes_uav.nearestObstacleVertex(near_neighbor_nodes_uav.kdtree, Vqn_uav_ ,near_neighbor_nodes_uav.obs_points);
// 		point_uav_x_ = ret_.x();  
// 		point_uav_y_ = ret_.y();  
// 		point_uav_z_ = ret_.z();  
// 		id_node_near_ = getWorldIndex(point_uav_x_, point_uav_y_, point_uav_z_);
// 		for (auto nt_:nodes_tree) {
// 			if (nt_->id_uav == id_node_near_){
// 				q_nearest_ = *nt_;
// 				node_.point_uav.x = nt_->point_uav.x;
// 				node_.point_uav.y = nt_->point_uav.y;
// 				node_.point_uav.z = nt_->point_uav.z;
// 				node_.parentNode = nt_;
// 				break;
// 			}
// 		}
// 		if(checkCatenary(node_, 2)){
// 			getParamsNode(node_);
// 			RRTNode* int_node_ = new RRTNode();
// 			*int_node_ = node_;
// 			saveNode(int_node_);
// 			// node_.id = getWorldIndex(node_.point.x, node_.point.y, node_.point.z);
// 			// node_.id_uav = getWorldIndex(node_.point_uav.x, node_.point_uav.y, node_.point_uav.z);
// 			q_nearest_ = node_;
// 			// printf("Probando el invento\n");
// 		}
// 	}
// 	RRTNode *q_n_ = new RRTNode(q_nearest_); 
// 	return q_n_;	
// }

RRTNode* RRTStar::getNearestNode(const RRTNode q_rand_) 
{
  	RRTNode* q_nearest_; 

	double p_ugv_x_, p_ugv_y_, p_ugv_z_, p_uav_x_, p_uav_y_, p_uav_z_; 
	double d_ugv_ , d_uav_, l_cat_;
	double k0_ ,k1_, k2_ ;
	k0_ = 6.0; // 5.0
	k1_ = 12.0;  // 15.0
	k2_ = 5.0;  // 5.0

	double cost_init_ = 10000000;
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
		if(cost_init_ > cost_){
			q_nearest_ = nt_;
			cost_init_ = cost_;
		}
	}

	return q_nearest_;
}

// RRTNode RRTStar::steering(const RRTNode &q_nearest_, const RRTNode &q_rand_, float factor_steer_)
// {
// 	RRTNode q_new_;
// 	float x_rand_ugv, y_rand_ugv, z_rand_ugv; 
// 	float x_nearest_ugv, y_nearest_ugv, z_nearest_ugv;
// 	float dir_ugv_x, dir_ugv_y, dir_ugv_z, uni_ugv_x, uni_ugv_y, uni_ugv_z; 
// 	float x_ugv_, y_ugv_, z_ugv_;
// 	float dist_nearest_rand;
// 	x_rand_ugv = q_rand_.point.x * step; 
// 	y_rand_ugv = q_rand_.point.y * step; 
// 	z_rand_ugv = q_rand_.point.z * step;
// 	x_nearest_ugv = q_nearest_.point.x * step; 
// 	y_nearest_ugv = q_nearest_.point.y * step; 
// 	z_nearest_ugv = q_nearest_.point.z * step;
// 	//Get the unitary vector from nearest to rand direction
// 	dir_ugv_x = x_rand_ugv - x_nearest_ugv;
// 	dir_ugv_y = y_rand_ugv - y_nearest_ugv; 
// 	dir_ugv_z = z_rand_ugv - z_nearest_ugv;
// 	dist_nearest_rand = sqrt(dir_ugv_x*dir_ugv_x + dir_ugv_y*dir_ugv_y + dir_ugv_z*dir_ugv_z);
// 	uni_ugv_x = dir_ugv_x/ dist_nearest_rand;
// 	uni_ugv_y = dir_ugv_y/ dist_nearest_rand;
// 	uni_ugv_z = dir_ugv_z/ dist_nearest_rand;
// 	// Move in direction nearest to rand with magnitude proporcional to factor_steer_
// 	x_ugv_ = (x_nearest_ugv + uni_ugv_x * factor_steer_); 
// 	y_ugv_ = (y_nearest_ugv + uni_ugv_y * factor_steer_); 
// 	z_ugv_ = (z_nearest_ugv + uni_ugv_z * factor_steer_); 	
// 	if ( (pow(x_ugv_ - x_nearest_ugv,2)+pow(y_ugv_ - y_nearest_ugv,2)+pow(z_ugv_ - z_nearest_ugv,2)) > (pow(x_rand_ugv - x_nearest_ugv,2)+pow(y_rand_ugv - y_nearest_ugv,2)+pow(z_rand_ugv - z_nearest_ugv,2)) ){
// 		q_new_.point.x = q_rand_.point.x ; 
// 		q_new_.point.y = q_rand_.point.y ; 
// 		q_new_.point.z = q_rand_.point.z;
// 	}
// 	else{
// 		q_new_.point.x = x_ugv_ * step_inv; 
// 		q_new_.point.y = y_ugv_ * step_inv; 
// 		q_new_.point.z = z_ugv_ * step_inv;
// 	}	
// 	if(is_coupled){
// 		return q_new_;
// 	}
// 	else{
// 		float x_rand_uav, y_rand_uav, z_rand_uav; 
// 		float x_nearest_uav, y_nearest_uav, z_nearest_uav;
// 		float dir_uav_x, dir_uav_y, dir_uav_z, uni_uav_x, uni_uav_y, uni_uav_z; 
// 		float x_uav_, y_uav_, z_uav_;
// 		x_rand_uav = q_rand_.point_uav.x * step; 
// 		y_rand_uav = q_rand_.point_uav.y * step; 
// 		z_rand_uav = q_rand_.point_uav.z * step;
// 		x_nearest_uav = q_nearest_.point_uav.x * step; 
// 		y_nearest_uav = q_nearest_.point_uav.y * step; 
// 		z_nearest_uav = q_nearest_.point_uav.z * step;		
// 		//Get the unitary vector from nearest to rand direction
// 		dir_uav_x = x_rand_uav - x_nearest_uav;
// 		dir_uav_y = y_rand_uav - y_nearest_uav; 
// 		dir_uav_z = z_rand_uav - z_nearest_uav;	
// 		dist_nearest_rand = sqrt(dir_uav_x*dir_uav_x + dir_uav_y*dir_uav_y + dir_uav_z*dir_uav_z);
// 		uni_uav_x = dir_uav_x/ dist_nearest_rand;
// 		uni_uav_y = dir_uav_y/ dist_nearest_rand;
// 		uni_uav_z = dir_uav_z/ dist_nearest_rand;
// 		// Move in direction nearest to rand with magnitude proporcional to factor_steer_
// 		x_uav_ = (x_nearest_uav + uni_uav_x * factor_steer_); 
// 		y_uav_ = (y_nearest_uav + uni_uav_y * factor_steer_); 
// 		z_uav_ = (z_nearest_uav + uni_uav_z * factor_steer_); 
// 		if((pow(x_uav_ - x_nearest_uav,2)+pow(y_uav_ - y_nearest_uav,2)+pow(z_uav_ - z_nearest_uav,2)) > (pow(x_rand_uav - x_nearest_uav,2)+pow(y_rand_uav - y_nearest_uav,2)+pow(z_rand_uav - z_nearest_uav,2)) ){
// 			q_new_.point_uav.x = q_rand_.point_uav.x ; 
// 			q_new_.point_uav.y = q_rand_.point_uav.y ; 
// 			q_new_.point_uav.z = q_rand_.point_uav.z;
// 		}
// 		else{
// 			q_new_.point_uav.x = x_uav_ * step_inv; 
// 			q_new_.point_uav.y = y_uav_ * step_inv; 
// 			q_new_.point_uav.z = z_uav_ * step_inv;
// 		}
// 		return q_new_;
// 	}
// }

RRTNode RRTStar::steering(const RRTNode &q_nearest_, const RRTNode &q_rand_, float factor_steer_)	
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
	
	// dist_nearest_rand = sqrt(dir_ugv_x*dir_ugv_x + dir_ugv_y*dir_ugv_y + dir_ugv_z*dir_ugv_z);
	dist_nearest_rand = sqrt(dir_ugv_x*dir_ugv_x + dir_ugv_y*dir_ugv_y );
	uni_ugv_x = dir_ugv_x/ dist_nearest_rand;
	uni_ugv_y = dir_ugv_y/ dist_nearest_rand;
	uni_ugv_z = dir_ugv_z/ dist_nearest_rand;

	// Move in direction nearest to rand with magnitude proporcional to factor_steer_
	x_ugv_ = (x_nearest_ugv + uni_ugv_x * factor_steer_); 
	y_ugv_ = (y_nearest_ugv + uni_ugv_y * factor_steer_); 
	// z_ugv_ = (z_nearest_ugv + uni_ugv_z * factor_steer_); 
	z_ugv_ = (z_nearest_ugv); 
	
	//Check if the position for ugv is in the air, in case is true, we keep as ugv_point the q_nearest_ugv position and we only steer in q_new_uav
	bool node_in_air_ = false;
	if(z_ugv_ > 0.2 && isUGVInside(x_ugv_*step_inv, y_ugv_*step_inv, z_ugv_*step_inv)){
		RRTNode n_air_;
		n_air_.point.x = x_ugv_ * step_inv;
		n_air_.point.y = y_ugv_ * step_inv;
		n_air_.point.z = z_ugv_ * step_inv;
		if(!isOccupied(n_air_))
		{
			node_in_air_ = true;
			ROS_ERROR("new node steering to the air , the keeped the point q_nearest_ugv in q_new_ugv");
		}

	}

	if ( (pow(x_ugv_ - x_nearest_ugv,2) + pow(y_ugv_ - y_nearest_ugv,2) + pow(z_ugv_ - z_nearest_ugv,2)) > 
		 (pow(x_rand_ugv - x_nearest_ugv,2) + pow(y_rand_ugv - y_nearest_ugv,2) + pow(z_rand_ugv - z_nearest_ugv,2)) ){
		q_new_.point.x = q_rand_.point.x ; 
		q_new_.point.y = q_rand_.point.y ; 
		q_new_.point.z = q_rand_.point.z;
	}
	else if(node_in_air_){
		q_new_.point.x = q_nearest_.point.x ; 
		q_new_.point.y = q_nearest_.point.y ; 
		q_new_.point.z = q_nearest_.point.z;
	}
	else{
		q_new_.point.x = x_ugv_ * step_inv; 
		q_new_.point.y = y_ugv_ * step_inv; 
		q_new_.point.z = z_ugv_ * step_inv;
	}
	
	if(is_coupled){
		return q_new_;
	}
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
		uni_uav_x = dir_uav_x/ dist_nearest_rand;
		uni_uav_y = dir_uav_y/ dist_nearest_rand;
		uni_uav_z = dir_uav_z/ dist_nearest_rand;

		// Move in direction nearest to rand with magnitude proporcional to factor_steer_
		x_uav_ = (x_nearest_uav + uni_uav_x * factor_steer_); 
		y_uav_ = (y_nearest_uav + uni_uav_y * factor_steer_); 
		z_uav_ = (z_nearest_uav + uni_uav_z * factor_steer_); 

		if((pow(x_uav_ - x_nearest_uav,2)+pow(y_uav_ - y_nearest_uav,2)+pow(z_uav_ - z_nearest_uav,2)) > (pow(x_rand_uav - x_nearest_uav,2)+pow(y_rand_uav - y_nearest_uav,2)+pow(z_rand_uav - z_nearest_uav,2)) ){
			q_new_.point_uav.x = q_rand_.point_uav.x ; 
			q_new_.point_uav.y = q_rand_.point_uav.y ; 
			q_new_.point_uav.z = q_rand_.point_uav.z;
		}
		else{
			q_new_.point_uav.x = x_uav_ * step_inv; 
			q_new_.point_uav.y = y_uav_ * step_inv; 
			q_new_.point_uav.z = z_uav_ * step_inv;
		}

		return q_new_;
	}
	
}

// bool RRTStar::obstacleFree(const RRTNode q_nearest_,const RRTNode q_new_)
// {
// 	std::vector<geometry_msgs::Point> pos_ugv_between_nodes, pos_uav_between_nodes;
// 	geometry_msgs::Point point_, point_nearest_ , point_new_;
// 	double  uni_ugv_x, uni_ugv_y, uni_ugv_z, dir_ugv_x, dir_ugv_y, dir_ugv_z; 
// 	double dist_nearest_new_;
// 	bool same_point_ = false;
// 	pos_ugv_between_nodes.clear(); 
// 	pos_uav_between_nodes.clear();
// 	//First part: That check no obtacles between UGV nodes - new and nearest
// 	point_new_.x = q_new_.point.x * step; 
// 	point_new_.y = q_new_.point.y * step; 
// 	point_new_.z = q_new_.point.z * step;
// 	point_nearest_.x = q_nearest_.point.x * step; 
// 	point_nearest_.y = q_nearest_.point.y * step; 
// 	point_nearest_.z = q_nearest_.point.z * step;	
// 	//Get the unitary vector from nearest to rand direction
// 	dir_ugv_x = point_new_.x - point_nearest_.x;
// 	dir_ugv_y = point_new_.y - point_nearest_.y; 
// 	dir_ugv_z = point_new_.z - point_nearest_.z;
// 	dist_nearest_new_ = sqrt(dir_ugv_x*dir_ugv_x + dir_ugv_y*dir_ugv_y + dir_ugv_z*dir_ugv_z);
// 	if (dist_nearest_new_ > 0.001){
// 		uni_ugv_x = dir_ugv_x/ dist_nearest_new_;
// 		uni_ugv_y = dir_ugv_y/ dist_nearest_new_;
// 		uni_ugv_z = dir_ugv_z/ dist_nearest_new_;
// 		// Move in direction nearest to rand with magnitude proporcional to step_
// 		int i_ = 0;
// 		do{
// 			i_ ++;
// 			point_.x = point_nearest_.x + uni_ugv_x * step * (double)i_; 
// 			point_.y = point_nearest_.y + uni_ugv_y * step * (double)i_; 
// 			point_.z = point_nearest_.z + uni_ugv_z * step * (double)i_; 			
// 			RRTNode check_point_;
// 			check_point_.point.x = point_.x*step_inv;
// 			check_point_.point.y = point_.y*step_inv;
// 			check_point_.point.z = point_.z*step_inv;	
// 			if (!checkNodeFeasibility(check_point_,false))
// 				return false;
// 			pos_ugv_between_nodes.push_back(point_);
// 		}while ( ( pow(point_.x - point_new_.x,2) + pow(point_.y - point_new_.y,2) + pow(point_.z - point_new_.z,2)) > step*step);
// 	}
// 	else{
// 		// ROS_INFO("RRTStar::obstacleFree : UGV node without collision because Q_new == Q_nearest");
// 	}
// 	if (is_coupled){
// 		return true;
// 	}
// 	else{	//Second part: That check no obtacles between UAV nodes - new and nearest
// 		double  uni_uav_x, uni_uav_y, uni_uav_z, dir_uav_x, dir_uav_y, dir_uav_z; 
// 		point_new_.x = q_new_.point_uav.x * step; 
// 		point_new_.y = q_new_.point_uav.y * step; 
// 		point_new_.z = q_new_.point_uav.z * step;
// 		point_nearest_.x = q_nearest_.point_uav.x * step; 
// 		point_nearest_.y = q_nearest_.point_uav.y * step; 
// 		point_nearest_.z = q_nearest_.point_uav.z * step;		
// 		//Get the unitary vector from nearest to rand direction
// 		dir_uav_x = point_new_.x - point_nearest_.x;
// 		dir_uav_y = point_new_.y - point_nearest_.y; 
// 		dir_uav_z = point_new_.z - point_nearest_.z;
// 		dist_nearest_new_ = sqrt(dir_uav_x*dir_uav_x + dir_uav_y*dir_uav_y + dir_uav_z*dir_uav_z);
// 		if (dist_nearest_new_ > 0.001){
// 			uni_uav_x = dir_uav_x/ dist_nearest_new_;
// 			uni_uav_y = dir_uav_y/ dist_nearest_new_;
// 			uni_uav_z = dir_uav_z/ dist_nearest_new_;
// 			// Move in direction nearest to rand with magnitude proporcional to step_
// 			int i_ = 0;
// 			do{
// 				i_ ++;
// 				point_.x = point_nearest_.x + uni_uav_x * step * (double)i_; 
// 				point_.y = point_nearest_.y + uni_uav_y * step * (double)i_; 
// 				point_.z = point_nearest_.z + uni_uav_z * step * (double)i_; 				
// 				RRTNode check_point_;
// 				check_point_.point_uav.x = point_.x*step_inv;
// 				check_point_.point_uav.y = point_.y*step_inv;
// 				check_point_.point_uav.z = point_.z*step_inv;	
// 				if (!checkNodeFeasibility(check_point_,true))
// 					return false;
// 				pos_uav_between_nodes.push_back(point_);
// 			}while ( ( pow(point_.x - point_new_.x,2) + pow(point_.y - point_new_.y,2) + pow(point_.z - point_new_.z,2)) > step*step);
// 			double r_;
// 			if(pos_ugv_between_nodes.size() > pos_uav_between_nodes.size()){
// 				r_ = (double)(pos_uav_between_nodes.size()-1.0)/(double)(pos_ugv_between_nodes.size()-1.0);
// 				for(size_t i = 0 ; i < pos_ugv_between_nodes.size() ; i++){
// 					int k_ = (int)(round(r_ * (i)));
// 					RRTNode cN_;
// 					cN_.point.x = pos_ugv_between_nodes[i].x*step_inv;
// 					cN_.point.y = pos_ugv_between_nodes[i].y*step_inv;
// 					cN_.point.z = pos_ugv_between_nodes[i].z*step_inv;
// 					cN_.point_uav.x = pos_uav_between_nodes[k_].x*step_inv;
// 					cN_.point_uav.y = pos_uav_between_nodes[k_].y*step_inv;
// 					cN_.point_uav.z = pos_uav_between_nodes[k_].z*step_inv;
// 					// printf("Here 3 i=%lu , k=%i r=%f pos_uav_between_nodes.size()= %lu , pos_ugv_between_nodes.size()=%lu\n",i,k_,r_,pos_uav_between_nodes.size(),pos_ugv_between_nodes.size());
// 					// printf("ugv=[%f %f %f]   uav=[%f %f %f]\n",cN_.point.x*step,cN_.point.y*step,cN_.point.z*step,cN_.point_uav.x*step,cN_.point_uav.y*step,cN_.point_uav.z*step);
// 					if(!checkCatenary(cN_, 2)){
// 						ROS_ERROR("A: EXIST OBSTACLE IN CATENARIES BETWEEN UGV-UAV GOING FROM NEAREST NODE TO NEW NODE");
// 						return false;
// 					}
// 				}
// 			}
// 			else if(pos_ugv_between_nodes.size() < pos_uav_between_nodes.size()){
// 				r_ = (double)(pos_ugv_between_nodes.size()-1.0)/(double)(pos_uav_between_nodes.size()-1.0);
// 				for(size_t i = 0 ; i < pos_uav_between_nodes.size() ; i++){
// 					int k_ = (int)(round(r_ * (i)));
// 					RRTNode cN_;
// 					cN_.point.x = pos_ugv_between_nodes[k_].x*step_inv;
// 					cN_.point.y = pos_ugv_between_nodes[k_].y*step_inv;
// 					cN_.point.z = pos_ugv_between_nodes[k_].z*step_inv;
// 					cN_.point_uav.x = pos_uav_between_nodes[i].x*step_inv;
// 					cN_.point_uav.y = pos_uav_between_nodes[i].y*step_inv;
// 					cN_.point_uav.z = pos_uav_between_nodes[i].z*step_inv;
// 					// printf("Here 4 i=%lu , k=%i r=%f pos_uav_between_nodes.size()= %lu pos_ugv_between_nodes.size()=%lu\n",i,k_,r_,pos_uav_between_nodes.size(),pos_ugv_between_nodes.size());
// 					// printf("ugv=[%f %f %f]   uav=[%f %f %f]\n",cN_.point.x*step,cN_.point.y*step,cN_.point.z*step,cN_.point_uav.x*step,cN_.point_uav.y*step,cN_.point_uav.z*step);
// 					if(!checkCatenary(cN_, 2)){
// 						ROS_ERROR("B: EXIST OBSTACLE IN CATENARIES BETWEEN UGV-UAV GOING FROM NEAREST NODE TO NEW NODE");
// 						return false;
// 					}
// 				}
// 			}
// 			else{
// 				for(size_t i = 0 ; i < pos_uav_between_nodes.size() ; i++){
// 					RRTNode cN_;
// 					cN_.point.x = pos_ugv_between_nodes[i].x*step_inv;
// 					cN_.point.y = pos_ugv_between_nodes[i].y*step_inv;
// 					cN_.point.z = pos_ugv_between_nodes[i].z*step_inv;
// 					cN_.point_uav.x = pos_uav_between_nodes[i].x*step_inv;
// 					cN_.point_uav.y = pos_uav_between_nodes[i].y*step_inv;
// 					cN_.point_uav.z = pos_uav_between_nodes[i].z*step_inv;
// 					// printf("Here 5 i=%lu pos_uav_between_nodes.size()= %lu pos_ugv_between_nodes.size()=%lu\n",i,pos_uav_between_nodes.size(),pos_ugv_between_nodes.size());
// 					// printf("ugv=[%f %f %f]   uav=[%f %f %f]\n",cN_.point.x*step,cN_.point.y*step,cN_.point.z*step,cN_.point_uav.x*step,cN_.point_uav.y*step,cN_.point_uav.z*step);
// 					if(!checkCatenary(cN_, 2)){
// 						ROS_ERROR("C: EXIST OBSTACLE IN CATENARIES BETWEEN UGV-UAV GOING FROM NEAREST NODE TO NEW NODE");
// 						return false;
// 					}
// 				}
// 			}
// 		}
// 		else{
// 			ROS_INFO("RRTStar::obstacleFree : UGV node without collision because Q_new == Q_nearest");
// 	}
// 		return true;	
// 	}
// }

bool RRTStar::obstacleFree(const RRTNode q_nearest_,const RRTNode q_new_)
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

	p_reel_nearest_ = getReelNode(q_nearest_);
	double l_cat_nearest_ = q_nearest_.length_cat;
	cSolver_.solve(p_reel_nearest_.x, p_reel_nearest_.y, p_reel_nearest_.z, point_nearest_uav_.x, point_nearest_uav_.y, point_nearest_uav_.z, l_cat_nearest_, points_cat_nearest_);

	double r_;

	// std::vector<geometry_msgs::Point> points_obstacles_;
	geometry_msgs::Point point_obs_;
	// points_obstacles_.clear();
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
			// printf("points_cat_nearest_[i]=[%f %f %f] points_cat_new_[i]=[%f %f %f]\n",
			// points_cat_nearest_[k_].x, points_cat_nearest_[k_].y, points_cat_nearest_[k_].z,points_cat_new_[i].x, points_cat_new_[i].y, points_cat_new_[i].z);
			// printf("r_ = %f  , i =%lu  ,  k_ =%i , dist_nearest_new_= %f , dir=[%f %f %f]  uni_=[%f %f %f] \n ",r_, i, k_, dist_nearest_new_,dir_x_, dir_y_, dir_z_, uni_x_,uni_y_,uni_z_);
			RRTNode check_point_;
			do{
				j_++;
				check_point_.point.x = (points_cat_new_[i].x + uni_x_ * step * (double)j_)*step_inv; 
				check_point_.point.y = (points_cat_new_[i].y + uni_y_ * step * (double)j_)*step_inv; 
				check_point_.point.z = (points_cat_new_[i].z + uni_z_ * step * (double)j_)*step_inv; 
				point_obs_.x = (points_cat_new_[i].x + uni_x_ * step * (double)j_);
				point_obs_.y = (points_cat_new_[i].y + uni_y_ * step * (double)j_);
				point_obs_.z = (points_cat_new_[i].z + uni_z_ * step * (double)j_);
				//To graph point looking for obstacle//
				// points_obstacles_.push_back(point_obs_);
				// printf("check_point_=[%i %i %i]",check_point_.point.x,check_point_.point.y,check_point_.point.z);
				// getPointsObsMarker(points_obstacles_);		
				// std::string y_ ;
				// std::cin >> y_ ;
				// std::cout << "Continue DO-WHILE loop : " << y_ << std::endl;
				///////////////////////////////////////	
				if (!checkNodeFeasibility(check_point_,false)){
					// ROS_ERROR("THERE IS A OBSTACLE BETWEEN CATENARY OF Q_NEW and Q_NEAREST");
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
			// printf("points_cat_nearest_[i]=[%f %f %f] points_cat_new_[i]=[%f %f %f]\n",
			// points_cat_nearest_[i].x, points_cat_nearest_[i].y, points_cat_nearest_[i].z,points_cat_new_[k_].x, points_cat_new_[k_].y, points_cat_new_[k_].z);
			// printf("r_ = %f  , i =%lu  ,  k_ =%i , dist_nearest_new_= %f , dir=[%f %f %f]  uni_=[%f %f %f] \n ",r_, i, k_, dist_nearest_new_,dir_x_, dir_y_, dir_z_, uni_x_,uni_y_,uni_z_);
			RRTNode check_point_;
			do{
				j_++;
				check_point_.point.x = (points_cat_new_[k_].x + uni_x_ * step * (double)j_)*step_inv; 
				check_point_.point.y = (points_cat_new_[k_].y + uni_y_ * step * (double)j_)*step_inv; 
				check_point_.point.z = (points_cat_new_[k_].z + uni_z_ * step * (double)j_)*step_inv; 
				point_obs_.x = (points_cat_new_[k_].x + uni_x_ * step * (double)j_);
				point_obs_.y = (points_cat_new_[k_].y + uni_y_ * step * (double)j_);
				point_obs_.z = (points_cat_new_[k_].z + uni_z_ * step * (double)j_);
				//To graph point looking for obstacle//
				// points_obstacles_.push_back(point_obs_);
				// printf("check_point_=[%i %i %i]",check_point_.point.x,check_point_.point.y,check_point_.point.z);
				// getPointsObsMarker(points_obstacles_);		
				// std::string y_ ;
				// std::cin >> y_ ;
				// std::cout << "Continue DO-WHILE loop : " << y_ << std::endl;
				///////////////////////////////////////		
				if (!checkNodeFeasibility(check_point_,false)){
					// ROS_ERROR("THERE IS A OBSTACLE BETWEEN CATENARY OF Q_NEW and Q_NEAREST");
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
			// printf("points_cat_nearest_[i]=[%f %f %f] points_cat_new_[i]=[%f %f %f]\n",
			// points_cat_nearest_[i].x, points_cat_nearest_[i].y, points_cat_nearest_[i].z,points_cat_new_[i].x, points_cat_new_[i].y, points_cat_new_[i].z);
			// printf("r_ = %f  , i =%lu , dist_nearest_new_= %f , dir=[%f %f %f]  uni_=[%f %f %f] \n ",r_, i,dist_nearest_new_,dir_x_, dir_y_, dir_z_, uni_x_,uni_y_,uni_z_);
			RRTNode check_point_;
			do{
				j_++;
				check_point_.point.x = (points_cat_new_[i].x + uni_x_ * step * (double)j_)*step_inv; 
				check_point_.point.y = (points_cat_new_[i].y + uni_y_ * step * (double)j_)*step_inv; 
				check_point_.point.z = (points_cat_new_[i].z + uni_z_ * step * (double)j_)*step_inv; 	
				point_obs_.x = (points_cat_new_[i].x + uni_x_ * step * (double)j_);
				point_obs_.y = (points_cat_new_[i].y + uni_y_ * step * (double)j_);
				point_obs_.z = (points_cat_new_[i].z + uni_z_ * step * (double)j_);
				//To graph point looking for obstacle//
				// points_obstacles_.push_back(point_obs_);
				// printf("check_point_=[%i %i %i]",check_point_.point.x,check_point_.point.y,check_point_.point.z);
				// getPointsObsMarker(points_obstacles_);	
				// std::string y_ ;
				// std::cin >> y_ ;
				// std::cout << "Continue DO-WHILE loop : " << y_ << std::endl;	
				///////////////////////////////////////			
				if (!checkNodeFeasibility(check_point_,false)){
					// ROS_ERROR("THERE IS A OBSTACLE BETWEEN CATENARY OF Q_NEW and Q_NEAREST");
					return false;
				}
			}while ( (  pow(point_obs_.x - points_cat_nearest_[i].x,2) + 
						pow(point_obs_.y - points_cat_nearest_[i].y,2) + 
						pow(point_obs_.z - points_cat_nearest_[i].z,2)) > step*step);
		}
	}	

	return true;	
}

// std::vector<std::vector<int>> RRTStar::getNearNodes(const RRTNode &q_new_, double radius_) 
// {
// 	std::vector<std::vector<int>> v_q_near_;
// 	std::vector<int> values_;
// 	std::vector<int> ids_;
// 	v_q_near_.clear();
// 	values_.clear();
// 	ids_.clear();
// 	if(is_coupled){
// 		point_t pt_;
// 		pt_ = {q_new_.point.x, q_new_.point.y, q_new_.point.z};
// 		KDTree tree(v_nodes_kdtree);
// 		auto nears_ = tree.neighborhood_points(pt_, radius_);
// 		for (auto nt_ : nears_) {
// 			values_.clear();
// 			for (int v_ : nt_) {
// 				values_.push_back(v_);
// 			}
// 			ids_.clear();
// 			int x_ugv_ = values_[0];
// 			int y_ugv_ = values_[1];
// 			int z_ugv_ = values_[2];
// 			int id_ugv_ = getWorldIndex(x_ugv_, y_ugv_, z_ugv_);
// 			ids_.push_back(id_ugv_);
// 			ids_.push_back(0);
// 			v_q_near_.push_back(ids_);
// 		}
// 	}
// 	else{
// 		point_t pt_;
// 		pt_ = {q_new_.point.x,q_new_.point.y,q_new_.point.z,q_new_.point_uav.x,q_new_.point_uav.y,q_new_.point_uav.z};		
// 		KDTree tree(v_nodes_kdtree);
// 		auto nears_ = tree.neighborhood_points(pt_, radius_);
// 		for (auto nt_ : nears_) {
// 			values_.clear();
// 			for (int v_ : nt_) {
// 				values_.push_back(v_);
// 			}
// 			ids_.clear();
// 			int x_ugv_ = values_[0];
// 			int y_ugv_ = values_[1];
// 			int z_ugv_ = values_[2];
// 			int x_uav_ = values_[3];
// 			int y_uav_ = values_[4];
// 			int z_uav_ = values_[5];
// 			int id_ugv_ = getWorldIndex(x_ugv_, y_ugv_, z_ugv_);
// 			int id_uav_ = getWorldIndex(x_uav_, y_uav_, z_uav_);
// 			ids_.push_back(id_ugv_);
// 			ids_.push_back(id_uav_);
// 			v_q_near_.push_back(ids_);
// 		}
//     }
// 		return v_q_near_;
// }
	
// std::vector<Eigen::Vector3d> RRTStar::getNearNodes(const RRTNode &q_nearest_, const RRTNode &q_new_, double radius_, bool check_uav_ ) 
// {
//   	std::vector<Eigen::Vector3d> v_q_near_; 
// 	v_q_near_.clear();
// 	Eigen::Vector3d Vnew_;
// 	if(!check_uav_){
// 		Vnew_.x() = q_new_.point.x;
// 		Vnew_.y() = q_new_.point.y;
// 		Vnew_.z() = q_new_.point.z;
// 		v_q_near_ = near_neighbor_nodes_ugv.radiusNearNodes(near_neighbor_nodes_ugv.kdtree, Vnew_, radius_, near_neighbor_nodes_ugv.obs_points);
// 		return v_q_near_;
// 	}
// 	else{
// 		Vnew_.x() = q_new_.point_uav.x;
// 		Vnew_.y() = q_new_.point_uav.y;
// 		Vnew_.z() = q_new_.point_uav.z;
// 		v_q_near_ = near_neighbor_nodes_ugv.radiusNearNodes(near_neighbor_nodes_ugv.kdtree, Vnew_, radius_, near_neighbor_nodes_ugv.obs_points);
// 		return v_q_near_;
// 	}
// }

std::vector<int> RRTStar::getNearNodes(const RRTNode &q_new_, double radius_) 
{
	std::vector<int> v_q_near_;
	std::vector<int> values_;
	v_q_near_.clear();
	values_.clear();

	point_t pt_;
	pt_ = {q_new_.point_uav.x, q_new_.point_uav.y, q_new_.point_uav.z};
		
	int r_ = radius_*(int) step_inv;
	printf("radius = %i , radius_ = %f , step_inv = %f\n", r_ , radius_, step_inv);
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


double RRTStar::costNode(const RRTNode q_new_)
{
	double cost_;
	double k0_, k1_, k2_, k3_, k4_; 
	double F0_, F1_, F2_, F3_;
	double r_security_ugv_ = 0.7;

	k0_ = 0.0; // For ugv : 20.0 
	k1_ = 1.0; // For uav: 5.0
	k2_ = 20.0;
	k3_ = 0.0;
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
								
		cost_ = k1_ * F0_ + k2_ * F1_ + q_new_.parentNode->cost;
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

double RRTStar::costBetweenNodes(const RRTNode q_near_, const RRTNode q_new_)
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

		k1_ = 20.0;
		k2_ = 5.0;
		double dist_ugv_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_x_*p_ugv_x_) + (p_ugv_z_*p_ugv_z_));
		double dist_uav_ =  sqrt((p_uav_x_*p_uav_x_) + (p_uav_x_*p_uav_x_) + (p_uav_z_*p_uav_z_));
		
		cost_ = k1_ * dist_ugv_ + k2_ * dist_uav_;
	}

	return cost_;
}

void RRTStar::getParamsNode(RRTNode &node_, bool is_init_)
{
	// int index_occupancy_ = getWorldIndex(node_.point.x, node_.point.y, node_.point.z);
 	double Cat1_, Cat2_;
	Cat1_ = 5.0;
	Cat2_ = 20.0;
	double r_security_cat_ = 0.1;
	
	Eigen::Vector3d point_node_, obs_near_ugv_;
	point_node_.x() = node_.point.x * step;
	point_node_.y() = node_.point.y * step;
	point_node_.z() = node_.point.z * step;
	obs_near_ugv_ = near_neighbor_obstacles.nearestObstacleMarsupial(near_neighbor_obstacles.kdtree, point_node_, near_neighbor_obstacles.obs_points);
	double dist_obs_ugv = (point_node_ - obs_near_ugv_).norm();

	Eigen::Vector3d obs_near_uav_;
	point_node_.x() = node_.point_uav.x * step;
	point_node_.y() = node_.point_uav.y * step;
	point_node_.z() = node_.point_uav.z * step;
	obs_near_uav_ = near_neighbor_obstacles.nearestObstacleMarsupial(near_neighbor_obstacles.kdtree, point_node_, near_neighbor_obstacles.obs_points);
	double dist_obs_uav = (point_node_ - obs_near_uav_).norm();

	node_.id = getWorldIndex(node_.point.x, node_.point.y, node_.point.z);
	node_.min_dist_obs_ugv = dist_obs_ugv;
	node_.min_dist_obs_uav = dist_obs_uav;

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
		// node_.cost = costNode(node_) + costBetweenNodes(node_, *node_.parentNode);
	}
}


bool RRTStar::checkUGVFeasibility(const RRTNode pf_, bool ugv_above_z_)
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

bool RRTStar::checkNodeFeasibility(const RRTNode pf_ , bool check_uav_)
{
	bool ret;

	if(check_uav_==false){
		if (isInside(pf_.point.x,pf_.point.y,pf_.point.z) ){
			if (isOccupied(pf_))
				ret = false;
			else
				ret = true;		
		}
		else
			ret = false;
		
		return ret;
	}
	else{
		if (isInside(pf_.point_uav.x,pf_.point_uav.y,pf_.point_uav.z)){
			if (isOccupied(pf_, check_uav_))
				ret = false;
			else
				ret = true;		
		}
		else
			ret = false;
		
		return ret;
	}	
}

bool RRTStar::checkPointsCatenaryFeasibility(const geometry_msgs::Point pf_)
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

// bool RRTStar::checkCatenary(RRTNode &q_init_, int mode_)
// {
// 	// mode 1: UGV-Goal  ,  mode 2: UGV-UAV
// 	geometry_msgs::Point p_reel_, p_final_;
// 	p_reel_ = getReelNode(q_init_);
// 	if(mode_ == 1){ 	
// 		p_final_.x = disc_final->point.x * step ;
// 		p_final_.y = disc_final->point.y * step ; 
// 		p_final_.z = disc_final->point.z * step ; 
// 	}
// 	else if(mode_ == 2){ 
// 		p_final_.x = q_init_.point_uav.x * step ;	
// 		p_final_.y = q_init_.point_uav.y * step ;   
// 		p_final_.z = q_init_.point_uav.z * step ;   
// 	}
// 	double dist_init_final_ = sqrt(pow(p_reel_.x - p_final_.x,2) + pow(p_reel_.y - p_final_.y,2) + pow(p_reel_.z - p_final_.z,2));
// 	double delta_ = 0.0;	//Initial Value
// 	std::vector<geometry_msgs::Point> points_catenary_;
// 	CatenarySolver cSolver_;
// 	cSolver_.setMaxNumIterations(100);
// 	bool check_catenary = true;
// 	bool founded_catenary = false;
// 	bool increase_catenary;
// 	double length_catenary_;
// 	int n_points_cat_dis_;
// 	while (check_catenary){
// 		increase_catenary = false;
// 		points_catenary_.clear();
// 		length_catenary_ = dist_init_final_* (1.001 + delta_);
// 		cSolver_.solve(p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z, length_catenary_, points_catenary_);
// 		if (points_catenary_.size() > 5){
// 			n_points_cat_dis_ = ceil(1.5*ceil(length_catenary_)); // parameter to ignore collsion points in the begining and in the end of catenary
// 			if (n_points_cat_dis_ < 5)
// 				n_points_cat_dis_ = 5;
// 			for (size_t i = 0 ; i < points_catenary_.size() ; i++){
// 				if ((i > n_points_cat_dis_ ) && (i < points_catenary_.size()-n_points_cat_dis_/2)){
// 					geometry_msgs::Point point_cat;
// 					point_cat.x = points_catenary_[i].x;
// 					point_cat.y = points_catenary_[i].y;
// 					point_cat.z = points_catenary_[i].z;
// 					Eigen::Vector3d p_in_cat_, obs_to_cat_;
// 					p_in_cat_.x() = point_cat.x;
// 					p_in_cat_.y() = point_cat.y;
// 					p_in_cat_.z() = point_cat.z;
// 					obs_to_cat_ = near_neighbor_obstacles.nearestObstacleVertex(near_neighbor_obstacles.kdtree, p_in_cat_, near_neighbor_obstacles.obs_points);
// 					double dist_cat_obs = (p_in_cat_ - obs_to_cat_).norm();
// 					q_init_.min_dist_obs_cat = dist_cat_obs;
// 					if (points_catenary_[i].z <= ws_z_min*step + 0.05){
// 						check_catenary = false;
// 						break;
// 					}
// 					if (!checkPointsCatenaryFeasibility(point_cat)){
// 						delta_ = delta_ + 0.005;
// 						increase_catenary = true;
// 						if (length_catenary_ > length_tether_max){
// 							check_catenary = false;
// 							break;
// 						}
// 						break;
// 					}
// 				}
// 			}
// 			if (check_catenary && !increase_catenary){
// 				founded_catenary = true;
// 				check_catenary = false;
// 				q_init_.length_cat = length_catenary_;
// 			}
// 		}
// 		else{
// 			check_catenary = false;
// 			ROS_ERROR("RRTStar::checkCatenary: points_catenary_.size<= 5 ,  q_init=[%f %f %f] q_final=[%f %f %f]",p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z);
// 		}
// 	}
// 	//In case not feasible to find catenary
// 	if (!founded_catenary ){
// 		q_init_.length_cat = -1.0;	
// 		q_init_.min_dist_obs_cat = -1.0;
// 		ROS_WARN("RRTStar::checkCatenary: Can't find catenary for node q_init=[%f %f %f] q_final=[%f %f %f]",p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z);
// 		if (!is_rand_node_)
// 			ROS_ERROR("RRTStar::checkCatenary: Can't find catenary for random node q_init=[%f %f %f] q_final=[%f %f %f]",p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z);
// 	}
// 	q_init_.catenary = founded_catenary;
// 	return founded_catenary;
// }

bool RRTStar::checkCatenary(RRTNode &q_init_, int mode_)
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
		// getCatenaryMarker(points_catenary_);
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
					obs_to_cat_ = near_neighbor_obstacles.nearestObstacleVertex(near_neighbor_obstacles.kdtree, p_in_cat_, near_neighbor_obstacles.obs_points);
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
				if (!checkPointsCatenaryFeasibility(point_cat)){
					delta_ = delta_ + 0.005;
					increase_catenary = true;
					// ROS_ERROR("Not Feaseble point of catenary p=[%f %f %f]",point_cat.x, point_cat.y, point_cat.z);
					break;
				}
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
			// ROS_ERROR("RRTStar::checkCatenary: points_catenary_.size<= 5 ,  q_init=[%f %f %f] q_final=[%f %f %f]",p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z);
		}
	}while (check_catenary);
	//In case not feasible to find catenary
	if (!founded_catenary ){
		q_init_.length_cat = -1.0;	
		q_init_.min_dist_obs_cat = -1.0;
		// ROS_WARN("RRTStar::checkCatenary: Can't find catenary for node q_init=[%f %f %f] q_final=[%f %f %f]",p_reel_.x, p_reel_.y, p_reel_.z, p_final_.x, p_final_.y, p_final_.z);
	}
	q_init_.catenary = founded_catenary;
	return founded_catenary;
}

geometry_msgs::Point RRTStar::getReelNode(const RRTNode &node_)
{
	geometry_msgs::Point pos_reel;
	float yaw_ugv;

	yaw_ugv = getYawFromQuat(rot_tf_ugv);
	pos_reel.x = node_.point.x*step + pos_reel_ugv.x *cos(yaw_ugv); 
	pos_reel.y = node_.point.y*step + pos_reel_ugv.y *sin(yaw_ugv);
	pos_reel.z = node_.point.z*step + pos_reel_ugv.z ;

	// RRTNode reel_node;
	// reel_node.point.x = pos_reel.x * step_inv;
	// reel_node.point.y = pos_reel.y * step_inv;
	// reel_node.point.z = pos_reel.z * step_inv;

	return pos_reel;
}


// void RRTStar::updateKdtree(const RRTNode ukT_)
// {
// 	if (is_coupled){
// 		Eigen::Vector3d Vsn_ugv_;
// 		Vsn_ugv_.x() = ukT_.point.x;  
// 		Vsn_ugv_.y() = ukT_.point.y;
// 		Vsn_ugv_.z() = ukT_.point.z;
// 		v_nodes_kdtree_ugv.push_back(Vsn_ugv_);
// 		near_neighbor_nodes_ugv.setInput(v_nodes_kdtree_ugv);
// 	}
// 	else
// 	{
// 		Eigen::Vector3d Vsn_ugv_, Vsn_uav_;
// 		Vsn_ugv_.x() = ukT_.point.x;  
// 		Vsn_ugv_.y() = ukT_.point.y;
// 		Vsn_ugv_.z() = ukT_.point.z;
// 		Vsn_uav_.x() = ukT_.point_uav.x;  
// 		Vsn_uav_.y() = ukT_.point_uav.y;
// 		Vsn_uav_.z() = ukT_.point_uav.z;
// 		v_nodes_kdtree_ugv.push_back(Vsn_ugv_);
// 		near_neighbor_nodes_ugv.setInput(v_nodes_kdtree_ugv);
// 		v_nodes_kdtree_uav.push_back(Vsn_uav_);
// 		near_neighbor_nodes_uav.setInput(v_nodes_kdtree_uav);
// 	}
// }

void RRTStar::updateKdtree(const RRTNode ukT_)
{
	point_t pt_;

	if (is_coupled){
		pt_ = {ukT_.point.x, ukT_.point.y, ukT_.point.z, ukT_.point_uav.x};
		v_nodes_kdtree.push_back(pt_);
	}
	else
	{
		pt_ = {ukT_.point_uav.x, ukT_.point_uav.y, ukT_.point_uav.z};
		v_nodes_kdtree.push_back(pt_);
	}
}

void RRTStar::readPointCloudMap(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	near_neighbor_obstacles.setInput(*msg);
	ROS_INFO_COND(debug, PRINTF_BLUE "RRTStar Planner: Receiving point cloud map to create Kdtree for Obstacles");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg,*cloud_in);

	// pcl::RandomSample<pcl::PointXYZ> rs_;
    // rs_.setInputCloud(cloud_in);
    // rs_.setSample(2);
	// rs_.filter(*cloud_out);
	// for (auto& point: *cloud_out){
    // std::cerr << "  PROBANDOOOOO  " << point.x << " "
    //                     << point.y << " "
    //                     << point.z << std::endl;
	// }

	ROS_INFO("size point cloud = [%lu]",cloud_in->size());
	geometry_msgs::Point point_;
	for (size_t i = 0 ; i < cloud_in->size() ; i ++){
		point_.x = cloud_in->points[i].x;
		point_.y = cloud_in->points[i].y;
		point_.z = cloud_in->points[i].z;
		v_points_ws_ugv.push_back(point_);
	}
	ROS_INFO("size v_points_ws_ugv = [%lu]",v_points_ws_ugv.size());
}

bool RRTStar::saveNode(RRTNode* sn_, bool is_init_)
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

inline void RRTStar::saveTakeOffNode(RRTNode* ston_)
{
	take_off_nodes.push_back(ston_); 
}

inline void RRTStar::clearNodes()
{
	for (auto nt_:nodes_tree) {
    	delete nt_;
  	}

  	for (auto ton_:take_off_nodes) {
    	delete ton_;
  	}
  
  	nodes_tree.clear();
  	take_off_nodes.clear();
  	got_to_goal = false;
}

void RRTStar::getGraphMarker()
{
    pointTreeMarkerUGV.markers.resize(nodes_tree.size());

	int count = 0; 
    for (auto nt_:nodes_tree) {
        pointTreeMarkerUGV.markers[count].header.frame_id = frame_id;
        pointTreeMarkerUGV.markers[count].header.stamp = ros::Time::now();
        pointTreeMarkerUGV.markers[count].ns = "tree_RRTStar_ugv";
        pointTreeMarkerUGV.markers[count].id = nt_->id;
        pointTreeMarkerUGV.markers[count].action = visualization_msgs::Marker::ADD;
        pointTreeMarkerUGV.markers[count].type = visualization_msgs::Marker::CYLINDER;
        pointTreeMarkerUGV.markers[count].lifetime = ros::Duration(180);
        pointTreeMarkerUGV.markers[count].pose.position.x = nt_->point.x * step; 
        pointTreeMarkerUGV.markers[count].pose.position.y = nt_->point.y * step; 
        pointTreeMarkerUGV.markers[count].pose.position.z = nt_->point.z * step ; //Move in Z to see the point over the map surface
        pointTreeMarkerUGV.markers[count].pose.orientation.x = 0.0;
        pointTreeMarkerUGV.markers[count].pose.orientation.y = 0.0;
        pointTreeMarkerUGV.markers[count].pose.orientation.z = 0.0;
        pointTreeMarkerUGV.markers[count].pose.orientation.w = 1.0;
        pointTreeMarkerUGV.markers[count].scale.x = 0.08;
        pointTreeMarkerUGV.markers[count].scale.y = 0.08;
        pointTreeMarkerUGV.markers[count].scale.z = 0.28;
        pointTreeMarkerUGV.markers[count].color.r=1.0;
        pointTreeMarkerUGV.markers[count].color.g=1.0;
        pointTreeMarkerUGV.markers[count].color.b=1.0;
        pointTreeMarkerUGV.markers[count].color.a=1.0; 
		count++;
    }	
    tree_rrt_star_ugv_pub_.publish(pointTreeMarkerUGV);

	if(!is_coupled){
		pointTreeMarkerUAV.markers.resize(nodes_tree.size());

		int count = 0; 
		for (auto nt_:nodes_tree) {
			pointTreeMarkerUAV.markers[count].header.frame_id = frame_id;
			pointTreeMarkerUAV.markers[count].header.stamp = ros::Time::now();
			pointTreeMarkerUAV.markers[count].ns = "tree_RRTStar_uav";
			pointTreeMarkerUAV.markers[count].id = nt_->id_uav;
			pointTreeMarkerUAV.markers[count].action = visualization_msgs::Marker::ADD;
			pointTreeMarkerUAV.markers[count].type = visualization_msgs::Marker::SPHERE;
			pointTreeMarkerUAV.markers[count].lifetime = ros::Duration(180);
			pointTreeMarkerUAV.markers[count].pose.position.x = nt_->point_uav.x * step; 
			pointTreeMarkerUAV.markers[count].pose.position.y = nt_->point_uav.y * step; 
			pointTreeMarkerUAV.markers[count].pose.position.z = nt_->point_uav.z * step;
			pointTreeMarkerUAV.markers[count].pose.orientation.x = 0.0;
			pointTreeMarkerUAV.markers[count].pose.orientation.y = 0.0;
			pointTreeMarkerUAV.markers[count].pose.orientation.z = 0.0;
			pointTreeMarkerUAV.markers[count].pose.orientation.w = 1.0;
			pointTreeMarkerUAV.markers[count].scale.x = 0.1;
			pointTreeMarkerUAV.markers[count].scale.y = 0.1;
			pointTreeMarkerUAV.markers[count].scale.z = 0.1;
			pointTreeMarkerUAV.markers[count].color.r=0.2;
			pointTreeMarkerUAV.markers[count].color.g=0.2;
			pointTreeMarkerUAV.markers[count].color.b=1.0;
			pointTreeMarkerUAV.markers[count].color.a=1.0; 
			count++;
		}	
		tree_rrt_star_uav_pub_.publish(pointTreeMarkerUAV);
	}
}

void RRTStar::getTakeOffNodesMarker()
{
	if(is_coupled){
		pointTakeOffMarker.markers.resize(take_off_nodes.size());

		int count_ = 0; 
		for (auto nt_:take_off_nodes) {
			pointTakeOffMarker.markers[count_].header.frame_id = frame_id;
			pointTakeOffMarker.markers[count_].header.stamp = ros::Time::now();
			pointTakeOffMarker.markers[count_].ns = "take_off_nodes_RRTStar";
			pointTakeOffMarker.markers[count_].id = nt_->id;
			pointTakeOffMarker.markers[count_].action = visualization_msgs::Marker::ADD;
			pointTakeOffMarker.markers[count_].type = visualization_msgs::Marker::SPHERE;
			pointTakeOffMarker.markers[count_].lifetime = ros::Duration(180);
			pointTakeOffMarker.markers[count_].pose.position.x = nt_->point.x * step; 
			pointTakeOffMarker.markers[count_].pose.position.y = nt_->point.y * step; 
			pointTakeOffMarker.markers[count_].pose.position.z = nt_->point.z * step;
			pointTakeOffMarker.markers[count_].pose.orientation.x = 0.0;
			pointTakeOffMarker.markers[count_].pose.orientation.y = 0.0;
			pointTakeOffMarker.markers[count_].pose.orientation.z = 0.0;
			pointTakeOffMarker.markers[count_].pose.orientation.w = 1.0;
			pointTakeOffMarker.markers[count_].scale.x = 0.1;
			pointTakeOffMarker.markers[count_].scale.y = 0.1;
			pointTakeOffMarker.markers[count_].scale.z = 0.1;
			pointTakeOffMarker.markers[count_].color.r=1.0;
			pointTakeOffMarker.markers[count_].color.g=0.0;
			pointTakeOffMarker.markers[count_].color.b=0.0;
			pointTakeOffMarker.markers[count_].color.a=0.5; 
			count_++;
		}	
		take_off_nodes_pub_.publish(pointTakeOffMarker);
	}
}

void RRTStar::getPathMarker(std::list<RRTNode*> pt_)
{
	geometry_msgs::Point _p1, _p2; 

	lines_ugv_marker_.markers.resize(pt_.size()-1);
	
	int i_ = 0;
	for (auto p_:pt_){
		_p2.x = p_->point.x*step;
		_p2.y = p_->point.y*step;
		_p2.z = p_->point.z*step+0.1;
		if (i_ > 0){
			lines_ugv_marker_.markers[i_-1].header.frame_id = frame_id;
			lines_ugv_marker_.markers[i_-1].header.stamp = ros::Time::now();
			lines_ugv_marker_.markers[i_-1].ns = "Line_ugv_RRTStar_Path";
			lines_ugv_marker_.markers[i_-1].id = i_ + pt_.size();
			lines_ugv_marker_.markers[i_-1].action = visualization_msgs::Marker::ADD;
			lines_ugv_marker_.markers[i_-1].type = visualization_msgs::Marker::LINE_STRIP;
			lines_ugv_marker_.markers[i_-1].lifetime = ros::Duration(180);
			lines_ugv_marker_.markers[i_-1].points.push_back(_p1);
			lines_ugv_marker_.markers[i_-1].points.push_back(_p2);
			lines_ugv_marker_.markers[i_-1].pose.orientation.x = 0.0;
			lines_ugv_marker_.markers[i_-1].pose.orientation.y = 0.0;
			lines_ugv_marker_.markers[i_-1].pose.orientation.z = 0.0;
			lines_ugv_marker_.markers[i_-1].pose.orientation.w = 1.0;
			lines_ugv_marker_.markers[i_-1].scale.x = 0.1;
			// lines_ugv_marker_.markers[i].scale.y = 0.3;
			// lines_ugv_marker_.markers[i].scale.z = 0.1;
			lines_ugv_marker_.markers[i_-1].color.a = 1.0;
			lines_ugv_marker_.markers[i_-1].color.r = 0.0;
			lines_ugv_marker_.markers[i_-1].color.g = 0.0;
			lines_ugv_marker_.markers[i_-1].color.b = 1.0;
		}
		_p1.x = p_->point.x*step;
		_p1.y = p_->point.y*step;
		_p1.z = p_->point.z*step+0.1;	//Move in Z to see the point over the map surface
		i_++;
	}
	lines_ugv_marker_pub_.publish(lines_ugv_marker_);

	if(!is_coupled){
		lines_uav_marker_.markers.resize(pt_.size()-1);
		i_ = 0;

		for (auto p_:pt_){
			_p2.x = p_->point_uav.x*step;
			_p2.y = p_->point_uav.y*step;
			_p2.z = p_->point_uav.z*step;
			if (i_ > 0){
				lines_uav_marker_.markers[i_-1].header.frame_id = frame_id;
				lines_uav_marker_.markers[i_-1].header.stamp = ros::Time::now();
				lines_uav_marker_.markers[i_-1].ns = "Line_uav_RRTStar_Path";
				lines_uav_marker_.markers[i_-1].id = i_ + pt_.size();
				lines_uav_marker_.markers[i_-1].action = visualization_msgs::Marker::ADD;
				lines_uav_marker_.markers[i_-1].type = visualization_msgs::Marker::LINE_STRIP;
				lines_uav_marker_.markers[i_-1].lifetime = ros::Duration(180);
				lines_uav_marker_.markers[i_-1].points.push_back(_p1);
				lines_uav_marker_.markers[i_-1].points.push_back(_p2);
				lines_uav_marker_.markers[i_-1].pose.orientation.x = 0.0;
				lines_uav_marker_.markers[i_-1].pose.orientation.y = 0.0;
				lines_uav_marker_.markers[i_-1].pose.orientation.z = 0.0;
				lines_uav_marker_.markers[i_-1].pose.orientation.w = 1.0;
				lines_uav_marker_.markers[i_-1].scale.x = 0.1;
				// lines_uav_marker_.markers[i].scale.y = 0.3;
				// lines_uav_marker_.markers[i].scale.z = 0.1;
				lines_uav_marker_.markers[i_-1].color.a = 1.0;
				lines_uav_marker_.markers[i_-1].color.r = 1.0;
				lines_uav_marker_.markers[i_-1].color.g = 1.0;
				lines_uav_marker_.markers[i_-1].color.b = 1.0;
			}
			_p1.x = p_->point_uav.x*step;
			_p1.y = p_->point_uav.y*step;
			_p1.z = p_->point_uav.z*step;
			i_++;
		}
		lines_uav_marker_pub_.publish(lines_uav_marker_);
	}
}

void RRTStar::getCatenaryMarker(std::vector<geometry_msgs::Point> points_catenary_){
	std::string string_marker;
    std::string ns_marker;

	double c_color1, c_color2, c_color3;
	visualization_msgs::MarkerArray oneCatenaryMarker;
	
	oneCatenaryMarker.markers.clear();
	oneCatenaryMarker.markers.resize(points_catenary_.size());

	for (size_t i = 0 ; i < points_catenary_.size() ; i++ ) {
		c_color1 = ((double)i / (double)points_catenary_.size())*0.5;
		c_color2 = ((double)i / (double)points_catenary_.size())*0.5;
		if (i%2 == 0)
		c_color3 = 0.5;
		else
		c_color3 = 0.0;
		oneCatenaryMarker.markers[i].header.frame_id = frame_id;
		oneCatenaryMarker.markers[i].header.stamp = ros::Time::now();
		oneCatenaryMarker.markers[i].ns = "one_catenary";
		oneCatenaryMarker.markers[i].id = 1 + i*10.0;
		oneCatenaryMarker.markers[i].action = visualization_msgs::Marker::ADD;
		oneCatenaryMarker.markers[i].type = visualization_msgs::Marker::SPHERE;
		oneCatenaryMarker.markers[i].lifetime = ros::Duration(180);
		oneCatenaryMarker.markers[i].pose.position.x = points_catenary_[i].x; 
		oneCatenaryMarker.markers[i].pose.position.y = points_catenary_[i].y; 
		oneCatenaryMarker.markers[i].pose.position.z = points_catenary_[i].z; //Move in Z to see the point over the map surface
		oneCatenaryMarker.markers[i].pose.orientation.x = 0.0;
		oneCatenaryMarker.markers[i].pose.orientation.y = 0.0;
		oneCatenaryMarker.markers[i].pose.orientation.z = 0.0;
		oneCatenaryMarker.markers[i].pose.orientation.w = 1.0;
		oneCatenaryMarker.markers[i].scale.x = 0.06;
		oneCatenaryMarker.markers[i].scale.y = 0.06;
		oneCatenaryMarker.markers[i].scale.z = 0.06;
		oneCatenaryMarker.markers[i].color.r = 1.0 - c_color1;
		oneCatenaryMarker.markers[i].color.g = c_color2;
		oneCatenaryMarker.markers[i].color.b = c_color3;
		oneCatenaryMarker.markers[i].color.a = 1.0; 
	}
	one_catenary_marker_pub_.publish(oneCatenaryMarker);
}

void RRTStar::getCatenaryPathMarker(std::list<RRTNode*> ct_)
{
    std::string string_marker;
    std::string ns_marker;
	double c_color1, c_color2, c_color3;
	std::vector<geometry_msgs::Point> points_catenary_;
	geometry_msgs::Point p_reel_;
	
	int count = 0; 
    for (auto nt_:ct_) {
		p_reel_ = getReelNode(*nt_);
		points_catenary_.clear();

		CatenarySolver cS_;
		cS_.setMaxNumIterations(100);
		
		double x_ugv_, y_ugv_, z_ugv_, x_uav_, y_uav_, z_uav_, len_cat_;
		x_ugv_ = p_reel_.x; 
		y_ugv_ = p_reel_.y; 
		z_ugv_ = p_reel_.z; 
		x_uav_ = nt_->point_uav.x*step; 
		y_uav_ = nt_->point_uav.y*step; 
		z_uav_ = nt_->point_uav.z*step; 
		len_cat_ = nt_->length_cat;
		printf("Values to Compute Catenary: ugv[%f %f %f]  uav[%f %f %f] len_cat[%f]\n",x_ugv_, y_ugv_, z_ugv_, x_uav_, y_uav_, z_uav_, len_cat_);
		cS_.solve(x_ugv_, y_ugv_, z_ugv_, x_uav_, y_uav_, z_uav_, len_cat_, points_catenary_);

		int id_ = nt_->id;
		
		if (count%2 == 0)
			c_color3 = 0.5;
		else
			c_color3 = 0.0;
		string_marker = std::to_string(count);
		ns_marker = "catenary_"+ string_marker;
	
		catenaryMarker.markers.clear();
		catenaryMarker.markers.resize(points_catenary_.size());

		for (size_t i = 0 ; i < points_catenary_.size() ; i++ ) {
			c_color1 = ((double)i / (double)points_catenary_.size())*0.5;
			c_color2 = ((double)i / (double)points_catenary_.size())*0.5;

			catenaryMarker.markers[i].header.frame_id = frame_id;
			catenaryMarker.markers[i].header.stamp = ros::Time::now();
			catenaryMarker.markers[i].ns = ns_marker;
			catenaryMarker.markers[i].id = id_ + i*10.0;
			catenaryMarker.markers[i].action = visualization_msgs::Marker::ADD;
			catenaryMarker.markers[i].type = visualization_msgs::Marker::SPHERE;
			catenaryMarker.markers[i].lifetime = ros::Duration(180);
			catenaryMarker.markers[i].pose.position.x = points_catenary_[i].x; 
			catenaryMarker.markers[i].pose.position.y = points_catenary_[i].y; 
			catenaryMarker.markers[i].pose.position.z = points_catenary_[i].z; //Move in Z to see the point over the map surface
			catenaryMarker.markers[i].pose.orientation.x = 0.0;
			catenaryMarker.markers[i].pose.orientation.y = 0.0;
			catenaryMarker.markers[i].pose.orientation.z = 0.0;
			catenaryMarker.markers[i].pose.orientation.w = 1.0;
			catenaryMarker.markers[i].scale.x = 0.06;
			catenaryMarker.markers[i].scale.y = 0.06;
			catenaryMarker.markers[i].scale.z = 0.06;
			catenaryMarker.markers[i].color.r = 1.0 - c_color1;
			catenaryMarker.markers[i].color.g = c_color2;
			catenaryMarker.markers[i].color.b = c_color3;
			catenaryMarker.markers[i].color.a = 1.0; 
		}
	count++;
	catenary_marker_pub_.publish(catenaryMarker);
	}	
}

void RRTStar::getAllCatenaryMarker()
{
	std::string string_marker;
    std::string ns_marker;

	double c_color1, c_color2, c_color3;
	
	int count = 0; 

	for (auto nt_:nodes_tree) {		
		std::vector<geometry_msgs::Point> points_catenary_;
		geometry_msgs::Point p_reel_, p_uav_;

		CatenarySolver cSolver_;
		cSolver_.setMaxNumIterations(100);

		p_reel_ = getReelNode(*nt_);

		p_uav_.x = nt_->point_uav.x*step; 
		p_uav_.y = nt_->point_uav.y*step; 
		p_uav_.z = nt_->point_uav.z*step;
		double l_cat_ = nt_->length_cat;

		cSolver_.solve(p_reel_.x, p_reel_.y, p_reel_.z, p_uav_.x, p_uav_.y, p_uav_.z, l_cat_, points_catenary_);

		int id_ = nt_->id;


		if (count%2 == 0)
			c_color3 = 0.5;
		else
			c_color3 = 0.0;

		string_marker = std::to_string(count);
		ns_marker = "catenary_"+ string_marker;
	
		allCatenaryMarker.markers.clear();
		allCatenaryMarker.markers.resize(points_catenary_.size());

		for (size_t i = 0 ; i < points_catenary_.size() ; i++ ) {
			c_color1 = ((double)i / (double)points_catenary_.size())*0.5;
			c_color2 = ((double)i / (double)points_catenary_.size())*0.5;

			allCatenaryMarker.markers[i].header.frame_id = frame_id;
			allCatenaryMarker.markers[i].header.stamp = ros::Time::now();
			allCatenaryMarker.markers[i].ns = ns_marker;
			allCatenaryMarker.markers[i].id = id_ + i*10.0;
			allCatenaryMarker.markers[i].action = visualization_msgs::Marker::ADD;
			allCatenaryMarker.markers[i].type = visualization_msgs::Marker::SPHERE;
			allCatenaryMarker.markers[i].lifetime = ros::Duration(180);
			allCatenaryMarker.markers[i].pose.position.x = points_catenary_[i].x; 
			allCatenaryMarker.markers[i].pose.position.y = points_catenary_[i].y; 
			allCatenaryMarker.markers[i].pose.position.z = points_catenary_[i].z; //Move in Z to see the point over the map surface
			allCatenaryMarker.markers[i].pose.orientation.x = 0.0;
			allCatenaryMarker.markers[i].pose.orientation.y = 0.0;
			allCatenaryMarker.markers[i].pose.orientation.z = 0.0;
			allCatenaryMarker.markers[i].pose.orientation.w = 1.0;
			allCatenaryMarker.markers[i].scale.x = 0.05;
			allCatenaryMarker.markers[i].scale.y = 0.05;
			allCatenaryMarker.markers[i].scale.z = 0.05;
			allCatenaryMarker.markers[i].color.r = 1.0 - c_color1;
			allCatenaryMarker.markers[i].color.g = c_color2;
			allCatenaryMarker.markers[i].color.b = c_color3;
			allCatenaryMarker.markers[i].color.a = 1.0; 
		}
	count++;
	all_catenary_marker_pub_.publish(allCatenaryMarker);
		
	}
}

void RRTStar::goalPointMarker()
{
	visualization_msgs::Marker marker_;
	marker_.header.frame_id = frame_id;
	marker_.header.stamp = ros::Time();
	marker_.ns = "goal_point";
	marker_.id = 0;
	marker_.type = visualization_msgs::Marker::SPHERE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.lifetime = ros::Duration(0);
	marker_.pose.position.x = final_position.x;
	marker_.pose.position.y = final_position.y;
	marker_.pose.position.z = final_position.z;
	marker_.pose.orientation.x = 0.0;
	marker_.pose.orientation.y = 0.0;
	marker_.pose.orientation.z = 0.0;
	marker_.pose.orientation.w = 1.0;
	marker_.scale.x = 0.4;
	marker_.scale.y = 0.4;
	marker_.scale.z = 0.4;
	marker_.color.r = 1.0;
	marker_.color.g = 0.0;
	marker_.color.b = 0.0;
	marker_.color.a = 1.0; 
	
	goal_point_pub_.publish(marker_);
}

void RRTStar::randPointMarker(RRTNode rn_)
{
	visualization_msgs::MarkerArray marker_rand;
	marker_rand.markers.resize(2);
	marker_rand.markers[0].header.frame_id = frame_id;
	marker_rand.markers[0].header.stamp = ros::Time();
	marker_rand.markers[0].ns = "rand_uav_point";
	marker_rand.markers[0].id = 0;
	marker_rand.markers[0].type = visualization_msgs::Marker::SPHERE;
	marker_rand.markers[0].action = visualization_msgs::Marker::ADD;
	marker_rand.markers[0].lifetime = ros::Duration(20);
	marker_rand.markers[0].pose.position.x = rn_.point.x*step;
	marker_rand.markers[0].pose.position.y = rn_.point.y*step;
	marker_rand.markers[0].pose.position.z = rn_.point.z*step+0.1;
	marker_rand.markers[0].pose.orientation.x = 0.0;
	marker_rand.markers[0].pose.orientation.y = 0.0;
	marker_rand.markers[0].pose.orientation.z = 0.0;
	marker_rand.markers[0].pose.orientation.w = 1.0;
	marker_rand.markers[0].scale.x = 0.4;
	marker_rand.markers[0].scale.y = 0.4;
	marker_rand.markers[0].scale.z = 0.4;
	marker_rand.markers[0].color.r = 0.0;
	marker_rand.markers[0].color.g = 1.0;
	marker_rand.markers[0].color.b = 0.0;
	marker_rand.markers[0].color.a = 1.0; 

	visualization_msgs::Marker marker_uav;
	marker_rand.markers[1].header.frame_id = frame_id;
	marker_rand.markers[1].header.stamp = ros::Time();
	marker_rand.markers[1].ns = "rand_uav_point";
	marker_rand.markers[1].id = 1;
	marker_rand.markers[1].type = visualization_msgs::Marker::SPHERE;
	marker_rand.markers[1].action = visualization_msgs::Marker::ADD;
	marker_rand.markers[1].lifetime = ros::Duration(20);
	marker_rand.markers[1].pose.position.x = rn_.point_uav.x*step;
	marker_rand.markers[1].pose.position.y = rn_.point_uav.y*step;
	marker_rand.markers[1].pose.position.z = rn_.point_uav.z*step;
	marker_rand.markers[1].pose.orientation.x = 0.0;
	marker_rand.markers[1].pose.orientation.y = 0.0;
	marker_rand.markers[1].pose.orientation.z = 0.0;
	marker_rand.markers[1].pose.orientation.w = 1.0;
	marker_rand.markers[1].scale.x = 0.4;
	marker_rand.markers[1].scale.y = 0.4;
	marker_rand.markers[1].scale.z = 0.4;
	marker_rand.markers[1].color.r = 0.4;
	marker_rand.markers[1].color.g = 1.0;
	marker_rand.markers[1].color.b = 0.4;
	marker_rand.markers[1].color.a = 1.0; 
	
	rand_point_pub_.publish(marker_rand);
}

void RRTStar::getPointsObsMarker(std::vector<geometry_msgs::Point> points_catenary_){
	std::string string_marker;
    std::string ns_marker;

	double c_color1, c_color2, c_color3;
	visualization_msgs::MarkerArray pointsMarker;
	
	pointsMarker.markers.clear();
	pointsMarker.markers.resize(points_catenary_.size());

	for (size_t i = 0 ; i < points_catenary_.size() ; i++ ) {
		c_color1 = ((double)i / (double)points_catenary_.size())*0.5;
		c_color2 = ((double)i / (double)points_catenary_.size())*0.5;
		if (i%2 == 0)
		c_color3 = 0.5;
		else
		c_color3 = 0.0;
		pointsMarker.markers[i].header.frame_id = frame_id;
		pointsMarker.markers[i].header.stamp = ros::Time::now();
		pointsMarker.markers[i].ns = "points_marker";
		pointsMarker.markers[i].id = 1 + i*10.0;
		pointsMarker.markers[i].action = visualization_msgs::Marker::ADD;
		pointsMarker.markers[i].type = visualization_msgs::Marker::SPHERE;
		pointsMarker.markers[i].lifetime = ros::Duration(180);
		pointsMarker.markers[i].pose.position.x = points_catenary_[i].x; 
		pointsMarker.markers[i].pose.position.y = points_catenary_[i].y; 
		pointsMarker.markers[i].pose.position.z = points_catenary_[i].z; //Move in Z to see the point over the map surface
		pointsMarker.markers[i].pose.orientation.x = 0.0;
		pointsMarker.markers[i].pose.orientation.y = 0.0;
		pointsMarker.markers[i].pose.orientation.z = 0.0;
		pointsMarker.markers[i].pose.orientation.w = 1.0;
		pointsMarker.markers[i].scale.x = 0.06;
		pointsMarker.markers[i].scale.y = 0.06;
		pointsMarker.markers[i].scale.z = 0.06;
		pointsMarker.markers[i].color.r = 1.0 - c_color1;
		pointsMarker.markers[i].color.g = c_color2;
		pointsMarker.markers[i].color.b = c_color3;
		pointsMarker.markers[i].color.a = 1.0; 
	}
	points_marker_pub_.publish(pointsMarker);
}

void RRTStar::clearMarkers()
{
	auto size_ = pointTreeMarkerUGV.markers.size();
	pointTreeMarkerUGV.markers.clear();
	pointTreeMarkerUGV.markers.resize(size_);
    for (auto i = 0 ; i < size_; i++){
        pointTreeMarkerUGV.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    tree_rrt_star_ugv_pub_.publish(pointTreeMarkerUGV);

	pointTreeMarkerUAV.markers.clear();
	pointTreeMarkerUAV.markers.resize(size_);
    for (auto i = 0 ; i < size_; i++){
        pointTreeMarkerUAV.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    tree_rrt_star_uav_pub_.publish(pointTreeMarkerUAV);

	size_ = pointTakeOffMarker.markers.size();
	pointTakeOffMarker.markers.clear();
	pointTakeOffMarker.markers.resize(size_);
    for (auto i = 0 ; i < size_; i++){
        pointTakeOffMarker.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    take_off_nodes_pub_.publish(pointTakeOffMarker);


	size_ = n_iter;
    lines_ugv_marker_.markers.clear();
	lines_ugv_marker_.markers.resize(size_);
	for (auto i = 0 ; i < size_; i++){
        lines_ugv_marker_.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    lines_ugv_marker_pub_.publish(lines_ugv_marker_);

	size_ = n_iter;
    lines_uav_marker_.markers.clear();
	lines_uav_marker_.markers.resize(size_);
	for (auto i = 0 ; i < size_; i++){
        lines_uav_marker_.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    lines_uav_marker_pub_.publish(lines_uav_marker_);
}

std::list<RRTNode*> RRTStar::getPath()
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
		while (current_node->parentNode != NULL){ 
			current_node = current_node->parentNode;
			path_.push_front(current_node);
			std::cout << "current_node :  " << current_node << " , parent= "<< current_node->parentNode << "  ,  points : [" 
			<< current_node->point.x*step << "," << current_node->point.y*step << "," << current_node->point.z*step << "],["
			<< current_node->point_uav.x*step << "," << current_node->point_uav.y*step << "," << current_node->point_uav.z*step << "]"<< std::endl;
			// std::string y_ ;
			// std::cout << "Press key to continue: " << y_ << std::endl;	
			// std::cin >> y_ ;
			// std::cout << "Pressed key : " << y_ << std::endl;	
		}
	}
	
	return path_;
}

void RRTStar::isGoal(const RRTNode st_) 
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
			got_to_goal = true;
		else
			got_to_goal = false;	
	}
	else{
		point_.x = st_.point_uav.x;
		point_.y = st_.point_uav.y;
		point_.z = st_.point_uav.z;

		double dist_goal_ = sqrt(pow(point_.x*step - final_position.x,2) + 
	  							 pow(point_.y*step - final_position.y,2) +
					 			 pow(point_.z*step - final_position.z,2) );
	
		printf("dist_goal = %f/%f \n",dist_goal_, goal_gap_m);

		if (dist_goal_ < goal_gap_m){
			got_to_goal = true;
		}
		else
			got_to_goal = false;	
	}
}

void RRTStar::configCatenaryCompute	(bool _u_c, bool _u_s_p, double _mf, double _l_m, 
									geometry_msgs::Vector3 _p_reel , geometry_msgs::Vector3 _p_ugv, geometry_msgs::Quaternion _r_ugv, 
									bool coupled_, int n_iter_ , double r_nn_, double s_s_, int s_g_r_)
{
	use_catenary = _u_c;
	use_search_pyramid =  _u_s_p;
	multiplicative_factor = _mf;
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
	radius_near_nodes = r_nn_;
	step_steer = s_s_;
	samp_goal_rate = s_g_r_;
}


bool RRTStar::setInitialPositionCoupled(DiscretePosition p_)
{
	if (isUGVInside(p_.x, p_.y, p_.z))
	{
		RRTStarNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(p_.x, p_.y, p_.z)];

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

bool RRTStar::setInitialPositionIndependent(DiscretePosition p1_, DiscretePosition p2_)
{
	if (isUGVInside(p1_.x, p1_.y, p1_.z) && isInside(p2_.x, p2_.y, p2_.z))
	{
		RRTStarNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(p1_.x, p1_.y, p1_.z)];

		if (initialNodeInWorld->node == NULL)
		{
			initialNodeInWorld->node = new RRTNode();
			initialNodeInWorld->node->point.x = p1_.x;
			initialNodeInWorld->node->point.y = p1_.y;
			initialNodeInWorld->node->point.z = p1_.z;
			initialNodeInWorld->node->point_uav.x = p2_.x;
			initialNodeInWorld->node->point_uav.y = p2_.y;
			initialNodeInWorld->node->point_uav.z = p2_.z;

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
		printf("disc_initial [%f %f %f /%f %f %f]\n",disc_initial->point.x*step,disc_initial->point.y*step,disc_initial->point.z*step,
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

bool RRTStar::setFinalPosition(DiscretePosition p_)
{
	if (isInside(p_.x, p_.y, p_.z))
	{
		RRTStarNodeLink3D *finalNodeInWorld = &discrete_world[getWorldIndex(p_.x, p_.y, p_.z)];

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

		printf("disc_final [%f %f %f /%f %f %f]\n",disc_final->point.x*step,disc_final->point.y*step,disc_final->point.z*step,
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
bool RRTStar::isInitialPositionUGVOccupied()
{

	if (isUGVOccupied(*disc_initial))
		return true;
	else
		return false;
}

bool RRTStar::isInitialPositionUAVOccupied()
{
	if (isOccupied(*disc_initial, true))
		return true;
	else
		return false;
}

bool RRTStar::isFinalPositionOccupied()
{
	if (isOccupied(*disc_final))
		return true;
	else
		return false;
}

bool RRTStar::isOccupied(RRTNode n_, bool check_uav_)
{
	if(check_uav_==false){
		return !discrete_world[getWorldIndex(n_.point.x, n_.point.y, n_.point.z)].notOccupied;
	}
	else
		return !discrete_world[getWorldIndex(n_.point_uav.x, n_.point_uav.y, n_.point_uav.z)].notOccupied;
	
}

bool RRTStar::isUGVOccupied(RRTNode n_)
{
	RRTNode n_z_displace_;
	n_z_displace_.point.x = n_.point.x;
	n_z_displace_.point.y = n_.point.y;
	n_z_displace_.point.z = n_.point.z + (v_inflation + step_inv);

	return !discrete_world[getWorldIndex(n_z_displace_.point.x, n_z_displace_.point.y, n_z_displace_.point.z)].notOccupied;
}

void RRTStar::publishOccupationMarkersMap()
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

float RRTStar::getYawFromQuat(Quaternion quat)
{
	double r, p, y;
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 M(q);
	M.getRPY(r, p, y);

	return y;
}

bool RRTStar::getTrajectory(Trajectory &trajectory)
{
	trajectory_msgs::MultiDOFJointTrajectoryPoint traj_marsupial_;

	traj_marsupial_.transforms.resize(2);
	traj_marsupial_.velocities.resize(2);
	traj_marsupial_.accelerations.resize(2);

	for(auto nt_ : rrt_path){
		traj_marsupial_.transforms[0].translation.x = nt_->point.x*step;
		traj_marsupial_.transforms[0].translation.y = nt_->point.y*step;
		traj_marsupial_.transforms[0].translation.z = nt_->point.z*step;
		traj_marsupial_.transforms[0].rotation.x = 0.0;
		traj_marsupial_.transforms[0].rotation.y = 0.0;
		traj_marsupial_.transforms[0].rotation.z = 0.0;
		traj_marsupial_.transforms[0].rotation.w = 1.0;
		traj_marsupial_.velocities[0].linear.x = 0.0;
		traj_marsupial_.velocities[0].linear.y = 0.0;
		traj_marsupial_.velocities[0].linear.z = 0.0;
		traj_marsupial_.accelerations[0].linear.x = 0.0;
		traj_marsupial_.accelerations[0].linear.y = 0.0;
		traj_marsupial_.accelerations[0].linear.z = 0.0;
		traj_marsupial_.transforms[1].translation.x = nt_->point_uav.x*step;
		traj_marsupial_.transforms[1].translation.y = nt_->point_uav.y*step;
		traj_marsupial_.transforms[1].translation.z = nt_->point_uav.z*step;
		traj_marsupial_.transforms[1].rotation.x = 0.0;
		traj_marsupial_.transforms[1].rotation.y = 0.0;
		traj_marsupial_.transforms[1].rotation.z = 0.0;
		traj_marsupial_.transforms[1].rotation.w = 1.0;
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



// double RRTStar::retCostPath(const std::list< RRTNode >& path)
// {
// 	double ret = 0;
// 	int count = 0;
// 	for (auto it = path.begin(); it != path.end(); it++, count++) {
// 	  if (count > 0) {
// 	    double cost_node = it->cost;
// 	    if (cost_node < 0.0){
// 	      ROS_INFO("The value of a node is negative");
// 	    }
// 	    ret += cost_node;
// 	  }
// 	}
// 	return ret;
// }

void RRTStar::updateMap(octomap_msgs::OctomapConstPtr message)
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

void RRTStar::updateMap(PointCloud cloud)
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

void RRTStar::updateMap(const PointCloud::ConstPtr &map)
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

octomap::OcTree RRTStar::updateMapReduced(octomap_msgs::OctomapConstPtr msg, 
									geometry_msgs::Vector3Stamped goal_, 
									geometry_msgs::Vector3Stamped start_, 
									std::vector<octomap::point3d> &full_ray_cast, 
									std::vector<octomap::point3d> &ray_cast_free,
									std::vector<octomap::point3d> &ray_cast_free_reduce,  
									std::vector<octomap::point3d> &ray_cast_coll, 
									std::vector<octomap::point3d> &no_ray_cast_free)
{
	full_ray_cast.clear(); ray_cast_free.clear(); ray_cast_free_reduce.clear(); ray_cast_coll.clear(); no_ray_cast_free.clear();

	double _resolution = 0.2;
	double R_, theta_, phi_;

	new_start.x = start_.vector.x + ((start_.vector.x - goal_.vector.x) / sqrt(pow(start_.vector.x - goal_.vector.x,2)+pow(start_.vector.y - goal_.vector.y,2)+pow(start_.vector.z - goal_.vector.z,2)));
	new_start.y = start_.vector.y + ((start_.vector.y - goal_.vector.y) / sqrt(pow(start_.vector.x - goal_.vector.x,2)+pow(start_.vector.y - goal_.vector.y,2)+pow(start_.vector.z - goal_.vector.z,2)));
	new_start.z = start_.vector.z + ((start_.vector.z - goal_.vector.z) / sqrt(pow(start_.vector.x - goal_.vector.x,2)+pow(start_.vector.y - goal_.vector.y,2)+pow(start_.vector.z - goal_.vector.z,2)));
	new_goal.x = goal_.vector.x + 0.5*((goal_.vector.x - start_.vector.x) / sqrt(pow(goal_.vector.x - start_.vector.x,2)+pow(goal_.vector.y - start_.vector.y,2)+pow(goal_.vector.z - start_.vector.z,2)));
	new_goal.y = goal_.vector.y + 0.5*((goal_.vector.y - start_.vector.y) / sqrt(pow(goal_.vector.x - start_.vector.x,2)+pow(goal_.vector.y - start_.vector.y,2)+pow(goal_.vector.z - start_.vector.z,2)));
	new_goal.z = goal_.vector.z + 0.5*((goal_.vector.z - start_.vector.z) / sqrt(pow(goal_.vector.x - start_.vector.x,2)+pow(goal_.vector.y - start_.vector.y,2)+pow(goal_.vector.z - start_.vector.z,2)));

	std::vector<octomap::point3d> vrc_, points_rcnf_, points_rcf_ , points_rcc_;
	std::vector<octomap::point3d> map_simplify_free, map_simplify_occupied;
	vrc_.clear(); points_rcnf_.clear(); points_rcc_.clear(); points_rcf_.clear(); map_simplify_free.clear(); map_simplify_occupied.clear();
	octomap::point3d s_(new_start.x, new_start.y, new_start.z);
	octomap::point3d g_(new_goal.x, new_goal.y, new_goal.z);
	octomap::point3d r_, e_, c_;
	octomap::OcTree *map_msg;
	octomap::OcTree map_simplify (0.05f);
	octomap::OcTree map_simplify_reduced (0.05f);
	bool r_cast_coll;

	map_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);

	angle_square_pyramid = M_PI/8;
	double hip_ = sqrt(pow(new_goal.x-new_start.x,2)+ pow(new_goal.y-new_start.y,2)+ pow(new_goal.z-new_start.z,2));
	sweep_range = hip_ * sin(angle_square_pyramid); 
	phi_min = -1.0*sweep_range ;	
	phi_max = sweep_range ;
	theta_min = -1.0* sweep_range;
	theta_max = sweep_range ;
	double z_max_obs = 0.0; 
	double max_z_to_explore = -100.0;

	bool obs_pyramd_floor = false;

	octomap::point3d Vgs(new_goal.x-new_start.x, new_goal.y-new_start.y, new_goal.z-new_start.z);

	//Get spherical parameter
	R_= sqrt(Vgs.x()*Vgs.x() + Vgs.y()*Vgs.y() + Vgs.z()*Vgs.z());

	if(Vgs.z() > 0.0)
		theta_ = atan(sqrt(Vgs.x()*Vgs.x()+Vgs.y()*Vgs.y())/(Vgs.z()));
	else if (Vgs.z() < 0.0)
		theta_ = M_PI - atan(sqrt(Vgs.x()*Vgs.x()+Vgs.y()*Vgs.y())/(Vgs.z()));
	else
		theta_ = 0.0;
	
	if (Vgs.y() >= 0.0 && Vgs.x()>= 0.0)
		phi_ = atan(fabs(Vgs.y()/Vgs.x()));
	else if (Vgs.y() >= 0.0 && Vgs.x() < 0.0)
		phi_ = M_PI - atan(fabs(Vgs.y()/Vgs.x()));
	else if (Vgs.y() < 0.0 && Vgs.x() < 0.0)
		phi_ = M_PI + atan(fabs(Vgs.y()/Vgs.x()));
	else
		phi_ = 2.0*M_PI - atan(fabs(Vgs.y()/Vgs.x()));

	//Transform cartesian coordinates to spherical vectorial base 
	float ur_x,ur_y,ur_z,ut_x,ut_y,ut_z,up_x,up_y,up_z;
	ur_x = sin(theta_) * cos(phi_);
	ur_y = sin(theta_) * sin(phi_);
	ur_z = cos(theta_);
	ut_x = cos(theta_) * cos(phi_);
	ut_y = cos(theta_) * sin(phi_);
	ut_z = -sin(theta_);
	up_x= -sin(phi_);
	up_y = cos(phi_);
	up_z = 0;
	base_sp << ur_x, ur_y, ur_z, 
			   ut_x, ut_y, ut_z, 
			   up_x, up_y, up_z;
    Eigen::Vector3f plane_sp;
	
	//1. Applying Ray Cast to detect 3D free collision space and with collision. The space where Ray cast has collision is used to set highest obstacle and reduce 3D space to set with full Ray 
	int counting3 = 0;
	for (double delta_theta = theta_min ; delta_theta <= theta_max; delta_theta = delta_theta + _resolution){
		for (double delta_phi = phi_min ; delta_phi <= phi_max; delta_phi = delta_phi + _resolution){	
			plane_sp << 0.0, 
						delta_theta, 
						delta_phi;
			Eigen::Matrix3f base_sp_inv =  base_sp.inverse();
			Eigen::Vector3f spheric_ = base_sp_inv * plane_sp;
			Eigen::Vector3f cartesian_;
			cartesian_ << spheric_(0,0)+new_goal.x,
						  spheric_(1,0)+new_goal.y, 
						  spheric_(2,0)+new_goal.z;
			octomap::point3d e_(cartesian_(0,0), cartesian_(1,0), cartesian_(2,0)); //current end(goal) point
			octomap::point3d d_(e_.x() - new_start.x , e_.y() - new_start.y , e_.z() - new_start.z ); //direction for rayCast

			r_cast_coll = map_msg->castRay(s_, d_, r_);

			// These four lines can be commented to improve compute time performance, are uncommented just to graph Ray-Casting complete
			map_msg->computeRay(s_, e_, vrc_);
			for (size_t i = 0; i < vrc_.size(); i ++){
				full_ray_cast.push_back(vrc_[i]);
			}
			
			if(!r_cast_coll){
				map_msg->computeRay(s_, e_, vrc_);
				for (size_t i = 0; i < vrc_.size(); i ++){
					ray_cast_free.push_back(vrc_[i]);
					map_simplify_free.push_back(vrc_[i]);
				}
				points_rcf_.push_back(e_);
			}
			else{
				double d1_ = pow(new_start.x - r_.x(),2) + pow(new_start.y - r_.y(),2) + pow(new_start.z - r_.z(),2);
				double d2_ = pow(new_start.x - e_.x(),2) + pow(new_start.y - e_.y(),2) + pow(new_start.z - e_.z(),2);
				if (d1_ >= d2_){
					map_msg->computeRay(s_, e_, vrc_);
					points_rcf_.push_back(e_);
					for (size_t i = 0; i < vrc_.size(); i ++){
						ray_cast_free.push_back(vrc_[i]);
						map_simplify_free.push_back(vrc_[i]);
					}
				}
				else{
					map_msg->computeRay(s_, r_, vrc_);
					points_rcnf_.push_back(e_);
					points_rcc_.push_back(r_);
					if ( (z_max_obs < r_.z()) && (d1_ < d2_*2.0/3.0)){
						z_max_obs = r_.z();
						max_z_to_explore = e_.z();
						max_theta_axe_reduced = spheric_(2,0);
					}
				}
			}
		}
	}

	if ( max_z_to_explore < new_goal.z){
		obs_pyramd_floor = true;
		max_theta_axe_reduced = sweep_range;
	}
	else{
		obs_pyramd_floor = false;
	}
	//2. Apply Ray to set and reduced free space between start and obstacle 
	for (size_t i = 0; i < points_rcc_.size(); i ++){
		map_msg->computeRay(s_, points_rcc_[i], vrc_);
		if ((points_rcc_[i].z() < z_max_obs || obs_pyramd_floor)){	
			for (size_t j = 0; j < vrc_.size(); j ++){
				ray_cast_coll.push_back(vrc_[j]);
				if (j < vrc_.size()-1)
					map_simplify_free.push_back(vrc_[j]);
				else
					map_simplify_occupied.push_back(vrc_[j]);
			}
		}
	}

	//3. Apply Ray to set reduced free space above highest collision 
	for (size_t i= 0 ; i < points_rcf_.size(); i++){
		octomap::point3d e_(points_rcf_[i].x(), points_rcf_[i].y(), points_rcf_[i].z()); //save the current end(goal) point
		map_msg->computeRay(s_, e_, vrc_);
		if ((points_rcf_[i].z() < max_z_to_explore || obs_pyramd_floor)){	
			counting3++;
			for (size_t i = 0; i < vrc_.size(); i ++){
				ray_cast_free_reduce.push_back(vrc_[i]);
				map_simplify_free.push_back(vrc_[i]);
			}
		}
	}

	//4. Process to get rayCast in opposite direction, from goal zone to start zone to fill the collision zone
	for (size_t i = 0; i < points_rcnf_.size(); i ++){
		if (points_rcnf_[i].z() < max_z_to_explore + _resolution/2.0 || obs_pyramd_floor){
			octomap::point3d d_(new_start.x - points_rcnf_[i].x(), new_start.y - points_rcnf_[i].y(), new_start.z - points_rcnf_[i].z()); //save the direction for rayCast
			map_msg->castRay(points_rcnf_[i], d_, c_,true);
			double d1_ = pow(new_start.x - points_rcnf_[i].x(),2) + pow(new_start.y - points_rcnf_[i].y(),2) + pow(new_start.z - points_rcnf_[i].z(),2);
			double d2_ = pow(c_.x() - points_rcnf_[i].x(),2) + pow(c_.y() - points_rcnf_[i].y(),2) + pow(c_.z() - points_rcnf_[i].z(),2);
			octomap::point3d new_s_(new_start.x,new_start.y,new_start.z);
			if (d2_ >= d1_ ){
				map_msg->computeRay(points_rcnf_[i], new_s_, vrc_);
			}
			else{
				map_msg->computeRay(points_rcnf_[i], c_, vrc_);
			}
			for (size_t j = 0; j < vrc_.size(); j++){
				no_ray_cast_free.push_back(vrc_[j]);
				if (j < vrc_.size()-1){
					map_simplify_free.push_back(vrc_[j]);
				}
				else{
					map_simplify_occupied.push_back(vrc_[j]);
				}
			}
		}	
	}

	//4. Create new octomap with desired space to navigate
	// insert some measurements of occupied cells
	for (size_t i=0 ; i < map_simplify_occupied.size() ; i++){
		map_simplify.updateNode(map_simplify_occupied[i], true); // integrate 'occupied' measurement
	}
  	// insert some measurements of free cells
	for (size_t i=0 ; i < map_simplify_free.size() ; i++){
		map_simplify.updateNode(map_simplify_free[i], false);  // integrate 'free' measurement
	}
	
	u_int64_t occupied_leafs = 0, free_leafs = 0;
	int nit = 0;

	std::vector<int> v_disc_x_, v_disc_y_, v_disc_z_;
	std::vector<int> v_disc_free_x_, v_disc_free_y_, v_disc_free_z_;
	v_disc_x_.clear(); v_disc_y_.clear(); v_disc_z_.clear();
	v_disc_free_x_.clear(), v_disc_free_y_.clear(), v_disc_free_z_.clear();
	bool match_;
	int x_, y_ , z_;
	float x_w, y_w, z_w;

	return map_simplify_reduced;
}

void RRTStar::clearMap()
{
	for (int i = 0; i < matrix_size; i++)
	{
		discrete_world[i].notOccupied = true;
	}
}

// bool RRTStar::setInitialPosition(DiscretePosition p_)
// {
// 	if (isUGVInside(p_.x, p_.y, p_.z))
// 	{
// 		RRTStarNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(p_.x, p_.y, p_.z)];
// 		if (initialNodeInWorld->node == NULL)
// 		{
// 			initialNodeInWorld->node = new RRTNode();
// 			initialNodeInWorld->node->point.x = p_.x;
// 			initialNodeInWorld->node->point.x = p_.y;
// 			initialNodeInWorld->node->point.x = p_.z;
// 			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
// 		}
// 		disc_initial = initialNodeInWorld->node;
// 		initial_position.x = p_.x * step;
// 		initial_position.y = p_.y * step;
// 		initial_position.z = p_.z * step;
// 		disc_initial->point = p_;
// 		disc_initial->parentNode = NULL;
// 		disc_initial->cost = 0.0;
// 		disc_initial->length_cat = -1.0;
// 		disc_initial->min_dist_obs_cat = -1.0;
// 		disc_initial->min_dist_obs_ugv = -1.0;
// 		return true;
// 	}
// 	else
// 	{
// 		//~ std::cerr << "ThetaStar: Initial point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
// 		disc_initial = NULL;
// 		return false;
// 	}
// }

// bool RRTStar::setInitialPosition(Vector3 p)
// {
	// 	initial_position_ugv = p;
	// 	DiscretePosition p_ = discretizePosition(p);
	// 	if (isUGVInside(p_.x, p_.y, p_.z))
	// 	{
	// 		RRTStarNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(
	// 			p_.x,
	// 			p_.y,
	// 			p_.z)];
	// 		if (initialNodeInWorld->node == NULL)
	// 		{
	// 			initialNodeInWorld->node = new ThetaStarNode3D();
	// 			initialNodeInWorld->node->point.x = p_.x;
	// 			initialNodeInWorld->node->point.y = p_.y;
	// 			initialNodeInWorld->node->point.z = p_.z;
	// 			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
	// 		}
	// 		disc_initial = initialNodeInWorld->node;
	// 		disc_initial->point = p_;
	// 		disc_initial->parentNode = disc_initial;
	// 		return true;
	// 	}
	// 	else
	// 	{
	// 		//ROS_ERROR("RRTStar: Initial point [%f, %f, %f] not valid.", p.x, p.y, p.z);
	// 		disc_initial = NULL;
	// 		return false;
	// 	}
// }

// bool RRTStar::setFinalPosition(Vector3 p)
// {
	// 	DiscretePosition p_ = discretizePosition(p);
	// 	if (isInside(p_.x, p_.y, p_.z))
	// 	{
	// 		RRTStarNodeLink3D *finalNodeInWorld = &discrete_world[getWorldIndex(
	// 			p_.x,
	// 			p_.y,
	// 			p_.z)];
	// 		if (finalNodeInWorld->node == NULL)
	// 		{
	// 			finalNodeInWorld->node = new ThetaStarNode3D();
	// 			finalNodeInWorld->node->point.x = p_.x;
	// 			finalNodeInWorld->node->point.y = p_.y;
	// 			finalNodeInWorld->node->point.z = p_.z;
	// 			finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
	// 		}
	// 		disc_final = finalNodeInWorld->node;
	// 		final_position = p;
	// 		disc_final->point = p_;
	// 		return true;
	// 	}
	// 	else
	// 	{
	// 		//~ std::cerr << "ThetaStar: Final point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
	// 		disc_final = NULL;
	// 		return false;
	// 	}
// }

Vector3 RRTStar::getInitialPosition()
{
	return initial_position_ugv;
}

Vector3 RRTStar::getFinalPosition()
{
	return final_position;
}

DiscretePosition RRTStar::discretizePosition(Vector3 p)
{
	DiscretePosition res;

	res.x = p.x * step_inv;
	res.y = p.y * step_inv;
	res.z = p.z * step_inv;

	return res;
}

// bool RRTStar::searchInitialPosition3d(float maxDistance)
// {
// 	DiscretePosition init_ = discretizePosition(initial_position_ugv); // Start coordinates
// 	int maxDistance_ = maxDistance / step; // celdas mas lejanas a 1 metro
// 	// Check init point, first if is inside the workspace and second if is occupied
// 	if (setValidInitialPosition(init_))
// 		return true;
// 	// Check from z=0 to d from the near xy ring to the far xy ring
// 	// for (int d = 1; d <= maxDistance_; d++)
// 	// {
// 	// 	for (int z = init_.z; z <= init_.z + d; z++)
// 	// 	{
// 	// 		if (searchInitialPositionInXyRing(init_.x, init_.y, z, d - (z - init_.z)))
// 	// 			return true;
// 	// 	}
// 	// }
// 	return false;
// }

bool RRTStar::searchFinalPosition3d(float maxDistance)
{
	DiscretePosition final_ = discretizePosition(final_position); // Start coordinates

	int maxDistance_ = maxDistance / step; // celdas mas lejanas a compromabar a maxDistance meters

	// Check final point, first if is inside the workspace and second if is occupied
	if (setValidFinalPosition(final_))
		return true;

	// Check from z=0 to d from the near xy ring to the far xy ring
	// for (int d = 1; d <= maxDistance_; d++)
	// {
	// 	for (int z = final_.z; z <= final_.z + d; z++)
	// 	{
	// 		if (searchFinalPositionInXyRing(final_.x, final_.y, z, d - (z - final_.z)))
	// 			return true;
	// 	}
	// }

	return false;
}

void RRTStar::setTimeOut(int sec)
{
	timeout = sec;
}

} //namespace



