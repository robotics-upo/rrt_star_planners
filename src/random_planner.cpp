#include "rrt_planners/random_planner.hpp"

namespace PathPlanners
{
//*****************************************************************
// 		 Random Algorithm Class Definitions (RRT, RRT*, biRRT)
//*****************************************************************

// Default constructor
RandomPlanner::RandomPlanner()
{
	// std::string node_name_ = "grid3D_node";
	// grid_3D = new Grid3d(node_name_);
	// ccm = new CatenaryCheckerManager("random_planner");
	ccm = NULL;
	grid_3D = NULL;
}

RandomPlanner::~RandomPlanner()
{
	delete grid_3D;

	v_points_ws_ugv.clear();
	discrete_world.clear();
	nodes_tree.clear();
  	take_off_nodes.clear();
	rrt_path.clear();
	length_catenary.clear();
	v_nodes_kdtree.clear();
	v_ugv_nodes_kdtree.clear();
	v_uav_nodes_kdtree.clear();
}

// Initialization: creates the occupancy matrix (discrete nodes) from the bounding box sizes, resolution, inflation and optimization arguments
void RandomPlanner::init(std::string plannerType, std::string frame_id_, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_,
				   float step_, float h_inflation_, float v_inflation_, ros::NodeHandlePtr nh_, 
				   double goal_gap_m_, bool debug_rrt_, double distance_obstacle_ugv_, double distance_obstacle_uav_, double distance_catenary_obstacle_, Grid3d *grid3D_,
				   bool nodes_marker_debug_, bool use_distance_function_, std::string map_file_, std::string path_, bool get_catenary_data_, std::string catenary_file_, bool use_parabola_ ,
				   CatenaryCheckerManager *ccm_)
{
	// Pointer to the nodeHandler
	nh = nh_;
	grid_3D = grid3D_;
	ccm = ccm_;

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
	goal_gap_m = goal_gap_m_;
	debug_rrt = debug_rrt_;
	use_distance_function = use_distance_function_;
	printf("Global Planner %s Node : Using distance function: %s \n", plannerType.c_str(), use_distance_function?"true":"false");

	markers_debug = false;
	nodes_marker_debug = nodes_marker_debug_;

	path = path_;
	map_file = map_file_;

	distance_obstacle_ugv = distance_obstacle_ugv_;
	distance_obstacle_uav = distance_obstacle_uav_;
	distance_catenary_obstacle = distance_catenary_obstacle_;

	get_catenary_data = get_catenary_data_;
	catenary_file = catenary_file_;
	use_parabola = use_parabola_;

	lines_ugv_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("path_ugv_rrt_star", 2, true);
	lines_uav_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("path_uav_rrt_star", 2, true);
	// occupancy_marker_pub_ = nh->advertise<PointCloud>("vis_marker_occupancy", 1, true);
	goal_point_pub_ = nh->advertise<visualization_msgs::Marker>("goal_point", 1, true);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marsupial", 1000, true);
	rand_point_pub_ = nh->advertise<visualization_msgs::MarkerArray>("rand_point", 2, true);
	new_point_pub_ = nh->advertise<visualization_msgs::MarkerArray>("new_point", 2, true);
	nearest_point_pub_ = nh->advertise<visualization_msgs::MarkerArray>("nearest_point", 2, true);
	points_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("points_marker", 10, true);
	nearest_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("new_catenaty", 1000, true);
	new_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("nearest_catenaty", 1000, true);
	one_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("one_catenary", 1000, true);
	
	if (planner_type == "birrt")
    	reducedMapPublisher = nh->advertise<octomap_msgs::Octomap>("octomap_feasible_goal_pos", 10);

	if (nodes_marker_debug){
		tree_rrt_star_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("tree_rrt_star_ugv", 2, true);
		tree_rrt_star_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("tree_rrt_star_uav", 2, true);
		take_off_nodes_pub_ = nh->advertise<visualization_msgs::MarkerArray>("take_off_nodes_rrt_star", 2, true);
	}
	if (markers_debug){
		reel1_point_pub_ = nh->advertise<visualization_msgs::Marker>("reel1_point", 1, true);
		reel2_point_pub_ = nh->advertise<visualization_msgs::Marker>("reel2_point", 1, true);
		all_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("all_catenaries_rrt", 10000, true);
	}
}

int RandomPlanner::computeTreeCoupled()
{
    std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;
	printf("RandomPlanner::computeTreeCoupled -->  STARTING --> star_point_ugv[%.2f %.2f %.2f]  goal_point=[%.2f %.2f %.2f] \n\n",
	initial_position_ugv.x, initial_position_ugv.y, initial_position_ugv.z, final_position.x, final_position.y, final_position.z);    

	v_nodes_kdtree.clear();
	v_ugv_nodes_kdtree.clear();
	v_uav_nodes_kdtree.clear();
	points_catenary_new_node.clear();

	saveNode(disc_initial,true);

	count_graph = 0;
	if(nodes_marker_debug)
		rrtgm.getGraphMarker(disc_initial, count_graph, tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_);			// Incremental algorithm --> the graph is generated in each calculation
	if (markers_debug)
		rrtgm.getTakeOffNodesMarker(take_off_nodes, take_off_nodes_pub_);

	updateKdtreeNode(*disc_initial);

 	double ret_val = -1.0; 
    count_loop = 0; 
	
    while (count_loop < n_iter)  // n_iter Max. number of nodes to expand for each round
    {
	  	count_loop++;
		// printf("__________________________________  RandomPlanner::computeTreeCoupled: STARTING WHILE LOOP[%i]  _________________________________\n",count_loop);
		RRTNode q_rand = getRandomNode();	// get a vector with one element in case of coupled configuration
		
		if (debug_rrt)
			printf("q_rand = [%f %f %f / %f %f %f]\n",q_rand.point.x*step,q_rand.point.y*step,q_rand.point.z*step,q_rand.point_uav.x*step,q_rand.point_uav.y*step,q_rand.point_uav.z*step);

		extendGraph(q_rand);
		if ((take_off_nodes.size() > 0) && num_goal_finded>0){
			printf("RandomPlanner::computeTreeCoupled -->  breaking while for in iteration=%i",count_loop);    
			break;
		}
	}

	if (take_off_nodes.size() > 0){
		rrt_path = getPath(); 
		printf("RandomPlanner::computeTreeCoupled -->  finded path for Coupled Marsupial Configuration-->  path size: %lu , number iteration: %i , take off nodes: %lu \n\n",
		rrt_path.size(), count_loop + 1, take_off_nodes.size()); 
		int i_=0;   
		for (auto pt_: rrt_path){
			printf("point_path[%i/%lu] : ugv=[%f %f %f]  uav=[%f %f %f]  cost=[%f]\n", i_, rrt_path.size()
			,pt_->point.x*step, pt_->point.y*step, pt_->point.z*step, pt_->point_uav.x*step, pt_->point_uav.y*step, pt_->point_uav.z*step, pt_->cost);
			i_++;
		}
		rrtgm.getPathMarker(rrt_path, lines_ugv_marker_pub_, lines_uav_marker_pub_);
		ret_val = rrt_path.size();
	}
	else
		printf("RandomPlanner::computeTreeCoupled -->  could't found path for Coupled Marsupial Configuration-->  number iteration: %lu \n\n", nodes_tree.size());    

  	std::cout << "Finishing Random Planner: Explored Graph Nodes Numbers: " << nodes_tree.size() <<std::endl;
  	std::cout << "Finishing Random Planner: Explored Graph Nodes Numbers to Take Off: " << take_off_nodes.size() <<std::endl;
	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;

  return ret_val; 
}

int RandomPlanner::computeTreesIndependent()
{
 	double ret_val = -1.0; 
	
// for (int count_rrt = 0 ; count_rrt < 100 ; count_rrt++){
	clearStatus();
	if(nodes_marker_debug)
		rrtgm.clearMarkersNodesTree(tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_, take_off_nodes_pub_);
	rrtgm.clearMarkers(lines_ugv_marker_pub_, lines_uav_marker_pub_);

	if(get_catenary_data)
		clock_gettime(CLOCK_REALTIME, &start_rrt);

	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;
	printf("RandomPlanner::computeTreesIndependent -->  STARTING --> star_point_ugv[%.2f %.2f %.2f/%.2f %.2f %.2f]  goal_point=[%.2f %.2f %.2f] \n\n", 
			initial_position_ugv.x, initial_position_ugv.y, initial_position_ugv.z, initial_position_uav.x, initial_position_uav.y, initial_position_uav.z, 
			final_position.x, final_position.y, final_position.z);  
    
	v_min_dist_obs_cat.clear(); v_length_cat.clear(); v_time_cat.clear();

	setInitialCostGoal(disc_final);
	
	
	if(!saveNode(disc_initial,true)){
		ROS_ERROR("RandomPlanner::computeTreesIndependent --> Not posible to get catenary in initial node");
		return 0;
	}else
		printf("RandomPlanner::computeTreesIndependent --> Saved initial node\n\n");

		rrtgm.getCatenaryMarker(points_catenary_new_node, one_catenary_marker_pub_);

	rrtgm.goalPointMarker(final_position, goal_point_pub_);
	
	count_graph = 0;
	if(nodes_marker_debug)
		rrtgm.getGraphMarker(disc_initial, count_graph, tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_);

	updateKdtreeNode(*disc_initial);
	updateKdtreeUGV(*disc_initial);
	updateKdtreeUAV(*disc_initial);

    count_loop = 0; 
	int count_total_loop = 0; 
	count_qnew_fail = count_fail_connect_goal = -1;


	while (count_loop < n_iter) { // n_iter Max. number of nodes to expand for each round
		// printf("\t\t-----  Planner (%s) :: computeTreeIndependent: iter=[%i/%i] , loop=[%i/%i] , total_node_save[%lu/%i]-----\n",planner_type.c_str(), count_loop+1, n_iter, count_total_loop+1, n_loop, nodes_tree.size(), (count_loop+1)+(500*count_total_loop));
		// printf("\t\t-----  Planner (%s) :: computeTreeIndependent: iter=[%i/%i] , loop=[%i/%i] , total_node_save[%lu/%i]-----\r",planner_type.c_str(), count_loop+1, n_iter, count_total_loop+1, n_loop, nodes_tree.size(), (count_loop+1)+(500*count_total_loop));
		
		RRTNode q_rand;
		
			if ((count_loop%samp_goal_rate)!=0)
				q_rand = getRandomNode();	
			else
				q_rand = getGoalNode();	

		if (debug_rrt)
			printf(" q_rand = [%f %f %f / %f %f %f] \n",q_rand.point.x*step,q_rand.point.y*step,q_rand.point.z*step,q_rand.point_uav.x*step,q_rand.point_uav.y*step,q_rand.point_uav.z*step);
		
		rrtgm.randNodeMarker(q_rand, rand_point_pub_, 1);

		new_solution = false;
		extendGraph(q_rand);
		count_loop++;

		// rrtgm.getCatenaryMarker(points_catenary_new_node, one_catenary_marker_pub_);
		/********************* To graph Catenary Node *********************/
		// std::string y_ ;
		// std::cin >> y_ ;
		// std::cout << "Continue DO-WHILE loop : " << y_ << std::endl;
		/******************************************************************/

		if ( ( (num_goal_finded>0) && (planner_type == "rrt") ) || 
			 ( (num_goal_finded>0) && (planner_type == "rrt_star") && (count_loop == n_iter) ) ||
			 ( (num_goal_finded>0) && (planner_type == "birrt") ) ){

			if (debug_rrt)
				printf("\n\n\nRandomPlanner::computeTreesIndependent -->  goal found for Coupled Marsupial Configuration.\n")	; 
			rrt_path = getPath();
			if (debug_rrt) 
				printf("RandomPlanner::computeTreesIndependent -->  path found for Coupled Marsupial Configuration--> (path size: %lu , iteration numbers: %i) : \n",rrt_path.size(), (count_loop)+(500*count_total_loop)); 
			if (planner_type == "rrt_star")
				printf("RandomPlanner::computeTreesIndependent -->  number of goals found: %i\n",num_goal_finded); 
			int i_=0;
			if (debug_rrt)   
				printf("\tPrinting the Path Nodes obteinded through planner (%s) : \n",planner_type.c_str());
			for (auto pt_: rrt_path){
				if (debug_rrt) 
					printf("\tRandom_planner_node[%i/%lu] :  ugv=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f]  uav=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f]  length_catenary=%.3f    cost=%.3f\n", i_, rrt_path.size(),
						   pt_->point.x*step, pt_->point.y*step, pt_->point.z*step, pt_->rot_ugv.x, pt_->rot_ugv.y, pt_->rot_ugv.z, pt_->rot_ugv.w, pt_->point_uav.x*step, 
						   pt_->point_uav.y*step, pt_->point_uav.z*step, pt_->rot_uav.x, pt_->rot_uav.y, pt_->rot_uav.z, pt_->rot_uav.w, pt_->length_cat, pt_->cost);
				i_++;
			}
			rrtgm.getPathMarker(rrt_path, lines_ugv_marker_pub_, lines_uav_marker_pub_);
			rrtgm.getCatenaryPathMarker(rrt_path, catenary_marker_pub_, grid_3D, distance_catenary_obstacle, map, nn_trav_ugv.kdtree, nn_trav_ugv.obs_points);
			ret_val = rrt_path.size();
			break;
		}
		
		else if (count_loop >= n_iter){
			count_total_loop++;
			count_loop = 0;
			if (count_total_loop > n_loop-1){
				if (debug_rrt)
					printf("RandomPlanner::computeTreesIndependent -->  could't find path for Coupled Marsupial Configuration-->  number iteration: %lu \n\n", nodes_tree.size());    
				ret_val = 0;
				break;
			}
			else {

				if (debug_rrt)
					printf("\n\t\t       Planner (%s) :: computeTreeIndependent: Starting new Loop      \n",planner_type.c_str());
			}
		}
	}
	// Data Analysis for Catenary 
	if(get_catenary_data){
		clock_gettime(CLOCK_REALTIME, &finish_rrt);
		sec_rrt = finish_rrt.tv_sec - start_rrt.tv_sec;
		msec_rrt = -(start_rrt.tv_nsec) + finish_rrt.tv_nsec;
		time_rrt = (msec_rrt + sec_rrt * 1000000000.0)/1000000000.0;
		// Compute means catenary values
		double mean_sum_values_dis_, mean_sum_values_length_, mean_sum_time_cat_, sum_values_dis_, sum_values_length_ , sum_time_cat_, min_dist_obs_cat_;
		mean_sum_values_dis_ = mean_sum_values_length_ = mean_sum_time_cat_ = sum_values_dis_ = sum_values_length_ = sum_time_cat_ = 0.0;
		min_dist_obs_cat_ = 10000000.0;
		double min_time_get_cat = 10000000.0;
		double max_time_get_cat = 0.0;
		
		printf("Catenary Analysis - Size Vectors: dist_cat_obs=[%lu] length_cat=[%lu] v_time_cat=[%lu]", v_min_dist_obs_cat.size(), v_length_cat.size(),v_time_cat.size());
		for (size_t i=0 ; i < v_min_dist_obs_cat.size(); i++){
			sum_values_dis_ = sum_values_dis_ + v_min_dist_obs_cat[i];
			sum_values_length_ = sum_values_length_ + v_length_cat[i];
			if ( min_dist_obs_cat_ > v_min_dist_obs_cat[i])
				min_dist_obs_cat_ = v_min_dist_obs_cat[i];
		}
		for (size_t i=0 ; i < v_time_cat.size(); i++){
			sum_time_cat_ = sum_time_cat_ + v_time_cat[i];
			if (max_time_get_cat < v_time_cat[i])
				max_time_get_cat = v_time_cat[i];	
			if (min_time_get_cat > v_time_cat[i])
				min_time_get_cat = v_time_cat[i];
		}

		double size_ = v_min_dist_obs_cat.size();
		mean_sum_values_dis_ = sum_values_dis_/size_;
		mean_sum_values_length_ = sum_values_length_/size_;
		mean_sum_time_cat_ = sum_time_cat_/size_;
		
		std::ifstream ifs_cat;
		std::ofstream ofs_cat;
		ifs_cat.open(catenary_file);
		if(ifs_cat) {
			std::cout << catenary_file <<" : File exists !!!!!!!!!! " << std::endl;
		} else {
		ofs_cat.open(catenary_file.c_str(), std::ofstream::app);
		ofs_cat <<"Time_Path;Mean_time_Cat;Max_time_Cat;Min_time_Cat;Mean_dist_Obst;Min_dist_obst;"<<std::endl;
		ofs_cat.close();
		std::cout << catenary_file <<" : File doesn't exist !!!!!!!!!! " << std::endl;
		}
		
		ofs_cat.open(catenary_file.c_str(), std::ofstream::app);
		if (ofs_cat.is_open()) {
			std::cout << "Saving catenary data in: " << catenary_file << std::endl;
			ofs_cat << time_rrt << ";"
					<< mean_sum_time_cat_ << ";"
					<< max_time_get_cat << ";"
					<< min_time_get_cat << ";"
					<< mean_sum_values_dis_ << ";"
					<< min_dist_obs_cat_ << ";"
					<< std::endl;
		} 
		else 
			std::cout << "Couldn't be open the output catenaty data in " << catenary_file << " finle" << std::endl;
		ofs_cat.close();

		ROS_INFO(PRINTF_YELLOW "Exporting catenary stats");

        ccm->exportStats("catenary_stats.txt");
	}

  	std::cout << "Finishing Random Planner: Explored Graph Nodes Numbers: " << nodes_tree.size() <<std::endl;
	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;

	if (markers_debug)
		rrtgm.getAllCatenaryMarker(nodes_tree, all_catenary_marker_pub_);

	ros::Duration(0.0).sleep();
// }
  	return ret_val; 
}

bool RandomPlanner::extendGraph(const RRTNode q_rand_)
{ 
	if(is_coupled){
		RRTNode* new_node = new RRTNode();
		RRTNode q_new ;			//Take the new node value before to save it as a node in the list

		RRTNode* q_nearest = getNearestNode(q_rand_); 

		bool exist_q_new_ = steering(*q_nearest, q_rand_, step_steer, q_new);
		
		RRTNode *q_min;
		if (checkNodeFeasibility(q_new,false)){
			if (obstacleFreeBetweenNodes(*q_nearest, q_new)){
				q_min = q_nearest;
			}
			else{
				ROS_INFO_COND(debug_rrt, PRINTF_RED"  RandomPlanner::extendGraph : Not Obstacle Free between q_new = [%f %f %f] and q_nearest =[%f %f %f]", q_new.point.x*step, q_new.point.y*step, q_new.point.z*step, q_nearest->point.x*step, q_nearest->point.y*step, q_nearest->point.z*step);
				return false;
			}
		}
		else{
			ROS_INFO_COND(debug_rrt, PRINTF_RED"  RandomPlanner::extendGraph : Not Feasible to extend point q_new = [%f %f %f]",q_new.point.x*step, q_new.point.y*step, q_new.point.z*step);
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
							// ROS_ERROR("RandomPlanner::extendGraph -->  exist collision between one of <X_near node> and <X_new node> !!");
						}
					}
				}
			}
		}
		q_new.parentNode = q_min;
		getParamsNode(q_new);
		
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
	else
	{
		RRTNode* new_node = new RRTNode();
		RRTNode q_new;	//Take the new node value before to save it as a node in the list
		
        // clock_gettime(CLOCK_REALTIME, &start_nearest);
			RRTNode* q_nearest = getNearestNode(q_rand_); 
        // clock_gettime(CLOCK_REALTIME, &finish_nearest);
		// sec_nearest = finish_nearest.tv_sec - start_nearest.tv_sec - 1;
        // msec_nearest = (1000000000 - start_nearest.tv_nsec) + finish_nearest.tv_nsec;
		// time_nearest = (msec_nearest + sec_nearest * 1000000000.0)/1000000000.0;
		
		rrtgm.randNodeMarker(*q_nearest, nearest_point_pub_, 0);

		if (debug_rrt) 
			printf(" q_nearest = [%f %f %f / %f %f %f] l=%f feasible_l=%s\n", q_nearest->point.x*step,q_nearest->point.y*step,q_nearest->point.z*step, q_nearest->point_uav.x*step,q_nearest->point_uav.y*step,q_nearest->point_uav.z*step, 
			q_nearest->length_cat, q_nearest->catenary? "true" : "false");
		
        // clock_gettime(CLOCK_REALTIME, &start_steer);
		bool exist_q_new_ = steering(*q_nearest, q_rand_, step_steer, q_new);
		// clock_gettime(CLOCK_REALTIME, &finish_steer);
		// sec_steer = finish_steer.tv_sec - start_steer.tv_sec - 1;
        // msec_steer = (1000000000 - start_steer.tv_nsec) + finish_steer.tv_nsec;
		// time_steer = (msec_steer + sec_steer * 1000000000.0)/1000000000.0;
		v_time_cat.push_back(time_cat);


		if (!exist_q_new_){
			if ((count_loop%samp_goal_rate)==0)
				count_fail_connect_goal++;
			// ROS_INFO( PRINTF_CYAN"  RRTPlanner::extendGraph : Not new point q_new=[%f %f %f / %f %f %f]  q_nearest=[%f %f %f / %f %f %f]",
            //        q_new.point.x*step, q_new.point.y*step, q_new.point.z*step, q_new.point_uav.x*step, q_new.point_uav.y*step, q_new.point_uav.z*step,
            //        q_nearest->point.x*step, q_nearest->point.y*step, q_nearest->point.z*step, q_nearest->point_uav.x*step, q_nearest->point_uav.y*step, q_nearest->point_uav.z*step);
			/********************* To graph Catenary Node *********************/
			// std::string y_ ;
			// std::cin >> y_ ;
			// std::cout << "Continue DO-WHILE loop : " << y_ << std::endl;
			/******************************************************************/
			if (debug_rrt) 
				printf(" exist_q_new_=%s\n", exist_q_new_? "true" : "false");
			return false;
		}

		if ((count_loop%samp_goal_rate)==0)
				count_fail_connect_goal = 0;

		rrtgm.randNodeMarker(q_new, new_point_pub_, 2);

		q_new.parentNode = q_nearest;
		getParamsNode(q_new);
		
		RRTNode *q_min;
		q_min = q_nearest;
		// ROS_INFO(PRINTF_BLUE"  RRTPlanner::extendGraph : Not Obstacle Free q_new=[%f %f %f / %f %f %f]  q_nearest=[%f %f %f / %f %f %f]", q_new.point.x*step, 
					// q_new.point.y*step, q_new.point.z*step, q_new.point_uav.x*step, q_new.point_uav.y*step, q_new.point_uav.z*step, q_nearest->point.x*step, 
					// q_nearest->point.y*step, q_nearest->point.z*step, q_nearest->point_uav.x*step, q_nearest->point_uav.y*step, q_nearest->point_uav.z*step);

		if (debug_rrt)
			printf(" q_min = [%f %f %f / %f %f %f] \n", q_min->point.x*step,q_min->point.y*step,q_min->point.z*step,q_min->point_uav.x*step,q_min->point_uav.y*step,q_min->point_uav.z*step);

        // clock_gettime(CLOCK_REALTIME, &start_near);
		std::vector<int> v_near_nodes = getNearNodes(q_new, radius_near_nodes) ;
		// clock_gettime(CLOCK_REALTIME, &finish_near);
		// sec_near = finish_near.tv_sec - start_near.tv_sec - 1;
        // msec_near = (1000000000 - start_near.tv_nsec) + finish_near.tv_nsec;
		// time_near = (msec_near + sec_near * 1000000000.0)/1000000000.0;

		updateKdtreeNode(q_new); 	//KdTree is updated after get Near Nodes because to not take it own node as a near node
		updateKdtreeUGV(q_new);
		updateKdtreeUAV(q_new);
		bool new_parentNode_ = false;

		
        // clock_gettime(CLOCK_REALTIME, &start_connect);
			// I . Open near nodes and connected with minimum accumulated cost
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
		// clock_gettime(CLOCK_REALTIME, &finish_connect);
		// sec_connect = finish_connect.tv_sec - start_connect.tv_sec - 1;
        // msec_connect = (1000000000 - start_connect.tv_nsec) + finish_connect.tv_nsec;
		// time_connect = (msec_connect + sec_connect * 1000000000.0)/1000000000.0;
		if (new_parentNode_ ){
			q_new.parentNode = q_min;
			updateParamsNode(q_new);
		}

		*new_node = q_new;
		
        // clock_gettime(CLOCK_REALTIME, &start_rewire);
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
		// clock_gettime(CLOCK_REALTIME, &finish_rewire);
		// sec_rewire = finish_rewire.tv_sec - start_rewire.tv_sec - 1;
        // msec_rewire = (1000000000 - start_rewire.tv_nsec) + finish_rewire.tv_nsec;
		// time_rewire = (msec_rewire + sec_rewire * 1000000000.0)/1000000000.0;

		int got_goal_aux_ = got_to_goal; 
		isGoal(q_new);
		if (got_to_goal != got_goal_aux_){
			saveNode(new_node);
			if(new_node->cost < disc_goal->cost){
				printf("\n");
				ROS_INFO(PRINTF_ROSE"\n\n\n\t\t !!!!!!!!!!!!!  Got it GOAL position new node->point : [%f %f %f/%f %f %f]  !!!!!!!!!!!!! \n\n",
						  new_node->point.x*step, new_node->point.y*step, new_node->point.z*step,
						  new_node->point_uav.x*step, new_node->point_uav.y*step, new_node->point_uav.z*step);
				disc_goal = new_node;
				num_goal_finded++;
				new_solution = true;
			}
		}	
		else
			saveNode(new_node);
		
		if(get_catenary_data){
			double min_dist_obs_cat_ = new_node->min_dist_obs_cat;
			double length_cat_ = new_node->length_cat;
			v_min_dist_obs_cat.push_back(min_dist_obs_cat_);
			v_length_cat.push_back(length_cat_);
		}
		
		if(nodes_marker_debug){
			count_graph++;
			rrtgm.getGraphMarker(new_node, count_graph, tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_);
		}

		// ROS_INFO(PRINTF_ROSE"  new node->point : [%f %f %f/%f %f %f] , catenary= %s , length_cat= %f , min_dist_obs_cat = %f , cost=%f\n" ,
		// 				  new_node->point.x*step, new_node->point.y*step, new_node->point.z*step,
		// 				  new_node->point_uav.x*step, new_node->point_uav.y*step, new_node->point_uav.z*step,
		// 				  new_node->catenary? "true":"false", new_node->length_cat, new_node->min_dist_obs_cat, new_node->cost);
		// std::cout << "points_catenary_.size() = " << points_catenary_new_node.size()  << std::endl <<std::endl;
		
		rrtgm.getCatenaryMarker(points_catenary_new_node, one_catenary_marker_pub_);
		
		return true;
	}
}

RRTNode RandomPlanner::getRandomNode() 
{
	// Random numbers
    std::random_device rd;   // obtain a random number from hardware
  	std::mt19937 eng(rd());  // seed the generator
	int max_ugv = (int)v_points_ws_ugv.size() -1;
  	std::uniform_int_distribution<int> distr_ugv(0, max_ugv);  // define the discrete range
  	std::uniform_int_distribution<int> distr_x_uav(ws_x_min, ws_x_max);  // define the discrete range
  	std::uniform_int_distribution<int> distr_y_uav(ws_y_min, ws_y_max);  // define the discrete range
  	std::uniform_int_distribution<int> distr_z_uav(ws_z_min+(0.0+0.6*step_inv), ws_z_max);  // define the range 1.0+0.6*step_inv

	RRTNode randomState_;
	bool found_node = false;
	bool catenary_state = false;


	// Get random position for UAV
	if (!is_coupled){
		do{
			randomState_.point_uav.x = distr_x_uav(eng);
			randomState_.point_uav.y = distr_y_uav(eng);
			randomState_.point_uav.z = distr_z_uav(eng);
			found_node = checkNodeFeasibility(randomState_,true);

		}while(found_node == false);		
	} else {
		do{
			randomState_.point_uav.x = disc_final->point_uav.x;
			randomState_.point_uav.y = disc_final->point_uav.y;
			randomState_.point_uav.z = disc_final->point_uav.z;
			found_node = checkNodeFeasibility(randomState_,true);
		} while(found_node == false);
	}
	
	// Get random position for UGV
	do{
		int num_rand = distr_ugv(eng);
		randomState_.point.x = v_points_ws_ugv[num_rand].x*step_inv;
		randomState_.point.y = v_points_ws_ugv[num_rand].y*step_inv;
		randomState_.point.z = v_points_ws_ugv[num_rand].z*step_inv;  
		if (randomState_.point.z < 1){
			found_node = checkNodeFeasibility(randomState_,false); 
		}
		else {
			found_node = checkNodeFeasibility(randomState_,true); 
		}
	}while(found_node == false);		
	

	return randomState_;
}

RRTNode RandomPlanner::getGoalNode() {
	RRTNode goalState;
	bool found_node = false;
	bool catenary_state = false;


	// Get random position for UAV
	if (!is_coupled){
		goalState.point_uav.x = disc_final->point_uav.x;
		goalState.point_uav.y = disc_final->point_uav.y;
		goalState.point_uav.z = disc_final->point_uav.z;
		found_node = checkNodeFeasibility(goalState,true);
	} else {
		if(sample_mode == 0){
			goalState.point.x = disc_final->point_uav.x;
			goalState.point.y = disc_final->point_uav.y;
			goalState.point.z = disc_initial->point.z;   
		}
		else if(sample_mode == 1){
			goalState.point.x = disc_final->point.x;
			goalState.point.y = disc_final->point.y;
			goalState.point.z = disc_final->point.z;
		}
	}
	return goalState;
}

RRTNode* RandomPlanner::getNearestNode(const RRTNode q_rand_) 
{
  	RRTNode* q_nearest_; 

	double p_ugv_x_, p_ugv_y_, p_ugv_z_, p_ugv_parent_x_, p_ugv_parent_y_, p_ugv_parent_z_, p_uav_x_, p_uav_y_, p_uav_z_; 
	double d_ugv_ , d_uav_, l_cat_, cos_angle;
	double k0_ ,k1_, k2_ ;
	k0_ = w_nearest_ugv; //  UGV
    k1_ = w_nearest_uav;  // UAV
	k2_ = w_nearest_smooth;  // Smoothness

	double cost_nearest_ = 10000000;
	for (auto nt_:nodes_tree) {
		p_ugv_x_ = q_rand_.point.x * step - nt_->point.x * step;
		p_ugv_y_ = q_rand_.point.y * step - nt_->point.y * step;
		p_ugv_z_ = q_rand_.point.z * step - nt_->point.z * step;
		p_uav_x_ = q_rand_.point_uav.x * step - nt_->point_uav.x * step;
		p_uav_y_ = q_rand_.point_uav.y * step - nt_->point_uav.y * step;
		p_uav_z_ = q_rand_.point_uav.z * step - nt_->point_uav.z * step;

		d_ugv_ =  ((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
		d_uav_ =  ((p_uav_x_*p_uav_x_) + (p_uav_y_*p_uav_y_) + (p_uav_z_*p_uav_z_));
		l_cat_ = nt_->length_cat;

		if (nt_->id != id_ugv_init  && nt_->id_uav != id_uav_init ){	
			//To get smoothness
			p_ugv_parent_x_ = nt_->parentNode->point.x * step - nt_->point.x * step;
			p_ugv_parent_y_ = nt_->parentNode->point.y * step - nt_->point.y * step;
			p_ugv_parent_z_ = nt_->parentNode->point.z * step - nt_->point.z * step;
			//Compute dot product and norm of vectors
			double dot_product = (p_ugv_parent_x_*p_ugv_x_) + (p_ugv_parent_y_*p_ugv_y_) + (p_ugv_parent_z_*p_ugv_z_);
			double norm_vector1 = sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
			double norm_vector2 = sqrt((p_ugv_parent_x_*p_ugv_parent_x_) + (p_ugv_parent_y_*p_ugv_parent_y_) + (p_ugv_parent_z_*p_ugv_parent_z_));
			if (norm_vector1 < 0.0001 || norm_vector2 < 0.0001 )
				cos_angle = 1.0;
			else
			cos_angle = dot_product/(norm_vector1 * norm_vector2);
		}else
			cos_angle = 1.0;

		double cost_ = k0_ * d_ugv_ + k1_* d_uav_ + k2_ * (1.0 - cos_angle ); //Cost to choose q_nearest

		if(cost_nearest_ > cost_){
			q_nearest_ = nt_;
			cost_nearest_ = cost_;
		}
	}
	return q_nearest_;
}

bool RandomPlanner::steering(const RRTNode &q_nearest_, const RRTNode &q_rand_, float factor_steer_, RRTNode &q_new_)	
{
	// RRTNode q_new_;
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
		
		Eigen::Vector3d p_node_, trav_point_ugv_, trav_point_ugv_aux;
		p_node_.x() = q_rand_.point.x;
		p_node_.y() = q_rand_.point.y;
		p_node_.z() = q_rand_.point.z;
		trav_point_ugv_aux = nn_trav_ugv.nearestObstacleMarsupial(nn_trav_ugv.kdtree, p_node_, nn_trav_ugv.obs_points);
		trav_point_ugv_ = nn_trav_ugv.nearestTraversabilityUGVMarsupial(nn_trav_ugv.kdtree, trav_point_ugv_aux, nn_trav_ugv.obs_points, 0.15);

		q_new_.point.x = trav_point_ugv_.x() * step_inv; 
		q_new_.point.y = trav_point_ugv_.y() * step_inv; 
		q_new_.point.z = trav_point_ugv_.z() * step_inv;
	}
	else{
		Eigen::Vector3d p_node_, trav_point_ugv_, trav_point_ugv_aux;
		p_node_.x() = x_ugv_;
		p_node_.y() = y_ugv_;
		p_node_.z() = z_ugv_;
		trav_point_ugv_aux = nn_trav_ugv.nearestObstacleMarsupial(nn_trav_ugv.kdtree, p_node_, nn_trav_ugv.obs_points);
		trav_point_ugv_ = nn_trav_ugv.nearestTraversabilityUGVMarsupial(nn_trav_ugv.kdtree, trav_point_ugv_aux, nn_trav_ugv.obs_points, 0.15);
		
		q_new_.point.x = trav_point_ugv_.x() * step_inv; 
		q_new_.point.y = trav_point_ugv_.y() * step_inv; 
		q_new_.point.z = trav_point_ugv_.z() * step_inv;
	}

	getOrientation(q_new_, q_nearest_, false);	

	// Here return the position of UGV in case in coupled mode or continue to get the position of UAV to return node
	if(is_coupled)
		return true;
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
		}
		else{
			q_new_.point_uav.x = x_uav_ * step_inv; 
			q_new_.point_uav.y = y_uav_ * step_inv; 
			q_new_.point_uav.z = z_uav_ * step_inv;
		}

		getOrientation(q_new_, q_nearest_, true);	
		
		// Following lines keep Get random sample for only UAV and nearest UGV position for UAV
		if(sample_mode == 1){ 
			std::vector<float> nearest_ugv_to_uav_  = getNearestUGVNode(q_new_);
			RRTNode q1_;
			q1_.point.x = (int)nearest_ugv_to_uav_[0];
			q1_.point.y = (int)nearest_ugv_to_uav_[1];
			q1_.point.z = (int)nearest_ugv_to_uav_[2];
			q1_.rot_ugv.x = nearest_ugv_to_uav_[3];
			q1_.rot_ugv.y = nearest_ugv_to_uav_[4];
			q1_.rot_ugv.z = nearest_ugv_to_uav_[5];
			q1_.rot_ugv.w = nearest_ugv_to_uav_[6];
			q1_.point_uav.x = q_new_.point_uav.x;
			q1_.point_uav.y = q_new_.point_uav.y;
			q1_.point_uav.z = q_new_.point_uav.z;
			q1_.rot_uav.x =  q_new_.rot_uav.x; 
			q1_.rot_uav.y =  q_new_.rot_uav.y; 
			q1_.rot_uav.z =  q_new_.rot_uav.z;
			q1_.rot_uav.w =  q_new_.rot_uav.w;

			bool check_ugv_feasible_ = checkNodeFeasibility(q1_,false);
			bool check_uav_feasible_ = checkNodeFeasibility(q1_,true);
			bool check_cat_feasible_ = checkCatenary(q1_, points_catenary_new_node);
			// Mode I: just steer UAV
			if (debug_rrt) {
				// if (!checkNodeFeasibility(q1_,false) || !checkNodeFeasibility(q1_,true) || !checkCatenary(q1_, 2, points_catenary_new_node)){
					printf(" q_new I: q_ugv[%f %f %f] q_uav[%f %f %f]\n", q1_.point.x*step, q1_.point.y*step, q1_.point.z*step, q1_.point_uav.x*step,q1_.point_uav.y*step ,q1_.point_uav.z*step); 
					printf(" checkNodeFeasibility(q1_,false) = %s\n",check_ugv_feasible_? "true" : "false");
					printf(" checkNodeFeasibility(q1_,true) = %s\n",check_uav_feasible_? "true" : "false");
					printf(" checkCatenary(q1_, points_catenary_new_node) = %s\n",check_cat_feasible_? "true" : "false");
				// }
			}
		
			clock_gettime(CLOCK_REALTIME, &start_cat);
			if (check_ugv_feasible_ && check_uav_feasible_ && check_cat_feasible_ && q1_.length_cat < min_dist_for_steer_ugv) {
				clock_gettime(CLOCK_REALTIME, &finish_cat);
				sec_cat = finish_cat.tv_sec - start_cat.tv_sec - 1;
				msec_cat = (1000000000 - start_cat.tv_nsec) + finish_cat.tv_nsec;
				time_cat = (msec_cat + sec_cat * 1000000000.0)/1000000000.0;
				bool obt_free_col_1_ = obstacleFreeBetweenNodes(q_nearest_, q1_);
				if (debug_rrt)
					printf(" obstacleFreeBetweenNodes(q_nearest_, q1_)=%s\n",obt_free_col_1_?"true":"false");
				if(obt_free_col_1_){
					q_new_ = q1_;
					count_qnew_fail = 0;
					// ROS_INFO(PRINTF_YELLOW"New position: UGV fix and moving UAV position q1_.length_cat = %f", q1_.length_cat);
					return true;
				}
				else{
					// ROS_INFO(PRINTF_YELLOW"NOT POSSIBLE New position: COLLISION TO UGV fix and moving UAV");
				}
			} // mode II: Steer UAV and UGV
			else{
				// ROS_INFO(PRINTF_YELLOW"NOT POSSIBLE New position: UGV fix and moving UAV");
			}
			
			// Mode II: steering UAV and UGV
			check_ugv_feasible_ = checkNodeFeasibility(q_new_,false);
			check_uav_feasible_ = checkNodeFeasibility(q_new_,true);
			check_cat_feasible_ = checkCatenary(q_new_, points_catenary_new_node);
			if (debug_rrt) {
				// if (!checkNodeFeasibility(q_new_,false) || !checkNodeFeasibility(q_new_,true) || !checkCatenary(q_new_, 2, points_catenary_new_node)){
					printf(" q_new II: q_ugv[%f %f %f] q_uav[%f %f %f]\n", q_new_.point.x*step, q_new_.point.y*step, q_new_.point.z*step, q_new_.point_uav.x*step ,q_new_.point_uav.y*step ,q_new_.point_uav.z*step); 
					printf(" checkNodeFeasibility(q_new_,false) = %s\n",check_ugv_feasible_? "true" : "false");
					printf(" checkNodeFeasibility(q_new_,true) = %s\n",check_uav_feasible_? "true" : "false");
					printf(" checkCatenary(q_new_, points_catenary_new_node) = %s\n",check_cat_feasible_? "true" : "false");
				// }
			}

			clock_gettime(CLOCK_REALTIME, &start_cat);
			if (check_ugv_feasible_ && check_uav_feasible_ && check_cat_feasible_){
				clock_gettime(CLOCK_REALTIME, &finish_cat);
				sec_cat = finish_cat.tv_sec - start_cat.tv_sec - 1;
				msec_cat = (1000000000 - start_cat.tv_nsec) + finish_cat.tv_nsec;
				time_cat = (msec_cat + sec_cat * 1000000000.0)/1000000000.0;
				bool obt_free_col_2_ = obstacleFreeBetweenNodes(q_nearest_, q_new_);
				if (debug_rrt) 
					printf(" obstacleFreeBetweenNodes(q_nearest_, q_new_)= %s\n",obt_free_col_2_?"true":"false");
				if(obt_free_col_2_){
					count_qnew_fail = 0;
					// ROS_INFO(PRINTF_RED"New position steer using random node");
					return true;
				} else {
					// ROS_INFO(PRINTF_RED"NOT POSSIBLE New position: COLLISION TO Steer using random node");
				}
			} else {
				// ROS_INFO(PRINTF_RED"NOT POSSIBLE New position: Steer using random node");
			}

			count_qnew_fail++;
			if  (count_qnew_fail > 5){
				RRTNode q2_;
				q2_.point.x = q_new_.point.x;
				q2_.point.y = q_new_.point.y;
				q2_.point.z = q_new_.point.z;
				q2_.rot_ugv.x =  q_new_.rot_ugv.x; 
				q2_.rot_ugv.y =  q_new_.rot_ugv.y; 
				q2_.rot_ugv.z =  q_new_.rot_ugv.z;
				q2_.rot_ugv.w =  q_new_.rot_ugv.w;
				std::vector<float> nearest_uav_to_ugv_  = getNearestUAVNode(q_new_);
				q2_.point_uav.x = (int)nearest_uav_to_ugv_[0];
				q2_.point_uav.y = (int)nearest_uav_to_ugv_[1];
				q2_.point_uav.z = (int)nearest_uav_to_ugv_[2];
				q2_.rot_uav.x = nearest_uav_to_ugv_[3];
				q2_.rot_uav.y = nearest_uav_to_ugv_[4];
				q2_.rot_uav.z = nearest_uav_to_ugv_[5];
				q2_.rot_uav.w = nearest_uav_to_ugv_[6];

			// Mode III: steering just UGV
			check_ugv_feasible_ = checkNodeFeasibility(q2_,false);
			check_uav_feasible_ = checkNodeFeasibility(q2_,true);
			check_cat_feasible_ = checkCatenary(q2_, points_catenary_new_node);
			if (debug_rrt) {
				// if (!checkNodeFeasibility(q2_,false) || !checkNodeFeasibility(q2_,true) || !checkCatenary(q2_, 2, points_catenary_new_node)) {
					printf(" q_new III: q_ugv[%f %f %f] q_uav[%f %f %f]\n", q2_.point.x*step, q2_.point.y*step, q2_.point.z*step, q2_.point_uav.x*step ,q2_.point_uav.y*step ,q2_.point_uav.z*step); 
					printf(" checkNodeFeasibility(q2_,false) = %s\n",check_ugv_feasible_? "true" : "false");
					printf(" checkNodeFeasibility(q2_,true) = %s\n",check_uav_feasible_? "true" : "false");
					printf(" checkCatenary(q2_, points_catenary_new_node) = %s\n",check_cat_feasible_? "true" : "false");	
				// }
			}

			clock_gettime(CLOCK_REALTIME, &start_cat);
			if (check_ugv_feasible_ && check_uav_feasible_ && check_cat_feasible_ && q2_.length_cat < min_dist_for_steer_ugv) {
				clock_gettime(CLOCK_REALTIME, &finish_cat);
				sec_cat = finish_cat.tv_sec - start_cat.tv_sec - 1;
				msec_cat = (1000000000 - start_cat.tv_nsec) + finish_cat.tv_nsec;
				time_cat = (msec_cat + sec_cat * 1000000000.0)/1000000000.0;
				bool obt_free_col_3_ = obstacleFreeBetweenNodes(q_nearest_, q2_);
				if (debug_rrt) 
					printf(" obstacleFreeBetweenNodes(q_nearest_, q2_)=%s\n",obt_free_col_3_?"true":"false");
				if(obt_free_col_3_){
					q_new_ = q2_;
					count_qnew_fail = 0;
					// ROS_INFO(PRINTF_ORANGE"New position: UAV fix and moving UGV position q1_.length_cat = %f", q2_.length_cat);
					return true;
				}
				else{
					// ROS_INFO(PRINTF_ORANGE"NOT POSSIBLE New position: COLLISION TO UAV fix and moving UGV");
					}
			}
			else{
				// ROS_INFO(PRINTF_ORANGE"NOT POSSIBLE New position: UAV fix and moving UGV");
			}
			}
		}
		return false;
	}
}

bool RandomPlanner::obstacleFreeBetweenNodes(const RRTNode q_nearest_,const RRTNode q_new_)
{
	geometry_msgs::Point  point_nearest_uav_ , point_new_uav_,p_reel_nearest_, p_reel_new_;
	std::vector<geometry_msgs::Point> points_cat_nearest_, points_cat_new_;
	points_cat_new_ = points_catenary_new_node;

	if (points_cat_new_.size() == 0) {

		ROS_INFO(PRINTF_RED"Chungo");
		return false;
	}


	point_new_uav_.x = q_new_.point_uav.x * step; 
	point_new_uav_.y = q_new_.point_uav.y * step; 
	point_new_uav_.z = q_new_.point_uav.z * step;

	point_nearest_uav_.x = q_nearest_.point_uav.x * step; 
	point_nearest_uav_.y = q_nearest_.point_uav.y * step; 
	point_nearest_uav_.z = q_nearest_.point_uav.z * step;

	//Get points for both catenary New_node and Nearest_node
	// CatenarySolver cSolver_;
	// cSolver_.setMaxNumIterations(100);

	p_reel_nearest_ = getReelNode(q_nearest_);
	// double l_cat_nearest_ = q_nearest_.length_cat;
	// cSolver_.solve(p_reel_nearest_.x, p_reel_nearest_.y, p_reel_nearest_.z, point_nearest_uav_.x, point_nearest_uav_.y, point_nearest_uav_.z, l_cat_nearest_, points_cat_nearest_);

    // std::cout << "RandomPlanner::obstacleFreeBetweenNodes l_cat_nearest_=" << q_nearest_.length_cat << std::endl;
    // std::cout << "RandomPlanner::obstacleFreeBetweenNodes p_reel[=" 
	// 		  << p_reel_nearest_.x << "," << p_reel_nearest_.y << "," << p_reel_nearest_.z 
	// 		  << "] p_nearest[" 
	// 		  << point_nearest_uav_.x << "," << point_nearest_uav_.y << "," << point_nearest_uav_.z << "]" <<std::endl;
	  

	double r_;

    // std::cout << "RandomPlanner::obstacleFreeBetweenNodes points_cat_nearest_.size()=" << q_nearest_.p_cat.size() << std::endl;
	// for(size_t i = 0 ; i < points_cat_new_.size() ; i++){
    // std::cout << "points_cat_new_=[" << points_cat_new_[i].x << "," << points_cat_new_[i].y << "," << points_cat_new_[i].z << "]" << std::endl;
	// }
	// for(size_t i = 0 ; i < q_nearest_.p_cat.size() ; i++){
    // 	std::cout << "points_cat_nearest_=[" << q_nearest_.p_cat[i].x << "," << q_nearest_.p_cat[i].y << "," << q_nearest_.p_cat[i].z << "]" << std::endl;	
	// }

	std::vector<geometry_msgs::Point> points_obstacles_;
	points_obstacles_.clear();
	if(points_cat_new_.size() > q_nearest_.p_cat.size()){
		r_ = (double)(q_nearest_.p_cat.size()-1.0)/(double)(points_cat_new_.size()-1.0);
		for(size_t i = 1 ; i < points_cat_new_.size() ; i++){
			int k_ = (int)(round(r_ * (i)));
			//Get the unitary vector from nearest to rand direction
    //   std::cout << "RandomPlanner::checkPointsCatenaryFeasibility points_cat_nearest_[=" << q_nearest_.p_cat[k_].x << "," << q_nearest_.p_cat[k_].y << "," << q_nearest_.p_cat[k_].z << "]" << std::endl;
			float dir_x_ = q_nearest_.p_cat[k_].x - points_cat_new_[i].x;
			float dir_y_ = q_nearest_.p_cat[k_].y - points_cat_new_[i].y; 
			float dir_z_ = q_nearest_.p_cat[k_].z - points_cat_new_[i].z;
			float dist_nearest_new_ = sqrt(dir_x_*dir_x_ + dir_y_*dir_y_ + dir_z_*dir_z_);
			float uni_x_, uni_y_, uni_z_;
			if (dist_nearest_new_ < 0.00001){
				uni_x_ = 0.0;
				uni_y_ = 0.0;
				uni_z_ = 0.0;
			}
			else{
				uni_x_ = dir_x_/ dist_nearest_new_;
				uni_y_ = dir_y_/ dist_nearest_new_;
				uni_z_ = dir_z_/ dist_nearest_new_;
			}
			int j_ = 1;
			geometry_msgs::Point point_obs_;
			RRTNode check_point_;
			do{
				point_obs_.x = (points_cat_new_[i].x + uni_x_*step * (double)j_);
				point_obs_.y = (points_cat_new_[i].y + uni_y_*step * (double)j_);
				point_obs_.z = (points_cat_new_[i].z + uni_z_*step * (double)j_);
				check_point_.point.x = (point_obs_.x)*step_inv; 
				check_point_.point.y = (point_obs_.y)*step_inv; 
				check_point_.point.z = (point_obs_.z)*step_inv; 
				j_++;
				bool is_feasible_ = checkPointsCatenaryFeasibility(check_point_);
	// printf("\t 1 RandomPlanner::checkPointsCatenaryFeasibility(check_point_)=%s check_point_[%f %f %f]\n",is_feasible_?"true":"false",(point_obs_.x), (point_obs_.y), (point_obs_.z));
				if(!is_feasible_){
					return false;
				}
			}while ( (  pow(point_obs_.x - q_nearest_.p_cat[k_].x,2) + 
						pow(point_obs_.y - q_nearest_.p_cat[k_].y,2) + 
						pow(point_obs_.z - q_nearest_.p_cat[k_].z,2)) > step*step);
		}
	}
	else if(points_cat_new_.size() < q_nearest_.p_cat.size()){
		r_ = (double)(points_cat_new_.size()-1.0)/(double)(q_nearest_.p_cat.size()-1.0);
		for(size_t i = 1 ; i < q_nearest_.p_cat.size() ; i++){
			int k_ = (int)(round(r_ * (i)));
    //   std::cout << "RandomPlanner::checkPointsCatenaryFeasibility points_cat_nearest_[=" << q_nearest_.p_cat[i].x << "," << q_nearest_.p_cat[i].y << "," << q_nearest_.p_cat[i].z << "]" << std::endl;
			//Get the unitary vector from nearest to rand direction
			float dir_x_ = q_nearest_.p_cat[i].x - points_cat_new_[k_].x;
			float dir_y_ = q_nearest_.p_cat[i].y - points_cat_new_[k_].y; 
			float dir_z_ = q_nearest_.p_cat[i].z - points_cat_new_[k_].z;
			float dist_nearest_new_ = sqrt(dir_x_*dir_x_ + dir_y_*dir_y_ + dir_z_*dir_z_);
			float uni_x_, uni_y_, uni_z_;
			if (dist_nearest_new_ < 0.00001){
				uni_x_ = 0.0;
				uni_y_ = 0.0;
				uni_z_ = 0.0;
			}
			else{
				uni_x_ = dir_x_/ dist_nearest_new_;
				uni_y_ = dir_y_/ dist_nearest_new_;
				uni_z_ = dir_z_/ dist_nearest_new_;
			}
			int j_ = 1;
			geometry_msgs::Point point_obs_;
			RRTNode check_point_;
			do{
				point_obs_.x = (points_cat_new_[k_].x + uni_x_ *step* (double)j_);
				point_obs_.y = (points_cat_new_[k_].y + uni_y_ *step* (double)j_);
				point_obs_.z = (points_cat_new_[k_].z + uni_z_ *step* (double)j_);
				check_point_.point.x = (point_obs_.x)*step_inv; 
				check_point_.point.y = (point_obs_.y)*step_inv; 
				check_point_.point.z = (point_obs_.z)*step_inv; 
				j_++;
				bool is_feasible_ = checkPointsCatenaryFeasibility(check_point_);
	// printf("\t 2 checkPointsCatenaryFeasibility(check_point_)=%s check_point_[%f %f %f]\n",is_feasible_?"true":"false", (point_obs_.x), (point_obs_.y), (point_obs_.z));
				if(!is_feasible_){
					return false;
				}
			}while ( (  pow(point_obs_.x - q_nearest_.p_cat[i].x,2) + 
						pow(point_obs_.y - q_nearest_.p_cat[i].y,2) + 
						pow(point_obs_.z - q_nearest_.p_cat[i].z,2)) > step*step);
		}
	}
	else{
		for(size_t i = 1 ; i < q_nearest_.p_cat.size() ; i++){
			//Get the unitary vector from nearest to rand direction
    //   std::cout << "RandomPlanner::checkPointsCatenaryFeasibility points_cat_nearest_[=" << q_nearest_.p_cat[i].x << "," << q_nearest_.p_cat[i].y << "," << points_cat_nearest_[i].z << "]" << std::endl;
			float dir_x_ = q_nearest_.p_cat[i].x - points_cat_new_[i].x;
			float dir_y_ = q_nearest_.p_cat[i].y - points_cat_new_[i].y; 
			float dir_z_ = q_nearest_.p_cat[i].z - points_cat_new_[i].z;
			float dist_nearest_new_ = sqrt(dir_x_*dir_x_ + dir_y_*dir_y_ + dir_z_*dir_z_);
			float uni_x_, uni_y_, uni_z_;
			if (dist_nearest_new_ < 0.00001){
				uni_x_ = 0.0;
				uni_y_ = 0.0;
				uni_z_ = 0.0;
			}
			else{
				uni_x_ = dir_x_/ dist_nearest_new_;
				uni_y_ = dir_y_/ dist_nearest_new_;
				uni_z_ = dir_z_/ dist_nearest_new_;
			}
			int j_ = 1;
			geometry_msgs::Point point_obs_;
			RRTNode check_point_;
			do{
				point_obs_.x = (points_cat_new_[i].x + uni_x_*step * (double)j_);
				point_obs_.y = (points_cat_new_[i].y + uni_y_*step * (double)j_);
				point_obs_.z = (points_cat_new_[i].z + uni_z_*step * (double)j_);
				check_point_.point.x = (point_obs_.x)*step_inv; 
				check_point_.point.y = (point_obs_.y)*step_inv; 
				check_point_.point.z = (point_obs_.z)*step_inv; 
				j_++;
				bool is_feasible_ = checkPointsCatenaryFeasibility(check_point_);
	// printf("\t 3 checkPointsCatenaryFeasibility(check_point_)=%s check_point_[%f %f %f]\n",is_feasible_?"true":"false",(point_obs_.x),(point_obs_.y),(point_obs_.z));
				if(!is_feasible_){
					return false;
				}
			}while ( (  pow(point_obs_.x - q_nearest_.p_cat[i].x,2) + 
						pow(point_obs_.y - q_nearest_.p_cat[i].y,2) +
						pow(point_obs_.z - q_nearest_.p_cat[i].z,2)) > step*step);
		}
	}	
	return true;	
}

std::vector<int> RandomPlanner::getNearNodes(const RRTNode &q_new_, double radius_) 
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

std::vector<float> RandomPlanner::getNearestUGVNode(const RRTNode &q_new_)  
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
    ret_.push_back(x_ugv_to_uav_);
    ret_.push_back(y_ugv_to_uav_);
    ret_.push_back(z_ugv_to_uav_);
    ret_.push_back(x_rot_ugv_);
    ret_.push_back(y_rot_ugv_);
    ret_.push_back(z_rot_ugv_);
    ret_.push_back(w_rot_ugv_);

	return ret_;
}

std::vector<float> RandomPlanner::getNearestUAVNode(const RRTNode &q_new_)  
{
	std::vector<float> ret_;
	ret_.clear();

	point_t pt_;
	pt_ = {(float)q_new_.point.x, (float)q_new_.point.y, (float)q_new_.point.z, q_new_.rot_uav.x, q_new_.rot_uav.y, q_new_.rot_uav.z, q_new_.rot_uav.w};
		
	KDTree treeuav(v_uav_nodes_kdtree);
	point_t nearest_ = treeuav.nearest_point(pt_,3);

    float x_uav_to_ugv_ = nearest_[0]; 
    float y_uav_to_ugv_ = nearest_[1];
    float z_uav_to_ugv_ = nearest_[2];
    float x_rot_uav_ = nearest_[3];
    float y_rot_uav_ = nearest_[4];
    float z_rot_uav_ = nearest_[5];
    float w_rot_uav_ = nearest_[6];
    ret_.push_back(x_uav_to_ugv_);
    ret_.push_back(y_uav_to_ugv_);
    ret_.push_back(z_uav_to_ugv_);
    ret_.push_back(x_rot_uav_);
    ret_.push_back(y_rot_uav_);
    ret_.push_back(z_rot_uav_);
    ret_.push_back(w_rot_uav_);

	return ret_;
}

void RandomPlanner::getOrientation(RRTNode &n_ , RRTNode p_, bool is_uav_)
{
	float yaw_;
	tf::Quaternion _quat;

	if (!is_uav_){
		yaw_ = atan2(n_.point.y - p_.point.y, n_.point.x - p_.point.x);
		_quat.setRPY(0.0, 0.0, yaw_);
		n_.rot_ugv.x = _quat.x();
		n_.rot_ugv.y = _quat.y();
		n_.rot_ugv.z = _quat.z();
		n_.rot_ugv.w = _quat.w();
	}
	else {
		yaw_ = atan2(n_.point_uav.y - p_.point_uav.y, n_.point_uav.x - p_.point_uav.x);
		_quat.setRPY(0.0, 0.0, yaw_);
		n_.rot_uav.x = _quat.x();
		n_.rot_uav.y = _quat.y();
		n_.rot_uav.z = _quat.z();
		n_.rot_uav.w = _quat.w();

	}
}

float RandomPlanner::getYawFromQuaternion(RRTNode n_, bool is_uav_)
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

double RandomPlanner::costNode(const RRTNode q_new_)
{
	double cost_, k0_, k1_, k2_, k3_, F0_, F1_, F2_, F3_;
	k0_ = 10.0;	// For ugv : 20.0 
	k1_ = 6.0;	// For uav: 5.0
	k2_ = 0.0;	// Diff length Cat
	k3_ = 1.0;

	double p_ugv_x_ = q_new_.point.x * step - q_new_.parentNode->point.x * step;
	double p_ugv_y_ = q_new_.point.y * step - q_new_.parentNode->point.y * step;
	double p_ugv_z_ = q_new_.point.z * step - q_new_.parentNode->point.z * step;
	double p_uav_x_ = q_new_.point_uav.x * step - q_new_.parentNode->point_uav.x * step;
	double p_uav_y_ = q_new_.point_uav.y * step - q_new_.parentNode->point_uav.y * step;
	double p_uav_z_ = q_new_.point_uav.z * step - q_new_.parentNode->point_uav.z * step;

	if(is_coupled){
		F0_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
		F1_ =  sqrt((p_uav_x_*p_uav_x_) + (p_uav_y_*p_uav_y_) + (p_uav_z_*p_uav_z_));
								
		cost_ = k0_ * F0_ + k1_ * F1_ + q_new_.parentNode->cost;
	}
	else{
		F0_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
		F1_ =  sqrt((p_uav_x_*p_uav_x_) + (p_uav_y_*p_uav_y_) + (p_uav_z_*p_uav_z_));
		F2_ = q_new_.length_cat - q_new_.parentNode->length_cat;
				
		cost_ = k0_ * F0_ +  k1_ * F1_ + k2_ * F2_ + k3_ * q_new_.parentNode->cost;
	}

	return cost_;
}

double RandomPlanner::costBetweenNodes(const RRTNode q_near_, const RRTNode q_new_)
{
	double cost_;
	double r_security_cat_ = 0.1;
	double k0_, k1_, k2_;
	double dist_ugv_, dist_uav_, cos_angle;
	double p_ugv_x_, p_ugv_y_, p_ugv_z_, p_uav_x_, p_uav_y_, p_uav_z_;
	k0_ = 10.0;	// For ugv
	k1_ = 10.0;	// For uav
	k2_ = 5.0;	// For ugv smoothness
	
	p_ugv_x_ = q_new_.point.x * step - q_near_.point.x * step;
	p_ugv_y_ = q_new_.point.y * step - q_near_.point.y * step;
	p_ugv_z_ = q_new_.point.z * step - q_near_.point.z * step;
	
	if(is_coupled){
		dist_ugv_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
		cost_ = k1_ * dist_ugv_ ;
	}
	else{
		p_uav_x_ = q_new_.point_uav.x * step - q_near_.point_uav.x * step;
		p_uav_y_ = q_new_.point_uav.y * step - q_near_.point_uav.y * step;
		p_uav_z_ = q_new_.point_uav.z * step - q_near_.point_uav.z * step;
		if (q_new_.id != id_ugv_init  && q_new_.id_uav != id_uav_init ){ //This condition to not compute in the first point	
			//To get smoothness
			double p_ugv_parent_x_ = q_new_.parentNode->point.x * step - q_new_.point.x * step;
			double p_ugv_parent_y_ = q_new_.parentNode->point.y * step - q_new_.point.y * step;
			double p_ugv_parent_z_ = q_new_.parentNode->point.z * step - q_new_.point.z * step;
			//Compute dot product and norm of vectors
			double dot_product = (p_ugv_parent_x_*p_ugv_x_) + (p_ugv_parent_y_*p_ugv_y_) + (p_ugv_parent_z_*p_ugv_z_);
			double norm_vector1 = sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
			double norm_vector2 = sqrt((p_ugv_parent_x_*p_ugv_parent_x_) + (p_ugv_parent_y_*p_ugv_parent_y_) + (p_ugv_parent_z_*p_ugv_parent_z_));
			if (norm_vector1 < 0.0001 || norm_vector2 < 0.0001 )
				cos_angle = 1.0;
			else
			cos_angle = dot_product/(norm_vector1 * norm_vector2);
		}else
			cos_angle = 1.0;

		dist_ugv_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_x_*p_ugv_x_) + (p_ugv_z_*p_ugv_z_));
		dist_uav_ =  sqrt((p_uav_x_*p_uav_x_) + (p_uav_x_*p_uav_x_) + (p_uav_z_*p_uav_z_));
		cost_ = k0_ * dist_ugv_ + k1_ * dist_uav_ + k2_* (1.0 - cos_angle );
	}
	return cost_;
}

void RandomPlanner::getParamsNode(RRTNode &node_, bool is_init_)
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

	geometry_msgs::Point p_node_uav_;
	p_node_uav_.x = node_.point_uav.x * step;
	p_node_uav_.y = node_.point_uav.y * step;
	p_node_uav_.z = node_.point_uav.z * step;
	double dist_obs_uav = ccm->getPointDistanceFullMap(use_distance_function, p_node_uav_);
	// Eigen::Vector3d obs_near_uav_;
	// point_node_.x() = node_.point_uav.x * step;
	// point_node_.y() = node_.point_uav.y * step;
	// point_node_.z() = node_.point_uav.z * step;
	// bool is_into_ = grid_3D->isIntoMap(point_node_.x(),point_node_.y(),point_node_.z());
	// double dist_obs_uav;
	// if (is_into_)
	// 	dist_obs_uav = grid_3D->getPointDist((double)point_node_.x(),(double)point_node_.y(),(double)point_node_.z());

	node_.id = getWorldIndex(node_.point.x, node_.point.y, node_.point.z);
	node_.min_dist_obs_ugv = dist_obs_ugv;
	node_.min_dist_obs_uav = dist_obs_uav;

	if(is_coupled){
		node_.id_uav = -1.0;
		if (node_.catenary)
			node_.cost_takeoff = Cat1_* node_.length_cat + Cat2_ * exp(r_security_cat_ - 1.0*node_.min_dist_obs_cat);
		else
			node_.cost_takeoff = -1.0;
	}
	else{
		node_.id_uav = getWorldIndex(node_.point_uav.x, node_.point_uav.y, node_.point_uav.z);
		node_.cost_takeoff = -1.0;
		if (is_init_){
			checkCatenary(node_, points_catenary_new_node);  //Not necessary because in steering method is check
			id_ugv_init = node_.id;
			id_uav_init = node_.id_uav;
		}
	}
	if (!is_init_)
		node_.cost = costNode(node_);
}

void RandomPlanner::updateParamsNode(RRTNode &node_)
{
	node_.cost = costNode(node_);
}

// bool RandomPlanner::checkUGVFeasibility(const RRTNode pf_, bool ugv_above_z_)
// {
// 	bool ret;
// 	if(ugv_above_z_){
// 		if (isUGVInside(pf_.point.x,pf_.point.y,pf_.point.z)){
// 			Eigen::Vector3d obs_to_ugv, p_ugv;
// 			p_ugv.x() =pf_.point.x * step ;
// 			p_ugv.y() =pf_.point.y * step ; 
// 			p_ugv.z() =pf_.point.z * step ; 
// 			obs_to_ugv = nn_obs_ugv.nearestObstacleMarsupial(nn_obs_ugv.kdtree, p_ugv, nn_obs_ugv.obs_points);
// 			double d_ = sqrt(pow(obs_to_ugv.x()-p_ugv.x(),2) + pow(obs_to_ugv.y()-p_ugv.y(),2) + pow(obs_to_ugv.z()-p_ugv.z(),2));
// 			if (d_ > )
// 			if (obs_to_ugv.z() < p_ugv.z())
// 				ret = false;	
// 			if (isUGVOccupied(pf_))
// 				ret = true;		
// 			else
// 				ret = false;
// 		}
// 		else
// 			ret = false;
// 	}	
// 	else{
// 		if (isUGVOccupied(pf_))
// 			ret = false;		
// 		else
// 			ret = true;
// 	}
// 	return ret;	
// }

bool RandomPlanner::checkNodeFeasibility(const RRTNode pf_ , bool check_uav_)
{
	bool ret;
	double d_;

	if(check_uav_==false){
			Eigen::Vector3d obs_to_ugv, pos_ugv;
			pos_ugv.x() =pf_.point.x * step ;
			pos_ugv.y() =pf_.point.y * step ; 
			pos_ugv.z() =pf_.point.z * step ; 
			obs_to_ugv = nn_obs_ugv.nearestObstacleVertex(nn_obs_ugv.kdtree, pos_ugv, nn_obs_ugv.obs_points);
			// printf("kdtreeUGV :  new[%f %f %f]  nearest[%f %f %f]\n",pos_ugv.x(),pos_ugv.y(),pos_ugv.z(),obs_to_ugv.x(),obs_to_ugv.y(),obs_to_ugv.z());
			double d_ = sqrt(pow(obs_to_ugv.x()-pos_ugv.x(),2) + pow(obs_to_ugv.y()-pos_ugv.y(),2) + pow(obs_to_ugv.z()-pos_ugv.z(),2));
			// printf("kdtreeUGV= %f , distance_obstacle_ugv= %f\n",d_, distance_obstacle_ugv);
			if (d_ > distance_obstacle_ugv)
				ret = true;
			else{
				if (obs_to_ugv.z() > pos_ugv.z())
					ret = false;	
				else	
					ret = true;	
			}
		return ret;
	}
	else{
		// printf("isInside = %s\n",isInside(pf_.point_uav.x,pf_.point_uav.y,pf_.point_uav.z)? "true" : "false");
		// printf("pf_ = [%i %i %i]\n",pf_.point_uav.x,pf_.point_uav.y,pf_.point_uav.z);
		// printf("step[%f] pf_ : [%f %f %f]\n", step, pf_.point_uav.x*step, pf_.point_uav.y*step, pf_.point_uav.z*step);
		if (isInside(pf_.point_uav.x,pf_.point_uav.y,pf_.point_uav.z)){
			// Eigen::Vector3d obs_to_uav, pos_uav;
			geometry_msgs::Point obs_to_uav, pos_uav; 
			pos_uav.x =pf_.point_uav.x * step ;
			pos_uav.y =pf_.point_uav.y * step ; 
			pos_uav.z =pf_.point_uav.z * step ; 
			d_ = ccm->getPointDistanceFullMap(use_distance_function, pos_uav);
			// printf("getPointDistanceFullMap= %f , distance_obstacle_uav= %f\n",d_, distance_obstacle_uav);
			if (d_ == -1.0)
				return false;

			// bool is_into_ = grid_3D->isIntoMap(pos_uav.x,pos_uav.y,pos_uav.z);
			// if (is_into_)
			// 	d_ = grid_3D->getPointDist((double)pos_uav.x,(double)pos_uav.y,(double)pos_uav.z);
			// else
			// 	return false;
			
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

bool RandomPlanner::checkPointsCatenaryFeasibility(const RRTNode pf_)
{
	bool ret;
	double d_;

	geometry_msgs::Point obs_to_uav, pos_uav; 
	pos_uav.x =pf_.point.x * step ;
	pos_uav.y =pf_.point.y * step ; 
	pos_uav.z =pf_.point.z * step ; 
	d_ = ccm->getPointDistanceFullMap(use_distance_function, pos_uav);
	if (d_ == -1.0){
		// printf("d_=%f , point=[%f %f %f]\n",d_,pos_uav.x,pos_uav.y,pos_uav.z);
		return false;
	}
	if (d_ > distance_catenary_obstacle)
		ret = true;
	else{
		// printf("d_=%f < distance_catenary_obstacle , point=[%f %f %f]\n",d_,pos_uav.x,pos_uav.y,pos_uav.z);
		ret = false;
	}
	return ret;
}

bool RandomPlanner::checkCatenary(RRTNode &q_init_, vector<geometry_msgs::Point> &points_catenary_)
{
	// mode 1: UGV-Goal  ,  mode 2: UGV-UAV
	geometry_msgs::Point p_reel_, p_final_;
	
	p_reel_ = getReelNode(q_init_);	

	p_final_.x = q_init_.point_uav.x * step ;	
	p_final_.y = q_init_.point_uav.y * step ;   
	p_final_.z = q_init_.point_uav.z * step ;   

	bool found_catenary = ccm->searchCatenary(p_reel_, p_final_, points_catenary_);
	if(found_catenary){
		// printf("\t RandomPlanner::checkCatenary: points_catenary_=%lu\n",points_catenary_.size());
		q_init_.p_cat.resize(points_catenary_. size());
		int i = 0;
		for (auto &x:points_catenary_) {
			q_init_.p_cat[i++] = x;
		}
		q_init_.min_dist_obs_cat = ccm->min_dist_obs_cat;
		q_init_.length_cat = ccm->length_cat_final;
		q_init_.catenary = found_catenary;
	}
	return found_catenary;
}

geometry_msgs::Point RandomPlanner::getReelNode(const RRTNode node_)
{
	geometry_msgs::Point pos_reel;
	float yaw_ugv;

	yaw_ugv = getYawFromQuaternion(node_,false);
	double lengt_vec =  sqrt(pos_reel_ugv.x*pos_reel_ugv.x + pos_reel_ugv.y*pos_reel_ugv.y);
	pos_reel.x = node_.point.x*step + lengt_vec *cos(yaw_ugv); 
	pos_reel.y = node_.point.y*step + lengt_vec *sin(yaw_ugv);
	pos_reel.z = node_.point.z*step + pos_reel_ugv.z ;

	return pos_reel;
}

void RandomPlanner::updateKdtreeNode(const RRTNode ukT_)
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

void RandomPlanner::updateKdtreeUGV(const RRTNode ukT_)
{
	point_t pt_;

	pt_ = {(float)ukT_.point.x, (float)ukT_.point.y, (float)ukT_.point.z, ukT_.rot_ugv.x, ukT_.rot_ugv.y, ukT_.rot_ugv.z, ukT_.rot_ugv.w};
	v_ugv_nodes_kdtree.push_back(pt_);
}

void RandomPlanner::updateKdtreeUAV(const RRTNode ukT_)
{
	point_t pt_;

	pt_ = {(float)ukT_.point_uav.x, (float)ukT_.point_uav.y, (float)ukT_.point_uav.z,ukT_.rot_uav.x, ukT_.rot_uav.y, ukT_.rot_uav.z, ukT_.rot_uav.w};
	v_uav_nodes_kdtree.push_back(pt_);
}

void RandomPlanner::readPointCloudTraversabilityMapUGV(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_trav_ugv.setInput(*msg);
	ROS_INFO_COND(debug_rrt, PRINTF_BLUE "RandomPlanner Planner: Receiving point cloud map to create Kdtree for Traversability UGV");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg,*cloud_in);

	ROS_INFO(PRINTF_BLUE"RandomPlanner::readPointCloudTraversabilityMapUGV  size point cloud = [%lu]",cloud_in->size());
	geometry_msgs::Point point_;
	for (size_t i = 0 ; i < cloud_in->size() ; i ++){
		point_.x = cloud_in->points[i].x;
		point_.y = cloud_in->points[i].y;
		point_.z = cloud_in->points[i].z;
		v_points_ws_ugv.push_back(point_);
	}
	ROS_INFO(PRINTF_BLUE"RandomPlanner::readPointCloudTraversabilityMapUGV  size v_points_ws_ugv = [%lu]",v_points_ws_ugv.size());
}

void RandomPlanner::readPointCloudMapForUGV(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_obs_ugv.setInput(*msg);
	ROS_INFO_COND(debug_rrt, PRINTF_BLUE "RandomPlanner Planner: Receiving point cloud map to create Kdtree for UGV Obstacles");
}

void RandomPlanner::readPointCloudMapForUAV(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_obs_uav.setInput(*msg);
	ROS_INFO_COND(debug_rrt, PRINTF_BLUE "RandomPlanner Planner: Receiving point cloud map to create Kdtree for UAV Obstacles");
}

bool RandomPlanner::saveNode(RRTNode* sn_, bool is_init_)
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

inline void RandomPlanner::saveTakeOffNode(RRTNode* ston_)
{
	take_off_nodes.push_back(ston_); 
}

inline void RandomPlanner::clearStatus()
{
	if (nodes_marker_debug)
		rrtgm.clearMarkersNodesTree(tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_, take_off_nodes_pub_);
	rrtgm.clearMarkers(lines_ugv_marker_pub_, lines_uav_marker_pub_);
  	
	nodes_tree.clear();
  	take_off_nodes.clear();
	rrt_path.clear();

  	got_to_goal = 0;
	num_goal_finded = 0;
	length_catenary.clear();
	v_nodes_kdtree.clear();
	v_ugv_nodes_kdtree.clear();
	v_uav_nodes_kdtree.clear();
	points_catenary_new_node.clear();
}

void RandomPlanner::clearNodesMarker()
{ 
	rrtgm.clearMarkersNodesTree(tree_rrt_star_ugv_pub_, tree_rrt_star_uav_pub_, take_off_nodes_pub_); 
}

void RandomPlanner::clearCatenaryGPMarker()
{ 
	rrtgm.clearCatenaryMarker(catenary_marker_pub_); 
}

void RandomPlanner::clearLinesGPMarker()
{
	rrtgm.clearMarkers(lines_ugv_marker_pub_, lines_uav_marker_pub_);
}

std::list<RRTNode*> RandomPlanner::getPath()
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
		// std:cout <<"node: " << current_node <<" , Parent: " << current_node->parentNode<< std::endl;
		}
	}
	
	return path_;
}

void RandomPlanner::isGoal(const RRTNode st_) 
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

bool RandomPlanner::getGlobalPath(Trajectory &trajectory)
{
	trajectory_msgs::MultiDOFJointTrajectoryPoint traj_marsupial_;

	traj_marsupial_.transforms.resize(2);
	traj_marsupial_.velocities.resize(2);
	traj_marsupial_.accelerations.resize(2);

	for(auto nt_ : rrt_path){
		traj_marsupial_.transforms[0].translation.x = nt_->point.x*step;
		traj_marsupial_.transforms[0].translation.y = nt_->point.y*step;
		if (nt_->point.z*step == 0.0)
			traj_marsupial_.transforms[0].translation.z = nt_->point.z*step + step/2.0;
		else
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
		traj_marsupial_.transforms[1].translation.x = nt_->point_uav.x * step;
		traj_marsupial_.transforms[1].translation.y = nt_->point_uav.y * step;
		traj_marsupial_.transforms[1].translation.z = nt_->point_uav.z * step;
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

void RandomPlanner::configRRTParameters(double _l_m, geometry_msgs::Vector3 _p_reel ,
                                        geometry_msgs::Vector3 _p_ugv, geometry_msgs::Quaternion _r_ugv,
                                        bool coupled_, int n_iter_ , int n_loop_, double r_nn_, double s_s_,
                                        int s_g_r_, int sample_m_, double min_l_steer_ugv_,
                                        double w_n_ugv_, double w_n_uav_, double w_n_smooth_)
{
	length_tether_max = _l_m;

	geometry_msgs::Point pos_reel;

	pos_reel_ugv = _p_reel;

	pos_reel.x = _p_reel.x;
	pos_reel.y = _p_reel.y;
	pos_reel.z = _p_reel.z;

	pos_tf_ugv = _p_ugv;
	rot_tf_ugv = _r_ugv;
	
	is_coupled = coupled_;
	n_iter = n_iter_;
	n_loop = n_loop_;
	radius_near_nodes = r_nn_;
	step_steer = s_s_;
	samp_goal_rate = s_g_r_;

	sample_mode = sample_m_; 

	min_dist_for_steer_ugv = min_l_steer_ugv_;
	w_nearest_ugv = w_n_ugv_ ;
	w_nearest_uav = w_n_uav_ ;
	w_nearest_smooth = w_n_smooth_ ;

	rrtgm.configGraphMarkers(frame_id, step, is_coupled, n_iter, pos_reel_ugv);

    ccm->init(grid_3D, distance_catenary_obstacle, distance_obstacle_ugv, distance_obstacle_uav,
	          length_tether_max, ws_z_min, step, use_parabola, use_distance_function, pos_reel,
			  false, !use_parabola);
}

bool RandomPlanner::setInitialPositionCoupled(RRTNode n_)
{
	if (isUGVInside(n_.point.x, n_.point.y, n_.point.z))
	{
		RRTNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(n_.point.x, n_.point.y, n_.point.z)];

		if (initialNodeInWorld->node == NULL)
		{
			initialNodeInWorld->node = new RRTNode();
			initialNodeInWorld->node->point.x = n_.point.x;
			initialNodeInWorld->node->point.y = n_.point.y;
			initialNodeInWorld->node->point.z = n_.point.z;

			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
		}
		disc_initial = initialNodeInWorld->node;

		initial_position_ugv.x = n_.point.x * step;
		initial_position_ugv.y = n_.point.y * step;
		initial_position_ugv.z = n_.point.z * step;

		disc_initial->point = n_.point;
		disc_initial->point_uav = n_.point_uav;
		disc_initial->rot_ugv = n_.rot_ugv;
		disc_initial->rot_uav = n_.rot_uav;
		disc_initial->parentNode = NULL;
		disc_initial->cost = 0.0;
		disc_initial->length_cat = -1.0;
		disc_initial->min_dist_obs_cat = -1.0;
		disc_initial->min_dist_obs_ugv = -1.0;

		return true;
	}
	else
	{
		disc_initial = NULL;
		return false;
	}
}

bool RandomPlanner::setInitialPositionIndependent(RRTNode n_)
{
	// printf("isUGVInside = %s\n",isUGVInside(n_.point.x, n_.point.y, n_.point.z)? "true" : "false");
	// printf("isInside = %s\n",isInside(n_.point_uav.x, n_.point_uav.y, n_.point_uav.z)? "true" : "false");
	
	if (isUGVInside(n_.point.x, n_.point.y, n_.point.z) && isInside(n_.point_uav.x, n_.point_uav.y, n_.point_uav.z)){
		RRTNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(n_.point.x, n_.point.y, n_.point.z)];

		if (initialNodeInWorld->node == NULL){
			initialNodeInWorld->node = new RRTNode();
			initialNodeInWorld->node->point.x = n_.point.x;
			initialNodeInWorld->node->point.y = n_.point.y;
			initialNodeInWorld->node->point.z = n_.point.z;
			initialNodeInWorld->node->rot_ugv.x = n_.rot_ugv.x;
			initialNodeInWorld->node->rot_ugv.y = n_.rot_ugv.y;
			initialNodeInWorld->node->rot_ugv.z = n_.rot_ugv.z;
			initialNodeInWorld->node->rot_ugv.w = n_.rot_ugv.w;

			initialNodeInWorld->node->point_uav.x = n_.point_uav.x;
			initialNodeInWorld->node->point_uav.y = n_.point_uav.y;
			initialNodeInWorld->node->point_uav.z = n_.point_uav.z;
			initialNodeInWorld->node->rot_uav.x = n_.rot_uav.x;
			initialNodeInWorld->node->rot_uav.y = n_.rot_uav.y;
			initialNodeInWorld->node->rot_uav.z = n_.rot_uav.z;
			initialNodeInWorld->node->rot_uav.w = n_.rot_uav.w;
			
			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
		}
		disc_initial = initialNodeInWorld->node;

		initial_position_ugv.x = n_.point.x * step;
		initial_position_ugv.y = n_.point.y * step;
		initial_position_ugv.z = n_.point.z * step;

		initial_position_uav.x = n_.point_uav.x * step;
		initial_position_uav.y = n_.point_uav.y * step;
		initial_position_uav.z = n_.point_uav.z * step;

		disc_initial->point = n_.point;
		disc_initial->point_uav = n_.point_uav;
		disc_initial->rot_ugv = n_.rot_ugv;
		disc_initial->rot_uav = n_.rot_uav;
		disc_initial->parentNode = NULL;
		disc_initial->cost = 0.0;
		disc_initial->length_cat = -1.0;
		disc_initial->min_dist_obs_cat = -1.0;
		disc_initial->min_dist_obs_ugv = -1.0;
		ROS_INFO(PRINTF_BLUE "RandomPlanner::setInitialPositionIndependent -->  disc_initial  UGV:[%f %f %f /%f %f %f %f]  UAV:[%f %f %f /%f %f %f %f]",
							disc_initial->point.x*step,disc_initial->point.y*step,disc_initial->point.z*step,
							disc_initial->rot_ugv.x, disc_initial->rot_ugv.y, disc_initial->rot_ugv.z, disc_initial->rot_ugv.w,
							disc_initial->point_uav.x*step, disc_initial->point_uav.y*step, disc_initial->point_uav.z*step,
							disc_initial->rot_uav.x, disc_initial->rot_uav.y, disc_initial->rot_uav.z, disc_initial->rot_uav.w);
		return true;
	}
	else{
		disc_initial = NULL;
		// ROS_ERROR("First position not posible to set");
		return false;
	}
}

bool RandomPlanner::setFinalPosition(DiscretePosition p_)
{
	if (isInside(p_.x, p_.y, p_.z)){
		RRTNodeLink3D *finalNodeInWorld = &discrete_world[getWorldIndex(p_.x, p_.y, p_.z)];

		Eigen::Vector3d p_node_, trav_point_ugv_;
		p_node_.x() = p_.x * step;
		p_node_.y() = p_.y * step;
		p_node_.z() = p_.z * step;
		trav_point_ugv_ = nn_trav_ugv.nearestObstacleMarsupial(nn_trav_ugv.kdtree, p_node_, nn_trav_ugv.obs_points);
		
		if (finalNodeInWorld->node == NULL){
			finalNodeInWorld->node = new RRTNode();
			finalNodeInWorld->node->point.x = p_.x;
			finalNodeInWorld->node->point.x = p_.y;
			finalNodeInWorld->node->point.x = p_.z;
			finalNodeInWorld->node->point_uav.x = trav_point_ugv_.x()*step_inv;
			finalNodeInWorld->node->point_uav.x = trav_point_ugv_.y()*step_inv;
			finalNodeInWorld->node->point_uav.x = trav_point_ugv_.z()*step_inv;

			finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
		}
		disc_final = finalNodeInWorld->node;
		
		disc_final->point_uav = p_;
		disc_final->point.x = trav_point_ugv_.x()*step_inv;
		disc_final->point.y = trav_point_ugv_.y()*step_inv;
		disc_final->point.z = trav_point_ugv_.z()*step_inv;
		final_position.x = disc_final->point_uav.x*step;
		final_position.y = disc_final->point_uav.y*step;
		final_position.z = disc_final->point_uav.z*step;

		ROS_INFO(PRINTF_BLUE "RandomPlanner::setFinalPosition -->  disc_final [%f %f %f /%f %f %f]",disc_final->point.x*step,disc_final->point.y*step,disc_final->point.z*step,
												disc_final->point_uav.x*step,disc_final->point_uav.y*step,disc_final->point_uav.z*step);
		
		// if (planner_type == "birrt"){
		// 	octomap::OcTree map_goal_pos_feasible = checkTraversablePointInsideCircle(final_position);
		// 	octomap_msgs::Octomap octomap_reduced;
		// 	octomap_reduced.binary = 1 ;
		// 	octomap_reduced.id = 1 ;
		// 	octomap_reduced.resolution =0.2 ;
		// 	octomap_reduced.header.frame_id = "/map";
		// 	octomap_reduced.header.stamp = ros::Time::now();
		// 	octomap_msgs::fullMapToMsg(map_goal_pos_feasible, octomap_reduced);
		// 	reducedMapPublisher.publish(octomap_reduced);
		// }
		return true;
	}
	else{
		disc_final = NULL;
		return false;
	}
}

inline void RandomPlanner::setInitialCostGoal(RRTNode* p_)
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

bool RandomPlanner::isInitialPositionUGVOccupied()
{
	return isUGVOccupied(*disc_initial);
}

bool RandomPlanner::isInitialPositionUAVOccupied()
{
	return isOccupied(*disc_initial, true);
}

bool RandomPlanner::isFinalPositionOccupied()
{
	return isOccupied(*disc_final,true);
}

bool RandomPlanner::isOccupied(RRTNode n_, bool check_uav_)
{
	if(check_uav_==false){
		return !discrete_world[getWorldIndex(n_.point.x, n_.point.y, n_.point.z)].notOccupied;
	}
	else
		return !discrete_world[getWorldIndex(n_.point_uav.x, n_.point_uav.y, n_.point_uav.z)].notOccupied;
}

bool RandomPlanner::isUGVOccupied(RRTNode n_)
{
	RRTNode n_z_displace_;
	n_z_displace_.point.x = n_.point.x;
	n_z_displace_.point.y = n_.point.y;
	n_z_displace_.point.z = n_.point.z + (v_inflation + step_inv);

	return !discrete_world[getWorldIndex(n_z_displace_.point.x, n_z_displace_.point.y, n_z_displace_.point.z)].notOccupied;
}

void RandomPlanner::updateMap(octomap_msgs::OctomapConstPtr message)
{
	// Clear current map in the discrete occupancy
	clearMap();

	// Read occupation data from the octomap_server
	map = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*message);
	
	/* Update discrete world with the read octomap data*/
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
				if (isInside(x_, y_, z_))
				{
					++nit;
					unsigned int world_index_ = getWorldIndex(x_, y_, z_);
					discrete_world[world_index_].notOccupied = false;
					// Inflates nodes
					if (h_inflation >= step || v_inflation >= step)
					{
						inflateNodeAsXyRectangle(x_, y_, z_);
					}
				}
			}
		}
	}
}

void RandomPlanner::clearMap()
{
	for (int i = 0; i < matrix_size; i++){
		discrete_world[i].notOccupied = true;
	}
}

DiscretePosition RandomPlanner::discretizePosition(Vector3 p)
{
	DiscretePosition res;

	res.x = p.x * step_inv;
	res.y = p.y * step_inv;
	res.z = p.z * step_inv;

	return res;
}

octomap::OcTree RandomPlanner::checkTraversablePointInsideCircle(Vector3 point_)
{
	double center_x , center_y, radius;
	radius = length_tether_max*length_tether_max + point_.z*point_.z ;
	center_x = point_.x;
	center_y = point_.y;

	octomap::point3d p_;
	octomap::OcTree map_circle (0.05f);

	for (size_t i = 0 ; i < v_points_ws_ugv.size() ; i ++){
		p_.x() = v_points_ws_ugv[i].x;
		p_.y() = v_points_ws_ugv[i].y;
		p_.z() = v_points_ws_ugv[i].z;
		if( (pow(v_points_ws_ugv[i].x-center_x,2) + pow(v_points_ws_ugv[i].y-center_y,2)) <= radius*radius)
			map_circle.updateNode(p_, true); // integrate 'occupied' measurement
		else
			map_circle.updateNode(p_, false);  // integrate 'free' measurement
	}
	return map_circle;
}

// double RandomPlanner::getPointDistanceFullMap(bool use_dist_func_, geometry_msgs::Vector3 p_)
// {
// 	double dist;
// 	Eigen::Vector3d obs_, pos_;

// 	if(use_dist_func_){
// 		bool is_into_ = grid_3D->isIntoMap(p_.x,p_.y,p_.z);
// 		if(is_into_)
// 			dist =  grid_3D->getPointDist((double)p_.x,(double)p_.y,(double)p_.z) ;
// 		else
// 			dist = -1.0;
// 	}
// 	else{
// 		pos_.x() = p_.x;
// 		pos_.y() = p_.y; 
// 		pos_.z() = p_.z; 
// 		obs_ = nn_obs_uav.nearestObstacleMarsupial(nn_obs_uav.kdtree, pos_, nn_obs_uav.obs_points);
// 		dist = sqrt(pow(obs_.x()-pos_.x(),2) + pow(obs_.y()-pos_.y(),2) + pow(obs_.z()-pos_.z(),2));
// 	}
// 	return dist;
// }

} // end class 
