#include "rrt_planners/random_uav_planner.hpp"

namespace PathPlanners
{
//*****************************************************************
// 		 Random Algorithm Class Definitions (RRT* for UAV + tether) 
//*****************************************************************

// Default constructor
RandomUAVPlanner::RandomUAVPlanner()
{
	ccm = NULL;
}

RandomUAVPlanner::~RandomUAVPlanner()
{
	grid_3D->~Grid3d();
	discrete_world.clear();
	nodes_tree.clear();
	rrt_path.clear();
	length_catenary.clear();
	v_nodes_kdtree.clear();
	v_uav_nodes_kdtree.clear();
}

// Initialization: creates the occupancy matrix (discrete nodes) from the bounding box sizes, resolution, inflation and optimization arguments
  void RandomUAVPlanner::init(std::string plannerName, std::string frame_id_,
                              float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_,
                              float ws_y_min_, float ws_z_min_, float step_, float h_inflation_, float v_inflation_,
                              ros::NodeHandlePtr nh_, double goal_gap_m_, bool debug_rrt_, double distance_obstacle_uav_,
                              double distance_catenary_obstacle_, Grid3d *grid3D_, bool nodes_marker_debug_,
                              bool use_distance_function_, std::string map_file_, bool get_catenary_data_,
                              std::string catenary_file_, bool use_parabola_, CatenaryCheckerManager *catenary_manager)
{
	// Pointer to the nodeHandler
	nh = nh_;
	grid_3D = grid3D_;
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
	matrix_size = (abs(ws_x_max_inflated) - ws_x_min_inflated + 1) *
    (abs(ws_y_max_inflated) - ws_y_min_inflated + 1) *
    (abs(ws_z_max_inflated) - ws_z_min_inflated + 1);
	discrete_world.resize(matrix_size);
	printf("Global Planner %s Node : Occupancy Matrix has %d nodes [%lu MB]\n", plannerName.c_str(), matrix_size, (uint_fast32_t)(matrix_size * sizeof(RRTNode)) / (1024 * 1024));
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
	printf("Global Planner %s Node : Using distance function: %s \n",
         plannerName.c_str(), use_distance_function?"true":"false");

	markers_debug = false;
	nodes_marker_debug = nodes_marker_debug_;

	map_file = map_file_;

	distance_obstacle_uav = distance_obstacle_uav_;
	distance_tether_obstacle = distance_catenary_obstacle_;
	just_line_of_sigth = jlos_;

	get_catenary_data = get_catenary_data_;
	catenary_file = catenary_file_;
	use_parabola = use_parabola_;

  	nh->param("n_intermediate", n_intermediate, 6);

	lines_uav_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("path_uav_rrt_star", 2, true);
	goal_point_pub_ = nh->advertise<visualization_msgs::Marker>("goal_point", 1, true);
	catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marsupial", 1000, true);
	rand_point_pub_ = nh->advertise<visualization_msgs::MarkerArray>("rand_point", 2, true);
	new_point_pub_ = nh->advertise<visualization_msgs::MarkerArray>("new_point", 2, true);
	nearest_point_pub_ = nh->advertise<visualization_msgs::MarkerArray>("nearest_point", 2, true);
	points_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("points_marker", 10, true);
	nearest_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("new_catenaty", 1000, true);
	new_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("nearest_catenaty", 1000, true);
	one_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("one_catenary", 1000, true);
	
	if (nodes_marker_debug){
		tree_rrt_star_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("tree_rrt_star_uav", 2, true);
	}
	if (markers_debug){
		reel1_point_pub_ = nh->advertise<visualization_msgs::Marker>("reel1_point", 1, true);
		reel2_point_pub_ = nh->advertise<visualization_msgs::Marker>("reel2_point", 1, true);
		all_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("all_catenaries_rrt", 10000, true);
	}
  ccm = catenary_manager;
}

int RandomUAVPlanner::computeTree()
{
 	double ret_val = -1; 
	
	clearStatus();
	if(nodes_marker_debug)
		rrtgm.clearUAVNodesTree(tree_rrt_star_uav_pub_);
	rrtgm.clearMarkers(lines_uav_marker_pub_);

  //	if(get_catenary_data)
  struct timespec start_rrt, finish_rrt;
  clock_gettime(CLOCK_REALTIME, &start_rrt);

	ROS_INFO("RandomUAVPlanner::computeTree -->  start_point_ugv[%.2f %.2f %.2f/%.2f %.2f %.2f]  goal_point=[%.2f %.2f %.2f] \n\n", 
         initial_position_ugv.x, initial_position_ugv.y, initial_position_ugv.z,
         initial_position_uav.x, initial_position_uav.y, initial_position_uav.z,
         final_position.x, final_position.y, final_position.z);  
    
	v_min_dist_obs_cat.clear(); v_length_cat.clear(); v_time_cat.clear();

	setInitialCostGoal(disc_final);

  if (!saveNode(disc_initial,true)) {
    std::cout << "RandomUAVPlanner::computeTree --> Initial node not feasible." << std::endl;
    return -1;
  } else {
    std::cout << "RandomUAVPlanner::computeTree --> Initial node saved." << std::endl;
  }
  rrtgm.getCatenaryMarker(points_catenary_new_node, one_catenary_marker_pub_);
	rrtgm.goalPointMarker(final_position, goal_point_pub_);
	
	count_graph = 0;

  updateKdtreeNode(*disc_initial);
  updateKdtreeUAV(*disc_initial);

  count_loop = 0; 
	int count_total_loop = 0; 
	count_qnew_fail = count_fail_connect_goal = -1;

	while (count_loop < n_iter) { // n_iter Max. number of nodes to expand for each round

    if (debug_rrt) {
      printf("\t\t  Planner::computeTree: iter=[%i/%i], loop=[%i/%i], nodes[%lu/%i]\r",
             count_loop+1, n_iter, count_total_loop+1, n_loop,
             nodes_tree.size(), (count_loop+1)+(500*count_total_loop));
    }
		
		RRTNode q_rand;

    if ((count_loop%samp_goal_rate)!=0)
      q_rand = getRandomNode();	
    else {
      q_rand.point_uav.x = disc_final->point_uav.x;
      q_rand.point_uav.y = disc_final->point_uav.y;
      q_rand.point_uav.z = disc_final->point_uav.z;
    }
    q_rand.point.x = initial_position_ugv.x * step_inv;
    q_rand.point.y = initial_position_ugv.y * step_inv;
    q_rand.point.z = initial_position_ugv.z * step_inv;

		if (debug_rrt)
			printf(" q_rand = [%f %f %f / %f %f %f] \n",
             q_rand.point.x*step, q_rand.point.y*step, q_rand.point.z*step,
             q_rand.point_uav.x * step, q_rand.point_uav.y * step, q_rand.point_uav.z * step);
		
		rrtgm.randNodeMarker(q_rand, rand_point_pub_, 1);

		new_solution = false;
		extendGraph(q_rand);
		count_loop++;

		if ( num_goal_found > 0 && count_loop == n_iter ){
			printf("\n\n\nRandomUAVPlanner::computeTree -->  goal found.\n")	; 
			rrt_path = getPath(); 
			printf("Path size: %lu , iterations: %i) : \n",rrt_path.size(), (count_loop)+(500*count_total_loop)); 

			int i_=0;
      //      printf("\tPrinting the Path Nodes obtainded : \n");
      //			for (auto pt_: rrt_path){
				// printf("\tRandom_uav_planner[%i/%lu] :  uav=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f]  length_catenary=%.3f    cost=%.3f\n", i_, rrt_path.size(),
        //       pt_->point_uav.x*step, pt_->point_uav.y*step, pt_->point_uav.z*step,
      //     pt_->rot_uav.x, pt_->rot_uav.y, pt_->rot_uav.z, pt_->rot_uav.w,
      //       pt_->length_cat, pt_->cost);
      //				i_++;
      //			}
			rrtgm.getPathMarker(rrt_path, lines_uav_marker_pub_, lines_uav_marker_pub_);
		 				ret_val = rrt_path.size();
		}	else if (count_loop >= n_iter) {
			count_total_loop++;
			count_loop = 0;
			if (count_total_loop > n_loop - 1){
				printf("RandomUAVPlanner::computeTreesUAV -->  could't find path Iterations: %lu \n\n", nodes_tree.size());    
				ret_val = 0;
				break;
			}	//else printf("\n\t\tPlanner :: computeTree: Starting new Loop \n");
		}
	}

  //  std::cout << "Finished Planner: Graph size: " << nodes_tree.size() <<std::endl;
//	std::cout << std::endl << "---------------------------------------------------------------------"
//            << std::endl << std::endl;

//  clock_gettime(CLOCK_REALTIME, &finish_rrt);
//  auto sec_rrt = finish_rrt.tv_sec - start_rrt.tv_sec;
//  auto msec_rrt = finish_rrt.tv_nsec - start_rrt.tv_nsec;
//  double time_rrt = msec_rrt*1e-9 + sec_rrt;
//  std::cout << "RRT time: " << time_rrt << std::endl << std::endl;

	if (markers_debug)
		rrtgm.getAllCatenaryMarker(nodes_tree, all_catenary_marker_pub_);

//  ccm->exportStats(catenary_file); (export them only at the end)

  return ret_val; 
}

bool RandomUAVPlanner::extendGraph(const RRTNode q_rand_)
{ 
  RRTNode* new_node = new RRTNode();
  RRTNode q_new;	//Take the new node value before to save it as a node in the list
  RRTNode* q_nearest = getNearestNode(q_rand_); 
		
  rrtgm.randNodeMarker(*q_nearest, nearest_point_pub_, 0);

  if (debug_rrt) 
    printf(" q_nearest = [%f %f %f / %f %f %f] l=%f feasible_l=%s\n", q_nearest->point.x*step,
           q_nearest->point.y*step,q_nearest->point.z*step, q_nearest->point_uav.x*step,
           q_nearest->point_uav.y*step,q_nearest->point_uav.z*step,
           q_nearest->length_cat, q_nearest->catenary? "true" : "false");
		
  bool exist_q_new_ = steering(*q_nearest, q_rand_, step_steer, q_new);
  if (!exist_q_new_){
    if ((count_loop%samp_goal_rate)==0)
      count_fail_connect_goal++;

    return false;
  }
  if ((count_loop%samp_goal_rate)==0)
    count_fail_connect_goal = 0;

  rrtgm.randNodeMarker(q_new, new_point_pub_, 2);
  q_new.parentNode = q_nearest;
  getParamsNode(q_new); // Here we check for catenary

  if (!obstacleFreeBetweenNodes(q_new, *q_nearest)) {
    return false;
  }

  RRTNode *q_min;
  q_min = q_nearest;
  if (debug_rrt)
    printf(" q_min = [%f %f %f / %f %f %f] \n", q_min->point.x*step,
           q_min->point.y*step,q_min->point.z*step,
           q_min->point_uav.x*step,q_min->point_uav.y*step,q_min->point_uav.z*step);

  //KdTree is updated after get Near Nodes because to not take it own node as a near 
  updateKdtreeNode(q_new); 	

  bool new_parentNode_ = false;
  // To make it easier --> not RRT*
  std::vector<int> v_near_nodes = getNearNodes(q_new, radius_near_nodes) ;
  // I . Open near nodes and connected with minimum accumulated

  for (size_t i = 0 ; i < v_near_nodes.size(); i++){
    for (auto nt_:nodes_tree) {
      if (nt_->id == v_near_nodes[i] ){
        if (obstacleFreeBetweenNodes(*nt_, q_new)){
          double C_ = nt_->cost + costBetweenNodes(*nt_,q_new);
          if (C_ < q_new.cost){
            q_min = nt_;
            new_parentNode_ = true;
          }
        } else {
          return false;
        }
      }
    }
  }
  if (new_parentNode_ ){
    q_new.parentNode = q_min;
    updateParamsNode(q_new);
  }
  
  *new_node = q_new;
  
  int got_goal_aux_ = got_to_goal; 
  isGoal(q_new); // Updates got_to_goal 
  if (got_to_goal != got_goal_aux_){
    saveNode(new_node);
    if(new_node->cost < disc_goal->cost){
      ROS_INFO(PRINTF_ROSE"\n\n\n\t\t Got GOAL position new node-> : [%f %f %f/%f %f %f]  \n",
               new_node->point.x*step, new_node->point.y*step, new_node->point.z*step,
               new_node->point_uav.x*step, new_node->point_uav.y*step, new_node->point_uav.z*step);
				disc_goal = new_node;
				num_goal_found++;
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
			rrtgm.getGraphMarker(new_node, count_graph, tree_rrt_star_uav_pub_, tree_rrt_star_uav_pub_);
		}
    rrtgm.getCatenaryMarker(points_catenary_new_node, one_catenary_marker_pub_);
		
		return true;
}

RRTNode RandomUAVPlanner::getRandomNode() 
{
 	// Random numbers
  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr_x_uav(ws_x_min, ws_x_max);  // define the discrete range
  std::uniform_int_distribution<int> distr_y_uav(ws_y_min, ws_y_max);  // define the discrete range
  std::uniform_int_distribution<int> distr_z_uav(ws_z_min+(0.0+0.6*step_inv), ws_z_max);  // define the range 1.0+0.6*step_inv
	RRTNode randomState_;
	bool found_node = false;
	bool catenary_state = false;

	// Get random position for UAV
  do{
    randomState_.point_uav.x = distr_x_uav(eng);
    randomState_.point_uav.y = distr_y_uav(eng);
    randomState_.point_uav.z = distr_z_uav(eng);
    found_node = checkNodeFeasibility(randomState_);
  }while(!found_node);

  return randomState_;
}

RRTNode* RandomUAVPlanner::getNearestNode(const RRTNode q_rand_) 
{
  RRTNode* q_nearest_ = NULL; 

	double p_uav_x_, p_uav_y_, p_uav_z_; 
	double d_uav_, l_cat_;
	
	double cost_nearest_ = 10000000;
	for (auto nt_:nodes_tree) {
    p_uav_x_ = q_rand_.point_uav.x * step - nt_->point_uav.x * step;
		p_uav_y_ = q_rand_.point_uav.y * step - nt_->point_uav.y * step;
		p_uav_z_ = q_rand_.point_uav.z * step - nt_->point_uav.z * step;

    d_uav_ =  ((p_uav_x_*p_uav_x_) + (p_uav_y_*p_uav_y_) + (p_uav_z_*p_uav_z_));
		l_cat_ = nt_->length_cat;

    double cost_ =  d_uav_; //Cost to choose q_nearest

		if(cost_nearest_ > cost_){
			q_nearest_ = nt_;
			cost_nearest_ = cost_;
		}
	}

	return q_nearest_;
}

bool RandomUAVPlanner::steering(const RRTNode &q_nearest_, const RRTNode &q_rand_, float factor_steer_, RRTNode &q_new_)	
{
  float x_rand_uav, y_rand_uav, z_rand_uav; 
  float x_nearest_uav, y_nearest_uav, z_nearest_uav;
  float dir_uav_x, dir_uav_y, dir_uav_z, uni_uav_x, uni_uav_y, uni_uav_z; 
  float x_uav_, y_uav_, z_uav_;

  // Get the ugv point
  q_new_.point = disc_initial->point;

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
			
  double dist_nearest_rand = sqrt(dir_uav_x*dir_uav_x + dir_uav_y*dir_uav_y + dir_uav_z*dir_uav_z);

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

  return checkNodeFeasibility(q_new_);
}

inline geometry_msgs::Point getInterpPoint(const geometry_msgs::Point &A,
                                           const geometry_msgs::Point &B,
                                           float i = 0.5) {
    geometry_msgs::Point p;
    float ot = 1.0 - i;

    p.x = i * A.x + ot * B.x;
    p.y = i * A.y + ot * B.y;
    p.z = i * A.z + ot * B.z;

    return p;
}

bool RandomUAVPlanner::obstacleFreeBetweenNodes(const RRTNode q_nearest_,
                                                const RRTNode q_new_)
{
  // New method --> we get the intermediate points with the mean of catenary lengths
  // --> obtain catenary and check
  // TODO: add more than one middle point
  geometry_msgs::Point middle_uav, middle_reel;
  geometry_msgs::Point  point_nearest_uav_ , point_new_uav_,p_reel_nearest_, p_reel_new_;

  float addition = 1.0/(float)n_intermediate;

	point_new_uav_.x = q_new_.point_uav.x * step; 
	point_new_uav_.y = q_new_.point_uav.y * step; 
	point_new_uav_.z = q_new_.point_uav.z * step;

	point_nearest_uav_.x = q_nearest_.point_uav.x * step; 
	point_nearest_uav_.y = q_nearest_.point_uav.y * step; 
	point_nearest_uav_.z = q_nearest_.point_uav.z * step;

  p_reel_nearest_ = getReelNode(q_nearest_);
  p_reel_new_ = getReelNode(q_new_); 

  bool ret_val = true;
  ccm->distance_tether_obstacle = distance_catenary_obstacle;
  //ROS_INFO("Setting the catenary obstacle dist to: %f", distance_catenary_obstacle);
  for (int i = 0; i <= n_intermediate && ret_val; i++) {
    float p = i * addition;
    float new_length = q_nearest_.length_cat * p + q_new_.length_cat * (1.0 - p);
    middle_uav = getInterpPoint(point_nearest_uav_, point_new_uav_, p);

    //middle_reel = getInterpPoint(p_reel_nearest_, p_reel_new_, p); (UGV is fixed --> no need)
    ret_val = ccm->checkCatenary(p_reel_new_, middle_uav, new_length);
  }

  return ret_val;
}

/*
bool RandomUAVPlanner::obstacleFreeBetweenNodes(const RRTNode q_nearest_,
                                                const RRTNode q_new_) {
  	geometry_msgs::Point  point_nearest_uav_ , point_new_uav_,p_reel_nearest_, p_reel_new_;
	std::vector<geometry_msgs::Point> points_cat_nearest_, points_cat_new_;
	points_cat_nearest_.clear();
	points_cat_new_.clear();
	points_cat_new_ = points_catenary_new_node;

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
*/

std::vector<int> RandomUAVPlanner::getNearNodes(const RRTNode &q_new_, double radius_) 
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

std::vector<float> RandomUAVPlanner::getNearestUAVNode(const RRTNode &q_new_)  
{
	std::vector<float> ret_;
	ret_.clear();

	point_t pt_;
	pt_ = {(float)q_new_.point.x, (float)q_new_.point.y, (float)q_new_.point.z, q_new_.rot_uav.x, q_new_.rot_uav.y, q_new_.rot_uav.z, q_new_.rot_uav.w};
		
	KDTree treeuav(v_uav_nodes_kdtree);
	point_t nearest_ = treeuav.nearest_point(pt_,3);


  for (auto i:nearest_) {
    ret_.push_back(i);
  }

	return ret_;
}

void RandomUAVPlanner::getOrientation(RRTNode &n_ , RRTNode p_)
{
	float yaw_;
	tf::Quaternion _quat;

  yaw_ = atan2(n_.point_uav.y - p_.point_uav.y, n_.point_uav.x - p_.point_uav.x);
  _quat.setRPY(0.0, 0.0, yaw_);
  n_.rot_uav.x = _quat.x();
  n_.rot_uav.y = _quat.y();
  n_.rot_uav.z = _quat.z();
  n_.rot_uav.w = _quat.w();
}

float RandomUAVPlanner::getYawFromQuaternion(RRTNode n_)
{
	double r_, p_, y_;

  tf::Quaternion q(n_.rot_uav.x, n_.rot_uav.y, n_.rot_uav.z, n_.rot_uav.w);
  tf::Matrix3x3 M(q);	
  M.getRPY(r_, p_, y_);
	

	return y_;
}

double RandomUAVPlanner::costNode(const RRTNode q_new_)
{
  double p_uav_x_ = q_new_.point_uav.x * step - q_new_.parentNode->point_uav.x * step;
	double p_uav_y_ = q_new_.point_uav.y * step - q_new_.parentNode->point_uav.y * step;
	double p_uav_z_ = q_new_.point_uav.z * step - q_new_.parentNode->point_uav.z * step;

  return sqrt((p_uav_x_*p_uav_x_) + (p_uav_y_*p_uav_y_) + (p_uav_z_*p_uav_z_));
}

double RandomUAVPlanner::costBetweenNodes(const RRTNode q_near_, const RRTNode q_new_)
{
	double p_uav_x_, p_uav_y_, p_uav_z_;
	
  p_uav_x_ = q_new_.point_uav.x * step - q_near_.point_uav.x * step;
  p_uav_y_ = q_new_.point_uav.y * step - q_near_.point_uav.y * step;
  p_uav_z_ = q_new_.point_uav.z * step - q_near_.point_uav.z * step;

  return  sqrt((p_uav_x_*p_uav_x_) + (p_uav_x_*p_uav_x_) + (p_uav_z_*p_uav_z_));
}

void RandomUAVPlanner::getParamsNode(RRTNode &node_, bool is_init_)
{
 	double Cat1_, Cat2_;
	Cat1_ = 5.0;
	Cat2_ = 20.0;
	double r_security_cat_ = 0.1;
	
	geometry_msgs::Point p_node_uav_;
	p_node_uav_.x = node_.point_uav.x * step;
	p_node_uav_.y = node_.point_uav.y * step;
	p_node_uav_.z = node_.point_uav.z * step;
	double dist_obs_uav = ccm->getPointDistanceFullMap(use_distance_function, p_node_uav_);
	
	node_.id = getWorldIndex(node_.point.x, node_.point.y, node_.point.z);
	node_.min_dist_obs_uav = dist_obs_uav;

  node_.id_uav = getWorldIndex(node_.point_uav.x, node_.point_uav.y, node_.point_uav.z);
  node_.cost_takeoff = -1.0;

  checkCatenary(node_, points_catenary_new_node); 
  if (is_init_){
    id_uav_init = node_.id_uav;
  } else {
		node_.cost = costNode(node_);
  }
}

void RandomUAVPlanner::updateParamsNode(RRTNode &node_)
{
	node_.cost = costNode(node_);
}

bool RandomUAVPlanner::checkNodeFeasibility(const RRTNode pf_)
{
	bool ret = false;
	double d_;

  if (isInside(pf_.point_uav.x,pf_.point_uav.y,pf_.point_uav.z)){
    geometry_msgs::Point obs_to_uav, pos_uav; 
    if (d_ == -1.0)
      return false;

    if (d_ > distance_obstacle_uav) 
      ret = true;
    else
      ret = false;
  }
		
  return ret;
}

bool RandomUAVPlanner::checkPointsCatenaryFeasibility(const RRTNode pf_)
{
	bool ret = false;
	double d_;

	geometry_msgs::Point obs_to_uav, pos_uav; 
	pos_uav.x = pf_.point.x * step ;
	pos_uav.y = pf_.point.y * step ; 
	pos_uav.z = pf_.point.z * step ; 
	d_ = ccm->getPointDistanceFullMap(use_distance_function, pos_uav);

	if (d_ > distance_tether_obstacle)
		ret = true;

	return ret;
}

bool RandomUAVPlanner::checkCatenary(RRTNode &q_init_, vector<geometry_msgs::Point> &points_catenary_)
{
	// mode 1: UGV-Goal  ,  mode 2: UGV-UAV
	geometry_msgs::Point p_reel_, p_final_;
	
	p_reel_ = getReelNode(q_init_);

  p_final_.x = q_init_.point_uav.x * step ;	
  p_final_.y = q_init_.point_uav.y * step ;   
  p_final_.z = q_init_.point_uav.z * step ;   

  if (debug_rrt) {
    printf("Checking catenary. P reel: %f %f %f. \t P Final: %f %f %f\n",
           p_reel_.x, p_reel_.y, p_reel_.z,
           p_final_.x, p_final_.y, p_final_.z);
  }

	bool found_catenary = ccm->searchCatenary(p_reel_, p_final_, points_catenary_);
	if(found_catenary){
		// printf("\t RandomUAVPlanner::checkCatenary: points_catenary_=%lu\n",points_catenary_.size());
		q_init_.p_cat = points_catenary_;
		q_init_.min_dist_obs_cat = ccm->min_dist_obs_cat;
		q_init_.length_cat = ccm->length_cat_final;
		q_init_.catenary = found_catenary;
	}
	return found_catenary;
}

geometry_msgs::Point RandomUAVPlanner::getReelNode(const RRTNode node_)
{
	geometry_msgs::Point pos_reel;
	float yaw_ugv;

	yaw_ugv = getYawFromQuaternion(node_);
	double lengt_vec =  sqrt(pos_reel_ugv.x*pos_reel_ugv.x + pos_reel_ugv.y*pos_reel_ugv.y);
	pos_reel.x = node_.point.x*step + lengt_vec *cos(yaw_ugv); 
	pos_reel.y = node_.point.y*step + lengt_vec *sin(yaw_ugv);
	pos_reel.z = node_.point.z*step + pos_reel_ugv.z ;

	return pos_reel;
}

void RandomUAVPlanner::updateKdtreeNode(const RRTNode ukT_)
{
	point_t pt_;

  pt_ = {(float)ukT_.point_uav.x, (float)ukT_.point_uav.y, (float)ukT_.point_uav.z};
  v_nodes_kdtree.push_back(pt_);
}

void RandomUAVPlanner::updateKdtreeUAV(const RRTNode ukT_)
{
	point_t pt_;

	pt_ = {(float)ukT_.point_uav.x, (float)ukT_.point_uav.y, (float)ukT_.point_uav.z,
         ukT_.rot_uav.x, ukT_.rot_uav.y, ukT_.rot_uav.z, ukT_.rot_uav.w};
	v_uav_nodes_kdtree.push_back(pt_);
}

void RandomUAVPlanner::readPointCloudMapForUAV(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_obs_uav.setInput(*msg);
	ROS_INFO_COND(debug_rrt, PRINTF_BLUE "RandomUAVPlanner Planner: Receiving point cloud map to create Kdtree for UAV Obstacles");
}

bool RandomUAVPlanner::saveNode(RRTNode* sn_, bool is_init_)
{
	if(is_init_){
		getParamsNode(*sn_,is_init_);
		if (!sn_->catenary)
			return false;
	}

	nodes_tree.push_back(sn_);

	return true;
}

inline void RandomUAVPlanner::clearStatus()
{
	if (nodes_marker_debug)
    rrtgm.clearUAVNodesTree(tree_rrt_star_uav_pub_);
	rrtgm.clearMarkers(lines_uav_marker_pub_);
  	
	nodes_tree.clear();
	rrt_path.clear();

  got_to_goal = 0;
	num_goal_found = 0;
	length_catenary.clear();
	v_nodes_kdtree.clear();
	v_uav_nodes_kdtree.clear();
	points_catenary_new_node.clear();
}

void RandomUAVPlanner::clearNodesMarker()
{ 
	rrtgm.clearUAVNodesTree(tree_rrt_star_uav_pub_); 
}

void RandomUAVPlanner::clearCatenaryGPMarker()
{ 
	rrtgm.clearCatenaryMarker(catenary_marker_pub_); 
}

void RandomUAVPlanner::clearLinesGPMarker()
{
	rrtgm.clearMarkers(lines_uav_marker_pub_);
}

std::list<RRTNode*> RandomUAVPlanner::getPath()
{
	std::list<RRTNode*> path_;
	RRTNode* current_node;
	
  current_node = disc_goal;
  path_.push_front(current_node);
  length_catenary.push_back(current_node->length_cat);
  while (current_node->parentNode != NULL){ 
    current_node = current_node->parentNode;
    path_.push_front(current_node);
    length_catenary.push_back(current_node->length_cat);
  }
	
	return path_;
}

bool RandomUAVPlanner::isGoal(const RRTNode st_) 
{
	geometry_msgs::Point point_;
  bool ret_val = false;
	
  point_.x = st_.point_uav.x;
  point_.y = st_.point_uav.y;
  point_.z = st_.point_uav.z;

  double dist_goal_ = sqrt(pow(point_.x*step - final_position.x,2) + 
                           pow(point_.y*step - final_position.y,2) +
                           pow(point_.z*step - final_position.z,2) );
	
  if (dist_goal_ < goal_gap_m){
    got_to_goal = got_to_goal + 1;
    ret_val = true;
  }
  return ret_val;
}

bool RandomUAVPlanner::getGlobalPath(Trajectory &trajectory)
{
	trajectory_msgs::MultiDOFJointTrajectoryPoint traj_marsupial_;

	traj_marsupial_.transforms.resize(2);
	traj_marsupial_.velocities.resize(2);
	traj_marsupial_.accelerations.resize(2);

	for(auto nt_ : rrt_path){
    // UGV (does not move)
		traj_marsupial_.transforms[0].translation.x = initial_position_ugv.x;
		traj_marsupial_.transforms[0].translation.y = initial_position_ugv.y;
    traj_marsupial_.transforms[0].translation.z = initial_position_ugv.z;

		traj_marsupial_.transforms[0].rotation.x = 0;
		traj_marsupial_.transforms[0].rotation.y = 0;
		traj_marsupial_.transforms[0].rotation.z = 0;
		traj_marsupial_.transforms[0].rotation.w = 1;
		traj_marsupial_.velocities[0].linear.x = 0.0;
		traj_marsupial_.velocities[0].linear.y = 0.0;
		traj_marsupial_.velocities[0].linear.z = 0.0;
		traj_marsupial_.accelerations[0].linear.x = 0.0;
		traj_marsupial_.accelerations[0].linear.y = 0.0;
		traj_marsupial_.accelerations[0].linear.z = 0.0;

    //UAV
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

void RandomUAVPlanner::configRRTParameters(double _l_m, geometry_msgs::Point _p_reel , geometry_msgs::Point _p_ugv,
                                           int n_iter_ , int n_loop_, double r_nn_, double s_s_, int s_g_r_)
{
	length_tether_max = _l_m;
	n_iter = n_iter_;
	n_loop = n_loop_;
	radius_near_nodes = r_nn_;
	step_steer = s_s_;
	samp_goal_rate = s_g_r_;

	geometry_msgs::Point pos_r;

	pos_reel_ugv = _p_reel;
	pos_tf_ugv = _p_ugv;

    pos_r.x = _p_reel.x;
    pos_r.y = _p_reel.y;
    pos_r.z = _p_reel.z;

	rrtgm.configGraphMarkers(frame_id, step, is_coupled, n_iter, pos_reel_ugv);
	ccm->init(grid_3D, distance_catenary_obstacle, 0.0, distance_obstacle_uav,
	          length_tether_max, ws_z_min, step, use_parabola, use_distance_function, pos_r,
			  false, !use_parabola);
}

bool RandomUAVPlanner::setInitialPositionIndependent(RRTNode n_)
{
	if (isInside(n_.point_uav.x, n_.point_uav.y, n_.point_uav.z)) {
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
		ROS_INFO(PRINTF_BLUE "RandomUAVPlanner::setInitialPositionIndependent -->  disc_initial  UGV:[%f %f %f /%f %f %f %f]  UAV:[%f %f %f /%f %f %f %f]",
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

bool RandomUAVPlanner::setFinalPosition(DiscretePosition p_)
{
	if (isInside(p_.x, p_.y, p_.z)){
		RRTNodeLink3D *finalNodeInWorld = &discrete_world[getWorldIndex(p_.x, p_.y, p_.z)];

		Eigen::Vector3d p_node_, trav_point_ugv_;
		p_node_.x() = p_.x * step;
		p_node_.y() = p_.y * step;
		p_node_.z() = p_.z * step;
		
		if (finalNodeInWorld->node == NULL){
			finalNodeInWorld->node = new RRTNode();
			finalNodeInWorld->node->point.x = p_.x;
			finalNodeInWorld->node->point.y = p_.y;
			finalNodeInWorld->node->point.z = p_.z;
			finalNodeInWorld->node->point_uav = finalNodeInWorld->node->point;

			finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
		}
		disc_final = finalNodeInWorld->node;
		
		disc_final->point_uav = p_;
		disc_final->point = disc_initial->point;
		final_position.x = disc_final->point_uav.x*step;
		final_position.y = disc_final->point_uav.y*step;
		final_position.z = disc_final->point_uav.z*step;

		ROS_INFO(PRINTF_BLUE "RandomUAVPlanner::setFinalPosition -->  final_position [%f %f %f]",
             final_position.x, final_position.y, final_position.z);

		return true;
	}
	else{
		disc_final = NULL;
		return false;
	}
}

inline void RandomUAVPlanner::setInitialCostGoal(RRTNode* p_)
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

bool RandomUAVPlanner::isInitialPositionUAVOccupied()
{
	return isOccupied(*disc_initial);
}

bool RandomUAVPlanner::isFinalPositionOccupied()
{
	return isOccupied(*disc_final);
}

bool RandomUAVPlanner::isOccupied(RRTNode n_)
{
  return !discrete_world[getWorldIndex(n_.point_uav.x, n_.point_uav.y, n_.point_uav.z)].notOccupied;
}

void RandomUAVPlanner::updateMap(octomap_msgs::OctomapConstPtr message)
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
    auto end = map->end_leafs();
		for (octomap::OcTree::leaf_iterator it = map->begin_leafs(); it != end; ++it)
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

void RandomUAVPlanner::clearMap()
{
	for (int i = 0; i < matrix_size; i++){
		discrete_world[i].notOccupied = true;
	}
}

DiscretePosition RandomUAVPlanner::discretizePosition(Vector3 p)
{
	DiscretePosition res;

	res.x = p.x * step_inv;
	res.y = p.y * step_inv;
	res.z = p.z * step_inv;

	return res;
}

} // end namespace
