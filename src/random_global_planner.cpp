#include <rrt_planners/random_global_planner.hpp>

namespace PathPlanners
{

RandomGlobalPlanner::RandomGlobalPlanner(std::string node_name_)
{
    //The tf buffer is used to lookup the base link position(tf from world frame to robot base frame)
    node_name = node_name_;

    nh.reset(new ros::NodeHandle("~"));

    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
    tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));
    
    configParams();
    configTopics();
    configServices();
    configRRTStar();
    configRandomPlanner();
    // graphMarker();  // This method graph the catenary for the initial position.
}

void RandomGlobalPlanner::graphMarker()
{
	CatenarySolver cS;
	std::vector<geometry_msgs::Point> points_catenary_;
    geometry_msgs::Vector3 pos_reel_ugv ;
    pos_reel_ugv.x = pos_reel_x;
    pos_reel_ugv.y = pos_reel_y;
    pos_reel_ugv.z = pos_reel_z;
    
    geometry_msgs::Point start_ugv, start_uav;
    geometry_msgs::TransformStamped position_ugv_,position_uav_;
    position_ugv_ = getRobotPoseUGV();
    position_uav_ = getRobotPoseUAV();
    start_ugv.x = position_ugv_.transform.translation.x;
    start_ugv.y = position_ugv_.transform.translation.y;
    start_ugv.z = position_ugv_.transform.translation.z + pos_reel_z; 
    start_uav.x = position_uav_.transform.translation.x;
    start_uav.y = position_uav_.transform.translation.y;
    start_uav.z = position_uav_.transform.translation.z;

    bool check_catenary = true;
	bool founded_catenary = false;
	bool increase_catenary;
	double length_catenary_;
	int n_points_cat_dis_;
	double security_dis_ca_ = 0.1;
	double delta_ = 0.0;	//Initial Value
    double dist_init_final_ = sqrt(pow(start_uav.x-start_ugv.x,2) + pow(start_uav.y-start_ugv.y,2) + pow(start_uav.z-start_ugv.z,2));

	std::string node_name_ = "grid3D_global_node";
	Grid3d grid_3D(node_name_);

    do{
		increase_catenary = false;
		points_catenary_.clear();
		length_catenary_ = dist_init_final_* (1.001 + delta_);
		if (length_catenary_ > length_tether_max){
			check_catenary = false;
			break;
		}
		cS.solve(start_ugv.x, start_ugv.y, start_ugv.z, start_uav.x, start_uav.y, start_uav.z, length_catenary_, points_catenary_);
		double d_min_point_cat = 100000;
		if (points_catenary_.size() > 5){
			n_points_cat_dis_ = ceil(1.5*ceil(length_catenary_)); // parameter to ignore collsion points in the begining and in the end of catenary
			if (n_points_cat_dis_ < 5)
				n_points_cat_dis_ = 5;
			for (size_t i = 0 ; i < points_catenary_.size() ; i++){
				geometry_msgs::Point point_cat;
				Eigen::Vector3d p_in_cat_, obs_to_cat_;
				if (points_catenary_[i].z < ws_z_min){
					check_catenary = false;
					break;
				}
				if ((i > n_points_cat_dis_ ) && (i < points_catenary_.size()-n_points_cat_dis_/2)){
					p_in_cat_.x() = points_catenary_[i].x;
					p_in_cat_.y() = points_catenary_[i].y;
					p_in_cat_.z() = points_catenary_[i].z;
					double dist_cat_obs;
					bool is_into_ = grid_3D.isIntoMap(p_in_cat_.x(),p_in_cat_.y(),p_in_cat_.z());
					if(is_into_)
						dist_cat_obs =  grid_3D.getPointDist((double)p_in_cat_.x(),(double)p_in_cat_.y(),(double)p_in_cat_.z()) ;
					else
						dist_cat_obs = -1.0;

					if (d_min_point_cat > dist_cat_obs){
						d_min_point_cat = dist_cat_obs;
					}
					if (dist_cat_obs < security_dis_ca_){
						delta_ = delta_ + 0.005;
						increase_catenary = true;
						break;
					}
				}
				point_cat.x = points_catenary_[i].x;
				point_cat.y = points_catenary_[i].y;
				point_cat.z = points_catenary_[i].z;
	
			}
			if (check_catenary && !increase_catenary){
				founded_catenary = true;
				check_catenary = false;
			}
		}
		else{
			check_catenary = false;
		}
	}while (check_catenary);


	rrtgm.configGraphMarkers(world_frame, map_resolution, coupled, n_iter, pos_reel_ugv);
    rrtgm.getCatenaryMarker(points_catenary_, initial_catenary_pub_);
}

//This function gets parameter from param server at startup if they exists, if not it passes default values
void RandomGlobalPlanner::configParams()
{
    //At startup, no goal and no costmap received yet
    seq = 0;
    timesReplaned = 0;
    mapRec = false;
    minPathLenght=0;

    //Get params from param server. If they dont exist give variables default values
// nh->param("show_config", showConfig, (bool)0);
    nh->param("planner_type", planner_type, (std::string)"rrt_star");
    nh->param("debug", debug, (bool)0);
    nh->param("timeout", timeout, (double)10);
// nh->param("initial_position_search_dist", initialSearchAround, (double)1);

    nh->param("ws_x_max", ws_x_max, (double)30);
    nh->param("ws_y_max", ws_y_max, (double)30);
    nh->param("ws_z_max", ws_z_max, (double)30);
    nh->param("ws_x_min", ws_x_min, (double)0);
    nh->param("ws_y_min", ws_y_min, (double)0);
    nh->param("ws_z_min", ws_z_min, (double)0);

    nh->param("map_resolution", map_resolution, (double)0.05);
    nh->param("map_h_inflaction", map_h_inflaction, (double)0.05);
    nh->param("map_v_inflaction", map_v_inflaction, (double)0.05);


    nh->param("traj_dxy_max", traj_dxy_max, (double)1);
    nh->param("traj_pos_tol", traj_pos_tol, (double)1);
    nh->param("traj_yaw_tol", traj_yaw_tol, (double)0.1);
    nh->param("traj_dz_max", traj_dz_max, (double)1);
    nh->param("traj_vxy_m", traj_vxy_m, (double)1);
    nh->param("traj_vz_m", traj_vz_m, (double)1);
    nh->param("traj_vxy_m_1", traj_vxy_m_1, (double)1);
    nh->param("traj_vz_m_1", traj_vz_m_1, (double)1);
    nh->param("traj_wyaw_m", traj_wyaw_m, (double)1);

    nh->param("n_iter", n_iter, (int)100);
    nh->param("n_loop", n_loop, (int)1);
    nh->param("world_frame", world_frame, (string) "/map");
    nh->param("ugv_base_frame", ugv_base_frame, (string) "ugv_base_link");
    nh->param("uav_base_frame", uav_base_frame, (string) "uav_base_link");
    nh->param("pos_reel_x", pos_reel_x, (double)0.4);
    nh->param("pos_reel_y", pos_reel_y, (double)0.0);
    nh->param("pos_reel_z", pos_reel_z, (double)0.22);
    nh->param("radius_near_nodes", radius_near_nodes, (double)1.0);
    nh->param("step_steer", step_steer, (double)0.5);
    nh->param("goal_gap_m", goal_gap_m, (double)0.2);
    nh->param("min_l_steer_ugv", min_l_steer_ugv, (double)5.0);

    nh->param("distance_obstacle_ugv", distance_obstacle_ugv, (double)1.0);
    nh->param("distance_obstacle_uav", distance_obstacle_uav, (double)1.0);
    nh->param("distance_catenary_obstacle", distance_catenary_obstacle, (double)0.1);

  	nh->param("write_data_for_analysis",write_data_for_analysis, (bool)0);
	nh->param("path", path, (std::string) "~/");

    nh->param("sample_mode", sample_mode, (int)0);
    nh->param("do_steer_ugv", do_steer_ugv, (bool)true);
    nh->param("coupled", coupled, (bool)true);
    nh->param("samp_goal_rate", samp_goal_rate, (int)10);
    nh->param("debug_rrt", debug_rrt, (bool)true);
     
	nh->param("name_output_file", name_output_file, (std::string) "optimization_test");
    nh->param("scenario_number", scenario_number,(int)1);
	nh->param("num_pos_initial", num_pos_initial,(int)1);
	nh->param("num_goal", num_goal,(int)0);


    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner 3D Node Configuration:");
    ROS_INFO_COND(showConfig, PRINTF_GREEN "   Workspace = X: [%.2f, %.2f]\t Y: [%.2f, %.2f]\t Z: [%.2f, %.2f]  ", ws_x_max, ws_x_min, ws_y_max, ws_y_min, ws_z_max, ws_z_min);

    ROS_INFO_COND(showConfig, PRINTF_GREEN "   Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "   World frame: %s, UGV base frame: %s, UAV base frame: %s ", world_frame.c_str(), ugv_base_frame.c_str(), uav_base_frame.c_str());

    // configMarkers("global_path_3d");
}

void RandomGlobalPlanner::configRRTStar()
{
    randPlanner.init(planner_type, world_frame, ws_x_max, ws_y_max, ws_z_max, ws_x_min, ws_y_min, ws_z_min, map_resolution, map_h_inflaction, map_v_inflaction, 
                    nh, goal_gap_m, debug_rrt, distance_obstacle_ugv, distance_obstacle_uav, distance_catenary_obstacle, grid3D);
}

void RandomGlobalPlanner::configTopics()
{
    replan_status_pub = nh->advertise<std_msgs::Bool>("replanning_status", 1);
    visMarkersPublisher = nh->advertise<visualization_msgs::Marker>("markers", 2);
    fullRayPublisher = nh->advertise<visualization_msgs::Marker>("full_ray", 2);
    rayCastFreePublisher = nh->advertise<visualization_msgs::Marker>("ray_cast_free", 2);
    rayCastFreeReducedPublisher = nh->advertise<visualization_msgs::Marker>("ray_cast_free_reduced", 2);
    rayCastCollPublisher = nh->advertise<visualization_msgs::Marker>("ray_cast_coll", 2);
    rayCastNoFreePublisher = nh->advertise<visualization_msgs::Marker>("ray_cast_no_free", 2);
    reducedMapPublisher = nh->advertise<octomap_msgs::Octomap>("octomap_reduced", 1000);

    cleanMarkersOptimizerPublisher = nh->advertise<std_msgs::Bool>("/clean_marker_optimizer", 1);
	initial_catenary_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_initial_position", 100, true);


    bool useOctomap;
    nh->param("use_octomap", useOctomap, (bool)false);

    sub_map = nh->subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &RandomGlobalPlanner::collisionMapCallBack, this);

    // point_cloud_map_sub_ = nh->subscribe( "/octomap_point_cloud_centers", 1,  &RandomGlobalPlanner::readPointCloudMapCallback, this);
    point_cloud_map_trav_sub_ = nh->subscribe( "/region_growing_traversability_pc_map", 1,  &RandomGlobalPlanner::readPointCloudTraversabilityMapCallback, this);
    point_cloud_map_ugv_sub_ = nh->subscribe( "/region_growing_obstacles_pc_map", 1,  &RandomGlobalPlanner::readPointCloudUGVObstaclesMapCallback, this);
    point_cloud_map_uav_sub_ = nh->subscribe( "/octomap_point_cloud_centers", 1,  &RandomGlobalPlanner::readPointCloudUAVObstaclesMapCallback, this);

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner 3D Topics and Subscriber Configurated");
    
}

void RandomGlobalPlanner::collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
{
    map = msg;
    randPlanner.updateMap(map);
    mapRec = true;
    sub_map.shutdown();

    ROS_INFO_COND(debug, PRINTF_GREEN "Global Planner: Collision Map Received");
}

void RandomGlobalPlanner::readPointCloudTraversabilityMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    randPlanner.readPointCloudTraversabilityMapUGV(msg);
    ROS_INFO_COND(debug, PRINTF_GREEN "Global Planner: UGV Traversability Map Navigation Received");
}

void RandomGlobalPlanner::readPointCloudUGVObstaclesMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    randPlanner.readPointCloudMapForUGV(msg);
    ROS_INFO_COND(debug, PRINTF_GREEN "Global Planner: UGV Obstacles Map Navigation Received");
}

void RandomGlobalPlanner::readPointCloudUAVObstaclesMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    randPlanner.readPointCloudMapForUAV(msg);
    ROS_INFO_COND(debug, PRINTF_GREEN "Global Planner: UAV Obstacles Map Navigation Received");
}

void RandomGlobalPlanner::configServices()
{
    execute_path_client_ptr.reset(new ExecutePathClient("/Execute_Plan", true));
    make_plan_server_ptr.reset(new MakePlanServer(*nh, "/Make_Plan", false));
    make_plan_server_ptr->registerGoalCallback(boost::bind(&RandomGlobalPlanner::makePlanGoalCB, this));
    make_plan_server_ptr->registerPreemptCallback(boost::bind(&RandomGlobalPlanner::makePlanPreemptCB, this));

    make_plan_server_ptr->start();
    execute_path_client_ptr->waitForServer();

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner 3D: Action client from global planner ready");
}

void RandomGlobalPlanner::configMarkers(std::string ns)
{
    lineMarker.ns = ns;
    lineMarker.header.frame_id = world_frame;
    lineMarker.header.stamp = ros::Time::now();
    lineMarker.id = rand();
    lineMarker.lifetime = ros::Duration(500);
    lineMarker.type = RVizMarker::LINE_STRIP;
    lineMarker.action = RVizMarker::ADD;
    lineMarker.pose.orientation.w = 1;
    lineMarker.color.r = 0.0;
    lineMarker.color.g = 1.0;
    lineMarker.color.b = 0.0;
    lineMarker.color.a = 1.0;
    lineMarker.scale.x = 0.1;

    waypointsMarker.ns = ns;
    waypointsMarker.header.frame_id = world_frame;
    waypointsMarker.header.stamp = ros::Time::now();
    waypointsMarker.id = lineMarker.id + 1;
    waypointsMarker.lifetime = ros::Duration(500);
    waypointsMarker.type = RVizMarker::POINTS;
    waypointsMarker.action = RVizMarker::ADD;
    waypointsMarker.pose.orientation.w = 1;
    waypointsMarker.color.r = 1.0;
    waypointsMarker.color.g = 1.0;
    waypointsMarker.color.b = 0.0;
    waypointsMarker.color.a = 1.0;
    waypointsMarker.scale.x = 0.15;
    waypointsMarker.scale.y = 0.15;
    // waypointsMarker.scale.z = 0.4;

    fullrayMarker.ns = ns;
    fullrayMarker.header.frame_id = world_frame;
    fullrayMarker.header.stamp = ros::Time::now();
    fullrayMarker.id = lineMarker.id + 1;
    fullrayMarker.lifetime = ros::Duration(500);
    fullrayMarker.type = RVizMarker::POINTS;
    fullrayMarker.action = RVizMarker::ADD;
    fullrayMarker.pose.orientation.w = 1;
    fullrayMarker.color.r = 1.0;
    fullrayMarker.color.g = 1.0;
    fullrayMarker.color.b = 1.0;
    fullrayMarker.color.a = 1.0;
    fullrayMarker.scale.x = 0.05;
    fullrayMarker.scale.y = 0.05;
    // fullrayMarker.scale.z = 0.05;
    
    raycastfreeMarker.ns = ns;
    raycastfreeMarker.header.frame_id = world_frame;
    // raycastfreeMarker.header.stamp = ros::Time::now();
    raycastfreeMarker.id = lineMarker.id + 1;
    raycastfreeMarker.lifetime = ros::Duration(500);
    raycastfreeMarker.type = RVizMarker::POINTS;
    raycastfreeMarker.action = RVizMarker::ADD;
    raycastfreeMarker.pose.orientation.w = 1;
    raycastfreeMarker.color.r = 0.6;
    raycastfreeMarker.color.g = 0.6;
    raycastfreeMarker.color.b = 0.6;
    raycastfreeMarker.color.a = 1.0;
    raycastfreeMarker.scale.x = 0.05;
    raycastfreeMarker.scale.y = 0.05;
    // raycastfreeMarker.scale.z = 0.05;

    raycastfreereducedMarker.ns = ns;
    raycastfreereducedMarker.header.frame_id = world_frame;
    // raycastfreereducedMarker.header.stamp = ros::Time::now();
    raycastfreereducedMarker.id = lineMarker.id + 1;
    raycastfreereducedMarker.lifetime = ros::Duration(500);
    raycastfreereducedMarker.type = RVizMarker::POINTS;
    raycastfreereducedMarker.action = RVizMarker::ADD;
    raycastfreereducedMarker.pose.orientation.w = 1;
    raycastfreereducedMarker.color.r = 1.0;
    raycastfreereducedMarker.color.g = 1.0;
    raycastfreereducedMarker.color.b = 0.0;
    raycastfreereducedMarker.color.a = 1.0;
    raycastfreereducedMarker.scale.x = 0.05;
    raycastfreereducedMarker.scale.y = 0.05;
    // raycastfreereducedMarker.scale.z = 0.05;

    raycastcollMarker.ns = ns;
    raycastcollMarker.header.frame_id = world_frame;
    // raycastcollMarker.header.stamp = ros::Time::now();
    raycastcollMarker.id = lineMarker.id + 1;
    raycastcollMarker.lifetime = ros::Duration(500);
    raycastcollMarker.type = RVizMarker::POINTS;
    raycastcollMarker.action = RVizMarker::ADD;
    raycastcollMarker.pose.orientation.w = 1;
    raycastcollMarker.color.r = 0.6;
    raycastcollMarker.color.g = 0.6;
    raycastcollMarker.color.b = 0.0;
    raycastcollMarker.color.a = 1.0;
    raycastcollMarker.scale.x = 0.05;
    raycastcollMarker.scale.y = 0.05;
    // raycastcollMarker.scale.z = 0.05;

    raycastnofreeMarker.ns = ns;
    raycastnofreeMarker.header.frame_id = world_frame;
    // raycastnofreeMarker.header.stamp = ros::Time::now();
    raycastnofreeMarker.id = lineMarker.id + 1;
    raycastnofreeMarker.lifetime = ros::Duration(500);
    raycastnofreeMarker.type = RVizMarker::POINTS;
    raycastnofreeMarker.action = RVizMarker::ADD;
    raycastnofreeMarker.pose.orientation.w = 1;
    raycastnofreeMarker.color.r = 0.3;
    raycastnofreeMarker.color.g = 0.3;
    raycastnofreeMarker.color.b = 0.2;
    raycastnofreeMarker.color.a = 1.0;
    raycastnofreeMarker.scale.x = 0.05;
    raycastnofreeMarker.scale.y = 0.05;
    // raycastfreenoMarker.scale.z = 0.05;
}

void RandomGlobalPlanner::sendPathToLocalPlannerServer()
{
    //Take the calculated path, insert it into an action, in the goal (ExecutePath.action)
    upo_actions::ExecutePathGoal goal_action;
    printfTrajectory(trajectory, "sendPathToLocalPlannerServer: Trajectory_state_1");
    goal_action.path = trajectory;
    // if (use_catenary){
        for(int i= 0; i<randPlanner.length_catenary.size() ; i++){
            int n_ = (int)(randPlanner.length_catenary.size() - i - 1);
            goal_action.length_catenary.push_back(randPlanner.length_catenary[n_]);
        }
    // }

    // randPlanner.~RandomPlanner(); // Destroy the global planner, grid and vector to free memory
    
    execute_path_client_ptr->sendGoal(goal_action);
}

void RandomGlobalPlanner::publishMakePlanFeedback()
{
    float x = 0, y = 0, z = 0;
    x = (getRobotPoseUGV().transform.translation.x - goal.vector.x);
    y = (getRobotPoseUGV().transform.translation.y - goal.vector.y);
    z = (getRobotPoseUGV().transform.translation.z - goal.vector.z);

    dist2Goal.data = sqrtf(x * x + y * y + z * z);
    make_plan_fb.distance_to_goal = dist2Goal;

    travel_time.data = (ros::Time::now() - start_time);
    make_plan_fb.travel_time = travel_time;
    /**
     *! We can start with the top speed and the first information is t = e/v with v=linear_max_speed
     *! from the param server and e the length of the path. 
     *? Then with the info RETURNED by the local planner or indirecly by the tracker about distance travelled
     *? (not in straight line) and the duration of the travel till the moment -> Calculate average speed 
     *TODO: When the tracker or local planner reports that they could not continue, either because the path
     *TODO: is blocked, the goal is occupied, there is a emergency stop, etc. -> Set estimation to infinty
    **/
    float speed;
    nh->param("/nav_node/linear_max_speed", speed, (float)0.4);
    float time = pathLength / speed;
    int m = 0;
    float s;
    if (time < 60)
    {
        s = time;
    }
    else
    {
        while (time > 60)
        {
            time -= 60;
            m++;
        }
        s = time;
    }
    string data_ = "Estimated Time of Arrival " + to_string(m) + string(":") + to_string((int)s);
    make_plan_fb.ETA.data = data_;

    /**
     * ! To start you can calculate the CLOSEST global WAYPOINT, gave the total number of waypoints you 
     * ! can easily calculate the fraction of the path 
     * 
    **/
    float totalWaypoints = trajectory.points.size();
    float currentWaypoint = getClosestWaypoint();
    globalWaypoint.data = currentWaypoint;
    make_plan_fb.global_waypoint = globalWaypoint;
    float percent = (float)(currentWaypoint / totalWaypoints) * 100;
    data_ = to_string(percent) + string("%");
    make_plan_fb.percent_achieved.data = data_;
    make_plan_server_ptr->publishFeedback(make_plan_fb);
}

int RandomGlobalPlanner::getClosestWaypoint()
{
    geometry_msgs::Transform robotPose = getRobotPoseUGV().transform;
    int waypoint = 0;
    float dist = std::numeric_limits<float>::max();
    float robotDist;
    for (auto it = trajectory.points.cbegin(); it != trajectory.points.cend(); it++)
    {
        robotDist = sqrtf(((robotPose.translation.x - it->transforms[0].translation.x) * (robotPose.translation.x - it->transforms[0].translation.x) +
                           (robotPose.translation.y - it->transforms[0].translation.y) * (robotPose.translation.y - it->transforms[0].translation.y) +
                           (robotPose.translation.z - it->transforms[0].translation.z) * (robotPose.translation.z - it->transforms[0].translation.z)));

        if (robotDist < dist)
        {
            waypoint++;
            dist = robotDist;
        }
    }
    return waypoint;
}

void RandomGlobalPlanner::makePlanGoalCB()
{
   //Cancel previous executing plan
    execute_path_client_ptr->cancelAllGoals();
    clearMarkers();
    // nbrRotationsExec = 0;
    countImpossible = 0;
    timesReplaned = 0;
    make_plan_res.replan_number.data = 0;

    start_time = ros::Time::now();
    upo_actions::MakePlanGoalConstPtr goal_ptr = make_plan_server_ptr->acceptNewGoal();

    goalPoseStamped = goal_ptr->global_goal;

    goal.vector.x = goalPoseStamped.pose.position.x;
    goal.vector.y = goalPoseStamped.pose.position.y;
    goal.vector.z = goalPoseStamped.pose.position.z;
    goal.header = goalPoseStamped.header;

    ROS_INFO_COND(debug, "Global Planner: Called Make Plan");

    if (calculatePath())
    {
        ROS_INFO_COND(debug, PRINTF_RED "\n\n\n     Global Planner: Succesfully calculated path\n\n\n");
        sendPathToLocalPlannerServer();
    }
    else
    { // What if it isnt possible to calculate path?
        make_plan_res.not_possible = true;
        make_plan_res.finished = false;
        make_plan_server_ptr->setAborted(make_plan_res, "Impossible to calculate a solution");
    }
}

void RandomGlobalPlanner::makePlanPreemptCB()
{
    make_plan_res.finished = false;
    travel_time.data = ros::Time::now() - start_time;
    make_plan_res.time_spent = travel_time;

    make_plan_server_ptr->setPreempted(make_plan_res, "Goal Preempted by User");
    ROS_INFO("Global Planner: Make plan preempt cb: cancelling");
    execute_path_client_ptr->cancelAllGoals();
    clearMarkers();
}

void RandomGlobalPlanner::clearMarkers()
{
    waypointsMarker.action = RVizMarker::DELETEALL;
    lineMarker.action = RVizMarker::DELETEALL;

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);

    lineMarker.points.clear();
    waypointsMarker.points.clear();

    lineMarker.action = RVizMarker::ADD;
    waypointsMarker.action = RVizMarker::ADD;
}

void RandomGlobalPlanner::clearMarkersRayCast()
{
    fullrayMarker.action = RVizMarker::DELETEALL; 
    raycastfreeMarker.action = RVizMarker::DELETEALL;
    raycastfreereducedMarker.action = RVizMarker::DELETEALL; 
    raycastcollMarker.action = RVizMarker::DELETEALL; 
    raycastnofreeMarker.action = RVizMarker::DELETEALL;

    fullRayPublisher.publish(fullrayMarker); 
    rayCastFreePublisher.publish(raycastfreeMarker); 
    rayCastFreeReducedPublisher.publish(raycastfreereducedMarker); 
    rayCastCollPublisher.publish(raycastcollMarker); 
    rayCastNoFreePublisher.publish(raycastnofreeMarker); 
    
    fullrayMarker.points.clear();
    raycastfreeMarker.points.clear();
    raycastfreereducedMarker.points.clear();
    raycastcollMarker.points.clear();
    raycastnofreeMarker.points.clear();


    fullrayMarker.action = RVizMarker::ADD; 
    raycastfreeMarker.action = RVizMarker::ADD;
    raycastfreereducedMarker.action = RVizMarker::ADD; 
    raycastcollMarker.action = RVizMarker::ADD; 
    raycastnofreeMarker.action = RVizMarker::ADD;
}

// This is the main function executed in loop
void RandomGlobalPlanner::plan()
{
    //TODO Maybe I can change the goalRunning flag by check if is active any goal

    if (make_plan_server_ptr->isActive())
    {
        publishMakePlanFeedback();
    }
    else
    {
        return;
    }

    // printf(PRINTF_CYAN"execute_path_client_ptr->getState().isDone()=[%s]\n",execute_path_client_ptr->getState().isDone()? "true":"false")
    if (execute_path_client_ptr->getState().isDone())
    {
        // clearMarkers();
        // printf(PRINTF_ORANGE"I AM HERE: Inside  IF Loop  RandomGlobalPlanner::plan()\n");

        if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            make_plan_res.finished = true;
            make_plan_res.not_possible = false;
            make_plan_res.replan_number.data = timesReplaned;
            make_plan_res.time_spent.data = (ros::Time::now() - start_time);
            make_plan_server_ptr->setSucceeded(make_plan_res);
        }

        if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_INFO("Global Planner: Path execution aborted by local planner...");
            replan();
        } //!It means that local planner couldn t find a local solution

        if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
        { //!Maybe when the goal is inside the local workspace and occupied

            ROS_INFO("Global Planner: Path execution preempted by local planner");
            replan();
            //Decide What to do next....
        }
        if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::REJECTED)
        { //!Maybe when the goal is inside the local workspace and occupied

            ROS_INFO("Global Planner: Path execution rejected by local planner");
            replan();
            //Decide What to do next....
        }
    }
}

bool RandomGlobalPlanner::replan()
{
    make_plan_res.replan_number.data++;

    //For the world to know the planner is replanning
    flg_replan_status.data = true;
    replan_status_pub.publish(flg_replan_status);

    if (calculatePath())
    {
        ++timesReplaned;
        ROS_INFO_COND(debug, "Global Planner: Succesfully calculated path");
        sendPathToLocalPlannerServer();
        return true;
    }
    else if (timesReplaned > 5)
    {
        timesReplaned = 0;
        make_plan_res.finished = false;
        make_plan_res.not_possible = true;

        flg_replan_status.data = false;
        replan_status_pub.publish(flg_replan_status);

        make_plan_server_ptr->setAborted(make_plan_res, "Tried to replan and aborted after replanning 5 times");
        execute_path_client_ptr->cancelAllGoals();
        return false;
    }
}

bool RandomGlobalPlanner::calculatePath()
{
    //It seems that you can get a goal and no map and try to get a path but setGoal and setStart will check if the points are valid
    //so if there is no map received it won't calculate a path
    bool ret = false;

    if (use3d && !mapRec)
        return ret;

    randPlanner.clearStatus(); 
    std_msgs::Bool clean_markers_optimizer_;
    clean_markers_optimizer_.data = true;
    cleanMarkersOptimizerPublisher.publish(clean_markers_optimizer_); 
    // if (isMarsupialCoupled()){ 

        if (setGoal() && setStart())
        {
            //Print succes start and goal points
            ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: Goal and start successfull set");
                
            // Path calculation
            ftime(&start);
           
            if (coupled)
                number_of_points = randPlanner.computeTreeCoupled();
            else
                number_of_points = randPlanner.computeTreesIndependent();

            ftime(&finish);

            seconds = finish.time - start.time - 1;
            milliseconds = (1000 - start.millitm) + finish.millitm;

            if (write_data_for_analysis){
                std::ofstream ofs_ugv, ofs_uav;
                // std::string output_file = path + "time_compute_initial_planner.txt";
                std::string output_file_ugv, output_file_uav;
	            output_file_ugv = path+name_output_file+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_goal_"+std::to_string(num_goal)+"_UGV"+".txt";
	            output_file_uav = path+name_output_file+"_stage_"+std::to_string(scenario_number)+"_InitPos_"+std::to_string(num_pos_initial)+"_goal_"+std::to_string(num_goal)+"_UAV"+".txt";

                ofs_ugv.open(output_file_ugv.c_str(), std::ofstream::app);
                if (ofs_ugv.is_open()) {
                    std::cout << "Saving time initial planning data in output file ugv: " << output_file_ugv << std::endl;
                    ofs_ugv << (milliseconds + seconds * 1000.0)/1000.0 <<";";
                } 
                else {
                    std::cout << "Couldn't be open the output data file for time initial planning ugv" << std::endl;
                }
                ofs_ugv.close();

                ofs_uav.open(output_file_uav.c_str(), std::ofstream::app);
                if (ofs_uav.is_open()) {
                    std::cout << "Saving time initial planning data in output file uav: " << output_file_uav << std::endl;
                    ofs_uav << (milliseconds + seconds * 1000.0)/1000.0 <<";";
                } 
                else {
                    std::cout << "Couldn't be open the output data file for time initial planning uav" << std::endl;
                }
                ofs_uav.close();

            }

            ROS_INFO(PRINTF_YELLOW "Global Planner: Time Spent in Global Path Calculation: %.1f ms", milliseconds + seconds * 1000);
            ROS_INFO(PRINTF_YELLOW "Global Planner: Number of points: %d", number_of_points);

            if (number_of_points > 0)
            {
                ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: Publishing trajectory");
                    
                publishTrajectory();

                // if (pathLength < minPathLenght)
                // {
                    // execute_path_client_ptr->cancelAllGoals();
                    // make_plan_server_ptr->setSucceeded();
                // }
                //Reset the counter of the number of times the planner tried to calculate a path without success
                countImpossible = 0;
                //If it was replanning before, reset flag
                if (flg_replan_status.data)
                {
                    flg_replan_status.data = false;
                    replan_status_pub.publish(flg_replan_status);
                }
                    ret = true;
            }
            else{
                countImpossible++;
            }
        }

    // }

    return ret;
}

void RandomGlobalPlanner::calculatePathLength()
{
    float x, y, z;
    pathLength = 0;
    for (size_t it = 0; it < (trajectory.points.size() - 1); it++)
    {
        x = trajectory.points[it].transforms[0].translation.x - trajectory.points[it + 1].transforms[0].translation.x;
        y = trajectory.points[it].transforms[0].translation.y - trajectory.points[it + 1].transforms[0].translation.y;
        z = trajectory.points[it].transforms[0].translation.z - trajectory.points[it + 1].transforms[0].translation.z;

        pathLength += sqrtf(x * x + y * y + z * z);
    }
    ROS_INFO_COND(debug,PRINTF_MAGENTA"Global path lenght: %f, number of points: %d", pathLength, (int)trajectory.points.size());

}

geometry_msgs::TransformStamped RandomGlobalPlanner::getRobotPoseUGV()
{
    geometry_msgs::TransformStamped ret;

    try
    {
        ret = tfBuffer->lookupTransform(world_frame, ugv_base_frame, ros::Time(0));
        
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Global Planner: Couldn't get UGV Pose (frame: %s), so not possible to set UGV start point; tf exception: %s", ugv_base_frame.c_str(),ex.what());
    }
    return ret;
}

geometry_msgs::TransformStamped RandomGlobalPlanner::getRobotPoseUAV()
{
    geometry_msgs::TransformStamped ret;

    try
    {
        ret = tfBuffer->lookupTransform(world_frame, uav_base_frame, ros::Time(0));
        
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Global Planner: Couldn't get UAV Pose (frame: %s), so not possible to set UAV start point; tf exception: %s", uav_base_frame.c_str(), ex.what());
    }
    return ret;
}

void RandomGlobalPlanner::publishTrajectory()
{
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = world_frame;
    trajectory.header.seq = ++seq;
    trajectory.points.clear();

    ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner 3D: Trajectory calculation... POINTS: %d",number_of_points);

    geometry_msgs::TransformStamped transform_robot_pose = getRobotPoseUGV();

    if (number_of_points > 1)
    {
        randPlanner.getTrajectory(trajectory);        
        printfTrajectory(trajectory, "After getting traj: ");
   
    }
    else if (number_of_points == 1)
    {
        // randPlanner.getTrajectoryYawFixed(trajectory, getYawFromQuat(transform_robot_pose.transform.rotation));
        trajectory_msgs::MultiDOFJointTrajectoryPoint pos;
        // pos.transforms.push_back(transform_robot_pose.transform);
        // trajectory.points.push_back(pos);
    }

    // Send the trajectory
    // Trajectory solution visualization marker

    trajectory_msgs::MultiDOFJointTrajectoryPoint goal_multidof;
    geometry_msgs::Transform transform_goal;

    transform_goal.rotation = goalPoseStamped.pose.orientation;
    transform_goal.translation.x = goalPoseStamped.pose.position.x;
    transform_goal.translation.y = goalPoseStamped.pose.position.y;
    transform_goal.translation.z = goalPoseStamped.pose.position.z;

    goal_multidof.transforms.resize(1, transform_goal);

    // trajectory.points.push_back(goal_multidof);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //!Calculate path length:
    // calculatePathLength();

    // clearMarkers();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    lineMarker.header.stamp = ros::Time::now();
    waypointsMarker.header.stamp = ros::Time::now();

    geometry_msgs::Point p;

    ROS_INFO("trajectory.points.size()=[%lu]",trajectory.points.size());
    for (size_t i = 0; i < trajectory.points.size(); i++)
    {
        p.x = trajectory.points[i].transforms[0].translation.x;
        p.y = trajectory.points[i].transforms[0].translation.y;
        p.z = trajectory.points[i].transforms[0].translation.z;

        lineMarker.points.push_back(p);
        waypointsMarker.points.push_back(p);
    }

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);
}

bool RandomGlobalPlanner::setGoal()
{
    bool ret = false;

    if (randPlanner.setValidFinalPosition(goal.vector))
    {
        ret = true;
    }
    else
    {
        ROS_ERROR("Global Planner: Failed to set final global position: [%.2f, %.2f, %.2f] ", goal.vector.x, goal.vector.y, goal.vector.z);
    }

    return ret;
}

bool RandomGlobalPlanner::setStart()
{
    geometry_msgs::Vector3Stamped start_ugv_, start_uav_;
    geometry_msgs::QuaternionStamped q_start_ugv_, q_start_uav_;
    bool ret = false;

    geometry_msgs::TransformStamped position_ugv_,position_uav_;
    position_ugv_ = getRobotPoseUGV();
    position_uav_ = getRobotPoseUAV();

    start_ugv_.vector.x = position_ugv_.transform.translation.x;
    start_ugv_.vector.y = position_ugv_.transform.translation.y;
    // start_ugv_.vector.z = position_ugv_.transform.translation.z + map_v_inflaction + map_resolution; //Added to be consecuent with the line 809, the the displacement of UGV is always apply
    start_ugv_.vector.z = position_ugv_.transform.translation.z ; 
    q_start_ugv_.quaternion.w = position_ugv_.transform.rotation.w;
    q_start_ugv_.quaternion.x = position_ugv_.transform.rotation.x;
    q_start_ugv_.quaternion.y = position_ugv_.transform.rotation.y;
    q_start_ugv_.quaternion.z = position_ugv_.transform.rotation.z;
    q_start_ugv_.header = position_ugv_.header;
    
    start_uav_.vector.x = position_uav_.transform.translation.x;
    start_uav_.vector.y = position_uav_.transform.translation.y;
    start_uav_.vector.z = position_uav_.transform.translation.z;
    q_start_uav_.quaternion.w = position_uav_.transform.rotation.w;
    q_start_uav_.quaternion.x = position_uav_.transform.rotation.x;
    q_start_uav_.quaternion.y = position_uav_.transform.rotation.y;
    q_start_uav_.quaternion.z = position_uav_.transform.rotation.z;
    q_start_uav_.header = position_uav_.header;
    printf("setStart position:  ugv[%f %f %f / %f %f %f %f]  uav[%f %f %f / %f %f %f %f]\n",
            start_ugv_.vector.x, start_ugv_.vector.y, start_ugv_.vector.z, 
            q_start_ugv_.quaternion.x , q_start_ugv_.quaternion.y, q_start_ugv_.quaternion.z, q_start_ugv_.quaternion.w,
            start_uav_.vector.x, start_uav_.vector.y, start_uav_.vector.z, 
            q_start_uav_.quaternion.x, q_start_uav_.quaternion.y, q_start_uav_.quaternion.z, q_start_uav_.quaternion.w);

    // if (start_ugv_.vector.z <= ws_z_min)
    //     start_ugv_.vector.z = ws_z_min + map_v_inflaction + map_resolution;

    if (randPlanner.setValidInitialPositionMarsupial(start_ugv_.vector,start_uav_.vector, q_start_ugv_.quaternion, q_start_uav_.quaternion))
    {
        ROS_INFO(PRINTF_MAGENTA "Global Planner 3D: Found a free initial UGV position): [%.2f, %.2f, %.2f]", start_ugv_.vector.x, start_ugv_.vector.y, start_ugv_.vector.z);
        ret = true;
    }
    else
    {
        ROS_ERROR("Global Planner 3D: Failed to set UGV initial global position(after search around): [%.2f, %.2f, %.2f]", start_ugv_.vector.x, start_ugv_.vector.y, start_ugv_.vector.z);
    }

    return ret;
}

void RandomGlobalPlanner::configRandomPlanner()
{
    nh->param("length_tether_max", length_tether_max, (double)10.0);

    geometry_msgs::Vector3 pos_reel_ugv, pos_ugv_;
    geometry_msgs::Quaternion rot_ugv_;
    pos_reel_ugv.x = pos_reel_x;
    pos_reel_ugv.y = pos_reel_y;
    pos_reel_ugv.z = pos_reel_z;
    pos_ugv_ = getRobotPoseUGV().transform.translation;
    rot_ugv_ = getRobotPoseUGV().transform.rotation;
    // bool coupled = isMarsupialCoupled();
    randPlanner.configRRTParameters(length_tether_max, pos_reel_ugv , pos_ugv_, rot_ugv_, coupled , n_iter, n_loop, radius_near_nodes, 
                                   step_steer, samp_goal_rate, sample_mode, do_steer_ugv, min_l_steer_ugv);

   	ROS_INFO(PRINTF_GREEN"Global Planner  configRandomPlanner() :  length_tether_max: %f , sample_mode=%i !!", length_tether_max, sample_mode);
	ROS_INFO(PRINTF_GREEN"Global Planner  configRandomPlanner() :  is_coupled=[%s] debug_rrt=[%s] debug=[%s]", coupled? "true" : "false", debug_rrt? "true" : "false", debug? "true" : "false");
}

geometry_msgs::Vector3 RandomGlobalPlanner::tfListenerReel()
{
    geometry_msgs::Vector3 _p_reel;
	tf::StampedTransform _transform;

    listener.lookupTransform("map", "reel_base_link", ros::Time(0), _transform);

	_p_reel.x = _transform.getOrigin().x();
	_p_reel.y = _transform.getOrigin().y();
	_p_reel.z = _transform.getOrigin().z();

	return _p_reel;
}

bool RandomGlobalPlanner::isMarsupialCoupled()
{
    geometry_msgs::TransformStamped position_ugv_;
    geometry_msgs::TransformStamped position_uav_;

    position_ugv_ = getRobotPoseUGV();
    position_uav_ = getRobotPoseUAV();
    float diff_x = fabs(position_ugv_.transform.translation.x - position_uav_.transform.translation.x);
    float diff_y = fabs(position_ugv_.transform.translation.y - position_uav_.transform.translation.y);
    float diff_z = fabs(position_ugv_.transform.translation.z - position_uav_.transform.translation.z);

    bool coupled_;

    if (diff_z > 0.3 && diff_y > 0.1 && diff_x > 0.1)
        coupled_ = false;
    else
        coupled_ = true;

    return coupled_;
}

void RandomGlobalPlanner::printfTrajectory(Trajectory trajectory, string trajectory_name)
{
    printf(PRINTF_YELLOW "%s trajectory [%d]:\n", trajectory_name.c_str(), (int)trajectory.points.size());

    for (unsigned int i = 0; i < trajectory.points.size(); i++)
    {
        double yaw = getYawFromQuat(trajectory.points[i].transforms[0].rotation);
        printf(PRINTF_BLUE "\t %d: [%.3f, %.3f] m\t[%f] deg\t [%.2f] sec\n", i, trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.y, yaw / 3.141592 * 180, trajectory.points[i].time_from_start.toSec());
    }

    printf(PRINTF_REGULAR);
}

float RandomGlobalPlanner::getYawFromQuat(Quaternion quat)
{
	double r, p, y;
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 M(q);
	M.getRPY(r, p, y);

	return y;
}

} // namespace PathPlanners