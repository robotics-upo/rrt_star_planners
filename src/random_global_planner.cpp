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

    // std::string node_name_grid_ = "grid3D_node";
	grid_3D = new Grid3d(node_name);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "Initialazing Trilinear Interpolation (grid3D) in Global Planner");
	grid_3D->computeTrilinearInterpolation();
    ROS_INFO_COND(showConfig, PRINTF_GREEN "Finished Trilinear Interpolation (grid3D) in Global Planner");

	CheckCM = new CatenaryCheckerManager(node_name);
    
    configParams();
    configTopics();
    configServices();
    configRRTStar();
}

//This function gets parameter from param server at startup if they exists, if not it passes default values
void RandomGlobalPlanner::configParams()
{
    //At startup, no goal and no costmap received yet
    seq = 0;
    timesReplaned = 0;
    mapRec = false;

    //Get params from param server. If they dont exist give variables default values
    nh->param("show_config", showConfig, (bool)0);
    nh->param("planner_type", planner_type, (std::string)"rrt_star");
    nh->param("debug", debug, (bool)0);

    nh->param("ws_x_max", ws_x_max, (double)30);
    nh->param("ws_y_max", ws_y_max, (double)30);
    nh->param("ws_z_max", ws_z_max, (double)30);
    nh->param("ws_x_min", ws_x_min, (double)0);
    nh->param("ws_y_min", ws_y_min, (double)0);
    nh->param("ws_z_min", ws_z_min, (double)0);

    nh->param("save_path_in_file", save_path_in_file, (bool) false);

    nh->param("map_resolution", map_resolution, (double)0.05);
    nh->param("map_h_inflaction", map_h_inflaction, (double)0.05);
    nh->param("map_v_inflaction", map_v_inflaction, (double)0.05);

    nh->param("n_iter", n_iter, (int)100);
    nh->param("n_loop", n_loop, (int)1);
    nh->param("world_frame", world_frame, (string) "/map");
    nh->param("ugv_base_frame", ugv_base_frame, (string) "ugv_base_link");
    nh->param("uav_base_frame", uav_base_frame, (string) "uav_base_link");
    nh->param("reel_base_frame", reel_base_frame, (string) "reel_base_link");
    nh->param("radius_near_nodes", radius_near_nodes, (double)1.0);
    nh->param("step_steer", step_steer, (double)0.5);
    nh->param("goal_gap_m", goal_gap_m, (double)0.2);
    nh->param("min_l_steer_ugv", min_l_steer_ugv, (double)5.0);

    nh->param("distance_obstacle_ugv", distance_obstacle_ugv, (double)1.0);
    nh->param("distance_obstacle_uav", distance_obstacle_uav, (double)1.0);
    nh->param("distance_tether_obstacle", distance_tether_obstacle, (double)0.1);

    nh->param("length_tether_max", length_tether_max, (double)10.0);

	nh->param("min_distance_add_new_point", min_distance_add_new_point, (double)1.0);

	nh->param("w_nearest_ugv", w_nearest_ugv, (double)1.0);
	nh->param("w_nearest_uav", w_nearest_uav, (double)1.0);
	nh->param("w_nearest_smooth", w_nearest_smooth, (double)1.0);

  	nh->param("write_data_for_analysis",write_data_for_analysis, (bool)0);
	nh->param("path", path, (std::string) "~/");
	nh->param("map_file", map_file, (std::string) "my_map");

    nh->param("sample_mode", sample_mode, (int)1); // value 1 or 2
    nh->param("do_steer_ugv", do_steer_ugv, (bool)true);
    nh->param("coupled", coupled, (bool)true);
    nh->param("samp_goal_rate", samp_goal_rate, (int)10);
    nh->param("debug_rrt", debug_rrt, (bool)true);
    nh->param("nodes_marker_debug", nodes_marker_debug, (bool)true);
    nh->param("pause_execution", pause_execution, (bool)true);
    nh->param("use_distance_function", use_distance_function, (bool)true); //Only related with tether and UAV distance
    nh->param("just_line_of_sigth", just_line_of_sigth, (bool)false); //Only related with tether and UAV distance
    
    nh->param("get_catenary_data", get_catenary_data, (bool)false);
    nh->param("catenary_file", catenary_file, (std::string) "catenary_file");
    nh->param("use_parable", use_parable, (bool)false);
    catenary_analysis_file = path + catenary_file + ".txt";

	nh->param("name_output_file", name_output_file, (std::string) "optimization_test");
	nh->param("num_pos_initial", num_pos_initial,(int)1);

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner 3D Node Configuration:");
    ROS_INFO_COND(showConfig, PRINTF_GREEN "   Workspace = X: [%.2f, %.2f]\t Y: [%.2f, %.2f]\t Z: [%.2f, %.2f]  ", ws_x_max, ws_x_min, ws_y_max, ws_y_min, ws_z_max, ws_z_min);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "   World frame: %s, UGV base frame: %s, UAV base frame: %s ", world_frame.c_str(), ugv_base_frame.c_str(), uav_base_frame.c_str());
}

void RandomGlobalPlanner::configRRTStar()
{
    randPlanner.init(planner_type, world_frame, ws_x_max, ws_y_max, ws_z_max, ws_x_min, ws_y_min, ws_z_min, map_resolution, map_h_inflaction, map_v_inflaction, 
                    nh, goal_gap_m, debug_rrt, distance_obstacle_ugv, distance_obstacle_uav, distance_tether_obstacle, grid_3D, nodes_marker_debug, use_distance_function,
                    map_file, path, get_catenary_data, catenary_analysis_file, use_parable, just_line_of_sigth);
    configRandomPlanner();

    CheckCM->Init(grid_3D, distance_tether_obstacle, distance_obstacle_ugv, distance_obstacle_uav, length_tether_max, ws_z_min, map_resolution, 
	use_parable, use_distance_function, pos_reel_ugv, just_line_of_sigth, true);
}

void RandomGlobalPlanner::configTopics()
{
    replan_status_pub = nh->advertise<std_msgs::Bool>("replanning_status", 1);
    clean_markers_optimizer_pub_ = nh->advertise<std_msgs::Bool>("/clean_marker_optimizer", 1);
    interpolated_path_ugv_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("interpolated_path_ugv_rrt_star", 2, true);
	interpolated_path_uav_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("interpolated_path_uav_rrt_star", 2, true);
    interpolated_catenary_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("interpolated_catenary_marsupial", 1000, true);
    initial_catenary_pub_ = nh->advertise<visualization_msgs::MarkerArray>("catenary_marsupial", 1000, true);

    sub_map = nh->subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &RandomGlobalPlanner::collisionMapCallBack, this);
    clean_nodes_marker_gp_sub_ = nh->subscribe("/clean_nodes_marker_gp", 1, &RandomGlobalPlanner::deleteNodesMarkersCallBack, this);
    clean_catenary_marker_gp_sub_ = nh->subscribe("/clean_catenary_marker_gp", 1, &RandomGlobalPlanner::deleteCatenaryGPCallBack, this);
    point_cloud_map_trav_sub_ = nh->subscribe( "/region_growing_traversability_pc_map", 1,  &RandomGlobalPlanner::readPointCloudTraversabilityMapCallback, this);
    point_cloud_map_ugv_sub_ = nh->subscribe( "/region_growing_obstacles_pc_map", 1,  &RandomGlobalPlanner::readPointCloudUGVObstaclesMapCallback, this);
    point_cloud_map_uav_sub_ = nh->subscribe( "/octomap_point_cloud_centers", 1,  &RandomGlobalPlanner::readPointCloudUAVObstaclesMapCallback, this);

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner 3D Topics and Subscriber Configurated");
}

void RandomGlobalPlanner::configServices()
{
    make_plan_server_ptr.reset(new MakePlanServer(*nh, "/Make_Plan", false));
    make_plan_server_ptr->registerGoalCallback(boost::bind(&RandomGlobalPlanner::makePlanGoalCB, this));
    make_plan_server_ptr->registerPreemptCallback(boost::bind(&RandomGlobalPlanner::makePlanPreemptCB, this));
    make_plan_server_ptr->start();

    if (!save_path_in_file){
        execute_path_client_ptr.reset(new ExecutePathClient("/Execute_Plan", true));
        execute_path_client_ptr->waitForServer();
    }

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner 3D: Action client from global planner ready");
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
    pc_obs_ugv = msg;
    ROS_INFO_COND(debug, PRINTF_GREEN "Global Planner: UGV Obstacles Map Navigation Received");
}

void RandomGlobalPlanner::readPointCloudUAVObstaclesMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    randPlanner.readPointCloudMapForUAV(msg);
    ROS_INFO_COND(debug, PRINTF_GREEN "Global Planner: UAV Obstacles Map Navigation Received");
}

void RandomGlobalPlanner::deleteNodesMarkersCallBack(const std_msgs::Bool::ConstPtr &msg)
{
	if (msg->data == true)
		randPlanner.clearNodesMarker();
}

void RandomGlobalPlanner::deleteCatenaryGPCallBack(const std_msgs::Bool::ConstPtr &msg)
{
	if (msg->data == true){
        randPlanner.clearCatenaryGPMarker();
        randPlanner.clearLinesGPMarker();
		clearCatenaryGPMarker();
    }
}

void RandomGlobalPlanner::makePlanGoalCB()
{
   //Cancel previous executing plan
    if (!save_path_in_file)
        execute_path_client_ptr->cancelAllGoals();
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

    clearLinesGPMarker();

    ROS_INFO_COND(debug, "Global Planner: Called Make Plan");

    if (calculatePath()){
        ROS_INFO(PRINTF_BLUE "\n\n     \t\t\t\tGlobal Planner: Succesfully calculated Global Path\n");
        ros::Duration(2.0).sleep();

        ROS_INFO(PRINTF_YELLOW "Global Planner: Number of points in path before interpolation: %lu ", trajectory.points.size());
        
        if (CheckCM->count_tether_coll){ // Here is update vector with collision tether status
            v_pos_coll_tether.clear();
            for (size_t i= 0 ; i < CheckCM->v_pos_coll_tether.size(); i++){
                v_pos_coll_tether = CheckCM->v_pos_coll_tether;
            }
        }

        /********************* To obligate pause method and check Planning result *********************/
                //    std::string y_ ;
                //    std::cout << " *** Graphed Path from RRT* : Press key to continue: " << std::endl;
                //    std::cin >> y_ ;
        /*************************************************************************************************/
             
        interpolatePointsGlobalPath(trajectory, randPlanner.length_catenary);
        for (size_t i=0; i< trajectory.points.size(); i++){
				printf("\tRandom_gobal_planner_node[%lu/%lu] :  ugv=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f]  uav=[%.3f %.3f %.3f / %.3f %.3f %.3f %.3f]  length_catenary=%.3f  params_cat=[%.3f %.3f %.3f]\n", 
                i, trajectory.points.size(),
				trajectory.points.at(i).transforms[0].translation.x, trajectory.points.at(i).transforms[0].translation.y, trajectory.points.at(i).transforms[0].translation.z, 
                trajectory.points.at(i).transforms[0].rotation.x, trajectory.points.at(i).transforms[0].rotation.y, trajectory.points.at(i).transforms[0].rotation.z, trajectory.points.at(i).transforms[0].rotation.w,
                trajectory.points.at(i).transforms[1].translation.x, trajectory.points.at(i).transforms[1].translation.y, trajectory.points.at(i).transforms[1].translation.z, 
                trajectory.points.at(i).transforms[1].rotation.x, trajectory.points.at(i).transforms[1].rotation.y, trajectory.points.at(i).transforms[1].rotation.z, trajectory.points.at(i).transforms[1].rotation.w,
                length_catenary[i], v_params_catenary[i].x, v_params_catenary[i].y, v_params_catenary[i].z);
		}

        ROS_INFO(PRINTF_YELLOW "\nGlobal Planner: Number of points in path after interpolation: %lu", trajectory.points.size());
        
        // randPlanner.clearCatenaryGPMarker();
        // randPlanner.clearLinesGPMarker();
        rrtgm.getPathMarker(trajectory,length_catenary, interpolated_path_ugv_marker_pub_, interpolated_path_uav_marker_pub_, interpolated_catenary_marker_pub_);

        ROS_INFO_COND(debug, PRINTF_YELLOW "\n\n     \t\t\t\tGlobal Planner: Succesfully calculated Interpolated Global Path\n");

        // /********************* To obligate pause method and check Planning result *********************/
        //            std::string yy_ ;
        //            std::cout << " *** Graphed Intepolated Path : Press key to continue: " << std::endl;
        //            std::cin >> yy_ ;
        /*************************************************************************************************/

        sendPathToLocalPlannerServer();
    }
    else{ 
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
    if (!save_path_in_file)
        execute_path_client_ptr->cancelAllGoals();
}

// This is the main function executed in loop
void RandomGlobalPlanner::plan()
{
    //TODO Maybe I can change the goalRunning flag by check if is active any goal
    if (!make_plan_server_ptr->isActive())
        return;

    if (!save_path_in_file){
        if (execute_path_client_ptr->getState().isDone())
        {
            if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                make_plan_res.finished = true;
                make_plan_res.not_possible = false;
                make_plan_res.replan_number.data = timesReplaned;
                make_plan_res.time_spent.data = (ros::Time::now() - start_time);
                make_plan_server_ptr->setSucceeded(make_plan_res);
            }

            if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::ABORTED){
                ROS_INFO("Global Planner: Path execution aborted by local planner...");
                replan();
            } //!It means that local planner couldn t find a local solution

            if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::PREEMPTED){//!Maybe when the goal is inside the local workspace and occupied
                ROS_INFO("Global Planner: Path execution preempted by local planner");
                replan();
            }
            if (execute_path_client_ptr->getState() == actionlib::SimpleClientGoalState::REJECTED){//!Maybe when the goal is inside the local workspace and occupied
                ROS_INFO("Global Planner: Path execution rejected by local planner");
                replan();
            }
        }
    }
}

void RandomGlobalPlanner::replan()
{
    make_plan_res.replan_number.data++;

    //For the world to know the planner is replanning
    flg_replan_status.data = true;
    replan_status_pub.publish(flg_replan_status);

    if (calculatePath()){
        ++timesReplaned;
        ROS_INFO_COND(debug, "Global Planner: Succesfully calculated path");
        sendPathToLocalPlannerServer();
        // return true;
    }
    else if (timesReplaned > 5)
    {
        timesReplaned = 0;
        make_plan_res.finished = false;
        make_plan_res.not_possible = true;

        flg_replan_status.data = false;
        replan_status_pub.publish(flg_replan_status);

        make_plan_server_ptr->setAborted(make_plan_res, "Tried to replan and aborted after replanning 5 times");
   
        if (!save_path_in_file)
            execute_path_client_ptr->cancelAllGoals();
    }
}

bool RandomGlobalPlanner::calculatePath()
{
    bool ret = false;

    if (use3d && !mapRec)
        return ret;

    randPlanner.clearStatus(); 
    std_msgs::Bool clean_markers_optimizer_;
    clean_markers_optimizer_.data = true;
    clean_markers_optimizer_pub_.publish(clean_markers_optimizer_); 
        if (setGoal() && setStart())
        {
            //Print succes start and goal points
            ROS_INFO_COND(debug, PRINTF_MAGENTA "Global Planner: Goal and start successfull set");
                
            // Path calculation
            clock_gettime(CLOCK_REALTIME, &start);
           
            if (coupled)
                number_of_points = randPlanner.computeTreeCoupled();
            else
                number_of_points = randPlanner.computeTreesIndependent();

            clock_gettime(CLOCK_REALTIME, &finish);
        	ros::Duration(2.0).sleep();

            seconds = finish.tv_sec - start.tv_sec - 1;
            milliseconds = (1000000000 - start.tv_nsec) + finish.tv_nsec;

            if (write_data_for_analysis){
                std::ofstream ofs_ugv, ofs_uav, ofs_time;
                std::string output_file_ugv, output_file_uav, time_random_planner;
	            output_file_ugv = path+"results_"+map_file+"_InitPos_"+std::to_string(num_pos_initial)+"_"+name_output_file+"_UGV"+".txt";
	            output_file_uav = path+"results_"+map_file+"_InitPos_"+std::to_string(num_pos_initial)+"_"+name_output_file+"_UAV"+".txt";

                float time_compute_GP = (milliseconds + seconds * 1000000000.0)/1000000000.0;
                
                //Save Time in UGV analize Path and Trajectory
                std::ifstream ifile1;
                ifile1.open(output_file_ugv);
                if(ifile1) {
                    std::cout << output_file_ugv <<" : File doesn't exists !!!!!!!!!! " << std::endl;
                } else {
                ofs_ugv.open(output_file_ugv.c_str(), std::ofstream::app);
                // ofs_ugv <<"tCGP;tCO;dTI;dTO;tTI;tTO;mean_doI;min_doI;mean_doO;min_doO;mean_dcI;min_dcI;mean_dcO;min_dcO;mean_vTI;max_vTI;mean_vTO;max_vTO;mean_aTI;max_aTI;mean_aTO;max_aTO;count_c;count_coll_U;pos_coll_I;count_coll_O;pos_coll_O"<<std::endl;
                ofs_ugv <<"TCGP;TCO;tTO;Mean_DOO;Min_DOO;Mean_DPO;Min_DPO;Mean_VTO;Max_VTO;Mean_ATO;Max_ATO;Count_CP"<<std::endl;
                ofs_ugv.close();
                std::cout << output_file_ugv <<" : File exist !!!!!!!!!! " << std::endl;
                }
                
                ofs_ugv.open(output_file_ugv.c_str(), std::ofstream::app);
                if (ofs_ugv.is_open()) {
                    std::cout << "Saving time initial planning data in output file ugv: " << output_file_ugv << std::endl;
                    ofs_ugv << time_compute_GP <<";";
                } 
                else 
                    std::cout << "Couldn't be open the output data file for time initial planning ugv" << std::endl;
                ofs_ugv.close();

                //Save Time in UAV analize Path and Trajectory
                std::ifstream ifile2;
                ifile2.open(output_file_uav);
                if(ifile2) {
                    std::cout << output_file_uav <<" : File exists !!!!!!!!!! " << std::endl;
                } else {
                ofs_uav.open(output_file_uav.c_str(), std::ofstream::app);
                // ofs_uav <<"tCGP;tCO;dTI;dTO;tTI;tTO;mean_doI;min_doI;mean_doO;min_doO;mean_dcI;min_dcI;mean_dcO;min_dcO;mean_vTI;max_vTI;mean_vTO;max_vTO;mean_aTI;max_aTI;mean_aTO;max_aTO;count_coll_I;pos_coll_I;count_coll_O;pos_coll_O"<<std::endl;
                ofs_uav <<"TCGP;TCO;tTO;Mean_DOO;Min_DOO;Mean_DPO;Min_DPO;Mean_VTO;Max_VTO;Mean_ATO;Max_ATO;Count_CP"<<std::endl;
                ofs_uav.close();
                std::cout << output_file_uav <<" : File doesn't exist !!!!!!!!!! " << std::endl;
                }
                ofs_uav.open(output_file_uav.c_str(), std::ofstream::app);
                if (ofs_uav.is_open()) {
                    std::cout << "Saving time initial planning data in output file uav: " << output_file_uav << std::endl;
                    ofs_uav << time_compute_GP <<";";
                } 
                else 
                    std::cout << "Couldn't be open the output data file for time initial planning uav" << std::endl;
                ofs_uav.close();
            }
            ROS_INFO(PRINTF_YELLOW "Global Planner: Time Spent in Global Path Calculation: %.1f ms", milliseconds + seconds * 1000);
            ROS_INFO(PRINTF_YELLOW "Global Planner: Number of points in path: %d", number_of_points);

            if (number_of_points > 0){
                trajectory.header.stamp = ros::Time::now();
                trajectory.header.frame_id = world_frame;
                trajectory.header.seq = ++seq;
                trajectory.points.clear();

                randPlanner.getGlobalPath(trajectory);    
                randPlanner.getParamsCatenary(v_params_catenary);    
                
                countImpossible = 0;    //Reset the counter of the number of times the planner tried to calculate a path without success
                if (flg_replan_status.data)     //If it was replanning before, reset flag
                {
                    flg_replan_status.data = false;
                    replan_status_pub.publish(flg_replan_status);
                }
                ret = true;
            }
            else
                countImpossible++;
        }
    return ret;
}

void RandomGlobalPlanner::sendPathToLocalPlannerServer()
{
    //Take the calculated path, insert it into an action, in the goal (ExecutePath.action)
    upo_actions::ExecutePathGoal goal_action;
    goal_action.path = trajectory;
    for(int i= 0; i<length_catenary.size() ; i++){
        goal_action.length_catenary.push_back(length_catenary[i]);
        goal_action.cat_param_x0.push_back(v_params_catenary[i].x);
        goal_action.cat_param_y0.push_back(v_params_catenary[i].y);
        goal_action.cat_param_a.push_back(v_params_catenary[i].z);
    }

    if (!save_path_in_file)
        execute_path_client_ptr->sendGoal(goal_action);
    else{
        ExportPath ep_(goal_action, path);
    }
}

bool RandomGlobalPlanner::setGoal()
{
    bool ret = false;
    if (randPlanner.setValidFinalPosition(goal.vector))
        ret = true;
    else
        ROS_ERROR("Global Planner: Failed to set final global position: [%.2f, %.2f, %.2f] ", goal.vector.x, goal.vector.y, goal.vector.z);

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

    if (randPlanner.setValidInitialPositionMarsupial(start_ugv_.vector,start_uav_.vector, q_start_ugv_.quaternion, q_start_uav_.quaternion)){
        ROS_INFO(PRINTF_MAGENTA "Global Planner 3D: Found a free initial UGV position): [%.2f, %.2f, %.2f]", start_ugv_.vector.x, start_ugv_.vector.y, start_ugv_.vector.z);
        ret = true;
    }else
        ROS_ERROR("Global Planner 3D: Failed to set UGV initial global position(after search around): [%.2f, %.2f, %.2f]", start_ugv_.vector.x, start_ugv_.vector.y, start_ugv_.vector.z);

    return ret;
}

geometry_msgs::TransformStamped RandomGlobalPlanner::getRobotPoseUGV()
{
    geometry_msgs::TransformStamped ret;
    try{
        ret = tfBuffer->lookupTransform(world_frame, ugv_base_frame, ros::Time(0));
    }    catch (tf2::TransformException &ex){
        ROS_WARN("Global Planner: Couldn't get UGV Pose (frame: %s), so not possible to set UGV start point; tf exception: %s", ugv_base_frame.c_str(),ex.what());
    }
    return ret;
}

geometry_msgs::TransformStamped RandomGlobalPlanner::getLocalPoseReel()
{
    geometry_msgs::TransformStamped ret;
    try{
        ret = tfBuffer->lookupTransform(ugv_base_frame, reel_base_frame,ros::Time(0));
    }    catch (tf2::TransformException &ex){
        ROS_WARN("Global Planner: Couldn't get Local Pose Reel(frame: %s), so not possible to set start point Catenary; tf exception: %s", reel_base_frame.c_str(),ex.what());
    }
    return ret;
}

geometry_msgs::TransformStamped RandomGlobalPlanner::getRobotPoseUAV()
{
    geometry_msgs::TransformStamped ret;
    try{
        ret = tfBuffer->lookupTransform(world_frame, uav_base_frame, ros::Time(0));
    }catch (tf2::TransformException &ex){
        ROS_WARN("Global Planner: Couldn't get UAV Pose (frame: %s), so not possible to set UAV start point; tf exception: %s", uav_base_frame.c_str(), ex.what());
    }
    return ret;
}

void RandomGlobalPlanner::interpolatePointsGlobalPath(Trajectory &trajectory_, std::vector<double> l_catenary_)
{
    float x_ugv_, y_ugv_, z_ugv_, x_uav_, y_uav_, z_uav_;
	double D_ugv_, D_uav_;

    Trajectory trajectory_aux_;
	length_catenary.clear();
    trajectory_msgs::MultiDOFJointTrajectoryPoint t_;
    t_.transforms.resize(2);
	t_.velocities.resize(2);
	t_.accelerations.resize(2);

	bisectionCatenary bc;

    vector<Vector3> points_cat_;
    vector<Vector3> v_params_catenary_aux_;
    v_params_catenary_aux_.clear();
    Vector3 params_;

    for (size_t i = 0; i < trajectory_.points.size()-1; i++)
    {
		// Get position and rotation vector for UGV
		x_ugv_ = trajectory_.points.at(i+1).transforms[0].translation.x - trajectory_.points.at(i).transforms[0].translation.x;
		y_ugv_ = trajectory_.points.at(i+1).transforms[0].translation.y - trajectory_.points.at(i).transforms[0].translation.y;
		z_ugv_ = trajectory_.points.at(i+1).transforms[0].translation.z - trajectory_.points.at(i).transforms[0].translation.z;
        
		// Get position and rotation vector for UAV
		x_uav_ = trajectory_.points.at(i+1).transforms[1].translation.x - trajectory_.points.at(i).transforms[1].translation.x;
		y_uav_ = trajectory_.points.at(i+1).transforms[1].translation.y - trajectory_.points.at(i).transforms[1].translation.y;
		z_uav_ = trajectory_.points.at(i+1).transforms[1].translation.z - trajectory_.points.at(i).transforms[1].translation.z;
        
        D_ugv_ = sqrt(x_ugv_ * x_ugv_ + y_ugv_ * y_ugv_ + z_ugv_ * z_ugv_);
        D_uav_ = sqrt(x_uav_ * x_uav_ + y_uav_ * y_uav_ + z_uav_ * z_uav_);
		
        // int j_ = (int)(l_catenary_.size() - i - 1); // Auxuliar position length catenary
        int j_ = (int)(i); // Auxuliar position length catenary

        if (v_pos_coll_tether.size() > 0){
            for (size_t i_ = 0 ; i_ < v_pos_coll_tether.size() ; i_++){
                if ((v_pos_coll_tether[i_] == j_)){
                    double l_;
                    if(getTetherLength(trajectory_.points.at(i).transforms[0].translation, trajectory_.points.at(i).transforms[0].rotation, trajectory_.points.at(i).transforms[1].translation, l_)){
                        l_catenary_[j_] = l_;  
                    }
                }
            }           
        }

		//First analize if distance between points in bigger that the maximum distance to create a new point
		if (D_uav_ > min_distance_add_new_point || D_ugv_ > min_distance_add_new_point){
			double D_;
			if (D_uav_ >= D_ugv_)
				D_ = D_uav_;
			else	
				D_ = D_ugv_;
			
			int n_interval_ = ceil(D_/min_distance_add_new_point);	         
            
            // CatenaryCheckerManager ccpp(node_name, grid_3D, pc_obs_ugv, pos_reel_ugv, distance_obstacle_ugv, distance_obstacle_uav, distance_tether_obstacle);

			for(int j=0 ; j < n_interval_ ; j++){
				// Get position and rotation vector for UGV
                geometry_msgs::Vector3 p1_, p2_;
                geometry_msgs::Quaternion q1_;

                // Get position and rotation vector for UGV
                p1_.x = trajectory_.points.at(i).transforms[0].translation.x;
                p1_.y = trajectory_.points.at(i).transforms[0].translation.y;
                p1_.z = trajectory_.points.at(i).transforms[0].translation.z;
                q1_.x = trajectory_.points.at(i).transforms[0].rotation.x;
                q1_.y = trajectory_.points.at(i).transforms[0].rotation.y;
                q1_.z = trajectory_.points.at(i).transforms[0].rotation.z;
                q1_.w = trajectory_.points.at(i).transforms[0].rotation.w;
                p2_.x = p1_.x + (x_ugv_/n_interval_)*j ;
                p2_.y = p1_.y + (y_ugv_/n_interval_)*j ;
                p2_.z = p1_.z + (z_ugv_/n_interval_)*j +0.001; 

                ROS_INFO(PRINTF_GREEN"UGV[%lu] Test , interval[%i] point[%f %f %f]", i,j, p2_.x, p2_.y, p2_.z);
                bool is_ugv_free_ = CheckCM->CheckFreeCollisionPoint(p2_, "UGV", i);
                
                if(is_ugv_free_){
                    t_.transforms[0].translation = p2_;
                    t_.transforms[0].rotation = q1_;
                    ROS_INFO(PRINTF_GREEN"Entro Iteracion UGV[%lu] j_=%i/%i", i,j,n_interval_);
                }else{
                    ROS_INFO(PRINTF_GREEN"Entro Random UGV[%lu] j_=%i/%i", i,j,n_interval_);
                    geometry_msgs::Vector3 p_ugv_;
                    bool please_ = randomMarsupialStatus(p1_, p2_, i,"UGV", p_ugv_);
                    if (please_){
                        ROS_INFO("YES , UGV RANDOM SOLUTION !!!");
                        t_.transforms[0].translation = p_ugv_;
                        t_.transforms[0].rotation = q1_;
                    }else
                        ROS_ERROR("FUCK, UGV NOT RANDOM SOLUTION !!");
                }

                // Get position and rotation vector for UAV
                p1_.x = trajectory_.points.at(i).transforms[1].translation.x;
                p1_.y = trajectory_.points.at(i).transforms[1].translation.y;
                p1_.z = trajectory_.points.at(i).transforms[1].translation.z;
				q1_.x = trajectory_.points.at(i).transforms[1].rotation.x;
				q1_.y = trajectory_.points.at(i).transforms[1].rotation.y;
				q1_.z = trajectory_.points.at(i).transforms[1].rotation.z;
				q1_.w = trajectory_.points.at(i).transforms[1].rotation.w;
                p2_.x =  p1_.x + (x_uav_/n_interval_)*j ;
				p2_.y =  p1_.y + (y_uav_/n_interval_)*j ;
				p2_.z =  p1_.z + (z_uav_/n_interval_)*j ;

                ROS_INFO(PRINTF_GREEN"UAV[%lu] Test , interval[%i] point[%f %f %f]", i,j, p2_.x, p2_.y, p2_.z);
                bool is_uav_free_ = CheckCM->CheckFreeCollisionPoint(p2_, "UAV", i);
                if(is_uav_free_){
                    t_.transforms[1].translation = p2_;
                    t_.transforms[1].rotation = q1_;
                    ROS_INFO(PRINTF_GREEN"Entro Iteracion UAV[%lu] j_=%i/%i", i,j,n_interval_);
                }else{
                    ROS_INFO(PRINTF_GREEN"Entro Random UAV[%lu] j_=%i/%i", i,j,n_interval_);
                    geometry_msgs::Vector3 p_uav_;
                    bool please_ = randomMarsupialStatus(p1_, p2_, i, "UAV", p_uav_);
                    if (please_){
                        ROS_INFO("YES , UAV RANDOM SOLUTION !!!");
                        t_.transforms[1].translation = p_uav_;
                        t_.transforms[1].rotation = q1_;
                    }else
                        ROS_ERROR("NOT, UAV NOT RANDOM SOLUTION !!");
                }

		        trajectory_aux_.points.push_back(t_);

                if(j != 0){  
                    param_cat_x0 = param_cat_y0 = param_cat_a = 0.0;
                    double l_tether_;
                    bool is_cat_ = getTetherLength(t_.transforms[0].translation, t_.transforms[0].rotation, t_.transforms[1].translation, l_tether_);             
                    
                    if(!is_cat_){
                        l_tether_ = l_catenary_[j_];
                    }
                    
                    length_catenary.push_back(l_tether_);
                    params_.x = param_cat_x0;
                    params_.y = param_cat_y0;
                    params_.z = param_cat_a;
                    v_params_catenary_aux_.push_back(params_);
                    ROS_INFO(PRINTF_YELLOW "Global Planner: interpolatePointsGlobalPath new node in: [%lu] ugv=[%0.3f %0.3f %0.3f] uav=[%0.3f %0.3f %0.3f] l=[%0.3f] %s", 
                        trajectory_aux_.points.size()-1, t_.transforms[0].translation.x,t_.transforms[0].translation.y,t_.transforms[0].translation.z, 
                        t_.transforms[1].translation.x, t_.transforms[1].translation.y, t_.transforms[1].translation.z, l_tether_,is_cat_?"true":"false");
                }
                else{
                    length_catenary.push_back(l_catenary_[j_]);
                    params_.x = v_params_catenary[j_].x;
                    params_.y = v_params_catenary[j_].y;
                    params_.z = v_params_catenary[j_].z;
                    v_params_catenary_aux_.push_back(params_);
                }
            }
		}
		else{
			// Get position and rotation vector for UGV
			t_.transforms[0].translation.x = trajectory_.points.at(i).transforms[0].translation.x;
			t_.transforms[0].translation.y = trajectory_.points.at(i).transforms[0].translation.y;
			t_.transforms[0].translation.z = trajectory_.points.at(i).transforms[0].translation.z;
			t_.transforms[0].rotation.x = trajectory_.points.at(i).transforms[0].rotation.x;
			t_.transforms[0].rotation.y = trajectory_.points.at(i).transforms[0].rotation.y;
			t_.transforms[0].rotation.z = trajectory_.points.at(i).transforms[0].rotation.z;
			t_.transforms[0].rotation.w = trajectory_.points.at(i).transforms[0].rotation.w;
			// Get position and rotation vector for UAV
			t_.transforms[1].translation.x = trajectory_.points.at(i).transforms[1].translation.x;
			t_.transforms[1].translation.y = trajectory_.points.at(i).transforms[1].translation.y;
			t_.transforms[1].translation.z = trajectory_.points.at(i).transforms[1].translation.z;
			t_.transforms[1].rotation.x = trajectory_.points.at(i).transforms[1].rotation.x;
			t_.transforms[1].rotation.y = trajectory_.points.at(i).transforms[1].rotation.y;
			t_.transforms[1].rotation.z = trajectory_.points.at(i).transforms[1].rotation.z;
			t_.transforms[1].rotation.w = trajectory_.points.at(i).transforms[1].rotation.w;
		    trajectory_aux_.points.push_back(t_);
            //Get length catenary
            length_catenary.push_back(l_catenary_[j_]);
            params_.x = v_params_catenary[i].x;
            params_.y = v_params_catenary[i].y;
            params_.z = v_params_catenary[i].z;
            v_params_catenary_aux_.push_back(params_);
		}
    }

	// Get position and rotation vector for UGV
	t_.transforms[0].translation.x = trajectory_.points.at(trajectory_.points.size()-1).transforms[0].translation.x;
	t_.transforms[0].translation.y = trajectory_.points.at(trajectory_.points.size()-1).transforms[0].translation.y;
	t_.transforms[0].translation.z = trajectory_.points.at(trajectory_.points.size()-1).transforms[0].translation.z;
	t_.transforms[0].rotation.x = trajectory_.points.at(trajectory_.points.size()-1).transforms[0].rotation.x;
	t_.transforms[0].rotation.y = trajectory_.points.at(trajectory_.points.size()-1).transforms[0].rotation.y;
	t_.transforms[0].rotation.z = trajectory_.points.at(trajectory_.points.size()-1).transforms[0].rotation.z;
	t_.transforms[0].rotation.w = trajectory_.points.at(trajectory_.points.size()-1).transforms[0].rotation.w;
	// Get position and rotation vector for UAV
	t_.transforms[1].translation.x = trajectory_.points.at(trajectory_.points.size()-1).transforms[1].translation.x;
	t_.transforms[1].translation.y = trajectory_.points.at(trajectory_.points.size()-1).transforms[1].translation.y;
	t_.transforms[1].translation.z = trajectory_.points.at(trajectory_.points.size()-1).transforms[1].translation.z;
	t_.transforms[1].rotation.x = trajectory_.points.at(trajectory_.points.size()-1).transforms[1].rotation.x;
	t_.transforms[1].rotation.y = trajectory_.points.at(trajectory_.points.size()-1).transforms[1].rotation.y;
	t_.transforms[1].rotation.z = trajectory_.points.at(trajectory_.points.size()-1).transforms[1].rotation.z;
	t_.transforms[1].rotation.w = trajectory_.points.at(trajectory_.points.size()-1).transforms[1].rotation.w;
	trajectory_aux_.points.push_back(t_);
    //Get length catenary
    length_catenary.push_back(l_catenary_[trajectory_.points.size()-1]);
    params_.x = v_params_catenary[trajectory_.points.size()-1].x;
    params_.y = v_params_catenary[trajectory_.points.size()-1].y;
    params_.z = v_params_catenary[trajectory_.points.size()-1].z;
    v_params_catenary_aux_.push_back(params_);

    trajectory_.points.clear();
    trajectory_ = trajectory_aux_;
    v_params_catenary.clear();
    v_params_catenary = v_params_catenary_aux_;
}

geometry_msgs::Vector3 RandomGlobalPlanner::getReelNode( double x_, double y_, double z_ , double r_x_, double r_y_, double r_z_, double r_w_)
{
	geometry_msgs::Vector3 pos_reel;
	double roll_, pitch_, yaw_;

	tf::Quaternion q(r_x_, r_y_, r_z_, r_w_);
	tf::Matrix3x3 M(q);	
	M.getRPY(roll_, pitch_, yaw_);

	double lengt_vec =  sqrt(pos_reel_ugv.x*pos_reel_ugv.x + pos_reel_ugv.y*pos_reel_ugv.y);
	pos_reel.x = x_ + lengt_vec *cos(yaw_); 
	pos_reel.y = y_ + lengt_vec *sin(yaw_);
	pos_reel.z = z_ + pos_reel_ugv.z ;

	return pos_reel;
}

void RandomGlobalPlanner::configRandomPlanner()
{

    geometry_msgs::Vector3 pos_ugv_;
    geometry_msgs::Quaternion rot_ugv_;
    geometry_msgs::TransformStamped reel_;
    pos_ugv_ = getRobotPoseUGV().transform.translation;
    rot_ugv_ = getRobotPoseUGV().transform.rotation;
    reel_ = getLocalPoseReel();
    pos_reel_ugv = reel_.transform.translation;
    randPlanner.configRRTParameters(length_tether_max, pos_reel_ugv , pos_ugv_, rot_ugv_, coupled , n_iter, n_loop, 
                                    radius_near_nodes, step_steer, samp_goal_rate, sample_mode, min_l_steer_ugv, w_nearest_ugv ,w_nearest_uav ,w_nearest_smooth);

    rrtgm.configGraphMarkers(world_frame, map_resolution, coupled, n_iter, pos_reel_ugv);

   	ROS_INFO(PRINTF_GREEN"Global Planner  configRandomPlanner() :  length_tether_max: %f , sample_mode=%i !!", length_tether_max, sample_mode);
	ROS_INFO(PRINTF_GREEN"Global Planner  configRandomPlanner() :  is_coupled=[%s] debug_rrt=[%s] debug_msgs=[%s]", coupled? "true" : "false", debug_rrt? "true" : "false", debug? "true" : "false");
}

void RandomGlobalPlanner::clearLinesGPMarker()
{
	rrtgm.clearMarkers(interpolated_path_ugv_marker_pub_, interpolated_path_uav_marker_pub_);
}

void RandomGlobalPlanner::clearCatenaryGPMarker()
{ 
	rrtgm.clearCatenaryMarker(interpolated_catenary_marker_pub_); 
}

bool RandomGlobalPlanner::randomMarsupialStatus(geometry_msgs::Vector3 p1_ , geometry_msgs::Vector3 p2_, int i_, string s_, geometry_msgs::Vector3 &pf_)
{
    geometry_msgs::Vector3 p_status_;
    vector<geometry_msgs::Vector3> points_catenary_;
    points_catenary_.clear();
    bool is_rand_p = true;
    double distr_x_ugv, distr_y_ugv, z_ugv_, distr_z_ugv, x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    z_ugv_ = p2_.z - p1_.z;
   
    // CatenaryCheckerManager ccpp(node_name, grid_3D, pc_obs_ugv, pos_reel_ugv, distance_obstacle_ugv, distance_obstacle_uav, distance_tether_obstacle);
    std::random_device rd;   // obtain a random number from hardware
  	std::mt19937 eng(rd());  // seed the generator
    if(p1_.x <= p2_.x){
        x_min_ = p1_.x;
        x_max_ = p2_.x;
    } else{
        x_min_ = p2_.x;
        x_max_ = p1_.x;
    }
    if(p1_.y <= p2_.y){
        y_min_ = p1_.y;
        y_max_ = p2_.y;
    } else{
        y_min_ = p2_.y;
        y_max_ = p1_.y;
    }
    if(p1_.z <= p2_.z){
        z_min_ = p1_.z;
        z_max_ = p2_.z;
    } else{
        z_min_ = p2_.z;
        z_max_ = p1_.z;
    }
    std::uniform_real_distribution<double> distr_x_(x_min_ , x_max_);  // define the discrete range
    std::uniform_real_distribution<double> distr_y_(y_min_ , y_max_);  // define the discrete range
    std::uniform_real_distribution<double> distr_z_(z_min_ , z_max_);  // define the range 

    int count_ = 0;
    do{
        p_status_.x = distr_x_(eng);
        p_status_.y = distr_y_(eng);
        p_status_.z = distr_z_(eng);
        if (count_ > 400){
            ROS_ERROR("MAS DE 400");
            is_rand_p = false;
            break;
        }
        count_++; 
    }while(!CheckCM->CheckFreeCollisionPoint(p_status_, s_, i_));
	ROS_INFO(PRINTF_GREEN"CatenaryCheckerManager::CheckFreeCollisionPoint :[%i] %s Point Not collision p[%.3f %.3f %.3f]", i_, s_.c_str(), p_status_.x, p_status_.y, p_status_.z);

    pf_ = p_status_;
    if (!is_rand_p)
        return false;
    else
        return true;
}

bool RandomGlobalPlanner::getTetherLength(geometry_msgs::Vector3 tp1_ , geometry_msgs::Quaternion tq1_, geometry_msgs::Vector3 tp2_, double &length_)
{
    geometry_msgs::Vector3 p_reel_, p_final_;
    p_reel_ = getReelNode(tp1_.x ,tp1_.y ,tp1_.z, tq1_.x, tq1_.y, tq1_.z, tq1_.w);
    p_final_ = tp2_; 

    v_p_catenary.clear();
    bool is_cat_ = CheckCM->SearchCatenary(p_reel_, p_final_, v_p_catenary);
    if (is_cat_){
        length_ = CheckCM->length_cat_final;
        param_cat_x0 = CheckCM->param_cat_x0;
        param_cat_y0 = CheckCM->param_cat_y0;
        param_cat_a = CheckCM->param_cat_a;
    }
    return is_cat_;
}

} // namespace PathPlannersgeometry_msgs::Vector3
