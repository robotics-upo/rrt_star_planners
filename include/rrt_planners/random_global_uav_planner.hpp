/*
Simon Martinez Rozas, 2021 UPO

Global Planner Class using RANDOM Algorithms (RRT, RRT*, biRRT)
*/
#ifndef RANDOM_UAV_GLOBALPLANNER__HPP__
#define RANDOM_UAV_GLOBALPLANNER__HPP__

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <memory>
#include <rrt_planners/random_uav_planner.hpp>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

#include <visualization_msgs/Marker.h>

#include "tf2/transform_datatypes.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Dynamic reconfigure auto generated libraries
#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/GlobalPlannerConfig.h>

#include <time.h>
#include <sys/timeb.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <upo_actions/ExecutePathAction.h>
#include <upo_actions/MakePlanAction.h>
#include <upo_actions/RotationInPlaceAction.h>

#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "misc/catenary_solver_ceres.hpp"
#include "catenary_checker/near_neighbor.hpp"
#include "rrt_planners/random_graph_markers.h"

#include "catenary_checker/catenary_checker_manager.h"
#include "catenary_checker/grid3d.hpp"
#include "catenary_checker/bisection_catenary_3D.h"


namespace PathPlanners
{
  class RandomGlobalUAVPlanner 
  {
    typedef actionlib::SimpleActionClient<upo_actions::ExecutePathAction> ExecutePathClient;
    typedef actionlib::SimpleActionServer<upo_actions::MakePlanAction> MakePlanServer;
    typedef actionlib::SimpleActionClient<upo_actions::RotationInPlaceAction> RotationInPlaceClient;

  public:
    //Default constructor
    RandomGlobalUAVPlanner(std::string node_name);

    /**
       Default destructor
    **/
    ~RandomGlobalUAVPlanner();

    /*
      @brief: This is the main function that should be executed in loop by the node
    */
    void plan();
            
  private:
    void sendPathToLocalPlannerServer();

    //Action server
    void makePlanPreemptCB();
    void makePlanGoalCB();

    // bool replan();
    void replan();
            
    /*
      @brief: Loads parameters from ros param server, if they are not present, load defaults ones
      It also configure markers and global map geometry 
    */
    void configParams();
    /*
      @brief: Load topics names from param server and if they are not present, set defaults topics names for 
      Subscribers and publishers
    */
    void configTopics();
    /*
      @brief: Config thetastar class parameters
    */
    void configRRTStar();
    /*
      @brief: It declares the two service servers
    */
    void configServices();

    //get UGV pose to know from where to plan
    geometry_msgs::TransformStamped getRobotPoseUGV();

    //get UAV pose to know from where to plan
    geometry_msgs::TransformStamped getRobotPoseUAV();

    //get Local Reel pose on UGV

    geometry_msgs::TransformStamped getLocalPoseReel();
    /*
      @brief: 
    */
    void collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg);
    void readPointCloudUAVObstaclesMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void deleteNodesMarkersCallBack(const std_msgs::BoolConstPtr &msg);
    void deleteCatenaryGPCallBack(const std_msgs::BoolConstPtr &msg);
    void pointsSub(const PointCloud::ConstPtr &points);
    /*
      @brief: 
    */
    void publishPath();

    bool calculatePath();

    /*
      @brief: These functions tries to pass the start and goal positions to the thetastar object
      They return true if the points are not occupied
    */
    bool setGoal();
    bool setStart();

    /*
      @brief: Get tf reel tether to compute catenary.
    */
    void configRandomPlanner();

    void interpolatePointsGlobalPath(Trajectory &trajectory_, std::vector<double> l_catenary_);
    void clearCatenaryGPMarker();
    void clearLinesGPMarker();
            
    geometry_msgs::Point getReelNode( double x_, double y_, double z_ ,
                                      double r_x_, double r_y_, double r_z_, double r_w_);

    CatenaryCheckerManager *CheckCM;

    /*              Class Variables                 */
    ros::NodeHandlePtr nh;

    visualization_msgs::Marker lineMarker, waypointsMarker;

    geometry_msgs::PoseStamped goalPoseStamped;
    geometry_msgs::Vector3Stamped goal;
    geometry_msgs::Vector3 start_rpy;
    geometry_msgs::Vector3 pos_reel_ugv;

    //Publishers and Subscribers
    ros::Publisher replan_status_pub, fullRayPublisher, rayCastFreePublisher,
      rayCastFreeReducedPublisher, rayCastCollPublisher; 
    ros::Publisher clean_markers_optimizer_pub_, initial_catenary_pub_, interpolated_path_uav_marker_pub_, interpolated_catenary_marker_pub_;
    ros::Subscriber goal_sub, sub_map, point_cloud_map_uav_sub_, point_cloud_map_trav_sub_,
      clean_nodes_marker_gp_sub_,clean_catenary_marker_gp_sub_;

    //Services servers
    ros::ServiceServer global_replanning_service, reset_global_costmap_service, plan_request_service;
    ros::ServiceClient recovery_rot_srv_client;

    //tf buffer used to get the base_link position on the map(i.e. tf base_link-map)
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_list;

    std::unique_ptr<tf::TransformListener> tf_list_ptr;

    std_msgs::Bool flg_replan_status;

    string ugv_base_frame, uav_base_frame, reel_base_frame, world_frame, node_name;
    double pos_reel_x, pos_reel_y, pos_reel_z;

    //Output variables
    int number_of_points;
    int seq;
    Trajectory trajectory;

    Grid3d *grid_3D;

    //These two flags can be configured as parameters
    bool showConfig, debug, debug_rrt, nodes_marker_debug;

    std::string path, name_output_file, map_file;
    int num_pos_initial;
    int countImpossible = 0;

    //Action client stuff
    std::unique_ptr<ExecutePathClient> execute_path_client_ptr;

    std::unique_ptr<MakePlanServer> make_plan_server_ptr;
    upo_actions::MakePlanFeedback make_plan_fb;
    upo_actions::MakePlanResult make_plan_res;

    std::unique_ptr<RotationInPlaceClient> rot_in_place_client_ptr;
    upo_actions::RotationInPlaceGoal rot_in_place_goal;

    //Variables to fill up the feedback 
    std_msgs::Duration travel_time;
    int timesReplaned;
    struct timespec start, finish;
    float seconds, milliseconds;
    ros::Time start_time;

    //! 3D specific variables
    bool mapRec, use3d; 
    bool use_distance_function; //Only related with tether and UAV distance
    RandomUAVPlanner randPlanner;
    PlannerGraphMarkers rrtgm;

    octomap_msgs::OctomapConstPtr map;

    double ws_x_max; // 32.2
    double ws_y_max; // 32.2
    double ws_z_max;
    double ws_x_min;
    double ws_y_min;
    double ws_z_min;
    double map_resolution;
    double map_h_inflaction;
    double map_v_inflaction; //JAC: Hasta aquí todo cero.

    bool pause_execution = false;
    double length_tether_max, radius_near_nodes, step_steer;
    int n_iter, n_loop, samp_goal_rate;
    double goal_gap_m;
    double distance_obstacle_uav, distance_catenary_obstacle; //Safe distance to obstacle to accept a point valid for UAV
    bool get_catenary_data_, optimize;
            
    double min_distance_add_new_point;
    std::vector<double> length_catenary;

    bool get_catenary_data, use_parable, use_both;
    std::string planner_type, catenary_file, catenary_analysis_file;

  }; //class RandomGlobalUAVPlanner

} //namespace PathPlanners

#endif
