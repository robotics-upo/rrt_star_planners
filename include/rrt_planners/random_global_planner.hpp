/*
Simon Martinez Rozas, 2021 UPO

Global Planner Class using RANDOM Algorithms (RRT, RRT*, biRRT)
*/
#ifndef RANDOM_GLOBALPLANNER__HPP__
#define RANDOM_GLOBALPLANNER__HPP__

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <memory>
#include <rrt_planners/random_planner.hpp>

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
#include "rrt_planners/export_rrt_path.hpp"

#include "catenary_checker/catenary_checker_manager.h"
#include "catenary_checker/grid3d.hpp"
#include "catenary_checker/bisection_catenary_3D.h"
// #include <catenary_checker/check_collision_path_planner.h>



namespace PathPlanners
{
    class RandomGlobalPlanner 
    {
        typedef actionlib::SimpleActionClient<upo_actions::ExecutePathAction> ExecutePathClient;
        typedef actionlib::SimpleActionServer<upo_actions::MakePlanAction> MakePlanServer;
        typedef actionlib::SimpleActionClient<upo_actions::RotationInPlaceAction> RotationInPlaceClient;

        public:
            //Default constructor
            RandomGlobalPlanner(std::string node_name);

            /**
                Default destructor
            **/
            // ~RandomGlobalPlanner();

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
            void readPointCloudTraversabilityMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
            void readPointCloudUGVObstaclesMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
            void readPointCloudUAVObstaclesMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
            void deleteNodesMarkersCallBack(const std_msgs::BoolConstPtr &msg);
            void deleteCatenaryGPCallBack(const std_msgs::BoolConstPtr &msg);
            void pointsSub(const PointCloud::ConstPtr &points);
            bool randomMarsupialStatus(geometry_msgs::Vector3 p_ , geometry_msgs::Vector3 p1_, int i_, string s_, geometry_msgs::Vector3 &pf_);
            bool getTetherLength(geometry_msgs::Vector3 tp1_ , geometry_msgs::Quaternion tq1_, geometry_msgs::Vector3 tp2_, double &length_);
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
            
            geometry_msgs::Vector3 getReelNode( double x_, double y_, double z_ , double r_x_, double r_y_, double r_z_, double r_w_);

	        CatenaryCheckerManager *CheckCM;

            /*              Class Variables                 */
            ros::NodeHandlePtr nh;

            visualization_msgs::Marker lineMarker, waypointsMarker;

            geometry_msgs::PoseStamped goalPoseStamped;
            geometry_msgs::Vector3Stamped goal;
            geometry_msgs::Vector3 start_rpy;
            geometry_msgs::Vector3 pos_reel_ugv;
            vector<int> v_pos_coll_tether;


            //Publishers and Subscribers
            ros::Publisher replan_status_pub, fullRayPublisher, rayCastFreePublisher, rayCastFreeReducedPublisher, rayCastCollPublisher; 
            ros::Publisher clean_markers_optimizer_pub_, initial_catenary_pub_, interpolated_path_ugv_marker_pub_, interpolated_path_uav_marker_pub_, interpolated_catenary_marker_pub_;
            ros::Subscriber goal_sub, sub_map, point_cloud_map_uav_sub_, point_cloud_map_ugv_sub_, point_cloud_map_trav_sub_, clean_nodes_marker_gp_sub_,clean_catenary_marker_gp_sub_;

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
            bool showConfig, debug, debug_rrt, nodes_marker_debug, save_path_in_file;

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
            RandomPlanner randPlanner;
	        PlannerGraphMarkers rrtgm;
            sensor_msgs::PointCloud2::ConstPtr pc_obs_ugv;

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

            bool write_data_for_analysis;
            bool pause_execution;
            double length_tether_max, radius_near_nodes, step_steer;
            int n_iter, n_loop, samp_goal_rate;
            double goal_gap_m;
            double min_l_steer_ugv;// min distance UGV-UAV to steer a new position of UGV
            double distance_obstacle_ugv, distance_obstacle_uav, distance_tether_obstacle; //Safe distance to obstacle to accept a point valid for UGV and UAV
            int sample_mode; // 0: random sample for UGV and UAV , 1: random sample only for UAV  
            bool do_steer_ugv; //able with sample_mode = 1 to steer ugv position in case to get ugv random position when is not able catenary
            double w_nearest_ugv ,w_nearest_uav ,w_nearest_smooth;

            bool coupled, get_catenary_data, use_parable;
            
            double min_distance_add_new_point;
            std::vector<double> length_catenary;

            std::string planner_type, catenary_file, catenary_analysis_file;

    }; //class RandomGlobalPlanner

} //namespace PathPlanners

#endif