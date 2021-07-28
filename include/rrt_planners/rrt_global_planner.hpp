
/*
Simon Martinez Rozas, 2021 UPO
Global Planner Class using RRT Algorithms
*/
#ifndef RRTSTAR_GLOBALPLANNER__HPP__
#define RRTSTAR_GLOBALPLANNER__HPP__

#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <memory>
#include <rrt_planners/rrt_planner.hpp>

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

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Dynamic reconfigure auto generated libraries
#include <dynamic_reconfigure/server.h>
#include <theta_star_2d/GlobalPlannerConfig.h>

#include <ctime>
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
#include "misc/bisection_catenary_3D.h"
#include "misc/grid3d.hpp"


namespace PathPlanners
{
    class RRTGlobalPlanner : public RRTPlanner
    {
        typedef actionlib::SimpleActionClient<upo_actions::ExecutePathAction> ExecutePathClient;
        typedef actionlib::SimpleActionServer<upo_actions::MakePlanAction> MakePlanServer;
        typedef actionlib::SimpleActionClient<upo_actions::RotationInPlaceAction> RotationInPlaceClient;

        public:
            //Default constructor
            RRTGlobalPlanner(std::string node_name);

            void printfTrajectory(Trajectory trajectory, string trajectory_name);

            bool isMarsupialCoupled();
            /**
                Default destructor
            **/
            // ~RRTGlobalPlanner();

            /*
            @brief: This is the main function that should be executed in loop by the node
            */
            void plan();
            
            void receiveGrid3D(Grid3d* G3D_);

        private:
            void clearMarkers();
            void clearMarkersRayCast();
            void sendPathToLocalPlannerServer();
            void publishMakePlanFeedback();

            //Action server
            void makePlanPreemptCB();
            void makePlanGoalCB();

            int getClosestWaypoint();
            bool replan();
            
            /*
            @brief: Loads parameters from ros param server, if they are not present, load defaults ones
                    It also configure markers and global map geometry 
            */
            void configMarkers(std::string ns);
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
            
            /*
            @brief: 
            */
            void collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg);
            void readPointCloudTraversabilityMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
            void readPointCloudUGVObstaclesMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
            void readPointCloudUAVObstaclesMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
            void pointsSub(const PointCloud::ConstPtr &points);
            /*
            @brief: 
            */
            void publishTrajectory();

            bool calculatePath();

            /*
            @brief: These functions tries to pass the start and goal positions to the thetastar object
                    They return true if the points are not occupied
            */
            bool setGoal();
            bool setStart();
            // bool setStartUGV();
            // bool setStartUAV();

            /*
            @brief: 
            */
            void calculatePathLength();
            float getYawFromQuat(Quaternion quat);
           

            /*
            @brief: Get tf reel tether to compute catenary.
            */
            geometry_msgs::Vector3 tfListenerReel();
            void configRRTPlanner();
            
            /*              Class Variables                 */
            ros::NodeHandlePtr nh;

            //!New Markers(Line strip + waypoints)
            visualization_msgs::Marker lineMarker, waypointsMarker, fullrayMarker, raycastfreeMarker, raycastfreereducedMarker, raycastcollMarker, raycastnofreeMarker;

            geometry_msgs::PoseStamped goalPoseStamped;
            geometry_msgs::Vector3Stamped goal;
            geometry_msgs::Vector3 start_rpy;

            //Publishers and Subscribers
            ros::Publisher replan_status_pub,visMarkersPublisher, fullRayPublisher, rayCastFreePublisher, rayCastFreeReducedPublisher, rayCastCollPublisher; 
            ros::Publisher rayCastNoFreePublisher, reducedMapPublisher, cleanMarkersOptimizerPublisher;
            ros::Subscriber goal_sub, sub_map, point_cloud_map_uav_sub_, point_cloud_map_ugv_sub_, point_cloud_map_trav_sub_;

            //Listener tf reel
            tf::TransformListener listener;

            //Services servers
            ros::ServiceServer global_replanning_service, reset_global_costmap_service, plan_request_service;
            ros::ServiceClient recovery_rot_srv_client;
            //ThetaStar object

            //tf buffer used to get the base_link position on the map(i.e. tf base_link-map)
            std::shared_ptr<tf2_ros::Buffer> tfBuffer;
            std::unique_ptr<tf2_ros::TransformListener> tf2_list;

            std::unique_ptr<tf::TransformListener> tf_list_ptr;
            std::unique_ptr<costmap_2d::Costmap2DROS> global_costmap_ptr;

            std_msgs::Bool flg_replan_status;

            float cost_weight;
            float occ_threshold;
            float lof_distance;

            string ugv_base_frame, uav_base_frame, world_frame, node_name;
            double pos_reel_x, pos_reel_y, pos_reel_z;

            //Output variables
            int number_of_points;
            int seq;
            float pathLength;
            Trajectory trajectory;

            //These two flags can be configured as parameters
            bool showConfig, debug, debug_rrt;

	        std::string path, name_output_file;
	        std::string path_grid3D;
	        int scenario_number, num_pos_initial, num_goal;
            int countImpossible = 0;

            //Action client stuff
            std::unique_ptr<ExecutePathClient> execute_path_client_ptr;

            std::unique_ptr<MakePlanServer> make_plan_server_ptr;
            upo_actions::MakePlanFeedback make_plan_fb;
            upo_actions::MakePlanResult make_plan_res;

            //
            std::unique_ptr<RotationInPlaceClient> rot_in_place_client_ptr;
            upo_actions::RotationInPlaceGoal rot_in_place_goal;

            //
            //Vairables to fill up the result MakePlan result field
            std_msgs::Duration  time_spent;
            //Variables to fill up the feedback 
            std_msgs::Float32 dist2Goal;
            std_msgs::Duration travel_time;
            std_msgs::String percent_achieved, ETA;
            std_msgs::UInt8 globalWaypoint;
            int timesReplaned;
            struct timeb start, finish;
            float seconds, milliseconds;
            float minPathLenght;
            ros::Time start_time;

            //! 3D specific variables
            bool mapRec;
            RRTPlanner rrtplanner;
            bool use3d;

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
            double goal_weight;
            double z_weight_cost;
            double z_not_inflate;
            double traj_dxy_max;
            double traj_dz_max;
            double traj_vxy_m;
            double traj_vz_m;
            double traj_vxy_m_1;
            double traj_vz_m_1;
            double traj_wyaw_m;
            double traj_pos_tol;
            double traj_yaw_tol;
            double timeout;
            double initialSearchAround;

            bool write_data_for_analysis;
            double length_tether_max, radius_near_nodes, step_steer;
            int n_iter, n_loop, samp_goal_rate;
            double goal_gap_m;
            double min_l_steer_ugv;// min distance UGV-UAV to steer a new position of UGV
            double distance_obstacle_ugv, distance_obstacle_uav; //Safe distance to obstacle to accept a point valid for UGV and UAV
            int sample_mode; // 0: random sample for UGV and UAV , 1: random sample only for UAV  
            bool do_steer_ugv; //able with sample_mode = 1 to steer ugv position in case to get ugv random position when is not able catenary

            bool coupled;

            std::string planner_type;

            Grid3d *grid3D;
    }; //class RRTGlobalPlanner

} //namespace PathPlanners

#endif