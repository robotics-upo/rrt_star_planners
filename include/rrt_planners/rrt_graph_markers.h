#ifndef _RRT_GRAPH_MARKERS_H_
#define _RRT_GRAPH_MARKERS_H_


#include <list>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Vector3.h>

#include "misc/catenary_solver_ceres.hpp"

#include <rrt_planners/RRTNode.h>



using namespace std;

// class RRTNode;

class RRTGraphMarkers
{
	public:
		RRTGraphMarkers();

		void configGraphMarkers(std::string frame_id_, float step_, bool is_coupled_, int n_iter_, geometry_msgs::Vector3 pos_reel_ugv);

		// void getGraphMarker(std::list<RRTNode*> nodes_tree_, ros::Publisher tree_rrt_star_ugv_pub_, ros::Publisher tree_rrt_star_uav_pub_);
		void getGraphMarker(RRTNode* nodes_tree_, int count, ros::Publisher tree_rrt_star_ugv_pub_, ros::Publisher tree_rrt_star_uav_pub_);
		void getTakeOffNodesMarker(std::list<RRTNode*> take_off_nodes_, ros::Publisher take_off_nodes_pub_);
		void getPathMarker(std::list<RRTNode*> pt_, ros::Publisher lines_ugv_marker_pub_, ros::Publisher lines_uav_marker_pub_);
		void getCatenaryMarker(vector<geometry_msgs::Point> points_catenary_, ros::Publisher one_catenary_marker_pub_);
		void getCatenaryPathMarker(std::list<RRTNode*> ct_, ros::Publisher catenary_marker_pub_);
		void getAllCatenaryMarker(std::list<RRTNode*> nodes_tree_, ros::Publisher all_catenary_marker_pub_);
		void goalPointMarker(geometry_msgs::Vector3 final_position_, ros::Publisher goal_point_pub_);
		void reelPointMarker1(geometry_msgs::Point p_, ros::Publisher reel1_point_pub_);
		void reelPointMarker2(geometry_msgs::Point p_, ros::Publisher reel2_point_pub_);
		void randNodeMarker(RRTNode rn_, ros::Publisher rand_point_pub_, int color_);
		void newNodeMarker(RRTNode rn_, ros::Publisher new_point_pub_);
		void nearestNodeMarker(RRTNode rn_, ros::Publisher nearest_point_pub_);
		void getPointsObsMarker(std::vector<geometry_msgs::Point> points_catenary_, ros::Publisher points_marker_pub_);
		void clearMarkers(ros::Publisher tree_rrt_star_ugv_pub_, ros::Publisher tree_rrt_star_uav_pub_, ros::Publisher take_off_nodes_pub_, ros::Publisher lines_ugv_marker_pub_, ros::Publisher lines_uav_marker_pub_);
		void clearCatenaryMarker(ros::Publisher c_m_pub_);

		geometry_msgs::Point getReelNode(const RRTNode node_);
		float getYawFromQuaternion(RRTNode n_, bool is_uav_);
		
		std::string frame_id;
		float step;
		bool is_coupled;
		int n_iter;

		geometry_msgs::Vector3 pos_reel_ugv;

		visualization_msgs::MarkerArray pointTreeMarkerUGV, pointTreeMarkerUAV;
  		visualization_msgs::MarkerArray pointTakeOffMarker, lines_ugv_marker, lines_uav_marker, catenaryMarker, allCatenaryMarker;

	protected:
};


#endif