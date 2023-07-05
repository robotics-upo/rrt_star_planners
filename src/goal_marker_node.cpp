#include <iostream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

ros::Publisher goal_point_pub;
std::string map;
double final_position_x, final_position_y, final_position_z;

void goalPointMarker()
{
	visualization_msgs::Marker marker_;
	marker_.header.frame_id = "world";
	marker_.header.stamp = ros::Time();
	marker_.ns = "goal_point";
	marker_.id = 0;
	marker_.type = visualization_msgs::Marker::SPHERE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.lifetime = ros::Duration(0);
	marker_.pose.position.x = final_position_x;
	marker_.pose.position.y = final_position_y;
	marker_.pose.position.z = final_position_z;
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
	
	goal_point_pub.publish(marker_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Goal Marker Node");
    ros::NodeHandlePtr nh;
    ros::NodeHandle pnh("~");

    printf("\n\tInitialazing Goal Marker NODE !!\n");
    
    nh.reset(new ros::NodeHandle("~"));

    pnh.param("map", map, (std::string) "map");
    // pnh.param("/"+map+"_pos_final_1/pose/x", final_position_x,(double) 0.0);
    // pnh.param("/"+map+"_pos_final_1/pose/y", final_position_y,(double) 0.0);
    // pnh.param("/"+map+"_pos_final_1/pose/z", final_position_z,(double) 0.0);
	pnh.param(map+"_pos_final_1/pose/x", final_position_x,(double) 0.0);
    pnh.param(map+"_pos_final_1/pose/y", final_position_y,(double) 0.0);
    pnh.param(map+"_pos_final_1/pose/z", final_position_z,(double) 0.0);

	std::string x_ = "/"+map+"_pos_final_1/pose/x";
	std::string y_ = "/"+map+"_pos_final_1/pose/y";
	std::string z_ = "/"+map+"_pos_final_1/pose/z";

    printf("\n\tGoal position:\n");
    printf("\t\t %s = %f\n",x_.c_str(),final_position_x);
    printf("\t\t %s = %f\n",y_.c_str(),final_position_y);
    printf("\t\t %s = %f\n",z_.c_str(),final_position_z);

    bool latch_topic = true;

    goal_point_pub= pnh.advertise<visualization_msgs::Marker>("/random_planner_node/goal_point", 100,  latch_topic);

    goalPointMarker();

    while (ros::ok()) {
      ros::spin();
    }	
	
	return 0;
}