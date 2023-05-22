#include <rrt_planners/random_global_uav_planner.hpp>

using namespace PathPlanners;

int main(int argc, char **argv)
{
    std::string node_name = "random_global_uav_planner_node";

	ros::init(argc, argv, node_name);

  RandomGlobalUAVPlanner RandomGP(node_name);
    
	ros::Rate loop_rate(5);
  while(ros::ok()){

    ros::spinOnce();
    RandomGP.plan();

    loop_rate.sleep();
  }
  return 0;
}
