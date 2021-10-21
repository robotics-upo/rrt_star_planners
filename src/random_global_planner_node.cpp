#include <rrt_planners/random_global_planner.hpp>

using namespace PathPlanners;

int main(int argc, char **argv)
{
    std::string node_name = "random_global_planner_node";

	ros::init(argc, argv, node_name);

    RandomGlobalPlanner RandomStarGP(node_name);
    
    // dynamic_reconfigure::Server<theta_star_2d::GlobalPlannerConfig> server;
  	// dynamic_reconfigure::Server<theta_star_2d::GlobalPlannerConfig>::CallbackType f;

  	// f = boost::bind(&RandomGlobalPlanner::dynReconfCb,&RRTStarGP,  _1, _2);
  	// server.setCallback(f);

	ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
        RandomStarGP.plan();

        loop_rate.sleep();
    }
    return 0;
}
