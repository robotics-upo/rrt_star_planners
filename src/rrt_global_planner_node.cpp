#include <rrt_planners/rrt_global_planner.hpp>

using namespace PathPlanners;

int main(int argc, char **argv)
{
    std::string node_name = "rrt_global_planner_node";

	ros::init(argc, argv, node_name);

    RRTGlobalPlanner RRTStarGP(node_name);
    
    // dynamic_reconfigure::Server<theta_star_2d::GlobalPlannerConfig> server;
  	// dynamic_reconfigure::Server<theta_star_2d::GlobalPlannerConfig>::CallbackType f;

  	// f = boost::bind(&RRTGlobalPlanner::dynReconfCb,&RRTStarGP,  _1, _2);
  	// server.setCallback(f);

	ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
        RRTStarGP.plan();

        loop_rate.sleep();
    }
    return 0;
}
