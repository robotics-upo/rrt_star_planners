#include <rrt_planners/rrt_global_planner.hpp>

using namespace PathPlanners;

int main(int argc, char **argv)
{

    std::string node_name = "rrt_global_planner_node";
    std::string node_name_grid = "grid3D_node";
    std::string path_grid3D;

	ros::init(argc, argv, node_name);
    ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.param<std::string>("path_grid3D",  path_grid3D, "~/");


    RRTGlobalPlanner RRTStarGP(node_name);
    
    // Grid3d* g3D_ = new Grid3d(node_name_grid, path_grid3D);
    // Grid3d* g3D_ = new Grid3d(node_name_grid);
    // RRTStarGP.receiveGrid3D(g3D_);

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