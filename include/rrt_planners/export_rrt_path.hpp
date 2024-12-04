#ifndef _EXPORT_PATH_HPP_
#define _EXPORT_PATH_HPP_


#include <ctime>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
// #include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <yaml-cpp/yaml.h>

#include <upo_actions/ExecutePathAction.h>


using namespace std;

class ExportPath
{

public:
    ExportPath();
    ExportPath(upo_actions::ExecutePathGoal p_, std::string path_);
};

#endif
