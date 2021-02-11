#ifndef RRT_STAR_NODE__H__
#define RRT_STAR_NODE__H__
#include <geometry_msgs/Vector3.h>

struct RRTNode
{
  // NodeState st;
  geometry_msgs::Vector3 point;
  int id;
  double cost;
  double h_cost;
  RRTNode *parentNode;
  // RRTStarNodeLink3D *nodeInWorld; // pointer to link from this node to its parent
  int p_id;
  // std::vector<RRTNode*> children;
  RRTNode()
  {
    parentNode = NULL;
    // dead = false;
  }
  // bool dead;
};
  
  
#endif