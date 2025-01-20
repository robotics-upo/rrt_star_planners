#ifndef __RRT_NODE_H__
#define __RRT_NODE_H__

#include <geometry_msgs/Point.h>

//*****************************************************************
//				Auxiliar Class for RRTPlanner Algorithm
//*****************************************************************

class DiscretePosition
{
public:
	int x, y, z;
};

class nodeOrientation
{
public:
	float x, y, z, w;
};

class RRTNode;

class RRTNodeLink3D
{
public:
	RRTNodeLink3D() : node(NULL), isInOpenList(false), isInCandidateList(false), notOccupied(true)
	{
	}
	RRTNode *node;  // Node from this link
	bool isInOpenList;		// algorithm open list flag
	bool isInCandidateList; // algorithm candidate list flag
	bool notOccupied;		// occupancy mark
	float cost;
};

class RRTNode
{
public:
	// NodeState st;
	DiscretePosition point;
	DiscretePosition point_uav;
	nodeOrientation rot_ugv = {0.0, 0.0, 0.0, 1.0};
	nodeOrientation rot_uav = {0.0, 0.0, 0.0, 1.0};
	int id;
	int id_uav;
	bool catenary;	//Inform the feasibility to get a catenary in node
	double length_cat;	//Length of node catenary
	double min_dist_obs_cat;	//Minimun distance from node catenary to obstacle
	double min_dist_obs_ugv;	//Minimun distance from UGV node to obstacle
	double min_dist_obs_uav;	//Minimun distance from UAV node to obstacle
	double cost;	// Refer to the cost in the node without consider the catenary
	double cost_takeoff;	//refer to the cost only in the nodes which the drone can take off
	std::vector<geometry_msgs::Point> p_cat;
	double param_cat_x0;
	double param_cat_y0;
	double param_cat_a ;
	float dist;
	// double h_cost;
	RRTNode *parentNode;
	RRTNodeLink3D *nodeInWorld; // pointer to link from this node to its parent
	// RRTNode()
	// {
	//   parentNode = NULL;
	// }

	// // Comparator '!=' definition
	// friend bool operator!=(const RRTNode &lhs, const RRTNode &rhs)
	// {
	// 	return lhs.point.x != rhs.point.x ||
	// 		   lhs.point.y != rhs.point.y ||
	// 		   lhs.point.z != rhs.point.z;
	// }
};

#endif
