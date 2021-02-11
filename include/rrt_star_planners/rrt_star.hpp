/**
 * @file rrt_star.hpp
 * @author vss2sn
 * @brief Contains the RRTStar class
 */

#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <limits>

#include "utils/utils.hpp"
#include "rrt_global_planners/RRTstarNode.h"


class DiscretePosition
{
  public:
    int x, y, z;
};

/**
 * @brief Class for RRT Star objects
 */
class RRTStar 
{

  public:
    /**
     * Constructor
     **/
    RRTStar();
    /**
     * @brief Find the nearest Node that has been seen by the algorithm. This does
     * not consider cost to reach the node.
     * @param new_node Node to which the nearest node must be found
     * @return nearest node
     */
    Node FindNearestPoint(Node& new_node);

    /**
     * @brief Check if there is any obstacle between the 2 nodes. As this planner
     * is for grid maps, the obstacles are square.
     * @param n_1 Node 1
     * @param n_2 Node 2
     * @return bool value of whether obstacle exists between nodes
     */
    bool CheckObstacle(const Node& n_1, const Node& n_2) const;

    /**
     * @brief Generates a random node
     * @return Generated node
     */
    Node GenerateRandomNode() const;

    /**
     * @brief Rewire the tree
     * @param new_node Node to which other nodes will be connected if their cost
     * decreases
     * @return void
     */
    void Rewire(const Node& new_node);

    /**
     * @brief Main algorithm of RRT*
     * @param grid Main grid
     * @param start_in starting node
     * @param goal_in goal node
     * @param max_iter_x_factor Maximum number of allowable iterations before
     * returning no path
     * @param threshold_in Maximum distance per move
     * @return path vector of nodes
     */
    std::vector<Node> rrt_star(
        std::vector<std::vector<int>>& grid, const Node& start_in,
        const Node& goal_in, const int max_iter_x_factor = 500,
        const double threshold_in = std::numeric_limits<double>::infinity());

    /**
     * @brief Check if goal is reachable from current node
     * @param new_node Current node
     * @return bool value of whether goal is reachable from current node
     */
    bool CheckGoalVisible(const Node& new_node);

    /**
     * @brief Create the obstacle list from the input grid
     * @param grid Input grid for algorithm
     * @return void
     */
    void CreateObstacleList(std::vector<std::vector<int>>& grid);

    	/**
		  Set initial position of the path only check if 
		  it's inside WS but not if it's occupied
			@param 3D position data (Discrete or Continuous)
			@return false if is outside the workspace
		**/
	bool setInitialPosition(DiscretePosition p_);
	bool setInitialPosition(Vector3 p);

	/**
		  Set final position of the path only check if 
		  it's inside WS but not if it's occupied
			@param 3D position data (Discrete or Continuous)
			@return false if is outside the workspace
		**/
	bool setFinalPosition(DiscretePosition p_);
	bool setFinalPosition(Vector3 p);

	/**
		  Set initial/final position of the path checking if 
		  it's inside WS and if it'ts occupied (--> Valid)
		   @param [x,y,z] discrete or continuous position
		   @return true if is a valid initial/final position and has been set correctly
		**/
	inline bool setValidInitialPosition(DiscretePosition p)
	{
		if (setInitialPosition(p))
		{
			if (!isInitialPositionOccupied())
			{
				ROS_INFO("RRTStar: Initial discrete position [%d, %d, %d] set correctly", p.x, p.y, p.z);
				return true;
			}
		}
		else
		{
			//if(PRINT_WARNINGS)
			ROS_WARN("RRTStar: Initial position outside the workspace attempt!!");
		}

		return false;
	}
	inline bool setValidInitialPosition(Vector3 p)
	{

		DiscretePosition pp = discretizePosition(p);

		return setValidInitialPosition(pp);
	}
	inline bool setValidFinalPosition(DiscretePosition p)
	{
		if (setFinalPosition(p))
		{
			if (!isFinalPositionOccupied())
			{
				ROS_INFO("RRTStar: Final discrete position [%d, %d, %d] set correctly", p.x, p.y, p.z);
				return true;
			}
		}
		else
		{
			//if(PRINT_WARNINGS)
			ROS_WARN("RRTStar: Final position outside the workspace attempt!! [%d, %d, %d]", p.x, p.y, p.z);
		}

		return false;
	}
	inline bool setValidFinalPosition(Vector3 p)
	{
		DiscretePosition pp = discretizePosition(p);

		return setValidFinalPosition(pp);
	}

    float step; // Resolution of the Matrix and its inverse
	  float step_inv;

  private:
    std::vector<Node> point_list_;
    std::vector<Node> obstacle_list_;
    std::vector<Node> near_nodes_;
    std::vector<double> near_nodes_dist_;
    Node start_, goal_;
    double threshold_ = 1;
    bool found_goal_ = false;
    int size_grid = 0;

  protected:

  	/** Inline Functions **/

	/**
		 Get discrete occupancy matrix index for this discrete (x,y,z) position
		   @param x, y, z discrete position values
		   @return the index in the occupancy matrix
		**/
	inline unsigned int getWorldIndex(int &x, int &y, int &z)
	{
		return (unsigned int)((x - ws_x_min_inflated) + (Lx) * ((y - ws_y_min_inflated) + (Ly) * (z - ws_z_min_inflated)));
	}
  
    NodeRRT *disc_initial, *disc_final; // Discretes

    int ws_x_max, ws_y_max, ws_z_max; // WorkSpace lenghts from origin (0,0,0)
	  int ws_x_min, ws_y_min, ws_z_min;
};

#endif  // RRT_STAR_H
