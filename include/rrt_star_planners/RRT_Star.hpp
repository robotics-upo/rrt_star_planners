#ifndef _RRT_STAR__HPP__
#define _RRT_STAR__HPP__

#include <math.h>
#include <random>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <octomap_msgs/Octomap.h> //Octomap Binary
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/filters/random_sample.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/StdVector>

// #include "rrt_star_planners/near_neighbor.hpp"
#include "marsupial_g2o/near_neighbor.hpp"
#include "rrt_star_planners/kdtree.hpp"

// #include "marsupial_g2o/kdtree_test.hpp"
#include "marsupial_g2o/catenary_solver_ceres.hpp"



#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"
#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_MAGENTA "\x1B[35m"
#define PRINTF_CYAN "\x1B[36m"
#define PRINTF_WHITE "\x1B[37m"

// Uncomment to set length catenary in nodes
#define USE_CATENARY_COMPUTE

namespace PathPlanners
{

typedef geometry_msgs::Vector3 Vector3;
typedef geometry_msgs::Quaternion Quaternion;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef trajectory_msgs::MultiDOFJointTrajectory Trajectory;
typedef visualization_msgs::Marker RVizMarker;


//*****************************************************************
//				Auxiliar Class for RRTStar Algorithm
//*****************************************************************

class DiscretePosition
{
public:
	int x, y, z;
};

class RRTNode;

class RRTStarNodeLink3D
{
public:
	RRTStarNodeLink3D() : node(NULL), isInOpenList(false), isInCandidateList(false), notOccupied(true)
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
	int id;
	int id_uav;
	bool catenary;	//Inform the feasibility to get a catenary in node
	double length_cat;	//Length of node catenary
	double min_dist_obs_cat;	//Minimun distance from node catenary to obstacle
	double min_dist_obs_ugv;	//Minimun distance from UGV node to obstacle
	double min_dist_obs_uav;	//Minimun distance from UAV node to obstacle
	double cost;	// Refer to the cost in the node without consider the catenary
	double cost_takeoff;	//refer to the cost only in the nodes which the drone can take off
	// double h_cost;
	RRTNode *parentNode;
	RRTStarNodeLink3D *nodeInWorld; // pointer to link from this node to its parent
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

//*****************************************************************
// 				RRTStar Algoritm Class Declaration
//*****************************************************************

class RRTStar
{
public:
	RRTStar();

	/**
		  Constructor with arguments
		   @param planner name for topic names 
		   @param frame_id for debug markers 
		   @param simetric or asimetric workspace centered at (0,0,0) [meters]
		   @param occupancy matrix resolution [meters]
		   @param occupancy matrix nodes inflation (horizontal and vertical, real + safety) [meters]
		   @param Lazy Theta* with Optimization: goal point factor [0 to inf]. Bigger -> distance to target more weight than distance to origin -> minor exploration -> shorter runtime, grater path length
		   @param Lazy Theta* weighted: Z axis weight cost [0 to inf]. 0 to 1 for Z priority exploration, 1 for symetric exploration and inf(~100) to not explore in Z.
		   @param Lazy Theta* bounded: Minimum Z that will be inflated vertically 
		   @param NodeHandle 
		**/
	RRTStar(std::string plannerName, std::string frame_id_, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_, float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, ros::NodeHandlePtr nh_, double goal_gap_m_);

	/**
		  Initialization
		   @param planner name for topic names 
		   @param frame_id for debug markers 
		   @param simetric or asimetric workspace centered at (0,0,0) [meters]
		   @param occupancy matrix resolution [meters]
		   @param occupancy matrix nodes inflation (horizontal and vertical, real + safety) [meters]
		   @param Lazy Theta* with Optimization: goal point factor [0 to inf]. Bigger -> distance to target more weight than distance to origin -> minor exploration -> shorter runtime, grater path length
		   @param Lazy Theta* weighted: Z axis weight cost [0 to inf]. 0 to 1 for Z priority exploration, 1 for symetric exploration and inf(~100) to not explore in Z.
		   @param Lazy Theta* bounded: Minimum Z that will be inflated vertically 
		   @param NodeHandle 
		**/
	void init(std::string plannerName, std::string frame_id_, float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_, float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, ros::NodeHandlePtr nh_, double goal_gap_m_);

  	~RRTStar();
  
  	virtual int computeTreeCoupled();      
  	virtual int computeTreesIndependent();      

  	float getYawFromQuat(Quaternion quat);


  	std::list<RRTNode *> nodes_tree; // TODO: single tree planners
  	// std::list<RRTNode *> nodes_tree_ugv, nodes_tree_uav; // TODO: single tree planners
  	std::list<RRTNode *> take_off_nodes; // TODO: single 
  
  	virtual void getGraphMarker();
	virtual void getTakeOffNodesMarker();
	virtual void getPathMarker(std::list<RRTNode*> pt_);
	virtual void getCatenaryMarker(std::vector<geometry_msgs::Point> points_catenary_);
	virtual void getCatenaryPathMarker(std::list<RRTNode*> ct_);
	virtual void getAllCatenaryMarker();
	virtual void goalPointMarker();
	void randPointMarker(RRTNode rn_);
	void getPointsObsMarker(std::vector<geometry_msgs::Point> points_catenary_);
	virtual void clearMarkers();
  	virtual void clearNodes();
	bool getTrajectory(Trajectory &trajectory);

  	// virtual int getGraphSize() {
    // 	return nodes_tree.size();
  	// }

	/**
		  Set initial position of the path only check if 
		  it's inside WS but not if it's occupied
			@param 3D position data (Discrete or Continuous)
			@return false if is outside the workspace
		**/
	bool setInitialPosition(DiscretePosition p_);
	bool setInitialPositionCoupled(DiscretePosition p_);
	bool setInitialPositionIndependent(DiscretePosition p1_, DiscretePosition p2_);
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
	// inline bool setValidInitialPosition(DiscretePosition p)
	// {
	// 	if (setInitialPosition(p))
	// 	{
	// 		if (!isInitialPositionOccupied())
	// 		{
	// 			ROS_INFO("ThetaStar: Initial discrete position [%d, %d, %d] set correctly", p.x, p.y, p.z);
	// 			return true;
	// 		}
	// 	}
	// 	else
	// 	{
	// 		ROS_WARN("ThetaStar: Initial position outside the workspace attempt!!");
	// 	}

	// 	return false;
	// }

	inline bool setValidInitialPositionMarsupial(DiscretePosition p1, DiscretePosition p2)
	{
		if (is_coupled){
			if (setInitialPositionCoupled(p1))
			{
				if (!isInitialPositionOccupied(false))
				{
					ROS_INFO("RRTStar: Initial discrete position UGV Coupled[%d, %d, %d] set correctly", p1.x, p1.y, p1.z);
					return true;
				}
			}
			else
			{
				ROS_WARN("RRTStar: Initial position UGV Coupled outside the workspace attempt!!");
			}

			return false;
		}
		else{
				if(setInitialPositionIndependent(p1,p2))
				{
					if (!isInitialPositionOccupied(false))
					{
						if(!isInitialPositionOccupied(true)){
							ROS_INFO("RRTStar: Initial Marsupial discrete position UGV [%d, %d, %d] and UAV [%d, %d, %d] set correctly", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
							return true;
						}
						else{
							ROS_WARN("RRTStar: Initial position UAV Marsupial independent configuration outside of the workspace attempt!!");
						}
					}
					else
						ROS_WARN("RRTStar: Initial position UGV Marsupial independent configuration outside of the workspace attempt!!");
				}		
				else
					ROS_WARN("RRTStar: Initial position Marsupial independent configuration outside of the workspace attempt!!");

			return false;
		}
	}

	

	// inline bool setValidInitialPosition(Vector3 p)
	// {

	// 	DiscretePosition pp = discretizePosition(p);

	// 	return setValidInitialPosition(pp);
	// }

	inline bool setValidInitialPositionMarsupial(Vector3 p1,Vector3 p2)
	{

		DiscretePosition pp1 = discretizePosition(p1);
		DiscretePosition pp2 = discretizePosition(p2);

		return setValidInitialPositionMarsupial(pp1,pp2);
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
			ROS_WARN("RRTStar: Final position outside the workspace attempt!! [%d, %d, %d]", p.x, p.y, p.z);
		}

		return false;
	}
	inline bool setValidFinalPosition(Vector3 p)
	{
		DiscretePosition pp = discretizePosition(p);

		return setValidFinalPosition(pp);
	}
	bool searchInitialPosition3d(float maxDistance);	 // Symetric search in nearest XY-Zup neighbours
	bool searchFinalPosition3d(float maxDistance);					   // Symetric search in nearest XY-Zup neighbours
	
  	inline bool isInside(Vector3 &v)
	{
		int x_ = v.x * step_inv;
		int y_ = v.y * step_inv;
		int z_ = v.z * step_inv;
		return isInside(x_, y_, z_);
	}

	Vector3 getInitialPosition();
	Vector3 getFinalPosition();

		/**
		  Override actual occupancy matrix
		   @param octomap msg
		**/
	void updateMap(octomap_msgs::OctomapConstPtr message);

	/**
		  Override actual occupancy matrix simplify
		   @param octomap msg
		   @param goal_
		   @param start_
		   @param rpy_
		   @param full_ray_cast
		   @param ray_cast_free
		   @param ray_cast_free_reduce
		   @param ray_cast_coll
		   @param no_ray_cast_free
		**/
	octomap::OcTree updateMapReduced(octomap_msgs::OctomapConstPtr msg, 
							geometry_msgs::Vector3Stamped goal_, 
							geometry_msgs::Vector3Stamped start_, 
							std::vector<octomap::point3d> &full_ray_cast, 
							std::vector<octomap::point3d> &ray_cast_free,
							std::vector<octomap::point3d> &ray_cast_free_reduce,  
							std::vector<octomap::point3d> &ray_cast_coll, 
							std::vector<octomap::point3d> &no_ray_cast_free);

	/**
		  Add a cloud to the actual occupancy matrix
		   @param pcl pointCloud 
		**/
	void updateMap(PointCloud cloud);
	/**
		 * 
		 * 
		**/
	void updateMap(const PointCloud::ConstPtr &map);

	/** 
		   Clear occupancy discrete matrix
		**/
	void clearMap();
	/** 
		   Clear occupancy discrete matrix reduced
		**/
	void clearMapReduced(size_t _size);
		/**
		  Set timeout to calculate the solution
			@param Natural number with time in seconds
		**/
	void setTimeOut(int sec);
	/**
		  Publish via topic the discrete map constructed
		**/
	virtual void publishOccupationMarkersMap();
	
	void configCatenaryCompute(bool _u_c, bool _u_s_p, double _mf, double _l_m, 
								geometry_msgs::Vector3 _p_reel , geometry_msgs::Vector3 _p_ugv, geometry_msgs::Quaternion _r_ugv,
							   	bool coupled_, int n_iter_ , double r_nn_, double s_s_, int s_g_r_);

	void readPointCloudMap(const sensor_msgs::PointCloud2::ConstPtr& msg);



	/** Variables **/

	float step; // Resolution of the Matrix and its inverse
	float step_inv;

	//Shearching Pyramid parameters
	std::vector<double> length_catenary;
	std::vector<double> length_catenary_aux;
	geometry_msgs::Vector3 pos_reel_ugv , pos_tf_ugv;
	geometry_msgs::Quaternion rot_tf_ugv;
	geometry_msgs::Vector3 new_start, new_goal;
	Eigen::Matrix3f base_sp;
	double angle_square_pyramid, max_theta_axe_reduced, sweep_range;
	double phi_min, phi_max, theta_min, theta_max ;

	NearNeighbor near_neighbor_nodes_ugv, near_neighbor_nodes_uav;
	NearNeighbor near_neighbor_obstacles;
	// std::vector<Eigen::Vector3d> v_node_kdtree_ugv, v_node_kdtree_uav;

	pointVec v_node_kdtree;

	RRTNode *disc_initial, *disc_final; // Discretes
	RRTNode *disc_goal; // That node is fill it by the node  that approach the goal in Independent configuration



protected:
	
	RRTNode getRandomNode(bool go_to_goal_ = false); 
	bool extendGraph(const RRTNode q_rand_);
	RRTNode* getNearestNode(const RRTNode q_rand_);
  	RRTNode steering(const RRTNode &q_nearest_, const RRTNode &q_rand_, float factor_steer_);
	bool obstacleFree(const RRTNode q_nearest, const RRTNode q_new);
	// std::vector<Eigen::Vector3d> getNearNodes(const RRTNode &q_nearest_, const RRTNode &q_new_, double radius_, bool check_uav_ =false);
	std::vector<std::vector<int>> getNearNodes(const RRTNode &q_new_, double radius_);
	bool checkUGVFeasibility(const RRTNode pf_, bool ugv_above_z_);
	bool checkPointFeasibility(const RRTNode pf_ , bool check_uav_);
	bool checkPointsCatenaryFeasibility(const geometry_msgs::Point pf_);
	bool checkCatenary(RRTNode &q_init_, int mode_);
	geometry_msgs::Point getReelNode(const RRTNode &node_);
	geometry_msgs::Vector3 getReelTfInNode(const RRTNode &q_init_);
	void updateKdtree(const RRTNode ukT_);
	void getParamsNode(RRTNode &node_, bool is_init_= false);
	bool saveNode(RRTNode* sn_, bool is_init_=false);
	void saveTakeOffNode(RRTNode* sn_);
	double costNode(const RRTNode q_new_);
	double costBetweenNodes(const RRTNode q_near_, const RRTNode q_new_);
  	
	void isGoal(const RRTNode st_);
  	std::list<RRTNode*> getPath();

	/**
		 Get discrete occupancy matrix index for this discrete (x,y,z) position
		   @param x, y, z discrete position values
		   @return the index in the occupancy matrix
		**/
	inline unsigned int getWorldIndex(int &x, int &y, int &z)
	{
		return (unsigned int)((x - ws_x_min_inflated) + (Lx) * ((y - ws_y_min_inflated) + (Ly) * (z - ws_z_min_inflated)));
	}
  /**
	 Check if a Node/continuousPosition/discretePosition is inside the worksspace
	   @param ThetaStar Node / Discrete position
	   @return true if is inside the ws
	**/
	inline bool isInside(RRTNode n_)
	{
		return isInside(n_.point.x, n_.point.y, n_.point.z);
	}
	inline bool isInside(int x, int y, int z)
	{
		return (x < (ws_x_max - 1) && x > (ws_x_min + 1)) &&
			   (y < (ws_y_max - 1) && y > (ws_y_min + 1)) &&
			   (z < (ws_z_max - 1) && z > (ws_z_min + 1));
	}
  
	DiscretePosition discretizePosition(Vector3 p);

	bool isInitialPositionOccupied(bool check_uav_);
	bool isFinalPositionOccupied();

		/** 
		 Check if a node is occupied
		   @param A RRTNode
		   @return true if is occupied in the occupancy matrix
		**/
	bool isOccupied(RRTNode n_, bool check_uav_ = false);

  

	/**
		  Inflate a occupied cells filling all cells around in the occupancy matrix
		  Improvement: fast version using memset() to set to zero (.notOccupied) all 
		  occupation matrix nodes that have to be inflated
			@param discrete position of the cell to inflate		
		**/
	inline void inflateNodeAsCube(int &x_, int &y_, int &z_)
	{
		// Inflation limits around the node
		int x_inflated_max = (x_ + h_inflation) + 1;
		int x_inflated_min = (x_ - h_inflation) - 1;
		int y_inflated_max = (y_ + h_inflation) + 1;
		int y_inflated_min = (y_ - h_inflation) - 1;
		int z_inflated_max = (z_ + v_inflation) + 1;
		int z_inflated_min = (z_ - v_inflation) - 1;

		// Loop 'x axis' by 'x axis' for all discrete occupancy matrix cube around the node that must be inflated

		// Due to the inflated occupancy matrix size increment, the inside checking is not neccesary
		for (int i = x_inflated_min; i < x_inflated_max; i++)
			for (int j = y_inflated_min; j <= y_inflated_max; j++)
				for (int k = z_inflated_min; k <= z_inflated_max; k++)
				{
					unsigned int world_index_ = getWorldIndex(i, j, k);
					discrete_world[world_index_].notOccupied = false;
				}
	}

	/**
		  Inflate a occupied cells filling all cells inside the around cylinder in 
		  the occupancy matrix. 
			@param discrete position of the cell to inflate
		**/
	inline void inflateNodeAsCylinder(int &x_, int &y_, int &z_)
	{
		// Get discretized radius of the inflation cylinder
		int R = h_inflation;

		// Inflation limits around the node
		int x_inflated_max = (x_ + h_inflation) + 1;
		int x_inflated_min = (x_ - h_inflation) - 1;
		int y_inflated_max = (y_ + h_inflation) + 1;
		int y_inflated_min = (y_ - h_inflation) - 1;
		int z_inflated_max = (z_ + v_inflation) + 1;
		int z_inflated_min = (z_ - v_inflation) - 1;

		// Loop throug inflation limits checking if it is inside the cylinder
		// Due to the inflated occupancy matrix size increment, the inside checking is not neccesary
		for (int i = x_inflated_min; i <= x_inflated_max; i++)
			for (int j = y_inflated_min; j <= y_inflated_max; j++)
				for (int k = z_inflated_min; k <= z_inflated_max; k++)
				{
					if (isInsideTheCylinder(i, j, x_, y_, R))
					{
						unsigned int world_index_ = getWorldIndex(i, j, k);
						discrete_world[world_index_].notOccupied = false;
					}
				}
	}

	/**
		  Inflate a occupied cells filling all cells around in the occupancy matrix ONLY HORIZONTALLY
			@param discrete position of the cell to inflate
		**/
	inline void inflateNodeAsXyRectangle(int &x_, int &y_, int &z_)
	{
		// Inflation limits
		int x_inflated_max = (x_ + h_inflation) + 1;
		int x_inflated_min = (x_ - h_inflation) - 1;
		int y_inflated_max = (y_ + h_inflation) + 1;
		int y_inflated_min = (y_ - h_inflation) - 1;

		// Due to the inflated occupancy matrix size increment, the inside checking is not neccesary
		for (int i = x_inflated_min; i <= x_inflated_max; i++)
			for (int j = y_inflated_min; j <= y_inflated_max; j++)
			{
				unsigned int world_index_ = getWorldIndex(i, j, z_);
				discrete_world[world_index_].notOccupied = false;
			}
	}

	/**
		  Check if the [x,y] position is inside the cylinder [xo,yo,R] 
		  (really is circle, yes...)
			@return true if it is inside
		**/
	inline bool isInsideTheCylinder(int &x, int &y, int &xo, int &yo, int &R)
	{
		int R_ = sqrt((x - xo) * (x - xo) + (y - yo) * (y - yo));

		if (R_ <= R)
			return true;
		else
			return false;
	}

	/** Variables **/

	ros::NodeHandlePtr nh; // Pointer to the process NodeHandle to publish topics

	PointCloud occupancy_marker; // Occupancy Map as PointCloud markers
	ros::Publisher occupancy_marker_pub_;

	RVizMarker markerRviz;	 // Explored nodes by ThetaStar


	bool got_to_goal = false;
	float goal_weight;   // Reduction of the initial position distance weight C(s) = factor * g(s) + h(s)
	float z_weight_cost; // Weight for height changes
	float z_not_inflate; // Altitude to not be inflated
	double minR;

	octomap::OcTree *map;
	std::vector<geometry_msgs::Point> v_points_ws_ugv;
  	ros::Publisher tree_rrt_star_ugv_pub_,tree_rrt_star_uav_pub_, take_off_nodes_pub_,lines_ugv_marker_pub_, lines_uav_marker_pub_, catenary_marker_pub_, all_catenary_marker_pub_;
  	ros::Publisher goal_point_pub_, rand_point_pub_, one_catenary_marker_pub_ , points_marker_pub_;
	visualization_msgs::MarkerArray pointTreeMarkerUGV, pointTreeMarkerUAV;
  	visualization_msgs::MarkerArray pointTakeOffMarker, lines_ugv_marker_, lines_uav_marker_, catenaryMarker, allCatenaryMarker;

	Vector3 initial_position_ugv, initial_position_uav, final_position;   // Continuous
	// RRTNode *disc_initial, *disc_final; // Discretes
	double goal_gap_m;

	std::list<RRTNode*> rrt_path;

	// Max time to get path
	int timeout;

	std::vector<RRTStarNodeLink3D> discrete_world; // Occupancy Matrix and its size

	int matrix_size;
	int K, n_iter;
	int ws_x_max, ws_y_max, ws_z_max; // WorkSpace lenghts from origin (0,0,0)
	int ws_x_min, ws_y_min, ws_z_min;
	int h_inflation; // Inflation (Real and Safe distances from the MAV CoG)
	int v_inflation;
	int ws_x_max_inflated, ws_y_max_inflated, ws_z_max_inflated; // Inflated WorkSpace, the real size of the Occupancy Matrix
	int ws_x_min_inflated, ws_y_min_inflated, ws_z_min_inflated; // ... less isInside() checking
	int Lx, Ly, Lz;												 // Inflated WorkSpace lenghts and theirs pre-computed inverses
	float Lx_inv, Ly_inv, Lz_inv;
	std::string frame_id;
	bool debug, is_coupled;
	bool use_catenary, use_search_pyramid;
	double multiplicative_factor, length_tether_max, radius_near_nodes, step_steer;
	int samp_goal_rate;
};

}
#endif
