/**
 * @file rrt_star.h
 * @author vss2sn
 * @brief Contains the RRT_Star class
 */

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "rrt_star_planners/rrt_star.hpp"

// constants
constexpr double half_grid_unit = 0.5;
constexpr double tol_l_limit = 0.000001;


RRTStar::RRTStar()
{
}

// Constructor with arguments
RRTStar::RRTStar(std::string plannerName, std::string frame_id, 
				float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_, 
				float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, 
				ros::NodeHandlePtr n)
{
	// Call to initialization
	init(plannerName, frame_id, ws_x_max_, ws_y_max_, ws_z_max_, ws_x_min_, ws_y_min_, ws_z_min_, step_, h_inflation_, v_inflation_, goal_weight_, z_weight_cost_, z_not_inflate_, n);
}

// Initialization: creates the occupancy matrix (discrete nodes) from the bounding box sizes, resolution, inflation and optimization arguments
void RRTStar::init(std::string plannerName, std::string frame_id, 
					float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_,
					float step_, float h_inflation_, float v_inflation_, float goal_weight_, float z_weight_cost_, float z_not_inflate_, 
					ros::NodeHandlePtr n)
{
	// Pointer to the nodeHandler
	nh = n;
	// Not target initially
	disc_initial = NULL;
	disc_final = NULL;
	// by default ~not timeout
	timeout = 100;
	// Init asymetric and inflated occupancy matrix
	ws_x_max = ((ws_x_max_ / step_) + 1);
	ws_y_max = ((ws_y_max_ / step_) + 1);
	ws_z_max = ((ws_z_max_ / step_) + 1);
	ws_x_min = ((ws_x_min_ / step_) - 1);
	ws_y_min = ((ws_y_min_ / step_) - 1);
	ws_z_min = ((ws_z_min_ / step_) - 1);

	step = step_;
	step_inv = 1.0 / step_;
	h_inflation = (int)(h_inflation_ / step_);
	v_inflation = (int)(v_inflation_ / step_);
	ws_x_max_inflated = (ws_x_max + 2 * h_inflation);
	ws_y_max_inflated = (ws_y_max + 2 * h_inflation);
	ws_z_max_inflated = (ws_z_max + 2 * v_inflation);
	ws_x_min_inflated = (ws_x_min - 2 * h_inflation);
	ws_y_min_inflated = (ws_y_min - 2 * h_inflation);
	ws_z_min_inflated = (ws_z_min - 2 * v_inflation);
	matrix_size = (abs(ws_x_max_inflated) - ws_x_min_inflated + 1) * (abs(ws_y_max_inflated) - ws_y_min_inflated + 1) * (abs(ws_z_max_inflated) - ws_z_min_inflated + 1);
	discrete_world.resize(matrix_size);
	printf("RRTStar (%s): Occupancy Matrix has %d nodes [%lu MB]\n", plannerName.c_str(), matrix_size, (uint_fast32_t)(matrix_size * sizeof(NodeRRT)) / (1024 * 1024));
	Lx = ws_x_max_inflated - ws_x_min_inflated + 1;
	Ly = ws_y_max_inflated - ws_y_min_inflated + 1;
	Lz = ws_z_max_inflated - ws_z_min_inflated + 1;
	Lx_inv = 1.0 / Lx;
	Ly_inv = 1.0 / Ly;
	Lz_inv = 1.0 / Lz;
	// Optimization
	goal_weight = goal_weight_;
	z_weight_cost = z_weight_cost_;
	// Floor condition
	z_not_inflate = z_not_inflate_;

	// if you want a trajectory, its params must be configured using setTrajectoryParams()
	trajectoryParamsConfigured = false;

}

std::vector<NodeRRT> RRTStar::computePath(std::vector<std::vector<int>>& grid,
                                    const NodeRRT& start_in, const NodeRRT& goal_in,
                                    int max_iter_x_factor,
                                    double threshold_in) {
  start_ = start_in;
  goal_ = goal_in;
  size_grid = grid.size();
  threshold_ = threshold_in;
  int max_iter = max_iter_x_factor * size_grid * size_grid;
  CreateObstacleList(grid);
  point_list_.push_back(start_);
  grid[start_.x_][start_.y_] = 2;
  int iter = 0;
  NodeRRT new_node = start_;
  if (CheckGoalVisible(new_node)) {
    found_goal_ = true;
  }




  while (true) {
    iter++;
    if (iter > max_iter) {
      if (!found_goal_) {
        NodeRRT no_path_node(-1, -1, -1, -1, -1, -1);
        point_list_.clear();
        point_list_.push_back(no_path_node);
      }
      return point_list_;
    }
    new_node = GenerateRandomNode();
    if (grid[new_node.x_][new_node.y_] == 1) {
      continue;
    }
    // Go back to beginning of loop if point is an obstacle
    NodeRRT nearest_node = FindNearestPoint(new_node);
    if (nearest_node.id_ == -1) {
      continue;
    }
    // Go back to beginning of loop if no near neighbour
    grid[new_node.x_][new_node.y_] = 2;
    // Setting to 2 implies visited/considered

    auto it_v = std::find_if(
        point_list_.begin(), point_list_.end(),
        [&](const NodeRRT& node) { return compareCoordinates(node, new_node); });
    if (it_v != point_list_.end() && new_node.cost_ < it_v->cost_) {
      point_list_.erase(it_v);
      point_list_.push_back(new_node);
    } else if (it_v == point_list_.end()) {
      point_list_.push_back(new_node);
    }
    Rewire(new_node);  // Rewire
    if (CheckGoalVisible(new_node)) {
      found_goal_ = true;
    }
    // Check if goal is visible
  }

}

NodeRRT RRTStar::FindNearestPoint(NodeRRT& new_node) {
  NodeRRT nearest_node(-1, -1, -1, -1, -1, -1);
  std::vector<NodeRRT>::const_iterator it_v;
  std::vector<NodeRRT>::const_iterator it_v_store;
  // NOTE: Use total cost not just distance
  auto dist = static_cast<double>(size_grid * size_grid);
  for (it_v = point_list_.begin(); it_v != point_list_.end(); ++it_v) {
    auto new_dist = static_cast<double>(
        std::sqrt((static_cast<double>(it_v->x_ - new_node.x_) *
                   static_cast<double>(it_v->x_ - new_node.x_)) +
                  (static_cast<double>(it_v->y_ - new_node.y_) *
                   static_cast<double>(it_v->y_ - new_node.y_))));
    if (new_dist > threshold_) {
      continue;
    }
    new_dist += it_v->cost_;

    if (CheckObstacle(*it_v, new_node)) {
      continue;
    }
    if (it_v->id_ == new_node.id_) {
      continue;
    }
    // The nearest nodes are stored while searching for the nearest node to
    // speed up th rewire process
    near_nodes_.push_back(*it_v);
    near_nodes_dist_.push_back(new_dist);
    if (it_v->pid_ == new_node.id_) {
      continue;
    }
    if (new_dist >= dist) {
      continue;
    }
    dist = new_dist;
    it_v_store = it_v;
  }
  if (dist != size_grid * size_grid) {
    nearest_node = *it_v_store;
    new_node.pid_ = nearest_node.id_;
    new_node.cost_ = dist;
  }
  return nearest_node;
}

bool RRTStar::CheckObstacle(const NodeRRT& n_1, const NodeRRT& n_2) const {
  if (n_2.y_ - n_1.y_ == 0) {
    double c = n_2.y_;
    for (const auto& obs_node : obstacle_list_) {
      if (!(((n_1.x_ >= obs_node.x_) && (obs_node.x_ >= n_2.x_)) ||
            ((n_1.x_ <= obs_node.x_) && (obs_node.x_ <= n_2.x_)))) {
        continue;
      }
      if (static_cast<double>(obs_node.y_) == c) {
        return true;
      }
    }
  } else {
    double slope = static_cast<double>(n_2.x_ - n_1.x_) /
                   static_cast<double>(n_2.y_ - n_1.y_);
    double c =
        static_cast<double>(n_2.x_) - slope * static_cast<double>(n_2.y_);
    for (const auto& obs_node : obstacle_list_) {
      if (!(((n_1.y_ >= obs_node.y_) && (obs_node.y_ >= n_2.y_)) ||
            ((n_1.y_ <= obs_node.y_) && (obs_node.y_ <= n_2.y_)))) {
        continue;
      }
      if (!(((n_1.x_ >= obs_node.x_) && (obs_node.x_ >= n_2.x_)) ||
            ((n_1.x_ <= obs_node.x_) && (obs_node.x_ <= n_2.x_)))) {
        continue;
      }
      std::vector<double> arr(4);
      // Using properties of a point and a line here.
      // If the obtacle lies on one side of a line, substituting its edge points
      // (all obstacles are grid sqaures in this example) into the equation of
      // the line passing through the coordinated of the two nodes under
      // consideration will lead to all four resulting values having the same
      // sign. Hence if their sum of the value/abs(value) is 4 the obstacle is
      // not in the way. If a single point is touched ie the substitution leads
      // ot a value under 10^-7, it is set to 0. Hence the obstacle has
      // 1 point on side 1, 3 points on side 2, the sum is 2 (-1+3)
      // 2 point on side 1, 2 points on side 2, the sum is 0 (-2+2)
      // 0 point on side 1, 3 points on side 2, (1 point on the line, ie,
      // grazes the obstacle) the sum is 3 (0+3)
      // Hence the condition < 3
      arr[0] = static_cast<double>(obs_node.x_) + half_grid_unit -
               slope * (static_cast<double>(obs_node.y_) + half_grid_unit) - c;
      arr[1] = static_cast<double>(obs_node.x_) + half_grid_unit -
               slope * (static_cast<double>(obs_node.y_) - half_grid_unit) - c;
      arr[2] = static_cast<double>(obs_node.x_) - half_grid_unit -
               slope * (static_cast<double>(obs_node.y_) + half_grid_unit) - c;
      arr[3] = static_cast<double>(obs_node.x_) - half_grid_unit -
               slope * (static_cast<double>(obs_node.y_) - half_grid_unit) - c;
      double count = 0;
      for (auto& a : arr) {
        if (std::fabs(a) <= tol_l_limit) {
          a = 0;
        } else {
          count += a / std::fabs(a);
        }
      }
      if (std::abs(count) < 3) {
        return true;
      }
    }
  }
  return false;
}

NodeRRT RRTStar::GenerateRandomNode() const {
  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr_x(ws_x_min, ws_x_max);  // define the range
  std::uniform_int_distribution<int> distr_y(ws_y_min, ws_y_max);  // define the range
  std::uniform_int_distribution<int> distr_z(ws_z_min, ws_z_max);  // define the range
  std::uniform_int_distribution<int> distr_yaw(0, 3.1415*2.0);  // define the range
  int x_ = distr_x(eng);
  int y_ = distr_y(eng);
  int z_ = distr_z(eng);
  int yaw_ = distr(eng);
  NodeRRT new_node(x_, y_ , z_, yaw_, 0, 0, size_grid * x + y + z, 0);
  return new_node;
}

void RRTStar::Rewire(const NodeRRT& new_node) {
  std::vector<NodeRRT>::iterator it_v;
  for (size_t i = 0; i < near_nodes_.size(); i++) {
    if (near_nodes_[i].cost_ > near_nodes_dist_[i] + new_node.cost_) {
      it_v = std::find_if(point_list_.begin(), point_list_.end(),
                          [&](const NodeRRT& node) {
                            return compareCoordinates(node, near_nodes_[i]);
                          });
      if (it_v != point_list_.end()) {
        it_v->pid_ = new_node.id_;
        it_v->cost_ = near_nodes_dist_[i] + new_node.cost_;
      }
    }
  }
  near_nodes_.clear();
  near_nodes_dist_.clear();
}

bool RRTStar::CheckGoalVisible(const NodeRRT& new_node) {
  if (!CheckObstacle(new_node, goal_)) {
    auto new_dist = static_cast<double>(
        std::sqrt(static_cast<double>((goal_.x_ - new_node.x_) *
                                      (goal_.x_ - new_node.x_)) +
                  static_cast<double>((goal_.y_ - new_node.y_) *
                                      (goal_.y_ - new_node.y_))));
    if (new_dist > threshold_) {
      return false;
    }
    new_dist += new_node.cost_;
    goal_.pid_ = new_node.id_;
    goal_.cost_ = new_dist;
    std::vector<NodeRRT>::iterator it_v;
    it_v = std::find_if(
        point_list_.begin(), point_list_.end(),
        [&](const NodeRRT& node) { return compareCoordinates(node, new_node); });
    if (it_v != point_list_.end() && goal_.cost_ < it_v->cost_) {
      point_list_.erase(it_v);
      point_list_.push_back(goal_);
    } else if (it_v == point_list_.end()) {
      point_list_.push_back(goal_);
    }
    return true;
  }
  return false;
}

void RRTStar::CreateObstacleList(std::vector<std::vector<int>>& grid) {
  for (int i = 0; i < size_grid; i++) {
    for (int j = 0; j < size_grid; j++) {
      if (grid[i][j] == 1) {
        NodeRRT obs(i, j, 0, 0, i * size_grid + j, 0);
        obstacle_list_.push_back(obs);
      }
    }
  }
}

void RRTStar::updateMap(octomap_msgs::OctomapConstPtr message)
{
	// Clear current map in the discrete occupancy
	clearMap();

	// Read occupation data from the octomap_server
	//octomap_msgs::binaryMsgToMap(message));
	map = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*message);
	/*
     * Update discrete world with the read octomap data
     */

	// Occupied and not occupied points, only for debug
	u_int64_t occupied_leafs = 0, free_leafs = 0;
	int nit = 0;

	// Read from first to the last leaf of the tree set its xyz (discretized) if it is occupied into the occupancy matrix

	if (map->begin_leafs() != NULL)
	{
		for (octomap::OcTree::leaf_iterator it = map->begin_leafs(), end = map->end_leafs(); it != end; ++it)
		{
			if (map->isNodeOccupied(*it))
			{
				// Get occupied cells
				float x_w = it.getX();
				float y_w = it.getY();
				float z_w = it.getZ();

				// Exact discretization
				int x_ = (int)(x_w * step_inv);
				int y_ = (int)(y_w * step_inv);
				int z_ = (int)(z_w * step_inv);

				// Set as occupied in the discrete matrix
				//ROS_INFO("[%.2f, %.2f, %.2f] step_inv: %.2f", x_w, y_w, z_w, step_inv);
				//ROS_INFO("Disc [%d, %d, %d]", x_, y_, z_);
				//usleep(5e4);
				if (isInside(x_, y_, z_))
				{
					++nit;
					//ROS_INFO(PRINTF_RED"[%.2f, %.2f, %.2f] step_inv: %.2f", x_w, y_w, z_w, step_inv);
					//ROS_INFO(PRINTF_RED"Disc [%d, %d, %d]", x_, y_, z_);
					//usleep(5e4);
					unsigned int world_index_ = getWorldIndex(x_, y_, z_);
					discrete_world[world_index_].notOccupied = false;
					//ROS_INFO(PRINTF_RED"Inside: [%.2f, %.2f, %.2f]", x_w,y_w, z_w);
					//
					// Inflates nodes
					if (h_inflation >= step || v_inflation >= step)
					{
						if (z_w > z_not_inflate)
							//inflateNodeAsCube(x_, y_, z_);
							//inflateNodeAsCylinder(x_, y_, z_);
							inflateNodeAsXyRectangle(x_, y_, z_);

						else
							inflateNodeAsCylinder(x_, y_, z_);
					}
				}

#ifdef PRINT_OCTREE_STATS
				occupied_leafs++;
#endif
			}
			else
			{
#ifdef PRINT_OCTREE_STATS
				free_leafs++;
#endif
			}
		}
	}

#ifdef PRINT_OCTREE_STATS
	std::cout << "Occupied cells: " << occupied_leafs << " NO occupied cells: " << free_leafs << "It counter: " << nit << std::endl;
	ROS_INFO("Occupied cells: %d\t No occupied cells: %d", occupied_leafs, free_leafs);
#endif
}

void RRTStar::updateMap(PointCloud cloud)
{
	/*
     * Update discrete world with the Point Cloud = ocuppieds cells
     */
	clearMap();
	double dist=0;
	
	BOOST_FOREACH (const pcl::PointXYZ &p, cloud.points)
	{
		// Get occupied points
		//const pcl::PointXYZ &p = cloud.points[it];
		float x_w = p.x;
		float y_w = p.y;
		float z_w = p.z;

		// Exact discretization
		int x_ = (int)(x_w * step_inv);
		int y_ = (int)(y_w * step_inv);
		int z_ = (int)(z_w * step_inv);

		if (isInside(x_, y_, z_))
		{
			unsigned int world_index_ = getWorldIndex(x_, y_, z_);
			dist=sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
			if(dist < minR){
				discrete_world[world_index_].notOccupied = true;
				continue;
			}
			else
				discrete_world[world_index_].notOccupied = false;
			
			// Inflates nodes
			if (h_inflation * step >= step || v_inflation * step >= step)
			{
				if (z_w > z_not_inflate)
				{
					inflateNodeAsCube(x_, y_, z_);
					//inflateNodeAsCylinder(x_, y_, z_);
					// inflateNodeAsXyRectangle(x_, y_, z_);
				}
				else
				{
					//inflateNodeAsXyRectangle(x_, y_, z_);
					inflateNodeAsCube(x_, y_, z_);
				}
			}
		}
	}
}

void RRTStar::updateMap(const PointCloud::ConstPtr &map)
{
	/*
     * Update discrete world with the Point Cloud = ocuppieds cells
     */
	BOOST_FOREACH (const pcl::PointXYZ &p, map->points)
	{
		// Get occupied points
		//const pcl::PointXYZ &p = cloud->points[it];
		float x_w = p.x;
		float y_w = p.y;
		float z_w = p.z;

		// Exact discretization
		int x_ = (int)(x_w * step_inv);
		int y_ = (int)(y_w * step_inv);
		int z_ = (int)(z_w * step_inv);

		if (isInside(x_, y_, z_))
		{
			unsigned int world_index_ = getWorldIndex(x_, y_, z_);
			discrete_world[world_index_].notOccupied = false;

			// Inflates nodes
			if (h_inflation * step >= step || v_inflation * step >= step)
			{
				if (z_w > z_not_inflate)
				{
					inflateNodeAsCube(x_, y_, z_);
					//inflateNodeAsCylinder(x_, y_, z_);
					// inflateNodeAsXyRectangle(x_, y_, z_);
				}
				else
				{
					inflateNodeAsCube(x_, y_, z_);
				}
			}
		}
	}
}

octomap::OcTree RRTStar::updateMapReduced(octomap_msgs::OctomapConstPtr msg, 
									geometry_msgs::Vector3Stamped goal_, 
									geometry_msgs::Vector3Stamped start_, 
									std::vector<octomap::point3d> &full_ray_cast, 
									std::vector<octomap::point3d> &ray_cast_free,
									std::vector<octomap::point3d> &ray_cast_free_reduce,  
									std::vector<octomap::point3d> &ray_cast_coll, 
									std::vector<octomap::point3d> &no_ray_cast_free)
{
	full_ray_cast.clear(); ray_cast_free.clear(); ray_cast_free_reduce.clear(); ray_cast_coll.clear(); no_ray_cast_free.clear();

	double _resolution = 0.2;
	double R_, theta_, phi_;

	new_start.x = start_.vector.x + ((start_.vector.x - goal_.vector.x) / sqrt(pow(start_.vector.x - goal_.vector.x,2)+pow(start_.vector.y - goal_.vector.y,2)+pow(start_.vector.z - goal_.vector.z,2)));
	new_start.y = start_.vector.y + ((start_.vector.y - goal_.vector.y) / sqrt(pow(start_.vector.x - goal_.vector.x,2)+pow(start_.vector.y - goal_.vector.y,2)+pow(start_.vector.z - goal_.vector.z,2)));
	new_start.z = start_.vector.z + ((start_.vector.z - goal_.vector.z) / sqrt(pow(start_.vector.x - goal_.vector.x,2)+pow(start_.vector.y - goal_.vector.y,2)+pow(start_.vector.z - goal_.vector.z,2)));
	new_goal.x = goal_.vector.x + 0.5*((goal_.vector.x - start_.vector.x) / sqrt(pow(goal_.vector.x - start_.vector.x,2)+pow(goal_.vector.y - start_.vector.y,2)+pow(goal_.vector.z - start_.vector.z,2)));
	new_goal.y = goal_.vector.y + 0.5*((goal_.vector.y - start_.vector.y) / sqrt(pow(goal_.vector.x - start_.vector.x,2)+pow(goal_.vector.y - start_.vector.y,2)+pow(goal_.vector.z - start_.vector.z,2)));
	new_goal.z = goal_.vector.z + 0.5*((goal_.vector.z - start_.vector.z) / sqrt(pow(goal_.vector.x - start_.vector.x,2)+pow(goal_.vector.y - start_.vector.y,2)+pow(goal_.vector.z - start_.vector.z,2)));

	std::vector<octomap::point3d> vrc_, points_rcnf_, points_rcf_ , points_rcc_;
	std::vector<octomap::point3d> map_simplify_free, map_simplify_occupied;
	vrc_.clear(); points_rcnf_.clear(); points_rcc_.clear(); points_rcf_.clear(); map_simplify_free.clear(); map_simplify_occupied.clear();
	octomap::point3d s_(new_start.x, new_start.y, new_start.z);
	octomap::point3d g_(new_goal.x, new_goal.y, new_goal.z);
	octomap::point3d r_, e_, c_;
	octomap::OcTree *map_msg;
	octomap::OcTree map_simplify (0.05f);
	octomap::OcTree map_simplify_reduced (0.05f);
	bool r_cast_coll;

	map_msg = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);

	angle_square_pyramid = M_PI/8;
	double hip_ = sqrt(pow(new_goal.x-new_start.x,2)+ pow(new_goal.y-new_start.y,2)+ pow(new_goal.z-new_start.z,2));
	sweep_range = hip_ * sin(angle_square_pyramid); 
	phi_min = -1.0*sweep_range ;	
	phi_max = sweep_range ;
	theta_min = -1.0* sweep_range;
	theta_max = sweep_range ;
	double z_max_obs = 0.0; 
	double max_z_to_explore = -100.0;

	bool obs_pyramd_floor = false;

	octomap::point3d Vgs(new_goal.x-new_start.x, new_goal.y-new_start.y, new_goal.z-new_start.z);

	//Get spherical parameter
	R_= sqrt(Vgs.x()*Vgs.x() + Vgs.y()*Vgs.y() + Vgs.z()*Vgs.z());

	if(Vgs.z() > 0.0)
		theta_ = atan(sqrt(Vgs.x()*Vgs.x()+Vgs.y()*Vgs.y())/(Vgs.z()));
	else if (Vgs.z() < 0.0)
		theta_ = M_PI - atan(sqrt(Vgs.x()*Vgs.x()+Vgs.y()*Vgs.y())/(Vgs.z()));
	else
		theta_ = 0.0;
	
	if (Vgs.y() >= 0.0 && Vgs.x()>= 0.0)
		phi_ = atan(fabs(Vgs.y()/Vgs.x()));
	else if (Vgs.y() >= 0.0 && Vgs.x() < 0.0)
		phi_ = M_PI - atan(fabs(Vgs.y()/Vgs.x()));
	else if (Vgs.y() < 0.0 && Vgs.x() < 0.0)
		phi_ = M_PI + atan(fabs(Vgs.y()/Vgs.x()));
	else
		phi_ = 2.0*M_PI - atan(fabs(Vgs.y()/Vgs.x()));

	//Transform cartesian coordinates to spherical vectorial base 
	float ur_x,ur_y,ur_z,ut_x,ut_y,ut_z,up_x,up_y,up_z;
	ur_x = sin(theta_) * cos(phi_);
	ur_y = sin(theta_) * sin(phi_);
	ur_z = cos(theta_);
	ut_x = cos(theta_) * cos(phi_);
	ut_y = cos(theta_) * sin(phi_);
	ut_z = -sin(theta_);
	up_x= -sin(phi_);
	up_y = cos(phi_);
	up_z = 0;
	base_sp << ur_x, ur_y, ur_z, 
			   ut_x, ut_y, ut_z, 
			   up_x, up_y, up_z;
    Eigen::Vector3f plane_sp;
	
	//1. Applying Ray Cast to detect 3D free collision space and with collision. The space where Ray cast has collision is used to set highest obstacle and reduce 3D space to set with full Ray 
	int counting3 = 0;
	for (double delta_theta = theta_min ; delta_theta <= theta_max; delta_theta = delta_theta + _resolution){
		for (double delta_phi = phi_min ; delta_phi <= phi_max; delta_phi = delta_phi + _resolution){	
			plane_sp << 0.0, 
						delta_theta, 
						delta_phi;
			Eigen::Matrix3f base_sp_inv =  base_sp.inverse();
			Eigen::Vector3f spheric_ = base_sp_inv * plane_sp;
			Eigen::Vector3f cartesian_;
			cartesian_ << spheric_(0,0)+new_goal.x,
						  spheric_(1,0)+new_goal.y, 
						  spheric_(2,0)+new_goal.z;
			octomap::point3d e_(cartesian_(0,0), cartesian_(1,0), cartesian_(2,0)); //current end(goal) point
			octomap::point3d d_(e_.x() - new_start.x , e_.y() - new_start.y , e_.z() - new_start.z ); //direction for rayCast

			r_cast_coll = map_msg->castRay(s_, d_, r_);

			// These four lines can be commented to improve compute time performance, are uncommented just to graph Ray-Casting complete
			map_msg->computeRay(s_, e_, vrc_);
			for (size_t i = 0; i < vrc_.size(); i ++){
				full_ray_cast.push_back(vrc_[i]);
			}
			
			if(!r_cast_coll){
				map_msg->computeRay(s_, e_, vrc_);
				for (size_t i = 0; i < vrc_.size(); i ++){
					ray_cast_free.push_back(vrc_[i]);
					map_simplify_free.push_back(vrc_[i]);
				}
				points_rcf_.push_back(e_);
			}
			else{
				double d1_ = pow(new_start.x - r_.x(),2) + pow(new_start.y - r_.y(),2) + pow(new_start.z - r_.z(),2);
				double d2_ = pow(new_start.x - e_.x(),2) + pow(new_start.y - e_.y(),2) + pow(new_start.z - e_.z(),2);
				if (d1_ >= d2_){
					map_msg->computeRay(s_, e_, vrc_);
					points_rcf_.push_back(e_);
					for (size_t i = 0; i < vrc_.size(); i ++){
						ray_cast_free.push_back(vrc_[i]);
						map_simplify_free.push_back(vrc_[i]);
					}
				}
				else{
					map_msg->computeRay(s_, r_, vrc_);
					points_rcnf_.push_back(e_);
					points_rcc_.push_back(r_);
					if ( (z_max_obs < r_.z()) && (d1_ < d2_*2.0/3.0)){
						z_max_obs = r_.z();
						max_z_to_explore = e_.z();
						max_theta_axe_reduced = spheric_(2,0);
					}
				}
			}
		}
	}

	if ( max_z_to_explore < new_goal.z){
		obs_pyramd_floor = true;
		max_theta_axe_reduced = sweep_range;
	}
	else{
		obs_pyramd_floor = false;
	}
	//2. Apply Ray to set and reduced free space between start and obstacle 
	for (size_t i = 0; i < points_rcc_.size(); i ++){
		map_msg->computeRay(s_, points_rcc_[i], vrc_);
		if ((points_rcc_[i].z() < z_max_obs || obs_pyramd_floor)){	
			for (size_t j = 0; j < vrc_.size(); j ++){
				ray_cast_coll.push_back(vrc_[j]);
				if (j < vrc_.size()-1)
					map_simplify_free.push_back(vrc_[j]);
				else
					map_simplify_occupied.push_back(vrc_[j]);
			}
		}
	}

	//3. Apply Ray to set reduced free space above highest collision 
	for (size_t i= 0 ; i < points_rcf_.size(); i++){
		octomap::point3d e_(points_rcf_[i].x(), points_rcf_[i].y(), points_rcf_[i].z()); //save the current end(goal) point
		map_msg->computeRay(s_, e_, vrc_);
		if ((points_rcf_[i].z() < max_z_to_explore || obs_pyramd_floor)){	
			counting3++;
			for (size_t i = 0; i < vrc_.size(); i ++){
				ray_cast_free_reduce.push_back(vrc_[i]);
				map_simplify_free.push_back(vrc_[i]);
			}
		}
	}

	//4. Process to get rayCast in opposite direction, from goal zone to start zone to fill the collision zone
	for (size_t i = 0; i < points_rcnf_.size(); i ++){
		if (points_rcnf_[i].z() < max_z_to_explore + _resolution/2.0 || obs_pyramd_floor){
			octomap::point3d d_(new_start.x - points_rcnf_[i].x(), new_start.y - points_rcnf_[i].y(), new_start.z - points_rcnf_[i].z()); //save the direction for rayCast
			map_msg->castRay(points_rcnf_[i], d_, c_,true);
			double d1_ = pow(new_start.x - points_rcnf_[i].x(),2) + pow(new_start.y - points_rcnf_[i].y(),2) + pow(new_start.z - points_rcnf_[i].z(),2);
			double d2_ = pow(c_.x() - points_rcnf_[i].x(),2) + pow(c_.y() - points_rcnf_[i].y(),2) + pow(c_.z() - points_rcnf_[i].z(),2);
			octomap::point3d new_s_(new_start.x,new_start.y,new_start.z);
			if (d2_ >= d1_ ){
				map_msg->computeRay(points_rcnf_[i], new_s_, vrc_);
			}
			else{
				map_msg->computeRay(points_rcnf_[i], c_, vrc_);
			}
			for (size_t j = 0; j < vrc_.size(); j++){
				no_ray_cast_free.push_back(vrc_[j]);
				if (j < vrc_.size()-1){
					map_simplify_free.push_back(vrc_[j]);
				}
				else{
					map_simplify_occupied.push_back(vrc_[j]);
				}
			}
		}	
	}

	//4. Create new octomap with desired space to navigate
	// insert some measurements of occupied cells
	for (size_t i=0 ; i < map_simplify_occupied.size() ; i++){
		map_simplify.updateNode(map_simplify_occupied[i], true); // integrate 'occupied' measurement
	}
  	// insert some measurements of free cells
	for (size_t i=0 ; i < map_simplify_free.size() ; i++){
		map_simplify.updateNode(map_simplify_free[i], false);  // integrate 'free' measurement
	}
	
	u_int64_t occupied_leafs = 0, free_leafs = 0;
	int nit = 0;

	std::vector<int> v_disc_x_, v_disc_y_, v_disc_z_;
	std::vector<int> v_disc_free_x_, v_disc_free_y_, v_disc_free_z_;
	v_disc_x_.clear(); v_disc_y_.clear(); v_disc_z_.clear();
	v_disc_free_x_.clear(), v_disc_free_y_.clear(), v_disc_free_z_.clear();
	bool match_;
	int x_, y_ , z_;
	float x_w, y_w, z_w;

	return map_simplify_reduced;
}

void RRTStar::publishOccupationMarkersMap()
{
	//~ occupancy_marker.points.clear();
	occupancy_marker.clear();
	for (int i = ws_x_min_inflated; i <= ws_x_max_inflated; i++)
		for (int j = ws_y_min_inflated; j <= ws_y_max_inflated; j++)
			for (int k = ws_z_min_inflated; k <= ws_z_max_inflated; k++)
			{
				unsigned int matrixIndex = getWorldIndex(i, j, k);

				if (!discrete_world[matrixIndex].notOccupied)
				{
					//~ geometry_msgs::Point point;
					pcl::PointXYZ point;
					point.x = i * step;
					point.y = j * step;
					point.z = k * step;
					//~ occupancy_marker.points.push_back(point);
					occupancy_marker.push_back(point);
				}
			}

	//    //Downsample published cloud
	//    cloud_voxel_grid.setLeafSize (0.2, 0.2, 0.2);
	//    PointCloud::Ptr cloud_ptr 			(new PointCloud);
	//    PointCloud::Ptr cloud_filtered_ptr 	(new PointCloud);
	//    *cloud_ptr = occupancy_marker;
	//    cloud_voxel_grid.setInputCloud (cloud_ptr);
	//    cloud_voxel_grid.filter (*cloud_filtered_ptr);
	//    occupancy_marker = *cloud_filtered_ptr;

	occupancy_marker_pub_.publish(occupancy_marker);
}

bool RRTStar::setInitialPosition(DiscretePosition p_)
{
	if (isInside(p_.x, p_.y, p_.z))
	{
		NodeRRT *initialNodeInWorld;
		// NodeRRT *initialNodeInWorld = &discrete_world[getWorldIndex(p_.x, p_.y, p_.z)];

		// if (initialNodeInWorld->node == NULL)
		// {
			// initialNodeInWorld->node = new NodeRRT();
			initialNodeInWorld->point.x = p_.x;
			initialNodeInWorld->point.x = p_.y;
			initialNodeInWorld->point.x = p_.z;

			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
		// }
		disc_initial = initialNodeInWorld->node;

		initial_position.x = p_.x * step;
		initial_position.y = p_.y * step;
		initial_position.z = p_.z * step;

		disc_initial->point = p_;
		disc_initial->parentNode = disc_initial;

		return true;
	}
	else
	{
		//~ std::cerr << "RRTStar: Initial point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
		disc_initial = NULL;
		return false;
	}
}

bool RRTStar::setInitialPosition(Vector3 p)
{
	initial_position = p;
	DiscretePosition p_ = discretizePosition(p);

	if (isInside(p_.x, p_.y, p_.z))
	{
		ThetaStartNodeLink3D *initialNodeInWorld = &discrete_world[getWorldIndex(
			p_.x,
			p_.y,
			p_.z)];

		if (initialNodeInWorld->node == NULL)
		{
			initialNodeInWorld->node = new ThetaStarNode3D();
			initialNodeInWorld->node->point.x = p_.x;
			initialNodeInWorld->node->point.x = p_.y;
			initialNodeInWorld->node->point.x = p_.z;

			initialNodeInWorld->node->nodeInWorld = initialNodeInWorld;
		}
		disc_initial = initialNodeInWorld->node;

		disc_initial->point = p_;
		disc_initial->parentNode = disc_initial;

		return true;
	}
	else
	{
		//ROS_ERROR("RRTStar: Initial point [%f, %f, %f] not valid.", p.x, p.y, p.z);
		disc_initial = NULL;
		return false;
	}
}

bool RRTStar::setFinalPosition(DiscretePosition p_)
{
	if (isInside(p_.x, p_.y, p_.z))
	{
		ThetaStartNodeLink3D *finalNodeInWorld = &discrete_world[getWorldIndex(
			p_.x,
			p_.y,
			p_.z)];

		if (finalNodeInWorld->node == NULL)
		{
			finalNodeInWorld->node = new ThetaStarNode3D();
			finalNodeInWorld->node->point.x = p_.x;
			finalNodeInWorld->node->point.x = p_.y;
			finalNodeInWorld->node->point.x = p_.z;

			finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
		}
		disc_final = finalNodeInWorld->node;

		final_position.x = p_.x * step;
		final_position.y = p_.y * step;
		final_position.z = p_.z * step;
		disc_final->point = p_;

		return true;
	}
	else
	{
		//~ std::cerr << "ThetaStar: Final point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
		disc_final = NULL;
		return false;
	}
}

bool RRTStar::setFinalPosition(Vector3 p)
{
	DiscretePosition p_ = discretizePosition(p);

	if (isInside(p_.x, p_.y, p_.z))
	{
		ThetaStartNodeLink3D *finalNodeInWorld = &discrete_world[getWorldIndex(
			p_.x,
			p_.y,
			p_.z)];

		if (finalNodeInWorld->node == NULL)
		{
			finalNodeInWorld->node = new ThetaStarNode3D();
			finalNodeInWorld->node->point.x = p_.x;
			finalNodeInWorld->node->point.y = p_.y;
			finalNodeInWorld->node->point.z = p_.z;

			finalNodeInWorld->node->nodeInWorld = finalNodeInWorld;
		}
		disc_final = finalNodeInWorld->node;

		final_position = p;
		disc_final->point = p_;

		return true;
	}
	else
	{

		//~ std::cerr << "ThetaStar: Final point ["<< p.x << ";"<< p.y <<";"<< p.z <<"] not valid." << std::endl;
		disc_final = NULL;
		return false;
	}
}

Vector3 RRTStar::getInitialPosition()
{
	return initial_position;
}

Vector3 RRTStar::getFinalPosition()
{
	return final_position;
}

DiscretePosition RRTStar::discretizePosition(Vector3 p)
{
	DiscretePosition res;

	res.x = p.x * step_inv;
	res.y = p.y * step_inv;
	res.z = p.z * step_inv;

	return res;
}

#ifdef BUILD_INDIVIDUAL
/**
 * @brief Script main function. Generates start and end nodes as well as grid,
 * then creates the algorithm object and calls the main algorithm function.
 * @return 0
 */
int main() {
  int size_grid = 11;
  std::vector<std::vector<int>> grid(size_grid, std::vector<int>(size_grid));
  MakeGrid(grid);

  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, size_grid - 1);  // define the range

  NodeRRT start(distr(eng), distr(eng), 0, 0, 0, 0);
  NodeRRT goal(distr(eng), distr(eng), 0, 0, 0, 0);

  start.id_ = start.x_ * size_grid + start.y_;
  start.pid_ = start.x_ * size_grid + start.y_;
  goal.id_ = goal.x_ * size_grid + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  // Make sure start and goal are not obstacles and their ids are correctly
  // assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid);

  RRTStar new_rrt_star;
  double threshold = 2;
  int max_iter_x_factor = 20;
  std::vector<NodeRRT> path_vector =
      new_rrt_star.computePath(grid, start, goal, max_iter_x_factor, threshold);
  PrintPath(path_vector, start, goal, grid);

  return 0;
}
#endif  // BUILD_INDIVIDUAL
