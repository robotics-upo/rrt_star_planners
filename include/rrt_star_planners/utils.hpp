/**
 * @file utils.hpp
 * @author vss2sn
 * @brief Contains common/commonly used funtions and classes
 */

#ifndef UTILS_H
#define UTILS_H

#include <vector>

#define BLACK "\x1b[1;30m"
#define RED "\x1b[1;31m"
#define GREEN "\x1b[1;32m"
#define YELLOW "\x1b[1;33m"
#define BLUE "\x1b[1;34m"
#define MAGENTA "\x1b[1;35m"
#define CYAN "\x1b[1;36m"
#define WHITE "\x1b[1;37m"
#define RESET "\x1b[1;0m"

/**
 * @brief NodeRRT class
 * @param x_ X value
 * @param y_ Y value
 * @param cost_ Cost to get to this node
 * @param h_cost_ Heuritic cost of this node
 * @param id_ Node's id
 * @param pid_ Node's parent's id
 */
class NodeRRT {
  // Variables used here are constantly accessed and checked; leaving public for
  // now.
 public:
  int point_x;
  int point_y;
  int point_z;
  int id;
  NodeRRT* parent;
  int p_id;
  std::vector<NodeRRT*> children;
  double cost;
  double h_cost;

  /**
   * @brief Constructor for Node class
   * @param x X value
   * @param y Y value
   * @param cost Cost to get to this node
   * @param h_cost Heuritic cost of this node
   * @param id Node's id
   * @param pid Node's parent's id
   */
  NodeRRT(const int x = 0, const int y = 0, const int z = 0, const int yaw = 0, 
      const double cost = 0, const double h_cost = 0, const int id = 0, const int pid = 0);

  /**
   * @brief Prints the values of the variables in the node
   * @return void
   */
  void PrintStatus() const;

  /**
   * @brief Overloading operator + for Node class
   * @param p node
   * @return Node with current node's and input node p's values added
   */
  NodeRRT operator+(const NodeRRT& p) const;

  /**
   * @brief Overloading operator - for Node class
   * @param p node
   * @return Node with current node's and input node p's values subtracted
   */
  NodeRRT operator-(const NodeRRT& p) const;

  /**
   * @brief Overloading operator = for Node class
   * @param p node
   * @return void
   */
  // void operator=(const Node& p);

  /**
   * @brief Overloading operator == for Node class
   * @param p node
   * @return bool whether current node equals input node
   */
  // bool operator==(const Node& p) const;

  /**
   * @brief Overloading operator != for Node class
   * @param p node
   * @return bool whether current node is not equal to input node
   */
  // bool operator!=(const Node& p) const;
};

/**
 * @brief Struct created to encapsulate function compare cost between 2 nodes.
 * Used in with multiple algorithms and classes
 */
struct compare_cost {
  /**
   * @brief Compare cost between 2 nodes
   * @param p1 Node 1
   * @param p2 Node 2
   * @return Returns whether cost to get to node 1 is greater than the cost to
   * get to node 2
   */
  bool operator()(const NodeRRT& p1, const NodeRRT& p2) const;
};

/**
 * @brief Get permissible motion primatives for the bot
 * @return vector of permissible motions
 */
std::vector<NodeRRT> GetMotion();

/**
 * @brief Prints the grid passed
 * @param grid Modify this grid
 * @return void
 */
void PrintGrid(const std::vector<std::vector<int>>& grid);

/**
 * @brief Prints the grid passed, highlighting the path taken
 * @param path_vector the path vector
 * @param start_ start node
 * @param goal_ goal node
 * @param grid Modify this grid
 * @return void
 */
void PrintPath(std::vector<NodeRRT>& path_vector, const NodeRRT& start_,
               const NodeRRT& goal_, std::vector<std::vector<int>>& grid);

/**
 * @brief Prints out the cost for reaching points on the grid in the grid shape
 * @param grid Grid on which algorithm is running
 * @param point_list Vector of all points that have been considered. Nodes in
 * vector contain cost.
 * @return void
 */
void PrintCost(const std::vector<std::vector<int>>& grid,
               const std::vector<NodeRRT>& point_list);

/**
 * @brief Creates a random grid of a given size
 * @param grid Modify this grid
 * @return void
 */
void MakeGrid(std::vector<std::vector<int>>& grid);
/**
 * @brief Prints the grid passed, highlighting the path taken, when the vector
 * is the path taken in order
 * @param path_vector the path vector
 * @param start start node
 * @param goal goal node
 * @param grid Modify this grid
 * @return void
 */
void PrintPathInOrder(const std::vector<NodeRRT>& path_vector, const NodeRRT& start,
                      const NodeRRT& goal, std::vector<std::vector<int>>& grid);

bool compareCoordinates(const NodeRRT& p1, const NodeRRT& p2);

bool checkOutsideBoundary(const NodeRRT& node, const int n);
#endif  // UTILS_H
