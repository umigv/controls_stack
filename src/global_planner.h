#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "constants.h"
#include <vector>




static const std::pair<int, int> goal = {2, 2};
static const std::pair<int, int> start = {0, 0};


class GlobalPlanner {
private: 
  nav_msgs::OccupancyGrid global_map; 
 // const nav_msgs::OccupancyGrid::ConstPtr & map_ptr =  & global_map;

  std::vector<std::pair<int, int>> path;

  std::vector<std::pair<int, int>> waypoints;
    
  int height;

  int width;

public:

  // Ctor
  GlobalPlanner(int height_in, int width_in, const std::vector<std::pair<int, int>> & waypoints_in);

  // Returns global_map Occupancy Grid
  nav_msgs::OccupancyGrid * getMap();

   std::vector<std::pair<int, int>> getPath();
    double cost_path();

  // Fits incoming local_map into global_map
  void updateGlobalMap(nav_msgs::OccupancyGrid local_map);

  // Returns true if the current path is still open; false if the current path is blocked by an obstacle
  bool checkPath();

  // Calculates path if necessary

  // .at() abstracts indexing into the occupancy grid
  int8_t& at(int row, int col);


  void setPath(const std::vector<std::pair<int, int>>& path_in);

private:

  


}; // class GlobalPlanner