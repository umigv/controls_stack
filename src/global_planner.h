#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>


static const int GLOBAL_HEIGHT = 1000;
static const int GLOBAL_WIDTH = 1000;

static const std::pair<int, int> goal = {999, 999};
static const std::pair<int, int> start = {0, 0};


class GlobalPlanner {
private: 

  nav_msgs::OccupancyGrid global_map; 
    
  std::vector<std::pair<int, int>> path;

  std::vector<std::pair<int, int>> waypoints;
    
  int height;

  int width;

public:

  // Ctor
  GlobalPlanner(int height_in, int width_in, std::vector<std::pair<int, int>> waypoints_in);



  // Fits incoming local_map into global_map
  void updateGlobalMap(nav_msgs::OccupancyGrid local_map);

  // Returns true if the current path is still open; false if the current path is blocked by an obstacle
  bool checkPath(std::vector<std::pair<int,int>> path);

  // Calculates path if necessary




}; // class GlobalPlanner