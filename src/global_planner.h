#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>


static const int GLOBAL_HEIGHT = 1000;
static const int GLOBAL_WIDTH = 1000;


class GlobalPlanner {
private: 

  std::vector<std::vector<int>> global_map; 
    
  std::vector<std::pair<int, int>> path;
    
  int height;

  int width;

public:

  // Ctor
  GlobalPlanner(int height_in, int width_in);


  void setPath(std::vector<std::pair<int,int>> path);

  // Fits incoming local_map into global_map
  void updateGlobalMap(nav_msgs::OccupancyGrid local_map);

  // Returns true if the current path is still open; false if the current path is blocked by an obstacle
  bool checkPath(std::vector<std::pair<int,int>> path);

  // Calculates path if necessary




}; // class GlobalPlanner