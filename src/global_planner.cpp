#include "global_planner.h"
#include "rpastar.h"

//Need_further_discussion_on_the_threshold
static const int threshold = 0;


GlobalPlanner::GlobalPlanner(int height_in, int width_in, const std::vector<std::pair<int, int>> & waypoints_in) 
        : height{height_in}, width{width_in}, waypoints(waypoints_in) { 
        // Fills global map with -1s to start
        for(int i = 0; i < height; ++i) {
          for(int j = 0; j < width; ++j) {
            this->at(i, j) = -1;
          }
        }
        
        //init path straight to goal
        rpastar runner(start, goal, &global_map);
        
    } // GlobalPlanner()

// Returns global_map Occupancy Grid
  nav_msgs::OccupancyGrid * GlobalPlanner:: getMap() {
    return  &global_map;
  }


bool GlobalPlanner::checkPath(){
    for (auto currentPos : path) {
      int row = currentPos.first;
      int col = currentPos.second;
      if (this->at(row, col) > threshold) {
        return false; //path not clear
      }
    }
    return true; //path is clear
}

double GlobalPlanner::cost_path(){
    double sum = 0;
    for (auto currentPos : path) {
      sum += double(currentPos.first + currentPos.second);
    }
    return sum; //path is clear
}




int8_t& GlobalPlanner::at(int row, int col) {
    int x = (row* width )+ col;
    return global_map.data.at(x);
  }

  void GlobalPlanner::setPath(const std::vector<std::pair<int, int>>& path_in) {
    path = path_in;
  }

std::vector<std::pair<int, int>> getPath();


