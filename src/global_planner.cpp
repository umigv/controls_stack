#include "global_planner.h"
#include "rpastar.h"

//Need_further_discussion_on_the_threshold
static const int threshold = 0;


GlobalPlanner::GlobalPlanner(int height_in, int width_in, std::vector<std::pair<int, int>>& waypoints_in) 
        : height{height_in}, width{width_in}, waypoints(waypoints_in) { 
        // Fills global map with -1s to start
        for(int i = 0; i < height; ++i) {
          for(int j = 0; j < width; ++j) {
            global_map[i][j] = -1;
          }
        }
        
        //init path straight to goal
        rpastar runner{start, goal, }
        
    } // GlobalPlanner()


bool GlobalPlanner::checkPath(){
    for (auto currentPos : path) {
      int x = currentPos.first;
      int y = currentPos.second;
      if (global_map[x][y] > threshold) {
        return false; //path not clear
      }
    }
    return true; //path is clear
}

