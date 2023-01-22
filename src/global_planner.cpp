#include "global_planner.h"


GlobalPlanner::GlobalPlanner(int height_in, int width_in) 
        : height{height_in}, width{width_in} { 
        // Fills global map with -1s to start
        for(int i = 0; i < height; ++i) {
          for(int j = 0; j < width; ++j) {
            global_map[i][j] = -1;
          }
        }
    } // GlobalPlanner()


GlobalPlanner::setPath(std::vector<std::pair<int,int>> path){
    for(int i = 0 ; i < path.size() ; i ++){

    }
}