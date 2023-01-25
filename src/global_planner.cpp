#include "global_planner.h"

//Need_further_discussion_on_the_threshold
static const int threshold = 0;


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
        Pair<int, int> c = path[i];
        path.push_back(c);
    }
    return;
}


bool GlobalPlanner::checkPath(std::vector<std::pair<int,int>> path){
    for (auto currentPos: path) {
      int x = currentPos.first();
      int y = currentPos.second();
      if (global_map[x][y] > threshold) {
        return false; //path not clear
      }
    }
    return true; //path is clear
}

