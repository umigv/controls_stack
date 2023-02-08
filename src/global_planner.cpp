#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "global_planner.h"
#include "rpastar.h"



GlobalPlanner::GlobalPlanner(int height_in, int width_in, const std::vector<std::pair<int, int>> & waypoints_in) 
        : height{height_in}, width{width_in}, waypoints(waypoints_in) { 
        // Fills global map with -1s to start
        // std::cout<<"gp ctor" << width << std::endl;
        global_map.info.width = width;
        global_map.info.height = height;
        
        for(int i = 0; i < height; ++i) {
          for(int j = 0; j < width; ++j) {
            global_map.data.push_back(-1);

          }
        }

        //init path straight to goal

        rpastar runner(start, waypoints[0], &global_map);
        runner.search();
        std::vector<std::pair<int,int>> init_path = runner.backtracker();
        path = init_path;
        // std::cout<< " printing path in gp ctor " << std::endl;

     for(auto i : path ) {
  
    std::cout<<i.first<< " " << i.second << std::endl;
    }

      //  std::cout<<"gp ctor end"<< std::endl;

    } // GlobalPlanner()

// Returns global_map Occupancy Grid
  nav_msgs::OccupancyGrid * GlobalPlanner:: getMap() {
    return  &global_map;
  }



//goes through the path and checks if its its clear 
bool GlobalPlanner::checkPath(){
      std::cout<< " printing path in check path " << std::endl;

     for(auto i : path ) {
  
      std::cout<<i.first<< " " << i.second << std::endl;
      }

    std::cout << "checking path...\n";
    for (auto currentPos : path) {
      int row = currentPos.first;
      int col = currentPos.second;
      int8_t check = this->at(row, col);
      check = check - '0'; // we subtracted '0' from check because check is an signed char, 
      // std::cout << "  occpancy grid " << std::to_string(check) << std::endl;
      // std::cout << "  threshold  grid " << std::to_string(threshold )<< std::endl;
      if (check != threshold) {
        std::cout << "path not clear\n";
        return false; //path not clear
      }
    }
    std::cout << "path clear\n";
    return true; //path is clear
}

double GlobalPlanner::cost_path(){
    double sum = 0;
    for (auto currentPos : path) {
      sum += double(currentPos.first + currentPos.second);
    }
    return sum; //path is clear
}

  void GlobalPlanner::updateGlobalMap(nav_msgs::OccupancyGrid local_map){
    
    global_map = local_map; 
    return;
  }




//returns a reference to a point o
int8_t& GlobalPlanner::at(int row, int col) {
   // std::cout << "at ," << row << " " << col << std::endl;
    int x = (row*width )+ col;
    return global_map.data.at(x);
  }

  void GlobalPlanner::setPath(const std::vector<std::pair<int, int>>& path_in) {
    path = path_in;
  }

std::vector<std::pair<int, int>>  GlobalPlanner::getPath() {
  return path;
}


#endif


// 5 54,40,0[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
// 5 54,40,0[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
// 5 54,40,0[0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0]
// 5 54,40,0[0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,1,0,0,0,0]
// 5 54,40,0[0,0,0,0,0,1,1,1,0,0,0,0,1,0,0,0,0,1,0,0,1,0,0,0,0]
// 5 54,40,0[0,0,0,0,0,1,1,1,0,0,0,0,1,0,0,1,0,1,0,0,1,0,0,0,0]