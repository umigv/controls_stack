#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "global_planner.h"

#include "geometry_msgs/Point.h"
#include "rpastar.h"

GlobalPlanner::GlobalPlanner(
    int height_in, int width_in,
    const std::vector<std::pair<int, int>>& waypoints_in)
    : height{height_in}, width{width_in}, waypoints(waypoints_in) {
    // Fills global map with -1s to start
    // std::cout<<"gp ctor" << width << std::endl;
    global_map.info.width = width;
    global_map.info.height = height;

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            global_map.data.push_back(-1);
        }
    }

    // init path straight to goal

    // std::cout<< " printing path in gp ctor " << std::endl;

    for (auto i : path) {
        std::cout << i.first << " " << i.second << std::endl;
    }

    //  std::cout<<"gp ctor end"<< std::endl;

}  // GlobalPlanner()

// Returns global_map Occupancy Grid
nav_msgs::OccupancyGrid* GlobalPlanner::getMap() { return &global_map; }

// goes through the path and checks if its its clear
bool GlobalPlanner::checkPath() {
    // If the path is empty, return false so we calculate an initial path
    if (path.empty()) {
        return false;
    }
        
    std::cout << " printing path in check path " << std::endl;

    for (auto i : path) {
        std::cout << i.first << " " << i.second << std::endl;
    }

    std::cout << "checking path...\n";
    for (auto currentPos : path) {
        int row = currentPos.first;
        int col = currentPos.second;
        int8_t check = this->at(row, col);
        check = check - '0';  // we subtracted '0' from check because check is
                              // an signed char,
        // std::cout << "  occpancy grid " << std::to_string(check) <<
        // std::endl; std::cout << "  threshold  grid " <<
        // std::to_string(threshold )<< std::endl;
        if (check != threshold) {
            std::cout << "path not clear\n";
            return false;  // path not clear
        }
    }
    std::cout << "path clear\n";
    return true;  // path is clear
}

double GlobalPlanner::cost_path() {
    double sum = 0;
    for (auto currentPos : path) {
        sum += double(currentPos.first + currentPos.second);
    }
    return sum;  // path is clear
}

void GlobalPlanner::updateGlobalMap(nav_msgs::OccupancyGrid local_map) {
    // Get map and current pose from Ben's node

    global_map = local_map;
    return;
}

std::pair<int, int> GlobalPlanner::getGoal() { return goal; }

std::pair<int, int> GlobalPlanner::getPose() { return pose; }

void GlobalPlanner::setGoal(std::pair<int, int> g) { goal = g; }

void GlobalPlanner::setPose(std::pair<int, int> s) { pose = s; }

// returns a reference to a point o
int8_t& GlobalPlanner::at(int row, int col) {
    // std::cout << "at ," << row << " " << col << std::endl;
    int x = (row * width) + col;
    return global_map.data.at(x);
}

void GlobalPlanner::setPath(const std::vector<std::pair<int, int>>& path_in) {
    path = path_in;
}

std::vector<std::pair<int, int>> GlobalPlanner::getPath() { return path; }

// Returns true if pose is close enough to current waypoint
bool GlobalPlanner::reachedGoal() {
    // TODO Will edit later
    int threshold = 3;
    int x_gap = waypoints[curr_waypoint].first - pose.first;
    int y_gap = waypoints[curr_waypoint].second - pose.second;
    return std::max(x_gap, y_gap) < threshold;
}

// Converts path, a vector of std::pairs, into a vector of PosedStamped messages
// for MoveBase
std::vector<geometry_msgs::PoseStamped> GlobalPlanner::convertPath() {
    std::vector<geometry_msgs::PoseStamped> pose_stamps;

    for (int i = 0; i < path.size() - 1; i++) {
        std::pair<int, int> coordinate = path.at(i);
        std::pair<int, int> next_corrdinate = path.at(i + 1);

        // PoseStamped is made up of a pose and a header

        geometry_msgs::Pose pose;
        // header is made up of a point (position) and and a quaternion
        // (orientation)
        // Constructs point, initializer list wouldn't work
        geometry_msgs::Point point;
        point.x =// /*
//  * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *   * Redistributions of source code must retain the above copyright notice,
//  *     this list of conditions and the following disclaimer.
//  *   * Redistributions in binary form must reproduce the above copyright
//  *     notice, this list of conditions and the following disclaimer in the
//  *     documentation and/or other materials provided with the distribution.
//  *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
//  *     contributors may be used to endorse or promote products derived from
//  *     this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  */

// // %Tag(FULLTEXT)%
// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include <vector>
// #include <string>
// #include "nav_msgs/OccupancyGrid.h"
// #include "nav_msgs/Odometry.h"
// #include "std_msgs/Header.h"
// #include "nav_msgs/MapMetaData.h"
// #include "nav_msgs/Path.h"
// #include "geometry_msgs/PoseStamped.h"
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include "rpastar.cpp"

// nav_msgs::OccupancyGrid process_array(const std_msgs::String::ConstPtr& msg);
// void chatterCallback(const std_msgs::String::ConstPtr& msg);
// nav_msgs::Path generate_path(std::vector<std::pair<int,int>> path);
// /**
//  * This tutorial demonstrates simple receipt of messages over the ROS system.
//  */

// class Listener
// {
// public:
//   void chatterCallbackTurtleBot(const nav_msgs::OccupancyGrid::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2);
//   void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg3);
//   void generate_path(std::vector<std::pair<int,int>> path, const nav_msgs::OccupancyGrid &map);
//   nav_msgs::Path get_path();
//   geometry_msgs::PoseStamped get_goal();

// private:
//   nav_msgs::Path nav_path;
//   geometry_msgs::PoseStamped goal_pose;
//   nav_msgs::OccupancyGrid map;
//   nav_msgs::Odometry pos;
// };

// nav_msgs::Path Listener::get_path()
// {
//   return nav_path;
// }

// geometry_msgs::PoseStamped Listener::get_goal()
// {
//   return goal_pose;
// }

// void Listener::generate_path(std::vector<std::pair<int,int>> path, const nav_msgs::OccupancyGrid &map)
// {
//   geometry_msgs::PoseStamped pose;
//   nav_path.header.frame_id = "map";
//   for (int i = 0; i < path.size(); i++)
//   {
//     float global_x = (path[i].first*map.info.resolution) + map.info.origin.position.x;
//     float global_y = (path[i].second*map.info.resolution) + map.info.origin.position.y;
//     pose.pose.position.x = global_y;
//     pose.pose.position.y = global_x;
//     pose.header.frame_id = "map";
//     nav_path.poses.push_back(pose);
//   }
// }

// void Listener::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg3)
// { 
//   goal_pose = *msg3;
//   int pos_x = (int)((pos.pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
//   int pos_y = (int)((pos.pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);

//   int goal_x = (int)((goal_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
//   int goal_y = (int)((goal_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);
//   // for (int i = 0; i < map.info.height; i++)
//   // {
//   //   for (int j = 0; j < map.info.width; j++)
//   //   {
//   //     std::cout << map.data[map.info.width*i + j] << "  ";
//   //   }
//   //   std::cout << std::endl;
//   // }
//   std::cout << pos_x << " " << pos_y << std::endl;
//   std::cout << goal_x << " " << goal_y << std::endl;
//   std::cout << "Running A*..." << std::endl << std::endl;
//   std::pair<int,int> start(pos_y, pos_x);
//   std::pair<int,int> end(goal_y, goal_x);
//   // rpastar runner = rpastar::rpastar(start, end, &map);
//   rpastar runner(start, end, &map);
//   runner.search();
//   nav_msgs::Path nav_path;
//   std::cout << "back to callback\n";
//   if (runner.goal_found())
//   {
//     std::vector<std::pair<int,int>> path = runner.backtracker();
//     std::cout << "Path found!" << std::endl;
//     generate_path(path, map);
//     // for (int i = 0; i < map.info.height; i++)
//     // {
//     //   for (int j = 0; j < map.info.width; j++)
//     //   {
//     //     std::pair<int, int> pair(i,j);
//     //     if (std::find(path.begin(), path.end(), pair) != path.end())
//     //     {
//     //       std:: cout << " *  ";
//     //     }
//     //     else
//     //     {
//     //       if ((int)map.data[map.info.width*i + j] > 0)
//     //       {
//     //         std::cout << " 1  ";
//     //       }
//     //       else if ((int)map.data[map.info.width*i + j] == 0)
//     //       {
//     //         std::cout << " 0  ";
//     //       }
//     //       else
//     //       {
//     //         std::cout << (int)map.data[map.info.width*i + j] << "  ";
//     //       }
//     //     }
//     //   }
//     //   std::cout << std::endl;
//     // }
//   }
//   else
//   {
//     std::cout << "No path found" << std::endl;

//   }
// }

// // %Tag(CALLBACK)%
// void Listener::chatterCallbackTurtleBot(const nav_msgs::OccupancyGrid::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2)
// { 
//   std::cout << "Updated Map and Odom" << std::endl;
//   map = *msg1;
//   pos = *msg2;
// }
// // %EndTag(CALLBACK)%

// // %Tag(CALLBACK)%
// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   //ROS_INFO("I heard: [%s]", msg->data.c_str());
//   std::cout << "Occupancy Grid received:" << std::endl;
//   nav_msgs::OccupancyGrid map = process_array(msg);
//   for (int i = 0; i < map.info.height; i++)
//   {
//     for (int j = 0; j < map.info.width; j++)
//     {
//       std::cout << map.data[map.info.width*i + j] << "  ";
//     }
//     std::cout << std::endl;
//   }
//   std::cout << "Running A*..." << std::endl << std::endl;
//   std::pair<int,int> start(msg->data[3]-'0', msg->data[4]-'0');
//   std::pair<int,int> end(msg->data[6]-'0', msg->data[7]-'0');
//   // rpastar runner = rpastar::rpastar(start, end, &map);
//   rpastar runner(start, end, &map);
//   runner.search();
//   std::vector<std::pair<int,int>> path = runner.backtracker();
//   std::cout << "Path found!" << std::endl;
//   for (int i = 0; i < map.info.height; i++)
//   {
//     for (int j = 0; j < map.info.width; j++)
//     {
//       std::pair<int, int> pair(i,j);
//       if (std::find(path.begin(), path.end(), pair) != path.end())
//       {
//         std:: cout << "*  ";
//       }
//       else
//       {
//         std::cout << map.data[map.info.width*i + j] << "  ";
//       }
//     }
//     std::cout << std::endl;
//   }
// }
// // %EndTag(CALLBACK)%

// nav_msgs::OccupancyGrid process_array(const std_msgs::String::ConstPtr& msg)
// {
//   std::string init_map = msg->data;
//   std::vector<signed char> vec;
//   nav_msgs::OccupancyGrid map;
//   int width = init_map[1]-'0';
//   int height = init_map[0]-'0';
//   map.info.width = width;
//   map.info.height = height;
//   init_map = init_map.substr(9);
//   for (auto &val : init_map)
//   {
//     if (!(val == ',' || val == '[' || val == ']'))
//     {
//         vec.push_back(val);
//     }
//   }
//   map.data = vec;

//   return map;
// }

// int main(int argc, char **argv)
// {
//   /**
//    * The ros::init() function needs to see argc and argv so that it can perform
//    * any ROS arguments and name remapping that were provided at the command line.
//    * For programmatic remappings you can use a different version of init() which takes
//    * remappings directly, but for most command-line programs, passing argc and argv is
//    * the easiest way to do it.  The third argument to init() is the name of the node.
//    *
//    * You must call one of the versions of ros::init() before using any other
//    * part of the ROS system.
//    */
//   ros::init(argc, argv, "listener");
//   /**
//    * NodeHandle is the main access point to communications with the ROS system.
//    * The first NodeHandle constructed will fully initialize this node, and the last
//    * NodeHandle destructed will close down the node.
//    */
//   ros::NodeHandle n;

//   /**
//    * The subscribe() call is how you tell ROS that you want to receive messages
//    * on a given topic.  This invokes a call to the ROS
//    * master node, which keeps a registry of who is publishing and who
//    * is subscribing.  Messages are passed to a callback function, here
//    * called chatterCallback.  subscribe() returns a Subscriber object that you
//    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
//    * object go out of scope, this callback will automatically be unsubscribed from
//    * this topic.
//    *
//    * The second parameter to the subscribe() function is the size of the message
//    * queue.  If messages are arriving faster than they are being processed, this
//    * is the number of messages that will be buffered up before beginning to throw
//    * away the oldest ones.
//    */
// // %Tag(SUBSCRIBER)%
//   ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/NavfnROS/plan", 1000);


// // %EndTag(SUBSCRIBER)%
//   Listener listener;
//   ros::Subscriber sub = n.subscribe("move_base_simple/goal", 1000, &Listener::goalCallback, &listener);
//   message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub(n, "map", 100);
//   message_filters::Subscriber<nav_msgs::Odometry> pos_sub(n, "odom", 100);
//   typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, nav_msgs::Odometry> MySyncPolicy;
//   message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), map_sub, pos_sub);
//   sync.registerCallback(boost::bind(&Listener::chatterCallbackTurtleBot, &listener, _1, _2));

//   // std::cout << path.poses[0].pose.position.x << " " << path.poses[0].pose.position.y << std::endl;
//   // std::cout << path.poses[1].pose.position.x << " " << path.poses[1].pose.position.y << std::endl;

//   /**
//    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
//    * callbacks will be called from within this thread (the main one).  ros::spin()
//    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
//    */
// // %Tag(SPIN)%  
//   ros::Rate r(10);
//   while (ros::ok())
//   {
//     nav_msgs::Path path = listener.get_path();
//     //geometry_msgs::PoseStamped goal_pose = listener.get_goal();
//     path_pub.publish(path);
//     //goal_pub.publish(goal_pose);
//     ros::spinOnce();
//     r.sleep();
//   }
// // %EndTag(SPIN)%

//   return 0;
// }
// // %EndTag(FULLTEXT)%
 double(coordinate.first);
        point.y = double(coordinate.second);
        point.z = 0.0;

        pose.position = point;

        // https://answers.ros.org/question/231941/how-to-create-orientation-in-geometry_msgsposestamped-from-angle/
        // Yaw angle is currently set to be the robot's angle relative to the y
        // axis TODO change if nessarcy
        pose.orientation = tf::createQuaternionMsgFromYaw(
            calcYaw(coordinate, next_corrdinate));

        // = {double(coordinate.first), double(coordinate.second),0};

        // TODO actualy add data to header
        std_msgs::Header header;
        geometry_msgs::PoseStamped curr_pose_stamp;
        curr_pose_stamp.header = header;
        curr_pose_stamp.pose = pose;
    }
    return pose_stamps;
}

// Place holder
// Inputs two int pairs, A (curr pose),, B (next pose)
double GlobalPlanner::calcYaw(std::pair<int, int> A, std::pair<int, int> B) {
    // the vector A (where the robot is facing) , 1,1 is the vector B because
    // that is "North"

    double mag =
        sqrt(pow(A.first, A.second)) *
        sqrt(pow(B.first, B.second));  // + sqrt(2) is the mag of unit vector
    double dotProduct =
        (A.first * B.first) +
        (A.second *
         B.second);  // dotProduct of anything times the unit vector is
    return acos(dotProduct / mag);
}

#endif

// 5 54,40,0[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
// 5 54,40,0[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
// 5 54,40,0[0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0]
// 5 54,40,0[0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,1,0,0,0,0]
// 5 54,40,0[0,0,0,0,0,1,1,1,0,0,0,0,1,0,0,0,0,1,0,0,1,0,0,0,0]
// 5 54,40,0[0,0,0,0,0,1,1,1,0,0,0,0,1,0,0,1,0,1,0,0,1,0,0,0,0]