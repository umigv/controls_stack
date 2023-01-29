#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include <string>
#include "global_planner.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "rpastar.cpp"
#include <sstream>

class Global_Static_Planner {
    private:
        geometry_msgs::PoseStamped local_pos;
        nav_msgs::Odometry local_odom;
        geometry_msgs::PoseStamped local_origin;

        geometry_msgs::PoseStamped global_pos;
        nav_msgs::Odometry global_odom;
        geometry_msgs::PoseStamped global_origin;

        nav_msgs::OccupancyGrid local_map;


        //not sure if need to store these
        std::pair<int, int> local_vec; //connects local pos to local origin
        std::pair<int, int> origin_vec; //connects origin of local and global map


    public:

        void map_callback(const nav_msgs::OccupancyGrid &map_msg) { //may be pointers to a reference???
            // Initialize local_origin
            local_origin.pos.pose.pose.position.x = map_msg.info.origin.position.x;
            local_origin.pos.pose.pose.position.y = map_msg.info.origin.position.y;

            //also add local pos?

            // CHANGE LATER
            ROS_INFO("Received map info..");
        }

        void odom_callback() (const nav_msgs::Odometry &odom_msg) {
            // print the message:
            // CHANGE LATER
            local_odom.pose.pose.position.x = odom_msg.pose.pose.position.x;
            local_odom.pose.pose.position.y = odom_msg.pose.pose.position.y;

            ROS_INFO("Received odom info..");
        }

        // receive local map and determine the robot position relative to local map origin
        std::pair<int, int> localPos() {
          
            // calculate the coordinates of the robot in the local map
            int pos_x = (int)((local_odom.pose.pose.position.x - local_map.info.origin.position.x) / local_map.info.resolution);
            int pos_y = (int)((local_odom.pose.pose.position.y - local_map.info.origin.position.y) / local_map.info.resolution);
            
            std::pair<int, int> local_pos(pos_x, pos_y);
            return local_pos;
        }
        

        //then find position from local map origin to global map origin
        std::pair<int, int> originTransform() {
            //assuming already on correct map resolution
            std::pair<int, int> transform;

            //x
            local_x = local_origin.pose.point.x; 
            global_x = global_origin.pose.point.x; 
            transform.first = global_x - local_x;

            //y           
            local_y = local_origin.pose.point.y; 
            global_y = global_origin.pose.point.y; 
            transform.second = global_y - local_y;

            return transform;
        }

        // add vectors of local->origin
        // transform from the local map to the global static map
        std::pair<int, int> mapTransform() {
            std::pair<int, int> origin_transform = originTransform();
            std::pair<int, int> local_pos = localPos();

            std::pair<int, int> map_transform = origin_transform + local_pos;
            return map_transform;
        }
        
        // helper function to be added..
        //add position into the global occupancy map
        nav_msgs::OccupancyGrid getOccupancyGrid() {
            
        }
};



//publish transformation
