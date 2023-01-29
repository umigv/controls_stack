#include "global_static_planner.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h" // the msg type we decide for the publisher
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

class Global_Static_Planner {
    private:
        geometry_msgs::PoseStamped local_pos;
        nav_msgs::Odometry local_odom;
        geometry_msgs::PoseStamped local_origin;

        geometry_msgs::PoseStamped global_pos;
        nav_msgs::Odometry global_odom;
        geometry_msgs::PoseStamped global_origin;

        std::pair<int, int> local_vec; //connects local pos to local origin
        std::pair<int, int> origin_vec; //connects origin of local and global map


    public:

        //initialize variables 
        Global_Static_Planner();

        // recieve local map and determine our position relative to local map origin
        std::pair<int, int> localPos(nav_msgs::OccupancyGrid local_map);

        //then find position from local map origin to global map origin
        std::pair<int, int> originTransform();

        // add vectors of local->origin
        // transform from the local map to the global static map
        std::pair<int, int> mapTransform();
        
        // helper function to be added..
        //add position into the global occupancy map
        nav_msgs::OccupancyGrid 

};

// recieve local map and determine our position relative to local map origin
std::pair<int, int> localPos(nav_msgs::OccupancyGrid local_map) {
    
}

void map_callback(const std_msgs::String::ConstPtr &msg) {
    // print the message:
    // CHANGE LATER
    ROS_INFO("Received map info..");
}

void odom_callback() (const std_msgs::String::ConstPtr &msg) {
    // print the message:
    // CHANGE LATER
    ROS_INFO("Received odom info..");
}


//subscriber node
int main(int argc, char** argv) {
    ros::init(argc,argv,"global_static_planner");

    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("map", 1000, map_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odom_callback);

    ros::Publisher gsp_pub = nh.advertise<nav_msgs::OccupancyGrid>("gsp", 1000);

    ros::Rate rate(10);

    while(ros::ok()){
        nav_msgs::OccupancyGrid gsp_msg;
        
        // recieve local map and determine our position relative to local map origin
        

        //then find position from local map origin to global map origin

        //publish our location in global map

        gsp_pub.publish(gsp_msg);

        ros::spinOnce();
        ros.sleep();

    }
}
