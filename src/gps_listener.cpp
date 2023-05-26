#include "ros/ros.h"
#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include <fstream>
#include <cmath>
#include <utility>
#include <deque>
#include <iomanip>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/Point.h"
#include "std_msgs/UInt32MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "tf2_msgs/TFMessage.h"
#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"

using std::string;
using std::pair;


// globalsZ
// std_msgs::float64 origin_x, origin_y;

struct Coordinate {
    Coordinate(double latIN, double longIN):latitude(latIN), longitude(longIN){}
    double latitude;
    double longitude;
};

// Struct for map metadata in more accessible format
struct OccGridInfo {

    OccGridInfo(float resolutionIn, uint32_t widthIn, uint32_t heightIn, float originXIn, float originYIN) {
        this->resolution = resolutionIn;
        this->width = widthIn;
        this->height = heightIn;
        this->origin_x = originXIn;
        this->origin_y = originYIN;
    }

    OccGridInfo() {
        resolution = 0;
        width = 0;
        height = 0;
        origin_x = 0;
        origin_y = 0;
    }

    void update(float resolutionIn, uint32_t widthIn, uint32_t heightIn, float originXIn, float originYIN) {
        this->resolution = resolutionIn;
        this->width = widthIn;
        this->height = heightIn;
        this->origin_x = originXIn;
        this->origin_y = originYIN;
    }

    float resolution;
    uint32_t width;
    uint32_t height;
    float origin_x;
    float origin_y;


};

class GPSdata {
public:
    //LATITUDE, LONGITUDE
    // This is for the final queue of x, y values to be sent to the global frame.
    std::deque<Coordinate> GOAL_POINTS; //This 
    std::deque<Coordinate> GOAL_GPS;
    // Get from tf transform function.
    // double rob_x, rob_y;
    sensor_msgs::NavSatFix gpsMsg;
    tf2_ros::Buffer *tfBuffer;
    // tf2_ros::TransformListener tfListener;
    uint32_t indexOfCurrentGoal = 1;
    OccGridInfo mapInfo;

    
    GPSdata(ros::NodeHandle nh_, tf2_ros::Buffer &tfBufferInstance) {

        this->tfBuffer = &tfBufferInstance;
        gps_sub = nh_.subscribe("/gps/fix", 100, &GPSdata::gpsCallback, this);
        map_sub = nh_.subscribe("/map", 100, &GPSdata::mapCallback, this); // Gets map metadata into mapInfo variable
        
        // cartographer_sub = nh_.subscribe("/tf_static", 100, &GPSdata::cartographerCallback, this); // gives x,y coords

    }
    
    //reads the text file of gps coords and returns correct x y coords
    void read_goal_coords() {

        // Coordinates inside but moved slightly
        GOAL_POINTS.emplace_back(42.294522, -83.708840);
        // Pretty far in one direction away
        GOAL_POINTS.emplace_back(42.5, -83.708840);
        // Less than a meter away
        GOAL_POINTS.emplace_back(42.2946, -83.708840);
        // About 8 meters away
        GOAL_POINTS.emplace_back(42.3, -83.8);
 
        GOAL_GPS = GOAL_POINTS;
    }


    void gps_transform() {
        for (size_t i = 0; i < GOAL_POINTS.size(); i++) {
            Coordinate x_point(gpsMsg.latitude, GOAL_POINTS[i].longitude);
            Coordinate y_point(GOAL_POINTS[i].latitude, gpsMsg.longitude);

            Coordinate currentLocation(gpsMsg.latitude, gpsMsg.longitude);

            double x_dist = distance_between_points(x_point, currentLocation);
            double y_dist = distance_between_points(y_point, currentLocation);
            
            GOAL_POINTS[i].latitude = x_dist;
            GOAL_POINTS[i].longitude = y_dist;
        }
    }
    
    
    // srv function boolean:
    bool service_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // bool service_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        string returnString = std::to_string(GOAL_POINTS.front().latitude) + "|" + std::to_string(GOAL_POINTS.front().longitude); //lat long issue?
        

        double robot_latitude = gpsMsg.latitude;
        double robot_longitude = gpsMsg.longitude;
        double goal_latitude = GOAL_GPS.front().latitude;
        double goal_longitude = GOAL_GPS.front().longitude;

        Coordinate xDifferenceCoordsRobot(robot_latitude, 0);
        Coordinate xDifferenceCoordsGoal(goal_latitude, 0);

        Coordinate yDifferenceCoordsRobot(0, robot_longitude);
        Coordinate yDifferenceCoordsGoal(0, goal_longitude);
        // distance_between_points();
        double final_goal_x = distance_between_points(xDifferenceCoordsRobot, xDifferenceCoordsGoal);
        double final_goal_y = distance_between_points(yDifferenceCoordsRobot, yDifferenceCoordsGoal);
        
        // double final_goal_x = GOAL_POINTS.front().latitude;
        // double final_goal_y = GOAL_POINTS.front().longitude;
        
        // TODO: Fix this so that it has the x and y in the map frame for the goal (within the map preferably)
        geometry_msgs::TransformStamped robot_transform = (*tfBuffer).lookupTransform("map", "chassis", ros::Time(0));

        double robot_x = robot_transform.transform.translation.x;
        double robot_y = robot_transform.transform.translation.y;

        double map_relative_goal_x = final_goal_x - robot_x;
        double map_relative_goal_y = final_goal_y - robot_y;

        struct Corner {
            
            Corner(double xIn, double yIn) : x(xIn), y(yIn) {}
            double x;
            double y;

        };

        // Occupancy grid corners
        Corner bottomLeft = Corner(mapInfo.origin_x, mapInfo.origin_y);
        Corner topLeft = Corner(mapInfo.origin_x, mapInfo.origin_y + (mapInfo.height * mapInfo.resolution));
        Corner bottomRight = Corner(mapInfo.origin_x + (mapInfo.width * mapInfo.resolution), mapInfo.origin_y);
        Corner topRight = Corner(bottomRight.x, topLeft.y);
        double max_x = bottomRight.x;
        double min_x = bottomLeft.x;
        double max_y = topLeft.y;
        double min_y = bottomLeft.y;

        // Goal position
        double goal_x = map_relative_goal_x;
        double goal_y = map_relative_goal_y;

        double return_x = goal_x; 
        double return_y = goal_y;
        
        // Make sure x is in occupancy grid
        if (goal_x >= max_x) {

            return_x = max_x - 0.25; // Make sure we aren't exactly on the edge

        } else if (goal_x <= min_x) {

            return_x = min_x + 0.25;

        } else {

            return_x = goal_x;

        }

        // Make sure y is in occupancy grid
        if (goal_y >= max_y) {

            return_y = max_y - 0.25;

        } else if (goal_y <= min_y) {

            return_y = min_y + 0.25;

        } else {

            return_y = goal_y;

        }

        returnString = std::to_string(return_x) + "|" + std::to_string(return_y);

        ROS_INFO("Service called!");
        ROS_INFO("Goal Latitude: %f", goal_latitude);
        ROS_INFO("Goal Longitude: %f", goal_longitude);
        ROS_INFO("Robot Latitude: %f", robot_latitude);
        ROS_INFO("Robot Longitude: %f", robot_longitude);
        ROS_INFO("Final Goal X: %f", final_goal_x);
        ROS_INFO("Final Goal Y: %f", final_goal_y);
        ROS_INFO("Robot Map X: %f", robot_x);
        ROS_INFO("Robot Map Y: %f", robot_y);
        ROS_INFO("Map Frame Raw Goal X: %f", map_relative_goal_x);
        ROS_INFO("Map Frame Raw Goal Y: %f", map_relative_goal_y);
        ROS_INFO("Inside Occupancy Grid Goal X: %f", return_x);
        ROS_INFO("Inside Occupancy Grid Goal Y: %f", return_y);


        res.success = true;
        res.message = returnString;
        return true;

    }


    // REQUIRES: takes in front of GOAL_POINTS
    // MODIFIES: queue containing the coordinates
    // EFFECTS: 
    bool location_is_close(Coordinate &goal_coords) {

        Coordinate currentLocation(gpsMsg.latitude, gpsMsg.longitude);
        // double dist = distance_between_points(goal_coords, currentLocation);
        double dist = distance_between_points(currentLocation, goal_coords);


        
        // ROS_INFO("\n");
        // ROS_INFO("Current Latitude %f", currentLocation.latitude);
        // ROS_INFO("Current Longitude %f", currentLocation.longitude);
        // ROS_INFO("Goal Latitude %f", goal_coords.latitude);
        // ROS_INFO("Goal Longitude %f", goal_coords.longitude);
        // ROS_INFO("Calculated Distance %f", dist);
        // ROS_INFO("\n");
        
        

        if (dist < 2.5) { 
        // distance between goal_coords and gps_coords is less than 1 m
            return true;
        }
        return false;
    }

private:
    // Subscriber
    ros::Subscriber gps_sub;
    ros::Subscriber cartographer_sub;
    ros::Subscriber map_sub;

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        // might need to fix this later
        // ROS_INFO("GPS Callback Function Called");
        gpsMsg = *msg;
        // ROS_INFO("GPS: %f, %f", msg->latitude, msg->longitude);
        return;
    }

    // void cartographerCallback(const tf2_msgs::TFMessage::ConstPtr &msg)
    // {        
    //     // update current location of robot in global frame (x,y in meters)
    //     rob_x = msg->transforms[0].transform.translation.x;
    //     rob_y = msg->transforms[0].transform.translation.y;

    // }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {

        float resolution = msg->info.resolution;
        uint32_t width = msg->info.width;
        uint32_t height = msg->info.height;;
        float origin_x = msg->info.origin.position.x;
        float origin_y = msg->info.origin.position.y;

        mapInfo.update(resolution, width, height, origin_x, origin_y);

    }


    //Expecting longitude latitude, needs to be latitude longitude.
    double distance_between_points(Coordinate &current, Coordinate &target)
    {
        // haversine formula
        double R = 6371e3; // meters
       // ROS_INFO("R Value %f", R);
        double phi1 = current.latitude * M_PI / 180.0; // φ, λ in radians
       // ROS_INFO("PI Value %f", M_PI);
        double phi2 = target.latitude * M_PI / 180.0;
       // ROS_INFO("phi1 %f", phi1);
       // ROS_INFO("phi2 %f", phi2);
        double delta_phi = (target.latitude - current.latitude) * M_PI / 180.0;
        double delta_lambda = (target.longitude - current.longitude) * M_PI / 180.0;
       // ROS_INFO("delta_phi %f", delta_phi);
       // ROS_INFO("delta_lambda %f", delta_lambda);

        double a = sin(delta_phi / 2) * sin(delta_phi / 2) +
                cos(phi1) * cos(phi2) *
                sin(delta_lambda / 2) * sin(delta_lambda / 2);

        double c = 2 * atan2(sqrt(a), sqrt(1 - a));

        double d = R * c; // in meters


        return d;
    }
    
};


int main(int argc, char **argv)
{
    // ORDER:
    // 1. GET GPS DATA from /gps/fix 
    // 2. REFERENCE POINT FROM /tf
    // 3. CALCULATE GOAL_POINTS IN TERMS OF GLOBAL FRAME
    // 4. ACTIVATE ROS SERVICE

    // Initializing the node
    ros::init(argc, argv, "gps_listener");

    // Initializing the node for the GPS
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    GPSdata gps_node(nh, tfBuffer);
    
    // read in givtxt
    gps_node.read_goal_coords();

    gps_node.gps_transform();



    ros::ServiceServer service = nh.advertiseService("goal_coords", &GPSdata::service_callback, &gps_node);




    ros::Rate rate(10);
    while (ros::ok())
    {
        if (gps_node.location_is_close(gps_node.GOAL_GPS.front()) && !gps_node.GOAL_GPS.empty())
        {
            gps_node.GOAL_POINTS.pop_front();
            gps_node.GOAL_GPS.pop_front();
            ROS_INFO("MOVED TO NEXT GOAL!!!!! %i", gps_node.indexOfCurrentGoal);
            gps_node.indexOfCurrentGoal++;
        }
        // GPS Transform part (Subscribing)
        //double gpsLong = gps_node.gpsMsg.longitude;
        //double gpsLat = gps_node.gpsMsg.latitude;

        // gps_transform(gps_node);
        //ROS_INFO("Current Latitude: %f", gpsLat);
        //ROS_INFO("Current Longitude: %f", gpsLong);

        ros::spinOnce();
        rate.sleep();
    }
}
