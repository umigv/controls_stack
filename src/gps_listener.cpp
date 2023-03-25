#include "ros/ros.h"
#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include <fstream>
#include <cmath>
#include <utility>
#include <deque>
#include "geometry_msgs/Point.h"
#include "std_msgs/UInt32MultiArray.h"
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

//Starts as longitude then latitude.
// This is for the final queue of x, y values to be sent to the global frame.
std::deque< pair<double, double> > GOAL_POINTS;

// srv function boolean:
bool service_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
// bool service_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    string returnString = std::to_string(GOAL_POINTS.front().first) + "|" + std::to_string(GOAL_POINTS.front().second);
    // res.x = GOAL_POINTS.front().first;
    // res.y = GOAL_POINTS.front().second;

    GOAL_POINTS.pop_front();
    // TODO: Change this to return the "current goal", which only starts returning the next goal when we've reached the current one
    //ROS_INFO("Index of current goal: ", res);
    res.success = true;
    res.message = returnString;
    return true;
}

class GPSdata
{
public:
    // Get from tf transform function.
    double rob_x, rob_y;
    sensor_msgs::NavSatFix gpsMsg;
    
    GPSdata(ros::NodeHandle nh_)
    {
        gps_sub = nh_.subscribe("/gps/fix", 100, &GPSdata::gpsCallback, this);
        cartographer_sub = nh_.subscribe("/tf", 100, &GPSdata::cartographerCallback, this); // gives x,y coords
        // occupancy_sub = nh_.subscribe("/map", 100, &GPSdata::occupancyCallback, this); // gives origin
    }
    
    //reads the text file of gps coords and returns correct x y coords
    std::deque< pair<double, double>> read_goal_coords() {
        std::ifstream in;
        std::deque<pair<double, double>> goals;
        
        // in.open("given_coords.txt");
        // string line = "";
        // for (int i = 0; i<5; i++) {
        //     getline(in, line);
        //     in >> line;
        //     // line.erase(line.end() - 1);
        //     std::cout << "stof test" << std::endl;
        //     std::cout << line << std::endl;
        //     double longitude = stof(line);
        //     in >> line;
        //     double latitude = stof(line);
        //     goals.push_back(std::make_pair(longitude, latitude));            
        //     // get rid of remaining newline
        //     // getline(in, line);
        //     // getline(in, line);       
        // in.close();

        goals.push_back(std::make_pair(0, 1));
        goals.push_back(std::make_pair(0, 1));
        goals.push_back(std::make_pair(1, 1));
        goals.push_back(std::make_pair(1, 0));
        goals.push_back(std::make_pair(0, 0));
        

        return goals;
    }

    void gps_transform() {
        for (size_t i = 0; i < GOAL_POINTS.size(); i++) {
            auto x_point = std::make_pair(GOAL_POINTS[i].first, gpsMsg.latitude);
            auto y_point = std::make_pair(gpsMsg.longitude, GOAL_POINTS[i].second);
            double x_dist = distance_between_points(x_point, std::make_pair(gpsMsg.longitude, gpsMsg.latitude));
            double y_dist = distance_between_points(y_point, std::make_pair(gpsMsg.longitude, gpsMsg.latitude));
            GOAL_POINTS[i].first = x_dist + rob_x;
            GOAL_POINTS[i].second = y_dist + rob_y;
        }
    }
    
private:
    // Subscriber
    ros::Subscriber gps_sub;
    ros::Subscriber cartographer_sub;

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        // might need to fix this later
        gpsMsg = *msg;
        // ROS_INFO("GPS: %f, %f", msg->latitude, msg->longitude);
        return;
    }

    void cartographerCallback(const tf2_msgs::TFMessage::ConstPtr &msg)
    {        
        // update current location of robot in global frame (x,y in meters)
        rob_x = msg->transforms[0].transform.translation.x;
        rob_y = msg->transforms[0].transform.translation.y;
    }

    double distance_between_points(std::pair<double, 
                        double> current, std::pair<double, double> target)
    {
        // haversine formula
        double R = 6371e3; // meters
        double phi1 = current.first * M_PI / 180; // φ, λ in radians
        double phi2 = target.first * M_PI / 180;
        double delta_phi = (target.first - current.first) * M_PI / 180;
        double delta_lambda = (target.second - current.second) * M_PI / 180;

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
    GPSdata gps_node(nh);
    
    // read in givtxt
    GOAL_POINTS = gps_node.read_goal_coords();

    gps_node.gps_transform();

  

    ros::ServiceServer service = nh.advertiseService("goal_coords", service_callback);


    ros::Rate rate(10);
    while (ros::ok())
    {
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
