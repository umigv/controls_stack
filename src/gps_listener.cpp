#include "ros/ros.h"
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <utility>
#include <queue>
#include "geometry_msgs/Point.msg"
#include "std_msgs/UInt32MultiArray"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3.h"

using std::string;

// globals
// std_msgs::float64 origin_x, origin_y;
std_msgs::float64 rob_x, rob_y;




// srv function boolean:
bool service_callback(std_msgs/geometry_msgs::Point::Response &res) {

    //Need to figure out how to get an instance of GPSdata into this function.
    get_next_goal(&res);
    

    ROS_INFO("sending back response: ", res);
    return true;

}

void get_next_goal(std_msgs/geometry_msgs::Point::Response &res) {
                    res.x = goals.top().first;
                    res.y = goals.top().second;
                    goals.pop();
    }

class GPSdata
{
public:
    std_msgs::float64 rob_x, rob_y;
    sensor_msgs::NavSatFix gpsMsg;

    //goals.first = x coord, goals.second = y coord.
    std::queue< pair<std_msgs::float64, std_msgs::float64> > goals;

    GPSdata(ros::NodeHandle nh_)
    {
        gps_sub = nh_.subscribe("/gps/fix", 100, &GPSdata::gpsCallback, this);
        cartographer_sub = nh_.subscribe("/tf", 100, &GPSdata::cartographerCallback, this); // gives x,y coords
        // occupancy_sub = nh_.subscribe("/map", 100, &GPSdata::occupancyCallback, this); // gives origin
    }

    //returns next goal to the service and pops top of queue goals
    
    
private:
    // Subscriber
    ros::Subscriber gps_sub;
    ros::Subscriber cartographer_sub;
    ros::Subscriber occupancy_sub;


    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        // might need to fix this later
        gpsMsg = *msg;
        ROS_INFO("GPS: %f, %f", msg->latitude, msg->longitude);
        return;
    }

    void cartographerCallback(const tf2_msgs::TFMessage::ConstPtr &msg)
    {        
        // update current location of robot in global frame (x,y in meters)
        rob_x = msg->transforms[0].transform.translation.x;
        rob_y = msg->transforms[0].transform.translation.y;
    }
};

// For constant gps points
struct LongLatStorage
{
    LongLatStorage(double in_long, double in_lat)
        : longitude(in_long), latitude(in_lat) {}
    LongLatStorage()
        : longitude(0.0), latitude(0.0) {}
    double longitude;
    double latitude;
};

double get_distance_between_points(std::pair<double, double> current, std::pair<double, double> target)
{
    // Coordinates are in <lat, long> pairs
    // lat is x and long is y
    double curr_lat = current.first;
    double curr_long = current.second;
    double target_lat = target.first;
    double target_long = target.second;

    double lat_diff = target_lat - curr_lat;    // Think of it as x diff
    double long_diff = target_long - curr_long; // Think of it as y diff

    // The distance is not in meters. It is based on GPS coordinates.
    return (sqrt(pow(lat_diff, 2) + pow(long_diff, 2)));
}

// returns angle
double get_angle_between_points(std::pair<double, double> current, std::pair<double, double> target)
{
    // Coordinates are in <lat, long> pairs
    // lat is x and long is y
    double curr_lat = current.first;
    double curr_long = current.second;
    double target_lat = target.first;
    double target_long = target.second;

    double lat_diff = target_lat - curr_lat;    // Think of it as x diff
    double long_diff = target_long - curr_long; // Think of it as y diff

    double return_angle;
    double change_to_degree = (180 / 3.1415926);

    double angle_supplement = long_diff >= 0 ? 0 : 180;
    if (lat_diff == 0) {
        return long_diff > 0 ? 0 : 180;
    }
    double return_angle = atan(long_diff / lat_diff) * change_to_degree + angle_supplement;
    return return_angle < 0 ? return_angle + 360 : return_angle;
}




// struct for intaking the two coordinate pairs to publish
// self is the current coord for the robot
// goal is the next constant + defined coord set by the competition
struct coordPairs
{
   coordPairs(int selfx, int selfy, int goalx, int goaly)
        : self(selfx, selfy), goal(goalx, goaly) {}
    std::pair<int, int> self;
    std::pair<int, int> goal;
};

void gps_transform(GPSdata &gps)
{
    double updated_longtitude = 0;
    double updated_latitude = 0;
}

void read_goal_coords() {
    std::ifstream in;
    in.open("given_coords.txt");

    string line = "";

    for (int i = 0; i<5; i++) {
        getline(in, line); // throwaway away comment line
        
        in >> line;
        line.erase(line.end()-1);
        float latitude = stof(line);

        in >> line;
        float longitude = stof(line);

        std_msgs::float64 input_latitude = latitude;
        std_msgs::float64 input_longitude = longitude;

        goals.push({input_latitude, input_longitude});
        
        // get rid of remaining newline
        getline(in, line);
    }

    
}

int main(int argc, char **argv)
{

    // read in given_cords.txt
    read_goal_coords();

    
    //std::queue<uint32_t[]> goals; 

    // subscribe to /gps/fix
    

    ros::init(argc, argv, "gps_listener");
    std::vector<LongLatStorage> given_gps_point;

    // Initializing the node for the GPS
    // ros::init(argc, argv, "gps_Subscriber");
    ros::NodeHandle nh;
    GPSdata gps_node(nh);  

    // Getting the data from the GPS
    double gpsLat = 0;
    double gpsLong = 0;

     ros::ServiceServer service = 
              nh.advertiseService("goal_coords", service_callback);


    ros::Rate rate(10);
    while (ros::ok())
    {

        // GPS Transform part (Subscribing)
        gpsLong = gps_node.gpsMsg.longitude;
        gpsLat = gps_node.gpsMsg.latitude;

        // gps_transform(gps_node);
        ROS_INFO("Current Latitude: %f", gpsLat);
        ROS_INFO("Current Longitude: %f", gpsLong);


        // change this as we are storing the "custom data" that we will be making
        float msg;

        // change this as well to new custom data
        std::stringstream returned_angle;

        // CHANGE THIS TO THE NEW TYPE BEING PUBLISHED AFTER WE MAKE THE STRUCT/CLASS
        // gps_output.data = ss.str();
        // msg >> gps_output;
        string temp;
        returned_angle >> temp;


        // change this too
        ROS_INFO("Calculated Angle: %f", returned_angle);
        // ROS_INFO(temp);




        ros::spinOnce();
        rate.sleep();
    }
}
