#include "ros/ros.h"
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include "sensor_msgs/NavSatFix.h"

using std::string;

class GPSdata
{
public:
    sensor_msgs::NavSatFix gpsMsg;

    GPSdata(ros::NodeHandle nh_)
    {
        // Subscribing to the topic /NavSAtFix
        gps_sub = nh_.subscribe("/gps/fix", 100, &GPSdata::gpsCallback, this);
    }
    // Callback Function for the GPS
private:
    // Subscriber
    ros::Subscriber gps_sub;

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        // might need to fix this later
        gpsMsg = *msg;
        ROS_INFO("GPS: %f, %f", msg->latitude, msg->longitude);
        return;
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

float get_distance_between_points(std::pair<float, float> current, std::pair<float, float> target)
{
    // Coordinates are in <lat, long> pairs
    // lat is x and long is y
    float curr_lat = current.first;
    float curr_long = current.second;
    float target_lat = target.first;
    float target_long = target.second;

    float lat_diff = target_lat - curr_lat;    // Think of it as x diff
    float long_diff = target_long - curr_long; // Think of it as y diff

    // The distance is not in meters. It is based on GPS coordinates.
    return (sqrt(pow(lat_diff, 2) + pow(long_diff, 2)));
}

// returns angle
float get_angle_between_points(std::pair<float, float> current, std::pair<float, float> target)
{
    // Coordinates are in <lat, long> pairs
    // lat is x and long is y
    float curr_lat = current.first;
    float curr_long = current.second;
    float target_lat = target.first;
    float target_long = target.second;

    float lat_diff = target_lat - curr_lat;    // Think of it as x diff
    float long_diff = target_long - curr_long; // Think of it as y diff

    float return_angle;
    float change_to_degree = (180 / 3.1415926);

    // Quandrant 1. x_diff > 0 and y_diff > 0
    if (lat_diff > 0 && long_diff > 0)
    {
        // Final angle = calculated angle
        return_angle = atan(long_diff / lat_diff) * change_to_degree;
    }
    // Quandrant 4. x_diff > 0 and y_diff < 0
    else if (lat_diff > 0 && long_diff < 0)
    {
        // Final angle = calculated angle + 90
        return_angle = atan(long_diff / lat_diff) * change_to_degree + 90;
    }
    // Quandrant 3. x_diff < 0 and y_diff < 0
    else if (lat_diff < 0 && long_diff < 0)
    {
        // Final angle = calculated angle + 180
        return_angle = atan(long_diff / lat_diff) * change_to_degree + 180;
    }
    // Quandrant 2. x_diff < 0 and y_diff > 0
    else if (lat_diff < 0 && long_diff > 0)
    {
        // Final angle = calculated angle + 270
        return_angle = atan(long_diff / lat_diff) * change_to_degree + 270;
    }
    // Edge case 1 (0/360 degrees). x_diff == 0 and y_diff > 0
    else if ((lat_diff == 0) && (long_diff > 0))
    {
        return_angle = 0;
    }
    // Edge case 2 (90 degrees). x_diff > 0 and y_diff == 0
    else if (lat_diff > 0 && (long_diff == 0))
    {
        return_angle = 90;
    }
    // Edge case 3 (180 degrees). x_diff == 0 and y_diff < 0
    else if ((lat_diff == 0) && (long_diff < 0))
    {
        return_angle = 180;
    }
    // Edge case 4 (270 degrees). x_diff < 0 and y_diff == 0
    else if ((lat_diff < 0) && (long_diff == 0))
    {
        return_angle = 270;
    }
    else
    {
        exit(1);
    }
    return return_angle;
}

/**/
// Requires: given gps format is latitude then longitude
void set_const_cords(std::vector<LongLatStorage> &given_gps_point)
{
    std::string data;
    std::fstream cord_in;
    cord_in.open("given_cords.txt");
    if (!cord_in.is_open())
    {
        // Possibly need to adjust
        exit(1);
    }
    int count = 0;
    LongLatStorage in_cord;
    while (cord_in >> data)
    {
        if (data.at(0) == '/')
        {
            std::string junk;
            getline(cord_in, junk);
        }
        else if (count == 0)
        {
            in_cord.latitude = stod(data);
            count++;
        }
        else if (count == 1)
        {
            in_cord.longitude = stod(data);

            // adds the longtitude data to the vector
            given_gps_point.push_back(in_cord);

            // resets
            count = 0;
        }
        else
        {
            exit(1);
        }
    }
}

// struct for intaking the two coordinate pairs to publish
// self is the current coord for the robot
// goal is the next constant + defined coord set by the competition
struct cordPairs
{

    cordPairs(int selfx, int selfy, int goalx, int goaly)
        : self(selfx, selfy), goal(goalx, goaly) {}
    std::pair<int, int> self;
    std::pair<int, int> goal;
};

void gps_transform(GPSdata &gps)
{
    double updated_longtitude = 0;
    double updated_latitude = 0;
}

int main(int argc, char **argv)
{

    // subsrcibe to /gps/fix
    // publisher /garmin+gps

    ros::init(argc, argv, "gps_listener");
    std::vector<LongLatStorage> given_gps_point;

    // Initializing the node for the GPS
    // ros::init(argc, argv, "gps_Subscriber");
    ros::NodeHandle nh;
    GPSdata gps_node(nh);

    // Getting the data from the GPS
    double gpsLat = 0;
    double gpsLong = 0;

    // Publisher file (NEED LOTTA REVIEW BEFORE GOING PUBLIC)

    // fix name later
    ros::init(argc, argv, "gps_publisher_node");

    // publishing node (change name later if needed)
    ros::NodeHandle gps_publisher_node;

    // need publisher type changed + change the buffer size bc Idk how long we'll
    // Need to figure out how to publish a custom message type for struct.
    // ros::Publisher gps_talker_pub = gps_publisher_node.advertise<cordPairs>("gps_talker_topic", 10000);

    ros::Rate rate(10);
    while (ros::ok())
    {

        // GPS Transform part (Subscribing)
        gpsLong = gps_node.gpsMsg.longitude;
        gpsLat = gps_node.gpsMsg.latitude;

        // gps_transform(gps_node);
        ROS_INFO("Current Latitude: %f", gpsLat);
        ROS_INFO("Current Longitude: %f", gpsLong);

        // Publighing part AND YES I KNOW THE STYLE IS A 0/10 BUT WE WILL FIX IT EVENTUALLY^TM

        // change this as we are storing the "custom data" that we will be making
        float msg;

        // change this as well to new custom data
        std::stringstream returned_angle;

        // CHANGE THIS TO THE NEW TYPE BEING PUBLISHED AFTER WE MAKE THE STRUCT/CLASS
        // gps_output.data = ss.str();
        // msg >> gps_output;
        string temp;
        returned_angle >> temp;

        // change the aprameter in .publish after custom data type is made
        // gps_talker_pub.publish(msg);
        gps_talker_pub.publish(msg);

        // change this too
        ROS_INFO("Calculated Angle: %f", returned_angle);
        // ROS_INFO(temp);

        ros::spinOnce();
        rate.sleep();
    }
}