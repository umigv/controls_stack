#include "ros/ros.h"
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include "sensor_msgs/NavSatFix.h"

using std::string;

class GPSdata {
    public:
    sensor_msgs::NavSatFix gpsMsg;

    GPSdata(ros::NodeHandle nh_) {
        // Subscribing to the topic /NavSAtFix
        gps_sub = nh_.subscribe("/gps/fix", 100, &gpsCallback, this);
    }
    // Callback Function for the GPS
    private:

    // Subscriber
    ros::Subscriber gps_sub;

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // might need to fix this later
        gpsMsg = *msg;
        ROS_INFO("GPS: %f, %f", msg->latitude, msg->longitude);
        return;
    }
};


//For constant gps points
struct LongLatStorage{
    LongLatStorage(double in_long, double in_lat)
        :longitude(in_long), latitude(in_lat){}
    LongLatStorage()
        : longitude(0.0), latitude(0.0) {}
    double longitude;
    double latitude;
};


/**/
//Requires: given gps format is latitude then longitude
void set_const_cords(std::vector<LongLatStorage>& given_gps_point){
    std::string data;
    std::fstream cord_in;
    cord_in.open("given_cords.txt");
    if (!cord_in.is_open()) {
    // Possibly need to adjust
        exit(1);
    }
    int count = 0;
    LongLatStorage in_cord;
    while (cord_in >> data) {
        if (data.at(0) == '/') {
            std::string junk;
            getline(cord_in, junk);
        }
        else if (count == 0) {
            in_cord.latitude = stod(data);
            count++;
        }
        else if (count == 1) {
            in_cord.longitude = stod(data);

            // adds the longtitude data to the vector
            given_gps_point.push_back(in_cord);

            // resets
            count = 0;
        }
        else {
            exit(1);
        }
    }
}

// struct for intaking the two coordinate pairs to publish
// self is the current coord for the robot
// goal is the next constant + defined coord set by the competition
struct cordPairs{
    
    cordPairs(int selfx, int selfy, int goalx, int goaly)
    :self(selfx, selfy), goal(goalx, goaly){}
    std::pair<int, int> self;
    std::pair<int, int> goal;
};

void gps_transform(GPSdata& gps){
    double updated_longtitude = 0;
    double updated_latitude = 0;   
}


int main(int argc, char **argv) {

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
    ros::init(argc, argv, "talker");

    // publishing node (change name later if needed)
    ros::NodeHandle gps_publisher_node;

    // need publisher type changed + change the buffer size bc Idk how long we'll 
    //Need to figure out how to publish a custom message type for struct.
    // ros::Publisher gps_talker_pub = gps_publisher_node.advertise<cordPairs>("gps_talker_topic", 10000);

    ros::Rate rate(10);
    while(ros::ok()) {


        // GPS Transform part (Subscribing)
        gpsLong = gps_node.gpsMsg.longitude;
        gpsLat = gps_node.gpsMsg.latitude;
    
        // gps_transform(gps_node);
        ROS_INFO("Current Latitude: %f", gpsLat);
        ROS_INFO("Current Longitude: %f", gpsLong);



        // Publighing part AND YES I KNOW THE STYLE IS A 0/10 BUT WE WILL FIX IT EVENTUALLY^TM
        
        // change this as we are storing the "custom data" that we will be making
        string msg;
        
        // change this as well to new custom data
        std::stringstream gps_output;

        // CHANGE THIS TO THE NEW TYPE BEING PUBLISHED AFTER WE MAKE THE STRUCT/CLASS
        // gps_output.data = ss.str();
        // msg >> gps_output;
        string temp;
        gps_output >> temp;

        // change the aprameter in .publish after custom data type is made
        // gps_talker_pub.publish(msg);

        // change this too
        ROS_INFO("Published message:");
        // ROS_INFO(temp);

        ros::spinOnce();
        rate.sleep();
    }

}