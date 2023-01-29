#include "ros/ros.h"
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include "sensor_msgs/NavSatFix.h"
// https://prod.liveshare.vsengsaas.visualstudio.com/join?3DD46D3FFE03E218255869499D65EBD61DEB


class GPSdata {
    public:
    gps_common::GPSFix gpsMsg;

    GPSdata(ros::NodeHandle nh_) {
        // Subscribing to the topic /NavSAtFix
        gps_sub = nh_.subscribe("/gps/fix", 100, gpsCallback);
    }

    // Callback Function for the GPS
    private:

    // Subscriber
    ros::Subscriber gps_sub;

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // might need to fix this later
        // ROS_INFO("GPS: %f, %f", msg->latitude, msg->longitude);
        return;
    }
};


//For constant gps points
struct LongLatStorage{
   //LongLatStorage(double in_long, double in_lat)
   //:longitude{in_long}, latitude{in_lat}
   const double longitude;
   const double latitude;
} 

//Requires: given gps format is latitude then longitude
void set_const_cords(std::vector<LongLatStorage>& given_gps_point){
    string data;
    fstream cord_in;
    cord_in.open("given_cords.txt");
    if (!cord_in.is_open()) {
    // Possibly need to adjust
        exit(1);
    }
    int count = 0;
    LongLatStorage in_cord;
    while (cord_in >> data) {
        if (data.at(0) == '/') {
            string junk;
            getline(cord_in, junk);
        }
        else if (count == 0) {
            in_cord.latitude = stod(data);
            count++;
        }
        else if (count == 1) {
            in_cord.longtitude = stod(data);

            // adds the longtitude data to the vector
            give_gps_point.pushback();
      
            // resets
            count = 0;
        }
        else {
            exit(1);
        }
    }
}
/*
void gps_transform(GPSdata& gps){
   
   
   double updated_longtitude = 0;
   double updated_latitude = 0;
   
   GPSdata.longitude * 111 *  

   //GPSdata.latitude    
}
*/

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

    ros::Rate rate(10);
    while(ros::ok()) {

      gpsLong = gps_node->gpsMsg.longitude;
      gpsLat = gps_node->gpsMsg.latitude;


    //   gps_transform(gps_node);

      ROS_INFO("Current Latitude: %f", gpsLat);
      ROS_INFO("Current Longitude: %f", gpsLong);

        spinOnce();
        rate.sleep();
    }

}
