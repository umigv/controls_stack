
#include "rpastar.h"
#include <ros/ros.h">
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>


const std::string kGPS_topic = "/odometry/filtered";
const std::string kCostmap_topic = "move_base/local_costmap/costmap";

using namespace std;

class calculate_path {
private:
	ros::Subscriber &gps;
	ros::Subscriber &costmap;
	rpastar &listener;
	bool at_target;
public:
	vector<geometry_msgs::position> solution_path;
	calculate_path(ros::Subscriber &gps_in, ros::Subscriber &costmap_in, rpastar &rp_astar)
		: gps(gps_in), costmap(costmap_in), listener(rp_astar), at_target(false) {}
	void operator()(const ros::TimerEvent&);
	bool get_at_target() const{
		return at_target;
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "/path_planning/listener");

	ros::NodeHandle n;

	rpastar listener = rpastar{};

	ros::Subscriber gps_sub = n.subscribe(kGPS_topic, 1000, listener.gpsCallback);
	ros::Subscriber costmap_sub = n.subscribe(kCostmap_topic, 1000, listener.costMapCallback);

	// Calculated path vector can be accessed via path.solution_path
	calculate_path path(gps_sub, costmap_sub, listener);
	
	// Update coordinates once before calculating path
	ros::spinOnce();

	// Start running A* at a frequency specified by period to keep updating path
	// as the area is traversed
	const double period = 0.05;
	ros::Timer timer = n.createTimer(ros::Duration(period), path);
	timer.start();

	// Keep spinning until you reach the target
    ros::Rate rate(10); // 10 hz
	while(!path.get_at_target()) {
		ros::spinOnce();
        rate.sleep();
    }

	// Stop running timer that calculates path
	timer.stop();

	ros::Publisher path_pub = n.advertise<vector<geometry_msgs::position>("path", 1000);
	path_pub.publish(path.solution_path);
	return 0;
}

void calculate_path::operator()(const ros::TimerEvent&) {
	// Attempt to find a solution
	at_target = listener.search();
}