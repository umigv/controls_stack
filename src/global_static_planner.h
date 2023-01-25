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


//subscribe to /map and /odom
class Global_Static_Planner {
    private:


    public:

};



//publish transformation
