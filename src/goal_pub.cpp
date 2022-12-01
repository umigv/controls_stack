#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include <string>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class Listener
{
public:
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose);
  geometry_msgs::PoseStamped get_goal();

private:
  geometry_msgs::PoseStamped goal_pose;
};

void Listener::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_pose_in)
{
    goal_pose = *goal_pose_in;
}

geometry_msgs::PoseStamped Listener::get_goal()
{
    return goal_pose;
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "goal_pub");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  Listener listener;
  ros::Subscriber sub = n.subscribe("move_base_simple/goal", 1000, &Listener::goalCallback, &listener);
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);


  // std::cout << path.poses[0].pose.position.x << " " << path.poses[0].pose.position.y << std::endl;
  // std::cout << path.poses[1].pose.position.x << " " << path.poses[1].pose.position.y << std::endl;

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%  
  ros::Rate r(10);
  while (ros::ok())
  {
    geometry_msgs::PoseStamped goal_pose = listener.get_goal();
    goal_pub.publish(goal_pose);
    ros::spinOnce();
    r.sleep();
  }
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
