## Synopsis

`goat_description` contains the URDF and supporting files to describe our robot, GOAT.

## Motivation

Creating a URDF allows us two benefits: first, we can use the URDF in simulation with software like Gazebo to more accurately model the behavior of our robot, and second, we can use the URDF to generate an 2D Odometry sensor stream from wheel JointStates provided by encoders.

## Installation

Clone this repository into your `catkin` workspace, then run `catkin build`. Optionally, run `catkin config --install` followed by `catkin build` to install the package to your `catkin` workspace.

## Contributors

`goat_description` is maintained by Jason Ning and Krishna Dihora.

## Teleop

Currently, teleoperation of the robot in the Gazebo sim requires the `turtlebot` package to be installed and made (remember to run `catkin build` and `source ~/catkin_ws/devel/setup.bash` before trying to launch anything!). 

1. Run `roslaunch goat_description mybot_world.launch` to start up the Gazebo sim
2. Start up the turtlebot teleoperation node. This node accepts keyboard input and publishes `geometry_msgs::Twist` messages that can be used. Do this in a new terminal window with `rosrun turtlesim turtle_teleop_key` and note that this terminal must be pulled up for the input to actually be accepted. 
3. In (yet again) a new terminal window, run `rqt_graph` and notice that there is a topic called `/cmd_vel` and a topic called `/turtle1/cmd_vel`. The `/turtle1/cmd_vel` is what the teleop node is publishing, and the `/cmd_vel` topic is what is being subcribed to that will actually move the robot in Gazebo; this gap needs to be bridged. A ROS tool exists to do this. Close the graph window and run `rosrun topic_tools relay /turtle1/cmd_vel /cmd_vel`. 

