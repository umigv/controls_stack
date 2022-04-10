# controls_stack

## Synopsis

`controls_stack` contains the simulation environment for both the robot and the lidar systems through velodyne. Please clone this repository into your catkin_ws/src folder to make sure everything runs smoothly.

## Motivation

This simulation model will help test the robot in an environment similar to the actual competition. While the simulation is not perfect, it will definitely help test the robot model as well as pathfinding algorithms and sensors.

## Contributors

`controls_stack` is maintained by Jason Ning, Krishna Dihora, Aarya Kulshrestha, Ashwin Saxena, and the rest of the Controls Subteam in ARV.

## Simulation

In order to run the simulation stack, do the following:

1. Make sure this repository is cloned into your catkin_ws/src folder.
2. Run `catkin_build`. You might have to move to the catkin_ws directory when running this command. If problems occur, `catkin clean` can be used to clear previous builds.
3. Next, run `source ~/catkin_ws/devel/setup.bash`. 
4. Use the command `roslaunch goat_description mybot_world.launch` to run the simulation model in gazebo and rviz.
5. The following section is for moving the robot around.
6. If you want to move the robot in the gazebo simulation, first setup a relay in an open terminal: `rosrun topic_tools relay /turtle1/cmd_vel /cmd_vel`.
7. Next you will need to setup the teleop controls: `rosrun turtlesim turtle_teleop_key`.
8. Make sure you are in the terminal window when using the directional keys to move the robot.
9. The IMU values are pushed to the `imu` node on ros. To see them, run `rostopic echo imu`.

