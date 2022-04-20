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
10. The odometry values are pushed to the `odom` node on ros.

## Cartographer Installation

Here is how to install Google Cartographer and make sure it correctly builds with the controls_stack repository. This section is based off of [This Document](https://docs.google.com/document/d/1dLoVytrA96HlgC0e1s-W8zJuX13aX9Q5IUZVzJPhOwI/edit?usp=sharing).

1. Make sure ROS Noetic is fully installed. Use the UMARV [Installing Ros](https://docs.google.com/document/d/1YBR9MZa_gXv4rLe0ycrgLoiFHV2Y7EJSUOpVpPO3f_U/edit) document to do this.
2. Make sure you delete these folders if they already exist:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;~/catkin_ws/src/cartographer
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;~/catkin_ws/src/cartographer_ros
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;~/catkin_ws/src/ceres_solver
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;~/catkin_ws/ceres_solver
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;~/catkin_ws/abseil-cpp
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;~/catkin_ws/protobuf
3. Run the following commands:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`cd ~/catkin_ws`
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`catkin clean`

### Getting Prerequisite Dependencies and Cartographer Source

Run the following commands in terminal in order:
> sudo apt-get update
> sudo apt-get install -y python3-wstool python3-rosdep ninja-build
> sudo apt-get install -y liblua5.3-dev python3-sphinx libeigen3-dev
> sudo apt-get install -y stow
> cd ~/catkin_ws/src/
> git clone https://github.com/cartographer-project/cartographer_ros
> git clone https://github.com/cartographer-project/cartographer

### FIX PROTOBUF INSTALLATION SCRIPT

In order to fix the problem with the protobuf installation version being incorrect, you will need to replace one of the scripts that came with cartographer. 

In the following directory:
> ~/catkin_ws/src/cartographer/scripts/

Find the script called `install_proto3.sh` and replace it with the `install_proto3_fixed.sh` script located in the `cartographer_files` folder in this repository.

If you still have catkin build errors with velodyne-gazebo-plugins, please see the section later about protobuf errors.

### Install Cartographer Depenedencies

Run the following commands in terminal in order:
> cd ~/catkin_ws
> src/cartographer/scripts/install_proto3_fixed.sh
> src/cartographer/scripts/install_debs_cmake.sh
> src/cartographer/scripts/install_abseil.sh
> src/cartographer/scripts/install_ceres.sh

These might take a while to run, but they should successfully install all the required dependencies.

Next, we want to make sure rosdep is up to date, so run the following terminal commands in order:
> sudo rosdep init

You may already have initialized rosdep, so if you get a message saying you already have files, don't worry and continue.

> rosdep update
> rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

### Build Cartographer

Time to build cartographer. If you have cloned the controls_stack repos, it should build goat_description and velodyne_simulation as well.

> cd ~/catkin_ws
> catkin build

All things should build successfully.

#### Protobuf Errors

In case there is an updated version of protobuf that is used by apt-get, there might be a problem with the installation.

First do the following steps:

> export PATH=/usr/bin:$PATH
> protoc --version

Note this version down as the apt-get version, then:

> export PATH=/usr/local/bin:$PATH
> protoc --version

Note this version down as the source version. Revert your path to apt-get:

> export PATH=/usr/bin:$PATH

If the source version of protoc is not the same as the apt-get version, we need to edit the script.

Look at the `install_proto3_fixed.sh` script in the `cartographer_files` folder. Change the VERSION variable to be the apt-get version of protoc you have installed (as found above). 

Now go back to the [FIX PROTOBUF INSTALLATION SCRIPT](#fix-protobuf-installation-script) section, moving this script into the correct location, and work from there. You may have to remove the protobuf folder from `catkin_ws` in order to re-run the script.