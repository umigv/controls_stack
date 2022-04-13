//
//  globalMap.h
//  UmarvGlobal
//
//  copied and pasted by Daniel Abrahm Chayes (Ben Rossano did more) on 4/5/22.
//  DAAAAAN

#ifndef globalMap_h
#define globalMap_h

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include "rpastar.h"

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace global_planner {

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
friend class rpastar
 GlobalPlanner() : costmap_ros_(NULL), initialized_(false) {}

 GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros):
    {
        initialize(name, costmap_ros);
    }

 void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        costmap_ros = costmap_ros;
        costmap_ = costmap_ros-> getCostmap();

        ros::NodeHandle private_nh("-/" + name);
        private_nh.param("step_size", step_size_, costmap_->getResolution());
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
        world_model_ = new base_local_planner::CostmapModel(*costmap_);

        initialized_ = true;
    } else {
        ROS_WARN("This planner has already been initialized... doing nothing");
    }
    
}

bool makePlan(const geometry_msgs::PoseStamped& start, 
       const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
 
     if(!initialized_){
       ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
       return false;
     }
 
     ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
 
     plan.clear();
     costmap_ = costmap_ros_->getCostmap();
 
     if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
       ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
           costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
       return false;
     }
 
     tf::Stamped<tf::Pose> goal_tf;
     tf::Stamped<tf::Pose> start_tf;

     poseStampedMsgToTF(goal,goal_tf);
     poseStampedMsgToTF(start,start_tf);
 
     double useless_pitch, useless_roll, goal_yaw, start_yaw;
     start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
     goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);
 
     //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
     double goal_x = goal.pose.position.x;
     double goal_y = goal.pose.position.y;
     double start_x = start.pose.position.x;
     double start_y = start.pose.position.y;

     rpastar pathplan({start_x, start_y}, {goal_x, goal_y});
     pathplan.search();
     pathplan.backtracker();
}

    
private:

    costmap_2d::Costmap2DROS* costmap_ros_;
    double step_size_, min_dist_from_robot_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::WorldModel* world_model_;
 
    

    bool initialized_;


#endif /* globalMap_h */
