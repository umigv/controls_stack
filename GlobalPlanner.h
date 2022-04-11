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

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace global_planner {

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
public:

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

    
private:

    costmap_2d::Costmap2DROS* costmap_ros_;
    double step_size_, min_dist_from_robot_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::WorldModel* world_model_;
 
    

    bool initialized_;


#endif /* globalMap_h */
