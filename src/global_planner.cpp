#include <pluginlib/class_list_macros.h>
#include "../include/global_planner.h"
#include "../include/rpastar.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;


bool checkPlan(const std::vector<geometry_msgs::PoseStamped>& plan, costmap_2d::Costmap2D* costmap);


//Default Constructor
namespace global_planner {

GlobalPlanner::GlobalPlanner() : costmap_ros_(NULL), initialized_(false){}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), initialized_(false) {
    initialize(name, costmap_ros);
}


void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        // ros::NodeHandle private_nh("~/" + name);
        // private_nh.param("step_size", step_size_, costmap_->getResolution());
        // private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
        world_model_ = new base_local_planner::CostmapModel(*costmap_); 

        initialized_ = true;

    }
    else
        ROS_WARN("This planner has already been initialized... doing nothing");
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
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
    int pos_x = (int)((start.pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int pos_y = (int)((start.pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());

    int goal_x = (int)((goal.pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int goal_y = (int)((goal.pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
    // for (int i = 0; i < map.info.height; i++)
    // {
    //   for (int j = 0; j < map.info.width; j++)
    //   {
    //     std::cout << map.data[map.info.width*i + j] << "  ";
    //   }
    //   std::cout << std::endl;
    // }
    std::cout << pos_x << " " << pos_y << std::endl;
    std::cout << goal_x << " " << goal_y << std::endl;
    
    
    if(!checkPlan(old_plan, costmap_)) {
        std::cout << "Running A*..." << std::endl << std::endl;
        std::pair<int,int> first(pos_y, pos_x);
        std::pair<int,int> last(goal_y, goal_x);
        // rpastar runner = rpastar::rpastar(start, end, &map);
        rpastar runner(first, last, costmap_);
        runner.search();
        if (runner.goal_found())
        {
            std::vector<std::pair<int,int>> path = runner.backtracker();
            std::cout << "Path found!" << std::endl;
            generate_path(costmap_,path,plan);
            old_plan = plan;
        }
    }
    else {
        std::cout << "Path still clear\n";
        plan = old_plan;
        
    }
    std::cout << "done with make plan\n";

    return true;
}

void GlobalPlanner::generate_path(const costmap_2d::Costmap2D* map, std::vector<std::pair<int,int>> &path, std::vector<geometry_msgs::PoseStamped>& plan)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.w = 0.5;
    pose.pose.orientation.x = 0.5;
    pose.pose.orientation.y = 0.5;
    pose.pose.orientation.z = 0.5;

    for (int i = 0; i < path.size(); i++)
    {
        float global_x = (path[i].first*map->getResolution()) + map->getOriginX();
        float global_y = (path[i].second*map->getResolution()) + map->getOriginY();
        pose.pose.position.x = global_y;
        pose.pose.position.y = global_x;
        pose.header.frame_id = "map";
        plan.push_back(pose);
    }

    return;
}

};


// Returns true if path is still valid, so we don't need to rerun a*; returns false if path is
// blocked or empty, need to run a*
bool checkPlan(const std::vector<geometry_msgs::PoseStamped>& plan, costmap_2d::Costmap2D* costmap ) {
    
    std::cout << "checking path....\n";
     if (plan.empty()) {
        std::cout << "Plan is empty\n";
        return false;
    }

    for(int i = 0 ; i < plan.size(); i++){
        geometry_msgs::PoseStamped curr_pos = plan[i];
        
        uint64_t pos_x = (uint64_t)((curr_pos.pose.position.x - costmap->getOriginX()) / costmap->getResolution());
        uint64_t pos_y = (uint64_t)((curr_pos.pose.position.y - costmap->getOriginY()) / costmap->getResolution());
        
        // getCost returns unsigned char
        int cost = costmap->getCost(pos_x, pos_y);
        std::cout << " cost : " << cost << "\n"; 
        if(cost >= LOW_THRESHOLD && cost <= UPPER_THRESHOLD){
            return false;
        }
    } 
    return true;
}     
    

    
        
    
    




