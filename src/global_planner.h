#include <nav_core/base_global_planner.h>
#include <tf/tf.h>

#include <vector>

#include "constants.h"
// static  std::pair<int, int> goal = {2, 2};
// static  std::pair<int, int> start = {0, 0};

std::pair<int, int> goal;

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
   private:
    nav_msgs::OccupancyGrid global_map;
    // const nav_msgs::OccupancyGrid::ConstPtr & map_ptr =  & global_map;

    std::vector<std::pair<int, int>> path;

    std::vector<std::pair<int, int>> waypoints;

    std::pair<int, int> pose;

    int height;

    int width;

    // Index of current waypoint in waypoints vector
    int curr_waypoint;

   public:
    // Ctor
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        // TODO: Everything
    };
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) {
        // TODO: Everything
        return true;
    };
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan, double& cost) {
        // TODO: Everything
        return true;
    };
    ~GlobalPlanner() = default;
    GlobalPlanner(int height_in, int width_in,
                  const std::vector<std::pair<int, int>>& waypoints_in);

    //  void initialize(std::string name,
    //                  costmap_2d::Costmap2DROS *ros_costmap) override;

    //  // Gets called when a new occupancy grid is received.
    //  bool makePlan(const geometry_msgs::PoseStamped &start,
    //                const geometry_msgs::PoseStamped &goal,
    //                std::vector<geometry_msgs::PoseStamped> &plan) override;

    //  bool makePlan(const geometry_msgs::PoseStamped &start,
    //                const geometry_msgs::PoseStamped &goal,
    //                std::vector<geometry_msgs::PoseStamped> &plan,
    //                double &cost) override;

    //  ~GlobalPlanner(){};

    // Returns global_map Occupancy Grid
    nav_msgs::OccupancyGrid* getMap();

    std::vector<std::pair<int, int>> getPath();
    double cost_path();

    // Fits incoming local_map into global_map
    void updateGlobalMap(nav_msgs::OccupancyGrid local_map);

    // Returns true if the current path is still open; false if the current path
    // is blocked by an obstacle
    bool checkPath();

    // finds angle of the robots trajectory relative to the the axis the robot
    // is facing at start
    double calcYaw(std::pair<int, int> curr_pose,
                   std::pair<int, int> next_pose);
    // Calculates path if necessary

    // .at() abstracts indexing into the occupancy grid
    int8_t& at(int row, int col);

    void setGoal(std::pair<int, int> g);
    void setPose(std::pair<int, int> p);

    std::pair<int, int> getGoal();
    std::pair<int, int> getPose();

    void setPath(const std::vector<std::pair<int, int>>& path_in);

    // Returns true if pose is close enough to current waypoint
    bool reachedGoal();

    // Converts path, a vector of std::pairs, into a vector of PosedStamped
    // messages for MoveBase
    std::vector<geometry_msgs::PoseStamped> convertPath();

   private:
};  // class GlobalPlanner