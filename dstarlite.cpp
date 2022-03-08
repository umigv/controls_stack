#include <iostream>
#include "dstarlite.h"



//default constructor
dstarlite::Node::Node()
{

}

//main constructor used for creating a Node

//constructor for the start node which sets rhs to 0
dstarlite::Node::Node(std::pair<int, int> state, float rhs_in) : state(state), rhs(rhs_in)
{
    g = INFINITY;
}

//calculates eulicidean distance between this node and another node. Returns infinity if either is an obstacle
float dstarlite::Node::calculateDistance(Node other)
{
    if (isObs || other.isObs)
    {
        return INFINITY;
    }
    return sqrt(pow(state.first - other.state.first, 2) +
                pow(state.second - other.state.second, 2));
}

bool dstarlite::Node::less_than(Node &other)
{
    if (k.first == other.k.first)
    {
        return k.second < other.k.second;
    }
    return k.first < other.k.first;
}

bool dstarlite::Node::greater_than(Node &other)
{
    if (k.first == other.k.first)
    {
        return k.second > other.k.second;
    }
    return k.first > other.k.first;
}

// bool dstarlite::Node::equal_to(Node &other) maybe don't need to keep track of ids
// {
//     return id == other.id;
// }

float dstarlite::Node::get_g()
{
    return g;
}
float dstarlite::Node::get_rhs()
{
    return rhs;
}

dstarlite::dstarlite() : km(0)
{
    //these are in the a* old implementation but don't think we need them as they are called in subscribe in listener.cpp
    // gpsCallback();
    // costMapCallback(); 


}

std::pair<int,int> dstarlite::calculate_key(Node &node)
{
    return {std::min(node.get_g(), node.get_rhs()) + node.calculateDistance(start) + km, std::min(node.get_g(), node.get_rhs())};
}

void dstarlite::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	// Fill out the costmap width and height from the occupancy grid info message
    int costmap_width = msg->info.width;
    int costmap_height = msg->info.height;

    // Fill out the needed "start" position member variable using info from the costmap origin message
    start = Node({msg->info.origin.orientation.x, msg->info.origin.orientation.y}, 0);

    //forgets about old priority queue and pushes new start node (current position) to priority queue so that it is not empty when starting the search
    U = std::priority_queue<dstarlite::Node, std::vector<dstarlite::Node>, customGreater>();
    U.push(target);

    // Fill out the costmap member variable using info from the occupancy grid costmap message
    for(int i = 0; i < costmap_width; i++){
        for(int j = 0; j < costmap_height; j++){
            cost_map[i][j] = msg->data[i + (j * costmap_width)];
        }
    }

    
}

void dstarlite::gpsCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // Fill out the needed "target" position member variable using info from the odom message
    target = Node({msg->pose.pose.position.x, msg->pose.pose.position.y}, INFINITY);
}

