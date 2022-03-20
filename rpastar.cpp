#include <iostream>
#include <vector>
#include "rpastar.h"



//default constructor
rpastar::Node::Node() : g_score(10000), h_score(0) {}

//main constructor used for creating a Node

//constructor for the start node which sets rhs to 0
rpastar::Node::Node(std::pair<int, int> state_in, Node * parent) : state(state), parent(parent) {
    g_score = parent->g_score + 1;
}

void rpastar::Node::set_h(Node &target)
{
    h_score = sqrt(pow(target.state.first - state.first, 2) - pow(target.state.second - state.second, 2));
}

void rpastar::Node::set_g(float g_in)
{
    g_score = g_in;
}

void rpastar::Node::set_state(int row, int col)
{
    state.first = row;
    state.second = col;
}

std::pair<int,int> rpastar::Node::get_state()
{
    return state;
}

float rpastar::Node::get_f_score()
{
    return g_score + h_score;
}

bool rpastar::Node::at_target(Node &target)
{
    return (state.first == target.state.first && state.second == target.state.second);
}


rpastar::rpastar()
{
    //these are in the a* old implementation but don't think we need them as they are called in subscribe in listener.cpp
    // gpsCallback();
    // costMapCallback(); 

}

void rpastar::find_target()
{

}

void rpastar::search()
{
    while (!U.empty())
    {
        Node current_node = U.top();
        U.pop();
        if (current_node.at_target(target))
        {
            break;
        }
        int i = current_node.get_state().first;
        int j = current_node.get_state().second;
        processNode(i-1, j, &current_node); // north
        processNode(i, j+1, &current_node); // east
        processNode(i+1, j, &current_node); // south
        processNode(i, j-1, &current_node); // west
        processNode(i-1, j+1, &current_node); // north-east
        processNode(i+1, j+1, &current_node);// south-east
        processNode(i-1, j-1, &current_node); // north-west
        processNode(i+1, j+1, &current_node); // south-west
        
    }
}

void rpastar::processNode(int row, int col, Node *parent)
{
    Node node(std::make_pair(row,col),parent);
}

// called each time a new costmap is recieved...
void rpastar::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	// Fill out the costmap width and height from the occupancy grid info message
    int costmap_width = msg->info.width;
    int costmap_height = msg->info.height;
    

//initializing graph of nodes
    // graph.clear();
    // graph.resize(costmap_height);
    // for (int i = 0; i < graph.size(); i++)
    // {
    //     graph[i].resize(costmap_width);
    // }
    find_target();
    //graph[costmap_height/2][costmap_width/2].set_g(0);

    
    // Fill out the needed "start" position member variable using info from the costmap origin message
    start = Node({costmap_height/2,costmap_width/2},0);

    //forgets about old priority queue and pushes new start node (current position) to priority queue so that it is not empty when starting the search
    U = std::priority_queue<rpastar::Node, std::vector<rpastar::Node>, customGreater>();
    U.push(start);


    // Fill out the costmap member variable using info from the occupancy grid costmap message
    for(int i = 0; i < costmap_width; i++){
        for(int j = 0; j < costmap_height; j++){
            cost_map[i][j] = msg->data[i + (j * costmap_width)];
            //graph[i][j].set_state(i,j);
        }
    }

    
}

void rpastar::gpsCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // Fill out the needed "target" position member variable using info from the odom message
    target = Node({msg->pose.pose.position.x, msg->pose.pose.position.y}, INFINITY);
}

