#include <iostream>
#include <vector>
#include "rpastar.h"
#include "cmath"

static const int threshold = 0;

//default constructor
//rpastar::Node::Node() : g_score(INFINITY), h_score(0) {}

//main constructor used for creating a Node

//constructor for the start node which sets rhs to 0
rpastar::Node::Node(int row, int col, Node * parent) : parent(parent) {
    state = {row,col};
    g_score = parent->g_score + 1;
}

rpastar::Node::Node() : parent(nullptr), state({0,0}), g_score(0) {}

void rpastar::Node::set_h(std::pair<int,int> &target_state)
{
    h_score = sqrt(pow(target_state.first - state.first, 2) + pow(target_state.second - state.second, 2));
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

void rpastar::Node::set_parent(Node *parent_in)
{
    parent = parent_in;
}

std::pair<int,int> rpastar::Node::get_state()
{
    return state;
}

float rpastar::Node::get_f_score() const
{
    return g_score + h_score;
}

bool rpastar::Node::at_target(std::pair<int,int> &target_state)
{
    return (state.first == target_state.first && state.second == target_state.second);
}


rpastar::rpastar(std::pair<int,int> start_state_in, std::pair<int,int> target_state_in, costmap_2d::Costmap2D * msg) : start_state(start_state_in), target_state(target_state_in)
{
    //these are in the a* old implementation but don't think we need them as they are called in subscribe in listener.cpp
    // gpsCall();
    costMapCallback(msg); 
}

void rpastar::find_target()
{

}

bool rpastar::goal_found()
{
    return path_found;
}


void rpastar::search()
{
    std::cout << "start search\n";
    path_found = false;
    while (!U.empty())
    {
        Node current_node = U.top();
        std::cout << current_node.get_state().first << ", " << current_node.get_state().second << std::endl;
        U.pop();
        open_set.erase(current_node.get_state());
        if (current_node.at_target(target_state))
        {
            path_found = true;
            break;
        }
        int i = current_node.get_state().first;
        int j = current_node.get_state().second;
        Node *parent_ptr = &graph[i][j];
        processNode(i-1, j, parent_ptr); // north
        processNode(i, j+1, parent_ptr); // east
        processNode(i+1, j, parent_ptr); // south
        processNode(i, j-1, parent_ptr); // west
        // processNode(i-1, j+1, &current_node); // north-east
        // processNode(i+1, j+1, &current_node);// south-east
        // processNode(i-1, j-1, &current_node); // north-west
        // processNode(i+1, j+1, &current_node); // south-west
        closed_set[current_node.get_state()] = current_node.get_f_score();
    }
    std::cout << "end search\n";

}

void rpastar::processNode(int row, int col, Node *parent)
{
    if (row < 0 || row >= cost_map.size() || col < 0 || col >= cost_map[0].size())
    {
        return;
    }
    if (cost_map[row][col] > threshold)
    {
        return;
    }
    Node new_node = Node(row,col,parent);
    new_node.set_h(target_state);
    auto open_it = open_set.find(new_node.get_state());
    if(open_it != open_set.end()){
        if (open_it->second <= new_node.get_f_score())
        {
            return;
        }
    }
    auto closed_it = closed_set.find(new_node.get_state());
    if(closed_it != closed_set.end()){
        if (closed_it->second <= new_node.get_f_score())
        {
            return;
        }
    }
    U.push(new_node);
    graph[row][col] = new_node;
    open_set[new_node.get_state()] = new_node.get_f_score();
    return;
}

// called each time a new costmap is recieved...
void rpastar::costMapCallback(costmap_2d::Costmap2D* msg)
{
	// Fill out the costmap width and height from the occupancy grid info message
    int costmap_width = msg->getSizeInCellsX();
    int costmap_height = msg->getSizeInCellsY();
    
    cost_map = std::vector<std::vector<int>>(costmap_height, std::vector<int>(costmap_width, 0));
    graph = std::vector<std::vector<rpastar::Node>>(costmap_height, std::vector<rpastar::Node>(costmap_width, rpastar::Node()));

//initializing graph of nodes
    // graph.clear();
    // graph.resize(costmap_height);
    // for (int i = 0; i < graph.size(); i++)
    // {
    //     graph[i].resize(costmap_width);
    // }
    //find_target();
    //graph[costmap_height/2][costmap_width/2].set_g(0);


    // Fill out the costmap member variable using info from the occupancy grid costmap message
    for(int i = 0; i < costmap_height; i++){
        for(int j = 0; j < costmap_width; j++){
            cost_map[i][j] = msg->getCost(j,i) - '0';
            graph[i][j].set_state(i,j);
        }
    }

    // Fill out the needed "start" position member variable using info from the costmap origin message
    //start_state = {costmap_height/2,costmap_width/2};
    graph[start_state.first][start_state.second].set_g(0); 
    graph[start_state.first][start_state.second].set_h(target_state); 
    graph[start_state.first][start_state.second].set_parent(nullptr); 

    //forgets about old priority queue and pushes new start node (current position) to priority queue so that it is not empty when starting the search
    U = std::priority_queue<rpastar::Node, std::vector<rpastar::Node>, customGreater>();
    U.push(graph[start_state.first][start_state.second]);
    open_set[start_state] = U.top().get_f_score();

    
}

// void rpastar::gpsCallback(const nav_msgs::Odometry::ConstPtr& msg){
//     // Fill out the needed "target" position member variable using info from the odom message
//     target_state = {msg->pose.pose.position.x, msg->pose.pose.position.y};
// }

std::vector<std::pair<int,int>> rpastar::backtracker()
{
    Node *curr_node = &graph[target_state.first][target_state.second];
    while(curr_node != nullptr)
    {
        path.push_back(curr_node->get_state());
        curr_node = curr_node->get_parent();
    }
    return path;
}