#ifndef _NODE_H
#define _NODE_H

#include <iostream>
#include <vector>
#include "dstarlite.h"

class Node
{
private:

    int id;
    float g;
    float rhs;
    bool isObs;
    bool neighbors_generated;
    std::pair<int, int> state;
    std::pair<float, float> k;
    std::vector<Node> neighbors;

public:

    Node();

    Node(std::pair<int, int> state, int id_in, float g_in, float rhs_in);

    std::pair<int,int> calculate_key();

    bool less_than(Node &other);

    bool greater_than(Node &other);

    bool equal_to(Node &other);
    
    bool is_obs();

    float get_rhs();

    int get_id();

    float get_g();

    std::vector<Node> get_neighbors();

    std::pair<int,int> get_state();

    std::pair<int,int> get_key();

    void set_rhs(float rhs_in);

    void set_g(float g_in);

    void add_to_neighbors();


    

};

extern Node start_node;

#endif