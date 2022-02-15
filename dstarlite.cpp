#include <iostream>
#include "dstarlite.h"





dstarlite::Node::Node()
{

}

dstarlite::Node::Node(std::pair<int, int> state, int id_in, float g_in, float rhs_in) : state(state), id(id_in), g(g_in), rhs(rhs_in)
{
    calculate_key();
}

float dstarlite::Node::calculateDistance(Node other)
{
    if (isObs || other.isObs)
    {
        return INFINITY;
    }
    return sqrt(pow(state.first - other.state.first, 2) +
                pow(state.second - other.state.second, 2));
}

std::pair<int,int> dstarlite::Node::calculate_key()
{
    k.first = std::min(g, rhs) + calculateDistance(dstarlite::start);
    k.second = std::min(g, rhs);
    return k;
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

bool dstarlite::Node::equal_to(Node &other)
{
    return id == other.id;
}

dstarlite::dstarlite()
{
    
    
}

dstarlite::Node dstarlite::get_start()
{
    return start;
}

dstarlite::Node dstarlite::get_target()
{
    return target;
}

