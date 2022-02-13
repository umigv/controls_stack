#include "node.h"
#include "globals.h"
#include <cmath>

static float calculateDistance(Node &lhs, Node &rhs)
{
    if (lhs.is_obs() || rhs.is_obs())
    {
        return INFINITY;
    }
    return sqrt(pow(lhs.get_state().first - rhs.get_state().first, 2) +
                pow(lhs.get_state().second - rhs.get_state().second, 2));
}

Node::Node() :
    state(std::pair<int,int>(0,0)), id(0), g(0), rhs(0) {}

Node::Node(std::pair<int, int> state, int id_in, float g_in, float rhs_in) : state(state), id(id_in), g(g_in), rhs(rhs_in)
{
    calculate_key();
}


std::pair<int,int> Node::calculate_key()
{
    if (globals::start_node == nullptr)
    {
        k.first = 0;
        k.second = 0;
    }
    else
    {
        k.first = std::min(g, rhs) + calculateDistance(*globals::start_node, *this) + globals::km;
        k.second = std::min(g, rhs);
    }
    return k;
}

bool Node::less_than(Node &other)
{
    if (k.first == other.k.first)
    {
        return k.second < other.k.second;
    }
    return k.first < other.k.first;
}

bool Node::greater_than(Node &other)
{
    if (k.first == other.k.first)
    {
        return k.second > other.k.second;
    }
    return k.first > other.k.first;
}

bool Node::equal_to(Node &other)
{
    return id == other.id;
}

bool Node::is_obs()
{
    return is_obs;
}

float Node::get_rhs()
{
    return rhs;
}

int Node::get_id()
{
    return id;
}

float Node::get_g()
{
    return g;
}

std::vector<Node> Node::get_neighbors()
{
    return neighbors;
}

std::pair<int,int> Node::get_state()
{
    return state;
}

std::pair<int,int> Node::get_key()
{
    return k;
}

void Node::set_rhs(float rhs_in)
{
    rhs = rhs_in;
}

void Node::set_g(float g_in)
{
    g = g_in;
}

void Node::add_to_neighbors()
{
    // python file adds neighbor id not actual node
}

