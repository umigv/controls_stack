
#ifndef _DSTARLITE_H
#define _DSTARLITE_H

#include <iostream>
#include <queue>
#include <vector>
#include "node.h"
#include "globals.h"


class customGreater {
    public:
        bool operator() (Node &lhs, Node &rhs)
        {
            return lhs.greater_than(rhs);
        }
};


class dstarlite
{
    private:
        std::priority_queue<Node, std::vector<Node>, customGreater> queue;
        const Node start;
        const Node target;
        std::vector<Node> path;
        //std::unordered_set<Node, node_hash, Compare_cord> closed_set;
        std::vector<std::vector<int>> cost_map;

    public:
        dstarlite();
        void search();
        void backtracker(std::vector<Node>& path);
        bool validNode(const Node& node);
        bool processNode(const int x, const int y, const Node* parent);
        void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        double calculateEuclideanDistance(const Node& node1, const Node& node2);

    protected:
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
            Node(std::pair<int, int> state, int id_in, float g_in, float rhs_in);
            std::pair<int,int> calculate_key();
            bool less_than(Node &other);
            bool greater_than(Node &other);
            bool equal_to(Node &other);
            void add_to_neighbors();
        };

        
};

#endif

