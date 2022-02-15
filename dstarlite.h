
#ifndef _DSTARLITE_H
#define _DSTARLITE_H

#include <iostream>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <vector>

class dstarlite
{
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
            Node();
            Node(std::pair<int, int> state, int id_in, float g_in, float rhs_in);
            std::pair<int,int> calculate_key();
            bool less_than(Node &other);
            bool greater_than(Node &other);
            bool equal_to(Node &other);
            void add_to_neighbors();
            float calculateDistance(Node other);
        };

        class customGreater {
            public:
            bool operator() (Node &lhs, Node &rhs)
            {
                return lhs.greater_than(rhs);
            }
        };

        class Node_hash{
            size_t operator()(const Node& node) const;
        };

        class Compare_coord{
            bool operator()(const Node& lhs, const Node& rhs);
        };

        const Node start;

    public:
        dstarlite();
        Node get_start();
        Node get_target();
        void search();
        void backtracker(std::vector<Node>& path);
        bool validNode(const Node& node);
        bool processNode(const int x, const int y, const Node* parent);
        // void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);
        // void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        double calculateEuclideanDistance(const Node& node1, const Node& node2);

    private:
        std::priority_queue<Node, std::vector<Node>, customGreater> queue;
        const Node target;
        std::vector<Node> path;
        std::unordered_set<Node, Node_hash, Compare_coord> closed_set;
        std::vector<std::vector<int>> cost_map;
        float km;     
};



#endif

