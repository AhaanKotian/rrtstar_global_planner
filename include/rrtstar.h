/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <cmath>
#include <random>
#include "visualization.h"

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace rrtstar_global_planner 
{

    class Node
    {
    public:
        unsigned int idx;//index of node in 1D array map[]
        double cost;
        Node* parent = NULL;
        Node();
        Node(int idx, Node* parent, double cost);
    };

    class Rrtstar : public nav_core::BaseGlobalPlanner 
    {
    public:

        Rrtstar();
        Rrtstar(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan
                    );

    private:

        costmap_2d::Costmap2DROS* costmap_ros_;
        double step_size_, min_dist_from_robot_;
        costmap_2d::Costmap2D* costmap_;
        unsigned char* map;
        base_local_planner::WorldModel* world_model_; 
        std::default_random_engine generator;
        unsigned int map_length;
        std::set<Node*> v;
        std::set<Node*> Q_near;
        Node goal_node;
        // std::unique_ptr<ros::NodeHandle> rosNode;
        // ros::Publisher col_path;
        // ros::Publisher new_pub;
        // ros::Publisher near_pub;
        // ros::Publisher tree_pub;
        // ros::Publisher rand_pub;

        // double footprintCost(double x_i, double y_i, double theta_i);

        bool initialized_;

        void rrtstar(int start, int goal);
        Node* sample();
        Node* nearest(Node q_rand);
        Node* steer(Node* q_near, Node q_rand);
        bool collisionfree(Node q_near, Node q_new);
        void Near(Node q_new);
        double cost(Node q1, Node q2);
        double dist_betn(Node q1, Node q2);
    };

    bool operator<(const Node& n1, const Node& n2);
};
#endif