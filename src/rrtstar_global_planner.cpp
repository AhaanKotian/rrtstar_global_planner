#include <pluginlib/class_list_macros.h>
#include "rrtstar.h"
PLUGINLIB_EXPORT_CLASS(rrtstar_global_planner::Rrtstar, nav_core::BaseGlobalPlanner)
namespace rrtstar_global_planner
{
    
    Node::Node() : idx(0), parent(NULL), cost(-1)
    {
    }

    Node::Node(int idx, Node* parent, double cost) : idx(idx), parent(parent), cost(cost)
    {
    }
    
    Rrtstar::Rrtstar() : costmap_ros_(NULL), initialized_(false)
    {

    }

    Rrtstar::Rrtstar(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), initialized_(false)
    {
        initialize(name, costmap_ros);
    }

    void Rrtstar::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);

            initialized_ = true;
        }
        else
        ROS_WARN("This planner has already been initialized... doing nothing");
    }


    bool Rrtstar::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        
        if(!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

        // if (!ros::isInitialized())
        // {
        //     int argc = 0;
        //     char **argv = NULL;

        //     ros::init(argc, argv, "rrtstar",
        //         ros::init_options::NoSigintHandler);
        // }
        // // Create ROS node
        // rosNode.reset(new ros::NodeHandle( "rrtstar" ));
        // col_path = rosNode->advertise<nav_msgs::Path>("rrtstar_collision_checker",1);
        // new_pub = rosNode->advertise<geometry_msgs::PoseStamped>("q_new",1);
        // near_pub = rosNode->advertise<geometry_msgs::PoseStamped>("q_near",1);
        // tree_pub = rosNode->advertise<visualization_msgs::Marker>("tree",1);
        // rand_pub = rosNode->advertise<geometry_msgs::PoseStamped>("q_rand",1);

        plan.clear();
        costmap_ = costmap_ros_->getCostmap();
        map = costmap_->getCharMap();
        unsigned int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
        map_length = nx*ny;
        
        double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
        
        unsigned int mx,my;

        if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
        { 
            double start_wx = start.pose.position.x;
            double start_wy = start.pose.position.y;
            unsigned int start_idx_oned;
            if(costmap_->worldToMap(start_wx,start_wy,mx,my))
                start_idx_oned = costmap_->getIndex(mx,my);
            
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        double goal_wx = goal.pose.position.x;
        double goal_wy = goal.pose.position.y;
        unsigned int goal_idx_oned;
        if(costmap_->worldToMap(goal_wx,goal_wy,mx,my))
            goal_idx_oned = costmap_->getIndex(mx,my);

        double start_wx = start.pose.position.x;
        double start_wy = start.pose.position.y;
        unsigned int start_idx_oned;
        if(costmap_->worldToMap(start_wx,start_wy,mx,my))
            start_idx_oned = costmap_->getIndex(mx,my);
        
        // ROS_INFO_STREAM(" source (in shortest path) = "<<start_idx_oned);
        // ROS_INFO_STREAM(" destination (in shortest path) = "<<goal_idx_oned);

        std::vector<Node> path;
         
        int destination = goal_idx_oned;
        goal_node.idx = destination;
        int source = start_idx_oned;

        rrtstar(source,destination);
        goal_node.parent =nearest(goal_node);
        Node n;
        n = goal_node;
        //ROS_INFO_STREAM(" --------------------------n.parent = "<<n.parent);
        while(n.parent != NULL)
        {
            path.push_back(n);
            ROS_INFO_STREAM("n.idx = "<<n.idx<<" & source = "<<source<<" & destination = "<<destination);
            n = *(n.parent);     
        }
        path.push_back(n);
        reverse(path.begin(),path.end());

        // ROS_INFO_STREAM("Pre-visualization");
        // //visualize tree
        // visualization_msgs::Marker tree;
        // rrtstar_global_planner::init_line(&tree);
        // Node* q1;
        // // Publish all edges
        // for (auto q : v)
        // {
        //     ROS_INFO_STREAM("q.idx = "<<q->idx<<" & source =  "<<source<<" & q.parent = "<<q->parent<< " & NULL = "<<NULL);
        //     if(q->parent != NULL)
        //     {
        //         double wx1,wy1,wx2,wy2;
        //         costmap_->indexToCells(q->idx,mx,my);
        //         costmap_->mapToWorld(mx,my,wx1,wy1);
        //         ROS_INFO_STREAM("q.parent = "<<(q->parent->idx));
        //         q1 = q->parent;
        //         ROS_INFO_STREAM("q1.idx = "<<(q1->idx));
        //         costmap_->indexToCells(q1->idx,mx,my);
        //         costmap_->mapToWorld(mx,my,wx2,wy2);
        //         ROS_INFO_STREAM("Edge : ( "<<q->idx<<" , "<<q1->idx<<" )");
        //         rrtstar_global_planner::pub_line(&tree, &tree_pub, wx1, wy1, wx2, wy2);
        //     }
        // }
        // ROS_INFO_STREAM("Post-visualization");
        ROS_INFO_STREAM(" PATH.SIZE() = "<<path.size());
        ros::Time plan_time = ros::Time::now();
        for(int i=0;i<path.size();i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = start.header.frame_id;
            unsigned int mx1,my1;
            costmap_->indexToCells(path[i].idx,mx1,my1);
            double wx,wy;
            costmap_->mapToWorld(mx1,my1,wx,wy);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
            //ROS_INFO_STREAM(" x = "<<plan[i].pose.position.x<<" AND y = "<<plan[i].pose.position.y);

            if(i!=0)
            {
                double wx1 = plan[i-1].pose.position.x , wy1 = plan[i-1].pose.position.y , wx2 = plan[i].pose.position.x , wy2 = plan[i].pose.position.y;
                plan.pop_back();
                //interpolate()
                double theta = atan((wy2 - wy1) / (wx2- wx1));
                double dist = sqrt(pow((wx1-wx2),2) + pow((wy1-wy2),2));
                double wx_new,wy_new;
                unsigned int k=1, new_idx;
                double step_size = M_SQRT2 * costmap_->getResolution();
                while(new_idx != path[i].idx)
                { 
                    if((wx2 > wx1 && wy2 > wy1) || (wx2 > wx1 && wy2 < wy1))
                    {
                        //ROS_INFO_STREAM("1ST OR 4TH QUADRANT");
                        wx_new = wx1 + k*step_size*cos(theta);
                        wy_new = wy1 + k*step_size*sin(theta);
                    }
                    else if((wx2 < wx1 && wy2 > wy1) || (wx2 < wx1 && wy2 < wy1))
                    {
                        //ROS_INFO_STREAM("2ND OR 3RD QUADRANT");
                        wx_new = wx1 - k*step_size*cos(theta);
                        wy_new = wy1 - k*step_size*sin(theta);
                    }
                    else //waste else condition because if not written it was giving transform error
                    {
                        //ROS_INFO_STREAM("ELSE CONDITION");
                        wx_new = wx1 + k*step_size*cos(theta);
                        wy_new = wy1 + k*step_size*sin(theta);

                    }
                    ROS_INFO_STREAM("wx_new = "<<wx_new<<"wy_new = "<<wy_new);
                    costmap_->worldToMap(wx_new, wy_new, mx1, my1);      
                    new_idx = costmap_->getIndex(mx1, my1);
                    if(new_idx == path[i].idx)
                        break;

                    geometry_msgs::PoseStamped ipose;
                    ipose.header.stamp = plan_time;
                    ipose.header.frame_id = start.header.frame_id;
                    ipose.pose.position.x = wx_new;
                    ipose.pose.position.y = wy_new;
                    ipose.pose.position.z = 0.0;
                    ipose.pose.orientation.x = 0.0;
                    ipose.pose.orientation.y = 0.0;
                    ipose.pose.orientation.z = 0.0;
                    ipose.pose.orientation.w = 1.0;
                    plan.push_back(ipose);
                    k++;

                }
            }
            plan.push_back(pose);
        }
        ROS_INFO_STREAM("plan.size() = "<<plan.size());


        return true;
    }

    void Rrtstar::rrtstar(int start, int goal)
    {
        ROS_INFO_STREAM("rrtstar() start");
        int max_iterations = 3000;
        int step_size;
        Node* n = new Node(start, NULL, 0);
        v.clear();
        v.insert(n);
        double c_min;//min cost of path
        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < max_iterations; i++)
        {
            //ROS_INFO_STREAM("I = "<<i);
            Node* q_rand = sample();
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            unsigned int mx1,my1;
            costmap_->indexToCells(q_rand->idx,mx1,my1);
            double wx,wy;
            costmap_->mapToWorld(mx1,my1,wx,wy);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            //rand_pub.publish(pose);

            Node* q_near = nearest(*q_rand);
            costmap_->indexToCells(q_near->idx,mx1,my1);
            costmap_->mapToWorld(mx1,my1,wx,wy);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            //near_pub.publish(pose);
            

            Node* q_new = steer(q_near, *q_rand);
            costmap_->indexToCells(q_new->idx,mx1,my1);
            costmap_->mapToWorld(mx1,my1,wx,wy);     
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            //new_pub.publish(pose);


            //ROS_INFO_STREAM("q_rand = "<<q_rand.idx<<" & q_near = "<<q_near.idx<<" & q_new = "<<q_new.idx);
            // costmap_->indexToCells(q_new.idx,mx1,my1);
            // double wx1,wy1;
            // costmap_->mapToWorld(mx1,my1,wx1,wy1);
            // ROS_INFO_STREAM("q_new(x,y)= ("<<wx1<<","<<wy1<<") & q_near(x,y) = ("<<wx<<","<<wy<<")");
            //ROS_INFO_STREAM("v.length = "<<v.size());
            if(collisionfree(*q_near, *q_new))
            {
                Near(*q_new);
                //ROS_INFO_STREAM("Q_near.size() = "<<Q_near.size()<<" & v.size() = "<<v.size());
                v.insert(q_new);
                q_new->parent = q_near; 
                c_min = q_near->cost + cost(*q_near, *q_new);
                //ROS_INFO_STREAM("--------------------------------------------");
                //ROS_INFO_STREAM("Edge : ( "<<q_near->idx<<" , "<<q_new.idx<<" ) & COST(q_new) = "<<c_min);
                //connect q_new to minimum cost path
                for(auto qnear : Q_near) //for(auto qnear : Q_near)
                {
                    //ROS_INFO_STREAM("Edge : ( "<<qnear->idx<<" , "<<q_new.idx<<" ) & COST(qnear) = "<< (qnear->cost + cost(*qnear, q_new))<<" & collisionfree() = "<<collisionfree(*qnear, q_new));
                    if(collisionfree(*qnear, *q_new) && qnear->cost + cost(*qnear, *q_new) < c_min)
                    {
                        q_new->parent = qnear; //ROS_INFO_STREAM("Edge : ( "<<q_new.parent->idx<<" , "<<q_new.idx<<" ) MIN COST WALA");
                        c_min = qnear->cost + cost(*qnear, *q_new);
                        q_new->cost = c_min;
                    }
                }

                //Rewire
                for(auto qnear : Q_near)//for(auto qnear : Q_near)
                {
                    if(collisionfree(*q_new, *qnear) && q_new->cost + cost(*q_new, *qnear) < qnear->cost)
                    {
                        qnear->parent = q_new; //ROS_INFO_STREAM("Edge : ( "<<qnear->parent->idx<<" , "<<qnear->idx<<" ) REWIRE WALA");
                        qnear->cost = q_new->cost + cost(*q_new, *qnear);
                    }
                }
                //ROS_INFO_STREAM("--------------------------------------------");
            }
            // ROS_INFO_STREAM("--------------------------------------------");
            // int k=0;
            // ROS_INFO_STREAM("v.size() = "<<v.size());
            // for(auto q : v)
            // {
            //     if(q->parent!=NULL)
            //         ROS_INFO_STREAM((k++)<<". q.idx = "<<q->idx<<" & q.parent.idx = "<<q->parent->idx<<" & q.cost = "<<q->cost);
            // }
            // ROS_INFO_STREAM("--------------------------------------------");
            
        }
        ROS_INFO_STREAM("rrtstar() end");
    }

    Node* Rrtstar::sample()
    {
        std::uniform_int_distribution<int> distribution(0 , map_length);
        while(true)
        {
            unsigned int idx = distribution(generator);
            if(map[idx] < 128)//50 is some arbitrary value chosen as threshold to mark if this cell is free cell
            {
                Node* sampled = new Node(idx, NULL, -1);
                return sampled;
            }
        }
    }

    Node* Rrtstar::nearest(Node q_rand)
    {
        //ROS_INFO_STREAM("q_rand.idx = "<<q_rand.idx);

        double dist, min_dist = INT16_MAX;
        Node* min_node;
        for(auto q : v)
        {       
            dist = dist_betn(*q,q_rand);
            //ROS_INFO_STREAM("q.idx = "<<q.idx<<" & dist_betn = "<<dist);
            if(dist < min_dist)
            {
                min_dist = dist;
                min_node = q;
            }
        }
        //ROS_INFO_STREAM("min_node.idx = "<<min_node.idx);

        return min_node;
    }

    Node* Rrtstar::steer(Node* q_near, Node q_rand)
    {
        double step_size = M_SQRT2 * costmap_->getResolution() * 20;
        if(dist_betn(*q_near,q_rand) <= step_size)
        {
            //ROS_INFO_STREAM("q_rand.idx = "<<q_rand.idx);
            Node* n = new Node(q_rand.idx, NULL, -1);
            if(collisionfree(*q_near, *n))
                n->parent = q_near;
            n->cost = q_near->cost + cost(*q_near, *n); 
            return n;
        }
        else
        {
            unsigned int mx,my;
            double wx_near, wy_near, wx_rand, wy_rand;

            unsigned int idx = q_near->idx;
            costmap_->indexToCells(idx,mx,my);            
            costmap_->mapToWorld(mx,my,wx_near,wy_near);

            idx = q_rand.idx;
            costmap_->indexToCells(idx,mx,my);            
            costmap_->mapToWorld(mx,my,wx_rand,wy_rand);

            double theta = atan((wy_near - wy_rand) / (wx_near - wx_rand));

            double wx_new, wy_new;
            if((wx_rand > wx_near && wy_rand > wy_near) || (wx_rand > wx_near && wy_rand < wy_near))
            {
                //ROS_INFO_STREAM("1ST OR 4TH QUADRANT");
                wx_new = wx_near + step_size*cos(theta);
                wy_new = wy_near + step_size*sin(theta);
            }
            else if((wx_rand < wx_near && wy_rand > wy_near) || (wx_rand < wx_near && wy_rand < wy_near))
            {
                //ROS_INFO_STREAM("2ND OR 3RD QUADRANT");
                wx_new = wx_near - step_size*cos(theta);
                wy_new = wy_near - step_size*sin(theta);
            }
            else //waste else condition because if not written it was giving transform error
            {
                //ROS_INFO_STREAM("ELSE CONDITION");
                wx_new = wx_near + step_size*cos(theta);
                wy_new = wy_near + step_size*sin(theta);

            }
            
            // ROS_INFO_STREAM("wx_near = "<<wx_near<< " & wy_near = s"<<wy_near);
            // ROS_INFO_STREAM("wx_new = "<<wx_new<< " & wy_new = s"<<wy_new);
            costmap_->worldToMap(wx_new, wy_new, mx, my);      
            unsigned int idx_new = costmap_->getIndex(mx, my);
            Node* n = new Node(idx_new, NULL, -1);
            if(collisionfree(*q_near, *n))
                n->parent = q_near;    
            n->cost = q_near->cost + cost(*q_near, *n);  
            //ROS_INFO_STREAM("&n = "<<n);
            return n;
        }

    }

    bool Rrtstar::collisionfree(Node q_near, Node q_new)
    {
        //ROS_INFO_STREAM("-------------------------------------------");
        if(q_near.idx == q_new.idx)
            return true;
        nav_msgs::Path line;
        line.header.frame_id = costmap_ros_->getGlobalFrameID();

        unsigned int mx,my;
        double wx_near, wy_near, wx_new, wy_new;

        unsigned int idx = q_near.idx;
        costmap_->indexToCells(idx,mx,my);            
        costmap_->mapToWorld(mx,my,wx_near,wy_near);
        //ROS_INFO_STREAM("q_near X = "<<wx_near<<"  &  q_near Y = "<<wy_near);
        idx = q_new.idx;
        costmap_->indexToCells(idx,mx,my);            
        costmap_->mapToWorld(mx,my,wx_new,wy_new);
        //ROS_INFO_STREAM("q_new X = "<<wx_new<<"  &  q_new Y = "<<wy_new);
        double theta = atan((wy_new - wy_near) / (wx_new - wx_near));
        double dist = sqrt(pow((wx_near-wx_new),2) + pow((wy_near-wy_new),2));

        double wx_col,wy_col;
        unsigned int idx_col, k=1;
        double step_size = M_SQRT2 * costmap_->getResolution();

        ros::Time plan_time = ros::Time::now();
        while(idx_col != q_new.idx)
        { 
            if((wx_new > wx_near && wy_new > wy_near) || (wx_new > wx_near && wy_new < wy_near))
            {
                //ROS_INFO_STREAM("1ST OR 4TH QUADRANT");
                wx_col = wx_near + k*step_size*cos(theta);
                wy_col = wy_near + k*step_size*sin(theta);
            }
            else if((wx_new < wx_near && wy_new > wy_near) || (wx_new < wx_near && wy_new < wy_near))
            {
                //ROS_INFO_STREAM("2ND OR 3RD QUADRANT");
                wx_col = wx_near - k*step_size*cos(theta);
                wy_col = wy_near - k*step_size*sin(theta);
            }
            else //waste else condition because if not written it was giving transform error
            {
                //ROS_INFO_STREAM("ELSE CONDITION");
                wx_col = wx_near + k*step_size*cos(theta);
                wy_col = wy_near + k*step_size*sin(theta);

            }
            
            costmap_->worldToMap(wx_col, wy_col, mx, my);      
            idx_col = costmap_->getIndex(mx, my);
            //ROS_INFO_STREAM("X = "<<wx_col<<"  &  Y = "<<wy_col<<"  &  idx_col = "<<idx_col<<"  & map[idx_col] = "<<int(map[idx_col]));

            // geometry_msgs::PoseStamped pose;
            // pose.header.stamp = plan_time;
            // pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            // pose.pose.position.x = wx_col;
            // pose.pose.position.y = wy_col;
            // pose.pose.position.z = 0.0;
            // pose.pose.orientation.x = 0.0;
            // pose.pose.orientation.y = 0.0;
            // pose.pose.orientation.z = 0.0;
            // pose.pose.orientation.w = 1.0;

            // line.poses.push_back(pose);

            if(map[idx_col] > 128)
            {
                //ROS_INFO_STREAM("-------------------------------------------");
                //col_path.publish(line);
                //line.poses.clear();
                return false;
            }
            k++;
        }
        //ROS_INFO_STREAM("-------------------------------------------");
        //col_path.publish(line);
        //line.poses.clear();

        return true;
        
    }

    void Rrtstar::Near(Node q_new)
    {
        Q_near.clear();
        double r = 100;
        for(auto q : v)
        {       
            if(dist_betn(*q,q_new) <= r)
                Q_near.insert(q);
        }
    }

    double Rrtstar::cost(Node q1, Node q2)
    {
        return costmap_->cellDistance(dist_betn(q1,q2)) + costmap_->cellDistance(dist_betn(q2,goal_node));    
    }

    double Rrtstar::dist_betn(Node q1, Node q2)
    {
        unsigned int mx,my;
        double wx1,wy1,wx2,wy2;

        unsigned int idx = q1.idx;
        costmap_->indexToCells(idx,mx,my);            
        costmap_->mapToWorld(mx,my,wx1,wy1);

        idx = q2.idx;
        costmap_->indexToCells(idx,mx,my);
        costmap_->mapToWorld(mx,my,wx2,wy2);

        return sqrt(pow((wx1-wx2),2) + pow((wy1-wy2),2));
    }

    bool operator<(const Node& n1, const Node& n2)
    {
        return n1.cost < n2.cost;
    }

};