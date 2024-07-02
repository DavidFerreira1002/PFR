/*----------------------------------------------------------------------------------------------------------------------------------
# Copyright (C) 2022, Federico Rollo
#
# Institute: Leonardo Labs (Leonardo S.p.a)
#
# This file is part of FollowMe. <https://github.com/FedericoRollo/followme>
#
# FollowMe is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# FollowMe is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License. If not, see http://www.gnu.org/licenses/
---------------------------------------------------------------------------------------------------------------------------------*/
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <vector>

class ObstacleAvoidanceNode
{
private:
    ros::NodeHandle n_;
    ros::Publisher filtered_vel_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber laserscan_sub_;
    geometry_msgs::Twist filtered_vel_;

    sensor_msgs::LaserScan scan_msg_;

    std::vector<geometry_msgs::Twist> obstacle_velocities_;
    float obstacle_consideration_range_;

public:
    void cmd_vel_callback(const geometry_msgs::Twist & msg);
    void laserscan_callback(const sensor_msgs::LaserScan & msg);

    void init();
    ObstacleAvoidanceNode(ros::NodeHandle n);
};

ObstacleAvoidanceNode::ObstacleAvoidanceNode(ros::NodeHandle n): n_{n}
{}

void ObstacleAvoidanceNode::init()
{
    std::string cmd_vel_topic, filtered_vel_topic, laserscan_topic;
    n_.getParam("/followme/cmd_vel_topic_name", cmd_vel_topic);
    n_.getParam("/obstacle_avoidance/filtered_vel_topic", filtered_vel_topic);
    n_.getParam("/obstacle_avoidance/laserscan_topic", laserscan_topic);
    n_.getParam("/obstacle_avoidance/obstacle_consideration_range", obstacle_consideration_range_);

    if (cmd_vel_topic.empty() || filtered_vel_topic.empty() || laserscan_topic.empty())
    {
        ROS_ERROR("Ros parameters are not properly initialized. Check topics names.");
        exit(1);
    }
        

    cmd_vel_sub_ = n_.subscribe(cmd_vel_topic, 10, &ObstacleAvoidanceNode::cmd_vel_callback, this);
    ROS_INFO("Subscribed obstacle avoidance layer to %s topic", cmd_vel_topic.c_str());
    filtered_vel_pub_ = n_.advertise<geometry_msgs::Twist>(filtered_vel_topic, 10);
    ROS_INFO("Obstacle avoidance layer is publishing on %s topic", filtered_vel_topic.c_str());
    laserscan_sub_ = n_.subscribe(laserscan_topic, 10, &ObstacleAvoidanceNode::laserscan_callback, this);

}


// * laser scan is behind the robot and starts from -pi and goes to pi
void ObstacleAvoidanceNode::laserscan_callback(const sensor_msgs::LaserScan & msg)
{
    scan_msg_ = msg;
}


void ObstacleAvoidanceNode::cmd_vel_callback(const geometry_msgs::Twist & msg)
{   
    // initialize filtered message
    filtered_vel_ = msg;
    // ROS_INFO("input_vel =\t x: %f, y=%f, th=%f", msg.linear.x, msg.linear.y, msg.angular.z);

    // initialize angle increment
    float angle_increment = scan_msg_.angle_increment;

    int idx{0};
    float min_theta{0};
    float min_range{obstacle_consideration_range_};
    bool obstacle_found{false};
    for (float range : scan_msg_.ranges)
    {
        // check if an obstale is near the robot (inside the awarness range)
        if(range < obstacle_consideration_range_ && range < min_range)
        {
            min_theta = angle_increment * idx;
            min_range = range;
            obstacle_found = true;
        }
        idx++;
    }

    // TODO Complete speed filtering (compute a single mean vel of ranges and then remove that from robot)
    if (obstacle_found)
    {
        ROS_INFO("Obstacle found");
        filtered_vel_.linear.x -= (1/obstacle_consideration_range_ - 1/min_range) * cos(min_theta);
        filtered_vel_.linear.y -= (1/obstacle_consideration_range_ - 1/min_range) * sin(min_theta);
    }

    // Saturating velocities 
    if(filtered_vel_.linear.x > 1.0)
        filtered_vel_.linear.x = 1.0;
    else if(filtered_vel_.linear.x < -1.0)
        filtered_vel_.linear.x = -1.0;
    if(filtered_vel_.linear.y > 1.0)
        filtered_vel_.linear.x = 1.0;
    else if(filtered_vel_.linear.y < -1.0)
        filtered_vel_.linear.y = -1.0;
    ROS_INFO("filt_vel =\t x: %f, y=%f, th=%f\n", filtered_vel_.linear.x, filtered_vel_.linear.y, filtered_vel_.angular.z);


    filtered_vel_pub_.publish(filtered_vel_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance_node");

    std::string ns = "obstacle_avoidance";
    ros::NodeHandle n(ns);

    ObstacleAvoidanceNode obstacle_avoidance(n);
    obstacle_avoidance.init();

    ros::spin();
}