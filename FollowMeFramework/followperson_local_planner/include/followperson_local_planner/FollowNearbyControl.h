#ifndef FOLLOWNEARBYCONTROL_H_
#define FOLLOWNEARBYCONTROL_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>


class FollowNearbyControl
{
public:
    FollowNearbyControl();

    void initialize(tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool keepPersonCenter();
    bool searchNearby();

private:
    bool initialized_;

    ros::NodeHandle nh_;
    tf2_ros::Buffer* tf_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher search_end_pub_;
    ros::Subscriber person_position_sub_;
    ros::Subscriber state_handler_sub_;
    ros::Subscriber target_side_sub_;

    std_msgs::Bool search_state_msg_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::string cmd_vel_topic_name_;
    double prop_gain;
    double deriv_gain;
    double max_ang_facing_vel;
    double previous_error;
    double angle_to_person;
    double yaw;
    double angular_error;
    double error_derivative;
    double calc_ang_vel;
    double yaw_goal_tolerance_;

    geometry_msgs::Twist cmd_vel_;

    ros::Time start_time_;
    geometry_msgs::PoseStamped person_position_;
    bool person_position_received_;
    bool search_start_flag_;
    bool search_end_flag_;
    std::string state_topic_name_;
    std::string current_state_;
    std::string current_side_;
    ros::Time current_time_;
    ros::Time previous_time_;
    ros::Duration dt_;

    void personPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg);
    void targetSideCallback(const std_msgs::String::ConstPtr& msg);

};

#endif // FOLLOWNEARBYCONTROL_H_