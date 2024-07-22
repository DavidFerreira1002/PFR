// followperson_local_planner.h

#ifndef FOLLOWPERSON_LOCAL_PLANNER_H_
#define FOLLOWPERSON_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <angles/angles.h>
#include <tf2/utils.h>
#include <std_msgs/String.h>
#include <thread>
#include "FollowNearbyControl.h"

namespace followperson_local_planner {

class FollowPersonLocalPlanner : public nav_core::BaseLocalPlanner {
public:
    FollowPersonLocalPlanner();
    ~FollowPersonLocalPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    
private:
    void runFollowNearbyControl();
    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    bool initialized_;
    ros::Subscriber person_position_sub_;
    ros::Subscriber state_handler_sub_;
    geometry_msgs::PoseStamped person_position_;
    bool person_position_received_;
    std::string state_;

    base_local_planner::TrajectoryPlannerROS traj_planner_;

    FollowNearbyControl follow_nearby_control;
    std::thread follow_nearby_control_thread_;

    void personPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void stateHandlerCallback(const std_msgs::String::ConstPtr& msg);
    double yaw_goal_tolerance_;
    double xy_goal_tolerance_;
    double distance_to_person_;
    double prop_gain;
    double deriv_gain;
    double max_ang_facing_vel;
    double previous_error;
    double angle_to_person;
    double yaw;
    double angular_error;
    double error_derivative;
    double calc_ang_vel;
    std::string state_topic_name_;
    ros::Time current_time_;
    ros::Time previous_time_;
    ros::Duration dt_;
};

} // namespace followperson_local_planner

#endif // FOLLOWPERSON_LOCAL_PLANNER_H_
