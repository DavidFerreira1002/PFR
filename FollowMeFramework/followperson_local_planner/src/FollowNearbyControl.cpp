
#include "followperson_local_planner/FollowNearbyControl.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <tf2/utils.h>
#include <std_msgs/String.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Bool.h>


FollowNearbyControl::FollowNearbyControl(): 
    initialized_(false),
    person_position_received_(false) 
{}

void FollowNearbyControl::initialize(tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        previous_error = 0.0;
        current_time_ = ros::Time::now();
        previous_time_ = current_time_;
        search_start_flag_ = false;
        search_end_flag_ = false;
        search_state_msg_.data = false;
        start_time_ = ros::Time::now();

        // Initialize parameters
        nh_.getParam("/FollowPersonLocalPlanner/yaw_goal_tolerance", yaw_goal_tolerance_);
        nh_.getParam("/FollowPersonLocalPlanner/prop_gain", prop_gain);
        nh_.getParam("/FollowPersonLocalPlanner/deriv_gain", deriv_gain);
        nh_.getParam("/FollowPersonLocalPlanner/max_ang_facing_vel", max_ang_facing_vel);
        nh_.getParam("/state/topic_name", state_topic_name_);
        nh_.getParam("/followme/cmd_vel_topic_name", cmd_vel_topic_name_);
        
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_, 10);
        search_end_pub_ = nh_.advertise<std_msgs::Bool>("/search_nearby_state", 2);
        person_position_sub_ = nh_.subscribe("/target_position", 10, &FollowNearbyControl::personPositionCallback, this);
        state_handler_sub_ = nh_.subscribe(state_topic_name_, 2, &FollowNearbyControl::stateCallback, this);
        target_side_sub_ = nh_.subscribe("/target_side",2,&FollowNearbyControl::targetSideCallback,this);

        initialized_ = true;
        ROS_INFO("Follow Nearby Control initialized");
    }
    else{
        ROS_WARN("Follow Nearby Control has already been initialized");
    }
}

bool FollowNearbyControl::keepPersonCenter(){
    if (!initialized_) {
        ROS_ERROR("FollowNearbyControl has not been initialized");
        return false;
    }

    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose)) {
        ROS_WARN("Could not get robot pose");
        return false;
    }

    if(current_state_ == "FOLLOW_NEARBY"){
        //ROS_INFO("ENTERED FOLLOW_NEARBY");
        
        if (!person_position_received_) {
        ROS_WARN("No person position received yet");
        return false;
        }

        geometry_msgs::PoseStamped transformed_person_position;
        try {
            tf_->transform(person_position_, transformed_person_position, costmap_ros_->getGlobalFrameID());
        } catch (tf2::TransformException &ex) {
            //ROS_WARN("Could not transform target position: %s", ex.what());
            return false;
        }    


        tf2::Transform person_pose;
        tf2::fromMsg(transformed_person_position.pose, person_pose);

        // Calculate the angle to the person
        tf2::Vector3 to_person = person_pose.getOrigin() - tf2::Vector3(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z);
        angle_to_person = atan2(to_person.y(), to_person.x());

        yaw = tf2::getYaw(robot_pose.pose.orientation);

        angular_error = angles::shortest_angular_distance(yaw, angle_to_person);

        current_time_ = ros::Time::now();
        dt_ = current_time_ - previous_time_;
        previous_time_ = current_time_;

        if(dt_.toSec() == 0.0){
            error_derivative = 0.0;
            previous_error = 0.0;
        }
        else{
            error_derivative = (angular_error - previous_error) / dt_.toSec();
            previous_error = angular_error;
        }
        //ROS_INFO("error_derivative: %f", error_derivative);
        //ROS_INFO("previous_error: %f", previous_error);
        //ROS_INFO("angular_error: %f", angular_error);
        //ROS_INFO("yaw_goal_tolerance_: %f", yaw_goal_tolerance_);
        // Adjust the robot's orientation to face the person
        if (fabs(angular_error) > yaw_goal_tolerance_) {
            calc_ang_vel = prop_gain * angular_error + deriv_gain * error_derivative;

            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = std::max(std::min(calc_ang_vel, max_ang_facing_vel),-max_ang_facing_vel);
            //ROS_INFO("ADJUSTING ROBOTS ORIENTATION");
        } else {
            //ROS_INFO("NOT ADJUSTING ROBOTS ORIENTATION");
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
        }
        cmd_vel_pub_.publish(cmd_vel_);
    }
    return true;
}

bool FollowNearbyControl::searchNearby(){
    if (!initialized_) {
            ROS_ERROR("FollowNearbyControl has not been initialized");
            return false;
        }

    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose)) {
        ROS_WARN("Could not get robot pose");
        return false;
    }
    
    if(current_state_ == "SEARCH_NEARBY"){

        if(!search_start_flag_ && !search_end_flag_)
        {
            search_start_flag_ = true;
            start_time_ = ros::Time::now();
            search_state_msg_.data = false;
            search_end_pub_.publish(search_state_msg_);
            
        }
        else
        {
            if((ros::Time::now() - start_time_).toSec() < 12.56)
            {

                if(current_side_ == "RIGHT"){                  
                    cmd_vel_.linear.x = 0.0;
                    cmd_vel_.angular.z = -0.5;
                }
                if(current_side_ == "LEFT"){                  
                    cmd_vel_.linear.x = 0.0;
                    cmd_vel_.angular.z = 0.5;
                }
            }
            else
            {
                cmd_vel_.linear.x = 0.0;
                cmd_vel_.angular.z = 0.0;
                // search ended, publish it to warn the executive layer
                search_end_flag_ = true;
                search_state_msg_.data = search_end_flag_;
                search_end_pub_.publish(search_state_msg_);

            }
            cmd_vel_pub_.publish(cmd_vel_);
        }
        
        
        
    }
    else{
        // reset search nearby if it needsd to be reset
        if (search_start_flag_ == true || search_end_flag_ == true){
            search_start_flag_ = false;
            search_end_flag_ = false;
            search_state_msg_.data = search_end_flag_;
            search_end_pub_.publish(search_state_msg_);
        }

    }
    return true;

}

void FollowNearbyControl::personPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    person_position_ = *msg;
    person_position_received_ = true;
}

void FollowNearbyControl::stateCallback(const std_msgs::String::ConstPtr& msg){
    current_state_ = msg->data;
}

void FollowNearbyControl::targetSideCallback(const std_msgs::String::ConstPtr& msg){
    current_side_ = msg->data;
}