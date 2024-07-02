#include <pluginlib/class_list_macros.h>
#include <followperson_local_planner/followperson_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <tf2/utils.h>
#include <std_msgs/String.h>


// Register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(followperson_local_planner::FollowPersonLocalPlanner, nav_core::BaseLocalPlanner)

namespace followperson_local_planner {

FollowPersonLocalPlanner::FollowPersonLocalPlanner() : initialized_(false), person_position_received_(false) {}

FollowPersonLocalPlanner::~FollowPersonLocalPlanner() {
    if (follow_nearby_control_thread_.joinable()) {
        follow_nearby_control_thread_.join();
    }
}

void FollowPersonLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        ros::NodeHandle nh("~/" + name);
        previous_error = 0.0;
        current_time_ = ros::Time::now();
        previous_time_ = current_time_;

        // Initialize parameters
        nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
        nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
        nh.param("distance_to_person", distance_to_person_, 1.0);
        nh.param("prop_gain", prop_gain, 1.0);
        nh.param("deriv_gain", deriv_gain, 0.1);
        nh.param("max_ang_facing_vel", max_ang_facing_vel, 1.0);
        nh.getParam("/state/topic_name", state_topic_name_);
        
        traj_planner_.initialize("TrajectoryPlannerROS", tf_, costmap_ros_);
        person_position_sub_ = nh.subscribe("/target_position", 10, &FollowPersonLocalPlanner::personPositionCallback, this);
        state_handler_sub_ = nh.subscribe(state_topic_name_,2, &FollowPersonLocalPlanner::stateHandlerCallback, this);

        follow_nearby_control.initialize(tf_, costmap_ros_);
        follow_nearby_control_thread_ = std::thread(&FollowPersonLocalPlanner::runFollowNearbyControl, this);

        initialized_ = true;
        ROS_INFO("FollowPersonLocalPlanner initialized");
    } else {
        ROS_WARN("FollowPersonLocalPlanner has already been initialized");
    }
}

void FollowPersonLocalPlanner::runFollowNearbyControl() {
    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        follow_nearby_control.keepPersonCenter();
        rate.sleep();
    }
}

bool FollowPersonLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_) {
        ROS_ERROR("FollowPersonLocalPlanner has not been initialized");
        return false;
    }

    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose)) {
        ROS_WARN("Could not get robot pose");
        return false;
    }

    if (state_ == "GENERAL"){

        // Use DWA Planner for normal navigation
        traj_planner_.setPlan(global_plan_);
        return traj_planner_.computeVelocityCommands(cmd_vel);
    }
    else if (state_ == "FOLLOW"){

        if (!person_position_received_) {
        ROS_WARN("No person position received yet");
        return false;
        }

        geometry_msgs::PoseStamped transformed_person_position;
        try {
            tf_->transform(person_position_, transformed_person_position, costmap_ros_->getGlobalFrameID());
        } catch (tf2::TransformException &ex) {
            //ROS_WARN("Could not transform target position: %s", ex.what());
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
        
        // Adjust the robot's orientation to face the person
        if (fabs(angular_error) > yaw_goal_tolerance_) {
            calc_ang_vel = prop_gain * angular_error + deriv_gain * error_derivative;
            
            cmd_vel.angular.z = std::max(std::min(calc_ang_vel, max_ang_facing_vel),-max_ang_facing_vel);
            cmd_vel.linear.x = 0.0;
            //ROS_INFO("ADJUSTING ROBOTS ORIENTATION");
        } else {
            // Use DWA Planner for normal navigation
            traj_planner_.setPlan(global_plan_);
            //ROS_INFO("USING NORMAL PLANNER");
            return traj_planner_.computeVelocityCommands(cmd_vel);
        }
    }
    else if(state_ == "SEARCH"){
        
        ROS_INFO("IN SEARCHING-NOT IMPLEMENTED YET");
    }
    else if(state_ == "FOLLOW_NEARBY"){
        return true;
    }
    else {
        ROS_WARN("Unknown state: %s", state_.c_str());
        return false;
    }

    return true;
}

bool FollowPersonLocalPlanner::isGoalReached() {
    if (!initialized_) {
        ROS_ERROR("FollowPersonLocalPlanner has not been initialized");
        return false;
    }

    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose)) {
        ROS_WARN("Could not get robot pose");
        return false;
    }

    tf2::Transform goal_pose;
    tf2::fromMsg(global_plan_.back().pose, goal_pose);

    double distance_to_goal = hypot(robot_pose.pose.position.x - goal_pose.getOrigin().x(), robot_pose.pose.position.y - goal_pose.getOrigin().y());
    double yaw_to_goal = tf2::getYaw(goal_pose.getRotation());

    if (distance_to_goal <= xy_goal_tolerance_ && fabs(angles::shortest_angular_distance(tf2::getYaw(robot_pose.pose.orientation), yaw_to_goal)) <= yaw_goal_tolerance_) {
        return true;
    }

    return false;
}

bool FollowPersonLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("FollowPersonLocalPlanner has not been initialized");
        return false;
    }

    global_plan_ = plan;
    return true;
}

void FollowPersonLocalPlanner::personPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    person_position_ = *msg;
    person_position_received_ = true;
}

void FollowPersonLocalPlanner::stateHandlerCallback(const std_msgs::String::ConstPtr& msg) {
    state_ = msg->data;
}

} // namespace followperson_local_planner
