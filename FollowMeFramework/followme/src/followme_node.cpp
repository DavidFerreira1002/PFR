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

#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "followme/TransformHandler.h"
#include "followme/GestureHandler.h"
#include "followme/GeneralControlHandler.h"
#include "followme/SearchNearbyHandler.h"
#include "followme/TargetSideHandler.h"

#include <std_msgs/String.h>
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

// STATE ENUM
enum states_t {GENERAL=0 ,STEADY, FOLLOW, SEARCH, WAIT, FOLLOW_NEARBY, SEARCH_NEARBY, ENUM_LENGTH};

// STATE HANDLER FOR STATE MACHINE
class States
{
public:
  states_t state_m;
  const std::string state_str_m[ENUM_LENGTH] = {"GENERAL", "STEADY", "FOLLOW", "SEARCH", "WAIT", "FOLLOW_NEARBY", "SEARCH_NEARBY"};

  States(ros::NodeHandle& nh, const std::string& topic_name)
  {
    state_pub_ = nh.advertise<std_msgs::String>(topic_name, 10, true);
    changeStateTo(GENERAL);
  }

  void changeStateTo(states_t new_state)
  {
    state_m = new_state;
    ROS_INFO("State-machine state changed to %s", state_str_m[new_state].c_str());

    //publish the new state
    std_msgs::String state_msg;
    state_msg.data = state_str_m[new_state];
    state_pub_.publish(state_msg);
  }

private:
  ros::Publisher state_pub_;

};

void deleteRobotGoal(MoveBaseClient& mvb)
{
  // delete the move base goal
  if (mvb.getState() == actionlib::SimpleClientGoalState::ACTIVE)
  {
    ROS_INFO("Deleted move base goal.");
    mvb.cancelGoal();
  }
}

int main(int argc, char **argv)
{
  // ---------------------ROS INITIALIZATION---------------------
  ros::init(argc, argv, "followme_node");

  //std::string ns = "followme";
  ros::NodeHandle n;

  // read tfs name from ros param server
  std::string map_tf, camera_tf, target_tf;
  n.getParam("/followme/target_tf_name", target_tf);
  n.getParam("/followme/map_tf_name", map_tf);
  n.getParam("/followme/camera_tf_name", camera_tf);

  // tf handler for target
  TransformHandler map_tf_handler(map_tf, target_tf);
  TransformHandler camera_tf_handler(camera_tf, target_tf);

  int gesture_filtering_number;
  n.getParam("/followme/gesture_filtering_number", gesture_filtering_number);

  // General control topic 
  std::string general_topic_name;
  n.getParam("/general/topic_name", general_topic_name);

  // General control handler
  GeneralControlHandler general_control_handler(general_topic_name, n);

  // Gesture handler
  GestureHandler gesture_handler(gesture_filtering_number, n);

  // Search nearby handler
  SearchNearbyHandler search_nearby_handler(n);

  // Target side handler
  TargetSideHandler target_side_handler(n);

  bool is_robot_waiting{true};

  // Gesture subscriber
  //ros::Subscriber gesture_sub = n.subscribe("/hand_gesture", 10, &GestureHandler::gestureCallback, &gesture_handler);
  
  // vel publisher
  std::string cmd_vel_topic, cmd_vel_father_frame_id;
  n.getParam("/followme/robot_base_tf_name", cmd_vel_father_frame_id);
  n.getParam("/followme/cmd_vel_topic_name", cmd_vel_topic);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 100);
  geometry_msgs::Twist cmd_vel;


  //--------------------------------------------------------------------
  TransformHandler robot_tf_handler(map_tf, cmd_vel_father_frame_id);

  // ------------ MOVE BASE ---------------------
  std::string move_base_node_name;
  n.getParam("/followme/move_base_node_name", move_base_node_name);
  MoveBaseClient move_base(move_base_node_name, true);

  while(!move_base.waitForServer(ros::Duration(10.0))){
      ROS_WARN_THROTTLE(1.0, "Waiting for the move_base action server to come up");
  }
  if (!move_base.isServerConnected())
  {
    ROS_ERROR("Cannot connect to Move Base action server. Make sure that the Move Base server has been started");
    exit(1);
  }

  // ------------------VARIABLES INITIALIZATION------------------

  // if the person is inside this circle the robot should stop moving
  double safety_circle_radius;
  n.getParam("/followme/safety_circle_radius", safety_circle_radius);

  // sequence counter
  unsigned int count{0};

  // state handler topic name
  std::string state_topic_name;
  n.getParam("/state/topic_name",state_topic_name);

  // state handler initialization
  States state_handler(n,state_topic_name);

  // position of target with respect to map
  geometry_msgs::Vector3 target_p_map;

  // goal initialization
  move_base_msgs::MoveBaseGoal curr_goal;
  move_base_msgs::MoveBaseGoal old_goal;
  curr_goal.target_pose.header.frame_id = map_tf;
  curr_goal.target_pose.header.seq = count;

  // useful variable init
  double prev_goal_x{}, prev_goal_y{}; // previous sent goal position
  double goal_distance_from_prev_goal{}; // distance between curent goal and previous goal
  double heading_angle{0}, robot_map_angle{0}, goal_angle{0}; // angle between target and robot
  double goal_threshold; // distance threshold after which the current goal can be considered a new goal
  n.getParam("/followme/goal_threshold", goal_threshold);

  // SEARCH state variables
  //double v_search{}, w_search{}; // linear and angular velocity used in SEARCH state
  //double constant_w; // constant angular velocity during SEARCH state
  //n.getParam("/followme/search_const_ang_vel", constant_w);
  //int direction{}; // track the direction of the person in the camera
  //double turn{}; // used to track a 360° turn in SEARCH state
  //geometry_msgs::Twist search_vel; // velocity msg used in SEARCH state
  bool search_start_flag = false;
  bool rotating_flag = false;

  double camera_target_distance;

  // time variables
  ros::Time start_time = ros::Time::now();
  ros::Time time;
  double time_prev{ros::Time::now().toSec()}, // [s] previous  time
         curr_time,                           // [s] curr time
         delta_t, last_time_seen;
         

  // -----------------LOOP START-----------------
  while (ros::ok()) {

    time = ros::Time::now();
    curr_time = time.toSec();

    delta_t = curr_time - time_prev;
    switch(state_handler.state_m)
    {

    case GENERAL: // ---------GENERAL CASE---------
    // Robot will behave like a normal robot, outside packages will be able to control
    //the robot by sending goals to move_base if necessary.
    // If a true is sent on general_control topic, the general will turn on, if a false is sent,
    //it will move to STEADY, and start the followme.

      if (!general_control_handler.getControlState())
      {
        state_handler.changeStateTo(STEADY);
      }
      
      break; 

    case STEADY: // --------STEADY CASE--------
      // stop the robot by deleting the goal
      deleteRobotGoal(move_base);

      // if general_control is true, change to GENERAL
      if (general_control_handler.getControlState())
      {
        state_handler.changeStateTo(GENERAL);
      }

      // if transform from camera to target exists change to follow
      if(camera_tf_handler.updateTransform())
      {
        if(is_robot_waiting)
        {
          state_handler.changeStateTo(WAIT);
        }
        else
        {
          state_handler.changeStateTo(FOLLOW);
        }

      }
      break;

    case FOLLOW: // --------FOLLOW CASE--------
      // if transform from camera to target does not exists change to search
      if(!camera_tf_handler.updateTransform() && curr_time - last_time_seen > 2.5)
      {
        // cancel last goal
        deleteRobotGoal(move_base);
        // compute last direction
        //direction = static_cast<int>((0 < heading_angle) - (heading_angle < 0));
        // reinitialize the turn variable
        //turn = M_PI*2;
        state_handler.changeStateTo(SEARCH);
        break;
      }
      else if(gesture_handler.getCurrentGesture() == 0)
      {
        state_handler.changeStateTo(WAIT);
        is_robot_waiting = true;
        break;
      }

      camera_tf_handler.updateTransform();
      if(camera_tf_handler.updateTransform()){
        last_time_seen = ros::Time::now().toSec();  
      }
      
      // get angle of target with respect to camera in camera_depth_optical_frame convention
      heading_angle = -std::atan2(camera_tf_handler.getPosition().x,
                                  camera_tf_handler.getPosition().z);

      camera_target_distance = camera_tf_handler.getDistance();
      // check if person is inside the safety circle
      if (camera_target_distance < safety_circle_radius)
      {
        // if (camera_target_distance > safety_circle_radius*2/3 && std::abs(heading_angle) > 0.2)
        // {
        //   v_search    =  0;
        //   w_search    =  heading_angle;
        // }
        // else
        // {
        //   // heading angle is set to zero because if the robot goes in searching the direction will be set
        //   // to zero and it does not turns (for sagety reasons)
        //   heading_angle = 0;
        //   v_search    =  0;
        //   w_search    =  0;
        // }

        // search_vel.linear.x = v_search;
        // search_vel.angular.z = w_search;

        // //pub.publish(search_vel);

        // deleteRobotGoal(move_base);
        state_handler.changeStateTo(FOLLOW_NEARBY);
        break;
      }


      // set GOAL POSITION using target position with respect to map
      map_tf_handler.updateTransform();
      target_p_map = map_tf_handler.getPosition();
      curr_goal.target_pose.pose.position.x = target_p_map.x;
      curr_goal.target_pose.pose.position.y = target_p_map.y;

      // set GOAL ORIENTATION using target heading angle with respect to robot
      robot_tf_handler.updateTransform();

      // get robot angle with respect to map
      robot_map_angle = 2*std::atan2(robot_tf_handler.getRotation().z,
                                     robot_tf_handler.getRotation().w);

      // set final goal angle
      goal_angle = robot_map_angle + heading_angle;

      curr_goal.target_pose.pose.orientation.z = std::sin(goal_angle/2);
      curr_goal.target_pose.pose.orientation.w = std::cos(goal_angle/2);

      // send a goal only if current goal position is distant more then a given threshold with respect to the previous goal
      goal_distance_from_prev_goal =
        std::sqrt(std::pow(prev_goal_x - curr_goal.target_pose.pose.position.x, 2) +
                  std::pow(prev_goal_y - curr_goal.target_pose.pose.position.y, 2));

      // if the current goal is distant from the previous goal more than a threshold
      if (goal_distance_from_prev_goal > goal_threshold)
      {
        
        //ROS_INFO_STREAM(curr_goal.target_pose.pose.position.x);
        //ROS_INFO_STREAM(curr_goal.target_pose.pose.position.y);
        //ROS_INFO_STREAM(curr_goal.target_pose.pose.orientation.z);
        //ROS_INFO_STREAM(curr_goal.target_pose.pose.orientation.w);
        curr_goal.target_pose.header.seq = count;
        curr_goal.target_pose.header.stamp = map_tf_handler.getTimeStamp(); // ros::Time::now();
        //save the current goal to use if necessary on the SEARCH case
        old_goal = curr_goal;

        move_base.sendGoal(curr_goal);
        ROS_INFO("Sent new goal to move_base.");
        prev_goal_x = curr_goal.target_pose.pose.position.x;
        prev_goal_y = curr_goal.target_pose.pose.position.y;
        count++;
      }

      break;

    case FOLLOW_NEARBY: //---------------------FOLLOW NEARBY CASE--------------------------
      deleteRobotGoal(move_base);

      // if transform from camera to target does not exists change to search
      if(!camera_tf_handler.updateTransform() && curr_time - last_time_seen > 2.5)
      {
        // compute last direction
        //direction = static_cast<int>((0 < heading_angle) - (heading_angle < 0));
        // reinitialize the turn variable
        //turn = M_PI*2;
        state_handler.changeStateTo(SEARCH_NEARBY);
        break;
      }
      else if(gesture_handler.getCurrentGesture() == 0)
      {
        state_handler.changeStateTo(WAIT);
        is_robot_waiting = true;
        break;
      }

      camera_tf_handler.updateTransform();
      if(camera_tf_handler.updateTransform()){
        last_time_seen = ros::Time::now().toSec();  
      }

      // get angle of target with respect to camera in camera_depth_optical_frame convention
      heading_angle = -std::atan2(camera_tf_handler.getPosition().x,
                                  camera_tf_handler.getPosition().z);

      camera_target_distance = camera_tf_handler.getDistance();
      // check if person is inside the safety circle
      if (camera_target_distance > safety_circle_radius)
      {
        state_handler.changeStateTo(FOLLOW);
        break;
      }
      break;

    case SEARCH: // --------SEARCH CASE--------  
      //Checks target first, then if it should rotate, then if the search has already started since
      // we only want one goal to be sent, if at the end of 15 secs it doesnt find it, we turn
      // on the rotating flag, it will now try to rotate.
      //While rotating it will check in what side the target was, rotate 90 degrees that way and then
      // rotate 180 degrees the other way, and go back 90 degrees to center again and change to STEADY.
      //Sadly we need to time it, so this will probably change if the rate goes under 10Hz.
      if(camera_tf_handler.updateTransform())
      {
        //reset the vars
        deleteRobotGoal(move_base);
        search_start_flag = false;
        rotating_flag = false;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        //change to FOLLOW because we found the target, I hope it is the target atleast
        state_handler.changeStateTo(FOLLOW);
      }
      else if(rotating_flag)
      {
        //first rotation 90
        if((ros::Time::now() - start_time).toSec() < (15.0 + 6.24))
        {
          if(target_side_handler.getTargetSide() == "LEFT")
          {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.5;
          }
          else
          {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -0.5;
          }

        }
        //second rotation 180
        else if((ros::Time::now() - start_time).toSec() < (15.0 + 6.24 + 12.56))
        {
          if(target_side_handler.getTargetSide() == "LEFT")
          {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -0.5;
          }
          else
          {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.5;            
          }
        }
        //third rotation 90 (back to the start)
        else if((ros::Time::now() - start_time).toSec() < (15.0 + 6.24 + 12.56 + 6.24))
        {
          if(target_side_handler.getTargetSide() == "LEFT")
          {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.5;
          }
          else
          {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -0.5;            
          }
        }
        else
        { 
          // Reset everything, some might not be necessary but just making sure
          //that the endpoints all agree 
          deleteRobotGoal(move_base);
          search_start_flag = false;
          rotating_flag = false;
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
          // We did not find the target, sad, change to STEADY
          state_handler.changeStateTo(STEADY);
        }
        cmd_vel_pub.publish(cmd_vel);
      }
      else if(!search_start_flag)
      {
        search_start_flag = true;
        move_base.sendGoal(old_goal);
        start_time = ros::Time::now();
      }
      else
      {
        if((ros::Time::now() - start_time).toSec() > 15.0)
        {
          deleteRobotGoal(move_base);
          search_start_flag = false;
          rotating_flag = true;
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
        }
      }
      break;

    case SEARCH_NEARBY: //--------------SEARCH NEARBY CASE--------------

      if(camera_tf_handler.updateTransform())
      {
        camera_target_distance = camera_tf_handler.getDistance();
        // check if person is inside the safety circle
        if (camera_target_distance < safety_circle_radius)
        {
          state_handler.changeStateTo(FOLLOW_NEARBY);
        }
        else
        {
          state_handler.changeStateTo(FOLLOW);
        } 
      }
      else if (search_nearby_handler.getSearchState())
      {
        state_handler.changeStateTo(SEARCH);
      }
      break;

    case WAIT: // --------WAIT CASE--------

      // delete the move base goal
      deleteRobotGoal(move_base);
      // reset previous goal
      prev_goal_x = 0;
      prev_goal_y = 0;
      //ROS_INFO("Current Gesture: %d", gesture_handler.getCurrentGesture());
      if(gesture_handler.getCurrentGesture() == 1)
      {
        state_handler.changeStateTo(FOLLOW);
        is_robot_waiting = false;
        break;
      }
      if(!camera_tf_handler.updateTransform())
      {
        state_handler.changeStateTo(STEADY);
        break;
      }

      break;

    case ENUM_LENGTH: // --------ENUM_LENGTH CASE - NOT A REAL CASE --------
      ROS_ERROR("State-machine state changed to ENUM_LENGTH state which is used only to have the length of the enum..");
      exit(1);
    };

    time_prev = curr_time;
    ros::spinOnce();
  }
  return 0;
}
