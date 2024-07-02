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

#ifndef GESTUREHANDLER_H
#define GESTUREHANDLER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>

class GestureHandler
{
private:

  ros::NodeHandle nh_;
  ros::Subscriber hand_subscriber_;
  int gesture_predicted_m;
  int gesture_number_m;
  int previous_gesture_m;
  int gesture_filtering_number_m;

public:
  GestureHandler(int filtering_number, ros::NodeHandle nh);

  void gestureCallback(const std_msgs::Int32& msg);

  int getCurrentGesture();


};

#endif // GESTUREHANDLER_H


