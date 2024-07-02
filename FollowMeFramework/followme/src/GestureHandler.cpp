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
#include "followme/GestureHandler.h"

GestureHandler::GestureHandler(int filtering_number, ros::NodeHandle nh): gesture_filtering_number_m{filtering_number}, nh_(nh)
{
  hand_subscriber_ = nh_.subscribe("/hand_gesture", 10, &GestureHandler::gestureCallback, this);
  gesture_number_m = 0;
  previous_gesture_m = -1;
  gesture_predicted_m = -1;
}

void GestureHandler::gestureCallback(const std_msgs::Int32& msg)
{
  if (previous_gesture_m == msg.data)
  {
    if (++gesture_number_m >= gesture_filtering_number_m)
    {
      gesture_predicted_m = msg.data;
      gesture_number_m = 0;
    }
  }
  else
  {
    gesture_number_m = 0;
  }
  previous_gesture_m = msg.data;
}

int GestureHandler::getCurrentGesture()
{
  return gesture_predicted_m;
}
