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

#ifndef TRANSFORMHANDLER_H
#define TRANSFORMHANDLER_H

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>


class TransformHandler
{
private:
  tf2_ros::Buffer tf2_buffer_m;
  tf2_ros::TransformListener tf2_listener_m;
  geometry_msgs::TransformStamped target_T_source_m;
  const std::string target_tf_m, source_tf_m;

public:
  TransformHandler(const std::string target_tf, const std::string source_tf);

  bool updateTransform();
  geometry_msgs::Vector3 getPosition();
  geometry_msgs::Quaternion getRotation();
  geometry_msgs::TransformStamped getTransform();
  ros::Time getTimeStamp();
  double getDistance();

};

#endif // TRANSFORMHANDLER_H

