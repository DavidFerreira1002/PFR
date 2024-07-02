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

#include "followme/TransformHandler.h"


TransformHandler::TransformHandler(const std::string target_tf, const std::string source_tf):
  tf2_listener_m(tf2_buffer_m),
  target_tf_m{target_tf},
  source_tf_m{source_tf}{}

bool TransformHandler::updateTransform()
{
  bool target_found{false};
  try
  {
    target_T_source_m = tf2_buffer_m.lookupTransform(target_tf_m, source_tf_m, ros::Time(0), ros::Duration(0.1));
    if (ros::Time::now().toSec() - target_T_source_m.header.stamp.toSec() < 1)
      target_found = true;
    else
      target_found = false;
  } catch (tf2::TransformException) {
    target_found = false;
  }
  return target_found;
}

geometry_msgs::Vector3 TransformHandler::getPosition()
{
  return target_T_source_m.transform.translation;
}

geometry_msgs::Quaternion TransformHandler::getRotation()
{
  return target_T_source_m.transform.rotation;
}

geometry_msgs::TransformStamped TransformHandler::getTransform()
{
  return target_T_source_m;
}

ros::Time TransformHandler::getTimeStamp()
{
  return target_T_source_m.header.stamp;
}

double TransformHandler::getDistance()
{
  double distance = std::sqrt(
        std::pow(target_T_source_m.transform.translation.x, 2) +
        std::pow(target_T_source_m.transform.translation.y, 2) +
        std::pow(target_T_source_m.transform.translation.z, 2));

  return distance;
}
