#include "followme/TargetSideHandler.h"

TargetSideHandler::TargetSideHandler(ros::NodeHandle nh) : target_side_("LEFT"), nh_(nh)
{   
    subscriber_ = nh_.subscribe("/target_side", 10, &TargetSideHandler::targetCallback, this);
}

void TargetSideHandler::targetCallback(const std_msgs::String::ConstPtr& msg)
{   
    target_side_ = msg->data;
}

std::string TargetSideHandler::getTargetSide() const
{
    return target_side_;
}