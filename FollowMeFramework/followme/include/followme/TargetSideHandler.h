#ifndef TARGET_SIDE_HANDLER_H
#define TARGET_SIDE_HANDLER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

class TargetSideHandler
{
public:
    TargetSideHandler(ros::NodeHandle nh);
    void targetCallback(const std_msgs::String::ConstPtr& msg);
    std::string getTargetSide() const;

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::string target_side_;
};

#endif // TARGET_SIDE_HANDLER_H
