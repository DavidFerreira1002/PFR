#ifndef GENERAL_CONTROL_HANDLER_H
#define GENERAL_CONTROL_HANDLER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class GeneralControlHandler
{
public:
    GeneralControlHandler(const std::string& topic_name, ros::NodeHandle nh);
    void controlCallback(const std_msgs::Bool::ConstPtr& msg);
    bool getControlState() const;

private:
    ros::NodeHandle nh_;
    ros::Subscriber bool_subscriber_;
    bool control_state_;
};

#endif // GENERAL_CONTROL_HANDLER_H
