#include "followme/GeneralControlHandler.h"

GeneralControlHandler::GeneralControlHandler(const std::string& topic_name, ros::NodeHandle nh) : control_state_(false), nh_(nh)
{   
    //ROS_INFO("Initializing subscriber for topic: %s", topic_name.c_str());
    bool_subscriber_ = nh_.subscribe(topic_name, 10, &GeneralControlHandler::controlCallback, this);
    //ROS_INFO("Subscriber initialized.");
    //ROS_INFO("Resolved topic name: %s", nh_.resolveName(topic_name).c_str());
    // Inside GeneralControlHandler constructor
    //ROS_INFO("GeneralControlHandler constructor called with node handle namespace: %s", nh_.getNamespace().c_str());


    // Manually call the callback with a default message
    //std_msgs::Bool initial_msg;
    //initial_msg.data = false;  // Set to the desired initial value
    //controlCallback(boost::make_shared<std_msgs::Bool>(initial_msg));

}

void GeneralControlHandler::controlCallback(const std_msgs::Bool::ConstPtr& msg)
{
    //ROS_INFO("Callback triggered with message: %s", msg->data ? "true" : "false");
    control_state_ = msg->data;
}

bool GeneralControlHandler::getControlState() const
{
    return control_state_;
}
