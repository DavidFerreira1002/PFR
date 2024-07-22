#ifndef SEARCH_NEARBY_HANDLER_H
#define SEARCH_NEARBY_HANDLER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class SearchNearbyHandler
{
public:
    SearchNearbyHandler(ros::NodeHandle nh);
    void searchCallback(const std_msgs::Bool::ConstPtr& msg);
    bool getSearchState() const;

private:
    ros::NodeHandle nh_;
    ros::Subscriber bool_subscriber_;
    bool search_state_;
};

#endif // SEARCH_NEARBY_HANDLER_H
