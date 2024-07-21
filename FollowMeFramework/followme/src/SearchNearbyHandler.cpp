#include "followme/SearchNearbyHandler.h"

SearchNearbyHandler::SearchNearbyHandler(ros::NodeHandle nh) : search_state_(false), nh_(nh)
{   
    bool_subscriber_ = nh_.subscribe("/search_nearby_state", 10, &SearchNearbyHandler::searchCallback, this);
}

void SearchNearbyHandler::searchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    search_state_ = msg->data;
}

bool SearchNearbyHandler::getSearchState() const
{
    return search_state_;
}