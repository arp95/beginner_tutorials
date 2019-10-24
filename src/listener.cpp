/**@copyright  MIT License (c) 2019 Arpit Aggarwal
 * @file       listener.cpp
 * @author     Arpit Aggarwal
 * @brief      This tutorial demonstrates simple receipt of messages over the ROS system.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
   * @brief Prints the message sent by the publisher.
   * @param[in] message It is a variable that is sent by the publisher.
   * @return type void.
   */
void chatterCallback(const std_msgs::String::ConstPtr& message) {
    ROS_INFO("I heard: [%s]", message->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle node;

    // Subscibe to a message
    ros::Subscriber sub = node.subscribe("chatter", 1000, chatterCallback);
    ros::spin();
    return 0;
}
