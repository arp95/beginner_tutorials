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
    if (message->data == "ERROR") {
        ROS_ERROR_STREAM("I heard ERROR Logger Level!");
    } else if (message->data == "WARN") {
        ROS_WARN_STREAM("I heard WARN Logger Level!");
    } else if (message->data == "FATAL") {
        ROS_FATAL_STREAM("I heard FATAL Logger Level!");
    } else if (message->data == "DEBUG") {
        ROS_DEBUG_STREAM("I heard DEBUG Logger Level!");
    } else if (message->data == "INFO") {
        ROS_INFO_STREAM("I heard INFO Logger Level!");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");

    // Use Logger Level INFO.
    ROS_INFO_STREAM("Started node listener.");

    // Subscribe to a message.
    ros::NodeHandle listener;
    ros::Subscriber sub = listener.subscribe("chatter", 1000, chatterCallback);
    ros::spin();
    return 0;
}
