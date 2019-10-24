/**@copyright  MIT License (c) 2019 Arpit Aggarwal
 * @file       talker.cpp
 * @author     Arpit Aggarwal
 * @brief      This tutorial demonstrates simple sending of messages over the ROS system.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle node;
    
    // Advertise on topic name chatter.
    ros::Publisher chatterPub = node.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loopRate(10);

    // Send message.
    int count = 0;
    while (ros::ok()) {
        std_msgs::String message;
        std::stringstream ss;
        ss << "hello world " << count;
        message.data = ss.str();
        ROS_INFO("%s", message.data.c_str());

        // Publish message
        chatterPub.publish(message);
        ros::spinOnce();
        loopRate.sleep();
        count = count + 1;
    }
    return 0;
}
