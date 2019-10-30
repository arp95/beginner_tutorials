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

    // Use Logger Level INFO.
    ROS_INFO_STREAM("Started node talker.");

    // Advertise on topic name chatter.
    ros::NodeHandle node;
    ros::Publisher chatterPub =
node.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loopRate(10);

    // Publish message.
    int count =0;
    while (ros::ok()) {
        std_msgs::String message;
        std::stringstream ss;

        // Use Logger Levels WARN, ERROR, FATAL, DEBUG, INFO.
        if (count%5 == 0) {
            ss << "WARN";
            ROS_WARN_STREAM("WARN Logger Level.");
        } else if (count%7 == 0) {
            ss << "ERROR";
            ROS_ERROR_STREAM("ERROR Logger Level.");
        } else if (count%11 == 0) {
            ss << "FATAL";
            ROS_FATAL_STREAM("FATAL Logger Level.");
        } else if (count%13 == 0) {
            ss << "DEBUG";
            ROS_DEBUG_STREAM("DEBUG Logger Level.");
        } else {
            ss << "INFO";
            ROS_INFO_STREAM("INFO Logger Level.");
        }

        message.data = ss.str();
        chatterPub.publish(message);
        ros::spinOnce();
        loopRate.sleep();
        count = count + 1;
    }
    return 0;
}
