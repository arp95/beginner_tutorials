/**@copyright  MIT License (c) 2019 Arpit Aggarwal
 * @file       talker.cpp
 * @author     Arpit Aggarwal
 * @brief      This tutorial demonstrates simple sending of messages over the ROS system.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <sstream>

/**
   * @brief Adds two integers.
   * @param[in] req Two numbers as requested by the client.
   * @param[in] res The result of addition of two numbers.
   * @return type bool.
   */
bool add(beginner_tutorials::AddTwoInts::Request  &req, beginner_tutorials::AddTwoInts::Response &res) {
    res.sum = req.a + req.b;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");

    // Using Logger Level INFO and get param.
    ROS_INFO_STREAM("Started node talker.");
    ros::NodeHandle node("~");
    std::string param;
    node.getParam("param", param);

    // Advertise on topic name chatter.
    ros::NodeHandle talker;
    ros::Publisher chatterPub =
talker.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loopRate(10);

    // Sample Logger Levels.
    ROS_INFO_STREAM("INFO Logger Level.");
    ROS_DEBUG_STREAM("DEBUG Logger Level.");
    ROS_WARN_STREAM("WARN Logger Level.");
    ROS_ERROR_STREAM("ERROR Logger Level.");
    ROS_FATAL_STREAM("FATAL Logger Level.");

    // Advertise service.
    ros::ServiceServer service = talker.advertiseService("add_two_ints", add);
    ROS_INFO_STREAM("Service ready.");

    // Publish INFO messages.
    while (ros::ok()) {
        std_msgs::String message;
        std::stringstream ss;

        // Use Logger Levels WARN, ERROR, FATAL, DEBUG, INFO.
        if (param == "warn") {
            ss << "WARN";
            ROS_WARN_STREAM("WARN Logger Level.");
        } else if (param == "error") {
            ss << "ERROR";
            ROS_ERROR_STREAM("ERROR Logger Level.");
        } else if (param == "fatal") {
            ss << "FATAL";
            ROS_FATAL_STREAM("FATAL Logger Level.");
        } else if (param == "debug") {
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
    }
    return 0;
}
