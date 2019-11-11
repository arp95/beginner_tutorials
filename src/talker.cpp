/**
 *  MIT License
 *
 *  Copyright (c) 2019 Arpit Aggarwal
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/**
 *@file       talker.cpp
 *@author     Arpit Aggarwal
 *@copyright  MIT License
 *@brief      This tutorial demonstrates simple sending of messages over the ROS system.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <tf/transform_broadcaster.h>
#include <sstream>

/**
   * @brief Adds two integers.
   * @param[in] req Two numbers as requested by the client.
   * @param[in] res The result of addition of two numbers.
   * @return type bool.
   */
bool add(beginner_tutorials::AddTwoInts::Request
&req, beginner_tutorials::AddTwoInts::Response &res) {
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

    // Use Logger Levels WARN, ERROR, FATAL, DEBUG, INFO.
    if (param == "warn") {
        ROS_WARN_STREAM("WARN Logger Level.");
    } else if (param == "error") {
        ROS_ERROR_STREAM("ERROR Logger Level.");
    } else if (param == "fatal") {
        ROS_FATAL_STREAM("FATAL Logger Level.");
    } else if (param == "debug") {
        ROS_DEBUG_STREAM("DEBUG Logger Level.");
    } else {
        ROS_INFO_STREAM("INFO Logger Level.");
    }
    // Advertise service.
    ros::ServiceServer service = talker.advertiseService("add_two_ints", add);
    ROS_INFO_STREAM("Service ready.");

    // Publish INFO messages.
    int count = 0;
    while (ros::ok()) {
        // tf Broadcast frame1.
        static tf::TransformBroadcaster tfBr;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(count, 0.0, 0.0));
        tf::Quaternion quaternion;
        quaternion.setRPY(0, 90, 90);
        transform.setRotation(quaternion);
        tfBr.sendTransform(tf::StampedTransform(
transform, ros::Time::now(), "world", "talk"));
        std_msgs::String message;
        std::stringstream ss;
        ROS_INFO_STREAM("Entered Loop.");

        // Use Logger Levels on the basis of count variable.
        if (count%3 == 0) {
            ss << "WARN";
            ROS_WARN_STREAM("WARN Logger Level.");
        } else if (count%5 == 0) {
            ss << "ERROR";
            ROS_ERROR_STREAM("ERROR Logger Level.");
        } else if (count%7 == 0) {
            ss << "FATAL";
            ROS_FATAL_STREAM("FATAL Logger Level.");
        } else if (count%11 == 0) {
            ss << "DEBUG";
            ROS_DEBUG_STREAM("DEBUG Logger Level.");
        } else {
            ss << "INFO";
            ROS_INFO_STREAM("INFO Logger Level.");
        }

        count = count + 1;
        message.data = ss.str();
        chatterPub.publish(message);
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
