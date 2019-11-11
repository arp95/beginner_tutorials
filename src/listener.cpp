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
 *@file       listener.cpp
 *@author     Arpit Aggarwal
 *@copyright  MIT License
 *@brief      This tutorial demonstrates simple receipt of messages over the ROS system.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddTwoInts.h"

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
    // Call service.
    ros::ServiceClient client = listener.serviceClient
<beginner_tutorials::AddTwoInts>("add_two_ints");
    beginner_tutorials::AddTwoInts srv;
    srv.request.a = 1;
    srv.request.b = 2;
    if (client.call(srv)) {
        ROS_INFO_STREAM("Sum is: " << srv.response.sum);
    } else {
        ROS_ERROR_STREAM("Error in service!");
    }
    ros::spin();
    return 0;
}
