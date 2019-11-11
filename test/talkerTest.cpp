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
 *@file       talkerTest.cpp
 *@author     Arpit Aggarwal
 *@copyright  MIT License
 *@brief      Talker Node test.
 */

#include <gtest/gtest.h>
#include "ros/ros.h"
#include <ros/service_client.h>
#include "beginner_tutorials/AddTwoInts.h"

/**
 * @brief      Tests whether the service exists or not.
 * @param      testTalkerNode         gtest framework type
 * @param      testServiceName        name of the test
 */
TEST(testTalkerNode, testServiceName) {
  // Create ros node handle.
  ros::NodeHandle node;

  // Create service client and check existence.
  ros::ServiceClient client = node.serviceClient
<beginner_tutorials::AddTwoInts>("add_two_ints");
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}

/**
 * @brief      Tests the service output.
 * @param      testTalkerNode         gtest framework type
 * @param      testServiceOutput      name of the test
 */
TEST(testTalkerNode, testServiceOutput) {
  // Create ros node handle.
  ros::NodeHandle node;

  // Create service client.
  ros::ServiceClient client = node.serviceClient
<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = 1;
  srv.request.b = 2;

  // Call service.
  client.call(srv);

  // check whether output is same or not.
  EXPECT_EQ(3, srv.response.sum);
}
