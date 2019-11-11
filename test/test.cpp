/**Copyright <2019> <Achal Pragneshkumar Vyas>
 *Copyright <YEAR> <COPYRIGHT HOLDER>

*Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are *met:

*1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

*2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the *documentation and/or other materials provided with the distribution.

*3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this *software without specific prior written permission.

*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT *LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT *HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT *LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON *ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE *USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
/**
 *@file      test.cpp
 *@author    Achal Pragneshkumar Vyas
 *@copyright BSD
 *@brief talker node
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/changeString.h"
#include "std_msgs/String.h"
/**
 * @brief      Testing the service existence
 * @param      testTalkerNode         gtest framework
 * @param      serviceExsistanceTest  Name of the test
 */
TEST(testTalkerNode, testServiceExsistance) {
  // Connects the node with ROS
  ros::NodeHandle n;

  // client to server registration
  auto client = n.serviceClient<beginner_tutorials::changeString>
  ("changeString");
  // A wait message
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}

/**
 * @brief      Tests if changeString service can replace the text 
 * @param      testTalkerNode            gtest framework
 * @param      testServiceMessageUpdate  Name of the test
 */
TEST(testTalkerNode, testServiceMessageUpdate) {
  // node handle creation to initialize the connection between nodes
  ros::NodeHandle n;

  // client to server registration
  auto client = n.serviceClient<beginner_tutorials::changeString>
  ("changeString");
  // Initialize the service to srv object
  beginner_tutorials::changeString srv;

  // changing the original text to some new text
  srv.request.stream = "input";

  // request
  client.call(srv.request, srv.response);

  EXPECT_STREQ("input", srv.response.streamUpdate.c_str());
}

