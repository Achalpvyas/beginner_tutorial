/**Copyright <2019> <Achal Pragneshkumar Vyas>
 *Copyright <YEAR> <COPYRIGHT HOLDER>

*Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are *met:

*1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

*2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the *documentation and/or other materials provided with the distribution.

*3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this *software without specific prior written permission.

*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT *LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT *HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT *LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON *ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE *USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file 		 talker.cpp
 * @author 		 Achal Vyas
 * @copyright            BSD
 * @brief 		 ROS Publisher
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include "beginner_tutorials/changeString.h"

extern std::string stream = " MY NAME IS ACHAL ";


/**
 * @brief a function that changes the stream message
 * @param req represents the data being sent to the service
 * @param res represents the data being provided by the client
 * @return bool
 */
bool streamUpdate(beginner_tutorials::changeString::Request &req,
                beginner_tutorials::changeString::Response &res) {
  stream = req.stream;
  ROS_INFO_STREAM("The string is changed to the new string: ");
  res.streamUpdate = req.stream;
  return true;
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function initializes the talker node
   */ 
  ros::init(argc, argv, "talker");

  // Using a variable to store freq
  int freq = 2;

  /**
   * NodeHandle maintains the communication with ROS.
   */
  ros::NodeHandle n;

  ros::Rate loop_rate(freq);
  // Let's generate log messages
  if (argc > 1) {
    // Converting string argument to integer
    freq = atoi(argv[1]);

    if (freq < 0) {
      // For negative frequency or less than zero
      ROS_FATAL_STREAM("Frequency cannot be less than zero");
      ros::shutdown();
      return 1;
    } else if (freq == 0) {
      // For frequency equals to '0'
      ROS_ERROR_STREAM("Frequency cannot be zero!!");
      ROS_INFO_STREAM("Changing frequency to 4hz");
      freq = 4;
    } else if (freq >= 10) {
      // For frequency greater than 10
      ROS_WARN_STREAM("very fast to read!!");
      ROS_INFO_STREAM("Changing frequency to 5hz");
      freq = 5;
    }
  }
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);

  // For service to changeBaseOutputString
  ros::ServiceServer server = n.advertiseService("changeString",
                                                 streamUpdate);
  int count = 0;
  while (ros::ok()) {
    ROS_DEBUG_STREAM_ONCE("Current frequency: " << freq);

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << stream;
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function represents the way of sending messages.
     */
    chatter_pub.publish(msg);

    // set translation
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(cos(ros::Time::now().toSec()),
                    sin(ros::Time::now().toSec()), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 1);

    // set rotation
    transform.setRotation(q);

    // broadcast the transform
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
