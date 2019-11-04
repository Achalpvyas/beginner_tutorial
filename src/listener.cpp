/**Copyright <2019> <Achal Pragneshkumar Vyas>

*Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are *met:

*1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

*2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the *documentation and/or other materials provided with the distribution.

*3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this *software without specific prior written permission.
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT *LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT *HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT *LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY *THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE O *THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *@file       listener.cpp
 *@author     Achal Vyas
 *@copyright  BSD
 *@brief      ROS subscriber 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>


void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  /**
   * The ros::init() starts ros node
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle estalishes and keeps contact with the node
   */
  ros::NodeHandle n;

 
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
