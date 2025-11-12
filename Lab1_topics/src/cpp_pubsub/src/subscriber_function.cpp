// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp" // Most common pieces of the ROS 2 system
#include "std_msgs/msg/string.hpp" // Includes the built-in message type you will use to publish data.
using std::placeholders::_1;

// Again our class is inherited from Node
class MinimalSubscriber : public rclcpp::Node
{
public:
  // PUBLIC CONSTRUCTOR
  MinimalSubscriber()
  : Node("minimal_subscriber") // There is no timer because the subscriber simply responds whenever data is published to the topic 
  {
    // Subscription is always listening in the topic specified
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  // MEMBER FUNCTION
 
  //The topic_callback function receives the string message data published over the topic, 
  // and simply writes it to the console using the RCLCPP_INFO macro.
  // Ntorice we specified the packages::msg::msg type:: 
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  // Data member
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


// Main function where the node actually execute 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // To start processing data from the node "MinimalSubscriber" , so we start listening
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  
  rclcpp::shutdown();
  return 0;
}

/* OBSERVATION AND COMMENTS:

-  Recall that the topic name and message type used by the publisher and
   subscriber must match to allow them to communicate.

-  Notice that we have a main for each node 


*/