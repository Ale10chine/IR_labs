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

#include <chrono>
#include <memory>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Node example class (Minimal publisher)
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

// MinimalPublisher is inherited from Node
class MinimalPublisher : public rclcpp::Node
{
public:
  // PUBLIC CONSTRUCTOR
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0) // Name the node and set count to 0
  {

    // Inizialization of the publisher that would publish on the topic "topic" with a max length of msgs of 10. Are
    // stored in a sort of cache and for the moment 10 is a magic number   
    // Topic must be unique and meaningful
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // Initializer of the timer which causes the timer_callback function to be executed twice a second
    // create_wall_timer is based on a wall clock 
    // the second argument of the function is the calback function that will be called 
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  // Function memmber timer_callback() is used to write a messsage and publish on the topic
  // The timer_callback function is where the message data is set and the messages are actually published.
  // The RCLCPP_INFO macro ensures every published message is printed to the console.
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }


  //  DATA MEMBERS: 

  // Declaration of the timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Declaration of the base publisher that will send a message of type string using a shared ptr
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; 

  // Delclaration of a counter
  size_t count_; 
};




// Main function where the node actually executes
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // To initialize ROS 2
  
  // To start processing data from the node "MinimalPublisher" including callbacks from the timer
  rclcpp::spin(std::make_shared<MinimalPublisher>()); 
  
  rclcpp::shutdown();
  return 0;
}

/* 
OBSERVATION AND COMMENTS:

  - Is important giving a name to each node in ros (for debugging monitoring and univoque identification purposes),
   so exploiting the initialization list of members of a class constructor.

*/