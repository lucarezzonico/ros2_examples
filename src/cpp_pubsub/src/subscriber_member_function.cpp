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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  // The node is named minimal_subscriber, and the constructor uses
  // the node’s create_subscription method to execute the callback.
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    // Topic name "topic" and message type <std_msgs::msg::String>
    // used by the subscriber must match the one off the publisher
    // to allow them to communicate.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    // There is no timer because the subscriber simply responds
    // whenever data is published to the topic "topic".
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Spins the MinimalSubscriber node to listening for incoming messages.
  // When a message arrives on the topic "topic",
  // it will trigger the callback function topic_callback
  // to process the message.
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  // For the publisher node, spinning meant starting the timer,
  // but for the subscriber it simply means preparing to receive messages
  // whenever they come.
  rclcpp::shutdown();
  return 0;
}

// chechout other publisher and subscriber examples at:
// https://github.com/ros2/examples/tree/humble/rclcpp/topics
// wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp