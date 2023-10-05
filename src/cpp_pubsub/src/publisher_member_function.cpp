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
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  // The public constructor names the node minimal_publisher
  // and initializes the private vaviable count_ to 0.
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    // This node (minimal_publisher) publishes messages of type <std_msgs::msg::String>
    // to a topic named "topic" at regular intervals using a timer.
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  // This line initializes the ROS 2 communication system.
  rclcpp::init(argc, argv);
  // This line creates an instance of the MinimalPublisher class
  // and passes it to the rclcpp::spin function.
  // The spin function enters a loop and waits for messages to be published
  // or for other events to occur.
  // It keeps the ROS 2 node running and processing messages until it's shut down.
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  // After the spin function exits (when you stop the program),
  // this line shuts down the ROS 2 node and releases any resources held by it.
  rclcpp::shutdown();
  return 0;
}

// chechout other publisher and subscriber examples at:
// https://github.com/ros2/examples/tree/humble/rclcpp/topics
// wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp