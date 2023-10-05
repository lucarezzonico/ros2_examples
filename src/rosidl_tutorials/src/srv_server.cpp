#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "rosidl_tutorials_msgs/srv/add_two_floats.hpp"

void handle_add_two_floats(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rosidl_tutorials_msgs::srv::AddTwoFloats::Request> request,
  std::shared_ptr<rosidl_tutorials_msgs::srv::AddTwoFloats::Response> response)
{
  (void)request_header;
  std::cout << "Incoming request" << std::endl;
  std::cout << "a: " << request->a << " b: " << request->b << std::endl;
  response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_floats_server");

  auto server = node->create_service<rosidl_tutorials_msgs::srv::AddTwoFloats>("add_two_floats", handle_add_two_floats);

  rclcpp::spin(node);

  rclcpp::shutdown();



  node = nullptr;

  return 0;
}