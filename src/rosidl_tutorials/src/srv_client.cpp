#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "rosidl_tutorials_msgs/srv/add_two_floats.hpp"

rosidl_tutorials_msgs::srv::AddTwoFloats_Response::SharedPtr send_request(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<rosidl_tutorials_msgs::srv::AddTwoFloats>::SharedPtr client,
  rosidl_tutorials_msgs::srv::AddTwoFloats_Request::SharedPtr request)
{
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return result.get();
  } else {
    return NULL;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_floats_client");

  auto client = node->create_client<rosidl_tutorials_msgs::srv::AddTwoFloats>("add_two_floats");
  auto request = std::make_shared<rosidl_tutorials_msgs::srv::AddTwoFloats::Request>();
  request->a = 2.0;
  request->b = 3.0;

  auto result = send_request(node, client, request);
  if (result) {
    printf("Result of add_two_floats: %f\n", result->sum);
  } else {
    printf("add_two_floats_client was interrupted. Exiting.\n");
  }

  return 0;
}