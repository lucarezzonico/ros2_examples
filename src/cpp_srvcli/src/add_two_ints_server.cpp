

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
               std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
              request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]",
              (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create a node named "add_two_ints_server"
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  // Creates a service named "add_two_ints" for that node
  // and automatically advertises it over the networks with the &add method
  // (ensure that this service is discoverable and accessible to other nodes on the ROS 2 network)
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
  
  // Spin the node, making the service available.
  rclcpp::spin(node);
  rclcpp::shutdown();
}

// chechout other service and client examples at:
// https://github.com/ros2/examples/tree/humble/rclcpp/services
