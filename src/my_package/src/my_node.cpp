#include <cstdio>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world my_package package\n");
  return 0;
}


// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("my_node");
//     auto voice = node->create_publisher<std_msgs::msg::String>("voice", 10);
//     rclcpp::Rate loop_rate(10);

//     while (rclcpp::ok())
//     {
//         std_msgs::msg::String msg;
//         msg.data = "hello from chatter node!";
//         voice->publish(msg);
//         rclcpp::spin_some(node);
//         loop_rate.sleep();
//     }

//     rclcpp::shutdown();
//     return 0;
// }
