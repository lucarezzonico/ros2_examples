#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

using std::placeholders::_1;

class AddressBookSubscriber : public rclcpp::Node
{
public:
  AddressBookSubscriber()
  : Node("address_book_subscriber")
  {
    address_book_subscriber_ =
    this->create_subscription<more_interfaces::msg::AddressBook>(
      "address_book", 10, std::bind(&AddressBookSubscriber::topic_callback, this, _1));
  }

private:
  // void topic_callback(const more_interfaces::msg::AddressBook & msg) const
  // {
  //   RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.first_name << "' '"
  //                                                       << msg.last_name << "' '"
  //                                                       << msg.phone_number << "' '"
  //                                                       << std::to_string(msg.phone_type) << "'");
  // }
  void topic_callback(const more_interfaces::msg::AddressBook & msg) const
  {
    for (auto contact : msg.address_book) {
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << contact.first_name << "' '"
                                                            << contact.last_name << "' '"
                                                            << contact.phone_number << "' '"
                                                            << std::to_string(contact.phone_type) << "'");
    }
  }
  rclcpp::Subscription<more_interfaces::msg::AddressBook>::SharedPtr address_book_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookSubscriber>());
  rclcpp::shutdown();
  return 0;
}
