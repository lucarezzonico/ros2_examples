cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_tutorials_msgs REQUIRED)

set(msg_files
  "msg/AddressBook.msg"
)

# set(srv_files
#   "srv/Service1.srv"
#   "srv/Service2.srv"
#    # etc
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # ${srv_files}
  DEPENDENCIES rosidl_tutorials_msgs
)

ament_export_dependencies(rosidl_default_runtime)


add_executable(pub_address_book src/pub_address_book.cpp)
ament_target_dependencies(pub_address_book rclcpp)

add_executable(sub_address_book src/sub_address_book.cpp)
ament_target_dependencies(sub_address_book rclcpp)

install(TARGETS
  pub_address_book
  sub_address_book
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

rosidl_target_interfaces(
  pub_address_book
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(
  sub_address_book
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()