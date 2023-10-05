cmake_minimum_required(VERSION 3.5)

project(rosidl_tutorials)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_tutorials_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

set(msg_files
  "msg/AddressBook.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES rosidl_tutorials_msgs
)

add_executable(publish_address_book 
  src/publish_address_book.cpp
)

add_executable(publish_contact 
  src/publish_contact.cpp
)

add_executable(srv_server
  src/srv_server.cpp
)

add_executable(srv_client
  src/srv_client.cpp
)

ament_target_dependencies(publish_address_book
  "rclcpp"
)

ament_target_dependencies(publish_contact
  "rclcpp"
  "rosidl_tutorials_msgs"
)

ament_target_dependencies(srv_server
  "rclcpp"
  "rosidl_tutorials_msgs"
)

ament_target_dependencies(srv_client
  "rclcpp"
  "rosidl_tutorials_msgs"
)

ament_export_dependencies(rosidl_default_runtime)

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(publish_address_book
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()


install(TARGETS publish_address_book
    DESTINATION lib/${PROJECT_NAME}
    )

install(TARGETS publish_contact
    DESTINATION lib/${PROJECT_NAME}
	)

install(TARGETS srv_server
    DESTINATION lib/${PROJECT_NAME}
    )

install(TARGETS srv_client
    DESTINATION lib/${PROJECT_NAME}
    )

ament_package()
