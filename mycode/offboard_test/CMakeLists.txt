cmake_minimum_required(VERSION 3.8)
project(offboard_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executables
add_executable(offboard_test_node src/offboard_test_node.cpp)
target_link_libraries(offboard_test_node PUBLIC
  ${geometry_msgs_TARGETS}
  ${px4_msgs_TARGETS}
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

add_executable(vision_bridge_node src/vision_bridge_node.cpp)
target_link_libraries(vision_bridge_node PUBLIC
  ${geometry_msgs_TARGETS}
  ${px4_msgs_TARGETS}
  rclcpp::rclcpp
  tf2_ros::tf2_ros
)

# Install
install(TARGETS
  offboard_test_node
  vision_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
