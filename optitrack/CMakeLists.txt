cmake_minimum_required(VERSION 3.8)
project(optitrack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/optitrack/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/optitrack/lib)

add_executable(talker src/optitrack_data_publisher.cpp)
target_link_libraries(talker PUBLIC 
  rclcpp::rclcpp 
  ${geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
  NatNet
)

# Set RPATH so talker can find libNatNet.so at runtime
set_target_properties(talker PROPERTIES
  INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/include/optitrack/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

add_executable(talker_with_CSVs src/optitrack_data_publisher_with_CSVs.cpp)
target_link_libraries(talker_with_CSVs PUBLIC 
  rclcpp::rclcpp 
  ${geometry_msgs_TARGETS}
  NatNet
)

# Set RPATH so talker_with_CSVs can find libNatNet.so at runtime
set_target_properties(talker_with_CSVs PROPERTIES
  INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/include/optitrack/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

add_executable(listener_with_stats src/optitrack_subscriber_with_statistics.cpp)
target_link_libraries(listener_with_stats PUBLIC 
  rclcpp::rclcpp 
  ${geometry_msgs_TARGETS}
)

install(TARGETS
	talker
	talker_with_CSVs
	listener_with_stats
	DESTINATION lib/${PROJECT_NAME})
	
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()