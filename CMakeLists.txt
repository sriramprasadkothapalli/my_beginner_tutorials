cmake_minimum_required(VERSION 3.5)
project(beginner_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Catch2 REQUIRED)

# Add Talker Node executable
add_executable(talker src/talker.cpp)

ament_target_dependencies(talker
  rclcpp
  std_msgs
  example_interfaces
  tf2
  tf2_ros
  geometry_msgs
)

# Add Talker Integration Test executable
add_executable(talker_integration_test src/talker_integration_test.cpp)

ament_target_dependencies(talker_integration_test
  rclcpp
  std_msgs
  tf2_ros
  geometry_msgs
  Catch2
)

# Install executables
install(TARGETS
  talker
  talker_integration_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing for Catch2
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_test(NAME TalkerIntegrationTest COMMAND talker_integration_test)
endif()

ament_package()
# Add Listener Node executable
add_executable(listener src/listener.cpp)

ament_target_dependencies(listener
  rclcpp
  std_msgs
)

# Install the Listener executable
install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME}
)