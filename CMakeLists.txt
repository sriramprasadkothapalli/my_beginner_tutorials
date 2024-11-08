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
add_executable(talker src/talker.cpp)

ament_target_dependencies(talker rclcpp std_msgs example_interfaces)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
