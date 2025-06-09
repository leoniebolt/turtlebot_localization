cmake_minimum_required(VERSION 3.8)
project(pro_lab)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)

# Executable
add_executable(kf_node src/kf_node.cpp)
add_executable(time_synchronizer src/time_synchronizer.cpp)

# Link libraries
ament_target_dependencies(kf_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)
target_link_libraries(kf_node Eigen3::Eigen)

ament_target_dependencies(time_synchronizer
  rclcpp
  nav_msgs
  sensor_msgs
  message_filters
)
target_link_libraries(time_synchronizer Eigen3::Eigen)


# Install
install(TARGETS
  kf_node
  time_synchronizer
  DESTINATION lib/${pro_lab}
)

ament_package()