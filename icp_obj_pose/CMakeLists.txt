cmake_minimum_required(VERSION 3.5)
project(icp_obj_pose)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)


add_executable(icp_obj_pose src/icp_obj_pose.cpp)
target_include_directories(icp_obj_pose 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(icp_obj_pose
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${PCL_LIBRARIES}
)

install(TARGETS icp_obj_pose
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(icp_obj_pose rclcpp sensor_msgs PCL pcl_conversions)

ament_auto_package(INSTALL_TO_SHARE launch)
