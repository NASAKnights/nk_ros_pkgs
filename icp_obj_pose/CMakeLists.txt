cmake_minimum_required(VERSION 3.5)
project(icp_obj_pose)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)  # OpenCV

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Executable
add_executable(icp_obj_pose src/icp_obj_pose.cpp)

# Link libraries
target_link_libraries(icp_obj_pose
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# Specify dependencies
ament_target_dependencies(icp_obj_pose
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
  OpenCV
)

# Install executable
install(TARGETS icp_obj_pose
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources, if necessary
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Package
ament_package()
