cmake_minimum_required(VERSION 3.10)
project(v4l2_camera)

# Enable C++14 or later
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUDEV REQUIRED libudev)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${LIBUDEV_LIBRARIES}
)


add_executable(v4l2_camera_node src/v4l2_camera_node.cpp src/v4l2_calibration.cpp)

ament_target_dependencies(v4l2_camera_node
  rclcpp
  rclcpp_action
  sensor_msgs
  OpenCV
  cv_bridge
  yaml-cpp
  ament_index_cpp
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CalibrateCamera.action"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(v4l2_camera_node yaml-cpp ${cpp_typesupport_target} ${LIBUDEV_LIBRARIES})

install(TARGETS v4l2_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(v4l2_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

endif()

ament_package()
