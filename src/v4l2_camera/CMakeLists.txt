cmake_minimum_required(VERSION 3.10)
project(v4l2_camera)

# Enable C++14 or later
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(v4l2_camera_node src/v4l2_camera_node.cpp)
add_executable(calibrate src/calibration.cpp)

ament_target_dependencies(v4l2_camera_node
  rclcpp
  rclcpp_action
  sensor_msgs
  OpenCV
  cv_bridge
  yaml-cpp
)

ament_target_dependencies(calibrate
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
  yaml-cpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CalibrateCamera.action"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(v4l2_camera_node yaml-cpp ${cpp_typesupport_target})
target_link_libraries(calibrate yaml-cpp)

install(TARGETS v4l2_camera_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS calibrate
  DESTINATION lib/${PROJECT_NAME}
)

find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

target_include_directories(v4l2_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
