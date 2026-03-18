cmake_minimum_required(VERSION 3.8)
project(ros2_uwb_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UWBRange.msg"
  "msg/UWBMultiRange.msg"
  "msg/UWBErrorDiagnostics.msg"
  DEPENDENCIES std_msgs
)

ament_package()
