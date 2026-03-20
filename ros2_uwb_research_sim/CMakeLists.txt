cmake_minimum_required(VERSION 3.8)
project(ros2_uwb_research_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ros2_uwb_msgs REQUIRED)

# GZ (formerly Ignition) Gazebo - Version Agnostic Find
find_package(gz-sim6 QUIET)
if(NOT gz-sim6_FOUND)
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER 6)
  add_library(gz-sim6::core ALIAS ignition-gazebo6::core)
endif()

find_package(gz-common5 QUIET)
if(NOT gz-common5_FOUND)
  find_package(ignition-common4 REQUIRED)
  add_library(gz-common5::core ALIAS ignition-common4::core)
endif()

find_package(gz-plugin2 QUIET)
if(NOT gz-plugin2_FOUND)
  find_package(ignition-plugin1 REQUIRED)
  add_library(gz-plugin2::core ALIAS ignition-plugin1::core)
endif()

find_package(gz-msgs10 QUIET)
if(NOT gz-msgs10_FOUND)
  find_package(ignition-msgs8 REQUIRED)
  add_library(gz-msgs10::gz-msgs10 ALIAS ignition-msgs8::ignition-msgs8)
endif()

find_package(gz-transport13 QUIET)
if(NOT gz-transport13_FOUND)
  find_package(ignition-transport11 REQUIRED)
  add_library(gz-transport13::gz-transport13 ALIAS ignition-transport11::ignition-transport11)
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Models Library
add_library(uwb_models SHARED
  src/physics_model.cpp
  src/gaussian_noise_model.cpp
  src/nlos_bias_model.cpp
  src/multipath_model.cpp
  src/clock_drift_model.cpp
  src/channel_model.cpp
)
target_include_directories(uwb_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(uwb_models
  Eigen3
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UWBResearchData.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Gazebo Plugin
add_library(UWBPlugin SHARED
  src/uwb_plugin.cpp
)
target_link_libraries(UWBPlugin
  uwb_models
  gz-sim6::core
  gz-common5::core
  gz-plugin2::core
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(UWBPlugin "${cpp_typesupport_target}")
add_dependencies(UWBPlugin ${PROJECT_NAME}__rosidl_typesupport_cpp)

ament_target_dependencies(UWBPlugin
  rclcpp
  sensor_msgs
  geometry_msgs
  ros2_uwb_msgs
)

# Install
install(TARGETS
  uwb_models
  UWBPlugin
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/runner.py
  scripts/researcher_node.py
  scripts/plotter.py
  scripts/benchmark.py
  scripts/position_accuracy_benchmark.py
  scripts/research_autopilot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch examples experiments urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Noise model statistical property tests
  ament_add_gtest(test_noise_models test/test_noise_models.cpp)
  target_link_libraries(test_noise_models uwb_models)
  target_include_directories(test_noise_models PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
  )

  # Channel model edge case tests (zero distance, large range, extreme params)
  ament_add_gtest(test_channel_edge_cases test/test_channel_edge_cases.cpp)
  target_link_libraries(test_channel_edge_cases uwb_models)
  target_include_directories(test_channel_edge_cases PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
  )
endif()

ament_package()
