cmake_minimum_required(VERSION 3.8)
project(ros2_uwb_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========================================================================
# Dependencies
# ========================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ros2_uwb_msgs REQUIRED)

set(COMMON_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ros2_uwb_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ========================================================================
# Anchor Manager Node
# ========================================================================
add_executable(anchor_manager_node
  src/anchor_manager_node.cpp
)
target_link_libraries(anchor_manager_node yaml-cpp)
ament_target_dependencies(anchor_manager_node ${COMMON_DEPS} Eigen3)

# ========================================================================
# Trilateration Solver
# ========================================================================
add_executable(uwb_trilateration_solver
  src/uwb_trilateration_solver.cpp
)
target_link_libraries(uwb_trilateration_solver yaml-cpp)
ament_target_dependencies(uwb_trilateration_solver ${COMMON_DEPS} Eigen3)

# ========================================================================
# UWB Visualization Node
# ========================================================================
add_executable(uwb_visualization_node
  src/uwb_visualization_node.cpp
)
ament_target_dependencies(uwb_visualization_node
  rclcpp sensor_msgs visualization_msgs Eigen3
)

# ========================================================================
# UWB Benchmark Node
# ========================================================================
add_executable(uwb_benchmark_node
  src/uwb_benchmark_node.cpp
)
ament_target_dependencies(uwb_benchmark_node ${COMMON_DEPS})

# ========================================================================
# UWB Range Preprocessor
# ========================================================================
add_executable(uwb_range_preprocessor
  src/uwb_range_preprocessor.cpp
)
ament_target_dependencies(uwb_range_preprocessor ${COMMON_DEPS})

# ========================================================================
# Install
# ========================================================================
install(TARGETS
  anchor_manager_node
  uwb_trilateration_solver
  uwb_range_preprocessor
  uwb_benchmark_node
  uwb_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_demo_bag.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ========================================================================
# Testing
# ========================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Gauss-Newton solver tests
  ament_add_gtest(test_trilateration test/test_trilateration.cpp)
  target_include_directories(test_trilateration PRIVATE ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(test_trilateration Eigen3)

  # Anchor manager configuration / transform-building tests
  ament_add_gtest(test_anchor_manager test/test_anchor_manager.cpp)
  target_include_directories(test_anchor_manager PRIVATE ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(test_anchor_manager Eigen3)

  # Covariance propagation and outlier rejection math tests
  ament_add_gtest(test_covariance test/test_covariance.cpp)
  target_include_directories(test_covariance PRIVATE ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(test_covariance Eigen3)
endif()

ament_package()
