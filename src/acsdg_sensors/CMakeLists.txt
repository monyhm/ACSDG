cmake_minimum_required(VERSION 3.8)
project(acsdg_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(acsdg_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(deps
  rclcpp
  acsdg_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  builtin_interfaces
)

# ── Shared include directory ─────────────────────────────────────────────
include_directories(include)

# ── radar_node ───────────────────────────────────────────────────────────
add_executable(radar_node src/radar_node.cpp)
ament_target_dependencies(radar_node ${deps})
target_include_directories(radar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── rf_node ──────────────────────────────────────────────────────────────
add_executable(rf_node src/rf_node.cpp)
ament_target_dependencies(rf_node ${deps})
target_include_directories(rf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── sensor_fusion_node ───────────────────────────────────────────────────
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
ament_target_dependencies(sensor_fusion_node ${deps})
target_include_directories(sensor_fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── Install ──────────────────────────────────────────────────────────────
install(TARGETS
  radar_node
  rf_node
  sensor_fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
