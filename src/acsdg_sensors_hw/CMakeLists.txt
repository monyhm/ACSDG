cmake_minimum_required(VERSION 3.8)
project(acsdg_sensors_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(acsdg_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Gazebo / gz-sim dependencies (Harmonic = gz-sim8) ───────────────────────
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

set(ros_deps
  rclcpp
  acsdg_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  builtin_interfaces
)

# ── Shared includes ──────────────────────────────────────────────────────────
include_directories(include)

# Gazebo versioned include directories (e.g. /usr/include/gz/sim8)
include_directories(
  ${gz-sim8_INCLUDE_DIRS}
  ${gz-transport13_INCLUDE_DIRS}
  ${gz-msgs10_INCLUDE_DIRS}
  ${gz-plugin2_INCLUDE_DIRS}
)

# ════════════════════════════════════════════════════════════════════════════
# GAZEBO PLUGINS  (MODULE libraries — loaded at runtime by gz-sim)
# ════════════════════════════════════════════════════════════════════════════

# ── EchoGuard Radar Plugin ───────────────────────────────────────────────────
add_library(echoguard_radar_plugin MODULE
  plugins/echoguard_radar_plugin.cpp)

target_link_libraries(echoguard_radar_plugin PUBLIC
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  gz-plugin${GZ_PLUGIN_VER}::register)

target_include_directories(echoguard_radar_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── RF360 Plugin ─────────────────────────────────────────────────────────────
add_library(rf360_plugin MODULE
  plugins/rf360_plugin.cpp)

target_link_libraries(rf360_plugin PUBLIC
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  gz-plugin${GZ_PLUGIN_VER}::register)

target_include_directories(rf360_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ════════════════════════════════════════════════════════════════════════════
# ROS2 EXECUTABLE NODES
# ════════════════════════════════════════════════════════════════════════════

# ── echoguard_node ───────────────────────────────────────────────────────────
add_executable(echoguard_node src/echoguard_node.cpp)
ament_target_dependencies(echoguard_node ${ros_deps})
target_include_directories(echoguard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── rf360_node ───────────────────────────────────────────────────────────────
add_executable(rf360_node src/rf360_node.cpp)
ament_target_dependencies(rf360_node ${ros_deps})
target_include_directories(rf360_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── boson_node ───────────────────────────────────────────────────────────────
add_executable(boson_node src/boson_node.cpp)
ament_target_dependencies(boson_node ${ros_deps})
target_include_directories(boson_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── sensor_fusion_node (updated version) ────────────────────────────────────
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
ament_target_dependencies(sensor_fusion_node ${ros_deps})
target_include_directories(sensor_fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ════════════════════════════════════════════════════════════════════════════
# INSTALL
# ════════════════════════════════════════════════════════════════════════════

install(TARGETS
  echoguard_node
  rf360_node
  boson_node
  sensor_fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  echoguard_radar_plugin
  rf360_plugin
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

ament_package()
