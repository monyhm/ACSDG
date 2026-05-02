cmake_minimum_required(VERSION 3.8)
project(acsdg_c2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(acsdg_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mavros_msgs REQUIRED)

# ── Python package (installs acsdg_c2/ to site-packages for imports) ─────
ament_python_install_package(${PROJECT_NAME})

# ── Python node executables ──────────────────────────────────────────────
foreach(node c2_engine_node interceptor_manager_node mission_manager_node)
  install(PROGRAMS acsdg_c2/${node}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${node})
endforeach()

# ── C++ node: interceptor_controller_node ────────────────────────────────
add_executable(interceptor_controller_node src/interceptor_controller_node.cpp)
ament_target_dependencies(interceptor_controller_node
  rclcpp
  acsdg_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  builtin_interfaces
  mavros_msgs
)

install(TARGETS interceptor_controller_node
  DESTINATION lib/${PROJECT_NAME})

# ── C++ node: coyote_controller_node ─────────────────────────────────────
add_executable(coyote_controller_node src/coyote_controller_node.cpp)
ament_target_dependencies(coyote_controller_node
  rclcpp
  acsdg_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  builtin_interfaces
)

install(TARGETS coyote_controller_node
  DESTINATION lib/${PROJECT_NAME})

# ── Launch files ──────────────────────────────────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
