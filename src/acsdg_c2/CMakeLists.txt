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

# ── Python package (installs acsdg_c2/ to site-packages for imports) ─────
ament_python_install_package(${PROJECT_NAME})

# ── Python node executables ──────────────────────────────────────────────
foreach(node c2_engine_node interceptor_manager_node mission_manager_node)
  install(PROGRAMS acsdg_c2/${node}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${node})
endforeach()

# ── Shared base library: weapon_controller_base ──────────────────────────
# Holds the topic plumbing, watchdog, return-to-home, and 20 Hz timer that
# every per-weapon controller (Anvil, Coyote, Phase-3 DroneHunter, …) used
# to duplicate. Subclasses link against this and override two virtuals.
add_library(weapon_controller_base
  src/weapon_controller_base.cpp
)
target_include_directories(weapon_controller_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(weapon_controller_base
  rclcpp acsdg_msgs geometry_msgs nav_msgs std_msgs builtin_interfaces
)

# ── C++ node: interceptor_controller_node (Anvil) ────────────────────────
add_executable(interceptor_controller_node src/interceptor_controller_node.cpp)
target_link_libraries(interceptor_controller_node weapon_controller_base)
ament_target_dependencies(interceptor_controller_node
  rclcpp acsdg_msgs geometry_msgs nav_msgs std_msgs builtin_interfaces
)

# ── C++ node: coyote_controller_node ─────────────────────────────────────
add_executable(coyote_controller_node src/coyote_controller_node.cpp)
target_link_libraries(coyote_controller_node weapon_controller_base)
ament_target_dependencies(coyote_controller_node
  rclcpp acsdg_msgs geometry_msgs nav_msgs std_msgs builtin_interfaces
)

# ── Install ──────────────────────────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS weapon_controller_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS interceptor_controller_node coyote_controller_node
  DESTINATION lib/${PROJECT_NAME})

# ── Launch files ─────────────────────────────────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
