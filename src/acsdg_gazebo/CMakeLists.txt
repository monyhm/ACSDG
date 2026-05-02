cmake_minimum_required(VERSION 3.8)
project(acsdg_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds/   DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/   DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/spawn_px4_sitl.sh
  DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS scripts/enemy_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME enemy_driver_node)
install(PROGRAMS scripts/gz_bridge_shim.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gz_bridge_shim)

ament_package()
