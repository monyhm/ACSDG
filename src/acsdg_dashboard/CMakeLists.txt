cmake_minimum_required(VERSION 3.8)
project(acsdg_dashboard)

find_package(ament_cmake REQUIRED)

# Install the built React app (web/build/) into share
install(DIRECTORY web/build/
  DESTINATION share/${PROJECT_NAME}/web
  OPTIONAL)

# Install the Python HTTP server node
install(PROGRAMS scripts/dashboard_server_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dashboard_server_node)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
