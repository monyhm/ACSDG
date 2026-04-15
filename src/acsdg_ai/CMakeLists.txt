cmake_minimum_required(VERSION 3.8)
project(acsdg_ai)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package (makes `import acsdg_ai` work)
ament_python_install_package(${PROJECT_NAME})

# Install each Python node as a bare executable (no .py extension)
foreach(node
    threat_predictor_node
    swarm_classifier_node
    learning_node)
  install(PROGRAMS acsdg_ai/${node}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${node})
endforeach()

# Install launch files and models placeholder
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

ament_package()
