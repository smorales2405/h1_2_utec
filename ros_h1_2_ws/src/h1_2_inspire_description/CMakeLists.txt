cmake_minimum_required(VERSION 3.8)
project(h1_2_inspire_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  meshes
  urdf
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Software coupling of the hands: the node (ros2 run h1_2_inspire_description hand_coupling_node.py)
# and the module it imports, installed side by side.
install(PROGRAMS
  scripts/hand_coupling.py
  scripts/hand_coupling_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
