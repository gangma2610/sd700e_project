cmake_minimum_required(VERSION 2.8.3)
project(robot_sd700e_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  combined_robot_hw
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller_manager hardware_interface  combined_robot_hw roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(myrobots_hardware_interfaces 
    src/myrobot_hw.cpp
)
target_link_libraries(myrobots_hardware_interfaces ${catkin_LIBRARIES})

add_executable(combo_control_node
    src/combo_control_node.cpp
)
target_link_libraries(combo_control_node ${catkin_LIBRARIES})