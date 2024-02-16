cmake_minimum_required(VERSION 3.5)
project(midi_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_dwb_controller REQUIRED)

# Add the following lines to set the CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH "opt/ros/foxy/share")

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
