cmake_minimum_required(VERSION 3.8)
project(midi_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
hardware_interface
rclcpp
rclcpp_lifecycle
rclcpp_components
pluginlib
realtime_tools
midi_custom_interfaces
std_msgs
)
find_package(ament_cmake REQUIRED)

foreach(depends IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${depends} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/hardware_info_node.cpp
  src/midi_hardware.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface midi_hardware.xml)
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)



# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()