cmake_minimum_required(VERSION 3.8)
project(rviz_min_max_cur_overlay)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt5
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MinMaxCurr.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Create the plugin library
add_library(overlay_display SHARED
  src/overlay_display.cpp
  include/rviz_min_max_cur_overlay/overlay_display.hpp
)

# Set visibility settings
set_target_properties(overlay_display PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
)

# Set include directories
target_include_directories(overlay_display
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Get the typesupport target for the messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Make sure the plugin is built after message generation
add_dependencies(overlay_display ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Link against dependencies
target_link_libraries(overlay_display
  Qt5::Widgets
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
  pluginlib::pluginlib
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Install the library
install(TARGETS overlay_display
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Register the plugin with rviz_common
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Export targets
ament_export_targets(export_${PROJECT_NAME})

# Export libraries
ament_export_libraries(overlay_display)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
  std_msgs
  geometry_msgs
)

ament_package()