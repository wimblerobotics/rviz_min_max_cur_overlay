cmake_minimum_required(VERSION 3.8)
project(rviz_min_max_cur_overlay)

# Set policy for Qt MOC
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

# Enable verbose output during build
set(CMAKE_VERBOSE_MAKEFILE ON)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt setup
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find packages
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OGRE REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

# Define message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MinMaxCurr.msg"
  DEPENDENCIES std_msgs
)

# Get typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Create a shared library
add_library(overlay_display SHARED
  src/overlay_display.cpp
  include/rviz_min_max_cur_overlay/overlay_display.hpp
)

# Set library properties
set_target_properties(overlay_display PROPERTIES
  OUTPUT_NAME "overlay_display"
  PREFIX "lib"
  LINKER_LANGUAGE CXX
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
)

# Include directories
target_include_directories(overlay_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OGRE_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Add component dependencies
ament_target_dependencies(overlay_display
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
  std_msgs
)

# Link libraries
target_link_libraries(overlay_display
  Qt5::Widgets
  ${OGRE_LIBRARIES}
  "${cpp_typesupport_target}"
)

# Ensure message generation completes before building the plugin
add_dependencies(overlay_display ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Installation - Fix: Remove duplicate installation of overlay_display target
install(TARGETS overlay_display
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install plugin_description.xml - Fix: Remove duplicate installation
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(overlay_display)
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
  std_msgs
  rosidl_default_runtime
)

ament_package()