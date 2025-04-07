cmake_minimum_required(VERSION 3.8)
project(rviz_min_max_cur_overlay)

# Set policy for Qt MOC
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

# Enable verbose output during build
set(CMAKE_VERBOSE_MAKEFILE ON)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt setup - Remove the problematic AUTOMOC_MOC_OPTIONS
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

# Set library properties - Removed the AUTOMOC_MOC_OPTIONS property
set_target_properties(overlay_display PROPERTIES
  OUTPUT_NAME "overlay_display"
  PREFIX "lib"
  LINKER_LANGUAGE CXX
)

# Include directories
target_include_directories(overlay_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OGRE_INCLUDE_DIRS}
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
add_dependencies(overlay_display ${PROJECT_NAME})

# Installation
install(TARGETS overlay_display
  EXPORT export_overlay_display
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Export dependencies
ament_export_targets(export_overlay_display HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
  std_msgs
  Qt5
)

ament_package()