cmake_minimum_required(VERSION 3.8)
project(rviz_min_max_cur_overlay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rosidl_default_generators REQUIRED)
find_package(OGRE REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

# Generate custom message first
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MinMaxCurr.msg"
  DEPENDENCIES std_msgs
)

# Get the include directories for the generated messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Create the plugin library
add_library(${PROJECT_NAME}_plugin SHARED
  src/overlay_display.cpp
)

# Ensure proper Qt behavior
target_compile_definitions(${PROJECT_NAME}_plugin PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# Set the build flags
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN 1)

# Add include directories
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OGRE_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  std_msgs
  rviz_ogre_vendor
)

# Link against Qt
target_link_libraries(${PROJECT_NAME}_plugin
  ${QT_LIBRARIES}
  Qt5::Widgets
  ${OGRE_LIBRARIES}
  "${cpp_typesupport_target}"
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugin description
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  rosidl_default_runtime
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()