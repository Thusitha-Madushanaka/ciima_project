cmake_minimum_required(VERSION 3.8)

project(custom_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(custom_controller_kinematics REQUIRED)


# ============================================================
# Controller library
# ============================================================

add_library(custom_controller
  SHARED
  src/custom_controller.cpp
)

target_include_directories(custom_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(custom_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  tf2_ros
  custom_controller_kinematics
)


# ============================================================
# Pluginlib
# ============================================================

pluginlib_export_plugin_description_file(
  controller_interface
  custom_controller.xml
)


# ============================================================
# Install
# ============================================================

install(
  TARGETS custom_controller
  EXPORT export_custom_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES custom_controller.xml
  DESTINATION share/${PROJECT_NAME}
)


# ============================================================
# Tests
# ============================================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()


# ============================================================
# Export
# ============================================================

ament_export_targets(
  export_custom_controller
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()