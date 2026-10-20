cmake_minimum_required(VERSION 3.8)

project(custom_controller_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Kinematics library
add_library(custom_controller_kinematics
  src/custom_controller_kinematics.cpp
  src/goal_controller.cpp
)

target_include_directories(custom_controller_kinematics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Test executable
add_executable(test_kinematics
  src/test_kinematics.cpp
)

target_link_libraries(test_kinematics
  custom_controller_kinematics
)

# Install library
install(
  TARGETS custom_controller_kinematics
  EXPORT export_custom_controller_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable where ros2 run expects it
install(
  TARGETS test_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(
  export_custom_controller_kinematics
  HAS_LIBRARY_TARGET
)

ament_package()