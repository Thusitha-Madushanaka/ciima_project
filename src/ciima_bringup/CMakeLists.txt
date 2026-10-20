cmake_minimum_required(VERSION 3.8)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/cmd_vel_converter.py
  scripts/follow_parallel.py
  scripts/caltab_tf_pub.py
  scripts/job_manager.py
  scripts/simulate_tf_pub.py
  scripts/front_distance.py
  scripts/converter_collision_msg.py
  scripts/keyboard_twiststamped.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
        launch 
        configs 
        DESTINATION share/${PROJECT_NAME})
ament_package()
