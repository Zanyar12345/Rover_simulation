cmake_minimum_required(VERSION 3.8)
project(robot_2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  PROGRAMS
    robot_2_description/publisher.py
    robot_2_description/input_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    urdf config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
