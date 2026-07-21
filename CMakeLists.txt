cmake_minimum_required(VERSION 3.8)
project(robot_2_gps_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install YOLO model file
install(FILES
  best.pt
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  robot_2_gps_description/autonomous_mission_controller.py
  robot_2_gps_description/autonomous_mission_test.py
  robot_2_gps_description/gps_logger.py
  robot_2_gps_description/height.py
  robot_2_gps_description/input_processor1.py
  robot_2_gps_description/joint_state_aggregator.py
  robot_2_gps_description/mode.py
  robot_2_gps_description/odom_calculator.py
  robot_2_gps_description/rscp_client.py
  robot_2_gps_description/vision_perception.py
  robot_2_gps_description/rscp_mock_server.py
  test_mission.py
  test_stage_1.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
