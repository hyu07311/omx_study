cmake_minimum_required(VERSION 3.5)
project(mobile_manipulator_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/scara_demo.py scripts/ur6_ik_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
