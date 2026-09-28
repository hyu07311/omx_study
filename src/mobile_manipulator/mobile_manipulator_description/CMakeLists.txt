cmake_minimum_required(VERSION 3.5)
project(mobile_manipulator_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
