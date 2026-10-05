cmake_minimum_required(VERSION 3.5)
project(omy_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/omy_ik_demo.py scripts/omy_pick_demo.py scripts/fake_d405_scene.py scripts/save_d405.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
