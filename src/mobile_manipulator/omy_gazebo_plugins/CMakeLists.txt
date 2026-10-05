cmake_minimum_required(VERSION 3.5)
project(omy_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

add_library(parallel_finger_plugin SHARED src/parallel_finger_plugin.cpp)
target_compile_features(parallel_finger_plugin PUBLIC cxx_std_17)
ament_target_dependencies(parallel_finger_plugin gazebo_dev)

install(TARGETS parallel_finger_plugin
  LIBRARY DESTINATION lib
)

# Gazebo 가 플러그인을 찾을 수 있게 lib 경로를 GAZEBO_PLUGIN_PATH 에 추가
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
