cmake_minimum_required(VERSION 3.8)
project(omy_motion_cpp)

if(NOT CMAKE_BUILD_TYPE)
  # 시간 측정이 목적이라 기본을 Release 로
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(urdf REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)

# ---------- 핵심 라이브러리 (ROS 노드 의존 없음) ----------
add_library(omy_motion_core SHARED
  src/kinematics.cpp
  src/collision_checker.cpp
  src/rrt_connect.cpp
  src/cartesian_path.cpp
  src/trajectory.cpp
)
target_include_directories(omy_motion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(omy_motion_core Eigen3 urdf moveit_core geometric_shapes moveit_msgs)

# ---------- 벤치마크 / 파이프라인 ----------
set(APP_DEPS rclcpp rclcpp_action moveit_core moveit_ros_planning moveit_msgs control_msgs
  trajectory_msgs visualization_msgs pluginlib tf2_eigen)
foreach(app ik_benchmark rrt_benchmark pick_place_pipeline)
  add_executable(${app} apps/${app}.cpp)
  target_link_libraries(${app} omy_motion_core)
  ament_target_dependencies(${app} ${APP_DEPS})
  install(TARGETS ${app} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(TARGETS omy_motion_core LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/export_ik_reference.py scripts/plot_results.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics omy_motion_core)
  target_compile_definitions(test_kinematics PRIVATE
    OMY_RESULTS_DIR="$ENV{HOME}/robot_ws/motion_cpp_results/ik")
endif()

ament_package()
