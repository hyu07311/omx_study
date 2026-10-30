#!/usr/bin/env python3
# 7주차 MoveIt 실습용: omy_bringup rviz_mock (mock ros2_control + 컨트롤러 + 가짜 D405 장면)
# + move_group + MotionPlanning RViz
#   ros2 launch omy_moveit_config demo_mock.launch.py [kinematics:=kdl|lma|trac_ik] [start_rviz:=false] [camera:=true]
#   camera 기본 false - MoveIt 실습에는 카메라 영상이 필요 없고 렌더링이 CPU 를 많이 써서 끔 (테이블·큐브 마커는 그대로)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, arguments, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=arguments.items(),
        condition=condition)


def generate_launch_description():
    kinematics = LaunchConfiguration('kinematics')
    return LaunchDescription([
        DeclareLaunchArgument('kinematics', default_value='kdl',
                              description='kdl | lma | trac_ik'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Render fake D405 images (CPU heavy)'),

        # MotionPlanning RViz 는 rviz_mock 보다 먼저 포함해야 함
        #   Humble 의 include 는 넘긴 인자를 바깥에도 설정해서, 아래 rviz_mock 에 넘기는 start_rviz:=false 가
        #   이 파일의 start_rviz 를 덮어씀 → 뒤에 두면 RViz 가 안 뜸
        #   (GroupAction 으로 막으면 rviz_mock 이 나중에 그리퍼 spawner 를 띄울 때 variant 를 못 찾아 launch 가 죽음)
        include('omy_moveit_config', 'moveit_rviz.launch.py', {'kinematics': kinematics},
                condition=IfCondition(LaunchConfiguration('start_rviz'))),
        include('omy_moveit_config', 'move_group.launch.py', {'kinematics': kinematics}),
        # 기존 RViz(omy_mock.rviz) 대신 위의 MotionPlanning RViz 를 씀
        include('omy_bringup', 'rviz_mock.launch.py',
                {'variant': 'f3m', 'start_rviz': 'false', 'camera': LaunchConfiguration('camera')}),
    ])
