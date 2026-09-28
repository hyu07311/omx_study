#!/usr/bin/env python3
# 6축 버전 Gazebo 시뮬레이션: gazebo + robot_state_publisher + spawn_entity + ros2_control 컨트롤러

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def spawner(controller):
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller, '--controller-manager', '/controller_manager'],
        output='screen')


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    controllers_file = PathJoinSubstitution([
        FindPackageShare('mobile_manipulator_bringup'), 'config', 'controllers_ur6.yaml'])

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([
                FindPackageShare('mobile_manipulator_description'),
                'urdf', 'mobile_manipulator_ur6.urdf.xacro']),
            ' use_sim:=true',
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'world': world, 'gui': gui, 'verbose': 'false'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_manipulator_ur6',
            '-x', x_pose, '-y', y_pose, '-z', '0.01', '-Y', yaw,
            # 첫 실행 시 gzserver 기동이 30초(기본값)를 넘기는 경우가 있음
            '-timeout', '120',
        ],
        output='screen')

    joint_state_broadcaster_spawner = spawner('joint_state_broadcaster')
    other_spawners = [spawner(name) for name in
                      ('diff_drive_controller', 'arm_controller', 'gripper_controller')]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('mobile_manipulator_description'), 'rviz', 'display.rviz']),
            '-f', 'odom',
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(start_rviz),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='false', description='Run rviz2'),
        DeclareLaunchArgument('gui', default_value='true', description='Run Gazebo client GUI'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('mobile_manipulator_bringup'), 'worlds', 'empty.world']),
            description='Gazebo world file'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz,

        # 로봇이 스폰된 뒤 → joint_state_broadcaster → 나머지 컨트롤러 순서로 로드
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=other_spawners)),
    ])
