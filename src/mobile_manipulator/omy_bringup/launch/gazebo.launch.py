#!/usr/bin/env python3
# OMY Gazebo 시뮬레이션: gazebo + robot_state_publisher + spawn_entity + ros2_control 컨트롤러
# URDF 의 world 링크에 fixed joint 로 붙어 있어서 Gazebo 에서도 바닥에 고정됨

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
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def spawner(controller, condition=None):
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller, '--controller-manager', '/controller_manager'],
        condition=condition,
        output='screen')


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    variant = LaunchConfiguration('variant')

    controllers_file = PathJoinSubstitution([
        FindPackageShare('omy_bringup'), 'config', 'controllers.yaml'])

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([
                FindPackageShare('omy_description'), 'urdf', 'omy.urdf.xacro']),
            ' use_sim:=true',
            ' variant:=', variant,
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
            '-entity', 'omy',
            # 첫 실행 시 gzserver 기동이 30초(기본값)를 넘기는 경우가 있음
            '-timeout', '120',
        ],
        output='screen')

    joint_state_broadcaster_spawner = spawner('joint_state_broadcaster')
    # 3m 은 그리퍼가 없으므로 gripper_controller 를 띄우지 않음
    other_spawners = [
        spawner('arm_controller'),
        spawner('gripper_controller',
                condition=IfCondition(PythonExpression(["'", variant, "' == 'f3m'"]))),
    ]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('omy_description'), 'rviz', 'omy.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(start_rviz),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='false', description='Run rviz2'),
        DeclareLaunchArgument('gui', default_value='true', description='Run Gazebo client GUI'),
        DeclareLaunchArgument(
            'variant', default_value='f3m',
            description='f3m (gripper + D405) or 3m (arm only)'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('omy_bringup'), 'worlds', 'table.world']),
            description='Gazebo world file'),

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
