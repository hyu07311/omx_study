#!/usr/bin/env python3
# OMY RViz 전용 실행 (Gazebo 없음): robot_state_publisher + ros2_control(mock) + 컨트롤러 + rviz2
# + fake_d405_scene (테이블/큐브 마커, ray casting 으로 만든 D405 영상/깊이/포인트클라우드, 집기 판정)
# 물리는 없지만 Gazebo 와 같은 액션/토픽을 써서 omy_ik_demo.py, omy_pick_demo.py 가 그대로 동작함

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
    variant = LaunchConfiguration('variant')

    controllers_file = PathJoinSubstitution([
        FindPackageShare('omy_bringup'), 'config', 'controllers_mock.yaml'])

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([
                FindPackageShare('omy_description'), 'urdf', 'omy.urdf.xacro']),
            ' use_sim:=false',
            ' variant:=', variant,
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_file],
        output='screen')

    joint_state_broadcaster_spawner = spawner('joint_state_broadcaster')
    # 3m 은 그리퍼가 없으므로 gripper_controller 를 띄우지 않음
    other_spawners = [
        spawner('arm_controller'),
        spawner('gripper_controller',
                condition=IfCondition(PythonExpression(["'", variant, "' == 'f3m'"]))),
    ]

    # 카메라가 달린 f3m 에서만
    fake_camera = Node(
        package='omy_bringup',
        executable='fake_d405_scene.py',
        condition=IfCondition(PythonExpression(["'", variant, "' == 'f3m'"])),
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('omy_description'), 'rviz', 'omy_mock.rviz'])],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'variant', default_value='f3m',
            description='f3m (gripper + D405) or 3m (arm only)'),
        DeclareLaunchArgument('start_rviz', default_value='true', description='Run rviz2'),

        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        fake_camera,
        rviz,

        # joint_state_broadcaster → 나머지 컨트롤러 순서로 로드
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=other_spawners)),
    ])
