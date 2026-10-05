#!/usr/bin/env python3
# RViz에서 OMY URDF 확인: robot_state_publisher + joint_state_publisher_gui + rviz2

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')
    variant = LaunchConfiguration('variant')

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([
                FindPackageShare('omy_description'), 'urdf', 'omy.urdf.xacro']),
            ' variant:=', variant,
        ]),
        value_type=str)

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('omy_description'), 'rviz', 'omy.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Run joint_state_publisher_gui (sliders)'),
        DeclareLaunchArgument(
            'variant',
            default_value='f3m',
            description='f3m (gripper + D405) or 3m (arm only)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui)),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_gui)),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'),
    ])
