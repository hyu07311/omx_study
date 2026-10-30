#!/usr/bin/env python3
# MotionPlanning 패널이 있는 RViz (move_group 과 같은 kinematics 설정을 넘겨야 손끝 마커 IK 가 맞음)

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from omy_moveit_common import KINEMATICS_FILES, moveit_config  # noqa: E402


def launch_setup(context):
    config = moveit_config(LaunchConfiguration('kinematics').perform(context))
    return [Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', str(config.package_path / 'config' / 'moveit.rviz')],
        parameters=[
            config.robot_description,
            config.robot_description_semantic,
            config.robot_description_kinematics,
            config.planning_pipelines,
            config.joint_limits,
        ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('kinematics', default_value='kdl', choices=list(KINEMATICS_FILES)),
        OpaqueFunction(function=launch_setup),
    ])
