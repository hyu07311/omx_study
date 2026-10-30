#!/usr/bin/env python3
# move_group 만 실행 (robot_state_publisher, ros2_control 은 omy_bringup 쪽에서 띄움)
# kinematics:=kdl|lma|trac_ik 로 IK 솔버 선택 (config/kinematics[_<이름>].yaml)
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
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            config.to_dict(),
            {
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'monitor_dynamics': False,
            },
        ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'kinematics', default_value='kdl', choices=list(KINEMATICS_FILES),
            description='IK solver plugin for group arm'),
        OpaqueFunction(function=launch_setup),
    ])
