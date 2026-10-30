#!/usr/bin/env python3
# omy_motion_cpp 앱 실행 - move_group 과 같은 MoveIt 설정 (URDF, SRDF, kinematics, OMPL) 을 파라미터로 넘김
#   ros2 launch omy_motion_cpp benchmark.launch.py app:=ik_benchmark|rrt_benchmark|pick_place_pipeline
#     [kinematics:=kdl|lma|trac_ik] [args:='-p trials:=100 ...' 대신 아래 params:=]
#   params:='trials:=100,seed:=3' 처럼 쉼표로 이어서 앱 파라미터 지정
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(get_package_share_directory('omy_moveit_config'), 'launch'))
from omy_moveit_common import moveit_config  # noqa: E402


def parse_value(text):
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    if text.lower() in ('true', 'false'):
        return text.lower() == 'true'
    if text.startswith('[') and text.endswith(']'):
        return [parse_value(v.strip()) for v in text[1:-1].split(';') if v.strip()]
    return text


def launch_setup(context):
    config = moveit_config(LaunchConfiguration('kinematics').perform(context))
    extra = {}
    for item in LaunchConfiguration('params').perform(context).split(','):
        if ':=' in item:
            k, v = item.split(':=', 1)
            extra[k.strip()] = parse_value(v.strip())
    return [Node(
        package='omy_motion_cpp',
        executable=LaunchConfiguration('app').perform(context),
        output='screen',
        emulate_tty=True,
        parameters=[config.to_dict(), extra],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('app', default_value='ik_benchmark'),
        DeclareLaunchArgument('kinematics', default_value='kdl'),
        DeclareLaunchArgument('params', default_value='',
                              description="app parameters, e.g. 'trials:=100,seed:=1' (lists: [a;b;c])"),
        OpaqueFunction(function=launch_setup),
    ])
