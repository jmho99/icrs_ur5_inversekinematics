#!/usr/bin/env python3

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    topic_ik = LaunchConfiguration(
        'topic_ik',
        default = os.path.join(
            get_package_share_directory('topic_ur5ik'),
            'src',
            'calc_sub.cpp'))

    service_ik = LaunchConfiguration('service_ik')
    action_ik = LaunchConfiguration('action_ik')

    topic_calc = launch_ros.actions.Node(
        package = 'calc_ur5ik_topic',
        namespace = topic_ik,
        executable = 'ur5ik',

    )
    return launch.LaunchDescription([
        topic_calc
    ])