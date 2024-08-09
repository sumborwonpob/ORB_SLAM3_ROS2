#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="orb_slam3_ros2",
            executable="orb_slam3_ros2",
            name="orb_slam3_ros2",
            output="screen",
            emulate_tty=True,
            parameters=[(
                os.path.join(get_package_share_directory('orb_slam3_ros2'),
	                'config', 'ROS', 'ros_config.yaml'),
            )],
        )
    ])
