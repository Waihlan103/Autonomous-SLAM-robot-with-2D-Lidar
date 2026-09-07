#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

     use_sim_time = LaunchConfiguration('use_sim_time')

     rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("odom_serial_pkg"), "rviz", "scan.rviz"]
    )

     rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        #condition=IfCondition(use_rviz),
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{
             "use_sim_time": use_sim_time
        }]
    )
     
     return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false', description='use sim time if true'),
            rviz_node,
        ]
    )