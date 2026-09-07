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

#====== Load URDF =======#
#      pkg_path = os.path.join(get_package_share_directory('my_odometry'))
#      xacro_file = os.path.join(pkg_path, 'urdf', 'white.xacro')
#      robot_description_config = xacro.process_file(xacro_file)

#      params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    
#      node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[params]
#     )

#      joint_state_node = Node(
#         name="joint_state_publisher",
#         package="joint_state_publisher",
#         executable="joint_state_publisher",
#     )

     #======= Static transform publisher========
     #static_imu_tf_pub = Node(
     #   package='tf2_ros',
     #   executable='static_transform_publisher',
     #   name='static_imu_tf_pub',
     #   arguments=['0.08', '0.0', '0.0',    
     #              '0.0', '0.0', '0.0', '1.0',  
     #              'base_link', 'imu'],  
    #)

     #========= odometry ==========#
     #odometry = Node(
     #   name="my_odometry",
     #   package="odom_serial_pkg",
     #   executable="odom_pub",
    #)
     odometry = Node(
	name="my_odometry",
	package="odom_serial_pkg",
	executable="odometry_node",
    )
     
     #========= twist mux ==========#
     twist_mux_params = os.path.join(get_package_share_directory('odom_serial_pkg'), 'config', 'twist_mux.yaml')
     twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('cmd_vel_out', '/cmd_vel')]
    )

     return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        #DeclareLaunchArgument('use_imu', default_value='true', description='use imu if true'),
        #static_imu_tf_pub,
        #joint_state_node,
        odometry,
        #node_robot_state_publisher,
        twist_mux_node
    ])
