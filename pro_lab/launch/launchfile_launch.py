#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import launch.logging

logger = launch.logging.get_logger('launchfile_launch')

def generate_launch_description():
    set_turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    
    logger.info("\n \n setting TURTLEBOT3_MODEL to 'burger' worked \n")

    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'headless': 'false'
        }.items(),
    )
    
    logger.info("\n \n turtlebot3_gazebo_launch included successfully \n")

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items(),
    )
    
    logger.info("\n \n slam_toolbox_launch included successfully \n")

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'],
        output='screen'
    )
    
    logger.info("\n \n rviz2_node created successfully \n")

    kf_node = Node(
        package='pro_lab',
        executable='kf_node',
        name='kf_instance',
        output='screen'
    )
    
    logger.info("\n \n kf_node created successfully \n")

    tf_broadcaster_node = Node(
        package='pro_lab',
        executable='tf_broadcaster_node',
        name='tf_broadcaster_instance',
        output='screen',
    )

    logger.info("\n \n tf_broadcaster created successfully \n")


    tf_node = Node(
        package='pro_lab',
        executable='map_odom_tf_broadcaster_node',
        name='map_odom_tf_broadcaster_instance',
        output='screen',
    )

    return LaunchDescription([
        set_turtlebot3_model,
        turtlebot3_gazebo_launch,
        slam_toolbox_launch,
        rviz2_node,
        kf_node,
        tf_node
    ])