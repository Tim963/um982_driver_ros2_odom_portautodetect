#!/usr/bin/env python3
"""
Launch file for UM982 GNSS Driver with Odometry and TF
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Serial port
        DeclareLaunchArgument(
            'serial_port',
            default_value='auto',
            description='Serial port for UM982 (auto for auto-detection)'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate'
        ),
        
        # Frame IDs
        DeclareLaunchArgument(
            'frame_id',
            default_value='gnss_link',
            description='Frame ID for GNSS messages'
        ),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom',
            description='Odometry frame ID'
        ),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='base_link',
            description='Base link frame ID'
        ),
        
        # NTRIP settings
        DeclareLaunchArgument(
            'ntrip_enabled',
            default_value='true',
            description='Enable NTRIP client'
        ),
        DeclareLaunchArgument(
            'ntrip_host',
            default_value='rtk2go.com',
            description='NTRIP caster host'
        ),
        DeclareLaunchArgument(
            'ntrip_port',
            default_value='2101',
            description='NTRIP caster port'
        ),
        DeclareLaunchArgument(
            'ntrip_mountpoint',
            default_value='DEU00WOLF0',
            description='NTRIP mountpoint'
        ),
        DeclareLaunchArgument(
            'ntrip_username',
            default_value='',
            description='NTRIP username (email for rtk2go)'
        ),
        DeclareLaunchArgument(
            'ntrip_password',
            default_value='none',
            description='NTRIP password'
        ),
        
        # Heading settings
        DeclareLaunchArgument(
            'heading_offset',
            default_value='0.0',
            description='Heading offset in degrees'
        ),
        
        # Output settings
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish TF odom->base_link'
        ),
        DeclareLaunchArgument(
            'publish_odom',
            default_value='true',
            description='Publish odometry topic'
        ),
        
        # Node
        Node(
            package='um982_driver',
            executable='um982_node',
            name='um982_driver',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'ntrip_enabled': LaunchConfiguration('ntrip_enabled'),
                'ntrip_host': LaunchConfiguration('ntrip_host'),
                'ntrip_port': LaunchConfiguration('ntrip_port'),
                'ntrip_mountpoint': LaunchConfiguration('ntrip_mountpoint'),
                'ntrip_username': LaunchConfiguration('ntrip_username'),
                'ntrip_password': LaunchConfiguration('ntrip_password'),
                'heading_offset': LaunchConfiguration('heading_offset'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'publish_odom': LaunchConfiguration('publish_odom'),
            }]
        ),
    ])
