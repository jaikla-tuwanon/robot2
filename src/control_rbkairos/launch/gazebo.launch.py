#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('control_rbkairos')
    robotnik_desc_path = get_package_share_directory('robotnik_description')
    
    # Environment for WSL graphics
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    
    # Set Gazebo resource path to find mesh files using model:// URI
    models_path = os.path.join(robotnik_desc_path, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ':' + os.environ['GZ_SIM_RESOURCE_PATH']
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path
    
    # File paths
    world_file = os.path.join(pkg_share, 'worlds', 'table.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rbkairos_real_with_mesh.urdf')
    
    # Load robot description and replace file:// with package://
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Replace absolute file:// paths with package:// URIs for Gazebo
    robot_description = robot_description.replace(
        'file:///home/tuwanon/week6_ws/install/robotnik_description/share/robotnik_description/',
        'package://robotnik_description/'
    )
    robot_description = robot_description.replace(
        'file:///home/tuwanon/week6_ws/install/robotnik_sensors/share/robotnik_sensors/',
        'package://robotnik_sensors/'
    )
    
    print('\n' + '='*60)
    print('🤖 RBKAIROS - Real Robot with 3D Mesh Model')
    print('='*60)
    print('🌍 World: table.world')
    print('🤖 Robot: 4-wheel differential drive with full 3D mesh')
    print('📡 Topic: /cmd_vel')
    print('='*60 + '\n')

    # Start Gazebo with GUI (running mode with -r flag)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # Spawn robot
    spawn_robot = TimerAction(
        period=2.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', urdf_file, '-name', 'rbkairos', '-z', '0.15'],
            output='screen'
        )]
    )

    # Robot State Publisher
    robot_state_pub = TimerAction(
        period=3.5,
        actions=[Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'
        )]
    )

    # Move Robot - starts after Gazebo plugin is ready (no controllers needed)
    move_robot = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[os.path.join(pkg_share, '..', '..', 'bin', 'move_robot')],
            output='screen'
        )]
    )
    
    return LaunchDescription([
        gazebo_launch,
        spawn_robot,
        robot_state_pub,
        move_robot,
    ])
