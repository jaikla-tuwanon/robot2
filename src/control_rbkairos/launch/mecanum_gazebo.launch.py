#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('control_rbkairos')
    robotnik_desc_share = get_package_share_directory('robotnik_description')
    robotnik_sensors_share = get_package_share_directory('robotnik_sensors')
    
    # Environment for WSL graphics
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    
    # Set Gazebo resource paths so it can find package:// URIs
    # Point to install folder so Gazebo can find robotnik_description and robotnik_sensors
    install_dir = os.path.join(pkg_share, '..', '..')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    # Add individual package share directories
    gz_resource_paths = [
        install_dir,  # install/ folder containing all packages
    ]
    
    # Also set ROS_PACKAGE_PATH for compatibility
    ros_package_path = os.environ.get('ROS_PACKAGE_PATH', '')
    ros_paths = [
        install_dir,
        os.path.join(install_dir, 'robotnik_description', 'share'),
        os.path.join(install_dir, 'robotnik_sensors', 'share'),
    ]
    os.environ['ROS_PACKAGE_PATH'] = ':'.join(ros_paths + ([ros_package_path] if ros_package_path else []))
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join(gz_resource_paths + ([gz_resource_path] if gz_resource_path else []))
    
    # File paths
    world_file = os.path.join(pkg_share, 'worlds', 'table.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rbkairos_real_with_mesh.urdf')
    
    # Controller config using PathJoinSubstitution (resolves at runtime)
    controller_config = PathJoinSubstitution([
        FindPackageShare('control_rbkairos'),
        'config',
        'mecanum_controller.yaml'
    ])
    
    # Load robot description
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Replace package:// URIs with absolute file:// paths for Gazebo Ignition
    robot_description = robot_description.replace(
        'package://robotnik_description/',
        f'file://{robotnik_desc_share}/'
    )
    robot_description = robot_description.replace(
        'package://robotnik_sensors/',
        f'file://{robotnik_sensors_share}/'
    )
    
    # Replace $(find ...) with absolute path for controller config
    controller_yaml_path = os.path.join(pkg_share, 'config', 'mecanum_controller.yaml')
    robot_description = robot_description.replace(
        '$(find control_rbkairos)/config/mecanum_controller.yaml',
        controller_yaml_path
    )
    
    # Write modified URDF to temporary file for spawning
    import tempfile
    temp_urdf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    temp_urdf.write(robot_description)
    temp_urdf.close()
    temp_urdf_path = temp_urdf.name
    
    print('\n' + '='*60)
    print('🤖 RBKAIROS - Mecanum Drive Robot (Assignment 2)')
    print('='*60)
    print('🌍 World: table.world')
    print('🤖 Robot: 4-wheel mecanum drive with full 3D mesh')
    print('📡 Topic: /cmd_vel')
    print('🎮 Movement: Forward, Backward, Left, Right (2m each)')
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
            arguments=['-file', temp_urdf_path, '-name', 'rbkairos', '-z', '0.15'],
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

    # Activate controllers (gz_ros2_control auto-loads them from URDF <parameters> tag)
    joint_state_broadcaster_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--inactive', '--controller-manager-timeout', '30'],
            output='screen'
        )]
    )

    mecanum_controller_spawner = TimerAction(
        period=12.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mecanum_drive_controller', '--inactive', '--controller-manager-timeout', '30'],
            output='screen'
        )]
    )

    # Bridge for the laser scanners
    laser_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='laser_bridge',
        arguments=[
            '/front_laser_lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        remappings=[
            ('/front_laser_lidar/scan', '/scan')
        ],
        output='screen'
    )

    # Static TF: odom -> base_footprint (for SLAM and RViz2)
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen'
    )

    # Note: mecanum_move ต้องรันแยกใน terminal เพื่อรับ keyboard input
    # Run separately: ros2 run control_rbkairos mecanum_move
    
    return LaunchDescription([
        gazebo_launch,
        spawn_robot,
        robot_state_pub,
        # Controllers auto-loaded and activated by gz_ros2_control plugin from URDF <parameters> tag
        laser_bridge_node,
        static_tf_odom,
    ])
