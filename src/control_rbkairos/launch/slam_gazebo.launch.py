#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('control_rbkairos')
    robotnik_desc_path = get_package_share_directory('robotnik_description')
    robotnik_sensors_path = get_package_share_directory('robotnik_sensors')
    
    # Set graphics environment variables for WSL (software rendering)
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'
    os.environ['GALLIUM_DRIVER'] = 'llvmpipe'
    
    # Set DISPLAY for WSL GUI support (auto-detect WSLg or fallback)
    if 'DISPLAY' not in os.environ:
        # Try WSLg first (Windows 11)
        if os.path.exists('/mnt/wslg'):
            os.environ['DISPLAY'] = ':0'
            os.environ['WAYLAND_DISPLAY'] = 'wayland-0'
        else:
            # Fallback to X11 forwarding
            os.environ['DISPLAY'] = ':0'
    
    # Set Gazebo resource path to find mesh files using model:// URI
    models_path = os.path.join(robotnik_desc_path, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path + ':' + os.environ['GZ_SIM_RESOURCE_PATH']
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path
    
    # File paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'rbkairos_real_with_mesh.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'table.world')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    
    # Read URDF content and replace file:// paths with package:// URIs
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    
    # Replace absolute file:// paths with package:// URIs for Gazebo
    urdf_content = urdf_content.replace(
        'file:///home/tuwanon/week6_ws/install/robotnik_description/share/robotnik_description/',
        'package://robotnik_description/'
    )
    urdf_content = urdf_content.replace(
        'file:///home/tuwanon/week6_ws/install/robotnik_sensors/share/robotnik_sensors/',
        'package://robotnik_sensors/'
    )
    
    robot_description = urdf_content
    
    print("\n" + "=" * 60)
    print("🗺️  RBKAIROS - SLAM Mapping with SLAM Toolbox")
    print("=" * 60)
    print("🌍 World: table.world")
    print("🤖 Robot: RBKairos with laser scanner")
    print("📡 Laser: /scan topic (180° front laser)")
    print("🗺️  SLAM: Online async SLAM")
    print("🎮 Control: Use teleop to drive robot and build map")
    print("=" * 60 + "\n")
    
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Start Gazebo with GUI
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )
    
    # Spawn robot - use string directly instead of temp file
    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', urdf_content,
                '-name', 'rbkairos',
                '-z', '0.15'
            ],
            output='screen'
        )]
    )
    
    # Robot State Publisher - always running
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen',
        respawn=True
    )
    
    # Spawn Joint State Broadcaster - optional, may not exist
    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
            output='screen',
            on_exit=lambda event, context: None  # Don't stop if fails
        )]
    )
    
    # Spawn Mecanum Drive Controller with cmd_vel remapping
    mecanum_controller_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'mecanum_drive_controller',
                '--param-file',
                os.path.join(pkg_share, 'config', 'mecanum_controller.yaml'),
                '--controller-manager-timeout', '30'
            ],
            remappings=[('/mecanum_drive_controller/cmd_vel', '/cmd_vel')],
            output='screen',
            on_exit=lambda event, context: None  # Don't stop if fails
        )]
    )
    
    # Laser bridge (Gazebo -> ROS2)
    laser_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='laser_bridge',
        arguments=['/front_laser_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/front_laser_lidar/scan', '/scan')],
        output='screen',
        respawn=True
    )
    
    # TF remapping nodes - remap rbkairos/* frames to standard names
    tf_remap_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_remap',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'rbkairos/odom']
    )
    
    # SLAM Toolbox (async online SLAM) - optional
    slam_toolbox_node = TimerAction(
        period=12.0,
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            respawn=False
        )]
    )
    
    # RViz2 - Visualization
    rviz_config = os.path.join(pkg_share, 'config', 'slam_rviz.rviz')
    rviz_node = TimerAction(
        period=15.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            respawn=False
        )]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        gazebo_launch,
        robot_state_pub,
        laser_bridge_node,
        tf_remap_base,
        spawn_robot,
        joint_state_broadcaster_spawner,
        mecanum_controller_spawner,
        slam_toolbox_node,
        rviz_node,
    ])
