"""Parameterized launch description for multi-AMR fleet simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_amr_bringup = get_package_share_directory('amr_fleet_bringup')

    robot_count_str = context.perform_substitution(LaunchConfiguration('robot_count'))
    try:
        robot_count = int(robot_count_str)
    except ValueError:
        robot_count = 2

    fleet_config_arg = context.perform_substitution(LaunchConfiguration('fleet_config'))
    world_name = context.perform_substitution(LaunchConfiguration('world'))
    headless_str = context.perform_substitution(LaunchConfiguration('headless')).lower()
    headless = headless_str in ('true', '1')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Resolve world path
    world_path = os.path.join(pkg_amr_bringup, 'worlds', f'{world_name}.sdf')
    gz_args = f'-r -s {world_path}' if headless else f'-r {world_path}'

    # Resolve robot configurations
    # Locate project root config directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    config_dir = os.path.join(workspace_root, 'config', 'robots')

    config_path = None
    if fleet_config_arg and os.path.isfile(fleet_config_arg):
        config_path = fleet_config_arg
    else:
        candidate = os.path.join(config_dir, f'fleet_{robot_count}_robots.yaml')
        default_candidate = os.path.join(config_dir, 'fleet_default.yaml')
        if os.path.isfile(candidate):
            config_path = candidate
        elif os.path.isfile(default_candidate):
            config_path = default_candidate

    robot_configs = []
    if config_path and os.path.isfile(config_path):
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        robot_configs = data.get('fleet', {}).get('robots', [])

    # If configuration has fewer robots than requested, generate remaining deterministic poses
    while len(robot_configs) < robot_count:
        idx = len(robot_configs)
        col = idx // 5
        row = idx % 5
        x_coord = 2.0 if col == 0 else 8.0
        y_coord = 2.0 + row * 3.0
        robot_configs.append({
            'id': f'amr_{idx}',
            'x': x_coord,
            'y': y_coord,
            'z': 0.15,
            'yaw': 0.0,
        })

    # Slice to exact robot_count
    robot_configs = robot_configs[:robot_count]

    log_info = LogInfo(
        msg=(
            f'[FLEET] Spawning {robot_count} robots into world "{world_name}" '
            f'(headless={headless})'
        )
    )

    # Launch Gazebo simulation server
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Global Clock bridge (GZ -> ROS)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Parameterized spawn for each robot
    spawn_actions = []
    spawn_launch_path = os.path.join(pkg_amr_bringup, 'launch', 'spawn_robot.launch.py')

    for r in robot_configs:
        spawn_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch_path),
                launch_arguments={
                    'robot_name': str(r['id']),
                    'x': str(r['x']),
                    'y': str(r['y']),
                    'z': str(r.get('z', 0.15)),
                    'yaw': str(r.get('yaw', 0.0)),
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        )

    # Optional RViz
    rviz_config = os.path.join(pkg_amr_bringup, 'rviz', 'fleet_default.rviz')
    if not os.path.isfile(rviz_config):
        rviz_config = os.path.join(pkg_amr_bringup, 'rviz', 'single_amr.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [log_info, gz_sim, clock_bridge] + spawn_actions + [rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_count',
            default_value='2',
            description='Number of AMRs to spawn in fleet simulation (1 to 10)',
        ),
        DeclareLaunchArgument(
            'fleet_config',
            default_value='',
            description='Path to custom YAML fleet configuration file (optional)',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_small',
            description='Simulation world name (without .sdf extension)',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo Harmonic in headless server mode',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock for all nodes',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for fleet visualization',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
