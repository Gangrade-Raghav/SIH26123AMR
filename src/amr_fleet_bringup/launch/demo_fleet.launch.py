"""
Demonstration launcher for NRDAS AMR Fleet Simulation.

Launches the polished warehouse simulation in Gazebo Harmonic with 3D GUI
centered on the operational floor, spawns a 5-robot fleet, and optionally
executes the coordinated demonstration trajectory sequence.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_amr_bringup = get_package_share_directory('amr_fleet_bringup')
    fleet_launch_path = os.path.join(pkg_amr_bringup, 'launch', 'fleet.launch.py')

    declared_arguments = [
        DeclareLaunchArgument(
            'robot_count',
            default_value='5',
            description='Number of AMRs in presentation demo (default: 5)',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (default: false for visual demo)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 alongside Gazebo simulation',
        ),
        DeclareLaunchArgument(
            'auto_start_demo',
            default_value='false',
            description='Automatically start coordinated demo trajectory sequence',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            description='Continuously loop demo trajectory missions',
        ),
    ]

    # Include core fleet launch
    fleet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fleet_launch_path),
        launch_arguments={
            'robot_count': LaunchConfiguration('robot_count'),
            'headless': LaunchConfiguration('headless'),
            'rviz': LaunchConfiguration('rviz'),
            'world': 'warehouse_small',
            'use_sim_time': 'true',
        }.items(),
    )

    # Optional auto-start trajectory coordinator
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    demo_script = os.path.join(workspace_root, 'scripts', 'run_fleet_demo.py')

    demo_process = ExecuteProcess(
        cmd=[
            'python3',
            demo_script,
            '--robot-count',
            LaunchConfiguration('robot_count'),
            '--loop',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_start_demo')),
    )

    return LaunchDescription(declared_arguments + [fleet_launch, demo_process])
