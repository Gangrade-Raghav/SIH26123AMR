"""Launch description for parameterized multi-robot fleet simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_count',
            default_value='2',
            description='Number of AMRs to spawn in fleet simulation',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo Harmonic server in headless mode',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_small',
            description='Simulation world name',
        ),
    ]

    robot_count = LaunchConfiguration('robot_count')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    log_setup = LogInfo(
        msg=[
            'Starting AMR Fleet Bringup | robots: ', robot_count,
            ' | headless: ', headless,
            ' | use_sim_time: ', use_sim_time,
        ]
    )

    return LaunchDescription(declared_arguments + [log_setup])
