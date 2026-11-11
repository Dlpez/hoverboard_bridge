"""Lanza solo el puente del hoverboard con la config y calibración probadas en MILO (10-11-2026).

    ros2 launch hoverboard_bridge hoverboard_bridge.launch.py hoverboard_port:=/dev/ttyUSB0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare('hoverboard_bridge'), 'config'])
    return LaunchDescription([
        DeclareLaunchArgument('hoverboard_port', default_value='/dev/hoverboard'),
        DeclareLaunchArgument('cmd_vel', default_value='/diff_drive_controller/cmd_vel_unstamped',
                              description='topic Twist de entrada (el mismo que usaba MILO)'),
        Node(
            package='hoverboard_bridge', executable='bridge', name='hoverboard_bridge',
            output='screen',
            parameters=[PathJoinSubstitution([cfg, 'hoverboard_bridge.yaml']),
                        {'port': LaunchConfiguration('hoverboard_port'),
                         'calibration_file': PathJoinSubstitution(
                             [cfg, 'hoverboard_calibracion.yaml'])}],
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel'))],
        ),
    ])
