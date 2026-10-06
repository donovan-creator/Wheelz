"""Start the bridge and the authenticated app gateway (which contains the policy)."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('wheelz_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='true'),
        DeclareLaunchArgument('robot_url', default_value=''),
        DeclareLaunchArgument('config', default_value=str(share / 'config/robot.yaml')),
        DeclareLaunchArgument('api_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('api_port', default_value='8766'),
        DeclareLaunchArgument('token_file', default_value=str(Path.home() / '.config/wheelz/app-token')),
        DeclareLaunchArgument('allowed_origins', default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/bringup.launch.py')),
            launch_arguments={'mock': LaunchConfiguration('mock'),
                              'robot_url': LaunchConfiguration('robot_url'),
                              'config': LaunchConfiguration('config')}.items()),
        Node(package='wheelz_ros', executable='app_gateway', output='screen',
             parameters=[{'api_host': LaunchConfiguration('api_host'),
                          'api_port': ParameterValue(LaunchConfiguration('api_port'), value_type=int),
                          'token_file': LaunchConfiguration('token_file'),
                          'allowed_origins': LaunchConfiguration('allowed_origins')}],
             remappings=[('wheelz/policy_cmd_vel', 'cmd_vel')]),
    ])
