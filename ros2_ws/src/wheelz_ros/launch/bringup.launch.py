from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    mock = LaunchConfiguration('mock').perform(context).lower()
    if mock not in ('true', 'false'):
        raise ValueError('mock must be true or false')
    mock = mock == 'true'
    url = LaunchConfiguration('robot_url').perform(context)
    camera_url = LaunchConfiguration('camera_url').perform(context)
    config = LaunchConfiguration('config').perform(context)
    if not mock and not url:
        raise ValueError('Real hardware requires robot_url:=http://ESP8266_IP')
    parameters = [config, {'robot_url': 'http://127.0.0.1:8080' if mock else url}]
    actions = []
    if mock:
        actions.append(ExecuteProcess(cmd=['ros2', 'run', 'wheelz_ros', 'mock_esp'],
                                      output='screen'))
        # These values describe the emulator only, never the real robot.
        parameters.append({'ticks_per_revolution': 360.0, 'left_encoder_sign': 1,
                           'right_encoder_sign': 1, 'wheel_diameter_m': 0.043,
                           'track_width_m': 0.16632})
        if not camera_url:
            camera_url = 'http://127.0.0.1:8080/stream'
    actions.append(Node(package='wheelz_ros', executable='driver',
                        name='wheelz_driver', parameters=parameters, output='screen'))
    if camera_url:
        actions.append(Node(package='wheelz_ros', executable='camera',
                            name='wheelz_camera', parameters=[{'camera_url': camera_url}],
                            output='screen'))
    return actions


def generate_launch_description():
    config = str(Path(get_package_share_directory('wheelz_ros')) / 'config' / 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='true',
                              description='Use loopback emulator; no physical motor commands'),
        DeclareLaunchArgument('robot_url', default_value='',
                              description='ESP8266 base URL, required when mock is false'),
        DeclareLaunchArgument('camera_url', default_value='',
                              description='ESP32 MJPEG URL; empty disables camera on real hardware'),
        DeclareLaunchArgument('config', default_value=config),
        OpaqueFunction(function=start),
    ])
