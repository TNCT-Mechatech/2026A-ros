"""Tune one injection rack motor: velocity PI, position P, then verify."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_node(context):
    operation = LaunchConfiguration('operation').perform(context)
    if operation not in ('velocity', 'tune', 'verify', 'inspect'):
        raise RuntimeError('operation must be velocity, tune, verify or inspect')
    parameters = {
        'serial_path': LaunchConfiguration('serial_path').perform(context),
        'gain_output_path': LaunchConfiguration('gain_output_path').perform(context),
        'target_motor_index': int(LaunchConfiguration('target_motor_index').perform(context)),
        'measurement_seconds': float(LaunchConfiguration('measurement_seconds').perform(context)),
    }
    if operation == 'inspect':
        parameters['inspect_switches'] = True
    if operation in ('tune', 'verify'):
        parameters['position_operation'] = operation
        for argument, parameter, convert in (
            ('position_max_rps', 'position_measurement_max_rps', float),
            ('position_range_counts', 'position_range_counts', int),
            ('position_travel_limit_counts', 'position_measurement_range_counts', int),
            ('position_response_time_seconds', 'position_response_time_seconds', float),
            ('position_filter_window_seconds', 'position_filter_window_seconds', float),
        ):
            value = LaunchConfiguration(argument).perform(context)
            if value:
                parameters[parameter] = convert(value)
    return [Node(
        package='param_checker',
        executable='param_checker' if operation in ('velocity', 'inspect') else 'position_checker',
        name='param_check', output='screen',
        parameters=[LaunchConfiguration('serial_config').perform(context), parameters],
        remappings=[('controller/joy', LaunchConfiguration('joy_topic').perform(context))],
    )]


def generate_launch_description():
    package = Path(get_package_share_directory('param_checker'))
    ros_home = Path(os.environ.get('ROS_HOME') or str(Path.home() / '.ros'))
    defaults = {
        'operation': ('velocity', 'velocity: identify PI; tune: identify position P; verify: position test; inspect: switch levels, Duty 0'),
        'serial_path': ('/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF505071655067254019-if02',
                        'Injection STM serial port'),
        'serial_config': (str(package / 'config' / 'inject_rack.yaml'), 'Calibrated mechanism settings'),
        'gain_output_path': (str(ros_home / 'natto' / 'inject_pid_gain.yaml'), 'Mechanism-specific gain YAML'),
        'target_motor_index': ('2', 'Motor index 0..3'),
        'measurement_seconds': ('15.0', 'Measurement/test duration'),
        'joy_topic': ('/inject/joy', 'Joystick topic'),
        'position_max_rps': ('', 'tune/verify speed limit in rps; empty uses serial_config'),
        'position_range_counts': ('', 'verify target range +/- raw counts; empty uses serial_config'),
        'position_travel_limit_counts': ('', 'tune/verify travel limit +/- raw counts; empty uses serial_config'),
        'position_response_time_seconds': ('', 'Desired position response time; empty uses serial_config'),
        'position_filter_window_seconds': ('', 'Count averaging window, 0.02..0.5 seconds; empty uses serial_config'),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value, description=description)
          for name, (value, description) in defaults.items()),
        OpaqueFunction(function=make_node),
    ])
