"""Keep YAML rack targets unless a launch override is explicitly provided."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def make_node(context):
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=bool)
        for name in ('auto_restart', 'inject_pid', 'rack_angle_profile')
    }
    for name, convert in (
        ('rack_target_angle_4_deg', float),
        ('rack_target_angle_5_deg', float),
        ('rack_angle_count_sign', int),
    ):
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = convert(value)
    # The binary creates inject_serial_node and roll_serial_node internally.
    return [Node(
        package='inject2026A', executable='main', output='screen',
        respawn=LaunchConfiguration('auto_restart'), respawn_delay=0.0,
        parameters=[LaunchConfiguration('serial_config').perform(context), parameters],
        remappings=[('inject/joy', LaunchConfiguration('joy_topic').perform(context))],
    )]


def generate_launch_description():
    package = Path(get_package_share_directory('inject2026A'))
    defaults = {
        'auto_restart': ('false', 'Respawn after the process exits'),
        'inject_pid': ('true', 'Rack position PID; false selects Duty control'),
        'rack_angle_profile': ('true', 'Buttons 4/5 move to configured rack angles'),
        'rack_target_angle_4_deg': ('', 'Button 4 angle; empty uses serial_config'),
        'rack_target_angle_5_deg': ('', 'Button 5 angle; empty uses serial_config'),
        'rack_angle_count_sign': ('', 'Raw count direction away from origin: +1 or -1; empty uses serial_config'),
        'serial_config': (str(package / 'config' / 'serial_ports.yaml'), 'Mechanism settings'),
        'joy_topic': ('/inject/joy', 'Joystick topic'),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value, description=description)
          for name, (value, description) in defaults.items()),
        OpaqueFunction(function=make_node),
    ])
