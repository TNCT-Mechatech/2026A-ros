from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    mode = value('navigation_mode')
    if mode not in ('buttons', 'sequence'):
        raise RuntimeError('navigation_mode must be buttons or sequence')
    if mode == 'sequence':
        return []
    for key in ('launch_behavior', 'launch_control', 'launch_dynamic_auto_control'):
        if value(key).lower() != 'true':
            raise RuntimeError(f'buttons mode requires {key}:=true')
    if value('launch_static_auto_control').lower() == 'true':
        raise RuntimeError('buttons mode requires launch_static_auto_control:=false')
    if value('reverse_y').lower() == 'true':
        raise RuntimeError('buttons mode uses destination files in map coordinates; reverse_y must be false')
    destinations = value('destinations_path')
    bindings = value('goal_buttons_path')
    # Validate both files before starting either node.
    from natto_navigation.config import load_destinations, load_bindings
    load_destinations(destinations)
    load_bindings(bindings, 16)
    return [
        Node(
            package='natto_navigation', executable='joy_goal_selector',
            namespace='navigation', output='screen',
            parameters=[{'bindings_path': bindings, 'enable_button': 16}],
            remappings=[('joy', '/controller/joy'), ('navigation_input', '/navigation/input')],
        ),
        Node(
            package='natto_navigation', executable='navigation_manager',
            namespace='navigation', output='screen',
            parameters=[{'destinations_path': destinations}],
            remappings=[
                ('navigation_input', '/navigation/input'),
                ('navigation_goal', '/navigation/goal'),
                ('navigation_path', '/navigation/path'),
                ('navigation_feedback', '/navigation/feedback'),
                ('navigation_status', '/navigation/status'),
                ('current_pose', '/localization/current_pose'),
                ('allow_auto_drive', '/controller/allow_auto_drive'),
                ('command_velocity', '/control/command_velocity'),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=configure)])
