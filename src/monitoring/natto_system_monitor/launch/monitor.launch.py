"""Passive diagnostics and optional allow-listed MCAP recording."""
from datetime import datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        raw = value(name).lower()
        if raw not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return raw == 'true'

    actions = []
    if enabled('monitor'):
        overrides = {'managed_navigation': enabled('managed_navigation'),
                     'graph.monitor_node': '/graph_monitor' if enabled('rosgraph') else ''}
        for argument, keys in {
            'monitor_lidar': ['front.enabled', 'rear.enabled'],
            'monitor_localization': ['localization.enabled'],
            'monitor_navigation': ['path.enabled', 'command.enabled'],
            'monitor_inject': ['inject.enabled'],
            'monitor_roll': ['roll.enabled'],
            'monitor_mecanum': ['mecanum.enabled'],
        }.items():
            if value(argument):
                for key in keys:
                    overrides[key] = enabled(argument)
        # Share the hardware launch's actual paths; avoid a second device list.
        if value('serial_ports_config'):
            with open(value('serial_ports_config'), encoding='utf-8') as stream:
                ports = yaml.safe_load(stream)
            for mechanism in ('inject', 'roll'):
                node_name = f'{mechanism}_serial_node'
                entry = ports.get(node_name, ports.get('/' + node_name, {}))
                device = entry.get('ros__parameters', {}).get('serial_path')
                if not isinstance(device, str) or not device:
                    raise ValueError(f'serial_ports_config requires {node_name}.ros__parameters.serial_path')
                overrides[f'{mechanism}.device'] = device
        if value('mecanum_device'):
            overrides['mecanum.device'] = value('mecanum_device')
        actions.append(Node(
            package='natto_system_monitor', executable='system_monitor',
            name='system_monitor', output='screen',
            parameters=[value('monitor_config'), overrides]))
        if enabled('aggregate'):
            actions.append(Node(
                package='diagnostic_aggregator', executable='aggregator_node',
                name='diagnostic_aggregator', output='screen',
                parameters=[value('diagnostics_config')]))
        if enabled('rosgraph'):
            actions.append(Node(
                package='rosgraph_monitor', executable='rosgraph_monitor_node',
                name='graph_monitor', output='screen',
                parameters=[value('rosgraph_config')]))
    if enabled('record'):
        with open(value('record_config'), encoding='utf-8') as stream:
            topics = yaml.safe_load(stream)['topics']
        if not isinstance(topics, list) or not topics or not all(
                isinstance(t, str) and t.startswith('/') and not any(c.isspace() for c in t)
                for t in topics):
            raise ValueError('record_config must contain a nonempty topics list of absolute topic names')
        name = value('bag_name') or datetime.now().strftime('trial_%Y%m%d_%H%M%S_%f')
        if Path(name).name != name or name in ('.', '..'):
            raise ValueError('bag_name must be a single directory name')
        folder = Path(value('bag_directory')).expanduser().resolve()
        output = folder / name
        if output.exists():
            raise FileExistsError(f'Bag already exists: {output}')
        folder.mkdir(parents=True, exist_ok=True)
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--storage', 'mcap',
                 '--storage-preset-profile', 'zstd_fast', '--output', str(output),
                 '--qos-profile-overrides-path', value('record_qos_config'),
                 '--topics', *dict.fromkeys(topics)], output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('natto_system_monitor'))
    defaults = {
        'monitor': 'true', 'aggregate': 'true', 'rosgraph': 'true', 'record': 'false',
        'managed_navigation': 'false',
        'serial_ports_config': '', 'mecanum_device': '',
        'monitor_lidar': '', 'monitor_localization': '', 'monitor_navigation': '',
        'monitor_inject': '', 'monitor_roll': '', 'monitor_mecanum': '',
        'bag_directory': str(Path.home() / 'rosbags'), 'bag_name': '',
        'monitor_config': str(share / 'config/monitor.yaml'),
        'diagnostics_config': str(share / 'config/diagnostics.yaml'),
        'rosgraph_config': str(share / 'config/rosgraph.yaml'),
        'record_config': str(share / 'config/recording.yaml'),
        'record_qos_config': str(share / 'config/recording_qos.yaml'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        OpaqueFunction(function=configure),
    ])
