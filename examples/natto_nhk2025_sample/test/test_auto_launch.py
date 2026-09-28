"""Expand real launch files, intercepting every process/component start.

No hardware is opened and no ROS node is started by these tests.
"""
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.actions import LoadComposableNodes


def expand(variant, arguments):
    commands = []

    def intercept(action, context):
        commands.append([perform_substitutions(context, part) for part in action.cmd])
        return None

    path = Path(get_package_share_directory('natto_nhk2025_sample')) / 'launch' / variant
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(path)), launch_arguments=arguments.items()),
    ]))
    with patch.object(ExecuteProcess, 'execute', intercept), patch.object(LoadComposableNodes, 'execute', return_value=None):
        assert service.run() == 0
    return commands


def executable(commands, package, name):
    suffix = f'/lib/{package}/{name}'
    return [command for command in commands if command[0].endswith(suffix)]


def parameters(command):
    result = {}
    for index, value in enumerate(command[:-1]):
        if value == '--params-file':
            document = yaml.safe_load(Path(command[index + 1]).read_text())
            for entry in document.values():
                result.update(entry.get('ros__parameters', {}))
    return result


@pytest.mark.parametrize('variant', ['mecanum_auto.launch.xml', 'mecanum_auto_red.launch.xml', 'mecanum_auto_blue.launch.xml'])
def test_default_runs_operation_monitoring_and_both_mechanisms_once(variant):
    commands = expand(variant, {})
    assert len(executable(commands, 'natto_robot_hardware', 'serial_motor_bridge')) == 1
    mechanisms = executable(commands, 'inject2026A', 'main')
    assert len(mechanisms) == 1
    assert 'inject/joy:=/inject/joy' in mechanisms[0]
    monitors = executable(commands, 'natto_system_monitor', 'system_monitor')
    assert len(monitors) == 1
    assert len(executable(commands, 'diagnostic_aggregator', 'aggregator_node')) == 1
    assert len(executable(commands, 'rosgraph_monitor', 'rosgraph_monitor_node')) == 1
    values = parameters(monitors[0])
    assert values['managed_navigation'] is True
    assert values['inject.enabled'] is True
    assert values['roll.enabled'] is True
    assert values['mecanum.enabled'] is True
    assert not any(command[:3] == ['ros2', 'bag', 'record'] for command in commands)


def test_disable_mechanisms_disables_their_watchdogs():
    commands = expand('mecanum_auto_red.launch.xml', {'launch_inject': 'false'})
    assert not executable(commands, 'inject2026A', 'main')
    values = parameters(executable(commands, 'natto_system_monitor', 'system_monitor')[0])
    assert values['inject.enabled'] is False
    assert values['roll.enabled'] is False


def test_disable_monitoring_keeps_operation_and_mechanisms():
    commands = expand('mecanum_auto_blue.launch.xml', {'launch_system_monitor': 'false'})
    assert len(executable(commands, 'inject2026A', 'main')) == 1
    assert len(executable(commands, 'natto_robot_hardware', 'serial_motor_bridge')) == 1
    assert not executable(commands, 'natto_system_monitor', 'system_monitor')
    assert not executable(commands, 'diagnostic_aggregator', 'aggregator_node')
    assert not executable(commands, 'rosgraph_monitor', 'rosgraph_monitor_node')


def test_custom_ports_and_recording_are_forwarded(tmp_path):
    ports = tmp_path / 'ports.yaml'
    ports.write_text(yaml.safe_dump({f'{name}_serial_node': {'ros__parameters': {'serial_path': f'/dev/test_{name}'}} for name in ('inject', 'roll')}))
    commands = expand('mecanum_auto_red.launch.xml', {
        'inject_serial_config': str(ports), 'serial_path': '/dev/test_mecanum',
        'inject_joy_topic': '/test/joy', 'record': 'true',
        'bag_directory': str(tmp_path), 'bag_name': 'trial',
    })
    mechanism = executable(commands, 'inject2026A', 'main')[0]
    assert str(ports) in mechanism
    assert 'inject/joy:=/test/joy' in mechanism
    values = parameters(executable(commands, 'natto_system_monitor', 'system_monitor')[0])
    assert values['inject.device'] == '/dev/test_inject'
    assert values['roll.device'] == '/dev/test_roll'
    assert values['mecanum.device'] == '/dev/test_mecanum'
    assert any(command[:3] == ['ros2', 'bag', 'record'] and str(tmp_path / 'trial') in command for command in commands)
