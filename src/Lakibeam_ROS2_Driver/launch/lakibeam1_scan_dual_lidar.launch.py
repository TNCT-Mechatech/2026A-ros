import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    frame_id0 = LaunchConfiguration('frame_id0')
    frame_id1 = LaunchConfiguration('frame_id1')

    output_topic0 = LaunchConfiguration('output_topic0')
    output_topic1 = LaunchConfiguration('output_topic1')

    hostip = LaunchConfiguration('hostip')

    port0 = LaunchConfiguration('port0')
    port1 = LaunchConfiguration('port1')

    sensorip0 = LaunchConfiguration('sensorip0')
    sensorip1 = LaunchConfiguration('sensorip1')

    inverted0 = LaunchConfiguration('inverted0')
    inverted1 = LaunchConfiguration('inverted1')

    angle_offset0 = LaunchConfiguration('angle_offset0')
    angle_offset1 = LaunchConfiguration('angle_offset1')

    scanfreq = LaunchConfiguration('scanfreq')
    filter_value = LaunchConfiguration('filter')
    laser_enable = LaunchConfiguration('laser_enable')
    scan_range_start = LaunchConfiguration('scan_range_start')
    scan_range_stop = LaunchConfiguration('scan_range_stop')

    declare_args = [
        DeclareLaunchArgument(
            'frame_id0',
            default_value='lakibeam_front',
        ),
        DeclareLaunchArgument(
            'frame_id1',
            default_value='lakibeam_rear',
        ),

        DeclareLaunchArgument(
            'output_topic0',
            default_value='scan_front',
        ),
        DeclareLaunchArgument(
            'output_topic1',
            default_value='scan_rear',
        ),

        DeclareLaunchArgument(
            'hostip',
            default_value='0.0.0.0',
        ),

        DeclareLaunchArgument(
            'port0',
            default_value='2368',
        ),
        DeclareLaunchArgument(
            'port1',
            default_value='2369',
        ),

        DeclareLaunchArgument(
            'sensorip0',
            default_value='',
        ),
        DeclareLaunchArgument(
            'sensorip1',
            default_value='',
        ),

        DeclareLaunchArgument(
            'inverted0',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'inverted1',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'angle_offset0',
            default_value='0',
        ),
        DeclareLaunchArgument(
            'angle_offset1',
            default_value='0',
        ),

        DeclareLaunchArgument(
            'scanfreq',
            default_value='',
        ),
        DeclareLaunchArgument(
            'filter',
            default_value='',
        ),
        DeclareLaunchArgument(
            'laser_enable',
            default_value='',
        ),
        DeclareLaunchArgument(
            'scan_range_start',
            default_value='',
        ),
        DeclareLaunchArgument(
            'scan_range_stop',
            default_value='',
        ),
    ]

    richbeam_lidar_node0 = Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name='richbeam_lidar_node0',
        output='screen',
        parameters=[{
            'frame_id': ParameterValue(frame_id0, value_type=str),
            'output_topic': ParameterValue(output_topic0, value_type=str),
            'hostip': ParameterValue(hostip, value_type=str),
            'sensorip': ParameterValue(sensorip0, value_type=str),
            'port': ParameterValue(port0, value_type=str),
            'inverted': ParameterValue(inverted0, value_type=bool),
            'angle_offset': ParameterValue(angle_offset0, value_type=int),
            'scanfreq': ParameterValue(scanfreq, value_type=str),
            'filter': ParameterValue(filter_value, value_type=str),
            'laser_enable': ParameterValue(laser_enable, value_type=str),
            'scan_range_start': ParameterValue(scan_range_start, value_type=str),
            'scan_range_stop': ParameterValue(scan_range_stop, value_type=str),
        }],
    )

    richbeam_lidar_node1 = Node(
        package='lakibeam1',
        executable='lakibeam1_scan_node',
        name='richbeam_lidar_node1',
        output='screen',
        parameters=[{
            'frame_id': ParameterValue(frame_id1, value_type=str),
            'output_topic': ParameterValue(output_topic1, value_type=str),
            'hostip': ParameterValue(hostip, value_type=str),
            'sensorip': ParameterValue(sensorip1, value_type=str),
            'port': ParameterValue(port1, value_type=str),
            'inverted': ParameterValue(inverted1, value_type=bool),
            'angle_offset': ParameterValue(angle_offset1, value_type=int),
            'scanfreq': ParameterValue(scanfreq, value_type=str),
            'filter': ParameterValue(filter_value, value_type=str),
            'laser_enable': ParameterValue(laser_enable, value_type=str),
            'scan_range_start': ParameterValue(scan_range_start, value_type=str),
            'scan_range_stop': ParameterValue(scan_range_stop, value_type=str),
        }],
    )

    ld = LaunchDescription()

    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(richbeam_lidar_node0)
    ld.add_action(richbeam_lidar_node1)

    return ld