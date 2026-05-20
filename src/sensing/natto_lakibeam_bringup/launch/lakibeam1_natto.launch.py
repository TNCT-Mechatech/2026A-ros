from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id")
    hostip = LaunchConfiguration("hostip")
    sensorip = LaunchConfiguration("sensorip")
    port = LaunchConfiguration("port")
    angle_offset = LaunchConfiguration("angle_offset")
    inverted = LaunchConfiguration("inverted")
    filter_threshold = LaunchConfiguration("filter_threshold")

    declare_frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="laser",
        description="LakiBeam LaserScan frame_id",
    )

    declare_hostip = DeclareLaunchArgument(
        "hostip",
        default_value="0.0.0.0",
        description="PC listen IP. 0.0.0.0 listens all interfaces.",
    )

    declare_sensorip = DeclareLaunchArgument(
        "sensorip",
        default_value="192.168.198.2",
        description="LakiBeam sensor IP when using RJ45.",
    )

    declare_port = DeclareLaunchArgument(
        "port",
        default_value="2368",
        description="UDP listen port",
    )

    declare_angle_offset = DeclareLaunchArgument(
        "angle_offset",
        default_value="0",
        description="LaserScan angle offset degree",
    )

    declare_inverted = DeclareLaunchArgument(
        "inverted",
        default_value="false",
        description="Invert scan direction",
    )

    declare_filter_threshold = DeclareLaunchArgument(
        "filter_threshold",
        default_value="0.83",
        description="natto_laser_filter threshold",
    )

    lakibeam_node = Node(
        package="lakibeam1",
        executable="lakibeam1_scan_node",
        name="lakibeam1_scan",
        output="screen",
        parameters=[
            {
                "frame_id": ParameterValue(frame_id, value_type=str),
                "output_topic": "scan",
                "inverted": ParameterValue(inverted, value_type=bool),
                "hostip": ParameterValue(hostip, value_type=str),
                "sensorip": ParameterValue(sensorip, value_type=str),
                "port": ParameterValue(port, value_type=str),
                "angle_offset": ParameterValue(angle_offset, value_type=int),
                "scanfreq": "30",
                "filter": "3",
                "laser_enable": "true",
                "scan_range_start": "45",
                "scan_range_stop": "315",
            }
        ],
    )

    laser_filter_node = Node(
        package="natto_laser_filter",
        executable="laser_filter",
        name="lakibeam_laser_filter",
        output="screen",
        parameters=[
            {
                "threshold": filter_threshold,
            }
        ],
        remappings=[
            ("input", "/scan"),
            ("output", "/lakibeam/filtered_scan"),
        ],
    )

    laserscan_to_pointcloud2_node = Node(
        package="natto_lidar_converter",
        executable="laserscan_to_pointcloud2",
        name="lakibeam_laserscan_to_pointcloud2",
        output="screen",
        parameters=[
            {
                "frame_id": frame_id,
            }
        ],
        remappings=[
            ("laserscan", "/lakibeam/filtered_scan"),
            ("pointcloud2", "/lakibeam/pointcloud2"),
        ],
    )

    return LaunchDescription(
        [
            declare_frame_id,
            declare_hostip,
            declare_sensorip,
            declare_port,
            declare_angle_offset,
            declare_inverted,
            declare_filter_threshold,
            lakibeam_node,
            laser_filter_node,
            laserscan_to_pointcloud2_node,
        ]
    )