from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    drone_ns_arg = DeclareLaunchArgument(
        "drone_namespace",
        default_value="/pequi/hermit",
        description="ROS2 namespace prefix for PX4 topics",
    )
    timeout_arg = DeclareLaunchArgument(
        "topic_timeout_sec",
        default_value="2.0",
        description="Seconds before a topic is considered stale",
    )
    rate_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="2.0",
        description="Diagnostic publish rate in Hz",
    )
    influxdb_arg = DeclareLaunchArgument(
        "enable_influxdb",
        default_value="false",
        description="Start the InfluxDB diagnostics logger node",
    )
    influxdb_url_arg = DeclareLaunchArgument(
        "influxdb_url",
        default_value="http://localhost:8086",
        description="InfluxDB v2 URL",
    )
    influxdb_org_arg = DeclareLaunchArgument(
        "influxdb_org",
        default_value="aerobatics",
        description="InfluxDB organisation name",
    )
    influxdb_bucket_arg = DeclareLaunchArgument(
        "influxdb_bucket",
        default_value="drone_diagnostics",
        description="InfluxDB bucket to write into",
    )

    config_file = PathJoinSubstitution([
        FindPackageShare("drone_diagnostics"),
        "config",
        "diagnostics_config.yaml",
    ])

    diagnostic_node = Node(
        package="drone_diagnostics",
        executable="diagnostic_node",
        name="drone_diagnostic",
        parameters=[
            config_file,
            {
                "drone_namespace": LaunchConfiguration("drone_namespace"),
                "topic_timeout_sec": LaunchConfiguration("topic_timeout_sec"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            },
        ],
        output="screen",
    )

    aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        parameters=[config_file],
        output="screen",
    )

    logger_node = Node(
        package="drone_diagnostics",
        executable="diagnostics_logger",
        name="diagnostics_logger",
        parameters=[{
            "drone_namespace": LaunchConfiguration("drone_namespace"),
            "influxdb_url": LaunchConfiguration("influxdb_url"),
            "influxdb_org": LaunchConfiguration("influxdb_org"),
            "influxdb_bucket": LaunchConfiguration("influxdb_bucket"),
        }],
        condition=IfCondition(LaunchConfiguration("enable_influxdb")),
        output="screen",
    )

    return LaunchDescription([
        drone_ns_arg,
        timeout_arg,
        rate_arg,
        influxdb_arg,
        influxdb_url_arg,
        influxdb_org_arg,
        influxdb_bucket_arg,
        diagnostic_node,
        aggregator_node,
        logger_node,
    ])
