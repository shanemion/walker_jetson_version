from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/serial/by-id/forcewalker_teensy",
                description="Serial device for the Teensy force/IMU bridge.",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
                description="Serial baud rate.",
            ),
            DeclareLaunchArgument(
                "calib_yaml",
                default_value="/opt/forcewalker/forcewalker/config/force_calibration.yaml",
                description="Force channel calibration YAML.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="forcewalker_force",
                description="Frame id for force channel messages.",
            ),
            DeclareLaunchArgument(
                "imu_frame_id",
                default_value="forcewalker_imu",
                description="Frame id for IMU messages.",
            ),
            Node(
                package="forcewalker_sensors",
                executable="teensy_force_bridge",
                name="teensy_force_bridge",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "baud": LaunchConfiguration("baud"),
                        "calib_yaml": LaunchConfiguration("calib_yaml"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "imu_frame_id": LaunchConfiguration("imu_frame_id"),
                    }
                ],
            ),
        ]
    )
