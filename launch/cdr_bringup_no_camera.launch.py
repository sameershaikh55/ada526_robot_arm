import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # 4. Launch Custom Dynamixel Robot Controller Node
    robot_controller_node = Node(
        package="ada526_robot_arm",
        executable="robot_controller",
        name="robot_controller_node",
        output="screen",
        parameters=[
            {
                "port_name": "/dev/ttyUSB0",
                "baudrate": 57600,
                "profile_velocity": 35,
                "profile_acceleration": 12,
            }
        ],
    )

    # 5. Launch Kinematics Core Node
    kinematics_node = Node(
        package="ada526_robot_arm",
        executable="kinematics_core",
        name="kinematics_node",
        output="screen",
        parameters=[{"challenge_state": "COORDINATE"}],
    )

    return LaunchDescription(
        [
            kinematics_node,
            robot_controller_node,
        ]
    )
