import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Include official RealSense launch file with align_depth and pointcloud enabled
    realsense_launch_dir = os.path.join(get_package_share_directory("realsense2_camera"), "launch")
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, "rs_launch.py")),
        launch_arguments={
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "color_width": "640",
            "color_height": "480",
            "depth_width": "640",
            "depth_height": "480",
        }.items(),
    )

    # 2. Launch static TF transform between robot base_link and camera_color_optical_frame
    # Arguments: [x(m), y(m), z(m), yaw, pitch, roll, parent_frame, child_frame]
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_base_tf",
        arguments=[
            "0.0665",   # X
            "0.0325",   # Y
            "0.0765",   # Z
            "-1.5708",  # Yaw (-90 deg)
            "0.0",      # Pitch (0 deg)
            "-1.5708",  # Roll (-90 deg)
            "base_link",
            "camera_color_optical_frame",
        ],
    )

    # 3. Launch Custom Vision & Deprojection Node
    img_processor_node = Node(
        package="ada526_robot_arm",
        executable="img_processor",
        name="image_processor_node",
        output="screen",
    )

    return LaunchDescription(
        [
            realsense_node,
            static_tf_node,
            img_processor_node,
        ]
    )
