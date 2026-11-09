#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2


class ImageProcessor(Node):
    """
    ADA526 Vision & Deprojection Node:
    1. Subscribes to RealSense aligned CameraInfo, RGB Image, and Aligned Depth Image.
    2. Performs Gaussian smoothing and color blob segmentation to find target centroid (u, v).
    3. Deprojects 2D pixel (u, v) + depth (Z) into 3D camera coordinates (X_c, Y_c, Z_c).
    4. Transforms 3D point from Camera Optical Frame {C} to Robot Base Frame {R}.
    5. Publishes PointStamped on /goal_point and /camera_point (for RViz visualization).
    """

    def __init__(self):
        super().__init__("image_processor_node")

        # 1. Subscribe to Camera Intrinsics (focal length fx, fy and principal point ppx, ppy)
        self.sub_intrinsics = self.create_subscription(
            CameraInfo,
            "/camera/aligned_depth_to_color/camera_info",
            self.sub_intrinsics_callback,
            10,
        )

        # 2. Subscribe to RGB Color Stream
        self.sub_color = self.create_subscription(Image, "/camera/color/image_raw", self.sub_color_callback, 10)

        # 3. Subscribe to Aligned Depth Stream (matches RGB pixel-for-pixel)
        self.sub_depth = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.sub_depth_callback,
            10,
        )

        # 4. Publishers:
        # /camera_point -> In camera_color_optical_frame (meters, for RViz2 visualization)
        # /goal_point   -> In robot base_link frame (millimeters, for IK controller)
        self.pub_cam_point = self.create_publisher(PointStamped, "/camera_point", 10)
        self.pub_goal_red = self.create_publisher(PoseStamped, "/goal_point_red", 10)
        self.pub_goal_green = self.create_publisher(PoseStamped, "/goal_point_green", 10)
        self.pub_debug_img = self.create_publisher(Image, "/vision/debug_image", 10)

        # Internal state properties
        self.br = CvBridge()
        self.intrinsics = pyrealsense2.intrinsics()
        self.intrinsics_received = False

        # Depth at centroid in meters
        self.depth_red = 0.0
        self.depth_green = 0.0

        # Default center [u, v] in pixels
        self.centroid_red = [320, 240]
        self.centroid_green = [320, 240]

        # Homogeneous Transformation Matrix T_R_C (Camera Frame {C} -> Robot Base Frame {R})
        # TODO: Update with Patrick's exact CAD measurements (currently identity + offset placeholder in m)
        self.T_R_C = np.array(
            [
                [0.0, -1.0, 0.0, 0.150],  # X_R (m)
                [-1.0, 0.0, 0.0, 0.0],  # Y_R (m)
                [0.0, 0.0, -1.0, 0.400],  # Z_R (m)
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        self.get_logger().info("ImageProcessor node initialized. Waiting for RealSense streams...")

    def sub_intrinsics_callback(self, msg: CameraInfo):
        """Extracts camera intrinsic matrix K parameters from CameraInfo message."""
        if not self.intrinsics_received:
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = pyrealsense2.distortion.none
            self.intrinsics.coeffs = [i for i in msg.d]
            self.intrinsics_received = True
            self.get_logger().info(
                f"Camera intrinsics locked: {msg.width}x{msg.height}, "
                f"fx={self.intrinsics.fx:.1f}, fy={self.intrinsics.fy:.1f}"
            )

    def sub_color_callback(self, msg: Image):
        """Processes RGB frame, extracts blob centroid, and deprojects to 3D."""
        if not self.intrinsics_received:
            return

        # Convert ROS Image message to OpenCV array (RGB8 from realsense2_camera)
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Step A: Gaussian Smoothing (reduces high-frequency sensor noise)
        smoothed_bgr = cv2.GaussianBlur(current_frame, (5, 5), 3)

        # Step B: Convert to Lab color space (separates luminance L from color planes a & b)
        lab_img = cv2.cvtColor(smoothed_bgr, cv2.COLOR_BGR2LAB)

        # In OpenCV Lab: 'a' channel (>145 is strong red), 'b' channel (<110 is strong green)
        mask_red = cv2.inRange(lab_img, np.array([20, 145, 120]), np.array([255, 255, 200]))
        mask_green = cv2.inRange(lab_img, np.array([20, 50, 80]), np.array([255, 115, 170]))

        # Morphological opening to clean isolated noise pixels
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

        # Step C: Blob Detection & Centroid Extraction
        # Red Blob
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_red:
            largest_red = max(contours_red, key=cv2.contourArea)
            if cv2.contourArea(largest_red) > 150:  # Minimum pixel area threshold
                M = cv2.moments(largest_red)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])
                    self.centroid_red = [u, v]

                    self.deproject_and_publish(
                        self.centroid_red, self.depth_red, self.pub_goal_red, current_frame, (255, 0, 0)
                    )

        # Green Blob
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_green:
            largest_green = max(contours_green, key=cv2.contourArea)
            if cv2.contourArea(largest_green) > 150:  # Minimum pixel area threshold
                M = cv2.moments(largest_green)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])
                    self.centroid_green = [u, v]

                    self.deproject_and_publish(
                        self.centroid_green, self.depth_green, self.pub_goal_green, current_frame, (0, 255, 0)
                    )

        # Publish annotated debug image to ROS topic (viewable in rqt_image_view or RViz2)
        self.pub_debug_img.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))

    def deproject_and_publish(self, centroid, depth, publisher, current_frame, draw_color):
        """Converts a specific 2D pixel + depth into 3D robot coordinates and publishes it."""
        if depth > 0.05:  # Valid non-zero depth (> 5 cm)
            try:
                # 1. Deproject to Camera Frame [X_c, Y_c, Z_c]
                point_3d_m = pyrealsense2.rs2_deproject_pixel_to_point(self.intrinsics, centroid, depth)

                cam_pt = PointStamped()
                cam_pt.header.stamp = self.get_clock().now().to_msg()
                cam_pt.header.frame_id = "camera_color_optical_frame"
                cam_pt.point.x = float(point_3d_m[0])
                cam_pt.point.y = float(point_3d_m[1])
                cam_pt.point.z = float(point_3d_m[2])
                self.pub_cam_point.publish(cam_pt)

                # 2. Transform to Robot Base Frame {R}
                P_C_m = np.array([point_3d_m[0], point_3d_m[1], point_3d_m[2], 1.0])
                P_R_m = self.T_R_C @ P_C_m

                goal_pose = PoseStamped()
                goal_pose.header.stamp = cam_pt.header.stamp
                goal_pose.header.frame_id = "base_link"
                goal_pose.pose.position.x = float(P_R_m[0])
                goal_pose.pose.position.y = float(P_R_m[1])
                goal_pose.pose.position.z = float(P_R_m[2])
                goal_pose.pose.orientation.w = 1.0  # Identity quaternion
                publisher.publish(goal_pose)

                # 3. Annotate the debug image specific to this color target
                label = f"[{P_C_m[0]:.2f}, {P_C_m[1]:.2f}, {P_C_m[2]:.2f}]"
                cv2.putText(
                    current_frame,
                    label,
                    (centroid[0] - 40, centroid[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    draw_color,
                    2,
                )
                cv2.circle(current_frame, (centroid[0], centroid[1]), 5, draw_color, -1)

            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"Deprojection error: {e}")

    def sub_depth_callback(self, msg: Image):
        """Retrieves depth at the current detected centroid [u, v] from aligned depth image."""
        try:
            depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            h, w = depth_image.shape

            def get_safe_depth(u, v):
                u_clipped = np.clip(u, 0, w - 1)
                v_clipped = np.clip(v, 0, h - 1)
                # Convert RealSense 16UC1 millimeters to meters
                return float(depth_image[v_clipped, u_clipped]) / 1000.0

            # Update both state properties independently
            self.depth_red = get_safe_depth(self.centroid_red[0], self.centroid_red[1])
            self.depth_green = get_safe_depth(self.centroid_green[0], self.centroid_green[1])

        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"Depth callback error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
