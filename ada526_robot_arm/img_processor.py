#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as image_msg
from sensor_msgs.msg import CameraInfo as camerainfo_msg
from geometry_msgs.msg import PointStamped as point_msg
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

# Optional import of machinevisiontoolbox as shown in ADA526 lecture notes
try:
    import machinevisiontoolbox as mvt
    MVT_AVAILABLE = True
except ImportError:
    MVT_AVAILABLE = False


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
        super().__init__('image_processor_node')

        # Declare configurable ROS 2 parameters
        self.declare_parameter('target_color', 'red')  # 'red' or 'blue'
        self.declare_parameter('show_debug_window', True)

        # 1. Subscribe to Camera Intrinsics (focal length fx, fy and principal point ppx, ppy)
        self.sub_intrinsics = self.create_subscription(
            camerainfo_msg,
            '/camera/aligned_depth_to_color/camera_info',
            self.sub_intrinsics_callback,
            10
        )

        # 2. Subscribe to RGB Color Stream
        self.sub_color = self.create_subscription(
            image_msg,
            '/camera/color/image_raw',
            self.sub_color_callback,
            10
        )

        # 3. Subscribe to Aligned Depth Stream (matches RGB pixel-for-pixel)
        self.sub_depth = self.create_subscription(
            image_msg,
            '/camera/aligned_depth_to_color/image_raw',
            self.sub_depth_callback,
            10
        )

        # 4. Publishers:
        # /camera_point -> In camera_color_optical_frame (meters, for RViz2 visualization)
        # /goal_point   -> In robot base_link frame (millimeters, for IK controller)
        self.pub_cam_point = self.create_publisher(point_msg, '/camera_point', 10)
        self.pub_goal_point = self.create_publisher(point_msg, '/goal_point', 10)
        self.pub_debug_img = self.create_publisher(image_msg, '/vision/debug_image', 10)

        # Internal state properties
        self.br = CvBridge()
        self.intrinsics = rs.intrinsics()
        self.intrinsics_received = False
        self.centroids = [320, 240]  # Default center [u, v] in pixels
        self.depth = 0.0             # Depth at centroid in meters

        # Homogeneous Transformation Matrix T_R_C (Camera Frame {C} -> Robot Base Frame {R})
        # Update with Patrick's exact CAD measurements (currently identity + offset placeholder in mm)
        self.T_R_C = np.array([
            [ 0.0, -1.0,  0.0,  150.0],  # X_R (mm)
            [-1.0,  0.0,  0.0,    0.0],  # Y_R (mm)
            [ 0.0,  0.0, -1.0,  400.0],  # Z_R (mm)
            [ 0.0,  0.0,  0.0,    1.0]
        ])

        self.get_logger().info('ImageProcessor node initialized. Waiting for RealSense streams...')

    def sub_intrinsics_callback(self, msg: camerainfo_msg):
        """Extracts camera intrinsic matrix K parameters from CameraInfo message."""
        if not self.intrinsics_received:
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = rs.distortion.none
            self.intrinsics.coeffs = [i for i in msg.d]
            self.intrinsics_received = True
            self.get_logger().info(
                f'Camera intrinsics locked: {msg.width}x{msg.height}, '
                f'fx={self.intrinsics.fx:.1f}, fy={self.intrinsics.fy:.1f}'
            )

    def sub_color_callback(self, msg: image_msg):
        """Processes RGB frame, extracts blob centroid, and deprojects to 3D."""
        if not self.intrinsics_received:
            return

        # Convert ROS Image message to OpenCV array (RGB8 from realsense2_camera)
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Step A: Gaussian Smoothing (reduces high-frequency sensor noise)
        smoothed_bgr = cv2.GaussianBlur(current_frame, (5, 5), 3)

        # Step B: Convert to Lab color space (separates luminance L from color planes a & b)
        lab_img = cv2.cvtColor(smoothed_bgr, cv2.COLOR_BGR2LAB)
        target_color = self.get_parameter('target_color').get_parameter_value().string_value

        # In OpenCV Lab: 'a' channel (>145 is strong red), 'b' channel (<110 is strong blue)
        if target_color == 'red':
            mask = cv2.inRange(lab_img, np.array([20, 145, 120]), np.array([255, 255, 200]))
        else:  # 'blue'
            mask = cv2.inRange(lab_img, np.array([20, 100, 40]), np.array([255, 160, 110]))

        # Morphological opening to clean isolated noise pixels
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Step C: Blob Detection & Centroid Extraction
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_blob = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_blob) > 150:  # Minimum pixel area threshold
                M = cv2.moments(largest_blob)
                if M['m00'] != 0:
                    u = int(M['m10'] / M['m00'])
                    v = int(M['m01'] / M['m00'])
                    self.centroids = [u, v]

                    # Draw visual overlay on debug frame
                    cv2.drawContours(current_frame, [largest_blob], -1, (0, 255, 0), 2)
                    cv2.circle(current_frame, (u, v), 5, (0, 0, 255), -1)

        # Step D: 2D-to-3D Deprojection using pyrealsense2
        if self.depth > 0.05:  # Valid non-zero depth (> 5 cm)
            try:
                # Returns [X_c, Y_c, Z_c] in meters in camera_color_optical_frame
                point_3d_m = rs.rs2_deproject_pixel_to_point(
                    self.intrinsics, self.centroids, self.depth
                )

                # 1. Publish in Camera Optical Frame (meters) for RViz2 verification
                cam_pt = point_msg()
                cam_pt.header.stamp = self.get_clock().now().to_msg()
                cam_pt.header.frame_id = 'camera_color_optical_frame'
                cam_pt.point.x = float(point_3d_m[0])
                cam_pt.point.y = float(point_3d_m[1])
                cam_pt.point.z = float(point_3d_m[2])
                self.pub_cam_point.publish(cam_pt)

                # 2. Convert to millimeters and Transform to Robot Base Frame {R}
                P_C_mm = np.array([
                    point_3d_m[0] * 1000.0,
                    point_3d_m[1] * 1000.0,
                    point_3d_m[2] * 1000.0,
                    1.0
                ])
                P_R_mm = self.T_R_C @ P_C_mm

                goal_pt = point_msg()
                goal_pt.header.stamp = cam_pt.header.stamp
                goal_pt.header.frame_id = 'base_link'
                goal_pt.point.x = float(P_R_mm[0])
                goal_pt.point.y = float(P_R_mm[1])
                goal_pt.point.z = float(P_R_mm[2])
                self.pub_goal_point.publish(goal_pt)

                # Annotate 3D coordinates on debug image
                label = f"Cam(mm): [{P_C_mm[0]:.0f}, {P_C_mm[1]:.0f}, {P_C_mm[2]:.0f}]"
                cv2.putText(current_frame, label, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            except Exception as e:
                self.get_logger().warn(f'Deprojection error: {e}')

        # Publish annotated debug image to ROS topic (viewable in rqt_image_view or RViz2)
        self.pub_debug_img.publish(self.br.cv2_to_imgmsg(current_frame, encoding='bgr8'))

    def sub_depth_callback(self, msg: image_msg):
        """Retrieves depth at the current detected centroid [u, v] from aligned depth image."""
        try:
            depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            u, v = self.centroids[0], self.centroids[1]
            # Note: numpy image indexing is [row, col] which corresponds to [v, u]
            h, w = depth_image.shape
            u_clipped = np.clip(u, 0, w - 1)
            v_clipped = np.clip(v, 0, h - 1)

            # RealSense 16UC1 depth image stores depth in millimeters; convert to meters
            raw_depth_mm = float(depth_image[v_clipped, u_clipped])
            self.depth = raw_depth_mm / 1000.0
        except Exception as e:
            self.get_logger().warn(f'Depth callback error: {e}')


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


if __name__ == '__main__':
    main()