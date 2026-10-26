#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

try:
    from dynamixel_sdk import (
        PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS,
        DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
    )
    DXL_AVAILABLE = True
except ImportError:
    DXL_AVAILABLE = False


# Dynamixel X-Series Control Table Addresses (Protocol 2.0)
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
LEN_GOAL_POSITION = 4
PROTOCOL_VERSION = 2.0


class RobotControllerNode(Node):
    """
    ADA526 Dynamixel ROS 2 Controller Node:
    - Subscribes to /joint_commands (std_msgs/Float64MultiArray in radians) for direct CDR demos.
    - Subscribes to /goal_point (geometry_msgs/PointStamped in mm) for vision-guided IK control.
    - Enforces velocity and acceleration profiles on Motor IDs 1..5 to prevent mechanical damage.
    """

    def __init__(self):
        super().__init__('robot_controller_node')

        # Declare ROS 2 parameters for hardware configuration
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('motor_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('profile_velocity', 40)       # Safe smooth speed limit
        self.declare_parameter('profile_acceleration', 15)   # Safe smooth accel limit
        self.declare_parameter('hardware_connected', False)  # False allows dry-run testing at home

        self.port_name = self.get_parameter('port_name').value
        self.baudrate = self.get_parameter('baudrate').value
        self.motor_ids = self.get_parameter('motor_ids').value
        self.prof_vel = self.get_parameter('profile_velocity').value
        self.prof_acc = self.get_parameter('profile_acceleration').value
        self.hw_connected = self.get_parameter('hardware_connected').value

        # Subscribers
        self.sub_joints = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_cmd_callback,
            10
        )
        self.sub_goal = self.create_subscription(
            PointStamped,
            '/goal_point',
            self.goal_point_callback,
            10
        )

        if self.hw_connected and DXL_AVAILABLE:
            self.init_dynamixels()
        else:
            self.get_logger().info(
                'Running in SIMULATION / DRY-RUN mode (hardware_connected:=False). '
                'Commands will be logged in terminal.'
            )

    def init_dynamixels(self):
        """Initializes U2D2 port, enables torque, and writes safe velocity/acceleration profiles."""
        self.portHandler = PortHandler(self.port_name)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION
        )

        if not self.portHandler.openPort():
            self.get_logger().error(f'Failed to open U2D2 port {self.port_name}!')
            self.hw_connected = False
            return

        if not self.portHandler.setBaudRate(self.baudrate):
            self.get_logger().error(f'Failed to set baudrate to {self.baudrate}!')
            self.hw_connected = False
            return

        for dxl_id in self.motor_ids:
            # Configure safe velocity and acceleration profiles before moving
            self.packetHandler.write4ByteTxRx(
                self.portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, self.prof_acc
            )
            self.packetHandler.write4ByteTxRx(
                self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, self.prof_vel
            )
            # Enable Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn(f'Motor ID {dxl_id} not responding!')
            else:
                self.get_logger().info(f'Motor ID {dxl_id} Torque Enabled (Vel={self.prof_vel}, Acc={self.prof_acc}).')

    @staticmethod
    def rad_to_ticks(angle_rad: float) -> int:
        """
        Converts joint angle in radians [-pi, +pi] to Dynamixel X-series ticks [0, 4095].
        Center (0.0 rad) = 2048 ticks.
        """
        ticks = int(2048 + (angle_rad / np.pi) * 2048)
        return int(np.clip(ticks, 0, 4095))

    def send_sync_goal_ticks(self, tick_list: list):
        """Uses GroupSyncWrite to command all motors simultaneously."""
        if not self.hw_connected:
            self.get_logger().info(f'[DRY-RUN] SyncWrite Goal Ticks: {tick_list}')
            return

        self.groupSyncWrite.clearParam()
        for dxl_id, goal_tick in zip(self.motor_ids, tick_list):
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(goal_tick)),
                DXL_HIBYTE(DXL_LOWORD(goal_tick)),
                DXL_LOBYTE(DXL_HIWORD(goal_tick)),
                DXL_HIBYTE(DXL_HIWORD(goal_tick))
            ]
            self.groupSyncWrite.addParam(dxl_id, param_goal_position)

        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'SyncWrite failed: {self.packetHandler.getTxRxResult(dxl_comm_result)}')

    def joint_cmd_callback(self, msg: Float64MultiArray):
        """Receives [q1, q2, q3, q4, q5] in radians and executes synchronized motion."""
        if len(msg.data) != len(self.motor_ids):
            self.get_logger().warn(f'Expected {len(self.motor_ids)} joint angles, got {len(msg.data)}.')
            return

        ticks = [self.rad_to_ticks(q) for q in msg.data]
        self.get_logger().info(f'Executing Joint Command (rad): {[round(q, 2) for q in msg.data]} -> Ticks: {ticks}')
        self.send_sync_goal_ticks(ticks)

    def goal_point_callback(self, msg: PointStamped):
        """Receives 3D target point [X_R, Y_R, Z_R] in base_link frame (mm)."""
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        self.get_logger().info(f'Received Target in Base Frame (mm): X={x:.1f}, Y={y:.1f}, Z={z:.1f}')
        # Hook for Hansi's IK solver:
        # q_sol, success = solve_ik(x, y, z)
        # if success: self.send_sync_goal_ticks([self.rad_to_ticks(q) for q in q_sol])


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()