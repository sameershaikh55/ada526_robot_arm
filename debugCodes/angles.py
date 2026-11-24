import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np


class RobotNode(Node):
    def __init__(self):
        super().__init__('kordniate_node')

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_names = [f"joint_{i + 1}" for i in range(5)]
    
    def publish_goal(self, joints):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(np.deg2rad(x)) for x in joints]
        self.joint_pub.publish(msg)
        self.get_logger().debug(f"Publishing: {msg.position}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        while rclpy.ok():
            text = input("joint1, joint2, joint3 ... (q to quit): ").strip()
            if text.lower() == 'q':
                break
            try:
                joints = list(map(float, text.replace(',', ' ').split()))
                if len(joints) != 5:
                    print("Expected exactly 5 joint values")
                    continue
            except ValueError:
                print("Invalid inpt")
                print()
                continue
            node.publish_goal(joints)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()