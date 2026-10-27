#!/usr/bin/env python3

from spatialmath import SE3
import time
import roboticstoolbox as rtb
from math import pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# This example shows how to check if your robot can reach all the corners of the base plate.
# If you see that the translational o andular distance is too large, the manipulability is close to zero or the Jacobian of the robot loses rank (rank < 5), you should consider changing the robot's link lengths and offsets.

# Setter opp robotens parameter
def buildRobot():
    robot = rtb.DHRobot(
        [
            rtb.RevoluteDH(d=0.2, a=0.0, alpha=pi/2, qlim=[-110 *pi/180, 110 *pi/180]), 
            rtb.RevoluteDH(a=0.20, d=0.0, qlim=[-80 *pi/180, 260 *pi/180]),
            rtb.RevoluteDH(a=0.28, d=0.0, qlim=[-170 *pi/180, 170 *pi/180]),
            rtb.RevoluteDH(a=0.05, d=0.0, alpha=pi/2,qlim=[-170 *pi/180, 170 *pi/180]),
            rtb.RevoluteDH(d=0.05, a=0.02, qlim=[-2*pi, 2*pi]),
        ], name="My Config 1 Robot")
    robot.qr = [0, 0.0, 0.0, -pi/2, 0.0]   
    return (robot)

class RobotNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_node")

        self.robot = buildRobot()
        self.robot.q = self.robot.qr
        self.joint_names = [f"joint_{i+1}" for i in range(self.robot.n)]
        self.traj = []
        self.trajState = False #Trajetery if true

        self.create_subscription(PoseStamped, "/goal_point", self.subscriber_pos, 10)    

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        self.create_timer(0.3, self.timer_callback)

    def timer_callback(self):
        # Publish step, and go to nex
        if self.traj:
            if self.trajState:
                self.robot.q = self.traj.pop(0)
            else:
                self.robot.q = self.traj[-1]
                self.traj.clear()
        self.publish_state()
 
    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(x) for x in self.robot.q]
        self.joint_pub.publish(msg)
        self.get_logger().info(f"Publishing: {msg.position}")

    def subscriber_pos(self, msg: Pose):
        p = msg.pose.position
        Tgoal = SE3(p.x, p.y, p.z)
        sol = self.robot.ikine_LM(
            Tgoal, 
            q0=self.robot.q, 
            mask=[1, 1, 1, 0, 0, 0]#Can add tol to get more acurate position, but need to see if motor are can do bether
            )

        if not sol.success:
            self.get_logger().warn(
                f"No solution for ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
            )
            return

        if self.trajState:
            self.traj = list(rtb.jtraj(self.robot.q, sol.q, 50).q)
        else:
            self.robot.q = sol.q
        self.get_logger().info("Mål akseptert, starter bevegelse")


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()