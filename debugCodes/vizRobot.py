from spatialmath import SE3
import time
import roboticstoolbox as rtb
from math import pi
# import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np

def buildRobot():
    robot = rtb.DHRobot(
    [
     	rtb.RevoluteDH(d=0.2, a=0.0, alpha=pi/2, qlim=[-110 *pi/180, 110 *pi/180]), 
        rtb.RevoluteDH(a=0.20, d=0.0, qlim=[-80 *pi/180, 80 *pi/180]),
        rtb.RevoluteDH(a=0.28, d=0.0, qlim=[-80 *pi/180, 80 *pi/180]),
        rtb.RevoluteDH(alpha=pi/2,offset=pi/2,qlim=[-80 *pi/180, 80 *pi/180]),
        rtb.RevoluteDH(d=0.05, a=0.00, qlim=[-2*pi, 2*pi]),
    ], name="My Config 1 Robot")
    robot.tool = SE3(0, 0.05, 0.05)
    robot.qr = [0, 0, 0, -80*pi/180, 0]
    return (robot)

class RobotViz(Node):
    def __init__(self): 
        super().__init__("robot_node")

        self.robot = buildRobot()
        self.robot.q = self.robot.qr
        self.joint_names = [f"joint_{i+1}" for i in range(self.robot.n)]
        self.joint_pub = self.create_subscription(JointState, "/joint_states", self.subscriber_pos,10)
        self.robot_plot = self.robot.plot(self.robot.qr, backend='pyplot') # plot the robot in its home position


    def subscriber_pos(self, msg: JointState):
        
        if len(msg.position) < self.robot.n:
            self.get_logger().warning(
                "Mottok ikkje nok leddvinklar."
            )
            return

        self.robot.q = list(msg.position[:self.robot.n])
        self.robot_plot.step()
        self.get_logger().info(
            f"Leddvinklar: {np.round(np.rad2deg(self.robot.q),0)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()