#!/usr/bin/env python3

from spatialmath import SE3
import time
import roboticstoolbox as rtb
from math import pi
# import numpy as np

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
        rtb.RevoluteDH(a=0.20, d=0.0, qlim=[-80 *pi/180, 80 *pi/180]),
        rtb.RevoluteDH(a=0.28, d=0.0, qlim=[-80 *pi/180, 80 *pi/180]),
        rtb.RevoluteDH(alpha=pi/2,offset=pi/2,qlim=[-80 *pi/180, 80 *pi/180]),
        rtb.RevoluteDH(d=0.05, a=0.00, qlim=[-2*pi, 2*pi]),
    ], name="My Config 1 Robot")
    robot.tool = SE3(0, 0.05, 0.05) 
    robot.qr = [0, 0, 0, 0, 0]
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

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

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


    def no_leagal_point(self, T_all: SE3):
        for T in T_all[1:]:
            if T.t[2] <= 0:
                return True

        return False
    
    def posisjons(self, Tgoal):
        sol = self.robot.ikine_LM(
            Tgoal, 
            q0=self.robot.q, 
            mask=[1, 1, 1, 0, 0, 0.5],
            tol = 1e-6, #Smaller tol => samller error
            joint_limits = True #Take q.lim in consideration
            )
        

        if sol.success and self.no_leagal_point(self.robot.fkine_all(sol.q)):
            self.get_logger().warn(f"Link throu the table, trying to move goal up")
            newTgoal = SE3(Tgoal.t[0], Tgoal.t[1], Tgoal.t[2] + 0.05)
            sol = self.posisjons(newTgoal)
            if sol:
                sol = self.robot.ikine_LM(
                    Tgoal, 
                    q0=sol.q, 
                    mask=[1, 1, 1, 0, 0, 0.5],
                    tol = 1e-6, #Smaller tol => samller error
                    joint_limits = True #Take q.lim in consideration
                )
            
        if sol:
            if not sol.success or self.no_leagal_point(self.robot.fkine_all(sol.q)):
                return None
        return sol
                     

    def subscriber_pos(self, msg: PoseStamped):
        p = msg.pose.position
        Tgoal = SE3(p.x, p.y, p.z)
        # sol = self.robot.ikine_LM(
        #     Tgoal, 
        #     q0=self.robot.q, 
        #     mask=[1, 1, 1, 0, 0, 0],
        #     tol = 1e-6, #Smaller tol => samller error
        #     joint_limits = True #Take q.lim in consideration
        #     )

        # if not sol.success:
        #     self.get_logger().warn(
        #         f"No solution for ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
        #     )
        #     return
        

        sol = self.posisjons(Tgoal)

        if not sol:
            self.get_logger().warn(
                f"No solution for ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
                )
            return 


        if self.trajState:
            self.traj = list(rtb.jtraj(self.robot.q, sol.q, 50).q)
        else:
            self.robot.q = sol.q
        self.get_logger().info("Goal accapted, moving to goal")


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