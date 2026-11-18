#!/usr/bin/env python3

from spatialmath import SE3
import time
import roboticstoolbox as rtb
from math import pi
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult

from scipy.interpolate import RBFInterpolator

# This example shows how to check if your robot can reach all the corners of the base plate.
# If you see that the translational o andular distance is too large, the manipulability is close to zero or the Jacobian of the robot loses rank (rank < 5), you should consider changing the robot's link lengths and offsets.

STATES = {"COORDINATE", "WAITING_FOR_TARGETS", "DONE" } #Allowd states 
Hz = 1
DT = 1/Hz 
PAUSE = 15 #sec to pause on point going
TRAJ_LENGTH = 3
D_START = 0.1



# (X, Y, Z) in mm: (DX, DY, DZ) i mm, error = actual - setpoint
calibration = {
    # Z = 50
    (575, 0, 60): (-25, 3, -26),
    (575, -250, 60): (-26, 2, -29),
    (575, 250, 60): (-27, 3, -31),
    (475, 0, 60):(-21, 5, -21),
    (475, -250, 60):(-31, 6, -28),
    (475,250,60):(-31, -8, -28),
    (350, 0, 60):(-28,9, -16),
    (350,-250,60):(-5,8,-4.5),
    (350,250,60):(1, 5,1),
    (575,0,100): (-29, 3, -27),
    
    # ... resten av rutenettet
    # Z = 150
    #(100, 100, 150): (3, -1, 4),
    # ...
}

pts = np.array(list(calibration.keys()), dtype=float) * 1e-3     # meter
err = np.array(list(calibration.values()), dtype=float) * 1e-3

rbf = RBFInterpolator(
    pts, err,
    kernel="thin_plate_spline",
    degree=1,          # lineær trend gir rolig ekstrapolering
    smoothing=1e-8,    # øk litt hvis målingene er støyete
)

def correct_position(x, y, z):
    dx, dy, dz = rbf([[x, y, z]])[0]
    return x - dx, y - dy, z - dz

# Setter opp robotens parameter
def buildRobot():
    robot = rtb.DHRobot(
        [
            rtb.RevoluteDH(d=0.197, a=0.0, alpha=pi / 2, qlim=[-110 * pi / 180, 110 * pi / 180]),
            rtb.RevoluteDH(a=0.29, d=0.0, offset = pi/2, qlim=[-95 * pi / 180, 90 * pi / 180]),
            rtb.RevoluteDH(a=0.34, d=0.0, qlim=[-95 * pi / 180, 90 * pi / 180]),
            rtb.RevoluteDH(a = 0.012, alpha=pi / 2, offset=pi / 2, qlim=[-100 * pi / 180, 80 * pi / 180]),
            rtb.RevoluteDH(d=0.072,  qlim=[-2 * pi, 2 * pi]),   
        ],
        name="My Config 1 Robot",
    )
    robot.tool = SE3(0.028, 0.0, 0.15)
    robot.qr = [0, 0, -pi/2, -pi/2, 0]
    return robot


class RobotNode(Node):
    def __init__(self) -> None:
        super().__init__("kinematics_node")

        self.robot = buildRobot()
        self.robot.q = self.robot.qr
        self.joint_names = [f"joint_{i + 1}" for i in range(self.robot.n)]
        self.traj = []

        self.target_points_found = False

        # State Machine Properties
        self.target_red = None
        self.target_green = None

        self.start = None
        self.end = None

        self.declare_parameter("enable_trajectory", False)
        self.traj_enable = self.get_parameter("enable_trajectory").value  # Trajetery if true
        
        self.declare_parameter("challenge_state", "COORDINATE")
        self.challenge_state = self.get_parameter("challenge_state").value

        self.create_subscription(PoseStamped, "/goal_point", self.subscriber_pos, 10)
        self.create_subscription(PoseStamped, "/goal_point_red", self.sub_red, 10)
        self.create_subscription(PoseStamped, "/goal_point_green", self.sub_green, 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.add_on_set_parameters_callback(self.parameter_change)
        self.create_timer(DT, self.timer_callback)

    def sub_red(self, msg: PoseStamped):
        p = msg.pose.position
        if not self.target_points_found:
            x, y, z = correct_position(p.x, p.y, p.z)
            self.target_red = SE3(x, y, z)

    def sub_green(self, msg: PoseStamped):
        p = msg.pose.position
        if not self.target_points_found:
            x, y, z = correct_position(p.x, p.y, p.z)
            self.target_green = SE3(x, y, z)

    def subscriber_pos(self, msg: PoseStamped):
        p = msg.pose.position
        x, y, z = correct_position(p.x, p.y, p.z)
        Tgoal = SE3(x, y, z)
        
        
        if self.traj_enable:
            TverticalGoal = SE3(Tgoal.t[0], Tgoal.t[1], Tgoal.t[2] + 0.15) #Stop point
            sol1 = self.posisjons(TverticalGoal)
            
            if not sol1:
                self.get_logger().error(f"No solution for ({TverticalGoal.t[0]:.2f}, {TverticalGoal.t[1]:.2f}, {TverticalGoal.t[2]:.2f})")
                return
            
            sol2 = self.posisjons(Tgoal, sol1.q, lastAngle=1)

            if not sol2:
                self.get_logger().error(f"No solution for ({Tgoal.t[0]:.2f}, {Tgoal.t[1]:.2f}, {Tgoal.t[2]:.2f})")
                return

            traj_up = rtb.jtraj(
                self.robot.q,
                sol1.q,
                TRAJ_LENGTH
            ).q

            traj_down = rtb.jtraj(
                sol1.q,
                sol2.q,
                TRAJ_LENGTH
            ).q

            self.traj = (
                list(traj_up)
                + [sol1.q] * PAUSE
                + list(traj_down)
            )

            self.get_logger().info(f"sol1.q: {np.round(sol1.q, 3)}")
            self.get_logger().info(f"sol2.q: {np.round(sol2.q, 3)}")

            self.get_logger().info(f"traj_up length: {len(traj_up)}")
            self.get_logger().info(f"traj_down length: {len(traj_down)}")

            self.get_logger().info(f"Final trajectory point: {self.traj[-1]}")

        else:
            sol = self.posisjons(Tgoal, lastAngle=0.3)
            if not sol:
                self.get_logger().error(f"No solution for ({Tgoal.t[0]:.2f}, {Tgoal.t[1]:.2f}, {Tgoal.t[2]:.2f})")
                sol = self.robot.ikine_LM(
                        Tgoal,
                        q0=self.robot.q,
                        mask=[1, 1, 1, 0, 0, 0.5],
                        tol=1e-6,  # Smaller tol => samller error
                        joint_limits=False,  # Take q.lim in consideration
                    )
                if not sol:
                    self.get_logger().error(f"No solution at all")
                                    
                    return
                self.get_logger().error(f"Vinkler: {np.round(np.rad2deg(sol.q), 0)}")
                return
            self.robot.q = sol.q
        self.get_logger().info("Goal accapted, moving to goal")


    def parameter_change(self, params):
        for p in params:
            if p.name == "challenge_state":
                if p.value not in STATES:
                    return SetParametersResult(successful = False, reason = f"Not leagal state. States : {sorted(STATES)}")
                self.challenge_state = p.value

                if p.value == "WAITING_FOR_TARGETS":
                    self.target_green = None
                    self.target_red = None
                    self.target_points_found = False
                    self.traj.clear()

                self.get_logger().info(f"challenge_state changed to'{p.value}'")
            if p.name == "enable_trajectory":
                if not isinstance(p.value, bool):
                    return SetParametersResult(successful = False, reason = f"Not booleon")
                self.traj_enable = p.value
                self.get_logger().info(f"traj changed to'{p.value}'")
           
        return SetParametersResult(successful = True)

    def timer_callback(self):

        # State Machine Logic
        if self.challenge_state == "WAITING_FOR_TARGETS":
            # Wait until vision node sees both balls
            if self.target_red and self.target_green:
                #self.robot.tool = SE3(0.028, 0.0, 0.07)
                self.target_points_found = True
                self.start, self.end = self.line(self.target_red, self.target_green)
                self.get_logger().info("Both targets locked. Calculating Red IK...")
                sol = self.robot.ikine_LM(
                        self.start,
                        q0=self.robot.q,
                        mask=[1, 1, 1, 0, 0.0, 0.8],
                        tol=1e-3,  # Smaller tol => samller error
                        joint_limits=True,  # Take q.lim in consideration
                    )
                if sol.success:
                    self.traj = list(rtb.jtraj(self.robot.q, sol.q, TRAJ_LENGTH).q) + [sol.q] * (int(round(PAUSE/DT))) 
                    self.challenge_state = "MOVING_TO_RED"
                else:
                    self.get_logger().warn(f"Red IK failed. {sol.residual}")
                    self.target_points_found = False

        elif self.challenge_state == "MOVING_TO_RED":
            # Once trajectory list is empty, motion to red is finished
            if not self.traj:
                self.get_logger().info("Red target hit. Calculating Green IK...")
                sol = self.robot.ikine_LM(
                        self.end,
                        q0=self.robot.q,
                        mask=[1, 1, 1, 0, 0.0, 0.8],
                        tol=1e-3,  # Smaller tol => samller error
                        joint_limits=True,  # Take q.lim in consideration
                    )
                if sol.success:

               #   T0 = self.robot.fkine(self.robot.q)
               #   Ts = rtb.ctraj(T0, self.end, TRAJ_LENGTH)
               #   qs, q = [], self.robot.q
               #   for T in Ts:
               #       sol = self.robot.ikine_LM(T, q0=q, mask=[1, 1, 1, 0, 0, 0], joint_limits=True)
               #       if not sol.success:
               #           self.get_logger().error("IK feila undervegs")
               #           return
               #       q = sol.q                            # førre løysing som startgjetting
               #       qs.append(q)

                    sol = self.robot.ikine_LM(self.end, q0=self.robot.q, mask=[1, 1, 1, 0, 0, 0], joint_limits=True)
                    if not sol.success:
                        self.get_logger().error("IK feila undervegs")
                        return
                    self.traj = sol.q



                    #self.traj = list(rtb.jtraj(self.robot.q, sol.q, TRAJ_LENGTH).q)
                    self.challenge_state = "MOVING_TO_GREEN"
                else:
                    self.get_logger().warn("Green IK failed.")

        elif self.challenge_state == "MOVING_TO_GREEN":
            # Once trajectory list is empty, motion to green is finished
            if not self.traj:
                self.get_logger().info("Green target hit. Challenge Complete.")
                self.challenge_state = "DONE"
                self.target_green = None
                self.target_red = None
                self.target_points_found = False
                self.traj.clear()

        elif self.challenge_state == "COORDINATE":
            self.robot.q = [-pi/2, 0, -pi/2, -pi/2, 0]
            self.challenge_state = "COORDINATE_MOVE"

        #elif self.challenge_state == "COORDINATE_MOVE":
            #self.get_logger().info("Waiting on new coordinate")

        
        # Publish step, and go to nex
        if self.traj:
            if self.traj_enable:
                self.robot.q = self.traj.pop(0)
            else:
                self.robot.q = self.traj[-1]
                self.traj.clear()

        # 3. Publish current state to RobotController
        self.publish_state()



    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(x) for x in self.robot.q]
        self.joint_pub.publish(msg)
        self.get_logger().debug(f"Publishing: {msg.position}")

    def no_leagal_point(self, T_all: SE3):
        for T in T_all[1:]:
            if T.t[2] <= 0:
                return True

        return False

    def posisjons(self, Tgoal, seed=None, lastAngle = 0):
        if seed is None:
            seed = self.robot.q

        sol = self.robot.ikine_LM(
            Tgoal,
            q0=seed,
            mask=[1, 1, 1, 0, 0, lastAngle],
            tol=1e-6,  # Smaller tol => samller error
            joint_limits=True,  # Take q.lim in consideration
           # search=True,
            slimit=1000,
            ilimit=1000,   
        )

        if sol.success and self.no_leagal_point(self.robot.fkine_all(sol.q)):
            self.get_logger().warn(f"Link throu the table, trying to move goal up")
            newTgoal = SE3(Tgoal.t[0], Tgoal.t[1], Tgoal.t[2] + 0.05)
            sol = self.posisjons(newTgoal, seed)
            if sol.success:
                sol = self.robot.ikine_LM(
                    Tgoal,
                    q0=sol.q,
                    mask=[1, 1, 1, 0, 0, lastAngle],
                    tol=1e-6,  # Smaller tol => samller error
                    joint_limits=True,  # Take q.lim in consideration
                    #search=True,
                    slimit=1000,
                    ilimit=1000,   
                )

        if sol:
            if not sol.success or self.no_leagal_point(self.robot.fkine_all(sol.q)):
                return None
        return sol

    def line(self, T_red, T_green):
        D_X = 0.0  # Flytt 5 cm mot negativ X

        red = T_red.t.copy()
        green = T_green.t.copy()

        # Flytt begge punktene i X-retning
        red[0] -= D_X
        green[0] -= D_X
        L = np.linalg.norm(green - red)

        v = (green - red) / L   # vector from red to green
        start = red - D_START * v
        end = green - D_START / 1 * v

        # X-akse horisontal
        x = np.array([v[0], v[1], 0.0])
        x /= np.linalg.norm(x)

        # Z-akse horisontal og vinkelrett på X
        z = np.cross(x, [0, 0, 1])
        z /= np.linalg.norm(z)

        # Y-akse vertikal
        y = np.cross(z, x)
        y /= np.linalg.norm(y)

        R = np.column_stack((x, y, z))

        Tstart = SE3.Rt(R, start)
        Tend = SE3.Rt(R, end)
        return Tstart, Tend


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


if __name__ == "__main__":
    main()

