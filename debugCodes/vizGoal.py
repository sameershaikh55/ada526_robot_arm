#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


class GoalPointPlotter(Node):
    def __init__(self):
        super().__init__("goal_point_plotter")
        self.create_subscription(PoseStamped, "/goal_point", self.subscriber_pos, 10)

        self.lock = threading.Lock()
        self.pos = None      # (x, y, z)
        self.dirty = False   # ny data tilgjengelig

    def subscriber_pos(self, msg: PoseStamped):
        p = msg.pose.position
        with self.lock:
            self.pos = (p.x, p.y, p.z)
            self.dirty = Truethreading


def main():
    rclpy.init()
    node = GoalPointPlotter()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("/goal_point")

    lim = 1.0
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_box_aspect((1, 1, 1))

    # Selve punktet
    (point,) = ax.plot([0], [0], [0], marker="o", markersize=10,
                       color="black", linestyle="None")

    # Hjelpelinjer fra punktet til hver vegg (x = rød, y = grønn, z = blå)
    (line_x,) = ax.plot([0, -lim], [0, 0], [0, 0], "--", color="red", lw=1.5)
    (line_y,) = ax.plot([0, 0], [0, -lim], [0, 0], "--", color="green", lw=1.5)
    (line_z,) = ax.plot([0, 0], [0, 0], [0, -lim], "--", color="blue", lw=1.5)

    # Tekstlapp ved punktet
    label = ax.text(0, 0, 0, "", fontsize=10,
                    bbox=dict(boxstyle="round", fc="white", alpha=0.8))

    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            with node.lock:
                if node.dirty and node.pos is not None:
                    x, y, z = node.pos
                    node.dirty = False
                else:
                    x = None

            if x is not None:
                point.set_data([x], [y])
                point.set_3d_properties([z])

                # Linjer fra punktet til veggene x=-lim, y=-lim, z=-lim
                line_x.set_data([x, -lim], [y, y])
                line_x.set_3d_properties([z, z])
                line_y.set_data([x, x], [y, -lim])
                line_y.set_3d_properties([z, z])
                line_z.set_data([x, x], [y, y])
                line_z.set_3d_properties([z, -lim])

                label.set_position((x, y))
                label.set_3d_properties(z + 0.08, zdir="z")
                label.set_text(f"x={x:.2f}\ny={y:.2f}\nz={z:.2f}")

                ax.set_title(f"/goal_point: ({x:.2f}, {y:.2f}, {z:.2f}) m")

            fig.canvas.draw_idle()
            plt.pause(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()