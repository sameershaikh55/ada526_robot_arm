import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class RobotNode(Node):
    def __init__(self):
        super().__init__('kordniate_node')

        self.joint_pub = self.create_publisher(PoseStamped, "/goal_point", 10)
        self.scale = 0.025




    def publish_goal(self, x : str, y, z):
     
        goal_pt = PoseStamped()
        goal_pt.header.stamp = self.get_clock().now().to_msg()
        goal_pt.header.frame_id = 'base_link'
        yAxie = round(float(ord(y.upper()) - 64 - 11) * self.scale,3)
        xAxie = round((24 - float(x)) * self.scale,3)

        goal_pt.pose.position.x = float(xAxie)
        goal_pt.pose.position.y = float(yAxie)
        goal_pt.pose.position.z = float(z)
        goal_pt.pose.orientation.w = 1.0

        self.joint_pub.publish(goal_pt)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        while rclpy.ok():
            text = input("A,1 (q to quit): ").strip()
            if text.lower() == 'q':
                break
            try:
                y, x = map(str, text.replace(',', ' ').split())
            except ValueError:
                print("Invalid inpt")
                print()
                continue
            node.publish_goal(x, y, 0.06)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()