import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class SimpleFollower(Node):
    def __init__(self):
        super().__init__('simple_follower')

        self.person_x = None
        self.person_size = 0.0
        self.obstacle_dist = 10.0

        self.create_subscription(Float32, '/person_x', self.x_cb, 10)
        self.create_subscription(Float32, '/person_size', self.size_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Simple Follower Started")

    def x_cb(self, msg):
        self.person_x = msg.data

    def size_cb(self, msg):
        self.person_size = msg.data

    def scan_cb(self, msg):
        valid = [r for r in msg.ranges if 0.12 < r < 3.0]
        self.obstacle_dist = min(valid) if valid else 3.0

    def control_loop(self):
        cmd = Twist()

        if self.person_size < 1.0:
            self.cmd_pub.publish(cmd)
            return

        # STOP if obstacle too close
        if self.obstacle_dist < 0.5:
            self.get_logger().warn("Obstacle detected - stopping")
            self.cmd_pub.publish(cmd)
            return

        # Maintain ~1 meter distance
        if self.person_size < 140:
            cmd.linear.x = 0.20
        elif self.person_size > 190:
            cmd.linear.x = -0.10
        else:
            cmd.linear.x = 0.0

        # Center person
        img_center = 320 / 2
        error = img_center - self.person_x
        cmd.angular.z = -0.003 * error

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = SimpleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
