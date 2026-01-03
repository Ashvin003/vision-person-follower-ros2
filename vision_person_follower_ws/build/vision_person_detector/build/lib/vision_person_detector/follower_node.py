import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class SimpleFollower(Node):
    def __init__(self):
        super().__init__('simple_follower')

        # Parameters
        self.target_distance = 1.0      # meters
        self.max_speed = 0.15
        self.search_turn_speed = 0.3
        self.obstacle_distance = 0.5

        # State
        self.person_size = 0.0
        self.person_seen = False
        self.obstacle_close = False

        # Subscribers
        self.create_subscription(Float32, '/person_size', self.person_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("✅ Simple Follower Started")

    def person_cb(self, msg):
        self.person_size = msg.data
        self.person_seen = True

    def scan_cb(self, msg):
        min_dist = min(msg.ranges)
        self.obstacle_close = min_dist < self.obstacle_distance

    def control_loop(self):
        cmd = Twist()

        # ----------------------------
        # CASE 1: PERSON IS DETECTED
        # ----------------------------
        if self.person_seen:

            # Convert bbox size to distance proxy
            # (bigger bbox = closer person)
            if self.person_size > 200:
                # TOO CLOSE → STOP (NO TURNING)
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            elif self.person_size < 120:
                # TOO FAR → MOVE FORWARD
                cmd.linear.x = self.max_speed
                cmd.angular.z = 0.0

            else:
                # PERFECT DISTANCE → STOP
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            # IMPORTANT: ignore obstacle logic when person exists
            self.cmd_pub.publish(cmd)
            self.person_seen = False
            return

        # ----------------------------
        # CASE 2: NO PERSON → SEARCH
        # ----------------------------
        if not self.person_seen:
            cmd.linear.x = 0.0
            cmd.angular.z = self.search_turn_speed
            self.get_logger().info("🔍 Searching for person...")

        # ----------------------------
        # CASE 3: OBSTACLE AVOIDANCE (ONLY WHEN NO PERSON)
        # ----------------------------
        if self.obstacle_close:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.4
            self.get_logger().warn("⚠️ Obstacle detected - turning")

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = SimpleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
