from rclpy.node import Node
from std_msgs.msg import String
import sys
import rclpy
class Publisher(Node):
    def __init__(self):
        super().__init__('pub')
        self.pub = self.create_publisher(String, 'wheel_distance', 10)
        self.rate = .05
        self.timer = self.create_timer(self.rate, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        self.count += 1
        msg.data = f"data {self.count}"
        self.get_logger().info(f"publishing {msg}")
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    gps = Publisher()
    try:
        rclpy.spin(gps)
    except KeyboardInterrupt:
        gps.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)

