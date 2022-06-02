from rclpy.node import Node
from std_msgs.msg import String
import sys
import rclpy
class Subscriber(Node):
    def __init__(self):
        super().__init__('sub')
        self.pub = self.create_subscription(String, 'wheel_distance', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f"{msg}")

def main(args=None):
    rclpy.init(args=args)
    gps = Subscriber()
    try:
        rclpy.spin(gps)
    except KeyboardInterrupt:
        gps.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)

