import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class ArucoMarkerPublisher(Node):
    def __init__(self, name):
        super().__init__('aruco_marker_publisher')
        self.publisher = self.create_publisher(String, 'aruco_marker_id', 10)
        self.name = name
        self.timer = self.create_timer(5, self.publish_marker_id)

    def publish_marker_id(self):
        msg = String()
        msg.data = self.name
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {self.name}')

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: python3 aruco_node.py <name>")
        return

    name = sys.argv[1]
    aruco_marker_publisher = ArucoMarkerPublisher(name)

    try:
        rclpy.spin(aruco_marker_publisher)
    except KeyboardInterrupt:
        pass

    aruco_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
