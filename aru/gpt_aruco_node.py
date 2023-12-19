import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ArucoMarkerPublisher(Node):
    def __init__(self):
        super().__init__('aruco_marker_publisher')
        self.publisher = self.create_publisher(String, 'aruco_marker_id', 10)
        self.names = ['so', 'ma', 'yeon', 'lee', 'soo']
        self.timer = self.create_timer(5, self.publish_marker_id)

    def publish_marker_id(self):
        name = self.names.pop(0)  # 리스트의 첫 번째 요소를 가져오고 제거
        msg = String()
        msg.data = name
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {name}')
        self.names.append(name)  # 리스트의 끝에 요소를 추가

def main(args=None):
    rclpy.init(args=args)

    aruco_marker_publisher = ArucoMarkerPublisher()

    try:
        rclpy.spin(aruco_marker_publisher)
    except KeyboardInterrupt:
        pass

    aruco_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
