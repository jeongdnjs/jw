import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoMarkerFinder(Node):
    def __init__(self):
        super().__init__('aruco_marker_finder')
        self.subscription = self.create_subscription(
            String, 
            'aruco_marker_id', 
            self.marker_callback,
            10)
        self.publisher = self.create_publisher(String, 'robot_command', 10)
        self.subscription
        self.current_marker_id = None

    def marker_callback(self, msg):
        name_to_id = {
            'zero': 0,
            'one': 1,
            'two': 2,
            'three': 3,
            'four': 4
        }
        self.current_marker_id = name_to_id.get(msg.data)
        if self.current_marker_id is not None:
            self.get_logger().info(f'Looking for ArUco Marker ID: {self.current_marker_id}')
        else:
            self.get_logger().info('Invalid marker name received')

    def find_aruco_marker(self):
        cap = cv2.VideoCapture(0)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        aruco_params = aruco.DetectorParameters()
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

        while True:
            rclpy.spin_once(self, timeout_sec=0)

            ret, frame = cap.read()
            if not ret:
                break

            marker_detected = False
            marker_corners, marker_ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

            if marker_ids is not None and self.current_marker_id in marker_ids:
                marker_detected = True
                index = list(marker_ids).index(self.current_marker_id)
                corners = marker_corners[index].reshape((4, 2))
                int_corners = np.int0(corners)
                cv2.polylines(frame, [int_corners], True, (0, 255, 0), 2)

                cX, cY = np.mean(corners, axis=0)
                marker_size = np.linalg.norm(corners[0] - corners[2])

                command = self.determine_robot_command(cX, marker_size, frame_width)
                self.publisher.publish(String(data=command))

                # 토픽 명령어 화면에 표시
                cv2.putText(frame, f'Command: {command}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            if not marker_detected:
                self.publisher.publish(String(data="no_marker"))

            cv2.imshow('Aruco Marker Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def determine_robot_command(self, cX, marker_size, frame_width):
        center_threshold = frame_width // 2
        size_threshold = 100

        if cX < center_threshold - 50:
            return 'left'
        elif cX > center_threshold + 50:
            return 'right'
        elif marker_size > size_threshold:
            return 'stop'
        else:
            return 'go'

def main(args=None):
    rclpy.init(args=args)
    aruco_marker_finder = ArucoMarkerFinder()

    try:
        rclpy.spin(aruco_marker_finder)
        aruco_marker_finder.find_aruco_marker()
    except KeyboardInterrupt:
        pass

    aruco_marker_finder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
