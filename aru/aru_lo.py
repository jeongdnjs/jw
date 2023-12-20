import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoMarkerFinder:
    def __init__(self):
        # 웹캠 설정
        self.cap = cv2.VideoCapture(0)
        # ArUco 마커 사전 설정
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()

    def find_aruco_marker(self):
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            marker_corners, marker_ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

            if marker_ids is not None:
                for index, marker_id in enumerate(marker_ids):
                    corners = marker_corners[index].reshape((4, 2))
                    int_corners = np.int0(corners)
                    cv2.polylines(frame, [int_corners], True, (0, 255, 0), 2)

                    cX, cY = np.mean(corners, axis=0)
                    marker_size = np.linalg.norm(corners[0] - corners[2])

                    # 마커 ID 및 정보 화면에 표시
                    cv2.putText(frame, f'ID: {marker_id[0]}', (int(cX), int(cY)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    cv2.putText(frame, f'cX: {cX:.2f}, Marker Size: {marker_size:.2f}', (int(cX), int(cY) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            cv2.imshow('Aruco Marker Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    finder = ArucoMarkerFinder()
    finder.find_aruco_marker()
