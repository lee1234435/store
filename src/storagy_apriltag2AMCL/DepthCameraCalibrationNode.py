import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import yaml

class DepthCameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('depth_camera_calibration_node')
        
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        # 체커보드의 가로와 세로 내 점 개수 설정 (내부 코너 수)
        self.chessboard_size = (9,6)
        self.square_size  = 0.015  # 각 정사각형의 크기 (미터 단위)

        # 카메라 이미지 토픽 구독 (Color 이미지)
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_image_callback,
            10)

        # Depth 카메라 이미지 토픽 구독 (Depth 이미지)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback,
            10)
        
        # 캘리브레이션용 데이터 초기화
        self.objpoints = []
        self.imgpoints = []

        # 3D 공간에서의 체커보드 패턴 좌표 준비
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size

    def color_image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_calibration()

    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        self.process_calibration()

    def process_calibration(self):
        if self.color_image is None or self.depth_image is None:
            return

        gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

        # 실시간으로 카메라 화면을 확인할 수 있도록 창을 띄움
        cv2.imshow('Calibration Image', self.color_image)

        if ret:
            # 체커보드 코너를 이미지에 표시
            cv2.drawChessboardCorners(self.color_image, self.chessboard_size, corners, ret)
            cv2.imshow('Calibration Image', self.color_image)

            if cv2.waitKey(1) & 0xFF == ord('s'):
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners)
                self.get_logger().info(f"Chessboard detected and saved. Total saved images: {len(self.imgpoints)}")

                if len(self.imgpoints) > 25:  # 15장이 넘으면 캘리브레이션을 진행
                    self.perform_calibration()

        cv2.waitKey(10)

    def perform_calibration(self):
        gray_shape = self.color_image.shape[::-1][1:]
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray_shape, None, None)

        if ret:
            self.get_logger().info(f"Calibration successful. Camera matrix: {camera_matrix}")
            self.save_calibration_data(camera_matrix, dist_coeffs)
        else:
            self.get_logger().warn("Calibration failed.")

        cv2.destroyAllWindows()

    def save_calibration_data(self, camera_matrix, dist_coeffs, filename="calibration_data.yaml"):
        data = {
            'camera_matrix': camera_matrix.tolist(),
            'dist_coeffs': dist_coeffs.tolist()
        }
        with open(filename, 'w') as f:
            yaml.dump(data, f)
        self.get_logger().info(f"Calibration data saved to {filename}")

    def load_calibration_data(self, filename="calibration_data.yaml"):
        with open(filename) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            camera_matrix = np.array(data['camera_matrix'])
            dist_coeffs = np.array(data['dist_coeffs'])
        self.get_logger().info(f"Loaded calibration data from {filename}")
        return camera_matrix, dist_coeffs


def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
