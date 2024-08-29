import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import apriltag
import yaml

class ArucoCoordinateNode(Node):
    def __init__(self):
        """초기 설정"""
        super().__init__('aruco_detector_node')

        # subscriber 생성 (RGB 이미지, Depth 이미지)
        self.image_subscription = self.create_subscription(Image, '/camera/color/image_raw', self.Image_callback, 10)
        self.depth_subscription = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # publisher 생성 (Tag 중심좌표, Tag 픽셀 좌표계)
        self.center_publisher = self.create_publisher(Float32MultiArray, 'aruco/center', 10)
        self.coordinate_publisher = self.create_publisher(Float32MultiArray, 'aruco/coordinate', 10)

        # 이미지 대기상태 로그를 출력하기 위한 타이머 (1초 간격)
        self.log_timer = self.create_timer(1.0, self.log_message)

        # 토픽 메시지 저장 변수 선언
        self.rgb_image = None
        self.depth_image = None

        # OpenCV 객체 생성
        self.bridge = CvBridge()

        # apriltag 감지기 생성
        self.detector = apriltag.apriltag(family='tag16h5')
        # 태그 ID는 '20'으로 지정
        self.target_id = 20

        # 카메라 캘리브레이션 데이터 로드
        self.load_calibration_data()

        # publish 처리 주기 (0.01초 간격으로 처리)
        self.timer = self.create_timer(0.01, self.process_image)
    
    def load_calibration_data(self):
        """캘리브레이션 데이터 로드"""

        # yaml파일에서 camera_matrix, dist_coeffs 가져오기
        with open('/home/k/Desktop/store/src/storagy_apriltag_tracking/config/calibration_data.yaml', 'r') as file:
            calibration_data = yaml.safe_load(file)
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.dist_coeffs = np.array(calibration_data['dist_coeffs'])

    def Image_callback(self, msg):
        """RGB 이미지 토픽 메시지 수신"""

        # RGB 이미지를 저장
        self.rgb_image = msg
    
    def depth_callback(self, msg):
        """Depth 이미지 토픽 메시지 수신"""

        # Depth 이미지를 OpenCV 이미지로 변환 후 저장
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
    def log_message(self):
        """이미지가 subscribe 될 때까지 대기상태 메시지 출력"""

        # RGB 이미지가 들어오지 않을 때의 메시지 출력
        if self.rgb_image is None:
            self.get_logger().info('RGB_Image is not subscribed, waiting again...')
        # Depth 이미지가 들어오지 않을 때의 메시지 출력
        elif self.depth_image is None:
            self.get_logger().info('Depth_Image is not subscribed, waiting again...')
        # 두 이미지 다 들어오면 pass
        else:
            pass
    
    def process_image(self):
        """RGB Image, Depth Image에서 태그 중심좌표, 픽셀 좌표계 계산 후 publish"""

        # RGB 이미지, Depth 이미지가 모두 들어왔을 때에만 이후 코드 진행
        if self.rgb_image is None or self.depth_image is None:
            return
        
        # RGB 이미지를 grayscale로 변환
        frame = self.bridge.imgmsg_to_cv2(self.rgb_image, "bgr8")
        # 왜곡 보정 수행
        undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        # 보정된 이미지를 grayscale로 변환
        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        # grayscale에서 apriltag = 'tag16h5'인 태그 인식
        detections = self.detector.detect(gray)

        for detection in detections:
            # 태그 ID = 20 인지 체크
            if detection['id'] == self.target_id:
                # 태그의 4개 꼭짓점 좌표를 4개 변수에 할당
                corners = detection['lb-rb-rt-lt']
                (bottomLeft, bottomRight, topRight, topLeft) = corners

                # 각 코너의 x,y 좌표를 이용해서 태그 중심좌표 계산
                cX = float((topLeft[0] + topRight[0] + bottomRight[0] + bottomLeft[0]) / 4.0)
                cY = float((topLeft[1] + topRight[1] + bottomRight[1] + bottomLeft[1]) / 4.0)

                # cX, cY가 이미지 경계 안에 있도록 제한
                cX = min(max(int(cX), 0), self.depth_image.shape[1] - 1)
                cY = min(max(int(cY), 0), self.depth_image.shape[0] - 1)

                # cX, cY를 기준으로 주변 영역(5x5 픽셀) 추출
                # 주변 여러 픽셀의 평균을 사용해서 신뢰할 수 있는 depth 값 계산
                # 단일 픽셀 depth값은 노이즈, 기타 불확실성 영향
                depth_window = self.depth_image[max(0, int(cY)-2):min(int(cY)+3, self.depth_image.shape[0]),
                                                        max(0, int(cX)-2):min(int(cX)+3, self.depth_image.shape[1])]

                # 5x5 픽셀들의 평균 depth 값 계산 
                # 밀리미터 단위 depth를 미터 단위로 변환
                distance = np.nanmean(depth_window) / 1000.0

                # Depth 값에 Nan이 들어있거나 0으로 나올 때 유효하지 않은 depth 값으로 판단하고 예외 처리
                if np.isnan(distance) or distance <= 0:
                    self.get_logger().info(f"Invalid depth value at ({cX:.2f}, {cY:.2f}, {distance:.2f})")
                    continue

                # 카메라 매트릭스에서 중심점과 초점 거리 가져오기
                cx = self.camera_matrix[0, 2]
                cy = self.camera_matrix[1, 2]
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]

                # 픽셀 좌표 cX, cY를 월드 좌표계의 미터 단위로 변환
                worldX = (cX - cx) * distance / fx
                worldY = (cY - cy) * distance / fy

                # 마커 (x,y,z) 좌표 publish
                coordinate_msg = Float32MultiArray()
                coordinate_msg.data = [float(worldX), float(worldY), float(distance)]
                self.coordinate_publisher.publish(coordinate_msg)
                self.get_logger().info(f'X : {worldX:.2f}, Y : {worldY:.2f}, Z : {distance:.2f}')

                center_msg = Float32MultiArray()
                center_msg.data = [float(cX), float(cY)]
                self.center_publisher.publish(center_msg)
                self.get_logger().info(f'cX : {cX:.2f}, cY : {cY:.2f}')

        # RGB, Depth 이미지 초기화
        self.rgb_image = None
        self.depth_image = None
        
def main(args=None):
    """ArucoCoordinateNode 클래스 반복 실행"""

    rclpy.init(args=args)
    node = ArucoCoordinateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()