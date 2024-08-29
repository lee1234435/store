import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class human_detection_yolov8n(Node):
    def __init__(self):
        super().__init__('human_detection_yolov8n')
        
        # ROS 2 구독자 생성
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # CvBridge 초기화
        self.bridge = CvBridge()

        # YOLOv8n 모델 로드
        self.model = YOLO('/home/storagy/Desktop/PINKLAB/yolov8n.pt')

        # OpenCV 창 생성
        cv2.namedWindow("RGB Image", cv2.WINDOW_NORMAL)
        
        # 이미지 저장을 위한 변수 초기화
        self.cv_image = None

        # 마우스 이벤트 콜백 함수 설정
        cv2.setMouseCallback("RGB Image", self.mouse_callback)
        
    def image_callback(self, msg):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLOv8 모델을 사용하여 사람 감지
        results = self.model(self.cv_image)[0]

        # 결과를 이미지에 그리기
        annotated_image = results.plot()

        # OpenCV 윈도우에 이미지 표시
        cv2.imshow("RGB Image", annotated_image)
        
        # 키 입력 대기
        cv2.waitKey(1)
        
    def mouse_callback(self, event, x, y, flags, param):
        # 왼쪽 버튼 클릭 시 좌표 출력
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.cv_image is not None:
                self.get_logger().info(f"Mouse clicked at: x={x}, y={y}")
    
    def destroy_node(self):
        # OpenCV 창 닫기
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = human_detection_yolov8n()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
