import rclpy
from rclpy.node import Node
import cv2
import apriltag
from Parameter_msgs.srv import Cam
from Parameter_msg.srv import Datasetdataadd
import os

class QRCodeScannerNode(Node):
    def __init__(self):
        super().__init__('code_scanner')
        self.get_logger().info('Starting code scanner')
        
        # 카메라를 초기화하고 항상 켜둠
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam.")
        
        
        # AprilTag Detector 설정 (16h5 태그 패밀리 사용)
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))

        # QR 버튼 서버: 신호 오면 QR 인식  cam on
        self.srv = self.create_service(Cam, 'cam_on', self.scan_qr_code) 
        
        #저장된 데이터를 데이터 베이스한테 전송 (더하기 위해서)
        self.client = self.create_client(Datasetdataadd,'add_data')
        self.request = Datasetdataadd.Request()
        
        # 스크린샷 저장 디렉토리 설정
        self.screenshot_dir = "/home/sineunji/ros2_ws/src/storagy_exhibition_ui/database"
        os.makedirs(self.screenshot_dir, exist_ok=True)
        self.frame_count = 0
    
    # AprilTag 데이터와 이미지를 데이터베이스에 전송 request <- 데이터 정보 넣기     
    def send_request(self, a, b):
        self.request.id = str(a)
        with open(b, 'rb') as image_file:
            self.request.png = image_file.read()
        self.future = self.client.call_async(self.request)
        return self.future
    
   
    #캠 킬지 말지 여부 서버 받음
    def scan_qr_code(self, request, response):
        
        self.get_logger().info(f'{request.camera_qr}')
        if request.camera_qr and self.cap.isOpened():
            count = 0
            #초기화 
            qr_image = None
            qr_data = None
            while(True):
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error("Failed to read from webcam.")
                    return None
                
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                results = self.detector.detect(gray)
                # ArUco 마커 탐지
                if results :
                    
                    r = results[0]
                    # 태그 ID와 위치 출력
                    print(f"Tag ID: {r.tag_id}")
                    print(f"Tag Center: {r.center}")
                    print(f"Tag Corners: {r.corners}")
                
                    # 태그의 각 코너에 원 그리기
                    for pt in r.corners:
                        pt = (int(pt[0]), int(pt[1]))
                        cv2.circle(frame, pt, 5, (0, 255, 0), -1)

                    # 태그의 중심에 원 그리기
                    center = (int(r.center[0]), int(r.center[1]))
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    if r.tag_id is not None:
                        qr_image = os.path.join(self.screenshot_dir, f"screenshot_frame_{self.frame_count}.png")
                        cv2.imwrite(qr_image, frame)  
                        self.get_logger().info(f'{r.tag_id}')
                        qr_data =str(r.tag_id)               
                        count = count + 1
                
                #코드가 인식된 프레임을 잠시 보여줌
                cv2.imshow("marker", frame)
                #cv2 정해진 공간 
                cv2.moveWindow("marker", 100, 100)
                cv2.waitKey(100)  

                #aruco 데이터가 있으면 while 문을 벗어나서 서비스 보냄
                if (qr_image and qr_data) and count > 80 :
                    break
            
            #사진 저장하기 위한 카운트 
            self.frame_count = self.frame_count + 1
            cv2.destroyAllWindows()
            # 창을 닫음
            # Tag 데이터와 이미지를 데이터베이스에 저장하기 위해 client 요청 정보 <= qr 데이터 정보 넣기 
            future =self.send_request(qr_data, qr_image)
            rclpy.spin_until_future_complete(self, future) 
            self.get_logger().info(f'보내기 성공함 {future.result()}')
        else:
            self.get_logger().info('Service not available, waiting again...')
            
def main(args=None):
    rclpy.init(args=args)
    node = QRCodeScannerNode()
    if node.cap.isOpened():  # 카메라가 성공적으로 열렸을 때만 spin
        rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    



