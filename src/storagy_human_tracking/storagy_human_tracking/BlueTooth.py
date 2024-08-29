import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import bluetooth
import time

class BlueTooth(Node):

    def __init__(self):
        super().__init__('BlueTooth')
        self.publisher_ = self.create_publisher(Bool, '/activation_signal', 10)
        self.timer = self.create_timer(0.5, self.receive_data)
        self.target_address = "38:8F:30:6D:A8:E5"  # 블루투스 장치 주소
        # self.target_address = "38:8F:30:6D:A8:E5"  # 블루투스 장치 
        self.sock = None  # 초기화 시 None으로 설정
        self.connect_bluetooth()

    def connect_bluetooth(self):
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.sock.settimeout(10)  # 타임아웃 설정

        try:
            self.sock.connect((self.target_address, 4))
            self.get_logger().info("First Connected to Bluetooth device")
        except bluetooth.btcommon.BluetoothError as e:
            self.get_logger().error(f"Failed to connect to Bluetooth device: {e}")
            self.sock = None

    def receive_data(self):
        if self.sock is None:
            self.get_logger().info("Socket is not connected, attempting to reconnect...")
            self.reset_bluetooth_connection()
            return

        try:
            # data = self.sock.recv(1024)
            data_flag = self.sock.recv(1024)
            self.get_logger().info("New Socket ==> Re-Connected to Bluetooth device")
            if data_flag:
                msg = Bool()
                msg.data = True  # 블루투스 신호가 있을 때 True로 설정
                self.publisher_.publish(msg)
                self.get_logger().info(f"Received Bluetooth data, publishing activation signal: {msg.data}")
        except bluetooth.btcommon.BluetoothError as e:
            self.get_logger().error(f"Bluetooth connection lost: {e}")
            self.sock.close()
            self.sock = None
            self.publish_deactivation()
            self.reset_bluetooth_connection()  # Bluetooth 재연결 시도

    def publish_deactivation(self):
        msg = Bool()
        msg.data = False  # 블루투스 신호가 없을 때 False로 설정
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing deactivation signal: {msg.data}")

    def reset_bluetooth_connection(self):
        self.get_logger().info("Attempting to reconnect to Bluetooth device...")
        if self.sock:
            self.sock.close()  # 기존 소켓 닫기
            self.sock = None  # 소켓 변수를 None으로 설정

        # 새로운 소켓 생성
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.sock.settimeout(10)  # 재연결 소켓에도 타임아웃 설정
        
        for attempt in range(10):  # 최대 100회 재연결 시도
            try:
                self.sock.connect((self.target_address, 4))
                self.get_logger().info("Reconnected to Bluetooth device")
                return  # 성공적으로 재연결되면 종료
            except bluetooth.btcommon.BluetoothError as e:
                self.get_logger().error(f"Reconnection attempt {attempt + 1} failed: {e}")
                time.sleep(2)  # 2초 후에 재시도

        self.get_logger().error("Failed to reconnect to Bluetooth device after multiple attempts")

    def destroy_node(self):
        if self.sock:
            self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BlueTooth()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
