import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sqlite3
from Parameter_msg.srv import Datasetdataadd

class DatabaseNode(Node):

    def __init__(self):
        super().__init__('database_node')
       
        #등록 후 앞으로 이동 
        self.publisher_ = self.create_publisher(Int32, '/arUco_tracking', 5)     
        self.timer = self.create_timer(1, self.query_database)
        
        #데이터베이스 추가
        self.srv = self.create_service(Datasetdataadd, 'add_data', self.add_data_callback)
        self.conn = None
        self.cursor = None 
       
        # 도착한 후에 경로값 (user_id, track)
        

        # 데이터베이스 연결 함수 
    def connect_db(self):
        conn = sqlite3.connect('/home/sineunji/ros2_ws/src/storagy_exhibition_ui/database/id.db')
        cursor = conn.cursor()
        return conn, cursor  
        
       # data추가함      
    def add_data_callback(self, request, response):
        self.conn, self.cursor=self.connect_db()    
        data = request.id
        image = request.png
        
        try:
            self.cursor.execute("INSERT INTO (user_id, qr_image, created_at) VALUES (?, ?, datetime('now'))", 
                                (data, image))
            self.conn.commit()
            self.get_logger().info(f'Inserted  data and image into database: {data}')
            response.datastore = True
            return response
            
        except sqlite3.IntegrityError:
            self.get_logger().info("data already exists in the database.")
            response.datastore = False
            return response
        finally :
            self.conn.close()


        # 쿼리 실행: ID가 존재하는지 확인 -> 앞으로 먼저 이동
    def query_database(self):
        # 데이터베이스 연결
        self.conn, self.cursor = self.connect_db()
        #존재하는가? 
        target_id = "20"
        query = "SELECT EXISTS(SELECT 1 FROM qr_codes WHERE user_id = ?)"
        self.cursor.execute(query, (target_id,))
        
        # 쿼리 결과를 한 번만 가져옵니다.
        result = self.cursor.fetchone()  # 쿼리 결과를 한 번만 가져옵니다.
        msg = Int32()
        if (result is not None and result[0]) == 1:  # EXISTS 쿼리는 1 또는 0을 반환합니다.
           msg.data = int(target_id)  # ID가 존재하는 경우
           self.publisher_.publish(msg)
        else:
            msg.data = 0
            self.publisher_.publish(msg)
        
        # 데이터베이스 연결 종료
        self.conn.close()
        self.get_logger().info(f'보냄: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    node = DatabaseNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
