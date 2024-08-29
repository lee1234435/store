import cv2
import numpy as np
from flask import Flask, request, render_template, redirect, url_for, jsonify
import sqlite3
from ultralytics import YOLO
import base64
import threading
import time
import subprocess
import logging 

app = Flask(__name__)

# YOLOv8n 모델 로드
model = YOLO('src/storagy_calculator_Web_Flask/models/product.pt')

# 로거 설정
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def get_db_connection():
    """
    데이터베이스 연결용
    """
    conn = sqlite3.connect('src/storagy_calculator_Web_Flask/database/database.db')
    conn.row_factory = sqlite3.Row
    return conn

def process_image(image):
    """
    yolov8n 모델 클래스 인식 및 이름 split 후 저장
    """
    results = model(image)
    detected_items = []

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            conf = box.conf[0]
            if conf > 0.5:
                class_id = int(box.cls[0].item())
                detected_item_str = result.names[class_id]
                print(f"Detected item string: {detected_item_str}")
                logging.info(f"Detected item string: {detected_item_str}")

                parts = detected_item_str.split('-')
                print(f"Parts after split: {parts}")
                logging.info(f"Parts after split: {parts}")

                if len(parts) == 5:
                    name = parts[0]
                    price = parts[1]
                    year = parts[2]
                    month = parts[3].zfill(2)
                    day = parts[4].zfill(2)
                    expiry_date = f"{year}-{month}-{day}"
                    detected_items.append({
                        'name': name,
                        'price': int(price),
                        'expiry_date': expiry_date,
                        'quantity': 1
                    })
                else:
                    error_message = f"Item data is not in the expected format: {detected_item_str}. Error: Unexpected number of parts in the item data"
                    print(error_message)
                    logging.error(error_message)

    return detected_items

def call_ros2_service():
    """ 
    terminal에서 ros2 sevice 호출 후 로봇에게 전송
    (Call the ROS2 service to signal the robot to move.)
    """
    try:
        print("ROS2 서비스 호출을 시작합니다.")
        logging.info("ROS2 서비스 호출을 시작합니다.")
        
        result = subprocess.run(
            ['ros2', 'service', 'call', '/start_parking', 'std_srvs/srv/Empty'],
            capture_output=True, 
            text=True
        )
        
        print(f"ROS2 서비스 응답: {result.stdout}")
        logging.info(f"ROS2 서비스 응답: {result.stdout}")
        
        return result.stdout
    except Exception as e:
        error_message = f"ROS2 서비스 호출 중 오류 발생: {e}"
        print(error_message)
        logging.error(error_message)
        return str(e)

@app.route('/')
def index():
    """
    index.html 연결 파트
    """
    
    conn = get_db_connection()
    items = conn.execute('SELECT * FROM items').fetchall()
    conn.close()
    return render_template('index.html', items=items)

@app.route('/update', methods=['POST'])
def update_item():
    """
    database 업데이트
    데이터를 기반으로 데이터베이스를 업데이트
    """
    name = request.form['name']
    price = float(request.form['price'])
    expiry_date = request.form['expiry_date']
    quantity = int(request.form['quantity'])

    conn = get_db_connection()
    
    existing_item = conn.execute('SELECT * FROM items WHERE name = ?', (name,)).fetchone()

    if existing_item:
        new_quantity = existing_item['quantity'] + quantity
        conn.execute('''
            UPDATE items
            SET price = ?, expiry_date = ?, quantity = ?
            WHERE name = ?
        ''', (price, expiry_date, new_quantity, name))
    else:
        conn.execute('''
            INSERT INTO items (name, price, expiry_date, quantity)
            VALUES (?, ?, ?, ?)
        ''', (name, price, expiry_date, quantity))

    conn.commit()
    conn.close()

    return redirect(url_for('index'))

@app.route('/process_image', methods=['POST'])
def process_image_route():
    """
    로컬 주소 웹에 연동
    이미지를 처리하여 항목 정보를 추출하고 그 정보를 사용해 데이터베이스를 업데이트
    """
    try:
        data = request.json
        image_data = data['image']
        
        header, encoded = image_data.split(',', 1)
        image_bytes = base64.b64decode(encoded)
        image = np.asarray(bytearray(image_bytes), dtype=np.uint8)
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)

        detected_items = process_image(image)

        conn = get_db_connection()
        for item in detected_items:
            name = item['name']
            price = item['price']
            expiry_date = item['expiry_date']
            quantity = item['quantity']

            existing_item = conn.execute('SELECT * FROM items WHERE name = ?', (name,)).fetchone()

            if existing_item:
                new_quantity = existing_item['quantity'] + quantity
                conn.execute('''
                    UPDATE items
                    SET price = ?, expiry_date = ?, quantity = ?
                    WHERE name = ?
                ''', (price, expiry_date, new_quantity, name))
            else:
                conn.execute('''
                    INSERT INTO items (name, price, expiry_date, quantity)
                    VALUES (?, ?, ?, ?)
                ''', (name, price, expiry_date, quantity))

        conn.commit()
        conn.close()

        return '', 204
    except Exception as e:
        logging.error(f"Image processing error: {str(e)}")
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/order_complete')
def order_complete():
    """
    call_ros2_service 를 웹의 결제 완료 버튼이랑 연동

    """
    print("결제 완료 버튼이 눌렸습니다. DB 초기화 및 ROS2 서비스 호출을 시작합니다.")
    logging.info("결제 완료 버튼이 눌렸습니다. DB 초기화 및 ROS2 서비스 호출을 시작합니다.")
    
    # Call the ROS2 service
    service_response = call_ros2_service()
    
    # Render the result page with the service response
    return render_template('result.html', service_response=service_response)

@app.route('/reset_db', methods=['POST'])
def reset_db():
    """
    결제 완료 버튼 눌렀을때 초기화
    """
    try:
        conn = get_db_connection()
        conn.execute('DELETE FROM items')  # 모든 항목 삭제
        conn.commit()
        conn.close()
        
        success_message = "데이터베이스가 성공적으로 초기화되었습니다."
        print(success_message)
        logging.info(success_message)
        
        return jsonify({"status": "success", "message": "Database reset successfully"}), 200
    except Exception as e:
        error_message = f"DB 초기화 중 오류 발생: {e}"
        print(error_message)
        logging.error(error_message)
        
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/get_items')
def get_items():
    """
    데이터베이스에서 항목을 조회하고, 그 결과를 JSON 형식으로 반환
    (서버와 클라이언트 간에 데이터 교환이 원활하게 이루어지도록 하며, 클라이언트 측에서 데이터를 처리)
    """
    conn = get_db_connection()
    items = conn.execute('SELECT * FROM items').fetchall()
    conn.close()
    return jsonify([dict(item) for item in items])

def update_db_periodically():
    """
    데이터 베이스 주기적으로 업데이트
    """
    while True:
        time.sleep(0.5)  # 0.5초마다 데이터베이스 업데이트
        conn = get_db_connection()
        # 예시로 데이터베이스를 업데이트하는 로직
        conn.close()

if __name__ == "__main__":
    threading.Thread(target=update_db_periodically, daemon=True).start()
    app.run(debug=True)
