'''
1. ROS 2와 Flask-SocketIO를 결합하여 마트 재고 및 유저 상태를 실시간으로 모니터링하는 서버 코드.
2. 로봇으로부터 재고 부족 알림을 수신하여 웹 대시보드에 팝업을 띄우고, 관리자의 승인을 받아 로봇에게 매대까지 이동하도록 명령을 내림.
3. 데이터베이스(SQLite)의 최신 정보를 1초 간격으로 스캐닝하여 웹 화면의 수치들을 자동으로 동기화.
'''


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image  # 로봇 카메라 영상 수신용 메시지 타입
from cv_bridge import CvBridge     # ROS 이미지 데이터를 OpenCV 형식으로 변환해주는 도구
import cv2
import base64

from flask import Flask, render_template
from flask_socketio import SocketIO
import sqlite3
import threading
import time
import os

# ==========================================
# 1. 서버 설정 및 통신 엔진 초기화
# ==========================================

app = Flask(__name__)
app.secret_key = 'mart_monitoring_secret' # 웹 세션 보안 키

# SocketIO 설정: 
# cors_allowed_origins="*": 보안 정책 상 다른 포트/도메인에서의 접근을 허용
# async_mode='threading': Flask 서버가 돌아가는 동안 ROS와 DB 스캔이 멈추지 않도록 스레딩 모드 사용
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# 연동할 SQLite 데이터베이스 파일 경로 (상대 경로로 수정 예정)
DB_PATH = '/home/rokey/Desktop/ZOLZOL_2/Flask/Customer/mart_1.db'

# ==========================================
# 2. MartMonitorNode 클래스 (ROS 2 통신 관리)
# ==========================================

class MartMonitorNode(Node):
    """
    센서 데이터(재고 부족, 영상)를 받아 웹으로 전달하고, 웹의 명령을 로봇에게 전달함.
    """

    def __init__(self):
        super().__init__('mart_monitor_node')
        self.bridge = CvBridge() # 이미지 변환 객체 초기화

        # [Subscription] 로봇(또는 다른 노드)으로부터 특정 상품의 재고 부족 신호를 수신
        self.stock_sub = self.create_subscription(
            String, 
            'out_of_stock', 
            self.product_start_callback, 
            10)

        # [Publication] 관리자의 승인을 받은 후 로봇에게 이동 명령을 내릴 채널     
        self.move_pub = self.create_publisher(
            String, 
            'robot_move_topic', 
            10)
        
        
        # [Subscription] 로봇의 카메라 영상을 받아 실시간 CCTV 구현
        self.video_sub = self.create_subscription(
            Image, 
            'webcam_video', 
            self.video_callback, 
            10
        )

        self.get_logger().info("ROS 2 Mart Monitor Node 가동 중...")

    
    def product_start_callback(self, msg):
        """상품 재고 부족 신호 수신 시 웹 관리자 화면에 승인 팝업 요청"""
        product_name = msg.data.strip()
        # 시스템에 등록된 유효한 상품 리스트
        valid_products = ["Chocobi", "Cocacola", "Oreo", "Powerade", "Saewookkang", "Seoul Milk"]
        
        if product_name in valid_products:
            self.get_logger().info(f"[재고 부족] {product_name} -> 웹 팝업 전송")
            # 웹 브라우저(admin_monitor_cctv.html)로 '재고 보충 팝업을 띄우라'고 신호 보냄
            socketio.emit('show_restock_popup', {'product': product_name})
        else:
            self.get_logger().warn(f"승인되지 않은 상품명: {product_name}")
    
    def publish_command(self, product_name):
            """웹에서 승인 버튼을 눌렀을 때 실제로 로봇에게 주행 좌표 이동 명령을 발행"""
        
        # 안전장치: 데이터가 딕셔너리일 경우 상품명만 추출
            if isinstance(product_name, dict):
                product_name = product_name.get('product', 'Unknown')


            try:
                # --- 1. DB 재고 업데이트 로직 ---
                conn = sqlite3.connect(DB_PATH)
                cursor = conn.cursor()

                # 현재 창고 재고 확인 (0개 미만으로 내려가지 않게 안전장치)
                cursor.execute("SELECT warehouse_qty FROM products WHERE name = ?", (product_name,))
                row = cursor.fetchone()
                
                if row and row[0] > 0:
                    # 창고 재고 1 감소, 매대 재고 1 증가
                    cursor.execute("""
                        UPDATE products 
                        SET warehouse_qty = warehouse_qty - 1, 
                            shelf_qty = shelf_qty + 1 
                        WHERE name = ?
                    """, (product_name,))
                    conn.commit()
                    self.get_logger().info(f"[DB 업데이트] {product_name}: 창고 -> 매대 이동 완료")
                else:
                    self.get_logger().warn(f"[재고 부족] 창고에 {product_name} 재고가 없습니다!")

                conn.close()
                
                msg = String()
                msg.data = product_name
                self.move_pub.publish(msg) # 여기서 실제로 robot_move_topic으로 전송합니다.
                self.get_logger().info(f"로봇에게 명령 전송 완료: {product_name}")

            except Exception as e:
                self.get_logger().error(f"DB 업데이트 중 에러 발생: {e}")

            
    def video_callback(self, msg):
        """로봇의 카메라 영상을 웹 브라우저가 읽을 수 있는 형식으로 변환하여 실시간 전송"""
        try:
            # 1. ROS Image 메시지를 OpenCV(BGR) 이미지 형식으로 변환
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. 영상 데이터를 JPEG 포맷으로 압축(인코딩)
            _, buffer = cv2.imencode('.jpg', cv_img)
            
            # 3. 바이너리 데이터를 텍스트(Base64)로 변환하여 네트워크 전송 준비
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            
            # 4. 웹소켓을 통해 브라우저의 'video_frame' 이벤트로 전송 (실시간 CCTV 효과)
            socketio.emit('video_frame', {'image': jpg_as_text})
            
        except Exception as e:
            self.get_logger().error(f"영상 처리 에러: {e}")

# 전역 변수로 ROS 노드 참조 (Flask 라우트에서 접근 가능하게 함)
ros_node = None


# ==========================================
# 3. 웹 서버(Flask) 통신 및 데이터 처리
# ==========================================

@socketio.on('confirm_restock')
def handle_restock(data):
    """관리자가 웹 화면에서 '보충 승인(YES)'을 클릭했을 때 호출되는 이벤트 처리기"""
    global ros_node
    print(f"DEBUG: 웹 수신 데이터 -> {data}")

    # 웹에서 넘어온 데이터 분석 (딕셔너리 또는 단일 문자열 여부 확인)
    if isinstance(data, dict):
        response = data.get('response')
        product = data.get('product', 'Unknown') # 웹이 보낸 상품명을 받음
    else:
        response = data
        product = "Unknown"

    print(f"DEBUG: 최종 수신 데이터 -> Response: {response}, Product: {product}")

    # 승인이 완료되었고 ROS 노드가 가동 중이라면 로봇에게 주행 명령 하달
    if response == 'yes' and ros_node:
        ros_node.publish_command(product)

# --- 대시보드 데이터 조회 (기존 로직 유지) ---
def get_mart_data():
    """SQLite 데이터베이스에서 현재 매장의 모든 재고 및 유저 상태 정보를 읽어오는 함수"""
    
    try:
        # DB 연결
        conn = sqlite3.connect(DB_PATH, check_same_thread=False)
        cursor = conn.cursor()

        # 1. 모든 상품의 재고 상태(매대 및 창고) 조회
        cursor.execute("SELECT name, price, shelf_qty, warehouse_qty FROM products")
        products = [{"name": r[0], "price": r[1], "shelf": r[2], "warehouse": r[3]} for r in cursor.fetchall()]
        
        # 2. 모든 유저 정보 및 그들의 장바구니 실시간 현황 조회
        cursor.execute("SELECT id, name, budget, total FROM users")
        user_rows = cursor.fetchall()
        users_with_carts = []

        for u_id, u_name, u_budget, u_total in user_rows:
            # 해당 유저가 현재 장바구니에 담은 물건 목록 조회
            cursor.execute("SELECT p.name, c.quantity FROM carts c JOIN products p ON c.product_id = p.id WHERE c.user_id = ?", (u_id,))
            cart_items = cursor.fetchall()

            # 장바구니 아이템들을 "상품명(수량개)" 형태의 문자열 리스트로 변환
            item_list = [f"{item[0]}({item[1]}개)" for item in cart_items]
            
            # 장바구니에 담긴 물건들의 총 금액(소계) 계산
            cursor.execute("SELECT SUM(quantity * price_each) FROM carts WHERE user_id = ?", (u_id,))
            current_cart_sum = cursor.fetchone()[0] or 0
            users_with_carts.append({
                "name": u_name, "budget": u_budget, "total": u_total, 
                "items": ", ".join(item_list) if item_list else "비어 있음", "cart_sum": current_cart_sum
            })
        conn.close()
        return {"users": users_with_carts, "products": products}
    except Exception as e:
        print(f"DB 에러: {e}")
        return {"users": [], "products": []}

def db_monitor_loop():
    """백그라운드에서 무한 루프를 돌며 1초마다 대시보드 데이터를 웹으로 자동 갱신"""
    while True:
        data = get_mart_data()
        # 모든 접속된 웹 브라우저에 최신 정보를 쏨 (실시간 동기화)
        socketio.emit('update_dashboard', data)
        time.sleep(1)

@app.route('/')
def index():
    """브라우저 접속 시 관리자용 CCTV 대시보드 페이지 출력"""
    return render_template('admin_monitor_cctv.html')


# ==========================================
# 4. 프로그램 메인 실행부
# ==========================================

if __name__ == '__main__':
    # 1. ROS 2 초기화
    rclpy.init()            
    ros_node = MartMonitorNode()

    # 2. 멀티스레드 실행
    # 스레드 1: ROS 2 통신 처리
    threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True).start()
    
    # 스레드 2: 1초마다 DB를 감시하여 웹 화면 갱신
    threading.Thread(target=db_monitor_loop, daemon=True).start()

    # 3. Flask 웹 서버 기동 (웹소켓 기반)
    # host='0.0.0.0': 외부 기기에서의 접속 허용
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, use_reloader=False)