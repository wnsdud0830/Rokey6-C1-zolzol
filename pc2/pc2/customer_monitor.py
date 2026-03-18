'''

1. 실시간 상품 탐지: YOLOv8 모델을 사용하여 웹캠 영상 속의 Chocobi, Oreo 등 6종의 상품을 실시간으로 식별합니다.

2. 인식 안정화 알고리즘: 오인식을 방지하기 위해 특정 영역(ROI) 내에서 상품이 20프레임(0.6초) 이상 연속 탐지될 때만 실제 구매 팝업을 생성합니다.

3. 웹 기반 장바구니 동기화: Flask-SocketIO를 통해 실시간 영상 스트리밍과 장바구니 데이터를 웹 대시보드에 지연 없이 동기화합니다.

4. 자동 재고 관리: 고객이 상품을 구매하면 DB의 매대 재고가 자동으로 차감되며, 재고 소진 시 로봇에게 보충 명령을 내립니다.

5. 예산 초과 경고 기능: 사용자의 총 지출액이 설정된 예산을 넘어서면 즉시 로봇에게 budget_warning 토픽을 발행하여 경고 알림을 수행합니다.

6. 로봇 보충 요청 (out_of_stock): 매대 재고가 0이 되면 로봇에게 out_of_stock 토픽을 발행하여 창고에서 물건을 가져오도록 유도합니다.

7. 고객 도착 감지 및 정지: 로봇으로부터 고객이 카운터에 도착했다는 신호를 받으면 웹에 결제창을 띄웁니다.

8. 결제 및 초기화 로직: 결제 완료 시 DB의 장바구니를 비우고 지출액을 초기화하며, 로봇에게 쇼핑 종료(finish_shopping) 신호를 전송합니다.

9. 멀티스레딩 병렬 처리: ROS 2 통신, YOLO 영상 분석, Flask 웹 서버가 서로 간섭 없이 동시에 작동하도록 멀티스레드 구조로 설계되었습니다.

10. 지능형 상호작용: 비전 데이터(YOLO)와 상태 데이터(DB)를 종합하여 로봇에게 물리적 명령(ROS 2)을 내리는 긴밀한 하드웨어-소프트웨어 연동을 보여줍니다.

'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template, request, session, redirect, url_for, flash
from flask_socketio import SocketIO
from ultralytics import YOLO
import sqlite3
import threading
import time
import cv2
import base64
import numpy as np
import os


# ==========================================
# 1. 서버 설정 및 전역 변수 초기화
# ==========================================
app = Flask(__name__)                      # Flask 웹 애플리케이션 객체 생성
app.secret_key = 'mart_secure_session_key' # 세션 데이터를 안전하게 암호화하기 위한 비밀키

# SocketIO 설정: 
# cors_allowed_origins="*": 모든 도메인에서의 웹소켓 접속 허용
# async_mode='threading': Flask와 함께 실행될 때 스레드 방식으로 비동기 제어
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

BASE_DIR = os.path.dirname(os.path.abspath(__file__)) # 현재 스크립트가 위치한 폴더의 절대 경로
DB_PATH = os.path.join(BASE_DIR, 'mart_1.db')         # SQLite DB 파일의 절대 경로 설정


# YOLO 모델 경로 (상대 경로로 수정 예정)
try:
    # 학습된 가중치 파일(.pt)을 불러와 객체 생성
    model = YOLO('/home/rokey/Desktop/ZOLZOL_2/Flask/Customer/detection.pt')
except Exception as e:
    print(f"YOLO 모델 로드 실패: {e}")
    model = None

# YOLO 모델이 인식할 수 있는 클래스 중 실제 처리할 상품 ID 리스트
# 0 : Chocobi
# 1 : Cocacola
# 2 : Oreo
# 3 : Powerade
# 4 : Saewookkang
# 5 : Seoul Milk
valid_classes = [0, 1, 2, 3, 4, 5] 

# [탐지 안정화 알고리즘 변수]
detection_counter = {}             # 프레임별로 탐지된 클래스의 연속 횟수를 저장하는 딕셔너리
CONFIRM_THRESHOLD = 60            # 동일 물체가 60프레임 이상 연속 탐지되어야 실제 상품으로 확정 (오탐지 방지)
last_popup_time = 0                # 구매 확인 팝업이 너무 자주 뜨지 않도록 시간 간격을 두기 위한 변수

ros_node = None                    # 메인 스레드에서 생성된 ROS 2 노드를 참조할 전역 변수

# ==========================================
# 2. 데이터베이스(DB) 관리 함수 레벨
# ==========================================
def get_db_connection():
    """SQLite 데이터베이스 연결 객체 반환"""
    # check_same_thread=False: 여러 스레드(Flask, SocketIO 등)에서 동시에 DB 접근 허용
    return sqlite3.connect(DB_PATH, check_same_thread=False)

def init_db():
    """시스템 기동 시 필요한 테이블 구조를 생성 및 초기화하는 함수"""
    conn = get_db_connection()
    try:
        cursor = conn.cursor()
        
        # 기존 장바구니 테이블 삭제 후 재구성 (Unique 제약 조건 등 최신 스키마 반영용)
        cursor.execute("DROP TABLE IF EXISTS carts") 

        # 장바구니 테이블: 유저-상품 조합이 중복되지 않도록 UNIQUE(user_id, product_id) 설정
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS carts (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                user_id INTEGER NOT NULL,
                product_id INTEGER NOT NULL,
                quantity INTEGER NOT NULL,
                price_each INTEGER NOT NULL,
                UNIQUE(user_id, product_id) 
            )
        """)
        
        # 상품 테이블: 이름, 가격, 매대재고(shelf), 창고재고(warehouse) 등 관리
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS products (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                yolo_class_id INTEGER NOT NULL UNIQUE,
                price INTEGER NOT NULL,
                shelf_qty INTEGER NOT NULL,
                warehouse_qty INTEGER NOT NULL,
                is_active INTEGER NOT NULL DEFAULT 1
            )
        """)

        # 유저 테이블: 사용자 기본 정보 및 설정된 예산(budget), 현재 지출 합계(total) 저장
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS users (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT,
                budget INTEGER DEFAULT 10000,
                total INTEGER DEFAULT 0,
                username TEXT,
                password TEXT
            )
        """)

        conn.commit()
    except Exception as e:
        print(f"DB 초기화 에러: {e}")
    finally:
        conn.close()

# ==========================================
# 3. ROS 2 로봇 관리 클래스 레벨 (MartRobotManager)
# ==========================================

class MartRobotManager(Node):
    """로봇과의 통신(발행/구독)을 전담하는 ROS 2 노드 클래스"""
    def __init__(self):
        super().__init__('mart_robot_manager')

        # [subscribe] 로봇이 사람이 왔음을 감지했을 때 신호 수신
        self.person_sub = self.create_subscription(String, 'person_detect', self.arrival_callback, 10)
        
        # [publication] 예산 초과 경고 및 재고 부족 시 로봇에게 명령 전송
        self.warning_pub = self.create_publisher(String, 'budget_warning', 10)  # 예산 초과 알림
        self.stock_pub = self.create_publisher(String, 'out_of_stock', 10)      # 상품 재고 보충 요청
        self.finish_pub = self.create_publisher(String, 'finish_shopping', 10)  # 쇼핑 종료 신호

    def arrival_callback(self, msg):
        """고객이 카운터 구역에 진입했을 때 호출되는 콜백 함수"""
        if msg.data.strip() == "counter arrival":   
            # 웹 브라우저(대시보드)에 결제창 팝업을 띄우라고 웹소켓으로 명령
            self.get_logger().info("카운터 도착 신호 수신: 결제 팝업을 요청합니다.")
            socketio.emit('show_checkout_popup')

    def publish_warning(self, user_name, budget, total):
        """예산 초과 시 로봇에게 경고 토픽 발행 (로봇이 경고음 등을 출력하도록 유도)"""
        msg = String()
        msg.data = f"Warning"
        self.warning_pub.publish(msg)

    def publish_out_of_stock(self, product_name):
        """상품 매대 재고 소진 시 로봇에게 보충 요청 발행"""
        msg = String()
        msg.data = f"{product_name}"
        self.stock_pub.publish(msg)
        self.get_logger().warn(f"발행 완료: {product_name} 재고 소진!")
    
    def publish_finish(self, user_name):
        """결제 완료 신호 발행"""
        msg = String()
        msg.data = "finish shopping"
        self.finish_pub.publish(msg)
        self.get_logger().info(f"발행 완료: {user_name} 결제 완료 신호 전송!")

def check_and_publish_warning(user_id):
    """사용자의 현재 쇼핑 총액이 예산을 초과했는지 DB에서 확인하고 ROS 신호 발행"""
    
    conn = get_db_connection()
    # DB에서 해당 유저의 이름, 예산, 현재까지의 총 구매액을 조회
    user = conn.execute("SELECT name, budget, total FROM users WHERE id = ?", (user_id,)).fetchone()
    conn.close()
    # 유저 정보가 존재하고, ROS 노드가 정상적으로 가동 중일 때만 로직 실행
    if user and ros_node:
        name, budget, total = user
        # 지출 총액(total)이 설정된 예산(budget)을 넘어섰는지 검사
        if total > budget:
            # 예산이 현재 지출금액보다 높으면 ROS 2 노드의 publish_warning 메서드를 호출하여 로봇에게 신호 전송
            ros_node.publish_warning(name, budget, total)

def get_user_cart_details(user_id):
    """사용자의 장바구니 현황(품목, 수량, 합계 등)을 DB에서 조회하여 JSON 형태로 반환"""

    try:
        conn = get_db_connection()
        # 1. 유저의 기본 요약 정보(이름, 예산, 총액) 조회
        user = conn.execute("SELECT name, budget, total FROM users WHERE id = ?", (user_id,)).fetchone()
        
        # 2. 장바구니와 상품 테이블을 JOIN하여 상세 내역 조회
        # 상품명, 담은 수량, 단가를 가져옴
        rows = conn.execute("""
            SELECT p.id, p.name, c.quantity, c.price_each FROM carts c 
            JOIN products p ON c.product_id = p.id WHERE c.user_id = ?
        """, (user_id,)).fetchall()
        
        # 3. 조회된 rows 데이터를 리스트 형태의 딕셔너리로 변환 (프론트엔드 전송용)
        # 각 항목의 소계(subtotal)를 계산 (수량 * 단가)
        items = [{"id": r[0], "name": r[1], "qty": r[2], "subtotal": r[2]*r[3]} for r in rows]
        conn.close()

        # 4. 최종적으로 웹 대시보드에 필요한 모든 정보를 묶어서 반환
        return {"name": user[0], "budget": user[1], "total_spent": user[2], "items": items}
    except: return None

# ==========================================
# 4. 실시간 영상 분석 및 비전 처리 레벨
# ==========================================

def process_webcam():
    """웹캠 영상을 받아 YOLO 모델로 상품을 탐지하고 결과를 브라우저로 전송하는 메인 비전 루프"""
    global last_popup_time
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    # ROI (관심 영역): 카메라 화면 전체가 아닌 특정 영역 안에서만 물체 탐지 수행 (성능 및 정확도 향상
    ROI_X1, ROI_Y1 = 100, 80
    ROI_X2, ROI_Y2 = 540, 400

    while True:
        ret, frame = cap.read()
        if not ret: continue
        

        if model:
            # 전체 화면 중 ROI 영역만 잘라내어(Crop) 모델에 입력
            roi_frame = frame[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]
            results = model.predict(roi_frame, conf=0.5, verbose=False, classes=valid_classes)
            detected_in_frame = set()

            # 가독성을 위해 원본 프레임에 ROI 영역을 파란색 사각형으로 표시
            cv2.rectangle(frame,(ROI_X1, ROI_Y1), (ROI_X2, ROI_Y2),(255, 0, 0),2) 
            
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    detected_in_frame.add(cls_id)

                    # ROI 내의 좌표를 원본 프레임 좌표로 변환하여 사각형(Bounding Box) 표시
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    x1 += ROI_X1
                    x2 += ROI_X1
                    y1 += ROI_Y1
                    y2 += ROI_Y1

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
     
            # [알고리즘] 연속 탐지 확정 로직
            for cls in detected_in_frame:
                # 해당 클래스 탐지 카운트 증가
                detection_counter[cls] = detection_counter.get(cls, 0) + 1
                # 60프레임 이상 연속으로 잡혔을 때만 상품 인식으로 간주
                if detection_counter[cls] >= CONFIRM_THRESHOLD:
                    # 팝업 간 간격(5초)을 두어 UI 방해 방지
                    if time.time() - last_popup_time > 5:
                        conn = get_db_connection()
                        res = conn.execute("SELECT name FROM products WHERE yolo_class_id = ?", (cls,)).fetchone()
                        prod_name = res[0] if res else f"품목 {cls}"

                        # 웹 대시보드에 "이 상품을 장바구니에 담으시겠습니까?" 팝업 요청
                        socketio.emit('show_purchase_popup', {'class_id': cls, 'prod_name': prod_name})
                        
                        last_popup_time = time.time()
                        detection_counter[cls] = 0 # 확정 후 카운트 초기화


        # OpenCV 프레임 이미지를 JPG 형식으로 인코딩 -> Base64 문자열로 변환하여 실시간 스트리밍 전송
        _, buffer = cv2.imencode('.jpg', cv2.resize(frame, (640, 480)))
        socketio.emit('webcam_frame', {'image': base64.b64encode(buffer).decode('utf-8')})
        time.sleep(0.03) # 약 30FPS 속도 제어


# ==========================================
# 5. 웹 서버 라우팅 및 비즈니스 로직 레벨 (Flask)
# ==========================================

@app.route('/login', methods=['GET', 'POST'])
def login():
    """사용자 로그인 처리 및 세션 관리"""
    if request.method == 'POST':
        username, password = request.form.get('username'), request.form.get('password')
        conn = get_db_connection()
        user = conn.execute("SELECT id FROM users WHERE username=? AND password=?", (username, password)).fetchone()
        conn.close()
        if user:
            session['user_id'] = user[0]
            return redirect(url_for('index'))
        flash('아이디 또는 비밀번호가 틀렸습니다.')
    return render_template('customer_login.html')

@app.route('/')
def index():
    """메인 모니터링 페이지 렌더링"""
    # 세션에 정보가 없으면 기본 사용자(1번)로 강제 접속 처리 (시연용)
    if 'user_id' not in session:
        conn = get_db_connection()
        user = conn.execute("SELECT id FROM users LIMIT 1").fetchone()
        conn.close()
        if user: session['user_id'] = user[0]
        else: return redirect(url_for('login'))
    return render_template('customer_monitor.html')

@socketio.on('request_my_data')
def handle_my_data():
    """웹 클라이언트가 초기화 또는 갱신을 위해 최신 사용자 데이터를 요청할 때의 응답"""
    user_id = session.get('user_id')
    if user_id: 
        # DB에서 현재 정보를 가져와 다시 웹소켓으로 응답
        socketio.emit('response_my_data', get_user_cart_details(user_id))

@socketio.on('confirm_purchase')
def handle_purchase(data):
    """고객이 웹 팝업에서 '구매'를 승인했을 때 실행되는 핵심 비즈니스 로직"""
    user_id, class_id = session.get('user_id'), data.get('class_id')
    conn = get_db_connection()
    product = conn.execute("SELECT id, name, price, shelf_qty, warehouse_qty FROM products WHERE yolo_class_id = ?", 
                            (class_id,)
                            ).fetchone()
    
    if product:
        p_id, p_name, price, s_qty, w_qty = product
        if s_qty > 0:
            # 1. 매대 재고(shelf_qty) 1개 차감
            conn.execute("UPDATE products SET shelf_qty = shelf_qty - 1 WHERE id = ?", (p_id,))
            
            # 2. 장바구니(carts) 테이블 업데이트: 이미 있는 상품이면 수량+1, 없으면 신규 추가 (UPSERT 방식)
            conn.execute("INSERT INTO carts (user_id, product_id, quantity, price_each) VALUES (?, ?, 1, ?) ON CONFLICT(user_id, product_id) DO UPDATE SET quantity=quantity+1", (user_id, p_id, price))
            
            # 3. 유저 테이블의 누적 지출액 가산
            conn.execute("UPDATE users SET total = total + ? WHERE id = ?", (price, user_id))
            conn.commit()
            
            # [실시간 재고 부족 처리]
            # 재고를 깎은 후 0이 되었다면 로봇에게 보충 명령 하달
            if s_qty - 1 == 0 and ros_node:
                if w_qty > 0:
                    # 매대는 비었으나 창고에 물건이 있는 경우 -> 로봇 보충 토픽 발행
                    if ros_node:
                        # 로봇에게 "창고에서 가져와"라는 의미로 특정 메시지 전송
                        ros_node.publish_out_of_stock(p_name) 
                        socketio.emit('error_message', {'msg': f'[{p_name}] 매대 재고 소진! 로봇이 창고({w_qty}개 남음)에서 보충 중입니다.'})
                else:
                    # 창고에도 재고가 아예 없는 경우
                    socketio.emit('error_message', {'msg': f'[{p_name}] 완전 품절! 창고에도 재고가 없습니다.'})
                
        else:
            socketio.emit('error_message', {'msg': f'[{p_name}] 재고가 모두 소진되었습니다!'})
    conn.close()

    # 구매 후 예산 초과 여부 즉시 확인 및 필요시 로봇 경고 발행
    check_and_publish_warning(user_id)
    # 웹 브라우저 화면 동기화
    socketio.emit('response_my_data', get_user_cart_details(user_id))

@socketio.on('delete_item')
def handle_delete(data):
    """장바구니에서 특정 상품을 뺄 때 호출되는 취소 로직"""
    user_id, p_id = session.get('user_id'), data.get('product_id')
    
    conn = get_db_connection()
    item = conn.execute("SELECT quantity, price_each FROM carts WHERE user_id=? AND product_id=?", (user_id, p_id)).fetchone()
    
    if item:
        qty, price = item
        if qty > 1:
            conn.execute("UPDATE carts SET quantity = quantity - 1 WHERE user_id=? AND product_id=?", (user_id, p_id))
        else:
            conn.execute("DELETE FROM carts WHERE user_id=? AND product_id=?", (user_id, p_id))
        
        # 취소했으므로 매대 재고 1개 복구 및 유저 총액 차감
        conn.execute("UPDATE products SET shelf_qty = shelf_qty + 1 WHERE id = ?", (p_id,))
        conn.execute("UPDATE users SET total = MAX(0, total - ?) WHERE id = ?", (price, user_id))
        conn.commit()
    
    conn.close()
    socketio.emit('response_my_data', get_user_cart_details(user_id))

@socketio.on('complete_checkout')
def handle_checkout():
    """고객이 결제 버튼을 눌러 모든 쇼핑을 완료했을 때 실행되는 초기화 로직"""
    user_id = session.get('user_id')
    if not user_id: return

    conn = get_db_connection()
    try:
        # 1. 사용자의 장바구니 비우기
        conn.execute("DELETE FROM carts WHERE user_id = ?", (user_id,))
        # 2. 누적 지출액 0원 초기화 (다음 쇼핑을 위해)
        conn.execute("UPDATE users SET total = 0 WHERE id = ?", (user_id,))
        conn.commit()
        
        # 3. 로봇에게 "쇼핑 끝났으니 복귀해"라는 의미로 피니시 토픽 발행
        if ros_node:
            ros_node.publish_finish("finish_shopping")

        # 3. 클라이언트에 완료 알림 및 데이터 갱신 신호 전송
        socketio.emit('error_message', {'msg': '결제가 완료되었습니다. 장바구니를 비웁니다.'})
        socketio.emit('response_my_data', get_user_cart_details(user_id))
    except Exception as e:
        print(f"결제 오류: {e}")
    finally:
        conn.close()

# ==========================================
# 6. 메인 실행 및 멀티스레드 제어 레벨
# ==========================================
if __name__ == '__main__':
    
    # 1. DB 스키마 초기화
    init_db()

    # 2. ROS 2 통신 시스템 초기화
    if not rclpy.ok(): rclpy.init()
    ros_node = MartRobotManager()

    # 멀티스레드 가동
    # 스레드 1: ROS 2 메시지 수신(spin)을 무한 루프로 실행
    threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True).start()
    
    # 스레드 2: OpenCV 및 YOLO 영상 처리 루프 실행
    threading.Thread(target=process_webcam, daemon=True).start()
    
    # 3. Flask 웹 서버 기동 (웹소켓 지원)
    # host='0.0.0.0': 같은 네트워크 내의 다른 기기에서도 접속 가능하게 허용
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)