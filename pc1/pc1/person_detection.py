import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

import cv2
from ultralytics import YOLO
import numpy as np

class YOLOCenterPublisher(Node):
    """
    YOLOv8을 이용한 멀티 객체 탐지 및 구역 기반 상태 관리 노드입니다.
    제공된 pb_2_1.py의 로직을 기반으로 함수 단위로 구조화되었습니다.
    """

    def __init__(self, model_path):
        super().__init__('detection_node') # 사진의 노드 이름 반영

        # --- 1. YOLO 모델 로드 ---
        self.model = YOLO(model_path)
        self.classNames = self.model.names 

        # --- 2. 구역 설정 (pb_2_1.py 원본 좌표 유지) ---
        # [Ready Zone] 계산대 구역
        self.ready_pts = np.array([
            [11,171],   # 좌측 상단
            [104, 149],   # 우측 상단
            [155, 234],  # 우측 중앙
            [46, 271],  #하단 중앙
        ], np.int32)
        # [Entrance Zone] 고객 입장 구역
        self.entry_pts = np.array([
            [97, 400],   # 좌측 상단
            [222, 333],  # 우측 상단
            [310, 460],  # 우측 하단
            [285, 480],   # 좌측 하단
            [130, 480]
        ], np.int32)
        self.last_status = ""

        # --- 3. 웹캠 설정 ---
        self.cap = cv2.VideoCapture(2) 
        if not self.cap.isOpened():
            self.get_logger().error('웹캠을 열 수 없습니다.')
            return

        # 해상도 정보 확인 및 출력
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f'웹캠 해상도: {int(width)} x {int(height)} @ {fps} FPS')

        # --- 4. Publisher 및 타이머 설정 ---
        self.publisher_ = self.create_publisher(Point, 'data_topic', 10)
        self.position_pub = self.create_publisher(String, 'person_detect', 10)

        # [사진 블록 1] camera_callback 등록
        self.timer = self.create_timer(0.1, self.camera_callback)
        self.get_logger().info('다중 인원 대응 구역 감지 노드가 시작되었습니다.')

    # --- [사진 기능 1: camera_callback] ---
    def camera_callback(self):
        ret, img = self.cap.read()
        if not ret:
            return

        # 기능 분할 호출
        results = self.yolo_human_detect(img)
        status_msg = self.classify_entry_exit_by_bbox(img, results)
        self.amr1_mission_command(status_msg)

        # 시각화 로직 (pb_2_1.py 내용 유지)
        self.render_visualization(img, status_msg)

    # --- [사진 기능 2: yolo_human_detect] ---
    def yolo_human_detect(self, img):
        # YOLO 모델로 사람(class 0) 탐지 (pb_2_1.py 로직)
        return self.model(img, stream=True, verbose=False, classes=[0])

    # --- [사진 기능 3: classify_entry_exit_by_bbox] ---
    def classify_entry_exit_by_bbox(self, img, results):
        any_in_ready = False
        any_in_entry = False
        
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if cls_id not in self.classNames or self.classNames[cls_id] != 'person':
                    continue    
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                # 발 위치(Bottom Center) 기준 판별 (pb_2_1.py 로직)
                cx, cy = (x1 + x2) / 2.0, float(y2)

                # 구역 내부 판별
                is_in_ready = cv2.pointPolygonTest(self.ready_pts, (cx, cy), False) >= 0
                is_in_entry = cv2.pointPolygonTest(self.entry_pts, (cx, cy), False) >= 0

                if is_in_ready: any_in_ready = True
                if is_in_entry: any_in_entry = True

                # 바운딩 박스 그리기
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 100, 0), 2)
                cv2.circle(img, (int(cx), int(cy)), 5, (0, 255, 0), -1)

        # 최종 상태 결정 (pb_2_1.py 우선순위 로직)
        if any_in_ready:
            return "counter arrival"
        elif any_in_entry:
            return "customer position"
        else:
            return "Not detected"

    # --- [사진 기능 4: amr1_mission_command] ---
    def amr1_mission_command(self, status_msg):
        # 상태 변경 시에만 로그 출력 (pb_2_1.py 로직)
        if status_msg != self.last_status:
            if status_msg == "counter arrival":
                self.get_logger().info('📢 알림: 계산대에 고객이 확인되었습니다!')
            elif status_msg == "customer position":
                self.get_logger().info('📢 알림: 입장 구역에 고객이 확인되었습니다!')
            elif status_msg == "Not detected":
                self.get_logger().info('👤 탐지된 인원이 없습니다.')
            
            self.last_status = status_msg

        # ROS 2 토픽 발행
        msg_pos = String()
        msg_pos.data = status_msg
        self.position_pub.publish(msg_pos)

    def render_visualization(self, img, status_msg):
        """기존 pb_2_1.py의 시각화 코드를 별도 관리"""
        any_in_ready = (status_msg == "counter arrival")
        any_in_entry = (status_msg == "customer position")

        r_color = (0, 255, 0) if any_in_ready else (0, 150, 0)
        e_color = (0, 255, 0) if any_in_entry else (0, 0, 255)

        cv2.polylines(img, [self.ready_pts], True, r_color, 2)
        cv2.putText(img, "Counter Zone", (self.ready_pts[0][0], self.ready_pts[0][1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, r_color, 2)

        cv2.polylines(img, [self.entry_pts], True, e_color, 2)
        cv2.putText(img, "Entrance Zone", (self.entry_pts[0][0], self.entry_pts[0][1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, e_color, 2)

        cv2.imshow('Multi-Person Zone Detection', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cleanup()

    def cleanup(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    # 모델 경로는 사용자 환경에 맞춰 유지
    model_path = '/home/rokey/ZOLZOL/zolzol_ws/src/pc1/pc1/human_only.pt' 
    node = YOLOCenterPublisher(model_path)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()