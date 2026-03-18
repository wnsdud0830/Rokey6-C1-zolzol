# ============================================
# AMR1 동작 코드
# --------------------------------------------
# 1) 구매자가 입구로 들어오면, 웹캠 → /person_detect == "customer position" 퍼블리시
# 2) 이 노드는 /person_detect를 구독하여 "counter arrival" 수신 시:
#    - Undock 수행
#    - 미리 정해둔 YAW 각도로 회전하는 PRESET_GOAL로 이동 (GO_PRESET 상태)
# 3) PRESET_GOAL 도착 후 FOLLOW 상태로 전환:
#    - OAK-D 카메라 RGB/Depth + YOLO로 person을 추적
#    - 일정 거리 유지
# 4) 구매자가 결제/구매 완료 후 구매 지점에 도착하면
#    - 웹캠 → /person_detect == "counter arrival" 퍼블리시해 결제 지점에 도착했음을 알림
#    - 이 노드는 amr1을 RETURN_GOAL로 이동
#    - 완전히 결제가 끝나면 /finish_shopping=finish shopping이 퍼블리시
#    - 이를 수신해 amr1을 종료 지점으로 이동
# ============================================


import os
import math
import threading
import time
from queue import Queue
import struct

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, PoseStamped, Quaternion

from cv_bridge import CvBridge
from ultralytics import YOLO

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

# ================================
# [설정 파트] - 시스템 파라미터
# ================================

# /person_detect == "customer position" 이 들어오면,
# TurtleBot의 yaw를 이 각도로 맞춘 방향(PRESET_GOAL)으로 이동
PRESET_GOAL_YAW = 1.745  # rad (GO_PRESET에서 사용할 yaw)

# 웹캠에서 사람 감지 시 들어오는 상황별 토픽 이름
TRIGGER_TOPIC = "/person_detect"            # String: "customer position", "counter arrival"
FINISH_SHOPPING_TOPIC = "/finish_shopping"  # String: "finish shopping"

# tracking에 사용할 카메라 이미지 / 깊이 / 속도 명령 토픽
RGB_TOPIC = "/robot6/oakd/rgb/image_raw/compressed"
DEPTH_TOPIC = "/robot6/oakd/stereo/image_raw/compressedDepth"
CMD_VEL_TOPIC = "/robot6/cmd_vel"

# YOLO에서 감지할 대상 클래스
TARGET_CLASS_KEYWORD = "person"

# 추적 파라미터
DESIRED_DISTANCE = 0.65    # 사람과의 목표 거리 (m)
DISTANCE_TOLERANCE = 0.1   # 허용 오차

# 탐색 파라미터
SEARCH_TIMEOUT = 1.0      # 물체를 놓치고 1초 뒤부터 탐색 시작

# P 제어 게인
K_LINEAR = 0.7       # 거리 오차에 대한 선속도 p 게인
K_ANGULAR = 0.005    # 화면 중심으로부터 x오차(좌우 오차)에 대한 각속도 p 게인

# clip 범위
MAX_LIN_VEL = 0.7    # 최대 선속도 제한
MAX_ANG_VEL = 1.0    # 최대 각속도 제한

# 모델 경로
ROBOT_YOLO_MODEL_PATH = "/home/rokey/Downloads/yolov8n.pt"

# /person_detect 토픽에서 "counter arrival" 수신 시 이동할 RETURN 목표 (map 기준)
RETURN_GOAL_X = -2.15
RETURN_GOAL_Y = -0.75
RETURN_GOAL_YAW = 0.4   # rad

# /finish_shopping 토픽에서 "finish shopping" 수신 시 이동할 FINISH 목표 (map 기준)
FINISH_GOAL_X = -0.45
FINISH_GOAL_Y = 0.0
FINISH_GOAL_YAW = 0.4   # rad

# dock status 오판/갱신 지연 대비: GO_PRESET 시작 시 undock을 한번 시도 할지
ALWAYS_TRY_UNDOCK_ON_GO_PRESET = True


# ============================================
# 유틸 함수: yaw(heading) → Quaternion 변환
# ============================================

"""
주어진 yaw(라디안)를 기준으로, Z축 회전에 해당하는 quaternion을 생성.
Nav2의 Pose.orientation에 직접 집어넣어서 사용.
"""
def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ============================================
# Person trigger 및 Follower Node
# ============================================

class PersonTriggerAndFollower(Node):
    """
    상태 요약:
      - WAIT_TRIGGER : /person_detect == "customer position" 올 때까지 대기
      - GO_PRESET    : TurtleBot4Navigator로 PRESET 동작. PRESET_GOAL_YAW 각도로 회전.
      - FOLLOW       : YOLO 기반 타겟 추적 (타겟 미검출 시 정지 유지)
      - GO_RETURN    : "counter arrival" 수신 시 RETURN_GOAL로 이동
      - GO_FINISH    : /finish_shopping 토픽에서 메시지 ("finish shopping") 수신 시 FINISH_GOAL로 이동
      - DONE         : 필요 시 확장
    """

    def __init__(self, model: YOLO):
        # ROS2 Node 초기화 (노드 이름: person_trigger_and_follower)
        super().__init__("person_trigger_and_follower")
        
        # YOLO / CV / Navigation 구성
        self.model = model
        self.bridge = CvBridge()
        self.navigator = TurtleBot4Navigator()

        # 상태 머신 변수 및 보호용 락 초기화
        self.state_lock = threading.Lock()
        self.stage = "WAIT_TRIGGER"    # 현재 상태
        self.triggered = False         # 최초 트리거 여부 (customer position 수신 여부)

        # Nav2 목표 이동 시작 시간 기록용 (isTaskComplete 체크용)
        self.nav_start_time = 0.0

        # finish 이벤트 1회 처리 + /person_detect 덮어쓰기 방지용 락
        self.finish_locked = False       #finish 동작 중 / 이후에 /person_detect로 상태가 덮어씌워지는 것 방지
        self.finish_triggered = False    #finish_shopping 신호를 이미 한 번 처리했는지 여부

        # Tracking 관련. depth 메시지 보호용 락 + latest_depth_msg
        self.depth_lock = threading.Lock()
        self.latest_depth_msg = None
        # RGB는 CompressedImage msg 단위로 최신 1개만 큐에 유지
        self.rgb_queue = Queue(maxsize=1)
        self.stop_evt = threading.Event()

        # 화면 표시용 frame 공유 변수
        self.visual_lock = threading.Lock()
        self.visual_frame = None

        # 화면 갱신 / YOLO 제어 주파수 관리
        self.IDLE_VIEW_HZ = 1.0       # FOLLOW가 아닐 때 화면 갱신 빈도
        self.FOLLOW_PROC_HZ = 10.0    # FOLLOW일 때 YOLO + 제어 실행 빈도
        self._last_idle_time = 0.0
        self._last_follow_time = 0.0
        self.VIEW_HZ = 10.0           # FOLLOW일 때 화면 갱신 빈도
        self._last_view_time = 0.0

        # cx 저역필터 초기값 (None이면 첫 값 그대로 반영)
        self.cx_filtered = None

        # 이미지 토픽 QoS 설정 : 최신 1개만 데이터 저장, BEST_EFFORT(빠르지만 일부 손실 허용), subscriber가 나중에 생기면 최신 메시지만 전달.
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ROS Pub/Sub 설정
        # 사람 감지 트리거 토픽 Subscription. /person_detect로 상태 전이 제어
        self.create_subscription(String, TRIGGER_TOPIC, self.trigger_cb, 10)
        # /finish_shopping으로 쇼핑 종료 시 FINISH_GOAL로 이동
        self.create_subscription(String, FINISH_SHOPPING_TOPIC, self.finish_cb, 10)
        # YOLO 추적용 Image/depth Subcription
        self.create_subscription(CompressedImage, RGB_TOPIC, self.rgb_cb, sensor_qos)
        self.create_subscription(CompressedImage, DEPTH_TOPIC, self.depth_cb, sensor_qos)
        #로봇 속도 명령 Publisher (cmd_vel)
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.get_logger().info(f"Target: {TARGET_CLASS_KEYWORD} | 초기 상태: WAIT_TRIGGER")

        # 메인 상태 머신 타이머. 0.2초마다 control_loop 호출
        self.control_timer = self.create_timer(0.2, self.control_loop)

        # YOLO tracking/detection thread. track_loop 스레드 시작
        # 별도 스레드로 동작, daemon=True → 노드 종료 시 같이 종료
        self.worker = threading.Thread(target=self.track_loop, daemon=True)
        self.worker.start()

    # ---------------------------------------------------
    # compressedDepth 디코더
    # ---------------------------------------------------

    """
    sensor_msgs/CompressedImage 형식의 compressedDepth를
    실제 depth 이미지(np.ndarray)로 복원하는 함수.

    - format에 "32FC1" 이 포함된 경우 a,b 파라미터를 추출해 float depth로 복원
    - 그 외에는 그대로 uint16 depth로 사용
    - 복원 실패 시 None 반환
    """
    def decode_compressed_depth(self, msg: CompressedImage):
        if msg is None or msg.data is None:
            return None

        data = bytes(msg.data)
        png_magic = b"\x89PNG\r\n\x1a\n"
        # PNG 본문 시작 위치 찾기
        idx = data.find(png_magic)
        if idx < 0:
            return None

        # PNG 부분만 잘라서 디코딩
        png_bytes = data[idx:]
        depth_png = cv2.imdecode(np.frombuffer(png_bytes, np.uint8), cv2.IMREAD_UNCHANGED)
        if depth_png is None:
            return None

        # 3채널로 나오는 경우 첫 채널만 사용
        if depth_png.ndim == 3:
            depth_png = depth_png[:, :, 0]

        fmt = (msg.format or "").lower()

        # 32FC1 (float depth) 인코딩인 경우 a,b 파라미터로 복원
        if "32fc1" in fmt:
            a = b = None

            # data 앞부분에서 a,b 읽어보기
            if len(data) >= 8:
                try:
                    a0, b0 = struct.unpack("<ff", data[0:8])
                    if np.isfinite(a0) and np.isfinite(b0):
                        a, b = a0, b0
                except Exception:
                    pass

            # PNG 앞쪽(직전 8바이트)에서 a,b 읽어보기
            if (a is None or b is None) and idx >= 8:
                try:
                    a1, b1 = struct.unpack("<ff", data[idx - 8:idx])
                    if np.isfinite(a1) and np.isfinite(b1):
                        a, b = a1, b1
                except Exception:
                    pass

            if a is None or b is None:
                return None

            raw = depth_png.astype(np.float32)
            denom = raw - b

            depth_m = np.full_like(raw, np.nan, dtype=np.float32)
            mask = denom != 0
            depth_m[mask] = a / denom[mask]
            return depth_m
        
        # 그 외에는 uint16(mm) depth 로 가정하여 반환
        return depth_png

    # ---------------------------------------------------
    # 공통 유틸 함수: 속도 명령 관련
    # ---------------------------------------------------

    # 선속도/각속도를 saturate 후 /cmd_vel에 발행
    def publish_cmd_vel(self, lin_x, ang_z):
        msg = Twist()
        msg.linear.x = float(np.clip(lin_x, -MAX_LIN_VEL, MAX_LIN_VEL))
        msg.angular.z = float(np.clip(ang_z, -MAX_ANG_VEL, MAX_ANG_VEL))
        self.cmd_vel_pub.publish(msg)

    # 로봇 정지 명령 발행
    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)

    # ---------------------------------------------------
    # finish_shopping 문자열 파서
    # ---------------------------------------------------

    #finish_shopping 토픽 payload가 유효한 finish 신호인지 검사.
    @staticmethod
    def _is_finish_signal(text: str) -> bool:
        if text is None:
            return False
        s = text.strip().lower()
        # 트리거: "finish shopping" (언더스코어 없음, 공백 포함)
        return s == "finish shopping"

    # ---------------------------------------------------
    # 트리거 콜백 (/person_detect)
    # ---------------------------------------------------

    """
    /person_detect 콜백. msg.data의 내용에 따라 stage 전환.
     - "customer position" : WAIT_TRIGGER 상태에서 GO_PRESET 상태로 전환
     - "counter arrival" : RETURN_GOAL로 이동하는 GO_RETURN 상태로 전환
    finish_locked/finsih_triggered 플래그로 finish 이동 중에 상태가 덮어씌워지지 않게 보호.
    """
    def trigger_cb(self, msg: String):
        status = (msg.data or "").strip()

        with self.state_lock:
            # finish 이동 중/완료 상태에서는 /person_detect가 stage를 덮어쓰지 않도록 차단
            # 단, 새 세션 시작( WAIT_TRIGGER에서 customer position )은 허용하고 finish 플래그를 리셋
            if self.finish_locked:
                if status == "customer position" and self.stage == "WAIT_TRIGGER":
                    # 새 세션 시작: finish 플래그 리셋
                    self.finish_locked = False
                    self.finish_triggered = False
                else:
                    return    # finish 동작 중이면 person_detect 무시

            if status == "customer position":
                if not self.triggered and self.stage == "WAIT_TRIGGER":
                    # 새 세션 시작 시 finish 플래그를 확실히 리셋
                    self.finish_locked = False
                    self.finish_triggered = False

                    self.triggered = True
                    self.stage = "GO_PRESET"
                    self.nav_start_time = 0.0
                    self.get_logger().info(f"신호 수신 [{status}] → PRESET으로 이동합니다.")

            elif status == "counter arrival":
                # counter arrival: 이미 GO_RETURN/DONE이면 무시 (한 번만 받고 이동)
                if self.stage in ("GO_RETURN", "DONE"):
                    return

                self.get_logger().info(f"신호 수신 [{status}] → RETURN GOAL로 이동합니다.")
                self.stage = "GO_RETURN"
                self.nav_start_time = 0.0

            elif status == "Not detected":
                pass

    # ---------------------------------------------------
    # finish_shopping 콜백
    # ---------------------------------------------------

    """
    payload가 "finish shopping"인 경우 FINISH_GOAL로 이동하는 GO_FINISH 상태로 전환.
    WAIT_TRIGGER 상태에서는 finish를 무시(세션 시작 전 신호는 무효).
    finish_triggered / finish_locked 로 중복 처리 및 상태 덮어쓰기 방지.
    """
    
    def finish_cb(self, msg: String):
        payload = (msg.data or "")

        # 트리거 문구 아니면 무시
        if not self._is_finish_signal(payload):
            return

        with self.state_lock:
            # WAIT_TRIGGER에서는 finish를 무시 (세션 시작 전 finish 메시지로 undock 막히는 현상 방지)
            if self.stage == "WAIT_TRIGGER":
                self.get_logger().warn(
                    f"finish_shopping 수신 [{payload.strip()}] but stage=WAIT_TRIGGER → 무시(세션 시작 후에만 유효)"
                )
                return

            # 이미 처리했으면 무시
            if self.finish_triggered or self.stage in ("GO_FINISH", "DONE"):
                return

            # finish 우선권 획득: 이후 /person_detect로 stage 덮어쓰기 방지
            self.finish_triggered = True
            self.finish_locked = True

            self.get_logger().info(f"finish_shopping 수신 [{payload.strip()}] → FINISH GOAL로 이동합니다.")
            self.stage = "GO_FINISH"
            self.nav_start_time = 0.0

    # ---------------------------------------------------
    # 상태 머신 루프
    # ---------------------------------------------------

    # 0.2초마다 호출되는 상태머신 메인 루프. 
    # stage 값에 따라 Nav2 목표를 설정/감시하고 상태 전환.
    def control_loop(self):
        with self.state_lock:
            stage = self.stage

        # 아무 것도 하지 않고 트리거 대기. /person_detect 콜백에서 stage 전환을 기다리는 상태
        if stage == "WAIT_TRIGGER":
            return

        # PRESET_GOAL로 이동 시작.
        elif stage == "GO_PRESET":
            if self.nav_start_time == 0.0:
                self._execute_undock_and_move_preset()
                self.nav_start_time = time.time()
                return

            # Navigator의 네비게이션 작업이 완료되었는지 체크
            if self.navigator.isTaskComplete():
                self.get_logger().info("PRESET GOAL 도착 → FOLLOW 모드로 전환.")
                with self.state_lock:
                    self.stage = "FOLLOW"
                    self.nav_start_time = 0.0

        # FOLLOW 상태에서의 세부 제어(추적/탐색)는 track_loop에서 처리
        elif stage == "FOLLOW":
            return

        # 계산대로 돌아가는 RETURN_GOAL 이동
        elif stage == "GO_RETURN":
            if self.nav_start_time == 0.0:
                self._execute_move_to_return_goal()
                self.nav_start_time = time.time()
                return

            if self.navigator.isTaskComplete():
                self.get_logger().info("RETURN GOAL 도착 → DONE 전환.")
                with self.state_lock:
                    self.stage = "DONE"

        # 쇼핑 종료 지점으로 이동
        elif stage == "GO_FINISH":
            if self.nav_start_time == 0.0:
                self._execute_move_to_finish_goal()
                self.nav_start_time = time.time()
                return

            if self.navigator.isTaskComplete():
                self.get_logger().info("FINISH GOAL 도착 → DONE 전환.")
                with self.state_lock:
                    self.stage = "DONE"

        elif stage == "DONE":
            return

    # ---------------------------------------------------
    # Nav2 이동 함수들
    # ---------------------------------------------------

    """
    GO_PRESET 상태에서 호출되는 함수.
    - 도킹 상태 여부와 상관없이 undock을 한 번 시도(ALWAYS_TRY_UNDOCK_ON_GO_PRESET)
    - PRESET_GOAL_YAW를 바라보는 Pose로 goToPose() 호출
    """
    def _execute_undock_and_move_preset(self):
        # dock 상태가 늦게 갱신되어 False가 나와 undock을 스킵하는 케이스 방지
        if ALWAYS_TRY_UNDOCK_ON_GO_PRESET:
            try:
                self.get_logger().info("GO_PRESET 시작 → undock() 1회 시도(안전 보강).")
                self.navigator.undock()
            except Exception as e:
                self.get_logger().warn(f"undock() 시도 중 예외(무시하고 진행): {e}")
        else:
            try:
                if self.navigator.getDockedStatus():
                    self.get_logger().info("Docked 상태 → undock() 실행.")
                    self.navigator.undock()
                else:
                    self.get_logger().info("이미 undocked 상태, 바로 이동.")
            except Exception as e:
                self.get_logger().warn(f"getDockedStatus/undock 예외(무시하고 진행): {e}")

        # PRESET_GOAL Pose 생성. x, y는 현재 위치 기준, yaw만 설정.
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.orientation = quat_from_yaw(PRESET_GOAL_YAW)

        self.get_logger().info(f"PRESET GOAL로 이동 시작: yaw={PRESET_GOAL_YAW:.3f}")
        self.navigator.goToPose(goal)

    # RETURN_GOAL_X/Y/YAW 로 이동하는 Nav2 호출 함수.
    def _execute_move_to_return_goal(self):
        # docked 상태면 필요 시 undock
        try:
            if self.navigator.getDockedStatus():
                self.get_logger().info(" Docked 상태 → undock() 실행 후 RETURN 이동.")
                self.navigator.undock()
        except Exception:
            pass

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = float(RETURN_GOAL_X)
        goal.pose.position.y = float(RETURN_GOAL_Y)
        goal.pose.orientation = quat_from_yaw(RETURN_GOAL_YAW)

        self.get_logger().info(
            f"RETURN GOAL로 이동 시작: x={RETURN_GOAL_X:.3f}, y={RETURN_GOAL_Y:.3f}, yaw={RETURN_GOAL_YAW:.3f}"
        )
        self.navigator.goToPose(goal)

    # FINISH_GOAL_X/Y/YAW 로 이동하는 Nav2 호출 함수.
    def _execute_move_to_finish_goal(self):
        try:
            if self.navigator.getDockedStatus():
                self.get_logger().info("Docked 상태 → undock() 실행 후 FINISH 이동.")
                self.navigator.undock()
        except Exception:
            pass

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = float(FINISH_GOAL_X)
        goal.pose.position.y = float(FINISH_GOAL_Y)
        goal.pose.orientation = quat_from_yaw(FINISH_GOAL_YAW)

        self.get_logger().info(
            f"FINISH GOAL로 이동 시작: x={FINISH_GOAL_X:.3f}, y={FINISH_GOAL_Y:.3f}, yaw={FINISH_GOAL_YAW:.3f}"
        )
        self.navigator.goToPose(goal)

    # ---------------------------------------------------
    # 데이터 콜백
    # ---------------------------------------------------

    """
    RGB 이미지 콜백 
    - decode는 track_loop에서 수행
    - 큐에 항상 최신 프레임 1장만 유지 (이전 것은 버림)
    """
    def rgb_cb(self, msg: CompressedImage):
        try:
            # 큐가 이미 차 있다면 오래된 프레임 하나 버리기
            if self.rgb_queue.full():
                self.rgb_queue.get_nowait()
            # 최신 프레임 추가
            self.rgb_queue.put_nowait(msg)
        except Exception as e:
            self.get_logger().error(f"RGB Queue Error: {e}")

    """
    Depth 이미지 콜백
    - decode는 track_loop에서 decode_compressed_depth로 수행
    - latest_depth에 마지막으로 들어온 depth 이미지만 보관
    - 여러 스레드에서 접근하므로 depth_lock으로 보호
    """
    def depth_cb(self, msg: CompressedImage):
        try:
            with self.depth_lock:
                self.latest_depth_msg = msg
        except Exception:
            pass

    # ---------------------------------------------------
    # 보조 함수
    # ---------------------------------------------------

    """
    RGB 이미지 좌표(x_rgb, y_rgb)를 Depth 이미지 좌표로 선형 매핑.
    - RGB와 Depth의 해상도가 다를 수 있으므로 비율 기반 변환
    - 범위를 넘어가지 않도록 클리핑
    """
    @staticmethod
    def map_rgb_to_depth(x_rgb, y_rgb, rgb_shape, depth_shape):
        rgb_h, rgb_w = rgb_shape[:2]
        depth_h, depth_w = depth_shape[:2]
        x_d = int(x_rgb * (depth_w / rgb_w))
        y_d = int(y_rgb * (depth_h / rgb_h))
        return max(0, min(depth_w - 1, x_d)), max(0, min(depth_h - 1, y_d))

    #Depth 픽셀 값을 meter 단위 거리로 변환
    @staticmethod
    def depth_to_meters(depth_val, dtype) -> float:
        if depth_val is None:
            return float("nan")
        v = float(depth_val)
        if np.isnan(v) or np.isinf(v):
            return float("nan")
        return v / 1000.0

    # ---------------------------------------------------
    # 추적 루프
    # ---------------------------------------------------

    """
    별도의 worker 스레드에서 동작 추적/제어 루프.
    - WAIT_TRIGGER/GO_PRESET/GO_RETURN/GO_FINISH/DONE 상태:
        YOLO 제어는 하지 않고, 화면만 IDLE_VIEW_HZ로 갱신
        cmd_vel은 WAIT_TRIGGER/DONE에서만 정지 명령 유지
    - FOLLOW 상태:
        VIEW_HZ로 화면 갱신
        FOLLOW_PROC_HZ로 YOLO + depth + 제어 수행
        사람 bbox의 cx를 저역필터 + deadband(20px)로 안정화
        사람을 일정 거리로 유지하며 따라감
        타겟 상실 시 회전 탐색 없이 정지 유지
    """
    def track_loop(self):
        # 마지막으로 타겟을 본 시각 (초 단위)
        last_seen_time = time.time()

        # ROS가 살아 있고(stop_evt가 set되지 않은 동안) 반복
        while rclpy.ok() and not self.stop_evt.is_set():
            try:
                # 현재 stage를 읽어옴 (락으로 보호)
                with self.state_lock:
                    stage = self.stage

                # 최신 RGB 프레임 가져오기
                try:
                    rgb_msg = self.rgb_queue.get(timeout=0.5)
                    rgb = self.bridge.compressed_imgmsg_to_cv2(
                        rgb_msg, desired_encoding="bgr8"
                    )
                except Exception:
                    # Nav2 이동 중(GO_PRESET/GO_RETURN/GO_FINISH)에는 stop_robot()으로 cmd_vel을 덮어쓰지 않음
                    if stage in ("WAIT_TRIGGER", "FOLLOW", "DONE"):
                        self.stop_robot()
                    continue

                now = time.time()
                vis = rgb.copy()

                # FOLLOW 상태가 아니면 추적 제어는 하지 않고 화면에 현재 stage만 표시
                if stage != "FOLLOW":
                    if (now - self._last_idle_time) >= (1.0 / self.IDLE_VIEW_HZ):
                        self._last_idle_time = now
                        cv2.putText(
                            vis,
                            f"Stage: {stage}",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 255),
                            2,
                        )
                        self._update_visual_frame(vis)

                    # WAIT_TRIGGER/DONE 에서만 정지 명령 유지. GO_PRESET/GO_RETURN/GO_FINISH 상태에서는 Nav2에서 속도 제어.
                    if stage in ("WAIT_TRIGGER", "DONE"):
                        self.stop_robot()
                    continue
                
                # 여기부터 stage == FOLLOW 상태

                # FOLLOW일 때 화면 갱신. (VIEW_HZ 기준)
                if (now - self._last_view_time) >= (1.0 / self.VIEW_HZ):
                    self._last_view_time = now
                    self._update_visual_frame(vis)

                # FOLLOW일 때 YOLO 제어 주기 제한
                if (now - self._last_follow_time) < (1.0 / self.FOLLOW_PROC_HZ):
                    time.sleep(0.001)
                    continue
                self._last_follow_time = now

                # Depth 메시지 확보 및 디코딩
                with self.depth_lock:
                    depth_msg = self.latest_depth_msg
                if depth_msg is None:
                    continue

                depth = self.decode_compressed_depth(depth_msg)
                if depth is None or (not hasattr(depth, "shape")):
                    continue
                
                # 이미지 중앙 x좌표 (각도 제어 기준)
                h, w, _ = rgb.shape
                center_x = w // 2

                # YOLO 추론 (conf=0.4 이상인 객체만 고려)
                results = self.model.predict(
                    rgb,
                    conf=0.4,
                    verbose=False,
                )

                best_target = None                # 이번 프레임에서 가장 tracking에 적합한 타겟
                class_names = self.model.names    # 모델 클래스 이름 dict 또는 list

                # YOLO 결과 중 person 클래스만 필터링 후 가장 가까운(거리 작은) 타겟 선택
                for r in results:
                    if r.boxes is None:
                        continue
                    for box in r.boxes:
                        cls = int(box.cls[0])
                        label = class_names[cls] if cls in class_names else str(cls)

                        # TARGET_CLASS_KEYWORD("person")가 label에 포함되지 않으면 무시
                        if TARGET_CLASS_KEYWORD not in label:
                            continue
                        
                        # 바운딩 박스 크기 기반 필터링

                        #화면 전체 크기와 바운딩 박스 크기 계산
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        rgb_h, rgb_w = vis.shape[:2]
                        total_area = rgb_h * rgb_w
                        box_area = (x2 - x1) * (y2 - y1)

                        # 화면에서 너무 작은 사람(먼 대상)은 무시 (1/15 이상만)
                        if box_area < (total_area / 15):
                            continue

                        cx = int((x1 + x2) / 2)
                        cy = int(y2)    # 바닥 쪽 (발 부근) 기준으로 depth 샘플
                        # Depth 이미지 좌표로 매핑 및 Depth 픽셀 값을 meter로 변환
                        dx, dy = self.map_rgb_to_depth(cx, cy, vis.shape, depth.shape)
                        dist = self.depth_to_meters(depth[dy, dx], depth.dtype)
                        
                        # 유효하지 않은 거리(NaN, 너무 가깝거나 먼 값) 필터링. 0.2m ~ 2.0m 이내만 유효
                        if np.isnan(dist) or dist <= 0.2 or dist > 2.0:
                            continue
                        
                        # 가장 가까운(거리 최소) 타겟을 best_target으로 선택
                        if best_target is None or dist < best_target["dist"]:
                            best_target = {
                                "cx": cx,
                                "cy": cy,
                                "dist": dist,
                                "label": label,
                                "x1": x1,
                                "y1": y1,
                                "x2": x2,
                                "y2": y2,
                                "conf": float(box.conf[0]),
                            }
                # ---------------------------------------------------
                # 로봇 제어 로직
                # ---------------------------------------------------

                if best_target is not None:
                    # 타겟 발견 -> 추적 모드
                    # 타겟을 본 시각 갱신
                    last_seen_time = time.time()

                    # -----------------------
                    # 좌우 오차 필터링 + deadband
                    # -----------------------

                    cx = best_target["cx"]

                    # 첫 프레임이면 필터 초기화
                    if self.cx_filtered is None:
                        self.cx_filtered = float(cx)
                    else:
                        # 저역필터: 0.7 * 이전 + 0.3 * 현재
                        self.cx_filtered = 0.7 * self.cx_filtered + 0.3 * float(cx)
                    
                    # 이미지 중심 대비 오차
                    err_x = center_x - self.cx_filtered

                    # 오차가 20픽셀 이하면 회전 명령 0 (미세 흔들림 방지)
                    # x오차에 비례하여 z축 각속도 결정
                    if abs(err_x) < 20:
                        ang_z = 0.0
                    else:
                        ang_z = K_ANGULAR * err_x

                    # 거리 제어: DESIRED_DISTANCE로 유지
                    # 실제 거리 - 목표 거리
                    # 목표 거리 안이면 정지, 아니면 P 제어
                    err_dist = best_target["dist"] - DESIRED_DISTANCE
                    if abs(err_dist) < DISTANCE_TOLERANCE:
                        lin_x = 0.0
                    else:
                        lin_x = K_LINEAR * err_dist

                    # 속도 명령 퍼블리시
                    self.publish_cmd_vel(lin_x, ang_z)

                    # 디버그용 Bounding Box 그리기 및 거리 표시
                    bx1, by1 = best_target["x1"], best_target["y1"]
                    bx2, by2 = best_target["x2"], best_target["y2"]
                    cv2.rectangle(vis, (bx1, by1), (bx2, by2), (0, 255, 0), 3)
                    cv2.putText(
                        vis,
                        f"TRACK: {best_target['dist']:.2f}m",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )
                else:
                    # 타겟을 못 보고 있는 상태
                    elapsed_time = time.time() - last_seen_time
                    if elapsed_time < SEARCH_TIMEOUT:
                        # 최근에 놓쳤다면 잠깐 정지(진동 방지)
                        self.stop_robot()
                        cv2.putText(
                            vis,
                            f"Wait... {elapsed_time:.1f}s",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 255),
                            2,
                        )
                    else:
                        # 탐색 회전 대신 완전 정지
                        self.stop_robot()
                        cv2.putText(
                            vis,
                            "LOST (Stop)",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 0, 255),
                            2,
                        )

                # 최신 vis 프레임 갱신
                self._update_visual_frame(vis)

            except Exception as e:
                # 예외 발생 시 루프가 죽지 않도록 로그 찍고 잠깐 쉰 뒤 계속
                self.get_logger().error(f"track_loop exception: {e}")
                time.sleep(0.01)
                continue
    
    # 메인 스레드에서 imshow 할 수 있도록 최신 화면(frame)을 visual_frame에 저장.
    def _update_visual_frame(self, frame):
        with self.visual_lock:
            self.visual_frame = frame

    # ---------------------------------------------------
    # 노드 종료 처리
    # ---------------------------------------------------

    """
    rclpy.shutdown() 전에 호출되는 정리 함수.
        - 로봇 정지
        - worker 스레드 종료
        - 로그 출력 후 부모 클래스 destroy_node() 호출
    """
    def destroy_node(self):
        self.stop_robot()
        self.stop_evt.set()
        try:
            self.worker.join(timeout=1.0)    # worker 스레드가 깔끔하게 종료되도록 join 시도
        except Exception:
            pass
        self.get_logger().info("PersonTriggerAndFollower 노드 종료.")
        super().destroy_node()

# ---------------------------------------------------
# 메인 함수 (ROS2 진입점)
# ---------------------------------------------------

"""
ROS2 초기화 후:
    - YOLO 모델 로드
    - PersonTriggerAndFollower 노드 생성 및 MultiThreadedExecutor에 추가
    - ROS2 spin은 별도 스레드에서 수행
    - 메인 스레드는 OpenCV 윈도우로 visual_frame을 계속 표시
    - 'q' 키 입력 시 종료
"""
def main():
    rclpy.init()

    if not os.path.exists(ROBOT_YOLO_MODEL_PATH):
        print(f"YOLO model not found: {ROBOT_YOLO_MODEL_PATH}")
        return

    model = YOLO(ROBOT_YOLO_MODEL_PATH)
    node = PersonTriggerAndFollower(model)

    # MultiThreadedExecutor:
    # track_loop는 파이썬 스레드, ROS 콜백은 Executor의 스레드 풀에서 처리
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    win_name = "AMR1 Person Follower"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    def spin_ros():
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass

    spin_thread = threading.Thread(target=spin_ros, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            frame = None
            with node.visual_lock:
                if node.visual_frame is not None:
                    frame = node.visual_frame.copy()

            if frame is not None:
                cv2.imshow(win_name, frame)

            key = cv2.waitKey(10) & 0xFF
            if key == ord("q"):
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.stop_evt.set()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()