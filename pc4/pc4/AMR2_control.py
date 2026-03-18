#!/usr/bin/env python3

'''

1. 배송 미션 수신 (admin_monitor.py 로부터 수신): 
    로봇은 admin_monitor.py 노드가 발행하는 **robot_move_topic**을 통해 
    이동할 상품명(예: Chocobi, Oreo 등)을 수신하고 배송 미션을 시작합니다.

2. 재고 경고 수신 (customer_monitor.py 로부터 수신): 
    customer_monitor.py 노드에서 재고 부족을 감지하여 out_of_stock 토픽을 발행하면, 
    로봇은 이를 받아 즉시 언도킹 후 관리자 위치로 이동하여 상황을 알립니다.

3. 정밀 경로 주행 및 자동 복귀: 
    수신된 상품명에 따라 route_library에 정의된 8개의 좌표를 순차적으로 통과하며 목적지 주행을 완료하고, 
    출발지로 돌아온 뒤 스스로 충전 스테이션에 **도킹(Docking)**하여 다음 미션을 대기합니다.

4. 주변 환경 인식 및 안전 제어: 
    주행 중 /robot1/scan 토픽을 통해 전방 30도 범위의 레이저 스캔 데이터를 실시간으로 모니터링하며, 
    0.5m 이내에 장애물이 발견되면 /robot1/cmd_vel 토픽으로 정지 명령을 발행하여 사고를 방지합니다.

5. 상태 기반 미션 보호: 
    로봇이 주행 중일 때는 새로운 명령을 차단하는 is_navigating 플래그를 활성화하여, 
    admin_monitor.py나 customer_monitor.py에서 추가 토픽이 들어오더라도 현재 미션을 안전하게 끝마칠 때까지 오작동을 방지합니다.
    
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# 터틀봇 4 전용 내비게이션 라이브러리 (방향 및 이동 제어)
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator,
    TurtleBot4Directions
)

import math
import time


# --- 상품별 이동 경로 라이브러리 ---
# 각 상품명에 대해 [x, y, 방향]의 리스트로 구성된 웨이포인트 정의
route_library = {
            "Chocobi": [
                [-0.83, -0.2, TurtleBot4Directions.EAST],
                [-0.83, -0.2, TurtleBot4Directions.EAST],
                [-0.73, -1.63, TurtleBot4Directions.NORTH],
                [0.67, -1.64, TurtleBot4Directions.SOUTH],  # 목적지
                [-0.73, -1.63, TurtleBot4Directions.WEST],
                [-0.83, -0.2, TurtleBot4Directions.NORTH],
                [0.0, 0.0, TurtleBot4Directions.NORTH],
                [0.0, 0.0, TurtleBot4Directions.NORTH]
            ],

            "Cocacola": [
                [-0.83, -0.2, TurtleBot4Directions.EAST],
                [-0.73, -1.63, TurtleBot4Directions.NORTH],
                [1.3, -1.5, TurtleBot4Directions.EAST],
                [1.09, -2.2, TurtleBot4Directions.WEST],    # 목적지
                [1.3, -1.5, TurtleBot4Directions.SOUTH],
                [-0.73, -1.63, TurtleBot4Directions.WEST],
                [-0.83, -0.2, TurtleBot4Directions.NORTH],
                [0.0, 0.0, TurtleBot4Directions.NORTH]
            ],

            "Oreo" : [
                [-0.83, -0.2, TurtleBot4Directions.EAST],
                [-0.83, -0.2, TurtleBot4Directions.EAST],
                [-0.73, -1.63, TurtleBot4Directions.NORTH],
                [2.32, -1.48, TurtleBot4Directions.SOUTH],  # 목적지
                [-0.73, -1.63, TurtleBot4Directions.WEST],
                [-0.83, -0.2, TurtleBot4Directions.NORTH],
                [0.0, 0.0, TurtleBot4Directions.NORTH],
                [0.0, 0.0, TurtleBot4Directions.NORTH]
            ],

            "Powerade" : [
                [-0.83, -0.2, TurtleBot4Directions.EAST],
                [-0.74, -3.88, TurtleBot4Directions.NORTH],
                [1.25, -3.93, TurtleBot4Directions.WEST],
                [1.25, -3.34, TurtleBot4Directions.EAST],   # 목적지
                [1.25, -3.93, TurtleBot4Directions.SOUTH],
                [-0.74, -3.88, TurtleBot4Directions.WEST],
                [-0.83, -0.2, TurtleBot4Directions.NORTH],
                [0.0, 0.0, TurtleBot4Directions.NORTH]
            ],

            "Seoul Milk" : [
                [-0.83, -0.2, TurtleBot4Directions.EAST],
                [-0.74, -3.88, TurtleBot4Directions.NORTH],
                [1.25, -3.93, TurtleBot4Directions.WEST],
                [1.25, -3.34, TurtleBot4Directions.EAST],   # 목적지
                [1.25, -3.93, TurtleBot4Directions.SOUTH],
                [-0.74, -3.88, TurtleBot4Directions.WEST],
                [-0.83, -0.2, TurtleBot4Directions.NORTH],
                [0.0, 0.0, TurtleBot4Directions.NORTH]
            ],

            "Saewookkang": [
                [-0.83, -0.2, TurtleBot4Directions.EAST],
                [-0.83, -0.2, TurtleBot4Directions.EAST],
                [-0.74, -3.88, TurtleBot4Directions.NORTH],
                [2.32, -3.67, TurtleBot4Directions.SOUTH],  # 목적지
                [-0.74, -3.88, TurtleBot4Directions.WEST],
                [-0.83, -0.2, TurtleBot4Directions.NORTH],
                [0.0, 0.0, TurtleBot4Directions.NORTH],
                [0.0, 0.0, TurtleBot4Directions.NORTH]
            ]
        }

class ProductNavigator(Node):

    def __init__(self):
        super().__init__('product_navigator')

        # 터틀봇 4 네비게이터 객체 생성 (로봇의 네임스페이스 'robot1' 지정)
        self.navigator = TurtleBot4Navigator(namespace='robot1')

        self.route_library = route_library

        # 속도 명령 발행자 (장애물 감지 시 정지용)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/robot1/cmd_vel', 10
        )

        # 레이저 스캔 구독자 (장애물 감지용)
        self.scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.scan_callback, 10
        )

        # 재고 부족 시 관리자 위치 이동 명령 구독
        self.subscription = self.create_subscription(
            String, 'out_of_stock', self.go_to_admin_callback, 10
        )

        # 외부(서버)로부터 상품 이동 명령 구독
        self.subscription = self.create_subscription(
            String, 'robot_move_topic', self.robot_move_callback, 10
        )

        # 안전 관련 설정
        self.safety_distance = 0.5      # 장애물 감지 거리 (0.5m)
        self.detection_angle = 30.0     # 전방 감지 각도 (좌우 30도)

        # 상태 제어 플래그
        self.is_navigating = False      # 현재 이동 동작 수행 중인지 여부
        self.initial_pose_set = False   # 로봇의 초기 위치가 설정되었는지 여부

        # # 로봇의 현재 방향(yaw) 기억용 변수 (기본값: 남쪽)
        self.current_yaw = TurtleBot4Directions.SOUTH

        self.get_logger().info("Product Navigator Ready!!")

    # --------------------------------------------------
    # --- 관리자 호출 콜백 함수 ---
    def go_to_admin_callback(self, msg):
        """재고 부족 토픽 수신 시 관리자 위치로 이동"""
        if self.is_navigating: return # 이미 이동 중이면 새 명령 무시
        self.is_navigating = True

        # 현재 도킹 상태라면 언도킹 수행
        if self.navigator.getDockedStatus():
            self.navigator.info('Undocikng...')
            self.navigator.undock()

            # 언도킹 액션이 완료될 때까지 대기
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
            
        self.navigator.waitUntilNav2Active()    # Nav2 시스템 활성화 대기

        # 초기 위치가 설정 안 됐다면 설정 (보통 최초 1회 실행)
        if not self.initial_pose_set:
            self.navigator.setInitialPose(
                self.navigator.getPoseStamped(
                    [0.0, 0.0],
                    TurtleBot4Directions.SOUTH
                )
            )
            self.initial_pose_set = True

        # 관리자 좌표로 이동 명령 (단일 지점 이동)
        goal_pose = self.navigator.getPoseStamped([-0.807,-0.095], TurtleBot4Directions.EAST)
        self.navigator.startToPose(goal_pose)

        # 도착할 때까지 대기
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("관리자 위치 도착 완료!")
        self.is_navigating = False # 작업 끝났으므로 해제

    # --------------------------------------------------
    # --- 상품 이동 명령 콜백 함수 ---
    def robot_move_callback(self, msg):
        """특정 상품명을 수신하면 해당 경로로 주행 시작"""
        if self.is_navigating:
            self.get_logger().info("이미 다른 작업이 진행 중입니다.")
            return

        name = msg.data.strip()
        if name not in self.route_library:
            self.get_logger().warn(f"Unknown product: {name}")
            return
        
        self.get_logger().info(f"--- 미션 시작: {name} ---")
        self.is_navigating = True
        self.current_target = name
        
        # 실제 경로 주행 함수 호출
        self.navigate_route(self.current_target)

    # --------------------------------------------------
    def create_pose(self, point):
        """리스트 좌표 [x, y, (yaw)]를 Nav2용 PoseStamped로 변환"""
        x, y = point[0], point[1]

        # 리스트에 방향 데이터가 포함된 경우
        if len(point) == 3:
            self.current_yaw = point[2]
            return self.navigator.getPoseStamped([x, y], point[2])
        # 방향 데이터가 없는 경우 마지막에 사용했던 방향 유지
        else:
            return self.navigator.getPoseStamped([x, y], self.current_yaw)

    # --------------------------------------------------
    # --- 핵심 경로 주행 로직 ---
    def navigate_route(self, product):
        """상품 위치까지 갔다가 돌아오는 시퀀스 제어"""

        # 1단계: 상품 목적지까지의 웨이포인트(0~3번) 설정
        waypoints = []
        for p in self.route_library[product][:4]:
            waypoints.append(self.create_pose(p))

        # 목적지행 주행 시작
        self.navigator.startFollowWaypoints(waypoints)
        waypoints = []  # 리스트 초기화
        
        ################################
        ### 중간 업무 시간 (예: 물건 하차/상차)
        ################################
        time.sleep(3.5) # 3.5초간 대기 (수정 가능)

        # 2단계: 복귀 경로 웨이포인트(4~7번) 설정
        for p in self.route_library[product][4:8]:
            waypoints.append(self.create_pose(p))

        # 복귀 주행 시작
        self.navigator.startFollowWaypoints(waypoints)

        # 복귀가 완료될 때까지 상태 체크
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        # 최종 도착 후 1초 대기 및 자동 도킹
        time.sleep(1.0)
        self.navigator.dock()
        
        self.is_navigating = False  # 미션 종료

    # --------------------------------------------------
    # --- 장애물 감지 스캔 콜백 ---
    def scan_callback(self, msg):
        """전방 30도 이내에 장애물이 가까워지면 로봇 정지"""
        if self.is_navigating:  # 이동 중일 때만 체크
            return

        angle = math.radians(self.detection_angle)
        idx = int(angle / msg.angle_increment)

        # 전방 기준 왼쪽(+)과 오른쪽(-) 범위의 데이터를 합침
        detect_ranges = msg.ranges[:idx] + msg.ranges[-idx:]

        for r in detect_ranges:
            # 설정한 안전 거리 이내에 물체가 감지되면 정지 함수 호출
            if msg.range_min < r < self.safety_distance:
                self.stop_robot()

    # --- 로봇 강제 정지 명령 ---
    def stop_robot(self):
        """로봇의 선속도와 각속도를 0으로 발행"""
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_vel_pub.publish(t)


def main():
    rclpy.init()                # ROS 2 초기화
    node = ProductNavigator()   # 노드 생성
    rclpy.spin(node)            # 노드 실행 유지 및 콜백 처리
    node.destroy_node()         # 종료 시 노드 파괴
    rclpy.shutdown()            # ROS 2 종료


if __name__ == '__main__':
    main()
