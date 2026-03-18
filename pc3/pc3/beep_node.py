#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
# ... 나머지 기존 임포트 ...

class ProductNavigator(Node):
    def __init__(self):
        super().__init__('product_navigator')
        
        # ... 기존 navigator 및 publisher 설정 ...
        self.audio_pub = self.create_publisher(AudioNoteVector, '/robot1/cmd_audio', 10)

        # 'out_of_stock' 토픽 구독 추가
        self.stock_sub = self.create_subscription(
            String, 
            'budget_warning', 
            self.budget_warning_callback, 
            10
        )

        self.get_logger().info("Product Navigator Ready with Stock Warning!!")

    # 삐뽀삐뽀 경고음 함수
    def play_warning_sound(self):
        msg = AudioNoteVector()
        msg.append = False
        
        # 경고음은 조금 더 긴박하게 880Hz와 988Hz(시)를 반복하거나 
        # 사용자가 요청한 880, 440 패턴을 사용합니다.
        for _ in range(2): # 삐뽀삐뽀 2번 반복
            for freq in [880, 440]:
                note = AudioNote()
                note.frequency = int(freq)
                note.max_runtime.sec = 0
                note.max_runtime.nanosec = 300000000 # 0.3초
                msg.notes.append(note)
        
        self.audio_pub.publish(msg)
        self.get_logger().warn("⚠️ Warning Sound Played: Budget Warning!")

    # out_of_stock 토픽 콜백
    def budget_warning_callback(self, msg):
        # 받은 메시지가 'warning'일 때만 소리 발생
        if msg.data.strip() == 'Warning':
            self.get_logger().error("!!! 예산 초과 !!! 예산 초과 !!!")
            self.play_warning_sound()




def main(args=None):
    # 1. rclpy 초기화
    rclpy.init(args=args)

    try:
        # 2. 노드 인스턴스 생성
        node = ProductNavigator()

        # 3. 멀티스레드 실행기 설정 (추천)
        # 소리 재생이나 내비게이션 대기 중에도 다른 콜백을 처리하기 위함입니다.
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)

        try:
            # 4. 노드 실행 (Ctrl+C를 누를 때까지 대기)
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrupt (SIGINT) - 노드를 종료합니다.')
        finally:
            # 5. 종료 전 정리 작업
            node.destroy_node()
            
    finally:
        # 6. rclpy 종료
        rclpy.shutdown()


if __name__ == '__main__':
    main()