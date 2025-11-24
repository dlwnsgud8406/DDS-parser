#!/usr/bin/env python3
"""
ROS2 QoS 호환성 테스트

같은 토픽에 서로 다른 QoS를 사용하는 Publisher와 Subscriber를 만들어
통신이 되는지 안 되는지 실제로 테스트합니다.

테스트 시나리오:
1. RELIABLE Publisher + RELIABLE Subscriber (✅ 호환)
2. RELIABLE Publisher + BEST_EFFORT Subscriber (✅ 호환)
3. BEST_EFFORT Publisher + RELIABLE Subscriber (❌ 비호환)
4. BEST_EFFORT Publisher + BEST_EFFORT Subscriber (✅ 호환)
5. VOLATILE Publisher + TRANSIENT_LOCAL Subscriber (❌ 비호환)
6. TRANSIENT_LOCAL Publisher + VOLATILE Subscriber (✅ 호환)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import time
import threading


class TestPublisher(Node):
    """테스트용 Publisher"""
    
    def __init__(self, name, topic, qos_profile, message_prefix):
        super().__init__(name)
        self.publisher = self.create_publisher(String, topic, qos_profile)
        self.message_prefix = message_prefix
        self.counter = 0
        
        # 1초마다 메시지 발행
        self.timer = self.create_timer(1.0, self.publish_message)
        
        qos_desc = self.describe_qos(qos_profile)
        self.get_logger().info(f'Publisher 시작: {name}')
        self.get_logger().info(f'  Topic: {topic}')
        self.get_logger().info(f'  QoS: {qos_desc}')
    
    def publish_message(self):
        msg = String()
        msg.data = f'{self.message_prefix} #{self.counter}'
        self.publisher.publish(msg)
        self.counter += 1
    
    def describe_qos(self, qos):
        rel = 'RELIABLE' if qos.reliability == ReliabilityPolicy.RELIABLE else 'BEST_EFFORT'
        dur = 'TRANSIENT_LOCAL' if qos.durability == DurabilityPolicy.TRANSIENT_LOCAL else 'VOLATILE'
        return f'{rel} / {dur}'


class TestSubscriber(Node):
    """테스트용 Subscriber"""
    
    def __init__(self, name, topic, qos_profile):
        super().__init__(name)
        self.received_count = 0
        self.last_message = None
        
        self.subscription = self.create_subscription(
            String,
            topic,
            self.message_callback,
            qos_profile
        )
        
        qos_desc = self.describe_qos(qos_profile)
        self.get_logger().info(f'Subscriber 시작: {name}')
        self.get_logger().info(f'  Topic: {topic}')
        self.get_logger().info(f'  QoS: {qos_desc}')
    
    def message_callback(self, msg):
        self.received_count += 1
        self.last_message = msg.data
        self.get_logger().info(f'✅ 수신: "{msg.data}" (총 {self.received_count}개)')
    
    def describe_qos(self, qos):
        rel = 'RELIABLE' if qos.reliability == ReliabilityPolicy.RELIABLE else 'BEST_EFFORT'
        dur = 'TRANSIENT_LOCAL' if qos.durability == DurabilityPolicy.TRANSIENT_LOCAL else 'VOLATILE'
        return f'{rel} / {dur}'


def create_qos(reliability, durability):
    """QoS 프로파일 생성"""
    qos = QoSProfile(
        reliability=reliability,
        durability=durability,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )
    return qos


def run_test(test_name, pub_reliability, pub_durability, sub_reliability, sub_durability, duration=5):
    """단일 테스트 실행"""
    print("\n" + "=" * 80)
    print(f"테스트: {test_name}")
    print("=" * 80)
    
    pub_rel = 'RELIABLE' if pub_reliability == ReliabilityPolicy.RELIABLE else 'BEST_EFFORT'
    pub_dur = 'TRANSIENT_LOCAL' if pub_durability == DurabilityPolicy.TRANSIENT_LOCAL else 'VOLATILE'
    sub_rel = 'RELIABLE' if sub_reliability == ReliabilityPolicy.RELIABLE else 'BEST_EFFORT'
    sub_dur = 'TRANSIENT_LOCAL' if sub_durability == DurabilityPolicy.TRANSIENT_LOCAL else 'VOLATILE'
    
    print(f"Publisher QoS: {pub_rel} / {pub_dur}")
    print(f"Subscriber QoS: {sub_rel} / {sub_dur}")
    print()
    
    # ROS2 초기화
    rclpy.init()
    
    try:
        # QoS 프로파일 생성
        pub_qos = create_qos(pub_reliability, pub_durability)
        sub_qos = create_qos(sub_reliability, sub_durability)
        
        # Publisher와 Subscriber 생성
        topic_name = f'/test_topic_{int(time.time())}'  # 유니크한 토픽명
        publisher = TestPublisher('test_pub', topic_name, pub_qos, test_name)
        subscriber = TestSubscriber('test_sub', topic_name, sub_qos)
        
        # Spin 시작
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)
        
        # 별도 스레드에서 실행
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        # 테스트 실행
        print(f"{duration}초 동안 테스트 중...\n")
        time.sleep(duration)
        
        # 결과 출력
        print("\n" + "-" * 80)
        print("테스트 결과:")
        print("-" * 80)
        print(f"발행된 메시지: 약 {duration}개 (1초에 1개)")
        print(f"수신된 메시지: {subscriber.received_count}개")
        
        if subscriber.received_count > 0:
            print(f"✅ 통신 성공! (마지막 메시지: \"{subscriber.last_message}\")")
            success = True
        else:
            print(f"❌ 통신 실패! (QoS 불일치로 인한 비호환)")
            success = False
        
        # 정리
        executor.shutdown()
        publisher.destroy_node()
        subscriber.destroy_node()
        
    finally:
        rclpy.shutdown()
    
    return success


def main():
    print("=" * 80)
    print("ROS2 QoS 호환성 실제 테스트")
    print("=" * 80)
    print()
    print("이 스크립트는 여러 QoS 조합을 테스트하여")
    print("어떤 경우에 통신이 되고 안 되는지 실제로 확인합니다.")
    print()
    input("준비되셨으면 Enter를 눌러주세요...")
    
    results = []
    
    # 테스트 1: RELIABLE + RELIABLE
    success = run_test(
        "Test 1: RELIABLE Pub + RELIABLE Sub",
        ReliabilityPolicy.RELIABLE,
        DurabilityPolicy.VOLATILE,
        ReliabilityPolicy.RELIABLE,
        DurabilityPolicy.VOLATILE,
        duration=5
    )
    results.append(("RELIABLE → RELIABLE", success))
    time.sleep(2)
    
    # 테스트 2: RELIABLE + BEST_EFFORT
    success = run_test(
        "Test 2: RELIABLE Pub + BEST_EFFORT Sub",
        ReliabilityPolicy.RELIABLE,
        DurabilityPolicy.VOLATILE,
        ReliabilityPolicy.BEST_EFFORT,
        DurabilityPolicy.VOLATILE,
        duration=5
    )
    results.append(("RELIABLE → BEST_EFFORT", success))
    time.sleep(2)
    
    # 테스트 3: BEST_EFFORT + RELIABLE
    success = run_test(
        "Test 3: BEST_EFFORT Pub + RELIABLE Sub",
        ReliabilityPolicy.BEST_EFFORT,
        DurabilityPolicy.VOLATILE,
        ReliabilityPolicy.RELIABLE,
        DurabilityPolicy.VOLATILE,
        duration=5
    )
    results.append(("BEST_EFFORT → RELIABLE", success))
    time.sleep(2)
    
    # 테스트 4: BEST_EFFORT + BEST_EFFORT
    success = run_test(
        "Test 4: BEST_EFFORT Pub + BEST_EFFORT Sub",
        ReliabilityPolicy.BEST_EFFORT,
        DurabilityPolicy.VOLATILE,
        ReliabilityPolicy.BEST_EFFORT,
        DurabilityPolicy.VOLATILE,
        duration=5
    )
    results.append(("BEST_EFFORT → BEST_EFFORT", success))
    time.sleep(2)
    
    # 테스트 5: VOLATILE + TRANSIENT_LOCAL
    success = run_test(
        "Test 5: VOLATILE Pub + TRANSIENT_LOCAL Sub",
        ReliabilityPolicy.RELIABLE,
        DurabilityPolicy.VOLATILE,
        ReliabilityPolicy.RELIABLE,
        DurabilityPolicy.TRANSIENT_LOCAL,
        duration=5
    )
    results.append(("VOLATILE → TRANSIENT_LOCAL", success))
    time.sleep(2)
    
    # 테스트 6: TRANSIENT_LOCAL + VOLATILE
    success = run_test(
        "Test 6: TRANSIENT_LOCAL Pub + VOLATILE Sub",
        ReliabilityPolicy.RELIABLE,
        DurabilityPolicy.TRANSIENT_LOCAL,
        ReliabilityPolicy.RELIABLE,
        DurabilityPolicy.VOLATILE,
        duration=5
    )
    results.append(("TRANSIENT_LOCAL → VOLATILE", success))
    
    # 최종 요약
    print("\n\n" + "=" * 80)
    print("전체 테스트 결과 요약")
    print("=" * 80)
    print()
    
    for test_desc, success in results:
        status = "✅ 통신 성공" if success else "❌ 통신 실패"
        print(f"{test_desc:40s} {status}")
    
    print("\n" + "=" * 80)
    print("QoS 호환성 규칙:")
    print("=" * 80)
    print("""
1. Reliability:
   ✅ RELIABLE Publisher는 BEST_EFFORT Subscriber와 통신 가능
   ❌ BEST_EFFORT Publisher는 RELIABLE Subscriber와 통신 불가
   → Publisher가 더 강한 QoS를 가져야 함

2. Durability:
   ✅ TRANSIENT_LOCAL Publisher는 VOLATILE Subscriber와 통신 가능
   ❌ VOLATILE Publisher는 TRANSIENT_LOCAL Subscriber와 통신 불가
   → Publisher가 더 강한 QoS를 가져야 함

3. 핵심 원칙:
   Publisher의 QoS ≥ Subscriber의 QoS
   """)


if __name__ == "__main__":
    main()
