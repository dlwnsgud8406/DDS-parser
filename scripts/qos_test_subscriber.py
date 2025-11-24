#!/usr/bin/env python3
"""
QoS 테스트 Subscriber

사용법:
    python scripts/qos_test_subscriber.py --reliability RELIABLE --durability VOLATILE
    python scripts/qos_test_subscriber.py --reliability BEST_EFFORT --durability VOLATILE
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import argparse


class QoSTestSubscriber(Node):
    
    def __init__(self, reliability, durability):
        super().__init__('qos_test_subscriber')
        
        # QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliability == 'RELIABLE' else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if durability == 'TRANSIENT_LOCAL' else DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            String,
            '/qos_test_topic',
            self.message_callback,
            qos
        )
        
        self.received_count = 0
        self.start_time = self.get_clock().now()
        
        print("=" * 60)
        print("QoS Test Subscriber 시작")
        print("=" * 60)
        print(f"Topic: /qos_test_topic")
        print(f"Reliability: {reliability}")
        print(f"Durability: {durability}")
        print("=" * 60)
        print()
        print("메시지 수신 대기 중...")
        print("종료하려면 Ctrl+C를 누르세요.")
        print()
    
    def message_callback(self, msg):
        self.received_count += 1
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        print(f'[{self.received_count:3d}] ✅ 수신: "{msg.data}" (경과: {elapsed:.1f}초)')


def main():
    parser = argparse.ArgumentParser(description='QoS 테스트용 Subscriber')
    parser.add_argument(
        '--reliability',
        choices=['RELIABLE', 'BEST_EFFORT'],
        default='RELIABLE',
        help='Reliability 정책'
    )
    parser.add_argument(
        '--durability',
        choices=['VOLATILE', 'TRANSIENT_LOCAL'],
        default='VOLATILE',
        help='Durability 정책'
    )
    
    args = parser.parse_args()
    
    rclpy.init()
    node = QoSTestSubscriber(args.reliability, args.durability)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n총 {node.received_count}개 메시지 수신')
        print('종료합니다...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
