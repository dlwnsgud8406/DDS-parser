#!/usr/bin/env python3
"""
QoS 테스트 Publisher

사용법:
    python scripts/qos_test_publisher.py --reliability RELIABLE --durability VOLATILE
    python scripts/qos_test_publisher.py --reliability BEST_EFFORT --durability VOLATILE
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import argparse


class QoSTestPublisher(Node):
    
    def __init__(self, reliability, durability):
        super().__init__('qos_test_publisher')
        
        # QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliability == 'RELIABLE' else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if durability == 'TRANSIENT_LOCAL' else DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher = self.create_publisher(String, '/qos_test_topic', qos)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.counter = 0
        
        print("=" * 60)
        print("QoS Test Publisher 시작")
        print("=" * 60)
        print(f"Topic: /qos_test_topic")
        print(f"Reliability: {reliability}")
        print(f"Durability: {durability}")
        print("=" * 60)
        print()
        print("1초마다 메시지를 발행합니다...")
        print("종료하려면 Ctrl+C를 누르세요.")
        print()
    
    def publish_message(self):
        msg = String()
        msg.data = f'Test message #{self.counter}'
        self.publisher.publish(msg)
        print(f'[{self.counter:3d}] 발행: "{msg.data}"')
        self.counter += 1


def main():
    parser = argparse.ArgumentParser(description='QoS 테스트용 Publisher')
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
    node = QoSTestPublisher(args.reliability, args.durability)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n종료합니다...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
