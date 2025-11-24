#!/usr/bin/env python3
"""
ROS2 QoS 기본값과 커스텀 설정 비교

이 스크립트는 ROS2에서 기본 QoS와 개발자가 커스텀한 QoS를 비교합니다.
"""

from enum import Enum
from dataclasses import dataclass
from typing import Dict


class Reliability(Enum):
    BEST_EFFORT = "BEST_EFFORT"
    RELIABLE = "RELIABLE"


class Durability(Enum):
    VOLATILE = "VOLATILE"
    TRANSIENT_LOCAL = "TRANSIENT_LOCAL"


class History(Enum):
    KEEP_LAST = "KEEP_LAST"
    KEEP_ALL = "KEEP_ALL"


@dataclass
class QoSProfile:
    name: str
    reliability: Reliability
    durability: Durability
    history: History
    depth: int
    description: str


# ROS2 제공 기본 QoS 프로파일들
DEFAULT_QOS_PROFILES = {
    "sensor_data": QoSProfile(
        name="Sensor Data",
        reliability=Reliability.BEST_EFFORT,
        durability=Durability.VOLATILE,
        history=History.KEEP_LAST,
        depth=5,
        description="센서 데이터용 (속도 중시). 예: /scan, /camera/image"
    ),
    
    "parameters": QoSProfile(
        name="Parameters",
        reliability=Reliability.RELIABLE,
        durability=Durability.VOLATILE,
        history=History.KEEP_LAST,
        depth=1000,
        description="파라미터 이벤트용 (신뢰성 중시). 예: /parameter_events"
    ),
    
    "services": QoSProfile(
        name="Services Default",
        reliability=Reliability.RELIABLE,
        durability=Durability.VOLATILE,
        history=History.KEEP_LAST,
        depth=10,
        description="서비스 Request/Reply용 (신뢰성 중시). 예: /set_parameters"
    ),
    
    "system_default": QoSProfile(
        name="System Default",
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
        history=History.KEEP_LAST,
        depth=1,
        description="시스템 정보용 (지속성 중시). 예: /robot_description"
    ),
    
    "default": QoSProfile(
        name="Default (기본값)",
        reliability=Reliability.RELIABLE,
        durability=Durability.VOLATILE,
        history=History.KEEP_LAST,
        depth=10,
        description="일반 토픽의 기본값. 명시하지 않으면 이것 사용"
    ),
}


# 실제 로봇에서 자주 사용하는 커스텀 QoS
CUSTOM_QOS_EXAMPLES = {
    "lidar_scan": QoSProfile(
        name="LiDAR Scan (커스텀)",
        reliability=Reliability.BEST_EFFORT,
        durability=Durability.VOLATILE,
        history=History.KEEP_LAST,
        depth=1,  # 최신 스캔만
        description="실시간 라이다 스캔. 지연 최소화를 위해 depth=1로 커스텀"
    ),
    
    "map": QoSProfile(
        name="Map (커스텀)",
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
        history=History.KEEP_LAST,
        depth=1,
        description="지도 데이터. 나중에 구독해도 받을 수 있도록 TRANSIENT_LOCAL 사용"
    ),
    
    "cmd_vel": QoSProfile(
        name="Command Velocity (커스텀)",
        reliability=Reliability.RELIABLE,  # 안전을 위해 RELIABLE
        durability=Durability.VOLATILE,
        history=History.KEEP_LAST,
        depth=1,
        description="로봇 제어 명령. 누락되면 위험하므로 RELIABLE로 변경"
    ),
    
    "tf": QoSProfile(
        name="TF (커스텀)",
        reliability=Reliability.RELIABLE,
        durability=Durability.VOLATILE,
        history=History.KEEP_LAST,
        depth=100,  # 많은 변환 저장
        description="좌표 변환. depth를 크게 해서 오래된 변환도 저장"
    ),
}


def print_qos_profile(profile: QoSProfile, indent=0):
    """QoS 프로파일을 예쁘게 출력"""
    prefix = "  " * indent
    print(f"{prefix}📋 {profile.name}")
    print(f"{prefix}   Reliability: {profile.reliability.value}")
    print(f"{prefix}   Durability:  {profile.durability.value}")
    print(f"{prefix}   History:     {profile.history.value}")
    print(f"{prefix}   Depth:       {profile.depth}")
    print(f"{prefix}   📝 {profile.description}")


def main():
    print("=" * 80)
    print("ROS2 QoS: 기본값 vs 커스텀 설정")
    print("=" * 80)
    print()
    
    # 1. ROS2 기본 QoS 프로파일
    print("🔵 ROS2가 제공하는 기본 QoS 프로파일")
    print("=" * 80)
    print()
    print("이것들은 ROS2가 미리 정의해둔 것입니다.")
    print("개발자가 명시적으로 설정하지 않으면 자동으로 사용됩니다.")
    print()
    
    for key, profile in DEFAULT_QOS_PROFILES.items():
        print_qos_profile(profile)
        print()
    
    # 2. 커스텀 QoS 예시
    print("\n" + "=" * 80)
    print("🔴 개발자가 커스텀한 QoS 프로파일 예시")
    print("=" * 80)
    print()
    print("실제 로봇 개발에서는 필요에 따라 QoS를 커스텀합니다.")
    print("성능, 안전성, 네트워크 상황에 맞게 조정합니다.")
    print()
    
    for key, profile in CUSTOM_QOS_EXAMPLES.items():
        print_qos_profile(profile)
        print()
    
    # 3. 언제 기본값? 언제 커스텀?
    print("\n" + "=" * 80)
    print("💡 언제 기본값을 쓰고, 언제 커스텀할까?")
    print("=" * 80)
    print()
    
    scenarios = [
        {
            "situation": "초보 개발자가 간단한 토픽 만들 때",
            "choice": "기본값 사용",
            "reason": "ROS2의 'default' QoS가 대부분 상황에서 잘 작동함",
            "code": "pub = node.create_publisher(String, '/my_topic', 10)  # QoS 명시 안 함"
        },
        {
            "situation": "센서 데이터 (카메라, 라이다) 전송",
            "choice": "기본값 사용 또는 약간 수정",
            "reason": "ROS2의 'sensor_data' QoS가 이미 최적화되어 있음",
            "code": "pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)"
        },
        {
            "situation": "로봇 제어 명령 (cmd_vel)",
            "choice": "커스텀 필요",
            "reason": "안전상 RELIABLE로 바꾸거나, 실시간성 위해 depth=1로 설정",
            "code": "qos = QoSProfile(reliability=RELIABLE, depth=1)\npub = create_publisher(Twist, '/cmd_vel', qos)"
        },
        {
            "situation": "지도, 로봇 모델 같은 정적 데이터",
            "choice": "커스텀 필요",
            "reason": "나중에 시작한 노드도 받아야 하므로 TRANSIENT_LOCAL 필수",
            "code": "qos = QoSProfile(durability=TRANSIENT_LOCAL, depth=1)\npub = create_publisher(OccupancyGrid, '/map', qos)"
        },
        {
            "situation": "네트워크가 불안정한 환경 (WiFi)",
            "choice": "커스텀 필요",
            "reason": "패킷 손실 대비해서 RELIABLE + depth 증가",
            "code": "qos = QoSProfile(reliability=RELIABLE, depth=50)\npub = create_publisher(String, '/status', qos)"
        },
        {
            "situation": "고주파 제어 루프 (1000Hz 이상)",
            "choice": "커스텀 필요",
            "reason": "지연 최소화 위해 BEST_EFFORT + depth=1",
            "code": "qos = QoSProfile(reliability=BEST_EFFORT, depth=1)\npub = create_publisher(JointState, '/joint_commands', qos)"
        },
    ]
    
    for i, scenario in enumerate(scenarios, 1):
        print(f"{i}. 상황: {scenario['situation']}")
        print(f"   선택: {scenario['choice']}")
        print(f"   이유: {scenario['reason']}")
        print(f"   코드 예시:")
        for line in scenario['code'].split('\n'):
            print(f"      {line}")
        print()
    
    # 4. 실전 팁
    print("=" * 80)
    print("🎯 실전 팁")
    print("=" * 80)
    print()
    
    tips = [
        "1. 처음에는 기본값으로 시작하세요",
        "   → 문제가 생기면 그때 커스텀",
        "",
        "2. Publisher와 Subscriber의 QoS가 호환되는지 확인하세요",
        "   → 'ros2 topic info /topic_name -v' 명령으로 확인 가능",
        "",
        "3. 같은 토픽이라도 노드마다 다른 QoS를 쓸 수 있어요",
        "   → 하나는 RELIABLE, 다른 하나는 BEST_EFFORT (호환 가능)",
        "",
        "4. 성능 vs 신뢰성의 트레이드오프",
        "   → RELIABLE: 느리지만 안전",
        "   → BEST_EFFORT: 빠르지만 손실 가능",
        "",
        "5. ROS2의 미리 정의된 프로파일 활용",
        "   → qos_profile_sensor_data",
        "   → qos_profile_parameters",
        "   → qos_profile_services_default",
        "   → qos_profile_system_default",
    ]
    
    for tip in tips:
        print(f"   {tip}")
    
    # 5. 통계
    print("\n" + "=" * 80)
    print("📊 실제 로봇 프로젝트 통계 (경험상)")
    print("=" * 80)
    print()
    print("   70% - 기본값 그대로 사용")
    print("   20% - ROS2 제공 프로파일 사용 (sensor_data 등)")
    print("   10% - 완전 커스텀")
    print()
    print("   → 대부분은 기본값으로 충분합니다!")
    print("   → 성능 문제나 특수한 요구사항이 있을 때만 커스텀")


if __name__ == "__main__":
    main()
