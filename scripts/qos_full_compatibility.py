#!/usr/bin/env python3
"""
QoS 호환성 전체 규칙

Reliability와 Durability 외에도 통신에 영향을 주는 모든 QoS 정책 확인
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class QoSCompatibilityRule:
    """QoS 호환성 규칙"""
    policy_name: str
    description: str
    compatibility_rule: str
    can_block_communication: bool
    examples: List[str]


# DDS 표준의 QoS 호환성 규칙
QOS_COMPATIBILITY_RULES = [
    QoSCompatibilityRule(
        policy_name="1. Reliability",
        description="메시지 전달 신뢰성",
        compatibility_rule="Publisher >= Subscriber (Publisher가 같거나 더 강해야 함)",
        can_block_communication=True,
        examples=[
            "✅ Pub: RELIABLE, Sub: RELIABLE → OK",
            "✅ Pub: RELIABLE, Sub: BEST_EFFORT → OK (Publisher가 더 강함)",
            "❌ Pub: BEST_EFFORT, Sub: RELIABLE → 통신 불가!"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="2. Durability",
        description="데이터 지속성",
        compatibility_rule="Publisher >= Subscriber (Publisher가 같거나 더 강해야 함)",
        can_block_communication=True,
        examples=[
            "✅ Pub: TRANSIENT_LOCAL, Sub: TRANSIENT_LOCAL → OK",
            "✅ Pub: TRANSIENT_LOCAL, Sub: VOLATILE → OK (Publisher가 더 강함)",
            "❌ Pub: VOLATILE, Sub: TRANSIENT_LOCAL → 통신 불가!"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="3. Deadline",
        description="메시지 최대 주기 제한",
        compatibility_rule="Publisher <= Subscriber (Publisher가 더 자주 보내야 함)",
        can_block_communication=True,
        examples=[
            "✅ Pub: 100ms, Sub: 200ms → OK (Publisher가 더 자주 보냄)",
            "❌ Pub: 200ms, Sub: 100ms → 통신 불가! (Subscriber 요구사항 미충족)",
            "✅ Pub: 무한, Sub: 무한 → OK (둘 다 제한 없음)"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="4. Liveliness",
        description="Publisher 생존 확인 메커니즘",
        compatibility_rule="정책 종류와 Lease Duration이 호환되어야 함",
        can_block_communication=True,
        examples=[
            "✅ Pub: AUTOMATIC, Sub: AUTOMATIC → OK",
            "❌ Pub: MANUAL, Sub: AUTOMATIC → 통신 불가 (정책 불일치)",
            "Lease Duration도 Publisher <= Subscriber 조건 필요"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="5. Ownership",
        description="다중 Publisher 제어권",
        compatibility_rule="Publisher와 Subscriber가 같은 정책을 사용해야 함",
        can_block_communication=True,
        examples=[
            "✅ Pub: SHARED, Sub: SHARED → OK",
            "✅ Pub: EXCLUSIVE, Sub: EXCLUSIVE → OK",
            "❌ Pub: SHARED, Sub: EXCLUSIVE → 통신 불가!"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="6. Presentation",
        description="메시지 순서 및 그룹화",
        compatibility_rule="Subscriber의 요구가 Publisher의 제공과 호환되어야 함",
        can_block_communication=True,
        examples=[
            "✅ Pub: ordered=true, Sub: ordered=false → OK",
            "❌ Pub: ordered=false, Sub: ordered=true → 통신 불가!",
            "✅ Pub: TOPIC scope, Sub: INSTANCE scope → OK"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="7. Partition",
        description="논리적 네트워크 분리",
        compatibility_rule="최소 하나의 Partition이 겹쳐야 함",
        can_block_communication=True,
        examples=[
            "✅ Pub: ['A', 'B'], Sub: ['B', 'C'] → OK ('B' 겹침)",
            "❌ Pub: ['A'], Sub: ['B'] → 통신 불가! (겹치는 Partition 없음)",
            "✅ Pub: [], Sub: [] → OK (기본 Partition)"
        ]
    ),
    
    # 여기서부터는 통신을 막지 않지만 영향을 주는 QoS들
    
    QoSCompatibilityRule(
        policy_name="8. History & Depth",
        description="메시지 큐 관리",
        compatibility_rule="호환성 검사 없음 (각자 독립적으로 사용)",
        can_block_communication=False,
        examples=[
            "✅ Pub: KEEP_LAST(10), Sub: KEEP_LAST(100) → OK (각자 관리)",
            "✅ Pub: KEEP_ALL, Sub: KEEP_LAST(5) → OK",
            "영향: Subscriber의 depth가 작으면 메시지 누락 가능"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="9. Lifespan",
        description="메시지 유효 기간",
        compatibility_rule="Publisher만 설정 (Subscriber는 자동으로 받음)",
        can_block_communication=False,
        examples=[
            "✅ Pub: 5초, Sub: 설정 안 함 → OK",
            "영향: 5초 지난 메시지는 자동 삭제됨",
            "통신 차단은 안 하지만 오래된 데이터는 못 받음"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="10. Time-Based Filter",
        description="Subscriber 수신 간격 제한",
        compatibility_rule="Subscriber만 설정 (독립적)",
        can_block_communication=False,
        examples=[
            "✅ Pub: 제한 없음, Sub: 100ms마다 → OK",
            "영향: Subscriber가 더 천천히 받음 (일부 메시지 무시)",
            "통신은 되지만 Subscriber가 선택적으로 받음"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="11. Latency Budget",
        description="지연 시간 힌트",
        compatibility_rule="호환성 검사 없음 (최적화 힌트일 뿐)",
        can_block_communication=False,
        examples=[
            "✅ 어떤 조합이든 OK (힌트일 뿐)",
            "영향: DDS가 네트워크 최적화 시 참고",
            "통신 차단 안 함"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="12. Transport Priority",
        description="전송 우선순위",
        compatibility_rule="호환성 검사 없음 (네트워크 계층 힌트)",
        can_block_communication=False,
        examples=[
            "✅ 어떤 조합이든 OK",
            "영향: 네트워크 큐 우선순위에만 영향",
            "통신 차단 안 함"
        ]
    ),
    
    QoSCompatibilityRule(
        policy_name="13. Resource Limits",
        description="메모리 제한",
        compatibility_rule="호환성 검사 없음 (각자 관리)",
        can_block_communication=False,
        examples=[
            "✅ 어떤 조합이든 OK",
            "영향: 각자의 메모리 사용량에만 영향",
            "제한 초과 시 메시지 드롭 가능"
        ]
    ),
]


def print_compatibility_summary():
    """QoS 호환성 요약"""
    print("=" * 80)
    print("QoS 호환성 규칙 전체 정리")
    print("=" * 80)
    print()
    
    # 통신을 막을 수 있는 QoS
    blocking_qos = [rule for rule in QOS_COMPATIBILITY_RULES if rule.can_block_communication]
    non_blocking_qos = [rule for rule in QOS_COMPATIBILITY_RULES if not rule.can_block_communication]
    
    print("🚫 통신을 막을 수 있는 QoS 정책 (반드시 확인!)")
    print("=" * 80)
    print()
    
    for rule in blocking_qos:
        print(f"{rule.policy_name}")
        print(f"   설명: {rule.description}")
        print(f"   호환 규칙: {rule.compatibility_rule}")
        print(f"   예시:")
        for example in rule.examples:
            print(f"      {example}")
        print()
    
    print("\n" + "=" * 80)
    print("✅ 통신을 막지 않는 QoS 정책 (성능/동작에만 영향)")
    print("=" * 80)
    print()
    
    for rule in non_blocking_qos:
        print(f"{rule.policy_name}")
        print(f"   설명: {rule.description}")
        print(f"   호환 규칙: {rule.compatibility_rule}")
        print(f"   예시:")
        for example in rule.examples:
            print(f"      {example}")
        print()


def print_decision_tree():
    """QoS 호환성 결정 트리"""
    print("\n" + "=" * 80)
    print("📊 통신 가능 여부 판단 플로우차트")
    print("=" * 80)
    print()
    
    print("""
1. Reliability 확인
   ├─ Publisher >= Subscriber? 
   │  ├─ YES → 다음 단계
   │  └─ NO  → ❌ 통신 불가!
   
2. Durability 확인
   ├─ Publisher >= Subscriber?
   │  ├─ YES → 다음 단계
   │  └─ NO  → ❌ 통신 불가!
   
3. Deadline 확인
   ├─ Publisher <= Subscriber? (또는 둘 다 무한)
   │  ├─ YES → 다음 단계
   │  └─ NO  → ❌ 통신 불가!
   
4. Liveliness 확인
   ├─ 정책 종류 일치? AND Lease Duration 호환?
   │  ├─ YES → 다음 단계
   │  └─ NO  → ❌ 통신 불가!
   
5. Ownership 확인
   ├─ 정책 일치 (SHARED ↔ SHARED 또는 EXCLUSIVE ↔ EXCLUSIVE)?
   │  ├─ YES → 다음 단계
   │  └─ NO  → ❌ 통신 불가!
   
6. Presentation 확인
   ├─ Subscriber 요구사항 <= Publisher 제공?
   │  ├─ YES → 다음 단계
   │  └─ NO  → ❌ 통신 불가!
   
7. Partition 확인
   ├─ 최소 하나의 Partition 겹침?
   │  ├─ YES → ✅ 통신 가능!
   │  └─ NO  → ❌ 통신 불가!
   
8. (선택) History, Depth, Lifespan 등
   └─ 통신은 되지만 성능/동작에 영향
    """)


def print_practical_guide():
    """실전 가이드"""
    print("\n" + "=" * 80)
    print("💡 실전 가이드")
    print("=" * 80)
    print()
    
    print("Q: Reliability와 Durability만 맞추면 되나요?")
    print("A: 아니요! 다음 7가지를 모두 확인해야 합니다:")
    print()
    
    checklist = [
        ("1. Reliability", "MUST", "Publisher >= Subscriber"),
        ("2. Durability", "MUST", "Publisher >= Subscriber"),
        ("3. Deadline", "MUST", "Publisher <= Subscriber"),
        ("4. Liveliness", "MUST", "정책 일치 + Lease 호환"),
        ("5. Ownership", "MUST", "정책 일치"),
        ("6. Presentation", "MUST", "Subscriber 요구 <= Publisher 제공"),
        ("7. Partition", "MUST", "최소 1개 겹침"),
    ]
    
    for item, importance, rule in checklist:
        print(f"   {item:20s} [{importance}] {rule}")
    
    print()
    print("Q: 그럼 다 확인해야 하나요?")
    print("A: 대부분의 경우 기본값이면 OK입니다:")
    print()
    
    print("   일반적인 경우 (90%):")
    print("      - Reliability, Durability만 신경 쓰면 됨")
    print("      - 나머지는 기본값이 보통 호환됨")
    print()
    
    print("   특수한 경우 (10%):")
    print("      - Deadline 설정한 경우")
    print("      - Ownership EXCLUSIVE 사용")
    print("      - Partition으로 네트워크 분리")
    print("      - 고급 Presentation 기능 사용")
    print()
    
    print("Q: ROS2에서 주의할 점은?")
    print("A: ROS2는 대부분 기본값을 잘 설정해둡니다:")
    print()
    print("   자주 문제되는 경우:")
    print("      ❌ BEST_EFFORT Publisher + RELIABLE Subscriber")
    print("      ❌ VOLATILE Publisher + TRANSIENT_LOCAL Subscriber")
    print("      ❌ Partition 다르게 설정")
    print()
    print("   해결책:")
    print("      ✅ ros2 topic info /topic_name -v 로 QoS 확인")
    print("      ✅ 불일치 발견 시 경고 메시지 출력됨")
    print("      ✅ Publisher를 더 강하게 설정하면 대부분 해결")


def main():
    print_compatibility_summary()
    print_decision_tree()
    print_practical_guide()
    
    # 최종 요약
    print("\n" + "=" * 80)
    print("📌 핵심 요약")
    print("=" * 80)
    print()
    print("❓ Reliability와 Durability만 맞으면 통신되나요?")
    print()
    print("   ❌ 아니요! 총 7가지 QoS가 호환되어야 합니다:")
    print()
    print("      1️⃣ Reliability      (가장 중요)")
    print("      2️⃣ Durability       (가장 중요)")
    print("      3️⃣ Deadline         (설정 시)")
    print("      4️⃣ Liveliness       (설정 시)")
    print("      5️⃣ Ownership        (EXCLUSIVE 시)")
    print("      6️⃣ Presentation     (고급 기능 시)")
    print("      7️⃣ Partition        (네트워크 분리 시)")
    print()
    print("   ✅ 하지만 실무에서는:")
    print("      → 90% 경우: Reliability + Durability만 확인하면 OK")
    print("      → 10% 경우: 나머지도 확인 필요 (고급 기능 사용 시)")
    print()
    print("   💡 결론:")
    print("      기본값 사용 시 → Reliability + Durability만 신경 쓰면 됨")
    print("      커스텀 설정 시 → 7가지 모두 확인 필요")


if __name__ == "__main__":
    main()
