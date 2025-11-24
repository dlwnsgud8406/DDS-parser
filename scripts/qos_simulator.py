#!/usr/bin/env python3
"""
QoS 호환성 시뮬레이터

ROS2 없이도 QoS 호환성 규칙을 시뮬레이션으로 테스트할 수 있습니다.
실제 통신은 하지 않고, DDS 표준의 QoS 호환성 규칙만 검증합니다.
"""

from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple


class Reliability(Enum):
    BEST_EFFORT = 0
    RELIABLE = 1
    
    def __lt__(self, other):
        return self.value < other.value
    
    def __le__(self, other):
        return self.value <= other.value
    
    def __str__(self):
        return self.name


class Durability(Enum):
    VOLATILE = 0
    TRANSIENT_LOCAL = 1
    
    def __lt__(self, other):
        return self.value < other.value
    
    def __le__(self, other):
        return self.value <= other.value
    
    def __str__(self):
        return self.name


@dataclass
class QoSProfile:
    reliability: Reliability
    durability: Durability
    
    def __str__(self):
        return f"{self.reliability} / {self.durability}"


def check_qos_compatibility(publisher_qos: QoSProfile, subscriber_qos: QoSProfile) -> Tuple[bool, str]:
    """
    DDS QoS 호환성 규칙 검사
    
    규칙:
    - Publisher의 Reliability >= Subscriber의 Reliability
    - Publisher의 Durability >= Subscriber의 Durability
    
    Returns:
        (호환 여부, 이유)
    """
    reliability_ok = publisher_qos.reliability >= subscriber_qos.reliability
    durability_ok = publisher_qos.durability >= subscriber_qos.durability
    
    if reliability_ok and durability_ok:
        return True, "✅ QoS 호환 - 통신 가능"
    
    reasons = []
    if not reliability_ok:
        reasons.append(
            f"❌ Reliability 불일치: Publisher({publisher_qos.reliability}) < Subscriber({subscriber_qos.reliability})"
        )
    if not durability_ok:
        reasons.append(
            f"❌ Durability 불일치: Publisher({publisher_qos.durability}) < Subscriber({subscriber_qos.durability})"
        )
    
    return False, " | ".join(reasons)


def run_test_case(test_name: str, pub_qos: QoSProfile, sub_qos: QoSProfile):
    """단일 테스트 케이스 실행"""
    print("\n" + "=" * 80)
    print(f"테스트: {test_name}")
    print("=" * 80)
    print(f"Publisher QoS:  {pub_qos}")
    print(f"Subscriber QoS: {sub_qos}")
    print()
    
    compatible, reason = check_qos_compatibility(pub_qos, sub_qos)
    
    print(reason)
    
    if compatible:
        print("💬 시뮬레이션: Publisher가 메시지를 발행하면 Subscriber가 수신합니다.")
    else:
        print("💬 시뮬레이션: Publisher가 메시지를 발행해도 Subscriber는 수신하지 못합니다.")
    
    return compatible


def main():
    print("=" * 80)
    print("QoS 호환성 시뮬레이터")
    print("=" * 80)
    print()
    print("이 프로그램은 DDS 표준의 QoS 호환성 규칙을 시뮬레이션합니다.")
    print("실제 ROS2 통신 없이도 QoS 호환성을 검증할 수 있습니다.")
    print()
    input("준비되셨으면 Enter를 눌러주세요...")
    
    results: List[Tuple[str, bool]] = []
    
    # 테스트 1: RELIABLE + RELIABLE
    compatible = run_test_case(
        "Test 1: RELIABLE Pub + RELIABLE Sub",
        QoSProfile(Reliability.RELIABLE, Durability.VOLATILE),
        QoSProfile(Reliability.RELIABLE, Durability.VOLATILE)
    )
    results.append(("RELIABLE → RELIABLE", compatible))
    input("\n계속하려면 Enter를 눌러주세요...")
    
    # 테스트 2: RELIABLE + BEST_EFFORT
    compatible = run_test_case(
        "Test 2: RELIABLE Pub + BEST_EFFORT Sub",
        QoSProfile(Reliability.RELIABLE, Durability.VOLATILE),
        QoSProfile(Reliability.BEST_EFFORT, Durability.VOLATILE)
    )
    results.append(("RELIABLE → BEST_EFFORT", compatible))
    input("\n계속하려면 Enter를 눌러주세요...")
    
    # 테스트 3: BEST_EFFORT + RELIABLE
    compatible = run_test_case(
        "Test 3: BEST_EFFORT Pub + RELIABLE Sub",
        QoSProfile(Reliability.BEST_EFFORT, Durability.VOLATILE),
        QoSProfile(Reliability.RELIABLE, Durability.VOLATILE)
    )
    results.append(("BEST_EFFORT → RELIABLE", compatible))
    input("\n계속하려면 Enter를 눌러주세요...")
    
    # 테스트 4: BEST_EFFORT + BEST_EFFORT
    compatible = run_test_case(
        "Test 4: BEST_EFFORT Pub + BEST_EFFORT Sub",
        QoSProfile(Reliability.BEST_EFFORT, Durability.VOLATILE),
        QoSProfile(Reliability.BEST_EFFORT, Durability.VOLATILE)
    )
    results.append(("BEST_EFFORT → BEST_EFFORT", compatible))
    input("\n계속하려면 Enter를 눌러주세요...")
    
    # 테스트 5: VOLATILE + TRANSIENT_LOCAL
    compatible = run_test_case(
        "Test 5: VOLATILE Pub + TRANSIENT_LOCAL Sub",
        QoSProfile(Reliability.RELIABLE, Durability.VOLATILE),
        QoSProfile(Reliability.RELIABLE, Durability.TRANSIENT_LOCAL)
    )
    results.append(("VOLATILE → TRANSIENT_LOCAL", compatible))
    input("\n계속하려면 Enter를 눌러주세요...")
    
    # 테스트 6: TRANSIENT_LOCAL + VOLATILE
    compatible = run_test_case(
        "Test 6: TRANSIENT_LOCAL Pub + VOLATILE Sub",
        QoSProfile(Reliability.RELIABLE, Durability.TRANSIENT_LOCAL),
        QoSProfile(Reliability.RELIABLE, Durability.VOLATILE)
    )
    results.append(("TRANSIENT_LOCAL → VOLATILE", compatible))
    input("\n계속하려면 Enter를 눌러주세요...")
    
    # 테스트 7: 복합 - 모든 불일치
    compatible = run_test_case(
        "Test 7: 최악의 경우 (모든 QoS 불일치)",
        QoSProfile(Reliability.BEST_EFFORT, Durability.VOLATILE),
        QoSProfile(Reliability.RELIABLE, Durability.TRANSIENT_LOCAL)
    )
    results.append(("BEST_EFFORT/VOLATILE → RELIABLE/TRANSIENT_LOCAL", compatible))
    input("\n계속하려면 Enter를 눌러주세요...")
    
    # 테스트 8: 복합 - 모든 일치
    compatible = run_test_case(
        "Test 8: 최상의 경우 (모든 QoS 강함)",
        QoSProfile(Reliability.RELIABLE, Durability.TRANSIENT_LOCAL),
        QoSProfile(Reliability.BEST_EFFORT, Durability.VOLATILE)
    )
    results.append(("RELIABLE/TRANSIENT_LOCAL → BEST_EFFORT/VOLATILE", compatible))
    
    # 최종 요약
    print("\n\n" + "=" * 80)
    print("전체 테스트 결과 요약")
    print("=" * 80)
    print()
    
    for test_desc, compatible in results:
        status = "✅ 호환 (통신 가능)" if compatible else "❌ 비호환 (통신 불가)"
        print(f"{test_desc:50s} {status}")
    
    # 통계
    total = len(results)
    success = sum(1 for _, c in results if c)
    fail = total - success
    
    print("\n" + "=" * 80)
    print(f"통계: 총 {total}개 테스트 중 {success}개 호환, {fail}개 비호환")
    print("=" * 80)
    
    # 규칙 설명
    print("\n" + "=" * 80)
    print("QoS 호환성 규칙 (DDS 표준)")
    print("=" * 80)
    print("""
1. Reliability (신뢰성):
   ✅ RELIABLE >= RELIABLE      (Publisher가 RELIABLE, Subscriber가 RELIABLE)
   ✅ RELIABLE >= BEST_EFFORT   (Publisher가 더 강함)
   ❌ BEST_EFFORT < RELIABLE    (Publisher가 약함 - 비호환)
   ✅ BEST_EFFORT >= BEST_EFFORT

2. Durability (내구성):
   ✅ TRANSIENT_LOCAL >= TRANSIENT_LOCAL
   ✅ TRANSIENT_LOCAL >= VOLATILE    (Publisher가 더 강함)
   ❌ VOLATILE < TRANSIENT_LOCAL     (Publisher가 약함 - 비호환)
   ✅ VOLATILE >= VOLATILE

3. 핵심 원칙:
   Publisher의 QoS >= Subscriber의 QoS
   
   즉, Publisher는 Subscriber가 요구하는 것보다 "같거나 더 나은" QoS를 제공해야 합니다.

4. 실제 예시:
   - 센서 데이터: BEST_EFFORT (빠름) ↔ BEST_EFFORT (빠름) ✅
   - 지도 데이터: RELIABLE + TRANSIENT_LOCAL (강함) → RELIABLE + VOLATILE (약함) ✅
   - 잘못된 예: BEST_EFFORT (약함) → RELIABLE (강함) ❌
   """)


if __name__ == "__main__":
    main()
