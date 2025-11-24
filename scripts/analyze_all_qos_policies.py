#!/usr/bin/env python3
"""
ROS2/DDS QoS 정책 전체 목록 및 실제 데이터 분석

1. DDS 표준의 모든 QoS 정책 설명
2. 실제 PCAP에서 추출한 QoS 분석
"""

import sys
from pathlib import Path
from collections import Counter

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor
from src.transformer import EndpointMapper, QoSAnalyzer


# DDS/ROS2의 모든 QoS 정책
QOS_POLICIES = {
    "Reliability": {
        "설명": "메시지 전달 신뢰성",
        "값": {
            "BEST_EFFORT": "빠르지만 손실 가능. UDP와 유사",
            "RELIABLE": "느리지만 반드시 도착. TCP와 유사"
        },
        "기본값": "RELIABLE (ROS2 기본)",
        "영향": "네트워크 성능, 지연 시간"
    },
    
    "Durability": {
        "설명": "데이터 지속성 (과거 데이터 보존)",
        "값": {
            "VOLATILE": "현재 데이터만. 구독 시작 후 데이터만 수신",
            "TRANSIENT_LOCAL": "최근 데이터 보존. 나중에 구독해도 받을 수 있음",
            "TRANSIENT": "디스크에 저장 (ROS2 미지원)",
            "PERSISTENT": "영구 저장 (ROS2 미지원)"
        },
        "기본값": "VOLATILE",
        "영향": "메모리 사용량, Late Joiner 지원"
    },
    
    "History": {
        "설명": "메시지 큐 관리 정책",
        "값": {
            "KEEP_LAST": "최근 N개만 보관 (depth로 지정)",
            "KEEP_ALL": "모든 메시지 보관 (메모리 허용 시)"
        },
        "기본값": "KEEP_LAST",
        "영향": "메모리 사용량, 메시지 손실"
    },
    
    "Depth": {
        "설명": "큐 크기 (History가 KEEP_LAST일 때)",
        "값": "1 ~ 수천 (정수)",
        "기본값": "10 (ROS2)",
        "영향": "메모리 사용량, 버퍼 오버플로"
    },
    
    "Deadline": {
        "설명": "메시지 최대 주기. 이 시간 내 도착 안 하면 경고",
        "값": "시간 (Duration)",
        "기본값": "무한 (제한 없음)",
        "영향": "실시간성 모니터링"
    },
    
    "Lifespan": {
        "설명": "메시지 유효 기간. 오래된 메시지는 자동 폐기",
        "값": "시간 (Duration)",
        "기본값": "무한 (만료 안 함)",
        "영향": "오래된 데이터 필터링"
    },
    
    "Liveliness": {
        "설명": "Publisher 생존 여부 확인 메커니즘",
        "값": {
            "AUTOMATIC": "자동으로 생존 신호 전송",
            "MANUAL_BY_TOPIC": "토픽마다 수동으로 신호",
            "MANUAL_BY_PARTICIPANT": "Participant 전체에 수동 신호"
        },
        "기본값": "AUTOMATIC",
        "영향": "장애 감지 속도"
    },
    
    "Lease Duration": {
        "설명": "Liveliness 신호 간격 (이 시간 내 신호 없으면 죽은 것으로 간주)",
        "값": "시간 (Duration)",
        "기본값": "무한",
        "영향": "장애 감지 민감도"
    },
    
    "Ownership": {
        "설명": "여러 Publisher 중 누가 제어권을 가질지",
        "값": {
            "SHARED": "모든 Publisher가 동등하게 전송",
            "EXCLUSIVE": "하나의 Publisher만 활성 (strength로 결정)"
        },
        "기본값": "SHARED",
        "영향": "다중 Publisher 관리"
    },
    
    "Ownership Strength": {
        "설명": "EXCLUSIVE Ownership일 때 우선순위",
        "값": "0 ~ 2^31 (정수)",
        "기본값": "0",
        "영향": "Publisher 우선순위"
    },
    
    "Resource Limits": {
        "설명": "메모리, 인스턴스 수 제한",
        "값": {
            "max_samples": "최대 샘플 수",
            "max_instances": "최대 인스턴스 수",
            "max_samples_per_instance": "인스턴스당 최대 샘플"
        },
        "기본값": "DDS 구현마다 다름",
        "영향": "메모리 사용량, OOM 방지"
    },
    
    "Presentation": {
        "설명": "메시지 그룹화 및 순서 보장",
        "값": {
            "access_scope": "INSTANCE/TOPIC/GROUP",
            "coherent_access": "그룹 내 원자성",
            "ordered_access": "순서 보장"
        },
        "기본값": "INSTANCE, False, False",
        "영향": "트랜잭션, 순서 보장"
    },
    
    "Partition": {
        "설명": "논리적 네트워크 분리 (같은 Partition만 통신)",
        "값": "문자열 배열",
        "기본값": "빈 배열 (모두 통신)",
        "영향": "네트워크 격리, 보안"
    },
    
    "Time-Based Filter": {
        "설명": "Subscriber가 받는 최소 메시지 간격",
        "값": "시간 (Duration)",
        "기본값": "0 (모두 받음)",
        "영향": "Subscriber CPU 사용량"
    },
    
    "Latency Budget": {
        "설명": "허용 가능한 최대 지연 시간 힌트",
        "값": "시간 (Duration)",
        "기본값": "0 (제한 없음)",
        "영향": "네트워크 최적화 힌트"
    },
    
    "Transport Priority": {
        "설명": "메시지 전송 우선순위 (네트워크 QoS)",
        "값": "0 ~ 2^31 (정수)",
        "기본값": "0",
        "영향": "네트워크 계층 우선순위"
    },
}


def print_all_qos_policies():
    """모든 QoS 정책 설명 출력"""
    print("=" * 80)
    print("DDS/ROS2 QoS 정책 전체 목록")
    print("=" * 80)
    print()
    print(f"총 {len(QOS_POLICIES)}가지 QoS 정책이 있습니다.")
    print()
    
    for i, (name, info) in enumerate(QOS_POLICIES.items(), 1):
        print(f"{i}. {name}")
        print(f"   설명: {info['설명']}")
        
        if isinstance(info['값'], dict):
            print(f"   값:")
            for val_name, val_desc in info['값'].items():
                print(f"     • {val_name}: {val_desc}")
        else:
            print(f"   값: {info['값']}")
        
        print(f"   기본값: {info['기본값']}")
        print(f"   영향: {info['영향']}")
        print()


def analyze_pcap_qos(pcap_file, max_packets=5000):
    """PCAP 파일에서 실제 QoS 분석"""
    print("=" * 80)
    print(f"실제 데이터 분석: {pcap_file}")
    print("=" * 80)
    print()
    
    # 1. PCAP 파싱
    print("[1/3] PCAP 파싱...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage 파싱\n")
    
    # 2. SEDP 매핑
    print("[2/3] SEDP 매핑...")
    endpoint_mapper = EndpointMapper()
    endpoint_mapper.build_mapping(submessages)
    enriched_submessages = endpoint_mapper.enrich_submessages(submessages)
    print(f"  ✓ 완료\n")
    
    # 3. QoS 추론
    print("[3/3] QoS 추론...")
    qos_analyzer = QoSAnalyzer()
    topic_qos_map = qos_analyzer.analyze_messages(enriched_submessages)
    print(f"  ✓ {len(topic_qos_map)}개 토픽 분석\n")
    
    # 통계 수집
    reliability_counter = Counter()
    durability_counter = Counter()
    frequency_ranges = {
        "매우 빠름 (>100Hz)": 0,
        "빠름 (10-100Hz)": 0,
        "보통 (1-10Hz)": 0,
        "느림 (<1Hz)": 0,
        "정적 (0Hz)": 0
    }
    
    for topic, qos in topic_qos_map.items():
        reliability_counter[qos['reliability']] += 1
        durability_counter[qos['durability']] += 1
        
        freq = qos['frequency_hz']
        if freq > 100:
            frequency_ranges["매우 빠름 (>100Hz)"] += 1
        elif freq > 10:
            frequency_ranges["빠름 (10-100Hz)"] += 1
        elif freq > 1:
            frequency_ranges["보통 (1-10Hz)"] += 1
        elif freq > 0:
            frequency_ranges["느림 (<1Hz)"] += 1
        else:
            frequency_ranges["정적 (0Hz)"] += 1
    
    # 결과 출력
    print("=" * 80)
    print("QoS 정책 통계")
    print("=" * 80)
    print()
    
    print(f"📊 분석된 토픽 수: {len(topic_qos_map)}개")
    print()
    
    print("1️⃣ Reliability (신뢰성):")
    for rel, count in reliability_counter.most_common():
        percentage = (count / len(topic_qos_map)) * 100
        print(f"   • {rel}: {count}개 ({percentage:.1f}%)")
    print()
    
    print("2️⃣ Durability (지속성):")
    for dur, count in durability_counter.most_common():
        percentage = (count / len(topic_qos_map)) * 100
        print(f"   • {dur}: {count}개 ({percentage:.1f}%)")
    print()
    
    print("3️⃣ Frequency (주파수 분포):")
    for range_name, count in frequency_ranges.items():
        if count > 0:
            percentage = (count / len(topic_qos_map)) * 100
            print(f"   • {range_name}: {count}개 ({percentage:.1f}%)")
    print()
    
    # 대표적인 예시
    print("=" * 80)
    print("대표적인 QoS 조합 예시")
    print("=" * 80)
    print()
    
    qos_combinations = Counter()
    for topic, qos in topic_qos_map.items():
        combo = f"{qos['reliability']} + {qos['durability']}"
        qos_combinations[combo] += 1
    
    print("QoS 조합별 토픽 수:")
    for combo, count in qos_combinations.most_common():
        percentage = (count / len(topic_qos_map)) * 100
        print(f"   • {combo}: {count}개 ({percentage:.1f}%)")
        
        # 예시 토픽 몇 개 보여주기
        examples = [t for t, q in topic_qos_map.items() 
                   if f"{q['reliability']} + {q['durability']}" == combo][:3]
        for ex in examples:
            print(f"     - {ex}")
    
    return topic_qos_map


def main():
    # 1. 모든 QoS 정책 설명
    print_all_qos_policies()
    
    input("\n계속하려면 Enter를 눌러주세요 (실제 데이터 분석)...\n")
    
    # 2. 실제 PCAP 데이터 분석
    pcap_file = "data/shm.pcapng"
    topic_qos_map = analyze_pcap_qos(pcap_file, max_packets=5000)
    
    # 3. 요약
    print("\n" + "=" * 80)
    print("요약")
    print("=" * 80)
    print()
    print(f"📚 DDS 표준 QoS 정책: 총 {len(QOS_POLICIES)}가지")
    print(f"📊 실제 추출된 토픽: {len(topic_qos_map)}개")
    print()
    print("주요 QoS 정책 (ROS2에서 자주 사용):")
    print("   1. Reliability (RELIABLE vs BEST_EFFORT)")
    print("   2. Durability (VOLATILE vs TRANSIENT_LOCAL)")
    print("   3. History & Depth (큐 관리)")
    print()
    print("기타 QoS 정책 (고급 사용):")
    print("   4. Deadline (주기 모니터링)")
    print("   5. Liveliness (생존 확인)")
    print("   6. Lifespan (메시지 만료)")
    print("   7. Ownership (다중 Publisher 제어)")
    print("   ... 외 8가지")


if __name__ == "__main__":
    main()
