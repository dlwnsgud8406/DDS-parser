#!/usr/bin/env python3
"""
QoS 정책 간접 추론 스크립트

명시적 QoS 정보가 없어도 메시지 패턴으로 QoS를 추론합니다.
"""

import sys
from pathlib import Path
from collections import defaultdict, Counter
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor


def analyze_message_patterns(df):
    """메시지 패턴으로 QoS 추론"""
    
    print("=" * 80)
    print("QoS 정책 간접 추론 (메시지 패턴 분석)")
    print("=" * 80)
    print()
    
    # 토픽별 통계
    topic_stats = defaultdict(lambda: {
        'total_count': 0,
        'data_messages': 0,
        'heartbeat_count': 0,
        'acknack_count': 0,
        'gap_count': 0,
        'timestamps': [],
        'submsg_types': Counter()
    })
    
    for _, row in df.iterrows():
        pids = row.get('pids', {})
        if not isinstance(pids, dict):
            continue
        
        topic = pids.get('PID_TOPIC_NAME_topic')
        if not topic:
            continue
        
        submsg_type = row.get('submsg_name', '')
        timestamp = row.get('timestamp')
        
        topic_stats[topic]['total_count'] += 1
        topic_stats[topic]['submsg_types'][submsg_type] += 1
        
        if submsg_type and 'DATA' in submsg_type:
            topic_stats[topic]['data_messages'] += 1
        elif submsg_type == 'HEARTBEAT':
            topic_stats[topic]['heartbeat_count'] += 1
        elif submsg_type == 'ACKNACK':
            topic_stats[topic]['acknack_count'] += 1
        elif submsg_type == 'GAP':
            topic_stats[topic]['gap_count'] += 1
        
        if timestamp:
            topic_stats[topic]['timestamps'].append(timestamp)
    
    # 토픽별 QoS 추론
    print("=" * 80)
    print("📊 토픽별 QoS 추론 결과")
    print("=" * 80)
    print()
    
    qos_summary = {
        'reliable': [],
        'best_effort': [],
        'transient_local': [],
        'volatile': []
    }
    
    for topic, stats in sorted(topic_stats.items(), key=lambda x: x[1]['total_count'], reverse=True):
        print(f"📌 {topic}")
        print(f"   총 메시지: {stats['total_count']:,}개")
        
        # Reliability 추론
        reliability = infer_reliability(stats)
        print(f"   ├─ Reliability: {reliability['type']} {reliability['confidence']}")
        print(f"   │  이유: {reliability['reason']}")
        
        if reliability['type'] == 'RELIABLE':
            qos_summary['reliable'].append(topic)
        else:
            qos_summary['best_effort'].append(topic)
        
        # Durability 추론
        durability = infer_durability(stats)
        print(f"   ├─ Durability: {durability['type']} {durability['confidence']}")
        print(f"   │  이유: {durability['reason']}")
        
        if durability['type'] == 'TRANSIENT_LOCAL':
            qos_summary['transient_local'].append(topic)
        else:
            qos_summary['volatile'].append(topic)
        
        # 주기 분석
        frequency = analyze_frequency(stats)
        print(f"   └─ 발행 주기: {frequency['description']}")
        if frequency['hz']:
            print(f"      → 약 {frequency['hz']:.1f}Hz")
        
        print()
    
    # 요약
    print("=" * 80)
    print("📈 QoS 정책 요약")
    print("=" * 80)
    print()
    
    print(f"▶ Reliability 분포:")
    print(f"  • RELIABLE: {len(qos_summary['reliable'])}개 토픽")
    if qos_summary['reliable'][:3]:
        for t in qos_summary['reliable'][:3]:
            print(f"    - {t}")
        if len(qos_summary['reliable']) > 3:
            print(f"    ... 외 {len(qos_summary['reliable']) - 3}개")
    
    print(f"\n  • BEST_EFFORT: {len(qos_summary['best_effort'])}개 토픽")
    if qos_summary['best_effort'][:3]:
        for t in qos_summary['best_effort'][:3]:
            print(f"    - {t}")
        if len(qos_summary['best_effort']) > 3:
            print(f"    ... 외 {len(qos_summary['best_effort']) - 3}개")
    
    print(f"\n▶ Durability 분포:")
    print(f"  • TRANSIENT_LOCAL: {len(qos_summary['transient_local'])}개 토픽")
    if qos_summary['transient_local'][:3]:
        for t in qos_summary['transient_local'][:3]:
            print(f"    - {t}")
    
    print(f"\n  • VOLATILE: {len(qos_summary['volatile'])}개 토픽")
    if qos_summary['volatile'][:3]:
        for t in qos_summary['volatile'][:3]:
            print(f"    - {t}")
        if len(qos_summary['volatile']) > 3:
            print(f"    ... 외 {len(qos_summary['volatile']) - 3}개")
    
    print()
    print("=" * 80)
    print("✅ 추론 완료!")
    print("=" * 80)


def infer_reliability(stats):
    """Reliability 추론"""
    
    # ACKNACK/HEARTBEAT 비율로 판단
    total = stats['total_count']
    acknack = stats['acknack_count']
    heartbeat = stats['heartbeat_count']
    
    # ACKNACK이 있으면 RELIABLE (재전송 확인)
    if acknack > 0:
        confidence = "⭐⭐⭐ (높음)"
        reason = f"ACKNACK {acknack}개 발견 (재전송 확인)"
        return {'type': 'RELIABLE', 'confidence': confidence, 'reason': reason}
    
    # HEARTBEAT 많으면 RELIABLE (상태 동기화)
    if heartbeat > total * 0.3:
        confidence = "⭐⭐ (중간)"
        reason = f"HEARTBEAT {heartbeat}개 ({heartbeat/total*100:.0f}%)"
        return {'type': 'RELIABLE', 'confidence': confidence, 'reason': reason}
    
    # 서비스 (rq/rr)는 기본 RELIABLE
    # 판단 불가시 BEST_EFFORT로 추론
    confidence = "⭐ (낮음)"
    reason = "재전송 메커니즘 미발견"
    return {'type': 'BEST_EFFORT', 'confidence': confidence, 'reason': reason}


def infer_durability(stats):
    """Durability 추론"""
    
    data_msgs = stats['data_messages']
    total = stats['total_count']
    
    # DATA 메시지 비율이 낮으면 TRANSIENT_LOCAL (재전송 가능)
    if data_msgs > 0 and data_msgs < 5:
        confidence = "⭐⭐ (중간)"
        reason = f"DATA 메시지 {data_msgs}개 (초기 전송 후 보존)"
        return {'type': 'TRANSIENT_LOCAL', 'confidence': confidence, 'reason': reason}
    
    # 대부분은 VOLATILE
    confidence = "⭐⭐ (중간)"
    reason = "실시간 스트리밍 패턴"
    return {'type': 'VOLATILE', 'confidence': confidence, 'reason': reason}


def analyze_frequency(stats):
    """발행 주기 분석"""
    
    timestamps = sorted(stats['timestamps'])
    if len(timestamps) < 2:
        return {'description': '주기 분석 불가', 'hz': None}
    
    # 시간 간격 계산
    intervals = []
    for i in range(1, min(100, len(timestamps))):  # 처음 100개만
        interval = timestamps[i] - timestamps[i-1]
        if interval > 0:
            intervals.append(interval)
    
    if not intervals:
        return {'description': '주기 분석 불가', 'hz': None}
    
    # 평균 간격
    avg_interval = sum(intervals) / len(intervals)
    hz = 1.0 / avg_interval if avg_interval > 0 else 0
    
    # 분류
    if hz > 100:
        desc = "매우 높은 주기 (>100Hz)"
    elif hz > 30:
        desc = "높은 주기 (30-100Hz)"
    elif hz > 10:
        desc = "중간 주기 (10-30Hz)"
    elif hz > 1:
        desc = "낮은 주기 (1-10Hz)"
    else:
        desc = "드물게 발행 (<1Hz)"
    
    return {'description': desc, 'hz': hz}


def main():
    if len(sys.argv) < 2:
        print("사용법: python scripts/infer_qos_from_behavior.py <pcap_file> [max_packets]")
        print()
        print("예시:")
        print("  python scripts/infer_qos_from_behavior.py data/shm.pcapng")
        print("  python scripts/infer_qos_from_behavior.py data/shm.pcapng 5000")
        sys.exit(1)
    
    pcap_file = sys.argv[1]
    max_packets = int(sys.argv[2]) if len(sys.argv) > 2 else None
    
    print("=" * 80)
    print("QoS 간접 추론 분석")
    print("=" * 80)
    print(f"파일: {pcap_file}")
    if max_packets:
        print(f"최대 패킷: {max_packets:,}")
    print()
    
    # 파싱
    print("[1/2] PCAP 파일 파싱 중...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=999999.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    print(f"  ✓ {len(df):,}개 submessage 파싱 완료\n")
    
    # 분석
    print("[2/2] 메시지 패턴 분석 중...")
    print()
    
    analyze_message_patterns(df)


if __name__ == "__main__":
    main()
