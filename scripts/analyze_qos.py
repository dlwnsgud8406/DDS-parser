#!/usr/bin/env python3
"""
QoS 정책 분석 스크립트

Excel 파일에서 QoS 정책 분포를 분석합니다
"""

import sys
from pathlib import Path
import pandas as pd
from collections import Counter

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor


# QoS 값 해석
RELIABILITY_MAP = {
    0x00000001: "RELIABLE",
    0x00000002: "BEST_EFFORT"
}

DURABILITY_MAP = {
    0x00000000: "VOLATILE",
    0x00000001: "TRANSIENT_LOCAL",
    0x00000002: "TRANSIENT",
    0x00000003: "PERSISTENT"
}

HISTORY_MAP = {
    0x00000000: "KEEP_LAST",
    0x00000001: "KEEP_ALL"
}


def analyze_qos(pcap_file, max_packets=None):
    print("=" * 80)
    print("QoS 정책 분석")
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
    
    # QoS 분석
    print("[2/2] QoS 정책 분석 중...")
    print()
    
    # 토픽별 QoS 수집
    topic_qos = {}
    
    for _, row in df.iterrows():
        pids = row.get('pids', {})
        if not isinstance(pids, dict):
            continue
        
        topic = pids.get('PID_TOPIC_NAME_topic')
        if not topic:
            continue
        
        if topic not in topic_qos:
            topic_qos[topic] = {
                'reliability': None,
                'durability': None,
                'history': None,
                'deadline_sec': None,
                'count': 0
            }
        
        # QoS 값 추출 (정수형만 저장)
        rel = pids.get('PID_RELIABILITY_kind')
        if rel is not None and isinstance(rel, int):
            topic_qos[topic]['reliability'] = rel
        
        dur = pids.get('PID_DURABILITY_kind')
        if dur is not None and isinstance(dur, int):
            topic_qos[topic]['durability'] = dur
        
        hist = pids.get('PID_HISTORY_kind')
        if hist is not None and isinstance(hist, int):
            topic_qos[topic]['history'] = hist
        
        deadline = pids.get('PID_DEADLINE_period_sec')
        if deadline is not None and isinstance(deadline, (int, float)):
            topic_qos[topic]['deadline_sec'] = deadline
        
        topic_qos[topic]['count'] += 1
    
    # 결과 출력
    print("=" * 80)
    print("📊 토픽별 QoS 정책")
    print("=" * 80)
    print()
    
    if not topic_qos:
        print("❌ QoS 정보가 있는 토픽을 찾을 수 없습니다.")
        print("   (SEDP 데이터가 없을 수 있습니다)")
        return
    
    # 토픽 정렬 (출현 횟수순)
    sorted_topics = sorted(topic_qos.items(), key=lambda x: x[1]['count'], reverse=True)
    
    for topic, qos in sorted_topics:
        print(f"📌 {topic}")
        print(f"   출현: {qos['count']:,}회")
        
        if qos['reliability'] is not None:
            rel_name = RELIABILITY_MAP.get(qos['reliability'], f"0x{qos['reliability']:08x}")
            print(f"   ├─ Reliability: {rel_name}")
        
        if qos['durability'] is not None:
            dur_name = DURABILITY_MAP.get(qos['durability'], f"0x{qos['durability']:08x}")
            print(f"   ├─ Durability: {dur_name}")
        
        if qos['history'] is not None:
            hist_name = HISTORY_MAP.get(qos['history'], f"0x{qos['history']:08x}")
            print(f"   ├─ History: {hist_name}")
        
        if qos['deadline_sec'] is not None:
            if qos['deadline_sec'] == 2147483647:
                print(f"   └─ Deadline: 무한대 (주기 없음)")
            else:
                hz = 1.0 / qos['deadline_sec'] if qos['deadline_sec'] > 0 else 0
                print(f"   └─ Deadline: {qos['deadline_sec']}초 (~{hz:.1f}Hz)")
        
        print()
    
    # 통계 요약
    print("=" * 80)
    print("📈 QoS 정책 통계")
    print("=" * 80)
    print()
    
    # Reliability 통계
    rel_counter = Counter()
    for qos in topic_qos.values():
        if qos['reliability'] is not None:
            rel_counter[qos['reliability']] += 1
    
    if rel_counter:
        print("▶ Reliability 분포:")
        for rel_val, count in rel_counter.most_common():
            rel_name = RELIABILITY_MAP.get(rel_val, f"0x{rel_val:08x}")
            print(f"  • {rel_name}: {count}개 토픽")
        print()
    
    # Durability 통계
    dur_counter = Counter()
    for qos in topic_qos.values():
        if qos['durability'] is not None:
            dur_counter[qos['durability']] += 1
    
    if dur_counter:
        print("▶ Durability 분포:")
        for dur_val, count in dur_counter.most_common():
            dur_name = DURABILITY_MAP.get(dur_val, f"0x{dur_val:08x}")
            print(f"  • {dur_name}: {count}개 토픽")
        print()
    
    # Deadline 분포
    deadline_values = [qos['deadline_sec'] for qos in topic_qos.values() 
                      if qos['deadline_sec'] is not None and qos['deadline_sec'] != 2147483647]
    
    if deadline_values:
        print("▶ Deadline 분포 (유한값만):")
        deadline_counter = Counter(deadline_values)
        for deadline, count in sorted(deadline_counter.items()):
            hz = 1.0 / deadline if deadline > 0 else 0
            print(f"  • {deadline}초 (~{hz:.1f}Hz): {count}개 토픽")
        print()
    
    print("=" * 80)
    print("✅ 분석 완료!")
    print("=" * 80)


def main():
    if len(sys.argv) < 2:
        print("사용법: python scripts/analyze_qos.py <pcap_file> [max_packets]")
        print()
        print("예시:")
        print("  python scripts/analyze_qos.py data/shm.pcapng")
        print("  python scripts/analyze_qos.py data/shm.pcapng 1000")
        sys.exit(1)
    
    pcap_file = sys.argv[1]
    max_packets = int(sys.argv[2]) if len(sys.argv) > 2 else None
    
    analyze_qos(pcap_file, max_packets)


if __name__ == "__main__":
    main()
