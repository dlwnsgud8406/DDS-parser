#!/usr/bin/env python3
"""
실제 PCAP에서 추출되는 PID 필드 이름 확인

어떤 PID 필드들이 실제로 존재하는지 확인합니다.
"""

import sys
from pathlib import Path
from collections import Counter

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor


def main():
    pcap_file = "data/shm.pcapng"
    max_packets = 1000
    
    print("=" * 80)
    print("실제 PCAP에서 PID 필드 이름 확인")
    print("=" * 80)
    print(f"파일: {pcap_file}")
    print(f"최대 패킷: {max_packets:,}")
    print()
    
    # PCAP 파싱
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    
    print(f"✓ {len(submessages):,}개 submessage 파싱\n")
    
    # 모든 PID 필드 수집
    all_pid_keys = Counter()
    pid_value_samples = {}
    
    for msg in submessages:
        pids = msg.get('pids', {})
        for key, value in pids.items():
            all_pid_keys[key] += 1
            if key not in pid_value_samples and value is not None:
                pid_value_samples[key] = value
    
    # QoS 관련 PID만 필터링
    qos_keywords = [
        'RELIABILITY', 'DURABILITY', 'DEADLINE', 'LATENCY',
        'LIVELINESS', 'OWNERSHIP', 'PRESENTATION', 'PARTITION',
        'HISTORY', 'LIFESPAN', 'TIME_BASED', 'TRANSPORT'
    ]
    
    print("=" * 80)
    print("QoS 관련 PID 필드")
    print("=" * 80)
    print()
    
    qos_pids = {}
    for key in sorted(all_pid_keys.keys()):
        if any(keyword in key.upper() for keyword in qos_keywords):
            qos_pids[key] = all_pid_keys[key]
    
    if qos_pids:
        for key in sorted(qos_pids.keys()):
            count = qos_pids[key]
            sample = pid_value_samples.get(key, 'N/A')
            print(f"   {key:60s}: {count:5d}회 (예시: {sample})")
    else:
        print("   ❌ QoS 관련 PID 필드를 찾을 수 없습니다!")
    
    print()
    print("=" * 80)
    print("전체 PID 필드 목록 (참고)")
    print("=" * 80)
    print()
    
    for key in sorted(all_pid_keys.keys()):
        count = all_pid_keys[key]
        sample = pid_value_samples.get(key, 'N/A')
        # 샘플 값이 너무 길면 잘라냄
        if isinstance(sample, str) and len(sample) > 50:
            sample = sample[:47] + "..."
        print(f"   {key:60s}: {count:5d}회 (예시: {sample})")


if __name__ == "__main__":
    main()
