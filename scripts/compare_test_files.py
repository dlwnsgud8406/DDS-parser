#!/usr/bin/env python3
"""
test1.pcapng vs test2.pcapng 상세 비교 분석

두 파일의 모든 셀(필드)을 하나하나 비교하여 차이점을 찾습니다.
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


def parse_file(pcap_file):
    """PCAP 파일 파싱"""
    print(f"\n파싱 중: {pcap_file}")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=None)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage")
    
    return submessages


def get_all_field_paths(msg, prefix=''):
    """
    중첩 딕셔너리의 모든 필드 경로 추출
    
    예: {'guid': {'hostId': 123}} → 'guid.hostId'
    """
    paths = {}
    
    for key, value in msg.items():
        current_path = f"{prefix}.{key}" if prefix else key
        
        if isinstance(value, dict):
            # 중첩 딕셔너리는 재귀적으로 탐색
            paths.update(get_all_field_paths(value, current_path))
        else:
            # 값 저장
            paths[current_path] = value
    
    return paths


def compare_messages(test1_msgs, test2_msgs):
    """메시지별 상세 비교"""
    
    print("\n" + "=" * 80)
    print("📊 기본 통계")
    print("=" * 80)
    
    print(f"test1: {len(test1_msgs)} submessages")
    print(f"test2: {len(test2_msgs)} submessages")
    print(f"차이: {abs(len(test1_msgs) - len(test2_msgs))} submessages")
    
    # 모든 필드 경로 수집
    print("\n" + "=" * 80)
    print("🔍 필드 구조 분석")
    print("=" * 80)
    
    test1_fields = set()
    test2_fields = set()
    
    for msg in test1_msgs:
        paths = get_all_field_paths(msg)
        test1_fields.update(paths.keys())
    
    for msg in test2_msgs:
        paths = get_all_field_paths(msg)
        test2_fields.update(paths.keys())
    
    print(f"\ntest1 고유 필드: {len(test1_fields)}개")
    print(f"test2 고유 필드: {len(test2_fields)}개")
    
    common_fields = test1_fields & test2_fields
    only_test1 = test1_fields - test2_fields
    only_test2 = test2_fields - test1_fields
    
    print(f"\n공통 필드: {len(common_fields)}개")
    print(f"test1만 있는 필드: {len(only_test1)}개")
    print(f"test2만 있는 필드: {len(only_test2)}개")
    
    if only_test1:
        print("\n▶ test1에만 있는 필드:")
        for field in sorted(only_test1)[:20]:
            print(f"  • {field}")
        if len(only_test1) > 20:
            print(f"  ... 외 {len(only_test1) - 20}개")
    
    if only_test2:
        print("\n▶ test2에만 있는 필드:")
        for field in sorted(only_test2)[:20]:
            print(f"  • {field}")
        if len(only_test2) > 20:
            print(f"  ... 외 {len(only_test2) - 20}개")
    
    # Submessage 타입 분석
    print("\n" + "=" * 80)
    print("📋 Submessage 타입 분포")
    print("=" * 80)
    
    test1_types = Counter(msg.get('submsg_name', 'UNKNOWN') for msg in test1_msgs)
    test2_types = Counter(msg.get('submsg_name', 'UNKNOWN') for msg in test2_msgs)
    
    all_types = set(test1_types.keys()) | set(test2_types.keys())
    
    print(f"\n{'타입':<30} {'test1':>10} {'test2':>10} {'차이':>10}")
    print("-" * 65)
    for submsg_type in sorted(all_types):
        t1_count = test1_types.get(submsg_type, 0)
        t2_count = test2_types.get(submsg_type, 0)
        diff = t2_count - t1_count
        diff_str = f"{diff:+d}" if diff != 0 else "0"
        print(f"{submsg_type:<30} {t1_count:>10} {t2_count:>10} {diff_str:>10}")
    
    # GUID 분석
    print("\n" + "=" * 80)
    print("🔐 GUID (Participant/Entity) 분석")
    print("=" * 80)
    
    test1_guids = set()
    test2_guids = set()
    
    for msg in test1_msgs:
        guid = msg.get('guid', {})
        if guid.get('hostId'):
            test1_guids.add((
                guid.get('hostId'),
                guid.get('appId'),
                guid.get('instanceId'),
                guid.get('entityId')
            ))
    
    for msg in test2_msgs:
        guid = msg.get('guid', {})
        if guid.get('hostId'):
            test2_guids.add((
                guid.get('hostId'),
                guid.get('appId'),
                guid.get('instanceId'),
                guid.get('entityId')
            ))
    
    print(f"\ntest1 고유 GUID: {len(test1_guids)}개")
    print(f"test2 고유 GUID: {len(test2_guids)}개")
    
    common_guids = test1_guids & test2_guids
    only_test1_guids = test1_guids - test2_guids
    only_test2_guids = test2_guids - test1_guids
    
    print(f"공통 GUID: {len(common_guids)}개")
    print(f"test1만: {len(only_test1_guids)}개")
    print(f"test2만: {len(only_test2_guids)}개")
    
    if only_test1_guids:
        print("\n▶ test1에만 있는 GUID:")
        for guid in sorted(only_test1_guids)[:10]:
            print(f"  • {guid}")
        if len(only_test1_guids) > 10:
            print(f"  ... 외 {len(only_test1_guids) - 10}개")
    
    if only_test2_guids:
        print("\n▶ test2에만 있는 GUID:")
        for guid in sorted(only_test2_guids)[:10]:
            print(f"  • {guid}")
        if len(only_test2_guids) > 10:
            print(f"  ... 외 {len(only_test2_guids) - 10}개")
    
    # PID 필드 상세 분석
    print("\n" + "=" * 80)
    print("🏷️  PID 필드 상세 분석")
    print("=" * 80)
    
    test1_pids = defaultdict(set)
    test2_pids = defaultdict(set)
    
    for msg in test1_msgs:
        pids = msg.get('pids', {})
        for key, value in pids.items():
            if value is not None:
                test1_pids[key].add(str(value)[:100])  # 긴 값은 자르기
    
    for msg in test2_msgs:
        pids = msg.get('pids', {})
        for key, value in pids.items():
            if value is not None:
                test2_pids[key].add(str(value)[:100])
    
    all_pid_keys = set(test1_pids.keys()) | set(test2_pids.keys())
    
    print(f"\n발견된 PID 타입: {len(all_pid_keys)}개")
    
    if all_pid_keys:
        print(f"\n{'PID 타입':<50} {'test1':>10} {'test2':>10}")
        print("-" * 75)
        for pid_key in sorted(all_pid_keys):
            t1_values = len(test1_pids.get(pid_key, set()))
            t2_values = len(test2_pids.get(pid_key, set()))
            print(f"{pid_key:<50} {t1_values:>10} {t2_values:>10}")
    
    # 시간 정보 분석
    print("\n" + "=" * 80)
    print("⏱️  시간 정보 분석")
    print("=" * 80)
    
    test1_times = [msg.get('timestamp') for msg in test1_msgs if msg.get('timestamp')]
    test2_times = [msg.get('timestamp') for msg in test2_msgs if msg.get('timestamp')]
    
    if test1_times:
        print(f"\ntest1 시간 범위:")
        print(f"  최소: {min(test1_times)}")
        print(f"  최대: {max(test1_times)}")
        print(f"  지속시간: {max(test1_times) - min(test1_times):.3f}초")
    
    if test2_times:
        print(f"\ntest2 시간 범위:")
        print(f"  최소: {min(test2_times)}")
        print(f"  최대: {max(test2_times)}")
        print(f"  지속시간: {max(test2_times) - min(test2_times):.3f}초")
    
    # 셀별 상세 비교 (처음 5개 메시지)
    print("\n" + "=" * 80)
    print("🔬 셀별 상세 비교 (처음 5개 메시지)")
    print("=" * 80)
    
    compare_count = min(5, len(test1_msgs), len(test2_msgs))
    
    for i in range(compare_count):
        print(f"\n--- 메시지 #{i+1} ---")
        
        msg1_flat = get_all_field_paths(test1_msgs[i])
        msg2_flat = get_all_field_paths(test2_msgs[i])
        
        all_fields = set(msg1_flat.keys()) | set(msg2_flat.keys())
        
        differences = []
        for field in sorted(all_fields):
            val1 = msg1_flat.get(field, '<없음>')
            val2 = msg2_flat.get(field, '<없음>')
            
            if val1 != val2:
                differences.append((field, val1, val2))
        
        if differences:
            print(f"\n차이나는 필드: {len(differences)}개")
            print(f"\n{'필드':<40} {'test1':<30} {'test2':<30}")
            print("-" * 105)
            for field, val1, val2 in differences[:20]:
                v1_str = str(val1)[:28]
                v2_str = str(val2)[:28]
                print(f"{field:<40} {v1_str:<30} {v2_str:<30}")
            if len(differences) > 20:
                print(f"... 외 {len(differences) - 20}개 필드")
        else:
            print("✅ 동일함")
    
    # 값 분포 비교 (주요 필드)
    print("\n" + "=" * 80)
    print("📈 주요 필드 값 분포 비교")
    print("=" * 80)
    
    key_fields = ['submsg_name', 'guid.entityId', 'guid.hostId', 'packet_num']
    
    for field in key_fields:
        print(f"\n▶ {field}:")
        
        test1_values = []
        test2_values = []
        
        for msg in test1_msgs:
            flat = get_all_field_paths(msg)
            if field in flat:
                test1_values.append(flat[field])
        
        for msg in test2_msgs:
            flat = get_all_field_paths(msg)
            if field in flat:
                test2_values.append(flat[field])
        
        if test1_values or test2_values:
            test1_unique = set(test1_values)
            test2_unique = set(test2_values)
            
            print(f"  test1 고유값: {len(test1_unique)}개")
            print(f"  test2 고유값: {len(test2_unique)}개")
            
            common = test1_unique & test2_unique
            print(f"  공통값: {len(common)}개")
            
            only_t1 = test1_unique - test2_unique
            only_t2 = test2_unique - test1_unique
            
            if only_t1:
                print(f"  test1만: {sorted(only_t1)[:5]}")
            if only_t2:
                print(f"  test2만: {sorted(only_t2)[:5]}")


def main():
    print("=" * 80)
    print("test1.pcapng vs test2.pcapng 상세 비교")
    print("=" * 80)
    
    # 파일 파싱
    test1_msgs = parse_file("data/test1.pcapng")
    test2_msgs = parse_file("data/test2.pcapng")
    
    # 비교 분석
    compare_messages(test1_msgs, test2_msgs)
    
    print("\n" + "=" * 80)
    print("✅ 분석 완료!")
    print("=" * 80)


if __name__ == "__main__":
    main()
