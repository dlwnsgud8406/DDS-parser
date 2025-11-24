#!/usr/bin/env python3
"""
ROS2 노드 이름 탐색 스크립트

PCAP 파일에서 ROS2 노드 이름이 어디에 저장되어 있는지 탐색
"""

import sys
from pathlib import Path
from collections import defaultdict

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor


def main():
    pcap_file = "data/shm.pcapng"
    max_packets = 1000
    
    print("=" * 80)
    print("ROS2 노드 이름 탐색")
    print("=" * 80)
    print(f"파일: {pcap_file}")
    print(f"최대 패킷: {max_packets}")
    print()
    
    # 1. PCAP 파싱
    print("[1/2] PCAP 파일 파싱 중...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage 파싱 완료\n")
    
    # 2. PID_ENTITY_NAME 탐색
    print("[2/2] PID_ENTITY_NAME 필드 탐색...")
    print()
    
    entity_names = defaultdict(list)
    participant_info = defaultdict(dict)
    
    for i, msg in enumerate(submessages):
        pids = msg.get('pids', {})
        
        # PID_ENTITY_NAME 확인
        entity_name = pids.get('PID_ENTITY_NAME_entity_name')
        
        if entity_name:
            submsg_name = msg.get('submsg_name', 'UNKNOWN')
            guid = msg.get('guid', {})
            
            # Participant GUID
            participant_guid = (
                guid.get('hostId'),
                guid.get('appId'),
                guid.get('instanceId')
            )
            
            entity_names[entity_name].append({
                'index': i,
                'submsg': submsg_name,
                'participant': participant_guid,
                'topic': pids.get('PID_TOPIC_NAME_topic'),
                'type': pids.get('PID_TYPE_NAME_typename'),
                'all_pids': list(pids.keys())
            })
            
            # Participant 정보 수집
            if participant_guid not in participant_info:
                participant_info[participant_guid] = {
                    'entity_names': set(),
                    'topics': set()
                }
            
            participant_info[participant_guid]['entity_names'].add(entity_name)
            topic = pids.get('PID_TOPIC_NAME_topic')
            if topic:
                participant_info[participant_guid]['topics'].add(topic)
    
    # 결과 출력
    if entity_names:
        print(f"✅ {len(entity_names)}개의 고유한 ENTITY_NAME 발견!\n")
        print("=" * 80)
        print("발견된 ENTITY_NAME 목록:")
        print("=" * 80)
        
        for entity_name, occurrences in sorted(entity_names.items()):
            print(f"\n📌 {entity_name}")
            print(f"   출현 횟수: {len(occurrences)}번")
            
            # 첫 번째 출현 정보
            first = occurrences[0]
            print(f"   첫 출현: #{first['index']} - {first['submsg']}")
            print(f"   Participant: {first['participant']}")
            if first['topic']:
                print(f"   Topic: {first['topic']}")
            if first['type']:
                print(f"   Type: {first['type']}")
            
            # 관련 PID 목록 (처음 것만)
            print(f"   PIDs: {', '.join(sorted(set(first['all_pids'])))}")
    else:
        print("❌ PID_ENTITY_NAME을 찾을 수 없습니다.")
        print()
        print("대안 탐색: Participant 관련 필드들...")
        
        # 다른 필드 탐색
        all_pid_fields = set()
        participant_guids = set()
        
        for msg in submessages[:100]:  # 처음 100개만
            pids = msg.get('pids', {})
            all_pid_fields.update(pids.keys())
            
            guid = msg.get('guid', {})
            if guid.get('hostId') and guid.get('appId'):
                participant_guids.add((
                    guid.get('hostId'),
                    guid.get('appId'),
                    guid.get('instanceId')
                ))
        
        print(f"\n발견된 Participant GUID: {len(participant_guids)}개")
        for i, pguid in enumerate(sorted(participant_guids), 1):
            print(f"  {i}. {pguid}")
        
        print(f"\n발견된 PID 필드 (처음 100개 메시지 기준):")
        participant_related = [f for f in sorted(all_pid_fields) if 'PARTICIPANT' in f or 'ENTITY' in f or 'NAME' in f]
        if participant_related:
            for field in participant_related:
                print(f"  • {field}")
        else:
            print("  (Participant/Entity 관련 필드 없음)")
    
    # Participant별 요약
    if participant_info:
        print("\n" + "=" * 80)
        print("Participant별 요약:")
        print("=" * 80)
        
        for i, (pguid, info) in enumerate(sorted(participant_info.items()), 1):
            print(f"\n{i}. Participant {pguid}")
            print(f"   Entity Names: {', '.join(sorted(info['entity_names']))}")
            print(f"   Topics ({len(info['topics'])}개):")
            for topic in sorted(info['topics'])[:5]:  # 최대 5개만
                print(f"     • {topic}")
            if len(info['topics']) > 5:
                print(f"     ... 외 {len(info['topics']) - 5}개")


if __name__ == "__main__":
    main()
