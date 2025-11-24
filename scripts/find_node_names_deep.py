#!/usr/bin/env python3
"""
ROS2 노드 이름 심층 탐색

PID_USER_DATA, PID_PROPERTY_LIST 등에서 노드 이름 찾기
"""

import sys
from pathlib import Path
from collections import defaultdict
import re

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor


def extract_node_names_from_string(text):
    """문자열에서 ROS2 노드 이름 패턴 추출"""
    if not text:
        return []
    
    # ROS2 노드 이름 패턴 (/, 알파벳, 숫자, _)
    # 예: /joint_state_publisher, /stella_md_node, /launch_ros_2385
    patterns = [
        r'/[a-zA-Z0-9_]+',  # /로 시작하는 노드 이름
        r'[a-zA-Z_][a-zA-Z0-9_]*_node',  # _node로 끝나는 이름
    ]
    
    found = []
    for pattern in patterns:
        matches = re.findall(pattern, str(text))
        found.extend(matches)
    
    return found


def main():
    pcap_file = "data/shm.pcapng"
    max_packets = 2000  # 더 많이 보기
    
    print("=" * 80)
    print("ROS2 노드 이름 심층 탐색")
    print("=" * 80)
    print(f"파일: {pcap_file}")
    print(f"최대 패킷: {max_packets}")
    print()
    
    # 예상되는 노드 이름들
    expected_nodes = [
        '/joint_state_publisher',
        '/launch_ros_2385',
        '/robot_state_publisher',
        '/static_tf_pub_laser',
        '/stella_ahrs_node',
        '/stella_md_node',
        '/ydlidar_ros2_driver_node',
    ]
    
    print("🎯 찾아야 할 노드 이름:")
    for node in expected_nodes:
        print(f"   • {node}")
    print()
    
    # 1. PCAP 파싱
    print("[1/3] PCAP 파일 파싱 중...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage 파싱 완료\n")
    
    # 2. Participant Discovery (DATA(p)) 패킷 찾기
    print("[2/3] Participant Discovery 패킷 탐색...")
    print("  (DATA(p) 타입의 submessage 중 노드 정보 포함)")
    print()
    
    participant_packets = []
    found_nodes = defaultdict(list)
    
    for i, msg in enumerate(submessages):
        submsg_name = msg.get('submsg_name', '')
        
        # DATA(p) = Participant Discovery
        if 'DATA(p)' not in submsg_name:
            continue
        
        pids = msg.get('pids', {})
        guid = msg.get('guid', {})
        
        # Participant GUID
        participant_guid = (
            guid.get('hostId'),
            guid.get('appId'),
            guid.get('instanceId')
        )
        
        # USER_DATA 확인
        user_data = pids.get('PID_USER_DATA_data')
        
        packet_info = {
            'index': i,
            'submsg': submsg_name,
            'participant_guid': participant_guid,
            'user_data': user_data,
            'all_pids': list(pids.keys())
        }
        
        # USER_DATA에서 노드 이름 찾기
        if user_data:
            # 예상 노드 이름 매칭
            for expected_node in expected_nodes:
                if expected_node in str(user_data):
                    found_nodes[expected_node].append({
                        'index': i,
                        'participant': participant_guid,
                        'user_data': user_data,
                        'submsg': submsg_name
                    })
            
            # 패턴 기반 추출
            extracted = extract_node_names_from_string(user_data)
            for name in extracted:
                if name not in found_nodes:
                    found_nodes[name].append({
                        'index': i,
                        'participant': participant_guid,
                        'user_data': user_data,
                        'submsg': submsg_name
                    })
        
        participant_packets.append(packet_info)
    
    print(f"  ✓ {len(participant_packets)}개의 DATA(p) 패킷 발견\n")
    
    # 3. 결과 출력
    print("[3/3] 노드 이름 추출 결과")
    print("=" * 80)
    
    if found_nodes:
        print(f"✅ {len(found_nodes)}개의 노드 이름 발견!\n")
        
        for node_name, occurrences in sorted(found_nodes.items()):
            print(f"📌 {node_name}")
            print(f"   출현 횟수: {len(occurrences)}번")
            
            first = occurrences[0]
            print(f"   첫 출현: #{first['index']}")
            print(f"   Participant GUID: {first['participant']}")
            print(f"   USER_DATA 샘플: {str(first['user_data'])[:100]}...")
            
            # 예상 노드인지 확인
            if node_name in expected_nodes:
                print(f"   ✅ 예상된 노드!")
            print()
    else:
        print("❌ 노드 이름을 찾을 수 없습니다.\n")
        print("DATA(p) 패킷 샘플 분석:")
        
        for i, packet in enumerate(participant_packets[:5], 1):  # 처음 5개만
            print(f"\n샘플 #{i} (패킷 인덱스: {packet['index']})")
            print(f"  Participant: {packet['participant_guid']}")
            print(f"  Submsg: {packet['submsg']}")
            print(f"  USER_DATA: {packet['user_data']}")
            print(f"  Available PIDs: {', '.join(packet['all_pids'][:10])}...")
    
    # Participant GUID별 그룹화
    print("\n" + "=" * 80)
    print("Participant GUID별 매핑:")
    print("=" * 80)
    
    guid_to_node = defaultdict(set)
    for node_name, occurrences in found_nodes.items():
        for occ in occurrences:
            guid_to_node[occ['participant']].add(node_name)
    
    if guid_to_node:
        for i, (pguid, nodes) in enumerate(sorted(guid_to_node.items()), 1):
            print(f"\n{i}. Participant {pguid}")
            for node in sorted(nodes):
                print(f"   → {node}")
    else:
        print("  (매핑 정보 없음)")


if __name__ == "__main__":
    main()
