#!/usr/bin/env python3
"""
ROS2 노드 이름을 토픽에서 역추적

ROS2에서는 노드가 다음과 같은 토픽을 자동 생성합니다:
- /rosout
- /parameter_events
- rq/<node_name>/...   (request)
- rr/<node_name>/...   (response)  
- rt/<node_name>/...   (regular topic)
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
from src.transformer import EndpointMapper
from src.transformer.node_extractor import NodeNameExtractor


def main():
    pcap_file = "data/shm.pcapng"
    max_packets = 5000
    
    print("=" * 80)
    print("ROS2 노드 이름 역추적 (토픽 기반)")
    print("=" * 80)
    print(f"파일: {pcap_file}")
    print(f"최대 패킷: {max_packets}")
    print()
    
    # 예상되는 노드 이름들 (정규화된 버전)
    expected_nodes = {
        'joint_state_publisher',
        'launch_ros_*',  # launch_ros_2385 정규화
        'robot_state_publisher',
        'static_tf_pub_laser',
        'stella_ahrs_node',
        'stella_md_node',
        'ydlidar_ros2_driver_node',
    }
    
    print("🎯 찾아야 할 노드 이름:")
    for node in sorted(expected_nodes):
        print(f"   • /{node}")
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
    
    # 2. Endpoint → Topic 매핑
    print("[2/3] SEDP 매핑 테이블 생성...")
    endpoint_mapper = EndpointMapper()
    endpoint_mapper.build_mapping(submessages)
    
    stats = endpoint_mapper.get_statistics()
    print(f"  ✓ {stats['total_endpoints']}개 endpoint 발견")
    print(f"  ✓ {stats['topics_count']}개 topic 발견\n")
    
    # 3. 토픽에서 노드 이름 추출
    print("[3/3] 토픽 분석 및 노드 추출")
    print("=" * 80)
    
    all_topics = sorted(stats['topics'])
    
    # 노드 추출기 생성
    extractor = NodeNameExtractor()
    
    # 노드별 토픽 그룹화
    node_topics = defaultdict(set)
    unknown_topics = []
    
    for topic in all_topics:
        node_name = extractor.extract_node_from_topic(topic)
        if node_name:
            node_topics[node_name].add(topic)
        else:
            unknown_topics.append(topic)
    
    # 발견된 노드 출력
    if node_topics:
        print(f"\n✅ {len(node_topics)}개의 노드 발견!\n")
        
        for i, (node_name, topics) in enumerate(sorted(node_topics.items()), 1):
            # 예상 노드인지 확인
            is_expected = node_name in expected_nodes
            marker = "✅" if is_expected else "🔍"
            
            print(f"{marker} {i}. /{node_name}")
            print(f"   토픽 수: {len(topics)}개")
            
            # 토픽 샘플 (최대 5개)
            for topic in sorted(topics)[:5]:
                print(f"     • {topic}")
            if len(topics) > 5:
                print(f"     ... 외 {len(topics) - 5}개")
            print()
        
        # 예상 노드와 비교
        found_nodes = set(node_topics.keys())
        missing = expected_nodes - found_nodes
        extra = found_nodes - expected_nodes
        
        print("=" * 80)
        print("노드 매칭 결과:")
        print("=" * 80)
        print(f"예상 노드: {len(expected_nodes)}개")
        print(f"발견 노드: {len(found_nodes)}개")
        print(f"일치: {len(found_nodes & expected_nodes)}개")
        
        if missing:
            print(f"\n❌ 찾지 못한 노드 ({len(missing)}개):")
            for node in sorted(missing):
                print(f"   • /{node}")
        
        if extra:
            print(f"\n🔍 예상 외 노드 ({len(extra)}개):")
            for node in sorted(extra):
                print(f"   • /{node}")
    else:
        print("❌ 노드를 찾을 수 없습니다.\n")
    
    # 패턴 없는 토픽들
    if unknown_topics:
        print(f"\n기타 토픽 ({len(unknown_topics)}개):")
        for topic in unknown_topics[:10]:
            print(f"   • {topic}")
        if len(unknown_topics) > 10:
            print(f"   ... 외 {len(unknown_topics) - 10}개")
    
    # Participant GUID → 노드 매핑 추출
    print("\n" + "=" * 80)
    print("Participant GUID → 노드 매핑:")
    print("=" * 80)
    
    # endpoint_map에서 역추적
    guid_to_nodes = defaultdict(set)
    
    for endpoint_guid, info in endpoint_mapper.endpoint_map.items():
        topic = info.get('topic', '')
        node_name = extractor.extract_node_from_topic(topic)
        
        if node_name:
            # Participant GUID = (hostId, appId, instanceId)
            participant_guid = endpoint_guid[:3]  # entityId 제외
            guid_to_nodes[participant_guid].add(node_name)
    
    if guid_to_nodes:
        print()
        for i, (pguid, nodes) in enumerate(sorted(guid_to_nodes.items()), 1):
            hostId, appId, instanceId = pguid
            print(f"{i}. Participant ({hostId:08x}, {appId:08x}, {instanceId:08x})")
            for node in sorted(nodes):
                print(f"   → /{node}")
            print()
    else:
        print("  (매핑 정보 없음)")


if __name__ == "__main__":
    main()
