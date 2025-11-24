#!/usr/bin/env python3
"""
노드별 토픽 분석 스크립트

각 노드가 몇 개의 토픽을 사용하는지 분석
"""

import sys
from pathlib import Path
from collections import defaultdict

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor
from src.transformer import EndpointMapper, NodeGrouper


def main():
    pcap_file = "data/shm.pcapng"
    max_packets = 5000
    
    print("=" * 80)
    print("노드별 토픽 분석")
    print("=" * 80)
    print(f"파일: {pcap_file}")
    print(f"최대 패킷: {max_packets:,}")
    print()
    
    # 1. PCAP 파싱
    print("[1/4] PCAP 파일 파싱...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage 파싱 완료\n")
    
    # 2. SEDP 매핑
    print("[2/4] SEDP 매핑 테이블 생성...")
    endpoint_mapper = EndpointMapper()
    endpoint_mapper.build_mapping(submessages)
    print(f"  ✓ {endpoint_mapper.get_statistics()['total_endpoints']}개 endpoint 발견\n")
    
    # 3. Topic 매핑
    print("[3/4] Topic 매핑...")
    enriched_submessages = endpoint_mapper.enrich_submessages(submessages)
    mapped_count = sum(1 for msg in enriched_submessages if 'topic' in msg)
    print(f"  ✓ {mapped_count:,}개 submessage에 topic 매핑됨\n")
    
    # 4. 노드별 그룹화 및 토픽 수집
    print("[4/4] 노드별 토픽 수집...")
    node_grouper = NodeGrouper()
    grouped_by_node = node_grouper.group_by_node(enriched_submessages)
    
    # 노드별 토픽 수집
    node_topics = defaultdict(set)
    node_message_count = defaultdict(int)
    
    for node_name, messages in grouped_by_node.items():
        node_message_count[node_name] = len(messages)
        for msg in messages:
            topic = msg.get('topic', 'unknown')
            if topic != 'unknown':
                node_topics[node_name].add(topic)
    
    print(f"  ✓ {len(grouped_by_node)}개 노드 분석 완료\n")
    
    # 결과 출력
    print("=" * 80)
    print("노드별 토픽 상세 분석")
    print("=" * 80)
    print()
    
    # 토픽 개수로 정렬 (많은 순)
    sorted_nodes = sorted(node_topics.items(), key=lambda x: len(x[1]), reverse=True)
    
    for node, topics in sorted_nodes:
        msg_count = node_message_count[node]
        print(f"📦 {node}")
        print(f"   토픽 수: {len(topics)}개")
        print(f"   메시지 수: {msg_count:,}개")
        print(f"   토픽 목록:")
        for topic in sorted(topics):
            print(f"     • {topic}")
        print()
    
    # 요약
    print("=" * 80)
    print("요약")
    print("=" * 80)
    
    multi_topic_nodes = [(n, len(t)) for n, t in node_topics.items() if len(t) > 1]
    single_topic_nodes = [(n, len(t)) for n, t in node_topics.items() if len(t) == 1]
    
    print(f"총 노드 수: {len(node_topics)}개\n")
    
    if multi_topic_nodes:
        print(f"✅ 여러 토픽을 가진 노드: {len(multi_topic_nodes)}개")
        for node, count in sorted(multi_topic_nodes, key=lambda x: x[1], reverse=True):
            print(f"   • {node}: {count}개 토픽")
    else:
        print("❌ 여러 토픽을 가진 노드 없음")
    
    print()
    
    if single_topic_nodes:
        print(f"단일 토픽 노드: {len(single_topic_nodes)}개")
        for node, count in single_topic_nodes:
            print(f"   • {node}: {count}개 토픽")


if __name__ == "__main__":
    main()
