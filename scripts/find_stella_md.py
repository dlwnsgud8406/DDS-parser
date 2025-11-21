#!/usr/bin/env python3
"""
stella_md_node가 왜 없는지 확인

stella_md_node 관련 토픽들을 찾아봅니다.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor
from src.transformer import EndpointMapper, NodeGrouper


def main():
    pcap_file = "data/shm.pcapng"
    max_packets = 2000
    
    print("=" * 80)
    print("stella_md_node 토픽 탐색")
    print("=" * 80)
    
    # 1. PCAP 파싱
    print("\n[1/3] PCAP 파일 파싱 중...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage 파싱 완료")
    
    # 2. Endpoint → Topic 매핑
    print("\n[2/3] SEDP 매핑 테이블 생성...")
    endpoint_mapper = EndpointMapper()
    endpoint_mapper.build_mapping(submessages)
    
    stats = endpoint_mapper.get_statistics()
    print(f"  ✓ {stats['total_endpoints']}개 endpoint 발견")
    print(f"  ✓ {stats['topics_count']}개 topic 발견")
    
    # 3. stella_md_node 관련 토픽 찾기
    print("\n[3/3] stella_md_node 관련 토픽 탐색")
    print("=" * 80)
    
    stella_topics = []
    for topic in sorted(stats['topics']):
        if 'stella_md' in topic.lower():
            stella_topics.append(topic)
    
    if stella_topics:
        print(f"\n✅ stella_md_node 관련 토픽 {len(stella_topics)}개 발견:")
        for topic in stella_topics:
            print(f"   • {topic}")
    else:
        print("\n❌ stella_md_node 관련 토픽을 찾을 수 없습니다.")
        print("\n모든 토픽 목록:")
        for i, topic in enumerate(sorted(stats['topics']), 1):
            print(f"  {i}. {topic}")
    
    # 4. 노드 그룹화 확인
    print("\n" + "=" * 80)
    print("노드 그룹화 결과:")
    print("=" * 80)
    
    enriched = endpoint_mapper.enrich_submessages(submessages)
    node_grouper = NodeGrouper()
    grouped = node_grouper.group_by_node(enriched)
    
    print(f"\n발견된 노드: {len(grouped)}개\n")
    for node_name, messages in sorted(grouped.items(), key=lambda x: len(x[1]), reverse=True):
        topics_in_node = set(msg.get('topic', '') for msg in messages if msg.get('topic'))
        print(f"• {node_name}: {len(messages)} messages, {len(topics_in_node)} topics")
        
        # stella_md 관련이면 토픽 출력
        if 'stella_md' in node_name.lower():
            print(f"  Topics:")
            for topic in sorted(topics_in_node)[:10]:
                print(f"    - {topic}")


if __name__ == "__main__":
    main()
