#!/usr/bin/env python3
"""
서비스 토픽 vs 노드 패턴 확인

rq/rr 다음에 오는 것이 노드인지 서비스인지 구분
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor
from src.transformer import EndpointMapper


def main():
    pcap_file = "data/shm.pcapng"
    max_packets = None
    
    print("=" * 80)
    print("서비스 토픽 패턴 분석")
    print("=" * 80)
    
    # 1. PCAP 파싱
    print("\n[1/2] PCAP 파일 파싱 중...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage 파싱 완료")
    
    # 2. Endpoint → Topic 매핑
    print("\n[2/2] SEDP 매핑 테이블 생성...")
    endpoint_mapper = EndpointMapper()
    endpoint_mapper.build_mapping(submessages)
    
    stats = endpoint_mapper.get_statistics()
    topics = sorted(stats['topics'])
    
    print("\n" + "=" * 80)
    print("rq/rr 패턴 분석")
    print("=" * 80)
    
    # 노드 패턴 (3개 세그먼트)
    node_topics = []
    # 서비스 패턴 (2개 세그먼트)
    service_topics = []
    
    for topic in topics:
        if topic.startswith('rq/') or topic.startswith('rr/'):
            segments = topic.split('/')
            if len(segments) == 3:
                # rq/<node>/<service> 형식
                node_topics.append(topic)
            elif len(segments) == 2:
                # rq/<service> 형식
                service_topics.append(topic)
    
    print(f"\n✅ 노드 패턴 (rq/rr/<node>/<service>): {len(node_topics)}개")
    for topic in node_topics[:10]:
        print(f"   • {topic}")
    if len(node_topics) > 10:
        print(f"   ... 외 {len(node_topics) - 10}개")
    
    print(f"\n⚠️  서비스 패턴 (rq/rr/<service>): {len(service_topics)}개")
    for topic in service_topics:
        print(f"   • {topic}")
    
    # teleop_keyboard 확인
    print("\n" + "=" * 80)
    print("teleop_keyboard 관련 토픽")
    print("=" * 80)
    
    teleop_topics = [t for t in topics if 'teleop_keyboard' in t]
    if teleop_topics:
        print(f"\n✅ teleop_keyboard 관련 토픽: {len(teleop_topics)}개")
        for topic in teleop_topics:
            print(f"   • {topic}")
    else:
        print("\n❌ teleop_keyboard 관련 토픽 없음")


if __name__ == "__main__":
    main()
