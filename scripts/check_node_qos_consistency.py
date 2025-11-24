#!/usr/bin/env python3
"""
노드별 QoS 일관성 검사

한 노드 안의 토픽들이 같은 QoS를 가지는지 확인
"""

import sys
from pathlib import Path
from collections import defaultdict

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor
from src.transformer import EndpointMapper, NodeGrouper, QoSAnalyzer


def main():
    pcap_file = "data/shm.pcapng"
    max_packets = 5000
    
    print("=" * 80)
    print("노드별 QoS 일관성 검사")
    print("=" * 80)
    print(f"파일: {pcap_file}")
    print(f"최대 패킷: {max_packets:,}")
    print()
    
    # 1. PCAP 파싱
    print("[1/5] PCAP 파일 파싱...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage 파싱 완료\n")
    
    # 2. SEDP 매핑
    print("[2/5] SEDP 매핑...")
    endpoint_mapper = EndpointMapper()
    endpoint_mapper.build_mapping(submessages)
    enriched_submessages = endpoint_mapper.enrich_submessages(submessages)
    print(f"  ✓ 완료\n")
    
    # 3. QoS 추론
    print("[3/5] QoS 추론...")
    qos_analyzer = QoSAnalyzer()
    topic_qos_map = qos_analyzer.analyze_messages(enriched_submessages)
    print(f"  ✓ {len(topic_qos_map)}개 토픽 분석 완료\n")
    
    # 4. 노드별 그룹화
    print("[4/5] 노드별 그룹화...")
    node_grouper = NodeGrouper()
    grouped_by_node = node_grouper.group_by_node(enriched_submessages)
    print(f"  ✓ {len(grouped_by_node)}개 노드 발견\n")
    
    # 5. 노드별 QoS 수집
    print("[5/5] 노드별 QoS 수집...")
    node_qos = defaultdict(lambda: defaultdict(list))
    
    for node_name, messages in grouped_by_node.items():
        for msg in messages:
            topic = msg.get('topic', 'unknown')
            if topic in topic_qos_map:
                qos = topic_qos_map[topic]
                node_qos[node_name]['topics'].append(topic)
                node_qos[node_name]['reliability'].append(qos['reliability'])
                node_qos[node_name]['durability'].append(qos['durability'])
                node_qos[node_name]['frequency'].append(qos['frequency_hz'])
    
    print(f"  ✓ 완료\n")
    
    # 결과 분석
    print("=" * 80)
    print("노드별 QoS 다양성 분석")
    print("=" * 80)
    print()
    
    for node_name in sorted(node_qos.keys()):
        info = node_qos[node_name]
        
        # 고유한 QoS 값들
        unique_reliability = set(info['reliability'])
        unique_durability = set(info['durability'])
        unique_topics = set(info['topics'])
        
        print(f"📦 {node_name}")
        print(f"   토픽 수: {len(unique_topics)}개")
        print(f"   Reliability: {', '.join(sorted(unique_reliability))}")
        print(f"   Durability: {', '.join(sorted(unique_durability))}")
        
        # QoS가 다양한가?
        if len(unique_reliability) > 1 or len(unique_durability) > 1:
            print(f"   ⚠️  이 노드는 여러 QoS를 사용합니다!")
            
            # 토픽별 상세 정보
            print(f"\n   토픽별 QoS:")
            topic_qos_detail = {}
            for topic in unique_topics:
                if topic in topic_qos_map:
                    qos = topic_qos_map[topic]
                    topic_qos_detail[topic] = qos
            
            for topic in sorted(topic_qos_detail.keys()):
                qos = topic_qos_detail[topic]
                freq = f"{qos['frequency_hz']:.1f} Hz" if qos['frequency_hz'] > 0 else "-"
                print(f"     • {topic}")
                print(f"       Reliability: {qos['reliability']}, Durability: {qos['durability']}, Freq: {freq}")
        else:
            print(f"   ✅ 일관된 QoS: {list(unique_reliability)[0]} / {list(unique_durability)[0]}")
        
        print()
    
    # 요약
    print("=" * 80)
    print("요약")
    print("=" * 80)
    
    consistent_nodes = []
    inconsistent_nodes = []
    
    for node_name, info in node_qos.items():
        unique_reliability = set(info['reliability'])
        unique_durability = set(info['durability'])
        
        if len(unique_reliability) == 1 and len(unique_durability) == 1:
            consistent_nodes.append(node_name)
        else:
            inconsistent_nodes.append(node_name)
    
    print(f"\n총 노드 수: {len(node_qos)}개")
    print(f"✅ QoS 일관된 노드: {len(consistent_nodes)}개")
    for node in consistent_nodes:
        print(f"   • {node}")
    
    print(f"\n⚠️  QoS 다양한 노드: {len(inconsistent_nodes)}개")
    for node in inconsistent_nodes:
        print(f"   • {node}")
    
    print("\n💡 결론:")
    if inconsistent_nodes:
        print("   → 한 노드 안에서도 토픽마다 다른 QoS를 사용할 수 있습니다!")
        print("   → 각 토픽은 독립적인 QoS 정책을 가집니다.")
    else:
        print("   → 이 데이터에서는 모든 노드가 일관된 QoS를 사용합니다.")


if __name__ == "__main__":
    main()
