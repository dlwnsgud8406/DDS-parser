#!/usr/bin/env python3
"""
PCAP → Excel 변환 스크립트

PCAP 파일을 파싱하여 v2.2 형식의 Excel 파일 생성
"""

import sys
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor
from src.transformer import TopicGrouper, NodeGrouper, EndpointMapper, PivotTableBuilder, QoSAnalyzer
from src.excel_writer import ExcelWriter
import pandas as pd
from collections import defaultdict, Counter


def main():
    parser = argparse.ArgumentParser(
        description="PCAP → Excel 변환 (v2.2 Multi-Row)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
    python scripts/generate_excel.py data/participated_57_first_bye.pcapng
    python scripts/generate_excel.py data/rtps_only_stream.pcapng -o output/stream.xlsx
    python scripts/generate_excel.py data/test.pcapng -w 2.0 -n 1000
        """
    )
    
    parser.add_argument(
        'pcap_file',
        help='입력 PCAP 파일 경로'
    )
    
    parser.add_argument(
        '-o', '--output',
        default=None,
        help='출력 Excel 파일 경로 [기본값: output/<pcap_name>_analysis.xlsx]'
    )
    
    parser.add_argument(
        '-w', '--window',
        type=float,
        default=1.0,
        help='시간 윈도우 크기 (초) [기본값: 1.0]'
    )
    
    parser.add_argument(
        '-n', '--max-packets',
        type=int,
        default=None,
        help='최대 처리 패킷 수 (테스트용) [기본값: 제한 없음]'
    )
    
    args = parser.parse_args()
    
    # 출력 파일명 생성
    if args.output is None:
        pcap_name = Path(args.pcap_file).stem
        output_path = f"output/{pcap_name}_analysis.xlsx"
    else:
        output_path = args.output
    
    print("=" * 80)
    print("RTPS PCAP → Excel 변환 (v2.2)")
    print("=" * 80)
    print(f"입력 파일: {args.pcap_file}")
    print(f"출력 파일: {output_path}")
    print(f"윈도우 크기: {args.window}초")
    if args.max_packets:
        print(f"최대 패킷 수: {args.max_packets:,}")
    print("=" * 80)
    
    try:
        # 1. PCAP 파싱
        print("\n[1/5] PCAP 파일 파싱...")
        source = PcapSource(args.pcap_file)
        parser = EnhancedRTPSParser()
        sink = DataFrameSink()
        processor = TimeWindowProcessor(
            window_seconds=args.window,
            max_packets=args.max_packets
        )
        
        processor.process_stream(source, parser, sink)
        df = sink.get_result()
        
        submessages = df.to_dict('records')
        print(f"  ✓ {len(submessages):,}개 submessage 파싱 완료")
        
        # 2. Endpoint → Topic 매핑 생성
        print("\n[2/6] SEDP 매핑 테이블 생성...")
        print(f"  → {len(submessages):,}개 submessage 분석 중...")
        endpoint_mapper = EndpointMapper()
        endpoint_mapper.build_mapping(submessages)
        
        stats = endpoint_mapper.get_statistics()
        print(f"  ✓ {stats['total_endpoints']}개 endpoint 발견")
        print(f"    - Writers: {stats['writers']}")
        print(f"    - Readers: {stats['readers']}")
        print(f"    - Topics: {stats['topics_count']}")
        
        if stats['topics_count'] > 0:
            print(f"    📋 Topic 목록:")
            for topic in sorted(stats['topics']):
                print(f"       • {topic}")
        
        # 3. 모든 submessage에 topic 정보 추가
        print("\n[3/6] Submessage에 Topic 매핑...")
        print(f"  → Endpoint 정보로 역매핑 중...")
        enriched_submessages = endpoint_mapper.enrich_submessages(submessages)
        
        mapped_count = sum(1 for msg in enriched_submessages if 'topic' in msg)
        unmapped_count = len(enriched_submessages) - mapped_count
        print(f"  ✓ {mapped_count:,}/{len(enriched_submessages):,}개 submessage에 topic 매핑됨")
        if unmapped_count > 0:
            print(f"    ⚠️  {unmapped_count:,}개는 topic 정보 없음 (SPDP, INFO 등)")
        
        # 4. 메시지 타입별 분리
        print("\n[4/6] 메시지 타입별 분리...")
        topic_grouper = TopicGrouper()
        separated = topic_grouper.separate_by_message_type(enriched_submessages)
        
        print(f"  ✓ SPDP: {len(separated['spdp'])}개")
        print(f"  ✓ SEDP Writers: {len(separated['sedp_writers'])}개")
        print(f"  ✓ SEDP Readers: {len(separated['sedp_readers'])}개")
        print(f"  ✓ User Traffic: {len(separated['user_traffic'])}개")
        
        # 5. QoS 추론
        print("\n[5/7] QoS 정책 추론...")
        qos_analyzer = QoSAnalyzer()
        topic_qos_map = qos_analyzer.analyze_messages(enriched_submessages)
        print(f"  ✓ {len(topic_qos_map)}개 토픽의 QoS 추론 완료")
        
        # 6. 노드별 그룹화
        print("\n[6/7] ROS2 노드별 그룹화...")
        node_grouper = NodeGrouper()
        grouped_by_node = node_grouper.group_by_node(enriched_submessages)
        print(f"  ✓ {len(grouped_by_node)}개 노드 발견")
        
        if len(grouped_by_node) == 0:
            print("  ⚠️  노드가 없습니다. Excel 생성을 중단합니다.")
            sys.exit(1)
        
        # 7. Pivot Table 생성 (노드별) + QoS 추가
        print("\n[7/8] 노드별 Pivot Table 생성 + QoS 매핑...")
        builder = PivotTableBuilder(window_size=args.window)
        pivot_tables = {}
        
        for node_name, messages in grouped_by_node.items():
            df_pivot = builder.build(messages, participant_id=None)
            
            # QoS 정보 추가
            if 'topic' in df_pivot.columns:
                df_pivot['reliability'] = df_pivot['topic'].map(
                    lambda t: topic_qos_map.get(t, {}).get('reliability', '')
                )
                df_pivot['durability'] = df_pivot['topic'].map(
                    lambda t: topic_qos_map.get(t, {}).get('durability', '')
                )
                df_pivot['frequency_hz'] = df_pivot['topic'].map(
                    lambda t: topic_qos_map.get(t, {}).get('frequency_hz', 0)
                )
            
            # 시트명 생성
            sheet_name = node_grouper.format_node_name_for_sheet(node_name)
            
            pivot_tables[sheet_name] = df_pivot
            print(f"  ✓ {sheet_name}: {len(df_pivot):,} 행 ({len(messages)} messages)")
        
        # Overview 및 QoS Summary 생성
        print("\n[8/9] Overview, QoS Summary, SEDP 시트 생성...")
        
        # QoS Summary 데이터
        qos_summary_data = qos_analyzer.get_qos_summary_dataframe()
        df_qos_summary = pd.DataFrame(qos_summary_data)
        print(f"  ✓ QoS Summary 시트: {len(df_qos_summary)} 토픽")
        
        node_summary = node_grouper.get_node_summary(grouped_by_node)
        overview_data = []
        
        for summary in node_summary:
            sheet_name = node_grouper.format_node_name_for_sheet(summary['node_name'])
            
            # Submessage types를 문자열로 변환
            submsg_types_str = ", ".join(
                f"{k}({v})" for k, v in sorted(summary['submsg_types'].items())
            )
            
            overview_data.append({
                'Node': summary['node_name'],
                'Sheet Name': sheet_name,
                'Messages': summary['message_count'],
                'Topics': summary['topic_count'],
                'Types': submsg_types_str
            })
        
        df_overview = pd.DataFrame(overview_data)
        print(f"  ✓ Overview 시트 생성: {len(df_overview)} 노드")
        
        # SEDP 시트 데이터 준비
        sedp_data = endpoint_mapper.get_sedp_dataframe_data()
        df_sedp = pd.DataFrame(sedp_data)
        print(f"  ✓ SEDP 시트: {len(df_sedp)} endpoints")
        
        # 9. Excel 쓰기
        print("\n[9/9] Excel 쓰기...")
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        
        writer = ExcelWriter(output_path)
        
        # Overview 시트
        writer.write_overview(df_overview)
        print(f"  ✓ Overview 시트 작성")
        
        # QoS Summary 시트
        writer.write_qos_summary(df_qos_summary)
        print(f"  ✓ QoS Summary 시트 작성")
        
        # SEDP 시트 (간단한 테이블)
        writer.write_sedp_sheet(df_sedp)
        print(f"  ✓ SEDP 시트 작성")
        
        # 노드별 시트
        writer.write_node_sheets(pivot_tables)
        print(f"  ✓ {len(pivot_tables)}개 노드 시트 작성")
        
        writer.save()
        print(f"  ✓ 파일 저장: {output_path}")
        
        # 9. 최종 요약
        print("\n" + "=" * 80)
        print("✅ 완료!")
        print("=" * 80)
        
        file_size = Path(output_path).stat().st_size
        total_excel_rows = sum(len(df) for df in pivot_tables.values())
        
        print(f"파일: {output_path}")
        print(f"크기: {file_size / 1024:.1f} KB ({file_size / (1024*1024):.2f} MB)")
        print(f"\n시트 구성:")
        print(f"  - Overview: {len(df_overview)} 노드")
        print(f"  - SEDP: {len(df_sedp)} endpoints")
        print(f"  - 노드 시트: {len(pivot_tables)}개")
        print(f"\n데이터:")
        print(f"  - 총 Submessages: {len(submessages):,}개")
        print(f"  - Topic 매핑됨: {mapped_count:,}개")
        print(f"  - Excel 행: {total_excel_rows:,}개")
        print(f"\n노드 목록:")
        for i, node_name in enumerate(sorted(grouped_by_node.keys()), start=1):
            msg_count = len(grouped_by_node[node_name])
            print(f"  {i}. {node_name} ({msg_count:,} messages)")
        
    except FileNotFoundError as e:
        print(f"\n❌ 오류: 파일을 찾을 수 없습니다 - {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 오류: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    sys.exit(main())
