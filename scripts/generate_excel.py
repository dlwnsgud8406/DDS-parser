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
from src.transformer import TopicGrouper, EndpointMapper, PivotTableBuilder
from src.excel_writer import ExcelWriter
import pandas as pd


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
        
        # 5. Topic별 그룹화
        print("\n[5/6] Topic별 그룹화...")
        grouped_by_topic = topic_grouper.group_by_topic(enriched_submessages)
        print(f"  ✓ {len(grouped_by_topic)}개 Topic 발견")
        
        if len(grouped_by_topic) == 0:
            print("  ⚠️  Topic이 없습니다. Excel 생성을 중단합니다.")
            sys.exit(1)
        
        # 6. Pivot Table 생성 (Topic별)
        print("\n[6/6] Topic별 Pivot Table 생성...")
        builder = PivotTableBuilder(window_size=args.window)
        pivot_tables = {}
        
        for topic, messages in grouped_by_topic.items():
            df_pivot = builder.build(messages, participant_id=None)
            
            # 시트명 생성
            sheet_name = topic_grouper.format_topic_name_for_sheet(topic)
            
            pivot_tables[sheet_name] = df_pivot
            print(f"  ✓ {sheet_name}: {len(df_pivot):,} 행 ({len(messages)} messages)")
        
        # Overview 생성 (Topic 기반)
        print("\n[7/7] Overview 및 SEDP 시트 생성...")
        
        topic_summary = topic_grouper.get_topic_summary(grouped_by_topic)
        overview_data = []
        
        for summary in topic_summary:
            sheet_name = topic_grouper.format_topic_name_for_sheet(summary['topic'])
            
            # Submessage types를 문자열로 변환
            submsg_types_str = ", ".join(
                f"{k}({v})" for k, v in sorted(summary['submsg_types'].items())
            )
            
            overview_data.append({
                'Topic': summary['topic'],
                'Sheet Name': sheet_name,
                'Messages': summary['message_count'],
                'Participants': summary['participant_count'],
                'Writers': summary['writer_count'],
                'Readers': summary['reader_count'],
                'Types': submsg_types_str
            })
        
        df_overview = pd.DataFrame(overview_data)
        print(f"  ✓ Overview 시트 생성: {len(df_overview)} topics")
        
        # SEDP 시트 데이터 준비
        sedp_data = endpoint_mapper.get_sedp_dataframe_data()
        df_sedp = pd.DataFrame(sedp_data)
        print(f"  ✓ SEDP 시트: {len(df_sedp)} endpoints")
        
        # 8. Excel 쓰기
        print("\n[8/8] Excel 쓰기...")
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        
        writer = ExcelWriter(output_path)
        
        # Overview 시트
        writer.write_overview(df_overview)
        print(f"  ✓ Overview 시트 작성")
        
        # SEDP 시트 (간단한 테이블)
        writer.write_sedp_sheet(df_sedp)
        print(f"  ✓ SEDP 시트 작성")
        
        # Topic별 시트
        writer.write_topic_sheets(pivot_tables)
        print(f"  ✓ {len(pivot_tables)}개 Topic 시트 작성")
        
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
        print(f"  - Overview: {len(df_overview)} topics")
        print(f"  - SEDP: {len(df_sedp)} endpoints")
        print(f"  - Topic 시트: {len(pivot_tables)}개")
        print(f"\n데이터:")
        print(f"  - 총 Submessages: {len(submessages):,}개")
        print(f"  - Topic 매핑됨: {mapped_count:,}개")
        print(f"  - Excel 행: {total_excel_rows:,}개")
        print(f"\nTopic 목록:")
        for i, topic in enumerate(sorted(grouped_by_topic.keys()), start=1):
            msg_count = len(grouped_by_topic[topic])
            print(f"  {i}. {topic} ({msg_count:,} messages)")
        
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
