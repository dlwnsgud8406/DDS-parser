#!/usr/bin/env python3

import sys
import argparse
import pandas as pd
from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser  # RTPSParser → EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor

def main():
    # 인자 파싱
    parser = argparse.ArgumentParser(
        description="DDS Parser",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
    예시:
        python main.py data/participated_57_first_bye.pcapng
        python main.py data/participated_57_first_bye.pcapng -w 2.0
        python main.py data/participated_57_first_bye.pcapng -o output.csv
            """
    )
    parser.add_argument(
        'pcap_file',
        help='입력 pcapng 파일 경로'
    )

    parser.add_argument(
        '-w', '--window',
        type=float,
        default=1.0,
        help='윈도우 크기 (초) [기본값: 1.0]'
    )
    
    parser.add_argument(
        '-o', '--output',
        default='output.csv',
        help='출력 CSV 파일 경로 [기본값: output.csv]'
    )
    
    parser.add_argument(
        '-n', '--max-packets',
        type=int,
        default=None,
        help='최대 처리 패킷 수 (테스트용) [기본값: 제한 없음]'
    )
    args = parser.parse_args()

    # 시작 메시지
    print("=" * 70)
    print("DDS Parser 시작")
    print("=" * 70)
    print(f"입력 파일: {args.pcap_file}")
    print(f"윈도우 크기: {args.window}초")
    print(f"출력 파일: {args.output}")
    if args.max_packets:
        print(f"최대 패킷 수: {args.max_packets:,}")
    print("=" * 70)

    # 파이프라인 구성
    try:
        source = PcapSource(args.pcap_file)
        parser = EnhancedRTPSParser()  # RTPSParser() → EnhancedRTPSParser()
        sink = DataFrameSink()
        processor = TimeWindowProcessor(
            window_seconds=args.window,
            max_packets=args.max_packets
        )
    except FileNotFoundError as e:
        print(f"오류: {e}")
        sys.exit(1)
    except ValueError as e:
        print(f"오류: {e}")
        sys.exit(1)
    
    print("\n파싱 처리 중...")
    try:
        processor.process_stream(source, parser, sink)
    except Exception as e:
        print(f"처리 중 오류 발생: {e}")
        sys.exit(1)
    
    df = sink.get_result()

    if len(df) == 0:
        print("경고: 파싱된 데이터가 없습니다.")
        sys.exit(0)
    
    print(f"파싱완료 - 총 {len(df)}개의 레코드가 파싱되었습니다.")

    # timestamp를 숫자로 변환
    df['timestamp'] = pd.to_numeric(df['timestamp'], errors='coerce')

    # 기본 통계
    print("\n" + "=" * 70)
    print("기본 통계")
    print("=" * 70)
    print(f"총 패킷 수: {len(df)}")
    print(f"컬럼 수: {len(df.columns)}")
    print(f"시간 범위: {df['timestamp'].max() - df['timestamp'].min():.2f}초")
    print(f"첫 패킷 시각 (UTC): {pd.to_datetime(df['timestamp'].min(), unit='s')}")
    print(f"마지막 패킷 시각 (UTC): {pd.to_datetime(df['timestamp'].max(), unit='s')}")

    # KST 변환 추가
    print("\n한국시간(KST) 변환 중...")
    df['timestamp_kst'] = pd.to_datetime(df['timestamp'], unit='s') \
        .dt.tz_localize('UTC') \
        .dt.tz_convert('Asia/Seoul')
    
    print("\n" + "=" * 70)
    print("샘플 데이터 (처음 5행)")
    print("=" * 70)
    print(df[['timestamp_kst', 'frame_number', 'submsg_id']].head())

    # CSV 저장
    print(f"\n 출력 파일로 저장 중: {args.output} ...")
    df.to_csv(args.output, index=False)
    print("저장 완료.")

    # 완료 메시지
    print("\n" + "=" * 70)
    print("완료!")
    print("=" * 70)
    print(f"결과 파일: {args.output}")
    print(f"총 {len(df)}행, {len(df.columns)}개 컬럼")

if __name__ == "__main__":
    main()
