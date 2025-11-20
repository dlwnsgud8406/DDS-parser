import pytest
from src.packet_source import PcapSource
from src.parser import RTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor

def test_end_to_end_pipeline(sample_pcap):
    """ 전체 파이프라인 E2E 테스트 """
    source = PcapSource(sample_pcap)
    parser = RTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0)

    processor.process_stream(source, parser, sink)

    df = sink.get_result()

    assert len(df) > 0, "DataFrame이 비어있음"
    assert 'timestamp' in df.columns, "DataFrame에 'timestamp' 컬럼이 없음"
    assert 'frame_number' in df.columns, "DataFrame에 'frame_number' 컬럼이 없음"
    assert 'submsg_id' in df.columns, "DataFrame에 'submsg_id' 컬럼이 없음"

    # 통계 출력
    print(f"\n{'='*60}")
    print(f"통합 테스트 결과:")
    print(f"{'='*60}")
    print(f"총 패킷 수: {len(df)}")
    print(f"컬럼 수: {len(df.columns)}")
    print(f"시간 범위: {df['timestamp'].max() - df['timestamp'].min():.2f}초")
    print(f"{'='*60}")
    print(f"\n샘플 데이터 (처음 5행):")
    print(df.head())
    print(f"\n컬럼 목록:")
    print(df.columns.tolist())


def test_different_window_sizes(sample_pcap):
    """ 다양안 윈도우 크기로 제스트"""
    window_sizes = [0.5, 1.0, 2.0, 5.0]

    for window_size in window_sizes:
        source = PcapSource(sample_pcap)
        parser = RTPSParser()
        sink = DataFrameSink()
        processor = TimeWindowProcessor(window_seconds=window_size)

        processor.process_stream(source, parser, sink)
        df = sink.get_result()

        assert len(df) > 0, f"윈도우 크기 {window_size}초에서 DataFrame이 비어있음"
        print(f"윈도우 크기 {window_size}초에서 패킷 수: {len(df)}")

def test_multiple_runs_same_result(sample_pcap):
    """ 같은 파일을 여러 번 실행해도 같은 결과 나오는지"""
    results = []

    for _ in range(3):
        source = PcapSource(sample_pcap)
        parser = RTPSParser()
        sink = DataFrameSink()
        processor = TimeWindowProcessor(window_seconds=1.0)

        processor.process_stream(source, parser, sink)
        df = sink.get_result()
        results.append(len(df))

    assert len(set(results)) == 1, f"실행마다 결과가 다릅니다.: {results}"

def test_empty_dataframe_columns(sample_pcap):
    """ DataFrame 컬럼 구조 확인 테스트 """
    source = PcapSource(sample_pcap)
    parser = RTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0)

    processor.process_stream(source, parser, sink)
    df = sink.get_result()

    required_columns = ['timestamp', 'frame_number', 'submsg_id']
    for col in required_columns:
        assert col in df.columns, f"DataFrame에 '{col}' 컬럼이 없음"
    
    assert df['timestamp'].dtype == float, "timestamp는 float이어야 함"



