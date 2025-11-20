import pytest
from src.packet_source import PcapSource
from src.parser import RTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor


def test_processor_initialization():
    """프로세서 초기화 테스트"""
    processor = TimeWindowProcessor(window_seconds=1.0)
    assert processor.window_seconds == 1.0


def test_processor_invalid_window():
    """잘못된 윈도우 크기는 에러"""
    with pytest.raises(ValueError):
        TimeWindowProcessor(window_seconds=0)
    
    with pytest.raises(ValueError):
        TimeWindowProcessor(window_seconds=-1)


def test_processor_simple_stream(sample_pcap):
    """간단한 스트림 처리"""
    source = PcapSource(sample_pcap)
    parser = RTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0)
    
    processor.process_stream(source, parser, sink)
    
    df = sink.get_result()
    
    # 결과 확인
    assert len(df) > 0, "DataFrame이 비어있음"
    assert 'timestamp' in df.columns, "timestamp 컬럼 없음"
    assert 'frame_number' in df.columns, "frame_number 컬럼 없음"
    assert 'submsg_id' in df.columns, "submsg_id 컬럼 없음"


def test_processor_window_count(sample_pcap):
    """윈도우 개수 확인"""
    source = PcapSource(sample_pcap)
    parser = RTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=0.1)  # 작은 윈도우
    
    processor.process_stream(source, parser, sink)
    
    df = sink.get_result()
    
    # timestamp 범위 확인
    if len(df) > 0:
        time_range = df['timestamp'].max() - df['timestamp'].min()
        print(f"시간 범위: {time_range:.2f}초")
        print(f"패킷 수: {len(df)}")


def test_processor_timestamp_preserved(sample_pcap):
    """timestamp가 DataFrame에 제대로 저장되는지 확인"""
    source = PcapSource(sample_pcap)
    parser = RTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0)
    
    processor.process_stream(source, parser, sink)
    
    df = sink.get_result()
    
    # timestamp 기본 검증
    assert 'timestamp' in df.columns
    assert df['timestamp'].notna().all()
    assert len(df) > 0
    
    # 값이 합리적인 범위인지만 확인
    assert (df['timestamp'] > 1600000000).all()
    assert (df['timestamp'] < 2000000000).all()