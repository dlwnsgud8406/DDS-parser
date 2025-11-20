import pytest
from src.packet_source import PacketSource, PcapSource

def test_pcap_source_file_exists(sample_pcap):
    """파일이 존재하면 정상 생성"""
    source = PcapSource(sample_pcap)
    assert source.filepath == sample_pcap
    assert source._packets is None

def test_pcap_source_file_not_found():
    """존재하지 않는 파일은 에러"""
    with pytest.raises(FileNotFoundError):
        PcapSource("nonexistent.pcapng")

def test_pcap_source_iteration(sample_pcap):
    """패킷을 순회할 수 있는지 확인"""
    source = PcapSource(sample_pcap)
    packets = list(source)

    assert len(packets) > 0, "패킷이 비어있음"
    assert source._packets is not None, "패킷이 로드되지 않음"

def test_pcap_source_caching(sample_pcap):
    """두 번 iter 호출해도 한 번만 로드"""
    source = PcapSource(sample_pcap)

    packets1= list(source)
    cached = source._packets

    packets2 = list(source)
    
    assert cached is source._packets
    assert len(packets1) == len(packets2)

def test_pcap_source_timestamp(sample_pcap):
    """timestamp가 UTC float인지 확인"""
    source = PcapSource(sample_pcap)
    packet = next(iter(source))

    ts = source.get_timestamp(packet)

    assert isinstance(ts, float), f"timestamp가 float이 아님: {type(ts)}"
    assert ts > 1600000000, "timestamp가 비정상적으로 낮음"
    assert ts > 1600000000, "timestamp가 비정상적으로 낮음"

def test_pcap_source_packet_structure(sample_pcap):
    """패킷 구조 확인"""
    source = PcapSource(sample_pcap)
    packet = next(iter(source))

    assert 'layers' in packet, "layers 키가 없음"
    assert 'frame' in packet['layers'], "frame 키가 없음"
    assert 'rtps' in packet['layers'], "rtps 키가 없음 (RTPS 필터 실패)"

def test_abstract_class_cannot_instantiate():
    """추상 클래스는 인스턴스화 불가"""
    with pytest.raises(TypeError):
        PacketSource()