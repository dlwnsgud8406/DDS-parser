import pytest

@pytest.fixture
def sample_pcap():
    """테스트용 pcap 파일 경로"""
    return "../data/participated_57_first_bye.pcapng"