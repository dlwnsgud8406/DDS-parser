#!/usr/bin/env python3
"""
EnhancedRTPSParser 통합 테스트
실제 PCAP 파일을 사용하여 파서 검증
"""

import pytest
import sys
from pathlib import Path

# 프로젝트 루트 추가
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.parser import EnhancedRTPSParser
from src.packet_source import PcapSource


class TestEnhancedRTPSParserIntegration:
    """EnhancedRTPSParser 통합 테스트"""
    
    @pytest.fixture
    def sample_pcap(self):
        """샘플 PCAP 경로"""
        pcap_path = Path(__file__).parent.parent.parent / "data" / "participated_57_first_bye.pcapng"
        if not pcap_path.exists():
            pytest.skip(f"PCAP 파일 없음: {pcap_path}")
        return str(pcap_path)
    
    @pytest.fixture
    def parser(self):
        """파서 인스턴스"""
        return EnhancedRTPSParser()
    
    def test_parser_initialization(self, parser):
        """파서 초기화 확인"""
        assert parser is not None
        assert parser.pid_extractor is not None
        assert len(parser.SUBMSG_ID_TO_NAME) > 0
    
    def test_parse_single_packet(self, parser, sample_pcap):
        """단일 패킷 파싱"""
        source = PcapSource(sample_pcap)
        
        # 첫 번째 RTPS 패킷 찾기
        for packet in source:
            layers = packet.get('layers', {})
            if 'rtps' in layers:
                results = parser.parse(packet)
                
                assert results is not None
                assert isinstance(results, list)
                assert len(results) > 0
                
                # 첫 번째 submessage 검증
                submsg = results[0]
                assert 'frame_number' in submsg
                assert 'timestamp' in submsg
                assert 'submsg_index' in submsg
                assert 'submsg_id' in submsg
                assert 'submsg_name' in submsg
                assert 'guid' in submsg
                assert 'entity' in submsg
                assert 'pids' in submsg
                
                break
    
    def test_parse_batch(self, parser, sample_pcap):
        """여러 패킷 일괄 파싱"""
        source = PcapSource(sample_pcap)
        packets = []
        
        # 처음 10개 패킷 수집
        for i, packet in enumerate(source):
            packets.append(packet)
            if i >= 9:
                break
        
        results = parser.parse_batch(packets)
        
        assert isinstance(results, list)
        assert len(results) > 0
        
        # 각 결과 검증
        for result in results:
            assert 'frame_number' in result
            assert 'submsg_id' in result
            assert 'pids' in result
    
    def test_guid_extraction(self, parser, sample_pcap):
        """GUID 추출 확인"""
        source = PcapSource(sample_pcap)
        
        guid_found = False
        for packet in source:
            results = parser.parse(packet)
            if results:
                for submsg in results:
                    guid = submsg['guid']
                    if all([guid['hostId'], guid['appId'], guid['instanceId']]):
                        guid_found = True
                        
                        # GUID 값 검증
                        assert isinstance(guid['hostId'], int)
                        assert isinstance(guid['appId'], int)
                        assert isinstance(guid['instanceId'], int)
                        
                        print(f"\n✓ GUID 추출 성공:")
                        print(f"  Host: {guid['hostId']:08x}")
                        print(f"  App:  {guid['appId']:08x}")
                        print(f"  Inst: {guid['instanceId']:08x}")
                        break
            if guid_found:
                break
        
        assert guid_found, "GUID를 가진 패킷을 찾지 못함"
    
    def test_entity_id_extraction(self, parser, sample_pcap):
        """Entity ID 추출 확인"""
        source = PcapSource(sample_pcap)
        
        entity_found = False
        for packet in source:
            results = parser.parse(packet)
            if results:
                for submsg in results:
                    entity = submsg['entity']
                    if entity['rdEntityId'] or entity['wrEntityId']:
                        entity_found = True
                        
                        print(f"\n✓ Entity ID 추출 성공:")
                        if entity['rdEntityId']:
                            print(f"  Reader: 0x{entity['rdEntityId']:08x}")
                        if entity['wrEntityId']:
                            print(f"  Writer: 0x{entity['wrEntityId']:08x}")
                        break
            if entity_found:
                break
        
        assert entity_found, "Entity ID를 가진 패킷을 찾지 못함"
    
    def test_pid_extraction(self, parser, sample_pcap):
        """PID 값 추출 확인"""
        source = PcapSource(sample_pcap)
        
        pid_found = False
        for packet in source:
            results = parser.parse(packet)
            if results:
                for submsg in results:
                    pids = submsg['pids']
                    if len(pids) > 0:
                        pid_found = True
                        
                        print(f"\n✓ PID 추출 성공 ({len(pids)}개):")
                        for key in list(pids.keys())[:5]:
                            print(f"  {key}: {pids[key]}")
                        if len(pids) > 5:
                            print(f"  ... 외 {len(pids) - 5}개")
                        break
            if pid_found:
                break
        
        assert pid_found, "PID를 가진 패킷을 찾지 못함"
    
    def test_submessage_classification(self, parser, sample_pcap):
        """Submessage 분류 확인 (DATA(p), DATA(w), DATA(r) 등)"""
        source = PcapSource(sample_pcap)
        
        submsg_types = set()
        for packet in source:
            results = parser.parse(packet)
            if results:
                for submsg in results:
                    submsg_types.add(submsg['submsg_name'])
        
        print(f"\n✓ 발견된 Submessage 타입:")
        for stype in sorted(submsg_types):
            print(f"  - {stype}")
        
        assert len(submsg_types) > 0, "Submessage를 찾지 못함"
    
    def test_full_pcap_parsing(self, parser, sample_pcap):
        """전체 PCAP 파싱 (성능 및 안정성 테스트)"""
        source = PcapSource(sample_pcap)
        packets = list(source)
        
        print(f"\n총 {len(packets)}개 패킷 파싱 중...")
        results = parser.parse_batch(packets)
        
        print(f"✓ {len(results)}개 submessage 파싱 완료")
        
        assert len(results) > 0
        
        # 통계 수집
        participants = set()
        entities = set()
        submsg_types = {}
        
        for submsg in results:
            # Participant
            guid = submsg['guid']
            if all([guid['hostId'], guid['appId'], guid['instanceId']]):
                participants.add((guid['hostId'], guid['appId'], guid['instanceId']))
            
            # Entity
            entity = submsg['entity']
            if entity['rdEntityId']:
                entities.add(entity['rdEntityId'])
            if entity['wrEntityId']:
                entities.add(entity['wrEntityId'])
            
            # Submessage Type
            stype = submsg['submsg_name']
            submsg_types[stype] = submsg_types.get(stype, 0) + 1
        
        print(f"\n통계:")
        print(f"  Participants: {len(participants)}개")
        print(f"  Entities: {len(entities)}개")
        print(f"  Submessage Types: {len(submsg_types)}개")
        
        # Phase 1 analyze_pcap.py 결과와 비교
        assert len(participants) == 5, f"Expected 5 participants, got {len(participants)}"
        assert len(entities) == 10, f"Expected 10 entities, got {len(entities)}"
    
    def test_extract_submessages(self, parser):
        """_extract_submessages() 메서드 테스트"""
        # 단일 submessage
        rtps_single = {
            "rtps_rtps_sm_id": "0x09"
        }
        result = parser._extract_submessages(rtps_single)
        assert len(result) == 1
        assert result[0]["rtps_rtps_sm_id"] == "0x09"
        
        # 여러 submessage
        rtps_multiple = {
            "rtps_rtps_sm_id": ["0x09", "0x15"],
            "rtps_rtps_sm_flags": ["0x01", "0x05"]
        }
        result = parser._extract_submessages(rtps_multiple)
        assert len(result) == 2
        assert result[0]["rtps_rtps_sm_id"] == "0x09"
        assert result[1]["rtps_rtps_sm_id"] == "0x15"
        assert result[0]["rtps_rtps_sm_flags"] == "0x01"
        assert result[1]["rtps_rtps_sm_flags"] == "0x05"


# pytest 실행용
if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
