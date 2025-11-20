#!/usr/bin/env python3
"""
PIDExtractor 단위 테스트
"""

import pytest
import sys
from pathlib import Path

# 프로젝트 루트 추가
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.parser.pid_extractor import PIDExtractor


class TestToInt:
    """to_int() 함수 테스트"""
    
    def test_none_returns_none(self):
        """None 입력 시 None 반환"""
        assert PIDExtractor.to_int(None) is None
    
    def test_int_returns_same(self):
        """정수 입력 시 그대로 반환"""
        assert PIDExtractor.to_int(15) == 15
        assert PIDExtractor.to_int(0) == 0
        assert PIDExtractor.to_int(4611) == 4611
    
    def test_hex_string(self):
        """16진수 문자열 변환"""
        assert PIDExtractor.to_int("0x0e") == 14
        assert PIDExtractor.to_int("0x0E") == 14
        assert PIDExtractor.to_int("0xff") == 255
        assert PIDExtractor.to_int("0x00001204") == 4612
    
    def test_decimal_string(self):
        """10진수 문자열 변환"""
        assert PIDExtractor.to_int("123") == 123
        assert PIDExtractor.to_int("0") == 0
        assert PIDExtractor.to_int("516") == 516
    
    def test_list_takes_first(self):
        """리스트는 첫 번째 원소 사용"""
        assert PIDExtractor.to_int(["0x09", "0x15"]) == 9
        assert PIDExtractor.to_int([42, 100]) == 42
        assert PIDExtractor.to_int(["14", "15"]) == 14
    
    def test_empty_list(self):
        """빈 리스트는 None"""
        assert PIDExtractor.to_int([]) is None
    
    def test_complex_string(self):
        """복잡한 문자열에서 숫자 추출"""
        assert PIDExtractor.to_int("value: 0x1a") == 26
        assert PIDExtractor.to_int("count=50") == 50


class TestExtractPIDValues:
    """extract_pid_values() 함수 테스트"""
    
    def test_empty_submsg(self):
        """빈 submessage는 빈 dict 반환"""
        result = PIDExtractor.extract_pid_values({})
        assert isinstance(result, dict)
        assert len(result) == 0
    
    def test_protocol_version(self):
        """Protocol Version 추출"""
        submsg = {
            "rtps_rtps_version_major": "2",
            "rtps_rtps_version_minor": "3"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_PROTOCOL_VERSION_major"] == "2"
        assert result["PID_PROTOCOL_VERSION_minor"] == "3"
    
    def test_vendor_id(self):
        """Vendor ID 추출"""
        submsg = {
            "rtps_rtps_vendorId": "0x0000010f"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert "PID_VENDOR_ID_vendorid" in result
        assert result["PID_VENDOR_ID_vendorid"] == "0x0000010f"
    
    def test_participant_guid(self):
        """Participant GUID 추출"""
        submsg = {
            "rtps_rtps_hostId": "0x010f50af",
            "rtps_rtps_appId": "0x25af9462",
            "rtps_rtps_sm_guidPrefix_instanceId": "0x01000000"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_PARTICIPANT_GUID_hostid"] == "0x010f50af"
        assert result["PID_PARTICIPANT_GUID_appid"] == "0x25af9462"
        assert result["PID_PARTICIPANT_GUID_instanceid"] == "0x01000000"
    
    def test_entity_id_writer(self):
        """Writer Entity ID 추출 및 분해"""
        submsg = {
            "rtps_rtps_sm_wrEntityId": "0x00001203"  # Writer Entity
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_PARTICIPANT_GUID_entityid"] == "0x00001203"
        assert result["PID_PARTICIPANT_GUID_entitykey"] == "0x000012"
        assert result["PID_PARTICIPANT_GUID_entitykind"] == "0x03"
    
    def test_entity_id_reader(self):
        """Reader Entity ID 추출 (Writer 없을 때)"""
        submsg = {
            "rtps_rtps_sm_rdEntityId": "0x00001204"  # Reader Entity
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_PARTICIPANT_GUID_entityid"] == "0x00001204"
        assert result["PID_PARTICIPANT_GUID_entitykey"] == "0x000012"
        assert result["PID_PARTICIPANT_GUID_entitykind"] == "0x04"
    
    def test_entity_id_both_prefers_writer(self):
        """Reader와 Writer 둘 다 있으면 Writer 우선"""
        submsg = {
            "rtps_rtps_sm_rdEntityId": "0x00001204",
            "rtps_rtps_sm_wrEntityId": "0x00001203"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        # Writer Entity ID가 선택되어야 함
        assert result["PID_PARTICIPANT_GUID_entityid"] == "0x00001203"
    
    def test_locators(self):
        """Locator 정보 추출"""
        submsg = {
            "rtps_rtps_locator_kind": ["1", "1", "1"],
            "rtps_rtps_locator_port": ["7400", "7401"],
            "rtps_rtps_locator_ipv4": ["192.168.0.1", "192.168.0.2"]
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        # METATRAFFIC (첫 번째)
        assert result["PID_METATRAFFIC_UNICAST_LOCATOR_kind"] == "1"
        assert result["PID_METATRAFFIC_UNICAST_LOCATOR_port"] == "7400"
        assert result["PID_METATRAFFIC_UNICAST_LOCATOR_ipv4"] == "192.168.0.1"
        
        # DEFAULT (세 번째 kind, 두 번째 port/ipv4)
        assert result["PID_DEFAULT_UNICAST_LOCATOR_kind"] == "1"
        assert result["PID_DEFAULT_UNICAST_LOCATOR_port"] == "7401"
        assert result["PID_DEFAULT_UNICAST_LOCATOR_ipv4"] == "192.168.0.2"
    
    def test_lease_duration(self):
        """Lease Duration 추출 및 계산"""
        submsg = {
            "rtps_rtps_param_ntpTime_sec": "100",
            "rtps_rtps_param_ntpTime_fraction": "0"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_PARTICIPANT_LEASE_DURATION_seconds"] == "100"
        assert result["PID_PARTICIPANT_LEASE_DURATION_fraction"] == "0"
        assert result["PID_PARTICIPANT_LEASE_DURATION_lease_duration"] == "100.000000s"
    
    def test_builtin_endpoint_set(self):
        """Builtin Endpoint Set 추출"""
        submsg = {
            "rtps_rtps_param_builtin_endpoint_set": "0x00000003"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_BUILTIN_ENDPOINT_SET_flags"] == "0x00000003"
    
    def test_key_hash(self):
        """KEY_HASH (inline QoS) 추출"""
        submsg = {
            "rtps_rtps_guid": "01:0f:50:af:25:af:94:62:01:00:00:00:00:01:00:c2"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_KEY_HASH_guid"] == "01:0f:50:af:25:af:94:62:01:00:00:00:00:01:00:c2"
    
    def test_status_info_disposed(self):
        """STATUS_INFO - DISPOSED 플래그"""
        submsg = {
            "rtps_rtps_param_status_info": "0x00000001"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_STATUS_INFO_flags"] == "DISPOSED"
    
    def test_status_info_unregistered(self):
        """STATUS_INFO - UNREGISTERED 플래그"""
        submsg = {
            "rtps_rtps_param_status_info": "0x00000002"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_STATUS_INFO_flags"] == "UNREGISTERED"
    
    def test_status_info_both(self):
        """STATUS_INFO - 두 플래그 모두"""
        submsg = {
            "rtps_rtps_param_status_info": "0x00000003"  # DISPOSED | UNREGISTERED
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert "DISPOSED" in result["PID_STATUS_INFO_flags"]
        assert "UNREGISTERED" in result["PID_STATUS_INFO_flags"]
    
    def test_pid_array_type_consistency(self):
        """PID 배열 - TYPE_CONSISTENCY"""
        submsg = {
            "rtps_rtps_param_id": ["0x0073"]
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_TYPE_CONSISTENCY"] == "present"
    
    def test_pid_array_multiple(self):
        """PID 배열 - 여러 PID"""
        submsg = {
            "rtps_rtps_param_id": ["0x0073", "0x0060", "0x0043"]
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_TYPE_CONSISTENCY"] == "present"
        assert result["PID_TYPE_MAX_SIZE_SERIALIZED_value"] == "present"
        assert result["PID_EXPECTS_INLINE_QOS_inline_qos"] == "present"
    
    def test_pid_topic_name(self):
        """PID - TOPIC_NAME"""
        submsg = {
            "rtps_rtps_param_id": ["0x0005"],
            "rtps_rtps_param_topic_name": "rt/chatter"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_TOPIC_NAME_topic"] == "rt/chatter"
    
    def test_pid_type_name(self):
        """PID - TYPE_NAME"""
        submsg = {
            "rtps_rtps_param_id": ["0x0007"],
            "rtps_rtps_param_type_name": "std_msgs::msg::dds_::String_"
        }
        result = PIDExtractor.extract_pid_values(submsg)
        
        assert result["PID_TYPE_NAME_typename"] == "std_msgs::msg::dds_::String_"


class TestClassifyDataKind:
    """classify_data_kind() 함수 테스트"""
    
    def test_participant_discovery_with_lease_duration(self):
        """Participant Discovery - LEASE_DURATION 기반"""
        pid_values = {
            "PID_PARTICIPANT_LEASE_DURATION_seconds": "100"
        }
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA(p)"
    
    def test_participant_discovery_with_builtin_endpoint(self):
        """Participant Discovery - BUILTIN_ENDPOINT_SET 기반"""
        pid_values = {
            "PID_BUILTIN_ENDPOINT_SET_flags": "0x00000003"
        }
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA(p)"
    
    def test_writer_discovery(self):
        """Writer Discovery - TYPE_MAX_SIZE_SERIALIZED 기반"""
        pid_values = {
            "PID_TYPE_MAX_SIZE_SERIALIZED_value": "present"
        }
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA(w)"
    
    def test_reader_discovery(self):
        """Reader Discovery - TYPE_CONSISTENCY 기반"""
        pid_values = {
            "PID_TYPE_CONSISTENCY": "present"
        }
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA(r)"
    
    def test_reader_discovery_with_inline_qos(self):
        """Reader Discovery - EXPECTS_INLINE_QOS 기반"""
        pid_values = {
            "PID_EXPECTS_INLINE_QOS_inline_qos": "present"
        }
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA(r)"
    
    def test_key_hash_participant(self):
        """KEY_HASH 기반 - Participant (entityKind=0xc1)"""
        pid_values = {
            "PID_KEY_HASH_guid": "01:0f:50:af:25:af:94:62:01:00:00:00:00:00:01:c1"
        }
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA(p)"
    
    def test_key_hash_writer(self):
        """KEY_HASH 기반 - Writer (entityKind=0xc2)"""
        pid_values = {
            "PID_KEY_HASH_guid": "01:0f:50:af:25:af:94:62:01:00:00:00:00:01:00:c2"
        }
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA(w)"
    
    def test_key_hash_reader(self):
        """KEY_HASH 기반 - Reader (entityKind=0xc7)"""
        pid_values = {
            "PID_KEY_HASH_guid": "01:0f:50:af:25:af:94:62:01:00:00:00:00:01:00:c7"
        }
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA(r)"
    
    def test_unregister_dispose_participant(self):
        """Participant Discovery + Unregister/Dispose"""
        pid_values = {
            "PID_BUILTIN_ENDPOINT_SET_flags": "0x00000003",
            "PID_STATUS_INFO_flags": "DISPOSED"
        }
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA(p[ud])"
    
    def test_unregister_dispose_writer(self):
        """Writer Discovery + Unregister/Dispose"""
        pid_values = {
            "PID_TYPE_MAX_SIZE_SERIALIZED_value": "present",
            "PID_STATUS_INFO_flags": "UNREGISTERED"
        }
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA(w[ud])"
    
    def test_default_data(self):
        """분류 불가능하면 기본 DATA 반환"""
        pid_values = {}
        result = PIDExtractor.classify_data_kind(pid_values)
        
        assert result == "DATA"


# pytest 실행용
if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
