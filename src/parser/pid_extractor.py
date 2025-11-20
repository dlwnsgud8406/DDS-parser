"""
PID (Parameter ID) 추출기

RTPS 패킷의 PID 값을 추출하고 분류하는 모듈
새로운 PID 정의 시스템을 사용하여 53개 PID 자동 추출
"""

import re
from typing import Dict, Any, Optional, List

from .pid_definitions import (
    PID_DEFINITIONS,
    PIDDefinition,
    get_pid_definition,
    PID_BY_ID,
)


class PIDExtractor:
    """RTPS Submessage에서 PID 값을 자동 추출"""
    
    # PID 상수는 하위 호환성을 위해 유지
    PID_PROTOCOL_VERSION = 0x0015
    PID_VENDOR_ID = 0x0016
    PID_PARTICIPANT_GUID = 0x0050
    PID_PARTICIPANT_LEASE_DURATION = 0x0002
    PID_BUILTIN_ENDPOINT_SET = 0x0058
    PID_KEY_HASH = 0x0070
    PID_STATUS_INFO = 0x0071
    PID_TYPE_CONSISTENCY = 0x0073
    PID_TYPE_MAX_SIZE_SERIALIZED = 0x0060
    PID_EXPECTS_INLINE_QOS = 0x0043
    PID_TOPIC_NAME = 0x0005
    PID_TYPE_NAME = 0x0007
    PID_RELIABILITY = 0x001a
    PID_LIVELINESS = 0x001b
    PID_OWNERSHIP = 0x001f
    PID_OWNERSHIP_STRENGTH = 0x0006
    
    @staticmethod
    def to_int(x) -> Optional[int]:
        """
        다양한 형태의 값을 정수로 변환
        
        Args:
            x: 변환할 값 (int, str, list 등)
            
        Returns:
            int | None: 변환된 정수, 실패 시 None
        """
        if x is None:
            return None
        if isinstance(x, int):
            return x
        if isinstance(x, list) and x:
            x = x[0]
        
        s = str(x)
        try:
            return int(s, 16) if s.lower().startswith("0x") else int(s)
        except:
            # 16진수 패턴 찾기
            m = re.search(r"0x[0-9a-fA-F]+", s)
            if m:
                return int(m.group(0), 16)
            # 10진수 패턴 찾기
            m = re.search(r"\d+", s)
            return int(m.group(0)) if m else None
    
    @staticmethod
    def extract_pid_values(submsg: Dict[str, Any]) -> Dict[str, Any]:
        """
        Submessage에서 모든 PID 값을 자동 추출
        
        PID 정의의 tshark_mapping을 사용하여 53개 PID를 자동으로 추출합니다.
        
        Args:
            submsg: RTPS submessage dictionary (tshark ek format)
            
        Returns:
            dict: PID별 추출된 값
                {
                    "PID_PROTOCOL_VERSION_major": "2",
                    "PID_VENDOR_ID_vendorid": "0x0000010f",
                    "PID_PARTICIPANT_GUID_hostid": "0x010f50af",
                    ...
                }
        """
        result = {}
        to_int = PIDExtractor.to_int
        
        # 모든 PID 정의를 순회하면서 자동 추출
        for pid_def in PID_DEFINITIONS:
            if not pid_def.tshark_mapping:
                continue
            
            # 각 필드에 대해 tshark 필드명으로 값 추출
            for field_name, tshark_field in pid_def.tshark_mapping.items():
                value = submsg.get(tshark_field)
                
                if value is not None:
                    # 컬럼 이름 생성
                    if pid_def.fields:
                        col_name = f"{pid_def.pid_name}_{field_name}"
                    else:
                        col_name = pid_def.pid_name
                    
                    # 값 저장 (리스트면 첫 번째 값 사용)
                    if isinstance(value, list):
                        result[col_name] = str(value[0]) if value else ""
                    else:
                        result[col_name] = str(value)
        
        # 특수 처리가 필요한 PID들 (계산된 값)
        result.update(PIDExtractor._extract_computed_pids(submsg))
        
        return result
    
    @staticmethod
    def _extract_computed_pids(submsg: Dict[str, Any]) -> Dict[str, Any]:
        """
        계산이 필요한 PID 값 추출
        
        Args:
            submsg: RTPS submessage dictionary
            
        Returns:
            dict: 계산된 PID 값들
        """
        result = {}
        to_int = PIDExtractor.to_int
        
        # Entity ID 추출 및 계산 (entityid, entitykey, entitykind 분리)
        # Reader Entity ID와 Writer Entity ID 모두 확인
        rd_entity_id = to_int(submsg.get("rtps_rtps_sm_rdEntityId"))
        wr_entity_id = to_int(submsg.get("rtps_rtps_sm_wrEntityId"))
        entity_id = wr_entity_id if wr_entity_id is not None else rd_entity_id
        
        if entity_id is not None:
            result["PID_PARTICIPANT_GUID_entityid"] = f"0x{entity_id:08x}"
            entity_key = (entity_id >> 8) & 0xFFFFFF
            entity_kind = entity_id & 0xFF
            result["PID_PARTICIPANT_GUID_entitykey"] = f"0x{entity_key:06x}"
            result["PID_PARTICIPANT_GUID_entitykind"] = f"0x{entity_kind:02x}"
        
        # Locator 배열 처리 (복잡한 인덱싱 필요)
        kinds = submsg.get("rtps_rtps_locator_kind")
        ports = submsg.get("rtps_rtps_locator_port")
        ipv4s = submsg.get("rtps_rtps_locator_ipv4")
        
        if kinds or ports or ipv4s:
            if isinstance(kinds, list) and len(kinds) >= 2:
                # METATRAFFIC_UNICAST (첫 번째)
                result["PID_METATRAFFIC_UNICAST_LOCATOR_kind"] = str(kinds[0])
                if isinstance(ports, list) and len(ports) >= 1:
                    result["PID_METATRAFFIC_UNICAST_LOCATOR_port"] = str(ports[0])
                if isinstance(ipv4s, list) and len(ipv4s) >= 1:
                    result["PID_METATRAFFIC_UNICAST_LOCATOR_ipv4"] = str(ipv4s[0])
                
                # DEFAULT_UNICAST (세 번째 kind, 두 번째 port/ipv4)
                if len(kinds) >= 3:
                    result["PID_DEFAULT_UNICAST_LOCATOR_kind"] = str(kinds[2])
                if isinstance(ports, list) and len(ports) >= 2:
                    result["PID_DEFAULT_UNICAST_LOCATOR_port"] = str(ports[1])
                if isinstance(ipv4s, list) and len(ipv4s) >= 2:
                    result["PID_DEFAULT_UNICAST_LOCATOR_ipv4"] = str(ipv4s[1])
        
        # Lease Duration 계산 (초 단위 변환)
        sec = to_int(submsg.get("rtps_rtps_param_ntpTime_sec"))
        frac = to_int(submsg.get("rtps_rtps_param_ntpTime_fraction"))
        if sec is not None and frac is not None:
            val = sec + frac / (2**32) if frac else sec
            result["PID_PARTICIPANT_LEASE_DURATION_lease_duration"] = f"{val:.6f}s"
        
        # Vendor ID 포맷팅 (0x 형식)
        vendor_id = to_int(submsg.get("rtps_rtps_vendorId"))
        if vendor_id is not None:
            result["PID_VENDOR_ID_vendorid"] = f"0x{vendor_id:08x}"
        
        # Status Info 플래그 디코딩
        status_info = to_int(submsg.get("rtps_rtps_param_status_info"))
        if status_info is not None:
            flags = []
            if status_info & 0x1:
                flags.append("DISPOSED")
            if status_info & 0x2:
                flags.append("UNREGISTERED")
            result["PID_STATUS_INFO_flags"] = "|".join(flags) if flags else f"0x{status_info:08x}"
        
        # PID 배열 처리 (rtps_rtps_param_id)
        param_ids = submsg.get("rtps_rtps_param_id")
        if param_ids:
            if not isinstance(param_ids, list):
                param_ids = [param_ids]
            
            for pid_hex in param_ids:
                pid = to_int(pid_hex)
                if pid is None:
                    continue
                
                # 특정 PID는 "present" 마커만 추가
                if pid == 0x0073:  # PID_TYPE_CONSISTENCY
                    result["PID_TYPE_CONSISTENCY"] = "present"
                elif pid == 0x0060:  # PID_TYPE_MAX_SIZE_SERIALIZED
                    result["PID_TYPE_MAX_SIZE_SERIALIZED_value"] = "present"
                elif pid == 0x0043:  # PID_EXPECTS_INLINE_QOS
                    result["PID_EXPECTS_INLINE_QOS_inline_qos"] = "present"
        
        return result
    
    @staticmethod
    def classify_data_kind(pid_values: Dict[str, str]) -> str:
        """
        DATA submessage 타입 분류
        
        DATA(p): Participant Discovery
        DATA(w): Writer Discovery
        DATA(r): Reader Discovery
        +[ud]: Unregister/Dispose 플래그
        
        Args:
            pid_values: extract_pid_values()의 결과
            
        Returns:
            str: "DATA(p)", "DATA(w)", "DATA(r)", "DATA(p[ud])" 등
        """
        has_key_hash = bool(pid_values.get("PID_KEY_HASH_guid"))
        has_status_info = bool(pid_values.get("PID_STATUS_INFO_flags"))
        label = "DATA"
        
        # 1) KEY_HASH가 있으면 entity ID로 분류
        if has_key_hash:
            key_hash = pid_values.get("PID_KEY_HASH_guid", "")
            if key_hash:
                parts = key_hash.split(":")
                if len(parts) == 16:
                    entity_kind = int(parts[-1], 16)
                    if entity_kind in (0x01, 0xc1):
                        label = "DATA(p)"
                    elif entity_kind in (0x02, 0x03, 0xc2, 0xc3):
                        label = "DATA(w)"
                    elif entity_kind in (0x04, 0x07, 0xc4, 0xc7):
                        label = "DATA(r)"
        
        # 2) KEY_HASH가 없으면 PID 기반 분류
        else:
            if "PID_TYPE_MAX_SIZE_SERIALIZED_value" in pid_values or "PID_TIME_BASED_FILTER" in pid_values:
                label = "DATA(w)"
            elif "PID_TYPE_CONSISTENCY" in pid_values or "PID_EXPECTS_INLINE_QOS_inline_qos" in pid_values:
                label = "DATA(r)"
            elif "PID_BUILTIN_ENDPOINT_SET_flags" in pid_values or "PID_PARTICIPANT_LEASE_DURATION_seconds" in pid_values:
                # Participant discovery는 BUILTIN_ENDPOINT_SET 또는 LEASE_DURATION만 있어도 분류
                label = "DATA(p)"
        
        # 3) STATUS_INFO가 있으면 [ud] 추가
        if has_status_info:
            if label.startswith("DATA(") and label.endswith(")"):
                label = label[:-1] + "[ud])"
            else:
                label += "[ud]"
        
        return label
