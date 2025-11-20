"""
Enhanced RTPS Parser

RTPS 패킷을 파싱하고 PID 값을 추출하는 향상된 파서
reference/300_DDS_parser.py를 기반으로 작성됨
"""

from typing import Dict, List, Any, Optional
from datetime import datetime
from .pid_extractor import PIDExtractor


class EnhancedRTPSParser:
    """
    RTPS 패킷을 파싱하고 PID 정보를 포함한 상세 정보 추출
    
    기존 SimpleParser와 달리 PID 값까지 추출하여 Excel 출력에 필요한
    모든 정보를 제공합니다.
    """
    
    # Submessage ID to Name 매핑
    SUBMSG_ID_TO_NAME = {
        0x01: "PAD",
        0x02: "DATA",
        0x03: "NOKEY_DATA",
        0x06: "ACKNACK",
        0x07: "HEARTBEAT",
        0x08: "GAP",
        0x09: "INFO_TS",
        0x0C: "INFO_SRC",
        0x0D: "INFO_REPLY_IP4",
        0x0E: "INFO_DST",
        0x0F: "INFO_REPLY",
        0x12: "NACK_FRAG",
        0x15: "DATA",
    }
    
    def __init__(self):
        self.pid_extractor = PIDExtractor()
        self.to_int = PIDExtractor.to_int
    
    def parse(self, packet: Dict[str, Any]) -> Optional[List[Dict[str, Any]]]:
        """
        단일 패킷을 파싱하여 submessage 리스트 반환
        
        Args:
            packet: tshark -T ek JSON 형식의 패킷
            
        Returns:
            list[dict] | None: 파싱된 submessage 리스트
                각 submessage는 다음 정보를 포함:
                {
                    'frame_number': int,
                    'timestamp': str,  # ISO format
                    'submsg_index': int,  # 1-based
                    'submsg_id': int,
                    'submsg_name': str,  # "DATA(p)", "HEARTBEAT" 등
                    'guid': {
                        'hostId': int,
                        'appId': int,
                        'instanceId': int
                    },
                    'entity': {
                        'rdEntityId': int | None,
                        'wrEntityId': int | None
                    },
                    'pids': {
                        'PID_PROTOCOL_VERSION_major': '2',
                        'PID_VENDOR_ID_vendorid': '0x0000010f',
                        ...
                    }
                }
        """
        layers = packet.get('layers', {})
        rtps_layer = layers.get('rtps', {})
        
        if not rtps_layer:
            return None
        
        # Frame 정보
        frame_layer = layers.get('frame', {})
        frame_number = frame_layer.get('frame_frame_number')
        timestamp = frame_layer.get('frame_frame_time')
        
        # Submessage 분리
        submessages = self._extract_submessages(rtps_layer)
        
        results = []
        for idx, submsg in enumerate(submessages, start=1):
            parsed = self._parse_submessage(
                submsg,
                frame_number=frame_number,
                timestamp=timestamp,
                submsg_index=idx
            )
            if parsed:
                results.append(parsed)
        
        return results if results else None
    
    def _extract_submessages(self, rtps_layer: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        RTPS 레이어에서 submessage들을 분리
        
        tshark는 하나의 RTPS 패킷에 여러 submessage가 있으면
        리스트 형태로 필드를 반환합니다. 이를 개별 submessage로 분리합니다.
        
        reference/300_DDS_parser.py의 extract_submessages() 참고
        """
        sm_id_list = rtps_layer.get("rtps_rtps_sm_id")
        if not sm_id_list:
            return []
        
        if not isinstance(sm_id_list, list):
            sm_id_list = [sm_id_list]
        
        N = len(sm_id_list)
        submessages = []
        
        for i in range(N):
            sub = {}
            for k, v in rtps_layer.items():
                if isinstance(v, list) and len(v) == N:
                    # submessage 개수와 일치하는 리스트 -> i번째 값 사용
                    sub[k] = v[i]
                elif isinstance(v, list) and len(v) > 0:
                    # 리스트지만 길이가 다르면 범위 내 값 사용
                    sub[k] = v[min(i, len(v) - 1)]
                else:
                    # 단일 값이면 모든 submessage에 공통
                    sub[k] = v
            submessages.append(sub)
        
        return submessages
    
    def _parse_submessage(
        self,
        submsg: Dict[str, Any],
        frame_number: Optional[str],
        timestamp: Optional[str],
        submsg_index: int
    ) -> Optional[Dict[str, Any]]:
        """
        개별 submessage를 파싱하여 구조화된 딕셔너리 반환
        """
        # Submessage ID
        sm_id = self.to_int(submsg.get("rtps_rtps_sm_id"))
        if sm_id is None:
            return None
        
        # PID 값 추출
        pid_values = self.pid_extractor.extract_pid_values(submsg)
        
        # Submessage Name (DATA는 더 상세하게 분류)
        base_name = self.SUBMSG_ID_TO_NAME.get(sm_id, f"UNKNOWN_0x{sm_id:02x}")
        if sm_id in (0x02, 0x15):  # DATA submessage
            submsg_name = self.pid_extractor.classify_data_kind(pid_values)
        else:
            submsg_name = base_name
        
        # GUID (Participant ID)
        hostId = self.to_int(submsg.get('rtps_rtps_hostId'))
        appId = self.to_int(submsg.get('rtps_rtps_appId'))
        instanceId = self.to_int(submsg.get('rtps_rtps_sm_guidPrefix_instanceId'))
        
        # Entity ID
        rd_entity_id = self.to_int(submsg.get("rtps_rtps_sm_rdEntityId"))
        wr_entity_id = self.to_int(submsg.get("rtps_rtps_sm_wrEntityId"))
        
        result = {
            'frame_number': int(frame_number) if frame_number else None,
            'timestamp': timestamp,
            'submsg_index': submsg_index,
            'submsg_id': sm_id,
            'submsg_name': submsg_name,
            'guid': {
                'hostId': hostId,
                'appId': appId,
                'instanceId': instanceId
            },
            'entity': {
                'rdEntityId': rd_entity_id,
                'wrEntityId': wr_entity_id
            },
            'pids': pid_values
        }
        
        return result
    
    def parse_batch(self, packets: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        여러 패킷을 일괄 파싱
        
        Args:
            packets: 패킷 리스트
            
        Returns:
            list[dict]: 파싱된 submessage 리스트 (평탄화됨)
        """
        results = []
        for packet in packets:
            parsed = self.parse(packet)
            if parsed:
                results.extend(parsed)
        return results
