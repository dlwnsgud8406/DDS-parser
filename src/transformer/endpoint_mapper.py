"""
Endpoint to Topic Mapper

SEDP (Simple Endpoint Discovery Protocol) 패킷에서
Endpoint GUID → Topic Name 매핑 테이블 생성
"""

from typing import List, Dict, Tuple, Any, Optional
from collections import defaultdict


class EndpointMapper:
    """
    GUID ↔ Topic 매핑 관리 (Stateful Protocol Analyzer)
    
    RTPS는 상태 기계(stateful) 프로토콜입니다:
    
    1단계: SEDP Discovery
       - DATA(w), DATA(r)에서 "GUID → Topic" 매핑 테이블 구축
       - 이게 전체 분석의 "정답 사전(lookup table)"이 됨
    
    2단계: User Traffic Join
       - HEARTBEAT, ACKNACK, DATA는 topic_name을 포함하지 않음
       - 대신 GUID만 가지고 다님 (이미 discovery 끝났다는 전제)
       - 1단계 테이블에 GUID로 join하여 topic 매핑
    
    3단계: Unknown 처리
       - 매핑 테이블에 없으면 → builtin/control 트래픽이거나 캡처 누락
       - 별도 분류하여 추적
    """
    
    # DDS Builtin Endpoint EntityId 패턴 (well-known)
    # DDS 스펙에서 정의된 예약된 EntityId
    BUILTIN_ENTITY_PATTERNS = {
        0x000000c2: 'SPDP_BUILTIN_PARTICIPANT_WRITER',
        0x000000c7: 'SPDP_BUILTIN_PARTICIPANT_READER',
        0x000002c2: 'SEDP_BUILTIN_PUBLICATIONS_WRITER',
        0x000002c7: 'SEDP_BUILTIN_PUBLICATIONS_READER',
        0x000003c2: 'SEDP_BUILTIN_SUBSCRIPTIONS_WRITER',
        0x000003c7: 'SEDP_BUILTIN_SUBSCRIPTIONS_READER',
        0x000100c2: 'BUILTIN_PARTICIPANT_MESSAGE_WRITER',
        0x000100c7: 'BUILTIN_PARTICIPANT_MESSAGE_READER',
        0x000200c2: 'BUILTIN_PARTICIPANT_STATELESS_WRITER',
        0x000200c7: 'BUILTIN_PARTICIPANT_STATELESS_READER',
        # Secure builtin endpoints
        0x000001c2: 'BUILTIN_PUBLICATIONS_SECURE_WRITER',
        0x000001c7: 'BUILTIN_PUBLICATIONS_SECURE_READER',
    }
    
    def __init__(self):
        # User Endpoint 매핑: (hostId, appId, instanceId, entityId) → topic_info
        # SEDP에서 발견된 user topic endpoint만 저장
        self.endpoint_map: Dict[Tuple[int, int, int, int], Dict[str, Any]] = {}
        
        # 통계: 매핑 실패 원인 추적
        self.unmapped_stats = {
            'builtin': 0,      # Builtin endpoint (well-known entityId)
            'unknown': 0,      # SEDP에 없는 user endpoint
            'no_guid': 0,      # GUID 정보 불완전
        }
    
    def build_mapping(self, submessages: List[Dict[str, Any]]) -> None:
        """
        SEDP 패킷에서 endpoint → topic 매핑 테이블 생성
        
        Args:
            submessages: 파싱된 submessage 리스트
        """
        # Use tqdm when available to show progress for large captures
        try:
            from tqdm import tqdm
            iterator = tqdm(submessages, desc="Building SEDP mapping", unit="msg")
        except Exception:
            iterator = submessages

        for msg in iterator:
            submsg_name = msg.get('submsg_name', '')
            
            # SEDP 패킷만 처리: DATA(w), DATA(r), DATA(w[ud]), DATA(r[ud])
            if not ('DATA(w' in submsg_name or 'DATA(r' in submsg_name):
                continue
            
            pids = msg.get('pids', {})
            topic = pids.get('PID_TOPIC_NAME_topic')
            
            # Topic이 없으면 스킵
            if not topic:
                continue
            
            # SEDP 패킷에도 topic 필드 추가 (토픽 시트에 포함되도록)
            msg['topic'] = topic
            msg['endpoint_kind'] = 'sedp_discovery'
            
            guid = msg.get('guid', {})
            entity = msg.get('entity', {})
            
            # CRITICAL: SEDP 패킷의 endpoint GUID는 PID_KEY_HASH에 있음!
            # PID_PARTICIPANT_GUID는 participant GUID (참여자)
            # PID_KEY_HASH는 endpoint GUID (writer/reader) ← 우리가 찾는 것!
            
            key_hash = pids.get('PID_KEY_HASH_guid')
            
            if key_hash:
                # Parse: "01:0f:50:af:db:35:2d:6b:01:00:00:00:00:00:01:03"
                parts = key_hash.split(':')
                if len(parts) == 16:
                    try:
                        hostId = int(''.join(parts[0:4]), 16)
                        appId = int(''.join(parts[4:8]), 16)
                        instanceId = int(''.join(parts[8:12]), 16)
                        entityId = int(''.join(parts[12:16]), 16)
                        
                        # DATA(w)면 writer, DATA(r)면 reader
                        if 'DATA(w' in submsg_name:
                            wrEntityId = entityId
                            rdEntityId = None
                        elif 'DATA(r' in submsg_name:
                            wrEntityId = None
                            rdEntityId = entityId
                        else:
                            continue
                    except (ValueError, IndexError):
                        continue
                else:
                    continue
            else:
                continue
            
            # GUID가 완전하지 않으면 스킵
            if hostId is None or appId is None or instanceId is None:
                continue
            
            # Writer endpoint 처리
            if 'DATA(w' in submsg_name and wrEntityId is not None:
                endpoint_guid = (hostId, appId, instanceId, wrEntityId)
                self.endpoint_map[endpoint_guid] = {
                    'topic': topic,
                    'type': pids.get('PID_TYPE_NAME_typename'),
                    'kind': 'writer',
                    'reliability': pids.get('PID_RELIABILITY_kind'),
                    'durability': pids.get('PID_DURABILITY_kind'),
                    'ownership': pids.get('PID_OWNERSHIP_kind'),
                }
            
            # Reader endpoint 처리
            elif 'DATA(r' in submsg_name and rdEntityId is not None:
                endpoint_guid = (hostId, appId, instanceId, rdEntityId)
                self.endpoint_map[endpoint_guid] = {
                    'topic': topic,
                    'type': pids.get('PID_TYPE_NAME_typename'),
                    'kind': 'reader',
                    'reliability': pids.get('PID_RELIABILITY_kind'),
                    'durability': pids.get('PID_DURABILITY_kind'),
                    'ownership': pids.get('PID_OWNERSHIP_kind'),
                }
    
    def get_topic_for_writer(
        self, 
        hostId: int, 
        appId: int, 
        instanceId: int, 
        wrEntityId: int
    ) -> Optional[str]:
        """
        Writer GUID로 Topic 조회
        
        Args:
            hostId, appId, instanceId, wrEntityId: Writer GUID 구성 요소
            
        Returns:
            str | None: Topic name
        """
        endpoint_guid = (hostId, appId, instanceId, wrEntityId)
        endpoint_info = self.endpoint_map.get(endpoint_guid)
        return endpoint_info['topic'] if endpoint_info else None
    
    def get_topic_for_reader(
        self,
        hostId: int,
        appId: int,
        instanceId: int,
        rdEntityId: int
    ) -> Optional[str]:
        """
        Reader GUID로 Topic 조회
        
        Args:
            hostId, appId, instanceId, rdEntityId: Reader GUID 구성 요소
            
        Returns:
            str | None: Topic name
        """
        endpoint_guid = (hostId, appId, instanceId, rdEntityId)
        endpoint_info = self.endpoint_map.get(endpoint_guid)
        return endpoint_info['topic'] if endpoint_info else None
    
    def get_endpoint_info(
        self,
        hostId: int,
        appId: int,
        instanceId: int,
        entityId: int
    ) -> Optional[Dict[str, Any]]:
        """
        전체 endpoint 정보 조회
        
        Returns:
            dict | None: {'topic': str, 'type': str, 'kind': str, 'qos': ...}
        """
        endpoint_guid = (hostId, appId, instanceId, entityId)
        return self.endpoint_map.get(endpoint_guid)
    
    def enrich_submessages(self, submessages: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        모든 submessage에 topic 정보 추가
        
        HEARTBEAT, ACKNACK, DATA(user) 등에 topic 필드를 추가합니다.
        
        Args:
            submessages: 파싱된 submessage 리스트
            
        Returns:
            list[dict]: topic 정보가 추가된 submessage 리스트 (원본 수정)
        """
        # Show progress if tqdm is available
        try:
            from tqdm import tqdm
            iterator = tqdm(submessages, desc="Enriching submessages", unit="msg")
        except Exception:
            iterator = submessages

        # 패킷별 컨텍스트: 같은 패킷 내 submessage는 GUID Prefix 공유
        current_frame = None
        current_guid_prefix = (None, None, None)
        
        for msg in iterator:
            # 이미 topic이 있으면 스킵 (SEDP 패킷은 이미 가지고 있음)
            if 'topic' in msg:
                continue
            
            frame_number = msg.get('frame_number')
            guid = msg.get('guid', {})
            entity = msg.get('entity', {})
            pids = msg.get('pids', {})
            
            # Non-SEDP submessage가 패킷 컨텍스트에서 PID_TOPIC_NAME을 물려받은 경우
            # 이 토픽 정보를 사용 (예: INFO_TS, HEARTBEAT 등이 같은 패킷의 SEDP와 함께 있을 때)
            pid_topic_name = pids.get('PID_TOPIC_NAME_topic')
            if pid_topic_name:
                # PID_TOPIC_NAME이 있으면 해당 토픽으로 분류
                msg['topic'] = pid_topic_name
                msg['endpoint_kind'] = 'context_inherited'
                continue
            
            # 패킷이 바뀌면 컨텍스트 리셋
            if frame_number != current_frame:
                current_frame = frame_number
                # 새 패킷의 GUID Prefix 저장 (첫 submessage에서)
                h = guid.get('hostId')
                a = guid.get('appId')
                i = guid.get('instanceId')
                if h is not None and a is not None and i is not None:
                    current_guid_prefix = (h, a, i)
            
            # GUID 추출: submessage 레벨에서 먼저, 없으면 PIDs에서, 없으면 패킷 컨텍스트에서
            hostId = guid.get('hostId')
            appId = guid.get('appId')
            instanceId = guid.get('instanceId')
            
            if hostId is None:
                hostId_str = pids.get('PID_PARTICIPANT_GUID_hostid')
                hostId = int(hostId_str, 16) if hostId_str and isinstance(hostId_str, str) else None
            if appId is None:
                appId_str = pids.get('PID_PARTICIPANT_GUID_appid')
                appId = int(appId_str, 16) if appId_str and isinstance(appId_str, str) else None
            if instanceId is None:
                instanceId_str = pids.get('PID_PARTICIPANT_GUID_instanceid')
                instanceId = int(instanceId_str, 16) if instanceId_str and isinstance(instanceId_str, str) else None
            
            # 여전히 없으면 패킷 컨텍스트 사용
            if hostId is None:
                hostId = current_guid_prefix[0]
            if appId is None:
                appId = current_guid_prefix[1]
            if instanceId is None:
                instanceId = current_guid_prefix[2]
            
            # Entity ID 추출: submessage 레벨에서 먼저, 없으면 PIDs에서
            wrEntityId = entity.get('wrEntityId')
            rdEntityId = entity.get('rdEntityId')
            
            if wrEntityId is None and rdEntityId is None:
                entityId_str = pids.get('PID_PARTICIPANT_GUID_entityid')
                if entityId_str:
                    entityId = int(entityId_str, 16) if isinstance(entityId_str, str) else entityId_str
                    # HEARTBEAT/DATA는 writer, ACKNACK는 reader로 추정
                    submsg_name = msg.get('submsg_name', '')
                    if 'HEARTBEAT' in submsg_name or submsg_name.startswith('DATA'):
                        wrEntityId = entityId
                    elif 'ACKNACK' in submsg_name:
                        rdEntityId = entityId
                    else:
                        # 기타 메시지는 둘 다 시도
                        wrEntityId = entityId
            
            # GUID가 완전하지 않으면 스킵
            if hostId is None or appId is None or instanceId is None:
                self.unmapped_stats['no_guid'] += 1
                continue
            
            # Entity ID 결정
            entityId = wrEntityId or rdEntityId
            if entityId is None:
                continue
            
            # Builtin endpoint 체크 (well-known EntityId)
            if entityId in self.BUILTIN_ENTITY_PATTERNS:
                msg['topic'] = f"__builtin__{self.BUILTIN_ENTITY_PATTERNS[entityId]}"
                msg['endpoint_kind'] = 'builtin'
                msg['builtin_type'] = self.BUILTIN_ENTITY_PATTERNS[entityId]
                continue
            
            # User Endpoint 매핑 시도
            mapped = False
            
            # Writer 기준 매핑 (HEARTBEAT, DATA(user), GAP 등)
            if wrEntityId is not None:
                endpoint_info = self.get_endpoint_info(hostId, appId, instanceId, wrEntityId)
                if endpoint_info:
                    msg['topic'] = endpoint_info['topic']
                    msg['endpoint_kind'] = 'writer'
                    msg['endpoint_type'] = endpoint_info.get('type')
                    mapped = True
            
            # Reader 기준 매핑 (ACKNACK 등)
            if not mapped and rdEntityId is not None:
                endpoint_info = self.get_endpoint_info(hostId, appId, instanceId, rdEntityId)
                if endpoint_info:
                    msg['topic'] = endpoint_info['topic']
                    msg['endpoint_kind'] = 'reader'
                    msg['endpoint_type'] = endpoint_info.get('type')
                    mapped = True
            
            # 매핑 실패: SEDP에 없는 endpoint (캡처 누락 or discovery 이전)
            if not mapped:
                self.unmapped_stats['unknown'] += 1
                msg['topic'] = '__unknown__'
                msg['endpoint_kind'] = 'unknown'
                # 디버깅용 GUID 정보 저장
                msg['unmapped_guid'] = {
                    'hostId': f"0x{hostId:08x}",
                    'appId': f"0x{appId:08x}",
                    'instanceId': f"0x{instanceId:08x}",
                    'entityId': f"0x{entityId:08x}"
                }
        
        return submessages
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        매핑 통계 정보 반환 (상태 머신 추적 결과)
        
        Returns:
            dict: {
                'total_endpoints': int,     # SEDP에서 발견된 user endpoint 수
                'writers': int,
                'readers': int,
                'topics': set[str],
                'endpoints_per_topic': dict[str, int],
                'unmapped_stats': {         # 매핑 실패 원인별 통계
                    'builtin': int,         # Builtin/control 트래픽
                    'unknown': int,         # SEDP 누락 user endpoint
                    'no_guid': int          # GUID 정보 불완전
                }
            }
        """
        writers = sum(1 for info in self.endpoint_map.values() if info['kind'] == 'writer')
        readers = sum(1 for info in self.endpoint_map.values() if info['kind'] == 'reader')
        
        topics = set()
        endpoints_per_topic = defaultdict(int)
        
        for info in self.endpoint_map.values():
            topic = info['topic']
            topics.add(topic)
            endpoints_per_topic[topic] += 1
        
        return {
            'total_endpoints': len(self.endpoint_map),
            'writers': writers,
            'readers': readers,
            'topics': topics,
            'topics_count': len(topics),
            'endpoints_per_topic': dict(endpoints_per_topic),
            'unmapped_stats': self.unmapped_stats.copy()
        }
    
    def get_sedp_dataframe_data(self) -> List[Dict[str, Any]]:
        """
        SEDP 시트용 DataFrame 데이터 생성
        
        Returns:
            list[dict]: SEDP endpoint 정보 리스트
                [
                    {
                        'endpoint_kind': 'writer',
                        'hostId': '0x...',
                        'appId': '0x...',
                        'instanceId': '0x...',
                        'entityId': '0x...',
                        'topic': '/rt/chatter',
                        'type': 'std_msgs::msg::dds_::String_',
                        'reliability': 'RELIABLE',
                        'durability': 'VOLATILE',
                        ...
                    },
                    ...
                ]
        """
        rows = []
        
        for endpoint_guid, info in self.endpoint_map.items():
            hostId, appId, instanceId, entityId = endpoint_guid
            
            row = {
                'endpoint_kind': info['kind'],
                'hostId': f"0x{hostId:08x}" if hostId is not None else None,
                'appId': f"0x{appId:08x}" if appId is not None else None,
                'instanceId': f"0x{instanceId:08x}" if instanceId is not None else None,
                'entityId': f"0x{entityId:08x}" if entityId is not None else None,
                'topic': info['topic'],
                'type': info.get('type'),
                'reliability': info.get('reliability'),
                'durability': info.get('durability'),
                'ownership': info.get('ownership'),
            }
            rows.append(row)
        
        # Topic 이름과 kind로 정렬
        rows.sort(key=lambda x: (x['topic'], x['endpoint_kind']))
        
        return rows
