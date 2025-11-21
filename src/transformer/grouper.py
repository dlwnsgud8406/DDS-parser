"""
Participant Grouper

파싱된 RTPS submessage를 Participant(노드)별로 그룹화
"""

from typing import List, Dict, Tuple, Any, Set
from collections import defaultdict
from .node_extractor import NodeNameExtractor


class ParticipantGrouper:
    """
    Participant(노드)별로 submessage 그룹화
    
    Excel 출력 시 각 노드별로 시트를 만들기 위해
    submessage들을 Participant GUID 기준으로 그룹화합니다.
    """
    
    def __init__(self):
        pass
    
    def group_by_participant(
        self, 
        submessages: List[Dict[str, Any]]
    ) -> Dict[Tuple[int, int, int], List[Dict[str, Any]]]:
        """
        Participant별로 submessage 그룹화
        
        Args:
            submessages: EnhancedRTPSParser.parse_batch()의 결과
                각 submessage는 'guid' 필드를 포함해야 함
        
        Returns:
            dict: {
                (hostId, appId, instanceId): [submsg1, submsg2, ...],
                ...
            }
            
        Example:
            >>> grouper = ParticipantGrouper()
            >>> submessages = parser.parse_batch(packets)
            >>> grouped = grouper.group_by_participant(submessages)
            >>> for participant_id, msgs in grouped.items():
            ...     print(f"Node {participant_id}: {len(msgs)} messages")
        """
        grouped = defaultdict(list)
        
        for submsg in submessages:
            # 두 가지 형식 모두 지원
            # 1. Nested: submsg['guid']['hostId']
            # 2. Flattened: submsg['hostId']
            guid = submsg.get('guid', {})
            if guid:
                # Nested 형식
                hostId = guid.get('hostId')
                appId = guid.get('appId')
                instanceId = guid.get('instanceId')
            else:
                # Flattened 형식
                hostId = submsg.get('hostId')
                appId = submsg.get('appId')
                instanceId = submsg.get('instanceId')
            
            # GUID가 모두 있는 경우만 그룹화
            if all([hostId is not None, appId is not None, instanceId is not None]):
                participant_key = (hostId, appId, instanceId)
                grouped[participant_key].append(submsg)
        
        return dict(grouped)
    
    def get_participant_summary(
        self,
        grouped_data: Dict[Tuple[int, int, int], List[Dict[str, Any]]]
    ) -> List[Dict[str, Any]]:
        """
        각 Participant의 요약 정보 생성
        
        Args:
            grouped_data: group_by_participant()의 결과
            
        Returns:
            list[dict]: [
                {
                    'participant_id': (hostId, appId, instanceId),
                    'hostId': int,
                    'appId': int,
                    'instanceId': int,
                    'message_count': int,
                    'entities': set[int],  # Reader/Writer Entity IDs
                    'submsg_types': dict[str, int]  # Type별 개수
                },
                ...
            ]
        """
        summary = []
        
        for participant_id, messages in grouped_data.items():
            hostId, appId, instanceId = participant_id
            
            # Entity ID 수집
            entities = set()
            for msg in messages:
                entity = msg.get('entity', {})
                if entity.get('rdEntityId'):
                    entities.add(entity['rdEntityId'])
                if entity.get('wrEntityId'):
                    entities.add(entity['wrEntityId'])
            
            # Submessage Type 통계
            submsg_types = defaultdict(int)
            for msg in messages:
                submsg_name = msg.get('submsg_name', 'UNKNOWN')
                submsg_types[submsg_name] += 1
            
            summary.append({
                'participant_id': participant_id,
                'hostId': hostId,
                'appId': appId,
                'instanceId': instanceId,
                'message_count': len(messages),
                'entities': entities,
                'submsg_types': dict(submsg_types)
            })
        
        # message_count 기준 정렬 (내림차순)
        summary.sort(key=lambda x: x['message_count'], reverse=True)
        
        return summary
    
    def format_participant_name(
        self,
        participant_id: Tuple[int, int, int],
        index: int = None
    ) -> str:
        """
        Participant ID를 읽기 쉬운 이름으로 변환
        
        Args:
            participant_id: (hostId, appId, instanceId)
            index: 선택적 인덱스 (1-based)
            
        Returns:
            str: "Node_1_010fba3f_3c705e6a" 형식
        """
        hostId, appId, instanceId = participant_id
        
        if index is not None:
            return f"Node_{index}_{hostId:08x}_{appId:08x}"
        else:
            return f"Node_{hostId:08x}_{appId:08x}_{instanceId:08x}"


class NodeGrouper:
    """
    ROS2 노드별로 submessage 그룹화
    
    토픽 이름에서 노드 이름을 추출하여 노드별로 메시지를 그룹화합니다.
    Excel 출력 시 각 노드별로 시트를 만듭니다.
    """
    
    def __init__(self):
        self.extractor = NodeNameExtractor()
    
    def group_by_node(
        self, 
        submessages: List[Dict[str, Any]]
    ) -> Dict[str, List[Dict[str, Any]]]:
        """
        ROS2 노드별로 submessage 그룹화
        
        Args:
            submessages: topic 필드가 있는 submessage 리스트
                각 submessage는 'topic' 필드를 포함해야 함
        
        Returns:
            dict: {
                'stella_md_node': [submsg1, submsg2, ...],
                'launch_ros_*': [submsg1, submsg2, ...],
                '__unknown__': [submsg1, submsg2, ...],  # 노드 미분류
                ...
            }
            
        Example:
            >>> grouper = NodeGrouper()
            >>> enriched = endpoint_mapper.enrich_submessages(submessages)
            >>> grouped = grouper.group_by_node(enriched)
            >>> for node_name, msgs in grouped.items():
            ...     print(f"Node {node_name}: {len(msgs)} messages")
        """
        grouped = defaultdict(list)
        
        for submsg in submessages:
            topic = submsg.get('topic')
            
            if not topic:
                # topic 없으면 __unknown__
                grouped['__unknown__'].append(submsg)
                continue
            
            # 토픽에서 노드 이름 추출
            node_name = self.extractor.extract_node_from_topic(topic)
            
            if node_name:
                grouped[node_name].append(submsg)
            else:
                # 노드 추출 실패 (builtin topics 등)
                grouped['__unknown__'].append(submsg)
        
        return dict(grouped)
    
    def get_node_summary(
        self,
        grouped_data: Dict[str, List[Dict[str, Any]]]
    ) -> List[Dict[str, Any]]:
        """
        각 노드의 요약 정보 생성
        
        Args:
            grouped_data: group_by_node()의 결과
            
        Returns:
            list[dict]: [
                {
                    'node_name': 'stella_md_node',
                    'message_count': 1234,
                    'topics': {'rt/odom', 'rq/stella_md_node/get_parametersRequest', ...},
                    'submsg_types': {'DATA': 100, 'HEARTBEAT': 50, ...}
                },
                ...
            ]
        """
        summary = []
        
        for node_name, messages in grouped_data.items():
            # 토픽 수집
            topics = set()
            for msg in messages:
                topic = msg.get('topic')
                if topic:
                    topics.add(topic)
            
            # Submessage Type 통계
            submsg_types = defaultdict(int)
            for msg in messages:
                submsg_name = msg.get('submsg_name', 'UNKNOWN')
                submsg_types[submsg_name] += 1
            
            summary.append({
                'node_name': node_name,
                'message_count': len(messages),
                'topics': topics,
                'topic_count': len(topics),
                'submsg_types': dict(submsg_types)
            })
        
        # message_count 기준 정렬 (내림차순)
        summary.sort(key=lambda x: x['message_count'], reverse=True)
        
        return summary
    
    def format_node_name_for_sheet(self, node_name: str) -> str:
        """
        노드 이름을 Excel 시트 이름으로 변환
        
        Args:
            node_name: 노드 이름 (예: "stella_md_node", "launch_ros_*")
            
        Returns:
            str: Excel 시트 이름 (31자 제한, 특수문자 제거)
        """
        if node_name == '__unknown__':
            return 'Unknown'
        
        # / 제거
        sheet_name = node_name.lstrip('/')
        
        # 특수문자 제거
        sheet_name = sheet_name.replace('/', '_').replace('\\', '_')
        sheet_name = sheet_name.replace('?', '_').replace('*', 'x')
        sheet_name = sheet_name.replace('[', '_').replace(']', '_')
        sheet_name = sheet_name.replace(':', '_')
        sheet_name = sheet_name.strip('_')
        
        # 31자 제한
        if len(sheet_name) > 31:
            # 끝부분이 더 의미있으므로 앞부분 자르기
            excess = len(sheet_name) - 31 + 3  # "..." 추가
            sheet_name = '...' + sheet_name[excess:]
        
        return sheet_name


class TopicGrouper:
    """
    Topic별로 submessage 그룹화
    
    Excel 출력 시 각 Topic별로 시트를 만들기 위해
    submessage들을 Topic 기준으로 그룹화합니다.
    """
    
    def __init__(self):
        pass
    
    def group_by_topic(
        self,
        submessages: List[Dict[str, Any]]
    ) -> Dict[str, List[Dict[str, Any]]]:
        """
        Topic별로 submessage 그룹화
        
        Args:
            submessages: EndpointMapper.enrich_submessages()로
                        topic 정보가 추가된 submessage 리스트
        
        Returns:
            dict: {
                'topic_name': [submsg1, submsg2, ...],
                ...
            }
            
        Example:
            >>> grouper = TopicGrouper()
            >>> enriched_msgs = endpoint_mapper.enrich_submessages(submessages)
            >>> grouped = grouper.group_by_topic(enriched_msgs)
            >>> for topic, msgs in grouped.items():
            ...     print(f"Topic {topic}: {len(msgs)} messages")
        """
        grouped = defaultdict(list)

        # Progress bar for large lists
        try:
            from tqdm import tqdm
            iterator = tqdm(submessages, desc="Grouping by topic", unit="msg")
        except Exception:
            iterator = submessages

        for submsg in iterator:
            topic = submsg.get('topic')

            # Topic이 있는 submessage만 그룹화
            if topic:
                grouped[topic].append(submsg)
        
        return dict(grouped)
    
    def separate_by_message_type(
        self,
        submessages: List[Dict[str, Any]]
    ) -> Dict[str, List[Dict[str, Any]]]:
        """
        메시지 타입별로 분리
        
        SPDP (Participant Discovery), SEDP (Endpoint Discovery),
        User Traffic을 분리합니다.
        
        Args:
            submessages: 파싱된 submessage 리스트
            
        Returns:
            dict: {
                'spdp': [DATA(p) messages],
                'sedp_writers': [DATA(w) messages],
                'sedp_readers': [DATA(r) messages],
                'user_traffic': [HEARTBEAT, ACKNACK, DATA(user), etc.]
            }
        """
        spdp = []
        sedp_writers = []
        sedp_readers = []
        user_traffic = []
        
        for msg in submessages:
            submsg_name = msg.get('submsg_name', '')
            
            if submsg_name == 'DATA(p)' or submsg_name == 'DATA(p[ud])':
                spdp.append(msg)
            elif submsg_name == 'DATA(w)' or submsg_name == 'DATA(w[ud])':
                sedp_writers.append(msg)
            elif submsg_name == 'DATA(r)' or submsg_name == 'DATA(r[ud])':
                sedp_readers.append(msg)
            else:
                # HEARTBEAT, ACKNACK, GAP, INFO_TS, DATA(user) 등
                user_traffic.append(msg)
        
        return {
            'spdp': spdp,
            'sedp_writers': sedp_writers,
            'sedp_readers': sedp_readers,
            'user_traffic': user_traffic
        }
    
    def get_topic_summary(
        self,
        grouped_data: Dict[str, List[Dict[str, Any]]]
    ) -> List[Dict[str, Any]]:
        """
        각 Topic의 요약 정보 생성
        
        Args:
            grouped_data: group_by_topic()의 결과
            
        Returns:
            list[dict]: [
                {
                    'topic': str,
                    'message_count': int,
                    'participants': set[tuple],  # (hostId, appId, instanceId)
                    'writers': int,
                    'readers': int,
                    'submsg_types': dict[str, int]
                },
                ...
            ]
        """
        summary = []
        
        try:
            from tqdm import tqdm
            iterator = tqdm(list(grouped_data.items()), desc="Summarizing topics", unit="topic")
        except Exception:
            iterator = grouped_data.items()

        for topic, messages in iterator:
            # Participant 수집
            participants = set()
            writers = set()
            readers = set()
            
            for msg in messages:
                guid = msg.get('guid', {})
                entity = msg.get('entity', {})
                
                hostId = guid.get('hostId')
                appId = guid.get('appId')
                instanceId = guid.get('instanceId')
                
                if all([hostId is not None, appId is not None, instanceId is not None]):
                    participants.add((hostId, appId, instanceId))
                
                # Writer/Reader endpoint 카운트
                endpoint_kind = msg.get('endpoint_kind')
                if endpoint_kind == 'writer' and entity.get('wrEntityId'):
                    writers.add((hostId, appId, instanceId, entity['wrEntityId']))
                elif endpoint_kind == 'reader' and entity.get('rdEntityId'):
                    readers.add((hostId, appId, instanceId, entity['rdEntityId']))
            
            # Submessage Type 통계
            submsg_types = defaultdict(int)
            for msg in messages:
                submsg_name = msg.get('submsg_name', 'UNKNOWN')
                submsg_types[submsg_name] += 1
            
            summary.append({
                'topic': topic,
                'message_count': len(messages),
                'participants': participants,
                'participant_count': len(participants),
                'writer_count': len(writers),
                'reader_count': len(readers),
                'submsg_types': dict(submsg_types)
            })
        
        # message_count 기준 정렬 (내림차순)
        summary.sort(key=lambda x: x['message_count'], reverse=True)
        
        return summary
    
    def format_topic_name_for_sheet(self, topic: str) -> str:
        """
        Topic 이름을 Excel 시트명으로 변환
        
        Excel 시트명 제약:
        - 최대 31자
        - 특수문자 제거: / \\ ? * [ ]
        
        개선된 전략:
        - Builtin/unknown은 간결하게 약어 사용
        - User topic은 의미 있는 부분 유지 (끝부분 우선)
        
        Args:
            topic: Topic 이름 (예: "rt/parameter_events")
            
        Returns:
            str: 시트명 (예: "parameter_events")
        """
        # Builtin endpoint 간결화
        if topic.startswith('__builtin__'):
            builtin_type = topic.replace('__builtin__', '')
            # 약어 매핑
            abbreviations = {
                'SPDP_BUILTIN_PARTICIPANT_WRITER': 'SPDP_Writer',
                'SPDP_BUILTIN_PARTICIPANT_READER': 'SPDP_Reader',
                'SEDP_BUILTIN_PUBLICATIONS_WRITER': 'SEDP_Pub_Writer',
                'SEDP_BUILTIN_PUBLICATIONS_READER': 'SEDP_Pub_Reader',
                'SEDP_BUILTIN_SUBSCRIPTIONS_WRITER': 'SEDP_Sub_Writer',
                'SEDP_BUILTIN_SUBSCRIPTIONS_READER': 'SEDP_Sub_Reader',
                'BUILTIN_PARTICIPANT_MESSAGE_WRITER': 'Part_Msg_Writer',
                'BUILTIN_PARTICIPANT_MESSAGE_READER': 'Part_Msg_Reader',
                'BUILTIN_PARTICIPANT_STATELESS_WRITER': 'Part_Stateless_W',
                'BUILTIN_PARTICIPANT_STATELESS_READER': 'Part_Stateless_R',
            }
            return abbreviations.get(builtin_type, builtin_type[:31])
        
        # Unknown
        if topic == '__unknown__':
            return 'Unknown'
        
        # User topic: 특수문자 제거
        sheet_name = topic.replace('/', '_').replace('\\', '_')
        sheet_name = sheet_name.replace('?', '_').replace('*', '_')
        sheet_name = sheet_name.replace('[', '_').replace(']', '_')
        sheet_name = sheet_name.strip('_')
        
        # 31자 제한: 끝부분이 더 의미있으므로 앞부분 자르기
        if len(sheet_name) > 31:
            # 예: rq_ydlidar_ros2_driver_node_get_parametersRequest
            # → ...driver_get_parametersReq (31자)
            excess = len(sheet_name) - 31 + 3  # "..." 추가
            sheet_name = '...' + sheet_name[excess:]
        
        return sheet_name

