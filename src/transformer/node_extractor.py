"""
Node Name Extractor

ROS2 토픽에서 노드 이름을 추출하고 정규화합니다.
"""

import re
from typing import Optional, Set, Dict, List
from collections import defaultdict


class NodeNameExtractor:
    """
    ROS2 노드 이름 추출 및 정규화
    
    토픽 패턴:
    - rq/<node_name>/...   (Request)
    - rr/<node_name>/...   (Response)
    - rt/<node_name>/...   (Regular Topic)
    
    정규화 규칙:
    - launch_ros_<PID> → launch_ros_*
    - _<숫자>로 끝나는 런처 → _* 
    """
    
    # 정규화 패턴: PID가 붙는 런처 노드들
    LAUNCHER_PATTERNS = [
        (r'^launch_ros_\d+$', 'launch_ros_*'),           # launch_ros_2385 → launch_ros_*
        (r'^_ros2cli_\d+$', '_ros2cli_*'),               # _ros2cli_12345 → _ros2cli_*
        (r'^_launch_\d+$', '_launch_*'),                 # _launch_9876 → _launch_*
    ]
    
    def __init__(self):
        self.raw_to_normalized: Dict[str, str] = {}
        self.normalized_to_raws: Dict[str, Set[str]] = defaultdict(set)
    
    def extract_node_from_topic(self, topic: str) -> Optional[str]:
        """
        토픽 이름에서 노드 이름 추출
        
        Args:
            topic: 토픽 이름 (예: "rq/stella_md_node/get_parametersRequest")
            
        Returns:
            str | None: 노드 이름 (예: "stella_md_node") 또는 None
            
        Note:
            rq/<node>/<service>, rr/<node>/<service> 패턴만 노드 이름을 포함합니다.
            (3개 세그먼트 필수)
            
            2개 세그먼트는 서비스 이름이므로 제외:
            - rq/start_scanRequest → None (서비스 이름) ❌
            - rq/stop_scanRequest → None (서비스 이름) ❌
            
            예시:
            - rq/stella_md_node/get_parametersRequest → stella_md_node ✅
            - rr/teleop_keyboard/describe_parametersReply → teleop_keyboard ✅
            - rq/start_scanRequest → None (2 세그먼트, 서비스) ❌
            - rt/cmd_vel → None (일반 토픽) ❌
        """
        if not topic:
            return None
        
        # 세그먼트 개수 확인
        segments = topic.split('/')
        
        # rq/<node>/<service> 또는 rr/<node>/<service> 형식만 허용 (3개 세그먼트)
        if len(segments) != 3:
            return None
        
        prefix = segments[0]
        node_name = segments[1]
        
        # rq 또는 rr 패턴만 허용
        if prefix not in ['rq', 'rr']:
            return None
        
        return self.normalize_node_name(node_name)
    
    def normalize_node_name(self, node_name: str) -> str:
        """
        노드 이름 정규화
        
        Args:
            node_name: 원본 노드 이름 (예: "launch_ros_2385")
            
        Returns:
            str: 정규화된 노드 이름 (예: "launch_ros_*")
        """
        # 이미 정규화된 적 있으면 캐시 반환
        if node_name in self.raw_to_normalized:
            return self.raw_to_normalized[node_name]
        
        # 정규화 패턴 매칭
        normalized = node_name
        for pattern, replacement in self.LAUNCHER_PATTERNS:
            if re.match(pattern, node_name):
                normalized = replacement
                break
        
        # 캐시 저장
        self.raw_to_normalized[node_name] = normalized
        self.normalized_to_raws[normalized].add(node_name)
        
        return normalized
    
    def get_all_nodes_from_topics(self, topics: List[str]) -> Dict[str, Set[str]]:
        """
        토픽 목록에서 모든 노드 추출 및 그룹화
        
        Args:
            topics: 토픽 이름 리스트
            
        Returns:
            dict: {normalized_node_name: {topic1, topic2, ...}}
        """
        node_topics = defaultdict(set)
        
        for topic in topics:
            node_name = self.extract_node_from_topic(topic)
            if node_name:
                node_topics[node_name].add(topic)
        
        return dict(node_topics)
    
    def get_raw_node_names(self, normalized_name: str) -> Set[str]:
        """
        정규화된 이름에 해당하는 원본 이름들 반환
        
        Args:
            normalized_name: 정규화된 노드 이름 (예: "launch_ros_*")
            
        Returns:
            set: 원본 노드 이름 집합 (예: {"launch_ros_2385", "launch_ros_13771"})
        """
        return self.normalized_to_raws.get(normalized_name, set())
    
    def is_launcher_node(self, node_name: str) -> bool:
        """
        런처 노드 여부 확인
        
        Args:
            node_name: 노드 이름
            
        Returns:
            bool: 런처 노드이면 True
        """
        normalized = self.normalize_node_name(node_name)
        return normalized.endswith('_*')
    
    def format_node_for_display(self, node_name: str) -> str:
        """
        노드 이름을 표시용으로 포맷팅
        
        Args:
            node_name: 노드 이름 (정규화된 이름)
            
        Returns:
            str: "/<node_name>" 형식
        """
        if not node_name.startswith('/'):
            return f"/{node_name}"
        return node_name


# 모듈 레벨 함수 (하위 호환성)
def extract_node_from_topic(topic: str) -> Optional[str]:
    """
    토픽 이름에서 노드 이름 추출 (정규화 포함)
    
    Args:
        topic: 토픽 이름
        
    Returns:
        str | None: 정규화된 노드 이름
    """
    extractor = NodeNameExtractor()
    return extractor.extract_node_from_topic(topic)


def normalize_node_name(node_name: str) -> str:
    """
    노드 이름 정규화
    
    Args:
        node_name: 원본 노드 이름
        
    Returns:
        str: 정규화된 노드 이름
    """
    extractor = NodeNameExtractor()
    return extractor.normalize_node_name(node_name)


if __name__ == "__main__":
    # 테스트
    extractor = NodeNameExtractor()
    
    test_cases = [
        "rq/launch_ros_2385/get_parametersRequest",
        "rq/launch_ros_13771/set_parametersRequest",
        "rq/stella_md_node/get_parametersRequest",
        "rt/cmd_vel",
        "rr/launch_ros_9999/list_parametersReply",
    ]
    
    print("=" * 80)
    print("NodeNameExtractor 테스트")
    print("=" * 80)
    
    for topic in test_cases:
        node = extractor.extract_node_from_topic(topic)
        print(f"Topic: {topic}")
        print(f"  → Node: {node}")
        print()
    
    # 정규화 캐시 확인
    print("=" * 80)
    print("정규화 매핑:")
    print("=" * 80)
    for normalized, raws in extractor.normalized_to_raws.items():
        print(f"{normalized}:")
        for raw in sorted(raws):
            print(f"  ← {raw}")
        print()
