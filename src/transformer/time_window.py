"""
Time Window Generator

파싱된 submessage를 시간 윈도우 단위로 그룹화
"""

from typing import List, Dict, Any
from datetime import datetime
from collections import defaultdict
import pandas as pd


class TimeWindowGenerator:
    """
    시간 윈도우 기반 데이터 그룹화
    
    Excel의 TimeInterval 컬럼을 생성하기 위해
    submessage들을 시간 구간별로 그룹화합니다.
    """
    
    def __init__(self, window_size: float = 1.0):
        """
        Args:
            window_size: 윈도우 크기 (초 단위), 기본값 1.0초
        """
        if window_size <= 0:
            raise ValueError("window_size must be greater than 0")
        
        self.window_size = window_size
    
    def generate_windows(
        self,
        submessages: List[Dict[str, Any]]
    ) -> Dict[str, List[Dict[str, Any]]]:
        """
        submessage를 시간 윈도우별로 그룹화
        
        Args:
            submessages: 파싱된 submessage 리스트
                각 submessage는 'timestamp' 필드를 포함해야 함
        
        Returns:
            dict: {
                "0-1s": [msg1, msg2, ...],
                "1-2s": [msg3, msg4, ...],
                ...
            }
        """
        if not submessages:
            return {}
        
        # timestamp를 datetime으로 변환
        for submsg in submessages:
            if isinstance(submsg.get('timestamp'), str):
                submsg['timestamp_dt'] = pd.to_datetime(submsg['timestamp'])
        
        # 첫 타임스탬프 찾기
        first_timestamp = min(
            submsg['timestamp_dt'] 
            for submsg in submessages 
            if 'timestamp_dt' in submsg
        )
        
        # 윈도우별로 그룹화
        windows = defaultdict(list)
        
        for submsg in submessages:
            if 'timestamp_dt' not in submsg:
                continue
            
            timestamp = submsg['timestamp_dt']
            elapsed = (timestamp - first_timestamp).total_seconds()
            
            # 어느 윈도우에 속하는지 계산
            window_index = int(elapsed / self.window_size)
            window_start = window_index * self.window_size
            window_end = window_start + self.window_size
            
            window_key = f"{window_start:.1f}-{window_end:.1f}s"
            windows[window_key].append(submsg)
        
        return dict(windows)
    
    def get_window_stats(
        self,
        windowed_data: Dict[str, List[Dict[str, Any]]]
    ) -> List[Dict[str, Any]]:
        """
        각 윈도우의 통계 정보 생성
        
        Args:
            windowed_data: generate_windows()의 결과
            
        Returns:
            list[dict]: [
                {
                    'window': '0-1s',
                    'message_count': int,
                    'submsg_types': dict[str, int]
                },
                ...
            ]
        """
        stats = []
        
        for window_key, messages in sorted(windowed_data.items()):
            submsg_types = defaultdict(int)
            for msg in messages:
                submsg_name = msg.get('submsg_name', 'UNKNOWN')
                submsg_types[submsg_name] += 1
            
            stats.append({
                'window': window_key,
                'message_count': len(messages),
                'submsg_types': dict(submsg_types)
            })
        
        return stats
    
    def create_time_series(
        self,
        submessages: List[Dict[str, Any]],
        value_key: str = 'submsg_id'
    ) -> pd.DataFrame:
        """
        시계열 DataFrame 생성
        
        Args:
            submessages: 파싱된 submessage 리스트
            value_key: 값으로 사용할 필드명
            
        Returns:
            pd.DataFrame: timestamp 인덱스를 가진 시계열 데이터
        """
        if not submessages:
            return pd.DataFrame()
        
        # timestamp와 value 추출
        data = []
        for submsg in submessages:
            if 'timestamp' not in submsg:
                continue
            
            timestamp = pd.to_datetime(submsg['timestamp'])
            
            # value_key가 nested dict인 경우 처리
            if '.' in value_key:
                keys = value_key.split('.')
                value = submsg
                for k in keys:
                    value = value.get(k, {})
            else:
                value = submsg.get(value_key)
            
            data.append({
                'timestamp': timestamp,
                value_key: value
            })
        
        if not data:
            return pd.DataFrame()
        
        df = pd.DataFrame(data)
        df.set_index('timestamp', inplace=True)
        df.sort_index(inplace=True)
        
        return df
