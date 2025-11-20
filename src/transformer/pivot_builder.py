#!/usr/bin/env python3
"""
Pivot Table Builder

파싱된 submessage를 Excel 출력용 진짜 Pivot 테이블 형식의 DataFrame으로 변환
"""

from typing import List, Dict, Any, Set, Tuple
import pandas as pd
from datetime import datetime, timezone
import pytz

# PID 정의를 가져와서 전체 PID 목록 사용
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.parser.pid_definitions import PID_DEFINITIONS


class PivotTableBuilder:
    """
    Excel 출력용 Pivot 테이블 DataFrame 생성
    
    진짜 피벗 테이블 구조:
    - 행: TimeInterval (시간 윈도우)
    - 열: 각 PID (Timestamp 컬럼과 Value 컬럼 쌍)
    - Timestamp: 한국 시간 (KST)
    - Value: dict 형태로 표현 (예: {major: 2, minor: 3})
    """
    
    def __init__(self, window_size: float = 1.0):
        """
        Args:
            window_size: 시간 윈도우 크기 (초 단위)
        """
        self.window_size = window_size
        self.kst = pytz.timezone('Asia/Seoul')
    
    def build(
        self,
        submessages: List[Dict[str, Any]],
        participant_id: tuple = None
    ) -> pd.DataFrame:
        """
        진짜 Pivot 테이블 형식의 DataFrame 생성
        
        Args:
            submessages: 파싱된 submessage 리스트 (단일 Participant)
            participant_id: (hostId, appId, instanceId) - 선택적
            
        Returns:
            pd.DataFrame: Pivot 테이블
                행: TimeInterval
                컬럼: idx, TimeInterval, PID_XXX_Timestamp, PID_XXX_Value, ...
        """
        if not submessages:
            return pd.DataFrame()
        
        # timestamp를 datetime으로 변환
        for submsg in submessages:
            ts = submsg.get('timestamp')
            if ts is not None:
                if isinstance(ts, str):
                    # UTC 타임스탬프 문자열을 datetime으로 변환
                    submsg['timestamp_dt'] = pd.to_datetime(ts, utc=True)
                elif isinstance(ts, (int, float)):
                    # Unix epoch를 datetime으로 변환
                    submsg['timestamp_dt'] = pd.to_datetime(ts, unit='s', utc=True)
        
        # 첫 타임스탬프 찾기
        timestamps_with_dt = [
            submsg['timestamp_dt'] 
            for submsg in submessages 
            if 'timestamp_dt' in submsg
        ]
        
        if not timestamps_with_dt:
            # timestamp가 하나도 없으면 빈 DataFrame 반환
            return pd.DataFrame()
        
        first_timestamp = min(timestamps_with_dt)
        
        # 모든 53개 PID를 PID 번호 순으로 정렬하여 가져오기
        all_pid_groups_ordered = self._get_all_pid_groups_ordered()
        
        # 시간 윈도우별로 그룹화
        windows = self._group_by_window(submessages, first_timestamp)
        
        # DataFrame 생성 (진짜 피벗 테이블)
        # 같은 윈도우에 여러 패킷이 있으면 각각을 별도 행으로 추가
        rows = []
        merge_info = []  # 셀 병합 정보: [(start_row, end_row, window_key), ...]
        
        # TimeInterval을 시간 순서대로 정렬
        sorted_window_keys = sorted(
            windows.keys(), 
            key=lambda x: float(x.split('-')[0].replace('s', ''))
        )
        
        current_row = 0
        for idx, window_key in enumerate(sorted_window_keys, start=1):
            window_msgs = windows[window_key]
            window_start_row = current_row
            
            # 윈도우 내 각 메시지를 별도 행으로 추가
            for msg_idx, msg in enumerate(window_msgs):
                row = {
                    'idx': idx,
                    'TimeInterval': window_key
                }
                
                # 모든 53개 PID를 PID 번호 순으로 처리
                for pid_group in all_pid_groups_ordered:
                    pid_data = self._extract_pid_data_from_single_msg(msg, pid_group)
                    row[f'{pid_group}_Timestamp'] = pid_data['timestamp']
                    row[f'{pid_group}_Value'] = pid_data['value']
                
                rows.append(row)
                current_row += 1
            
            # 윈도우에 여러 메시지가 있으면 병합 정보 저장
            window_end_row = current_row - 1
            if window_end_row > window_start_row:
                merge_info.append((window_start_row, window_end_row, window_key))
        
        df = pd.DataFrame(rows)
        
        # 병합 정보를 DataFrame에 첨부 (ExcelWriter에서 사용)
        df.attrs['merge_info'] = merge_info
        
        return df
    
    def _get_all_pid_groups_ordered(self) -> List[str]:
        """
        모든 53개 PID를 PID 번호 순으로 정렬하여 반환
        
        Returns:
            List[str]: PID 이름 리스트 (PID 번호 순)
        """
        # PID 정의를 PID 번호 순으로 정렬
        sorted_pids = sorted(PID_DEFINITIONS, key=lambda p: p.pid_id)
        return [pid.pid_name for pid in sorted_pids]
    
    def _collect_all_pid_groups(self, submessages: List[Dict[str, Any]]) -> Set[str]:
        """
        모든 submessage에서 PID 그룹 수집
        
        PID 그룹: PID_PROTOCOL_VERSION, PID_VENDOR_ID 등
        (하위 필드들은 Value에 dict로 표현)
        """
        all_pid_groups = set()
        
        for submsg in submessages:
            pids = submsg.get('pids', {})
            
            for pid_key in pids.keys():
                # PID 그룹 이름 추출
                # 예: "PID_PROTOCOL_VERSION_major" -> "PID_PROTOCOL_VERSION"
                pid_group = self._extract_pid_group_name(pid_key)
                if pid_group:
                    all_pid_groups.add(pid_group)
        
        return all_pid_groups
    
    def _extract_pid_group_name(self, pid_key: str) -> str:
        """
        PID 키에서 그룹 이름 추출
        
        예:
        - "PID_PROTOCOL_VERSION_major" -> "PID_PROTOCOL_VERSION"
        - "PID_VENDOR_ID_vendorid" -> "PID_VENDOR_ID"
        - "PID_PARTICIPANT_GUID_appid" -> "PID_PARTICIPANT_GUID"
        """
        # PID로 시작하지 않으면 무시
        if not pid_key.startswith('PID_'):
            return None
        
        # 잘 알려진 PID 목록 (순서대로 확인)
        known_pids = [
            'PID_PROTOCOL_VERSION',
            'PID_VENDOR_ID',
            'PID_PARTICIPANT_GUID',
            'PID_PARTICIPANT_LEASE_DURATION',
            'PID_BUILTIN_ENDPOINT_SET',
            'PID_PROPERTY_LIST',
            'PID_TYPE_MAX_SIZE_SERIALIZED',
            'PID_ENTITY_NAME',
            'PID_KEY_HASH',
            'PID_STATUS_INFO',
            'PID_EXPECTS_INLINE_QOS',
            'PID_DOMAIN_ID',
            'PID_DOMAIN_TAG',
        ]
        
        for known_pid in known_pids:
            if pid_key.startswith(known_pid):
                return known_pid
        
        # 알려진 PID가 아니면 첫 3개 언더스코어까지를 PID 이름으로 간주
        parts = pid_key.split('_')
        if len(parts) >= 3:
            return '_'.join(parts[:3])
        
        return pid_key
    
    def _group_by_window(
        self,
        submessages: List[Dict[str, Any]],
        first_timestamp: datetime
    ) -> Dict[str, List[Dict[str, Any]]]:
        """시간 윈도우별로 그룹화"""
        from collections import defaultdict
        
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
    
    def _extract_pid_data_from_single_msg(
        self,
        msg: Dict[str, Any],
        pid_group: str
    ) -> Dict[str, Any]:
        """
        단일 메시지에서 특정 PID 그룹의 값 추출
        
        Args:
            msg: 단일 submessage
            pid_group: PID 그룹 이름 (예: "PID_PROTOCOL_VERSION")
            
        Returns:
            dict: {'timestamp': KST 시간 문자열, 'value': dict 형태 문자열 또는 NULL}
        """
        pids = msg.get('pids', {})
        
        # 이 PID 그룹에 속하는 모든 필드 수집
        pid_fields = {}
        for pid_key, pid_value in pids.items():
            if pid_key.startswith(pid_group):
                # 필드명 추출 (예: "PID_PROTOCOL_VERSION_major" -> "major")
                field_name = pid_key[len(pid_group):].lstrip('_')
                if field_name:
                    pid_fields[field_name] = pid_value
                elif pid_key == pid_group:
                    # PID_SENTINEL처럼 필드가 없는 경우
                    if pid_value:
                        pid_fields['_value'] = pid_value
        
        if pid_fields:
            # Timestamp: 한국 시간 (KST)
            timestamp_dt = msg.get('timestamp_dt')
            if timestamp_dt:
                kst_time = timestamp_dt.astimezone(self.kst)
                timestamp_str = kst_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            else:
                timestamp_str = None
            
            # Value: dict 형태로 표현
            value_str = self._format_pid_value(pid_fields)
            
            return {
                'timestamp': timestamp_str,
                'value': value_str
            }
        
        # PID가 없으면 빈 값
        return {'timestamp': None, 'value': None}
    
    def _extract_pid_data_from_window(
        self,
        window_msgs: List[Dict[str, Any]],
        pid_group: str
    ) -> Dict[str, Any]:
        """
        특정 윈도우에서 특정 PID 그룹의 첫 번째 값 추출
        
        ⚠️  DEPRECATED: 이 메서드는 첫 번째 메시지만 사용합니다.
        대신 _extract_pid_data_from_single_msg() 사용을 권장합니다.
        
        Args:
            window_msgs: 윈도우 내 submessages
            pid_group: PID 그룹 이름 (예: "PID_PROTOCOL_VERSION")
            
        Returns:
            dict: {'timestamp': KST 시간 문자열, 'value': dict 형태 문자열 또는 NULL}
        """
        for msg in window_msgs:
            result = self._extract_pid_data_from_single_msg(msg, pid_group)
            if result['timestamp'] is not None or result['value'] is not None:
                return result
        
        # PID가 없으면 빈 값 (NULL)
        return {'timestamp': None, 'value': None}
    
    def _format_pid_value(self, pid_fields: Dict[str, Any]) -> str:
        """
        PID 필드를 dict 형태 문자열로 포맷팅
        
        Args:
            pid_fields: {'major': '2', 'minor': '3'}
            
        Returns:
            str: "{major: 2, minor: 3}"
        """
        if not pid_fields:
            return None
        
        # 단일 필드면 값만 반환
        if len(pid_fields) == 1:
            return str(list(pid_fields.values())[0])
        
        # 여러 필드면 dict 형태로
        items = []
        for key, value in sorted(pid_fields.items()):
            items.append(f"{key}: {value}")
        
        return "{" + ", ".join(items) + "}"
    
    def build_summary(
        self,
        grouped_data: Dict[tuple, List[Dict[str, Any]]]
    ) -> pd.DataFrame:
        """
        전체 Participant 요약 DataFrame 생성 (Overview 시트용)
        
        Args:
            grouped_data: {(hostId, appId, instanceId): [submsg, ...], ...}
            
        Returns:
            pd.DataFrame: Overview 데이터
                컬럼: Node, HostId, AppId, MessageCount, EntityCount, TimeSpan, SubmsgTypes
        """
        rows = []
        
        for idx, (participant_id, messages) in enumerate(grouped_data.items(), start=1):
            hostId, appId, instanceId = participant_id
            
            # Entity 수집
            entities = set()
            for msg in messages:
                entity = msg.get('entity', {})
                if entity.get('rdEntityId'):
                    entities.add(entity['rdEntityId'])
                if entity.get('wrEntityId'):
                    entities.add(entity['wrEntityId'])
            
            # 시간 범위 계산
            timestamps = []
            for msg in messages:
                if isinstance(msg.get('timestamp'), str):
                    ts = pd.to_datetime(msg['timestamp'], utc=True)
                    timestamps.append(ts)
            
            if timestamps:
                time_span = (max(timestamps) - min(timestamps)).total_seconds()
                time_span_str = f"{time_span:.2f}s"
            else:
                time_span_str = "N/A"
            
            # Submessage Type 통계
            submsg_types = {}
            for msg in messages:
                stype = msg.get('submsg_name', 'UNKNOWN')
                submsg_types[stype] = submsg_types.get(stype, 0) + 1
            
            row = {
                'Node': f"Node_{idx}",
                'HostId': f"{hostId:08x}",
                'AppId': f"{appId:08x}",
                'MessageCount': len(messages),
                'EntityCount': len(entities),
                'TimeSpan': time_span_str,
                'SubmsgTypes': ', '.join(sorted(submsg_types.keys()))
            }
            
            rows.append(row)
        
        df = pd.DataFrame(rows)
        return df
