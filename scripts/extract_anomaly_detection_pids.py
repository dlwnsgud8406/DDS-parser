#!/usr/bin/env python3
"""
이상탐지용 PID 추출 스크립트

ML 기반 이상탐지에 유용한 13개의 PID만 선별적으로 파싱합니다.

선택된 PID 목록:
1. PID_TOPIC_NAME (0x0005) - 토픽 이름
2. PID_TYPE_NAME (0x0007) - 메시지 타입 이름
3. PID_RELIABILITY (0x001A) - Reliable or Best-Effort
4. PID_DURABILITY (0x001D) - Volatile / Transient
5. PID_LIVELINESS (0x001B) - 자동/수동/수명 기반 활성화
6. PID_DEADLINE (0x0023) - 메시지 주기 보장
7. PID_LATENCY_BUDGET (0x0027) - 수용 가능한 지연시간
8. PID_DESTINATION_ORDER (0x0025) - 메시지 순서 보장 정책
9. PID_USER_DATA (0x002C) - 노드에 부여된 사용자 데이터
10. PID_HISTORY (0x0040) - KEEP_LAST or KEEP_ALL
11. PID_RESOURCE_LIMIT (0x0041) - 버퍼 크기 제한
12. PID_TYPE_MAX_SIZE_SERIALIZED (0x0061) - 최대 직렬화 크기
13. PID_STATUS_INFO (0x0071) - 상태 변화 정보 (예: disposed)

이상탐지 관점:
- 예상치 못한 토픽 출현, 토픽 변경 탐지
- 타입 불일치, 잘못된 타입 변경
- QoS 다운그레이드 (ex: Reliable → Best-Effort)
- 데이터 유지 정책 변화 감지
- 비정상 노드 종료, 수명 위반
- 메시지 수 감소나 주기 변화 탐지
- 지연성 변화에 민감한 시스템에서 유용
- 순서 변경 탐지 (오더 바뀌는 이상)
- 노드 인증, 속성 기반 탐지 가능성
- 메시지 유실에 민감한 토픽 감지 가능
- 지나친 리소스 제한도 이상 탐지 힌트
- 비정상적으로 큰 메시지 감지
- participant 종료, 비정상 detach
"""

import sys
from pathlib import Path
from collections import defaultdict, Counter
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor


# 이상탐지용 PID 목록 (hex code 포함)
ANOMALY_DETECTION_PIDS = {
    'PID_TOPIC_NAME': {
        'code': '0x0005',
        'description': '토픽 이름',
        'anomaly_purpose': '예상치 못한 토픽 출현, 토픽 변경 탐지',
        'fields': ['topic']
    },
    'PID_TYPE_NAME': {
        'code': '0x0007',
        'description': '메시지 타입 이름',
        'anomaly_purpose': '타입 불일치, 잘못된 타입 변경',
        'fields': ['typename']
    },
    'PID_RELIABILITY': {
        'code': '0x001A',
        'description': 'Reliable or Best-Effort',
        'anomaly_purpose': 'QoS 다운그레이드 (ex: Reliable → Best-Effort)',
        'fields': ['kind', 'max_blocking_time_sec', 'max_blocking_time_frac']
    },
    'PID_DURABILITY': {
        'code': '0x001D',
        'description': 'Volatile / Transient',
        'anomaly_purpose': '데이터 유지 정책 변화 감지',
        'fields': ['kind']
    },
    'PID_LIVELINESS': {
        'code': '0x001B',
        'description': '자동/수동/수명 기반 활성화',
        'anomaly_purpose': '비정상 노드 종료, 수명 위반',
        'fields': ['kind', 'lease_duration_sec', 'lease_duration_frac']
    },
    'PID_DEADLINE': {
        'code': '0x0023',
        'description': '메시지 주기 보장',
        'anomaly_purpose': '메시지 수 감소나 주기 변화 탐지',
        'fields': ['period_sec', 'period_frac']
    },
    'PID_LATENCY_BUDGET': {
        'code': '0x0027',
        'description': '수용 가능한 지연시간',
        'anomaly_purpose': '지연성 변화에 민감한 시스템에서 유용',
        'fields': ['duration_sec', 'duration_frac']
    },
    'PID_DESTINATION_ORDER': {
        'code': '0x0025',
        'description': '메시지 순서 보장 정책',
        'anomaly_purpose': '순서 변경 탐지 (오더 바뀌는 이상)',
        'fields': ['kind']
    },
    'PID_USER_DATA': {
        'code': '0x002C',
        'description': '노드에 부여된 사용자 데이터',
        'anomaly_purpose': '노드 인증, 속성 기반 탐지 가능성',
        'fields': ['data']
    },
    'PID_HISTORY': {
        'code': '0x0040',
        'description': 'KEEP_LAST or KEEP_ALL',
        'anomaly_purpose': '메시지 유실에 민감한 토픽 감지 가능',
        'fields': ['kind', 'depth']
    },
    'PID_RESOURCE_LIMIT': {
        'code': '0x0041',
        'description': '버퍼 크기 제한',
        'anomaly_purpose': '지나친 리소스 제한도 이상 탐지 힌트',
        'fields': ['max_samples', 'max_instances', 'max_samples_per_instance']
    },
    'PID_TYPE_MAX_SIZE_SERIALIZED': {
        'code': '0x0061',
        'description': '최대 직렬화 크기',
        'anomaly_purpose': '비정상적으로 큰 메시지 감지',
        'fields': ['value']
    },
    'PID_STATUS_INFO': {
        'code': '0x0071',
        'description': '상태 변화 정보 (예: disposed)',
        'anomaly_purpose': 'participant 종료, 비정상 detach',
        'fields': ['statusinfo']
    }
}


def extract_anomaly_detection_pids(submessages):
    """이상탐지용 PID만 추출"""
    
    extracted_data = defaultdict(list)
    field_presence = Counter()
    
    print(f"총 {len(submessages):,}개의 submessage 분석 중...\n")
    
    for msg in submessages:
        pids = msg.get('pids', {})
        timestamp = msg.get('timestamp')
        submsg_type = msg.get('submsg_name', '')
        
        # Participant/Endpoint GUID (노드 식별용)
        guid = msg.get('guid', {})
        host_id = guid.get('hostId', 0)
        app_id = guid.get('appId', 0)
        instance_id = guid.get('instanceId', 0)
        entity_id = guid.get('entityId', 0)
        
        participant_guid = f"{host_id:08x}:{app_id:08x}:{instance_id:08x}" if isinstance(host_id, int) else '?:?:?'
        endpoint_guid = f"{participant_guid}:{entity_id:08x}" if isinstance(entity_id, int) else f"{participant_guid}:?"
        
        record = {
            'timestamp': timestamp,
            'submsg_type': submsg_type,
            'participant_guid': participant_guid,
            'endpoint_guid': endpoint_guid
        }
        
        # 각 이상탐지 PID 추출
        for pid_name, pid_info in ANOMALY_DETECTION_PIDS.items():
            fields = pid_info['fields']
            
            # 모든 필드가 있는지 확인
            has_data = False
            for field in fields:
                field_key = f"{pid_name}_{field}"
                value = pids.get(field_key)
                
                if value is not None:
                    has_data = True
                    field_presence[pid_name] += 1
                
                record[field_key] = value
            
            # 데이터가 하나라도 있으면 기록
            if has_data:
                extracted_data[pid_name].append(record.copy())
    
    return extracted_data, field_presence


def format_value(key, value):
    """값을 읽기 쉬운 형식으로 변환"""
    if value is None:
        return 'N/A'
    
    # Hex 값
    if isinstance(value, str) and value.startswith('0x'):
        return value
    
    # Kind 값 매핑
    if 'kind' in key.lower():
        if 'RELIABILITY' in key:
            return {0: 'BEST_EFFORT', 1: 'RELIABLE'}.get(value, f'0x{value:02x}' if isinstance(value, int) else value)
        elif 'DURABILITY' in key:
            return {0: 'VOLATILE', 1: 'TRANSIENT_LOCAL', 2: 'TRANSIENT', 3: 'PERSISTENT'}.get(
                value, f'0x{value:02x}' if isinstance(value, int) else value
            )
        elif 'LIVELINESS' in key:
            return {0: 'AUTOMATIC', 1: 'MANUAL_BY_PARTICIPANT', 2: 'MANUAL_BY_TOPIC'}.get(
                value, f'0x{value:02x}' if isinstance(value, int) else value
            )
        elif 'HISTORY' in key:
            return {0: 'KEEP_LAST', 1: 'KEEP_ALL'}.get(value, f'0x{value:02x}' if isinstance(value, int) else value)
        elif 'DESTINATION_ORDER' in key:
            return {0: 'BY_RECEPTION_TIMESTAMP', 1: 'BY_SOURCE_TIMESTAMP'}.get(
                value, f'0x{value:02x}' if isinstance(value, int) else value
            )
    
    # Duration/Period 값 (sec + frac)
    if 'sec' in key.lower() or 'frac' in key.lower():
        if isinstance(value, int):
            if value == 0x7FFFFFFF or value == 0xFFFFFFFF:
                return 'INFINITE'
            elif value == 0:
                return '0'
            else:
                return str(value)
    
    # 긴 문자열 자르기
    if isinstance(value, str) and len(value) > 50:
        return value[:47] + '...'
    
    return str(value)


def main():
    if len(sys.argv) < 2:
        print("사용법: python scripts/extract_anomaly_detection_pids.py <pcap_file> [max_packets]")
        print()
        print("예시:")
        print("  python scripts/extract_anomaly_detection_pids.py data/shm.pcapng")
        print("  python scripts/extract_anomaly_detection_pids.py data/shm.pcapng 5000")
        sys.exit(1)
    
    pcap_file = sys.argv[1]
    max_packets = int(sys.argv[2]) if len(sys.argv) > 2 else None
    
    print("=" * 80)
    print("이상탐지용 PID 추출 (13개)")
    print("=" * 80)
    print(f"파일: {pcap_file}")
    if max_packets:
        print(f"최대 패킷: {max_packets:,}")
    print()
    
    # 1. PCAP 파싱
    print("[1/3] PCAP 파일 파싱 중...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=999999.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage 파싱 완료\n")
    
    # 2. 이상탐지 PID 추출
    print("[2/3] 이상탐지용 PID 추출 중...")
    extracted_data, field_presence = extract_anomaly_detection_pids(submessages)
    print(f"  ✓ 추출 완료\n")
    
    # 3. 결과 출력
    print("[3/3] 추출 결과 분석")
    print("=" * 80)
    print()
    
    print("📋 이상탐지용 PID 목록 (13개)")
    print("=" * 80)
    print()
    
    for i, (pid_name, pid_info) in enumerate(ANOMALY_DETECTION_PIDS.items(), 1):
        count = field_presence.get(pid_name, 0)
        status = "✅" if count > 0 else "❌"
        
        print(f"{status} {i:2d}. {pid_name:35s} ({pid_info['code']})")
        print(f"      설명: {pid_info['description']}")
        print(f"      이상탐지 목적: {pid_info['anomaly_purpose']}")
        print(f"      발견 횟수: {count:,}개")
        
        # 샘플 데이터 출력 (처음 3개만)
        if pid_name in extracted_data and extracted_data[pid_name]:
            print(f"      샘플 데이터:")
            samples = extracted_data[pid_name][:3]
            
            for j, sample in enumerate(samples, 1):
                print(f"         #{j}:")
                for field in pid_info['fields']:
                    field_key = f"{pid_name}_{field}"
                    value = sample.get(field_key)
                    formatted = format_value(field_key, value)
                    print(f"            {field:30s} = {formatted}")
        
        print()
    
    # 통계 요약
    print("=" * 80)
    print("📊 추출 통계")
    print("=" * 80)
    print()
    
    total_pids = len(ANOMALY_DETECTION_PIDS)
    found_pids = sum(1 for count in field_presence.values() if count > 0)
    missing_pids = total_pids - found_pids
    
    print(f"전체 이상탐지 PID: {total_pids}개")
    print(f"데이터 발견: {found_pids}개")
    print(f"데이터 없음: {missing_pids}개")
    print()
    
    if found_pids > 0:
        print("✅ 발견된 PID:")
        for pid_name, count in sorted(field_presence.items(), key=lambda x: x[1], reverse=True):
            print(f"   • {pid_name:35s}: {count:6,}회")
    
    if missing_pids > 0:
        missing_list = [pid for pid in ANOMALY_DETECTION_PIDS.keys() if pid not in field_presence]
        print()
        print("❌ 데이터 없는 PID:")
        for pid_name in missing_list:
            print(f"   • {pid_name:35s} - {ANOMALY_DETECTION_PIDS[pid_name]['description']}")
    
    print()
    print("=" * 80)
    print("💡 다음 단계:")
    print("=" * 80)
    print()
    print("1. 발견된 PID를 DataFrame으로 변환")
    print("2. 각 PID 값의 변화 패턴 분석")
    print("3. Decision Tree / ML 모델 학습")
    print("   - 정상 패턴 학습")
    print("   - 이상 패턴 탐지")
    print("4. 실시간 모니터링 시스템 구축")
    print()
    print("📌 핵심 이상탐지 시나리오:")
    print("   • 토픽 이름 변경 → PID_TOPIC_NAME 모니터링")
    print("   • QoS 다운그레이드 → PID_RELIABILITY, PID_DURABILITY 모니터링")
    print("   • 노드 비정상 종료 → PID_STATUS_INFO, PID_LIVELINESS 모니터링")
    print("   • 메시지 주기 변화 → PID_DEADLINE + 타임스탬프 분석")
    print("   • 순서 이상 → PID_DESTINATION_ORDER 모니터링")
    print("   • 큰 메시지 전송 → PID_TYPE_MAX_SIZE_SERIALIZED 모니터링")


if __name__ == "__main__":
    main()
