#!/usr/bin/env python3
"""
이상탐지용 엑셀 생성 스크립트

13개의 이상탐지용 PID만 포함한 최적화된 엑셀 파일을 생성합니다.
불필요한 PID 컬럼은 전부 제거하여 ML 학습에 최적화된 데이터셋을 만듭니다.

포함되는 컬럼:
1. 기본 정보: timestamp, submsg_type, participant_guid, endpoint_guid
2. 이상탐지 PID (13개):
   - PID_TOPIC_NAME
   - PID_TYPE_NAME
   - PID_RELIABILITY
   - PID_DURABILITY
   - PID_LIVELINESS
   - PID_DEADLINE
   - PID_LATENCY_BUDGET
   - PID_DESTINATION_ORDER
   - PID_USER_DATA
   - PID_HISTORY
   - PID_RESOURCE_LIMIT
   - PID_TYPE_MAX_SIZE_SERIALIZED
   - PID_STATUS_INFO
"""

import sys
from pathlib import Path
import pandas as pd
from collections import defaultdict

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor


# 이상탐지용 PID 필드 정의
ANOMALY_DETECTION_FIELDS = {
    'PID_TOPIC_NAME': ['topic'],
    'PID_TYPE_NAME': ['typename'],
    'PID_RELIABILITY': ['kind', 'max_blocking_time_sec', 'max_blocking_time_frac'],
    'PID_DURABILITY': ['kind'],
    'PID_LIVELINESS': ['kind', 'lease_duration_sec', 'lease_duration_frac'],
    'PID_DEADLINE': ['period_sec', 'period_frac'],
    'PID_LATENCY_BUDGET': ['duration_sec', 'duration_frac'],
    'PID_DESTINATION_ORDER': ['kind'],
    'PID_USER_DATA': ['data'],
    'PID_HISTORY': ['kind', 'depth'],
    'PID_RESOURCE_LIMIT': ['max_samples', 'max_instances', 'max_samples_per_instance'],
    'PID_TYPE_MAX_SIZE_SERIALIZED': ['value'],
    'PID_STATUS_INFO': ['statusinfo']
}


def format_value_for_excel(key, value):
    """엑셀에 저장할 값 포맷팅"""
    if value is None:
        return None
    
    # Kind 값 매핑
    if 'kind' in key.lower():
        if 'RELIABILITY' in key:
            mapping = {0: 'BEST_EFFORT', 1: 'RELIABLE'}
        elif 'DURABILITY' in key:
            mapping = {0: 'VOLATILE', 1: 'TRANSIENT_LOCAL', 2: 'TRANSIENT', 3: 'PERSISTENT'}
        elif 'LIVELINESS' in key:
            mapping = {0: 'AUTOMATIC', 1: 'MANUAL_BY_PARTICIPANT', 2: 'MANUAL_BY_TOPIC'}
        elif 'HISTORY' in key:
            mapping = {0: 'KEEP_LAST', 1: 'KEEP_ALL'}
        elif 'DESTINATION_ORDER' in key:
            mapping = {0: 'BY_RECEPTION_TIMESTAMP', 1: 'BY_SOURCE_TIMESTAMP'}
        else:
            return value
        
        if isinstance(value, int):
            return mapping.get(value, f'0x{value:02x}')
        return value
    
    # Duration/Period 값
    if 'sec' in key.lower() or 'frac' in key.lower():
        if isinstance(value, int):
            if value == 0x7FFFFFFF or value == 0xFFFFFFFF:
                return 'INFINITE'
            elif value == 0:
                return 0
    
    # Hex 문자열은 그대로
    if isinstance(value, str) and value.startswith('0x'):
        return value
    
    return value


def extract_anomaly_detection_data(submessages):
    """이상탐지용 PID만 추출"""
    
    rows = []
    
    print(f"총 {len(submessages):,}개의 submessage에서 데이터 추출 중...")
    
    for i, msg in enumerate(submessages):
        if (i + 1) % 10000 == 0:
            print(f"  진행: {i+1:,}/{len(submessages):,} ({(i+1)/len(submessages)*100:.1f}%)")
        
        pids = msg.get('pids', {})
        timestamp = msg.get('timestamp')
        submsg_type = msg.get('submsg_name', '')
        
        # Participant/Endpoint GUID
        guid = msg.get('guid', {})
        host_id = guid.get('hostId', 0)
        app_id = guid.get('appId', 0)
        instance_id = guid.get('instanceId', 0)
        entity_id = guid.get('entityId', 0)
        
        participant_guid = f"{host_id:08x}:{app_id:08x}:{instance_id:08x}" if isinstance(host_id, int) else None
        endpoint_guid = f"{participant_guid}:{entity_id:08x}" if isinstance(entity_id, int) and participant_guid else None
        
        # 기본 컬럼
        row = {
            'timestamp': timestamp,
            'submsg_type': submsg_type,
            'participant_guid': participant_guid,
            'endpoint_guid': endpoint_guid
        }
        
        # 이상탐지 PID 추출
        has_any_pid = False
        for pid_name, fields in ANOMALY_DETECTION_FIELDS.items():
            for field in fields:
                field_key = f"{pid_name}_{field}"
                value = pids.get(field_key)
                
                if value is not None:
                    has_any_pid = True
                
                # 포맷팅된 값 저장
                formatted_value = format_value_for_excel(field_key, value)
                row[field_key] = formatted_value
        
        # PID가 하나라도 있으면 행 추가
        if has_any_pid:
            rows.append(row)
    
    print(f"  완료: {len(rows):,}개의 행 생성")
    
    return rows


def create_anomaly_detection_dataframe(rows):
    """DataFrame 생성"""
    
    print("\nDataFrame 생성 중...")
    
    df = pd.DataFrame(rows)
    
    # 컬럼 순서 정렬
    base_columns = ['timestamp', 'submsg_type', 'participant_guid', 'endpoint_guid']
    
    # 이상탐지 PID 컬럼 (알파벳 순서)
    pid_columns = []
    for pid_name in sorted(ANOMALY_DETECTION_FIELDS.keys()):
        for field in ANOMALY_DETECTION_FIELDS[pid_name]:
            col_name = f"{pid_name}_{field}"
            if col_name in df.columns:
                pid_columns.append(col_name)
    
    # 최종 컬럼 순서
    ordered_columns = base_columns + pid_columns
    existing_columns = [col for col in ordered_columns if col in df.columns]
    
    df = df[existing_columns]
    
    print(f"  ✓ DataFrame 생성 완료: {len(df):,} 행 × {len(df.columns)} 컬럼")
    
    return df


def save_to_excel(df, output_file):
    """엑셀 파일로 저장"""
    
    print(f"\n엑셀 파일 저장 중: {output_file}")
    
    with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
        # 메인 시트
        df.to_excel(writer, sheet_name='Anomaly_Detection_Data', index=False)
        
        # 통계 시트
        stats_data = []
        
        # 컬럼별 통계
        for col in df.columns:
            if col in ['timestamp', 'submsg_type', 'participant_guid', 'endpoint_guid']:
                continue
            
            non_null_count = df[col].notna().sum()
            unique_count = df[col].nunique()
            
            if non_null_count > 0:
                stats_data.append({
                    'PID_Field': col,
                    'Non_Null_Count': non_null_count,
                    'Unique_Values': unique_count,
                    'Data_Percentage': f"{non_null_count / len(df) * 100:.2f}%"
                })
        
        stats_df = pd.DataFrame(stats_data)
        stats_df = stats_df.sort_values('Non_Null_Count', ascending=False)
        stats_df.to_excel(writer, sheet_name='Statistics', index=False)
        
        # 메타데이터 시트
        metadata = pd.DataFrame([
            {'Key': 'Total Rows', 'Value': len(df)},
            {'Key': 'Total Columns', 'Value': len(df.columns)},
            {'Key': 'PID Fields', 'Value': len(df.columns) - 4},  # 기본 컬럼 4개 제외
            {'Key': 'Data Source', 'Value': 'DDS RTPS PCAP'},
            {'Key': 'Purpose', 'Value': 'Anomaly Detection / ML Training'},
        ])
        metadata.to_excel(writer, sheet_name='Metadata', index=False)
    
    print(f"  ✓ 엑셀 파일 저장 완료!")


def main():
    if len(sys.argv) < 3:
        print("사용법: python scripts/generate_anomaly_detection_excel.py <pcap_file> <output_excel>")
        print()
        print("예시:")
        print("  python scripts/generate_anomaly_detection_excel.py data/shm.pcapng output/anomaly_detection.xlsx")
        sys.exit(1)
    
    pcap_file = sys.argv[1]
    output_file = sys.argv[2]
    
    print("=" * 80)
    print("이상탐지용 엑셀 생성 (13개 PID만 포함)")
    print("=" * 80)
    print(f"입력 파일: {pcap_file}")
    print(f"출력 파일: {output_file}")
    print()
    
    # 1. PCAP 파싱
    print("[1/4] PCAP 파일 파싱 중...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=999999.0, max_packets=None)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage 파싱 완료\n")
    
    # 2. 이상탐지 데이터 추출
    print("[2/4] 이상탐지용 PID 추출 중...")
    rows = extract_anomaly_detection_data(submessages)
    print()
    
    # 3. DataFrame 생성
    print("[3/4] DataFrame 생성 중...")
    result_df = create_anomaly_detection_dataframe(rows)
    print()
    
    # 4. 엑셀 저장
    print("[4/4] 엑셀 파일 저장 중...")
    save_to_excel(result_df, output_file)
    print()
    
    # 최종 요약
    print("=" * 80)
    print("✅ 작업 완료!")
    print("=" * 80)
    print()
    print(f"📊 생성된 데이터:")
    print(f"   • 총 행: {len(result_df):,}개")
    print(f"   • 총 컬럼: {len(result_df.columns)}개")
    print(f"   • 기본 컬럼: 4개 (timestamp, submsg_type, participant_guid, endpoint_guid)")
    print(f"   • PID 컬럼: {len(result_df.columns) - 4}개")
    print()
    print(f"📁 출력 파일: {output_file}")
    print()
    print("💡 엑셀 시트:")
    print("   • Anomaly_Detection_Data: 메인 데이터")
    print("   • Statistics: PID별 통계")
    print("   • Metadata: 데이터셋 정보")
    print()
    print("🎯 다음 단계:")
    print("   1. 엑셀 파일을 pandas로 로드")
    print("   2. Feature engineering 수행")
    print("   3. ML 모델 학습 (Decision Tree, Random Forest, Isolation Forest 등)")
    print("   4. 이상탐지 시스템 구축")


if __name__ == "__main__":
    main()
