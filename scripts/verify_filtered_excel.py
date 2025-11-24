#!/usr/bin/env python3
"""생성된 Excel 파일 구조 검증"""

import pandas as pd

def verify_structure():
    """필터링된 Excel 구조 검증"""
    
    print("=" * 80)
    print("생성된 Excel 파일 구조 검증")
    print("=" * 80)
    
    # 파일 읽기
    xl_file = pd.ExcelFile('output/anomaly_detection_filtered.xlsx')
    
    print(f"\n시트 개수: {len(xl_file.sheet_names)}")
    print(f"시트 목록: {xl_file.sheet_names}\n")
    
    # 첫 번째 노드 시트 검증
    sheet_name = xl_file.sheet_names[0]
    df = pd.read_excel('output/anomaly_detection_filtered.xlsx', 
                       sheet_name=sheet_name, 
                       nrows=5)
    
    print(f"시트: {sheet_name}")
    print(f"컬럼 개수: {len(df.columns)}")
    print(f"\n컬럼 구조:")
    
    columns = df.columns.tolist()
    for i, col in enumerate(columns):
        print(f"  {i:2d}. {col}")
    
    # Timestamp/Value 쌍 확인
    print("\n첫 3행 데이터 샘플 (처음 6개 컬럼):")
    print(df.iloc[:3, :6])
    
    # PID_TOPIC_NAME 데이터 확인
    if 'PID_TOPIC_NAME' in columns:
        topic_idx = columns.index('PID_TOPIC_NAME')
        print(f"\nPID_TOPIC_NAME 데이터 (컬럼 {topic_idx}, {topic_idx+1}):")
        print(df.iloc[:3, [topic_idx, topic_idx+1]])
    
    # PID_RELIABILITY 데이터 확인
    if 'PID_RELIABILITY' in columns:
        rel_idx = columns.index('PID_RELIABILITY')
        print(f"\nPID_RELIABILITY 데이터 (컬럼 {rel_idx}, {rel_idx+1}):")
        print(df.iloc[:3, [rel_idx, rel_idx+1]])
    
    # 패턴 검증
    print("\nTimestamp/Value 쌍 패턴 검증:")
    pid_cols = [i for i, col in enumerate(columns) if col.startswith('PID_')]
    for i in pid_cols[:5]:  # 처음 5개만
        next_col = columns[i+1] if i+1 < len(columns) else "N/A"
        is_unnamed = 'Unnamed' in str(next_col)
        print(f"  {columns[i]:40s} -> {next_col:20s} {'✓' if is_unnamed else '✗'}")
    
    print("\n" + "=" * 80)
    print("검증 완료!")
    print("=" * 80)

if __name__ == '__main__':
    verify_structure()
