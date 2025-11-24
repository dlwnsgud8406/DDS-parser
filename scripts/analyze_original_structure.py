#!/usr/bin/env python3
"""원본 Excel 파일의 Timestamp/Value 쌍 구조 분석"""

import pandas as pd
import sys

def analyze_structure():
    """원본 Excel 구조 분석"""
    
    # 첫 번째 노드 시트 읽기
    df = pd.read_excel('output/shm_all_fixed.xlsx', 
                       sheet_name='Node_1_010f50af..01000000', 
                       nrows=5)
    
    print("=" * 80)
    print("원본 Excel 구조 분석")
    print("=" * 80)
    
    # 전체 컬럼 목록
    columns = df.columns.tolist()
    print(f"\n총 컬럼 수: {len(columns)}")
    
    # 처음 20개 컬럼 출력
    print("\n처음 20개 컬럼:")
    for i, col in enumerate(columns[:20]):
        print(f"{i:3d}. {col}")
    
    # PID_TOPIC_NAME 위치 찾기
    if 'PID_TOPIC_NAME' in columns:
        topic_idx = columns.index('PID_TOPIC_NAME')
        print(f"\nPID_TOPIC_NAME 위치: {topic_idx}")
        print(f"  컬럼 {topic_idx}: {columns[topic_idx]}")
        print(f"  컬럼 {topic_idx+1}: {columns[topic_idx+1]}")
        
        print("\n샘플 데이터:")
        print(df.iloc[:3, [topic_idx, topic_idx+1]])
    
    # PID_RELIABILITY 위치 찾기
    if 'PID_RELIABILITY' in columns:
        rel_idx = columns.index('PID_RELIABILITY')
        print(f"\nPID_RELIABILITY 위치: {rel_idx}")
        print(f"  컬럼 {rel_idx}: {columns[rel_idx]}")
        print(f"  컬럼 {rel_idx+1}: {columns[rel_idx+1]}")
        
        print("\n샘플 데이터:")
        print(df.iloc[:3, [rel_idx, rel_idx+1]])
    
    # Unnamed 컬럼 패턴 분석
    unnamed_cols = [i for i, col in enumerate(columns) if 'Unnamed' in str(col)]
    print(f"\nUnnamed 컬럼 개수: {len(unnamed_cols)}")
    print(f"위치: {unnamed_cols[:10]}...")  # 처음 10개만
    
    # PID 컬럼 패턴 분석
    pid_cols = [i for i, col in enumerate(columns) if col.startswith('PID_')]
    print(f"\nPID 컬럼 개수: {len(pid_cols)}")
    
    # 패턴 확인: PID 다음에 항상 Unnamed가 오는지
    print("\nPID 컬럼 패턴 검증 (PID 다음에 Unnamed가 오는지):")
    for i in pid_cols[:10]:  # 처음 10개만 확인
        next_col = columns[i+1] if i+1 < len(columns) else "N/A"
        is_unnamed = 'Unnamed' in str(next_col)
        print(f"  {columns[i]:40s} -> {next_col:20s} {'✓' if is_unnamed else '✗'}")

if __name__ == '__main__':
    analyze_structure()
