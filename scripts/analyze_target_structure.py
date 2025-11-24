#!/usr/bin/env python3
"""shm_nodes_fixed.xlsx 구조 분석"""

import pandas as pd
from openpyxl import load_workbook

def analyze_target_structure():
    """목표 Excel 구조 분석"""
    
    print("=" * 80)
    print("shm_nodes_fixed.xlsx 구조 분석")
    print("=" * 80)
    
    # pandas로 데이터 읽기
    xl_file = pd.ExcelFile('output/shm_nodes_fixed.xlsx')
    
    print(f"\n시트 개수: {len(xl_file.sheet_names)}")
    print(f"시트 목록: {xl_file.sheet_names}\n")
    
    # 첫 번째 데이터 시트 분석 (Overview 제외)
    data_sheets = [s for s in xl_file.sheet_names if s != 'Overview']
    if not data_sheets:
        print("데이터 시트를 찾을 수 없습니다.")
        return
    
    sheet_name = data_sheets[0]
    df = pd.read_excel('output/shm_nodes_fixed.xlsx', sheet_name=sheet_name, nrows=10)
    
    print(f"시트: {sheet_name}")
    print(f"총 컬럼 수: {len(df.columns)}")
    
    # 컬럼 목록 출력
    print("\n전체 컬럼 목록:")
    columns = df.columns.tolist()
    for i, col in enumerate(columns):
        print(f"  {i:3d}. {col}")
    
    print("\n처음 5행 데이터 (처음 6개 컬럼):")
    print(df.iloc[:5, :6])
    
    # openpyxl로 셀 병합 확인
    print("\n셀 병합 정보:")
    wb = load_workbook('output/shm_nodes_fixed.xlsx')
    ws = wb[sheet_name]
    
    print(f"  병합된 셀 범위 개수: {len(ws.merged_cells.ranges)}")
    print(f"  처음 10개 병합 범위:")
    for i, merged_range in enumerate(list(ws.merged_cells.ranges)[:10]):
        print(f"    {i+1}. {merged_range}")
    
    # PID 컬럼 패턴 분석
    pid_cols = [col for col in columns if col.startswith('PID_')]
    print(f"\nPID 컬럼 개수: {len(pid_cols)}")
    print(f"PID 컬럼 목록:")
    for pid in pid_cols[:15]:  # 처음 15개만
        print(f"  - {pid}")
    
    # Unnamed 컬럼 확인
    unnamed_cols = [col for col in columns if 'Unnamed' in str(col)]
    print(f"\nUnnamed 컬럼 개수: {len(unnamed_cols)}")
    
    # 패턴 확인
    print("\n컬럼 패턴 (PID 다음에 Unnamed?):")
    for i, pid in enumerate(pid_cols[:10]):
        pid_idx = columns.index(pid)
        if pid_idx + 1 < len(columns):
            next_col = columns[pid_idx + 1]
            is_unnamed = 'Unnamed' in str(next_col)
            print(f"  {pid:40s} -> {next_col:20s} {'✓' if is_unnamed else '✗'}")

if __name__ == '__main__':
    analyze_target_structure()
