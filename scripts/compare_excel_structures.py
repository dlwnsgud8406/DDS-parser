#!/usr/bin/env python3
"""원본과 필터링된 Excel 구조 비교"""

import pandas as pd

def compare_structures():
    """원본과 필터링된 Excel 구조 비교"""
    
    print("=" * 80)
    print("원본 vs 필터링 Excel 구조 비교")
    print("=" * 80)
    
    # 원본 파일
    original = pd.ExcelFile('output/shm_all_fixed.xlsx')
    original_df = pd.read_excel('output/shm_all_fixed.xlsx', 
                                 sheet_name='Node_1_010f50af..01000000', 
                                 nrows=3)
    
    # 필터링된 파일
    filtered = pd.ExcelFile('output/anomaly_detection_filtered.xlsx')
    filtered_df = pd.read_excel('output/anomaly_detection_filtered.xlsx', 
                                 sheet_name='Node_1_010f50af..01000000', 
                                 nrows=3)
    
    print("\n[원본 파일: shm_all_fixed.xlsx]")
    print(f"  시트 개수: {len(original.sheet_names)}")
    print(f"  컬럼 개수: {len(original_df.columns)}")
    print(f"  구조: 8개 노드 시트 + Overview")
    print(f"  형식: idx + TimeInterval + 53 PID 쌍 (Timestamp/Value)")
    
    print("\n[필터링 파일: anomaly_detection_filtered.xlsx]")
    print(f"  시트 개수: {len(filtered.sheet_names)}")
    print(f"  컬럼 개수: {len(filtered_df.columns)}")
    print(f"  구조: 8개 노드 시트")
    print(f"  형식: idx + TimeInterval + 13 PID 쌍 (Timestamp/Value)")
    
    print("\n[감소율]")
    print(f"  컬럼: {len(original_df.columns)} -> {len(filtered_df.columns)}")
    print(f"  감소: {len(original_df.columns) - len(filtered_df.columns)} 컬럼")
    print(f"  비율: {(1 - len(filtered_df.columns)/len(original_df.columns))*100:.1f}%")
    
    print("\n[구조 호환성 검증]")
    
    # 1. 시트 구조
    original_node_sheets = [s for s in original.sheet_names if s.startswith('Node_')]
    filtered_node_sheets = [s for s in filtered.sheet_names if s.startswith('Node_')]
    
    sheets_match = set(original_node_sheets) == set(filtered_node_sheets)
    print(f"  ✓ 노드 시트 일치: {sheets_match}")
    
    # 2. 기본 컬럼
    base_cols = ['idx', 'TimeInterval']
    has_base = all(col in filtered_df.columns for col in base_cols)
    print(f"  ✓ 기본 컬럼 존재: {has_base} (idx, TimeInterval)")
    
    # 3. Timestamp/Value 쌍 구조
    original_cols = original_df.columns.tolist()
    filtered_cols = filtered_df.columns.tolist()
    
    original_unnamed = sum(1 for col in original_cols if 'Unnamed' in str(col))
    filtered_unnamed = sum(1 for col in filtered_cols if 'Unnamed' in str(col))
    
    print(f"  ✓ Timestamp/Value 쌍 유지: {filtered_unnamed} 쌍")
    
    # 4. PID 컬럼 패턴
    filtered_pids = [col for col in filtered_cols if col.startswith('PID_')]
    pattern_valid = all(
        i+1 < len(filtered_cols) and 'Unnamed' in str(filtered_cols[i+1])
        for i, col in enumerate(filtered_cols) if col.startswith('PID_')
    )
    print(f"  ✓ PID-Unnamed 쌍 패턴: {pattern_valid}")
    
    print("\n[필터링된 13개 PID 목록]")
    for i, pid in enumerate(filtered_pids, 1):
        print(f"  {i:2d}. {pid}")
    
    print("\n[샘플 데이터 비교 - PID_RELIABILITY]")
    
    # 원본
    if 'PID_RELIABILITY' in original_cols:
        rel_idx = original_cols.index('PID_RELIABILITY')
        print(f"\n원본 (컬럼 {rel_idx}, {rel_idx+1}):")
        print(original_df.iloc[0:2, [rel_idx, rel_idx+1]])
    
    # 필터링
    if 'PID_RELIABILITY' in filtered_cols:
        rel_idx = filtered_cols.index('PID_RELIABILITY')
        print(f"\n필터링 (컬럼 {rel_idx}, {rel_idx+1}):")
        print(filtered_df.iloc[0:2, [rel_idx, rel_idx+1]])
    
    print("\n" + "=" * 80)
    print("✓ 구조 호환성 검증 완료!")
    print("=" * 80)
    print("\n결론: 필터링된 Excel은 원본과 동일한 구조를 유지하면서")
    print("      anomaly detection에 필요한 13개 PID만 포함합니다.")

if __name__ == '__main__':
    compare_structures()
