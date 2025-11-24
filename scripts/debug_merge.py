#!/usr/bin/env python3
"""셀 병합 로직 디버깅"""

import pandas as pd

def debug_merge_logic():
    """병합 로직 테스트"""
    
    # 필터링된 파일 읽기
    df = pd.read_excel('output/anomaly_detection_filtered.xlsx', 
                       sheet_name='Node_1_010f50af..01000000',
                       nrows=20)
    
    print("=" * 80)
    print("병합 로직 디버깅")
    print("=" * 80)
    
    print(f"\nDataFrame 행 수: {len(df)}")
    print(f"\n처음 15행:")
    print(df[['idx', 'TimeInterval']].head(15))
    
    # get_merge_ranges 로직 테스트
    print("\n\nidx 컬럼 병합 범위 계산:")
    
    if 'idx' in df.columns:
        values = df['idx'].values
        print(f"  값 개수: {len(values)}")
        print(f"  처음 10개 값: {values[:10]}")
        
        # 병합 범위 계산 (NaN 처리 포함)
        merge_ranges = []
        start_idx = 0
        current_value = values[0]
        
        for i in range(1, len(values)):
            # NaN 처리
            current_is_nan = pd.isna(current_value)
            next_is_nan = pd.isna(values[i])
            
            values_differ = (current_is_nan != next_is_nan) or \
                           (not current_is_nan and not next_is_nan and values[i] != current_value)
            
            if values_differ:
                if i - start_idx > 1:
                    merge_ranges.append((start_idx + 3, i + 2))
                    print(f"  병합: 행 {start_idx + 3}~{i + 2} (값: {current_value})")
                start_idx = i
                current_value = values[i]
        
        # 마지막 범위
        if len(values) - start_idx > 1:
            merge_ranges.append((start_idx + 3, len(values) + 2))
            print(f"  병합: 행 {start_idx + 3}~{len(values) + 2} (값: {current_value})")
        
        print(f"\n  총 병합 범위: {len(merge_ranges)}")

if __name__ == '__main__':
    debug_merge_logic()
