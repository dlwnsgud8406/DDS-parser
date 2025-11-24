#!/usr/bin/env python3
"""shm_nodes_fixed.xlsx 전체 시트 구조 비교"""

import pandas as pd

def compare_all_sheets():
    """모든 시트 구조 비교"""
    
    print("=" * 80)
    print("shm_nodes_fixed.xlsx 모든 시트 구조")
    print("=" * 80)
    
    xl_file = pd.ExcelFile('output/shm_nodes_fixed.xlsx')
    
    for sheet_name in xl_file.sheet_names:
        if sheet_name == 'Overview':
            continue
            
        df = pd.read_excel('output/shm_nodes_fixed.xlsx', sheet_name=sheet_name, nrows=3)
        
        print(f"\n[{sheet_name}]")
        print(f"  컬럼 수: {len(df.columns)}")
        print(f"  컬럼: {', '.join(df.columns.tolist())}")
        print(f"  행 수: {len(df)}")

if __name__ == '__main__':
    compare_all_sheets()
