#!/usr/bin/env python3
"""필터링된 Excel 컬럼 구조 확인"""

import pandas as pd

df = pd.read_excel('output/anomaly_detection_filtered.xlsx', 
                   sheet_name='Unknown',  # SEDP_Endpoints 다음 첫 데이터 시트
                   nrows=5)

print("=" * 80)
print("필터링된 Excel 컬럼 구조")
print("=" * 80)

print(f"\n총 컬럼 수: {len(df.columns)}")
print(f"\n컬럼 목록:")
for i, col in enumerate(df.columns):
    print(f"  {i:2d}. {col}")

print(f"\n처음 3행 데이터 (처음 6개 컬럼):")
print(df.iloc[:3, :6])
