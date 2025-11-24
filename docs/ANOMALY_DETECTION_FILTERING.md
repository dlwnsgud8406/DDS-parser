# Anomaly Detection을 위한 Excel 필터링

## 개요

원본 Excel 파일(`shm_all_fixed.xlsx`)에서 ML 기반 anomaly detection에 필요한 13개 PID만 추출하여 새로운 Excel 파일을 생성합니다.

## 주요 특징

- **원본 구조 완전 유지**: 8개 노드 시트 + Timestamp/Value 쌍 구조
- **74.1% 컬럼 감소**: 108개 → 28개 컬럼
- **호환성 보장**: 기존 분석 도구와 완벽 호환

## 필터링 스크립트

### `scripts/filter_anomaly_pids_from_excel.py`

원본 Excel에서 필요한 13개 PID만 추출하는 스크립트입니다.

**사용법**:
```bash
python scripts/filter_anomaly_pids_from_excel.py
```

**입력**: `output/shm_all_fixed.xlsx`  
**출력**: `output/anomaly_detection_filtered.xlsx`

## 포함된 13개 PID

1. **PID_TOPIC_NAME** (0x0005) - Topic 이름
2. **PID_TYPE_NAME** (0x0007) - 메시지 타입
3. **PID_RELIABILITY** (0x001A) - Reliable/Best-Effort
4. **PID_DURABILITY** (0x001D) - Volatile/Transient
5. **PID_LIVELINESS** (0x001B) - Liveness 정책
6. **PID_DEADLINE** (0x0023) - 메시지 주기
7. **PID_LATENCY_BUDGET** (0x0027) - 지연 허용치
8. **PID_DESTINATION_ORDER** (0x0025) - 메시지 순서
9. **PID_USER_DATA** (0x002C) - 노드 사용자 데이터
10. **PID_HISTORY** (0x0040) - Keep last/all
11. **PID_RESOURCE_LIMIT** (0x0041) - 버퍼 제한
12. **PID_TYPE_MAX_SIZE_SERIALIZED** (0x0061) - 최대 메시지 크기
13. **PID_STATUS_INFO** (0x0071) - 상태 정보

## 구조 비교

### 원본 (`shm_all_fixed.xlsx`)
- **시트**: 9개 (Overview + 8개 노드)
- **컬럼**: 108개
- **구조**: idx + TimeInterval + 53 PID 쌍

### 필터링 (`anomaly_detection_filtered.xlsx`)
- **시트**: 8개 (노드만, Overview 제외)
- **컬럼**: 28개
- **구조**: idx + TimeInterval + 13 PID 쌍
- **감소율**: 74.1%

## 검증 스크립트

### `scripts/verify_filtered_excel.py`

생성된 Excel 파일의 구조를 검증합니다.

```bash
python scripts/verify_filtered_excel.py
```

### `scripts/compare_excel_structures.py`

원본과 필터링된 Excel의 구조를 비교합니다.

```bash
python scripts/compare_excel_structures.py
```

## 구조 호환성

✓ 노드 시트 일치  
✓ 기본 컬럼 존재 (idx, TimeInterval)  
✓ Timestamp/Value 쌍 유지  
✓ PID-Unnamed 쌍 패턴 유지

## 데이터 커버리지 (전체 PCAP 기준)

- **PID_TOPIC_NAME**: 114,853 (98.14%)
- **PID_LIVELINESS**: 12,947
- **PID_RELIABILITY**: 10,082
- **PID_DEADLINE**: 10,082
- **PID_LATENCY_BUDGET**: 10,082
- **PID_DURABILITY**: 2,865
- **PID_DESTINATION_ORDER**: 2,865
- **PID_USER_DATA**: 2,176
- **PID_TYPE_MAX_SIZE_SERIALIZED**: 1,601

9/13 PIDs에 데이터가 존재합니다.

## ML 활용

이 필터링된 Excel 파일은 다음과 같은 anomaly detection 작업에 사용할 수 있습니다:

- Decision Tree 기반 이상 탐지
- Random Forest 분류
- Isolation Forest 이상 탐지
- AutoEncoder 기반 이상 탐지

필요 없는 80개 컬럼을 제거하여 모델 학습 속도와 정확도를 향상시킬 수 있습니다.
