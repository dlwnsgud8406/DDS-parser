# ROS2 QoS 호환성 테스트 가이드

## 개요

같은 토픽을 사용하는 Publisher와 Subscriber가 **서로 다른 QoS 설정**을 가질 때 통신이 되는지 안 되는지 직접 테스트할 수 있습니다.

## 테스트 방법

### 방법 1: 자동 테스트 (추천)

모든 QoS 조합을 자동으로 테스트합니다:

```bash
python scripts/test_qos_compatibility.py
```

**실행 시간**: 약 40초 (6가지 시나리오 × 5초 + 대기 시간)

**테스트 시나리오**:
1. ✅ RELIABLE → RELIABLE (성공 예상)
2. ✅ RELIABLE → BEST_EFFORT (성공 예상)
3. ❌ BEST_EFFORT → RELIABLE (실패 예상)
4. ✅ BEST_EFFORT → BEST_EFFORT (성공 예상)
5. ❌ VOLATILE → TRANSIENT_LOCAL (실패 예상)
6. ✅ TRANSIENT_LOCAL → VOLATILE (성공 예상)

---

### 방법 2: 수동 테스트 (터미널 2개 사용)

#### 테스트 1: RELIABLE ↔ RELIABLE (✅ 호환)

**터미널 1 (Publisher)**:
```bash
python scripts/qos_test_publisher.py --reliability RELIABLE --durability VOLATILE
```

**터미널 2 (Subscriber)**:
```bash
python scripts/qos_test_subscriber.py --reliability RELIABLE --durability VOLATILE
```

**예상 결과**: ✅ 메시지 수신 성공

---

#### 테스트 2: RELIABLE → BEST_EFFORT (✅ 호환)

**터미널 1 (Publisher)**:
```bash
python scripts/qos_test_publisher.py --reliability RELIABLE --durability VOLATILE
```

**터미널 2 (Subscriber)**:
```bash
python scripts/qos_test_subscriber.py --reliability BEST_EFFORT --durability VOLATILE
```

**예상 결과**: ✅ 메시지 수신 성공
**이유**: Publisher가 더 강한 QoS(RELIABLE)를 제공하므로 호환됨

---

#### 테스트 3: BEST_EFFORT → RELIABLE (❌ 비호환)

**터미널 1 (Publisher)**:
```bash
python scripts/qos_test_publisher.py --reliability BEST_EFFORT --durability VOLATILE
```

**터미널 2 (Subscriber)**:
```bash
python scripts/qos_test_subscriber.py --reliability RELIABLE --durability VOLATILE
```

**예상 결과**: ❌ 메시지 수신 실패
**이유**: Subscriber가 RELIABLE을 요구하지만 Publisher는 BEST_EFFORT만 제공

---

#### 테스트 4: BEST_EFFORT ↔ BEST_EFFORT (✅ 호환)

**터미널 1 (Publisher)**:
```bash
python scripts/qos_test_publisher.py --reliability BEST_EFFORT --durability VOLATILE
```

**터미널 2 (Subscriber)**:
```bash
python scripts/qos_test_subscriber.py --reliability BEST_EFFORT --durability VOLATILE
```

**예상 결과**: ✅ 메시지 수신 성공

---

#### 테스트 5: VOLATILE → TRANSIENT_LOCAL (❌ 비호환)

**터미널 1 (Publisher)**:
```bash
python scripts/qos_test_publisher.py --reliability RELIABLE --durability VOLATILE
```

**터미널 2 (Subscriber)**:
```bash
python scripts/qos_test_subscriber.py --reliability RELIABLE --durability TRANSIENT_LOCAL
```

**예상 결과**: ❌ 메시지 수신 실패
**이유**: Subscriber가 과거 데이터(TRANSIENT_LOCAL)를 요구하지만 Publisher는 현재 데이터(VOLATILE)만 제공

---

#### 테스트 6: TRANSIENT_LOCAL → VOLATILE (✅ 호환)

**터미널 1 (Publisher)**:
```bash
python scripts/qos_test_publisher.py --reliability RELIABLE --durability TRANSIENT_LOCAL
```

**터미널 2 (Subscriber)**:
```bash
python scripts/qos_test_subscriber.py --reliability RELIABLE --durability VOLATILE
```

**예상 결과**: ✅ 메시지 수신 성공
**이유**: Publisher가 더 강한 QoS(TRANSIENT_LOCAL)를 제공하므로 호환됨

---

## QoS 호환성 규칙

### 1. Reliability (신뢰성)

| Publisher | Subscriber | 호환 여부 | 이유 |
|-----------|-----------|----------|------|
| RELIABLE | RELIABLE | ✅ 호환 | 완벽한 일치 |
| RELIABLE | BEST_EFFORT | ✅ 호환 | Publisher가 더 강함 |
| BEST_EFFORT | RELIABLE | ❌ 비호환 | Subscriber 요구사항 미충족 |
| BEST_EFFORT | BEST_EFFORT | ✅ 호환 | 완벽한 일치 |

**핵심**: Publisher의 Reliability ≥ Subscriber의 Reliability

---

### 2. Durability (내구성)

| Publisher | Subscriber | 호환 여부 | 이유 |
|-----------|-----------|----------|------|
| TRANSIENT_LOCAL | TRANSIENT_LOCAL | ✅ 호환 | 완벽한 일치 |
| TRANSIENT_LOCAL | VOLATILE | ✅ 호환 | Publisher가 더 강함 |
| VOLATILE | TRANSIENT_LOCAL | ❌ 비호환 | Subscriber 요구사항 미충족 |
| VOLATILE | VOLATILE | ✅ 호환 | 완벽한 일치 |

**핵심**: Publisher의 Durability ≥ Subscriber의 Durability

---

## 실제 사용 예시

### 예시 1: 센서 데이터 (/scan)
```
Publisher: BEST_EFFORT + VOLATILE  (실시간, 빠른 전송)
Subscriber: BEST_EFFORT + VOLATILE (실시간, 최신 데이터만)
→ ✅ 호환
```

### 예시 2: 지도 데이터 (/map)
```
Publisher: RELIABLE + TRANSIENT_LOCAL  (신뢰성, 과거 데이터 보존)
Subscriber: RELIABLE + VOLATILE       (신뢰성, 현재 데이터만)
→ ✅ 호환 (Publisher가 더 강함)
```

### 예시 3: 잘못된 설정
```
Publisher: BEST_EFFORT + VOLATILE     (빠른 전송)
Subscriber: RELIABLE + TRANSIENT_LOCAL (신뢰성 + 과거 데이터 요구)
→ ❌ 비호환 (통신 불가)
```

---

## 디버깅 팁

### 통신이 안 될 때 확인할 것

1. **QoS 호환성 확인**:
   ```bash
   ros2 topic info /qos_test_topic -v
   ```
   Publisher와 Subscriber의 QoS 설정을 확인

2. **연결 상태 확인**:
   ```bash
   ros2 node list
   ros2 topic list
   ros2 topic echo /qos_test_topic
   ```

3. **경고 메시지 확인**:
   ROS2는 QoS 불일치 시 경고를 출력합니다:
   ```
   [WARN] [rmw_fastrtps_cpp]: New publisher discovered on topic '/qos_test_topic', 
   but QoS compatibility was not met
   ```

---

## 참고

- **DDS 표준**: QoS 정책은 OMG DDS 표준에서 정의
- **ROS2 문서**: [QoS 설정 가이드](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)
- **호환성 매트릭스**: Publisher QoS ≥ Subscriber QoS (더 강한 보장 제공)

---

## 요약

| 규칙 | 설명 |
|------|------|
| 🔑 **핵심 원칙** | Publisher의 QoS ≥ Subscriber의 QoS |
| ✅ **호환 OK** | RELIABLE → BEST_EFFORT, TRANSIENT_LOCAL → VOLATILE |
| ❌ **호환 NO** | BEST_EFFORT → RELIABLE, VOLATILE → TRANSIENT_LOCAL |
| 💡 **팁** | 중요한 데이터는 Publisher를 강하게(RELIABLE + TRANSIENT_LOCAL) |
