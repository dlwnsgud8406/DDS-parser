"""
DDS 핵심 개념: Topic Name, Type Name, QoS
"""

print("=" * 80)
print("📚 DDS 3대 핵심 개념")
print("=" * 80)
print()

print("1️⃣ Topic Name (토픽 이름)")
print("-" * 80)
print("""
역할: "어떤 데이터?" - 데이터 채널 식별

예시:
  - "rt/cmd_vel"         : 로봇 속도 명령
  - "rt/imu/yaw"         : IMU yaw 각도
  - "ros_discovery_info" : ROS 노드 discovery 정보

비유: TV 채널 이름
  - MBC, KBS, SBS ... (채널 이름으로 선택)
  
Pub-Sub 매칭:
  ✅ Publisher와 Subscriber가 같은 Topic Name이면 연결됨
  ❌ 다르면 연결 안 됨
  
특징:
  - 재시작해도 변하지 않음 (코드에 하드코딩)
  - 이게 "계약서" 역할!
""")

print("\n2️⃣ Type Name (타입 이름)")
print("-" * 80)
print("""
역할: "어떤 형식?" - 데이터 구조 정의

예시:
  - "geometry_msgs::msg::dds_::Twist_"
  - "sensor_msgs::msg::dds_::Imu_"
  - "std_msgs::msg::dds_::String_"

비유: 파일 포맷
  - .jpg, .png, .pdf ... (같은 포맷이어야 읽을 수 있음)
  
데이터 구조:
  struct Twist {
      linear: {x, y, z}
      angular: {x, y, z}
  }

Pub-Sub 매칭:
  ✅ Publisher와 Subscriber의 Type이 같아야 통신 가능
  ❌ 다르면 deserialize(역직렬화) 실패
  
특징:
  - Topic Name과 함께 체크됨
  - 같은 토픽도 Type이 다르면 연결 안 됨!
""")

print("\n3️⃣ QoS (Quality of Service)")
print("-" * 80)
print("""
역할: "어떻게 전송?" - 통신 품질/방식 설정

주요 QoS 정책:

📦 Reliability (신뢰성):
  - RELIABLE:   데이터 손실 없이 반드시 전달 (TCP 스타일)
  - BEST_EFFORT: 빠르게 전송, 손실 허용 (UDP 스타일)
  
  예시:
    - 로봇 명령 (cmd_vel):  RELIABLE (중요!)
    - 센서 데이터 (imu):     BEST_EFFORT (빠르게!)

💾 Durability (영속성):
  - VOLATILE:   실시간만 (지금 연결된 Subscriber에게만)
  - TRANSIENT_LOCAL: Late-joiner에게도 전달 (최근 데이터 저장)
  
  예시:
    - 실시간 센서: VOLATILE
    - 설정 정보:   TRANSIENT_LOCAL

👑 Ownership (소유권):
  - SHARED:     여러 Publisher 가능
  - EXCLUSIVE:  단 하나의 Publisher만 (우선순위로 결정)

⏱️ Deadline, Lifespan, History, ...
  - 데이터 유효 시간, 버퍼 크기 등 세부 설정

Pub-Sub 매칭:
  ✅ QoS가 호환되어야 연결됨
  ❌ 예: Publisher=RELIABLE, Subscriber=BEST_EFFORT → 연결 안 됨!
  
특징:
  - Topic/Type이 같아도 QoS가 안 맞으면 통신 불가!
  - 유연한 통신 제어 가능
""")

print("\n" + "=" * 80)
print("🎯 3가지가 모두 맞아야 통신 성공!")
print("=" * 80)
print()
print("""
Publisher A:
  ✓ Topic:  "rt/cmd_vel"
  ✓ Type:   "Twist"
  ✓ QoS:    RELIABLE, VOLATILE

Subscriber X:
  ✓ Topic:  "rt/cmd_vel"       ← 같음! ✅
  ✓ Type:   "Twist"             ← 같음! ✅
  ✓ QoS:    RELIABLE, VOLATILE  ← 호환! ✅
  
→ 연결 성공! 🎉

Subscriber Y:
  ✓ Topic:  "rt/cmd_vel"       ← 같음! ✅
  ✓ Type:   "String"            ← 다름! ❌
  
→ 연결 실패! (Type 불일치)

Subscriber Z:
  ✓ Topic:  "rt/cmd_vel"       ← 같음! ✅
  ✓ Type:   "Twist"             ← 같음! ✅
  ✓ QoS:    BEST_EFFORT         ← 불일치! ❌
  
→ 연결 실패! (QoS 호환 불가)
""")

print("=" * 80)
print("📊 SEDP에서 전달되는 정보")
print("=" * 80)
print()
print("""
SEDP Discovery 패킷:
┌────────────────────────────────────────────┐
│ PID_TOPIC_NAME:    "rt/cmd_vel"           │ ← Topic
│ PID_TYPE_NAME:     "Twist"                │ ← Type
│ PID_RELIABILITY:   RELIABLE               │ ← QoS
│ PID_DURABILITY:    VOLATILE               │ ← QoS
│ PID_OWNERSHIP:     SHARED                 │ ← QoS
│ PID_KEY_HASH:      (host, appId, ...)    │ ← GUID
└────────────────────────────────────────────┘

Subscriber는 이 3가지를 확인:
  1. Topic 이름이 내가 원하는 것?
  2. Type이 내가 예상하는 것?
  3. QoS가 내 설정과 호환되는 것?
  
→ 모두 OK면 GUID 저장하고 연결!
""")

print("=" * 80)
print("🔄 재시작 시 동작")
print("=" * 80)
print()
print("""
Publisher 재시작:
  - appId:    바뀜! (0xAAA → 0xBBB)
  - Topic:    안 바뀜! ("rt/cmd_vel")
  - Type:     안 바뀜! ("Twist")
  - QoS:      안 바뀜! (RELIABLE)

Subscriber 판단:
  1. Topic "rt/cmd_vel" 확인 → 내가 구독 중! ✅
  2. Type "Twist" 확인 → 내 타입과 동일! ✅
  3. QoS RELIABLE 확인 → 호환됨! ✅
  4. → 새 GUID (0xBBB) 저장!

→ 자동으로 재연결! 🔄
""")

print("=" * 80)
print("💡 비유로 정리")
print("=" * 80)
print()
print("""
우편 배송 시스템:

📍 Topic Name = 주소
   - "서울시 강남구 테헤란로 123번지"
   - 같은 주소여야 배달됨

📦 Type Name = 상자 규격
   - "소형 상자", "대형 상자", "냉장 배송"
   - 맞는 상자로 보내야 받을 수 있음

🚚 QoS = 배송 방식
   - "등기 배송" (RELIABLE)
   - "일반 우편" (BEST_EFFORT)
   - "당일 배송" (Deadline)

→ 3가지가 모두 맞아야 배송 성공!
""")

