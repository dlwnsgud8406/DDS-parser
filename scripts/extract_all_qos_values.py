#!/usr/bin/env python3
"""
PCAP에서 모든 QoS 정책 값 추출

실제 패킷에서 Reliability, Durability 외에도
Deadline, Liveliness, Ownership, Presentation, Partition 등
모든 QoS 정책의 실제 값을 추출합니다.
"""

import sys
from pathlib import Path
from collections import Counter, defaultdict

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor


def extract_all_qos_from_pids(submessages):
    """PIDs에서 모든 QoS 관련 값 추출"""
    
    qos_data = {
        'reliability': Counter(),
        'durability': Counter(),
        'deadline': [],
        'latency_budget': [],
        'liveliness': Counter(),
        'ownership': Counter(),
        'ownership_strength': [],
        'presentation': defaultdict(Counter),
        'partition': defaultdict(int),
        'lifespan': [],
        'time_based_filter': [],
        'history': Counter(),
        'depth': [],
        'transport_priority': [],
    }
    
    topics_with_qos = defaultdict(dict)
    
    for msg in submessages:
        pids = msg.get('pids', {})
        topic = pids.get('PID_TOPIC_NAME_topic', 'unknown')
        
        # 1. Reliability
        reliability = pids.get('PID_RELIABILITY_kind')
        if reliability is not None:
            qos_data['reliability'][reliability] += 1
            topics_with_qos[topic]['reliability'] = reliability
        
        # 2. Durability
        durability = pids.get('PID_DURABILITY_kind')
        if durability is not None:
            qos_data['durability'][durability] += 1
            topics_with_qos[topic]['durability'] = durability
        
        # 3. Deadline
        deadline_sec = pids.get('PID_DEADLINE_period_sec')
        deadline_nanosec = pids.get('PID_DEADLINE_period_nanosec')
        if deadline_sec is not None and deadline_nanosec is not None:
            deadline_val = (deadline_sec, deadline_nanosec)
            qos_data['deadline'].append(deadline_val)
            topics_with_qos[topic]['deadline'] = deadline_val
        
        # 4. Latency Budget
        latency_sec = pids.get('PID_LATENCY_BUDGET_duration_sec')
        latency_nanosec = pids.get('PID_LATENCY_BUDGET_duration_nanosec')
        if latency_sec is not None and latency_nanosec is not None:
            latency_val = (latency_sec, latency_nanosec)
            qos_data['latency_budget'].append(latency_val)
            topics_with_qos[topic]['latency_budget'] = latency_val
        
        # 5. Liveliness
        liveliness_kind = pids.get('PID_LIVELINESS_kind')
        if liveliness_kind is not None:
            qos_data['liveliness'][liveliness_kind] += 1
            topics_with_qos[topic]['liveliness'] = liveliness_kind
        
        # 6. Ownership
        ownership = pids.get('PID_OWNERSHIP_kind')
        if ownership is not None:
            qos_data['ownership'][ownership] += 1
            topics_with_qos[topic]['ownership'] = ownership
        
        # 7. Ownership Strength
        strength = pids.get('PID_OWNERSHIP_STRENGTH_value')
        if strength is not None:
            qos_data['ownership_strength'].append(strength)
            topics_with_qos[topic]['ownership_strength'] = strength
        
        # 8. Presentation
        pres_access_scope = pids.get('PID_PRESENTATION_access_scope')
        pres_coherent = pids.get('PID_PRESENTATION_coherent_access')
        pres_ordered = pids.get('PID_PRESENTATION_ordered_access')
        if pres_access_scope is not None:
            qos_data['presentation']['access_scope'][pres_access_scope] += 1
            topics_with_qos[topic]['presentation_access_scope'] = pres_access_scope
        if pres_coherent is not None:
            qos_data['presentation']['coherent'][pres_coherent] += 1
            topics_with_qos[topic]['presentation_coherent'] = pres_coherent
        if pres_ordered is not None:
            qos_data['presentation']['ordered'][pres_ordered] += 1
            topics_with_qos[topic]['presentation_ordered'] = pres_ordered
        
        # 9. Partition
        partition_name = pids.get('PID_PARTITION_name')
        if partition_name is not None:
            qos_data['partition'][partition_name] += 1
            topics_with_qos[topic]['partition'] = partition_name
        
        # 10. Lifespan
        lifespan_sec = pids.get('PID_LIFESPAN_duration_sec')
        lifespan_nanosec = pids.get('PID_LIFESPAN_duration_nanosec')
        if lifespan_sec is not None and lifespan_nanosec is not None:
            lifespan_val = (lifespan_sec, lifespan_nanosec)
            qos_data['lifespan'].append(lifespan_val)
            topics_with_qos[topic]['lifespan'] = lifespan_val
        
        # 11. Time-Based Filter
        tbf_sec = pids.get('PID_TIME_BASED_FILTER_minimum_separation_sec')
        tbf_nanosec = pids.get('PID_TIME_BASED_FILTER_minimum_separation_nanosec')
        if tbf_sec is not None and tbf_nanosec is not None:
            tbf_val = (tbf_sec, tbf_nanosec)
            qos_data['time_based_filter'].append(tbf_val)
            topics_with_qos[topic]['time_based_filter'] = tbf_val
        
        # 12. History
        history_kind = pids.get('PID_HISTORY_kind')
        history_depth = pids.get('PID_HISTORY_depth')
        if history_kind is not None:
            qos_data['history'][history_kind] += 1
            topics_with_qos[topic]['history_kind'] = history_kind
        if history_depth is not None:
            qos_data['depth'].append(history_depth)
            topics_with_qos[topic]['history_depth'] = history_depth
        
        # 13. Transport Priority
        transport_priority = pids.get('PID_TRANSPORT_PRIORITY_value')
        if transport_priority is not None:
            qos_data['transport_priority'].append(transport_priority)
            topics_with_qos[topic]['transport_priority'] = transport_priority
    
    return qos_data, topics_with_qos


def format_duration(sec, nanosec):
    """Duration을 읽기 쉬운 형식으로 변환"""
    if sec == 0x7FFFFFFF and nanosec == 0xFFFFFFFF:
        return "INFINITE (무한)"
    elif sec == 0 and nanosec == 0:
        return "0 (즉시)"
    else:
        total_ms = sec * 1000 + nanosec / 1_000_000
        if total_ms < 1:
            return f"{nanosec} ns"
        elif total_ms < 1000:
            return f"{total_ms:.1f} ms"
        else:
            return f"{sec}.{nanosec//1_000_000:03d} s"


def main():
    pcap_file = "data/shm.pcapng"
    max_packets = 5000
    
    print("=" * 80)
    print("PCAP에서 모든 QoS 정책 값 추출")
    print("=" * 80)
    print(f"파일: {pcap_file}")
    print(f"최대 패킷: {max_packets:,}")
    print()
    
    # 1. PCAP 파싱
    print("[1/2] PCAP 파싱...")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=max_packets)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage 파싱\n")
    
    # 2. QoS 추출
    print("[2/2] 모든 QoS 정책 추출...")
    qos_data, topics_with_qos = extract_all_qos_from_pids(submessages)
    print(f"  ✓ {len(topics_with_qos)}개 토픽에서 QoS 추출\n")
    
    # 결과 출력
    print("\n" + "=" * 80)
    print("QoS 정책별 값 분석")
    print("=" * 80)
    print()
    
    # 1. Reliability
    print("1️⃣ Reliability (신뢰성)")
    print("-" * 80)
    if qos_data['reliability']:
        for kind, count in qos_data['reliability'].most_common():
            kind_name = {
                0: "BEST_EFFORT",
                1: "RELIABLE"
            }.get(kind, f"UNKNOWN({kind})")
            percentage = (count / sum(qos_data['reliability'].values())) * 100
            print(f"   {kind_name:20s}: {count:4d}개 ({percentage:5.1f}%)")
    else:
        print("   ❌ 데이터 없음 (기본값 사용 추정)")
    print()
    
    # 2. Durability
    print("2️⃣ Durability (지속성)")
    print("-" * 80)
    if qos_data['durability']:
        for kind, count in qos_data['durability'].most_common():
            kind_name = {
                0: "VOLATILE",
                1: "TRANSIENT_LOCAL",
                2: "TRANSIENT",
                3: "PERSISTENT"
            }.get(kind, f"UNKNOWN({kind})")
            percentage = (count / sum(qos_data['durability'].values())) * 100
            print(f"   {kind_name:20s}: {count:4d}개 ({percentage:5.1f}%)")
    else:
        print("   ❌ 데이터 없음 (기본값 사용 추정)")
    print()
    
    # 3. Deadline
    print("3️⃣ Deadline (메시지 최대 주기)")
    print("-" * 80)
    if qos_data['deadline']:
        deadline_counter = Counter(qos_data['deadline'])
        for (sec, nanosec), count in deadline_counter.most_common(10):
            formatted = format_duration(sec, nanosec)
            print(f"   {formatted:30s}: {count:4d}개")
    else:
        print("   ✅ 설정 안 함 (기본값: INFINITE)")
        print("   → 모든 토픽이 기본값 사용 → 호환 문제 없음!")
    print()
    
    # 4. Latency Budget
    print("4️⃣ Latency Budget (지연 시간 힌트)")
    print("-" * 80)
    if qos_data['latency_budget']:
        latency_counter = Counter(qos_data['latency_budget'])
        for (sec, nanosec), count in latency_counter.most_common(10):
            formatted = format_duration(sec, nanosec)
            print(f"   {formatted:30s}: {count:4d}개")
    else:
        print("   ✅ 설정 안 함 (기본값: 0)")
        print("   → 호환성에 영향 없음 (힌트일 뿐)")
    print()
    
    # 5. Liveliness
    print("5️⃣ Liveliness (생존 확인)")
    print("-" * 80)
    if qos_data['liveliness']:
        for kind, count in qos_data['liveliness'].most_common():
            kind_name = {
                0: "AUTOMATIC",
                1: "MANUAL_BY_PARTICIPANT",
                2: "MANUAL_BY_TOPIC"
            }.get(kind, f"UNKNOWN({kind})")
            percentage = (count / sum(qos_data['liveliness'].values())) * 100
            print(f"   {kind_name:25s}: {count:4d}개 ({percentage:5.1f}%)")
    else:
        print("   ✅ 설정 안 함 (기본값: AUTOMATIC)")
        print("   → 모든 토픽이 기본값 사용 → 호환 문제 없음!")
    print()
    
    # 6. Ownership
    print("6️⃣ Ownership (제어권)")
    print("-" * 80)
    if qos_data['ownership']:
        for kind, count in qos_data['ownership'].most_common():
            kind_name = {
                0: "SHARED",
                1: "EXCLUSIVE"
            }.get(kind, f"UNKNOWN({kind})")
            percentage = (count / sum(qos_data['ownership'].values())) * 100
            print(f"   {kind_name:20s}: {count:4d}개 ({percentage:5.1f}%)")
    else:
        print("   ✅ 설정 안 함 (기본값: SHARED)")
        print("   → 모든 토픽이 기본값 사용 → 호환 문제 없음!")
    print()
    
    # 7. Ownership Strength
    print("7️⃣ Ownership Strength (우선순위)")
    print("-" * 80)
    if qos_data['ownership_strength']:
        strength_counter = Counter(qos_data['ownership_strength'])
        for strength, count in strength_counter.most_common(10):
            print(f"   {strength:10d}: {count:4d}개")
    else:
        print("   ✅ 설정 안 함 (기본값: 0)")
        print("   → SHARED Ownership 사용 시 무의미")
    print()
    
    # 8. Presentation
    print("8️⃣ Presentation (순서/그룹화)")
    print("-" * 80)
    if qos_data['presentation']['access_scope']:
        print("   Access Scope:")
        for scope, count in qos_data['presentation']['access_scope'].most_common():
            scope_name = {
                0: "INSTANCE",
                1: "TOPIC",
                2: "GROUP"
            }.get(scope, f"UNKNOWN({scope})")
            print(f"      {scope_name:15s}: {count:4d}개")
        
        if qos_data['presentation']['coherent']:
            print("   Coherent Access:")
            for coherent, count in qos_data['presentation']['coherent'].most_common():
                coherent_name = "TRUE" if coherent else "FALSE"
                print(f"      {coherent_name:15s}: {count:4d}개")
        
        if qos_data['presentation']['ordered']:
            print("   Ordered Access:")
            for ordered, count in qos_data['presentation']['ordered'].most_common():
                ordered_name = "TRUE" if ordered else "FALSE"
                print(f"      {ordered_name:15s}: {count:4d}개")
    else:
        print("   ✅ 설정 안 함 (기본값: INSTANCE, false, false)")
        print("   → 모든 토픽이 기본값 사용 → 호환 문제 없음!")
    print()
    
    # 9. Partition
    print("9️⃣ Partition (네트워크 분리)")
    print("-" * 80)
    if qos_data['partition']:
        for partition, count in sorted(qos_data['partition'].items(), 
                                      key=lambda x: x[1], reverse=True)[:10]:
            print(f"   '{partition}': {count:4d}개")
    else:
        print("   ✅ 설정 안 함 (기본값: 빈 배열)")
        print("   → 모든 토픽이 같은 Partition → 호환 문제 없음!")
    print()
    
    # 10. History
    print("🔟 History & Depth (큐 관리)")
    print("-" * 80)
    if qos_data['history']:
        print("   History Kind:")
        for kind, count in qos_data['history'].most_common():
            kind_name = {
                0: "KEEP_LAST",
                1: "KEEP_ALL"
            }.get(kind, f"UNKNOWN({kind})")
            percentage = (count / sum(qos_data['history'].values())) * 100
            print(f"      {kind_name:15s}: {count:4d}개 ({percentage:5.1f}%)")
    else:
        print("   History Kind:")
        print("      ✅ 설정 안 함 (기본값: KEEP_LAST)")
    
    if qos_data['depth']:
        depth_counter = Counter(qos_data['depth'])
        print("   Depth (큐 크기):")
        for depth, count in depth_counter.most_common(10):
            print(f"      {depth:10d}: {count:4d}개")
    else:
        print("   Depth:")
        print("      ✅ 설정 안 함 (기본값: 10)")
    
    print("   → History/Depth는 호환성에 영향 없음 (각자 관리)")
    print()
    
    # 11-13. 나머지
    print("1️⃣1️⃣ 기타 QoS 정책")
    print("-" * 80)
    
    if qos_data['lifespan']:
        print("   Lifespan (메시지 만료):")
        lifespan_counter = Counter(qos_data['lifespan'])
        for (sec, nanosec), count in list(lifespan_counter.most_common(3)):
            formatted = format_duration(sec, nanosec)
            print(f"      {formatted}: {count}개")
    else:
        print("   Lifespan: ✅ 설정 안 함 (기본값: INFINITE)")
    
    if qos_data['time_based_filter']:
        print("   Time-Based Filter (수신 간격):")
        tbf_counter = Counter(qos_data['time_based_filter'])
        for (sec, nanosec), count in list(tbf_counter.most_common(3)):
            formatted = format_duration(sec, nanosec)
            print(f"      {formatted}: {count}개")
    else:
        print("   Time-Based Filter: ✅ 설정 안 함 (기본값: 0)")
    
    if qos_data['transport_priority']:
        print("   Transport Priority (전송 우선순위):")
        tp_counter = Counter(qos_data['transport_priority'])
        for priority, count in list(tp_counter.most_common(3)):
            print(f"      {priority}: {count}개")
    else:
        print("   Transport Priority: ✅ 설정 안 함 (기본값: 0)")
    
    print()
    
    # 최종 요약
    print("\n" + "=" * 80)
    print("📊 최종 요약")
    print("=" * 80)
    print()
    
    summary = [
        ("Reliability", bool(qos_data['reliability']), True),
        ("Durability", bool(qos_data['durability']), True),
        ("Deadline", bool(qos_data['deadline']), True),
        ("Liveliness", bool(qos_data['liveliness']), True),
        ("Ownership", bool(qos_data['ownership']), True),
        ("Presentation", bool(qos_data['presentation']['access_scope']), True),
        ("Partition", bool(qos_data['partition']), True),
        ("History", bool(qos_data['history']), False),
        ("Lifespan", bool(qos_data['lifespan']), False),
        ("Time-Based Filter", bool(qos_data['time_based_filter']), False),
        ("Transport Priority", bool(qos_data['transport_priority']), False),
    ]
    
    print("🔴 통신 차단 가능 (호환성 확인 필수):")
    for name, has_data, can_block in summary:
        if can_block:
            status = "📊 명시적 설정됨" if has_data else "✅ 기본값 (안전)"
            print(f"   • {name:20s}: {status}")
    
    print()
    print("🟢 통신 차단 안 함 (성능에만 영향):")
    for name, has_data, can_block in summary:
        if not can_block:
            status = "📊 명시적 설정됨" if has_data else "✅ 기본값"
            print(f"   • {name:20s}: {status}")
    
    print()
    print("💡 결론:")
    blocking_set = [name for name, has_data, can_block in summary if can_block and has_data]
    if len(blocking_set) == 2 and 'Reliability' in blocking_set and 'Durability' in blocking_set:
        print("   ✅ Reliability와 Durability만 명시적으로 설정됨")
        print("   ✅ 나머지는 모두 기본값 사용")
        print("   → Reliability + Durability만 확인하면 충분!")
    else:
        print(f"   ⚠️  {len(blocking_set)}개의 QoS가 명시적으로 설정됨:")
        for name in blocking_set:
            print(f"      • {name}")
        print("   → 이 QoS들의 호환성을 모두 확인해야 함!")


if __name__ == "__main__":
    main()
