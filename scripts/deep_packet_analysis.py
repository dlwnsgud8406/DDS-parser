#!/usr/bin/env python3
"""
test1 vs test2 패킷 내용 상세 분석

패킷 레벨, 필드 레벨, 값 레벨에서 모든 차이를 찾아냅니다.
"""

import sys
from pathlib import Path
from collections import defaultdict, Counter
import json

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.packet_source import PcapSource
from src.parser import EnhancedRTPSParser
from src.sink import DataFrameSink
from src.processor import TimeWindowProcessor


def parse_file(pcap_file):
    """PCAP 파일 파싱"""
    print(f"파싱 중: {pcap_file}")
    source = PcapSource(pcap_file)
    parser = EnhancedRTPSParser()
    sink = DataFrameSink()
    processor = TimeWindowProcessor(window_seconds=1.0, max_packets=None)
    
    processor.process_stream(source, parser, sink)
    df = sink.get_result()
    submessages = df.to_dict('records')
    print(f"  ✓ {len(submessages):,}개 submessage\n")
    
    return submessages


def analyze_packet_content_distribution(msgs, label):
    """패킷 내용 분포 분석"""
    print(f"\n{'='*80}")
    print(f"📦 {label} 패킷 내용 분포 분석")
    print('='*80)
    
    # 1. submsg_name별 세부 분석
    print("\n▶ Submessage 타입별 상세 통계:")
    submsg_stats = defaultdict(lambda: {
        'count': 0,
        'has_pids': 0,
        'pid_count': [],
        'has_entity': 0,
        'has_guid': 0,
        'avg_fields': []
    })
    
    for msg in msgs:
        submsg_name = msg.get('submsg_name', 'UNKNOWN')
        submsg_stats[submsg_name]['count'] += 1
        
        # PID 존재 여부
        pids = msg.get('pids', {})
        if pids:
            submsg_stats[submsg_name]['has_pids'] += 1
            submsg_stats[submsg_name]['pid_count'].append(len(pids))
        
        # Entity 정보
        entity = msg.get('entity', {})
        if entity:
            submsg_stats[submsg_name]['has_entity'] += 1
        
        # GUID 정보
        guid = msg.get('guid', {})
        if guid and guid.get('hostId'):
            submsg_stats[submsg_name]['has_guid'] += 1
        
        # 전체 필드 수
        submsg_stats[submsg_name]['avg_fields'].append(len(msg.keys()))
    
    print(f"\n{'타입':<20} {'개수':>8} {'PID비율':>10} {'평균PID':>10} {'Entity%':>10} {'GUID%':>10}")
    print('-'*80)
    for submsg_name in sorted(submsg_stats.keys()):
        stats = submsg_stats[submsg_name]
        count = stats['count']
        pid_ratio = f"{stats['has_pids']/count*100:.1f}%" if count > 0 else "0%"
        avg_pids = f"{sum(stats['pid_count'])/len(stats['pid_count']):.1f}" if stats['pid_count'] else "0"
        entity_pct = f"{stats['has_entity']/count*100:.1f}%" if count > 0 else "0%"
        guid_pct = f"{stats['has_guid']/count*100:.1f}%" if count > 0 else "0%"
        
        print(f"{submsg_name:<20} {count:>8} {pid_ratio:>10} {avg_pids:>10} {entity_pct:>10} {guid_pct:>10}")
    
    # 2. PID 값 분포
    print(f"\n▶ PID 필드 값 분포 (상위 10개):")
    all_pid_values = defaultdict(Counter)
    
    for msg in msgs:
        pids = msg.get('pids', {})
        for key, value in pids.items():
            if value is not None:
                # 값을 문자열로 변환 (해시 가능하게)
                str_val = str(value)[:50]  # 너무 긴 값은 자르기
                all_pid_values[key][str_val] += 1
    
    for pid_key in sorted(all_pid_values.keys())[:10]:
        print(f"\n  {pid_key}:")
        top_values = all_pid_values[pid_key].most_common(5)
        for value, count in top_values:
            print(f"    • {value}: {count}회")
    
    # 3. Entity ID 분포
    print(f"\n▶ Entity ID 분포:")
    entity_ids = Counter()
    entity_kinds = Counter()
    
    for msg in msgs:
        entity = msg.get('entity', {})
        if entity.get('wrEntityId'):
            entity_ids[entity['wrEntityId']] += 1
        if entity.get('rdEntityId'):
            entity_ids[entity['rdEntityId']] += 1
        
        guid = msg.get('guid', {})
        if guid.get('entityId'):
            entity_kinds[guid['entityId']] += 1
    
    print(f"  고유 Entity ID: {len(entity_ids)}개")
    if entity_ids:
        print(f"  상위 5개:")
        for entity_id, count in entity_ids.most_common(5):
            print(f"    • 0x{entity_id:08x}: {count}회")
    
    print(f"\n  GUID Entity ID: {len(entity_kinds)}개")
    if entity_kinds:
        print(f"  상위 5개:")
        for entity_id, count in entity_kinds.most_common(5):
            print(f"    • 0x{entity_id:08x}: {count}회")
    
    # 4. Host ID 및 App ID 분포
    print(f"\n▶ Participant 식별자 분포:")
    host_ids = Counter()
    app_ids = Counter()
    instance_ids = Counter()
    
    for msg in msgs:
        guid = msg.get('guid', {})
        if guid.get('hostId'):
            host_ids[guid['hostId']] += 1
        if guid.get('appId'):
            app_ids[guid['appId']] += 1
        if guid.get('instanceId'):
            instance_ids[guid['instanceId']] += 1
    
    print(f"  고유 Host ID: {len(host_ids)}개")
    for host_id, count in host_ids.most_common(5):
        print(f"    • 0x{host_id:08x}: {count}회")
    
    print(f"\n  고유 App ID: {len(app_ids)}개")
    for app_id, count in app_ids.most_common(5):
        print(f"    • {app_id} (0x{app_id:08x}): {count}회")
    
    print(f"\n  고유 Instance ID: {len(instance_ids)}개")
    for inst_id, count in instance_ids.most_common(5):
        print(f"    • 0x{inst_id:08x}: {count}회")


def compare_specific_messages(test1_msgs, test2_msgs):
    """특정 메시지 타입별 상세 비교"""
    print(f"\n{'='*80}")
    print("🔬 메시지 타입별 상세 비교")
    print('='*80)
    
    # 메시지 타입별로 그룹화
    test1_by_type = defaultdict(list)
    test2_by_type = defaultdict(list)
    
    for msg in test1_msgs:
        submsg_name = msg.get('submsg_name', 'UNKNOWN')
        test1_by_type[submsg_name].append(msg)
    
    for msg in test2_msgs:
        submsg_name = msg.get('submsg_name', 'UNKNOWN')
        test2_by_type[submsg_name].append(msg)
    
    # DATA(p) - Participant Discovery 상세 비교
    print("\n▶ DATA(p) - Participant Discovery 메시지 비교:")
    
    if 'DATA(p)' in test1_by_type and 'DATA(p)' in test2_by_type:
        t1_datap = test1_by_type['DATA(p)']
        t2_datap = test2_by_type['DATA(p)']
        
        print(f"  test1: {len(t1_datap)}개")
        print(f"  test2: {len(t2_datap)}개")
        
        # 첫 번째 DATA(p) 메시지 상세 비교
        if t1_datap and t2_datap:
            print("\n  첫 번째 DATA(p) 메시지 비교:")
            msg1 = t1_datap[0]
            msg2 = t2_datap[0]
            
            # PID 비교
            pids1 = msg1.get('pids', {})
            pids2 = msg2.get('pids', {})
            
            print(f"\n    PID 필드 개수: test1={len(pids1)}, test2={len(pids2)}")
            
            # 차이나는 PID만 출력
            all_pid_keys = set(pids1.keys()) | set(pids2.keys())
            different_pids = []
            
            for key in sorted(all_pid_keys):
                val1 = pids1.get(key)
                val2 = pids2.get(key)
                if val1 != val2:
                    different_pids.append((key, val1, val2))
            
            if different_pids:
                print(f"\n    차이나는 PID 필드: {len(different_pids)}개")
                for key, val1, val2 in different_pids[:15]:
                    v1_str = str(val1)[:40] if val1 else '<None>'
                    v2_str = str(val2)[:40] if val2 else '<None>'
                    print(f"      • {key}:")
                    print(f"        test1: {v1_str}")
                    print(f"        test2: {v2_str}")
    
    # HEARTBEAT 비교
    print("\n▶ HEARTBEAT 메시지 비교:")
    
    if 'HEARTBEAT' in test1_by_type and 'HEARTBEAT' in test2_by_type:
        t1_hb = test1_by_type['HEARTBEAT']
        t2_hb = test2_by_type['HEARTBEAT']
        
        print(f"  test1: {len(t1_hb)}개")
        print(f"  test2: {len(t2_hb)}개")
        print(f"  차이: {len(t1_hb) - len(t2_hb)}개")
        
        # HEARTBEAT의 Entity ID 분포
        t1_entities = Counter(msg.get('guid', {}).get('entityId') for msg in t1_hb)
        t2_entities = Counter(msg.get('guid', {}).get('entityId') for msg in t2_hb)
        
        print(f"\n  test1 Entity ID 분포 (상위 5개):")
        for entity_id, count in t1_entities.most_common(5):
            if entity_id:
                print(f"    • 0x{entity_id:08x}: {count}회")
        
        print(f"\n  test2 Entity ID 분포 (상위 5개):")
        for entity_id, count in t2_entities.most_common(5):
            if entity_id:
                print(f"    • 0x{entity_id:08x}: {count}회")


def compare_locator_info(test1_msgs, test2_msgs):
    """네트워크 위치(Locator) 정보 비교"""
    print(f"\n{'='*80}")
    print("🌐 네트워크 Locator 정보 비교")
    print('='*80)
    
    def extract_locators(msgs):
        locators = {
            'unicast_ports': Counter(),
            'multicast_ports': Counter(),
            'metatraffic_unicast_ports': Counter(),
            'metatraffic_multicast_ports': Counter(),
            'default_unicast_ports': Counter(),
            'default_multicast_ports': Counter(),
            'ipv4_addresses': Counter()
        }
        
        for msg in msgs:
            pids = msg.get('pids', {})
            
            # 각종 포트 수집
            if pids.get('PID_UNICAST_LOCATOR_port'):
                locators['unicast_ports'][pids['PID_UNICAST_LOCATOR_port']] += 1
            if pids.get('PID_MULTICAST_LOCATOR_port'):
                locators['multicast_ports'][pids['PID_MULTICAST_LOCATOR_port']] += 1
            if pids.get('PID_METATRAFFIC_UNICAST_LOCATOR_port'):
                locators['metatraffic_unicast_ports'][pids['PID_METATRAFFIC_UNICAST_LOCATOR_port']] += 1
            if pids.get('PID_METATRAFFIC_MULTICAST_LOCATOR_port'):
                locators['metatraffic_multicast_ports'][pids['PID_METATRAFFIC_MULTICAST_LOCATOR_port']] += 1
            if pids.get('PID_DEFAULT_UNICAST_LOCATOR_port'):
                locators['default_unicast_ports'][pids['PID_DEFAULT_UNICAST_LOCATOR_port']] += 1
            if pids.get('PID_DEFAULT_MULTICAST_LOCATOR_port'):
                locators['default_multicast_ports'][pids['PID_DEFAULT_MULTICAST_LOCATOR_port']] += 1
            
            # IP 주소
            if pids.get('PID_UNICAST_LOCATOR_ipv4'):
                locators['ipv4_addresses'][pids['PID_UNICAST_LOCATOR_ipv4']] += 1
        
        return locators
    
    t1_locators = extract_locators(test1_msgs)
    t2_locators = extract_locators(test2_msgs)
    
    # 포트 비교
    print("\n▶ 포트 번호 비교:")
    
    for locator_type in ['unicast_ports', 'multicast_ports', 'metatraffic_unicast_ports', 
                         'metatraffic_multicast_ports', 'default_unicast_ports', 'default_multicast_ports']:
        t1_ports = t1_locators[locator_type]
        t2_ports = t2_locators[locator_type]
        
        if t1_ports or t2_ports:
            print(f"\n  {locator_type.replace('_', ' ').title()}:")
            
            all_ports = set(t1_ports.keys()) | set(t2_ports.keys())
            for port in sorted(all_ports):
                t1_count = t1_ports.get(port, 0)
                t2_count = t2_ports.get(port, 0)
                
                if t1_count > 0 or t2_count > 0:
                    diff = t2_count - t1_count
                    diff_str = f"({diff:+d})" if diff != 0 else ""
                    print(f"    포트 {port}: test1={t1_count}, test2={t2_count} {diff_str}")
    
    # IP 주소 비교
    print("\n▶ IPv4 주소 분포:")
    all_ips = set(t1_locators['ipv4_addresses'].keys()) | set(t2_locators['ipv4_addresses'].keys())
    
    for ip in sorted(all_ips):
        t1_count = t1_locators['ipv4_addresses'].get(ip, 0)
        t2_count = t2_locators['ipv4_addresses'].get(ip, 0)
        print(f"  {ip}: test1={t1_count}, test2={t2_count}")


def compare_qos_policies(test1_msgs, test2_msgs):
    """QoS 정책 비교"""
    print(f"\n{'='*80}")
    print("⚙️  QoS 정책 비교")
    print('='*80)
    
    qos_fields = [
        'PID_RELIABILITY_max_blocking_time',
        'PID_DURABILITY_kind',
        'PID_LIVELINESS_kind',
        'PID_LIVELINESS_lease_duration',
        'PID_DEADLINE_period',
        'PID_LATENCY_BUDGET_duration',
        'PID_OWNERSHIP_kind',
        'PID_DESTINATION_ORDER_kind',
        'PID_PRESENTATION_access_scope',
    ]
    
    def extract_qos(msgs):
        qos_stats = defaultdict(Counter)
        
        for msg in msgs:
            pids = msg.get('pids', {})
            for qos_field in qos_fields:
                # 필드 이름 변형 처리
                for key in pids.keys():
                    if qos_field in key:
                        value = pids[key]
                        qos_stats[key][str(value)] += 1
        
        return qos_stats
    
    t1_qos = extract_qos(test1_msgs)
    t2_qos = extract_qos(test2_msgs)
    
    all_qos_keys = set(t1_qos.keys()) | set(t2_qos.keys())
    
    if all_qos_keys:
        print("\n발견된 QoS 정책:")
        for qos_key in sorted(all_qos_keys):
            print(f"\n▶ {qos_key}:")
            
            t1_values = t1_qos.get(qos_key, Counter())
            t2_values = t2_qos.get(qos_key, Counter())
            
            all_values = set(t1_values.keys()) | set(t2_values.keys())
            
            for value in sorted(all_values):
                t1_count = t1_values.get(value, 0)
                t2_count = t2_values.get(value, 0)
                
                if t1_count > 0 or t2_count > 0:
                    print(f"  값 {value}: test1={t1_count}, test2={t2_count}")
    else:
        print("\n⚠️  QoS 정책 정보를 찾을 수 없습니다.")


def main():
    print("="*80)
    print("test1 vs test2 패킷 내용 상세 분석")
    print("="*80)
    print()
    
    # 파일 파싱
    test1_msgs = parse_file("data/test1.pcapng")
    test2_msgs = parse_file("data/test2.pcapng")
    
    # 분석 수행
    analyze_packet_content_distribution(test1_msgs, "test1")
    analyze_packet_content_distribution(test2_msgs, "test2")
    
    compare_specific_messages(test1_msgs, test2_msgs)
    compare_locator_info(test1_msgs, test2_msgs)
    compare_qos_policies(test1_msgs, test2_msgs)
    
    print("\n" + "="*80)
    print("✅ 상세 분석 완료!")
    print("="*80)


if __name__ == "__main__":
    main()
