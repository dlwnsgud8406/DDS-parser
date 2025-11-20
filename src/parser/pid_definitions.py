#!/usr/bin/env python3
"""
PID (Parameter ID) 정의 및 필드 매핑

RTPS 2.5 표준에 따른 전체 PID 목록 및 각 PID의 필드 구조 정의
"""

from typing import Dict, List, Optional


class PIDDefinition:
    """PID 정의 클래스"""
    
    def __init__(self, pid_id: int, pid_name: str, fields: Optional[List[str]] = None, 
                 tshark_mapping: Optional[Dict[str, str]] = None):
        """
        Args:
            pid_id: PID 16진수 ID (예: 0x0015)
            pid_name: PID 이름 (예: PID_PROTOCOL_VERSION)
            fields: 출력 컬럼 필드 목록 (예: ['major', 'minor'])
            tshark_mapping: tshark JSON 필드 매핑 (예: {'major': 'rtps_rtps_version_major'})
        """
        self.pid_id = pid_id
        self.pid_name = pid_name
        self.fields = fields or []
        self.tshark_mapping = tshark_mapping or {}
    
    @property
    def hex_id(self) -> str:
        """16진수 문자열 반환"""
        return f"0x{self.pid_id:04x}"


# 전체 PID 정의
PID_DEFINITIONS = [
    # ===== Control PIDs =====
    PIDDefinition(0x0000, "PID_PAD"),
    PIDDefinition(0x0001, "PID_SENTINEL"),
    
    # ===== Participant Discovery PIDs =====
    PIDDefinition(
        0x0015, "PID_PROTOCOL_VERSION",
        fields=['major', 'minor'],
        tshark_mapping={
            'major': 'rtps_rtps_version_major',
            'minor': 'rtps_rtps_version_minor'
        }
    ),
    
    PIDDefinition(
        0x0016, "PID_VENDOR_ID",
        fields=['vendorid'],
        tshark_mapping={'vendorid': 'rtps_rtps_vendorId'}
    ),
    
    PIDDefinition(
        0x0050, "PID_PARTICIPANT_GUID",
        fields=['hostid', 'appid', 'instanceid', 'entityid', 'entitykey', 'entitykind'],
        tshark_mapping={
            'hostid': 'rtps_rtps_hostId',
            'appid': 'rtps_rtps_appId',
            'instanceid': 'rtps_rtps_sm_guidPrefix_instanceId',
            'entityid': 'rtps_rtps_sm_guid_entityId'
        }
    ),
    
    PIDDefinition(
        0x0002, "PID_PARTICIPANT_LEASE_DURATION",
        fields=['seconds', 'fraction', 'lease_duration'],
        tshark_mapping={
            'seconds': 'rtps_rtps_param_ntpTime_sec',
            'fraction': 'rtps_rtps_param_ntpTime_fraction'
        }
    ),
    
    PIDDefinition(
        0x0058, "PID_BUILTIN_ENDPOINT_SET",
        fields=['flags'],
        tshark_mapping={'flags': 'rtps_rtps_param_builtin_endpoint_set'}
    ),
    
    # ===== Endpoint Discovery PIDs =====
    PIDDefinition(
        0x0005, "PID_TOPIC_NAME",
        fields=['topic'],
        tshark_mapping={'topic': 'rtps_rtps_param_topicName'}
    ),
    
    PIDDefinition(
        0x0007, "PID_TYPE_NAME",
        fields=['typename'],
        tshark_mapping={'typename': 'rtps_rtps_param_type_name'}
    ),
    
    PIDDefinition(
        0x005A, "PID_ENDPOINT_GUID",
        fields=['hostid', 'appid', 'instanceid', 'entityid', 'entitykey', 'entitykind'],
        tshark_mapping={
            'hostid': 'rtps_param_guid_hostId',
            'appid': 'rtps_param_guid_appId',
            'instanceid': 'rtps_param_guid_instanceId',
            'entityid': 'rtps_param_guid_entityId'
        }
    ),
    
    # ===== QoS Policy PIDs =====
    PIDDefinition(
        0x001A, "PID_RELIABILITY",
        fields=['kind', 'max_blocking_time_sec', 'max_blocking_time_frac'],
        tshark_mapping={
            'kind': 'rtps_reliability_kind',
            'max_blocking_time_sec': 'rtps_rtps_param_ntpTime_sec',
            'max_blocking_time_frac': 'rtps_rtps_param_ntpTime_fraction'
        }
    ),
    
    PIDDefinition(
        0x001D, "PID_DURABILITY",
        fields=['kind'],
        tshark_mapping={'kind': 'rtps_rtps_durability'}
    ),
    
    PIDDefinition(
        0x001E, "PID_DURABILITY_SERVICE",
        fields=['service_cleanup_delay', 'history_kind', 'history_depth', 
                'max_samples', 'max_instances', 'max_samples_per_instance'],
        tshark_mapping={
            'service_cleanup_delay': 'rtps_rtps_durability_service_cleanup_delay',
            'history_kind': 'rtps_rtps_durability_service_history_kind',
            'history_depth': 'rtps_rtps_durability_service_history_depth',
            'max_samples': 'rtps_rtps_durability_service_max_samples',
            'max_instances': 'rtps_rtps_durability_service_max_instances',
            'max_samples_per_instance': 'rtps_rtps_durability_service_max_samples_per_instance'
        }
    ),
    
    PIDDefinition(
        0x0023, "PID_DEADLINE",
        fields=['period_sec', 'period_frac'],
        tshark_mapping={
            'period_sec': 'rtps_rtps_param_ntpTime_sec',
            'period_frac': 'rtps_rtps_param_ntpTime_fraction'
        }
    ),
    
    PIDDefinition(
        0x0027, "PID_LATENCY_BUDGET",
        fields=['duration_sec', 'duration_frac'],
        tshark_mapping={
            'duration_sec': 'rtps_rtps_param_ntpTime_sec',
            'duration_frac': 'rtps_rtps_param_ntpTime_fraction'
        }
    ),
    
    PIDDefinition(
        0x001B, "PID_LIVELINESS",
        fields=['kind', 'lease_duration_sec', 'lease_duration_frac'],
        tshark_mapping={
            'kind': 'rtps_rtps_liveliness_kind',
            'lease_duration_sec': 'rtps_rtps_param_ntpTime_sec',
            'lease_duration_frac': 'rtps_rtps_param_ntpTime_fraction'
        }
    ),
    
    PIDDefinition(
        0x001F, "PID_OWNERSHIP",
        fields=['kind'],
        tshark_mapping={'kind': 'rtps_rtps_ownership'}
    ),
    
    PIDDefinition(
        0x0006, "PID_OWNERSHIP_STRENGTH",
        fields=['strength'],
        tshark_mapping={'strength': 'rtps_rtps_param_strength'}
    ),
    
    PIDDefinition(
        0x002B, "PID_LIFESPAN",
        fields=['duration_sec', 'duration_frac'],
        tshark_mapping={
            'duration_sec': 'rtps_rtps_param_ntpTime_sec',
            'duration_frac': 'rtps_rtps_param_ntpTime_fraction'
        }
    ),
    
    PIDDefinition(
        0x0021, "PID_PRESENTATION",
        fields=['access_scope', 'coherent_access', 'ordered_access'],
        tshark_mapping={
            'access_scope': 'rtps_rtps_presentation_access_scope',
            'coherent_access': 'rtps_rtps_presentation_coherent_access',
            'ordered_access': 'rtps_rtps_presentation_ordered_access'
        }
    ),
    
    PIDDefinition(
        0x0025, "PID_DESTINATION_ORDER",
        fields=['kind'],
        tshark_mapping={'kind': 'rtps_rtps_destination_order'}
    ),
    
    PIDDefinition(
        0x0029, "PID_PARTITION",
        fields=['num_partitions', 'partitions'],
        tshark_mapping={
            'num_partitions': 'rtps_rtps_param_partition_num',
            'partitions': 'rtps_rtps_param_partition'
        }
    ),
    
    PIDDefinition(
        0x0040, "PID_HISTORY",
        fields=['kind', 'depth'],
        tshark_mapping={
            'kind': 'rtps_rtps_history_kind',
            'depth': 'rtps_rtps_history_depth'
        }
    ),
    
    PIDDefinition(
        0x0041, "PID_RESOURCE_LIMIT",
        fields=['max_samples', 'max_instances', 'max_samples_per_instance'],
        tshark_mapping={
            'max_samples': 'rtps_rtps_resource_limit_max_samples',
            'max_instances': 'rtps_rtps_resource_limit_max_instances',
            'max_samples_per_instance': 'rtps_rtps_resource_limit_max_samples_per_instance'
        }
    ),
    
    # ===== User Data PIDs =====
    PIDDefinition(
        0x002C, "PID_USER_DATA",
        fields=['data'],
        tshark_mapping={'data': 'rtps_rtps_param_userData'}
    ),
    
    PIDDefinition(
        0x002D, "PID_GROUP_DATA",
        fields=['data'],
        tshark_mapping={'data': 'rtps_rtps_param_groupData'}
    ),
    
    PIDDefinition(
        0x002E, "PID_TOPIC_DATA",
        fields=['data'],
        tshark_mapping={'data': 'rtps_rtps_param_topicData'}
    ),
    
    # ===== Locator PIDs =====
    PIDDefinition(
        0x002F, "PID_UNICAST_LOCATOR",
        fields=['kind', 'port', 'ipv4'],
        tshark_mapping={
            'kind': 'rtps_rtps_locator_kind',
            'port': 'rtps_rtps_locator_port',
            'ipv4': 'rtps_rtps_locator_ipv4'
        }
    ),
    
    PIDDefinition(
        0x0030, "PID_MULTICAST_LOCATOR",
        fields=['kind', 'port', 'ipv4'],
        tshark_mapping={
            'kind': 'rtps_rtps_locator_kind',
            'port': 'rtps_rtps_locator_port',
            'ipv4': 'rtps_rtps_locator_ipv4'
        }
    ),
    
    PIDDefinition(
        0x0031, "PID_DEFAULT_UNICAST_LOCATOR",
        fields=['kind', 'port', 'ipv4'],
        tshark_mapping={
            'kind': 'rtps_rtps_locator_kind',
            'port': 'rtps_rtps_locator_port',
            'ipv4': 'rtps_rtps_locator_ipv4'
        }
    ),
    
    PIDDefinition(
        0x0032, "PID_METATRAFFIC_UNICAST_LOCATOR",
        fields=['kind', 'port', 'ipv4'],
        tshark_mapping={
            'kind': 'rtps_rtps_locator_kind',
            'port': 'rtps_rtps_locator_port',
            'ipv4': 'rtps_rtps_locator_ipv4'
        }
    ),
    
    PIDDefinition(
        0x0033, "PID_METATRAFFIC_MULTICAST_LOCATOR",
        fields=['kind', 'port', 'ipv4'],
        tshark_mapping={
            'kind': 'rtps_rtps_locator_kind',
            'port': 'rtps_rtps_locator_port',
            'ipv4': 'rtps_rtps_locator_ipv4'
        }
    ),
    
    PIDDefinition(
        0x0048, "PID_DEFAULT_MULTICAST_LOCATOR",
        fields=['kind', 'port', 'ipv4'],
        tshark_mapping={
            'kind': 'rtps_rtps_locator_kind',
            'port': 'rtps_rtps_locator_port',
            'ipv4': 'rtps_rtps_locator_ipv4'
        }
    ),
    
    # ===== Inline QoS PIDs =====
    PIDDefinition(
        0x0070, "PID_KEY_HASH",
        fields=['guid'],
        tshark_mapping={'guid': 'rtps_rtps_guid'}
    ),
    
    PIDDefinition(
        0x0071, "PID_STATUS_INFO",
        fields=['flags'],
        tshark_mapping={'flags': 'rtps_rtps_param_status_info'}
    ),
    
    PIDDefinition(
        0x0043, "PID_EXPECTS_INLINE_QOS",
        fields=['inline_qos'],
        tshark_mapping={'inline_qos': 'rtps_rtps_param_expects_inline_qos'}
    ),
    
    # ===== Type Support PIDs =====
    PIDDefinition(
        0x0060, "PID_TYPE_MAX_SIZE_SERIALIZED",
        fields=['value'],
        tshark_mapping={'value': 'rtps_rtps_param_type_max_size_serialized'}
    ),
    
    PIDDefinition(
        0x0073, "PID_TYPE_CONSISTENCY",
        fields=['kind', 'ignore_sequence_bounds', 'ignore_string_bounds', 
                'ignore_member_names', 'prevent_type_widening', 'force_type_validation',
                'ignore_enum_literal_names'],
        tshark_mapping={
            'kind': 'rtps_rtps_param_type_consistency_kind',
            'ignore_sequence_bounds': 'rtps_rtps_param_ignore_sequence_bounds',
            'ignore_string_bounds': 'rtps_rtps_param_ignore_string_bounds',
            'ignore_member_names': 'rtps_rtps_param_ignore_member_names',
            'prevent_type_widening': 'rtps_rtps_param_prevent_type_widening',
            'force_type_validation': 'rtps_rtps_param_force_type_validation',
            'ignore_enum_literal_names': 'rtps_rtps_param_ignore_enum_literal_names'
        }
    ),
    
    # ===== Content Filter PIDs =====
    PIDDefinition(
        0x0035, "PID_CONTENT_FILTER_PROPERTY",
        fields=['content_filter_topic_name', 'related_topic_name', 
                'filter_class_name', 'filter_expression', 'expression_parameters_num'],
        tshark_mapping={
            'content_filter_topic_name': 'rtps_rtps_param_contentFilterTopicName',
            'related_topic_name': 'rtps_rtps_param_relatedTopicName',
            'filter_class_name': 'rtps_rtps_param_filterClassName',
            'filter_expression': 'rtps_rtps_param_filter_expression',
            'expression_parameters_num': 'rtps_rtps_param_expression_parameters_num'
        }
    ),
    
    PIDDefinition(
        0x0055, "PID_CONTENT_FILTER_INFO",
        fields=['filter_name', 'filter_expression', 'num_channels'],
        tshark_mapping={
            'filter_name': 'rtps_rtps_param_locator_filter_list_filter_name',
            'filter_expression': 'rtps_rtps_param_locator_filter_list_filter_exp',
            'num_channels': 'rtps_rtps_param_locator_filter_list_num_channels'
        }
    ),
    
    PIDDefinition(
        0x0004, "PID_TIME_BASED_FILTER",
        fields=['minimum_separation_sec', 'minimum_separation_frac'],
        tshark_mapping={
            'minimum_separation_sec': 'rtps_rtps_param_ntpTime_sec',
            'minimum_separation_frac': 'rtps_rtps_param_ntpTime_fraction'
        }
    ),
    
    # ===== Additional PIDs =====
    PIDDefinition(
        0x000F, "PID_DOMAIN_ID",
        fields=['domain_id'],
        tshark_mapping={'domain_id': 'rtps_rtps_domain_id'}
    ),
    
    PIDDefinition(
        0x0034, "PID_PARTICIPANT_MANUAL_LIVELINESS_COUNT",
        fields=['count'],
        tshark_mapping={'count': 'rtps_rtps_participant_manual_liveliness_count'}
    ),
    
    PIDDefinition(
        0x0049, "PID_TRANSPORT_PRIORITY",
        fields=['value'],
        tshark_mapping={'value': 'rtps_rtps_param_transport_priority'}
    ),
    
    PIDDefinition(
        0x0052, "PID_GROUP_GUID",
        fields=['guid'],
        tshark_mapping={'guid': 'rtps_rtps_param_group_guid'}
    ),
    
    PIDDefinition(
        0x0056, "PID_COHERENT_SET",
        fields=['sequence_number'],
        tshark_mapping={'sequence_number': 'rtps_rtps_coherent_set_seqnum'}
    ),
    
    PIDDefinition(
        0x0057, "PID_DIRECTED_WRITE",
        fields=['num_readers'],
        tshark_mapping={'num_readers': 'rtps_rtps_directed_write_num_readers'}
    ),
    
    PIDDefinition(
        0x0059, "PID_PROPERTY_LIST",
        fields=['num_properties', 'properties'],
        tshark_mapping={
            'num_properties': 'rtps_rtps_property_list_num_properties',
            'properties': 'rtps_rtps_property_list_property'
        }
    ),
    
    PIDDefinition(
        0x0061, "PID_ORIGINAL_WRITER_INFO",
        fields=['writer_guid', 'sequence_number'],
        tshark_mapping={
            'writer_guid': 'rtps_rtps_original_writer_guid',
            'sequence_number': 'rtps_rtps_original_writer_seqnum'
        }
    ),
    
    PIDDefinition(
        0x0062, "PID_ENTITY_NAME",
        fields=['entity_name'],
        tshark_mapping={'entity_name': 'rtps_rtps_param_entityName'}
    ),
    
    PIDDefinition(
        0x0063, "PID_GROUP_COHERENT_SET",
        fields=['sequence_number'],
        tshark_mapping={'sequence_number': 'rtps_rtps_group_coherent_set_seqnum'}
    ),
    
    PIDDefinition(
        0x0077, "PID_BUILTIN_ENDPOINT_QOS",
        fields=['flags'],
        tshark_mapping={'flags': 'rtps_rtps_param_builtin_endpoint_qos'}
    ),
    
    PIDDefinition(
        0x4014, "PID_DOMAIN_TAG",
        fields=['tag'],
        tshark_mapping={'tag': 'rtps_rtps_domain_tag'}
    ),
]


# PID 조회용 딕셔너리 생성
PID_BY_ID: Dict[int, PIDDefinition] = {pid.pid_id: pid for pid in PID_DEFINITIONS}
PID_BY_HEX: Dict[str, PIDDefinition] = {pid.hex_id: pid for pid in PID_DEFINITIONS}
PID_BY_NAME: Dict[str, PIDDefinition] = {pid.pid_name: pid for pid in PID_DEFINITIONS}


def get_pid_definition(identifier) -> Optional[PIDDefinition]:
    """
    PID 정의 조회
    
    Args:
        identifier: PID ID (int), hex string (str), 또는 name (str)
    
    Returns:
        PIDDefinition 객체 또는 None
    """
    if isinstance(identifier, int):
        return PID_BY_ID.get(identifier)
    elif isinstance(identifier, str):
        if identifier.startswith('0x'):
            return PID_BY_HEX.get(identifier.lower())
        else:
            return PID_BY_NAME.get(identifier)
    return None


def get_all_column_names() -> List[str]:
    """
    모든 PID 필드를 기반으로 컬럼 이름 생성
    
    Returns:
        전체 컬럼 이름 리스트 (PID_NAME_field 형식)
    """
    columns = []
    for pid in sorted(PID_DEFINITIONS, key=lambda p: p.pid_id):
        if not pid.fields:
            # 필드가 없으면 PID 이름만
            columns.append(pid.pid_name)
        else:
            # 필드가 있으면 PID_NAME_field 형식
            for field in pid.fields:
                columns.append(f"{pid.pid_name}_{field}")
    return columns


if __name__ == "__main__":
    print("=" * 80)
    print("PID 정의 목록")
    print("=" * 80)
    print(f"총 {len(PID_DEFINITIONS)}개 PID 정의")
    print()
    
    for pid in PID_DEFINITIONS:
        fields_str = f" ({len(pid.fields)} fields)" if pid.fields else " (no fields)"
        print(f"{pid.hex_id}  {pid.pid_name:<40s}{fields_str}")
    
    print()
    print("=" * 80)
    print(f"총 컬럼 수: {len(get_all_column_names())}")
    print("=" * 80)
