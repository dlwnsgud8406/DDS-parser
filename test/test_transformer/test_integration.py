#!/usr/bin/env python3
"""
Transformer 통합 테스트
실제 PCAP 파일을 사용하여 데이터 변환 검증
"""

import pytest
import sys
from pathlib import Path

# 프로젝트 루트 추가
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.parser import EnhancedRTPSParser
from src.packet_source import PcapSource
from src.transformer import ParticipantGrouper, TimeWindowGenerator, PivotTableBuilder


class TestTransformerIntegration:
    """Transformer 통합 테스트"""
    
    @pytest.fixture
    def sample_pcap(self):
        """샘플 PCAP 경로"""
        pcap_path = Path(__file__).parent.parent.parent / "data" / "participated_57_first_bye.pcapng"
        if not pcap_path.exists():
            pytest.skip(f"PCAP 파일 없음: {pcap_path}")
        return str(pcap_path)
    
    @pytest.fixture
    def parsed_data(self, sample_pcap):
        """파싱된 데이터"""
        parser = EnhancedRTPSParser()
        source = PcapSource(sample_pcap)
        packets = list(source)
        return parser.parse_batch(packets)
    
    def test_participant_grouping(self, parsed_data):
        """Participant별 그룹화 테스트"""
        grouper = ParticipantGrouper()
        grouped = grouper.group_by_participant(parsed_data)
        
        print(f"\n✓ {len(grouped)}개 Participant로 그룹화")
        
        assert len(grouped) == 5, f"Expected 5 participants, got {len(grouped)}"
        
        for participant_id, messages in grouped.items():
            hostId, appId, instanceId = participant_id
            print(f"  - Node {hostId:08x}_{appId:08x}: {len(messages)} messages")
            assert len(messages) > 0
    
    def test_participant_summary(self, parsed_data):
        """Participant 요약 정보 생성 테스트"""
        grouper = ParticipantGrouper()
        grouped = grouper.group_by_participant(parsed_data)
        summary = grouper.get_participant_summary(grouped)
        
        print(f"\n✓ Participant 요약:")
        for info in summary:
            print(f"  - Node: {info['participant_id']}")
            print(f"    Messages: {info['message_count']}")
            print(f"    Entities: {len(info['entities'])}")
            print(f"    Types: {list(info['submsg_types'].keys())[:3]}...")
        
        assert len(summary) == 5
        assert all('message_count' in info for info in summary)
        assert all('entities' in info for info in summary)
    
    def test_participant_name_formatting(self):
        """Participant 이름 포맷팅 테스트"""
        grouper = ParticipantGrouper()
        
        participant_id = (0x010fba3f, 0x3c705e6a, 0x01000000)
        
        # 인덱스 있음
        name_with_idx = grouper.format_participant_name(participant_id, index=1)
        assert name_with_idx == "Node_1_010fba3f_3c705e6a"
        
        # 인덱스 없음
        name_full = grouper.format_participant_name(participant_id)
        assert name_full == "Node_010fba3f_3c705e6a_01000000"
    
    def test_time_window_generation(self, parsed_data):
        """시간 윈도우 생성 테스트"""
        time_gen = TimeWindowGenerator(window_size=1.0)
        windowed = time_gen.generate_windows(parsed_data)
        
        print(f"\n✓ {len(windowed)}개 시간 윈도우 생성:")
        for window_key in sorted(windowed.keys())[:5]:
            print(f"  - {window_key}: {len(windowed[window_key])} messages")
        
        assert len(windowed) > 0
        # 실제 데이터는 14개 윈도우 (불연속적인 타임스탬프)
        assert len(windowed) >= 10
    
    def test_window_stats(self, parsed_data):
        """윈도우 통계 생성 테스트"""
        time_gen = TimeWindowGenerator(window_size=1.0)
        windowed = time_gen.generate_windows(parsed_data)
        stats = time_gen.get_window_stats(windowed)
        
        print(f"\n✓ 윈도우 통계 (처음 3개):")
        for stat in stats[:3]:
            print(f"  - {stat['window']}: {stat['message_count']} messages")
            print(f"    Types: {stat['submsg_types']}")
        
        assert len(stats) == len(windowed)
        assert all('window' in s for s in stats)
        assert all('message_count' in s for s in stats)
    
    def test_time_series_creation(self, parsed_data):
        """시계열 DataFrame 생성 테스트"""
        time_gen = TimeWindowGenerator(window_size=1.0)
        
        # submsg_id를 값으로 하는 시계열
        df = time_gen.create_time_series(parsed_data[:100], value_key='submsg_id')
        
        print(f"\n✓ 시계열 DataFrame 생성:")
        print(f"  - Shape: {df.shape}")
        print(f"  - Index: {df.index.name}")
        print(f"  - Columns: {df.columns.tolist()}")
        
        assert not df.empty
        assert df.index.name == 'timestamp'
        assert 'submsg_id' in df.columns
    
    def test_pivot_table_building(self, parsed_data):
        """Pivot 테이블 생성 테스트"""
        # 단일 Participant 데이터로 테스트
        grouper = ParticipantGrouper()
        grouped = grouper.group_by_participant(parsed_data)
        
        # 첫 번째 Participant 선택
        first_participant = list(grouped.keys())[0]
        first_messages = grouped[first_participant]
        
        builder = PivotTableBuilder(window_size=1.0)
        df = builder.build(first_messages, participant_id=first_participant)
        
        print(f"\n✓ Pivot 테이블 생성:")
        print(f"  - Shape: {df.shape}")
        print(f"  - Columns (처음 5개): {df.columns.tolist()[:5]}")
        print(f"\n  처음 2행:")
        print(df.head(2))
        
        assert not df.empty
        assert 'idx' in df.columns
        assert 'TimeInterval' in df.columns
        # PID 컬럼이 있는지 확인 (_Timestamp, _Value 형식)
        pid_columns = [col for col in df.columns if '_Timestamp' in col or '_Value' in col]
        assert len(pid_columns) > 0, "PID 컬럼이 없습니다"
    
    def test_summary_dataframe(self, parsed_data):
        """요약 DataFrame 생성 테스트 (Overview 시트용)"""
        grouper = ParticipantGrouper()
        grouped = grouper.group_by_participant(parsed_data)
        
        builder = PivotTableBuilder()
        df_summary = builder.build_summary(grouped)
        
        print(f"\n✓ Overview DataFrame 생성:")
        print(df_summary)
        
        assert not df_summary.empty
        assert len(df_summary) == 5  # 5개 Participant
        assert 'Node' in df_summary.columns
        assert 'HostId' in df_summary.columns
        assert 'MessageCount' in df_summary.columns
        assert 'EntityCount' in df_summary.columns
    
    def test_full_pipeline(self, sample_pcap):
        """전체 파이프라인 테스트 (Parse → Group → Transform)"""
        print(f"\n{'='*70}")
        print("전체 변환 파이프라인 테스트")
        print(f"{'='*70}\n")
        
        # 1. Parse
        print("1. 파싱 중...")
        parser = EnhancedRTPSParser()
        source = PcapSource(sample_pcap)
        packets = list(source)
        submessages = parser.parse_batch(packets)
        print(f"   ✓ {len(packets)} packets → {len(submessages)} submessages")
        
        # 2. Group by Participant
        print("\n2. Participant별 그룹화 중...")
        grouper = ParticipantGrouper()
        grouped = grouper.group_by_participant(submessages)
        print(f"   ✓ {len(grouped)} participants")
        
        # 3. Transform each Participant
        print("\n3. 각 Participant별 Pivot 테이블 생성 중...")
        builder = PivotTableBuilder(window_size=1.0)
        
        participant_dataframes = {}
        for idx, (participant_id, messages) in enumerate(grouped.items(), start=1):
            df = builder.build(messages, participant_id=participant_id)
            node_name = grouper.format_participant_name(participant_id, index=idx)
            participant_dataframes[node_name] = df
            print(f"   ✓ {node_name}: {df.shape}")
        
        # 4. Create Overview
        print("\n4. Overview 생성 중...")
        df_overview = builder.build_summary(grouped)
        print(f"   ✓ Overview: {df_overview.shape}")
        
        print(f"\n{'='*70}")
        print("✅ 전체 파이프라인 성공!")
        print(f"{'='*70}")
        
        # 검증
        assert len(participant_dataframes) == 5
        assert not df_overview.empty
        
        # 모든 DataFrame이 필수 컬럼을 가지는지 확인
        for node_name, df in participant_dataframes.items():
            assert 'idx' in df.columns, f"{node_name}에 idx 컬럼 없음"
            assert 'TimeInterval' in df.columns, f"{node_name}에 TimeInterval 컬럼 없음"


# pytest 실행용
if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
