from abc import ABC, abstractmethod
import pandas as pd

class DataSink(ABC):
    """데이터 출력 추상 인터페이스"""

    @abstractmethod
    def write(self, data: dict):
        """데이터 한 행 쓰기"""
        pass
    @abstractmethod
    def get_result(self):
        """최종 결과 반환"""
        pass

class DataFrameSink(DataSink):
    """Data Frame으로 저장하는 Sink"""
    def __init__(self):
        # Todo 1. self.rows = [] 초기화
        self.rows = []

    def write(self, data: dict):
        # Flatten nested dictionaries (guid, entity) but keep pids nested
        flattened = {}
        for key, value in data.items():
            if key == 'pids':
                # Keep pids nested for PivotTableBuilder
                flattened['pids'] = value
            elif isinstance(value, dict):
                # Flatten other nested dicts (guid, entity)
                flattened.update(value)
            else:
                flattened[key] = value
        self.rows.append(flattened)

    def get_result(self) -> pd.DataFrame:
        return pd.DataFrame(self.rows)
        