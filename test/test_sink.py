import pytest
from src.sink import DataSink, DataFrameSink
import pandas as pd


def test_dataframe_sink_write():
    """데이터 쓰기 테스트"""
    sink = DataFrameSink()
    
    sink.write({'a': 1, 'b': 2})
    sink.write({'a': 3, 'b': 4})
    
    df = sink.get_result()
    
    assert len(df) == 2, "행 개수가 맞지 않음"
    assert list(df.columns) == ['a', 'b'], "컬럼이 맞지 않음"
    assert df['a'].tolist() == [1, 3], "a 컬럼 값이 맞지 않음"
    assert df['b'].tolist() == [2, 4], "b 컬럼 값이 맞지 않음"


def test_dataframe_sink_empty():
    """빈 sink 테스트"""
    sink = DataFrameSink()
    df = sink.get_result()
    
    assert len(df) == 0, "빈 DataFrame이어야 함"
    assert isinstance(df, pd.DataFrame), "DataFrame 타입이어야 함"


def test_dataframe_sink_mixed_columns():
    """컬럼이 다른 데이터 처리"""
    sink = DataFrameSink()
    
    sink.write({'a': 1, 'b': 2})
    sink.write({'a': 3, 'c': 5})  # 'b' 없고 'c' 있음
    
    df = sink.get_result()
    
    assert len(df) == 2
    assert set(df.columns) == {'a', 'b', 'c'}, "모든 컬럼이 있어야 함"
    assert pd.isna(df.loc[0, 'c']), "없는 값은 NaN이어야 함"
    assert pd.isna(df.loc[1, 'b']), "없는 값은 NaN이어야 함"


def test_abstract_class_cannot_instantiate():
    """추상 클래스는 인스턴스화 불가"""
    with pytest.raises(TypeError):
        DataSink()