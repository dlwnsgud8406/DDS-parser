"""
QoS 정책 추론 모듈

메시지 패턴으로부터 QoS 정책을 추론합니다.
"""

from collections import defaultdict, Counter
from typing import Dict, List, Any


class QoSAnalyzer:
    """메시지 패턴 기반 QoS 추론"""
    
    def __init__(self):
        self.topic_stats = defaultdict(lambda: {
            'total_count': 0,
            'heartbeat_count': 0,
            'acknack_count': 0,
            'data_count': 0,
            'timestamps': [],
            'submsg_types': Counter()
        })
    
    def analyze_messages(self, messages: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
        """
        메시지 리스트를 분석하여 토픽별 QoS 추론
        
        Returns:
            {
                'topic_name': {
                    'reliability': 'RELIABLE' | 'BEST_EFFORT',
                    'durability': 'TRANSIENT_LOCAL' | 'VOLATILE',
                    'frequency_hz': float,
                    'confidence': str
                }
            }
        """
        # 통계 수집
        for msg in messages:
            topic = msg.get('topic') or msg.get('pids', {}).get('PID_TOPIC_NAME_topic')
            if not topic:
                continue
            
            submsg_type = msg.get('submsg_name', '')
            timestamp = msg.get('timestamp')
            
            stats = self.topic_stats[topic]
            stats['total_count'] += 1
            stats['submsg_types'][submsg_type] += 1
            
            if submsg_type == 'HEARTBEAT':
                stats['heartbeat_count'] += 1
            elif submsg_type == 'ACKNACK':
                stats['acknack_count'] += 1
            elif 'DATA' in submsg_type:
                stats['data_count'] += 1
            
            if timestamp:
                stats['timestamps'].append(timestamp)
        
        # QoS 추론
        qos_map = {}
        for topic, stats in self.topic_stats.items():
            qos_map[topic] = self._infer_qos(topic, stats)
        
        return qos_map
    
    def _infer_qos(self, topic: str, stats: Dict) -> Dict[str, Any]:
        """개별 토픽의 QoS 추론"""
        
        # Reliability 추론
        reliability = self._infer_reliability(stats)
        
        # Durability 추론
        durability = self._infer_durability(stats)
        
        # 주파수 계산
        frequency = self._calculate_frequency(stats)
        
        return {
            'reliability': reliability['type'],
            'reliability_confidence': reliability['confidence'],
            'durability': durability['type'],
            'durability_confidence': durability['confidence'],
            'frequency_hz': frequency,
            'message_count': stats['total_count']
        }
    
    def _infer_reliability(self, stats: Dict) -> Dict[str, str]:
        """Reliability 추론"""
        total = stats['total_count']
        acknack = stats['acknack_count']
        heartbeat = stats['heartbeat_count']
        
        # ACKNACK 존재 → RELIABLE (재전송 확인)
        if acknack > 0:
            return {
                'type': 'RELIABLE',
                'confidence': 'HIGH'
            }
        
        # HEARTBEAT 많음 → RELIABLE
        if heartbeat > total * 0.3:
            return {
                'type': 'RELIABLE',
                'confidence': 'MEDIUM'
            }
        
        # 기본값
        return {
            'type': 'BEST_EFFORT',
            'confidence': 'LOW'
        }
    
    def _infer_durability(self, stats: Dict) -> Dict[str, str]:
        """Durability 추론"""
        data_count = stats['data_count']
        
        # DATA 메시지가 적음 → TRANSIENT_LOCAL (보존)
        if 0 < data_count < 5:
            return {
                'type': 'TRANSIENT_LOCAL',
                'confidence': 'MEDIUM'
            }
        
        # 기본값 (스트리밍)
        return {
            'type': 'VOLATILE',
            'confidence': 'MEDIUM'
        }
    
    def _calculate_frequency(self, stats: Dict) -> float:
        """발행 주파수 계산 (Hz)"""
        timestamps = sorted(stats['timestamps'])
        
        if len(timestamps) < 2:
            return 0.0
        
        # 처음 100개 간격만 사용
        intervals = []
        for i in range(1, min(100, len(timestamps))):
            interval = timestamps[i] - timestamps[i-1]
            if interval > 0:
                intervals.append(interval)
        
        if not intervals:
            return 0.0
        
        avg_interval = sum(intervals) / len(intervals)
        return 1.0 / avg_interval if avg_interval > 0 else 0.0
    
    def get_qos_summary_dataframe(self) -> List[Dict]:
        """QoS Summary 시트용 데이터 생성"""
        summary = []
        
        for topic, stats in sorted(
            self.topic_stats.items(),
            key=lambda x: x[1]['total_count'],
            reverse=True
        ):
            qos = self._infer_qos(topic, stats)
            
            # 주파수를 Period(ms)로 변환
            hz = qos['frequency_hz']
            if hz > 0 and hz < 1000:
                period_ms = round(1000 / hz, 1)
                frequency_display = f"{round(hz, 1)} Hz ({period_ms} ms)"
            elif hz >= 1000:
                frequency_display = f"{round(hz, 0)} Hz"
            else:
                frequency_display = '-'
            
            summary.append({
                'Topic': topic,
                'Reliability': qos['reliability'],
                'Rel_Confidence': qos['reliability_confidence'],
                'Durability': qos['durability'],
                'Dur_Confidence': qos['durability_confidence'],
                'Publish_Rate': frequency_display,
                'Total_Messages': qos['message_count']
            })
        
        return summary
