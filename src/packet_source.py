from abc import ABC, abstractmethod
from typing import Iterator, Any

import subprocess
import json
import os

class PacketSource(ABC):
    """
    패킷 소스 추상 인터페이스
    
    배치 모드(pcapng 파일) 또는 실시간 모드(네트워크 캡처)를 
    추상화하여 동일한 인터페이스로 처리
    """
    
    @abstractmethod
    def __iter__(self) -> Iterator[Any]:
        """패킷을 하나씩 yield"""
        pass

    @abstractmethod
    def get_timestamp(self, packet) -> float:
        """패킷의 UTC Unix timestamp 반환"""
        pass


class PcapSource(PacketSource):
    """pcapng 파일 소스 - 스트리밍 방식"""

    def __init__(self, filepath: str):
        if not os.path.isfile(filepath):
            raise FileNotFoundError(f"파일을 찾을 수 없습니다: {filepath}")
        
        self.filepath = filepath
        self._packets = None
        self._streaming = True  # 스트리밍 모드 활성화
        self._process = None  # tshark 프로세스 참조
    
    def _load_packets(self):
        """
        레거시 방식: 모든 패킷을 메모리에 로드 (작은 파일용)
        주의: 큰 파일(>10MB)에서는 메모리 부족 발생 가능
        """
        cmd = ['tshark', '-r', self.filepath, '-Y', 'rtps', '-T', 'ek']
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        packets = []
        for line in result.stdout.strip().split('\n'):
            if not line or line.startswith('{"index":'):
                continue
            try:
                packets.append(json.loads(line))
            except:
                pass

        self._packets = packets
        
    def _stream_packets(self):
        """
        스트리밍 방식: 패킷을 하나씩 파싱 (큰 파일용)
        메모리 효율적이며 실시간 처리 가능
        """
        cmd = ['tshark', '-r', self.filepath, '-Y', 'rtps', '-T', 'ek']
        
        # Popen을 사용하여 스트리밍 처리
        self._process = subprocess.Popen(
            cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1  # 라인 버퍼링
        )
        
        try:
            for line in self._process.stdout:
                line = line.strip()
                if not line or line.startswith('{"index":'):
                    continue
                try:
                    packet = json.loads(line)
                    yield packet
                except json.JSONDecodeError:
                    continue
        finally:
            # Terminate process if still running
            if self._process and self._process.poll() is None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait()
        
    def __iter__(self):
        """
        스트리밍 모드 활성화 시 메모리 효율적인 방식 사용
        """
        if self._streaming:
            # 스트리밍 방식 (큰 파일에 적합)
            return self._stream_packets()
        else:
            # 레거시 방식 (작은 파일에 적합)
            if self._packets is None:
                self._load_packets()
            return iter(self._packets)

    def get_timestamp(self, packet) -> float:
        # ek 형식: 'frame_frame_time_epoch' 사용
        timestamp_str = packet['layers']['frame']['frame_frame_time_epoch']
        
        # ISO 형식 (2025-11-19T07:41:28.307555235Z) 처리
        if isinstance(timestamp_str, str) and 'T' in timestamp_str:
            from datetime import datetime
            dt = datetime.fromisoformat(timestamp_str.replace('Z', '+00:00'))
            return dt.timestamp()
        else:
            return float(timestamp_str)
    
    def close(self):
        """리소스 정리 (필요 시 tshark 프로세스 종료)"""
        if self._process and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
        