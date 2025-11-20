class TimeWindowProcessor:
    """ 시간 윈도우 단위로 패킷 처리 """
    def __init__(self, window_seconds: float, max_packets: int = None):
        
        # Todo 1. 입력 검증(window_seconds > 0)
        if window_seconds <= 0:
            raise ValueError("간격이 0보다 커야합니다.")
        
        # Todo 2. self.window_seconds 저장
        self.window_seconds = window_seconds
        self.max_packets = max_packets
        
        pass

    def process_stream(self, source, parser, sink):
        # Todo 3. 변수 초기화
        first_timestamp = None
        current_window_start = None
        window_buffer = []
        packet_count = 0
        last_print_count = 0

        # Todo 4. source를 순회하며 패킷 처리
        # Use tqdm if available to show a progress bar for long captures. Fallback to plain iteration.
        try:
            from tqdm import tqdm
            iterator = tqdm(source, desc="Parsing packets", unit="pkt")
        except Exception:
            iterator = source

        for packet in iterator:
            # keep a simple counter for summary and max_packets handling
            packet_count += 1
            
            # 최대 패킷 수 제한
            if self.max_packets and packet_count > self.max_packets:
                # If using tqdm, close the bar after breaking
                try:
                    iterator.close()
                except Exception:
                    pass
                print(f"\n  최대 패킷 수({self.max_packets:,})에 도달했습니다.")
                # Force stop the underlying generator/stream
                try:
                    if hasattr(source, '_stream_packets'):
                        # Stop streaming
                        pass
                except Exception:
                    pass
                break
            timestamp = source.get_timestamp(packet)
            
            if first_timestamp is None:
                first_timestamp = timestamp
                current_window_start = first_timestamp
    
            window_end = current_window_start + self.window_seconds
            if timestamp < window_end:
                window_buffer.append((packet, timestamp))
            else:
                self._flush_window(window_buffer, parser, sink)
                current_window_start = window_end
                window_buffer = [(packet, timestamp)]

        # Todo 5. 마지막 윈도우 처리
        if window_buffer:
            self._flush_window(window_buffer, parser, sink)
        
        # 최종 카운트 출력
        if packet_count > 0:
            try:
                # close tqdm if used
                iterator.close()
            except Exception:
                pass
            print(f"  처리 완료: 총 {packet_count:,}개 패킷               ")

    def _flush_window(self, packet_buffer, parser, sink):
        # Todo 6. 버퍼의 각 패킷을 파싱하여 sink.write() 호출
        for packet, timestamp in packet_buffer:
            parsed_data_list = parser.parse(packet)
            
            # Parser now returns a list of submessages (or None)
            if parsed_data_list:
                for parsed_data in parsed_data_list:
                    # Unix epoch timestamp로 덮어쓰기 (processor가 제공하는 float 값 사용)
                    # EnhancedRTPSParser는 ISO 문자열을 반환하므로 여기서 Unix epoch로 교체
                    parsed_data['timestamp'] = timestamp
                    sink.write(parsed_data)
