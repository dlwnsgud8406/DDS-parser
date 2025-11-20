class RTPSParser:
    """ RTPS 패킷 파서 클래스 """

    def parse(self, packet) -> dict:
        """ 파싱 """
        # Todo 1. layers 추출
        layers = packet.get('layers', {})

        # Todo 2. rtps,frame 레이어 추출
        rtps = layers.get('rtps', {})
        frame = layers.get('frame', {})

        # Todo 3. 기본 필드 추출하여 dict 반환
        result = {
            'frame_number': frame.get('frame_frame_number'),
            'submsg_id': rtps.get('rtps_rtps_sm_id')
        }
        return result