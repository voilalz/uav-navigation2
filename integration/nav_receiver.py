"""MCU导航帧接收器；完整工程实现由流解析、状态校验和本地超时组成。"""
from dataclasses import dataclass
from typing import Optional
import math
import struct

NAV_OUTPUT_STRUCT=struct.Struct('<Qddd3f4f3f2fBI')
PAYLOAD_LENGTHS={1:85,2:13,3:17,4:69,0x10:107,0x11:106,0x20:29}

@dataclass(frozen=True)
class NavOutput:
    t_us: int
    lat_deg: float
    lon_deg: float
    height_m: float
    vel_ned: tuple
    quaternion: tuple
    angular_rate: tuple
    h_sigma: float
    v_sigma: float
    mode: int
    health: int

def crc16(data: bytes) -> int:
    c=0xffff
    for byte in data:
        c^=byte<<8
        for _ in range(8):
            c=((c<<1)^0x1021)&0xffff if c&0x8000 else (c<<1)&0xffff
    return c

def valid_output(o: NavOutput,max_horizontal_sigma_m=float('inf'),reject_health_mask=0):
    values=(o.lat_deg,o.lon_deg,o.height_m,*o.vel_ned,*o.quaternion,*o.angular_rate,o.h_sigma,o.v_sigma)
    return (all(math.isfinite(v) for v in values)
            and 0<o.t_us<1<<64 and abs(o.lat_deg)<=90 and abs(o.lon_deg)<=180
            and -1000<o.height_m<100000 and 0<o.h_sigma<=max_horizontal_sigma_m and o.v_sigma>0
            and o.mode in range(5) and 0<=o.health<1<<32 and not(o.health&reject_health_mask)
            and abs(sum(v*v for v in o.quaternion)-1)<.002)

def decode_nav_output(payload):
    if len(payload)!=NAV_OUTPUT_STRUCT.size:
        raise ValueError('NAV_OUTPUT必须为85字节')
    v=NAV_OUTPUT_STRUCT.unpack(payload)
    return NavOutput(v[0],v[1],v[2],v[3],v[4:7],v[7:11],v[11:14],v[14],v[15],v[16],v[17])

class NavReceiver:
    def __init__(self, timeout_s=.5, max_horizontal_sigma_m=100., reject_health_mask=165):
        if not math.isfinite(timeout_s) or timeout_s<=0 or not math.isfinite(max_horizontal_sigma_m) or max_horizontal_sigma_m<=0 or not 0<=reject_health_mask<1<<32:
            raise ValueError('超时、协方差门限和health掩码必须有效')
        self.timeout_s=timeout_s
        self.max_sigma=max_horizontal_sigma_m
        self.reject_health_mask=reject_health_mask
        self.crc_errors=0
        self.rejected=0
        self.length_errors=0
        self.dropped_frames=0
        self.latest: Optional[NavOutput]=None
        self._buffer=bytearray()
        self._received_at=None
        self._last_seq=None
        self._last_t_us=None
        self._last_byte_at=None

    def feed(self,data: bytes,now: float):
        if not math.isfinite(now):
            raise ValueError('接收时间必须为本地单调时钟')
        if data:
            if self._last_byte_at is not None and (now<self._last_byte_at or now-self._last_byte_at>.5):
                self._buffer.clear()
            self._last_byte_at=now
        outputs=[]
        self._buffer.extend(data)
        while len(self._buffer)>=2:
            start=self._buffer.find(b'\xa5\x5a')
            if start<0:
                self._buffer[:]=self._buffer[-1:] if self._buffer[-1]==0xa5 else b''
                break
            del self._buffer[:start]
            if len(self._buffer)<6:break
            msg_id,length,seq=struct.unpack_from('<BBH',self._buffer,2)
            if msg_id in PAYLOAD_LENGTHS and length!=PAYLOAD_LENGTHS[msg_id]:
                self.length_errors+=1
                del self._buffer[0]
                continue
            total=length+8
            if len(self._buffer)<total:break
            if crc16(self._buffer[2:total-2])!=struct.unpack_from('<H',self._buffer,total-2)[0]:
                self.crc_errors+=1
                del self._buffer[0]
                continue
            payload=bytes(self._buffer[6:total-2])
            del self._buffer[:total]
            if msg_id!=1:continue
            o=decode_nav_output(payload)
            seq_delta=(seq-self._last_seq)&0xffff if self._last_seq is not None else 1
            if (not valid_output(o,self.max_sigma,self.reject_health_mask)
                    or not 0<seq_delta<0x8000
                    or (self._last_t_us is not None and o.t_us<=self._last_t_us)
                    or (self._received_at is not None and now<self._received_at)):
                self.rejected+=1
                self.latest=None
                continue
            # seq跨七种消息共享，差值只作丢帧诊断，不声称全是导航帧。
            if self._last_seq is not None:self.dropped_frames+=max(0,seq_delta-1)
            self._last_seq=seq
            self._last_t_us=o.t_us
            self._received_at=now
            self.latest=o
            outputs.append(o)
        return outputs

    def current(self,now: float) -> Optional[NavOutput]:
        if (not math.isfinite(now) or self.latest is None or self._received_at is None
                or now<self._received_at or now-self._received_at>self.timeout_s):
            return None
        return self.latest

    def reset_session(self):
        """显式开启MCU新会话；统计保留，禁止自动用超时清掉反重放状态。"""
        self._buffer.clear()
        self._last_seq=self._last_t_us=self._last_byte_at=self._received_at=None
        self.latest=None
