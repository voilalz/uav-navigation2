"""GPS_INPUT试验桥；不实现External AHRS，不伪造GNSS周/卫星数/速度精度。"""
try:
    from .nav_receiver import NavOutput, NavReceiver, valid_output
except ImportError:
    from nav_receiver import NavOutput, NavReceiver, valid_output
import argparse
import math
import socket
import struct
import sys
import time
from collections import deque

def x25_crc(data: bytes) -> int:
    crc=0xffff
    for byte in data:
        tmp=byte^(crc&0xff)
        tmp^=(tmp<<4)&0xff
        crc=((crc>>8)^(tmp<<8)^(tmp<<3)^(tmp>>4))&0xffff
    return crc

def gps_input_packet(output: NavOutput,seq=0,system_id=42,component_id=220,gps_id=0,geoid_separation_m=None):
    if not valid_output(output):raise ValueError('无效导航输出不能打包GPS_INPUT')
    if not all(isinstance(v,int) and 0<=v<=255 for v in (seq,system_id,component_id,gps_id)):
        raise ValueError('MAVLink编号必须为uint8')
    if geoid_separation_m is not None and not math.isfinite(geoid_separation_m):
        raise ValueError('大地水准面分离量必须有限')
    # 椭球高 h = MSL高度 H + geoid N。无N时明确ignore ALT。
    flags=2|4|32
    altitude=0.
    if geoid_separation_m is None:flags|=1
    else:altitude=output.height_m-geoid_separation_m
    # GPS_INPUT未提供week的ignore位；0表示本桥未提供绝对GNSS时。
    # time_usec直接保留MCU启动后单调时间，不从系统时钟生成伪GPS周。
    payload=struct.pack('<QIii9fHHBBB',output.t_us,0,round(output.lat_deg*1e7),round(output.lon_deg*1e7),
                        altitude,0.,0.,*output.vel_ned,0.,output.h_sigma,output.v_sigma,
                        flags,0,gps_id,3,0)
    header=bytes((0xfd,len(payload),0,0,seq,system_id,component_id,232,0,0))
    return header+payload+struct.pack('<H',x25_crc(header[1:]+payload+bytes((151,))))

class Bridge:
    def __init__(self, receiver=None,**options):
        self.receiver=receiver or NavReceiver()
        self.options=options
        self._last_sent_t_us=None
        self._seq=0
        self._pending=deque(maxlen=256)

    def ingest(self,data,now):
        outputs=self.receiver.feed(data,now)
        if self.receiver.current(now) is None:self._pending.clear()
        else:self._pending.extend((o,now) for o in outputs)
        return outputs

    def next_packet(self,now):
        if self.receiver.current(now) is None:
            self._pending.clear()
            return None
        while self._pending:
            o,received_at=self._pending.popleft()
            if now>=received_at and now-received_at<=self.receiver.timeout_s and o.t_us!=self._last_sent_t_us:break
        else:return None
        packet=gps_input_packet(o,seq=self._seq,**self.options)
        self._seq=(self._seq+1)&255
        self._last_sent_t_us=o.t_us
        return packet

def endpoint(text):
    host,port=text.rsplit(':',1)
    port=int(port)
    if not 0<port<65536:raise ValueError('UDP端口越界')
    return host,port

def serial_open(port,baud):
    try:import serial
    except ImportError as exc:raise RuntimeError('串口模式需要安装pyserial；UDP模式仅使用标准库') from exc
    return serial.Serial(port,baudrate=baud,timeout=.05,write_timeout=.5)

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    inputs=p.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--listen',help='接收导航帧UDP地址，例如192.168.42.2:14560')
    inputs.add_argument('--input-serial',help='导航MCU串口，例如/dev/ttyUSB0')
    outputs=p.add_mutually_exclusive_group(required=True)
    outputs.add_argument('--output-udp',help='飞控MAVLink UDP目标，例如127.0.0.1:14550')
    outputs.add_argument('--output-serial',help='飞控MAVLink串口，例如/dev/ttyTHS1')
    p.add_argument('--input-baud',type=int,default=921600)
    p.add_argument('--output-baud',type=int,default=921600)
    p.add_argument('--timeout',type=float,default=.5)
    p.add_argument('--max-horizontal-sigma',type=float,default=100.)
    p.add_argument('--reject-health-mask',type=lambda v:int(v,0),default=165)
    p.add_argument('--geoid-separation',type=float,default=None,help='验证过的当地N=h-H，米；省略时ignore高度')
    p.add_argument('--system-id',type=int,default=42)
    p.add_argument('--component-id',type=int,default=220)
    p.add_argument('--gps-id',type=int,default=0)
    args=p.parse_args(argv)
    source=sink=None
    try:
        rx=NavReceiver(args.timeout,args.max_horizontal_sigma,args.reject_health_mask)
        bridge=Bridge(rx,system_id=args.system_id,component_id=args.component_id,gps_id=args.gps_id,geoid_separation_m=args.geoid_separation)
        if args.listen:
            source=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            source.bind(endpoint(args.listen));source.settimeout(.05)
        else:source=serial_open(args.input_serial,args.input_baud)
        if args.output_udp:
            sink=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);target=endpoint(args.output_udp)
        else:sink=serial_open(args.output_serial,args.output_baud)
        while True:
            if args.listen:
                try:data,_=source.recvfrom(65535)
                except socket.timeout:data=b''
            else:data=source.read(8192)
            now=time.monotonic()
            if data:bridge.ingest(data,now)
            while True:
                packet=bridge.next_packet(now)
                if not packet:break
                if args.output_udp:sink.sendto(packet,target)
                else:
                    if sink.write(packet)!=len(packet):raise RuntimeError('串口导航写入不完整')
    except KeyboardInterrupt:return 0
    except (ValueError,OSError,RuntimeError) as exc:
        print(str(exc),file=sys.stderr);return 2
    finally:
        if source is not None:source.close()
        if sink is not None:sink.close()

if __name__=='__main__':raise SystemExit(main())
