import dataclasses
import math
import struct
import unittest
import socket
import subprocess
import sys
import time
from integration.nav_receiver import NavOutput,NavReceiver,crc16
from integration.mavlink_bridge import Bridge,gps_input_packet,x25_crc

def reference_ccitt(data):
    c=0xffff
    for b in data:
        c^=b<<8
        for _ in range(8):c=((c<<1)^0x1021)&0xffff if c&0x8000 else (c<<1)&0xffff
    return c

def reference_x25(data):
    c=0xffff
    for b in data:
        c^=b
        for _ in range(8):c=(c>>1)^0x8408 if c&1 else c>>1
    return c

def sample(t_us=1_000_000,health=0):
    return NavOutput(t_us,45.,-90.,100.,(1.,-2.,.5),(1.,0.,0.,0.),(.25,-.5,2.),3.,4.,2,health)

def frame(o=None,seq=1):
    o=o or sample()
    p=struct.pack('<Qddd3f4f3f2fBI',o.t_us,o.lat_deg,o.lon_deg,o.height_m,*o.vel_ned,*o.quaternion,*o.angular_rate,o.h_sigma,o.v_sigma,o.mode,o.health)
    b=b'\xa5\x5a'+struct.pack('<BBH',1,len(p),seq)+p
    return b+struct.pack('<H',reference_ccitt(b[2:]))

class BridgeTest(unittest.TestCase):
    def test_protocol_known_crc_and_golden(self):
        self.assertEqual(crc16(b'123456789'),0x29b1)
        rx=NavReceiver(reject_health_mask=0)
        o=sample(0x0102030405060708,0x78563412)
        data=frame(o,65535)
        self.assertEqual(data[6:14],bytes.fromhex('0807060504030201'))
        self.assertEqual(len(data),93)
        self.assertEqual(data,bytes.fromhex('a55a0155ffff0807060504030201000000000080464000000000008056c000000000000059400000803f000000c00000003f0000803f0000000000000000000000000000803e000000bf00000040000040400000804002123456780174'))
        self.assertEqual(rx.feed(data,0.),[o])

    def test_stream_noise_crc_damage_recovery(self):
        rx=NavReceiver();data=bytearray(frame());data[15]^=1
        self.assertEqual(rx.feed(b'noise\xa5'+data[:31],0.),[])
        self.assertEqual(rx.feed(data[31:]+frame(),.1),[sample()])
        self.assertEqual(rx.crc_errors,1)

    def test_expiry_uses_local_monotonic_time(self):
        rx=NavReceiver();rx.feed(frame(),12.)
        self.assertEqual(rx.current(12.49),sample())
        self.assertIsNone(rx.current(12.500001))
        self.assertIsNone(rx.current(11.))

    def test_duplicate_timestamp_and_regression_not_rebroadcast(self):
        b=Bridge();b.ingest(frame(),1.)
        self.assertIsNotNone(b.next_packet(1.1))
        self.assertIsNone(b.next_packet(1.2))
        self.assertEqual(b.ingest(frame(seq=2),1.3),[])
        self.assertEqual(b.ingest(frame(sample(999_000),3),1.4),[])
        self.assertIsNone(b.next_packet(1.4))
        b.ingest(frame(sample(1_010_000),4),1.5)
        self.assertIsNotNone(b.next_packet(1.6))
        self.assertIsNone(b.next_packet(2.01))

    def test_health_covariance_and_invalid_quaternion_clear_current(self):
        for bad in [dataclasses.replace(sample(),health=128),dataclasses.replace(sample(),h_sigma=301.),dataclasses.replace(sample(),h_sigma=0.),dataclasses.replace(sample(),v_sigma=math.nan),dataclasses.replace(sample(),quaternion=(0.,0.,0.,0.))]:
            rx=NavReceiver();rx.feed(frame(),0.)
            self.assertEqual(rx.feed(frame(dataclasses.replace(bad,t_us=1_001_000),2),.1),[])
            self.assertIsNone(rx.current(.2))

    def test_sequence_wrap_and_replay(self):
        rx=NavReceiver();rx.feed(frame(seq=65535),0.)
        self.assertEqual(rx.feed(frame(sample(1_001_000),0),.1),[sample(1_001_000)])
        self.assertEqual(rx.feed(frame(sample(1_002_000),0),.2),[])

    def test_mavlink_crc_and_packing_matches_official_offsets(self):
        self.assertEqual(x25_crc(b'123456789'),0x6f91)
        p=gps_input_packet(sample(),seq=5)
        self.assertEqual(p[:10],bytes.fromhex('fd3f0000052adce80000'))
        payload=p[10:-2]
        self.assertEqual(len(payload),63)
        self.assertEqual(struct.unpack_from('<Q',payload,0)[0],1_000_000)
        self.assertEqual(struct.unpack_from('<I',payload,8)[0],0)
        self.assertEqual(struct.unpack_from('<ii',payload,12),(450_000_000,-900_000_000))
        self.assertEqual(struct.unpack_from('<3f',payload,32),(1.,-2.,.5))
        self.assertEqual(struct.unpack_from('<2f',payload,48),(3.,4.))
        self.assertEqual(struct.unpack_from('<H',payload,56)[0],1|2|4|32)
        self.assertEqual(struct.unpack_from('<H',payload,58)[0],0)
        self.assertEqual(payload[61:63],bytes((3,0)))
        self.assertEqual(struct.unpack('<H',p[-2:])[0],reference_x25(p[1:-2]+bytes((151,))))

    def test_ellipsoid_to_msl_requires_explicit_geoid(self):
        p=gps_input_packet(sample(),geoid_separation_m=35.)
        self.assertEqual(struct.unpack_from('<f',p,30)[0],65.)
        self.assertEqual(struct.unpack_from('<H',p,66)[0],2|4|32)
        with self.assertRaises(ValueError):gps_input_packet(sample(),geoid_separation_m=math.nan)

    def test_batched_serial_frames_preserve_output_rate(self):
        b=Bridge();b.ingest(frame()+frame(sample(1_033_333),2)+frame(sample(1_066_667),3),1.)
        packets=[b.next_packet(1.1) for _ in range(3)]
        self.assertTrue(all(packets))
        self.assertEqual([struct.unpack_from('<Q',p,10)[0] for p in packets],[1_000_000,1_033_333,1_066_667])
        self.assertIsNone(b.next_packet(1.1))

    def test_truncated_unknown_frame_timeout_resets_stream(self):
        rx=NavReceiver()
        self.assertEqual(rx.feed(bytes.fromhex('a55a7fff0000')+b'truncated',0.),[])
        self.assertEqual(rx.feed(frame(),.6),[sample()])

    def test_cli_udp_transport_and_stale_shutdown(self):
        sink=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sink.bind(('127.0.0.1',0));sink.settimeout(.2)
        reserve=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);reserve.bind(('127.0.0.1',0));port=reserve.getsockname()[1];reserve.close()
        process=subprocess.Popen([sys.executable,'-m','integration.mavlink_bridge','--listen',f'127.0.0.1:{port}','--output-udp',f'127.0.0.1:{sink.getsockname()[1]}'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        source=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            packet=None
            for attempt in range(10):
                source.sendto(frame(sample(1_000_000+attempt*33_333),attempt+1),('127.0.0.1',port))
                try:packet,_=sink.recvfrom(1024);break
                except socket.timeout:
                    self.assertIsNone(process.poll(),'bridge exited during UDP startup')
            self.assertIsNotNone(packet,'UDP bridge did not receive/forward navigation')
            self.assertEqual(packet[0],0xfd)
            self.assertEqual(packet[7:10],bytes((232,0,0)))
            self.assertEqual(struct.unpack('<H',packet[-2:])[0],reference_x25(packet[1:-2]+bytes((151,))))
            # 下一次等待超过expiry，桥仍运行且没有缓存重发。
            sink.settimeout(.65)
            with self.assertRaises(socket.timeout):sink.recvfrom(1024)
            self.assertIsNone(process.poll())
        finally:
            process.terminate();process.communicate(timeout=2);source.close();sink.close()

    def test_reboot_requires_explicit_session_reset(self):
        rx=NavReceiver();rx.feed(frame(),0.)
        self.assertEqual(rx.feed(frame(sample(1000),0),1.),[])
        rx.reset_session()
        self.assertEqual(rx.feed(frame(sample(1000),0),1.1),[sample(1000)])

if __name__=='__main__':unittest.main()
