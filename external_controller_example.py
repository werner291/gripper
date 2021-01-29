import socket
import struct
import math

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect(('127.0.0.1', 11235))

channels = ['frame_n', 'swivel', 'link1', 'link2', 'gripper', 'finger_0', 'finger_1', 'finger_2', 'finger_0_2', 'finger_1_2', 'finger_0_2']

while True:
    buf = s.recv(8 + 10*4, socket.MSG_WAITALL)
    
    frame_info = struct.unpack('>Qffffffffff', buf)
    print(list(zip(channels, frame_info)))
    
    frame_n = frame_info[0]
    
    target_speeds = [math.sin(frame_n / 10.0) for _ in channels[1:]]
    buf = struct.pack('>ffffffffff', *target_speeds)
    s.send(buf)
