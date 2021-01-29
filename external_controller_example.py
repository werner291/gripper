# This script serves as an example and/or starting point
# for communicating with the robotic arm simulator.

import socket
import struct
import math

# The simulator acts as a server, and must be started beforehand.
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Adjust as needed to the expected address / port.
s.connect(('127.0.0.1', 11235))

# Names of each channel, including the frame number.
# The simulator sends the joint angles in a binary format.
channels = ['frame_n', 'swivel', 'link1', 'link2', 'gripper', 'finger_0', 'finger_1', 'finger_2', 'finger_0_2', 'finger_1_2', 'finger_0_2']

# Communication with the server is synchronized: it will send a message to the client,
# then expect a response back, before continuing to the next frame.
while True:

    # Each frame, the server will send exactly 8 + 10*4 bytes exactly once.
    # That's a single, unsigned 64-bit integer, and 10 32-bit floats, all in big-endian format.
    buf = s.recv(8 + 10*4, socket.MSG_WAITALL)

    # Decodes the bytes
    frame_info = struct.unpack('>Qffffffffff', buf)

    # Print out what we have. Perhaps, use this for forward kinematics.
    print(list(zip(channels, frame_info)))

    # The first number in each data buffer is a frame number.
    # This should be going up by 1 every frame, so you can use this
    # to check the decoding results.
    frame_n = frame_info[0]

    # In response, the client must send back 10 floats: one for each joint.
    # The values are target angular velocities for each joint.
    target_speeds = [math.sin(frame_n / 10.0) for _ in channels[1:]]

    # Encode as big-endian 32-bit floats and send to the simulator.
    buf = struct.pack('>ffffffffff', *target_speeds)
    s.send(buf)
