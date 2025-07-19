import serial
import time
import struct
import math
from interface import (PACKETS, OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_DRIVE_DIRECT, OPCODE_STREAM_SENSORS)

roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

def drive(left, right):
  roomba.write(OPCODE_DRIVE_DIRECT + left.to_bytes(2, "big", signed=True) + right.to_bytes(2, "big", signed=True))
def read_stream(packets):
    length = 1 + 1 + sum(1 + PACKETS[pid][1] for pid in packets) + 1
    data = roomba.read(length)
    if len(data) != length:
        raise RuntimeError("Did not receive expected number of bytes from Roomba")

    header = data[0]
    n_bytes = data[1]
    packet_data = list(data[2:-1])
    checksum = data[-1]
    print("Read", header, n_bytes, checksum)
    assert header == 19
    assert n_bytes == len(packet_data)
    assert (header + n_bytes + sum(packet_data) + checksum) & 0xFF == 0

    readings = []
    for packet_id in packets:
        if not packet_data:
            break
        pid = packet_data.pop(0)
        if pid != packet_id:
            raise ValueError(f"Packet ID mismatch: expected {packet_id}, got {pid}")
        fmt, size = PACKETS[packet_id]
        if len(packet_data) < size:
            break
        data_bytes = [packet_data.pop(0) for _ in range(size)]
        value = struct.unpack(fmt, bytes(data_bytes))[0]
        readings.append(value)
    return readings
def wait_and_read_stream(packets):
    while roomba.in_waiting == 0:
        time.sleep(1 / 1000)
    return read_stream(packets)

roomba.write(OPCODE_START)
roomba.write(OPCODE_SAFE)
time.sleep(0.2)

roomba.write(OPCODE_STREAM_SENSORS + bytes([2, 43, 44]))

drive(250, -250)

roomba.read_all()
left_start, right_start = wait_and_read_stream([43, 44])
while True:
  left_current, right_current = wait_and_read_stream([43, 44])
  left_delta = left_current - left_start
  right_delta = right_current - right_start
  left_mm = left_delta * math.pi * 72 / 508.8
  right_mm = right_delta * math.pi * 72 / 508.8

  print(left_mm, right_mm)
  full_rotation = 235 * math.pi
  if abs(left_mm) > full_rotation or abs(right_mm) > full_rotation:
    break

drive(0, 0)
roomba.write(OPCODE_STOP)
