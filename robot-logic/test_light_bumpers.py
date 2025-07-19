import serial
import time
import struct
from interface import (PACKETS, OPCODE_START, OPCODE_SAFE, OPCODE_DRIVE_DIRECT, OPCODE_STREAM_SENSORS)

roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

def drive(left, right):
  roomba.write(OPCODE_DRIVE_DIRECT + left.to_bytes(2, "big", signed=True) + right.to_bytes(2, "big", signed=True))
def read_stream(packets):
    length = 1 + 1 + sum(1 + PACKETS[pid][1] for pid in packets) + 1
    data = roomba.read(length)
    if len(data) != length:
        return None

    header = data[0]
    n_bytes = data[1]
    packet_data = list(data[2:-1])
    checksum = data[-1]
    if header != 19:
        return None
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
    while roomba.in_waiting < 4:
        time.sleep(1 / 1000)
    return read_stream(packets)

roomba.write(OPCODE_START)
roomba.write(OPCODE_SAFE)
time.sleep(0.2)

roomba.write(OPCODE_STREAM_SENSORS + bytes([6, 46, 47, 48, 49, 50, 51]))
time.sleep(0.1)
roomba.read_all()
print("\033[?25l", end="")  # Hide cursor
try:
    while True:
      data = wait_and_read_stream([46, 47, 48, 49, 50, 51])
      if data is None:
        continue
      (l, lf, lc, rc, rf, r) = data
      print(f"\r{l:4} {lf:4} {lc:4} {rc:4} {rf:4} {r:4} " +
        "".join("█" if val > 500 else "▓" if val > 200 else "░" if val > 50 else " "
                    for val in [l, lf, lc, rc, rf, r]), end="", flush=True)
finally:
    print("\033[?25h", end="")  # Show cursor
