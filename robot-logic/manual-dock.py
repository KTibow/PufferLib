import serial
import time
import struct
import datetime
from interface import (PACKETS, OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_STREAM_SENSORS, OPCODE_DRIVE_DIRECT)

# Device codes for dock debugging
GREEN_CODES = [164, 165, 172, 173]
RED_CODES = [168, 169, 172, 173]
FF_CODES = [161, 165, 169, 173]

roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

def drive(left, right):
    roomba.write(OPCODE_DRIVE_DIRECT + right.to_bytes(2, "big", signed=True) + left.to_bytes(2, "big", signed=True))

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
time.sleep(0.2)
roomba.write(OPCODE_SAFE)
time.sleep(0.2)

# Stream IR sensors and dirt detect
roomba.write(OPCODE_STREAM_SENSORS + bytes([4, 17, 52, 53]))
time.sleep(0.1)
roomba.read_all()

last_green = time.time()
last_red = time.time()
last_ff = time.time()

print("\033[?25l", end="")  # Hide cursor
try:
    while True:
        data = wait_and_read_stream([17, 52, 53])
        if data is None:
            continue

        ir_center, ir_left, ir_right = data

        if ir_center in GREEN_CODES or ir_left in GREEN_CODES or ir_right in GREEN_CODES:
            last_green = time.time()
            print("Green")
        if ir_center in RED_CODES or ir_left in RED_CODES or ir_right in RED_CODES:
            last_red = time.time()
            print("Red")
        if ir_center in FF_CODES or ir_left in FF_CODES or ir_right in FF_CODES:
            last_ff = time.time()

        green_intensity = last_green - last_red
        red_intensity = last_red - last_green
        drive(
          int(100 - red_intensity * 100),
          int(100 - green_intensity * 100)
        )
finally:
    print("\033[?25h", end="")  # Show cursor
    roomba.write(OPCODE_STOP)
