#!/usr/bin/env python3
import serial
import time
import struct
from interface import (PACKETS, OPCODE_DRIVE_DIRECT, OPCODE_MOTORS, OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_STREAM_SENSORS)

def read_stream(roomba, packets):
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

def wait_and_read_stream(roomba, packets):
    while roomba.in_waiting < 4:
        time.sleep(1 / 1000)
    return read_stream(roomba, packets)

def main():
    # Connect to Roomba
    roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)
    def drive(left, right):
      roomba.write(OPCODE_DRIVE_DIRECT + right.to_bytes(2, "big", signed=True) + left.to_bytes(2, "big", signed=True))
    print("Setting up Roomba...")
    roomba.write(OPCODE_START)
    roomba.write(OPCODE_SAFE)
    time.sleep(0.2)
    roomba.write(bytes([150, 0]))
    drive(250, 250)
    time.sleep(0.1)
    roomba.read_all()

    # Start streaming dirt detect data (packet ID 15)
    roomba.write(OPCODE_STREAM_SENSORS + bytes([1, 15]))
    time.sleep(0.1)
    roomba.read_all()
    print("Roomba ready!")

    print("Motors ON. Streaming dirt detect data. Press Ctrl+C to stop.")
    roomba.write(OPCODE_MOTORS + bytes([0b00000110]))

    print("\033[?25l", end="")  # Hide cursor
    try:
        while True:
            data = wait_and_read_stream(roomba, [15])
            if data is None:
                continue
            dirt_level = data[0]
            print(f"Dirt Detect: {dirt_level:3} " +
                  ("█" * min(dirt_level // 10, 20)).ljust(20))
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        print("\033[?25h", end="")  # Show cursor
        roomba.write(OPCODE_MOTORS + bytes([0b00000000]))
        roomba.write(OPCODE_STOP)
        roomba.close()

if __name__ == "__main__":
    main()
