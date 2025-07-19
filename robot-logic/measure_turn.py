import serial
import time
from interface import ( OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_SEND_SENSORS, OPCODE_DRIVE_DIRECT )

roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

roomba.write(OPCODE_START)
roomba.write(OPCODE_SAFE)
time.sleep(0.2)
roomba.write(bytes([150, 0]))
time.sleep(0.2)

roomba.read_all()
roomba.write(OPCODE_SEND_SENSORS + bytes([2, 43, 44]))
while roomba.in_waiting == 0:
    time.sleep(1 / 1000)
start_left_encoder, start_right_encoder = (int.from_bytes(roomba.read(2), "big", signed=True), int.from_bytes(roomba.read(2), "big", signed=True))

def drive(left, right):
  roomba.write(OPCODE_DRIVE_DIRECT + left.to_bytes(2, "big", signed=True) + right.to_bytes(2, "big", signed=True))

drive(29, -29)
start_time = time.time()
input("Hit enter upon 4 turns")
end_time = time.time()

roomba.read_all()
roomba.write(OPCODE_SEND_SENSORS + bytes([2, 43, 44]))
while roomba.in_waiting == 0:
    time.sleep(1 / 1000)
end_left_encoder, end_right_encoder = (int.from_bytes(roomba.read(2), "big", signed=True), int.from_bytes(roomba.read(2), "big", signed=True))

print(f"s: {end_time - start_time}")
print(f"left encoder: {start_left_encoder} -> {end_left_encoder} ({end_left_encoder - start_left_encoder})")
print(f"right encoder: {start_right_encoder} -> {end_right_encoder} ({end_right_encoder - start_right_encoder})")

drive(0, 0)
time.sleep(0.2)
roomba.write(OPCODE_STOP)
time.sleep(0.2)
