import serial
import time
import math
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

# Drive forward at 100 mm/s for 5 seconds
speed_mm_per_s = 100
drive_time_s = 5
expected_distance_mm = speed_mm_per_s * drive_time_s

drive(speed_mm_per_s, speed_mm_per_s)
time.sleep(drive_time_s)

roomba.read_all()
roomba.write(OPCODE_SEND_SENSORS + bytes([2, 43, 44]))
while roomba.in_waiting == 0:
    time.sleep(1 / 1000)
end_left_encoder, end_right_encoder = (int.from_bytes(roomba.read(2), "big", signed=True), int.from_bytes(roomba.read(2), "big", signed=True))

left_encoder_diff = end_left_encoder - start_left_encoder
right_encoder_diff = end_right_encoder - start_right_encoder
avg_diff = (left_encoder_diff + right_encoder_diff) / 2
true_mm = avg_diff * math.pi * 72/508.8

print(f"Expected distance: {expected_distance_mm} mm")
print(f"Left encoder change: {left_encoder_diff}")
print(f"Right encoder change: {right_encoder_diff}")
print(f"Average encoder change: {avg_diff}")
print(f"True mm: {true_mm}")

drive(0, 0)
time.sleep(0.2)
roomba.write(OPCODE_STOP)
time.sleep(0.2)
