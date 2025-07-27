import serial
import time
import math
from interface import ( OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_SEND_SENSORS, OPCODE_DRIVE_PWM )

roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

roomba.write(OPCODE_START)
roomba.write(OPCODE_SAFE)
time.sleep(0.2)
roomba.write(bytes([150, 0]))
time.sleep(0.2)

def drive(left, right):
    roomba.write(OPCODE_DRIVE_PWM + left.to_bytes(2, "big", signed=True) + right.to_bytes(2, "big", signed=True))

def get_encoder_values():
    roomba.read_all()
    roomba.write(OPCODE_SEND_SENSORS + bytes([2, 43, 44]))
    while roomba.in_waiting == 0:
        time.sleep(1 / 1000)
    left = int.from_bytes(roomba.read(2), "big", signed=True)
    right = int.from_bytes(roomba.read(2), "big", signed=True)
    return left, right

def handle_encoder_overflow(current, previous):
    diff = current - previous
    if diff > 32767:
        diff -= 65536
    elif diff < -32768:
        diff += 65536
    return diff

def to_mm(encoder_ticks):
    return encoder_ticks * math.pi * 72 / 508.8

# Quick sweep parameters
test_duration = 0.2  # Short duration to see step changes quickly
direction = 1
results = []

print("Speed sweep starting - watch for step changes in movement!")
print("Speed | Direction | Encoder Change | Actual Speed")
print("-" * 50)

for speed_magnitude in range(0, 255):
    d = test_duration
    speed = speed_magnitude * direction
    if speed_magnitude > 0 and speed_magnitude % 20 == 0:
        direction *= -1
        speed = speed_magnitude * direction
        print(f"--- Direction change at speed {speed_magnitude} ---")
        drive(0, 0)
        time.sleep(0.5)
        drive(speed, speed)
        time.sleep(0.5)

    # Get initial encoder
    start_left, start_right = get_encoder_values()

    # Drive at test speed
    drive(speed, speed)
    time.sleep(d)

    # Stop and measure
    end_left, end_right = get_encoder_values()

    # Calculate movement
    left_diff = handle_encoder_overflow(end_left, start_left)
    right_diff = handle_encoder_overflow(end_right, start_right)
    avg_diff = (left_diff + right_diff) / 2
    actual_speed = to_mm(avg_diff) / d

    direction_str = "FWD" if direction > 0 else "BWD"
    print(f"{speed:4d} | {direction_str:9s} | {avg_diff:13.1f} | {actual_speed:11.1f}")

    results.append({
        'commanded_speed': speed,
        'actual_speed': actual_speed,
        'encoder_change': avg_diff
    })

# Stop and cleanup
drive(0, 0)
time.sleep(0.2)
roomba.write(OPCODE_STOP)
time.sleep(0.2)
roomba.close()

print(f"\nSwept through {len(results)} speeds. Look for step changes in the actual speed column!")
