import serial
import time
import math
from interface import (OPCODE_START, OPCODE_SAFE, OPCODE_STOP,
                      OPCODE_SEND_SENSORS, OPCODE_DRIVE_DIRECT)

ENCODER_TO_MM = math.pi * 72 / 508.8
TEST_DURATION = 3.0
SETTLING_TIME = 0.1

def linear_regression(x_vals, y_vals):
    """Calculate linear regression without external libraries"""
    n = len(x_vals)
    sum_x = sum(x_vals)
    sum_y = sum(y_vals)
    sum_xy = sum(x * y for x, y in zip(x_vals, y_vals))
    sum_x2 = sum(x * x for x in x_vals)
    sum_y2 = sum(y * y for y in y_vals)

    slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
    intercept = (sum_y - slope * sum_x) / n

    # Calculate R-squared
    y_mean = sum_y / n
    ss_tot = sum((y - y_mean) ** 2 for y in y_vals)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(x_vals, y_vals))
    r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0

    return slope, intercept, r_squared

def rmse(actual, expected):
    """Calculate root mean square error"""
    return math.sqrt(sum((a - e) ** 2 for a, e in zip(actual, expected)) / len(actual))

# Initialize Roomba
roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)
roomba.write(OPCODE_START)
roomba.write(OPCODE_SAFE)
time.sleep(0.2)
roomba.write(bytes([150, 0]))
time.sleep(0.2)

# Test speed pairs - forward/backward to minimize space usage
test_pairs = [
    # Straight movement pairs
    [(50, 50), (-50, -50)],
    [(100, 100), (-100, -100)],
    [(150, 150), (-150, -150)],
    [(200, 200), (-200, -200)],
    [(300, 300), (-300, -300)],
    [(400, 400), (-400, -400)],
    [(500, 500), (-500, -500)],

    # Right turn / left turn pairs
    [(100, 50), (-100, -50)],
    [(50, 100), (-50, -100)],
    [(150, 100), (-150, -100)],
    [(100, 150), (-100, -150)],

    # Spin clockwise / counterclockwise pairs
    [(50, -50), (-50, 50)],
    [(100, -100), (-100, 100)],
    [(300, -300), (-300, 300)],
    [(500, -500), (-500, 500)],

    # Single wheel pairs
    [(100, 0), (-100, 0)],
    [(0, 100), (0, -100)],

    # Mixed movement pairs
    [(100, -50), (-100, 50)],
    [(75, -25), (-75, 25)]
]

left_expected = []
left_actual = []
right_expected = []
right_actual = []

print(f"Running {len(test_pairs)} test pairs ({len(test_pairs)*2} total tests)...")

for pair_num, test_pair in enumerate(test_pairs):
    print(f"\nPair {pair_num+1}/{len(test_pairs)}:")

    for test_num, (left_exp, right_exp) in enumerate(test_pair):
        print(f"  Test {test_num+1}/2: L={left_exp:4d}, R={right_exp:4d} mm/s", end=" -> ")

        # Get initial encoder values
        roomba.read_all()
        roomba.write(OPCODE_SEND_SENSORS + bytes([2, 43, 44]))
        while roomba.in_waiting == 0:
            time.sleep(0.001)
        start_left = int.from_bytes(roomba.read(2), "big", signed=True)
        start_right = int.from_bytes(roomba.read(2), "big", signed=True)

        # Start driving
        roomba.write(OPCODE_DRIVE_DIRECT +
                    right_exp.to_bytes(2, "big", signed=True) +
                    left_exp.to_bytes(2, "big", signed=True))
        time.sleep(SETTLING_TIME)

        # Time the test
        test_start = time.time()
        time.sleep(TEST_DURATION)

        # Stop and get final readings
        roomba.write(OPCODE_DRIVE_DIRECT + bytes([0, 0, 0, 0]))
        test_end = time.time()

        roomba.read_all()
        roomba.write(OPCODE_SEND_SENSORS + bytes([2, 43, 44]))
        while roomba.in_waiting == 0:
            time.sleep(0.001)
        end_left = int.from_bytes(roomba.read(2), "big", signed=True)
        end_right = int.from_bytes(roomba.read(2), "big", signed=True)

        # Calculate actual speeds
        actual_time = test_end - test_start
        left_ticks = end_left - start_left
        right_ticks = end_right - start_right

        left_actual_speed = (left_ticks * ENCODER_TO_MM) / actual_time
        right_actual_speed = (right_ticks * ENCODER_TO_MM) / actual_time

        left_expected.append(left_exp)
        left_actual.append(left_actual_speed)
        right_expected.append(right_exp)
        right_actual.append(right_actual_speed)

        print(f"L={left_actual_speed:6.1f}, R={right_actual_speed:6.1f} mm/s")

        # Short pause between tests in a pair, longer pause between pairs
        if test_num == 0:
            time.sleep(0.2)  # Brief pause before reverse movement
        else:
            time.sleep(0.8)  # Longer pause before next pair

# Stop Roomba
roomba.write(OPCODE_DRIVE_DIRECT + bytes([0, 0, 0, 0]))
time.sleep(0.2)
roomba.write(OPCODE_STOP)
time.sleep(0.2)
roomba.close()

# Calculate regressions
left_slope, left_intercept, left_r2 = linear_regression(left_expected, left_actual)
right_slope, right_intercept, right_r2 = linear_regression(right_expected, right_actual)

# Calculate RMSE
left_rmse = rmse(left_actual, left_expected)
right_rmse = rmse(right_actual, right_expected)

print("\n" + "="*60)
print("CALIBRATION RESULTS")
print("="*60)

print(f"\nLEFT WHEEL:")
print(f"  Linear regression: actual = {left_slope:.4f} * expected + {left_intercept:.2f}")
print(f"  R-squared: {left_r2:.4f}")
print(f"  RMSE: {left_rmse:.2f} mm/s")

print(f"\nRIGHT WHEEL:")
print(f"  Linear regression: actual = {right_slope:.4f} * expected + {right_intercept:.2f}")
print(f"  R-squared: {right_r2:.4f}")
print(f"  RMSE: {right_rmse:.2f} mm/s")

print(f"\nSUGGESTED CORRECTION FACTORS:")
print(f"  Left wheel: multiply desired speed by {1/left_slope:.4f}")
print(f"  Right wheel: multiply desired speed by {1/right_slope:.4f}")
