#!/usr/bin/env python3
print("Importing...")
import serial
import time
import torch
import torch.nn as nn
import numpy as np
import struct
from interface import (OPCODE_MOTORS, OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_DRIVE_DIRECT,
                      OPCODE_SEND_SENSOR, OPCODE_SEND_SENSORS, PACKETS)

max_speed = 250
speed_factor = 1
dt = 0.05
actual_dt = dt / speed_factor

# Simple neural network matching the trained model structure
class RoombaNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = nn.Sequential(nn.Linear(4, 128), nn.GELU())  # 4 inputs: left bumper importance, right bumper importance, left light bumper, right light bumper
        self.decoder_mean = nn.Linear(128, 2)  # 128 hidden -> 2 wheel speeds
        self.decoder_logstd = nn.Parameter(torch.zeros(1, 2))
        self.value = nn.Linear(128, 1)  # Value function (not used for inference)

    def forward(self, x):
        hidden = self.encoder(x)
        mean = self.decoder_mean(hidden)
        return mean

def drive(roomba, left_speed, right_speed):
    # Clamp to valid range and convert to bytes
    left = max(-max_speed, min(max_speed, int(left_speed)))
    right = max(-max_speed, min(max_speed, int(right_speed)))
    cmd = OPCODE_DRIVE_DIRECT + right.to_bytes(2, "big", signed=True) + left.to_bytes(2, "big", signed=True)
    roomba.write(cmd)

def read_sensors(roomba):
    """Read bumper sensors (packet 7) and analog light bumper sensors (packets 46-51)"""
    roomba.read_all()
    # Request bumper (7) and all 6 analog light bumper sensors (46-51)
    roomba.write(OPCODE_SEND_SENSORS + bytes([7, 7, 46, 47, 48, 49, 50, 51]))

    # SEND_SENSORS returns only the data bytes, not packet ids.
    # Bumper packet (7) is 1 byte, each light bumper packet (46-51) is 2 bytes.
    expected_length = 1 + 6 * 2  # 1 + 12 = 13 bytes total
    data = roomba.read(expected_length)
    if len(data) != expected_length:
        return False, False, 0.0, 0.0  # Default values if read fails

    # Parse bumper data (first byte)
    bumper_byte = data[0]
    right_bump = bool(bumper_byte & 0x01)  # Bit 0
    left_bump = bool((bumper_byte & 0x02) >> 1)  # Bit 1

    # Parse analog light bumper data (remaining 12 bytes, 2 bytes per sensor)
    # Sensors: Left (46), Front Left (47), Center Left (48), Center Right (49), Front Right (50), Right (51)
    light_sensors = []
    for i in range(6):
        byte_offset = 1 + i * 2  # Start after bumper byte, 2 bytes per sensor
        high_byte = data[byte_offset]
        low_byte = data[byte_offset + 1]
        raw_value = (high_byte << 8) | low_byte  # Combine to 12-bit value (0-4095)
        # Scale 0-1000 range to 0-1 with clamping as requested
        scaled_value = min(1.0, max(0.0, raw_value / 1000.0))
        light_sensors.append(scaled_value)

    # Aggregate sensors into left/right groups (matching simulation logic)
    # Left group: sensors 0, 1, 2 (Left, Front Left, Center Left)
    # Right group: sensors 3, 4, 5 (Center Right, Front Right, Right)
    left_light_bumper = max(light_sensors[0], light_sensors[1], light_sensors[2])
    right_light_bumper = max(light_sensors[3], light_sensors[4], light_sensors[5])

    return left_bump, right_bump, left_light_bumper, right_light_bumper

def main():
    # Load trained model
    print("Loading model...")
    net = RoombaNet()
    state_dict = torch.load("puffer_roomba_EX-125.pt", map_location="cpu")
    net.load_state_dict(state_dict)
    net.eval()

    # Connect to Roomba
    print("Setting up Roomba...")
    roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)
    roomba.write(OPCODE_START)
    roomba.write(OPCODE_SAFE)
    time.sleep(0.2)
    roomba.write(bytes([150, 0]))
    time.sleep(0.1)
    roomba.read_all()

    print("Running neural network control... Press Ctrl+C to stop")
    # roomba.write(OPCODE_MOTORS + bytes([0b00000110]))

    # Initialize bumper importance values and light bumpers
    left_bumper_importance = 0.0
    right_bumper_importance = 0.0
    left_light_bumper = 0.0  # Analog left light bumper strength from actual sensors (0-1)
    right_light_bumper = 0.0  # Analog right light bumper strength from actual sensors (0-1)

    try:
        step = 0
        while True:
            start_time = time.time()
            # Read all sensors
            left_bump, right_bump, left_light_bumper, right_light_bumper = read_sensors(roomba)

            # Update bumper importance values
            if left_bump:
                left_bumper_importance = 2.0
            else:
                left_bumper_importance = max(0.0, left_bumper_importance - 0.05)

            if right_bump:
                right_bumper_importance = 2.0
            else:
                right_bumper_importance = max(0.0, right_bumper_importance - 0.05)

            # Create observation array with 4 values
            obs = np.array([left_bumper_importance, right_bumper_importance, left_light_bumper, right_light_bumper], dtype=np.float32)

            # Run neural network
            with torch.no_grad():
                obs_tensor = torch.from_numpy(obs).unsqueeze(0)
                actions = net(obs_tensor)[0].numpy()

            # Scale actions from [-1,1] to wheel speeds in mm/s
            base_left_speed = actions[0] * max_speed
            base_right_speed = actions[1] * max_speed

            # Apply speed factor to wheel speeds
            left_speed = base_left_speed * speed_factor
            right_speed = base_right_speed * speed_factor

            # Send to robot
            drive(roomba, left_speed, right_speed)

            processing_time = time.time() - start_time
            # Calculate actual timestep duration (extended by 1/speed_factor to maintain distance)
            print(f"Step {step:03d} ({processing_time:.3f}s): bumps {left_bumper_importance:.2f},{right_bumper_importance:.2f} light {left_light_bumper:.3f},{right_light_bumper:.3f} ({int(left_bump)},{int(right_bump)}) -> actions={actions} speeds=({left_speed:.0f}, {right_speed:.0f}) mm/s [factor={speed_factor}]")
            if processing_time < actual_dt:
                time.sleep(actual_dt - processing_time)
            step += 1

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        drive(roomba, 0, 0)
        # roomba.write(OPCODE_MOTORS + bytes([0b00000000]))
        roomba.write(OPCODE_STOP)
        roomba.close()

if __name__ == "__main__":
    main()
