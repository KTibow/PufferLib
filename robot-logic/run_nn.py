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
        self.encoder = nn.Sequential(nn.Linear(3, 128), nn.GELU())  # 3 inputs: left bumper importance, right bumper importance, light bumper
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
    """Read bumper sensors (packet 7) and light bumper sensors (packet 45)"""
    roomba.read_all()
    roomba.write(OPCODE_SEND_SENSORS + bytes([2, 7, 45]))

    # SEND_SENSORS returns only the data bytes, not packet ids.
    # Bumper packet (7) is 1 byte, light bumper packet (45) is 1 byte.
    expected_length = 2
    data = roomba.read(expected_length)
    print(data)
    if len(data) != expected_length:
        return False, False, 0.0  # Default values if read fails

    # Parse bumper data (first byte)
    bumper_byte = data[0]
    right_bump = bool(bumper_byte & 0x01)  # Bit 0
    left_bump = bool((bumper_byte & 0x02) >> 1)  # Bit 1

    # Parse light bumper data (second byte)
    light_byte = data[1]

    # Return 1.0 if any light bumper bit is set (byte != 0), 0.0 otherwise
    light_bumper = 1.0 if light_byte != 0 else 0.0

    return left_bump, right_bump, light_bumper

def main():
    # Load trained model
    print("Loading model...")
    net = RoombaNet()
    state_dict = torch.load("puffer_roomba_EX-122.pt", map_location="cpu")
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
    roomba.write(OPCODE_MOTORS + bytes([0b00000110]))

    # Initialize bumper importance values and light bumper
    left_bumper_importance = 0.0
    right_bumper_importance = 0.0
    light_bumper = 0.0  # Binary light bumper from actual sensors

    try:
        step = 0
        while True:
            start_time = time.time()
            # Read all sensors
            left_bump, right_bump, light_bumper = read_sensors(roomba)

            # Update bumper importance values
            if left_bump:
                left_bumper_importance = 2.0
            else:
                left_bumper_importance = max(0.0, left_bumper_importance - 0.05)

            if right_bump:
                right_bumper_importance = 2.0
            else:
                right_bumper_importance = max(0.0, right_bumper_importance - 0.05)

            # Create observation array with 3 values
            obs = np.array([left_bumper_importance, right_bumper_importance, light_bumper], dtype=np.float32)

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
            print(f"Step {step:03d} ({processing_time:.3f}s): bumps {left_bumper_importance:.2f},{right_bumper_importance:.2f} light {light_bumper:.0f} ({int(left_bump)},{int(right_bump)}) -> actions={actions} speeds=({left_speed:.0f}, {right_speed:.0f}) mm/s [factor={speed_factor}]")
            if processing_time < actual_dt:
                time.sleep(actual_dt - processing_time)
            step += 1

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        drive(roomba, 0, 0)
        roomba.write(OPCODE_MOTORS + bytes([0b00000000]))
        roomba.write(OPCODE_STOP)
        roomba.close()

if __name__ == "__main__":
    main()
