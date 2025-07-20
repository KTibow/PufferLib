#!/usr/bin/env python3
import serial
import time
import torch
import torch.nn as nn
import numpy as np
from interface import (OPCODE_MOTORS, OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_DRIVE_DIRECT,
                      OPCODE_SEND_SENSOR)

max_speed = 500
speed_factor = 0.5
dt = 0.05
actual_dt = dt / speed_factor

# Simple neural network matching the trained model structure
class RoombaNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = nn.Sequential(nn.Linear(3, 128))  # Changed input size from 2 to 3
        self.decoder_mean = nn.Linear(128, 2)  # 128 hidden -> 2 wheel speeds
        self.decoder_logstd = nn.Parameter(torch.zeros(1, 2))
        self.value = nn.Linear(128, 1)  # Value function (not used for inference)

    def forward(self, x):
        hidden = torch.relu(self.encoder(x))
        mean = self.decoder_mean(hidden)
        return mean

def drive(roomba, left_speed, right_speed):
    # Clamp to valid range and convert to bytes
    left = max(-max_speed, min(max_speed, int(left_speed)))
    right = max(-max_speed, min(max_speed, int(right_speed)))
    cmd = OPCODE_DRIVE_DIRECT + left.to_bytes(2, "big", signed=True) + right.to_bytes(2, "big", signed=True)
    roomba.write(cmd)

def read_bumpers(roomba):
    """Read bumper sensors using packet 7"""
    roomba.read_all()
    roomba.write(OPCODE_SEND_SENSOR + bytes([7]))
    time.sleep(0.01)  # Small delay for response
    data = roomba.read(1)
    if len(data) != 1:
        return False, False  # Default to no bump if read fails

    byte_val = data[0]
    right_bump = bool(byte_val & 0x01)  # Bit 0
    left_bump = bool((byte_val & 0x02) >> 1)  # Bit 1
    return left_bump, right_bump

def main():
    # Load trained model
    print("Loading model...")
    net = RoombaNet()
    state_dict = torch.load("puffer_roomba_EX-117.pt", map_location="cpu")
    net.load_state_dict(state_dict)
    net.eval()
    print("Model loaded!")

    # Connect to Roomba
    roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)
    print("Setting up Roomba...")
    roomba.write(OPCODE_START)
    roomba.write(OPCODE_SAFE)
    time.sleep(0.2)
    roomba.write(bytes([150, 0]))
    time.sleep(0.1)
    roomba.read_all()
    print("Roomba ready!")

    print("Running neural network control... Press Ctrl+C to stop")
    roomba.write(OPCODE_MOTORS + bytes([0b00000110]))

    # Initialize bumper importance values
    left_bumper_importance = 0.0
    right_bumper_importance = 0.0

    try:
        step = 0
        while True:
            start_time = time.time()
            # Read current bumper states
            left_bump, right_bump = read_bumpers(roomba)

            # Update bumper importance values
            if left_bump:
                left_bumper_importance = 2.0
            else:
                left_bumper_importance = max(0.0, left_bumper_importance - 0.05)

            if right_bump:
                right_bumper_importance = 2.0
            else:
                right_bumper_importance = max(0.0, right_bumper_importance - 0.05)

            # Calculate minutes since start based on steps and dt
            minutes_since_start = (step * dt) / 60.0

            # Create observation array with 3 values
            obs = np.array([left_bumper_importance, right_bumper_importance, minutes_since_start], dtype=np.float32)

            # Run neural network
            with torch.no_grad():
                obs_tensor = torch.from_numpy(obs).unsqueeze(0)
                actions = net(obs_tensor)[0].numpy()

            # Scale actions from [-1,1] to wheel speeds in mm/s
            # Max wheel speed is 50 cm/s = 500 mm/s according to roomba.h
            base_left_speed = actions[0] * max_speed
            base_right_speed = actions[1] * max_speed

            # Apply speed factor to wheel speeds
            left_speed = base_left_speed * speed_factor
            right_speed = base_right_speed * speed_factor

            # Send to robot
            drive(roomba, left_speed, right_speed)

            processing_time = time.time() - start_time
            # Calculate actual timestep duration (extended by 1/speed_factor to maintain distance)
            print(f"Step {step:03d} ({processing_time:.3f}s): bumps {left_bumper_importance:.2f},{right_bumper_importance:.2f} ({int(left_bump)},{int(right_bump)}) min_since_start={minutes_since_start:.2f} -> actions={actions} speeds=({left_speed:.0f}, {right_speed:.0f}) mm/s [factor={speed_factor}]")
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
