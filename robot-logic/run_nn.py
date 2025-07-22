#!/usr/bin/env python3
print("Importing...")
import serial
import time
import torch
import torch.nn as nn
import numpy as np
from interface import (OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_DRIVE_DIRECT,  # noqa: E402
                      OPCODE_SEND_SENSORS)

max_speed = 250
speed_factor = 1
dt = 0.05
actual_dt = dt / speed_factor

# Simple neural network matching the trained model structure
class RoombaNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = nn.Sequential(nn.Linear(4, 128), nn.GELU())  # 4 inputs: [left_bumper, right_bumper, left_bumper_memory, right_bumper_memory]
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

def calculate_light_bumper_strength(raw_value):
    """Scale raw sensor value (0-4095) to light bumper strength.
    Scale 0-1000 range to 0-0.5 and clamp at 0.5 max.
    """
    return min(0.5, raw_value / 1000.0 * 0.5)

def read_sensors(roomba):
    """Read bumper sensors (packet 7) and analog light bumper sensors (packets 46-51)
    Returns unified bumper values matching simulation format.
    """
    roomba.read_all()
    # Request bumper (7) and all 6 analog light bumper sensors (46-51)
    roomba.write(OPCODE_SEND_SENSORS + bytes([7, 7, 46, 47, 48, 49, 50, 51]))

    # SEND_SENSORS returns only the data bytes, not packet ids.
    # Bumper packet (7) is 1 byte, each light bumper packet (46-51) is 2 bytes.
    expected_length = 1 + 6 * 2  # 1 + 12 = 13 bytes total
    data = roomba.read(expected_length)
    if len(data) != expected_length:
        return 0.0, 0.0  # Default unified bumper values if read fails

    # Parse bumper data (first byte)
    bumper_byte = data[0]
    right_bump = bool(bumper_byte & 0x01)  # Bit 0
    left_bump = bool((bumper_byte & 0x02) >> 1)  # Bit 1

    # Parse analog light bumper data (remaining 12 bytes, 2 bytes per sensor)
    # Sensors: Left (46), Front Left (47), Center Left (48), Center Right (49), Front Right (50), Right (51)
    light_strengths = []
    for i in range(6):
        byte_offset = 1 + i * 2  # Start after bumper byte, 2 bytes per sensor
        high_byte = data[byte_offset]
        low_byte = data[byte_offset + 1]
        raw_value = (high_byte << 8) | low_byte  # Combine to 12-bit value (0-4095)
        strength = calculate_light_bumper_strength(raw_value)
        light_strengths.append(strength)

    # Aggregate sensors into left/right groups (matching simulation logic)
    # Left group: sensors 0, 1, 2 (Left, Front Left, Center Left)
    # Right group: sensors 3, 4, 5 (Center Right, Front Right, Right)
    left_light_strength = max(light_strengths[0], light_strengths[1], light_strengths[2])
    right_light_strength = max(light_strengths[3], light_strengths[4], light_strengths[5])

    # Create unified bumper values: physical bumper forces to 1.0, otherwise use light bumper strength
    left_unified = 1.0 if left_bump else left_light_strength
    right_unified = 1.0 if right_bump else right_light_strength

    return left_unified, right_unified

def main():
    # Load trained model
    print("Loading model...")
    net = RoombaNet()
    state_dict = torch.load("puffer_roomba_EX-146.pt", map_location="cpu")
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

    # Initialize unified bumper values and memory (matching simulation format)
    left_bumper = 0.0  # Current unified bumper value (1.0 for physical bump, 0.0-0.5 for light bumper)
    right_bumper = 0.0  # Current unified bumper value
    left_bumper_memory = 0.0  # Decaying memory of left bumper hits
    right_bumper_memory = 0.0  # Decaying memory of right bumper hits

    try:
        step = 0
        while True:
            start_time = time.time()
            # Read unified bumper sensors (combines physical bumper + light bumper)
            left_bumper, right_bumper = read_sensors(roomba)

            # Update memory with unified logic: take maximum of current unified bumper value or existing memory
            left_bumper_memory = max(left_bumper_memory, left_bumper)
            right_bumper_memory = max(right_bumper_memory, right_bumper)

            # Decay memory values by subtracting timestep/3 for 3 second decay (minimum 0.0)
            left_bumper_memory = max(0.0, left_bumper_memory - dt/3)
            right_bumper_memory = max(0.0, right_bumper_memory - dt/3)

            # Create observation array matching simulation format: [left_bumper, right_bumper, left_bumper_memory, right_bumper_memory]
            obs = np.array([left_bumper, right_bumper, left_bumper_memory, right_bumper_memory], dtype=np.float32)

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
            print(f"Step {step:03d} ({processing_time:.3f}s): unified_bumpers {left_bumper:.3f},{right_bumper:.3f} memory {left_bumper_memory:.3f},{right_bumper_memory:.3f} -> actions={actions} speeds=({left_speed:.0f}, {right_speed:.0f}) mm/s [factor={speed_factor}]")
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
