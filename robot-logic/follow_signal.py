import serial
import time
import struct
import datetime
from interface import (PACKETS, OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_STREAM_SENSORS, OPCODE_DRIVE_DIRECT, OPCODE_LEDS)

roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

# Configuration: Set the signal type to follow and turn direction
FOLLOW_SIGNAL = "red"  # Options: "force_field", "green", "red"
FOLLOW_SIDE = "right"           # Options: "left", "right" (turn direction when not following signal)
STAY_WITHIN_SIGNAL = False       # True: stay within signal, False: stay outside signal

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

def drive(left, right):
    roomba.write(OPCODE_DRIVE_DIRECT + right.to_bytes(2, "big", signed=True) + left.to_bytes(2, "big", signed=True))

def set_led_color(correction_needed):
    if correction_needed:
        # Red: hue=255, intensity=255
        roomba.write(OPCODE_LEDS + bytes([0, 255, 255]))
    else:
        # Green: hue=0, intensity=255
        roomba.write(OPCODE_LEDS + bytes([0, 0, 255]))

def has_force_field(ir_code):
    """Check if IR code indicates force field detection"""
    force_field_codes = [161, 165, 169, 173]  # All codes with force field
    return ir_code in force_field_codes

def has_green_buoy(ir_code):
    """Check if IR code indicates green buoy detection"""
    green_buoy_codes = [164, 165, 172, 173]  # All codes with green buoy
    return ir_code in green_buoy_codes

def has_red_buoy(ir_code):
    """Check if IR code indicates red buoy detection"""
    red_buoy_codes = [168, 169, 172, 173]  # All codes with red buoy
    return ir_code in red_buoy_codes

def decode_ir_signals(code):
    """Decode IR character code to signal abbreviation"""
    if code == 0:
        return "   "
    elif code == 160:
        return "???"      # Reserved
    elif code == 161:
        return "  F"      # Force Field
    elif code == 164:
        return "G  "      # Green Buoy
    elif code == 165:
        return "G F"     # Green Buoy + Force Field
    elif code == 168:
        return " R "      # Red Buoy
    elif code == 169:
        return " RF"     # Red Buoy + Force Field
    elif code == 172:
        return "GR "     # Red Buoy + Green Buoy
    elif code == 173:
        return "GRF"    # Red Buoy + Green Buoy + Force Field
    else:
        return f"{code:3}"

class SignalFilter:
    """Filter glitchy IR readings to determine signal status"""
    def __init__(self, required_readings=3):
        self.required_readings = required_readings
        self.no_signal_count = 0
        self.current_status = False  # Start assuming no signal

    def update(self, ir_center, ir_left, ir_right, signal_type="force_field"):
        """Update filter with new IR readings and return filtered signal status"""
        # Check if any sensor detects the specified signal
        if signal_type == "force_field":
            signal_detected = (has_force_field(ir_center) or
                             has_force_field(ir_left) or
                             has_force_field(ir_right))
        elif signal_type == "green":
            signal_detected = (has_green_buoy(ir_center) or
                             has_green_buoy(ir_left) or
                             has_green_buoy(ir_right))
        elif signal_type == "red":
            signal_detected = (has_red_buoy(ir_center) or
                             has_red_buoy(ir_left) or
                             has_red_buoy(ir_right))
        else:
            signal_detected = False

        if signal_detected:
            # Signal detected, reset counter and set status
            self.no_signal_count = 0
            self.current_status = True
        else:
            # No signal detected, increment counter
            self.no_signal_count += 1
            # Only change status if we've had enough consecutive non-detections
            if self.no_signal_count >= self.required_readings:
                self.current_status = False

        return self.current_status

def main():
    print("Starting signal following robot...")
    print(f"Following: {FOLLOW_SIGNAL}")
    print(f"Logic: {FOLLOW_SIGNAL} detected -> go straight, No {FOLLOW_SIGNAL} -> turn {FOLLOW_SIDE}")
    print(f"Stay {'within' if STAY_WITHIN_SIGNAL else 'outside'} the signal")
    print("Press Ctrl+C to stop")
    print()

    roomba.write(OPCODE_START)
    time.sleep(0.2)
    roomba.write(OPCODE_SAFE)
    time.sleep(0.2)

    # Stream IR sensors
    roomba.write(OPCODE_STREAM_SENSORS + bytes([3, 17, 52, 53]))
    time.sleep(0.1)
    roomba.read_all()

    filter = SignalFilter(required_readings=8)
    last_led_state = None  # Track LED state to avoid unnecessary updates

    # Movement speeds
    speed = 38  # mm/s

    print("\033[?25l", end="")  # Hide cursor
    try:
        while True:
            data = wait_and_read_stream([17, 52, 53])
            if data is None:
                continue

            ir_center, ir_left, ir_right = data
            timestamp = datetime.datetime.now().isoformat()

            # Update signal filter
            signal_active = filter.update(ir_center, ir_left, ir_right, FOLLOW_SIGNAL)

            # Control robot based on signal status and STAY_WITHIN_SIGNAL toggle
            correction_needed = (not signal_active) if STAY_WITHIN_SIGNAL else signal_active

            # Update LED if state changed
            if last_led_state != correction_needed:
                set_led_color(correction_needed)
                last_led_state = correction_needed

            # Stay within signal: go straight if signal detected, turn if not
            if correction_needed:
                if FOLLOW_SIDE == "left":
                    drive(-speed, speed)
                    action = "TURN LEFT  "
                else:
                    drive(speed, -speed)
                    action = "TURN RIGHT "
            else:
                if FOLLOW_SIDE == "left":
                    drive(speed + 2, speed)
                else:
                    drive(speed, speed + 2)
                action = "GO STRAIGHT"

            # Display status
            left_sig = decode_ir_signals(ir_left)
            center_sig = decode_ir_signals(ir_center)
            right_sig = decode_ir_signals(ir_right)

            status = "ACTIVE" if signal_active else "INACTIVE"

            print(f"\r{timestamp} | IR: {left_sig}/{center_sig}/{right_sig} | "
                  f"{FOLLOW_SIGNAL.title()}: {status:8} | Action: {action} | "
                  f"Filter count: {filter.no_signal_count}",
                  end="", flush=True)

    except KeyboardInterrupt:
        print("\nStopping robot...")
    finally:
        print("\033[?25h", end="")  # Show cursor
        drive(0, 0)  # Stop robot
        time.sleep(0.2)
        roomba.write(OPCODE_STOP)

if __name__ == "__main__":
    main()
