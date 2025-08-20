import serial
import time
import struct
import datetime
from interface import (PACKETS, OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_STREAM_SENSORS)

# Device codes for dock debugging
DEVICE_CODES = {
    160: "Roomba 600 (Reserved)",
    161: "Drive-on Charger (Force Field)",
    164: "Green Buoy",
    165: "Green Buoy + Force Field",
    168: "Red Buoy",
    169: "Red Buoy + Force Field",
    172: "Red Buoy + Green Buoy",
    173: "Red Buoy + Green Buoy + Force Field"
}

roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

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

def colorize_signal(signal):
    """Colorize G/R/F as green/red/blue using ANSI escape codes"""
    # Map each character to color
    color_map = {
        'G': '\033[32mG\033[0m',  # Green
        'R': '\033[31mR\033[0m',  # Red
        'F': '\033[34mF\033[0m',  # Blue
        ' ': ' ',
        '?': '\033[90m?\033[0m'
    }
    # Replace each character with color if present
    return ''.join(color_map.get(c, c) for c in signal)

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

def visualize_ir_status(ir_left, ir_center, ir_right):
    """Show IR sensor status as Left / Center / Right, colorized"""
    left_signals = colorize_signal(decode_ir_signals(ir_left))
    center_signals = colorize_signal(decode_ir_signals(ir_center))
    right_signals = colorize_signal(decode_ir_signals(ir_right))

    return f"{left_signals}/{center_signals}/{right_signals}"

# def log_to_file(timestamp, ir_center, ir_left, ir_right, dirt_detect):
#     """Log readings to file"""
#     with open("ir_readings.log", "a") as f:
#         f.write(f"{timestamp},{ir_center},{ir_left},{ir_right},{dirt_detect}\n")

def main():
    print("Starting IR tracking for dock debugging...")
    print("IR sensor packets: 17 (center), 52 (left), 53 (right), 15 (dirt detect)")
    print("Press Ctrl+C to stop")
    print()

    # # Initialize log file
    # with open("ir_readings.log", "w") as f:
    #     f.write("timestamp,ir_center,ir_left,ir_right,dirt_detect\n")

    roomba.write(OPCODE_START)
    time.sleep(0.2)
    roomba.write(OPCODE_SAFE)
    time.sleep(0.2)

    # Stream IR sensors and dirt detect
    roomba.write(OPCODE_STREAM_SENSORS + bytes([4, 17, 52, 53]))
    time.sleep(0.1)
    roomba.read_all()

    print("\033[?25l", end="")  # Hide cursor
    try:
        while True:
            data = wait_and_read_stream([17, 52, 53])
            if data is None:
                continue

            ir_center, ir_left, ir_right = data
            timestamp = datetime.datetime.now().isoformat()

            # Create visualization showing decoded signals
            ir_status = visualize_ir_status(ir_left, ir_center, ir_right)

            # Display readings with raw values and decoded signals
            print(f"\r{timestamp} | Signals: {ir_status:15}",
                  end="", flush=True)

    except KeyboardInterrupt:
        print("\nStopping IR tracking...")
    finally:
        print("\033[?25h", end="")  # Show cursor
        roomba.write(OPCODE_STOP)
        # print(f"\nReadings saved to ir_readings.log")

if __name__ == "__main__":
    main()
