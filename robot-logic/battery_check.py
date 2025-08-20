import serial
import time
import struct
from interface import OPCODE_START, OPCODE_CHANGE_STREAM_STATUS, OPCODE_STOP, OPCODE_SEND_SENSORS

def get_battery_percentage():
    roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

    # Initialize connection
    roomba.write(OPCODE_START)
    time.sleep(0.2)
    roomba.write(OPCODE_CHANGE_STREAM_STATUS + bytes([0]))

    # Request battery charge (packet 25) and capacity (packet 26)
    roomba.read_all()
    roomba.write(OPCODE_SEND_SENSORS + bytes([2, 25, 26]))
    time.sleep(0.1)

    # Read response
    data = roomba.read(4)

    # Parse battery charge (packet 25)
    charge = struct.unpack('>H', data[0:2])[0]
    # Parse battery capacity (packet 26)
    capacity = struct.unpack('>H', data[2:4])[0]
    print(data, charge, capacity)

    if capacity > 0:
        percentage = (charge / capacity) * 100
    else:
        percentage = 0

    roomba.write(OPCODE_STOP)
    roomba.close()

    return round(percentage, 1)

if __name__ == "__main__":
    try:
        percentage = get_battery_percentage()
        print(f"{percentage}%")
    except Exception as e:
        print(f"Error: {e}")
