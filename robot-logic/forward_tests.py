import serial
import time
from interface import ( OPCODE_START, OPCODE_SAFE, OPCODE_STOP, OPCODE_SEND_SENSORS, OPCODE_DRIVE_DIRECT )

roomba = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

roomba.write(OPCODE_START)
roomba.write(OPCODE_SAFE)
time.sleep(0.2)

def drive(left, right):
  roomba.write(OPCODE_DRIVE_DIRECT + left.to_bytes(2, "big", signed=True) + right.to_bytes(2, "big", signed=True))

drive(100, 100)
time.sleep(20)

drive(0, 0)
time.sleep(0.2)
roomba.write(OPCODE_STOP)
time.sleep(0.2)
