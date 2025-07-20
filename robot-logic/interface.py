"""
Opcodes for the iRobot Create 2.
"""

PACKETS = {
    7: ('>B', 1),
    15: ('>B', 1),
    43: ('>h', 2),
    44: ('>h', 2),
    45: ('>B', 1),
    46: ('>H', 2),
    47: ('>H', 2),
    48: ('>H', 2),
    49: ('>H', 2),
    50: ('>H', 2),
    51: ('>H', 2),
}

OPCODE_START: bytes = (128).to_bytes(1, "big")
"""
This command starts the OI, so you have to send it before any other commands.
If the robot's currently in another mode, it will set it to Passive mode.

Format: [128]
Available: Always available.
Mode change: Sets mode to Passive, and the robot will beep.
"""
OPCODE_RESET: bytes = (7).to_bytes(1, "big")
"""
This command resets the robot, as if you had removed and reinserted the battery.

Format: [7]
Available: Always available.
Mode change: Exits the OI, and the robot will make a tune when it's done.
"""
OPCODE_STOP: bytes = (173).to_bytes(1, "big")
"""
This command stops the OI. Streams will stop, and commands won't work.

Format: [173]
Available: if the OI is connected.
Mode change: Exits the OI, and the robot will beep.
"""
OPCODE_BAUD: bytes = (129).to_bytes(1, "big")
"""
This command sets the baud rate in bits per second (bps).
The default baud rate is 115200 bps, but [it can be changed](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=4) to 19200 bps.
The baud rate is held unless robot has to be restarted.
You must wait 100ms after sending this command before sending more commands at the new baud rate.

Format: [129, [baud rate](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=8)]
Available: if the OI is connected.
Mode change: Doesn't change mode.
"""


OPCODE_SAFE: bytes = (131).to_bytes(1, "big")
"""
This command puts the robot in safe mode.
It allows you to control the robot, but it will exit if any problem is detected.
All LEDs will be turned off.

Format: [131]
Available: if the OI is connected.
Mode change: Changes mode to Safe.
"""
OPCODE_FULL: bytes = (132).to_bytes(1, "big")
"""
This command puts the robot in full mode.
It allows you to control the robot with safety features turned off.

Format: [132]
Available: if the OI is connected.
Mode change: Changes mode to Full.
"""


OPCODE_CLEAN: bytes = (135).to_bytes(1, "big")
"""
This command starts a normal cleaning cycle.
If it's already cleaning, it will pause any cycle.

Format: [135]
Available: if the OI is connected.
Mode change: Changes mode to Passive.
"""
OPCODE_SPOT: bytes = (134).to_bytes(1, "big")
"""
This command starts a spot cleaning cycle.
If it's already cleaning, it will pause any cycle.

Format: [134]
Available: if the OI is connected.
Mode change: Changes mode to Passive.
"""
OPCODE_DOCK: bytes = (143).to_bytes(1, "big")
"""
This command tells the robot to go around until it sees the dock, then to drive onto it.
If it's already cleaning, it will pause any cycle.

Format: [143]
Available: if the OI is connected.
Mode change: Changes mode to Passive.
"""
OPCODE_POWER: bytes = (133).to_bytes(1, "big")
"""
This command turns off the robot.

Format: [133]
Available: if the OI is connected.
Mode change: Exits the OI.
"""
OPCODE_SCHEDULE: bytes = (167).to_bytes(1, "big")
"""
This command sets the cleaning schedule.
If the robot is already in the schedule/clock UX, it won't work.

Time format:
| Day | Code | Hour | Code | Minute | Code |
|-----|------|------|------|--------|------|
| Sun | 0    | 1AM  | 0    | 00     | 0    |
| Mon | 1    | 5AM  | 5    | 10     | 10   |
| Tue | 2    | 10AM | 10   | 20     | 20   |
| Wed | 3    | 12PM | 12   | 30     | 30   |
| Thu | 4    | 3PM  | 15   | 40     | 40   |
| Fri | 5    | 6PM  | 18   | 50     | 50   |
| Sat | 6    | 11PM | 23   | 00     | 60   |

Format: [167, [some kind of checksum](http://www.robotreviews.com/chat/viewtopic.php?f=1&t=12328), Sun hour, Sun minute, Mon hour, Mon minute, etc.]
Available: if the OI is connected.
Mode change: Doesn't change mode.
"""
OPCODE_CLOCK: bytes = (168).to_bytes(1, "big")
"""
This command sets the time.
If the robot is already in the schedule/clock UX, it won't work.

Time format:
| Day | Code | Hour | Code | Minute | Code |
|-----|------|------|------|--------|------|
| Sun | 0    | 1AM  | 0    | 00     | 0    |
| Mon | 1    | 5AM  | 5    | 10     | 10   |
| Tue | 2    | 10AM | 10   | 20     | 20   |
| Wed | 3    | 12PM | 12   | 30     | 30   |
| Thu | 4    | 3PM  | 15   | 40     | 40   |
| Fri | 5    | 6PM  | 18   | 50     | 50   |
| Sat | 6    | 11PM | 23   | 00     | 60   |

Format: [168, day, hour, minute]
Available: if the OI is connected.
Mode change: Doesn't change mode.
"""


OPCODE_DRIVE: bytes = (137).to_bytes(1, "big")
"""
This command controls the robot's drive wheels (with speed/turn amount).
This is a weird/proprietary command, so [check the official docs](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=12).
Basically, if it's negative, you subtract the value from 65535 (positive is normal).

Format: [137, speed (2 bits, big endian, -500-500), radius (2 bits, big endian, -2000-2000)]
Available: in Safe/Full mode.
Mode change: Doesn't change mode.
"""
OPCODE_DRIVE_DIRECT: bytes = (145).to_bytes(1, "big")
"""
This command controls the robot's drive wheels (with mm/s for each wheel).
This is a weird/proprietary command, so [check the official docs](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=13).
Basically, if it's negative, you subtract the value from 65535 (positive is normal).

Format: [145, left wheel (2 bits, big endian, -500-500), radius (2 bits, right wheel, -500-500)]
Available: in Safe/Full mode.
Mode change: Doesn't change mode.
"""
OPCODE_DRIVE_PWM: bytes = (146).to_bytes(1, "big")
"""
This command controls the robot's drive wheels (with how much power to send to each wheel).
This is a weird/proprietary command, so [check the official docs](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=13).
Basically, if it's negative, you subtract the value from 65535 (positive is normal).

Format: [146, left wheel (2 bits, big endian, -255-255), right wheel (2 bits, big endian, -255-255)]
Available: in Safe/Full mode.
Mode change: Doesn't change mode.
"""
OPCODE_MOTORS: bytes = (138).to_bytes(1, "big")
"""
This command lets you toggle the brushes and vacuum.
The byte sent is a [combination of multiple bits](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=14).

Format: [138, byte]
Available: in Safe/Full mode.
Mode change: Doesn't change mode.
"""
OPCODE_MOTORS_PWM: bytes = (144).to_bytes(1, "big")
"""
This command lets you set the power of the brushes and vacuum.
Basically, if it's negative, you subtract the value from 255 (positive is normal).

Format: [144, main brush (-127-127), side brush (-127-127), vacuum (0-127)]
Available: in Safe/Full mode.
Mode change: Doesn't change mode.
"""
OPCODE_LEDS: bytes = (139).to_bytes(1, "big")
"""
This command lets you set the LEDs.
The first data byte sent is a [combination of multiple bits](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=15).

Format: [139, byte, Clean led hue, Clean led brightness]
Available: in Safe/Full mode.
Mode change: Doesn't change mode.
"""
OPCODE_SCHEDULE_LEDS: bytes = (162).to_bytes(1, "big")
"""
This command lets you set the LEDs surrounding the display for the scheduling system.
The data bytes are a [combination of multiple bits](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=15).

Format: [162, byte, byte]
Available: in Safe/Full mode.
Mode change: Doesn't change mode.
"""
OPCODE_SCHEDULE_DISPLAY: bytes = (163).to_bytes(1, "big")
"""
This command lets you set the display for the scheduling system.
The data bytes are a [combination of multiple bits](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=16).

Format: [162, bytes for display from left to right x4]
Available: in Safe/Full mode.
Mode change: Doesn't change mode.
"""
OPCODE_EMULATE_BUTTONS: bytes = (165).to_bytes(1, "big")
"""
This command lets you push buttons on the robot for 1/6th of a second.
The data byte is a [combination of multiple bits](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=16).

Format: [165, byte]
Available: if the OI is connected.
Mode change: Doesn't change mode.
"""
OPCODE_SCHEDULE_DISPLAY_ASCII: bytes = (164).to_bytes(1, "big")
"""
This command lets you set the display for the scheduling system, but with ASCII characters.

Format: [164, characters for display from left to right x4]
Available: in Safe/Full mode.
Mode change: Doesn't change mode.
"""
OPCODE_STORE_SONG: bytes = (140).to_bytes(1, "big")
"""
This command lets you store a song (up to 4 at a time).
View the [official note list](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=18).

Among Us: `roomba.write((140).to_bytes(1, "big") + b"\x00\x0B\x40\x16\x43\x16\x46\x16\x49\x16\x46\x16\x43\x16\x40\x16\x00\x32\x40\x0C\x43\x0C\x40\x0C")`

Format: [140, song number (0-3), song note count (1-16), note 1 tone, note 1 length, etc.]
Available: if the OI is connected.
Mode change: Doesn't change mode.
"""
OPCODE_PLAY_SONG: bytes = (141).to_bytes(1, "big")
"""
This command lets you play a song.

Format: [141, song number (0-3)]
Available: if the OI is connected.
Mode change: Doesn't change mode.
"""


OPCODE_SEND_SENSOR: bytes = (142).to_bytes(1, "big")
"""
This command lets you request a sensor packet (check [the sensor packet docs](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=22)).

Format: [142, sensor packet ID]
Available: if the OI is connected.
Mode change: Doesn't change mode.
"""
OPCODE_SEND_SENSORS: bytes = (149).to_bytes(1, "big")
"""
This command lets you request multiple sensor packets (check [the sensor packet docs](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=22)).

Format: [149, sensor packet count, sensor packet ID 1, sensor packet ID 2, etc.]
Available: if the OI is connected.
Mode change: Doesn't change mode.
"""
OPCODE_STREAM_SENSORS: bytes = (148).to_bytes(1, "big")
"""
This command lets you request sensor packets every 15ms (check [the sensor packet docs](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=22)).
Check the [format of the returned data](https://www.irobot.com/~/media/mainsite/pdfs/about/stem/create/create_2_open_interface_spec.pdf#page=21).

Format: [148, sensor packet count, sensor packet ID 1, sensor packet ID 2, etc.]
Available: if the OI is connected.
Mode change: Doesn't change mode.
"""
OPCODE_CHANGE_STREAM_STATUS: bytes = (150).to_bytes(1, "big")
"""
This command lets you toggle the stream of sensor packets.

Format: [150, operation (0-off, 1-on)]
Available: if the OI is connected.
Mode change: Doesn't change mode.
"""
