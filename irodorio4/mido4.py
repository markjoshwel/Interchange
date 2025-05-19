import queue
import threading
from math import ceil
from sys import stderr
from time import sleep
from typing import NamedTuple, Sequence

import hid
import rtmidi
import serial
from rich.console import Console

MIDO4_MIDI_NAME = "mido4 Virtual Instrument Output"
MIDO4_MIDI_CHANNEL = 1
MIDO4_CONTROLLER_PRESSURE_MIN = 15
MIDO4_CONTROLLER_PRESSURE_MAX = 200
MIDO4_OCTAVE_STARTING = 1  # as in, octave one

CHU3_IO4_VID = 0x0CA3  # sega io4 vendor number/id
CHU3_IO4_PID = 0x0021  # sega io4 product number/id
CHU3_IO4_REPORT_ID = 16  # host -> io4 commands
CHU3_IO4_AIR_TOWER_IDX_LEFT = 30
CHU3_IO4_AIR_TOWER_IDX_RIGHT = 32
CHU3_IO4_CMD_SET_COMMUNICATION_TIMEOUT = 0x01
CHU3_IO4_CMD_SET_SAMPLING_COUNT = 0x02
CHU3_IO4_CMD_CLEAR_BOARD_STATUS = 0x03
CHU3_IO4_SAMPLING_COUNT = 10
CHU3_IO4_COMM_TIMEOUT_US = 10  # 2000us = 2ms
CHU3_IO4_READ_TIMEOUT_MS = 1
CHU3_RW_PACKET_SIZE_BYTES = 64
CHU3_SLIDER_SYNC_HEADER = b"\xff\x01"
CHU4_SLIDER_INPUT_CHUNK_BY_DIVIDER = 1
CHU3_SLIDER_CMD_LED = 0x02

MIDI_OCTAVE_ZERO = 20
MIDI_OCTAVE_TET = 12
MIDI_BASE_A = 1
MIDI_BASE_Bb = 2
MIDI_BASE_B = 3
MIDI_BASE_C = 4
MIDI_BASE_Cs = 5
MIDI_BASE_D = 6
MIDI_BASE_Eb = 7
MIDI_BASE_E = 8
MIDI_BASE_F = 9
MIDI_BASE_Fs = 10
MIDI_BASE_G = 11
MIDI_BASE_Gs = 12


Air6Type = tuple[bool, bool, bool, bool, bool, bool] | Sequence[bool]
Ground32Type = tuple[int, ...] | Sequence[int]


class Chu3SliderInputThread:
    # sync byte in io4 is FF, at the start of the 64 bytes

    def __init__(self, serial_device):
        self.serial_device: serial.Serial = serial_device
        self.buffer: bytearray = bytearray()
        self.inputs: queue.Queue[Ground32Type] = queue.Queue()
        self.running: bool = False
        self.thread: threading.Thread = threading.Thread(
            target=self._reader, daemon=True
        )

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def _reader(self):
        while self.running:
            num_waiting = self.serial_device.in_waiting
            if num_waiting:
                data = self.serial_device.read(num_waiting)
                self.buffer.extend(data)
            else:
                continue

            while len(self.buffer) >= CHU3_RW_PACKET_SIZE_BYTES:
                # find the index of the sync header byte
                header_index = self.buffer.find(CHU3_SLIDER_SYNC_HEADER[0])
                if header_index > 0:
                    # discard any bytes before the sync header
                    del self.buffer[:header_index]
                if len(self.buffer) < CHU3_RW_PACKET_SIZE_BYTES:
                    break

                # extract a complete packet and remove it from the buffer
                packet = bytes(self.buffer[:CHU3_RW_PACKET_SIZE_BYTES])
                del self.buffer[:CHU3_RW_PACKET_SIZE_BYTES]
                self.inputs.put(Chu3Controller.parse_ground_sensors(packet))


class Chu3AirInputThread:
    def __init__(self, hid_device):
        self.hid_device = hid_device  # type: ignore
        self.inputs: queue.Queue[Air6Type] = queue.Queue()
        self.running: bool = False
        self.thread: threading.Thread = threading.Thread(
            target=self._reader, daemon=True
        )

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def _reader(self):
        while self.running:
            data_io4 = self.hid_device.read(
                max_length=CHU3_RW_PACKET_SIZE_BYTES,
                timeout_ms=CHU3_IO4_READ_TIMEOUT_MS,
            )  # type: ignore

            if (data_io4 is not None) and (len(data_io4) == CHU3_RW_PACKET_SIZE_BYTES):
                self.inputs.put(
                    Chu3Controller.parse_air_sensors(
                        data_io4[CHU3_IO4_AIR_TOWER_IDX_LEFT],
                        data_io4[CHU3_IO4_AIR_TOWER_IDX_RIGHT],
                    )
                )


RGBType = tuple[int, int, int] | Sequence[int]


class Chu3LEDColours(NamedTuple):
    """
    ground31_brightness: int
        brightness for the 31 ground LEDs
    ground31: Sequence[RGBType]
        31 LED colour values (16 ground, 15 dividers, 1 ignored)
    air_brightness: int
        brightness for both air towers
    left_air24: Sequence[RGBType]
        6 * 4 LED colour values for the left AIR tower
    right_air24: Sequence[RGBType]
        6 * 4 LED colour values for the right AIR tower

    all values are 0-255
    """

    ground31_brightness: int
    ground31: Sequence[RGBType]
    air_brightness: int
    air24_left: Sequence[RGBType]
    air24_right: Sequence[RGBType]


class Chu3SensorReadout(NamedTuple):
    """
    ground32: Ground32Type
        32 ground sensor values, 0-255 for sense/touch pressure/capacitance
        -1 if it wasn't read within the frame of reading
    air6: Air6Type
        6 air sensor values, True for detection/sensor blockage, False for no detection
    """

    ground32: Ground32Type
    air6: Air6Type


class Chu3Controller:
    """
    really shoddy implementation of a chunithm io4 controller

    out of a 1000hz tasoller, i can only get roughly 33hz out of the ground sensors
    and 75hz out of the air sensors. rip.
    """

    _init: bool = False

    air6: Air6Type = [False] * 6
    air_spawned: bool = False
    air_thread: Chu3AirInputThread
    air_readouts: int = 0

    ground32: Ground32Type = [0] * 32
    ground_spawned: bool = False
    ground_thread: Chu3SliderInputThread
    ground_readouts: int = 0

    def init(self, spawn_air: bool = True, spawn_slider: bool = True) -> None:
        if self._init:
            return

        # print("chu3controller.init: clear board status", file=stderr)
        # self.clear_board_status()

        # print("chu3controller.init: set communication timeout", file=stderr)
        # self.set_communication_timeout()

        # print("chu3controller.init: set sampling count", file=stderr)
        # self.set_sampling_count()

        if spawn_air:
            self.spawn_air_thread()

        if spawn_slider:
            self.spawn_slider_thread()

            print("chu3controller.init: enabling slider report", file=stderr)
            self.slider_enable_report()

        self._init = True

    def spawn_air_thread(self):
        print("chu3controller.init: spawning air input thread", file=stderr)
        self.air_thread = Chu3AirInputThread(self.device_hid)
        self.air_thread.start()
        self.air_spawned = True

    def spawn_slider_thread(self):
        print("chu3controller.init: spawning slider input thread", file=stderr)
        self.ground_thread = Chu3SliderInputThread(self.device_com)
        self.ground_thread.start()
        self.ground_spawned = True

    def set_communication_timeout(
        self, timeout_us: int = CHU3_IO4_COMM_TIMEOUT_US
    ) -> None:
        # struct {
        #     uint8_t bReportId = 16;
        #     uint8_t bCmd = 1;
        #     uint8_t bTimeout;
        #     uint8_t Rsv[61];
        # }
        # https://gitea.tendokyu.moe/tasoller/host-aprom/src/branch/master/docs/IO4.md#set-communication-timeout-01
        # work on the assumption that its multiples of 200us
        packet = bytearray(64)
        packet[0] = CHU3_IO4_REPORT_ID
        packet[1] = CHU3_IO4_CMD_SET_COMMUNICATION_TIMEOUT
        packet[2] = timeout_us  # 2000us = 2ms
        self.device_hid.write(packet)

    def set_sampling_count(self, count: int = CHU3_IO4_SAMPLING_COUNT) -> None:
        # struct {
        #     uint8_t bReportId = 16;
        #     uint8_t bCmd = 2;
        #     uint8_t bSamplingCount;
        #     uint8_t Rsv[61];
        # }
        packet = bytearray(64)
        packet[0] = CHU3_IO4_REPORT_ID
        packet[1] = CHU3_IO4_CMD_SET_SAMPLING_COUNT
        packet[2] = count
        self.device_hid.write(packet)

    def clear_board_status(self) -> None:
        # struct {
        #     uint8_t bReportId = 16;
        #     uint8_t bCmd = 3;
        #     uint8_t Rsv[62];
        # }
        # unsets timeout and sampling count
        packet = bytearray(64)
        packet[0] = CHU3_IO4_REPORT_ID
        packet[1] = CHU3_IO4_CMD_CLEAR_BOARD_STATUS
        self.device_hid.write(packet)

    def slider_enable_report(self) -> None:
        """
        Enable automatic slider report transmission on the CDC (serial) interface.

        This constructs a slider command packet. The protocol expects:
        - Byte 0: SLIDER_SYNC (0xFF)
        - Byte 1: Command (SLIDER_CMD_Rx_REPORT_ENABLE, 0x03)
        - Byte 2: Length (0, since there is no payload)
        - Byte 3: Checksum (- (SLIDER_SYNC + Command + Length)) & 0xFF
        """
        SLIDER_SYNC = 0xFF
        SLIDER_CMD_Rx_REPORT_ENABLE = 0x03
        length = 0
        total = (
            SLIDER_SYNC + SLIDER_CMD_Rx_REPORT_ENABLE + length
        )  # should be 0xFF + 0x03 = 0x102
        checksum = (-total) & 0xFF  # Correct checksum: (-0x102) mod 256 = 0xFE
        packet = bytearray([SLIDER_SYNC, SLIDER_CMD_Rx_REPORT_ENABLE, length, checksum])
        self.device_com.write(packet)
        # print("Sent slider report enable command, packet:", packet, file=stderr)

    def set_ground_led_colours(self, colours: Chu3LEDColours) -> None:
        """
        Set the LED colors for the ground slider.

        The protocol expects:
        - Byte 0: SLIDER_SYNC (0xFF)
        - Byte 1: Command (SLIDER_CMD_Rx_LED, 0x02)
        - Byte 2: Length (97 for LED data)
        - Byte 3: Brightness
        - Bytes 4-99: RGB values (32 sets of 3 bytes)
        - Byte 100: Checksum
        """
        SLIDER_SYNC = 0xFF
        length = 97  # 1 byte brightness + (32 * 3) bytes RGB data

        # Create the packet
        packet = bytearray([SLIDER_SYNC, CHU3_SLIDER_CMD_LED, length])

        # Add ground brightness
        packet.append(colours.ground31_brightness & 0xFF)

        # Add ground RGB values
        first_packet: RGBType | None = None
        for rgb in colours.ground31[::-1]:
            if first_packet is None:
                first_packet = [
                    rgb[2] & 0xFF,  # Blue
                    rgb[1] & 0xFF,  # Green
                    rgb[0] & 0xFF,  # Red
                ]
            packet.extend(
                [
                    rgb[2] & 0xFF,  # Blue
                    rgb[1] & 0xFF,  # Green
                    rgb[0] & 0xFF,  # Red
                ]
            )
        if first_packet:
            packet.extend(first_packet)

        # Calculate checksum (sum of all bytes negated)
        checksum = (-sum(packet)) & 0xFF
        packet.append(checksum)

        # Send the packet
        self.device_com.write(packet)
        self.device_com.flush()

    def open(
        self,
        nonblocking: bool = True,
        init=True,
        ignore_io4_errors: bool = False,
    ) -> None:
        last_exception: Exception | None = None

        try:
            self.device_com = serial.Serial(port="COM1", baudrate=38400, timeout=0.1)
            print(
                f"chu3controller.open: com: opened {self.device_com.name} on COM1 ({self.device_com.baudrate} baud)",
                file=stderr,
            )
            connected = True
            connected_com = True

        except Exception as e:
            print(
                f"chu3controller.open: com: {e.__class__.__name__}: {e}",
                file=stderr,
            )
            last_exception = e

        try:
            self.device_hid = hid.device()
            self.device_hid.open(CHU3_IO4_VID, CHU3_IO4_PID)
            connected = True
            connected_hid = True

        except Exception as e:
            print(
                f"chu3controller.open: io4/hid: {e.__class__.__name__}: {e}",
                file=stderr,
            )
            last_exception = e

        if "TASOLLER" in (product_string := self.device_hid.get_product_string()) and (
            not ignore_io4_errors
        ):
            self.device_hid.close()
            connected_hid = False
            connected = False

            message = (
                f"your controller should be emulating arcade io! not '{product_string}'"
            )
            print(
                f"chu3controller.open: io4/hid: {message}",
                file=stderr,
            )
            print(
                "... a note from the bloodied hands before you:",
                "... ... reboot. a few times maybe.",
                "... ...",
                "... ... and if that's still not working out for ya:",
                "... ... have you tried removing all ghost HID devices in device manager?",
                "... ... sometimes, windows hates you, and the tasoller may be stuck as",
                "... ... 'TASOLLER HID' even if it is not in keyboard mode.",
                "... ...",
                "... ... try the copy-pasting the following powershell script in an admin",
                "... ... powershell window:",
                "... ...",
                "... ...   # List all ghost HIDClass devices (devices in the HIDClass with an unknown status)",
                "... ...   $ghost_hid_devs = Get-PnpDevice -Class HIDClass | Where-Object { $_.Status -eq 'Unknown' }",
                "... ...   # Loop through each ghost HID device and remove it using pnputil"
                "",
                "... ...   foreach ($dev in $ghost_hid_devs) { pnputil /remove-device $dev.InstanceId }",
                "... ...",
                "... ... finally, do one last reboot.",
                "... ... cheers~",
                sep="\n",
            )
            last_exception = ValueError(message)
            raise last_exception

        if last_exception is not None:
            print(
                f"chu3controller.open: {last_exception.__class__.__name__}: {last_exception}",
                file=stderr,
            )
            exit(-1)

        print(
            f"chu3controller.open: io4/hid: opened {self.device_hid.get_manufacturer_string()} - {self.device_hid.get_product_string()} ({self.device_hid.get_serial_number_string()})",
            file=stderr,
        )

        self.device_hid.set_nonblocking(int(nonblocking))

        if init:
            self.init()

    def read(self) -> Chu3SensorReadout:
        # read air from hid/io4
        try:
            self.air6: Air6Type = self.air_thread.inputs.get(
                block=False,
                timeout=None,
            )
            self.air_readouts += 1
        except queue.Empty:
            pass

        # read ground from serial/com
        try:
            self.ground32: Ground32Type = self.ground_thread.inputs.get(
                block=False,
                timeout=None,
            )
            self.ground_readouts += 1
        except queue.Empty:
            pass

        return Chu3SensorReadout(self.ground32, self.air6)

    @staticmethod
    def parse_air_sensors(l: int, r: int) -> tuple[bool, bool, bool, bool, bool, bool]:
        """
        Given two integer values l and r read from the HID device,
        parse and return a tuple of six booleans representing the blocked state
        of the 6 air sensors (1 through 6). A value of True means the sensor is blocked.

        The HID values are 8-bit numbers that are multiples of 8. The meaningful
        data is contained in the upper 3 bits (i.e. value >> 3 gives a number from 0-7).

        Sensor mapping:
          - Left value (l >> 3) encodes sensors 1, 3, 5:
              • Sensor 1 corresponds to bit 2 (0b100).
              • Sensor 3 corresponds to bit 1 (0b010).
              • Sensor 5 corresponds to bit 0 (0b001).
          - Right value (r >> 3) encodes sensors 2, 4, 6 similarly.

        In the unblocked state, each side should be 0b111 (7). If a sensor is blocked,
        its corresponding bit is cleared (0). For example, if sensor 1 is blocked, then
        the left value becomes 0b011 (3), which when multiplied by 8 gives 24.
        """

        """
        air     l	r
        
        -       56	56
        1       24	56
        12      24	24
        123     8	24
        1234    8	8
        12345   0	8
        123456  0	0
        
        12	24	24
        23	40	24
        34	40	40
        45	48	40
        56	48	48
        
        1	24	56
        2	56	24
        3	40	56
        4	56	40
        5	48	56
        6	56	48
        """

        # Extract the 3-bit masks from l and r.
        l_mask = l >> 3  # Should be a value between 0 and 7.
        r_mask = r >> 3  # Should be a value between 0 and 7.

        # For left side sensors: sensor 1, sensor 3, sensor 5.
        sensor1_blocked = (l_mask & 0b100) == 0  # If bit2 is 0, sensor 1 is blocked.
        sensor3_blocked = (l_mask & 0b010) == 0  # If bit1 is 0, sensor 3 is blocked.
        sensor5_blocked = (l_mask & 0b001) == 0  # If bit0 is 0, sensor 5 is blocked.

        # For right side sensors: sensor 2, sensor 4, sensor 6.
        sensor2_blocked = (r_mask & 0b100) == 0  # If bit2 is 0, sensor 2 is blocked.
        sensor4_blocked = (r_mask & 0b010) == 0  # If bit1 is 0, sensor 4 is blocked.
        sensor6_blocked = (r_mask & 0b001) == 0  # If bit0 is 0, sensor 6 is blocked.

        # Return sensors in order: 1, 2, 3, 4, 5, 6.
        return (
            sensor1_blocked,
            sensor2_blocked,
            sensor3_blocked,
            sensor4_blocked,
            sensor5_blocked,
            sensor6_blocked,
        )

    @staticmethod
    def parse_ground_sensors(data: bytes) -> Ground32Type:
        padding = 3
        ground32 = [0] * 32

        pr_dl = data[: 16 + padding]
        # if pr_dl[0:2] == CHU3_SLIDER_SYNC_HEADER:
        ground32[16:] = pr_dl[padding:][::-1]

        pl_dr = data[16 + padding : 16 + 16 + padding]
        # if pl_dr[0:2] == CHU3_SLIDER_SYNC_HEADER:
        ground32[:16] = pl_dr[::-1]

        return ground32

    def close(self):
        print("chu3controller.close: stopping collection and clearing", file=stderr)
        if self.air_spawned:
            self.air_thread.stop()
        if self.ground_spawned:
            self.ground_thread.stop()
        self.clear_board_status()

        print("chu3controller.close: closing device", file=stderr)
        self.device_hid.close()
        self.device_com.close()

        print("chu3controller.close: closed", file=stderr)


def mido4_map_fingering_to_midi(
    valveh: bool,
    valve1: bool,
    valve2: bool,
    valve3: bool,
    valve4: bool,
    octave_variant: int,
) -> int:
    """returns -1 if the fingering is not recognised"""

    base: int = 0
    pattern = (valveh, valve1, valve2, valve3)

    if pattern == (False, False, False, False):
        # Bb -> ____ (no valves pressed)
        base = MIDI_BASE_Bb

    elif pattern == (False, True, True, True):
        # B -> 123_
        base = MIDI_BASE_B

    elif pattern == (False, True, False, True):
        # C -> 1_3_
        base = MIDI_BASE_C

    elif pattern == (False, False, True, True):
        # C# -> _23_
        base = MIDI_BASE_Cs

    elif pattern == (False, True, True, False):
        # D -> 12__
        base = MIDI_BASE_D

    elif pattern == (False, True, False, False):
        # Eb -> 1___
        base = MIDI_BASE_Eb

    elif pattern == (False, False, True, False):
        # E -> _2__
        base = MIDI_BASE_E

    elif pattern == (True, False, False, False):
        # F -> H____
        base = MIDI_BASE_F

    elif pattern == (True, False, True, True):
        # F# -> H_23_
        base = MIDI_BASE_Fs

    elif pattern == (True, True, True, False):
        # G -> H12__
        base = MIDI_BASE_G

    elif pattern == (True, True, False, False):
        # G# -> H1___
        base = MIDI_BASE_Gs

    elif pattern == (True, False, True, False):
        # A -> H_2__
        base = MIDI_BASE_A + 12

    else:
        return -1

    # start at octave 1 = A is octave (20) + note (1) = 21
    # start at octave 0 = A is octave (8) + note (1) = 9
    # start at octave -2 = C is octave (-4) + note (4) = 0
    octave = (
        MIDO4_OCTAVE_STARTING
        + octave_variant
        + 2  # because the midi starts at C-2 (two octaves below the 'zeroth' octave)
    ) * 12 - 4  # 12 notes per octave, -4 because midi starts at C-2 (note #0)

    return base + octave


def mido4_process_readout(
    ground32: Ground32Type,
    air6: Air6Type,
) -> tuple[int, int]:
    """
    LAYOUT (GROUND32)
    ... THIS WAY UP TO A CHUNITHM SCREEN (TASOLLER LOGO FOR TASOLLERS)
    ... LEFT <--------> RIGHT
    ... ... ground32[1] ground32[3] ground32[5] <---> ground32[31]
    ... ... ground32[0] ground32[2] ground32[4] <---> ground32[30]
    ... THIS WAY DOWN TO A HUMAN

    LAYOUT (AIR6)
    ... THIS WAY UP TO THE SKY
    ... ... air6[5]
    ... ... air6[4]
    ... ... air6[3]
    ... ... air6[2]
    ... ... air6[1]
    ... ... air6[0]
    ... THIS WAY DOWN TO THE GROUND

    VALVE MAPPINGS
    ... ---- "INWARD"    "OUTWARD"
    ... H -> ground32[9] ground32[8]
    ... 1 -> ground32[7] ground32[6]
    ... 2 -> ground32[5] ground32[4]
    ... 3 -> ground32[3] ground32[2]
    ... 4 -> ground32[1] ground32[0]
    """

    ratio = 127 / MIDO4_CONTROLLER_PRESSURE_MAX

    valveh_velocity: int = ceil(
        min(
            ground32[9] + ground32[8],
            MIDO4_CONTROLLER_PRESSURE_MAX,
        )
        * ratio
    )
    valveh_on_inward: bool = ground32[9] > MIDO4_CONTROLLER_PRESSURE_MIN

    valve1_velocity: int = ceil(
        min(
            ground32[7] + ground32[6],
            MIDO4_CONTROLLER_PRESSURE_MAX,
        )
        * ratio
    )
    valve1_on_inward: bool = ground32[7] > MIDO4_CONTROLLER_PRESSURE_MIN

    valve2_velocity: int = ceil(
        min(
            ground32[5] + ground32[4],
            MIDO4_CONTROLLER_PRESSURE_MAX,
        )
        * ratio
    )
    valve2_on_inward: bool = ground32[5] > MIDO4_CONTROLLER_PRESSURE_MIN

    valve3_velocity: int = ceil(
        min(
            ground32[3] + ground32[2],
            MIDO4_CONTROLLER_PRESSURE_MAX,
        )
        * ratio
    )
    valve3_on_inward: bool = ground32[3] > MIDO4_CONTROLLER_PRESSURE_MIN

    valve4_velocity: int = ceil(
        min(
            ground32[1] + ground32[0],
            MIDO4_CONTROLLER_PRESSURE_MAX,
        )
        * ratio
    )
    valve4_on_inward: bool = ground32[1] > MIDO4_CONTROLLER_PRESSURE_MIN
    velocity: int = max(
        valve1_velocity, valve2_velocity, valve3_velocity, valve4_velocity
    )

    octave_variant: int = 0
    for i, value in enumerate(air6):
        if value:
            octave_variant = i + 1

    return (
        mido4_map_fingering_to_midi(
            valveh_velocity >= MIDO4_CONTROLLER_PRESSURE_MIN,
            valve1_velocity >= MIDO4_CONTROLLER_PRESSURE_MIN,
            valve2_velocity >= MIDO4_CONTROLLER_PRESSURE_MIN,
            valve3_velocity >= MIDO4_CONTROLLER_PRESSURE_MIN,
            valve4_velocity >= MIDO4_CONTROLLER_PRESSURE_MIN,
            octave_variant,
        ),
        velocity
        if (
            valve1_on_inward
            or valve2_on_inward
            or valve3_on_inward
            or valve4_velocity
            or valveh_on_inward
        )
        else 0,
    )


import math


def _rainbow(position: float) -> tuple[int, int, int]:
    h: float = 240.0 * position
    s: float = 1.0
    v: float = 1.0

    h_sector: float = h / 60.0
    i: int = int(h_sector)
    f: float = h_sector - i
    p: float = v * (1 - s)  # p is always 0 when s == 1
    q: float = v * (1 - f * s)
    t: float = v * (1 - (1 - f) * s)

    if i == 0:
        r, g, b = v, t, p
    elif i == 1:
        r, g, b = q, v, p
    elif i == 2:
        r, g, b = p, v, t
    elif i == 3:
        r, g, b = p, q, v
    elif i == 4:
        r, g, b = t, p, v
    else:  # i == 5 (should not occur with h in [0,240])
        r, g, b = v, p, q

    return (int(r * 255), int(g * 255), int(b * 255))


def mido4_blinkenlights(
    note: tuple[int, int], ground32: Ground32Type, air6: Air6Type
) -> Chu3LEDColours:
    """
    GROUND31 FORMAT: 16 GROUND, 15 DIVIDERS
    ... LEFT <--------> RIGHT
    ... KEYS ->  ground31[0], ground31[2], ground31[4], ..., ground31[30]
    ... DIVIDERS -> ground31[1], ground31[3], ground31[5], ..., ground31[29]

    AIR24 FORMAT: 6 * 4 LEDS
    ... UNKNOWN LOL

    VALVE MAPPINGS
    ... ---- "INWARD"    "OUTWARD"
    ... H -> ground32[9] ground32[8]
    ... 1 -> ground32[7] ground32[6]
    ... 2 -> ground32[5] ground32[4]
    ... 3 -> ground32[3] ground32[2]
    ... 4 -> ground32[1] ground32[0]
    """

    air24 = [(0, 0, 0)] * 24
    for i, value in enumerate(air6):
        if value:
            air24[i * 4] = (101, 78, 163)
            air24[i * 4 + 1] = (84, 60, 143)
            air24[i * 4 + 2] = (68, 42, 124)
            air24[i * 4 + 3] = (52, 23, 105)

    ground31 = [(0, 0, 0)] * 31

    # valve dividers
    ground31[1] = (255, 255, 255)
    ground31[3] = (255, 255, 255)
    ground31[5] = (255, 255, 255)
    ground31[7] = (255, 255, 255)

    # valve colours
    note_lowest = (MIDO4_OCTAVE_STARTING + 2) * 12 - 4 + 1
    note_highest = (MIDO4_OCTAVE_STARTING + 8 + 2) * 12 - 4
    note_colour: tuple[int, int, int] = _rainbow(note[0] / (note_highest - note_lowest))

    # if note[0] != -1:
    #     valveh = note_colour if (ground32[9] + ground32[8]) > 0 else (128, 128, 128)
    #     valve1 = note_colour if (ground32[7] + ground32[6]) > 0 else (128, 128, 128)
    #     valve2 = note_colour if (ground32[5] + ground32[4]) > 0 else (128, 128, 128)
    #     valve3 = note_colour if (ground32[3] + ground32[2]) > 0 else (128, 128, 128)
    ground31[0] = note_colour if (ground32[1] or ground32[0]) else (128, 128, 128)
    ground31[2] = note_colour if (ground32[3] or ground32[2]) else (128, 128, 128)
    ground31[4] = note_colour if (ground32[5] or ground32[4]) else (128, 128, 128)
    ground31[6] = note_colour if (ground32[7] or ground32[6]) else (128, 128, 128)
    ground31[8] = note_colour if (ground32[9] or ground32[8]) else (128, 128, 128)

    # slider representation, top 6*2
    # ... air 6[5] -> ground31[31] + ground31[30]
    # ... ...
    # ... air 6[0] -> ground31[21] + ground31[20]
    if air6[5]:
        ground31[30] = (255, 255, 255)
        ground31[29] = (255, 255, 255)

    if air6[4]:
        ground31[28] = (255, 255, 255)
        ground31[27] = (255, 255, 255)

    if air6[3]:
        ground31[26] = (255, 255, 255)
        ground31[25] = (255, 255, 255)

    if air6[2]:
        ground31[24] = (255, 255, 255)
        ground31[23] = (255, 255, 255)

    if air6[1]:
        ground31[22] = (255, 255, 255)
        ground31[21] = (255, 255, 255)

    if air6[0]:
        ground31[20] = (255, 255, 255)
        ground31[19] = (255, 255, 255)

    return Chu3LEDColours(
        ground31_brightness=255,
        ground31=ground31,
        air_brightness=255,
        air24_left=air24,
        air24_right=air24,
    )


def mido4():
    print("mido4: opening midi port", file=stderr)
    midiout = rtmidi.MidiOut()  # type: ignore
    available_ports = midiout.get_ports()
    if available_ports:
        midiout.open_port(0)
        print(f"mido4: opened on port 0", file=stderr)
    else:
        midiout.open_virtual_port(MIDO4_MIDI_NAME)
        print(f"mido4: opened port under '{MIDO4_MIDI_NAME}'", file=stderr)
    
    controller = Chu3Controller()
    controller.open()

    last_note: tuple[int, int] = (-1, -1)
    try:
        with midiout:
            while True:
                readout = controller.read()
                note = mido4_process_readout(
                    readout.ground32,
                    readout.air6,
                )
                
                if (
                    (note[0] != -1)
                    # and (note[0] != last_note[0])
                ):
                    last_note = note
                    midiout.send_message([0x90, *note])
                    print(note)
                
                controller.set_ground_led_colours(
                    mido4_blinkenlights(
                        note,
                        readout.ground32,
                        readout.air6,
                    )
                )

    except Exception:
        from traceback import print_exc

        print_exc()

    except KeyboardInterrupt:
        pass

    print("mido4: cleaning up...")
    del midiout
    controller.close()


if __name__ == "__main__":
    mido4()
    # mido4()
