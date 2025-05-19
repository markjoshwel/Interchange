import hid
import serial
from typing import NamedTuple, Sequence
from sys import stderr
from rich.console import Console
from time import sleep
import threading
import queue


CHU3_IO4_VID = 0x0CA3  # sega io4 vendor number/id
CHU3_IO4_PID = 0x0021  # sega io4 product number/id

CHU3_IO4_REPORT_ID = 16  # host -> io4 commands
CHU3_IO4_AIR_TOWER_IDX_LEFT = 30
CHU3_IO4_AIR_TOWER_IDX_RIGHT = 32
CHU3_IO4_CMD_SET_COMMUNICATION_TIMEOUT = 0x01
CHU3_IO4_CMD_SET_SAMPLING_COUNT = 0x02
CHU3_IO4_CMD_CLEAR_BOARD_STATUS = 0x03
CHU3_IO4_CMD_SET_PWM_OUTPUT = 0x04
CHU3_IO4_CMD_SET_UNIQUE_OUTPUT = 0x41
CHU3_IO4_SAMPLING_COUNT = 15
CHU3_IO4_COMM_TIMEOUT_US = 10  # 2000us = 2ms
CHU3_IO4_READ_TIMEOUT_MS = 1

# byte  0     : report id (ignored)
# bytes 1–32  : 32 bytes representing ground (slider/touch) sensor values
# bytes 33–38 : 6 bytes representing air (tower) sensor values
# byte  39    : A sensitivity/hardness value for the inputs
# bytes 40–63 : ignored
CHU3_RW_PACKET_SIZE_BYTES = 64
CHU3_SLIDER_SYNC_HEADER = b"\xff\x01"
CHU4_SLIDER_INPUT_CHUNK_BY_DIVIDER = 1
CHU3_SLIDER_CMD_LED = 0x02
CHU3_LED_DATA_SIZE = 32  # 16 ground leds + 15 divider, the last one is ignored


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
        Set the LED colors for both ground and air sensors.

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
            else:
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

    def set_air_led_colours(self, colours: Chu3LEDColours) -> None:
        """
        Set the LED colors for air sensors using custom protocol command 0x5B.
        
        struct {
            uint8_t u8Cmd = 0x5B;
            uint8_t u8GroundBrightness;
            uint8_t u8TowerBrightness;
            grb_t aGround[31];
            grb_t aTowerLeft[24];
            grb_t aTowerRight[24];
        }

        The protocol expects:
        - Byte 0: CMD_CUSTOM_RGB (0x5B)
        - Byte 1: Ground brightness (0-255)
        - Byte 2: Air brightness (0-255)
        - Bytes 3-95: Ground LED RGB values (31 sets of 3 bytes)
        - Bytes 96-167: Left air tower RGB values (24 sets of 3 bytes)
        - Bytes 168-239: Right air tower RGB values (24 sets of 3 bytes)
        """
        CMD_CUSTOM_RGB = 0x5B

        # Create the packet
        packet = bytearray([CMD_CUSTOM_RGB])

        # Add brightness values
        packet.append(0)  # Ground brightness (not used for air)
        packet.append(colours.air_brightness & 0xFF)

        # Add empty ground data (31 LEDs * 3 bytes)
        packet.extend([0] * (31 * 3))

        # Add tower data (24 LEDs per tower)
        for rgb in colours.air24_left:
            packet.extend(
                [
                    rgb[1] & 0xFF,  # Green
                    rgb[0] & 0xFF,  # Red
                    rgb[2] & 0xFF,  # Blue
                ]
            )
        
        for rgb in colours.air24_right:
            packet.extend(
                [
                    rgb[1] & 0xFF,  # Green
                    rgb[0] & 0xFF,  # Red
                    rgb[2] & 0xFF,  # Blue
                ]
            )

        self.device_hid.write(packet)

    def set_led_colours(self, colours: Chu3LEDColours) -> None:
        """Set both ground and air LED colors."""
        self.set_ground_led_colours(colours)
        self.set_air_led_colours(colours)

    def open(
        self,
        nonblocking: bool = True,
        init=True,
        keep_trying: bool = False,
        ignore_io4_errors: bool = False,
    ) -> None:
        connected: bool = False
        connected_com: bool = False
        connected_hid: bool = False
        last_exception: Exception = Exception()

        try:
            while not connected:
                try:
                    if not connected_com:
                        self.device_com = serial.Serial(
                            port="COM1", baudrate=38400, timeout=0.1
                        )
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
                    if not connected_hid:
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

                if "TASOLLER" in (
                    product_string := self.device_hid.get_product_string()
                ) and (not ignore_io4_errors):
                    self.device_hid.close()
                    connected_hid = False
                    connected = False

                    message = f"your controller should be emulating arcade io! not '{product_string}'"
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

                if connected:
                    break

                if keep_trying:
                    print(
                        "chu3controller.open: io4/hid: failed to open, retrying in 5s",
                        file=stderr,
                    )
                    sleep(5)

                else:
                    if connected_com:
                        self.device_com.close()
                    if connected_hid:
                        self.device_hid.close()
                    raise last_exception

        except KeyboardInterrupt:
            if connected_com:
                self.device_com.close()
            if connected_hid:
                self.device_hid.close()
            print(
                "chu3controller.open: premature KeyboardInterrupt, closing!",
                file=stderr,
            )
            raise last_exception

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


def sensor_to_truecolour_block(
    value: int,
    sensor_range: int = 256,
    start_color: tuple[int, int, int] = (255, 255, 255),
    end_color: tuple[int, int, int] = (255, 0, 0),
) -> str:
    # clamp and normalise to 0-1 scale
    clamped_value = max(0, min(value, sensor_range - 1))
    normalized = clamped_value / (sensor_range - 1) if sensor_range > 1 else 0

    r = int(start_color[0] + (end_color[0] - start_color[0]) * normalized)
    g = int(start_color[1] + (end_color[1] - start_color[1]) * normalized)
    b = int(start_color[2] + (end_color[2] - start_color[2]) * normalized)

    return f"[rgb({r},{g},{b})]█[/rgb({r},{g},{b})]"


def slider_to_truecolor_block(sliders: Air6Type) -> str:
    return " ".join("[red]█[/red]" if v else "[white]█[/white]" for v in sliders)


def test():
    from time import time_ns

    controller = Chu3Controller()
    controller.open()

    console = Console()
    start = time_ns()
    readouts = 1
    try:
        last_readout = None
        while True:
            # console.clear()
            readout = controller.read()
            if readout != last_readout:
                readouts += 1
                last_readout = readout

            # ground 1-16
            out = ""

            for i in range(1, 32, 2):
                out += sensor_to_truecolour_block(readout.ground32[i])

            out += "\n"
            for i in range(0, 32, 2):
                out += sensor_to_truecolour_block(readout.ground32[i])

            # air 1-6
            out += "\n"
            out += slider_to_truecolor_block(readout.air6)
            out += "\n"

            console.print(out)
            # break

    except KeyboardInterrupt:
        pass

    end = time_ns()
    print(f"hz: {readouts / ((end - start) / 1e9)}")
    controller.close()


def readout_test():
    from time import time_ns

    controller = Chu3Controller()
    controller.open()

    start = time_ns()
    end = start + (1e9 * 5)
    while time_ns() < end:
        controller.read()

    controller.close()

    print(
        "mido4 v1 5s readout report",
        f"... air6 ro/5s     -> {controller.air_readouts}",
        f"... air6 ro/s      -> {controller.air_readouts / 5}",
        f"... air6 hz        -> {(air_hz := controller.air_readouts / ((end - start) / 1e9)):.3f}hz",
        f"... air6 ms        -> {1000 * (1 / air_hz):.3f}ms",
        f"... ground32 ro/5s -> {controller.ground_readouts}",
        f"... ground32 ro/s  -> {controller.ground_readouts / 5}",
        f"... ground32 hz    -> {(ground_hz := controller.ground_readouts / ((end - start) / 1e9)):.3f}hz",
        f"... ground32 ms    -> {1000 * (1 / ground_hz):.3f}ms",
        sep="\n",
    )


def color_test():
    controller = Chu3Controller()
    controller.open()

    rainbow_ground32 = []
    for i in range(32):
        position = (i * 6) // 32  # Split into 6 segments
        if position == 0:  # Red to Yellow
            r, g, b = 255, (i * 255) // 5, 0
        elif position == 1:  # Yellow to Green
            r, g, b = 255 - ((i - 5) * 255) // 5, 255, 0
        elif position == 2:  # Green to Cyan
            r, g, b = 0, 255, ((i - 10) * 255) // 5
        elif position == 3:  # Cyan to Blue
            r, g, b = 0, 255 - ((i - 15) * 255) // 5, 255
        elif position == 4:  # Blue to Magenta
            r, g, b = ((i - 20) * 255) // 5, 0, 255
        else:  # Magenta to Red
            r, g, b = 255, 0, 255 - ((i - 25) * 255) // 5
        rainbow_ground32.append((r, g, b))

    try:
        while True:
            readout = controller.read()
            controller.set_led_colours(
                Chu3LEDColours(
                    ground31_brightness=255,
                    ground31=[
                        ((v, v, v) if v >= 24 else r)
                        for r, v in zip(rainbow_ground32, readout.ground32)
                    ],
                    air_brightness=255,
                    air24_left=(air24 := [
                        (255, 255, 255) if v else (0, 0, 0)
                        for v in [x for x in readout.air6 for _ in range(4)]
                    ]),
                    air24_right=air24,
                )
            )
            sleep(0.008)

    except KeyboardInterrupt:
        pass

    controller.close()


def experimental_color_test():
    def set_led_colours(self: Chu3Controller, colours: Chu3LEDColours) -> None:
        """Set both ground and air LED colors using custom RGB protocol."""
        SYNC_BYTE = 0xE0
        HOST_ADDR = 1
        LED_ADDR = 2
        CMD_CUSTOM_RGB = 0x5B
        
        # Create the payload first
        payload = bytearray([
            CMD_CUSTOM_RGB,      # Command byte
            colours.ground31_brightness & 0xFF,  # Ground brightness
            colours.air_brightness & 0xFF,       # Tower brightness
        ])
        
        # Add ground RGB values (31 LEDs)
        for rgb in colours.ground31:
            payload.extend([
                rgb[1] & 0xFF,  # G
                rgb[0] & 0xFF,  # R
                rgb[2] & 0xFF   # B
            ])
        
        # Add left tower RGB values (24 LEDs)
        for rgb in colours.air24_left:
            payload.extend([
                rgb[1] & 0xFF,  # G
                rgb[0] & 0xFF,  # R
                rgb[2] & 0xFF   # B
            ])
        
        # Add right tower RGB values (24 LEDs)
        for rgb in colours.air24_right:
            payload.extend([
                rgb[1] & 0xFF,  # G
                rgb[0] & 0xFF,  # R
                rgb[2] & 0xFF   # B
            ])
    
        # Create the full packet with framing
        packet = bytearray([
            SYNC_BYTE,      # Sync byte
            LED_ADDR,       # Destination (LED board)
            HOST_ADDR,      # Source (host)
            len(payload),   # Length of payload
        ])
        packet.extend(payload)
        
        # Calculate checksum (sum of all bytes except sync)
        checksum = sum(packet[1:]) & 0xFF
        packet.append(checksum)
        
        # Send via COM port
        self.device_com.write(packet)
        self.device_com.flush()
    
    controller = Chu3Controller()
    controller.open()
    # set everything to white
    set_led_colours(
        controller,
        Chu3LEDColours(
            ground31_brightness=255,
            ground31=[(128, 128, 128)] * 31,
            air_brightness=255,
            air24_left=[(128, 128, 128)] * 24,
            air24_right=[(128, 128, 128)] * 24,
        )
    )
    input("...")
    controller.close()


if __name__ == "__main__":
    experimental_color_test()
    # color_test()
