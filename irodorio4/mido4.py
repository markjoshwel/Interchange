import hid
import serial
from typing import NamedTuple, Sequence
from sys import stderr
from rich.console import Console
from time import sleep


CHU3_IO4_VID = 0x0CA3  # sega io4 vendor number/id
CHU3_IO4_PID = 0x0021  # sega io4 product number/id

CHU3_IO4_REPORT_ID = 16  # host -> io4 commands
CHU3_IO4_CMD_SET_COMMUNICATION_TIMEOUT = 0x01
CHU3_IO4_CMD_SET_SAMPLING_COUNT = 0x02
CHU3_IO4_CMD_CLEAR_BOARD_STATUS = 0x03
CHU3_IO4_CMD_SET_PWM_OUTPUT = 0x04
CHU3_IO4_CMD_SET_UNIQUE_OUTPUT = 0x41
CHU3_IO4_SAMPLING_COUNT = 15
CHU3_IO4_COMM_TIMEOUT_US = 10  # 2000us = 2ms

# byte  0     : report id (ignored)
# bytes 1–32  : 32 bytes representing ground (slider/touch) sensor values
# bytes 33–38 : 6 bytes representing air (tower) sensor values
# byte  39    : A sensitivity/hardness value for the inputs
# bytes 40–63 : ignored
CHU3_READ_BYTES = 64


class Chu3InputLEDs(NamedTuple):
    """
    ground32_brightness: int
        brightness for the 32 ground LEDs
    ground32: Sequence[int]
        32 LED values
    air6_brightness: int
        brightness for the 6 air LEDs
    air6: Sequence[int]
        6 LED values

    all values are 0-255
    """

    ground32_brightness: int
    ground32: Sequence[int]
    air6_brightness: int
    air6: Sequence[int]


class Chu3SensorReadout(NamedTuple):
    """
    ground32: Sequence[int]
        32 ground sensor values, 0-255 for sense/touch pressure/capacitance
        -1 if it wasn't read within the frame of reading
    air6: Sequence[bool | None]
        6 air sensor values, True for detection/sensor blockage, False for no detection
        None if it wasn't read within the frame of reading
    """

    ground32: Sequence[int]
    air6: Sequence[bool | None]


class Chu3Controller:
    _init: bool = False
    
    def set_communication_timeout(self, timeout_us: int = CHU3_IO4_COMM_TIMEOUT_US) -> None:
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
        total = SLIDER_SYNC + SLIDER_CMD_Rx_REPORT_ENABLE + length  # should be 0xFF + 0x03 = 0x102
        checksum = (-total) & 0xFF  # Correct checksum: (-0x102) mod 256 = 0xFE
        packet = bytearray([SLIDER_SYNC, SLIDER_CMD_Rx_REPORT_ENABLE, length, checksum])
        self.device_com.write(packet)
        # print("Sent slider report enable command, packet:", packet)
    
    
    def init(self) -> None:
        if self._init:
            return
        
        print("chu3controller.init: clear board status", file=stderr)
        self.clear_board_status()

        print("chu3controller.init: set communication timeout", file=stderr)
        self.set_communication_timeout()

        print("chu3controller.init: set sampling count", file=stderr)
        self.set_sampling_count()
        
        print("chu3controller.init: enable slider report", file=stderr)
        self.slider_enable_report()

        self._init = True
    
    
    def open(self, nonblocking: bool = True, init=True) -> None:
        self.device_com = serial.Serial(
            port="COM1",
            baudrate=115200,
            timeout=0.1
        )
        print(
            f"chu3controller.open: opened {self.device_com.name} on COM1 ({self.device_com.baudrate} baud)",
        )

        self.device_hid = hid.device()
        self.device_hid.open(CHU3_IO4_VID, CHU3_IO4_PID)    

        print(
            f"chu3controller.open: opened {self.device_hid.get_manufacturer_string()} - {self.device_hid.get_product_string()} ({self.device_hid.get_serial_number_string()})",
            file=stderr,
        )

        self.device_hid.set_nonblocking(int(nonblocking))
        
        if init:
            self.init()


    def read(self, timeout_ms=CHU3_IO4_COMM_TIMEOUT_US * 1000) -> Chu3SensorReadout:
        slider32 = [-1] * 32
        air6: tuple[bool | None, ...]
        
        # read air from hid/io4
        data_io4 = self.device_hid.read(max_length=CHU3_READ_BYTES, timeout_ms=timeout_ms)
        if data_io4 is None:
            air6 = (None,) * 6
        else:
            air6 = self.parse_air_sensors(data_io4[32], data_io4[33])
        
        # read ground from serial/com
        data_serial = self.device_com.read(64)
        print(data_serial)
        
        return Chu3SensorReadout(slider32, air6)
    

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

    def close(self):
        print("chu3controller.init: clear board status", file=stderr)
        self.clear_board_status()

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


def slider_to_truecolor_block(sliders: tuple[bool, ...]) -> str:
    return " ".join("[red]█[/red]" if v else "[white]█[/white]" for v in sliders)


def test():
    controller = Chu3Controller()
    controller.open()
    
    console = Console()
    try:
        while True:
            # console.clear()
            data_serial = controller.device_com.read(64)
            print("".join(
                # turn into a ascii character from A to Z
                chr(65 + v // 10)
                for v in data_serial[:32]
            ))
            # top_16 = data_serial[:16]
            # bottom_16 = data_serial[16:32]
            # console.print("".join(sensor_to_truecolour_block(v) for v in top_16))
            # console.print("".join(sensor_to_truecolour_block(v) for v in bottom_16))
            # if len(data_serial) == 64:
            #     console.print("".join(sensor_to_truecolour_block(v) for v in data_serial[:32]))
    except KeyboardInterrupt:
        pass
    
    controller.close()


if __name__ == "__main__":
    test()
