"""CAN-bus driver for Waveshare PiperX grippers via USB-CAN serial adapter."""

import time
from typing import TYPE_CHECKING

try:
    import serial
except ImportError as e:
    if TYPE_CHECKING:
        raise e
    else:
        serial = None  # type: ignore

# CAN IDs
GRIPPER_CTRL_ID = 0x159
GRIPPER_FB_ID = 0x2A8


# -- Byte conversion helpers --------------------------------------------------


def _int32_to_be(value: int) -> list[int]:
    """Convert signed int32 to 4 big-endian bytes."""
    if not -2147483648 <= value <= 2147483647:
        raise OverflowError(f"int32 overflow: {value}")
    if value < 0:
        value = (value + 0x100000000) & 0xFFFFFFFF
    else:
        value &= 0xFFFFFFFF
    return [
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ]


def _uint16_to_be(value: int) -> list[int]:
    """Convert unsigned uint16 to 2 big-endian bytes."""
    if not 0 <= value <= 0xFFFF:
        raise OverflowError(f"uint16 overflow: {value}")
    return [
        (value >> 8) & 0xFF,
        value & 0xFF,
    ]


def _int32_from_be(bytes4: list[int]) -> int:
    """Parse 4 big-endian bytes into signed int32."""
    v = (bytes4[0] << 24) | (bytes4[1] << 16) | (bytes4[2] << 8) | bytes4[3]
    if v & 0x80000000:
        v -= 0x100000000
    return v


def _uint16_from_be(bytes2: list[int]) -> int:
    """Parse 2 big-endian bytes into unsigned uint16."""
    return (bytes2[0] << 8) | bytes2[1]


# -- Waveshare USB-CAN serial framing -----------------------------------------


def _build_serial_frame_std(id_11bit: int, data_bytes: list[int]) -> bytearray:
    """Build a Waveshare USB-CAN standard data frame.

    See: https://files.waveshare.com/wiki/USB-CAN-A/Demo/USB%20(Serial%20port)%20to%20CAN%20protocol%20defines.pdf
    """
    dlc = len(data_bytes)
    if not 0 <= dlc <= 8:
        raise ValueError(f"DLC must be 0-8, got {dlc}")
    frame = bytearray()
    frame.append(0xAA)  # header
    frame.append(0xC0 | dlc)  # standard + data frame
    frame.append(id_11bit & 0xFF)  # ID low (little-endian)
    frame.append((id_11bit >> 8) & 0xFF)  # ID high
    frame.extend(data_bytes)  # CAN payload
    frame.append(0x55)  # tail
    return frame


def _read_one_frame(ser: "serial.Serial") -> tuple[int, list[int]] | None:
    """Read one variable-length frame from serial.

    Returns (can_id, data_bytes) or None on timeout / sync loss.
    """
    # Sync to header 0xAA
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == 0xAA:
            break

    type_b = ser.read(1)
    if not type_b:
        return None
    type_b = type_b[0]

    dlc = type_b & 0x0F
    is_ext = bool(type_b & (1 << 5))
    is_remote = bool(type_b & (1 << 4))

    if is_remote:
        ser.read((4 if is_ext else 2) + dlc + 1)
        return None

    if is_ext:
        id_bytes = ser.read(4)
        if len(id_bytes) < 4:
            return None
        can_id = id_bytes[0] | (id_bytes[1] << 8) | (id_bytes[2] << 16) | (id_bytes[3] << 24)
    else:
        id_bytes = ser.read(2)
        if len(id_bytes) < 2:
            return None
        can_id = id_bytes[0] | (id_bytes[1] << 8)

    data = ser.read(dlc)
    if len(data) < dlc:
        return None

    tail = ser.read(1)
    if not tail or tail[0] != 0x55:
        return None

    return can_id, list(data)


# -- Driver class --------------------------------------------------------------


class WavesharePiperDriver:
    """Driver for Waveshare PiperX gripper over USB-CAN serial adapter.

    Args:
        port: serial port path (e.g. ``/dev/ttyUSB0``).
        baudrate: serial baudrate (default 2_000_000 per Waveshare spec).
        max_angle: maximum gripper angle in raw units (76101 for follower, 81000 for leader).
        default_effort: gripper effort in 0.001 N·m units (default 2000 = 2 N·m).
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 2_000_000,
        max_angle: int = 76_101,
        default_effort: int = 2000,
    ):
        self.port = port
        self.baudrate = baudrate
        self.max_angle = max_angle
        self.default_effort = default_effort
        self.ser: "serial.Serial | None" = None

    def start(self):
        """Open serial port and run initialization sequence."""
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=1,
            timeout=0.1,
        )
        # Initialization: send high-effort command to wake the gripper,
        # then zero-effort to release. This switches the gripper from its
        # boot CAN ID to the standard feedback ID.
        self._send_ctrl(grippers_angle=0, effort=self.default_effort, status_code=0x01)
        time.sleep(1.0)
        self._send_ctrl(grippers_angle=0, effort=0, status_code=0x01)

    def stop(self):
        """Disable gripper and close serial port."""
        if self.ser is not None and self.ser.is_open:
            self._send_ctrl(grippers_angle=0, effort=0, status_code=0x00)
            self.ser.close()
            self.ser = None

    def state(self) -> dict:
        """Read gripper feedback.

        Returns:
            Dict with ``gripper_position`` normalized to [0, 1] (0=closed, 1=open).
        """
        assert self.ser is not None
        frame = _read_one_frame(self.ser)
        if frame is None:
            return {"gripper_position": 0.0}

        can_id, data = frame
        if can_id != GRIPPER_FB_ID or len(data) < 7:
            return {"gripper_position": 0.0}

        grippers_angle = _int32_from_be(data[0:4])
        pos = max(0.0, min(1.0, grippers_angle / self.max_angle))
        return {"gripper_position": pos}

    def moveG(self, target_pos: float):
        """Move gripper to target position.

        Args:
            target_pos: normalized position in [0, 1] (0=closed, 1=open).
        """
        target_pos = max(0.0, min(1.0, target_pos))
        angle = int(target_pos * self.max_angle)
        self._send_ctrl(grippers_angle=angle, effort=self.default_effort, status_code=0x01)

    def _send_ctrl(self, grippers_angle: int, effort: int, status_code: int, set_zero: int = 0x00):
        """Build and send a gripper control CAN frame."""
        assert self.ser is not None
        payload = [
            *_int32_to_be(grippers_angle),
            *_uint16_to_be(effort),
            status_code & 0xFF,
            set_zero & 0xFF,
        ]
        frame = _build_serial_frame_std(GRIPPER_CTRL_ID, payload)
        self.ser.write(frame)
        self.ser.flush()
