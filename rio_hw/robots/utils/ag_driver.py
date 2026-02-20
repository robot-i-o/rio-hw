"""
Non-blocking interface for controlling the AG95 gripper via Modbus RTU over TCP/IP.
"""

import socket
import struct
import time
from enum import IntEnum


class GripperState(IntEnum):
    CLOSED = 0
    OPENED = 1


class AgGripperDriver:
    def __init__(self, robot_ip, tcp_port=54321, member_id=1, timeout=1.0, freq=30):
        self.robot_ip = robot_ip
        self.tcp_port = tcp_port
        self.member_id = member_id
        self.sock = None
        self.timeout = timeout
        self.freq = freq

        # Calculate drain interval to support at least 50Hz response
        # Drain at least twice the control frequency or 50Hz, whichever is higher
        drain_freq = max(freq * 2, 50)
        self.drain_interval = 1.0 / drain_freq

        # State tracking
        self.is_ready = False
        self.binary_state = GripperState.CLOSED
        self.state = 0

        # Avoid redundant writes
        self._last_position_sent = None
        self._last_drain_time = 0

    @staticmethod
    def _crc16(data: bytes) -> bytes:
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack("<H", crc)

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.robot_ip, self.tcp_port))
        time.sleep(1.0)

    def close_conn(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def _drain_buffer(self):
        """Periodically drain receive buffer at calculated frequency."""
        now = time.time()
        if now - self._last_drain_time > self.drain_interval:
            self.sock.setblocking(False)
            try:
                while True:
                    data = self.sock.recv(4096)
                    if not data:
                        break
            except BlockingIOError:
                pass
            finally:
                self.sock.setblocking(True)
                self._last_drain_time = now

    def _write_register_blocking(self, reg_addr: int, value: int):
        """Blocking write - use only for setup."""
        frame = bytearray()
        frame.append(self.member_id)
        frame.append(0x06)
        frame += struct.pack(">H", reg_addr)
        frame += struct.pack(">H", value)
        frame += self._crc16(frame)

        self.sock.sendall(frame)

        try:
            resp = self.sock.recv(8)
            return resp
        except TimeoutError as err:
            raise RuntimeError("No response from gripper") from err

    def _write_register_fast(self, reg_addr: int, value: int):
        """Fast non-blocking write."""
        frame = bytearray()
        frame.append(self.member_id)
        frame.append(0x06)
        frame += struct.pack(">H", reg_addr)
        frame += struct.pack(">H", value)
        frame += self._crc16(frame)

        # Fire and forget
        try:
            self.sock.send(frame)
        except BlockingIOError:
            pass  # Buffer full, skip this write

        # Drain buffer at calculated frequency
        self._drain_buffer()

    def initialize(self, full=False):
        """Initialize gripper. full=True => full init (find min & max)."""
        val = 0xA5 if full else 0x01
        self._write_register_blocking(0x0100, val)
        time.sleep(2)

    def set_force(self, percent: int):
        """Set grip force: 20-100%."""
        pct = max(20, min(100, percent))
        self._write_register_blocking(0x0101, pct)

    def set_position(self, permil: int):
        """Set target position: 0-1000‰ (0=closed, 1000=open)."""
        pos = max(0, min(1000, permil))

        # Skip if position hasn't changed
        if pos == self._last_position_sent:
            self.state = pos
            return

        self.state = pos
        self._last_position_sent = pos
        self._write_register_fast(0x0103, pos)

    def open(self):
        """Fully open gripper."""
        self.set_position(1000)
        self.binary_state = GripperState.OPENED

    def close(self):
        """Fully close gripper."""
        self.set_position(0)
        self.binary_state = GripperState.CLOSED

    def grip_width(self, width_permille):
        """Grip to a custom opening."""
        self.set_position(width_permille)

    def setup(self, force=50, do_calibration=True):
        """Initialize and set defaults."""
        self.initialize(full=do_calibration)
        self.set_force(force)
        self.close()  # Ensure gripper starts closed
        self.is_ready = True
