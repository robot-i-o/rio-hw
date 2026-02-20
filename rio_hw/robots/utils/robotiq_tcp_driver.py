"""
Non-blocking interface for controlling the Robotiq 2F-85/2F-140 via Modbus RTU over TCP/IP.
Updated to match the official register mappings from the instruction manual.
"""

import socket
import struct
import time
from enum import IntEnum


class GripperState(IntEnum):
    CLOSED = 0
    OPENED = 1


class RobotiqTcpDriver:
    def __init__(self, robot_ip, tcp_port=54321, member_id=0x09, timeout=1.0, freq=30):
        self.robot_ip = robot_ip
        self.tcp_port = tcp_port
        self.member_id = member_id  # Default is 0x09 (9) per manual
        self.sock = None
        self.timeout = timeout
        self.freq = freq

        # Register addresses per manual
        self.OUTPUT_BASE = 0x03E8  # 1000 - Robot Output / Gripper Input
        self.INPUT_BASE = 0x07D0  # 2000 - Robot Input / Gripper Output

        # State tracking
        self.is_ready = False
        self.binary_state = GripperState.CLOSED

        # Status from input registers
        self.gACT = 0  # Activation status
        self.gGTO = 0  # Action status
        self.gSTA = 0  # Gripper status (0-3)
        self.gOBJ = 0  # Object detection status (0-3)
        self.gFLT = 0  # Fault status
        self.gPO = 0  # Actual position (0x00=open, 0xFF=closed)
        self.gCU = 0  # Current

        # Output register cache (bytes 0-5)
        self.output_regs = bytearray(6)

        # Read request tracking
        self._last_read_time = 0
        self.read_interval = 1.0 / freq

        # Drain tracking
        self._last_drain_time = 0
        self.drain_interval = 1.0 / max(freq * 2, 50)

    @staticmethod
    def _crc16(data: bytes) -> bytes:
        """Calculate Modbus RTU CRC16."""
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

    @staticmethod
    def _normalized_to_byte(normalized: float) -> int:
        """
        Convert normalized position (0.0-1.0) to byte value (0x00-0xFF).
        1.0 = fully open (0x00)
        0.0 = fully closed (0xFF)
        """
        normalized = max(0.0, min(1.0, normalized))
        return int((1.0 - normalized) * 255)

    @staticmethod
    def _byte_to_normalized(byte_val: int) -> float:
        """
        Convert byte value (0x00-0xFF) to normalized position (0.0-1.0).
        0x00 = 1.0 (fully open)
        0xFF = 0.0 (fully closed)
        """
        return 1.0 - (byte_val / 255.0)

    def connect(self):
        """Establish TCP connection to the robot."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.robot_ip, self.tcp_port))
        time.sleep(0.5)

    def close_conn(self):
        """Close TCP connection."""
        if self.sock:
            self.sock.close()
            self.sock = None

    def _drain_write_responses(self):
        """Drain responses from write commands periodically."""
        now = time.time()
        if now - self._last_drain_time > self.drain_interval:
            old_timeout = self.sock.gettimeout()
            self.sock.settimeout(0.001)
            try:
                while True:
                    data = self.sock.recv(4096)
                    if not data:
                        break
            except TimeoutError:
                pass
            finally:
                self.sock.settimeout(old_timeout)
                self._last_drain_time = now

    def _write_output_registers(self):
        """
        Write all 6 output register bytes using FC16 (Preset Multiple Registers).
        Per manual: Only FC16 is supported for writing, not FC06.
        """
        # FC16 frame: MemberID | 0x10 | StartAddr(2) | NumRegs(2) | ByteCount | Data | CRC
        num_registers = 3  # 6 bytes = 3 registers (16-bit each)
        byte_count = 6

        frame = bytearray()
        frame.append(self.member_id)
        frame.append(0x10)  # FC16
        frame += struct.pack(">H", self.OUTPUT_BASE)
        frame += struct.pack(">H", num_registers)
        frame.append(byte_count)
        frame += self.output_regs
        frame += self._crc16(frame)

        self.sock.send(frame)

    def _read_input_registers(self):
        """
        Read input registers using FC03 (Read Holding Registers).
        Reads 3 registers (6 bytes) starting from INPUT_BASE (0x07D0).
        """
        # FC03 frame: MemberID | 0x03 | StartAddr(2) | NumRegs(2) | CRC
        frame = bytearray()
        frame.append(self.member_id)
        frame.append(0x03)  # FC03
        frame += struct.pack(">H", self.INPUT_BASE)
        frame += struct.pack(">H", 3)  # Read 3 registers (6 bytes)
        frame += self._crc16(frame)

        # Clear buffer before reading
        old_timeout = self.sock.gettimeout()
        self.sock.settimeout(0.001)
        try:
            while self.sock.recv(4096):
                pass
        except TimeoutError:
            pass

        # Send read request and wait for response
        self.sock.settimeout(0.1)  # 100ms timeout
        try:
            self.sock.sendall(frame)
            resp = self.sock.recv(256)

            # Response format: MemberID | 0x03 | ByteCount | Data | CRC
            if len(resp) >= 9 and resp[1] == 0x03:
                byte_count = resp[2]
                if byte_count == 6:
                    # Parse status registers
                    data = resp[3:9]

                    # Byte 0: GRIPPER STATUS
                    status = data[0]
                    self.gACT = status & 0x01
                    self.gGTO = (status >> 3) & 0x01
                    self.gSTA = (status >> 4) & 0x03
                    self.gOBJ = (status >> 6) & 0x03

                    # Byte 1: Reserved

                    # Byte 2: FAULT STATUS
                    self.gFLT = data[2]

                    # Byte 3: Position Request Echo
                    # (not currently stored)

                    # Byte 4: POSITION (actual position)
                    self.gPO = data[4]

                    # Byte 5: CURRENT
                    self.gCU = data[5]

        except TimeoutError:
            pass  # No response, keep old values
        finally:
            self.sock.settimeout(old_timeout)

    def update(self):
        """Call this every control loop iteration."""
        now = time.time()

        # Drain old write responses
        self._drain_write_responses()

        # Send read request if it's time
        if (now - self._last_read_time) >= self.read_interval:
            self._read_input_registers()
            self._last_read_time = now

    def state(self, refresh=True) -> float:
        """
        Get cached position (non-blocking).
        Returns normalized position: 1.0 = fully open, 0.0 = fully closed.
        """
        if refresh:
            self._read_input_registers()
        pos = self._byte_to_normalized(self.gPO)
        state = {"gripper_position": pos}
        return state

    def reset(self):
        """Reset the gripper by clearing all output registers."""
        self.output_regs[0] = 0x00
        self.output_regs[1] = 0x00
        self.output_regs[2] = 0x00
        self.output_regs[3] = 0x00
        self.output_regs[4] = 0x00
        self.output_regs[5] = 0x00

        old_timeout = self.sock.gettimeout()
        self.sock.settimeout(1.0)
        self._write_output_registers()

        try:
            _ = self.sock.recv(256)
        except TimeoutError:
            pass
        finally:
            self.sock.settimeout(old_timeout)

        time.sleep(0.5)

    def activate(self, timeout=10.0, debug=True):
        """
        Activate the gripper by setting rACT bit.
        Per manual: This must be the first action before any other operations.
        The gripper will perform auto-calibration when activated.

        Args:
            timeout: Maximum time to wait for activation (seconds)
            debug: Print status updates during activation
        """
        if debug:
            print("Resetting gripper first...")

        # Reset first (like reference code does)
        self.reset()

        if debug:
            print("Sending activation command...")

        # Set rACT bit (bit 0) in ACTION REQUEST (byte 0)
        self.output_regs[0] = 0x01  # rACT = 1
        self.output_regs[1] = 0x00  # Reserved
        self.output_regs[2] = 0x00  # Reserved
        self.output_regs[3] = 0x00  # Position
        self.output_regs[4] = 0xFF  # Speed (max)
        self.output_regs[5] = 0x80  # Force (medium)

        # Write registers using FC16
        old_timeout = self.sock.gettimeout()
        self.sock.settimeout(1.0)
        self._write_output_registers()

        # Wait for response
        try:
            resp = self.sock.recv(256)
            if debug:
                print(f"Write response received: {resp.hex()}")
        except TimeoutError:
            if debug:
                print("No write response (timeout)")
        finally:
            self.sock.settimeout(old_timeout)

        # Poll gSTA until activation completes (gSTA==3)
        if debug:
            print("Waiting for activation to complete (gSTA==3)...")

        start_time = time.time()
        last_gSTA = -1

        while (time.time() - start_time) < timeout:
            time.sleep(0.1)
            self._read_input_registers()

            # Debug output when gSTA changes
            if debug and self.gSTA != last_gSTA:
                elapsed = time.time() - start_time
                print(f"  [{elapsed:.1f}s] gSTA={self.gSTA}, gACT={self.gACT}, gFLT=0x{self.gFLT:02X}")
                last_gSTA = self.gSTA

            if self.gSTA == 3:
                if debug:
                    print(f"✓ Activation completed in {time.time() - start_time:.2f}s")
                return

        # Timeout - show final status
        status = self.get_status()
        raise TimeoutError(
            f"Activation did not complete within {timeout}s.\n"
            f"  gSTA={status['gSTA']} (should be 3)\n"
            f"  gACT={status['gACT']}\n"
            f"  gFLT=0x{status['gFLT']:02X}\n"
            f"  gPO={status['position']}"
        )

    def set_force(self, force_byte: int):
        """
        Set grip force.
        Per manual: 0x00 = minimum force, 0xFF = maximum force.
        """
        force_byte = max(0x00, min(0xFF, force_byte))
        self.output_regs[5] = force_byte

    def set_speed(self, speed_byte: int):
        """
        Set gripper speed.
        Per manual: 0x00 = minimum speed, 0xFF = maximum speed.
        """
        speed_byte = max(0x00, min(0xFF, speed_byte))
        self.output_regs[4] = speed_byte

    def moveL(self, position: float, wait=False, timeout=10.0):
        position_byte = self._normalized_to_byte(position)
        self.output_regs[3] = position_byte

        # Set rGTO bit (bit 3) to initiate motion, keep rACT (bit 0) set
        self.output_regs[0] = 0x09  # rACT=1, rGTO=1 (binary: 00001001)

        self._write_output_registers()

        if wait:
            return self.wait_for_motion(timeout)
        self.update()
        return None

    def wait_for_motion(self, timeout=10.0):
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            time.sleep(0.05)
            self.update()

            # gOBJ status (ignore if gGTO==0)
            if self.gGTO == 1:
                if self.gOBJ == 1 or self.gOBJ == 2:
                    # Object detected (stopped due to contact)
                    return (self._byte_to_normalized(self.gPO), True)
                elif self.gOBJ == 3:
                    # At requested position, no object
                    return (self._byte_to_normalized(self.gPO), False)

        raise TimeoutError(f"Motion did not complete within {timeout}s")

    def open(self, wait=False, timeout=10.0):
        result = self.moveL(1.0, wait=wait, timeout=timeout)
        self.binary_state = GripperState.OPENED
        return result

    def close(self, wait=False, timeout=10.0):
        result = self.moveL(0.0, wait=wait, timeout=timeout)
        self.binary_state = GripperState.CLOSED
        return result

    def grip_width(self, position: float, wait=False, timeout=10.0):
        return self.moveL(position, wait=wait, timeout=timeout)

    def stop(self):
        """Stop gripper motion by clearing rGTO bit."""
        self.output_regs[0] = 0x01  # rACT=1, rGTO=0
        self._write_output_registers()

    def deactivate(self):
        """Deactivate gripper by clearing rACT bit."""
        self.output_regs[0] = 0x00  # Clear all bits
        self._write_output_registers()
        self.is_ready = False

    def start(self, force=0x80, speed=0xFF):
        """
        Initialize and set defaults.
        force: 0x00-0xFF (default 0x80 = medium)
        speed: 0x00-0xFF (default 0xFF = maximum)
        """
        self.connect()
        self.activate()
        self.set_force(force)
        self.set_speed(speed)
        self.close()
        self.is_ready = True

    def get_status(self):
        """Return dictionary with current gripper status."""
        return {
            "gACT": self.gACT,  # Activation status
            "gGTO": self.gGTO,  # Action status
            "gSTA": self.gSTA,  # Gripper status (0-3)
            "gOBJ": self.gOBJ,  # Object detection (0-3)
            "gFLT": self.gFLT,  # Fault status
            "position": self._byte_to_normalized(self.gPO),  # Actual position (0.0-1.0, 1.0=open)
            "current": self.gCU,  # Current draw
            "is_ready": self.is_ready,
        }
