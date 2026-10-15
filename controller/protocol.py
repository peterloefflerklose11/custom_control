"""Small ST3215 wire implementation; no station or compiled extension needed.

Packet: FF FF | motor ID | length | instruction | parameters | checksum.
Checksum is the inverted sum from motor ID through the last parameter.
Register/packet reference and its license are copied into reference/protocol/.
"""
import time

TORQUE_ENABLE = 0x28
ACCELERATION = 0x29
GOAL_POSITION = 0x2A
GOAL_SPEED = 0x2E
TORQUE_LIMIT = 0x30
PRESENT_POSITION = 0x38
MODE = 0x21
OFFSET = 0x1F


def packet(motor, instruction, params=b""):
    body = bytes([motor, len(params) + 2, instruction]) + bytes(params)
    return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])


def sync_packet(address, width, values):
    if not values or width not in (1, 2):
        raise ValueError("SyncWrite needs values of width 1 or 2")
    data = bytes([address, width])
    for motor, value in sorted(values.items()):
        if not 1 <= motor <= 253 or not 0 <= value < (1 << (8 * width)):
            raise ValueError("Invalid motor ID or register value")
        data += bytes([motor]) + int(value).to_bytes(width, "little")
    if len(data) > 253:
        raise ValueError("SyncWrite packet too large")
    return packet(0xFE, 0x83, data)


def normal_position(raw):
    """ST3215 positions can use sign/magnitude; normalize to 0..4095."""
    magnitude = raw & 0x0FFF
    return (-magnitude if raw & 0x8000 else magnitude) & 0x0FFF


class ServoBus:
    def __init__(self, path, baudrate, timeout):
        import serial
        # Exclusive opens prevent a second cooperative process using this bus.
        self.port = serial.Serial(path, baudrate=baudrate, timeout=timeout,
                                  write_timeout=timeout, exclusive=True)
        self.timeout = timeout

    def _send(self, data):
        if self.port.write(data) != len(data):
            raise OSError("Incomplete serial write")

    def read(self, motor, address, width):
        self.port.reset_input_buffer()
        self._send(packet(motor, 0x02, bytes([address, width])))
        deadline = time.monotonic() + self.timeout
        received = bytearray()
        while time.monotonic() < deadline:
            received.extend(self.port.read(1))
            # Discard noise, retaining a possible first header byte.
            start = received.find(b"\xff\xff")
            if start < 0:
                received[:] = received[-1:]
                continue
            del received[:start]
            if len(received) < 4:
                continue
            size = received[3] + 4
            if size < 6 or size > 260:
                raise OSError("Invalid servo response length")
            if len(received) < size:
                continue
            response = bytes(received[:size])
            if response[2] != motor or response[3] != width + 2:
                raise OSError(f"Unexpected servo response for motor {motor}")
            if sum(response[2:]) & 0xFF != 0xFF:
                raise OSError(f"Servo checksum failed for motor {motor}")
            if response[4]:
                raise OSError(f"Motor {motor} reported error bits 0x{response[4]:02x}")
            return int.from_bytes(response[5:-1], "little")
        raise TimeoutError(f"Motor {motor} did not answer register 0x{address:02x}")

    def write(self, address, width, values):
        # Broadcast SyncWrite has no acknowledgement. Startup writes are
        # explicitly read back by hardware.py; tracking checks watch goals.
        self._send(sync_packet(address, width, values))

    def close(self):
        self.port.close()
