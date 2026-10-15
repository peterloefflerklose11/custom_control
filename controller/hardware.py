"""Direct motor access, plus an in-memory robot for tests and dry runs."""
from pathlib import Path
import numpy as np
from .calibration import ROLES, JOINT_INDICES, GRIPPER_INDICES
from .protocol import (ServoBus, TORQUE_ENABLE, GOAL_POSITION, PRESENT_POSITION,
                       GOAL_SPEED, ACCELERATION, TORQUE_LIMIT, MODE, OFFSET, normal_position)


class Robot:
    def __init__(self, cfg, calibration):
        self.cfg, self.calibration = cfg, calibration
        self.buses = {}
        self.engaged = False
        self.shutdown_home_blocked = False
        self.last_goal = None

    def open(self):
        from serial.tools.list_ports import comports
        known = {str(Path(p.device).resolve()): p.serial_number for p in comports()}
        try:
            for role in ROLES:
                path = self.cfg["ports"][role]
                expected = self.calibration.saved[role]["serial"]
                if known.get(str(Path(path).resolve())) != expected:
                    raise RuntimeError(f"{role}: USB serial does not match saved calibration {expected}")
                bus = ServoBus(path, self.cfg["baudrate"], self.cfg["serial_timeout_s"])
                self.buses[role] = bus
                for motor in range(1, 9):
                    if bus.read(motor, MODE, 1) != 0:
                        raise RuntimeError(f"{role}/{motor}: not in position mode")
                    # The station saves Offset as a signed little-endian i16.
                    raw = bus.read(motor, OFFSET, 2)
                    offset = raw - 65536 if raw & 0x8000 else raw
                    saved = self.calibration.saved[role]["motors"][str(motor)]["offset"]
                    if offset != saved:
                        raise RuntimeError(f"{role}/{motor}: EEPROM offset differs from copied calibration")
            self.read()
        except BaseException:
            self.close()
            raise

    def _counts(self):
        # No held/stale values: every motor must answer every observation.
        return {role: {m: normal_position(bus.read(m, PRESENT_POSITION, 2))
                       for m in range(1, 9)} for role, bus in self.buses.items()}

    def read(self):
        return self.calibration.decode(self._counts())

    def engage(self):
        # Write the left arm's CURRENT goals before enabling its torque.
        counts = self._counts()
        for role in ROLES:
            for i, motor in enumerate(range(1, 9)):
                if not self.calibration.minimum[role][i] <= counts[role][motor] <= self.calibration.maximum[role][i]:
                    raise RuntimeError(f"{role}/{motor}: current count outside saved range; refusing torque writes")
        pose = self.calibration.decode(counts)
        self.calibration.encode(pose)  # reject out-of-range startup pose
        for role, bus in self.buses.items():
            bus.write(GOAL_POSITION, 2, counts[role])
            for motor, value in counts[role].items():
                if normal_position(bus.read(motor, GOAL_POSITION, 2)) != value:
                    raise RuntimeError(f"{role}/{motor}: hold goal not accepted")
        self.last_goal = pose.copy()
        # Set only RAM profiles. EEPROM is never modified by this controller.
        for role, bus in self.buses.items():
            for address, width, value in ((GOAL_SPEED, 2, self.cfg["servo_speed"]),
                                           (ACCELERATION, 1, self.cfg["servo_acceleration"]),
                                           (TORQUE_LIMIT, 2, self.cfg["servo_torque_limit"])):
                self._verified_write(bus, role, address, width, value)
        for role, bus in self.buses.items():
            self._verified_write(bus, role, TORQUE_ENABLE, 1, 1)
        self.engaged = True
        return pose

    @staticmethod
    def _verified_write(bus, role, address, width, value):
        for _ in range(3):
            bus.write(address, width, dict.fromkeys(range(1, 9), value))
            if all(bus.read(m, address, width) == value for m in range(1, 9)):
                return
        raise RuntimeError(f"{role}: register 0x{address:02x} did not accept {value}")

    def command(self, target):
        if not self.engaged:
            raise RuntimeError("Robot is disarmed")
        # Defense at the final write boundary: never silently alter a command.
        delta = np.abs(np.asarray(target) - self.last_goal)
        if np.any(delta[JOINT_INDICES] > self.cfg["max_joint_step_rad"] + 1e-9):
            raise ValueError("Motor write exceeds joint step limit")
        if np.any(delta[GRIPPER_INDICES] > self.cfg["max_gripper_step"] + 1e-9):
            raise ValueError("Motor write exceeds gripper step limit")
        goals = self.calibration.encode(target)
        for role, bus in self.buses.items():
            bus.write(GOAL_POSITION, 2, goals[role])
        self.last_goal = np.array(target, copy=True)

    def close(self):
        # Keep the servo's last position goal and torque state. Automatically
        # dropping torque could drop an arm; automatically parking could hit
        # an obstacle. Supporting/relaxing the arms is an operator action.
        for bus in self.buses.values():
            bus.close()
        self.buses.clear()


class MockRobot:
    """Deterministic simulated readings; never opens serial devices."""
    def __init__(self, cfg, calibration):
        home = calibration.robot["home_joints_rad"]
        self.state = np.array(home + [0.5], dtype=float)
        self.last_goal = self.state.copy()
        self.engaged = False
        self.shutdown_home_blocked = False

    def open(self):
        pass

    def read(self):
        self.state += 0.8 * (self.last_goal - self.state)
        return self.state.copy()

    def engage(self):
        self.engaged = True
        return self.read()

    def command(self, target):
        if not self.engaged:
            raise RuntimeError("Mock robot is disarmed")
        self.last_goal = np.array(target, copy=True)

    def close(self):
        pass
