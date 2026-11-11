import argparse
import struct
import sys
import time

import pysoem

CONTROLWORD = 0x6040
STATUSWORD = 0x6041
MODE_OF_OPERATION = 0x6060
MODE_OF_OPERATION_DISPLAY = 0x6061
POSITION_ACTUAL = 0x6064
VELOCITY_ACTUAL = 0x606C
TARGET_VELOCITY = 0x60FF

# The second axis objects are at the first axis index + 0x800 (0x6840 = 0x6040 + 0x800)
AXIS2_OFFSET = 0x800

CW_DISABLE_VOLTAGE = 0x0000
CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_FAULT_RESET = 0x0080

CYCLE_TIME_S = 0.002

# Abort motion if the measured velocity exceeds this multiple of --max-velocity
ACTUAL_VELOCITY_ABORT_FACTOR = 1.5

MODE_NAMES = {
    0: "No mode",
    1: "Profile position",
    3: "Profile velocity",
    4: "Profile torque",
    6: "Homing",
    7: "Interpolated position",
    8: "Cyclic synchronous position",
    9: "Cyclic synchronous velocity",
    10: "Cyclic synchronous torque",
}


def sdo_uint(slave, index, subindex):
    return int.from_bytes(slave.sdo_read(index, subindex), "little")


def read_pdo_map(slave, assign_index):
    """Return a list of (index, subindex, bitlen) for all PDOs assigned in assign_index."""
    entries = []
    for i in range(1, sdo_uint(slave, assign_index, 0) + 1):
        pdo_index = sdo_uint(slave, assign_index, i)
        for j in range(1, sdo_uint(slave, pdo_index, 0) + 1):
            value = sdo_uint(slave, pdo_index, j)
            entries.append((value >> 16, (value >> 8) & 0xFF, value & 0xFF))
    return entries


def byte_offset(entries, index, subindex=0):
    """Return the byte offset of an object inside the process image, or None if unmapped."""
    bit = 0
    for obj_index, obj_sub, bitlen in entries:
        if obj_index == index and obj_sub == subindex:
            if bit % 8:
                raise RuntimeError(f"0x{index:04X} is not byte aligned in the PDO mapping")
            return bit // 8
        bit += bitlen
    return None


def decode_statusword(sw):
    if sw & 0x4F == 0x40:
        return "Switch on disabled"
    if sw & 0x6F == 0x21:
        return "Ready to switch on"
    if sw & 0x6F == 0x23:
        return "Switched on"
    if sw & 0x6F == 0x27:
        return "Operation enabled"
    if sw & 0x6F == 0x07:
        return "Quick stop active"
    if sw & 0x4F == 0x0F:
        return "Fault reaction active"
    if sw & 0x4F == 0x08:
        return "Fault"
    if sw & 0x4F == 0x00:
        return "Not ready to switch on"
    return "Unknown"


class Axis:
    """Object access and CiA 402 state handling for one axis of the drive."""

    def __init__(self, drive, name, rx_entries, tx_entries, base=0):
        self.drive = drive
        self.name = name
        self.cw_off = byte_offset(rx_entries, CONTROLWORD + base)
        self.sw_off = byte_offset(tx_entries, STATUSWORD + base)
        if self.cw_off is None:
            raise RuntimeError(f"{name}: controlword 0x{CONTROLWORD + base:04X} is not in the "
                               "RxPDO mapping")
        if self.sw_off is None:
            raise RuntimeError(f"{name}: statusword 0x{STATUSWORD + base:04X} is not in the "
                               "TxPDO mapping")
        self.mode_index = MODE_OF_OPERATION + base
        self.mode_disp_index = MODE_OF_OPERATION_DISPLAY + base
        self.target_vel_index = TARGET_VELOCITY + base
        self.mode_off = byte_offset(rx_entries, self.mode_index)
        self.target_vel_off = byte_offset(rx_entries, self.target_vel_index)
        self.mode_disp_off = byte_offset(tx_entries, self.mode_disp_index)
        self.pos_off = byte_offset(tx_entries, POSITION_ACTUAL + base)
        self.vel_off = byte_offset(tx_entries, VELOCITY_ACTUAL + base)
        self.mode = None
        self.controlword = CW_DISABLE_VOLTAGE
        self.target_velocity = 0
        self.statusword = 0

    def pack_outputs(self, out):
        struct.pack_into("<H", out, self.cw_off, self.controlword)
        if self.mode is not None and self.mode_off is not None:
            struct.pack_into("<b", out, self.mode_off, self.mode)
        if self.target_vel_off is not None:
            struct.pack_into("<i", out, self.target_vel_off, int(self.target_velocity))

    def unpack_inputs(self, data):
        self.statusword = struct.unpack_from("<H", data, self.sw_off)[0]

    def cycle(self):
        """Run one process data cycle and return this axis' statusword."""
        self.drive.cycle()
        return self.statusword

    def _read(self, fmt, offset):
        if offset is None:
            return None
        return struct.unpack_from(fmt, self.drive.slave.input, offset)[0]

    def mode_display(self):
        return self._read("<b", self.mode_disp_off)

    def position(self):
        return self._read("<i", self.pos_off)

    def velocity(self):
        return self._read("<i", self.vel_off)

    def telemetry(self):
        mode = self.mode_display()
        mode_text = "n/a" if mode is None else f"{mode} ({MODE_NAMES.get(mode, 'unknown')})"
        pos = self.position()
        vel = self.velocity()
        return (f"{self.name}: statusword 0x{self.statusword:04X} "
                f"({decode_statusword(self.statusword)}), "
                f"mode display {mode_text}, "
                f"position {'n/a' if pos is None else pos}, "
                f"velocity {'n/a' if vel is None else vel}")

    def command(self, controlword, mask, expected, timeout=2.0):
        """Send controlword until (statusword & mask) == expected."""
        self.controlword = controlword
        end = time.monotonic() + timeout
        sw = self.cycle()
        while (sw & mask) != expected:
            if time.monotonic() > end:
                raise RuntimeError(
                    f"{self.name}: timeout with controlword 0x{controlword:04X}: statusword "
                    f"0x{sw:04X} ({decode_statusword(sw)}), expected 0x{expected:04X}"
                )
            sw = self.cycle()
        print(f"{self.name}: controlword 0x{controlword:04X} -> statusword 0x{sw:04X} "
              f"({decode_statusword(sw)})")
        return sw

    def select_mode(self, mode, timeout=2.0):
        """Write Modes of operation and wait until the display object shows it."""
        if self.mode_off is None:
            raise RuntimeError(f"{self.name}: modes of operation 0x{self.mode_index:04X} is not "
                               "in the RxPDO mapping")
        if self.mode_disp_off is None:
            raise RuntimeError(f"{self.name}: modes of operation display "
                               f"0x{self.mode_disp_index:04X} is not in the TxPDO mapping")
        self.mode = mode
        end = time.monotonic() + timeout
        while self.cycle() is not None and self.mode_display() != mode:
            if time.monotonic() > end:
                raise RuntimeError(
                    f"{self.name}: mode {mode} not accepted: mode display is "
                    f"{self.mode_display()}"
                )
        print(f"{self.name}: mode of operation {mode} ({MODE_NAMES.get(mode, 'unknown')}) active")

    def enable(self):
        sw = self.cycle()
        if sw & 0x4F in (0x08, 0x0F):
            print(f"{self.name}: drive in fault, sending fault reset")
            self.controlword = CW_FAULT_RESET
            self.drive.run_cycles(0.1)
            self.command(CW_DISABLE_VOLTAGE, 0x4F, 0x40)
        self.command(CW_SHUTDOWN, 0x6F, 0x21)
        self.command(CW_SWITCH_ON, 0x6F, 0x23)
        self.command(CW_ENABLE_OPERATION, 0x6F, 0x27)

    def check_safe(self, max_velocity):
        """Raise if the drive faulted or the measured velocity is beyond the limit."""
        if self.statusword & 0x08:
            raise RuntimeError(f"{self.name}: drive fault, statusword 0x{self.statusword:04X}")
        if self.statusword & 0x6F != 0x27:
            raise RuntimeError(
                f"{self.name}: drive left Operation enabled, statusword 0x{self.statusword:04X}"
            )
        vel = self.velocity()
        if vel is not None and abs(vel) > max_velocity * ACTUAL_VELOCITY_ABORT_FACTOR:
            raise RuntimeError(
                f"{self.name}: measured velocity {vel} exceeds {ACTUAL_VELOCITY_ABORT_FACTOR} x "
                f"max velocity {max_velocity}"
            )


class Drive:
    """Process data exchange for the slave. All axes share one process image."""

    def __init__(self, master, slave):
        self.master = master
        self.slave = slave
        self.axes = []

    def add_axis(self, name, rx_entries, tx_entries, base=0):
        axis = Axis(self, name, rx_entries, tx_entries, base)
        self.axes.append(axis)
        return axis

    def cycle(self):
        out = bytearray(len(self.slave.output))
        for axis in self.axes:
            axis.pack_outputs(out)
        self.slave.output = bytes(out)
        self.master.send_processdata()
        wkc = self.master.receive_processdata(2000)
        time.sleep(CYCLE_TIME_S)
        if wkc != self.master.expected_wkc:
            print(f"Warning: working counter {wkc}, expected {self.master.expected_wkc}")
        for axis in self.axes:
            axis.unpack_inputs(self.slave.input)

    def telemetry(self):
        return " | ".join(axis.telemetry() for axis in self.axes)

    def print_telemetry(self, prefix=""):
        for axis in self.axes:
            print(f"{prefix}{axis.telemetry()}")

    def run_cycles(self, seconds, log_interval=None):
        end = time.monotonic() + seconds
        next_log = time.monotonic()
        while time.monotonic() < end:
            self.cycle()
            if log_interval is not None and time.monotonic() >= next_log:
                self.print_telemetry("  ")
                next_log += log_interval

    def disable(self):
        """Zero all target velocities, then step every axis back to Switch on disabled."""
        for axis in self.axes:
            axis.target_velocity = 0
        self.run_cycles(0.2)
        for axis in self.axes:
            axis.controlword = CW_SHUTDOWN
        self.run_cycles(0.2)
        for axis in self.axes:
            axis.controlword = CW_DISABLE_VOLTAGE
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            self.cycle()
            if all(axis.statusword & 0x4F == 0x40 for axis in self.axes):
                break
        for axis in self.axes:
            print(f"{axis.name}: statusword after shutdown: 0x{axis.statusword:04X} "
                  f"({decode_statusword(axis.statusword)})")

    def ramp_velocity(self, targets, accel, limits, log_interval=0.25):
        """Move each axis target velocity to its value in `targets` at `accel` units/s^2.

        `targets` and `limits` map Axis -> target velocity / max velocity. Safety checks run
        on every axis in `targets` each cycle.
        """
        starts = {axis: axis.target_velocity for axis in targets}
        durations = {axis: abs(targets[axis] - starts[axis]) / accel for axis in targets}
        total = max(durations.values())
        t0 = time.monotonic()
        next_log = t0
        while True:
            elapsed = time.monotonic() - t0
            for axis in targets:
                d = durations[axis]
                fraction = 1.0 if d == 0 else min(elapsed / d, 1.0)
                axis.target_velocity = round(
                    starts[axis] + (targets[axis] - starts[axis]) * fraction)
            self.cycle()
            for axis in targets:
                axis.check_safe(limits[axis])
            if time.monotonic() >= next_log:
                self.print_move_telemetry(targets)
                next_log += log_interval
            if elapsed >= total:
                return

    def hold_velocity(self, seconds, axes, limits, log_interval=0.25):
        end = time.monotonic() + seconds
        next_log = time.monotonic()
        while time.monotonic() < end:
            self.cycle()
            for axis in axes:
                axis.check_safe(limits[axis])
            if time.monotonic() >= next_log:
                self.print_move_telemetry(axes)
                next_log += log_interval

    def print_move_telemetry(self, axes):
        for axis in axes:
            print(f"  target {axis.target_velocity}: " + axis.telemetry())


def goto_state(master, slave, state, name, timeout_us=2_000_000):
    slave.state = state
    slave.write_state()
    master.state_check(state, timeout_us)
    master.read_state()
    print(f"Slave AL state after {name} request: 0x{slave.state:02X}, AL status code 0x{slave.al_status:04X}")
    return slave.state == state


def go_operational(master, slave, drive):
    master.state = pysoem.OP_STATE
    drive.cycle()
    master.write_state()
    for _ in range(100):
        drive.cycle()
        master.read_state()
        if slave.state == pysoem.OP_STATE:
            print(f"Slave AL state: 0x{slave.state:02X} (OP)")
            return
    raise RuntimeError(
        f"Slave did not reach OP: state 0x{slave.state:02X}, AL status code 0x{slave.al_status:04X}"
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read CiA 402 drive status over EtherCAT, optionally enable the drive and "
                    "run a velocity move (axis 1: 0x6040..0x60FF, axis 2: 0x6840..0x68FF). "
                    "Velocity units are those of the drive's object 0x60FF/0x606C.")
    parser.add_argument("interface", nargs="?", default="eth0")
    parser.add_argument("--enable", action="store_true",
                        help="run the CiA 402 enable sequence (the motor may be energized)")
    parser.add_argument("--hold", type=float, default=3.0,
                        help="seconds to hold the drive enabled when no velocity is set (default 3)")
    parser.add_argument("--mode", type=int, default=None,
                        help="value written to Modes of operation 0x6060 (for example 9 = CSV)")
    parser.add_argument("--velocity", type=int, default=None,
                        help="target velocity written to 0x60FF; requires --enable, --mode and "
                             "--max-velocity")
    parser.add_argument("--max-velocity", type=int, default=None,
                        help="limit: |--velocity| must not exceed it, and the move aborts if the "
                             f"measured velocity exceeds {ACTUAL_VELOCITY_ABORT_FACTOR} x this value")
    parser.add_argument("--accel", type=int, default=None,
                        help="velocity ramp in units/s^2 (required with --velocity)")
    parser.add_argument("--motion-time", type=float, default=None,
                        help="seconds to hold the target velocity (required with --velocity)")
    parser.add_argument("--velocity2", type=int, default=None,
                        help="target velocity written to 0x68FF (second axis); requires --enable, "
                             "--mode, --max-velocity2, --accel and --motion-time. The mode, ramp "
                             "and hold time are shared with the first axis")
    parser.add_argument("--max-velocity2", type=int, default=None,
                        help="limit for the second axis, same meaning as --max-velocity")
    args = parser.parse_args()

    for suffix, velocity, max_velocity in (
            ("", args.velocity, args.max_velocity),
            ("2", args.velocity2, args.max_velocity2)):
        if velocity is None:
            if max_velocity is not None:
                parser.error(f"--max-velocity{suffix} requires --velocity{suffix}")
            continue
        missing = [name for name, value in (
            ("--enable", args.enable or None), ("--mode", args.mode),
            (f"--max-velocity{suffix}", max_velocity), ("--accel", args.accel),
            ("--motion-time", args.motion_time)) if value is None]
        if missing:
            parser.error(f"--velocity{suffix} also requires " + ", ".join(missing))
        if max_velocity <= 0 or args.accel <= 0 or args.motion_time <= 0:
            parser.error(f"--max-velocity{suffix}, --accel and --motion-time must be positive")
        if abs(velocity) > max_velocity:
            parser.error(f"|--velocity{suffix}| {abs(velocity)} exceeds --max-velocity{suffix} "
                         f"{max_velocity}")
    return args


def main():
    args = parse_args()

    master = pysoem.Master()
    drive = None
    try:
        master.open(args.interface)
        count = master.config_init()

        if count == 0:
            raise RuntimeError("No EtherCAT slaves discovered")

        print(f"Discovered {count} slave(s)")

        slave = master.slaves[0]

        print("Slave name:", slave.name)
        print(f"Vendor ID: 0x{slave.man:08X}")
        print(f"Product ID: 0x{slave.id:08X}")

        master.read_state()
        print(f"Slave AL state before: 0x{slave.state:02X}")

        # Request PRE-OP explicitly so the mailbox (CoE/SDO) becomes active
        if not goto_state(master, slave, pysoem.PREOP_STATE, "PRE-OP"):
            raise RuntimeError("Slave did not reach PRE-OP")

        # CiA 402 statusword over SDO, if implemented and accessible
        try:
            data = slave.sdo_read(STATUSWORD, 0)
            status = struct.unpack("<H", data)[0]
            print(f"Drive statusword (SDO): 0x{status:04X} ({decode_statusword(status)})")
        except Exception as e:
            print("Statusword unavailable:", type(e).__name__, e)
            abort_code = getattr(e, "abort_code", None)
            if abort_code is not None:
                print(f"SDO abort code: 0x{abort_code:08X}")

        # Read the PDO mapping from the slave: 0x1C12 = RxPDO assign, 0x1C13 = TxPDO assign
        rx_entries = read_pdo_map(slave, 0x1C12)
        tx_entries = read_pdo_map(slave, 0x1C13)
        print("RxPDO (master -> slave):", [f"0x{i:04X}:{s:02X}/{b}b" for i, s, b in rx_entries])
        print("TxPDO (slave -> master):", [f"0x{i:04X}:{s:02X}/{b}b" for i, s, b in tx_entries])

        io_size = master.config_map()
        print(f"Process image size: {io_size} bytes "
              f"(out {len(slave.output)}, in {len(slave.input)})")

        if not master.state_check(pysoem.SAFEOP_STATE, 2_000_000) == pysoem.SAFEOP_STATE:
            master.read_state()
            raise RuntimeError(
                f"Slave did not reach SAFE-OP: state 0x{slave.state:02X}, "
                f"AL status code 0x{slave.al_status:04X}"
            )
        print("Slave AL state: 0x04 (SAFE-OP)")

        drive = Drive(master, slave)
        # An axis is only driven when requested; otherwise its output bytes stay zero.
        # Axis 1 is also the default when no velocity is given (status read, enable and hold).
        moves = {}
        axes = []
        if args.velocity is not None or args.velocity2 is None:
            axis1 = drive.add_axis("Axis 1", rx_entries, tx_entries)
            axes.append(axis1)
            if args.velocity is not None:
                moves[axis1] = (args.velocity, args.max_velocity)
        if args.velocity2 is not None:
            axis2 = drive.add_axis("Axis 2", rx_entries, tx_entries, AXIS2_OFFSET)
            axes.append(axis2)
            moves[axis2] = (args.velocity2, args.max_velocity2)
        for axis in moves:
            if axis.target_vel_off is None:
                raise RuntimeError(f"{axis.name}: target velocity 0x{axis.target_vel_index:04X} "
                                   "is not in the RxPDO mapping")
        go_operational(master, slave, drive)

        drive.run_cycles(0.5)
        drive.print_telemetry("Drive (PDO): ")

        if args.mode is not None:
            for axis in axes:
                axis.select_mode(args.mode)
            drive.run_cycles(0.2)
            drive.print_telemetry("Drive (PDO): ")

        if args.enable:
            for axis in axes:
                axis.enable()
            if not moves:
                drive.run_cycles(args.hold, log_interval=0.5)
                drive.print_telemetry("Final: ")
            else:
                limits = {axis: limit for axis, (_, limit) in moves.items()}
                for axis, (velocity, limit) in moves.items():
                    print(f"{axis.name}: ramping to velocity {velocity} at {args.accel}/s^2, "
                          f"hold {args.motion_time}s, limit {limit}")
                drive.ramp_velocity({a: v for a, (v, _) in moves.items()}, args.accel, limits)
                drive.hold_velocity(args.motion_time, list(moves), limits)
                print("Ramping down to 0")
                drive.ramp_velocity({a: 0 for a in moves}, args.accel, limits)
                drive.run_cycles(0.5, log_interval=0.25)
                drive.print_telemetry("Final: ")
        else:
            print("Enable sequence skipped (pass --enable to run it)")
    finally:
        if drive is not None:
            # Remove target velocity and drive enable before leaving OP
            try:
                drive.disable()
            except Exception as e:
                print("Could not disable drive:", type(e).__name__, e)
        try:
            if master.slaves:
                master.state = pysoem.INIT_STATE
                master.write_state()
        except Exception:
            pass
        master.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", type(e).__name__, e)
        sys.exit(1)
