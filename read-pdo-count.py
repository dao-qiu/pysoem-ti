import argparse
import sys
import time

import pysoem

RXPDO_ASSIGN = 0x1C12
TXPDO_ASSIGN = 0x1C13

# Object index base of each axis, from the PDO mapping read on the tested slave
# (0x1600/0x1A00 -> 0x6000, 0x1601/0x1A01 -> 0x6800, 0x1602/0x1A02 -> 0x7000)
AXIS_BASES = (0x6000, 0x6800, 0x7000)

# (label, offset from axis base, signed) for the objects read per axis
AXIS_OBJECTS = (
    ("statusword", 0x41, False),
    ("mode display", 0x61, True),
    ("position actual", 0x64, True),
    ("velocity actual", 0x6C, True),
    ("torque actual", 0x77, True),
)

AL_STATE_NAMES = {0x01: "INIT", 0x02: "PRE-OP", 0x03: "BOOT", 0x04: "SAFE-OP", 0x08: "OP"}

CYCLE_TIME_S = 0.002
OP_REQUEST_CYCLES = 1000
OP_SETTLE_CYCLES = 100


def sdo_uint(slave, index, subindex):
    return int.from_bytes(slave.sdo_read(index, subindex), "little")


def sdo_int(slave, index, subindex, signed):
    return int.from_bytes(slave.sdo_read(index, subindex), "little", signed=signed)


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


def list_pdos(slave, assign_index):
    """Return {pdo_index: [(index, subindex, bitlen), ...]} for the PDOs assigned in assign_index."""
    pdos = {}
    for i in range(1, sdo_uint(slave, assign_index, 0) + 1):
        pdo_index = sdo_uint(slave, assign_index, i)
        entries = []
        for j in range(1, sdo_uint(slave, pdo_index, 0) + 1):
            value = sdo_uint(slave, pdo_index, j)
            entries.append((value >> 16, (value >> 8) & 0xFF, value & 0xFF))
        pdos[pdo_index] = entries
    return pdos


def print_pdos(title, pdos):
    print(f"  {title}: {len(pdos)}")
    for pdo_index, entries in pdos.items():
        total_bits = sum(bitlen for _, _, bitlen in entries)
        print(f"    0x{pdo_index:04X}: {len(entries)} mapped objects, {total_bits} bits "
              f"({(total_bits + 7) // 8} bytes)")
        offset_bits = 0
        for index, subindex, bitlen in entries:
            print(f"      +{offset_bits // 8:3d}  0x{index:04X}:{subindex:02X}  {bitlen:2d} bit")
            offset_bits += bitlen


def object_locations(pdos):
    """Return {(index, subindex): (byte_offset, byte_length)} for PDOs laid out back to back."""
    locations = {}
    bit = 0
    for entries in pdos.values():
        for index, subindex, bitlen in entries:
            if bit % 8 or bitlen % 8:
                raise RuntimeError(f"0x{index:04X}:{subindex:02X} is not byte aligned")
            locations[(index, subindex)] = (bit // 8, bitlen // 8)
            bit += bitlen
    return locations


def read_axis_sdo(slave, base):
    """Return {label: int or error text} read over SDO (needs PRE-OP or higher)."""
    values = {}
    for label, offset, signed in AXIS_OBJECTS:
        try:
            values[label] = sdo_int(slave, base + offset, 0, signed)
        except Exception as e:
            values[label] = f"unavailable ({type(e).__name__} {e})"
    return values


def read_axis_pdo(slave, locations, base):
    """Return {label: int or None} from the current input process image."""
    image = bytes(slave.input)
    values = {}
    for label, offset, signed in AXIS_OBJECTS:
        location = locations.get((base + offset, 0))
        if location is None:
            values[label] = None
            continue
        start, length = location
        values[label] = int.from_bytes(image[start:start + length], "little", signed=signed)
    return values


def format_value(label, value):
    if value is None:
        return "not mapped"
    if isinstance(value, str):
        return value
    if label == "statusword":
        return f"0x{value:04X} ({decode_statusword(value)})"
    return str(value)


def print_axes(slave, tx_locations=None):
    """Print the five values per axis. SDO always, plus PDO when tx_locations is given."""
    for axis, base in enumerate(AXIS_BASES, start=1):
        print(f"  Axis {axis} (objects 0x{base:04X}..):")
        sdo = read_axis_sdo(slave, base)
        pdo = read_axis_pdo(slave, tx_locations, base) if tx_locations is not None else None
        for label, offset, _ in AXIS_OBJECTS:
            line = f"    {label:16s} 0x{base + offset:04X}"
            if pdo is not None:
                line += f"  PDO: {format_value(label, pdo[label]):<34s}"
            line += f"  SDO: {format_value(label, sdo[label])}"
            print(line)


def print_states(master, title):
    master.read_state()
    for number, slave in enumerate(master.slaves):
        state = slave.state & 0x0F
        print(f"{title} slave {number}: AL state 0x{slave.state:02X} "
              f"({AL_STATE_NAMES.get(state, 'unknown')}), AL status code 0x{slave.al_status:04X}")


def request_state(master, state, name, timeout_us=2_000_000):
    master.state = state
    master.write_state()
    master.state_check(state, timeout_us)
    master.read_state()
    for number, slave in enumerate(master.slaves):
        if slave.state != state:
            raise RuntimeError(
                f"Slave {number} did not reach {name}: state 0x{slave.state:02X}, "
                f"AL status code 0x{slave.al_status:04X}"
            )
    print(f"All slaves in {name}")


def exchange_cycles(master, cycles):
    """Exchange all-zero process data for `cycles` cycles. Returns the number of wkc mismatches."""
    bad_wkc = 0
    for _ in range(cycles):
        for slave in master.slaves:
            slave.output = bytes(len(slave.output))
        master.send_processdata()
        wkc = master.receive_processdata(2000)
        if wkc != master.expected_wkc:
            bad_wkc += 1
        time.sleep(CYCLE_TIME_S)
    return bad_wkc


def read_without_state_change(master):
    """Default: only read. Works if the slaves are already in PRE-OP or higher."""
    master.read_state()
    for number, slave in enumerate(master.slaves):
        state = slave.state & 0x0F
        print(f"Slave {number}: {slave.name}")
        print(f"  AL state: 0x{slave.state:02X} ({AL_STATE_NAMES.get(state, 'unknown')}), "
              f"AL status code 0x{slave.al_status:04X}")
        if state < 0x02:
            print("  Mailbox (SDO) is not active in this state, so no values can be read "
                  "without a state change. Nothing was changed. Use --state preop or op.")
            continue
        print_axes(slave)
        print_pdos("RxPDOs assigned (0x1C12)", list_pdos(slave, RXPDO_ASSIGN))
        print_pdos("TxPDOs assigned (0x1C13)", list_pdos(slave, TXPDO_ASSIGN))


def read_with_state_change(master, target):
    print_states(master, "Before:")
    request_state(master, pysoem.PREOP_STATE, "PRE-OP")

    pdos = {}
    for number, slave in enumerate(master.slaves):
        pdos[number] = (list_pdos(slave, RXPDO_ASSIGN), list_pdos(slave, TXPDO_ASSIGN))

    if target == "preop":
        for number, slave in enumerate(master.slaves):
            print(f"Slave {number}: {slave.name} (values over SDO, slave left in PRE-OP)")
            print_axes(slave)
            print_pdos("RxPDOs assigned (0x1C12)", pdos[number][0])
            print_pdos("TxPDOs assigned (0x1C13)", pdos[number][1])
        return

    # OP: config_map() builds the process image and moves the slaves to SAFE-OP, then OP is
    # requested. All outputs stay zero, so every controlword is 0x0000 and no drive is enabled.
    # SAFE-OP is not used for reading: on the TI CiA402 firmware the input values are only valid
    # in OP (see README).
    io_size = master.config_map()
    print(f"Process image size: {io_size} bytes")
    if master.state_check(pysoem.SAFEOP_STATE, 2_000_000) != pysoem.SAFEOP_STATE:
        master.read_state()
        details = ", ".join(f"slave {n}: state 0x{s.state:02X}, AL status code 0x{s.al_status:04X}"
                            for n, s in enumerate(master.slaves))
        raise RuntimeError("Slaves did not reach SAFE-OP (" + details + ")")
    print("All slaves in SAFE-OP")

    # Same order as go_operational() in read-motor-drive-status.py, which reached OP on this
    # slave: one process data cycle, then the OP request, then cycles until OP is reported.
    master.state = pysoem.OP_STATE
    exchange_cycles(master, 1)
    master.write_state()
    bad_wkc_total = 0
    for cycle in range(1, OP_REQUEST_CYCLES + 1):
        bad_wkc_total += exchange_cycles(master, 1)
        master.read_state()
        if all(slave.state == pysoem.OP_STATE for slave in master.slaves):
            print(f"OP reached after {cycle} cycles")
            break
        if cycle % 100 == 0:
            details = ", ".join(
                f"slave {n}: state 0x{s.state:02X}, AL status code 0x{s.al_status:04X}"
                for n, s in enumerate(master.slaves))
            print(f"  waiting for OP, cycle {cycle}: {details}, "
                  f"working counter mismatches so far {bad_wkc_total}")
    else:
        details = ", ".join(f"slave {n}: state 0x{s.state:02X}, AL status code 0x{s.al_status:04X}"
                            for n, s in enumerate(master.slaves))
        raise RuntimeError(f"Slaves did not reach OP after {OP_REQUEST_CYCLES} cycles ({details}), "
                           f"working counter mismatches {bad_wkc_total}, expected {master.expected_wkc}")
    print("All slaves in OP (outputs zero)")
    bad_wkc = exchange_cycles(master, OP_SETTLE_CYCLES)
    if bad_wkc:
        print(f"Warning: working counter differed from expected {master.expected_wkc} in "
              f"{bad_wkc} of {OP_SETTLE_CYCLES} cycles")

    for number, slave in enumerate(master.slaves):
        print(f"Slave {number}: {slave.name} (PDO = process image, SDO = mailbox read)")
        print_axes(slave, object_locations(pdos[number][1]))
        print_pdos("RxPDOs assigned (0x1C12)", pdos[number][0])
        print_pdos("TxPDOs assigned (0x1C13)", pdos[number][1])
    print_states(master, "After:")


def main():
    parser = argparse.ArgumentParser(
        description="Read PDO assignment and the statusword, mode display, position, velocity, "
                    "and torque of three axes. Without --state nothing is changed.")
    parser.add_argument("interface", nargs="?", default="eth0")
    parser.add_argument("--state", choices=("preop", "op"), default=None,
                        help="preop: request PRE-OP and read over SDO. op: request PRE-OP, build "
                             "the process image, go through SAFE-OP to OP with all outputs zero "
                             "(no drive is enabled), and read the inputs from the process image "
                             "and over SDO. The script does not request a state after reading "
                             "and never requests INIT; it stops exchanging process data when "
                             "it exits, so the slave may leave OP on its own.")
    args = parser.parse_args()

    master = pysoem.Master()
    try:
        master.open(args.interface)
        count = master.config_init()
        if count == 0:
            raise RuntimeError("No EtherCAT slaves discovered")

        if args.state is None:
            read_without_state_change(master)
        else:
            read_with_state_change(master, args.state)
    finally:
        master.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", type(e).__name__, e)
        sys.exit(1)
