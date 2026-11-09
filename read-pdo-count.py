import sys

import pysoem

RXPDO_ASSIGN = 0x1C12
TXPDO_ASSIGN = 0x1C13


def sdo_uint(slave, index, subindex):
    return int.from_bytes(slave.sdo_read(index, subindex), "little")


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


def main():
    interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    master = pysoem.Master()
    try:
        master.open(interface)
        count = master.config_init()
        if count == 0:
            raise RuntimeError("No EtherCAT slaves discovered")

        for number, slave in enumerate(master.slaves):
            # SDO access needs PRE-OP (the slave is in INIT after config_init on this setup)
            slave.state = pysoem.PREOP_STATE
            slave.write_state()
            master.state_check(pysoem.PREOP_STATE, 2_000_000)
            master.read_state()
            if slave.state != pysoem.PREOP_STATE:
                raise RuntimeError(
                    f"Slave {number} did not reach PRE-OP: state 0x{slave.state:02X}, "
                    f"AL status code 0x{slave.al_status:04X}"
                )

            rx = list_pdos(slave, RXPDO_ASSIGN)
            tx = list_pdos(slave, TXPDO_ASSIGN)
            print(f"Slave {number}: {slave.name}")
            print_pdos("RxPDOs assigned (0x1C12)", rx)
            print_pdos("TxPDOs assigned (0x1C13)", tx)
    finally:
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
