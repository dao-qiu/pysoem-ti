import sys

import pysoem

RXPDO_ASSIGN = 0x1C12
TXPDO_ASSIGN = 0x1C13


def sdo_uint(slave, index, subindex):
    return int.from_bytes(slave.sdo_read(index, subindex), "little")


def list_pdos(slave, assign_index):
    """Return {pdo_index: number_of_mapped_entries} for the PDOs assigned in assign_index."""
    pdos = {}
    for i in range(1, sdo_uint(slave, assign_index, 0) + 1):
        pdo_index = sdo_uint(slave, assign_index, i)
        pdos[pdo_index] = sdo_uint(slave, pdo_index, 0)
    return pdos


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
            print(f"  RxPDOs assigned (0x1C12): {len(rx)}")
            for pdo_index, entries in rx.items():
                print(f"    0x{pdo_index:04X}: {entries} mapped objects")
            print(f"  TxPDOs assigned (0x1C13): {len(tx)}")
            for pdo_index, entries in tx.items():
                print(f"    0x{pdo_index:04X}: {entries} mapped objects")
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
