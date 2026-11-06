import pysoem
master = pysoem.Master()
try:
    master.open("eth0") # This is the ethernet port used for ethercat communication
 
    count = master.config_init()
    print(f"Discovered {count} EtherCAT slave(s)")
 
    for i, slave in enumerate(master.slaves):
        print(f"Slave {i}: {slave.name}")
        print(f"  Vendor ID: 0x{slave.man:x}")
        print(f"  Product ID: 0x{slave.id:x}")
finally:
    master.close()
