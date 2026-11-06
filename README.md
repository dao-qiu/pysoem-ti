# read-motor-drive-status.py

EtherCAT master script (pysoem) that reads the CiA 402 status of a drive, optionally runs the
enable sequence, and optionally runs a cyclic synchronous velocity (CSV) move.

Tested setup (from the runs recorded while developing this script):

- Master: TI AM62L EVM (`am62lxx-evm`), Python 3.12.9, run as root, interface `eth0`
- Slave: `TI EtherCAT Toolkit CiA402 for AM243X.R5`, Vendor ID `0x0000059D`, Product ID `0x54490024`
- pysoem wheel: `pysoem-1.1.13` for `cp312` / `manylinux2014_aarch64`

Other slaves, firmware, or pysoem versions have not been tested.

## Safety

`--enable` energizes the drive, and `--velocity` can turn the motor.

- Run the first tests with the motor unloaded and mechanically free.
- Keep a hardware emergency stop within reach. The limit checks in this script run in Python on
  Linux, which is not real time. They are not a safety function.
- Velocity units are those of the drive's objects `0x60FF` / `0x606C`. The units of the tested
  firmware have not been established (see [Open points](#open-points)). Start with small values.

## Installation on the target

If the board has no internet access, download the wheel on a host PC and copy it over:

```
# host
pip download pysoem --only-binary=:all: --platform manylinux2014_aarch64 \
    --python-version 3.12 --no-deps -d wheels
scp wheels/pysoem-*.whl root@<board-ip>:~/ethercat/

# board
cd ~/ethercat
python3 -m pip install --no-index --no-deps pysoem-*.whl
python3 -c "import pysoem; print(pysoem.__version__)"
```

Copy the script to the board and run it as root (raw socket access is required):

```
scp read-motor-drive-status.py root@<board-ip>:~/ethercat/
```

## Usage

```
python3 read-motor-drive-status.py [interface] [--enable] [--hold SECONDS] [--mode N]
        [--velocity V --max-velocity LIMIT --accel A --motion-time SECONDS]
```

| Argument | Description |
|---|---|
| `interface` | Network interface connected to the EtherCAT slave. Default `eth0`. |
| `--enable` | Run the CiA 402 enable sequence (shutdown, switch on, enable operation). The motor may be energized. |
| `--hold SECONDS` | Time to stay enabled when no `--velocity` is given. Default `3`. |
| `--mode N` | Value written to Modes of operation `0x6060`. The script waits until `0x6061` shows it. Example: `9` = cyclic synchronous velocity. |
| `--velocity V` | Target velocity written to `0x60FF`. Signed. Requires `--enable`, `--mode`, `--max-velocity`, `--accel`, and `--motion-time`. |
| `--max-velocity LIMIT` | Positive limit. `abs(--velocity)` must not exceed it. The move aborts if the measured velocity exceeds 1.5 x this value. |
| `--accel A` | Velocity ramp in units/s², used for both ramp up and ramp down. |
| `--motion-time SECONDS` | Time to hold the target velocity between the ramps. |

The script rejects `--velocity` at argument parsing if any required companion argument is missing,
if a limit, ramp, or time is not positive, or if `abs(--velocity)` exceeds `--max-velocity`.

## Example configurations

### 1. Read-only status (drive stays disabled)

```
python3 read-motor-drive-status.py eth0
```

Moves the slave INIT -> PRE-OP -> SAFE-OP -> OP, prints the statusword (over SDO and over PDO),
the PDO mapping, and the telemetry. The controlword stays `0x0000`, so the drive is not enabled.
The script ends with the drive disabled and the slave in INIT.

Output recorded on the tested slave:

```
Slave AL state after PRE-OP request: 0x02, AL status code 0x0000
Drive statusword (SDO): 0x0000 (Not ready to switch on)
...
Slave AL state: 0x04 (SAFE-OP)
Slave AL state: 0x08 (OP)
Drive (PDO): statusword 0x1240 (Switch on disabled), mode display 0 (No mode), position 0, velocity 0
```

### 2. Enable the drive and hold

```
python3 read-motor-drive-status.py eth0 --enable --hold 3
```

Runs the enable sequence and holds for 3 s. Recorded statuswords on the tested slave:
`0x1221` (ready to switch on), `0x1233` (switched on), `0x1237` (operation enabled).

### 3. Select a mode of operation and enable

```
python3 read-motor-drive-status.py eth0 --enable --mode 9 --hold 3
```

Writes `0x6060 = 9` and waits for `0x6061` to show 9 before enabling.

### 4. Velocity move

```
python3 read-motor-drive-status.py eth0 --enable --mode 9 --velocity 100 \
    --max-velocity 200 --accel 50 --motion-time 2
```

Ramps the target from 0 to 100 at 50 units/s² (2 s), holds 100 for 2 s, ramps back to 0, holds 0
for 0.5 s, and disables the drive. This exact command was run on the tested slave. The measured
velocity followed the target and the statusword stayed `0x1237` throughout.

### 5. Reverse direction

```
python3 read-motor-drive-status.py eth0 --enable --mode 9 --velocity -100 \
    --max-velocity 200 --accel 50 --motion-time 2
```

A negative `--velocity` writes a negative `0x60FF`. This form has not been run on the hardware, and
which shaft direction a negative value produces has not been established.

### 6. Conservative first test

```
python3 read-motor-drive-status.py eth0 --enable --mode 9 --velocity 10 \
    --max-velocity 20 --accel 5 --motion-time 2
```

Small values for a first run on a new setup. The numbers are placeholders and are not derived from
any drive limit.

## What the script does

1. Opens the interface, runs `config_init()`, and prints the slave name, vendor ID, and product ID.
2. Requests PRE-OP and prints the AL state and AL status code. Without this, SDO access fails with
   `WkcError` (the slave is in INIT after `config_init()` on the tested setup).
3. Reads statusword `0x6041` over SDO. On failure it prints the exception type and the SDO abort
   code if available.
4. Reads the PDO mapping from `0x1C12` (RxPDO) and `0x1C13` (TxPDO) and computes the byte offsets of
   the CiA 402 objects. The layout is not hard-coded.
5. Runs `config_map()`, goes to SAFE-OP and OP, and exchanges process data every 2 ms.
6. Optionally writes the mode (`--mode`), runs the enable sequence (`--enable`), and runs the
   velocity move (`--velocity`).
7. On exit, including after an error: sets the target velocity to 0, sends shutdown, then sends
   controlword `0x0000` until the statusword shows Switch on disabled, prints the final statusword,
   and sets the slave to INIT.

Only the first axis is controlled (`0x6040`..`0x60FF`). The tested slave also maps a second set of
objects at `0x68xx`. Its output bytes stay at zero.

### Abort conditions during a velocity move

Checked every cycle. The script stops the move, then runs the shutdown above:

- the fault bit (`0x0008`) is set in the statusword
- the drive leaves Operation enabled
- the measured velocity exceeds 1.5 x `--max-velocity` (`ACTUAL_VELOCITY_ABORT_FACTOR` in the script)

These abort paths have not been triggered on the hardware.

### Warnings

`Warning: working counter N, expected M` means a process data frame was not fully processed by the
slave. It was not observed in the recorded runs.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `pip` retries and `No matching distribution found` | The board has no internet access. Use the offline wheel install above. |
| `WkcError` on the SDO read | The slave is in INIT. The script requests PRE-OP to avoid this. |
| `AttributeError: module 'pysoem' has no attribute ...` | A constant or attribute differs in the installed pysoem version. `PREOP_STATE` (not `PRE_OP_STATE`) is the name on pysoem 1.1.13. |
| `Slave did not reach PRE-OP / SAFE-OP / OP` | The message includes the AL status code. Look it up in the slave's documentation. |
| `Timeout with controlword ...` | The drive did not reach the expected CiA 402 state. The message shows the last statusword. |
| `Target velocity 0x60FF is not in the RxPDO mapping` | The slave's PDO mapping lacks the object. Check the printed RxPDO list. |
| Permission or socket errors | Run as root and make sure the interface is up (`ip link set eth0 up`). |

## Open points

- The units of `0x60FF` / `0x606C` on the tested firmware are not established. In the recorded
  `--velocity 100` run the position counter advanced about 188,000 per second while the velocity
  read 100. The factor group objects (`0x608F`, `0x6091`, `0x6092`) have not been read.
- The measured velocity equalled the target at every sample, including during the ramps. Whether a
  physical motor was turning, or the firmware reports the commanded value, has not been established.
- Which sign of `0x60FF` is clockwise has not been established.
- The 1.5 x overspeed factor is an arbitrary choice, not a drive value.
