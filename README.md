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
        [--velocity2 V2 --max-velocity2 LIMIT2] [--continuous]
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
| `--motion-time SECONDS` | Time to hold the target velocity between the ramps. Required with `--velocity` / `--velocity2` unless `--continuous` is given. |
| `--continuous` | Hold the target velocity until Ctrl+C instead of for `--motion-time`. Mutually exclusive with `--motion-time`; requires `--velocity` or `--velocity2`. On Ctrl+C the script ramps down at `--accel`, then runs the normal shutdown. The safety checks stay active while holding. The arrow keys and space bar change axis 1 while holding (see [Keyboard control](#keyboard-control-in---continuous-mode)). |
| `--velocity2 V2` | Target velocity written to `0x68FF` (second axis). Signed. Requires `--enable`, `--mode`, `--max-velocity2`, `--accel`, and `--motion-time`. Can be used with or without `--velocity`. Without `--velocity`, the first axis is not touched: its controlword stays `0x0000`. |
| `--max-velocity2 LIMIT2` | Same meaning as `--max-velocity`, for the second axis. |

`--mode`, `--accel`, and `--motion-time` are shared by both axes. Both axes ramp in the same loop
and hold for the same time. Each axis has its own limit and its own safety checks.

The script rejects `--velocity` / `--velocity2` at argument parsing if any required companion argument is missing,
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

### 6. Velocity move on the second axis only, or on both

```
python3 read-motor-drive-status.py eth0 --enable --mode 9 --velocity2 100 \
    --max-velocity2 200 --accel 50 --motion-time 2

python3 read-motor-drive-status.py eth0 --enable --mode 9 --velocity 100 --max-velocity 200 \
    --velocity2 -100 --max-velocity2 200 --accel 50 --motion-time 2
```

The second axis uses the objects at the first axis index + `0x800` (`0x6840`, `0x6860`, `0x68FF`,
`0x6841`, `0x6861`, `0x6864`, `0x686C`). Neither form has been run on hardware.

### 7. Continuous motion

```
python3 read-motor-drive-status.py eth0 --enable --mode 9 --velocity 100 \
    --max-velocity 200 --accel 50 --continuous
```

Ramps to 100 and holds until Ctrl+C. This form has not been run on hardware. If the SSH session
drops without delivering Ctrl+C, the script's behavior was not tested; the drive is not stopped
by anything other than this script or the slave's own handling.

### 8. Conservative first test

```
python3 read-motor-drive-status.py eth0 --enable --mode 9 --velocity 10 \
    --max-velocity 20 --accel 5 --motion-time 2
```

Small values for a first run on a new setup. The numbers are placeholders and are not derived from
any drive limit.

### Keyboard control in --continuous mode

While `--continuous` holds the velocity, the arrow keys and the space bar change **axis 1 only**.
Axis 2 is not affected. This needs a terminal on stdin (an SSH session works) and `--velocity` given
for axis 1. If axis 1 is not moving or stdin is not a terminal, the script prints a message and the
keys are disabled.

| Key | Action |
|---|---|
| Up | Increase the speed by 5 (up to `--max-velocity`). |
| Down | Decrease the speed by 5 (not below 0). |
| Left | Direction CCW (negative velocity). |
| Right | Direction CW (positive velocity). |
| Space | Stop axis 1: the target velocity is set to 0 immediately, without a ramp. The drive stays enabled, so Up starts it again in the current direction. |

The target moves to the new goal at `--accel`, so a direction change ramps through 0 and never steps
from +v to -v. Space is the exception: it zeroes the target at once. It does not disable the drive,
because the slave only passes the target on while the controlword is Enable Operation; Ctrl+C still
ramps down and disables. How quickly the motor itself stops after the zero is set by the motor
firmware's own speed ramp, which I have not measured. The starting direction comes from the sign of
`--velocity` (negative = CCW). The step of 5 is `KEY_VELOCITY_STEP` in the script.

Which sign of `0x60FF` is clockwise on the tested slave has not been established (see
[Open points](#open-points)). The script assumes positive = CW (`DIRECTION_CW` / `DIRECTION_CCW`
in the script). If the shaft turns the other way, swap those two values. This has been tested with
a simulated terminal and drive only, not on the hardware.

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

The first axis (`0x6040`..`0x60FF`) is controlled unless only `--velocity2` is given. The second
axis (`0x6840`..`0x68FF`) is only controlled when `--velocity2` is given. An axis that is not
controlled has its output bytes left at zero. The `0x68xx`
mapping was seen in the recorded `logs.log` RxPDO/TxPDO lists; that the `+0x800` offset applies to
every second-axis object has not been verified against the slave's documentation.

### Abort conditions during a velocity move

Checked every cycle. The script stops the move, then runs the shutdown above:

- the fault bit (`0x0008`) is set in the statusword
- the drive leaves Operation enabled
- the measured velocity exceeds 1.5 x `--max-velocity` (`ACTUAL_VELOCITY_ABORT_FACTOR` in the script)

These abort paths have not been triggered on the hardware.

### Warnings

`Warning: working counter N, expected M` means a process data frame was not fully processed by the
slave. It was not observed in the recorded runs.

## Web UI (motor-web.py)

`motor-web.py` runs the same EtherCAT control as `read-motor-drive-status.py` and serves a web page
with buttons to turn each motor on or off, change the velocity, and reverse the direction. It
imports `Axis` and `Drive` from `read-motor-drive-status.py`, so keep both files in the same
directory. It uses only the Python standard library besides `pysoem`.

This was tested only against a simulated drive on a PC. It has not been run against the real slave.
Run the first tests with the motor unloaded, and keep a hardware emergency stop within reach (see
[Safety](#safety)).

### Network setup

- EtherCAT uses raw frames on `eth0`. Serve the page on a different interface. In the setup used
  while developing it, the board's `eth1` (`172.168.1.82`) is connected to the PC (`172.168.1.1`).
- The PC may have an HTTP proxy configured. If the browser shows a proxy error, add the board's
  address to the browser's "No proxy for" list (Firefox: Settings, General, Network Settings). This
  fixed the connection to a test server in that setup. See `notes.md`.

### Start

On the board, as root:

```
python3 motor-web.py eth0 --bind 172.168.1.82 --max-velocity 20 --step 5 --accel 5
```

The values are placeholders for a conservative first test and are not derived from any drive limit.
After the slave reaches OP, the script prints:

```
Serving on http://172.168.1.82:8080/?token=<token>
```

Open that full URL in the browser. The `<token>` is random and changes on each start. Without the
token, every request is rejected (the page shows `{"error": "bad token"}`). To use a fixed token, add
`--token mysecret` and open `http://172.168.1.82:8080/?token=mysecret`. The token is sent unencrypted
over HTTP.

Stop the script with Ctrl+C. It ramps the motors down and disables them before leaving OP.

### Options

```
python3 motor-web.py [interface] --bind ADDRESS --max-velocity LIMIT --step STEP --accel A
        [--port PORT] [--token TOKEN] [--heartbeat SECONDS] [--mode N] [--axes LIST]
```

| Argument | Description |
|---|---|
| `interface` | Network interface connected to the EtherCAT slave. Default `eth0`. |
| `--bind ADDRESS` | Required. IP address to serve the page on, for example the address of `eth1`. There is no default so that the page is not served on the EtherCAT port by accident. |
| `--port PORT` | TCP port. Default `8080`. |
| `--token TOKEN` | Secret every request must carry. A random one is generated and printed if not given. |
| `--max-velocity LIMIT` | Required. Positive limit enforced by the server. The set speed is clamped to `0..LIMIT`. The script also aborts an axis if its measured velocity exceeds 1.5 x this value. |
| `--step STEP` | Required. Velocity change for each Faster / Slower press. Must not exceed `--max-velocity`. |
| `--accel A` | Required. Velocity ramp in units/s², used when turning on, changing speed, reversing, and turning off. |
| `--heartbeat SECONDS` | If no request reaches the server for this long, the motors ramp down and are disabled. Default `3`. |
| `--mode N` | Modes of operation written to every controlled axis. Default `9` (cyclic synchronous velocity). |
| `--axes LIST` | Axes to control: `1`, `2`, or `1,2`. Default `1,2`. Axis 2 uses the objects at the axis 1 index + `0x800`. |

Velocity units are those of the drive's objects `0x60FF` / `0x606C` (see [Open points](#open-points)).

### Buttons

For each axis:

| Button | Action |
|---|---|
| Turn ON / Turn OFF | ON runs the CiA 402 enable sequence and then ramps to the set speed. OFF ramps to 0 at `--accel`, then disables the axis. |
| Faster / Slower | Changes the set speed by `--step`. The speed stays between 0 and `--max-velocity`. |
| Reverse | Flips the direction. The velocity ramps through 0 at `--accel`. It is never a step from +v to -v. |

The set speed is shown as a signed value (the sign is the direction). The set speed starts at 0, so
press Faster after Turn ON to move the motor.

`STOP ALL` sets every target velocity to 0 on the next cycle, without a ramp, and then disables all
axes.

### Safety behavior

- Speed limits are enforced on the server. The page cannot exceed `--max-velocity`.
- If the page stops sending requests (closed tab, lost link), the motors ramp down and are disabled
  after `--heartbeat` seconds. The page polls about every 300 ms while it is open.
- Per axis, a fault bit, leaving Operation enabled, or a measured velocity above 1.5 x
  `--max-velocity` stops that axis, turns all axes off, and shows the error on the page. Turning the
  axis on again clears the error.
- If the control loop itself stops (for example an exception), the motors are disabled and the page
  shows the error.
- Plain HTTP with a token is not strong protection. Anyone on the same network who gets the token can
  drive the motors. Bind only to the interface you need.
- The velocity and position shown on the page are the values the slave reports. They are the commanded
  values, not measurements (see `notes.md`).
- What the drive does if the script is killed or frames stop was not established.

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
