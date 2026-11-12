# Why the velocity read back over EtherCAT (0x606C) is not a measured velocity

Labels follow the project rules: unlabeled statements were read in the files named. Everything else
is labeled `[ASSUMPTION]`, `[UNCERTAIN]`, `[GUESS]`, or `[GENERAL KNOWLEDGE]`.

## Summary

On the code read, `0x606C` (Velocity actual value) carries the motor core's commanded, ramp-limited
speed setpoint (`gSpdSetPoint`), not a measured speed. Reading it faster or more carefully from the
master cannot make it accurate. The firmware has to publish the measured speed.

## Observation that started this

In the recorded `--velocity 100` run (see `README.md`, Open points), the measured velocity equalled
the target at every sample, including during the ramps.

## Data path, from the sources read

Paths are under `/home/a0500327/ti`. `ISDK` = `ind_comms_sdk_am243x_2025_00_00_08`, `M` =
`motor_control_sdk_am243x_09_01_00_06/examples/tidep_01032_dual_motor_drive/single_chip_servo/am243x-lp`.

1. Master writes `0x60FF` (axis 1) or `0x68FF` (axis 2). The axis offset is `OBD_CIA402_AXIS_OFFSET
   0x0800` (`ecSlvApiDef_CiA402.h:334`), so axis 2 is axis 1 + 0x800.
2. EtherCAT slave, `ISDK/examples/industrial_comms/ethercat_subdevice_demo/device_profiles/402_cia/ESL_cia402Demo.c:762-778`
   (build with `ENABLE_TIDEP01032_REF_DESIGN_CONFIG`, defined in the 2025 `r5fss1-0_freertos` makefile):
   - `gCurTargetVelocity[axis] = targetVelocity` (from `0x60FF`)
   - `actualVelocity = gCurActualVelocity[axis]`
   - `actualVelocity` is written to `0x606C`
3. Motor core, `M/r5fss0-0_nortos/single_chip_servo.c:902-909` (M1, index 0) and
   `M/r5fss0-1_nortos/single_chip_servo.c:1260-1267` (M2, index 1), under `BUILDLEVEL ==
   CLOSED_LOOP_SPEED`:
   ```
   gSpdTarget = gCurTargetVelocity[n];
   if (gSpdSetPoint < gSpdTarget) gSpdSetPoint += MAX_SPD_CHANGE;
   else if (gSpdSetPoint > gSpdTarget) gSpdSetPoint -= MAX_SPD_CHANGE;
   // update the gCurActualVelocity[n] based on the gSpdSetPoint
   gCurActualVelocity[n] = (uint32_t)gSpdSetPoint;
   ```
   `settings.h:52` sets `BUILDLEVEL` to `CLOSED_LOOP_SPEED` in both cores.
4. The measured speed is computed separately, `single_chip_servo.c:832` (M1) and `:1221` (M2):
   `speed = (angleDelta / 360.0) / ISR_PRD_IN_MINUTES`. It is used as feedback in the speed PI
   (`gIqRef = arm_pi_f32(&gPiSpd, gSpdSetPoint - speed)`, line 911). It is never written to
   `gCurActualVelocity`.

So the chain `0x60FF` -> `gCurTargetVelocity` -> `gSpdSetPoint` -> `gCurActualVelocity` -> `0x606C`
contains no measurement.

## Why the readback equals the target

`MAX_SPD_CHANGE` is 0.12 (`settings.h:68`) and `ISR_PRD_IN_SECONDS` is 0.00002, a 50 kHz ISR
(`settings.h:74`). If `gSpdSetPoint` is updated once per ISR, the setpoint slews at up to about
6000 units/s. `[ASSUMPTION]` The master ramp in `read-motor-drive-status.py` (for example
50 units/s^2) is far slower, so the setpoint tracks the target within one sample and the
readback equals the target.

The `#else` branch of the slave (`ESL_cia402Demo.c:779-801`, used when the ref-design define is not
set) writes the target straight to `0x606C`, which would also read "actual = target". `[UNCERTAIN]`
I have not established which firmware build is flashed on the tested slave.

## Other problems in the same path

- `gCurActualVelocity[n] = (uint32_t)gSpdSetPoint` casts a float to unsigned. `[GENERAL KNOWLEDGE]`
  Converting a negative float to an unsigned type is undefined behavior in C. The target is a
  signed `int32_t`, so negative velocities may read wrong. Not tested.
- The unit of `gSpdSetPoint`, `gSpdTarget`, and `speed` in this code is RPM (`speed` is computed per
  minute). The slave reports the number as `0x606C` without conversion. I have not read what the
  factor group objects `0x608F`, `0x6091`, `0x6092`, `0x6096` report.
- Position (`0x6064`) cannot cross-check velocity: in `ESL_cia402Demo.c:962-965` the slave
  integrates position from the velocity value (`DRIVE_GEAR_RELATION 0.0010922` x cycle time).
  `[UNCERTAIN]` That path may not be active in the ref-design build. The ~188,000 counts/s observed at
  velocity 100 does not match my rough calculation from `DRIVE_GEAR_RELATION`, and I cannot explain the gap.
- `gCurActualVelocity` is defined in section `.gEtherCatCia402` in the slave and in both motor
  cores. I did not read the linker files, so I have not established that all three use one shared
  address. `r5fss0-1` defines the arrays twice (lines 743-744 and 1113-1114); I did not reconcile
  that.

## Does reading the actual velocity involve the encoder?

Yes. The measured `speed` is derived from an encoder angle:

- `single_chip_servo.c` includes the EnDat position-sense driver
  (`position_sense/endat/include/endat_drv.h`).
- In the FOC ISR, `localEnDatGetSingleMulti(&mechTheta, &multiTurn, 0)` reads the mechanical angle
  and multi-turn position from the encoder (line ~798).
- `angleDelta = mechTheta - gLastMechTheta` (line 822), with rollover handling at +/-50 degrees
  (lines 824-828), then `speed = (angleDelta / 360.0) / ISR_PRD_IN_MINUTES` (line 832).
- Only if `USE_PURE_OPEN_LOOP` or `USE_OPEN_LOOP_WITH_SDDF` is defined, `myMechTheta` (a
  generated angle) is used instead of the encoder angle. I saw `USE_OPEN_LOOP_WITH_SDDF`
  commented out at `settings.h:105`. I did not check `USE_PURE_OPEN_LOOP`.
- `[UNCERTAIN]` Whether an EnDat encoder is actually attached to the tested motors is not
  established by anything I read; the code path assumes one.

`[ASSUMPTION]` Each ISR computes `speed` from a single angle delta, so it is probably quantized
and noisy and would need filtering before being published. I have not looked at its raw values.

## What is needed for an accurate readback

1. Firmware (both motor cores, outside this repo): write the measured, filtered `speed` to
   `gCurActualVelocity[n]` as a signed `int32_t` (rounded), instead of `gSpdSetPoint`.
2. Confirm the arrays share one address across the slave and both motor cores (linker files).
3. Establish the unit seen on the wire (RPM in this code) and, if wanted, the factor group objects.
4. Cross-check against an independent reference (debugger watch on `speed`, scope, or external
   encoder reading), and test with the shaft loaded or blocked: an unloaded run cannot show a
   difference between setpoint and measurement.
5. Optional on the master: dump `0x608F`, `0x6090`, `0x6091`, `0x6092`, `0x6096` over SDO for both
   axes. Not implemented.

No change to `read-motor-drive-status.py` is sufficient on its own.

# Why the position read back over EtherCAT (0x6064) is not a measured position

## Summary

On the code read, `0x6064` (Position actual value) is dead-reckoned in the EtherCAT slave by
integrating the velocity readback. It is not read from the encoder. Since that velocity is the
ramped setpoint (see above), the position is the integral of the commanded speed.

## Data path, from the sources read

`ISDK` = `ind_comms_sdk_am243x_2025_00_00_08`; `M` as above.

1. Slave, `ISDK/.../402_cia/ESL_cia402Demo.c:962-981`:
   - `actualVelocity` is read from the velocity actual value object (`0x606C`).
   - `positionActualValue += actualVelocity * DRIVE_GEAR_RELATION * cycleTime`
     (`DRIVE_GEAR_RELATION 0.0010922`, line 162).
   - The result is clamped to `int32_t` and written to `0x6064`.
2. The mode `switch` at lines 1006-1018 selects `EC_SLV_APP_CSP`, `EC_SLV_APP_CSV`, or
   `EC_SLV_APP_CST`. I did not read the name of the function that contains the integration at
   lines 962-981, or whether it runs in every mode. `[UNCERTAIN]`
3. In CSP mode (`ESL_cia402Demo.c:835-882`) the slave reads `0x6064`, computes
   `(targetPosition - actualPosition) / incFactor`, and writes that to `0x606C`. This is also not
   a measurement.
4. The motor cores do not publish a position to the slave. In `M/r5fss0-0_nortos/single_chip_servo.c`
   and `M/r5fss0-1_nortos/single_chip_servo.c`, I found only `gCurTargetVelocity[]` and
   `gCurActualVelocity[]` in section `.gEtherCatCia402`. I grepped for position-related names and
   found no shared position array.
5. `gTxData` (section `.gTxDataSection`) carries the position target and mode of operation, and is
   used by `CLOSED_LOOP_CIA402` (`single_chip_servo.c:922-969`). `settings.h:52` selects
   `CLOSED_LOOP_SPEED`, so this path is not the one built. `[ASSUMPTION]` the flashed firmware uses the
   `settings.h` value as read; I have not established that.

## What the encoder already provides in the motor cores

- `localEnDatGetSingleMulti()` (`single_chip_servo.c:668-697`) returns `mechTheta` in degrees
  (`pos / priv->step * 360`, from a 25-bit position field) and `multiTurn` (revolution count).
- It is called every FOC interrupt (line 798 for M1; line 786 in `r5fss0-1` for the same core
  index 0 read, and channel index 2 elsewhere in `r5fss0-1` at lines 1056/1187). I did not read
  how encoder channels map to motors beyond `gEndat_multi_ch_mask` using `ENDAT_MULTI_CH0` for M1
  and `ENDAT_MULTI_CH2` for M2 (`single_chip_servo.c:1515-1530`).
- Zero reference: the motor core uses `gMechAngleOffset` for the electrical angle and
  `gStartingMultiTurn` / `gLastMultiTurn` for multiturn handling (lines 930-935, 1124-1135).
  I did not trace these in detail.

## What it takes to read the real position

1. Add a shared array, for example `gCurActualPosition[AXES_NUMBER]`, in section `.gEtherCatCia402`
   in the slave and in both motor cores.
2. In each motor core, write the encoder position (`multiTurn` and `mechTheta` combined into one
   signed `int32_t` of counts) into the array at the axis index (0 for M1, 1 for M2).
3. In the slave, replace the integration at `ESL_cia402Demo.c:962-981` with a read of that array
   into `0x6064`.
4. Decide units and wrap: counts per revolution come from `priv->step`, whose value I did not
   trace. A 32-bit `0x6064` limits the number of revolutions. Decide whether `0x6064` is absolute
   encoder position or relative to power-up.
5. Confirm the arrays share one address across the three images (linker files not read).
6. Verify against an independent source: with the drive disabled, turn the shaft by hand and
   compare `0x6064` with the encoder value in a debugger. The synthetic position does not move in
   that case.

The master script needs no change; it already reads `0x6064` from the PDO.

## Open point

`[UNCERTAIN]` The ~188,000 counts per second observed at velocity 100 (README, Open points) is
consistent with integration of the commanded velocity, but I could not reproduce that number from
`DRIVE_GEAR_RELATION` and the cycle time. The unit of `cycleTime` and the cycle time used on the
tested slave were not read.

# Reaching a web server on the AM62Lx from the PC browser

## Result

A test web server on the AM62Lx was reached from the PC's Firefox after adding the board's address to
Firefox's "No proxy for" list (Settings > General > Network Settings). Reported by the user; the
exact Firefox proxy mode selected was not recorded.

## Setup used

- PC and board are connected over the board's `eth1` (user statement). On the PC, `enp3s0` is
  `172.168.1.1/24`; the board is `172.168.1.82` (user confirmed the address). `eth0` on the board
  stays dedicated to EtherCAT.
- On the board (Python 3.12.9, observed in the `Server:` header of the response):
  ```
  mkdir -p /tmp/webtest && cd /tmp/webtest
  echo "hello from am62l" > index.html
  python3 -m http.server 8080 --bind 172.168.1.82
  ```
  `--bind 172.168.1.82` serves only on `eth1`. The server runs in the foreground and prints nothing
  until a request arrives; "Serving HTTP on ..." is the expected only output.

## What went wrong first and why

- Browser error: "The proxy could not connect to the destination in time."
- Cause: the PC has `http_proxy`/`HTTP_PROXY=http://webproxy.ext.ti.com:80`, with `no_proxy` listing
  only `localhost,127.0.0.1,ti.com`. The GNOME proxy mode is `'auto'` with an empty ignore list.
  The proxy cannot reach the private `172.168.1.x` link, so the request timed out there.
- A `curl` without `--noproxy '*'` went to the proxy (`172.16.188.71:80`) and timed out. With
  `--noproxy '*'` it reached the board and returned `HTTP/1.0 200 OK` with body
  `hello from am62l`. This showed the board's server and link were fine.
- `[ASSUMPTION]` The browser was using the same proxy; the exemption in Firefox fixed it, which is
  consistent with that.

## For later

- The motor control page will need the same proxy exemption. Command-line tools need
  `--noproxy '*'` or `no_proxy` extended with `172.168.1.82`.
- An alternative that needs no browser setting is an SSH port forward
  (`ssh -L 8081:172.168.1.82:8080 root@172.168.1.82`, then `http://localhost:8081`). Not tested.
- Port 8080 had nothing listening before the test server was started. The motor web UI does not exist
  yet.
