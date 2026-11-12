import argparse
import hmac
import importlib.util
import json
import os
import secrets
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import pysoem

# Rebuild the snapshot for the web page every this many control cycles
SNAPSHOT_EVERY_CYCLES = 25
# Longest time step used when ramping, so a slow cycle never produces a big jump
MAX_RAMP_DT_S = 0.05
# After a stop request, keep Enable Operation while sending target velocity 0 for this long before
# dropping the controlword. The slave only passes 0x60FF on to the motor core while Enable Operation
# is set (ESL_cia402Demo.c), so dropping the controlword in the same cycle can leave the old target
# in place. Drive.disable() in read-motor-drive-status.py holds 0.2 s the same way.
STOP_HOLD_S = 0.2


def load_drive_module():
    """Import read-motor-drive-status.py (the hyphens prevent a normal import)."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "read-motor-drive-status.py")
    spec = importlib.util.spec_from_file_location("read_motor_drive_status", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class AxisCommand:
    def __init__(self):
        self.enabled = False
        self.speed = 0
        self.direction = 1


class Controller:
    """Cyclic control of the axes. Only the thread running run() may call pysoem.

    Other threads use set_enabled / change_speed / toggle_direction / stop_all / touch /
    snapshot, which only touch state guarded by self.lock.
    """

    def __init__(self, rds, drive, axes, max_velocity, step, accel, heartbeat_s):
        self.rds = rds
        self.drive = drive
        self.axes = axes
        self.max_velocity = max_velocity
        self.step_size = step
        self.accel = accel
        self.heartbeat_s = heartbeat_s
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.commands = [AxisCommand() for _ in axes]
        self.errors = [None for _ in axes]
        self.notice = ""
        self.estop = False
        self.fatal = None
        self.last_seen = time.monotonic()
        self._snapshot = {}
        self._active = [False for _ in axes]
        self._target = [0.0 for _ in axes]
        self._last_step = time.monotonic()
        self._hold_until = 0.0
        self._cycles = 0

    # Called from web threads

    def touch(self):
        with self.lock:
            self.last_seen = time.monotonic()

    def set_enabled(self, index, on):
        with self.lock:
            if on:
                self.errors[index] = None
            self.commands[index].enabled = bool(on)

    def change_speed(self, index, delta):
        with self.lock:
            cmd = self.commands[index]
            cmd.speed = max(0, min(self.max_velocity, cmd.speed + delta))

    def toggle_direction(self, index):
        with self.lock:
            self.commands[index].direction *= -1

    def stop_all(self):
        """Zero every target velocity on the next cycle (no ramp), then disable."""
        with self.lock:
            self.estop = True
            for cmd in self.commands:
                cmd.enabled = False

    def snapshot(self):
        with self.lock:
            return dict(self._snapshot)

    # Called from the control thread only

    def _build_snapshot(self):
        axes = []
        for i, axis in enumerate(self.axes):
            cmd = self.commands[i]
            axes.append({
                "name": axis.name,
                "on": cmd.enabled,
                "active": self._active[i],
                "speed": cmd.speed,
                "direction": cmd.direction,
                "target": axis.target_velocity,
                "statusword": f"0x{axis.statusword:04X}",
                "state": self.rds.decode_statusword(axis.statusword),
                "velocity": axis.velocity(),
                "position": axis.position(),
                "error": self.errors[i],
            })
        return {"axes": axes, "notice": self.notice, "fatal": self.fatal,
                "max_velocity": self.max_velocity, "step": self.step_size}

    def _fault(self, index, message):
        """Stop the axis at once and ask the other axes to ramp down."""
        axis = self.axes[index]
        axis.target_velocity = 0
        self._target[index] = 0.0
        axis.controlword = self.rds.CW_DISABLE_VOLTAGE
        self._active[index] = False
        with self.lock:
            self.errors[index] = message
            self.notice = f"{axis.name}: {message}"
            for cmd in self.commands:
                cmd.enabled = False
            self.commands[index].speed = 0
        print(f"{axis.name}: {message}")

    def _ramp(self, index, goal, dt):
        current = self._target[index]
        delta = self.accel * dt
        current = min(goal, current + delta) if goal > current else max(goal, current - delta)
        self._target[index] = current
        self.axes[index].target_velocity = round(current)

    def step(self):
        now = time.monotonic()
        dt = min(now - self._last_step, MAX_RAMP_DT_S)
        self._last_step = now
        with self.lock:
            if now - self.last_seen > self.heartbeat_s:
                if any(cmd.enabled for cmd in self.commands):
                    self.notice = "Heartbeat lost, stopping"
                    print(self.notice)
                for cmd in self.commands:
                    cmd.enabled = False
            wanted = [(cmd.enabled, cmd.direction * cmd.speed) for cmd in self.commands]
            estop, self.estop = self.estop, False
        if estop:
            self._hold_until = now + STOP_HOLD_S
            for i, axis in enumerate(self.axes):
                self._target[i] = 0.0
                axis.target_velocity = 0

        for i, axis in enumerate(self.axes):
            enabled, goal = wanted[i]
            if enabled and not self._active[i]:
                try:
                    axis.enable()
                except RuntimeError as e:
                    self._fault(i, str(e))
                    continue
                self._active[i] = True
            self._ramp(i, goal if enabled else 0, dt)
            if (not enabled and self._active[i] and self._target[i] == 0
                    and now >= self._hold_until):
                axis.controlword = self.rds.CW_DISABLE_VOLTAGE
                self._active[i] = False

        self.drive.cycle()

        for i, axis in enumerate(self.axes):
            if self._active[i]:
                try:
                    axis.check_safe(self.max_velocity)
                except RuntimeError as e:
                    self._fault(i, str(e))

        self._cycles += 1
        if self._cycles % SNAPSHOT_EVERY_CYCLES == 0:
            snapshot = self._build_snapshot()
            with self.lock:
                self._snapshot = snapshot

    def run(self, on_exit=None):
        try:
            while not self.stop_event.is_set():
                self.step()
        except Exception as e:
            self.fatal = f"{type(e).__name__}: {e}"
            print("Control loop stopped:", self.fatal)
        finally:
            try:
                self.drive.disable()
            except Exception as e:
                print("Could not disable drive:", type(e).__name__, e)
            with self.lock:
                self._snapshot = self._build_snapshot() if self.fatal is None else {
                    "axes": [], "fatal": self.fatal, "notice": "", "max_velocity": self.max_velocity,
                    "step": self.step_size}
            if on_exit is not None:
                on_exit()


PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Motor control</title>
<style>
body { font-family: sans-serif; margin: 1em; }
.axis { border: 1px solid #888; padding: 0.6em; margin-bottom: 1em; max-width: 34em; }
button { font-size: 1.1em; padding: 0.5em 0.9em; margin: 0.2em; }
.on { background: #9d9; } .off { background: #ddd; }
#stop { background: #e66; font-weight: bold; }
.err { color: #b00; } pre { margin: 0.3em 0; }
</style></head><body>
<h2>Motor control</h2>
<div id="axes"></div>
<button id="stop" onclick="send(-1, 'stop_all')">STOP ALL</button>
<div id="msg" class="err"></div>
<p>Velocity and position shown are the values reported by the slave (commanded, not measured).</p>
<script>
const token = new URLSearchParams(location.search).get('token') || '';
const hdr = {'X-Token': token, 'Content-Type': 'application/json'};
async function send(axis, action) {
  try { await fetch('/api/cmd', {method: 'POST', headers: hdr, body: JSON.stringify({axis, action})}); }
  catch (e) { document.getElementById('msg').textContent = 'Connection lost'; }
}
function render(s) {
  const root = document.getElementById('axes');
  root.innerHTML = '';
  (s.axes || []).forEach((a, i) => {
    const d = document.createElement('div');
    d.className = 'axis';
    d.innerHTML = '<b>' + a.name + '</b> ' + (a.active ? 'ENABLED' : 'disabled') +
      '<pre>set speed: ' + (a.direction < 0 ? '-' : '+') + a.speed + ' (max ' + s.max_velocity +
      ')\\ntarget: ' + a.target + '\\nstatus: ' + a.statusword + ' ' + a.state +
      '\\nreported velocity: ' + a.velocity + '\\nreported position: ' + a.position + '</pre>' +
      (a.error ? '<div class="err">' + a.error + '</div>' : '');
    const mk = (label, action, cls) => {
      const b = document.createElement('button');
      b.textContent = label; if (cls) b.className = cls;
      b.onclick = () => send(i, action); d.appendChild(b);
    };
    mk(a.on ? 'Turn OFF' : 'Turn ON', 'toggle', a.on ? 'on' : 'off');
    mk('Slower (-' + s.step + ')', 'slower');
    mk('Faster (+' + s.step + ')', 'faster');
    mk('Reverse', 'reverse');
    root.appendChild(d);
  });
  document.getElementById('msg').textContent = s.fatal || s.notice || '';
}
async function poll() {
  try {
    const r = await fetch('/api/status', {headers: hdr});
    if (r.status === 403) { document.getElementById('msg').textContent = 'Bad token'; }
    else { render(await r.json()); }
  } catch (e) { document.getElementById('msg').textContent = 'Connection lost'; }
}
setInterval(poll, 300); poll();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    controller = None
    token = ""

    def log_message(self, fmt, *args):
        pass

    def _authorized(self):
        query = parse_qs(urlparse(self.path).query)
        given = self.headers.get("X-Token") or (query.get("token") or [""])[0]
        return hmac.compare_digest(given.encode(), self.token.encode())

    def _send(self, code, body, content_type="application/json"):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if not self._authorized():
            self._send(403, json.dumps({"error": "bad token"}))
            return
        path = urlparse(self.path).path
        if path == "/":
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif path == "/api/status":
            if self.controller is None:
                self._send(503, json.dumps({"error": "starting"}))
                return
            self.controller.touch()
            self._send(200, json.dumps(self.controller.snapshot()))
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        if not self._authorized():
            self._send(403, json.dumps({"error": "bad token"}))
            return
        if urlparse(self.path).path != "/api/cmd" or self.controller is None:
            self._send(404, json.dumps({"error": "not found"}))
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            request = json.loads(self.rfile.read(min(length, 1024)) or b"{}")
            action = request["action"]
            index = int(request.get("axis", -1))
        except (ValueError, KeyError, TypeError):
            self._send(400, json.dumps({"error": "bad request"}))
            return
        ctl = self.controller
        ctl.touch()
        if action == "stop_all":
            ctl.stop_all()
        elif 0 <= index < len(ctl.axes):
            if action == "toggle":
                ctl.set_enabled(index, not ctl.commands[index].enabled)
            elif action == "faster":
                ctl.change_speed(index, ctl.step_size)
            elif action == "slower":
                ctl.change_speed(index, -ctl.step_size)
            elif action == "reverse":
                ctl.toggle_direction(index)
            else:
                self._send(400, json.dumps({"error": "unknown action"}))
                return
        else:
            self._send(400, json.dumps({"error": "bad axis"}))
            return
        self._send(200, json.dumps({"ok": True}))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Web UI to enable/disable the motors, change velocity and direction. "
                    "Runs cyclic synchronous velocity control on the EtherCAT slave.")
    parser.add_argument("interface", nargs="?", default="eth0",
                        help="EtherCAT interface (default eth0)")
    parser.add_argument("--bind", required=True,
                        help="IP address to serve on, for example the address of eth1. Not "
                             "defaulted so the page is never served on the EtherCAT port by accident")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--token", default=None,
                        help="secret required by every request; a random one is generated and "
                             "printed if not given")
    parser.add_argument("--max-velocity", type=int, required=True,
                        help="velocity limit enforced by the server; also the overspeed abort "
                             "reference (1.5 x this value)")
    parser.add_argument("--step", type=int, required=True,
                        help="velocity change per Faster / Slower button press")
    parser.add_argument("--accel", type=int, required=True,
                        help="velocity ramp in units/s^2, also used when reversing and stopping")
    parser.add_argument("--heartbeat", type=float, default=3.0,
                        help="seconds without a request from the page after which the motors are "
                             "ramped down and disabled (default 3)")
    parser.add_argument("--mode", type=int, default=9,
                        help="Modes of operation written to every axis (default 9 = CSV)")
    parser.add_argument("--axes", default="1,2",
                        help="comma-separated axes to control, 1 and/or 2 (default 1,2)")
    args = parser.parse_args()
    if args.max_velocity <= 0 or args.step <= 0 or args.accel <= 0 or args.heartbeat <= 0:
        parser.error("--max-velocity, --step, --accel and --heartbeat must be positive")
    if args.step > args.max_velocity:
        parser.error("--step must not exceed --max-velocity")
    try:
        args.axis_numbers = sorted({int(a) for a in args.axes.split(",")})
    except ValueError:
        parser.error("--axes must be a comma-separated list of 1 and/or 2")
    if not args.axis_numbers or not set(args.axis_numbers) <= {1, 2}:
        parser.error("--axes must be a comma-separated list of 1 and/or 2")
    return args


def setup_drive(rds, master, args):
    """Bring the slave to OP and select the mode. Mirrors main() in read-motor-drive-status.py."""
    count = master.config_init()
    if count == 0:
        raise RuntimeError("No EtherCAT slaves discovered")
    print(f"Discovered {count} slave(s)")
    slave = master.slaves[0]
    print("Slave name:", slave.name)
    master.read_state()
    if not rds.goto_state(master, slave, pysoem.PREOP_STATE, "PRE-OP"):
        raise RuntimeError("Slave did not reach PRE-OP")
    rx_entries = rds.read_pdo_map(slave, 0x1C12)
    tx_entries = rds.read_pdo_map(slave, 0x1C13)
    master.config_map()
    if not master.state_check(pysoem.SAFEOP_STATE, 2_000_000) == pysoem.SAFEOP_STATE:
        master.read_state()
        raise RuntimeError(f"Slave did not reach SAFE-OP: state 0x{slave.state:02X}, "
                           f"AL status code 0x{slave.al_status:04X}")
    drive = rds.Drive(master, slave)
    axes = []
    for number in args.axis_numbers:
        base = 0 if number == 1 else rds.AXIS2_OFFSET
        axis = drive.add_axis(f"Axis {number}", rx_entries, tx_entries, base)
        if axis.target_vel_off is None:
            raise RuntimeError(f"{axis.name}: target velocity 0x{axis.target_vel_index:04X} is "
                               "not in the RxPDO mapping")
        axes.append(axis)
    rds.go_operational(master, slave, drive)
    drive.run_cycles(0.5)
    for axis in axes:
        axis.select_mode(args.mode)
    drive.run_cycles(0.2)
    return drive, axes


def main():
    args = parse_args()
    rds = load_drive_module()
    token = args.token or secrets.token_urlsafe(8)
    Handler.token = token

    # Bind first so a bad address or busy port fails before the motors are touched
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.daemon_threads = True

    def on_term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)

    master = pysoem.Master()
    drive = None
    controller = None
    thread = None
    try:
        master.open(args.interface)
        drive, axes = setup_drive(rds, master, args)
        controller = Controller(rds, drive, axes, args.max_velocity, args.step, args.accel,
                                args.heartbeat)
        Handler.controller = controller
        thread = threading.Thread(target=controller.run, kwargs={"on_exit": server.shutdown})
        thread.start()
        print(f"Serving on http://{args.bind}:{args.port}/?token={token}")
        print("Press Ctrl+C to stop; the motors are ramped down and disabled on exit.")
        server.serve_forever()
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        if controller is not None:
            controller.stop_event.set()
            controller.stop_all()
        if thread is not None:
            thread.join()
        elif drive is not None:
            try:
                drive.disable()
            except Exception as e:
                print("Could not disable drive:", type(e).__name__, e)
        server.server_close()
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
