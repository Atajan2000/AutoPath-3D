"""One Webots + AutoPath launch session, owned and cleaned up by the caller (WSL + Windows Webots).

Used by test_e2e_webots.py and as a startup-only reliability check:

    source ros2/env.sh
    python3 ros2/autopath3d_ros/test/webots_session.py --startup-check 5 [--out DIR]

A session
  * refuses to start while another Webots instance is running (one simulator at a time),
  * starts `ros2 launch autopath3d_ros warehouse.launch.py ...` in its own process group,
  * detects readiness from the launch output (both extern controllers report
    "Controller successfully connected"), not from sleeps,
  * classifies the start: READY, WEBOTS_EXITED (exit code recorded), CONTROLLER_TIMEOUT,
  * on close stops only its own processes: SIGINT to the launch, then its process group, then the
    Windows Webots processes whose command line contains this session's temporary world file,
  * keeps the full launch log in its output directory.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import threading
import time

READY_MARK = "Controller successfully connected to robot in Webots simulation"
CONTROLLERS = ("Crazyflie", "AutoPathSceneManager")
DIED_RE = re.compile(r"\[webots-1\]: process has died \[pid (\d+), exit code (-?\d+)")
STARTED_RE = re.compile(r"\[webots-1\]: process started with pid \[(\d+)\]")


def windows_webots_processes(tag: str | None = None) -> list[tuple[int, str, str]]:
    """(pid, name, command line) of Windows webots.exe / webots-bin.exe, optionally filtered by tag."""
    ps = ("Get-CimInstance Win32_Process -Filter \"Name='webots.exe' OR Name='webots-bin.exe'\" | "
          "ForEach-Object { \"$($_.ProcessId)`t$($_.Name)`t$($_.CommandLine)\" }")
    try:
        out = subprocess.run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", ps],
                             capture_output=True, text=True, timeout=60).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    rows = []
    for line in out.splitlines():
        parts = line.strip().split("\t", 2)
        if len(parts) == 3 and parts[0].isdigit() and (tag is None or tag in parts[2]):
            rows.append((int(parts[0]), parts[1], parts[2]))
    return rows


class WebotsSession:
    def __init__(self, out_dir: str, launch_args: list[str], port: int, env: dict | None = None,
                 ready_timeout_s: float = 120.0):
        self.out_dir = out_dir
        self.launch_args = launch_args
        self.port = port
        self.env = dict(os.environ if env is None else env, AUTOPATH_WEBOTS_PORT=str(port))
        self.ready_timeout_s = ready_timeout_s
        self.log_path = os.path.join(out_dir, "launch.log")
        self.proc: subprocess.Popen | None = None
        self.lines: list[str] = []
        self.connected: set[str] = set()
        self.webots_wsl_pid: int | None = None
        self.webots_exit: int | None = None
        self.world_tag: str | None = None
        self.record: dict = {"port": port, "launch_args": launch_args}
        self._lock = threading.Lock()
        self._reader: threading.Thread | None = None

    # ------------------------------------------------------------------ start
    def start(self) -> str:
        """Start and wait for readiness. Returns READY, WEBOTS_EXITED, LAUNCH_EXITED,
        CONTROLLER_TIMEOUT or WEBOTS_ALREADY_RUNNING (nothing started)."""
        os.makedirs(self.out_dir, exist_ok=True)
        others = windows_webots_processes()
        if others:
            self.record.update(startup="WEBOTS_ALREADY_RUNNING", other_webots=[(p, n) for p, n, _ in others])
            return "WEBOTS_ALREADY_RUNNING"
        cmd = ["ros2", "launch", "autopath3d_ros", "warehouse.launch.py", *self.launch_args]
        self.record["cmd"] = cmd
        self.t0 = time.monotonic()
        self.record["start_wall"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self._log = open(self.log_path, "w")
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                     text=True, bufsize=1, env=self.env, start_new_session=True)
        self.pgid = os.getpgid(self.proc.pid)
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        status = "CONTROLLER_TIMEOUT"
        while time.monotonic() - self.t0 < self.ready_timeout_s:
            with self._lock:
                if self.webots_exit is not None:
                    status = "WEBOTS_EXITED"
                    break
                if self.connected >= set(CONTROLLERS):
                    status = "READY"
                    break
            if self.proc.poll() is not None:
                status = "LAUNCH_EXITED"
                break
            time.sleep(0.2)
        self.record.update(startup=status, t_startup_s=round(time.monotonic() - self.t0, 2),
                           webots_exit_code=self.webots_exit, connected=sorted(self.connected),
                           world_tag=self.world_tag)
        return status

    def _read(self):
        for line in self.proc.stdout:
            self._log.write(line)
            self._log.flush()
            with self._lock:
                self.lines.append(line)
                m = STARTED_RE.search(line)
                if m:
                    self.webots_wsl_pid = int(m.group(1))
                    self.world_tag = self._world_tag(self.webots_wsl_pid)
                m = DIED_RE.search(line)
                if m:
                    self.webots_exit = int(m.group(2))
                if READY_MARK in line:
                    for c in CONTROLLERS:
                        if f"[{c}]" in line or f"controller_{c}" in line:
                            self.connected.add(c)

    @staticmethod
    def _world_tag(pid: int) -> str | None:
        """Unique temporary world file name of this session's Webots (from the interop cmdline)."""
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                args = f.read().decode(errors="replace").split("\0")
        except OSError:
            return None
        for a in args:
            m = re.search(r"(tmp[^\\/]*_world_with_URDF_robot\.wbt)", a)
            if m:
                return m.group(1)
        return None

    # ------------------------------------------------------------------ run / stop
    def wait(self, timeout_s: float) -> int | None:
        """Wait for the launch to exit by itself (scenario runs end on their own)."""
        try:
            return self.proc.wait(timeout_s)
        except subprocess.TimeoutExpired:
            return None

    def close(self) -> dict:
        """Stop this session's processes only and verify nothing of it is left."""
        steps = []
        if self.proc is not None and self.proc.poll() is None:
            os.kill(self.proc.pid, signal.SIGINT)
            steps.append("sigint_launch")
            try:
                self.proc.wait(45)
            except subprocess.TimeoutExpired:
                pass
        for sig, name in ((signal.SIGTERM, "sigterm_group"), (signal.SIGKILL, "sigkill_group")):
            if self._group_alive():
                try:
                    os.killpg(self.pgid, sig)
                    steps.append(name)
                except ProcessLookupError:
                    break
                time.sleep(2.0)
        if self.proc is not None:
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                pass
        mine = windows_webots_processes(self.world_tag) if self.world_tag else []
        for pid, name, _ in mine:          # only Webots started with this session's world file
            subprocess.run(["taskkill.exe", "/PID", str(pid), "/T", "/F"], capture_output=True)
            steps.append(f"taskkill_{name}_{pid}")
        if self._reader is not None:
            self._reader.join(5)
        if hasattr(self, "_log"):
            self._log.close()
        left_win = windows_webots_processes(self.world_tag) if self.world_tag else []
        self.record.update(launch_returncode=None if self.proc is None else self.proc.returncode,
                           webots_exit_code=self.webots_exit, cleanup_steps=steps,
                           leftover_wsl_group=self._group_alive(), leftover_windows=[(p, n) for p, n, _ in left_win],
                           log=self.log_path)
        with open(os.path.join(self.out_dir, "session.json"), "w") as f:
            json.dump(self.record, f, indent=1)
        return self.record

    def _group_alive(self) -> bool:
        if not hasattr(self, "pgid"):
            return False
        try:
            os.killpg(self.pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def tail(self, n: int = 25, keys=("RESULT", "SUPERVISOR_MODE", "failed", "Error", "ERROR", "refused")) -> str:
        with self._lock:
            sel = [ln.rstrip() for ln in self.lines if any(k in ln for k in keys)]
        return "\n".join(sel[-n:])


def wait_no_webots(timeout_s: float = 90.0) -> bool:
    """One simulator at a time: wait until no Windows Webots process is left (returns False on timeout)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if not windows_webots_processes():
            return True
        time.sleep(2.0)
    return False


def startup_check(n: int, out: str, port0: int, launch_args: list[str] | None = None) -> list[dict]:
    """Bounded startup-only check: Webots + both controllers (default: no planning stack, no
    mission); stops each session once READY. Every attempt is reported."""
    res = []
    for i in range(n):
        if not wait_no_webots():
            res.append({"attempt": i, "startup": "WEBOTS_ALREADY_RUNNING"})
            print(json.dumps(res[-1]), flush=True)
            break
        s = WebotsSession(os.path.join(out, f"startup_{i:02d}"), launch_args or ["stack:=false"], port0 + i)
        status = s.start()
        rec = s.close()
        rec["attempt"] = i
        res.append(rec)
        print(json.dumps({k: rec.get(k) for k in ("attempt", "port", "startup", "t_startup_s", "webots_exit_code",
                                                  "connected", "cleanup_steps", "leftover_wsl_group",
                                                  "leftover_windows")}), flush=True)
        if status == "WEBOTS_ALREADY_RUNNING":
            break
    ok = sum(r.get("startup") == "READY" for r in res)
    print(f"startup check: {ok}/{len(res)} READY", flush=True)
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--startup-check", type=int, required=True, metavar="N")
    ap.add_argument("--out", default=os.path.expanduser("~/autopath_e2e/startup_" + time.strftime("%Y%m%d_%H%M%S")))
    ap.add_argument("--port0", type=int, default=1236, help="ports port0..port0+N-1 (keep below 1250 on this host: "
                                                           "1250-1349 is a Windows excluded range)")
    ap.add_argument("launch_args", nargs="*", help="launch arguments (default: stack:=false)")
    a = ap.parse_args()
    r = startup_check(a.startup_check, a.out, a.port0, a.launch_args)
    raise SystemExit(0 if r and all(x.get("startup") == "READY" for x in r) else 1)
