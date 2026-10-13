"""COM-port Linux console transport, with the same interface as BoardSSH.

This is a serial shell, not SSH tunneled over serial. Commands are framed and
serialized; the controller runs in a background subshell with run-owned output
and an exit-status file. No Ctrl-C byte is sent to an unidentified foreground
program. Controller stop still uses the runner's PID-checked SIGINT/TERM path.
"""
import base64
import hashlib
import os
from pathlib import Path
import re
import shlex
import threading
import time
import uuid


class SerialBoard:
    def __init__(self, args, serial_port=None):
        self.lock = threading.RLock()
        self.closed = False
        self.desynchronized = False
        if serial_port is None:
            import serial
            serial_port = serial.Serial(args.serial_port, args.serial_baud,
                                        timeout=.1, write_timeout=5)
        self.serial = serial_port
        self.transport_metadata = dict(kind="serial_console", port=args.serial_port,
                                       baud=args.serial_baud, ssh=False)
        try:
            self._login(getattr(args, "ssh_user", None) or "root",
                        getattr(args, "serial_password_env", None))
            self.run("command -v base64 >/dev/null && command -v dd >/dev/null && command -v sha256sum >/dev/null", timeout=10)
        except BaseException:
            self.close()
            raise

    def _write(self, data):
        if self.serial.write(data) != len(data):
            raise RuntimeError("serial short write; console synchronization unknown")
        self.serial.flush()

    def _login(self, username, password_env):
        # Wake only; never reset a board or signal a foreground application.
        self._write(b"\n")
        deadline, response = time.monotonic() + 2, b""
        sent_user = sent_password = False
        while time.monotonic() < deadline:
            response += self.serial.read(4096)
            if re.search(rb"(?:login|username):\s*$", response, re.I):
                if sent_user:
                    raise RuntimeError("serial login rejected")
                self._write((username + "\n").encode())
                response, sent_user = b"", True
                deadline = time.monotonic() + 5
            elif re.search(rb"password:\s*$", response, re.I):
                password = os.environ.get(password_env, "") if password_env else ""
                if sent_password or not password:
                    raise RuntimeError("COM console requires login: use --serial-password-env NAME or log in with a terminal first")
                self._write((password + "\n").encode())
                response, sent_password = b"", True
                deadline = time.monotonic() + 5
            elif re.search(rb"[#$]\s*$", response):
                return
        # A shell with an unusual prompt can still pass the framed probe.

    def run(self, command, check=True, timeout=30):
        with self.lock:
            self._frame_complete = False
            try:
                return self._run_framed(command, check=check, timeout=timeout)
            except BaseException:
                # Ctrl-C between send and final status is as uncertain as an
                # I/O timeout. Never feed cleanup into that unknown foreground.
                if not self._frame_complete:
                    self.desynchronized = True
                raise

    def _run_framed(self, command, check=True, timeout=30):
        with self.lock:
            if self.closed:
                raise RuntimeError("serial console is closed")
            if self.desynchronized:
                raise RuntimeError("serial console desynchronized; refusing further shell input")
            token = uuid.uuid4().hex
            begin, end = "PS_BEGIN_" + token, "PS_END_" + token
            encoded = base64.b64encode(command.encode()).decode("ascii")
            # Splitting literal markers prevents terminal command echo from
            # being mistaken for the output frames.
            script = (f"printf '\\n%s%s\\n' PS_BEGIN_ {token}; "
                      f"printf %s {shlex.quote(encoded)} | base64 -d | sh; "
                      f"ps_status=$?; printf '\\n%s%s:%s\\n' PS_END_ {token} \"$ps_status\"\n")
            try:
                self._write(script.encode())
            except Exception:
                self.desynchronized = True
                raise
            deadline, pending, lines, started = time.monotonic() + timeout, b"", [], False
            while time.monotonic() < deadline:
                try:
                    pending += self.serial.read(4096)
                except Exception:
                    self.desynchronized = True
                    raise
                while b"\n" in pending:
                    raw, pending = pending.split(b"\n", 1)
                    line = raw.rstrip(b"\r").decode("utf-8", errors="replace")
                    if line == begin:
                        started = True
                        continue
                    if not started:
                        continue
                    if line.startswith(end + ":"):
                        code = line[len(end) + 1:]
                        if not code.isdigit():
                            self.desynchronized = True
                            raise RuntimeError("invalid serial shell exit frame")
                        self._frame_complete = True
                        output = "\n".join(lines).strip()
                        if check and int(code):
                            raise RuntimeError(f"serial board command failed ({code}): {command}\n{output}")
                        return output
                    lines.append(line)
            # Never issue another ordinary command into an unknown foreground
            # state after a timeout. Existing evidence remains on the board.
            self.desynchronized = True
            raise TimeoutError("serial command timed out; command/exit status unknown")

    def put_file(self, local, remote):
        data = Path(local).read_bytes()
        self.run(f"test ! -e {shlex.quote(remote)} && : > {shlex.quote(remote)}")
        for offset in range(0, len(data), 768):
            encoded = base64.b64encode(data[offset:offset + 768]).decode("ascii")
            self.run(f"printf %s {shlex.quote(encoded)} | base64 -d >> {shlex.quote(remote)}")
        digest = self.run(f"sha256sum {shlex.quote(remote)}").split()[0]
        if digest != hashlib.sha256(data).hexdigest():
            raise RuntimeError("serial upload SHA-256 mismatch")

    def get_file(self, remote, local):
        size = int(self.run(f"wc -c < {shlex.quote(remote)}"))
        digest = self.run(f"sha256sum {shlex.quote(remote)}").split()[0]
        chunk_size = 24576
        hasher = hashlib.sha256()
        # File is deliberately kept even when a later transfer fails.
        with Path(local).open("xb") as stream:
            for index, offset in enumerate(range(0, size, chunk_size)):
                result = self.run(f"dd if={shlex.quote(remote)} bs={chunk_size} skip={index} count=1 2>/dev/null | base64", timeout=30)
                chunk = base64.b64decode("".join(result.split()), validate=True)
                if len(chunk) != min(chunk_size, size - offset):
                    raise RuntimeError("serial download truncated or file changed")
                stream.write(chunk)
                stream.flush()
                hasher.update(chunk)
        if hasher.hexdigest() != digest:
            raise RuntimeError("serial download SHA-256 mismatch; partial evidence retained")

    def exec_long(self, command):
        # command contains the runner's unique run directory and PID file.
        # Separate shell prevents a controller's exec from replacing console.
        token = uuid.uuid4().hex
        prefix = "/tmp/phase_serial_" + token
        log, status = prefix + ".log", prefix + ".exit"
        worker = f"( {command} ); ps_rc=$?; printf '%s\\n' \"$ps_rc\" > {shlex.quote(status)}"
        self.run(f"test ! -e {shlex.quote(log)} && "
                 f"nohup sh -c {shlex.quote(worker)} > {shlex.quote(log)} 2>&1 < /dev/null &")
        return SerialOutput(self, log, status)

    def close(self):
        with self.lock:
            self.closed = True
            self.serial.close()


class SerialOutput:
    def __init__(self, board, log, status):
        self.board, self.log, self.status_path = board, log, status
        self.channel = self
        self.exit_code = None

    def __iter__(self):
        offset, pending = 0, b""
        while True:
            # 3 KiB bounds one console transaction to ~0.4 s at 115200 baud.
            data = self.board.run(f"dd if={shlex.quote(self.log)} bs=1 skip={offset} count=3072 2>/dev/null | base64", timeout=10)
            chunk = base64.b64decode("".join(data.split()), validate=True)
            offset += len(chunk)
            pending += chunk
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                yield line.rstrip(b"\r").decode("utf-8", errors="replace") + "\n"
            if self.exit_status_ready():
                # Worker writes status only after stdout closes. Drain final
                # bytes after discovering the status, not before.
                size = int(self.board.run(f"wc -c < {shlex.quote(self.log)}"))
                if offset >= size:
                    if pending:
                        yield pending.decode("utf-8", errors="replace")
                    return
            time.sleep(.05)

    def exit_status_ready(self):
        if self.exit_code is None:
            result = self.board.run(f"if test -f {shlex.quote(self.status_path)}; then cat {shlex.quote(self.status_path)}; else echo RUNNING; fi", timeout=10)
            if result != "RUNNING":
                if not re.fullmatch(r"\d+", result):
                    raise RuntimeError("serial controller exit status unavailable")
                self.exit_code = int(result)
        return self.exit_code is not None

    def recv_exit_status(self):
        return self.exit_code if self.exit_code is not None else -1
