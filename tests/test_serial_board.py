import base64
import hashlib
from pathlib import Path
import re
import sys
import tempfile
import types
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from serial_board import SerialBoard, SerialOutput


class FakeConsole:
    def __init__(self):
        self.buffer = bytearray()
        self.writes = []
        self.commands = []
        self.closed = False
        self.reply = lambda command: ("", 0)

    def write(self, data):
        self.writes.append(data)
        if data == b"\n":
            self.buffer.extend(b"root@board:~# ")
        else:
            script = data.decode()
            token = re.search(r"PS_BEGIN_ ([0-9a-f]+)", script)[1]
            encoded = re.search(r"printf %s ([A-Za-z0-9+/=]+) \| base64 -d \| sh", script)[1]
            command = base64.b64decode(encoded).decode()
            self.commands.append(command)
            output, code = self.reply(command)
            self.buffer.extend(data + f"\r\nPS_BEGIN_{token}\r\n{output}\r\nPS_END_{token}:{code}\r\nroot# ".encode())
        return len(data)

    def flush(self): pass

    def read(self, size):
        # Fragment boundaries including UTF-8 and frame lines.
        result = bytes(self.buffer[:min(size, 7)])
        del self.buffer[:len(result)]
        return result

    def close(self): self.closed = True


class SerialTests(unittest.TestCase):
    def setUp(self):
        self.port = FakeConsole()
        args = types.SimpleNamespace(serial_port="TEST_CONSOLE", serial_baud=115200,
                                     serial_password_env=None, ssh_user="root")
        self.board = SerialBoard(args, serial_port=self.port)

    def tearDown(self): self.board.close()

    def test_framing_ignores_echo_and_keeps_unicode(self):
        self.port.reply = lambda command: ("[State 2] 中文\nline2", 0)
        self.assertEqual(self.board.run("printf ignored"), "[State 2] 中文\nline2")
        self.assertNotIn(b"\x03", b"".join(self.port.writes))

    def test_nonzero_status_is_not_success(self):
        self.port.reply = lambda command: ("failed", 2)
        with self.assertRaisesRegex(RuntimeError, "failed \\(2\\)"):
            self.board.run("false")
        self.assertEqual(self.board.run("false", check=False), "failed")

    def test_timeout_poisoning_prevents_late_response_misattribution(self):
        self.port.read = lambda size: b""
        with self.assertRaises(TimeoutError): self.board.run("sleep 100", timeout=.001)
        count = len(self.port.writes)
        with self.assertRaisesRegex(RuntimeError, "desynchronized"):
            self.board.run("echo second")
        self.assertEqual(count, len(self.port.writes))

    def test_keyboard_interrupt_poisons_only_incomplete_frame(self):
        def interrupt(size):
            raise KeyboardInterrupt()
        self.port.read = interrupt
        with self.assertRaises(KeyboardInterrupt): self.board.run("command")
        count = len(self.port.writes)
        with self.assertRaisesRegex(RuntimeError, "desynchronized"):
            self.board.run("cleanup")
        self.assertEqual(count, len(self.port.writes))

    def test_chunked_download_and_hash(self):
        data = bytes(range(256)) * 120
        def reply(command):
            if command.startswith("wc -c"): return str(len(data)), 0
            if command.startswith("sha256sum"): return hashlib.sha256(data).hexdigest() + "  file", 0
            if command.startswith("dd "):
                index = int(re.search(r"skip=(\d+)", command)[1])
                return base64.b64encode(data[index * 24576:(index + 1) * 24576]).decode(), 0
            return "", 0
        self.port.reply = reply
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "data.csv"
            self.board.get_file("/remote/data.csv", target)
            self.assertEqual(data, target.read_bytes())
            with self.assertRaises(FileExistsError): self.board.get_file("/remote/data.csv", target)

    def test_transfer_failure_retains_written_chunk(self):
        data = b"x" * 24576
        def reply(command):
            if command.startswith("wc -c"): return "25000", 0
            if command.startswith("sha256sum"): return "0" * 64, 0
            if "skip=0" in command: return base64.b64encode(data).decode(), 0
            return "", 0
        self.port.reply = reply
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "data.csv"
            with self.assertRaisesRegex(RuntimeError, "truncated"):
                self.board.get_file("/remote/data.csv", target)
            self.assertEqual(data, target.read_bytes())

    def test_controller_background_uses_original_command(self):
        result = self.board.exec_long("cd /run1 && exec /board/program /run1/config.txt")
        self.assertIsInstance(result, SerialOutput)
        self.assertIn("nohup sh -c", self.port.commands[-1])
        self.assertIn("exec /board/program /run1/config.txt", self.port.commands[-1])
        self.assertIsNone(result.exit_code)

    def test_output_drains_last_bytes_after_exit_file(self):
        text = "[DataLogger] 日志文件: Data_test.csv\n[State 2] ready\nlast"
        data = text.encode()
        class Board:
            def run(self, command, **kwargs):
                if command.startswith("dd "):
                    offset = int(re.search(r"skip=(\d+)", command)[1])
                    return base64.b64encode(data[offset:offset + 3072]).decode()
                if command.startswith("wc"): return str(len(data))
                return "0"
        output = SerialOutput(Board(), "/tmp/log", "/tmp/exit")
        self.assertEqual("".join(output), text)
        self.assertEqual(output.recv_exit_status(), 0)


if __name__ == "__main__": unittest.main()
