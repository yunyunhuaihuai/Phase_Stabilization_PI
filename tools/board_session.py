"""Key/agent SSH access and run-owned controller evidence; no device access on import."""
import hashlib
import csv
import math
from pathlib import Path
import posixpath
import re
import shlex
import time


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def recv_exit_status(channel, timeout_s=10):
    deadline = time.monotonic() + timeout_s
    while not channel.exit_status_ready():
        if time.monotonic() >= deadline:
            raise RuntimeError("SSH exit status timed out; EOF is not success")
        time.sleep(.02)
    code = channel.recv_exit_status()
    if not isinstance(code, int) or isinstance(code, bool) or code < 0:
        raise RuntimeError(f"SSH did not provide a valid exit status: {code!r}")
    return code


def remote_paths(project, binary, run_id):
    if not project.startswith("/") or not binary.startswith("/"):
        raise ValueError("board project and binary must be absolute POSIX paths")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", run_id):
        raise ValueError("unsafe run identity")
    directory = posixpath.join(project, "runtime_runs", run_id)
    config = posixpath.join(directory, "config.txt")
    pid = posixpath.join(directory, "controller.pid")
    script = f"echo $$ > {shlex.quote(pid)}; exec stdbuf -oL {shlex.quote(binary)} {shlex.quote(config)}"
    launch = f"cd {shlex.quote(directory)} && exec sh -c {shlex.quote(script)}"
    # Both signals take the existing C++ signal handler and VSMD stop path.
    # Match this run's unique config before either signal; never use killall/KILL.
    matches = (f'test -r "/proc/$pid/cmdline" && '
               f'tr "\\000" "\\n" < "/proc/$pid/cmdline" | grep -Fxq -- {shlex.quote(config)}')
    stop = (f"if test -f {shlex.quote(pid)}; then pid=$(cat {shlex.quote(pid)}); "
            'case "$pid" in ""|*[!0-9]*) exit 2;; esac; '
            f'if {matches}; then kill -INT "$pid"; sleep 2; '
            f'if {matches}; then kill -TERM "$pid"; fi; fi; fi')
    return dict(directory=directory, config=config, pid=pid, binary=binary,
                launch=launch, stop=stop)


class BoardSSH:
    def __init__(self, args):
        import paramiko
        self.ssh = paramiko.SSHClient()
        self.ssh.load_system_host_keys()
        if args.known_hosts:
            self.ssh.load_host_keys(args.known_hosts)
        # Unknown keys must be enrolled by the operator once with normal ssh.
        self.ssh.set_missing_host_key_policy(paramiko.RejectPolicy())
        ssh_config = Path.home() / ".ssh" / "config"
        lookup = {}
        if ssh_config.is_file():
            config = paramiko.SSHConfig()
            with ssh_config.open(encoding="utf-8") as stream:
                config.parse(stream)
            lookup = config.lookup(args.host)
        key = args.ssh_key or lookup.get("identityfile")
        proxy = paramiko.ProxyCommand(lookup["proxycommand"]) if lookup.get("proxycommand") else None
        self.ssh.connect(lookup.get("hostname", args.host),
                         port=args.ssh_port or int(lookup.get("port", 22)),
                         username=args.ssh_user or lookup.get("user", "root"),
                         key_filename=key, allow_agent=True, look_for_keys=True,
                         timeout=15, auth_timeout=15, banner_timeout=15, sock=proxy)

    def run(self, command, check=True, timeout=30):
        _, stdout, _ = self.ssh.exec_command(command, timeout=timeout)
        stdout.channel.set_combine_stderr(True)
        text = stdout.read().decode("utf-8", errors="replace").strip()
        status = recv_exit_status(stdout.channel)
        if check and status:
            raise RuntimeError(f"board command failed ({status}): {command}\n{text}")
        return text

    def exec_long(self, command):
        # No PTY: a lost notebook connection must not substitute terminal HUP
        # for the explicit SIGINT/TERM controlled shutdown path.
        _, stdout, _ = self.ssh.exec_command(command)
        stdout.channel.set_combine_stderr(True)
        return stdout

    def put_file(self, local, remote):
        with self.ssh.open_sftp() as sftp:
            sftp.put(str(local), remote)

    def get_file(self, remote, local):
        with self.ssh.open_sftp() as sftp:
            sftp.get(remote, str(local))

    def close(self):
        self.ssh.close()


def capture_board(board, paths, project, local_config, algo):
    try:
        from experiment_plan import verify_config
    except ImportError:
        from tools.experiment_plan import verify_config
    running = board.run("pgrep -af '[P]haseStabilization' || true")
    if running:
        raise RuntimeError("controller already running; refusing concurrent run: " + running)
    board.run(f"test -x {shlex.quote(paths['binary'])}")
    resolved_binary = board.run(f"readlink -f {shlex.quote(paths['binary'])}")
    # Record deployed provenance before adding this run's runtime evidence.
    git_commit = board.run(f"git -C {shlex.quote(project)} rev-parse HEAD 2>/dev/null || echo unavailable")
    git_status = board.run(f"git -C {shlex.quote(project)} status --porcelain=v1 --untracked-files=normal 2>/dev/null || echo unavailable")
    board.run(f"mkdir -p {shlex.quote(posixpath.dirname(paths['directory']))} && mkdir {shlex.quote(paths['directory'])}")
    board.put_file(local_config, paths["config"])
    remote_text = board.run(f"cat {shlex.quote(paths['config'])}")
    fields = verify_config(remote_text, algo)
    remote_hash = board.run(f"sha256sum {shlex.quote(paths['config'])}").split()[0]
    if remote_hash != sha256_file(local_config):
        raise RuntimeError("remote config SHA-256 does not match this run's local snapshot")
    binary_hash = board.run(f"sha256sum {shlex.quote(paths['binary'])}").split()[0]
    if not re.fullmatch(r"[0-9a-fA-F]{64}", binary_hash):
        raise RuntimeError("invalid binary SHA-256")
    return dict(binary_path=resolved_binary, binary_requested_path=paths["binary"],
                binary_sha256=binary_hash, config_sha256=remote_hash,
                config_fields=fields, remote_config_path=paths["config"],
                git_commit=git_commit, git_status=git_status,
                git_status_time="before creating this run's runtime directory")


def logger_filename(line):
    marker = "[DataLogger] 日志文件:"
    if marker not in line:
        return None
    candidate = line.partition(marker)[2].strip().split()[0]
    # Only literal filename reported by this controller stream is accepted.
    if not re.fullmatch(r"Data_[A-Za-z0-9_.-]+\.csv", candidate):
        raise RuntimeError(f"invalid DataLogger run filename: {candidate!r}")
    return candidate


def download_artifacts(board, paths, name, output_dir, allow_partial=False):
    if not name or not re.fullmatch(r"Data_[A-Za-z0-9_.-]+\.csv", name):
        raise RuntimeError("missing this run's DataLogger filename; never selecting logs by mtime")
    result = dict(files={}, sha256={}, missing=[], validation_errors=[])
    for kind, suffix in (("raw_csv", ""), ("diagnostics_csv", ".diagnostics.csv"),
                         ("metadata", ".metadata.txt"), ("safety_csv", ".safety.csv")):
        remote = posixpath.join(paths["directory"], name + suffix)
        local = Path(output_dir) / (name + suffix)
        if board.run(f"test -f {shlex.quote(remote)} || echo MISSING", check=False) == "MISSING":
            result["missing"].append(kind)
            continue
        if local.exists():
            raise RuntimeError(f"refusing to overwrite evidence: {local}")
        try:
            board.get_file(remote, local)
            result["files"][kind] = str(local)
            result["sha256"][kind] = sha256_file(local)
        except Exception as exc:
            result["validation_errors"].append(f"download {kind}: {exc}")
    metadata = result["files"].get("metadata")
    if metadata:
        text = Path(metadata).read_text(encoding="utf-8", errors="replace")
        config_path = next((line.partition("=")[2].strip() for line in text.splitlines()
                            if line.startswith("CONFIG_PATH=")), None)
        if config_path != paths["config"]:
            result["validation_errors"].append(f"metadata CONFIG_PATH mismatch: {config_path!r}")
    result["complete"] = not result["missing"] and not result["validation_errors"]
    # Caller stores evidence and then decides whether the completed run is valid.
    return result


def evidence_health(run):
    """In-loop signals establish run integrity, never out-of-loop performance."""
    reasons, details = [], {}
    files = run.get("downloaded_artifacts", {}).get("files", {})

    def number(row, key):
        value = float(row[key])
        if not math.isfinite(value):
            raise ValueError(f"nonfinite {key}")
        return value

    try:
        with open(files["raw_csv"], encoding="utf-8", newline="") as stream:
            raw = list(csv.DictReader(stream))
        with open(files["diagnostics_csv"], encoding="utf-8", newline="") as stream:
            diagnostics = list(csv.DictReader(stream))
        metadata = dict(line.split("=", 1) for line in Path(files["metadata"]).read_text(
            encoding="utf-8").splitlines() if "=" in line)
        details.update(raw_rows=len(raw), diagnostic_rows=len(diagnostics))
        if not raw or len(raw) != len(diagnostics):
            reasons.append("incomplete_or_misaligned_board_rows")
        cycles = [number(row, "Cycle") for row in diagnostics]
        counters = dict(
            cycle_sequence_discontinuous=sum(b != a + 1 for a, b in zip(cycles, cycles[1:]))
                + int(bool(cycles) and cycles[0] != 1),
            invalid_sample_interval=sum(number(row, "Sample_dt_s") <= 0 for row in diagnostics),
            sample_interval_out_of_bounds=sum(number(row, "Sample_dt_s") > .1 for row in diagnostics[1:]),
            cycle_time_out_of_bounds=sum(number(row, "Previous_cycle_total_us") > 100000 for row in diagnostics),
            deadline_late=sum(number(row, "Deadline_late") != 0 for row in diagnostics),
            negative_send_result=sum(number(row, "Send_result") < 0 for row in diagnostics),
            abnormal_controller_status=sum(row["Controller_status"] != "ready" for row in diagnostics),
            physical_delta_limit_exceeded=sum(abs(number(row, "Physical_delta_steps")) > number(metadata,"MAX_PHYSICAL_DELTA") for row in diagnostics),
            raw_fault_rows=sum(row["State"].upper() == "FAULT" for row in raw))
        details.update(counters)
        reasons.extend(key for key, value in counters.items() if value)
        gain = number(metadata, "CALIBRATED_B0")
        details["calibrated_b0"] = gain
        if not .0005001 < gain < .0039999:
            reasons.append("calibration_gain_at_boundary")
        if any(number(metadata, key) != 5500 for key in ("EFFECTIVE_LOGICAL_LIMIT", "MAX_PHYSICAL_DELTA")):
            reasons.append("metadata_effective_limit_mismatch")
        if raw:
            elapsed = [number(row, "Elapsed_s") for row in raw]
            details.update(board_first_elapsed_s=elapsed[0], board_last_elapsed_s=elapsed[-1])
            if elapsed[0] < 0 or elapsed[0] > .25 or any(b <= a for a, b in zip(elapsed, elapsed[1:])):
                reasons.append("board_time_coverage_or_order_invalid")
            if elapsed[-1] < run.get("completed_plan_elapsed_s", 0) - .25:
                reasons.append("board_did_not_cover_event_plan")
    except (OSError, KeyError, ValueError) as exc:
        reasons.append("board_evidence_parse_failure")
        details["parse_error"] = str(exc)
    log = Path(run["controller_log"])
    if log.exists():
        text = log.read_text(encoding="utf-8", errors="replace")
        if "FATAL" in text or "guard_fault" in text or "[RunawayGuard] TRIP" in text:
            reasons.append("controller_fatal_or_guard_trip")
    try:
        if sha256_file(run["config_path"]) != run["config_sha256"]:
            reasons.append("config_snapshot_hash_mismatch")
        if run.get("board", {}).get("config_sha256") != run["config_sha256"]:
            reasons.append("remote_config_hash_unverified")
    except OSError:
        reasons.append("config_snapshot_missing")
    return reasons, details
