#!/usr/bin/env python3
"""Archive unmodified 86100C local files for a named run; no SCPI or timing inference."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import tempfile

try:
    from board_session import sha256_file
    from out_of_loop import write_json
except ImportError:
    from tools.board_session import sha256_file
    from tools.out_of_loop import write_json


def archive(batch_dir, run_id, sources):
    batch_dir = Path(batch_dir).resolve()
    manifest_path = batch_dir / "batch_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    matching = [run for run in manifest["runs"] if run["run_id"] == run_id]
    if len(matching) != 1:
        raise ValueError("run-id must exactly match one run in batch_manifest.json")
    run = matching[0]
    if manifest.get("scope_capture") != "local":
        raise ValueError("batch was not configured for instrument-local scope capture")
    destination = Path(run["scope_local_export_dir"]).resolve()
    if destination.parent != Path(run["run_dir"]).resolve():
        raise ValueError("scope export directory does not belong to the selected run")
    destination.mkdir(exist_ok=True)
    originals = [Path(item).resolve() for item in sources]
    if not originals or any(not path.is_file() for path in originals):
        raise ValueError("all sources must be existing regular files")
    if len({path.name.casefold() for path in originals}) != len(originals):
        raise ValueError("source basenames collide")
    for path in originals:
        target = destination / path.name
        if target.exists():
            raise FileExistsError(f"refusing to overwrite existing native evidence: {target}")
        if path.parent == destination:
            raise ValueError("source is already inside this run's native evidence directory")
    records = []
    for path in originals:
        target = destination / path.name
        with tempfile.NamedTemporaryFile(dir=destination, prefix=".native-", delete=False) as stream:
            temporary = Path(stream.name)
            try:
                with path.open("rb") as source:
                    shutil.copyfileobj(source, stream)
                stream.flush()
                os.fsync(stream.fileno())
            except BaseException:
                temporary.unlink(missing_ok=True)
                raise
        try:
            if sha256_file(path) != sha256_file(temporary):
                raise IOError("native file changed during copy or checksum mismatch")
            os.replace(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)
        records.append(dict(file=str(target), sha256=sha256_file(target),
                            bytes=target.stat().st_size, imported_local=datetime.now().astimezone().isoformat(),
                            source_path=str(path)))
    run.setdefault("scope_native_files", []).extend(records)
    run["scope_native_file_status"] = "archived_unparsed_timing_unverified"
    write_json(run["run_manifest_path"], run)
    write_json(manifest_path, manifest)
    return records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch_dir", type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--source", action="append", required=True, type=Path,
                        help="one original native file; repeat for multiple files")
    args = parser.parse_args(argv)
    for record in archive(args.batch_dir, args.run_id, args.source):
        print(record["file"], record["sha256"])


if __name__ == "__main__":
    main()
