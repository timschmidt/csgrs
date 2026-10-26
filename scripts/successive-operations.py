#!/usr/bin/env python3
"""Build attributable successive-operation probes and retain interrupted runs.

Build: successive-operations.py build --output /tmp/probe-build
Run:   successive-operations.py run --binary /tmp/probe-build/successive_operations \
           --output /tmp/probe-run --timeout 300 -- '4-gon drills'

The build embeds its source snapshot and the actual rustc invocation. Run can
use a copied binary from any working directory; it never attributes that binary
to the current checkout. Evidence directories must be new and outside the repo.
This native Unix driver uses Python 3.11+ and Cargo/rustc, without extra packages.
"""

import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time
import uuid


ROOT = Path(__file__).resolve().parents[1]
BUILD_INPUT = "CSGRS_SUCCESSIVE_BUILD_INPUT"
BUILD_INFO = "CSGRS_SUCCESSIVE_BUILD_INFO"


def digest_file(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def git(*args):
    return subprocess.check_output(["git", "-C", str(ROOT), *args])


def source_snapshot():
    tracked = git("ls-files", "-z").split(b"\0")
    # Include new build inputs without reading unrelated personal files.
    untracked = git(
        "ls-files", "-z", "--others", "--exclude-standard", "--", "src", "crates",
        "tests", "benchmarks", "scripts", ".cargo", "Cargo.*", "build.rs", "rust-toolchain*",
    ).split(b"\0")
    rows = []
    for encoded in sorted(set(tracked + untracked) - {b""}):
        relative = os.fsdecode(encoded)
        path = ROOT / relative
        if path.is_symlink():
            content = os.fsencode(os.readlink(path))
            digest, mode = hashlib.sha256(content).hexdigest(), "symlink"
        elif path.is_file():
            digest = digest_file(path)
            mode = "executable" if path.stat().st_mode & 0o111 else "file"
        elif not path.exists():
            digest, mode = None, "deleted"
        else:
            raise RuntimeError(f"unsupported source entry: {relative}")
        rows.append([relative, mode, digest])
    encoded_rows = json.dumps(rows, ensure_ascii=True, separators=(",", ":")).encode()
    patch = git("diff", "--binary", "--no-ext-diff", "HEAD")
    return {
        "revision": git("rev-parse", "HEAD").decode().strip(),
        "tree_sha256": hashlib.sha256(encoded_rows).hexdigest(),
        "dirty_patch_sha256": hashlib.sha256(patch).hexdigest(),
        "lock_sha256": digest_file(ROOT / "Cargo.lock"),
        "files": len(rows),
        "untracked_inputs": [os.fsdecode(path) for path in sorted(set(untracked) - {b""})],
    }, rows, patch


def new_output(path):
    path = path.resolve()
    if path.is_relative_to(ROOT):
        raise ValueError("evidence output must be outside the source repository")
    path.mkdir(parents=True, exist_ok=False)
    return path


def rustc_wrapper(arguments):
    """Cargo invokes this executable as RUSTC_WRAPPER rustc [arguments]."""
    environment = os.environ.copy()
    if "--crate-name" in arguments:
        index = arguments.index("--crate-name")
        if arguments[index + 1] == "successive_operations":
            if not Path(arguments[0]).name.startswith("rustc"):
                raise RuntimeError("nested compiler wrappers are not supported by this probe driver")
            source = Path(environment[BUILD_INPUT])
            info = json.loads(source.read_text())
            info["compiler"] = {
                "command": arguments,
                "version": subprocess.check_output([arguments[0], "-Vv"], text=True),
                "cwd": str(Path.cwd()),
            }
            environment[BUILD_INFO] = json.dumps(info, sort_keys=True, separators=(",", ":"))
            save_json(source.parent / "compiler.json", info)
    os.execvpe(arguments[0], arguments, environment)


def build(args):
    output = new_output(args.output)
    snapshot, rows, patch = source_snapshot()
    save_json(output / "source-files.json", rows)
    (output / "dirty.patch").write_bytes(patch)
    for relative in snapshot["untracked_inputs"]:
        source_path = ROOT / relative
        destination = output / "untracked" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if source_path.is_symlink():
            destination.symlink_to(os.readlink(source_path))
        else:
            shutil.copy2(source_path, destination)
    command = [
        "cargo", "bench", "--locked", "--bench", "successive_operations", "--no-run",
        "--message-format=json", "--profile", args.profile, "--features", args.features,
    ]
    if not args.default_features:
        command.append("--no-default-features")
    if args.target:
        command += ["--target", args.target]
    info = {
        "schema": 1, "build_id": uuid.uuid4().hex, "source": snapshot,
        "cargo_command": command, "source_directory": str(ROOT),
        "build_environment": {
            key: value for key, value in os.environ.items()
            if key in {"RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS", "RUSTC", "SOURCE_DATE_EPOCH"}
            or key.startswith(("CARGO_PROFILE_", "CARGO_TARGET_"))
        },
    }
    source = output / "build-input.json"
    save_json(source, info)
    environment = os.environ.copy()
    if environment.get("RUSTC_WORKSPACE_WRAPPER"):
        raise ValueError("remove RUSTC_WORKSPACE_WRAPPER for this attributed build")
    environment["RUSTC_WRAPPER"] = str(Path(__file__).resolve())
    environment[BUILD_INPUT] = str(source)
    # An invocation-specific stamp makes Cargo rebuild this benchmark. The
    # wrapper replaces it with the complete actual compiler record before rustc.
    environment[BUILD_INFO] = info["build_id"]
    with (output / "cargo.jsonl").open("w") as stdout, (output / "cargo.log").open("w") as stderr:
        result = subprocess.run(command, cwd=ROOT, env=environment, stdout=stdout, stderr=stderr)
    if result.returncode:
        raise RuntimeError(f"Cargo failed ({result.returncode}); see {output / 'cargo.log'}")
    if source_snapshot()[0] != snapshot:
        raise RuntimeError("source changed during the build; this build is not admitted")
    artifacts = [json.loads(line) for line in (output / "cargo.jsonl").read_text().splitlines()]
    binary = next(
        Path(item["executable"]) for item in artifacts
        if item.get("executable") and item.get("target", {}).get("name") == "successive_operations"
    )
    retained = output / "successive_operations"
    shutil.copy2(binary, retained)
    info = json.loads((output / "compiler.json").read_text())
    save_json(output / "build.json", {
        "build": info, "binary_sha256": digest_file(retained),
        "cargo_records_sha256": digest_file(output / "cargo.jsonl"),
        "source_verified_after_build": True,
    })
    print(retained)


def journal_summary(path):
    counts = collections.Counter()
    rows = 0
    completed = False
    if path.exists():
        with path.open() as stream:
            for line in stream:
                if not line.endswith("\n"):
                    continue  # A partial final write is never a completed row.
                if line == "# completed\n":
                    completed = True
                elif not line.startswith(("#", "chain\t")):
                    columns = line.rstrip("\n").split("\t")
                    if len(columns) == 18:
                        rows += 1
                        counts[columns[2]] += 1
    return {"rows": rows, "outcomes": dict(counts), "completion_marker": completed}


def stop_process(process, signum):
    try:
        os.killpg(process.pid, signum)
    except ProcessLookupError:
        pass


def run(args):
    output = new_output(args.output)
    binary = args.binary.resolve()
    binary_hash = digest_file(binary)
    info = json.loads(subprocess.check_output([str(binary), "--build-info"], text=True, timeout=30))
    if not info.get("compiler"):
        raise ValueError("binary has no attributed build; build it with this driver first")
    arguments = args.arguments
    if arguments[:1] == ["--"]:
        arguments = arguments[1:]
    manifest = {
        "schema": 1, "run_id": uuid.uuid4().hex, "status": "incomplete",
        "build": info, "binary": str(binary), "binary_sha256": binary_hash,
        "driver_sha256": digest_file(__file__),
        "arguments": arguments, "cwd": str(Path.cwd()),
        "started_unix_ns": time.time_ns(), "timeout_seconds": args.timeout,
        "host": {
            "platform": platform.platform(), "machine": platform.machine(),
            "cpu_count": os.cpu_count(),
            "affinity": sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None,
        },
    }
    save_json(output / "run.json", manifest)
    environment = os.environ.copy()
    environment["SUCCESSIVE_OPERATIONS_RAW"] = str(output / "rows.tsv")
    with (output / "stdout.log").open("w") as stdout, (output / "stderr.log").open("w") as stderr:
        process = subprocess.Popen(
            [str(binary), *arguments], env=environment, stdout=stdout, stderr=stderr,
            stdin=subprocess.DEVNULL, start_new_session=True,
        )
        try:
            manifest["pid"] = process.pid
            save_json(output / "run.json", manifest)
            code = process.wait(timeout=args.timeout)
            manifest["status"] = "completed" if code == 0 else "interrupted" if code < 0 else "failed"
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
            manifest["status"] = "timeout" if isinstance(error, subprocess.TimeoutExpired) else "interrupted"
            stop_process(process, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                stop_process(process, signal.SIGKILL)
                process.wait()
            code = 124 if manifest["status"] == "timeout" else 130
    manifest.update(journal_summary(output / "rows.tsv"))
    manifest.update(ended_unix_ns=time.time_ns(), exit_code=code, process_returncode=process.returncode)
    if code == 0 and (not manifest["completion_marker"] or not manifest["rows"]):
        manifest["status"] = "incomplete"
        code = 1
    if digest_file(binary) != binary_hash:
        manifest["status"] = "binary_changed"
        code = 1
    manifest["driver_exit_code"] = code
    if (output / "rows.tsv").exists():
        manifest["rows_sha256"] = digest_file(output / "rows.tsv")
    save_json(output / "run.json", manifest)
    print(json.dumps({key: manifest[key] for key in ("status", "rows", "outcomes")}, sort_keys=True))
    return code


def main():
    # Cargo's compiler probes and compilations use this file as an executable,
    # while the user's CLI always starts with build or run.
    if os.environ.get(BUILD_INPUT) and sys.argv[1:2] not in (["build"], ["run"]):
        rustc_wrapper(sys.argv[1:])
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    builder = commands.add_parser("build")
    builder.add_argument("--output", type=Path, required=True)
    builder.add_argument("--features", default="curve,stl-io,obj-io")
    builder.add_argument("--default-features", action="store_true")
    builder.add_argument("--profile", default="release")
    builder.add_argument("--target")
    runner = commands.add_parser("run")
    runner.add_argument("--binary", type=Path, required=True)
    runner.add_argument("--output", type=Path, required=True)
    runner.add_argument("--timeout", type=float)
    runner.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if os.name != "posix":
        parser.error("this driver requires a native Unix host")
    if args.command == "run":
        def interrupted(_signum, _frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupted)
    return build(args) if args.command == "build" else run(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        sys.exit(str(error))
