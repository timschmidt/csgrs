#!/usr/bin/env python3
"""Run candidate proof guard tests and retain each actual Verus invocation."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
CRATE = ROOT / "target/scalar-evaluation-candidate/source/crates/hyperreal"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    phase, entrypoint = sys.argv[1:]
    assert entrypoint == "test_bit_metadata_rejections.py"
    out = EVIDENCE / phase / "proof-checks"
    out.mkdir()
    pin = json.loads((CRATE / "verification/toolchain.json").read_text())
    tool = ROOT / "crates/hyperreal/target/verus" / pin["version"] / "verus-x86-linux"
    tool_hashes = {str(path.relative_to(tool)): digest(path)
                   for path in sorted(tool.rglob("*")) if path.is_file()}
    os.environ["VERUS"] = str(tool / "verus")
    os.environ["PYTHONDONTWRITEBYTECODE"] = "1"
    sys.dont_write_bytecode = True
    sys.path[:0] = [str(CRATE / "scripts"), str(CRATE / "verification")]
    import verify_verus
    reports = CRATE / "target/proof-checks" / phase
    assert not reports.exists()
    reports.mkdir(parents=True)
    verify_verus.REPORTS = reports
    actual_run = subprocess.run
    calls = []

    def retained_run(command, *args, **kwargs):
        is_proof = (str(command[0]) == str(tool / "verus")
                    and any(str(arg).endswith("lib.rs") for arg in command))
        record = None
        if is_proof:
            library = next(Path(arg) for arg in command if str(arg).endswith("lib.rs"))
            record = {
                "command": [str(arg) for arg in command],
                "rust_inputs_sha256": {str(path.relative_to(library.parent)): digest(path)
                                       for path in sorted(library.parent.rglob("*.rs"))},
            }
        result = actual_run(command, *args, **kwargs)
        if record is not None:
            record.update(exit_code=result.returncode, stdout=result.stdout, stderr=result.stderr)
            calls.append(record)
        return result

    subprocess.run = retained_run
    script = CRATE / "verification" / entrypoint
    sys.argv = [str(script)]
    code = 1
    try:
        runpy.run_path(str(script), run_name="__main__")
        code = 0
    except SystemExit as error:
        code = error.code if isinstance(error.code, int) else (1 if error.code else 0)
        if code:
            raise
    finally:
        subprocess.run = actual_run
        (out / "verus-invocations.jsonl.gz").write_bytes(gzip.compress(
            "".join(json.dumps(call, sort_keys=True) + "\n" for call in calls).encode(), mtime=0
        ))
        shutil.copytree(reports, out / "dependencies")
        shutil.copyfile(__file__, out / "runner.py")
        unchanged = tool_hashes == {str(path.relative_to(tool)): digest(path)
                                    for path in sorted(tool.rglob("*")) if path.is_file()}
        (out / "tooling.json").write_text(json.dumps({
            "entrypoint": str(script), "entrypoint_sha256": digest(script),
            "exit_code": code, "pinned_toolchain": pin, "tool_directory": str(tool),
            "tool_sha256": tool_hashes, "tool_unchanged": unchanged,
            "verus_invocations": len(calls),
            "baseline_successes": sum(call["exit_code"] == 0 for call in calls),
            "rejected_invocations": sum(call["exit_code"] != 0 for call in calls),
            "invocation_log_sha256": digest(out / "verus-invocations.jsonl.gz"),
            "scope": "The repository guard tests retain their original rejection criteria. Each proof invocation records its exact temporary Rust input hashes, output and exit status. This is not whole-crate proof or empirical admission.",
        }, indent=2) + "\n")
        assert unchanged
    sys.exit(code)


if __name__ == "__main__":
    main()
