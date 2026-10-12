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
    assert entrypoint == "test_rejections.py"
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
    cache_phase = EVIDENCE / "typed-api-prototype-v2265-eager-bound-decoder-cache-rejections"
    cache = json.loads((cache_phase / "proof-checks/tooling.json").read_text())
    assert cache["tool_sha256"] == tool_hashes and cache["tool_unchanged"] and cache["exit_code"] == 0
    assert cache["rejected_invocations"] == 41 and cache["baseline_successes"] == 1
    assert digest(cache_phase / "source-hashes.json.gz") == "cc51e3938c1cb6d772778a4683b7412812e234e69cfb1550d6b6e14120d264ad"
    original_entrypoint = script.read_text()
    omitted = "    reject_cache_mutations()\n"
    assert original_entrypoint.count(omitted) == 1
    selected_entrypoint = original_entrypoint.replace(omitted, "")
    (out / "selected-entrypoint.py").write_text(selected_entrypoint)
    (out / "selection.json").write_text(json.dumps({
        "original_entrypoint": str(script), "original_entrypoint_sha256": digest(script),
        "selected_entrypoint_sha256": digest(out / "selected-entrypoint.py"),
        "only_removed_call": omitted, "separately_completed_cache_phase": cache_phase.name,
        "cache_tooling_sha256": digest(cache_phase / "proof-checks/tooling.json"),
        "scope": "Run every other mutation and positive baseline unchanged. The cache group already ran on these exact source/verifier identities; it is combined explicitly, not rerun or omitted from total coverage.",
    }, indent=2) + "\n")
    sys.argv = [str(script)]
    code = 1
    try:
        exec(compile(selected_entrypoint, str(script), "exec"), {"__name__": "__main__", "__file__": str(script)})
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
