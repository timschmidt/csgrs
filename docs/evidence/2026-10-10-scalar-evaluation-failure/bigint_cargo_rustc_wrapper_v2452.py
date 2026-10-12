#!/usr/bin/python3
"""Verify the selected BigInt/BigUint crate while preserving Cargo's compiler arguments."""
import hashlib
import json
import os
import re
from pathlib import Path
import subprocess
import sys
import time

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    rustc, *arguments = sys.argv[1:]
    if "--crate-name" not in arguments or arguments[arguments.index("--crate-name") + 1] != "num_bigint":
        os.execv(rustc, [rustc, *arguments])
    config = json.loads(Path(os.environ["CSGRS_BIGUINT_EXPORT_CONFIG"]).read_text())
    reports = Path(config["reports"])
    assert not (reports / "compiler-invocation.json").exists()
    assert Path(os.environ["CARGO_MANIFEST_DIR"]).resolve() == Path(config["vendor"]).resolve()
    assert str(Path(config["vendor"]) / "src/lib.rs") in arguments
    assert arguments[arguments.index("--crate-type") + 1] == "lib"
    assert sha(Path(__file__)) == config["wrapper_sha256"]
    tool = Path(config["verus"])
    assert sha(tool) == config["verus_sha256"]
    command = [str(tool), "--no-cheating", "--output-json", "--compile", "--export", config["export"], *arguments]
    protocol = {"rustc": rustc, "cargo_arguments": arguments, "verus_command": command,
                "wrapper_sha256": sha(Path(__file__)), "config_sha256": sha(Path(os.environ["CSGRS_BIGUINT_EXPORT_CONFIG"])),
                "cargo_manifest_dir": os.environ["CARGO_MANIFEST_DIR"]}
    (reports / "compiler-invocation.json").write_text(json.dumps(protocol, indent=2) + "\n")
    start = time.monotonic()
    descriptors = sorted({int(number) for pair in re.findall(
        r"--jobserver-(?:auth|fds)=(\d+),(\d+)", os.environ.get("CARGO_MAKEFLAGS", "")) for number in pair})
    for descriptor in descriptors:
        os.fstat(descriptor)
    run = subprocess.run(command, capture_output=True, text=True, pass_fds=tuple(descriptors))
    stdout, stderr = reports / "compiler.stdout.json", reports / "compiler.stderr.log"
    stdout.write_text(run.stdout)
    stderr.write_text(run.stderr)
    try:
        parsed = json.loads(run.stdout).get("verification-results", {})
    except json.JSONDecodeError:
        parsed = {}
    diagnostics = [json.loads(line) for line in run.stderr.splitlines()]
    warnings = [item for item in diagnostics if item.get("level") == "warning"]
    passed = (run.returncode == 0 and parsed.get("verified") == 453
              and parsed.get("errors") == 0 and not warnings)
    export = Path(config["export"])
    result = {"exit_code": run.returncode, "seconds": time.monotonic() - start,
              "verification_results": parsed, "gate_passed": passed,
              "warnings": len(warnings), "inherited_jobserver_descriptors": descriptors,
              "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr),
              "export_sha256": sha(export) if export.exists() else None,
              "compiler_invocation_sha256": sha(reports / "compiler-invocation.json")}
    (reports / "compiler-result.json").write_text(json.dumps(result, indent=2) + "\n")
    # Cargo receives the original compiler's structured diagnostics, while the
    # separate verifier summary remains in the immutable evidence directory.
    sys.stderr.write(run.stderr)
    sys.exit(0 if passed else (run.returncode or 1))


if __name__ == "__main__":
    main()
