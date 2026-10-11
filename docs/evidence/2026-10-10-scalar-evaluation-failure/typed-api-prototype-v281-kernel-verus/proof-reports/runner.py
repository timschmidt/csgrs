#!/usr/bin/env python3
"""Run the unchanged repository proof runner and retain its terminal reports."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
CRATE = ROOT / "target/scalar-evaluation-candidate/source/crates/hyperreal"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = EVIDENCE / sys.argv[1] / "proof-reports"
    out.mkdir()
    pin = json.loads((CRATE / "verification/toolchain.json").read_text())
    tool = ROOT / "crates/hyperreal/target/verus" / pin["version"] / "verus-x86-linux"
    tool_hashes = {str(path.relative_to(tool)): digest(path)
                   for path in sorted(tool.rglob("*")) if path.is_file()}
    assert (tool / "verus").is_file()
    reports = CRATE / "target/verification"
    # These generated reports were archived by each previous invocation. Clear
    # only their known names so a failed proof cannot inherit stale success.
    report_names = ["dependencies-build.stdout", "dependencies-build.stderr", "dependencies.json",
                    "verus.json", "verus.stderr", "evidence.json"]
    for name in report_names:
        (reports / name).unlink(missing_ok=True)
    command = [sys.executable, str(CRATE / "scripts/verify_verus.py"), "verify"]
    result = subprocess.run(command, cwd=CRATE, env=os.environ | {
        "VERUS": str(tool / "verus"), "PYTHONDONTWRITEBYTECODE": "1",
    })
    for name in report_names:
        path = reports / name
        if path.exists():
            shutil.copyfile(path, out / name)
    assert tool_hashes == {str(path.relative_to(tool)): digest(path)
                          for path in sorted(tool.rglob("*")) if path.is_file()}
    shutil.copyfile(__file__, out / "runner.py")
    (out / "tooling.json").write_text(json.dumps({
        "command": command, "exit_code": result.returncode,
        "pinned_toolchain": pin, "tool_directory": str(tool),
        "tool_sha256": tool_hashes, "tool_unchanged": True,
        "report_sha256": {name: digest(out / name) for name in report_names if (out / name).exists()},
        "scope": "Full repository proof target with --no-cheating, pinned dependency identity checks and declared contract/theorem checks; full-crate completion and baseline acceptance remain separate open gates.",
    }, indent=2) + "\n")
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
