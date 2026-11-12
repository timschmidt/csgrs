#!/usr/bin/env python3
"""Run the maintained Q30 differential client against an exact sibling checkout."""

import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--toolchain", default="1.88.0")
    parser.add_argument("--bench", action="store_true", help="run the paired out-of-line cost probe")
    args = parser.parse_args()
    crate = Path(__file__).resolve().parents[1]
    firmware = args.firmware.resolve()
    if not (firmware / "crates/alumina-foc/Cargo.toml").is_file():
        parser.error("--firmware must name an Alumina Firmware checkout")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = output / "Cargo.toml"
    if manifest.exists():
        parser.error("use a fresh output directory to preserve prior evidence")
    # JSON strings are valid TOML basic strings for these local file paths.
    manifest.write_text(f'''[package]
name = "interval-alumina-conformance"
version = "0.0.0"
edition = "2024"
publish = false
[dependencies]
hyperinterval = {{ path = {json.dumps(str(crate))}, features = ["fixed"] }}
alumina-foc = {{ path = {json.dumps(str(firmware / "crates/alumina-foc"))} }}
[[test]]
name = "q30"
path = {json.dumps(str(crate / "conformance/alumina_q30.rs"))}
[[bin]]
name = "fixed_cost"
path = {json.dumps(str(crate / "conformance/fixed_cost.rs"))}
[workspace]
''')
    command = ["cargo", "+" + args.toolchain, "test", "--manifest-path", str(manifest)]
    with (output / "test.log").open("w") as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    if result.returncode == 0 and args.bench:
        command = ["cargo", "+" + args.toolchain, "run", "--release", "--manifest-path", str(manifest), "--bin", "fixed_cost"]
        with (output / "samples.csv").open("w") as samples, (output / "bench-build.log").open("w") as log:
            result = subprocess.run(command, stdout=samples, stderr=log)
    (output / "result.json").write_text(json.dumps({
        "command": command, "exit": result.returncode,
        "firmware_revision": subprocess.check_output(
            ["git", "-C", str(firmware), "rev-parse", "HEAD"], text=True
        ).strip(),
    }, indent=2) + "\n")
    print((output / "test.log").read_text())
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
