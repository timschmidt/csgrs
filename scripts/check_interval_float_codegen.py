#!/usr/bin/env python3
"""Inspect a release strict-float client for qualified x86_64 and AArch64 GNU targets.

Archives the build, disassembly and scan result outside the source tree. This
checks emitted kernel code; it is not a universal correctness or cost proof.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default="stable")
    parser.add_argument("--target", help="Rust target triple when cross-compiling")
    parser.add_argument("--objdump", default="objdump", help="GNU objdump for the target")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    compiler = subprocess.check_output(
        ["rustc", f"+{args.toolchain}", "-vV"], text=True
    )
    host = next(line.removeprefix("host: ") for line in compiler.splitlines() if line.startswith("host: "))
    target = args.target or host
    arm = target == "aarch64-unknown-linux-gnu"
    if not arm and not target.startswith("x86_64-"):
        raise SystemExit(f"unsupported codegen qualification target: {target}")
    (output / "compiler.txt").write_text(compiler)
    command = [
        "cargo", f"+{args.toolchain}", "build", "--locked", "--release",
        "-p", "hyperinterval", "--example", "float_cost",
        "--no-default-features", "--features",
        "float-enclosure-aarch64-linux-gnu" if arm else "float-enclosure",
        "--message-format=json",
    ]
    if args.target:
        command.extend(["--target", target])
    (output / "command.json").write_text(json.dumps(command, indent=2) + "\n")
    with (output / "build.jsonl").open("w") as stdout, (output / "build.log").open("w") as stderr:
        subprocess.run(command, cwd=root, stdout=stdout, stderr=stderr, check=True)
    artifacts = [json.loads(line) for line in (output / "build.jsonl").read_text().splitlines()]
    binaries = [
        entry["executable"] for entry in artifacts
        if entry.get("reason") == "compiler-artifact"
        and entry.get("target", {}).get("name") == "float_cost"
        and entry.get("executable")
    ]
    if len(binaries) != 1:
        raise SystemExit(f"expected one float_cost executable, got {binaries}")
    binary = Path(binaries[0])
    disassembly = subprocess.check_output([args.objdump, "-d", "-C", str(binary)], text=True)
    (output / "disassembly.txt").write_text(disassembly)
    for tool, name in [(["nm", "-S", "-C"], "symbols.txt"), (["size"], "size.txt")]:
        (output / name).write_text(subprocess.check_output([*tool, str(binary)], text=True))
    blocks = {}
    current = None
    for line in disassembly.splitlines():
        header = re.match(r"^[0-9a-f]+ <(.*)>:$", line)
        if header:
            current = header[1]
            blocks[current] = []
        elif current:
            blocks[current].append(line)
    selected = {
        name: lines for name, lines in blocks.items()
        if "hyperinterval::float::" in name or "hyperinterval::ieee::" in name
        or "hyperinterval::bounds::" in name or "hyperinterval::sqrt::" in name
        or "core::num::int_sqrt::" in name
        or name.startswith("hyperinterval_strict_")
        or name.startswith(("__udiv", "__umod"))
    }
    required = {f"hyperinterval_strict_{kind}{width}" for kind in ["chain", "surface", "sqrt"] for width in [32, 64]}
    if not required.issubset(selected):
        raise SystemExit(f"missing inspection entry points: {required - selected.keys()}")
    if not any("hyperinterval::ieee::" in name and "finite" in name for name in selected):
        raise SystemExit("the opaque integer finite-validation boundary was not emitted")
    forbidden = re.compile(
        r"^(?:v?(?:(?:add|sub|mul|div|min|max|sqrt|round|cmp|rcp|rsqrt)(?:ss|sd|ps|pd)"
        r"|(?:fmadd|fmsub|fnmadd|fnmsub|fmaddsub|fmsubadd)(?:132|213|231)(?:ss|sd|ps|pd)"
        r"|u?comi(?:ss|sd)|cvt\w+)|f(?:add|sub|mul|div|sqrt|com|ucom|prem|abs|chs|rndint)\w*)$"
    )
    if arm:
        # FMOV can transfer bits between integer and SIMD registers. Arithmetic,
        # comparisons and conversions can depend on FPCR even for finite values.
        forbidden = re.compile(r"(?:f(?!mov$)\w+|[su]cvtf|bf(?:cvt|dot|mmla|mlal)\w*)")
    findings = []
    for name, lines in selected.items():
        for line in lines:
            fields = line.split("\t")
            if len(fields) >= 3 and fields[2].split() and forbidden.fullmatch(fields[2].split()[0]):
                findings.append({"symbol": name, "instruction": line.strip()})
    (output / "kernel.asm").write_text("\n".join(name + ":\n" + "\n".join(lines) for name, lines in selected.items()))
    report = {
        "build_environment": {key: os.environ.get(key) for key in [
            "CARGO_TARGET_DIR", "RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS",
            "CARGO_PROFILE_RELEASE_LTO", "CARGO_PROFILE_RELEASE_CODEGEN_UNITS",
        ]},
        "binary": str(binary), "sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
        "bytes": binary.stat().st_size, "target": target, "symbols": sorted(selected), "findings": findings,
        "scope": "Emitted Hyperinterval float/IEEE/Bounds functions, six client entry points, integer-square-root and unsigned wide-division helpers; bit moves permitted, runtime/panic code excluded.",
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    if findings:
        raise SystemExit(f"native floating arithmetic/comparison found: {findings}")
    print(f"verified {len(selected)} emitted symbols: no native floating arithmetic/comparison")


if __name__ == "__main__":
    main()
