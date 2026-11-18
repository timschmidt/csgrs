#!/usr/bin/env python3
"""Execute exact interval conformance clients in a real WebAssembly runtime.

Requires Python, Node with WebAssembly BigInt support, the selected Rust WASM
target, and Binaryen's wasm-dis. All generated artifacts stay in --output.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time


def floating_operations(assembly):
    floating = sorted(set(re.findall(r"\b(?:f32|f64|i32|i64)\.[a-z][a-z0-9_]*", assembly)))
    return [op for op in floating if (
        op.startswith(("f32.", "f64.")) and op not in {"f32.reinterpret_i32", "f64.reinterpret_i64"}
    ) or "_f32" in op and op != "i32.reinterpret_f32" or "_f64" in op and op != "i64.reinterpret_f64"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default="stable")
    parser.add_argument("--target", choices=("wasm32-unknown-unknown", "wasm32-wasip1"), default="wasm32-unknown-unknown")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    client = root / "crates/hyperinterval/conformance/wasm-float"
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = os.environ | {"CARGO_TARGET_DIR": str(output / "target"), "CARGO_TERM_COLOR": "never"}
    target = args.target
    runtime = ["--wasi-p1"] if target == "wasm32-wasip1" else []
    ieee = ["--features", "float-enclosure-wasip1"] if runtime else []
    commands = []

    def run(name, command, *, timeout=900):
        start = time.monotonic()
        with (output / f"{name}.log").open("w") as log, (output / f"{name}.stderr.log").open("w") as errors:
            result = subprocess.run(command, cwd=root, env=env, stdout=log, stderr=errors, timeout=timeout)
        commands.append({"name": name, "command": command, "exit_code": result.returncode, "seconds": time.monotonic() - start})
        (output / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        if result.returncode:
            raise SystemExit(f"{name} failed; see {output / (name + '.log')} and its .stderr.log")

    run("compiler", ["rustc", f"+{args.toolchain}", "-vV"])
    run("oracle", ["python3", str(client / "oracle.py"), str(output / "corpus.json")])
    run("build", ["cargo", f"+{args.toolchain}", "build", "--locked", "--release", "--manifest-path", str(client / "Cargo.toml"), "--target", target, *ieee])
    run("lint", ["cargo", f"+{args.toolchain}", "clippy", "--locked", "--manifest-path", str(client / "Cargo.toml"), "--target", target, *ieee, "--", "-D", "warnings"])
    binary = output / "interval_float.wasm"
    shutil.copyfile(output / "target" / target / "release/interval_float_wasm_conformance.wasm", binary)
    run("execute", ["node", str(client / "check.mjs"), str(binary), str(output / "corpus.json"), *runtime])
    run("disassemble", ["wasm-dis", str(binary), "-o", str(output / "module.wat")])
    assembly = (output / "module.wat").read_text()
    forbidden = floating_operations(assembly)
    if forbidden or "memory.grow" in assembly or "(import " in assembly:
        raise SystemExit(f"Unexpected arithmetic, allocation/import path: {forbidden}")
    run("guest-stack", ["node", str(client / "stack.mjs"), str(binary), str(output / "module.wat"), str(output / "corpus.json")])
    run("dependencies", ["cargo", f"+{args.toolchain}", "tree", "--locked", "--manifest-path", str(client / "Cargo.toml"), "--edges", "normal", "--prefix", "none", "--target", target, *ieee])
    dependencies = (output / "dependencies.log").read_text().splitlines()
    expected = ["interval-float-wasm-conformance", "hyperinterval"]
    if target == "wasm32-wasip1":
        expected.append("target-tuple")
    if [line.split()[0] for line in dependencies] != expected:
        raise SystemExit(f"Unexpected dependency closure: {dependencies}")
    lattice_manifest = root / "crates/hyperlattice-interval/conformance/primitive-layouts/wasm/Cargo.toml"
    run("lattice-build", ["cargo", f"+{args.toolchain}", "build", "--locked", "--release", "--manifest-path", str(lattice_manifest), "--target", target, *ieee])
    run("lattice-lint", ["cargo", f"+{args.toolchain}", "clippy", "--locked", "--manifest-path", str(lattice_manifest), "--target", target, *ieee, "--", "-D", "warnings"])
    lattice_binary = output / "interval_layout.wasm"
    shutil.copyfile(output / "target" / target / "release/interval_layout_wasm.wasm", lattice_binary)
    run("lattice-execute", ["node", str(client / "check.mjs"), str(lattice_binary), str(output / "corpus.json"), "--lattice", *runtime])
    fixed = root / "crates/hyperlattice-interval/conformance/fixed-wasm"
    fixed_options = ["--locked", "--manifest-path", str(fixed / "Cargo.toml"), "--target", target]
    run("fixed-lattice-build", ["cargo", f"+{args.toolchain}", "build", "--release", *fixed_options])
    run("fixed-lattice-lint", ["cargo", f"+{args.toolchain}", "clippy", *fixed_options, "--", "-D", "warnings"])
    fixed_binary = output / "interval_fixed_lattice.wasm"
    shutil.copyfile(output / "target" / target / "release/interval_fixed_lattice_wasm.wasm", fixed_binary)
    fixed_wat = output / "fixed-lattice.wat"
    run("fixed-lattice-disassemble", ["wasm-dis", str(fixed_binary), "-o", str(fixed_wat)])
    fixed_assembly = fixed_wat.read_text()
    if floating_operations(fixed_assembly) or "memory.grow" in fixed_assembly:
        raise SystemExit("Unexpected floating arithmetic or memory growth in fixed lattice client")
    run("fixed-lattice-execute", ["node", str(fixed / "check.mjs"), str(fixed_binary)], timeout=60)
    dyadic_manifest = root / "crates/hyperinterval/conformance/wasm-dyadic/Cargo.toml"
    run("dyadic-build", ["cargo", f"+{args.toolchain}", "build", "--locked", "--release", "--manifest-path", str(dyadic_manifest), "--target", target, *ieee])
    run("dyadic-lint", ["cargo", f"+{args.toolchain}", "clippy", "--locked", "--manifest-path", str(dyadic_manifest), "--target", target, *ieee, "--", "-D", "warnings"])
    dyadic_binary = output / "interval_dyadic.wasm"
    shutil.copyfile(output / "target" / target / "release/interval_dyadic_wasm_conformance.wasm", dyadic_binary)
    run("dyadic-execute", ["node", str(client / "check.mjs"), str(dyadic_binary), str(output / "corpus.json"), "--egress", *runtime])
    run("dyadic-disassemble", ["wasm-dis", str(dyadic_binary), "-o", str(output / "dyadic.wat")])
    dyadic_assembly = (output / "dyadic.wat").read_text()
    dyadic_floating = floating_operations(dyadic_assembly)
    if dyadic_floating or (not runtime and "(import " in dyadic_assembly):
        raise SystemExit(f"Unexpected dyadic floating arithmetic or imports: {dyadic_floating}")
    report = json.loads((output / "execute.log").read_text())
    report.update({"toolchain": args.toolchain, "target": target, "binary_bytes": binary.stat().st_size,
                   "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
                   "corpus_sha256": hashlib.sha256((output / "corpus.json").read_bytes()).hexdigest(),
                   "floating_arithmetic": forbidden, "memory_grow": False,
                   "no_allocator_consumer": True, "dependencies": dependencies,
                   "guest_stack": json.loads((output / "guest-stack.log").read_text()),
                   "lattice": json.loads((output / "lattice-execute.log").read_text()),
                   "lattice_binary_sha256": hashlib.sha256(lattice_binary.read_bytes()).hexdigest(),
                   "fixed_lattice": json.loads((output / "fixed-lattice-execute.log").read_text()),
                   "fixed_lattice_binary_sha256": hashlib.sha256(fixed_binary.read_bytes()).hexdigest(),
                   "dyadic": json.loads((output / "dyadic-execute.log").read_text()),
                   "dyadic_binary_sha256": hashlib.sha256(dyadic_binary.read_bytes()).hexdigest(),
                   "dyadic_floating_arithmetic": dyadic_floating})
    bridge = client.parent / "wasm-real-bridge"
    manifest = bridge / "Cargo.toml"
    run("real-bridge-metadata", ["cargo", f"+{args.toolchain}", "metadata", "--locked", "--no-deps",
                                 "--format-version", "1", "--manifest-path", str(manifest)])
    metadata = json.loads((output / "real-bridge-metadata.log").read_text())
    required = metadata["packages"][0]["rust_version"]
    release = re.search(r"^release: (\d+\.\d+\.\d+)", (output / "compiler.log").read_text(), re.MULTILINE)
    if release is None:
        raise SystemExit("Missing rustc release in compiler.log")
    bridge_report = {"rustc": release[1], "rust_version": required, "executed": False}
    if tuple(map(int, release[1].split("."))) >= tuple(map(int, required.split("."))):
        options = ["--locked", "--manifest-path", str(manifest), "--target", target]
        run("real-bridge-build", ["cargo", f"+{args.toolchain}", "build", "--release", *options])
        run("real-bridge-lint", ["cargo", f"+{args.toolchain}", "clippy", *options, "--", "-D", "warnings"])
        binary_path = output / "interval_real_bridge.wasm"
        shutil.copyfile(output / "target" / target / "release/interval_real_bridge_wasm_conformance.wasm", binary_path)
        run("real-bridge-execute", ["node", str(bridge / "check.mjs"), str(binary_path), *runtime])
        bridge_report.update(json.loads((output / "real-bridge-execute.log").read_text()))
        bridge_report.update(executed=True, binary_sha256=hashlib.sha256(binary_path.read_bytes()).hexdigest())
    report["real_bridge"] = bridge_report
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"WASM {args.toolchain}: {report['cases']} exact cases passed; {report['binary_bytes']} bytes; no floating arithmetic or allocator")
    print(f"Fixed lattice: {report['fixed_lattice']['calls']} calls passed in two fresh instances")


if __name__ == "__main__":
    main()
