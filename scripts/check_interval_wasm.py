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
    # wasm-dis emits folded instructions. Skip quoted names and data, which
    # can themselves contain text that looks like an instruction.
    tokens = re.finditer(r'"(?:\\.|[^"\\])*"|;;[^\n]*|\(([if]\d+(?:x\d+)?)\.([a-z][a-z0-9_]*)(?=[\s)])', assembly)
    forbidden = set()
    for token in tokens:
        shape, operation = token.groups()
        if shape is None:
            continue
        opcode = f"{shape}.{operation}"
        if opcode in {"i32.reinterpret_f32", "i64.reinterpret_f64", "f32.reinterpret_i32", "f64.reinterpret_i64"}:
            continue
        if shape in {"f32x4", "f64x2"} and operation in {"extract_lane", "replace_lane", "splat"}:
            continue  # Transfer bits between same-width lanes and scalars.
        if shape.startswith("f") or re.search(r"_f\d+(?:x\d+)?(?:_|$)", operation):
            forbidden.add(opcode)
    return sorted(forbidden)


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
    control = output / "codegen-control.wasm"
    run("assemble-codegen-control", ["wasm-as", "--enable-simd", str(client / "codegen-control.wat"), "-o", str(control)])
    control_wat = output / "codegen-control.wat"
    run("disassemble-codegen-control", ["wasm-dis", str(control), "-o", str(control_wat)])
    detected = floating_operations(control_wat.read_text())
    if detected != ["f32.add", "f32x4.add", "f64x2.convert_low_i32x4_s", "i32x4.trunc_sat_f64x2_s_zero"]:
        raise SystemExit(f"Unexpected floating operations in the codegen control: {detected}")
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
    report.update({"toolchain": args.toolchain, "target": target, "codegen_control": detected,
                   "binary_bytes": binary.stat().st_size,
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
    if runtime:
        runner = root / "crates/hyperlattice-interval/conformance/run-wasi-test.mjs"
        test_command = ["cargo", f"+{args.toolchain}", "test", "--locked", "--release",
                        "-p", "hyperlattice-interval", "--no-default-features", "--target", target,
                        "--config", f"target.{target}.runner={json.dumps(['node', str(runner)])}"]
        passed = {}
        for name, features, selection in [
            ("arithmetic", "fixed,dyadic,float-enclosure-wasip1", ["--lib", "--tests"]),
            ("replay", "fixed,serde", ["--test", "replay"]),
            ("shape_replay", "fixed,serde", ["--test", "shape_replay"]),
        ]:
            log = f"wasi-lattice-{name}"
            # A guest panic can trap before libtest flushes its captured diagnostics.
            run(log, [*test_command, "--features", features, *selection,
                      "--", "--test-threads=1", "--nocapture"])
            transcript = (output / f"{log}.log").read_text()
            passed[name] = sum(map(int, re.findall(r"^test result: ok\. (\d+) passed;", transcript, re.MULTILINE)))
            if not passed[name]:
                raise SystemExit(f"No passing guest tests in {log}")
        report["lattice_guest_tests"] = passed
        print(f"WASI Preview 1 lattice tests: {sum(passed.values())} passed")
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"WASM {args.toolchain}: {report['cases']} arithmetic cases passed; {report['binary_bytes']} bytes; no floating arithmetic or allocator")
    print(f"Fixed lattice: {report['fixed_lattice']['calls']} calls passed in two fresh instances")


if __name__ == "__main__":
    main()
