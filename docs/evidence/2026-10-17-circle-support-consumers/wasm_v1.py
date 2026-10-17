"""Exercise actual generated JS/TS consumers with the unchanged candidate."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import Runner, V, W, dump, read, sha

assert read(V / "consumer-validation-v1/result.json")["complete"]
assert read(V / "consumer-validation-v1/result.json")["cargo_target_closed"]
assert read(V / "consumer-validation-v1/result.json")["sources_unchanged"]
runtime = Path("/run/user/1000/csgrs-circle-support-wasm-v1")
assert set(p.name for p in runtime.iterdir()) == {"write-control"}
old = W.parent / "export-projection-contracts"
tool = read(old / "wasm-tools-v1/result.json")
assert tool["exit_code"] == 0 and tool["version"] == "wasm-bindgen 0.2.126"
assert sha(tool["binary"]) == tool["binary_sha256"]
typescript = old / "wasm-source-v1/tests/wasm/node_modules/typescript"
assert read(typescript / "package.json")["version"] == "5.7.3"
pins = {str(p): sha(p) for p in typescript.rglob("*") if p.is_file()}
pins[tool["binary"]] = tool["binary_sha256"]
r = Runner("wasm-validation-v1", runtime / "target-v1", Path(__file__), virtual_gib=64, runtime_root=runtime)
dump(r.output / "tools.json", {
    "read_only_sha256": pins, "wasm_bindgen": tool["binary"], "tsc": str(typescript / "bin/tsc"),
    "virtual_address_space_note": "64 GiB permits V8 address reservations; this is not a heap-byte bound.",
    "runtime_control_sha256": sha(runtime / "write-control"),
})
r.run("shell-syntax", ["bash", "-n", str(r.source / "scripts/test-wasm-bindings.sh")])
r.run("wasm-consumers", ["bash", str(r.source / "scripts/test-wasm-bindings.sh")],
      extra={"WASM_BINDGEN": tool["binary"], "TSC": str(typescript / "bin/tsc")})
r.run("wasm-clippy", [
    "cargo", "clippy", "--offline", "--locked", "--profile", "wasm", "--target", "wasm32-unknown-unknown",
    "--no-default-features", "--features", "wasm", "--lib", "--", "-D", "warnings",
])
assert all(sha(p) == h for p, h in pins.items())
artifacts = [
    r.target / "wasm32-unknown-unknown/wasm/csgrs.wasm",
    *sorted((r.target / "wasm-bindings-node").glob("*")),
]
raise SystemExit(r.finish({
    "read_only_tools_unchanged": True,
    "artifacts": {str(p): {"sha256": sha(p), "bytes": p.stat().st_size} for p in artifacts if p.is_file()},
}))
