"""Inspect the one-attribute pure-merge inlining variant without timing it."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PHASE = E / "typed-api-prototype-v1921-inlined-cache-merge-codegen"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "inlined-cache-merge-generated-code-v1921"
    binary = ROOT / "target/inlined-cache-merge-generated-code-v1921/probe"
    assert not out.exists() and not binary.parent.exists()
    baseline = read(E / "cache-merge-inlining-baseline-v1920/baseline.json")
    before = json.loads(gzip.decompress((E / baseline["previous_diagnostic_source_record"]).read_bytes()))
    current = json.loads(gzip.decompress((PHASE / "source-hashes.json.gz").read_bytes()))
    changed = {name: digest for name, digest in current.items() if before[name] != digest}
    name = "crates/hyperreal/src/computable/node/bounds.rs"
    assert set(changed) == set(baseline["archived_sources_sha256"]) and len(changed) == 1
    assert len(current) == len(before) == 24746
    old = (E / "cache-merge-inlining-baseline-v1920/inputs" / name).read_text()
    start = old.index("impl BoundCache {")
    end = old.index("impl BoundInfo {", start)
    method = old[start:end]
    assert method.count("#[inline]") == 1
    assert (SOURCE / name).read_text() == old[:start] + method.replace("#[inline]", "#[inline(always)]", 1) + old[end:]
    out.mkdir()
    binary.parent.mkdir()
    (out / "reproduce.py").write_bytes(Path(__file__).read_bytes())
    fixture = E / "typed-api-prototype-v1850-bound-merge-atan-cost/probe.rs"
    probe = out / "probe.rs"
    probe.write_bytes(fixture.read_bytes())
    commands = []

    def run(label, command):
        start = time.monotonic()
        with (out / (label + ".stdout")).open("wb") as stdout, (out / (label + ".stderr")).open("wb") as stderr:
            result = subprocess.run(command, cwd=SOURCE, stdout=stdout, stderr=stderr)
        commands.append({"name": label, "command": command, "exit_code": result.returncode,
                         "seconds": time.monotonic() - start})
        (out / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        assert result.returncode == 0, (label, result.returncode)

    run("included-format", ["rustfmt", "--edition", "2024", "--check", str(SOURCE / name), str(SOURCE / "crates/hyperreal/src/computable/node/representation.rs")])
    run("workspace-format", ["cargo", "fmt", "--all", "--", "--check"])
    run("release-build", read(E / "typed-api-prototype-v1843-bound-merge-release-build/command.json")["command"])
    original = next(row["command"] for row in read(E / "typed-api-prototype-v1850-bound-merge-atan-cost/commands.json") if row["name"] == "build")
    command = [str(probe) if arg == str(fixture) else arg for arg in original]
    assert command[-2] == "-o"
    command[-1] = str(binary)
    run("probe-build", command)
    libraries = {arg.split("=", 1)[0]: arg.split("=", 1)[1]
                 for index, arg in enumerate(command) if index and command[index - 1] == "--extern"}
    symbols = subprocess.check_output(["nm", "-S", "--size-sort", "-C", str(binary)], text=True)
    (out / "symbols.txt.gz").write_bytes(gzip.compress(symbols.encode(), mtime=0))
    reference = read(E / "bound-merge-static-diagnosis-v1914/summary.json")
    functions = {}
    for label, row in reference["functions"]["baseline"].items():
        matches = [line.split(maxsplit=3) for line in symbols.splitlines()
                   if len(line.split(maxsplit=3)) == 4 and line.split(maxsplit=3)[3] == row["symbol"]]
        assert len(matches) == 1, label
        address, size, kind, _ = matches[0]
        start, length = int(address, 16), int(size, 16)
        run(label + "-assembly", ["objdump", "-d", "-C", "--no-show-raw-insn", f"--start-address={start}",
                                 f"--stop-address={start + length}", str(binary)])
        functions[label] = {"symbol": row["symbol"], "address": address, "bytes": length,
            "address_mod_64": start % 64, "retained_bytes": row["bytes"],
            "rejected_bytes": reference["functions"]["candidate"][label]["bytes"],
            "direct_match_bytes": read(E / baseline["previous_codegen"])["functions"][label]["bytes"]}
    summary = {"status": "Compiled static diagnostic only; no runtime or cost admission",
        "source_record_sha256": sha(PHASE / "source-hashes.json.gz"),
        "changed_since_direct_cache_match_sha256": changed,
        "only_merge_inlining_attribute_changed": True,
        "baseline": "cache-merge-inlining-baseline-v1920/baseline.json",
        "baseline_sha256": sha(E / "cache-merge-inlining-baseline-v1920/baseline.json"),
        "probe_sha256": sha(probe), "binary": str(binary), "binary_sha256": sha(binary),
        "binary_bytes": binary.stat().st_size,
        "libraries": {label: {"path": path, "sha256": sha(Path(path))} for label, path in libraries.items()},
        "functions": functions, "timed_processes": 0,
        "limits": "Static function sizes/layout can motivate a new frozen validation run but cannot override the v1857 rejection or establish timing, semantics, or whole-board parity.",
        "artifacts_sha256": {path.name: sha(path) for path in sorted(out.iterdir()) if path.is_file()}}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"functions": functions, "binary_bytes": summary["binary_bytes"], "timed_processes": 0}))


if __name__ == "__main__":
    main()
