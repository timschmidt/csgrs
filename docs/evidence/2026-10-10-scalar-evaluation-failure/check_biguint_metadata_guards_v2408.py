"""Reject executable defects in the actual shared BigUint preview files."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREVIEW = E / "biguint-metadata-candidate-v2406"
OUT = E / "biguint-metadata-guards-v2408"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2386-state.json"
    assert sha(checkpoint_path) == "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    support = E / "biguint-metadata-proof-v2407"
    assert read(support / "result.json")["exit_code"] == 0
    assert read(support / "stdout.json")["verification-results"]["verified"] == 462
    support_protocol = read(support / "protocol.json")
    inputs = support_protocol["preview_files_sha256"]
    assert all(sha(PREVIEW / n) == h for n, h in inputs.items())
    pin = read(SOURCE / "crates/hyperreal/verification/toolchain.json")
    tool = ROOT / "crates/hyperreal/target/verus" / pin["version"] / "verus-x86-linux"
    tooling = read(E / "typed-api-prototype-v2380-word-scan-verus/proof-reports/tooling.json")
    assert all(sha(tool / n) == h for n, h in tooling["tool_sha256"].items())
    mutations = []
    for width in [32, 64]:
        helper = f"vendor/num-bigint/src/biguint/metadata_u{width}.rs"
        methods = "vendor/num-bigint/src/biguint/metadata_impl.rs"
        for function, before, after in [
            ("WIDTH", f"pub(crate) const WIDTH: u64 = {width};", f"pub(crate) const WIDTH: u64 = {width + 1};"),
            ("first_different", "index += 1;", "index += 2;"),
            ("last_nonzero", "length -= 1;", "length -= 2;"),
            ("trim_zero_suffix", "words.truncate(length);", "words.truncate(0);"),
            ("zero_count", "Some(index as u64 * WIDTH + u64::from(count))", "Some(index as u64 * WIDTH + u64::from(count) + 1)"),
            ("one_count", "index as u64 * WIDTH + u64::from(count)", "index as u64 * WIDTH + u64::from(count) + 1"),
            ("bit_count", "words.len() as u64 * WIDTH - u64::from(count)", "words.len() as u64 * WIDTH + u64::from(count)"),
        ]:
            mutations.append({"width": width, "file": helper, "function": function, "before": before, "after": after})
        for function, before, after in [
            ("bits", "metadata::bit_count(self.data.as_slice())", "0"),
            ("trailing_zeros", "metadata::zero_count(self.data.as_slice())", "None"),
            ("trailing_ones", "metadata::one_count(self.data.as_slice())", "0"),
        ]:
            mutations.append({"width": width, "file": methods, "function": function, "before": before, "after": after})
    assert len(mutations) == 20
    OUT.mkdir()
    rows = []
    for index, mutation in enumerate([None, *mutations]):
        name = "baseline" if mutation is None else f"u{mutation['width']}-{mutation['function']}"
        directory = OUT / f"{index:02d}-{name}"
        directory.mkdir()
        entry = (PREVIEW / "proof.rs").read_text()
        files = {}
        if mutation:
            original_path = PREVIEW / mutation["file"]
            original = original_path.read_text()
            before, after = mutation["before"], mutation["after"]
            if mutation["function"] == "WIDTH":
                assert original.count(before) == 1
                changed = original.replace(before, after)
            else:
                anchor = "fn " + mutation["function"] + "("
                assert original.count(anchor) == 1
                start = original.index(anchor)
                indent = "    " if original_path.name == "metadata_impl.rs" else ""
                end = original.index("\n" + indent + "}\n", start) + len(indent) + 3
                assert original[start:end].count(before) == 1
                changed = original[:start] + original[start:end].replace(before, after) + original[end:]
            changed_path = directory / original_path.name
            changed_path.write_text(changed)
            anchor = f"mod biguint_u{mutation['width']} {{"
            start = entry.index(anchor)
            end = entry.index("\n}\n", start) + 3
            assert entry[start:end].count(str(original_path)) == 1
            entry = entry[:start] + entry[start:end].replace(str(original_path), str(changed_path)) + entry[end:]
            files[changed_path.name] = sha(changed_path)
        path = directory / "probe.rs"
        path.write_text(entry)
        files[path.name] = sha(path)
        command = [str(tool / "verus"), "--edition=2024", "--crate-type=lib", "--no-cheating", "--output-json", str(path)]
        rows.append({"index": index, "name": name, "mutation": mutation, "directory": directory.name,
                     "files_sha256": files, "command": command})
    protocol = {
        "status": "Frozen before running the shared-source baseline and twenty mutations",
        "runner_sha256": sha(Path(__file__)), "preview": PREVIEW.name, "preview_files_sha256": inputs,
        "support_result_sha256": sha(support / "result.json"), "support_protocol_sha256": sha(support / "protocol.json"),
        "retained_checkpoint": checkpoint_path.name, "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "tool_sha256": tooling["tool_sha256"], "runs": rows,
        "scope": "One executable width defect, six helper defects and three public-method defects for each native word width. Each mutation edits an exact shared production file and replaces only its selected-width include. All twenty must fail with proof errors after a clean unmutated baseline; frontend rejection is insufficient. No retained proof/runtime admission or source mutation follows from these support controls.",
    }
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    results = []
    for row in rows:
        directory = OUT / row["directory"]
        assert all(sha(directory / n) == h for n, h in row["files_sha256"].items())
        start = time.monotonic()
        result = subprocess.run(row["command"], cwd=ROOT,
            env=os.environ | {"PYTHONDONTWRITEBYTECODE": "1"}, capture_output=True, text=True)
        (directory / "stdout.json").write_text(result.stdout)
        (directory / "stderr.log").write_text(result.stderr)
        verification = json.loads(result.stdout)["verification-results"]
        if row["index"] == 0:
            passed = result.returncode == 0 and verification["verified"] == 462 and verification["errors"] == 0 and not result.stderr
        else:
            passed = (result.returncode != 0 and verification.get("errors", 0) > 0
                and verification.get("verified", 0) > 0 and not verification.get("encountered-vir-error", False)
                and any(s in result.stderr for s in ["postcondition not satisfied", "assertion failed",
                    "precondition not satisfied", "requires not satisfied", "invariant not satisfied",
                    "arithmetic underflow", "arithmetic overflow"]))
        data = {"index": row["index"], "name": row["name"], "exit_code": result.returncode,
            "seconds": time.monotonic() - start, "input_files_sha256": row["files_sha256"],
            "stdout_sha256": sha(directory / "stdout.json"), "stderr_sha256": sha(directory / "stderr.log"),
            "verification_results": verification, "gate_passed": passed}
        (directory / "result.json").write_text(json.dumps(data, indent=2) + "\n")
        results.append(data)
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        assert all(sha(PREVIEW / n) == h for n, h in inputs.items())
        assert all(sha(tool / n) == h for n, h in tooling["tool_sha256"].items())
        assert all(sha(directory / n) == h for n, h in row["files_sha256"].items())
        (OUT / "result.json").write_text(json.dumps({
            "protocol_sha256": sha(OUT / "protocol.json"), "runs": results,
            "complete": len(results) == len(rows) and passed, "source_unchanged": True,
            "main_unchanged": True, "preview_unchanged": True, "tooling_unchanged": True,
            "retained_proof_coverage_changed": False,
        }, indent=2) + "\n")
        assert passed, row["name"]
        print(json.dumps({"name": row["name"], "gate_passed": passed, "exit_code": result.returncode}), flush=True)


if __name__ == "__main__":
    main()
