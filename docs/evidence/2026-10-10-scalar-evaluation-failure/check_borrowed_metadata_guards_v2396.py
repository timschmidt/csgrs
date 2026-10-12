"""Reject executable defects in the unadmitted borrowed-metadata prototype."""
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
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    out = E / "borrowed-metadata-guards-v2396"
    assert not out.exists()
    checkpoint_path = E / "typed-api-prototype-v2386-state.json"
    assert sha(checkpoint_path) == "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    support = E / "borrowed-metadata-probe-v2395"
    assert read(support / "result.json")["exit_code"] == 0
    assert read(support / "stdout.json")["verification-results"]["verified"] == 454
    proposal = E / "borrowed-metadata-proposal-v2395.rs"
    assert sha(proposal) == read(support / "protocol.json")["probe_sha256"]
    pin = read(SOURCE / "crates/hyperreal/verification/toolchain.json")
    tool = ROOT / "crates/hyperreal/target/verus" / pin["version"] / "verus-x86-linux"
    tooling = read(E / "typed-api-prototype-v2380-word-scan-verus/proof-reports/tooling.json")
    assert all(sha(tool / n) == h for n, h in tooling["tool_sha256"].items())
    original = proposal.read_text()
    mutations = []
    for width in [32, 64]:
        mutations += [
            (f"first_different_u{width}", "index += 1;", "index += 2;"),
            (f"last_nonzero_u{width}", "length -= 1;", "length -= 2;"),
            (f"trim_zero_suffix_u{width}", "words.truncate(length);", "words.truncate(0);"),
            (f"zero_count_u{width}", f"Some(index as u64 * {width} + u64::from(count))",
             f"Some(index as u64 * {width} + u64::from(count) + 1)"),
            (f"one_count_u{width}", f"index as u64 * {width} + u64::from(count)",
             f"index as u64 * {width} + u64::from(count) + 1"),
            (f"bit_count_u{width}", f"words.len() as u64 * {width} - u64::from(count)",
             f"words.len() as u64 * {width} + u64::from(count)"),
        ]
    out.mkdir()
    rows = []
    for index, mutation in enumerate([None, *mutations]):
        text = original
        name = "baseline"
        if mutation:
            name, before, after = mutation
            anchor = "fn " + name + "("
            assert text.count(anchor) == 1
            start = text.index(anchor)
            end = text.index("\n}\n", start) + 3
            assert text[start:end].count(before) == 1
            text = text[:start] + text[start:end].replace(before, after) + text[end:]
        directory = out / f"{index:02d}-{name}"
        directory.mkdir()
        path = directory / "probe.rs"
        path.write_text(text)
        command = [str(tool / "verus"), "--edition=2024", "--crate-type=lib", "--no-cheating",
                   "--output-json", str(path)]
        rows.append({"index": index, "name": name, "mutation": mutation,
            "directory": directory.name, "source_sha256": sha(path), "command": command})
    protocol = {"status": "Frozen before running the prototype baseline and twelve mutations",
        "runner_sha256": sha(Path(__file__)), "proposal_sha256": sha(proposal),
        "retained_checkpoint": checkpoint_path.name, "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"],
        "tool_sha256": tooling["tool_sha256"], "runs": rows,
        "scope": "Twelve executable-body defects must fail proof verification after one unmutated baseline. Rejecting a frontend error is insufficient. These controls apply only to the new standalone prototype and do not increase retained production contracts, model theorems or guard counts. No SOURCE/main/runtime changes, new assumptions, performance claim or dependency-body proof."}
    (out / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    results = []
    for row in rows:
        directory = out / row["directory"]
        start = time.monotonic()
        result = subprocess.run(row["command"], cwd=ROOT,
            env=os.environ | {"PYTHONDONTWRITEBYTECODE": "1"}, capture_output=True, text=True)
        (directory / "stdout.json").write_text(result.stdout)
        (directory / "stderr.log").write_text(result.stderr)
        verification = json.loads(result.stdout)["verification-results"]
        if row["index"] == 0:
            passed = result.returncode == 0 and verification["verified"] == 454 and verification["errors"] == 0
        else:
            passed = (result.returncode != 0 and verification.get("errors", 0) > 0
                and verification.get("verified", 0) > 0 and not verification.get("encountered-vir-error", False)
                and any(s in result.stderr for s in ["postcondition not satisfied", "assertion failed",
                    "precondition not satisfied", "requires not satisfied", "invariant not satisfied",
                    "arithmetic underflow", "arithmetic overflow"]))
        data = {"index": row["index"], "name": row["name"], "exit_code": result.returncode,
            "seconds": time.monotonic() - start, "source_sha256": sha(directory / "probe.rs"),
            "stdout_sha256": sha(directory / "stdout.json"), "stderr_sha256": sha(directory / "stderr.log"),
            "verification_results": verification, "gate_passed": passed}
        (directory / "result.json").write_text(json.dumps(data, indent=2) + "\n")
        results.append(data)
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        assert all(sha(tool / n) == h for n, h in tooling["tool_sha256"].items())
        (out / "result.json").write_text(json.dumps({"protocol_sha256": sha(out / "protocol.json"),
            "runs": results, "complete": len(results) == len(rows) and passed,
            "source_unchanged": True, "main_unchanged": True, "tooling_unchanged": True,
            "retained_proof_coverage_changed": False}, indent=2) + "\n")
        assert passed, row["name"]
        print(json.dumps({"name": row["name"], "gate_passed": passed, "exit_code": result.returncode}), flush=True)


if __name__ == "__main__":
    main()
