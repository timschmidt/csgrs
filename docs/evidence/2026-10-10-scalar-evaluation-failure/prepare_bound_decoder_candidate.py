"""Freeze the retained decoder inputs and rebuild immutable cost probes for layout comparison."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / "bound-decoder-baseline-v2168"
    binaries = ROOT / "target/bound-decoder-baseline-v2168"
    assert not out.exists() and not binaries.exists()
    restoration_path = E / "scalar-retained-restoration-review-v2166.json"
    restoration = read(restoration_path)
    source_record = E / restoration["source_record"]
    assert sha(source_record) == restoration["source_record_sha256"]
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(sources) == 24746 and all(sha(SOURCE / n) == h for n, h in sources.items())
    assert len(guard) == 1167 and all(sha(ROOT / n) == h for n, h in guard.items())
    support_path = E / "decoder-expression-support-v2167/summary.json"
    support = read(support_path)
    assert support["source_record_sha256"] == restoration["source_record_sha256"]
    assert [(r["name"], r["exit_code"]) for r in support["results"]] == [("optional_magnitude", 1), ("masked_tag", 0)]
    assert "then_some` is not supported" in (support_path.parent / "optional_magnitude.stderr").read_text()
    assert "2 verified, 0 errors" in (support_path.parent / "masked_tag.stdout").read_text()
    planned = ["crates/hyperreal/src/computable/node/representation.rs",
               "crates/hyperreal/verification/cache_encoding_model.rs",
               "crates/hyperreal/verification/scope.json",
               "crates/hyperreal/verification/test_cache_rejections.py",
               "crates/hyperreal/verification/README.md"]
    registry = read(E / "integer-coarse-zero-proof-validation-v2083.json")["proof_and_guard_inputs_sha256"]
    proof_inputs = {n: sources[n] for n in registry}
    assert len(proof_inputs) == 44
    families = read(E / "integer-coarse-zero-initial-cost-review-v2100.json")["families"]
    commands = []
    for family in ["atan", "acos", "integer", "inverse_trig", "normal_form", "default", "algebraic"]:
        item = families[family]
        phase = E / item["phases"]["baseline"]
        prior_build = next(row for row in read(phase / "commands.json") if row["name"] == "build")
        command = list(prior_build["command"])
        original_binary = Path(command[command.index("-o") + 1])
        assert sha(original_binary) == item["binary_sha256"]["baseline"]
        command[command.index("-o") + 1] = str(binaries / family)
        libraries = {}
        for i, arg in enumerate(command):
            if arg == "--extern":
                name = command[i + 1].split("=", 1)[1]
                libraries[name] = sha(Path(name))
        commands.append({"family": family, "command": command, "libraries_sha256": libraries,
            "original_binary": str(original_binary), "original_binary_sha256": sha(original_binary),
            "original_build_command": prior_build["command"], "probe": str((phase / "probe.rs").relative_to(E)),
            "probe_sha256": sha(phase / "probe.rs")})
    out.mkdir()
    binaries.mkdir()
    for name in planned:
        p = out / "inputs" / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes((SOURCE / name).read_bytes())
    protocol = {"status": "Retained decoder baseline frozen before any source edit or probe compilation",
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_record": restoration["source_record"], "source_record_sha256": sha(source_record),
        "retained_runtime_checkpoint": restoration["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": restoration["retained_runtime_checkpoint_sha256"],
        "restoration_record": restoration_path.name, "restoration_record_sha256": sha(restoration_path),
        "expression_support": str(support_path.relative_to(E)), "expression_support_sha256": sha(support_path),
        "generator_sha256": sha(Path(__file__)), "planned_changed_files": planned,
        "archived_sources_sha256": {n: sources[n] for n in planned}, "proof_and_guard_inputs_sha256": proof_inputs,
        "build_commands": commands, "source_files_verified": len(sources), "main_files_verified": len(guard),
        "proposal": "Include the actual AtomicFacts::decode_bound body in the pinned Verus target with an exact all-u64 contract matching bound_word. Replace unsupported bool::then_some with an equivalent pure if expression, preserving every tag/sign/optional signed exponent/exact flag. Add ghost-only mask bounds, connect actual decoding to existing round-trip/field-update lemmas, and add independent negative guards. Keep all publication/caller/runtime test bodies unchanged.",
        "next_checks": [
            "Run the full conditional proof target, complete mutation suite and acceptance checks; producer truth and atomic ordering remain separate obligations.",
            "Compare compiled consumer probes against these freshly rebuilt retained binaries before deciding the extent of fresh cost measurements. Identical executable bytes may reuse their existing workload receipts; any changed binary still requires the full original fixtures and caps. Never mark an old failed gate passed.",
            "Run the unchanged runtime checks and capped/numerical/public replay qualifications proportionate to the actual source/compiled changes. Preserve all full-size fixtures and all earlier failures. A proof alone cannot justify a runtime regression or close the fixed pre-Verus acceptance baseline.",
        ],
        "limits": "This preparation compiles seven probes but runs no timing or query processes. Copied expression probes only establish verifier support; the actual decoder is not yet verified. All thirteen whole-crate groups and main/native/full-board admission remain open."}
    (out / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    results = []
    for item in commands:
        for name, digest in item["libraries_sha256"].items():
            assert sha(Path(name)) == digest
        assert sha(E / item["probe"]) == item["probe_sha256"]
        start = time.monotonic()
        result = subprocess.run(item["command"], cwd=ROOT, capture_output=True, timeout=120)
        (out / (item["family"] + ".stdout")).write_bytes(result.stdout)
        (out / (item["family"] + ".stderr")).write_bytes(result.stderr)
        binary = Path(item["command"][item["command"].index("-o") + 1])
        row = {"family": item["family"], "exit_code": result.returncode, "seconds": time.monotonic() - start,
               "binary": str(binary), "binary_sha256": sha(binary) if binary.exists() else None}
        row["matches_historical_executable_bytes"] = row["binary_sha256"] == item["original_binary_sha256"]
        results.append(row)
        (out / "build-results.json").write_text(json.dumps(results, indent=2) + "\n")
        assert result.returncode == 0 and result.stderr == b"", item["family"]
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    for item in commands:
        assert all(sha(Path(n)) == h for n, h in item["libraries_sha256"].items())
        assert sha(Path(item["original_binary"])) == item["original_binary_sha256"]
    summary = {"protocol_sha256": sha(out / "protocol.json"), "source_record_sha256": sha(source_record),
        "builds_passed": len(results), "results": results, "timing_processes_run": 0,
        "source_files_verified": len(sources), "main_files_verified": len(guard),
        "runtime_source_changed": False, "actual_decoder_verified": False,
        "artifacts_sha256": {str(p.relative_to(out)): sha(p) for p in sorted(out.rglob("*")) if p.is_file()}}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"builds_passed": len(results), "historical_binary_matches": sum(r["matches_historical_executable_bytes"] for r in results), "runtime_source_changed": False}))


if __name__ == "__main__":
    main()
