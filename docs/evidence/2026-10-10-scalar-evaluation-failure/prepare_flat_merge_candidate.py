"""Preserve the rejected experiment and freeze a single caller-branching variant."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    diagnosis = E / "bound-merge-static-diagnosis-v1914"
    baseline = E / "flat-merge-baseline-v1915"
    assert not diagnosis.exists() and not baseline.exists()
    rejection_path = E / "bound-merge-cost-rejection-v1857.json"
    rejection = read(rejection_path)
    assert not rejection["cost_gate_passed"] and not rejection["runtime_retained"]
    source_record = E / rejection["source_record"]
    assert sha(source_record) == rejection["source_record_sha256"]
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    for name, digest in sources.items():
        assert sha(SOURCE / name) == digest, name
    proof_path = E / "bound-merge-proof-validation-v1840.json"
    proof = read(proof_path)
    assert proof["source_record_sha256"] == rejection["source_record_sha256"]
    retained_path = E / rejection["retained_runtime_checkpoint"]
    assert sha(retained_path) == rejection["retained_runtime_checkpoint_sha256"]
    paired = read(E / "bound-merge-preconditioned-cost-v1857/summary.json")
    binaries = paired["provenance"]["atan"]
    assert all(sha(Path(path)) == binaries["binary_sha256_at_paired_run"][which]
               for which, path in binaries["binaries"].items())
    diagnosis.mkdir()
    (diagnosis / "reproduce.py").write_bytes(Path(__file__).read_bytes())
    functions = {
        "publish": "<hyperreal::computable::node::AtomicFacts>::publish_bound",
        "approx": "<hyperreal::computable::node::Computable>::approx_signal",
        "cache": "<hyperreal::computable::node::EvaluationCache>::at_precision",
        "main": "probe::main",
    }
    rows, commands = {}, []
    for which, path in binaries["binaries"].items():
        command = ["nm", "-S", "--size-sort", "-C", path]
        symbols = subprocess.check_output(command, text=True)
        commands.append(command)
        (diagnosis / f"{which}-symbols.txt.gz").write_bytes(gzip.compress(symbols.encode(), mtime=0))
        rows[which] = {}
        for label, name in functions.items():
            selected = [row.split(maxsplit=3) for row in symbols.splitlines()
                        if len(row.split(maxsplit=3)) == 4 and row.split(maxsplit=3)[3] == name]
            assert len(selected) == 1, (which, label)
            address, size, kind, _ = selected[0]
            start, length = int(address, 16), int(size, 16)
            command = ["objdump", "-d", "-C", "--no-show-raw-insn", f"--start-address={start}",
                       f"--stop-address={start + length}", path]
            output = subprocess.check_output(command, text=True)
            commands.append(command)
            assembly = diagnosis / f"{which}-{label}.asm"
            assembly.write_text(output)
            rows[which][label] = {"symbol": name, "address": address, "bytes": length,
                "address_mod_64": start % 64, "assembly": assembly.name, "assembly_sha256": sha(assembly)}
    assert rows["baseline"]["publish"]["bytes"] == 517
    assert rows["candidate"]["publish"]["bytes"] == 860
    assert rows["baseline"]["approx"]["bytes"] == rows["candidate"]["approx"]["bytes"] == 4982
    assert rows["baseline"]["cache"]["bytes"] == rows["candidate"]["cache"]["bytes"] == 1113
    assert rows["baseline"]["main"]["bytes"] == rows["candidate"]["main"]["bytes"] == 8659
    assert all(sha(Path(path)) == binaries["binary_sha256_at_paired_run"][which]
               for which, path in binaries["binaries"].items())
    inputs = [
        "crates/hyperreal/src/computable/node/approximation_queries.rs",
        "crates/hyperreal/src/computable/node/structural_analysis.rs",
        "crates/hyperreal/src/computable/node/representation.rs",
        "crates/hyperreal/src/computable/node/bounds.rs",
    ]
    diag = {
        "status": "Static diagnostic only; no new timing samples or causal conclusion",
        "source_record_sha256": rejection["source_record_sha256"],
        "rejection_record": rejection_path.name, "rejection_record_sha256": sha(rejection_path),
        "binary_provenance": binaries, "functions": rows, "commands": commands,
        "source_inputs_sha256": {name: sha(SOURCE / name) for name in inputs},
        "fixture": "typed-api-prototype-v1850-bound-merge-atan-cost/probe.rs",
        "fixture_sha256": sha(E / "typed-api-prototype-v1850-bound-merge-atan-cost/probe.rs"),
        "observations": [
            "The cached fixtures finish each root at the measured precision before timing. In the inspected source, a completed cached_at_precision result returns before publication. This relies on that fixture/setup contract and is not a general reachability proof.",
            "The bound-publication function grows from 517 to 860 bytes. The approximation, cache and probe-main function sizes are unchanged. The cache function moves from address modulo 64 equal to 48 to modulo 64 equal to zero. Equal size does not establish equivalent instructions or timing.",
            "The generated publication code has a different branch/loop structure. A source variant will preserve the old top-level Invalid-or-Unknown branch before delegating valid known bounds to the proved merge method. This is a hypothesis for reducing generated code, not an attribution of the cached timing increases.",
        ],
    }
    diag["artifacts_sha256"] = {path.name: sha(path) for path in sorted(diagnosis.iterdir()) if path.is_file()}
    (diagnosis / "summary.json").write_text(json.dumps(diag, indent=2) + "\n")
    baseline.mkdir()
    archived = {}
    for name in proof["changed_sources_sha256"]:
        path = baseline / "inputs" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((SOURCE / name).read_bytes())
        archived[name] = sha(path)
    assert archived == proof["changed_sources_sha256"] and len(archived) == 6
    expected = read(E.parent / "2026-10-10-artifact-numeric-keys/retention.json")["staged_verification"]["preexisting_status"]
    status = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=all"], cwd=ROOT, text=True).splitlines()
    assert [line for line in status if not line[3:].startswith(str(E.relative_to(ROOT)) + "/")] == expected
    data = {
        "status": "Frozen before editing the single caller-branching variant",
        "retained_runtime_checkpoint": retained_path.name, "retained_runtime_checkpoint_sha256": sha(retained_path),
        "rejected_source_record": rejection["source_record"], "rejected_source_record_sha256": rejection["source_record_sha256"],
        "rejected_source_files_verified": len(sources), "archived_rejected_sources_sha256": archived,
        "rejection_record": rejection_path.name, "rejection_record_sha256": sha(rejection_path),
        "static_diagnosis": str((diagnosis / "summary.json").relative_to(E)),
        "static_diagnosis_sha256": sha(diagnosis / "summary.json"),
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "change_boundary": "Only the combined-bound selection expression in AtomicFacts::publish_bound: preserve Invalid-or-Unknown as the original top-level incoming-value branch, then use BoundInfo::merge for other valid retained bounds. The pure merge body, all ghost lemmas, diagnostics, atomic load/test hook, encode/mask/CAS suffix, tests and fixtures remain byte-identical to the rejected source.",
        "initial_diagnostic_gate": "First format and release-build the candidate and compile the unchanged atan probe without executing it. Inspect generated publication code against both v1836 and the rejected variant. Static size/layout is not a timing pass.",
        "required_gates": "Before retention, verify the actual production proof source, acceptance/negative checks, nine runtime checks, all seven unchanged cost families with the same 52 full-size fixtures, caps, alternating pairs and predeclared family-repeat rule, then unchanged capped/numerical/public replays and receipt/fact audits. A fresh comparison uses v1836 as baseline and preserves the v1857 rejection. No threshold/cap/fixture reduction or reset of older fixed baselines is permitted.",
        "main_runtime_admitted": False, "original_worktree_entries_preserved": len(expected),
    }
    (baseline / "baseline.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"static_diagnosis": diagnosis.name, "baseline": baseline.name,
        "source_files_verified": len(sources), "archived_rejected_source_files": len(archived),
        "retained_runtime_checkpoint": retained_path.name}))


if __name__ == "__main__":
    main()
