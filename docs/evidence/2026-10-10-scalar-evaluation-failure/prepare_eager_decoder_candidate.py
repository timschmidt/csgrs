"""Freeze and apply an eager-payload decoder experiment from the exact retained source."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
OUT = E / "eager-bound-decoder-baseline-v2257"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
digest = lambda data: hashlib.sha256(data).hexdigest()
snapshot = lambda path: json.loads(gzip.decompress(path.read_bytes()))


def main():
    assert len(sys.argv) == 2 and sys.argv[1] in ["prepare", "apply"]
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert len(guard) == 1167 and all(sha(ROOT / n) == h for n, h in guard.items())
    if sys.argv[1] == "apply":
        protocol = read(OUT / "protocol.json")
        assert sha(Path(__file__)) == protocol["helper_sha256"]
        assert not (OUT / "application.json").exists()
        sources = snapshot(E / protocol["retained_source_record"])
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert sha(OUT / "proposal.patch") == protocol["proposal_patch_sha256"]
        for n, h in protocol["candidate_sources_sha256"].items():
            assert sha(OUT / "candidate" / n) == h
        subprocess.run(["git", "apply", "--check", str(OUT / "proposal.patch")], cwd=SOURCE, check=True)
        for name in protocol["candidate_sources_sha256"]:
            (SOURCE / name).write_bytes((OUT / "candidate" / name).read_bytes())
        expected = snapshot(OUT / "candidate-source-hashes.json.gz")
        assert all(sha(SOURCE / n) == h for n, h in expected.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        (OUT / "application.json").write_text(json.dumps({"source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz"),
            "source_files_verified": len(expected), "changed_files": 5, "main_files_verified": len(guard),
            "validation_started": False, "runtime_retained": False}, indent=2) + "\n")
        print(json.dumps({"changed_files": 5, "source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz"),
                          "runtime_retained": False}))
        return
    assert not OUT.exists()
    restoration_path = E / "decoder-retained-restoration-review-v2255.json"
    restoration = read(restoration_path)
    analysis_path = E / "bound-decoder-static-differences-v2256/summary.json"
    analysis = read(analysis_path)
    rejected_path = E / "bound-decoder-cost-rejection-v2247.json"
    rejected = read(rejected_path)
    assert not rejected["cost_gate_passed"] and not rejected["runtime_retained"]
    assert analysis["new_workload_processes"] == 0
    source_record = E / restoration["source_record"]
    expected = restoration["source_record_sha256"]
    assert sha(source_record) == expected == analysis["current_source_record_sha256"]
    sources = snapshot(source_record)
    assert len(sources) == 24746 and all(sha(SOURCE / n) == h for n, h in sources.items())
    prior_proof = read(E / "bound-decoder-proof-validation-v2171.json")
    archived = E / "decoder-retained-restoration-v2248/rejected"
    candidate = {}
    for name, value in prior_proof["changed_sources_sha256"].items():
        assert sha(archived / name) == value
        candidate[name] = (archived / name).read_text()
    representation = "crates/hyperreal/src/computable/node/representation.rs"
    before = ('                let msd = if value & Self::MSD_PRESENT != 0 {\n'
              '                    Some(((value >> Self::MSD_SHIFT) as u32) as i32)\n')
    after = ('                let magnitude = ((value >> Self::MSD_SHIFT) as u32) as i32;\n'
             '                let msd = if value & Self::MSD_PRESENT != 0 {\n'
             '                    Some(magnitude)\n')
    assert candidate[representation].count(before) == 1
    candidate[representation] = candidate[representation].replace(before, after)
    tests = "crates/hyperreal/verification/test_cache_rejections.py"
    old_mutation = ("    ('decode_bound', 'Some(((value >> Self::MSD_SHIFT) as u32) as i32)',\n"
                    "     'Some((((value >> Self::MSD_SHIFT) as u32) & 0x7fff_ffff) as i32)'),")
    new_mutation = ("    ('decode_bound', 'let magnitude = ((value >> Self::MSD_SHIFT) as u32) as i32;',\n"
                    "     'let magnitude = (((value >> Self::MSD_SHIFT) as u32) & 0x7fff_ffff) as i32;'),")
    assert candidate[tests].count(old_mutation) == 1
    candidate[tests] = candidate[tests].replace(old_mutation, new_mutation)
    assert len(candidate) == 5
    candidate_hashes = {n: digest(t.encode()) for n, t in candidate.items()}
    assert sum(candidate_hashes[n] != prior_proof["changed_sources_sha256"][n] for n in candidate) == 2
    baseline_protocol_path = E / "bound-decoder-binary-comparison-protocol-v2181.json"
    baseline_protocol = read(baseline_protocol_path)
    for row in baseline_protocol["build_commands"]:
        assert sha(Path(row["baseline_binary"])) == row["baseline_binary_sha256"]
        assert sha(E / row["probe"]) == row["probe_sha256"]
    OUT.mkdir()
    patch = []
    for name, text in candidate.items():
        for folder, content in [("retained", (SOURCE / name).read_bytes()), ("candidate", text.encode())]:
            path = OUT / folder / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
        patch.extend(difflib.unified_diff((SOURCE / name).read_text().splitlines(True), text.splitlines(True),
                                        fromfile="a/" + name, tofile="b/" + name))
    (OUT / "proposal.patch").write_text("".join(patch))
    proposed = sources | candidate_hashes
    (OUT / "candidate-source-hashes.json.gz").write_bytes(gzip.compress(
        (json.dumps(proposed, sort_keys=True) + "\n").encode(), mtime=0))
    protocol = {"status": "Frozen before applying the eager-payload candidate or checking its generated code",
        "helper_sha256": sha(Path(__file__)), "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "retained_source_record": str(source_record.relative_to(E)), "retained_source_record_sha256": expected,
        "retained_runtime_checkpoint": restoration["retained_runtime_checkpoint"],
        "retained_runtime_checkpoint_sha256": restoration["retained_runtime_checkpoint_sha256"],
        "restoration_record": restoration_path.name, "restoration_record_sha256": sha(restoration_path),
        "static_analysis": str(analysis_path.relative_to(E)), "static_analysis_sha256": sha(analysis_path),
        "rejected_candidate": rejected_path.name, "rejected_candidate_sha256": sha(rejected_path),
        "retained_sources_sha256": {n: sources[n] for n in candidate}, "candidate_sources_sha256": candidate_hashes,
        "source_record": str((OUT / "candidate-source-hashes.json.gz").relative_to(E)),
        "source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz"),
        "proposal_patch_sha256": sha(OUT / "proposal.patch"),
        "baseline_binary_protocol": baseline_protocol_path.name, "baseline_binary_protocol_sha256": sha(baseline_protocol_path),
        "proposal": "Restore the prior actual-decoder contract and five field lemmas, with the magnitude payload computed before selecting Some/None. Adjust only the signed-payload mutation anchor to the new local assignment. No field, tag, sign, cast, flag, publication operation or call site changes.",
        "early_gate": "Check formatting, the complete conditional proof, a fresh release library build and all seven unchanged static consumer probes. No workload timing is authorized by this early gate. Complete executable identity alone permits the previously described receipt-reuse route; any remaining byte differences require separate review before freezing further runtime gates. All prior cost failures remain failed.",
        "limits": "This is an isolated unretained experiment. The all-u64 decoder contract does not establish incoming certificate truth, existence, atomic/caller ordering, aggregate resource bounds or main/native/full-board admission. The fixed pre-Verus baseline, every property/target status and all thirteen obligation groups remain unchanged."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    print(json.dumps({"frozen_files": 5, "changes_from_rejected_source": 2,
                      "source_record_sha256": protocol["source_record_sha256"], "runtime_retained": False}))


if __name__ == "__main__":
    main()
