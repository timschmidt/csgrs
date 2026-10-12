"""Freeze and restore the exact retained source, then run six restoration checks."""
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
OUT = E / "decoder-retained-restoration-v2248"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def check_main():
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert len(guard) == 1167 and all(sha(ROOT / n) == h for n, h in guard.items())
    return len(guard)


def prepare():
    assert not OUT.exists()
    check_main()
    rejected_path = E / "bound-decoder-cost-rejection-v2247.json"
    rejected = read(rejected_path)
    review_path = E / "bound-decoder-binary-comparison-v2181/summary.json"
    review = read(review_path)
    assert not rejected["runtime_retained"] and not review["runtime_retained"]
    checkpoint_path = E / rejected["retained_runtime_checkpoint"]
    assert sha(checkpoint_path) == rejected["retained_runtime_checkpoint_sha256"]
    checkpoint = read(checkpoint_path)
    before_path = E / rejected["source_record"]
    retained_path = E / checkpoint["source_record"]
    assert sha(before_path) == rejected["source_record_sha256"] == review["source_record_sha256"]
    assert sha(retained_path) == checkpoint["source_record_sha256"]
    before, retained = snapshot(before_path), snapshot(retained_path)
    assert before.keys() == retained.keys() and len(before) == 24746
    assert all(sha(SOURCE / n) == h for n, h in before.items())
    names = sorted(n for n in before if before[n] != retained[n])
    assert len(names) == 5
    archives = [E / "bound-decoder-baseline-v2168/inputs"]
    restorations = {}
    patches = []
    for name in names:
        choices = [p / name for p in archives if (p / name).is_file() and sha(p / name) == retained[name]]
        assert choices, name
        restorations[name] = {"rejected_sha256": before[name], "retained_sha256": retained[name],
                              "retained_archive": str(choices[0].relative_to(E))}
        patches.extend(difflib.unified_diff((SOURCE / name).read_text().splitlines(True),
            choices[0].read_text().splitlines(True), fromfile="a/" + name, tofile="b/" + name))
    baseline = read(E / "domain-predicate-candidate-validation-v1772.json")
    commands = []
    for version, prior_version, suffix in [(2249, 1764, "scalar-tests"), (2250, 1766, "release-build"),
                                          (2251, 1767, "workspace-fmt"), (2252, 1768, "included-fmt")]:
        previous = f"typed-api-prototype-v{prior_version}-domain-predicate-{suffix}"
        commands.append({"phase": f"typed-api-prototype-v{version}-retained-restoration-{suffix}",
                         "command": baseline["phases"][previous]["command"], "prior_phase": previous})
    verus_phase = "typed-api-prototype-v2253-retained-restoration-verus"
    acceptance_phase = "typed-api-prototype-v2254-retained-restoration-acceptance"
    commands.extend([
        {"phase": verus_phase, "command": [sys.executable, str(E / "run_candidate_verus.py"), verus_phase]},
        {"phase": acceptance_phase, "command": [sys.executable, str(E / "run_candidate_proof_checks.py"),
                                                acceptance_phase, "test_acceptance.py"]},
    ])
    OUT.mkdir()
    for name in names:
        path = OUT / "rejected" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((SOURCE / name).read_bytes())
    patch = OUT / "restoration.patch"
    patch.write_text("".join(patches))
    subprocess.run(["git", "apply", "--check", str(patch)], cwd=SOURCE, check=True)
    protocol = {"status": "Frozen before restoring the previously retained source",
        "helper_sha256": sha(Path(__file__)), "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "checkpoint": checkpoint_path.name, "checkpoint_sha256": sha(checkpoint_path),
        "rejected_source_record": str(before_path.relative_to(E)), "rejected_source_record_sha256": sha(before_path),
        "retained_source_record": str(retained_path.relative_to(E)), "retained_source_record_sha256": sha(retained_path),
        "rejection_record": rejected_path.name, "rejection_record_sha256": sha(rejected_path),
        "diagnostic_review": review_path.name, "diagnostic_review_sha256": sha(review_path),
        "source_files_verified": len(before), "main_files_verified": 1167, "restorations": restorations,
        "restoration_patch_sha256": sha(patch), "commands": commands,
        "runner_hashes": {n: sha(E / n) for n in ["validate_candidate_v2.py", "run_candidate_verus.py", "run_candidate_proof_checks.py"]},
        "limits": [
            "Restore only these five isolated files from hash-verified immutable archives. Main source and the 49 original worktree entries remain untouched. The rejected proposals and their tests/proofs remain archived, with every earlier failure unchanged.",
            "The restored runtime and proof sources match v1836 exactly. Prior broad/cost/replay/mutation evidence is evidence for those identical retained bytes, not new executions. No failed proposal gains admission through restoration.",
            "Run six fresh restoration checks, including full scalar units, release build, both formatting checks, conditional Verus and acceptance tests. The retained target has 645 conditional units, 97 contracts, 130 model theorems and thirteen open groups. The unretained decoder's additional proofs remain archived and do not describe restored production code.",
            "After restoration verification, inspect the archived executable differences before selecting a proof-compatible next implementation experiment. Preserve the failed original-cap gate; do not retry unchanged timings until one passes. Producer validity, independent existence, atomic/caller ordering, the fixed pre-Verus baseline and main/native/full-board admission remain open, as does the full properties plan.",
        ]}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    print(json.dumps({"restoration_frozen": True, "files": len(names), "fresh_commands": len(commands)}))


def restore(protocol):
    out = OUT / "restoration.json"
    assert not out.exists()
    before = snapshot(E / protocol["rejected_source_record"])
    retained = snapshot(E / protocol["retained_source_record"])
    assert all(sha(SOURCE / n) == h for n, h in before.items())
    check_main()
    for name, item in protocol["restorations"].items():
        assert sha(OUT / "rejected" / name) == item["rejected_sha256"] == before[name]
        assert sha(E / item["retained_archive"]) == item["retained_sha256"] == retained[name]
    assert sha(OUT / "restoration.patch") == protocol["restoration_patch_sha256"]
    subprocess.run(["git", "apply", "--check", str(OUT / "restoration.patch")], cwd=SOURCE, check=True)
    for name, item in protocol["restorations"].items():
        (SOURCE / name).write_bytes((E / item["retained_archive"]).read_bytes())
    actual = {n: sha(SOURCE / n) for n in retained}
    assert actual == retained
    record = OUT / "restored-source-hashes.json.gz"
    record.write_bytes(gzip.compress((json.dumps(actual, sort_keys=True) + "\n").encode(), mtime=0))
    assert sha(record) == protocol["retained_source_record_sha256"]
    check_main()
    out.write_text(json.dumps({"restored_checkpoint": protocol["checkpoint"],
        "restored_source_record_sha256": sha(record), "restored_files": 5, "source_files_verified": len(actual),
        "main_files_verified": 1167, "validation_complete": False, "new_runtime_admission": False}, indent=2) + "\n")
    print(json.dumps({"restored_files": 5, "source_files_identical_to_retained": len(actual)}))


def validate(protocol):
    out = E / "decoder-retained-restoration-validation-v2254.json"
    assert not out.exists() and (OUT / "restoration.json").exists()
    for n, h in protocol["runner_hashes"].items():
        assert sha(E / n) == h
    rows = []
    for item in protocol["commands"]:
        phase = E / item["phase"]
        assert not phase.exists()
        subprocess.run([sys.executable, str(E / "validate_candidate_v2.py"), item["phase"],
                        "--exec", *item["command"]], cwd=ROOT, check=True)
        result = read(phase / "result.json")
        assert sha(phase / "source-hashes.json.gz") == protocol["retained_source_record_sha256"]
        rows.append(item | {"result": result})
        out.write_text(json.dumps({"source_record_sha256": protocol["retained_source_record_sha256"],
            "protocol_sha256": sha(OUT / "protocol.json"), "complete": len(rows) == 6 and result["exit_code"] == 0,
            "phases": rows, "new_runtime_admission": False}, indent=2) + "\n")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"], item["phase"]
        print(item["phase"], "passed", flush=True)


if __name__ == "__main__":
    assert len(sys.argv) == 2
    if sys.argv[1] == "prepare":
        prepare()
    else:
        protocol = read(OUT / "protocol.json")
        assert sha(Path(__file__)) == protocol["helper_sha256"]
        assert sha(E / protocol["checkpoint"]) == protocol["checkpoint_sha256"]
        if sys.argv[1] == "restore":
            restore(protocol)
        else:
            assert sys.argv[1] == "validate"
            validate(protocol)
