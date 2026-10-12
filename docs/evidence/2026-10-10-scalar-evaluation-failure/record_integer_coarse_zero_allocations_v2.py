"""Audit fresh public/raw integer allocations and scope the unchanged root study."""
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
TARGET = ROOT / "target/scalar-evaluation-candidate/target"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = E / "integer-coarse-zero-allocation-review-v2092.json"
    assert not out.exists()
    proof = read(E / "integer-coarse-zero-proof-validation-v2083.json")
    focused = E / proof["focused_phase"]
    result = read(focused / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert sha(focused / "source-hashes.json.gz") == proof["source_record_sha256"]
    assert "test result: ok. 5 passed; 0 failed; 0 ignored;" in (focused / "stdout.log").read_text()
    pattern = (r"integer scale allocation raw=(true|false), bits=(\d+), negative=(true|false), "
               r"precision=(-?\d+), allocations=Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}")
    rows = []
    for raw, bits, negative, precision, calls, total, largest in re.findall(pattern, (focused / "stderr.log").read_text()):
        rows.append({"raw": raw == "true", "bits": int(bits), "negative": negative == "true",
            "precision": int(precision), "calls": int(calls), "bytes": int(total), "largest": int(largest)})
    expected = {(raw, bits, negative, precision) for raw in [False, True]
                for bits in [65, 4097, 65537, 1000001] for negative in [False, True]
                for precision in [bits - 1, bits, bits + 1, 2147483647]}
    observed = {(r["raw"], r["bits"], r["negative"], r["precision"]) for r in rows}
    assert len(rows) == len(observed) == len(expected) == 64 and observed == expected
    assert all(row["bytes"] <= 128 for row in rows)
    new_branch = [row for row in rows if not row["raw"] and row["precision"] > row["bits"]]
    assert len(new_branch) == 16 and all((row["calls"], row["bytes"], row["largest"]) == (1, 40, 40) for row in new_branch)
    raw_branch = [row for row in rows if row["raw"] and row["precision"] > row["bits"]]
    assert len(raw_branch) == 16 and all(row["calls"] == row["bytes"] == row["largest"] == 0 for row in raw_branch)
    frozen = json.loads(gzip.decompress((focused / "source-hashes.json.gz").read_bytes()))
    test_name = "crates/hyperreal/src/computable/node/integer_scale_tests.rs"
    assert sha(SOURCE / test_name) == frozen[test_name]
    retained = read(E / proof["retained_runtime_checkpoint"])
    baseline_sources = json.loads(gzip.decompress((E / retained["source_record"]).read_bytes()))
    assert frozen[test_name] == baseline_sources[test_name]
    old_study_path = E / "nth-root-enclosure-allocation-study-v2021/summary.json"
    old_study = read(old_study_path)
    old_protocol_path = E / old_study["protocol"]
    old_protocol = read(old_protocol_path)
    assert sha(old_protocol_path) == old_study["protocol_sha256"]
    assert all(sha(old_study_path.parent / name) == digest for name, digest in old_study["artifacts_sha256"].items())
    assert old_study["counts_stable"] and old_study["exact_endpoints_equal"] and not old_study["increases"]
    root_name = old_protocol["candidate_file"]
    assert sha(SOURCE / root_name) == frozen[root_name] == old_protocol["candidate_file_sha256"]
    num = Path(old_study["production_num_dependency"])
    assert sha(num) == old_study["libraries_sha256"][str(num)]
    build = E / "typed-api-prototype-v2086-integer-coarse-zero-release-build"
    assert read(build / "result.json")["exit_code"] == 0
    assert sha(build / "source-hashes.json.gz") == proof["source_record_sha256"]
    deps = TARGET / "release/deps"
    scalars = [path for path in deps.glob("libhyperreal-*.rlib")
               if sha(path) == sha(TARGET / "release/libhyperreal.rlib")]
    assert len(scalars) == 1
    fingerprints = TARGET / "release/.fingerprint"
    metadata = read(fingerprints / scalars[0].stem.removeprefix("lib") / "lib-hyperreal.json")
    num_identity = next(row[3] for row in metadata["deps"] if row[1] == "num")
    num_receipt = fingerprints / num.stem.removeprefix("lib") / "lib-num"
    assert int.from_bytes(bytes.fromhex(num_receipt.read_text()), "little") == num_identity
    data = {
        "status": "Fresh integer allocation controls pass; root helper observations retain their original scope",
        "source_record_sha256": proof["source_record_sha256"], "focused_phase": focused.name,
        "focused_stdout_sha256": sha(focused / "stdout.log"), "focused_stderr_sha256": sha(focused / "stderr.log"),
        "test_source": test_name, "test_source_sha256": frozen[test_name], "tests_unchanged_from_retained_v1836": True,
        "observations": sorted(rows, key=lambda row: (row["raw"], row["bits"], row["negative"], row["precision"])),
        "public_cases": 32, "raw_cases": 32, "fresh_budgeted_shortcut_cases": 16,
        "budgeted_shortcut_allocation_observation": {"calls": 1, "bytes": 40, "largest": 40},
        "raw_zero_allocation_controls": 16, "maximum_observed_bytes": max(row["bytes"] for row in rows),
        "reused_copied_root_study": str(old_study_path.relative_to(E)), "reused_copied_root_study_sha256": sha(old_study_path),
        "root_kernel_sha256": frozen[root_name], "unchanged_num_dependency": str(num), "num_dependency_sha256": sha(num),
        "fresh_root_allocation_measurements": 0,
        "prior_audit_correction": "integer-coarse-zero-allocation-audit-correction-v2092.json",
        "limits": [
            "Fresh counts measure the existing test allocator around individual public or raw integer approximation calls. Fixture construction and the independent oracle are outside that window. Public approx_with_budget enters BudgetScope inside the counter window; its 40-byte allocation remains included. The raw call does not enter that scope. Source inspection shows BudgetScope::enter creates an Rc<ActiveBudget>; allocation counts alone do not identify the allocating call site. The tests also assert exact outputs, one consumed work unit for public calls, unchanged budgets for raw calls and no numeric cache publication.",
            "The historical retained broad tests passed the same allocation ceiling but did not emit these per-case counts. This is fresh candidate evidence against an unchanged test contract, not a matched before/after allocation measurement.",
            "The earlier 84-case root study is reused only for its byte-identical copied helper and matching num dependency. It is not a fresh measurement of this production caller, total memory, timing, query work or peak/live allocation. Its original source and measurement limitations remain authoritative.",
            "These observations do not pass any timing or replay gate. The v2021 timing rejection remains failed, retained runtime remains v1836, and all main/native/full-board and whole-crate proof obligations remain open.",
        ],
    }
    out.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"fresh_integer_allocation_cases": 64, "budgeted_shortcut_cases_with_one_40_byte_allocation": 16,
                      "maximum_bytes": data["maximum_observed_bytes"], "fresh_root_allocation_measurements": 0}))


if __name__ == "__main__":
    main()
