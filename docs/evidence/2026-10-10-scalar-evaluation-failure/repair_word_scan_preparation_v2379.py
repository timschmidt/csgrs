"""Disambiguate the count-range mutation before any candidate source is applied."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "prepare_word_scan_candidate_v2378.py"
    target = E / "prepare_word_scan_candidate_v2379.py"
    failure = E / "word-scan-preparation-failure-v2378.json"
    record = E / "word-scan-preparation-repair-v2379.json"
    assert all(not p.exists() for p in [target, failure, record])
    partial = E / "word-scan-baseline-v2378"
    assert not (partial / "protocol.json").exists() and not (partial / "application.json").exists()
    model = partial / "candidate/crates/hyperreal/verification/word_scan_model.rs"
    text = model.read_text()
    start = text.index("proof fn digit_scan_count_fits_u64(")
    end = text.index("\n}\n", start) + 3
    old = "requires width > 0, index < length, within < width,"
    assert text[start:end].count(old) == 2
    code = parent.read_text()
    for anchor in [old, "requires width > 0, index < length, within <= width,"]:
        before = json.dumps(anchor)
        after = json.dumps(anchor + "\n        width * length <= u64::MAX,\n    ensures")
        assert code.count(before) == 1
        code = code.replace(before, after)
    assert code.count('OUT = E / "word-scan-baseline-v2378"') == 1
    code = code.replace('OUT = E / "word-scan-baseline-v2378"', 'OUT = E / "word-scan-baseline-v2379"')
    ast.parse(code)
    target.write_text(code)
    failure.write_text(json.dumps({"status": "Preparation rejected a nonunique mutation anchor before source application",
        "failed_preparation_sha256": sha(parent), "partial_model_sha256": sha(model),
        "function": "digit_scan_count_fits_u64", "anchor_occurrences": 2,
        "reason": "The same requirement text occurs in the function contract and its nonlinear proof assertion. The preparer rejected the ambiguous mutation.",
        "source_applied": False, "partial_candidate_preserved": str(partial.relative_to(E))}, indent=2) + "\n")
    record.write_text(json.dumps({"status": "Frozen correction before preparing a separate candidate",
        "repair_sha256": sha(Path(__file__)), "parent_sha256": sha(parent), "new_preparation_sha256": sha(target),
        "failure_record_sha256": sha(failure),
        "change": "Extend only the within-digit mutation anchor through the following ensures keyword. The intended single contract mutation and all proof statements remain unchanged."}, indent=2) + "\n")
    print(json.dumps({"frozen_unique_anchor_repair": True, "source_changed": False}))


if __name__ == "__main__":
    main()
