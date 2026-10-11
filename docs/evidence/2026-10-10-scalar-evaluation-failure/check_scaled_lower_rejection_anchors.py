#!/usr/bin/env python3
"""Preflight every guard mutation before running the pinned proof suite."""
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
CRATE = ROOT / "target/scalar-evaluation-candidate/source/crates/hyperreal"
sys.path.insert(0, str(CRATE / "verification"))
import test_rejections as core
import test_bound_rejections as bound
import test_cache_rejections as cache
import test_structural_rejections as structural


def main():
    inputs = {}

    def source(name):
        path = CRATE / name
        inputs[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        return path.read_text()

    count = 0
    for name, before, _ in core.MUTATIONS:
        prefix = "verification/" if name in core.MODEL_FILES else "src/verified/"
        assert source(prefix + name).count(before) == 1, (name, before)
        count += 1
    for module, production, model in [
        (bound, "bounds.rs", "bound_denotation.rs"),
        (cache, "representation.rs", "cache_encoding_model.rs"),
    ]:
        text = source("src/computable/node/" + production)
        for method, before, after in module.MUTATIONS:
            module.mutate_body(text, method, before, after)
            count += 1
        text = source("verification/" + model)
        for name, before, _ in module.MODEL_MUTATIONS:
            assert text.count(before) == 1, name
            count += 1
    text = source("src/structural.rs")
    for typename, method, before, after in structural.MUTATIONS:
        structural.mutate_impl(text, typename, method, before, after)
        count += 1
    for name, expression in [("NONE", "0"), ("PI", "1 << 0"), ("EXP", "1 << 1"),
                             ("SQRT", "1 << 2"), ("LOG", "1 << 3"), ("TRIG", "1 << 4"), ("OPAQUE", "1 << 15")]:
        assert text.count(f"pub const {name}: Self = Self({expression});") == 1
        count += 1
    assert source("src/verified/word.rs").count("let left = left_numerator.checked_mul(right_denominator)?;") == 1
    count += 1
    assert count == 213
    for name in ["test_rejections.py", "test_bound_rejections.py", "test_cache_rejections.py", "test_structural_rejections.py"]:
        source("verification/" + name)
    out = EVIDENCE / "rejection-anchor-preflight-v251.json"
    assert not out.exists()
    out.write_text(json.dumps({"matched_mutations": count, "inputs_sha256": inputs,
                               "checker_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                               "scope": "Unique source anchors only. Actual mathematical rejection is checked separately by the full pinned Verus guard run."}, indent=2) + "\n")
    print(f"All {count} mutation anchors uniquely matched, including constants and proof bypass.")


if __name__ == "__main__":
    main()
