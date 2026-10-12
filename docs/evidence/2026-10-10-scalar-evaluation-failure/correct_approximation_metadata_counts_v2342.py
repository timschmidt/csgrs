"""Correct import order and README counts after the immutable guard run."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())


def main():
    out = E / "approximation-metadata-count-correction-v2342"
    assert not out.exists()
    validation = E / "approximation-metadata-validation-v2341.json"
    checks = read(validation)
    assert not checks["complete"] and len(checks["phases"]) == 4
    for phase, row in checks["phases"].items():
        assert row["result"]["exit_code"] == (1 if phase.endswith("included-fmt") else 0)
        assert row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert read(E / phase / "result.json") == row["result"]
    source_record = E / "typed-api-prototype-v2341-approximation-metadata-included-fmt/source-hashes.json.gz"
    assert sha(source_record) == "090aedf5a30f3cbaf202a47a4c5deca5e437fbe2e248d23ed76004dda1303e39"
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    scope = read(SOURCE / "crates/hyperreal/verification/scope.json")
    assert len(scope["verified_model_theorems"]) == 137
    assert sum(n.startswith("computable_bounds::") for n in scope["verified_model_theorems"]) == 30
    model_name = "crates/hyperreal/verification/bound_denotation.rs"
    old_import = "use vstd::arithmetic::power2::{lemma2_to64, pow2, lemma_pow2_strictly_increases};"
    new_import = "use vstd::arithmetic::power2::{lemma_pow2_strictly_increases, lemma2_to64, pow2};"
    old_model = (SOURCE / model_name).read_text()
    assert old_model.count(old_import) == 1
    new_model = old_model.replace(old_import, new_import)
    failure = E / "typed-api-prototype-v2341-approximation-metadata-included-fmt"
    assert read(failure / "result.json")["exit_code"] == 1
    output = (failure / "stdout.log").read_text()
    assert output.count("Diff in ") == 1 and "bound_denotation.rs:5:" in output
    assert old_import in output and new_import in output
    name = "crates/hyperreal/verification/README.md"
    before = (SOURCE / name).read_text()
    after = before
    for old, new in [("one hundred thirty-five arithmetic model theorems", "one hundred thirty-seven arithmetic model theorems"),
                     ("Twenty-eight denotation lemmas", "Thirty denotation lemmas")]:
        assert after.count(old) == 1
        after = after.replace(old, new)
    out.mkdir()
    (out / "before.md").write_text(before)
    (out / "after.md").write_text(after)
    (out / "before.rs").write_text(old_model)
    (out / "after.rs").write_text(new_model)
    (out / "correction.patch").write_text("".join(difflib.unified_diff(
        before.splitlines(True), after.splitlines(True), fromfile="a/" + name, tofile="b/" + name))
        + "".join(difflib.unified_diff(old_model.splitlines(True), new_model.splitlines(True),
                    fromfile="a/" + model_name, tofile="b/" + model_name)))
    (out / "protocol.json").write_text(json.dumps({
        "status": "Frozen after passing guards and the retained import-order formatting failure, before correction",
        "runner_sha256": sha(Path(__file__)), "validation_sha256": sha(validation),
        "previous_source_record_sha256": sha(source_record),
        "file": name, "before_sha256": sources[name], "after_sha256": sha(out / "after.md"),
        "model_file": model_name, "model_before_sha256": sources[model_name],
        "model_after_sha256": sha(out / "after.rs"), "old_import": old_import, "new_import": new_import,
        "format_failure": failure.name, "format_failure_result_sha256": sha(failure / "result.json"),
        "correction_patch_sha256": sha(out / "correction.patch"),
        "reason": "Sort the three imports as rustfmt requires and correct the model catalog from 135 to 137 and bound-denotation catalog from 28 to 30. Every function body, specification, test and runtime source stays identical.",
        "required_followup": "Run the complete conditional proof and both formatting checks on corrected sources. Audit unchanged runtime inputs and reconstruct every affected guard input to show that the sole reordered import preserves its exact imported paths and commutes with each mutation. Historical guards retain their original source identities; no fresh guard run is inferred."
    }, indent=2) + "\n")
    (SOURCE / name).write_text(after)
    (SOURCE / model_name).write_text(new_model)
    sources[name] = sha(SOURCE / name)
    sources[model_name] = sha(SOURCE / model_name)
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    record = out / "source-hashes.json.gz"
    record.write_bytes(gzip.compress((json.dumps(sources, sort_keys=True) + "\n").encode(), mtime=0))
    (out / "application.json").write_text(json.dumps({"source_record_sha256": sha(record),
        "changed_files": [name, model_name], "changed_runtime_files": 0,
        "changed_proof_bodies_and_specifications": 0, "reordered_imports": 1,
        "status": "Applied; final proof/input audit pending"}, indent=2) + "\n")
    print(json.dumps({"changed_files": 2, "changed_proof_bodies": 0, "source_record_sha256": sha(record)}))


if __name__ == "__main__":
    main()
