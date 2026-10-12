"""Freeze and apply word-factor and metadata-flag models from checkpoint v2345."""
import ast
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
OUT = E / "trailing-metadata-baseline-v2350"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))
PREFIX = "crates/hyperreal/verification/"


def main():
    assert len(sys.argv) == 2 and sys.argv[1] in {"prepare", "apply"}
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    if sys.argv[1] == "apply":
        protocol = read(OUT / "protocol.json")
        assert protocol["runner_sha256"] == sha(Path(__file__))
        assert not (OUT / "application.json").exists()
        before = snapshot(E / protocol["retained_source_record"])
        assert all(sha(SOURCE / n) == h for n, h in before.items())
        assert sha(OUT / "proposal.patch") == protocol["proposal_patch_sha256"]
        assert all(sha(OUT / "candidate" / n) == h for n, h in protocol["candidate_files_sha256"].items())
        for n in protocol["new_source_files"]:
            assert not (SOURCE / n).exists()
        subprocess.run(["git", "apply", "--check", str(OUT / "proposal.patch")], cwd=SOURCE, check=True)
        for n in protocol["candidate_files_sha256"]:
            (SOURCE / n).write_bytes((OUT / "candidate" / n).read_bytes())
        after = snapshot(OUT / "candidate-source-hashes.json.gz")
        assert all(sha(SOURCE / n) == h for n, h in after.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        (OUT / "application.json").write_text(json.dumps({"source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz"),
            "verification_files_changed_or_added": 5, "runtime_source_files_changed": 0,
            "proof_retained": False, "retained_checkpoint": protocol["retained_checkpoint"]}, indent=2) + "\n")
        print(json.dumps({"source_files": len(after), "runtime_source_files_changed": 0,
                          "source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz")}))
        return
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2345-state.json"
    assert sha(checkpoint_path) == "a75220d56a289d2e83e36a426fb6f1f04b808e684b0889e5d360924408d940b9"
    checkpoint = read(checkpoint_path)
    before = snapshot(E / checkpoint["source_record"])
    assert len(before) == 24746 and all(sha(SOURCE / n) == h for n, h in before.items())
    support = read(E / "trailing-bit-support-milestone-v2349.json")
    assert support["retained_checkpoint_sha256"] == sha(checkpoint_path)
    names = ["bit_length_model.rs", "test_rejections.py", "scope.json", "README.md"]
    originals = {PREFIX + n: (SOURCE / PREFIX / n).read_text() for n in names}
    candidate = dict(originals)
    primitive = (E / "trailing-bit-factorization-proposal-v2348.rs").read_text()
    assert primitive.count("\nverus! {\n") == 1 and primitive.endswith("}\n")
    primitive = primitive.split("\nverus! {\n", 1)[1][:-2]
    start = primitive.index("proof fn zero_count_factor_u64(")
    end = primitive.index("\n}\n", start) + 3
    primitive = primitive[:start] + primitive[end:]
    primitive = primitive.replace("zero_count_factor_u64(complement);",
        "crate::verified::gcd::trailing_zeros_factor_u64(complement);")
    primitive = primitive.replace("zero_count_factor_u32", "trailing_zeros_factor_u32")
    primitive = primitive.replace("one_count_factor_u32", "trailing_ones_factor_u32")
    primitive = primitive.replace("one_count_factor_u64", "trailing_ones_factor_u64")
    primitive = primitive.replace("proof fn ", "pub(crate) proof fn ")
    extra = primitive + "\n" + (E / "trailing-metadata-flag-proposal-v2350.rs").read_text()
    model_name = PREFIX + "bit_length_model.rs"
    model = candidate[model_name]
    old_import = "use vstd::std_specs::bits::{u64_leading_zeros, axiom_u64_leading_zeros};"
    assert model.count(old_import) == 1 and model.endswith("}\n")
    model = model.replace(old_import, "use vstd::std_specs::bits::*;")
    candidate[model_name] = model[:-2] + extra + "\n}\n"
    test_name = PREFIX + "test_bit_metadata_rejections.py"
    assert test_name not in before and not (SOURCE / test_name).exists()
    candidate[test_name] = (E / "trailing-metadata-guard-proposal-v2350.py").read_text()
    entry_name = PREFIX + "test_rejections.py"
    needle = "from test_structural_rejections import main as reject_structural_mutations  # noqa: E402\n"
    assert candidate[entry_name].count(needle) == 1
    candidate[entry_name] = candidate[entry_name].replace(needle, needle +
        "from test_bit_metadata_rejections import main as reject_bit_metadata_mutations  # noqa: E402\n")
    needle = "    reject_structural_mutations()\n"
    assert candidate[entry_name].count(needle) == 1
    candidate[entry_name] = candidate[entry_name].replace(needle, needle + "    reject_bit_metadata_mutations()\n")
    functions = ["trailing_zeros_factor_u32", "trailing_ones_factor_u32", "trailing_ones_factor_u64",
                 "odd_factor_of_power_is_unit", "trailing_zeros_determine_power_of_two",
                 "trailing_ones_determine_all_ones"]
    scope = json.loads(candidate[PREFIX + "scope.json"])
    assert len(scope["verified_executable_contracts"]) == 98 and len(scope["verified_model_theorems"]) == 137
    assert all("bit_length_model::" + n not in scope["verified_model_theorems"] for n in functions)
    scope["verified_model_theorems"] += ["bit_length_model::" + n for n in functions]
    candidate[PREFIX + "scope.json"] = json.dumps(scope, indent=2) + "\n"
    readme = candidate[PREFIX + "README.md"]
    needle = "one hundred thirty-seven arithmetic model theorems"
    assert readme.count(needle) == 1
    readme = readme.replace(needle, "one hundred forty-three arithmetic model theorems")
    needle = "The numerical-bound metadata bridge connects explicit bit-length, all-ones\n"
    assert readme.count(needle) == 1
    paragraph = (
        "The bit-length model also derives native trailing-one factorizations for\n"
        "32-bit and 64-bit words, including an all-ones word plus one in unbounded\n"
        "arithmetic. Positive odd-factor certificates and exact bit length determine\n"
        "the power-of-two and all-ones flags. The 64-bit zero-factor proof is reused\n"
        "from the actual verified helper. BigUint storage and normalization, whole\n"
        "digit scans, metadata range checks and the bound producer's caller remain\n"
        "outside this conditional model boundary; runtime evaluation order is unchanged.\n\n")
    candidate[PREFIX + "README.md"] = readme.replace(needle, paragraph + needle)
    OUT.mkdir()
    for name, text in candidate.items():
        path = OUT / "candidate" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    subprocess.run(["rustfmt", "--edition", "2024", "--config-path", str(SOURCE / "crates/.rustfmt.toml"),
        "--config", "skip_children=true", str(OUT / "candidate" / model_name)], cwd=ROOT, check=True)
    candidate[model_name] = (OUT / "candidate" / model_name).read_text()
    mutation_tree = ast.parse(candidate[test_name])
    mutations = ast.literal_eval(next(n.value for n in mutation_tree.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "MUTATIONS" for t in n.targets)))
    assert len(mutations) == 12
    for name, old, new in mutations:
        body = candidate[model_name]
        anchor = "proof fn " + name + "("
        assert body.count(anchor) == 1, name
        start = body.index(anchor)
        end = body.index("\n}\n", start) + 3
        assert body[start:end].count(old) == 1, (name, old)
        assert old != new
    patch = []
    for name, text in candidate.items():
        original = originals.get(name, "")
        if name in originals:
            path = OUT / "retained" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(original)
        patch.extend(difflib.unified_diff(original.splitlines(True), text.splitlines(True),
            fromfile="a/" + name if name in originals else "/dev/null", tofile="b/" + name))
    (OUT / "proposal.patch").write_text("".join(patch))
    hashes = {n: sha(OUT / "candidate" / n) for n in candidate}
    assert len(hashes) == 5 and all(before.get(n) != h for n, h in hashes.items())
    (OUT / "candidate-source-hashes.json.gz").write_bytes(gzip.compress(
        (json.dumps(before | hashes, sort_keys=True) + "\n").encode(), mtime=0))
    (OUT / "protocol.json").write_text(json.dumps({"status": "Frozen before applying or verifying the model extension",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path), "retained_source_record": checkpoint["source_record"],
        "retained_source_record_sha256": checkpoint["source_record_sha256"],
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "support_milestone_sha256": sha(E / "trailing-bit-support-milestone-v2349.json"),
        "candidate_files_sha256": hashes, "new_source_files": [test_name],
        "source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz"),
        "proposal_patch_sha256": sha(OUT / "proposal.patch"), "runtime_source_files_changed": 0,
        "new_model_theorems": ["bit_length_model::" + n for n in functions], "new_negative_controls": len(mutations),
        "unchanged_production_contracts": 98, "unchanged_open_obligation_groups": 13,
        "gates": "Run the full conditional proof, twelve new negative controls and their positive baseline, acceptance checks and both formatting commands including the added bit-length model. Reuse old guards or runtime evidence only after auditing their compiled inputs, entrypoint-only additions, dependency/tool identities and unchanged ordinary runtime/build/probe sources. Preserve every failed phase. No simultaneous SOURCE mutation or overlapping validations.",
        "scope": "Six conditional model lemmas only. The three new general flag lemmas have not been verified by the standalone support probes. BigUint storage/import, normalized multiword scans, machine metadata ranges, producer/caller/atomic and independent-existence obligations remain open. No runtime or property/target admission follows."}, indent=2) + "\n")
    print(json.dumps({"frozen_verification_files": 5, "new_model_theorems": 6,
                      "new_negative_controls": 12, "runtime_source_files_changed": 0}))


if __name__ == "__main__":
    main()
