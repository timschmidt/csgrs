"""Freeze and apply the conditional generic digit-scan model and negative controls."""
import ast
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
OUT = E / "digit-scan-baseline-v2363"
PREFIX = "crates/hyperreal/verification/"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))

MUTATIONS = [
    ("digit_prefix_bound", "#[trigger] digits[index] < pow2(width),",
     "#[trigger] digits[index] <= pow2(width),"),
    ("digit_prefix_zero", "ensures digit_prefix(digits, width, length) == 0 <==>",
     "ensures digit_prefix(digits, width, length) == 1 <==>"),
    ("digit_prefix_full", "#[trigger] digits[index] + 1 == pow2(width),",
     "#[trigger] digits[index] + 1 <= pow2(width),"),
    ("upper_digits_preserve_odd_factor", "requires odd > 0, odd % 2 == 1, shift < width,",
     "requires odd > 0, shift < width,"),
    ("upper_digits_preserve_odd_factor", "requires odd > 0, odd % 2 == 1, shift < width,",
     "requires odd > 0, odd % 2 == 1, shift <= width,"),
    ("all_full_digits_factor", "ensures has_odd_factor(digit_value(digits, width) + 1, width * digits.len()),",
     "ensures has_odd_factor(digit_value(digits, width) + 1, width * digits.len() + 1),"),
    ("first_nonzero_digit_factor", "#[trigger] digits[lower] == 0,",
     "#[trigger] digits[lower] <= 1,"),
    ("first_nonzero_digit_factor", "odd > 0, odd % 2 == 1, digits[index] == odd * pow2(zeros),",
     "odd > 0, odd % 2 == 1, digits[index] + 1 == odd * pow2(zeros),"),
    ("first_nonmaximal_digit_factor", "#[trigger] digits[lower] + 1 == pow2(width),",
     "#[trigger] digits[lower] + 1 <= pow2(width),"),
    ("first_nonmaximal_digit_factor", "odd > 0, odd % 2 == 1, digits[index] + 1 == odd * pow2(ones),",
     "odd > 0, odd % 2 == 1, digits[index] + 2 == odd * pow2(ones),"),
    ("normalized_zero_iff_empty", "requires normalized_digits(digits, width),",
     "requires true,"),
    ("normalized_digit_bit_length", "requires normalized_digits(digits, width), digits.len() > 0,",
     "requires digits.len() > 0,"),
    ("normalized_digit_bit_length", "has_bit_length(digits[digits.len() - 1], top_bits),",
     "has_bit_length(digits[digits.len() - 1], top_bits + 1),"),
    ("zero_scan_determines_power_of_two", "ensures (width * index as nat + zeros == (bits - 1) as nat)",
     "ensures (width * index as nat + zeros == bits)"),
    ("zero_scan_determines_power_of_two", "crate::magnitude_model::has_bit_length(digit_value(digits, width), bits),",
     "bits > 0,"),
    ("one_scan_determines_all_ones", "ensures (width * index as nat + ones == bits)",
     "ensures (width * index as nat + ones + 1 == bits)"),
    ("full_digit_scan_determines_all_ones", "ensures (width * digits.len() == bits)",
     "ensures (width * digits.len() + 1 == bits)"),
    ("u32_digit_bit_length", "has_bit_length(word as nat, (32 - u32_leading_zeros(word)) as nat),",
     "has_bit_length(word as nat, (31 - u32_leading_zeros(word)) as nat),"),
    ("u32_digit_bit_length", "0 <= u32_leading_zeros(word) <= 32,",
     "0 <= u32_leading_zeros(word) < 32,"),
]


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
        assert all(not (SOURCE / n).exists() for n in protocol["new_source_files"])
        subprocess.run(["git", "apply", "--check", str(OUT / "proposal.patch")], cwd=SOURCE, check=True)
        for name in protocol["candidate_files_sha256"]:
            (SOURCE / name).write_bytes((OUT / "candidate" / name).read_bytes())
        after = snapshot(OUT / "candidate-source-hashes.json.gz")
        assert all(sha(SOURCE / n) == h for n, h in after.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        (OUT / "application.json").write_text(json.dumps({
            "source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz"),
            "verification_files_changed_or_added": 6, "runtime_source_files_changed": 0,
            "proof_retained": False, "retained_checkpoint": protocol["retained_checkpoint"],
        }, indent=2) + "\n")
        print(json.dumps({"source_files": len(after), "runtime_source_files_changed": 0,
                          "source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz")}))
        return
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2358-state.json"
    assert sha(checkpoint_path) == "5af0c6067447ffcf6f531b7b3cb206b83532225b99f6c8a1f66cbca7698bc991"
    checkpoint = read(checkpoint_path)
    before = snapshot(E / checkpoint["source_record"])
    assert len(before) == 24747 and all(sha(SOURCE / n) == h for n, h in before.items())
    support_path = E / "digit-scan-connections-probe-v2362"
    support = read(support_path / "result.json")
    assert support["exit_code"] == 0 and support["source_unchanged"] and support["tooling_unchanged"]
    assert read(support_path / "stdout.json")["verification-results"]["verified"] == 418
    proposal = E / "digit-scan-connected-proposal-v2362.rs"
    assert sha(proposal) == read(support_path / "protocol.json")["probe_sha256"]
    names = ["lib.rs", "test_rejections.py", "scope.json", "README.md"]
    originals = {PREFIX + n: (SOURCE / PREFIX / n).read_text() for n in names}
    candidate = dict(originals)
    body = proposal.read_text().split("\nverus! {\n", 1)[1]
    model = ("//! Conditional digit scans and metadata for a power-of-two radix.\n"
        "//! Actual dependency storage, iterators and machine counts remain open.\n"
        "use crate::magnitude_model::has_bit_length;\n"
        "use vstd::arithmetic::{div_mod::*, power2::*};\n"
        "use vstd::prelude::*;\nuse vstd::std_specs::bits::*;\n\nverus! {\n" + body)
    model = model.replace("pub proof fn", "pub(crate) proof fn")
    model = model.replace("pub open spec fn", "pub(crate) open spec fn")
    functions = re.findall(r"pub\(crate\) proof fn (\w+)\(", model)
    assert len(functions) == len(set(functions)) == 17
    model_name = PREFIX + "digit_scan_model.rs"
    test_name = PREFIX + "test_digit_scan_rejections.py"
    assert all(n not in before and not (SOURCE / n).exists() for n in [model_name, test_name])
    candidate[model_name] = model
    test = (SOURCE / PREFIX / "test_bit_metadata_rejections.py").read_text()
    tree = ast.parse(test)
    assignment = next(n for n in tree.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "MUTATIONS" for t in n.targets))
    lines = test.splitlines(True)
    test = "".join(lines[:assignment.lineno - 1]) + "MUTATIONS = " + repr(MUTATIONS) + "\n" + "".join(lines[assignment.end_lineno:])
    for old, new in [
        ("Reject defects in the conditional bit-count and bit-pattern model bridges.", "Reject defects in conditional generic digit scans and metadata connections."),
        ('prefix="hyperreal-bit-metadata-rejections-"', 'prefix="hyperreal-digit-scan-rejections-"'),
        ('["magnitude_model.rs", "bit_length_model.rs"]', '["magnitude_model.rs", "bit_length_model.rs", "digit_scan_model.rs"]'),
        ('mod bit_length_model;\\n")', 'mod bit_length_model;\\nmod digit_scan_model;\\n")'),
        ('model = root / "bit_length_model.rs"', 'model = root / "digit_scan_model.rs"'),
        ("Unmutated bit-metadata proofs failed", "Unmutated digit-scan proofs failed"),
        ("Expected bit-metadata rejection", "Expected digit-scan rejection"),
        ("Rejected incorrect bit metadata", "Rejected incorrect digit scan"),
    ]:
        assert test.count(old) == 1, old
        test = test.replace(old, new)
    ast.parse(test)
    candidate[test_name] = test
    for name, old, addition in [
        ("lib.rs", "mod bit_length_model;\n", "mod digit_scan_model;\n"),
        ("test_rejections.py", "from test_bit_metadata_rejections import main as reject_bit_metadata_mutations  # noqa: E402\n",
         "from test_digit_scan_rejections import main as reject_digit_scan_mutations  # noqa: E402\n"),
        ("test_rejections.py", "    reject_bit_metadata_mutations()\n", "    reject_digit_scan_mutations()\n"),
    ]:
        assert candidate[PREFIX + name].count(old) == 1
        candidate[PREFIX + name] = candidate[PREFIX + name].replace(old, old + addition)
    scope = json.loads(candidate[PREFIX + "scope.json"])
    assert len(scope["verified_executable_contracts"]) == 98 and len(scope["verified_model_theorems"]) == 143
    scope["verified_model_theorems"] += ["digit_scan_model::" + n for n in functions]
    candidate[PREFIX + "scope.json"] = json.dumps(scope, indent=2) + "\n"
    readme = candidate[PREFIX + "README.md"]
    old = "one hundred forty-three arithmetic model theorems"
    assert readme.count(old) == 1
    readme = readme.replace(old, "one hundred sixty arithmetic model theorems")
    anchor = "The numerical-bound metadata bridge connects explicit bit-length, all-ones\n"
    assert readme.count(anchor) == 1
    paragraph = (
        "The generic digit-scan model connects first-nonzero and first-nonmaximal\n"
        "digit premises to whole-value odd factors and the retained metadata flag\n"
        "lemmas. It covers empty and all-maximal storage, normalized zero and bit\n"
        "length, and the public 32-bit leading-zero contract. Its arithmetic uses\n"
        "unbounded counts and an explicit digit width. These conditional models\n"
        "do not verify the dependency's storage, actual iterator scan, count range,\n"
        "sign import or producer/caller connection. Runtime evaluation order is unchanged.\n\n")
    candidate[PREFIX + "README.md"] = readme.replace(anchor, paragraph + anchor)
    OUT.mkdir()
    for name, text in candidate.items():
        path = OUT / "candidate" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    subprocess.run(["rustfmt", "--edition", "2024", "--config-path", str(SOURCE / "crates/.rustfmt.toml"),
        "--config", "skip_children=true", str(OUT / "candidate" / model_name)], cwd=ROOT, check=True)
    candidate[model_name] = (OUT / "candidate" / model_name).read_text()
    for name, old, new in MUTATIONS:
        model = candidate[model_name]
        start = model.index("proof fn " + name + "(")
        end = model.index("\n}\n", start) + 3
        assert model[start:end].count(old) == 1 and old != new, (name, old)
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
    assert len(hashes) == 6 and all(before.get(n) != h for n, h in hashes.items())
    (OUT / "candidate-source-hashes.json.gz").write_bytes(gzip.compress(
        (json.dumps(before | hashes, sort_keys=True) + "\n").encode(), mtime=0))
    (OUT / "protocol.json").write_text(json.dumps({
        "status": "Frozen before applying or verifying the generic digit-scan extension",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path),
        "retained_source_record": checkpoint["source_record"],
        "retained_source_record_sha256": checkpoint["source_record_sha256"],
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "support_result_sha256": sha(support_path / "result.json"),
        "candidate_files_sha256": hashes, "new_source_files": [model_name, test_name],
        "source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz"),
        "proposal_patch_sha256": sha(OUT / "proposal.patch"), "runtime_source_files_changed": 0,
        "new_model_theorems": ["digit_scan_model::" + n for n in functions],
        "new_negative_controls": len(MUTATIONS), "unchanged_production_contracts": 98,
        "unchanged_open_obligation_groups": 13,
        "gates": "Full conditional proof, nineteen new negatives and their unmutated baseline, acceptance and both formatting commands. Prior guards and runtime evidence require exact-input, dependency and tool audits. Preserve every failed phase; no simultaneous SOURCE edits or overlapping validations.",
        "scope": "Seventeen conditional arithmetic lemmas only. Actual dependency digit views and normalization, iterator behavior, machine-width metadata range, BigInt sign/magnitude import, caller and producer truth, independent real existence and atomic publication remain open. No runtime, main or property/target admission.",
    }, indent=2) + "\n")
    print(json.dumps({"frozen_verification_files": 6, "new_model_theorems": len(functions),
                      "new_negative_controls": len(MUTATIONS), "runtime_source_files_changed": 0}))


if __name__ == "__main__":
    main()
