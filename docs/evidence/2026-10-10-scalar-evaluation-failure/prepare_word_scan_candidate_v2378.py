"""Freeze typed word views and scan connections with their negative controls."""
import ast
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import pprint
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
OUT = E / "word-scan-baseline-v2378"
PREFIX = "crates/hyperreal/verification/"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def mutations():
    rows = [
        ("trimmed_zero_suffix_preserves_value", "#[trigger] digits[index] == 0,",
         "#[trigger] digits[index] <= 1,"),
        ("u64_digit_prefix_matches_production", "== crate::verified::limbs::prefix(words, length),",
         "== crate::verified::limbs::prefix(words, length) + 1,"),
        ("digit_scan_count_fits_u64", "requires width > 0, index < length, within < width,",
         "requires width > 0, index < length, within <= width,"),
        ("digit_scan_count_fits_u64", "width * length <= u64::MAX,\n    ensures",
         "true,\n    ensures"),
    ]
    for width in [32, 64]:
        suffix = f"u{width}"
        rows += [
            (f"typed_digit_bounds_{suffix}", f"word_digits_{suffix}(words)[index] < pow2({width}),",
             f"word_digits_{suffix}(words)[index] < pow2({width - 1}),"),
            (f"typed_normalized_bit_length_{suffix}", f"requires normalized_{suffix}(words),",
             "requires true,"),
            (f"typed_zero_scan_{suffix}", f"requires first_nonzero_{suffix}(words, index),",
             "requires 0 <= index < words.len(), words[index] != 0,"),
            (f"typed_one_scan_{suffix}", f"requires first_nonmaximal_{suffix}(words, index),",
             f"requires 0 <= index < words.len(), words[index] != {suffix}::MAX,"),
            (f"typed_zero_scan_none_{suffix}", "#[trigger] words[index] == 0,",
             "#[trigger] words[index] <= 1,"),
            (f"typed_one_scan_none_{suffix}", f"#[trigger] words[index] == {suffix}::MAX,",
             f"#[trigger] words[index] >= {suffix}::MAX - 1,"),
            (f"typed_zero_suffix_truncation_{suffix}", "length == 0 || words[length - 1] > 0,",
             "length == 0 || words[length - 1] >= 0,"),
            (f"typed_zero_suffix_truncation_{suffix}", "#[trigger] words[index] == 0,",
             "#[trigger] words[index] <= 1,"),
        ]
    assert len(rows) == 20
    return rows


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
        for n in protocol["candidate_files_sha256"]:
            (SOURCE / n).write_bytes((OUT / "candidate" / n).read_bytes())
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
    checkpoint_path = E / "typed-api-prototype-v2373-state.json"
    assert sha(checkpoint_path) == "163bf723328ab72df0a56025f9a55960eda13eb4c6767669965b61e66333508a"
    checkpoint = read(checkpoint_path)
    before = snapshot(E / checkpoint["source_record"])
    assert len(before) == 24749 and all(sha(SOURCE / n) == h for n, h in before.items())
    support_path = E / "word-scan-probe-v2377"
    support = read(support_path / "result.json")
    assert support["exit_code"] == 0 and support["source_unchanged"] and support["tooling_unchanged"]
    assert read(support_path / "stdout.json")["verification-results"]["verified"] == 438
    proposal = E / "word-scan-proposal-v2377.rs"
    assert sha(proposal) == read(support_path / "protocol.json")["probe_sha256"]
    names = ["lib.rs", "test_rejections.py", "scope.json", "README.md"]
    originals = {PREFIX + n: (SOURCE / PREFIX / n).read_text() for n in names}
    candidate = dict(originals)
    text = proposal.read_text()
    anchor = "mod word_scan_model {\n"
    assert text.count(anchor) == 1 and text.endswith("}\n}\n")
    model = ("//! Typed word views connected to generic digit denotation and scans.\n"
             "//! Actual dependency storage, iterators and length bounds remain open.\n"
             + text.split(anchor, 1)[1][:-2])
    functions = re.findall(r"pub\(crate\) proof fn (\w+)\(", model)
    assert len(functions) == len(set(functions)) == 18
    model_name = PREFIX + "word_scan_model.rs"
    test_name = PREFIX + "test_word_scan_rejections.py"
    assert all(n not in before and not (SOURCE / n).exists() for n in [model_name, test_name])
    candidate[model_name] = model
    test = (SOURCE / PREFIX / "test_digit_scan_rejections.py").read_text()
    tree = ast.parse(test)
    assignment = next(n for n in tree.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "MUTATIONS" for t in n.targets))
    lines = test.splitlines(True)
    rows = mutations()
    rendered = "MUTATIONS = [\n"
    for row in rows:
        rendered += "    " + pprint.pformat(row, width=96).replace("\n", "\n    ") + ",\n"
    rendered += "]\n"
    test = "".join(lines[:assignment.lineno - 1]) + rendered + "".join(lines[assignment.end_lineno:])
    for old, new in [
        ("Reject defects in conditional generic digit scans and metadata connections.", "Reject defects in typed word views, scans, truncation and range premises."),
        ('prefix="hyperreal-digit-scan-rejections-"', 'prefix="hyperreal-word-scan-rejections-"'),
        ('["magnitude_model.rs", "bit_length_model.rs", "digit_scan_model.rs"]',
         '["magnitude_model.rs", "bit_length_model.rs", "digit_scan_model.rs", "word_scan_model.rs"]'),
        ('mod digit_scan_model;\\n")', 'mod digit_scan_model;\\nmod word_scan_model;\\n")'),
        ('model = root / "digit_scan_model.rs"', 'model = root / "word_scan_model.rs"'),
        ("Unmutated digit-scan proofs failed", "Unmutated word-scan proofs failed"),
        ("Expected digit-scan rejection", "Expected word-scan rejection"),
        ("Rejected incorrect digit scan", "Rejected incorrect word scan"),
    ]:
        assert test.count(old) == 1, old
        test = test.replace(old, new)
    ast.parse(test)
    candidate[test_name] = test
    for name, old, addition in [
        ("lib.rs", "mod real_approximation_model;\n", "mod word_scan_model;\n"),
        ("test_rejections.py", "from test_digit_scan_rejections import main as reject_digit_scan_mutations  # noqa: E402\n",
         "from test_word_scan_rejections import main as reject_word_scan_mutations  # noqa: E402\n"),
        ("test_rejections.py", "    reject_digit_scan_mutations()\n", "    reject_word_scan_mutations()\n"),
    ]:
        assert candidate[PREFIX + name].count(old) == 1
        candidate[PREFIX + name] = candidate[PREFIX + name].replace(old, old + addition)
    scope = json.loads(candidate[PREFIX + "scope.json"])
    assert len(scope["verified_executable_contracts"]) == 98 and len(scope["verified_model_theorems"]) == 160
    scope["verified_model_theorems"] += ["word_scan_model::" + n for n in functions]
    candidate[PREFIX + "scope.json"] = json.dumps(scope, indent=2) + "\n"
    readme = candidate[PREFIX + "README.md"]
    old = "one hundred sixty arithmetic model theorems"
    assert readme.count(old) == 1
    readme = readme.replace(old, "one hundred seventy-eight arithmetic model theorems")
    anchor = "The numerical-bound metadata bridge connects explicit bit-length, all-ones\n"
    assert readme.count(anchor) == 1
    paragraph = (
        "Typed 32-bit and 64-bit word views now connect to the generic digit model.\n"
        "Their conditional proofs cover normalized bit lengths, both scan outcomes,\n"
        "and value-preserving zero-suffix truncation. The 64-bit view agrees with\n"
        "the actual verified limb prefix. A count-range lemma requires an explicit\n"
        "maximum aggregate bit count; it does not derive that limit from Vec allocation.\n"
        "Actual storage imports, normalization/iterator execution and producer calls\n"
        "still need implementation refinement proofs.\n\n")
    candidate[PREFIX + "README.md"] = readme.replace(anchor, paragraph + anchor)
    OUT.mkdir()
    for n, text in candidate.items():
        p = OUT / "candidate" / n
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
    for n in [model_name, PREFIX + "lib.rs"]:
        subprocess.run(["rustfmt", "--edition", "2024", "--config-path", str(SOURCE / "crates/.rustfmt.toml"),
            "--config", "skip_children=true", str(OUT / "candidate" / n)], cwd=ROOT, check=True)
        candidate[n] = (OUT / "candidate" / n).read_text()
    for name, old, new in rows:
        model = candidate[model_name]
        start = model.index("proof fn " + name + "(")
        end = model.index("\n}\n", start) + 3
        assert model[start:end].count(old) == 1 and old != new, (name, old)
    patch = []
    for n, text in candidate.items():
        original = originals.get(n, "")
        if n in originals:
            p = OUT / "retained" / n
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(original)
        patch.extend(difflib.unified_diff(original.splitlines(True), text.splitlines(True),
            fromfile="a/" + n if n in originals else "/dev/null", tofile="b/" + n))
    (OUT / "proposal.patch").write_text("".join(patch))
    hashes = {n: sha(OUT / "candidate" / n) for n in candidate}
    assert len(hashes) == 6 and all(before.get(n) != h for n, h in hashes.items())
    (OUT / "candidate-source-hashes.json.gz").write_bytes(gzip.compress(
        (json.dumps(before | hashes, sort_keys=True) + "\n").encode(), mtime=0))
    (OUT / "protocol.json").write_text(json.dumps({
        "status": "Frozen before applying the typed word-scan model",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path), "retained_source_record": checkpoint["source_record"],
        "retained_source_record_sha256": checkpoint["source_record_sha256"],
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "support_result_sha256": sha(support_path / "result.json"),
        "candidate_files_sha256": hashes, "new_source_files": [model_name, test_name],
        "source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz"),
        "proposal_patch_sha256": sha(OUT / "proposal.patch"), "runtime_source_files_changed": 0,
        "new_model_theorems": ["word_scan_model::" + n for n in functions], "new_negative_controls": len(rows),
        "unchanged_production_contracts": 98, "unchanged_open_obligation_groups": 13,
        "gates": "Full conditional proof, twenty fresh negative controls and baseline, acceptance and both format commands. Reuse earlier guards and runtime evidence only after exact-input, dependency and tool audits. No SOURCE edits or overlapping validations while commands run.",
        "scope": "Eighteen conditional model lemmas connect typed native word sequences to retained digit arithmetic. Actual dependency storage, normalization and first-match iterators, range premises, sign import, approximation and caller truth, independent existence and atomic publication remain open. This does not admit runtime/main changes or complete any property or target.",
    }, indent=2) + "\n")
    print(json.dumps({"frozen_verification_files": 6, "new_model_theorems": len(functions),
                      "new_negative_controls": len(rows), "runtime_source_files_changed": 0}))


if __name__ == "__main__":
    main()
