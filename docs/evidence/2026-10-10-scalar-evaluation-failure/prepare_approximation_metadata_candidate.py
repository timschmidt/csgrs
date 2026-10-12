"""Freeze a proof-only metadata bridge and its negative controls from v2335."""
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
OUT = E / "approximation-metadata-baseline-v2336"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))

MUTATIONS = [
    ("metadata magnitude separation", "requires bits > 1,", "requires bits > 0,"),
    ("metadata endpoint bit length", "crate::magnitude_model::has_bit_length(magnitude, bits as nat),", "magnitude > 1,"),
    ("metadata all-ones flag", "all_ones == (magnitude + 1 == pow2(bits as nat)),", "true,"),
    ("metadata power-of-two flag", "power_of_two == (magnitude == pow2((bits as nat - 1) as nat)),", "true,"),
    ("metadata upper-binade carry", "(bits, if all_ones { 0u64 } else { 1u64 }, false, precision), Some(exponent)),",
     "(bits, if all_ones { 1u64 } else { 1u64 }, false, precision), Some(exponent)),"),
    ("metadata magnitude import", "magnitude as int == if approximation < 0 { -approximation } else { approximation },", "true,"),
    ("metadata approximation sign", "sign == if approximation > 0 { Sign::Plus } else { Sign::Minus },",
     "sign == if approximation > 0 { Sign::Minus } else { Sign::Plus },"),
    ("metadata approximation validity", "approximates(value, approximation, binary_unit(precision as int)),", "true,"),
    ("metadata checked exponent correction", "(magnitude_bits, if reaches_next_binade { 0u64 } else { 1u64 }, false, precision), msd),",
     "(magnitude_bits, if reaches_next_binade { 0u64 } else { 1u64 }, true, precision), msd),"),
    ("metadata exact binade boundaries", "exact_msd == (msd.is_some() && !is_power_of_two && !reaches_next_binade),",
     "exact_msd == msd.is_some(),"),
]


def model_mutations(text):
    result = []
    for node in ast.parse(text).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "MODEL_MUTATIONS" for t in node.targets):
            result = ast.literal_eval(node.value)
        if isinstance(node, ast.AugAssign) and isinstance(node.target, ast.Name) and node.target.id == "MODEL_MUTATIONS":
            result += ast.literal_eval(node.value)
    return result


def main():
    assert len(sys.argv) == 2 and sys.argv[1] in {"prepare", "apply"}
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    if sys.argv[1] == "apply":
        protocol = read(OUT / "protocol.json")
        assert protocol["runner_sha256"] == sha(Path(__file__))
        assert not (OUT / "application.json").exists()
        sources = snapshot(E / protocol["retained_source_record"])
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert sha(OUT / "proposal.patch") == protocol["proposal_patch_sha256"]
        assert all(sha(OUT / "candidate" / n) == h for n, h in protocol["candidate_files_sha256"].items())
        subprocess.run(["git", "apply", "--check", str(OUT / "proposal.patch")], cwd=SOURCE, check=True)
        for name in protocol["candidate_files_sha256"]:
            (SOURCE / name).write_bytes((OUT / "candidate" / name).read_bytes())
        candidate = snapshot(OUT / "candidate-source-hashes.json.gz")
        assert all(sha(SOURCE / n) == h for n, h in candidate.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        (OUT / "application.json").write_text(json.dumps({"source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz"),
            "changed_verification_files": len(protocol["candidate_files_sha256"]),
            "runtime_source_files_changed": 0, "verification_started": False,
            "proof_retained": False, "retained_runtime_checkpoint": protocol["retained_runtime_checkpoint"]}, indent=2) + "\n")
        print(json.dumps({"changed_verification_files": 4, "runtime_source_files_changed": 0,
                          "source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz")}))
        return
    assert not OUT.exists()
    state_path = E / "typed-api-prototype-v2335-state.json"
    state = read(state_path)
    assert sha(state_path) == "7d840a2160d3e48baa67a3cc4ca2388f6270b141960bc0441f2df888fade7228"
    assert state["conditional_proof"]["verified_units"] == 655 and not state["main_runtime_admitted"]
    sources = snapshot(E / state["source_record"])
    assert len(sources) == 24746 and all(sha(SOURCE / n) == h for n, h in sources.items())
    names = ["bound_denotation.rs", "test_bound_rejections.py", "scope.json", "README.md"]
    prefix = "crates/hyperreal/verification/"
    originals = {prefix + n: (SOURCE / prefix / n).read_text() for n in names}
    candidate = dict(originals)
    model_name = prefix + "bound_denotation.rs"
    model = candidate[model_name]
    needle = "use vstd::arithmetic::power2::lemma2_to64;"
    assert model.count(needle) == 1
    model = model.replace(needle, "use vstd::arithmetic::power2::{lemma2_to64, pow2, lemma_pow2_strictly_increases};")
    needle = "proof fn negation_preserves_bound_denotation("
    assert model.count(needle) == 1
    proposal = E / "approximation-metadata-proposal-v2336.rs"
    model = model.replace(needle, proposal.read_text() + "\n" + needle)
    candidate[model_name] = model
    test_name = prefix + "test_bound_rejections.py"
    old_mutations = model_mutations(candidate[test_name])
    for name, before, after in old_mutations + MUTATIONS:
        assert model.count(before) == 1, (name, model.count(before))
        assert before != after
    extra = "\n# Conditional metadata premises and actual checked-exponent linkage must matter.\nMODEL_MUTATIONS += [\n"
    extra += "".join("    " + repr(row) + ",\n" for row in MUTATIONS) + "]\n\n"
    needle = "\ndef mutate_body(source, method, before, after):\n"
    assert candidate[test_name].count(needle) == 1
    candidate[test_name] = candidate[test_name].replace(needle, extra + needle)
    assert model_mutations(candidate[test_name]) == old_mutations + MUTATIONS
    scope = json.loads(candidate[prefix + "scope.json"])
    assert len(scope["verified_executable_contracts"]) == 98 and len(scope["verified_model_theorems"]) == 135
    new_theorems = ["computable_bounds::approximation_bit_pattern_endpoints",
                    "computable_bounds::approximation_metadata_produces_valid_bound"]
    assert all(name not in scope["verified_model_theorems"] for name in new_theorems)
    scope["verified_model_theorems"] += new_theorems
    candidate[prefix + "scope.json"] = json.dumps(scope, indent=2) + "\n"
    needle = "Binary scaling shifts exact real binades by the same mathematical exponent.\n"
    paragraph = (
        "The numerical-bound metadata bridge connects explicit bit-length, all-ones\n"
        "and power-of-two premises to the closed one-unit interval endpoints. It uses\n"
        "the actual checked exponent and bound-constructor contracts, including an\n"
        "absent exponent and both signs. All-ones magnitudes require an upper-binade\n"
        "carry; exact binades exclude both power-of-two and all-ones endpoints.\n"
        "These are conditional model theorems. The actual BigInt metadata/import,\n"
        "bound_from_approx caller, valid approximation and publication invariant\n"
        "remain outside this proof boundary. No runtime code or metadata evaluation\n"
        "order changes, and all thirteen whole-crate obligation groups stay open.\n\n"
    )
    assert candidate[prefix + "README.md"].count(needle) == 1
    candidate[prefix + "README.md"] = candidate[prefix + "README.md"].replace(needle, paragraph + needle)
    assert all(name.startswith(prefix) for name in candidate) and len(candidate) == 4
    candidate_hashes = {n: hashlib.sha256(t.encode()).hexdigest() for n, t in candidate.items()}
    assert all(candidate_hashes[n] != sources[n] for n in candidate)
    OUT.mkdir()
    patch = []
    for name, text in candidate.items():
        for directory, content in [("retained", originals[name]), ("candidate", text)]:
            path = OUT / directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        patch.extend(difflib.unified_diff(originals[name].splitlines(True), text.splitlines(True),
            fromfile="a/" + name, tofile="b/" + name))
    (OUT / "proposal.patch").write_text("".join(patch))
    (OUT / "candidate-source-hashes.json.gz").write_bytes(gzip.compress(
        (json.dumps(sources | candidate_hashes, sort_keys=True) + "\n").encode(), mtime=0))
    protocol = {"status": "Frozen before the proof-only metadata bridge is applied or verified",
        "runner_sha256": sha(Path(__file__)), "proposal_sha256": sha(proposal),
        "base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "retained_runtime_checkpoint": state_path.name, "retained_runtime_checkpoint_sha256": sha(state_path),
        "retained_source_record": state["source_record"], "retained_source_record_sha256": state["source_record_sha256"],
        "retained_files_sha256": {n: sources[n] for n in candidate}, "candidate_files_sha256": candidate_hashes,
        "source_record_sha256": sha(OUT / "candidate-source-hashes.json.gz"),
        "proposal_patch_sha256": sha(OUT / "proposal.patch"), "runtime_source_files_changed": 0,
        "new_model_theorems": new_theorems, "new_negative_controls": len(MUTATIONS),
        "unchanged_production_contracts": 98, "unchanged_open_obligation_groups": 13,
        "gates": "Run the complete conditional proof, all acceptance/mutation guards including ten new controls, and applicable formatting/source-identity checks. Keep each failed proof attempt under its own phase. Reuse runtime tests/cost/replays only after a separate dependency/input audit confirms that all ordinary runtime source/build/probe inputs are identical to v2335; no new timing or absolute-resource claim follows.",
        "scope": "Two conditional model lemmas connect bit-pattern premises and actual checked-exponent/constructor contracts. No actual BigInt method, producer caller, runtime metadata evaluation order, atomic publication or real-existence proof is added. All property/target statuses, resource failures, fixed baseline and main/native/full-board obligations remain unchanged."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    print(json.dumps({"frozen_verification_files": 4, "new_model_theorems": 2,
                      "new_negative_controls": 10, "runtime_source_files_changed": 0}))


if __name__ == "__main__":
    main()
