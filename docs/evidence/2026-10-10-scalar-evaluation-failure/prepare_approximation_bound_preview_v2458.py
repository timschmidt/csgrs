"""Freeze a shared bound-producer extraction without modifying SOURCE."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
OUT = E / "approximation-bound-candidate-v2458"
PROPOSAL = E / "approximation-bound-shared-proposal-v2458.rs"
PREFIX = "crates/hyperreal/"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    OUT.mkdir()
    original_node = (SOURCE / PREFIX / "src/computable/node.rs").read_text()
    original_structural = (SOURCE / PREFIX / "src/computable/node/structural_analysis.rs").read_text()
    start = original_structural.index("    fn bound_from_approx(")
    end = original_structural.index("    fn rational_kernel_sign(", start)
    original_method = original_structural[start:end]
    assert original_method.endswith("    }\n\n")
    (OUT / "original-method.rs").write_text(original_method)
    node_marker = 'include!("node/structural_analysis.rs");'
    assert original_node.count(node_marker) == 1
    new_node = original_node.replace(node_marker, node_marker + '\ninclude!("node/approximation_bound.rs");')
    new_structural = original_structural[:start] + original_structural[end:]
    candidate = {PREFIX + "src/computable/node.rs": new_node,
                 PREFIX + "src/computable/node/structural_analysis.rs": new_structural,
                 PREFIX + "src/computable/node/approximation_bound.rs": PROPOSAL.read_text()}
    before = {PREFIX + "src/computable/node.rs": original_node,
              PREFIX + "src/computable/node/structural_analysis.rs": original_structural}
    patches = []
    for name, text in candidate.items():
        path = OUT / "candidate" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        if name in before:
            old = OUT / "original" / name
            old.parent.mkdir(parents=True, exist_ok=True)
            old.write_text(before[name])
        patches.append("".join(difflib.unified_diff(before.get(name, "").splitlines(True), text.splitlines(True),
            fromfile="a/" + name, tofile="b/" + name, n=0)))
    (OUT / "candidate.patch").write_text("".join(patches))
    verification = SOURCE / PREFIX / "verification"
    bounds = (verification / "computable_bounds.rs").read_text()
    external_start = bounds.index("verus! {")
    external_end = bounds.index('\ninclude!("../src/computable/node/bounds.rs");', external_start)
    removed = bounds[external_start:external_end]
    assert "#[verifier::external_type_specification]" in removed and "pub struct ImportedSign(Sign);" in removed
    bounds = bounds[:external_start] + bounds[external_end:]
    bounds = bounds.replace("use num::bigint::Sign;", "use num::bigint::{BigInt, Sign};")
    bounds = bounds.replace('"../src/computable/node/bounds.rs"', '"' + str(SOURCE / PREFIX / "src/computable/node/bounds.rs") + '"')
    bounds = bounds.replace('"bound_denotation.rs"', '"' + str(verification / "bound_denotation.rs") + '"')
    shared = OUT / "candidate" / PREFIX / "src/computable/node/approximation_bound.rs"
    bounds += '\n// Only the static producer method is under test; no graph representation is imported.\n'
    bounds += '#[verus_verify]\nstruct Computable;\ninclude!("' + str(shared) + '");\n'
    (OUT / "computable_bounds.rs").write_text(bounds)
    entrypoint = (verification / "lib.rs").read_text()
    entrypoint = entrypoint.replace('"../src/verified/mod.rs"', '"' + str(SOURCE / PREFIX / "src/verified/mod.rs") + '"')
    entrypoint = entrypoint.replace('"../src/structural.rs"', '"' + str(SOURCE / PREFIX / "src/structural.rs") + '"')
    for name in ["bit_length_model", "computable_bounds", "computable_cache", "digit_scan_model",
                 "integer_approximation_model", "magnitude_model", "rational_model", "real_approximation_model", "word_scan_model"]:
        source = OUT / "computable_bounds.rs" if name == "computable_bounds" else verification / (name + ".rs")
        marker = f"mod {name};"
        assert entrypoint.count(marker) == 1
        entrypoint = entrypoint.replace(marker, f'#[path = "{source}"]\n' + marker)
    (OUT / "proof.rs").write_text(entrypoint)
    files = {str(p.relative_to(OUT)): sha(p) for p in sorted(OUT.rglob("*")) if p.is_file()}
    protocol = {"status": "Shared producer preview frozen before proof or runtime execution",
        "runner_sha256": sha(Path(__file__)), "proposal_sha256": sha(PROPOSAL),
        "retained_checkpoint_sha256": sha(checkpoint_path), "source_record_sha256": checkpoint["source_record_sha256"],
        "preview_files_sha256": files, "candidate_source_paths": list(candidate),
        "original_method_sha256": sha(OUT / "original-method.rs"),
        "source_changed": False, "main_changed": False,
        "scope": "Prepare the actual three-file extraction: a node include, removal from structural_analysis and the new shared associated-function file. The proof target includes that exact proposed file, with a fieldless owner because this static method never reads graph fields. All bound/model/kernel bodies remain the existing SOURCE files. Remove only the redundant external Sign type declaration because the imported dependency now proves its actual enum. The new annotations require normalized sign/magnitude and a u64 storage count, prove exact metadata output and all real values satisfying the one-unit error contract. Statement bindings expose results to ghost proofs; the trailing-zero scan remains inside the successful-exponent short circuit. No SOURCE edit, runtime admission, whole-node proof or retained count increase occurs in this preparation."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    print(json.dumps({"preview_files": len(files), "candidate_source_files": len(candidate),
        "source_changed": False, "proposal_sha256": sha(PROPOSAL)}))


if __name__ == "__main__":
    main()
