"""Review the pinned metadata implementation and preserve lazy producer decisions."""
import gzip
import hashlib
import json
from pathlib import Path
import tomllib
import tarfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def section(path, first, last):
    return {"path": str(path), "sha256": sha(path), "first_line": first, "last_line": last,
            "text": "\n".join(path.read_text().splitlines()[first - 1:last])}


def main():
    output = E / "numerical-bound-metadata-review-v2279-v2.json"
    assert not output.exists()
    source_record = E / "eager-bound-decoder-baseline-v2257/candidate-source-hashes.json.gz"
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    lock_path = SOURCE / "Cargo.lock"
    lock = tomllib.loads(lock_path.read_text())
    packages = [p for p in lock["package"] if p["name"] == "num-bigint"]
    assert len(packages) == 1 and packages[0]["version"] == "0.4.6"
    registry = Path("/tmp/csgrs-properties-cargo/registry")
    dependency = registry / "src/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6"
    archive = registry / "cache/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6.crate"
    assert sha(archive) == packages[0]["checksum"]
    with tarfile.open(archive, "r:gz") as package:
        for name in ["src/biguint.rs", "src/bigint.rs", "src/lib.rs"]:
            assert package.extractfile("num-bigint-0.4.6/" + name).read() == (dependency / name).read_bytes()
        license_text = package.extractfile("num-bigint-0.4.6/LICENSE-MIT").read().decode()
    dependency_sections = [section(dependency / "src/biguint.rs", 31, 36),
        section(dependency / "src/biguint.rs", 848, 873),
        section(dependency / "src/biguint.rs", 988, 1004),
        section(dependency / "src/bigint.rs", 541, 546),
        section(dependency / "src/bigint.rs", 932, 935),
        section(dependency / "src/bigint.rs", 949, 952),
        section(dependency / "src/lib.rs", 225, 242)]
    production = section(SOURCE / "crates/hyperreal/src/computable/node/structural_analysis.rs", 785, 812)
    assert "let exact_msd = msd.is_some()" in production["text"]
    assert "&& magnitude.trailing_zeros()" in production["text"]
    assert "if magnitude_bits <= 1" in production["text"]
    assert "magnitude.trailing_ones() == magnitude_bits" in production["text"]
    paths = ["crates/hyperreal/src/verified/word.rs",
        "crates/hyperreal/src/computable/node/bounds.rs",
        "crates/hyperreal/verification/bound_denotation.rs",
        "crates/hyperreal/verification/real_approximation_model.rs",
        "crates/hyperreal/verification/magnitude_model.rs",
        "crates/hyperreal/verification/bit_length_model.rs",
        "crates/hyperreal/verification/computable_bounds.rs"]
    output.write_text(json.dumps({
        "status": "Read-only next-proof design review; no implementation or formal coverage added",
        "reader_failure": "numerical-bound-metadata-reader-failure-v2279.json",
        "reader_failure_sha256": sha(E / "numerical-bound-metadata-reader-failure-v2279.json"),
        "schema_correction": "Compare source bytes directly to the locked, hash-verified crate archive because this extraction has no checksum sidecar. No source, test, proof or admission claim changes.",
        "source_record_sha256": sha(source_record), "runner_sha256": sha(Path(__file__)),
        "predecessor_review": "numerical-bound-producer-next-review-v2180.json",
        "predecessor_review_sha256": sha(E / "numerical-bound-producer-next-review-v2180.json"),
        "lock_sha256": sha(lock_path), "dependency": packages[0],
        "dependency_archive_sha256": sha(archive), "dependency_files_match_locked_archive": True, "dependency_license": license_text,
        "dependency_sections_read": dependency_sections, "production_section_read": production,
        "supporting_source_sha256": {p: sha(SOURCE / p) for p in paths},
        "findings": [
            "The locked dependency is num-bigint 0.4.6. BigUint stores a Vec of target-selected u32/u64 digits. bits reads the last digit's leading zeros, while trailing_zeros and trailing_ones search from the low end. BigInt sign and magnitude are field accessors. These inspected bodies do not establish all constructors' normalization or a Verus refinement of dependency storage.",
            "The production path returns Unknown at zero/one magnitude without the trailing-bit searches. Above that threshold it computes trailing ones, then a checked exponent, and only calls trailing_zeros when that exponent is representable. Moving a power-of-two flag to an eagerly evaluated helper argument could add a low-limb scan when the exponent is absent. Any later extraction must preserve this short-circuit behavior or measure its cost explicitly.",
            "For magnitude m with b bits, the candidate upper exponent is p+b when m is all ones, otherwise p+b-1. The closed error interval uses m+1: an all-ones magnitude reaches the next power exactly, so a strict upper certificate requires the raised exponent. For an exact binade, excluding m=2^(b-1) ensures m-1 is at least that lower power; excluding all ones keeps m+1 strictly below 2^b. These are mathematical derivations for a future checked lemma, not newly verified claims.",
            "With b>1, p in i32 and a subtraction of at most one, the mathematical exponent is at least i32::MIN+1. The producer's absent exponent therefore comes from the upper range limit. It retains a separated sign even when magnitude metadata is absent. The general checked_bit_length_msd helper still handles both range limits for other callers.",
            "Existing separated_approximation_yields_a_bound requires endpoint inequalities; checked_bit_length_msd and with_sign_msd provide actual production contracts. A useful next proof can connect bit-length/all-ones/power-of-two premises, the checked exponent and constructor contract to those endpoint inequalities. A model bridge alone must remain labeled conditional; it does not put bound_from_approx or BigInt methods inside the verification target.",
        ],
        "suggested_order_after_decoder_retention": [
            "Prove the endpoint inequalities from explicit integer bit-pattern premises and the existing production exponent/constructor contracts, with mutations for every boundary and sign obligation.",
            "Connect actual dependency bit metadata and normalized storage to those premises without assuming new method specifications. Account for both target digit widths and range conversions.",
            "If runtime extraction is required, freeze a separate candidate and preserve metadata short-circuiting, unchanged public/full-size fixtures, generated code and original resource limits before evaluating it.",
            "Keep valid approximations, independent real existence, structural-before-numeric publication and concurrent atomic ordering as distinct caller/invariant obligations.",
        ],
        "source_changed": False, "new_formal_units": 0, "new_test_or_cost_processes": 0,
        "runtime_retained": False, "source_files_verified": len(sources), "main_files_verified": len(guard),
        "limits": "Only the listed sections and previously named model/contract definitions were read. Hashing other files does not credit a whole-file audit. This review changes no gate, status, runtime or proof obligation; completing the eager decoder's pending broad/replay/fact checks remains first.",
    }, indent=2) + "\n")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    print(json.dumps({"dependency": "num-bigint 0.4.6", "checksums_verified": True,
        "short_circuit_constraint_recorded": True, "source_changed": False, "new_formal_units": 0}))


if __name__ == "__main__":
    main()
