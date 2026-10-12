"""Record the remaining source-to-model gap after reviewing the numerical bound producer."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def body(text, signature):
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


def main():
    output = E / "numerical-bound-producer-next-review-v2180.json"
    assert not output.exists()
    proof_path = E / "bound-decoder-proof-validation-v2171.json"
    proof = read(proof_path)
    validation = read(E / "bound-decoder-candidate-validation-v2180.json")
    assert validation["complete"] and validation["source_record_sha256"] == proof["source_record_sha256"]
    source_record = E / proof["source_record"]
    assert sha(source_record) == proof["source_record_sha256"]
    sources = json.loads(gzip.decompress(source_record.read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    production = "crates/hyperreal/src/computable/node/structural_analysis.rs"
    word = "crates/hyperreal/src/verified/word.rs"
    denotation = "crates/hyperreal/verification/bound_denotation.rs"
    bits = "crates/hyperreal/verification/bit_length_model.rs"
    include = "crates/hyperreal/verification/computable_bounds.rs"
    files = [production, word, denotation, bits, include]
    inputs = {n: sha(SOURCE / n) for n in files}
    producer_text = (SOURCE / production).read_text()
    producer = body(producer_text, "    fn bound_from_approx(")
    publication = body(producer_text, "    fn store_cache_value(")
    assert all(term in producer for term in ["magnitude.bits()", "magnitude.trailing_ones()",
        "magnitude.trailing_zeros()", "checked_bit_length_msd", "BoundInfo::with_sign_msd"])
    assert "include!(\"../src/computable/node/bounds.rs\")" in (SOURCE / include).read_text()
    report = read(E / proof["fresh_verification_phase"] / "proof-reports/verus.json")
    names = report["func-details"]
    assert "hyperreal_proofs::computable_cache::AtomicFacts::decode_bound" in names
    assert not any("bound_from_approx" in name for name in names)
    assert "hyperreal_proofs::verified::word::checked_bit_length_msd" in names
    assert "hyperreal_proofs::computable_bounds::separated_approximation_yields_a_bound" in names
    main = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert all(sha(ROOT / n) == h for n, h in main.items())
    record = {"status": "Read-only review of the next production/model gap; no new proof or source change",
        "source_record_sha256": proof["source_record_sha256"], "decoder_proof_record": proof_path.name,
        "decoder_proof_record_sha256": sha(proof_path), "reviewed_inputs_sha256": inputs,
        "reviewed_production_functions": {"bound_from_approx": producer, "store_cache_value": publication},
        "reviewed_sections": {
            word: "checked_bit_length_msd and its exact mathematical range contract; adjacent upper_bit_length_msd for the distinct below-minimum clamping case.",
            denotation: "separated_approximation_yields_a_bound and its certified closed-interval endpoint premises.",
            bits: "Complete file: normalized-limb denotation, leading-zero/bit-length connection, binary-shift length and width-based order.",
            include: "Complete include wrapper: it includes bounds.rs and bound_denotation.rs, not structural_analysis.rs.",
        },
        "findings": [
            "The actual decoder is now included in the conditional target. Computable::bound_from_approx is still absent; the new codec proof does not justify the incoming certificate.",
            "The producer declines magnitudes zero/one, obtains the bigint bit length, raises the upper exponent when every magnitude bit is one, and withholds an exact binade for either a power of two or one below the next power. It delegates exponent range arithmetic to the already verified checked_bit_length_msd.",
            "The existing denotation lemma requires the closed one-unit magnitude interval to fit a strict upper bound and, when exact, a lower binade. Those premises are not yet connected to the runtime bits/trailing_ones/trailing_zeros/sign calls or to BigUint storage normalization.",
            "The normalized-limb bit-length lemmas are conditional on their sequence representation. Actual BigUint storage/import and the leading/trailing-bit metadata remain separate refinement obligations; do not replace them with assumed method specifications or credit a copied model as production coverage.",
            "store_cache_value checks cancellation/exhaustion, obtains the numerical bound, discovers structural evidence, rechecks exhaustion, and then publishes the bound before storing the numeric approximation. This source reading does not verify operand existence, approximate-value validity, concurrency or that ordering's invariant.",
        ],
        "candidate_next_work_after_decoder_qualification": "Connect the numerical producer's bit-pattern decisions and checked exponent to the closed-interval certificate lemma. Preserve all all-ones/power-of-two neighbors, both signs, exponent boundaries and existing public warming/work-budget behavior. Keep the actual BigInt metadata/import, independent existence and atomic publication obligations explicit; any source extraction needs its own correctness and compiled-code/resource evidence.",
        "source_files_verified": len(sources), "main_files_verified": len(main),
        "source_changed": False, "new_formal_units": 0, "runtime_retained": False,
        "limits": "Selected functions and named sections were read; file hashing does not imply a whole-file source audit. The decoder's own remaining binary/cost/replay gates take precedence. This review closes no property, target, whole-crate obligation or main/native/full-board admission requirement."}
    output.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"review": output.name, "new_formal_units": 0, "source_changed": False}))


if __name__ == "__main__":
    main()
