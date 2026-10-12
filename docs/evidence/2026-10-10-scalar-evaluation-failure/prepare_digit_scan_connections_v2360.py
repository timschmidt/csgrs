"""Connect generic digit arithmetic to the actual retained metadata lemmas."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "digit-scan-model-proposal-v2359.rs"
    connections = E / "digit-scan-connections-proposal-v2360.rs"
    model = SOURCE / "crates/hyperreal/verification/bit_length_model.rs"
    proposal = E / "digit-scan-connected-proposal-v2360.rs"
    runner_parent = E / "probe_digit_scan_model_v2359.py"
    runner = E / "probe_digit_scan_connections_v2360.py"
    record = E / "digit-scan-connection-preparation-v2360.json"
    assert all(not p.exists() for p in [proposal, runner, record])
    text = parent.read_text()
    assert text.endswith("}\n")
    start = model.read_text().index("pub(crate) proof fn leading_zeros_determine_word_bit_length(")
    end = model.read_text().index("\n}\n", start) + 3
    leading = model.read_text()[start:end]
    leading = leading.replace("pub(crate) proof fn", "pub proof fn")
    leading = leading.replace("leading_zeros_determine_word_bit_length", "u32_digit_bit_length")
    leading = leading.replace("u64", "u32").replace("64", "32")
    # The pinned vstd provides one constant-powers lemma for both widths.
    leading = leading.replace("lemma2_to32()", "lemma2_to64()")
    imports = '#![feature(proc_macro_hygiene)]\n'
    for module, relative in [
        ("verified", "src/verified/mod.rs"),
        ("magnitude_model", "verification/magnitude_model.rs"),
        ("bit_length_model", "verification/bit_length_model.rs"),
    ]:
        imports += '#[path = ' + json.dumps(str(SOURCE / "crates/hyperreal" / relative)) + ']\n'
        imports += 'mod ' + module + ';\n'
    # Put crate-level attributes before ordinary items and after the module docs.
    before = "use vstd::prelude::*;"
    assert text.count(before) == 1
    text = text.replace(before, imports + before +
        "\nuse crate::magnitude_model::has_bit_length;\nuse vstd::std_specs::bits::*;")
    text = text[:-2] + connections.read_text() + "\n" + leading + "\n}\n"
    proposal.write_text(text)
    code = runner_parent.read_text()
    for before, after in [
        ("digit-scan-model-proposal-v2359.rs", proposal.name),
        ('out = E / "digit-scan-model-probe-v2359"', 'out = E / "digit-scan-connections-probe-v2360"'),
        ("Probe generic digit-scan arithmetic without changing the candidate.",
         "Probe digit-scan connections to actual retained metadata lemmas."),
        ("Do radix-parametric prefix identities and first matching digit premises establish whole-value odd factors for zero and one scans, including empty and all-maximal storage?",
         "Do generic digit scans, normalization and native u32 leading-zero arithmetic connect to the actual retained bit-length and metadata flag lemmas?"),
        ("Use only the pinned vstd standard-library axioms already supplied by Verus. Introduce no external method specification, assumption or trusted arithmetic axiom.",
         "Compile the actual retained verified helpers, magnitude model and bit-length model alongside the generic scan proposal. Use the pinned vstd axioms and no new trusted assumptions. Report verifier units separately from new proof functions; imported existing units are not new coverage."),
    ]:
        assert code.count(before) == 1, before
        code = code.replace(before, after)
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({
        "status": "Frozen before the connected support probe",
        "preparation_sha256": sha(Path(__file__)),
        "inputs_sha256": {str(p.relative_to(ROOT)): sha(p) for p in [parent, connections, model, runner_parent]},
        "proposal_sha256": sha(proposal), "runner_sha256": sha(runner),
        "added_proof_functions": 6, "previous_generic_proof_functions": 11,
        "retained_checkpoint": "typed-api-prototype-v2358-state.json",
        "source_changed": False,
        "scope": "Five generic connections and a u32 leading-zero lemma, with actual retained imports. These are conditional mathematical proofs, not an import of BigUint storage, Vec iterator behavior or machine-width counts. No retained coverage credit until integration and guards. Preserve completed probes unchanged.",
    }, indent=2) + "\n")
    print(json.dumps({"frozen_connected_digit_probe": True, "candidate_source_changed": False}))


if __name__ == "__main__":
    main()
