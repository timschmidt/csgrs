"""Freeze probes of the exact storage operations used by the locked dependency."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

FORWARD = r'''
pub(crate) fn forward_TYPE(words: &[TYPE]) -> (result: Option<usize>)
    ensures match result {
        Some(index) => first_nonzero_TYPE(words@, index as int),
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == 0,
    },
{
    words.iter().position(|&digit| digit != 0)
}
'''

REVERSE = r'''
pub(crate) fn reverse_TYPE(words: &[TYPE]) -> (result: Option<usize>)
    ensures match result {
        Some(index) => 0 <= index < words.len() && words@[index as int] != 0
            && forall|higher: int| index < higher < words.len() ==> #[trigger] words@[higher] == 0,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == 0,
    },
{
    words.iter().rposition(|&digit| digit != 0)
}
'''

SHRINK = r'''
pub(crate) fn shrink_TYPE(words: &mut Vec<TYPE>)
    ensures words@ == old(words)@,
{
    if words.len() < words.capacity() / 4 {
        words.shrink_to_fit();
    }
}
'''

TRUNCATE = r'''
pub(crate) fn truncate_TYPE(words: &mut Vec<TYPE>, length: usize)
    requires length <= old(words).len(),
        length == 0 || old(words)@[length as int - 1] > 0,
        forall|index: int| length <= index < old(words).len() ==> #[trigger] old(words)@[index] == 0,
    ensures word_value_TYPE(words@) == word_value_TYPE(old(words)@), normalized_TYPE(words@),
{
    proof { typed_zero_suffix_truncation_TYPE(words@, length as int); }
    words.truncate(length);
}
'''


def main():
    record = E / "storage-contract-probe-preparation-v2387.json"
    assert not record.exists()
    parent = E / "probe_word_scan_v2377.py"
    imports = "//! Test pinned dependency call contracts without changing production source.\n#![feature(proc_macro_hygiene)]\n"
    for name, path in [("verified", "src/verified/mod.rs"),
        ("magnitude_model", "verification/magnitude_model.rs"),
        ("bit_length_model", "verification/bit_length_model.rs"),
        ("digit_scan_model", "verification/digit_scan_model.rs"),
        ("word_scan_model", "verification/word_scan_model.rs")]:
        imports += '#[path = ' + json.dumps(str(SOURCE / "crates/hyperreal" / path)) + ']\nmod ' + name + ';\n'
    imports += "mod storage_probe {\nuse crate::word_scan_model::*;\nuse vstd::prelude::*;\n\nverus! {\n"
    rows = []
    for offset, (kind, template) in enumerate([("forward", FORWARD), ("reverse", REVERSE),
                                             ("shrink", SHRINK), ("truncate", TRUNCATE)]):
        version = 2387 + offset
        proposal = E / f"storage-{kind}-proposal-v{version}.rs"
        runner = E / f"probe_storage_{kind}_v{version}.py"
        assert not proposal.exists() and not runner.exists()
        proposal.write_text(imports + "\n".join(template.replace("TYPE", f"u{width}") for width in [32, 64]) + "\n}\n}\n")
        code = parent.read_text()
        for before, after in [
            ("word-scan-proposal-v2377.rs", proposal.name),
            ("word-scan-probe-v2377", f"storage-{kind}-probe-v{version}"),
            ("typed-api-prototype-v2373-state.json", "typed-api-prototype-v2386-state.json"),
            ("163bf723328ab72df0a56025f9a55960eda13eb4c6767669965b61e66333508a", "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"),
            ("typed-api-prototype-v2367-digit-scan-final-verus", "typed-api-prototype-v2380-word-scan-verus"),
            ("Probe typed word views and scans against retained digit lemmas.", f"Probe the pinned {kind} storage-operation contract."),
            ("Do typed u32/u64 word views, scan witnesses and zero-suffix truncation connect to retained generic digit denotation and actual native factor helpers, with explicit count-range premises?",
             f"Does the pinned verifier support the dependency's exact {kind} call expression for u32/u64 storage, and can its existing contract prove the stated result?"),
        ]:
            assert code.count(before) == 1, before
            code = code.replace(before, after)
        ast.parse(code)
        runner.write_text(code)
        rows.append({"kind": kind, "version": version, "proposal": proposal.name,
            "proposal_sha256": sha(proposal), "runner": runner.name, "runner_sha256": sha(runner),
            "phase": f"storage-{kind}-probe-v{version}"})
    inspected = {}
    for name in ["iter", "slice", "vec"]:
        path = Path(f"/tmp/csgrs-pinned-verus-{name}-671956e.rs")
        inspected[name] = {"sha256": sha(path), "url": "https://raw.githubusercontent.com/verus-lang/verus/671956ec527d3b7164779f767bdbfe769bedce6c/source/vstd/std_specs/" + name + ".rs"}
    record.write_text(json.dumps({"status": "Frozen before four sequential storage-contract probes",
        "preparation_sha256": sha(Path(__file__)), "parent_runner_sha256": sha(parent),
        "retained_checkpoint": "typed-api-prototype-v2386-state.json", "probes": rows,
        "pinned_specification_sources_inspected": inspected, "source_changed": False,
        "scope": "Probe the exact position, rposition, capacity/shrink and truncate call forms used by num-bigint 0.4.6 on both native widths. These wrappers are support harnesses, not imported production bodies or retained coverage. The truncate proof consumes the already-proved typed suffix premise. Add no assumptions, trusted method specifications or substitutions for an unsupported call. Preserve every result separately and run probes sequentially."}, indent=2) + "\n")
    print(json.dumps({"frozen_storage_contract_probes": len(rows), "source_changed": False}))


if __name__ == "__main__":
    main()
