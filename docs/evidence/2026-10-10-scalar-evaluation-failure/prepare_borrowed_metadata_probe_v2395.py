"""Prototype executable slice scans, logical normalization and checked metadata."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

BODY = r'''
pub(crate) fn first_different_TYPE(words: &[TYPE], skipped: TYPE) -> (result: Option<usize>)
    ensures match result {
        Some(index) => index < words.len() && words@[index as int] != skipped
            && forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == skipped,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == skipped,
    },
{
    let mut index = 0;
    while index < words.len()
        invariant index <= words.len(),
            forall|lower: int| 0 <= lower < index ==> #[trigger] words@[lower] == skipped,
        decreases words.len() - index,
    {
        if words[index] != skipped {
            return Some(index);
        }
        index += 1;
    }
    None
}

pub(crate) fn last_nonzero_TYPE(words: &[TYPE]) -> (result: Option<usize>)
    ensures match result {
        Some(index) => index < words.len() && words@[index as int] != 0
            && forall|higher: int| index < higher < words.len() ==> #[trigger] words@[higher] == 0,
        None => forall|index: int| 0 <= index < words.len() ==> #[trigger] words@[index] == 0,
    },
{
    let mut length = words.len();
    while length > 0
        invariant length <= words.len(),
            forall|higher: int| length <= higher < words.len() ==> #[trigger] words@[higher] == 0,
        decreases length,
    {
        if words[length - 1] != 0 {
            return Some(length - 1);
        }
        length -= 1;
    }
    None
}

pub(crate) fn trim_zero_suffix_TYPE(words: &mut Vec<TYPE>)
    ensures word_value_TYPE(final(words)@) == word_value_TYPE(old(words)@),
        normalized_TYPE(final(words)@),
{
    let length = match last_nonzero_TYPE(words.as_slice()) {
        Some(index) => index + 1,
        None => 0,
    };
    proof { typed_zero_suffix_truncation_TYPE(words@, length as int); }
    words.truncate(length);
}

pub(crate) fn zero_count_TYPE(words: &[TYPE]) -> (result: Option<u64>)
    requires WIDTH * words.len() <= u64::MAX,
    ensures match result {
        Some(count) => has_odd_factor(word_value_TYPE(words@), count as nat)
            && count < WIDTH * words.len(),
        None => word_value_TYPE(words@) == 0,
    },
{
    match first_different_TYPE(words, 0) {
        Some(index) => {
            let count = words[index].trailing_zeros();
            proof {
                typed_zero_scan_TYPE(words@, index as int);
                digit_scan_count_fits_u64(words.len() as nat, WIDTH, index as nat, count as nat);
            }
            Some(index as u64 * WIDTH + u64::from(count))
        },
        None => {
            proof { typed_zero_scan_none_TYPE(words@); }
            None
        },
    }
}

pub(crate) fn one_count_TYPE(words: &[TYPE]) -> (result: u64)
    requires WIDTH * words.len() <= u64::MAX,
    ensures has_odd_factor(word_value_TYPE(words@) + 1, result as nat),
        result <= WIDTH * words.len(),
{
    match first_different_TYPE(words, TYPE::MAX) {
        Some(index) => {
            let count = words[index].trailing_ones();
            proof {
                typed_one_scan_TYPE(words@, index as int);
                digit_scan_count_fits_u64(words.len() as nat, WIDTH, index as nat, count as nat);
            }
            index as u64 * WIDTH + u64::from(count)
        },
        None => {
            proof { typed_one_scan_none_TYPE(words@); }
            words.len() as u64 * WIDTH
        },
    }
}

pub(crate) fn bit_count_TYPE(words: &[TYPE]) -> (result: u64)
    requires normalized_TYPE(words@), WIDTH * words.len() <= u64::MAX,
    ensures result == word_bit_length_TYPE(words@),
        word_value_TYPE(words@) == 0 <==> result == 0,
        result > 0 ==> has_bit_length(word_value_TYPE(words@), result as nat),
{
    proof { typed_normalized_bit_length_TYPE(words@); }
    if words.len() == 0 {
        0
    } else {
        let count = words[words.len() - 1].leading_zeros();
        words.len() as u64 * WIDTH - u64::from(count)
    }
}
'''


def main():
    parent_proposal = E / "storage-truncate-proposal-v2394.rs"
    parent_runner = E / "probe_storage_truncate_v2394.py"
    proposal = E / "borrowed-metadata-proposal-v2395.rs"
    runner = E / "probe_borrowed_metadata_v2395.py"
    record = E / "borrowed-metadata-probe-preparation-v2395.json"
    assert all(not p.exists() for p in [proposal, runner, record])
    prefix = parent_proposal.read_text().split("mod storage_probe {\n", 1)[0]
    prefix += "mod borrowed_metadata {\nuse crate::word_scan_model::*;\n"
    prefix += "use crate::digit_scan_model::has_odd_factor;\nuse crate::magnitude_model::has_bit_length;\n"
    prefix += "use vstd::prelude::*;\n\nverus! {\n"
    body = "\n".join(BODY.replace("TYPE", f"u{width}").replace("WIDTH", str(width)) for width in [32, 64])
    proposal.write_text(prefix + body + "\n}\n}\n")
    code = parent_runner.read_text()
    for old, new in [
        (parent_proposal.name, proposal.name),
        ("storage-truncate-probe-v2394", "borrowed-metadata-probe-v2395"),
        ("Probe the pinned truncate storage-operation contract.", "Probe an executable borrowed-slice metadata prototype."),
        ("Does the pinned verifier support the dependency's exact truncate call expression for u32/u64 storage, and can its existing contract prove the stated result?",
         "Can explicit indexed scans, value-preserving truncation and native bit counts discharge the retained typed-word contracts and machine arithmetic under explicit range premises?"),
    ]:
        assert code.count(old) == 1, old
        code = code.replace(old, new)
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({"status": "Frozen before checking the executable borrowed-slice prototype",
        "preparation_sha256": sha(Path(__file__)), "parent_proposal_sha256": sha(parent_proposal),
        "parent_runner_sha256": sha(parent_runner), "proposal_sha256": sha(proposal), "runner_sha256": sha(runner),
        "executable_functions": 12, "source_changed": False,
        "scope": "Six functions per native width: first different, last nonzero, logical normalization and three bit counts. All memory accesses and count arithmetic must verify using existing pinned contracts. This is a new implementation prototype, not a proof of the dependency's position/rposition/capacity/shrink bodies or an admitted runtime replacement. BigUint import, normalization allocation policy, actual callers, count-range premises, fixed-baseline correctness/cost/resource gates and whole-crate requirements remain. No new trusted assumptions or retained coverage credit."}, indent=2) + "\n")
    print(json.dumps({"frozen_borrowed_metadata_prototype": True, "executable_functions": 12, "source_changed": False}))


if __name__ == "__main__":
    main()
