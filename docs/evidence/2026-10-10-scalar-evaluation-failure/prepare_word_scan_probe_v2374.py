"""Freeze typed word-sequence connections to the retained generic digit model."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

COMMON = r'''
pub proof fn trimmed_zero_suffix_preserves_value(digits: Seq<nat>, width: nat, length: int)
    requires 0 <= length <= digits.len(),
        forall|index: int| length <= index < digits.len() ==> #[trigger] digits[index] == 0,
    ensures digit_value(digits.subrange(0, length), width) == digit_value(digits, width),
{
    let suffix = digits.subrange(length, digits.len() as int);
    assert forall|index: int| 0 <= index < suffix.len() implies #[trigger] suffix[index] == 0 by {
        assert(suffix[index] == digits[length + index]);
    }
    digit_prefix_zero(suffix, width, suffix.len() as int);
    digit_prefix_split(digits, width, length, digits.len() as int);
    digit_prefix_equal(digits, digits.subrange(0, length), width, length);
    assert(digit_weight(width, length as nat) * 0nat == 0) by (nonlinear_arith);
}

pub proof fn u64_digit_prefix_matches_production(words: Seq<u64>, length: int)
    requires 0 <= length <= words.len(),
    ensures digit_prefix(word_digits_u64(words), 64, length)
        == crate::verified::limbs::prefix(words, length),
    decreases length,
{
    if length > 0 {
        u64_digit_prefix_matches_production(words, length - 1);
    }
}

pub proof fn u64_digit_value_matches_production(words: Seq<u64>)
    ensures word_value_u64(words) == crate::verified::limbs::value(words),
{
    u64_digit_prefix_matches_production(words, words.len() as int);
}

pub proof fn digit_scan_count_fits_u64(length: nat, width: nat, index: nat, within: nat)
    requires width > 0, index < length, within < width,
        width * length <= u64::MAX,
    ensures index <= u64::MAX, width * index <= u64::MAX,
        width * index + within < width * length,
        width * index + within <= u64::MAX,
{
    assert(index <= u64::MAX && width * index <= u64::MAX
        && width * index + within < width * length
        && width * index + within <= u64::MAX) by (nonlinear_arith)
        requires width > 0, index < length, within < width,
            width * length <= u64::MAX;
}
'''

TYPED = r'''
pub open spec fn word_digits_TYPE(words: Seq<TYPE>) -> Seq<nat> {
    Seq::new(words.len(), |index: int| words[index] as nat)
}

pub open spec fn word_value_TYPE(words: Seq<TYPE>) -> nat {
    digit_value(word_digits_TYPE(words), WIDTH)
}

pub open spec fn normalized_TYPE(words: Seq<TYPE>) -> bool {
    words.len() == 0 || words[words.len() - 1] > 0
}

pub open spec fn word_bit_length_TYPE(words: Seq<TYPE>) -> nat {
    if words.len() == 0 { 0 }
    else { (WIDTH * words.len() - TYPE_leading_zeros(words[words.len() - 1])) as nat }
}

pub open spec fn first_nonzero_TYPE(words: Seq<TYPE>, index: int) -> bool {
    0 <= index < words.len() && words[index] != 0
        && forall|lower: int| 0 <= lower < index ==> #[trigger] words[lower] == 0
}

pub open spec fn first_nonmaximal_TYPE(words: Seq<TYPE>, index: int) -> bool {
    0 <= index < words.len() && words[index] != TYPE::MAX
        && forall|lower: int| 0 <= lower < index ==> #[trigger] words[lower] == TYPE::MAX
}

pub proof fn typed_digit_bounds_TYPE(words: Seq<TYPE>)
    ensures word_digits_TYPE(words).len() == words.len(),
        forall|index: int| 0 <= index < words.len()
            ==> #[trigger] word_digits_TYPE(words)[index] < pow2(WIDTH),
        normalized_digits(word_digits_TYPE(words), WIDTH) == normalized_TYPE(words),
{
    lemma2_to64();
}

pub proof fn typed_normalized_bit_length_TYPE(words: Seq<TYPE>)
    requires normalized_TYPE(words),
    ensures word_value_TYPE(words) == 0 <==> words.len() == 0,
        word_bit_length_TYPE(words) == 0 <==> words.len() == 0,
        word_bit_length_TYPE(words) <= WIDTH * words.len(),
        words.len() > 0 ==> has_bit_length(word_value_TYPE(words), word_bit_length_TYPE(words)),
{
    typed_digit_bounds_TYPE(words);
    normalized_zero_iff_empty(word_digits_TYPE(words), WIDTH);
    if words.len() > 0 {
        let top = words[words.len() - 1];
        LEADING_PROOF(top);
        let bits = (WIDTH - TYPE_leading_zeros(top)) as nat;
        normalized_digit_bit_length(word_digits_TYPE(words), WIDTH, bits);
    }
}

pub proof fn typed_zero_scan_TYPE(words: Seq<TYPE>, index: int)
    requires first_nonzero_TYPE(words, index),
    ensures has_odd_factor(word_value_TYPE(words),
            WIDTH * index as nat + TYPE_trailing_zeros(words[index]) as nat),
        WIDTH * index as nat + TYPE_trailing_zeros(words[index]) as nat < WIDTH * words.len(),
{
    let digits = word_digits_TYPE(words);
    let word = words[index];
    ZERO_PROOF(word);
    let zeros = TYPE_trailing_zeros(word) as nat;
    let odd = (word >> TYPE_trailing_zeros(word)) as nat;
    assert forall|lower: int| 0 <= lower < index implies #[trigger] digits[lower] == 0 by {
        assert(words[lower] == 0);
    }
    first_nonzero_digit_factor(digits, WIDTH, index, zeros, odd);
}

pub proof fn typed_one_scan_TYPE(words: Seq<TYPE>, index: int)
    requires first_nonmaximal_TYPE(words, index),
    ensures has_odd_factor(word_value_TYPE(words) + 1,
            WIDTH * index as nat + TYPE_trailing_ones(words[index]) as nat),
        WIDTH * index as nat + TYPE_trailing_ones(words[index]) as nat < WIDTH * words.len(),
{
    let digits = word_digits_TYPE(words);
    let word = words[index];
    crate::bit_length_model::trailing_ones_factor_TYPE(word);
    axiom_TYPE_trailing_ones(word);
    lemma2_to64();
    let ones = TYPE_trailing_ones(word) as nat;
    let odd = (word as nat + 1) / pow2(ones);
    assert forall|lower: int| 0 <= lower < index implies #[trigger] digits[lower] + 1 == pow2(WIDTH) by {
        assert(words[lower] == TYPE::MAX);
    }
    first_nonmaximal_digit_factor(digits, WIDTH, index, ones, odd);
}

pub proof fn typed_zero_scan_none_TYPE(words: Seq<TYPE>)
    requires forall|index: int| 0 <= index < words.len() ==> #[trigger] words[index] == 0,
    ensures word_value_TYPE(words) == 0,
{
    let digits = word_digits_TYPE(words);
    assert forall|index: int| 0 <= index < digits.len() implies #[trigger] digits[index] == 0 by {
        assert(words[index] == 0);
    }
    digit_prefix_zero(digits, WIDTH, digits.len() as int);
}

pub proof fn typed_one_scan_none_TYPE(words: Seq<TYPE>)
    requires forall|index: int| 0 <= index < words.len() ==> #[trigger] words[index] == TYPE::MAX,
    ensures has_odd_factor(word_value_TYPE(words) + 1, WIDTH * words.len()),
{
    let digits = word_digits_TYPE(words);
    lemma2_to64();
    assert forall|index: int| 0 <= index < digits.len() implies #[trigger] digits[index] + 1 == pow2(WIDTH) by {
        assert(words[index] == TYPE::MAX);
    }
    all_full_digits_factor(digits, WIDTH);
}

pub proof fn typed_zero_suffix_truncation_TYPE(words: Seq<TYPE>, length: int)
    requires 0 <= length <= words.len(),
        length == 0 || words[length - 1] > 0,
        forall|index: int| length <= index < words.len() ==> #[trigger] words[index] == 0,
    ensures word_value_TYPE(words.subrange(0, length)) == word_value_TYPE(words),
        normalized_TYPE(words.subrange(0, length)),
{
    let digits = word_digits_TYPE(words);
    assert forall|index: int| length <= index < digits.len() implies #[trigger] digits[index] == 0 by {
        assert(words[index] == 0);
    }
    trimmed_zero_suffix_preserves_value(digits, WIDTH, length);
    assert(word_digits_TYPE(words.subrange(0, length)) =~= digits.subrange(0, length));
}
'''


def main():
    proposal = E / "word-scan-proposal-v2374.rs"
    runner = E / "probe_word_scan_v2374.py"
    record = E / "word-scan-probe-preparation-v2374.json"
    assert all(not p.exists() for p in [proposal, runner, record])
    prefix = "//! Conditional typed word views; the actual dependency storage import remains open.\n#![feature(proc_macro_hygiene)]\n"
    for name, path in [("verified", "src/verified/mod.rs"),
        ("magnitude_model", "verification/magnitude_model.rs"),
        ("bit_length_model", "verification/bit_length_model.rs"),
        ("digit_scan_model", "verification/digit_scan_model.rs")]:
        prefix += '#[path = ' + json.dumps(str(SOURCE / "crates/hyperreal" / path)) + ']\nmod ' + name + ';\n'
    prefix += "use crate::digit_scan_model::*;\nuse crate::magnitude_model::has_bit_length;\n"
    prefix += "use vstd::prelude::*;\nuse vstd::arithmetic::power2::*;\nuse vstd::std_specs::bits::*;\n\nverus! {\n"
    body = COMMON
    for width in [32, 64]:
        text = TYPED.replace("TYPE", f"u{width}").replace("WIDTH", str(width))
        leading = "crate::digit_scan_model::u32_digit_bit_length" if width == 32 else "crate::bit_length_model::leading_zeros_determine_word_bit_length"
        zero = "crate::bit_length_model::trailing_zeros_factor_u32" if width == 32 else "crate::verified::gcd::trailing_zeros_factor_u64"
        body += text.replace("LEADING_PROOF", leading).replace("ZERO_PROOF", zero)
    proposal.write_text(prefix + body + "\n}\n")
    parent = E / "probe_digit_scan_connections_v2362.py"
    code = parent.read_text()
    for old, new in [
        ("digit-scan-connected-proposal-v2362.rs", proposal.name),
        ("digit-scan-connections-probe-v2362", "word-scan-probe-v2374"),
        ("typed-api-prototype-v2358-state.json", "typed-api-prototype-v2373-state.json"),
        ("5af0c6067447ffcf6f531b7b3cb206b83532225b99f6c8a1f66cbca7698bc991", "163bf723328ab72df0a56025f9a55960eda13eb4c6767669965b61e66333508a"),
        ("typed-api-prototype-v2352-trailing-metadata-verus", "typed-api-prototype-v2367-digit-scan-final-verus"),
        ("Probe digit-scan connections to actual retained metadata lemmas.", "Probe typed word views and scans against retained digit lemmas."),
        ("Do generic digit scans, normalization and native u32 leading-zero arithmetic connect to the actual retained bit-length and metadata flag lemmas?",
         "Do typed u32/u64 word views, scan witnesses and zero-suffix truncation connect to retained generic digit denotation and actual native factor helpers, with explicit count-range premises?"),
    ]:
        assert code.count(old) == 1, old
        code = code.replace(old, new)
    ast.parse(code)
    runner.write_text(code)
    record.write_text(json.dumps({"status": "Frozen before typed word-sequence support probing",
        "preparation_sha256": sha(Path(__file__)), "parent_runner_sha256": sha(parent),
        "proposal_sha256": sha(proposal), "runner_sha256": sha(runner), "new_proof_functions": 18,
        "retained_checkpoint": "typed-api-prototype-v2373-state.json", "source_changed": False,
        "scope": "Seven proofs per word width and four shared connections. The u64 denotation is tied to the actual verified limb prefix. First-match, normalization and maximum aggregate bit-count premises are explicit. This is not a Vec iterator proof, an actual BigUint import, a new retained theorem count or runtime change; preserve every failed support attempt."}, indent=2) + "\n")
    print(json.dumps({"frozen_word_scan_probe": True, "new_proof_functions": 18, "source_changed": False}))


if __name__ == "__main__":
    main()
