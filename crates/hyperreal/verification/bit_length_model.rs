//! Connect the verified limb denotation to mathematical bit lengths.
//! BigUint storage normalization and the runtime import still need refinement.
use vstd::prelude::*;
use vstd::arithmetic::power2::*;
use vstd::arithmetic::div_mod::*;
use vstd::std_specs::bits::{u64_leading_zeros, axiom_u64_leading_zeros};
use crate::verified::limbs::{prefix, value, weight, prefix_bound};
use crate::magnitude_model::has_bit_length;

verus! {

/// The recursive standard-library specification determines the exact binade,
/// including a word whose top bit occupies the entire native width.
pub(crate) proof fn leading_zeros_determine_word_bit_length(word: u64)
    ensures
        0 <= u64_leading_zeros(word) <= 64,
        word == 0 <==> u64_leading_zeros(word) == 64,
        word > 0 ==> has_bit_length(word as nat, (64 - u64_leading_zeros(word)) as nat),
    decreases word,
{
    axiom_u64_leading_zeros(word);
    reveal(u64_leading_zeros);
    if word == 1 {
        reveal(u64_leading_zeros);
        lemma2_to64();
    } else if word > 1 {
        let half = word / 2;
        leading_zeros_determine_word_bit_length(half);
        let previous = (64 - u64_leading_zeros(half)) as nat;
        assert(previous > 0);
        lemma_fundamental_div_mod(word as int, 2);
        lemma_mod_bound(word as int, 2);
        lemma_pow2_unfold(previous);
        lemma_pow2_unfold(previous + 1);
        assert(pow2(previous) <= word && word < pow2(previous + 1)) by (nonlinear_arith)
            requires
                pow2(previous) == 2 * pow2((previous - 1) as nat),
                pow2(previous + 1) == 2 * pow2(previous),
                pow2((previous - 1) as nat) <= half, half < pow2(previous),
                word as int == 2 * (half as int) + (word as int) % 2,
                0 <= (word as int) % 2 < 2;
    }
}

pub(crate) open spec fn normalized_words(words: Seq<u64>) -> bool {
    words.len() == 0 || words[words.len() - 1] > 0
}

pub(crate) open spec fn limb_bit_length(words: Seq<u64>) -> nat {
    if words.len() == 0 { 0 }
    else { (64 * (words.len() - 1) + 64 - u64_leading_zeros(words[words.len() - 1])) as nat }
}

/// There is no machine-width truncation in this mathematical length. The
/// dependency import still must establish canonical top limbs and range checks.
pub(crate) proof fn normalized_limb_bit_length(words: Seq<u64>)
    requires normalized_words(words),
    ensures
        words.len() == 0 ==> value(words) == 0 && limb_bit_length(words) == 0,
        words.len() > 0 ==> has_bit_length(value(words), limb_bit_length(words)),
{
    if words.len() > 0 {
        let last = words.len() as int - 1;
        let top = words[last];
        leading_zeros_determine_word_bit_length(top);
        let bits = (64 - u64_leading_zeros(top)) as nat;
        let lower = prefix(words, last);
        let unit = weight(last as nat);
        prefix_bound(words, last);
        lemma_pow2_pos(64 * last as nat);
        lemma_pow2_adds(64 * last as nat, (bits - 1) as nat);
        lemma_pow2_adds(64 * last as nat, bits);
        assert(value(words) == lower + top * unit);
        assert(pow2((limb_bit_length(words) - 1) as nat) <= value(words)
            && value(words) < pow2(limb_bit_length(words))) by (nonlinear_arith)
            requires
                unit > 0, 0 <= lower < unit,
                pow2((bits - 1) as nat) <= top, top < pow2(bits),
                value(words) == lower + top * unit,
                pow2((limb_bit_length(words) - 1) as nat) == unit * pow2((bits - 1) as nat),
                pow2(limb_bit_length(words)) == unit * pow2(bits);
    }
}

/// A positive normalized integer's exact bit length grows by the complete
/// mathematical shift. This is the width test used before a borrowed scan.
pub(crate) proof fn binary_shift_bit_length(magnitude: nat, bits: nat, shift: nat)
    requires has_bit_length(magnitude, bits),
    ensures has_bit_length(magnitude * pow2(shift), bits + shift),
{
    lemma_pow2_pos(shift);
    lemma_pow2_adds((bits - 1) as nat, shift);
    lemma_pow2_adds(bits, shift);
    assert(pow2((bits + shift - 1) as nat) <= magnitude * pow2(shift)
        && magnitude * pow2(shift) < pow2(bits + shift)) by (nonlinear_arith)
        requires pow2(shift) > 0,
            pow2((bits + shift - 1) as nat) == pow2((bits - 1) as nat) * pow2(shift),
            pow2(bits + shift) == pow2(bits) * pow2(shift),
            pow2((bits - 1) as nat) <= magnitude < pow2(bits);
}

/// Distinct exact widths determine magnitude order without scanning limbs.
pub(crate) proof fn bit_lengths_determine_order(left: nat, right: nat, left_bits: nat, right_bits: nat)
    requires has_bit_length(left, left_bits), has_bit_length(right, right_bits), left_bits < right_bits,
    ensures left < right,
{
    if left_bits < right_bits - 1 {
        lemma_pow2_strictly_increases(left_bits, (right_bits - 1) as nat);
    }
}

}
