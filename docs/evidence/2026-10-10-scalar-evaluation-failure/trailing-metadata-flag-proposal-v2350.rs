// These arithmetic bridges consume an odd-factor certificate. Actual dependency
// storage, first-match scans and machine-width metadata imports remain open.
pub(crate) proof fn odd_factor_of_power_is_unit(odd: nat, shift: nat, exponent: nat)
    requires odd > 0, odd % 2 == 1,
        odd * pow2(shift) == pow2(exponent),
    ensures odd == 1, shift == exponent,
{
    lemma_pow2_pos(shift);
    lemma_pow2_pos(exponent);
    if shift > exponent {
        lemma_pow2_strictly_increases(exponent, shift);
        assert(false) by (nonlinear_arith)
            requires odd > 0, pow2(shift) > pow2(exponent),
                pow2(shift) > 0, odd * pow2(shift) == pow2(exponent);
    } else if shift < exponent {
        let difference = (exponent - shift) as nat;
        lemma_pow2_adds(shift, difference);
        assert(odd == pow2(difference)) by (nonlinear_arith)
            requires pow2(shift) > 0,
                odd * pow2(shift) == pow2(exponent),
                pow2(exponent) == pow2(shift) * pow2(difference);
        lemma_pow2_unfold(difference);
        let half = pow2((difference - 1) as nat);
        lemma_mod_multiples_vanish(half as int, 0, 2);
        assert(odd % 2 == 0);
    } else {
        assert(odd == 1) by (nonlinear_arith)
            requires pow2(shift) > 0, odd * pow2(shift) == pow2(shift);
    }
}

pub(crate) proof fn trailing_zeros_determine_power_of_two(
    magnitude: nat, bits: nat, zeros: nat, odd: nat,
)
    requires has_bit_length(magnitude, bits),
        odd > 0, odd % 2 == 1, magnitude == odd * pow2(zeros),
    ensures (zeros == (bits - 1) as nat) == (magnitude == pow2((bits - 1) as nat)),
{
    let exponent = (bits - 1) as nat;
    lemma_pow2_pos(exponent);
    lemma_pow2_unfold(bits);
    if zeros == exponent {
        assert(odd < 2) by (nonlinear_arith)
            requires magnitude == odd * pow2(exponent),
                magnitude < 2 * pow2(exponent), pow2(exponent) > 0;
        assert(odd == 1);
    } else if magnitude == pow2(exponent) {
        odd_factor_of_power_is_unit(odd, zeros, exponent);
    }
}

pub(crate) proof fn trailing_ones_determine_all_ones(
    magnitude: nat, bits: nat, ones: nat, odd: nat,
)
    requires has_bit_length(magnitude, bits),
        odd > 0, odd % 2 == 1, magnitude + 1 == odd * pow2(ones),
    ensures (ones == bits) == (magnitude + 1 == pow2(bits)),
{
    lemma_pow2_pos(bits);
    if ones == bits {
        assert(odd <= 1) by (nonlinear_arith)
            requires magnitude + 1 == odd * pow2(bits),
                magnitude + 1 <= pow2(bits), pow2(bits) > 0;
        assert(odd == 1);
    } else if magnitude + 1 == pow2(bits) {
        odd_factor_of_power_is_unit(odd, ones, bits);
    }
}
