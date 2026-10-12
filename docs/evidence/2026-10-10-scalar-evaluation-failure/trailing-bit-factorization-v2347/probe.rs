use vstd::prelude::*;
use vstd::arithmetic::{div_mod::*, power2::*};
use vstd::std_specs::bits::*;

verus! {

proof fn zero_count_factor_u32(value: u32)
    requires value > 0,
    ensures
        u32_trailing_zeros(value) < 32,
        0 < (value >> u32_trailing_zeros(value)) <= value,
        (value >> u32_trailing_zeros(value)) % 2 == 1,
        value == (value >> u32_trailing_zeros(value)) * pow2(u32_trailing_zeros(value) as nat),
{
    axiom_u32_trailing_zeros(value);
    let shift = u32_trailing_zeros(value) as u32;
    let odd = value >> shift;
    assert((value >> shift) % 2 == 1) by (bit_vector)
        requires (value >> shift) & 1u32 == 1u32;
    assert(0 < odd <= value) by (bit_vector)
        requires odd == value >> shift, odd % 2 == 1;
    vstd::bits::lemma_u32_mul_pow2_le_max_iff_max_shr(odd, shift, value);
    vstd::bits::lemma_u32_shl_is_mul(odd, shift);
    assert((value >> shift) << shift == value) by (bit_vector)
        requires shift < 32, value << ((32 - shift) as u32) == 0;
}

proof fn one_count_factor_u32(word: u32)
    ensures
        u32_trailing_ones(word) <= 32,
        ({let unit = pow2(u32_trailing_ones(word) as nat);
          let magnitude = word as nat + 1;
          &&& magnitude / unit > 0
          &&& (magnitude / unit) % 2 == 1
          &&& magnitude == (magnitude / unit) * unit}),
{
    axiom_u32_trailing_ones(word);
    let complement = !word;
    axiom_u32_trailing_zeros(complement);
    let shift = u32_trailing_ones(word) as nat;
    lemma2_to32();
    lemma_pow2_pos(shift);
    let unit = pow2(shift);
    let magnitude = word as nat + 1;
    assert((word as u128) + (complement as u128) == u32::MAX as u128) by (bit_vector)
        requires complement == !word;
    if complement == 0 {
        assert(shift == 32);
        assert(magnitude == unit);
        lemma_div_self(unit as int);
    } else {
        zero_count_factor_u32(complement);
        let odd = (complement >> u32_trailing_zeros(complement)) as nat;
        let other = pow2((32 - shift) as nat);
        lemma_pow2_adds(shift, (32 - shift) as nat);
        assert(magnitude == (other as int - odd as int) * unit) by (nonlinear_arith)
            requires word as nat + complement as nat == u32::MAX,
                magnitude == word as nat + 1,
                complement == odd * unit,
                pow2(32) == u32::MAX as nat + 1,
                pow2(32) == unit * other;
        assert(other > odd) by (nonlinear_arith)
            requires magnitude > 0, unit > 0,
                magnitude == (other as int - odd as int) * unit;
        let quotient = (other - odd) as nat;
        lemma_fundamental_div_mod_converse_div(magnitude as int, unit as int, quotient as int, 0);
        lemma_pow2_unfold((32 - shift) as nat);
        let half = pow2((31 - shift) as nat);
        lemma_fundamental_div_mod(odd as int, 2);
        assert(quotient as int == 2 * (half as int - (odd as int) / 2 - 1) + 1);
        lemma_mod_multiples_vanish(half as int - (odd as int) / 2 - 1, 1, 2);
    }
}

proof fn zero_count_factor_u64(value: u64)
    requires value > 0,
    ensures
        u64_trailing_zeros(value) < 64,
        0 < (value >> u64_trailing_zeros(value)) <= value,
        (value >> u64_trailing_zeros(value)) % 2 == 1,
        value == (value >> u64_trailing_zeros(value)) * pow2(u64_trailing_zeros(value) as nat),
{
    axiom_u64_trailing_zeros(value);
    let shift = u64_trailing_zeros(value) as u64;
    let odd = value >> shift;
    assert((value >> shift) % 2 == 1) by (bit_vector)
        requires (value >> shift) & 1u64 == 1u64;
    assert(0 < odd <= value) by (bit_vector)
        requires odd == value >> shift, odd % 2 == 1;
    vstd::bits::lemma_u64_mul_pow2_le_max_iff_max_shr(odd, shift, value);
    vstd::bits::lemma_u64_shl_is_mul(odd, shift);
    assert((value >> shift) << shift == value) by (bit_vector)
        requires shift < 64, value << ((64 - shift) as u64) == 0;
}

proof fn one_count_factor_u64(word: u64)
    ensures
        u64_trailing_ones(word) <= 64,
        ({let unit = pow2(u64_trailing_ones(word) as nat);
          let magnitude = word as nat + 1;
          &&& magnitude / unit > 0
          &&& (magnitude / unit) % 2 == 1
          &&& magnitude == (magnitude / unit) * unit}),
{
    axiom_u64_trailing_ones(word);
    let complement = !word;
    axiom_u64_trailing_zeros(complement);
    let shift = u64_trailing_ones(word) as nat;
    lemma2_to64();
    lemma_pow2_pos(shift);
    let unit = pow2(shift);
    let magnitude = word as nat + 1;
    assert((word as u128) + (complement as u128) == u64::MAX as u128) by (bit_vector)
        requires complement == !word;
    if complement == 0 {
        assert(shift == 64);
        assert(magnitude == unit);
        lemma_div_self(unit as int);
    } else {
        zero_count_factor_u64(complement);
        let odd = (complement >> u64_trailing_zeros(complement)) as nat;
        let other = pow2((64 - shift) as nat);
        lemma_pow2_adds(shift, (64 - shift) as nat);
        assert(magnitude == (other as int - odd as int) * unit) by (nonlinear_arith)
            requires word as nat + complement as nat == u64::MAX,
                magnitude == word as nat + 1,
                complement == odd * unit,
                pow2(64) == u64::MAX as nat + 1,
                pow2(64) == unit * other;
        assert(other > odd) by (nonlinear_arith)
            requires magnitude > 0, unit > 0,
                magnitude == (other as int - odd as int) * unit;
        let quotient = (other - odd) as nat;
        lemma_fundamental_div_mod_converse_div(magnitude as int, unit as int, quotient as int, 0);
        lemma_pow2_unfold((64 - shift) as nat);
        let half = pow2((63 - shift) as nat);
        lemma_fundamental_div_mod(odd as int, 2);
        assert(quotient as int == 2 * (half as int - (odd as int) / 2 - 1) + 1);
        lemma_mod_multiples_vanish(half as int - (odd as int) / 2 - 1, 1, 2);
    }
}
}
