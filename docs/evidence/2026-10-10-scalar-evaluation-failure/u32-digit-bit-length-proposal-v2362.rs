pub proof fn u32_digit_bit_length(word: u32)
    ensures
        0 <= u32_leading_zeros(word) <= 32,
        word == 0 <==> u32_leading_zeros(word) == 32,
        word > 0 ==> has_bit_length(word as nat, (32 - u32_leading_zeros(word)) as nat),
{
    axiom_u32_leading_zeros(word);
    if word > 0 {
        let count = u32_leading_zeros(word);
        let bits = (32 - count) as nat;
        let low = (31 - count) as u32;
        lemma_pow2_pos((bits - 1) as nat);
        vstd::bits::lemma_u32_shr_is_div(word, low);
        assert((word >> low) > 0) by (bit_vector)
            requires (word >> low) & 1u32 != 0u32;
        let lower = pow2((bits - 1) as nat);
        lemma_fundamental_div_mod(word as int, lower as int);
        lemma_mod_bound(word as int, lower as int);
        assert(lower <= word) by (nonlinear_arith)
            requires lower > 0, word as nat / lower > 0,
                word as int == (word as int / lower as int) * lower as int
                    + word as int % lower as int,
                word as int % lower as int >= 0;
        if bits == 32 {
            lemma2_to64();
        } else {
            let high = bits as u32;
            vstd::bits::lemma_u32_shr_is_div(word, high);
            assert(word >> high == 0);
            let upper = pow2(bits);
            lemma_pow2_pos(bits);
            lemma_fundamental_div_mod(word as int, upper as int);
            lemma_mod_bound(word as int, upper as int);
        }
    }
}
