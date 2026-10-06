// Wire-format specifications and lossless packing of actual cache metadata.
// These results do not establish atomic publication or cache validity.
verus! {
spec fn bound_sign_bits(sign: Option<Sign>) -> u64 {
    match sign { None => 0, Some(Sign::Plus) => 1, Some(Sign::Minus) => 2, Some(Sign::NoSign) => 3 }
}

spec fn bound_word(value: u64) -> BoundCache {
    match value & 3 {
        0 => BoundCache::Invalid,
        1 => BoundCache::Valid(BoundInfo::Unknown),
        2 => BoundCache::Valid(BoundInfo::Zero),
        _ => BoundCache::Valid(BoundInfo::NonZero {
            sign: match (value >> 2) & 3 {
                0 => None, 1 => Some(Sign::Plus), 2 => Some(Sign::Minus), _ => Some(Sign::NoSign),
            },
            msd: if value & 16 != 0 { Some(((value >> 32) as u32) as i32) } else { None },
            exact_msd: value & 32 != 0,
        }),
    }
}

spec fn encoded_bound_bits(value: BoundCache) -> u64 {
    match value {
        BoundCache::Invalid => 0,
        BoundCache::Valid(BoundInfo::Unknown) => 1,
        BoundCache::Valid(BoundInfo::Zero) => 2,
        BoundCache::Valid(BoundInfo::NonZero { sign, msd, exact_msd }) => {
            let base = 3u64 | (bound_sign_bits(sign) << 2u32);
            let with_magnitude = match msd {
                Some(e) => base | (16u64 | ((e as u32 as u64) << 32u32)),
                None => base,
            };
            if exact_msd { with_magnitude | 32u64 } else { with_magnitude }
        },
    }
}

proof fn packed_bound_fields(sign: u64, magnitude: i32, present: bool, exact: bool)
    requires sign <= 3,
    ensures ({
        let base = 3u64 | (sign << 2u32);
        let with_magnitude = if present { base | (16u64 | ((magnitude as u32 as u64) << 32u32)) } else { base };
        let bits = if exact { with_magnitude | 32u64 } else { with_magnitude };
        &&& bits & 3u64 == 3u64
        &&& (bits >> 2u32) & 3u64 == sign
        &&& (bits & 16u64 != 0u64) == present
        &&& (bits & 32u64 != 0u64) == exact
        &&& present ==> ((bits >> 32u32) as u32) as i32 == magnitude
        &&& bits & 0xffff_ffc0u64 == 0u64
    }),
{
    assert({
        let base = 3u64 | (sign << 2u32);
        let with_magnitude = if present { base | (16u64 | ((magnitude as u32 as u64) << 32u32)) } else { base };
        let bits = if exact { with_magnitude | 32u64 } else { with_magnitude };
        &&& bits & 3u64 == 3u64
        &&& (bits >> 2u32) & 3u64 == sign
        &&& (bits & 16u64 != 0u64) == present
        &&& (bits & 32u64 != 0u64) == exact
        &&& present ==> ((bits >> 32u32) as u32) as i32 == magnitude
        &&& bits & 0xffff_ffc0u64 == 0u64
    }) by (bit_vector) requires sign <= 3u64;
}

proof fn encoded_bound_round_trip(value: BoundCache, result: u64)
    requires call_ensures(AtomicFacts::encode_bound, (value,), result),
    ensures bound_word(result) == value, result & 0xffff_ffc0u64 == 0,
{
    match value {
        BoundCache::Valid(BoundInfo::NonZero { sign, msd, exact_msd }) => {
            match msd {
                Some(e) => packed_bound_fields(bound_sign_bits(sign), e, true, exact_msd),
                None => packed_bound_fields(bound_sign_bits(sign), 0, false, exact_msd),
            }
            assert(bound_word(result) == value);
            assert(result & 0xffff_ffc0u64 == 0);
        },
        _ => {
            assert(result <= 2);
            assert(result & 3u64 == result && result & 0xffff_ffc0u64 == 0u64) by (bit_vector)
                requires result <= 2u64;
        },
    }
}

spec fn exact_sign_denotation(value: u64) -> ExactSignCache {
    match (value >> 6) & 7 {
        0 => ExactSignCache::Invalid,
        1 => ExactSignCache::Unknown,
        2 => ExactSignCache::Valid(Sign::Minus),
        3 => ExactSignCache::Valid(Sign::NoSign),
        _ => ExactSignCache::Valid(Sign::Plus),
    }
}

// The production compare-exchange loops use these exact word expressions.
// Proving the expressions does not verify the atomic operations or loop invariant.
proof fn cache_field_masks()
    ensures AtomicFacts::NON_BOUND_MASK == 0xffff_07c0u64,
        AtomicFacts::EXACT_SIGN_MASK == 0x1c0u64,
        AtomicFacts::NON_BOUND_MASK & 0xffff_ffff_0000_003fu64 == 0,
{
    assert(AtomicFacts::NON_BOUND_MASK == 0xffff_07c0u64) by (bit_vector);
    assert(AtomicFacts::EXACT_SIGN_MASK == 0x1c0u64) by (bit_vector);
    assert(0xffff_07c0u64 & 0xffff_ffff_0000_003fu64 == 0u64) by (bit_vector);
}

proof fn bound_denotation_ignores_other_fields(left: u64, right: u64)
    requires left & 0xffff_ffff_0000_003fu64 == right & 0xffff_ffff_0000_003fu64,
    ensures bound_word(left) == bound_word(right),
{
    assert({
        &&& left & 3u64 == right & 3u64
        &&& (left >> 2u32) & 3u64 == (right >> 2u32) & 3u64
        &&& left & 16u64 == right & 16u64
        &&& left & 32u64 == right & 32u64
        &&& left >> 32u32 == right >> 32u32
    }) by (bit_vector)
        requires left & 0xffff_ffff_0000_003fu64 == right & 0xffff_ffff_0000_003fu64;
}

proof fn bound_update_preserves_other_fields(current: u64, value: BoundCache, encoded: u64)
    requires call_ensures(AtomicFacts::encode_bound, (value,), encoded),
    ensures bound_word((current & AtomicFacts::NON_BOUND_MASK) | encoded) == value,
        ((current & AtomicFacts::NON_BOUND_MASK) | encoded) & AtomicFacts::NON_BOUND_MASK
            == current & AtomicFacts::NON_BOUND_MASK,
        (((current & AtomicFacts::NON_BOUND_MASK) | encoded) >> 6) & 7 == (current >> 6) & 7,
        exact_sign_denotation((current & AtomicFacts::NON_BOUND_MASK) | encoded)
            == exact_sign_denotation(current),
{
    cache_field_masks();
    encoded_bound_round_trip(value, encoded);
    let updated = (current & AtomicFacts::NON_BOUND_MASK) | encoded;
    assert(encoded & 0xffff_07c0u64 == 0u64) by (bit_vector)
        requires encoded & 0xffff_ffc0u64 == 0u64;
    assert(updated & 0xffff_ffff_0000_003fu64 == encoded & 0xffff_ffff_0000_003fu64) by (bit_vector)
        requires updated == (current & 0xffff_07c0u64) | encoded;
    bound_denotation_ignores_other_fields(updated, encoded);
    assert(updated & 0xffff_07c0u64 == current & 0xffff_07c0u64) by (bit_vector)
        requires updated == (current & 0xffff_07c0u64) | encoded,
            encoded & 0xffff_07c0u64 == 0u64;
    assert((updated >> 6u32) & 7u64 == (current >> 6u32) & 7u64) by (bit_vector)
        requires updated & 0xffff_07c0u64 == current & 0xffff_07c0u64;
}

proof fn exact_sign_update_preserves_bound(current: u64, value: ExactSignCache, encoded: u64)
    requires call_ensures(AtomicFacts::encode_exact_sign, (value,), encoded),
    ensures bound_word((current & !AtomicFacts::EXACT_SIGN_MASK) | encoded) == bound_word(current),
        exact_sign_denotation((current & !AtomicFacts::EXACT_SIGN_MASK) | encoded) == value,
        (((current & !AtomicFacts::EXACT_SIGN_MASK) | encoded) >> 6) & 7 <= 4,
        ((current & !AtomicFacts::EXACT_SIGN_MASK) | encoded) & !AtomicFacts::EXACT_SIGN_MASK
            == current & !AtomicFacts::EXACT_SIGN_MASK,
{
    cache_field_masks();
    let updated = (current & !AtomicFacts::EXACT_SIGN_MASK) | encoded;
    assert(updated & !0x1c0u64 == current & !0x1c0u64) by (bit_vector)
        requires updated == (current & !0x1c0u64) | encoded, encoded & !0x1c0u64 == 0u64;
    assert(updated & 0xffff_ffff_0000_003fu64 == current & 0xffff_ffff_0000_003fu64) by (bit_vector)
        requires updated & !0x1c0u64 == current & !0x1c0u64;
    bound_denotation_ignores_other_fields(updated, current);
    assert((updated >> 6u32) & 7u64 == (encoded >> 6u32) & 7u64) by (bit_vector)
        requires updated == (current & !0x1c0u64) | encoded;
}

spec fn initial_fact_word(bound: u64, sign: u64, inverse_trig: bool, demand: i16) -> u64 {
    bound | sign | ((demand as u16 as u64) << AtomicFacts::LINEAR_DEMAND_SHIFT)
        | AtomicFacts::INVERSE_TRIG_OR_PI_KNOWN
        | if inverse_trig { AtomicFacts::CONTAINS_INVERSE_TRIG_OR_PI } else { 0 }
}

proof fn initial_cache_word_preserves_all_fields(
    bound: BoundCache, sign: ExactSignCache, bound_bits: u64, sign_bits: u64,
    inverse_trig: bool, demand: i16,
)
    requires call_ensures(AtomicFacts::encode_bound, (bound,), bound_bits),
        call_ensures(AtomicFacts::encode_exact_sign, (sign,), sign_bits),
    ensures ({
        let bits = initial_fact_word(bound_bits, sign_bits, inverse_trig, demand);
        &&& bound_word(bits) == bound
        &&& exact_sign_denotation(bits) == sign
        &&& ((bits >> 6) & 7) <= 4
        &&& ((bits & AtomicFacts::LINEAR_DEMAND_MASK) >> AtomicFacts::LINEAR_DEMAND_SHIFT) as u16 as i16 == demand
        &&& bits & AtomicFacts::INVERSE_TRIG_OR_PI_KNOWN != 0
        &&& (bits & AtomicFacts::CONTAINS_INVERSE_TRIG_OR_PI != 0) == inverse_trig
    }),
{
    cache_field_masks();
    encoded_bound_round_trip(bound, bound_bits);
    assert(AtomicFacts::LINEAR_DEMAND_MASK == 0xffff_0000u64) by (bit_vector);
    assert(AtomicFacts::INVERSE_TRIG_OR_PI_KNOWN == 0x200u64) by (bit_vector);
    assert(AtomicFacts::CONTAINS_INVERSE_TRIG_OR_PI == 0x400u64) by (bit_vector);
    let bits = initial_fact_word(bound_bits, sign_bits, inverse_trig, demand);
    assert({
        &&& bits & 0xffff_ffff_0000_003fu64 == bound_bits & 0xffff_ffff_0000_003fu64
        &&& (bits >> 6u32) & 7u64 == (sign_bits >> 6u32) & 7u64
        &&& ((bits & 0xffff_0000u64) >> 16u32) as u16 as i16 == demand
        &&& bits & 0x200u64 != 0u64
        &&& (bits & 0x400u64 != 0u64) == inverse_trig
    }) by (bit_vector)
        requires bound_bits & 0xffff_ffc0u64 == 0u64, sign_bits & !0x1c0u64 == 0u64,
            bits == bound_bits | sign_bits | ((demand as u16 as u64) << 16u32)
                | 0x200u64 | if inverse_trig { 0x400u64 } else { 0u64 };
    bound_denotation_ignores_other_fields(bits, bound_bits);
}
}
