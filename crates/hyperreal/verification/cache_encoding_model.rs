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
}
