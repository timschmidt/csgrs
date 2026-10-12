#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

/// Exact IEEE-754 decomposition. Finite values denote
/// `(-1)^negative * significand * 2^exponent`, including signed zero.
#[cfg_attr(verus_keep_ghost, verus_verify)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FloatParts {
    Finite {
        negative: bool,
        significand: u64,
        exponent: i32,
    },
    Infinity,
    NotANumber,
}

#[cfg(verus_keep_ghost)]
verus! {
// Arithmetic definitions of the IEEE fields, independent of the executable
// bit masks. The implicit leading bit is present only in normal encodings.
pub(crate) open spec fn decodes(
    parts: FloatParts, negative: bool, exponent: int, fraction: int,
    exponent_max: int, implicit_bit: int, bias_and_fraction_bits: int,
) -> bool {
    match parts {
        FloatParts::Infinity => exponent == exponent_max && fraction == 0,
        FloatParts::NotANumber => exponent == exponent_max && fraction != 0,
        FloatParts::Finite { negative: n, significand: s, exponent: e } =>
            exponent < exponent_max && n == negative
            && s == fraction + (if exponent == 0 { 0 } else { implicit_bit })
            && e == (if exponent == 0 { 1 } else { exponent }) - bias_and_fraction_bits,
    }
}
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures
        decodes(result, bits >= 0x8000_0000,
            (bits as int / 0x0080_0000) % 256, bits as int % 0x0080_0000,
            255, 0x0080_0000, 150),
        match result {
            FloatParts::Finite { significand, exponent, .. } =>
                significand < 0x0100_0000 && -149 <= exponent <= 104,
            _ => true,
        },
))]
#[inline]
pub(crate) fn decode_f32(bits: u32) -> FloatParts {
    let negative = (bits & 0x8000_0000) != 0;
    let exponent = (bits >> 23) & 0xff;
    let fraction = bits & 0x007f_ffff;
    proof! {
        assert(((bits & 0x8000_0000) != 0) == (bits >= 0x8000_0000)) by (bit_vector);
        assert(((bits >> 23) & 0xff) == (bits / 0x0080_0000) % 256) by (bit_vector);
        assert((bits & 0x007f_ffff) == bits % 0x0080_0000) by (bit_vector);
        assert(((bits >> 23) & 0xff) <= 255 && (bits & 0x007f_ffff) < 0x0080_0000) by (bit_vector);
    }
    if exponent == 255 {
        if fraction == 0 {
            FloatParts::Infinity
        } else {
            FloatParts::NotANumber
        }
    } else if exponent == 0 {
        FloatParts::Finite {
            negative,
            significand: fraction as u64,
            exponent: -149,
        }
    } else {
        FloatParts::Finite {
            negative,
            significand: (0x0080_0000 + fraction) as u64,
            exponent: exponent as i32 - 150,
        }
    }
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures
        decodes(result, bits >= 0x8000_0000_0000_0000,
            (bits as int / 0x0010_0000_0000_0000) % 2048,
            bits as int % 0x0010_0000_0000_0000,
            2047, 0x0010_0000_0000_0000, 1075),
        match result {
            FloatParts::Finite { significand, exponent, .. } =>
                significand < 0x0020_0000_0000_0000 && -1074 <= exponent <= 971,
            _ => true,
        },
))]
#[inline]
pub(crate) fn decode_f64(bits: u64) -> FloatParts {
    let negative = (bits & 0x8000_0000_0000_0000) != 0;
    let exponent = (bits >> 52) & 0x7ff;
    let fraction = bits & 0x000f_ffff_ffff_ffff;
    proof! {
        assert(((bits & 0x8000_0000_0000_0000) != 0) == (bits >= 0x8000_0000_0000_0000)) by (bit_vector);
        assert(((bits >> 52) & 0x7ff) == (bits / 0x0010_0000_0000_0000) % 2048) by (bit_vector);
        assert((bits & 0x000f_ffff_ffff_ffff) == bits % 0x0010_0000_0000_0000) by (bit_vector);
        assert(((bits >> 52) & 0x7ff) <= 2047 && (bits & 0x000f_ffff_ffff_ffff) < 0x0010_0000_0000_0000) by (bit_vector);
    }
    if exponent == 2047 {
        if fraction == 0 {
            FloatParts::Infinity
        } else {
            FloatParts::NotANumber
        }
    } else if exponent == 0 {
        FloatParts::Finite {
            negative,
            significand: fraction,
            exponent: -1074,
        }
    } else {
        FloatParts::Finite {
            negative,
            significand: 0x0010_0000_0000_0000 + fraction,
            exponent: exponent as i32 - 1075,
        }
    }
}
