//! Decode raw IEEE binary32/binary64 bits for exact rational ingress.
//!
//! The finite case retains a sign, integer significand and binary exponent;
//! constructing/reducing the rational belongs to the conversion layer. Infinities
//! and NaNs become separate tags that the caller turns into typed import errors.
//! The decoders inspect fixed-width integers without floating arithmetic,
//! allocation, caches or a caller-supplied precision/resource budget.
//!
//! Cargo and the verification entry point use the same executable bodies. Under
//! verus_keep_ghost, the decodes relation describes arithmetic field extraction
//! and verus_spec attaches it to each decoder; proof blocks relate bit masks to
//! division/remainder. Ordinary builds erase those proof-only definitions and
//! annotations. This source connection is not a claim that a verifier has run or
//! that the whole rational import/canonicalization path has been formally proved.

#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

/// Finite signed binary factors or a nonfinite category produced by IEEE decoding.
///
/// A finite value denotes (-1)^negative * significand * 2^exponent. Its zero
/// sign is retained here, even though Rational later canonicalizes zero. The
/// decoders establish format-specific field bounds; manually constructing this
/// enum does not validate those bounds or require a unique factorization.
///
/// Clone and Copy duplicate the scalar fields; Debug displays the variant and
/// finite fields. Derived equality compares variants and stored fields, not
/// mathematical real values or IEEE comparison behavior: opposite signed zeros
/// are distinct, equal values with different factorizations can be distinct,
/// and any two NotANumber tags compare equal. Nonfinite signs and NaN payloads
/// have already been discarded before that comparison.
#[cfg_attr(verus_keep_ghost, verus_verify)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FloatParts {
    /// Exact finite encoding, including normal, subnormal and signed-zero inputs.
    /// The integer significand includes the implicit leading bit only for normals;
    /// no cancellation of trailing powers of two is performed here.
    Finite {
        /// Original finite input sign bit; true means negative, including negative zero.
        negative: bool,
        /// Nonnegative integer multiplier of the binary power, zero exactly for decoded zero.
        /// Binary32 decoding yields fewer than 2^24; binary64 yields fewer than 2^53.
        /// Those are decoder guarantees, not constraints on arbitrary enum construction.
        significand: u64,
        /// Binary scale after accounting for significand width, not the raw exponent field.
        /// Decoded ranges are -149..=104 for binary32 and -1074..=971 for binary64;
        /// subnormal and zero encodings use the lower endpoint of their format.
        exponent: i32,
    },
    /// Either infinity sign, deliberately merged for the caller's Infinity error.
    /// This tag does not preserve the sign needed to reconstruct the original bits.
    Infinity,
    /// Any NaN encoding, deliberately merged for the caller's NotANumber error.
    /// Sign, payload and signaling/quiet distinction are not retained.
    NotANumber,
}

#[cfg(verus_keep_ghost)]
verus! {
// Arithmetic definitions of the IEEE fields, independent of the executable
// bit masks. The implicit leading bit is present only in normal encodings.
/// Relate a decoded category to arithmetic IEEE fields and format constants.
///
/// The executable postconditions supply sign, raw exponent and fraction extracted
/// by division/remainder. binary32 uses (exponent_max, implicit_bit,
/// bias_and_fraction_bits) = (255, 2^23, 150); binary64 uses (2047, 2^52, 1075).
/// Finite results match the sign, add the implicit bit exactly when the exponent
/// is nonzero, and use max(raw_exponent, 1) minus the bias/width constant for
/// valid nonnegative fields. The nonfinite branches ignore sign and compare the
/// raw fraction with zero to distinguish infinity from NaN.
///
/// This is a ghost relation, not a runtime validator or a universal format model.
/// It does not itself require nonnegative/in-range raw fields or well-formed
/// format constants; those assumptions come from the decoder's field extraction
/// and concrete parameters. In particular, arbitrary integer arguments must not
/// be interpreted as valid IEEE input merely because this relation holds.
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

/// Classify every u32 bit pattern and extract its exact finite binary factors.
///
/// Normal encodings add the implicit 2^23 significand bit and subtract 150
/// from the raw exponent. Subnormals and both zeros use exponent -149 without
/// that bit. Finite outputs have significand < 2^24 and exponent in
/// -149..=104; their original sign is retained. No reduction or rounding is
/// performed. All-ones exponents select Infinity for zero fraction and NotANumber
/// otherwise, discarding their sign and any NaN payload.
///
/// There is no invalid input pattern, allocation or typed error at this layer.
/// The rational importer rejects the two nonfinite tags separately. Conditional
/// verification annotations specify field agreement and finite bounds for this
/// same body; the fixed-width proof blocks are erased from ordinary builds.
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

/// Classify every u64 bit pattern and extract its exact finite binary factors.
///
/// Normal encodings add the implicit 2^52 significand bit and subtract 1075
/// from the raw exponent. Subnormals and both zeros use exponent -1074 without
/// that bit. Finite outputs have significand < 2^53 and exponent in
/// -1074..=971; their original sign is retained. No reduction or rounding is
/// performed. All-ones exponents select Infinity for zero fraction and NotANumber
/// otherwise, discarding their sign and any NaN payload.
///
/// There is no invalid input pattern, allocation or typed error at this layer.
/// The rational importer rejects the two nonfinite tags separately. Conditional
/// verification annotations specify field agreement and finite bounds for this
/// same body; the fixed-width proof blocks are erased from ordinary builds.
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
