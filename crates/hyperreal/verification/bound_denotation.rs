// Real-denotation refinements of the included production bound contracts.
// Each call_ensures names the actual checked method; incoming certificates
// and the production callers remain separate obligations.
use crate::real_approximation_model::{binary_unit, binary_unit_positive, binary_unit_composes,
    approximates, separated_approximation_certifies_sign};
use vstd::arithmetic::power2::lemma2_to64;

verus! {
pub(crate) open spec fn absolute(value: real) -> real {
    if value >= 0real { value } else { -value }
}

pub(crate) open spec fn real_binade(value: real, exponent: int) -> bool {
    binary_unit(exponent) <= absolute(value) < 2real * binary_unit(exponent)
}

pub(crate) open spec fn sign_denotes(sign: Option<Sign>, value: real) -> bool {
    match sign {
        None => true,
        Some(Sign::NoSign) => value == 0real,
        Some(Sign::Plus) => value > 0real,
        Some(Sign::Minus) => value < 0real,
    }
}

pub(crate) open spec fn public_sign_denotes(sign: RealSign, value: real) -> bool {
    match sign {
        RealSign::Negative => value < 0real,
        RealSign::Zero => value == 0real,
        RealSign::Positive => value > 0real,
    }
}

// This predicate covers sign, nonzero and exact magnitude certificates.
// It intentionally does not claim an error bound for inexact planning metadata.
pub(crate) open spec fn bound_denotes(bound: BoundInfo, value: real) -> bool {
    match bound {
        BoundInfo::Unknown => true,
        BoundInfo::Zero => value == 0real,
        BoundInfo::NonZero { sign, msd, exact_msd } => value != 0real
            && sign_denotes(sign, value)
            && match msd { Some(e) => exact_msd ==> real_binade(value, e as int), None => true },
    }
}

proof fn constructor_preserves_certificates(
    sign: Sign, msd: Option<i32>, exact_msd: bool, value: real, result: BoundInfo,
)
    requires sign_denotes(Some(sign), value),
        match sign {
            Sign::NoSign => true,
            _ => match msd { Some(e) => exact_msd ==> real_binade(value, e as int), None => true },
        },
        call_ensures(BoundInfo::with_sign_msd, (sign, msd, exact_msd), result),
    ensures bound_denotes(result, value),
{}

proof fn separated_approximation_yields_a_bound(
    value: real, approximation: int, unit: real, sign: Sign, msd: Option<i32>, result: BoundInfo,
)
    requires approximates(value, approximation, unit),
        approximation > 1 || approximation < -1,
        if approximation > 0 { sign == Sign::Plus } else { sign == Sign::Minus },
        call_ensures(BoundInfo::with_sign_msd, (sign, msd, false), result),
    ensures bound_denotes(result, value),
{
    separated_approximation_certifies_sign(value, approximation, unit);
    constructor_preserves_certificates(sign, msd, false, value, result);
}

proof fn negation_preserves_bound_denotation(bound: BoundInfo, value: real, result: BoundInfo)
    requires bound_denotes(bound, value), call_ensures(BoundInfo::negate, (bound,), result),
    ensures bound_denotes(result, -value),
{
    assert(absolute(-value) == absolute(value));
}

proof fn inverse_preserves_bound_denotation(bound: BoundInfo, value: real, result: BoundInfo)
    requires value != 0real, bound_denotes(bound, value),
        call_ensures(BoundInfo::inverse, (bound,), result),
    ensures bound_denotes(result, 1real / value),
{
    assert(value > 0real ==> 1real / value > 0real) by (nonlinear_arith);
    assert(value < 0real ==> 1real / value < 0real) by (nonlinear_arith);
    assert(1real / value != 0real) by (nonlinear_arith) requires value != 0real;
}

proof fn positive_root_binade(value: real, root: real, exponent: i32)
    requires value > 0real, root >= 0real, root * root == value,
        real_binade(value, exponent as int),
    ensures real_binade(root, exponent as int / 2),
{
    let half = exponent as int / 2;
    let remainder = exponent as int - 2 * half;
    assert(remainder == 0 || remainder == 1);
    binary_unit_positive(half);
    binary_unit_composes(half, half);
    binary_unit_composes(2 * half, remainder);
    lemma2_to64();
    assert(binary_unit(0) == 1real && binary_unit(1) == 2real) by (nonlinear_arith)
        requires vstd::arithmetic::power2::pow2(0) == 1,
            vstd::arithmetic::power2::pow2(1) == 2;
    let unit = binary_unit(half);
    let factor = binary_unit(remainder);
    assert(factor == 1real || factor == 2real);
    assert(binary_unit(exponent as int) == unit * unit * factor);
    assert(unit * unit * factor <= value < 2real * unit * unit * factor) by (nonlinear_arith)
        requires binary_unit(exponent as int) == unit * unit * factor,
            binary_unit(exponent as int) <= value < 2real * binary_unit(exponent as int);
    assert(unit * unit <= root * root < 4real * unit * unit) by (nonlinear_arith)
        requires unit > 0real, factor == 1real || factor == 2real,
            unit * unit * factor <= value < 2real * unit * unit * factor,
            root * root == value;
    assert(unit <= root < 2real * unit) by (nonlinear_arith)
        requires unit > 0real, root >= 0real,
            unit * unit <= root * root < 4real * unit * unit;
}

proof fn square_root_preserves_bound_denotation(
    bound: BoundInfo, value: real, root: real, result: BoundInfo,
)
    requires root >= 0real, root * root == value, bound_denotes(bound, value),
        call_ensures(BoundInfo::sqrt, (bound,), result),
    ensures bound_denotes(result, root),
{
    assert(value == 0real ==> root == 0real) by (nonlinear_arith)
        requires root * root == value;
    assert(value > 0real ==> root > 0real) by (nonlinear_arith)
        requires root >= 0real, root * root == value;
    match bound {
        BoundInfo::NonZero { sign: Some(Sign::Plus), msd: Some(exponent), exact_msd: true } => {
            positive_root_binade(value, root, exponent);
            assert(i32::MIN <= exponent as int / 2 <= i32::MAX);
        },
        _ => {},
    }
}

proof fn queried_magnitude_is_a_certificate(bound: &BoundInfo, value: real, result: Option<Option<i32>>)
    requires bound_denotes(*bound, value), call_ensures(BoundInfo::known_msd, (bound,), result),
    ensures result == Some(None) ==> value == 0real,
        match result { Some(Some(e)) => real_binade(value, e as int), _ => true },
{}

proof fn queried_sign_is_a_certificate(bound: &BoundInfo, value: real, result: Option<Sign>)
    requires bound_denotes(*bound, value), call_ensures(BoundInfo::known_sign, (bound,), result),
    ensures sign_denotes(result, value),
{}

proof fn precision_facts_are_certificates(
    bound: &BoundInfo, value: real, result: (Option<Sign>, Option<Option<i32>>),
)
    requires bound_denotes(*bound, value),
        call_ensures(BoundInfo::certified_sign_and_msd, (bound,), result),
    ensures sign_denotes(result.0, value),
        result.1 == Some(None) ==> value == 0real,
        match result.1 { Some(Some(e)) => real_binade(value, e as int), _ => true },
{}

proof fn exported_magnitude_is_a_certificate(bound: &BoundInfo, value: real, result: Option<MagnitudeBits>)
    requires bound_denotes(*bound, value), call_ensures(BoundInfo::magnitude_bits, (bound,), result),
    ensures match result {
        Some(bits) => value != 0real && (bits.exact_msd ==> real_binade(value, bits.msd as int)),
        None => true,
    },
{}

proof fn public_sign_preserves_denotation(sign: Sign, value: real, result: RealSign)
    requires sign_denotes(Some(sign), value), call_ensures(public_sign, (sign,), result),
    ensures public_sign_denotes(result, value),
{}

proof fn private_sign_preserves_denotation(sign: RealSign, value: real, result: Sign)
    requires public_sign_denotes(sign, value), call_ensures(private_sign, (sign,), result),
    ensures sign_denotes(Some(result), value),
{}

proof fn binary_scaling_preserves_real_binade(value: real, exponent: int, offset: int)
    requires real_binade(value, exponent),
    ensures real_binade(value * binary_unit(offset), exponent + offset),
{
    binary_unit_positive(offset);
    binary_unit_composes(exponent, offset);
    let scale = binary_unit(offset);
    let unit = binary_unit(exponent);
    assert(absolute(value * scale) == absolute(value) * scale) by (nonlinear_arith)
        requires scale > 0real;
    assert(real_binade(value * scale, exponent + offset)) by (nonlinear_arith)
        requires scale > 0real, unit <= absolute(value) < 2real * unit,
            absolute(value * scale) == absolute(value) * scale,
            binary_unit(exponent + offset) == unit * scale;
}

proof fn binary_offset_preserves_bound_denotation<F: FnOnce(i32) -> Option<i32>>(
    bound: BoundInfo, value: real, offset: int, f: F, result: BoundInfo,
)
    requires bound_denotes(bound, value),
        forall|exponent: i32, mapped: Option<i32>| #[trigger] call_ensures(f, (exponent,), mapped)
            ==> mapped == checked_metadata_exponent(exponent as int + offset),
        call_ensures(BoundInfo::map_msd, (bound, f), result),
    ensures bound_denotes(result, value * binary_unit(offset)),
{
    binary_unit_positive(offset);
    assert(value > 0real ==> value * binary_unit(offset) > 0real) by (nonlinear_arith)
        requires binary_unit(offset) > 0real;
    assert(value < 0real ==> value * binary_unit(offset) < 0real) by (nonlinear_arith)
        requires binary_unit(offset) > 0real;
    assert(value != 0real ==> value * binary_unit(offset) != 0real) by (nonlinear_arith)
        requires binary_unit(offset) > 0real;
    match bound {
        BoundInfo::NonZero { msd: Some(exponent), exact_msd: true, .. } => {
            binary_scaling_preserves_real_binade(value, exponent as int, offset);
        },
        _ => {},
    }
}
}
