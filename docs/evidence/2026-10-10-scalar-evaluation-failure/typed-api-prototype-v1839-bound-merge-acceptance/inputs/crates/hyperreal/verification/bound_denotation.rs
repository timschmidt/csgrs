// Real-denotation refinements of the included production bound contracts.
// Each call_ensures names the actual checked method; incoming certificates
// and the production callers remain separate obligations.
use crate::real_approximation_model::{
    approximates, binary_unit, binary_unit_composes, binary_unit_positive,
    separated_approximation_certifies_sign,
};
use vstd::arithmetic::power2::lemma2_to64;

verus! {
pub(crate) open spec fn absolute(value: real) -> real {
    if value >= 0real { value } else { -value }
}

pub(crate) open spec fn real_binade(value: real, exponent: int) -> bool {
    binary_unit(exponent) <= absolute(value) < 2real * binary_unit(exponent)
}

pub(crate) open spec fn magnitude_upper_bound(value: real, exponent: int) -> bool {
    absolute(value) < 2real * binary_unit(exponent)
}

proof fn binary_unit_successor(exponent: int)
    ensures binary_unit(exponent + 1) == 2real * binary_unit(exponent),
        binary_unit(exponent) > 0real,
{
    binary_unit_positive(exponent);
    binary_unit_composes(exponent, 1);
    lemma2_to64();
    assert(binary_unit(1) == 2real) by (nonlinear_arith)
        requires vstd::arithmetic::power2::pow2(0) == 1,
            vstd::arithmetic::power2::pow2(1) == 2;
}

proof fn magnitude_upper_product(x: real, y: real, left: int, right: int)
    requires magnitude_upper_bound(x, left), magnitude_upper_bound(y, right),
    ensures magnitude_upper_bound(x * y, left + right + 1),
{
    binary_unit_positive(left);
    binary_unit_positive(right);
    binary_unit_composes(left, right);
    binary_unit_successor(left + right);
    let a = absolute(x); let b = absolute(y);
    let u = binary_unit(left); let v = binary_unit(right);
    assert(absolute(x * y) == absolute(x) * absolute(y)) by (nonlinear_arith);
    assert(a * b < 4real * u * v) by (nonlinear_arith)
        requires 0real <= a < 2real * u, 0real <= b < 2real * v, u > 0real, v > 0real;
    assert(absolute(x * y) < 2real * binary_unit(left + right + 1)) by (nonlinear_arith)
        requires absolute(x * y) == a * b, a * b < 4real * u * v,
            binary_unit(left + right + 1) == 2real * binary_unit(left + right),
            binary_unit(left + right) == u * v;
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

// Inexact exponents certify an upper magnitude bound. Exact exponents also
// certify the lower binade boundary. Producers still owe these input facts.
pub(crate) open spec fn bound_denotes(bound: BoundInfo, value: real) -> bool {
    match bound {
        BoundInfo::Unknown => true,
        BoundInfo::Zero => value == 0real,
        BoundInfo::NonZero { sign, msd, exact_msd } => value != 0real
            && sign_denotes(sign, value)
            && match msd {
                Some(e) => magnitude_upper_bound(value, e as int)
                    && (exact_msd ==> real_binade(value, e as int)),
                None => true,
            },
    }
}

// Compatibility is a premise about two certificates for one immutable real.
// It is not established merely by storing them in the same cache slot.
pub(crate) open spec fn compatible_bounds(left: BoundInfo, right: BoundInfo) -> bool {
    match (left, right) {
        (BoundInfo::Unknown, _) | (_, BoundInfo::Unknown) => true,
        (BoundInfo::Zero, BoundInfo::Zero) => true,
        (BoundInfo::NonZero { sign: ls, msd: lm, exact_msd: le },
         BoundInfo::NonZero { sign: rs, msd: rm, exact_msd: re }) =>
            (match (ls, rs) { (Some(l), Some(r)) => l == r, _ => true })
            && (match (lm, rm) {
                (Some(l), Some(r)) => (le ==> l <= r) && (re ==> r <= l),
                _ => true,
            }),
        _ => false,
    }
}

// An unavailable exponent conveys no magnitude, even if exact_msd is set.
// Every available sign, zero/nonzero fact and lower/upper bound must survive.
pub(crate) open spec fn retains_bound(before: BoundInfo, after: BoundInfo) -> bool {
    match (before, after) {
        (BoundInfo::Unknown, _) => true,
        (BoundInfo::Zero, BoundInfo::Zero) => true,
        (BoundInfo::NonZero { sign: bs, msd: bm, exact_msd: be },
         BoundInfo::NonZero { sign: as_, msd: am, exact_msd: ae }) =>
            (bs.is_some() ==> as_ == bs)
            && (match bm {
                Some(b) => match am {
                    Some(a) => a <= b && (be ==> ae && a == b),
                    None => false,
                },
                None => true,
            }),
        _ => false,
    }
}

proof fn exact_binade_cannot_exceed_upper(value: real, exact: int, upper: int)
    requires real_binade(value, exact), magnitude_upper_bound(value, upper),
    ensures exact <= upper,
{
    if upper < exact {
        magnitude_upper_bound_is_monotone(value, upper, exact - 1);
        binary_unit_successor(exact - 1);
    }
}

proof fn bounds_for_one_real_are_compatible(left: BoundInfo, right: BoundInfo, value: real)
    requires bound_denotes(left, value), bound_denotes(right, value),
    ensures compatible_bounds(left, right),
{
    if let (BoundInfo::NonZero { msd: Some(l), exact_msd: le, .. },
            BoundInfo::NonZero { msd: Some(r), exact_msd: re, .. }) = (left, right) {
        if le { exact_binade_cannot_exceed_upper(value, l as int, r as int); }
        if re { exact_binade_cannot_exceed_upper(value, r as int, l as int); }
    }
}

proof fn merged_bound_preserves_denotation(
    retained: BoundInfo, incoming: BoundInfo, value: real, result: BoundInfo,
)
    requires bound_denotes(retained, value), bound_denotes(incoming, value),
        call_ensures(BoundInfo::merge, (retained, incoming), result),
    ensures bound_denotes(result, value),
{}

proof fn merged_bound_retains_compatible_certificates(
    retained: BoundInfo, incoming: BoundInfo, result: BoundInfo,
)
    requires compatible_bounds(retained, incoming),
        call_ensures(BoundInfo::merge, (retained, incoming), result),
    ensures retains_bound(retained, result), retains_bound(incoming, result),
{}

proof fn merged_bound_preserves_domain_certificates(
    retained: &BoundInfo, incoming: &BoundInfo, merged: &BoundInfo, domain: RequiredRealDomain,
    retained_answer: bool, incoming_answer: bool, merged_answer: bool,
)
    requires compatible_bounds(*retained, *incoming),
        call_ensures(BoundInfo::merge, (*retained, *incoming), *merged),
        call_ensures(BoundInfo::proves_domain, (retained, domain), retained_answer),
        call_ensures(BoundInfo::proves_domain, (incoming, domain), incoming_answer),
        call_ensures(BoundInfo::proves_domain, (merged, domain), merged_answer),
    ensures (retained_answer || incoming_answer) ==> merged_answer,
{
    merged_bound_retains_compatible_certificates(*retained, *incoming, *merged);
}

proof fn constructor_preserves_certificates(
    sign: Sign, msd: Option<i32>, exact_msd: bool, value: real, result: BoundInfo,
)
    requires sign_denotes(Some(sign), value),
        match sign {
            Sign::NoSign => true,
            _ => match msd {
                Some(e) => magnitude_upper_bound(value, e as int)
                    && (exact_msd ==> real_binade(value, e as int)),
                None => true,
            },
        },
        call_ensures(BoundInfo::with_sign_msd, (sign, msd, exact_msd), result),
    ensures bound_denotes(result, value),
{}

proof fn separated_approximation_yields_a_bound(
    value: real, approximation: int, unit: real, sign: Sign, msd: Option<i32>, exact_msd: bool, result: BoundInfo,
)
    requires approximates(value, approximation, unit),
        approximation > 1 || approximation < -1,
        if approximation > 0 { sign == Sign::Plus } else { sign == Sign::Minus },
        match msd {
            Some(e) => (absolute(approximation as real) + 1real) * unit < 2real * binary_unit(e as int)
                && (exact_msd ==> binary_unit(e as int) <= (absolute(approximation as real) - 1real) * unit),
            None => true,
        },
        call_ensures(BoundInfo::with_sign_msd, (sign, msd, exact_msd), result),
    ensures bound_denotes(result, value),
{
    separated_approximation_certifies_sign(value, approximation, unit);
    assert((absolute(approximation as real) - 1real) * unit <= absolute(value)
        <= (absolute(approximation as real) + 1real) * unit) by (nonlinear_arith)
        requires unit > 0real,
            -unit <= approximation as real * unit - value <= unit;
    constructor_preserves_certificates(sign, msd, exact_msd, value, result);
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
    if let BoundInfo::NonZero { msd: Some(exponent), exact_msd: true, .. } = bound {
        let e = exponent as int;
        binary_unit_positive(e); binary_unit_positive(-e);
        binary_unit_composes(e, -e);
        lemma2_to64();
        assert(binary_unit(0) == 1real) by (nonlinear_arith)
            requires vstd::arithmetic::power2::pow2(0) == 1;
        let a = absolute(value); let b = absolute(1real / value);
        let u = binary_unit(e); let inverse = binary_unit(-e);
        assert(absolute(value) * absolute(1real / value) == 1real) by (nonlinear_arith)
            requires value != 0real;
        assert(b <= inverse) by (nonlinear_arith)
            requires a >= u, u > 0real, inverse > 0real, u * inverse == 1real,
                a * b == 1real, b >= 0real;
    }
}

proof fn squaring_preserves_bound_denotation(bound: BoundInfo, value: real, result: BoundInfo)
    requires bound_denotes(bound, value), call_ensures(BoundInfo::square, (bound,), result),
    ensures bound_denotes(result, value * value),
{
    assert(value != 0real ==> value * value > 0real) by (nonlinear_arith);
    if let BoundInfo::NonZero { msd: Some(exponent), .. } = bound {
        magnitude_upper_product(value, value, exponent as int, exponent as int);
    }
}

proof fn multiplication_preserves_bound_denotation(
    left: BoundInfo, right: BoundInfo, x: real, y: real, result: BoundInfo,
)
    requires bound_denotes(left, x), bound_denotes(right, y),
        call_ensures(BoundInfo::multiply, (left, right), result),
    ensures bound_denotes(result, x * y),
{
    assert(x != 0real && y != 0real ==> x * y != 0real) by (nonlinear_arith);
    assert(x > 0real && y > 0real ==> x * y > 0real) by (nonlinear_arith);
    assert(x < 0real && y < 0real ==> x * y > 0real) by (nonlinear_arith);
    assert(x > 0real && y < 0real ==> x * y < 0real) by (nonlinear_arith);
    assert(x < 0real && y > 0real ==> x * y < 0real) by (nonlinear_arith);
    if let (BoundInfo::NonZero { msd: Some(l), .. }, BoundInfo::NonZero { msd: Some(r), .. }) = (left, right) {
        magnitude_upper_product(x, y, l as int, r as int);
    }
}

proof fn distinct_real_binades_order_magnitudes(x: real, y: real, left: int, right: int)
    requires real_binade(x, left), real_binade(y, right), left < right,
    ensures absolute(x) < absolute(y),
{
    binary_unit_positive(left);
    binary_unit_composes(left, right - left);
    vstd::arithmetic::power2::lemma_pow2_strictly_increases(0, (right - left) as nat);
    lemma2_to64();
    assert(binary_unit(right - left) >= 2real) by (nonlinear_arith)
        requires right - left > 0,
            vstd::arithmetic::power2::pow2((right - left) as nat) >= 2,
            crate::magnitude_model::binary_denominator(right - left) == 1;
    assert(2real * binary_unit(left) <= binary_unit(right)) by (nonlinear_arith)
        requires binary_unit(left) > 0real, binary_unit(right - left) >= 2real,
            binary_unit(right) == binary_unit(left) * binary_unit(right - left);
}

proof fn magnitude_upper_bound_is_monotone(value: real, exponent: int, upper: int)
    requires magnitude_upper_bound(value, exponent), exponent <= upper,
    ensures magnitude_upper_bound(value, upper),
{
    binary_unit_positive(exponent);
    binary_unit_positive(upper);
    if exponent < upper {
        distinct_real_binades_order_magnitudes(
            binary_unit(exponent), binary_unit(upper), exponent, upper,
        );
    }
}

// These are real-domain conditions, not convergence ranges for a series.
// A real denotation and the validity of its incoming bound are premises;
// graph existence, cache producers and callers remain separate obligations.
spec fn real_domain_holds(domain: RequiredRealDomain, value: real) -> bool {
    match domain {
        RequiredRealDomain::Nonzero => value != 0real,
        RequiredRealDomain::Nonnegative => value >= 0real,
        RequiredRealDomain::Positive => value > 0real,
        RequiredRealDomain::AboveMinusOne => value > -1real,
        RequiredRealDomain::AtLeastOne => value >= 1real,
        RequiredRealDomain::ClosedUnit => -1real <= value <= 1real,
        RequiredRealDomain::OpenUnit => -1real < value < 1real,
        RequiredRealDomain::Probability => 0real < value < 1real,
    }
}

proof fn upper_magnitude_proves_open_unit(value: real, exponent: i32)
    requires magnitude_upper_bound(value, exponent as int), exponent < 0,
    ensures -1real < value < 1real,
{
    magnitude_upper_bound_is_monotone(value, exponent as int, -1);
    binary_unit_successor(-1);
    lemma2_to64();
    assert(binary_unit(0) == 1real) by (nonlinear_arith)
        requires vstd::arithmetic::power2::pow2(0) == 1;
    assert(absolute(value) < 1real);
}

proof fn positive_binade_proves_at_least_one(value: real, exponent: i32)
    requires value > 0real, real_binade(value, exponent as int), exponent >= 0,
    ensures value >= 1real,
{
    lemma2_to64();
    assert(binary_unit(0) == 1real) by (nonlinear_arith)
        requires vstd::arithmetic::power2::pow2(0) == 1;
    if exponent > 0 {
        distinct_real_binades_order_magnitudes(1real, value, 0, exponent as int);
    }
}

// call_ensures refers to the included production body, not a duplicate model
// implementation. False remains lack of this particular sufficient proof.
proof fn accepted_bound_proves_real_domain(
    bound: &BoundInfo, domain: RequiredRealDomain, value: real, result: bool,
)
    requires bound_denotes(*bound, value),
        call_ensures(BoundInfo::proves_domain, (bound, domain), result), result,
    ensures real_domain_holds(domain, value),
{
    if let BoundInfo::NonZero { msd: Some(exponent), .. } = *bound {
        match domain {
            RequiredRealDomain::AtLeastOne => {
                positive_binade_proves_at_least_one(value, exponent);
            },
            RequiredRealDomain::AboveMinusOne | RequiredRealDomain::ClosedUnit
                | RequiredRealDomain::OpenUnit | RequiredRealDomain::Probability => {
                upper_magnitude_proves_open_unit(value, exponent);
            },
            _ => {},
        }
    }
}

// Connect the included production exponent conversion to an incoming upper
// certificate. The rational bit-length/comparison producer remains an obligation.
proof fn clamped_upper_exponent_preserves_certificates(
    value: real, numerator_bits: u64, denominator_bits: u64, offset: i32,
    exact_msd: bool, result: Option<(i32, bool)>,
)
    requires magnitude_upper_bound(value,
            crate::verified::word::bit_length_msd(numerator_bits, denominator_bits, false, offset)),
        exact_msd ==> real_binade(value,
            crate::verified::word::bit_length_msd(numerator_bits, denominator_bits, false, offset)),
        call_ensures(crate::verified::word::upper_bit_length_msd,
            (numerator_bits, denominator_bits, offset), result),
    ensures match result {
        Some((upper, unclamped)) => magnitude_upper_bound(value, upper as int)
            && (exact_msd && unclamped ==> real_binade(value, upper as int)),
        None => true,
    },
{
    if let Some((upper, _)) = result {
        magnitude_upper_bound_is_monotone(value,
            crate::verified::word::bit_length_msd(numerator_bits, denominator_bits, false, offset),
            upper as int);
    }
}

proof fn addition_preserves_bound_denotation(
    left: BoundInfo, right: BoundInfo, x: real, y: real, result: BoundInfo,
)
    requires bound_denotes(left, x), bound_denotes(right, y),
        call_ensures(BoundInfo::add, (left, right), result),
    ensures bound_denotes(result, x + y),
{
    match (left, right) {
        (BoundInfo::NonZero { sign: ls, msd: Some(l), exact_msd: le },
         BoundInfo::NonZero { sign: rs, msd: Some(r), exact_msd: re }) => {
            if le && re {
                if l < r {
                    distinct_real_binades_order_magnitudes(x, y, l as int, r as int);
                } else if r < l {
                    distinct_real_binades_order_magnitudes(y, x, r as int, l as int);
                }
            }
            let opposed = matches!((ls, rs),
                (Some(Sign::Plus), Some(Sign::Minus)) | (Some(Sign::Minus), Some(Sign::Plus)));
            magnitude_upper_sum(x, y, l as int, r as int, opposed);
        },
        _ => {},
    }
}

proof fn magnitude_upper_sum(x: real, y: real, left: int, right: int, opposed: bool)
    requires magnitude_upper_bound(x, left), magnitude_upper_bound(y, right),
        opposed ==> ((x > 0real && y < 0real) || (x < 0real && y > 0real)),
    ensures magnitude_upper_bound(x + y,
        (if left > right { left } else { right }) + if opposed { 0int } else { 1int }),
{
    let maximum = if left > right { left } else { right };
    binary_unit_positive(left); binary_unit_positive(right); binary_unit_positive(maximum);
    if left < maximum {
        distinct_real_binades_order_magnitudes(binary_unit(left), binary_unit(maximum), left, maximum);
    }
    if right < maximum {
        distinct_real_binades_order_magnitudes(binary_unit(right), binary_unit(maximum), right, maximum);
    }
    assert(absolute(x) < 2real * binary_unit(maximum));
    assert(absolute(y) < 2real * binary_unit(maximum));
    if opposed {
        assert(absolute(x + y) <= absolute(x) || absolute(x + y) <= absolute(y));
    } else {
        binary_unit_successor(maximum);
        assert(absolute(x + y) <= absolute(x) + absolute(y));
    }
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
        BoundInfo::NonZero { sign: Some(Sign::Plus), msd: Some(exponent), exact_msd } => {
            positive_root_upper_bound(value, root, exponent);
            if exact_msd { positive_root_binade(value, root, exponent); }
            assert(i32::MIN <= exponent as int / 2 <= i32::MAX);
        },
        _ => {},
    }
}

proof fn positive_root_upper_bound(value: real, root: real, exponent: i32)
    requires value > 0real, root >= 0real, root * root == value,
        magnitude_upper_bound(value, exponent as int),
    ensures magnitude_upper_bound(root, exponent as int / 2),
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
    assert(root * root < 4real * unit * unit) by (nonlinear_arith)
        requires unit > 0real, factor == 1real || factor == 2real,
            binary_unit(exponent as int) == unit * unit * factor,
            value < 2real * binary_unit(exponent as int), root * root == value;
    assert(root < 2real * unit) by (nonlinear_arith)
        requires unit > 0real, root >= 0real, root * root < 4real * unit * unit;
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
        Some(bits) => value != 0real && magnitude_upper_bound(value, bits.msd as int)
            && (bits.exact_msd ==> real_binade(value, bits.msd as int)),
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
        BoundInfo::NonZero { msd: Some(exponent), exact_msd, .. } => {
            binary_scaling_preserves_magnitude_upper_bound(value, exponent as int, offset);
            if exact_msd { binary_scaling_preserves_real_binade(value, exponent as int, offset); }
        },
        _ => {},
    }
}

proof fn binary_scaling_preserves_magnitude_upper_bound(value: real, exponent: int, offset: int)
    requires magnitude_upper_bound(value, exponent),
    ensures magnitude_upper_bound(value * binary_unit(offset), exponent + offset),
{
    binary_unit_positive(offset);
    binary_unit_composes(exponent, offset);
    let scale = binary_unit(offset); let unit = binary_unit(exponent);
    assert(absolute(value * scale) == absolute(value) * scale) by (nonlinear_arith)
        requires scale > 0real;
    assert(magnitude_upper_bound(value * scale, exponent + offset)) by (nonlinear_arith)
        requires scale > 0real, absolute(value) < 2real * unit,
            absolute(value * scale) == absolute(value) * scale,
            binary_unit(exponent + offset) == unit * scale;
}
}
