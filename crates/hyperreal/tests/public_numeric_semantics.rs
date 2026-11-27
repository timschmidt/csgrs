//! Public numeric convenience API witnesses with exact rational/integer expectations.
//!
//! The finite corpus covers integer rounding, rational remainder, angle/aggregate
//! helpers and selected linear algebra aliases. Adjacent-integer choices may differ
//! after refinement while remaining valid. These tests do not establish arbitrary
//! symbolic completion, resource bounds or cancellation behavior.

use hyperreal::{Computable, Problem, Rational, Real};
use num::BigInt;

/// Construct an exact signed fixture fraction; panic if the supplied denominator is zero.
fn r(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

#[test]
/// Check nine zero/half-integer fixtures from -7/2 to 7/2 against explicit floor, ceiling, truncation and round results.
/// Require the fractional result to equal x-trunc(x); half ties round away from zero.
fn certified_integer_operations_cover_signs_and_half_ties() {
    let cases = [
        (-7, 2, -4, -3, -3, -4),
        (-5, 2, -3, -2, -2, -3),
        (-3, 2, -2, -1, -1, -2),
        (-1, 2, -1, 0, 0, -1),
        (0, 2, 0, 0, 0, 0),
        (1, 2, 0, 1, 0, 1),
        (3, 2, 1, 2, 1, 2),
        (5, 2, 2, 3, 2, 3),
        (7, 2, 3, 4, 3, 4),
    ];

    for (numerator, denominator, floor, ceil, trunc, round) in cases {
        let value = r(numerator, denominator);
        assert_eq!(value.floor_certified().unwrap(), BigInt::from(floor));
        assert_eq!(value.ceil_certified().unwrap(), BigInt::from(ceil));
        assert_eq!(value.trunc_certified().unwrap(), BigInt::from(trunc));
        assert_eq!(value.round_certified().unwrap(), BigInt::from(round));
        assert_eq!(value.fract_certified().unwrap(), value - Real::from(trunc),);
    }
}

#[test]
/// Check 8,224 numerator/denominator pairs against integer Euclidean floor/ceiling oracles for both Real and Computable.
/// Also check +/-pi, sqrt(2), exp(sqrt(2)), and 2^4096+pi before/after formatting separate instances.
/// Each result may choose either adjacent integer; cache-state bitwise agreement is not required.
/// The sine-square plus cosine-square identity must return one at the exact integer boundary.
fn near_integer_is_a_total_adjacent_integer_choice() {
    for denominator in 1_i64..=32 {
        for numerator in -128_i64..=128 {
            let exact = Rational::fraction(numerator, denominator as u64).unwrap();
            let value = Real::new(exact.clone());
            let choice = value.near_integer();
            let computable_choice = Computable::rational(exact).near_integer();
            let floor = numerator.div_euclid(denominator);
            let ceil = -(-numerator).div_euclid(denominator);
            assert!(
                choice == BigInt::from(floor) || choice == BigInt::from(ceil),
                "near_integer({numerator}/{denominator}) returned {choice}"
            );
            assert!(
                computable_choice == BigInt::from(floor) || computable_choice == BigInt::from(ceil),
                "Computable::near_integer({numerator}/{denominator}) returned {computable_choice}"
            );
        }
    }

    for (value, floor, ceil) in [
        (Real::pi(), 3_i64, 4_i64),
        (-Real::pi(), -4, -3),
        (Real::from(2_i32).sqrt().unwrap(), 1, 2),
        (Real::from(2_i32).sqrt().unwrap().exp().unwrap(), 4, 5),
    ] {
        let choice = value.near_integer();
        assert!(choice == BigInt::from(floor) || choice == BigInt::from(ceil));
    }

    let huge = BigInt::from(1_u8) << 4_096_usize;
    let shifted = Real::integer(huge.clone()) + Real::pi();
    let cold = shifted.near_integer();
    assert!(cold == &huge + 3_u8 || cold == &huge + 4_u8);

    // A finer query may change which permitted adjacent integer a cache yields,
    // but it cannot make the multivalued result leave the certified pair.
    let warmed = Real::integer(huge.clone()) + Real::pi();
    let _ = warmed.to_string();
    let warm_choice = warmed.near_integer();
    assert!(warm_choice == &huge + 3_u8 || warm_choice == huge + 4_u8);

    let angle = Real::from(2_i32).sqrt().unwrap();
    let sine = angle.clone().sin();
    let cosine = angle.cos();
    let trig_identity = &sine * &sine + &cosine * &cosine;
    // This exact integer boundary needs no equality decision: either adjacent
    // choice is necessarily the integer itself.
    assert_eq!(trig_identity.near_integer(), BigInt::from(1_u8));
}

#[test]
/// Check floor, ceiling and nearest integer for pi, and truncation toward zero for negative pi.
fn certified_integer_operations_handle_irrational_values() {
    assert_eq!(Real::pi().floor_certified().unwrap(), BigInt::from(3));
    assert_eq!(Real::pi().ceil_certified().unwrap(), BigInt::from(4));
    assert_eq!((-Real::pi()).trunc_certified().unwrap(), BigInt::from(-3));
    assert_eq!(Real::pi().round_certified().unwrap(), BigInt::from(3));
}

#[test]
/// For 41 quarter-integers, require the remainder modulo 3/2 to lie in [0, 3/2),
/// leave an integer quotient and reconstruct the exact dividend. Separately require NotANumber for zero modulus.
fn euclidean_remainder_is_nonnegative_and_reconstructs_input() {
    let modulus = r(3, 2);
    for numerator in -20_i64..=20 {
        let value = r(numerator, 4);
        let remainder = value.rem_euclid_certified(&modulus).unwrap();
        assert!(remainder >= Real::zero());
        assert!(remainder < modulus);

        let quotient = ((value.clone() - remainder.clone()) / modulus.clone()).unwrap();
        assert!(quotient.is_integer());
        assert_eq!(quotient * modulus.clone() + remainder, value);
    }

    assert_eq!(
        Real::one().rem_euclid_certified(&Real::zero()),
        Err(Problem::NotANumber),
    );
}

#[test]
/// Check exact 180-degree/pi conversion and the mean/sample deviation of [1, 2, 3].
/// Require None for an empty mean and one-element sample deviation; no generic failure category is inspected.
fn angle_and_aggregate_helpers_preserve_exact_results() {
    assert_eq!(Real::from(180).to_radians(), Real::pi());
    assert_eq!(Real::pi().to_degrees(), Real::from(180));
    assert_eq!(Real::mean(&[]), None);

    let values = [Real::from(1), Real::from(2), Real::from(3)];
    assert_eq!(Real::mean(&values), Some(Real::from(2)));
    assert_eq!(Real::sample_stddev(&values), Some(Real::one()));
    assert_eq!(Real::sample_stddev(&values[..1]), None);
}

#[test]
/// Check the 3-4-5 and 2-3-6-7 norm identities, plus hypot(x, 4)-x at x=3 and x=-3.
fn hypot_helpers_match_exact_pythagorean_values() {
    assert_eq!(
        Real::hypot2(&Real::from(3), &Real::from(4)).unwrap(),
        Real::from(5),
    );
    assert_eq!(
        Real::hypot3(&Real::from(2), &Real::from(3), &Real::from(6)).unwrap(),
        Real::from(7),
    );
    assert_eq!(
        Real::hypot_minus(&Real::from(3), &Real::from(4)).unwrap(),
        Real::from(2),
    );
    assert_eq!(
        Real::hypot_minus(&Real::from(-3), &Real::from(4)).unwrap(),
        Real::from(8),
    );
}

#[test]
/// Compare selected dot/linear/affine entry points with explicit integer expectations.
/// Check mismatched sum-products lengths return ParseError, ascending-coefficient polynomial evaluation
/// at two returns 17, the empty polynomial gives zero, and the rational polynomial gives 17/3.
/// Agreement between active/convenience variants does not independently prove their algorithms.
fn public_linear_algebra_helpers_match_expanded_arithmetic() {
    let a = [Real::from(1), Real::from(2), Real::from(3), Real::from(4)];
    let b = [Real::from(5), Real::from(6), Real::from(7), Real::from(8)];
    let dot3 = Real::from(38);
    let dot4 = Real::from(70);

    assert_eq!(Real::mul_add(&a[1], &b[1], &a[0]), Real::from(13));
    assert_eq!(Real::sum_products(&a, &b).unwrap(), dot4);
    assert_eq!(
        Real::sum_products(&a[..3], &b[..2]),
        Err(Problem::ParseError)
    );
    assert_eq!(
        Real::diff_of_products(&a[3], &b[2], &a[1], &b[0]),
        Real::from(18)
    );

    assert_eq!(
        Real::dot3_refs([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]),
        dot3
    );
    assert_eq!(
        Real::active_dot3_refs([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]),
        dot3
    );
    assert_eq!(
        Real::dot4_refs([&a[0], &a[1], &a[2], &a[3]], [&b[0], &b[1], &b[2], &b[3]]),
        dot4
    );
    assert_eq!(
        Real::active_dot4_refs([&a[0], &a[1], &a[2], &a[3]], [&b[0], &b[1], &b[2], &b[3]]),
        dot4
    );
    assert_eq!(
        Real::linear_combination3_refs([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]),
        dot3
    );
    assert_eq!(
        Real::active_linear_combination3_refs([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]),
        dot3
    );
    assert_eq!(
        Real::linear_combination4_refs([&a[0], &a[1], &a[2], &a[3]], [&b[0], &b[1], &b[2], &b[3]]),
        dot4
    );
    assert_eq!(
        Real::active_linear_combination4_refs(
            [&a[0], &a[1], &a[2], &a[3]],
            [&b[0], &b[1], &b[2], &b[3]]
        ),
        dot4
    );
    assert_eq!(
        Real::affine_combination3_refs(
            [&a[0], &a[1], &a[2]],
            [&b[0], &b[1], &b[2]],
            &Real::from(9)
        ),
        Real::from(47)
    );
    assert_eq!(
        Real::affine_combination4_refs(
            [&a[0], &a[1], &a[2], &a[3]],
            [&b[0], &b[1], &b[2], &b[3]],
            &Real::from(9)
        ),
        Real::from(79)
    );
    assert_eq!(Real::affine(&a[0], &a[1], &a[2]), Real::from(7));

    let coeffs = [Real::from(1), Real::from(2), Real::from(3)];
    assert_eq!(Real::eval_poly(&coeffs, &Real::from(2)), Real::from(17));
    assert_eq!(Real::eval_poly(&[], &Real::from(2)), Real::zero());
    assert_eq!(
        Real::eval_rational_poly(&coeffs, &[Real::from(1), Real::from(1)], &Real::from(2)).unwrap(),
        r(17, 3)
    );
}

#[test]
/// Require four signed-product-sum paths to yield 2*3-4*5=-14.
/// Integral fixtures meet the known-exact/shared-denominator preconditions; no heterogeneous denominator case is covered.
fn exact_product_sum_entry_points_agree() {
    let a = Real::from(2);
    let b = Real::from(3);
    let c = Real::from(4);
    let d = Real::from(5);
    let signs = [true, false];
    let terms = [[&a, &b], [&c, &d]];
    let expected = Real::from(-14);

    assert_eq!(
        Real::exact_rational_signed_product_sum(signs, terms),
        Some(expected.clone())
    );
    assert_eq!(
        Real::exact_rational_signed_product_sum_known_exact(signs, terms),
        expected
    );
    assert_eq!(
        Real::exact_rational_signed_product_sum_known_shared_denominator(signs, terms),
        expected
    );
    assert_eq!(Real::active_signed_product_sum(signs, terms), expected);
}

#[test]
/// Check selected rational/identity accessors, inversion, two-element sums and ordered min/max values.
/// Compare the lbeta alias with ln_beta for (2, 3); this is alias agreement, not an independent logarithmic oracle.
fn public_aliases_and_structural_accessors_are_consistent() {
    let third = r(1, 3);
    assert!(third.is_rational());
    assert!(third.prefer_fraction());
    assert_eq!(
        third.exact_rational_ref(),
        Some(&Rational::fraction(1, 3).unwrap())
    );
    assert_eq!(third.inverse_ref().unwrap(), Real::from(3));
    assert!(third.definitely_not_equal(&Real::one()));
    assert!(Real::one().definitely_one());
    assert_eq!(Real::zero().zero_or_one(), Some(false));
    assert_eq!(Real::one().zero_or_one(), Some(true));
    assert_eq!(Real::sum_owned([Real::one(), Real::from(2)]), Real::from(3));
    let values = [Real::one(), Real::from(2)];
    assert_eq!(Real::sum_refs(values.iter()), Real::from(3));
    assert_eq!(Real::from(1).min(&Real::from(2)), &Real::from(1));
    assert_eq!(Real::from(1).max(&Real::from(2)), &Real::from(2));
    assert_eq!(
        Real::lbeta(&Real::from(2), &Real::from(3)),
        Real::ln_beta(&Real::from(2), &Real::from(3))
    );
}
