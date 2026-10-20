//! Enclosure and sign contracts of recursive quadratic coefficient fields.
//!
//! Expected signs come from independently built `Real` closed forms under
//! STRICT predicates, or from equalities known by construction (for example
//! `2*sqrt(2) == sqrt(8)` and `sqrt(c^2) == c` for positive `c`). Values over
//! `2^(1/3)` have no compact witness, so they exercise the dense interval,
//! norm and projected-zero authorities rather than scalar replay.

#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

use hyperreal::{Real, RealSign};
use hypersolve::SelectedAlgebraContext;
use hypersolve::classification::{Classification, UncertaintyReason};
use hypersolve::recursive_quadratic_field::*;
use hypersolve::{AlgebraicRootRepresentation, AlgebraicRootValidationStatus};
use support::*;

/// A rank-one base over `c = 2^(1/3)` with the given speed polynomials.
fn cube_field(first: &[Real], second: &[Real]) -> RecursiveQuadraticField {
    RecursiveQuadraticField::base(
        vec![cube_root_two()],
        axis_tensor(first),
        axis_tensor(second),
    )
    .expect("rank-one speeds")
}

/// The selected axis value `c`.
fn cube(field: &RecursiveQuadraticField) -> RecursiveQuadraticValue {
    field.retained_root_value(&cube_root_two()).expect("axis")
}

/// `2*sqrt2 - sqrt8` over the cube-root tuple: zero by construction, with
/// no stored cancellation and no compact scalar witness.
fn hidden_zero(field: &RecursiveQuadraticField) -> RecursiveQuadraticValue {
    let first = axis_value(
        field,
        &[real(0), real(1)],
        &[real(2)],
        &[real(0)],
        &[real(0)],
    )
    .unwrap();
    let second = axis_value(
        field,
        &[real(0), real(1)],
        &[real(0)],
        &[real(1)],
        &[real(0)],
    )
    .unwrap();
    first.subtract(&second).unwrap()
}

/// `F(sqrt(c^2))` and its generator minus `c`, a hidden zero in an extension.
fn hidden_extension_zero() -> (RecursiveQuadraticField, RecursiveQuadraticValue) {
    let field = cube_field(&[real(1)], &[real(1)]);
    let c = cube(&field);
    let extension = field.extension(c.square().unwrap()).unwrap();
    let value = extension
        .element(
            c.scale(&real(-1)).unwrap(),
            field.constant(real(1)).unwrap(),
        )
        .unwrap();
    (extension, value)
}

#[test]
fn intervals_enclose_tower_values_at_every_precision() {
    let tower = Tower::new();
    let value = tower
        .second
        .element(
            generator(&tower.first).scale(&real(-2)).unwrap(),
            tower.first.constant(real(3)).unwrap(),
        )
        .unwrap();
    let expected = real(-2) * Tower::first_closed_form() + real(3) * Tower::second_closed_form();
    for steps in [0, 4, 32] {
        assert_encloses(&value.interval(steps).unwrap(), &expected);
        assert_encloses(
            &value
                .interval_with_coefficient_precision(steps, Some(-64))
                .unwrap(),
            &expected,
        );
    }
    // Over a witnessed tuple the retained replay is an exact point.
    let point = value.interval_over_source_box(&[], Some(-64)).unwrap();
    assert_real_eq(&point.lower, &expected);
    assert_real_eq(&point.upper, &expected);
    let lifted = tower
        .second
        .lift(&int_value(&tower.base, [1, 1, 0, 0]))
        .unwrap();
    assert_encloses(
        &lifted
            .interval_over_source_box_with_witnesses(&[], None, false)
            .unwrap(),
        &(real(1) + root_of(2)),
    );
}

#[test]
fn intervals_over_unwitnessed_axes_refine_and_shrink() {
    let field = cube_field(&[real(2)], &[real(0), real(1)]);
    // c + 3*sqrt2 - sqrt(c)
    let value = axis_value(
        &field,
        &[real(0), real(1)],
        &[real(3)],
        &[real(-1)],
        &[real(0)],
    )
    .unwrap();
    let c = cube_root_two_real();
    let expected = &c + real(3) * root_of(2) - sqrt(c.clone());
    let coarse = value.interval(0).unwrap();
    let fine = value.interval(64).unwrap();
    assert_encloses(&coarse, &expected);
    assert_encloses(&fine, &expected);
    assert_real_lt(
        &(&fine.upper - &fine.lower),
        &(&coarse.upper - &coarse.lower),
    );
    // A box whose length differs from the tuple is refused.
    assert!(value.interval_over_source_box(&[], Some(-64)).is_none());
    let box_sources = vec![cube_root_two()];
    assert_encloses(
        &value
            .interval_over_source_box(&box_sources, Some(-64))
            .unwrap(),
        &expected,
    );
    assert_eq!(
        value.sign_over_source_box(&box_sources),
        strict_sign(&expected)
    );
    // The tuple caches its refinement; its debug form names its axes.
    let base = base_arc(&field);
    let refined = base.source_box(8);
    assert_eq!(refined.len(), 1);
    assert_encloses(
        &hypersolve::RealInterval {
            lower: refined[0].interval.lower.clone(),
            upper: refined[0].interval.upper.clone(),
        },
        &c,
    );
    assert_real_lt(
        &(&refined[0].interval.upper - &refined[0].interval.lower),
        &real(1),
    );
    assert!(format!("{base:?}").contains("axes: 1"));
    // A witnessed source becomes an exact point box.
    let witnessed = RecursiveQuadraticField::base(
        vec![sqrt_two_root()],
        constant_tensor(1, real(1)),
        constant_tensor(1, real(1)),
    )
    .unwrap();
    let sources = base_arc(&witnessed).source_box(4);
    assert_real_eq(sources[0].exact_point_witness().unwrap(), &root_of(2));
    assert!(base_arc(&quadratic_base(2, 3)).source_box(4).is_empty());
}

#[test]
fn intervals_decline_imaginary_extension_radicands() {
    // `extension` leaves radicand positivity to its caller. A deliberately
    // negative radicand must fail closed: no real enclosure or sign.
    let base = quadratic_base(2, 3);
    let imaginary = base.extension(int_value(&base, [-1, 0, 0, 0])).unwrap();
    let root = generator(&imaginary);
    assert!(root.interval(4).is_none());
    assert_eq!(
        root.sign_with_nonzero_certificate().unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    assert!(root.bounded_interval_sign(0..=16).is_none());
    // The rational part alone still has an enclosure.
    assert_encloses(
        &imaginary.constant(real(4)).unwrap().interval(4).unwrap(),
        &real(4),
    );
}

#[test]
fn bounded_signs_separate_transverse_values_only() {
    let field = cube_field(&[real(2)], &[real(3)]);
    let c = cube_root_two_real();
    // c - 5/4 > 0 since 2 > (5/4)^3 = 125/64.
    let above = cube(&field)
        .subtract(&field.constant(fraction(5, 4)).unwrap())
        .unwrap();
    assert_eq!(
        strict_sign(&(&c - fraction(5, 4))),
        Some(RealSign::Positive)
    );
    assert_eq!(
        above.bounded_interval_sign(0..=64),
        Some(RealSign::Positive)
    );
    // An empty refinement range proves nothing.
    assert_eq!(above.bounded_interval_sign(1..=1), None);
    assert_eq!(
        above.bounded_or_exact_real_witness_sign(),
        Some(RealSign::Positive)
    );
    // c - 13/10 < 0.
    let below = cube(&field)
        .subtract(&field.constant(fraction(13, 10)).unwrap())
        .unwrap();
    assert_eq!(
        strict_sign(&(&c - fraction(13, 10))),
        Some(RealSign::Negative)
    );
    assert_eq!(
        below.bounded_or_exact_real_witness_sign(),
        Some(RealSign::Negative)
    );
    // A hidden zero never separates to a nonzero sign and has no witness.
    let zero = hidden_zero(&field_with_dependent_speeds());
    assert!(matches!(
        zero.bounded_interval_sign(0..=16),
        None | Some(RealSign::Zero)
    ));
    assert_eq!(zero.exact_real_witness_sign_through(-128), None);
    // A structural zero is decided without refinement.
    let stored = field.constant(Real::zero()).unwrap();
    assert_eq!(
        stored.bounded_or_exact_real_witness_sign(),
        Some(RealSign::Zero)
    );
}

fn field_with_dependent_speeds() -> RecursiveQuadraticField {
    cube_field(&[real(2)], &[real(8)])
}

#[test]
fn exact_witness_signs_decide_zero_and_nonzero_values() {
    // Speeds 2 and 8 make 2*sqrt2 - sqrt8 a hidden zero with exact witnesses.
    let field = quadratic_base(2, 8);
    let zero = int_value(&field, [0, 2, -1, 0]);
    assert!(!zero.is_structurally_zero());
    assert_eq!(
        zero.exact_real_witness_sign_through(-64),
        Some(RealSign::Zero)
    );
    assert_eq!(
        zero.bounded_or_exact_real_witness_sign(),
        Some(RealSign::Zero)
    );
    assert_sign(zero.sign(&TestContext::strict()), RealSign::Zero);
    let positive = int_value(&field, [0, 3, -1, 0]);
    assert_eq!(
        positive.exact_real_witness_sign_through(-64),
        Some(RealSign::Positive)
    );
    assert_eq!(
        int_value(&field, [0, 1, -1, 0]).exact_real_witness_sign_through(-64),
        Some(RealSign::Negative)
    );
    // No witness, no answer.
    let unwitnessed = cube(&cube_field(&[real(1)], &[real(1)]));
    assert_eq!(unwitnessed.exact_real_witness_sign_through(-64), None);
}

#[test]
fn signs_of_transverse_values_match_closed_forms() {
    let policy = TestContext::strict();
    let tower = Tower::new();
    // g2 - 6/5 > 0 and g2 - 5/4 < 0 since g2 = (1 + sqrt2)^(1/4) ~ 1.2465.
    let g2 = generator(&tower.second);
    for (offset, expected) in [
        (fraction(6, 5), RealSign::Positive),
        (fraction(5, 4), RealSign::Negative),
    ] {
        assert_eq!(
            strict_sign(&(Tower::second_closed_form() - &offset)),
            Some(expected)
        );
        let value = g2
            .subtract(&tower.second.constant(offset).unwrap())
            .unwrap();
        assert_sign(value.sign(&policy), expected);
        assert_sign(value.sign(&TestContext::approximate()), expected);
    }
    let field = cube_field(&[real(2)], &[real(3)]);
    let value = cube(&field)
        .subtract(&field.constant(fraction(5, 4)).unwrap())
        .unwrap();
    assert_sign(value.sign(&policy), RealSign::Positive);
    assert_sign(
        field.constant(Real::zero()).unwrap().sign(&policy),
        RealSign::Zero,
    );
}

#[test]
fn hidden_base_zeros_are_decided_exactly() {
    let policy = TestContext::strict();
    let field = field_with_dependent_speeds();
    let zero = hidden_zero(&field);
    assert!(!zero.is_structurally_zero());
    assert!(zero.exact_real_value_with_retained_witnesses().is_none());
    assert_sign(zero.sign(&policy), RealSign::Zero);
    // An approximate context first completes the strict replay; no
    // approximation may be consumed for an exactly decidable zero.
    let approximate = TestContext::approximate();
    assert_sign(zero.sign(&approximate), RealSign::Zero);
    assert_eq!(approximate.approximations(), 0);
    // The projected-norm fallback certifies the same zero.
    assert_sign(
        zero.sign_with_projected_zero_fallback(&policy),
        RealSign::Zero,
    );
    // A bounded pass may decline, but never guesses a nonzero sign.
    assert_zero_or_undecided(policy.bounded_exact_predicate_pass(|| zero.sign(&policy)));
}

#[test]
fn hidden_extension_zeros_are_decided_by_their_magnitude() {
    let policy = TestContext::strict();
    let (_, zero) = hidden_extension_zero();
    assert!(!zero.is_structurally_zero());
    assert!(zero.exact_real_value_with_retained_witnesses().is_none());
    assert_sign(zero.sign(&policy), RealSign::Zero);
    assert_sign(
        zero.sign_with_projected_zero_fallback(&policy),
        RealSign::Zero,
    );
    // A bounded pass may decline the hidden zero, never misclassify it.
    assert_zero_or_undecided(policy.bounded_exact_predicate_pass(|| zero.sign(&policy)));
}

#[test]
fn hidden_zero_extension_values_sign_their_nonzero_neighbours() {
    let policy = TestContext::strict();
    let (extension, zero) = hidden_extension_zero();
    // sqrt(c^2) - c + 1/1000 > 0 and sqrt(c^2) - c - 1/1000 < 0.
    for (offset, expected) in [
        (fraction(1, 1000), RealSign::Positive),
        (fraction(-1, 1000), RealSign::Negative),
    ] {
        let value = zero.add(&extension.constant(offset).unwrap()).unwrap();
        assert_sign(value.sign(&policy), expected);
        assert_eq!(
            value.sign_with_projected_zero_fallback(&policy).unwrap(),
            Classification::Uncertain(UncertaintyReason::RealSign)
        );
        assert_sign(value.sign_with_nonzero_certificate(), expected);
        assert_sign(
            value.sign_with_nonzero_certificate_over_source_box(&[cube_root_two()]),
            expected,
        );
    }
}

#[test]
fn nonzero_certificates_refine_until_separation() {
    let field = cube_field(&[real(1)], &[real(1)]);
    // c - 63/50 is about -7.9e-5 (1.26^3 = 2.000376 > 2), close enough to
    // need several refinements; the expected sign is the STRICT closed form.
    let c = cube_root_two_real();
    let offset = fraction(63, 50);
    let expected = strict_sign(&(&c - &offset)).unwrap();
    let value = cube(&field)
        .subtract(&field.constant(offset).unwrap())
        .unwrap();
    assert_sign(value.sign_with_nonzero_certificate(), expected);
    assert_sign(
        value.sign_with_nonzero_certificate_over_source_box(&[cube_root_two()]),
        expected,
    );
    // An invalid source box falls back to the tuple's own refinement.
    let mut invalid = cube_root_two();
    invalid.validation.status = AlgebraicRootValidationStatus::InvalidInterval;
    assert_sign(
        value.sign_with_nonzero_certificate_over_source_box(&[invalid.clone()]),
        expected,
    );
    assert_eq!(
        value
            .sign_over_progressively_refined_source_box(&[invalid], 0..=512, true)
            .unwrap(),
        None
    );
    assert_eq!(
        value
            .sign_over_progressively_refined_source_box(&[cube_root_two()], 0..=512, false)
            .unwrap(),
        Some(expected)
    );
    assert_eq!(
        value
            .sign_over_progressively_refined_source_box(&[cube_root_two()], 3..=3, false)
            .unwrap(),
        None
    );
}

#[test]
fn affine_positive_root_signs_use_exact_witnesses() {
    let policy = TestContext::strict();
    let field = quadratic_base(2, 3);
    // retained + radical*sqrt(radicand) over Q(sqrt2, sqrt3).
    for (retained, radical, radicand) in [
        ([1, 0, 0, 0], [1, 0, 0, 0], [2, 0, 0, 0]),
        ([-2, 0, 0, 0], [1, 0, 0, 0], [3, 0, 0, 0]),
        ([0, 1, 0, 0], [-1, 0, 0, 0], [2, 0, 0, 0]),
        ([3, 0, 0, 0], [0, 0, -1, 0], [3, 0, 0, 0]),
    ] {
        let expected = base_closed_form(2, 3, retained)
            + base_closed_form(2, 3, radical) * sqrt(base_closed_form(2, 3, radicand));
        assert_sign(
            RecursiveQuadraticValue::affine_positive_root_sign(
                &int_value(&field, retained),
                &int_value(&field, radical),
                &int_value(&field, radicand),
                None,
                None,
                &policy,
            ),
            strict_sign(&expected).unwrap(),
        );
    }
}

#[test]
fn affine_positive_root_signs_over_unwitnessed_axes() {
    let policy = TestContext::strict();
    let field = cube_field(&[real(1)], &[real(1)]);
    let c = cube(&field);
    let c_real = cube_root_two_real();
    let constant = |value: Real| field.constant(value).unwrap();
    // Expected signs are STRICT signs of the closed forms in c = 2^(1/3).
    for (retained, radical, radicand, closed) in [
        (
            c.clone(),
            constant(real(-1)),
            constant(fraction(3, 2)),
            &c_real - sqrt(fraction(3, 2)),
        ),
        (
            c.scale(&real(-1)).unwrap(),
            constant(real(1)),
            constant(real(2)),
            -&c_real + root_of(2),
        ),
        (
            constant(real(1)),
            c.clone(),
            constant(real(2)),
            real(1) + &c_real * root_of(2),
        ),
    ] {
        assert_sign(
            RecursiveQuadraticValue::affine_positive_root_sign(
                &retained, &radical, &radicand, None, None, &policy,
            ),
            strict_sign(&closed).unwrap(),
        );
    }
}

#[test]
fn affine_positive_root_signs_of_hidden_zeros_use_the_squared_magnitude() {
    let policy = TestContext::strict();
    let field = cube_field(&[real(1)], &[real(1)]);
    let c = cube(&field);
    let minus_c = c.scale(&real(-1)).unwrap();
    let one = field.constant(real(1)).unwrap();
    let c_squared = c.square().unwrap();
    // -c + 1*sqrt(c^2) = 0 by construction, with opposite term signs and a
    // structurally zero magnitude c^2 - c^2.
    for (retained_sign, radical_sign) in [
        (None, None),
        (Some(RealSign::Negative), None),
        (None, Some(RealSign::Positive)),
        (Some(RealSign::Negative), Some(RealSign::Positive)),
    ] {
        let sign = policy
            .bounded_exact_predicate_pass(|| {
                RecursiveQuadraticValue::affine_positive_root_sign(
                    &minus_c,
                    &one,
                    &c_squared,
                    retained_sign,
                    radical_sign,
                    &policy,
                )
            })
            .unwrap();
        assert_eq!(sign, Classification::Decided(RealSign::Zero));
    }
    // Supplied (true) zero term signs decide without any magnitude.
    let dependent = field_with_dependent_speeds();
    let zero = hidden_zero(&dependent);
    let two = dependent.constant(real(2)).unwrap();
    assert_sign(
        policy.bounded_exact_predicate_pass(|| {
            RecursiveQuadraticValue::affine_positive_root_sign(
                &zero,
                &zero,
                &two,
                Some(RealSign::Zero),
                Some(RealSign::Zero),
                &policy,
            )
        }),
        RealSign::Zero,
    );
}

#[test]
fn affine_positive_root_signs_of_undecidable_terms_follow_the_policy() {
    let field = field_with_dependent_speeds();
    let zero = hidden_zero(&field);
    let two = field.constant(real(2)).unwrap();
    // 0 + 0*sqrt(2): a bounded pass may decline, never misclassify.
    let strict = TestContext::strict();
    assert_zero_or_undecided(strict.bounded_exact_predicate_pass(|| {
        RecursiveQuadraticValue::affine_positive_root_sign(&zero, &zero, &two, None, None, &strict)
    }));
    // The complete strict pass signs each term exactly.
    assert_sign(
        RecursiveQuadraticValue::affine_positive_root_sign(&zero, &zero, &two, None, None, &strict),
        RealSign::Zero,
    );
    // An approximate context may also consume its terminal for this zero.
    let approximate = TestContext::approximate();
    assert_sign(
        RecursiveQuadraticValue::affine_positive_root_sign(
            &zero,
            &zero,
            &two,
            None,
            None,
            &approximate,
        ),
        RealSign::Zero,
    );
}

#[test]
fn nested_positive_root_affine_signs_use_factored_witnesses() {
    let policy = TestContext::strict();
    let field = quadratic_base(2, 3);
    let value = |parts| int_value(&field, parts);
    // sqrt2 * (1 + 1*sqrt2) - (sqrt2 + 2) = 0.
    assert_sign(
        RecursiveQuadraticValue::nested_positive_root_affine_sign(
            &value([1, 0, 0, 0]),
            &value([1, 0, 0, 0]),
            &value([2, 0, 0, 0]),
            &value([-2, -1, 0, 0]),
            &value([2, 0, 0, 0]),
            None,
            None,
            None,
            &policy,
        ),
        RealSign::Zero,
    );
    // sqrt3 * (1 - sqrt2) + 1 = 1 - sqrt3*(sqrt2 - 1) > 0.
    let expected = root_of(3) * (real(1) - root_of(2)) + real(1);
    assert_sign(
        RecursiveQuadraticValue::nested_positive_root_affine_sign(
            &value([1, 0, 0, 0]),
            &value([-1, 0, 0, 0]),
            &value([2, 0, 0, 0]),
            &value([1, 0, 0, 0]),
            &value([3, 0, 0, 0]),
            None,
            None,
            None,
            &policy,
        ),
        strict_sign(&expected).unwrap(),
    );
}

#[test]
fn nested_positive_root_affine_signs_over_unwitnessed_axes() {
    let policy = TestContext::strict();
    let field = cube_field(&[real(1)], &[real(1)]);
    let c_real = cube_root_two_real();
    // Each input is `a + b*c` with rational `a, b`, authored directly; the
    // expected sign is the STRICT sign of the same closed form in `Real`.
    let affine = |[a, b]: [Real; 2]| {
        let closed = &a + &b * &c_real;
        let value = axis_value(&field, &[a, b], &[real(0)], &[real(0)], &[real(0)]).unwrap();
        (value, closed)
    };
    let pair = |a: i64, b: i64| [real(a), real(b)];
    // sqrt(outer) * (retained + radical*sqrt(inner)) + constant.
    let cases = [
        // c + sqrt2 > 0 and constant 1 > 0 agree.
        [pair(0, 1), pair(1, 0), pair(2, 0), pair(1, 0), pair(1, 0)],
        // sqrt(1) * (c + sqrt2) - 1 > 0: opposite signs, positive magnitude.
        [pair(0, 1), pair(1, 0), pair(2, 0), pair(-1, 0), pair(1, 0)],
        // sqrt(1/4) * (c + sqrt2) - 3 < 0: opposite signs, negative magnitude.
        [
            pair(0, 1),
            pair(1, 0),
            pair(2, 0),
            pair(-3, 0),
            [fraction(1, 4), real(0)],
        ],
        // A zero constant defers to the inner sign: sqrt2 * (c - sqrt2) < 0.
        [pair(0, 1), pair(-1, 0), pair(2, 0), pair(0, 0), pair(2, 0)],
        // A zero inner affine term defers to the constant -c.
        [pair(0, 0), pair(0, 0), pair(2, 0), pair(0, -1), pair(3, 0)],
    ];
    for [retained, radical, inner, offset, outer] in cases {
        let (retained, retained_real) = affine(retained);
        let (radical, radical_real) = affine(radical);
        let (inner, inner_real) = affine(inner);
        let (offset, offset_real) = affine(offset);
        let (outer, outer_real) = affine(outer);
        let expected =
            sqrt(outer_real) * (retained_real + radical_real * sqrt(inner_real)) + offset_real;
        assert_sign(
            RecursiveQuadraticValue::nested_positive_root_affine_sign(
                &retained, &radical, &inner, &offset, &outer, None, None, None, &policy,
            ),
            strict_sign(&expected).unwrap(),
        );
    }
}

#[test]
fn nested_positive_root_affine_signs_record_supplied_signs() {
    let policy = TestContext::strict();
    let field = cube_field(&[real(1)], &[real(1)]);
    let c = cube(&field);
    let constant = |value: Real| field.constant(value).unwrap();
    // c + sqrt2 > 0 with an opposing constant -10: magnitude negative.
    assert_sign(
        RecursiveQuadraticValue::nested_positive_root_affine_sign(
            &c,
            &constant(real(1)),
            &constant(real(2)),
            &constant(real(-10)),
            &constant(real(1)),
            Some(RealSign::Positive),
            Some(RealSign::Positive),
            Some(RealSign::Negative),
            &policy,
        ),
        RealSign::Negative,
    );
    assert_eq!(
        strict_sign(&(cube_root_two_real() + root_of(2) - real(10))),
        Some(RealSign::Negative)
    );
    // A hidden-zero inner term: sqrt2 * (0 + 0*sqrt2) + 2 > 0. A bounded
    // pass may decline the inner zero, never misclassify the sum.
    let dependent = field_with_dependent_speeds();
    let zero = hidden_zero(&dependent);
    let two = dependent.constant(real(2)).unwrap();
    assert_sign_or_undecided(
        policy.bounded_exact_predicate_pass(|| {
            RecursiveQuadraticValue::nested_positive_root_affine_sign(
                &zero, &zero, &two, &two, &two, None, None, None, &policy,
            )
        }),
        RealSign::Positive,
    );
}

#[test]
fn projected_zero_fallback_rejects_nonzero_and_unsupported_values() {
    let policy = TestContext::strict();
    let field = quadratic_base(2, 3);
    let nonzero = int_value(&field, [1, 1, 0, 0]);
    assert_eq!(
        nonzero.sign_with_projected_zero_fallback(&policy).unwrap(),
        Classification::Uncertain(UncertaintyReason::RealSign)
    );
    let zero = int_value(&quadratic_base(2, 8), [0, 2, -1, 0]);
    assert_sign(
        zero.sign_with_projected_zero_fallback(&policy),
        RealSign::Zero,
    );
}

#[test]
fn sign_at_projected_zero_replays_the_authored_sheet() {
    let policy = TestContext::strict();
    let (_, zero) = hidden_extension_zero();
    let sources = vec![cube_root_two()];
    let mut component = |value: &RecursiveQuadraticValue| value.sign(&policy);
    assert_sign(
        zero.sign_at_projected_zero(&sources, &policy, &mut component),
        RealSign::Zero,
    );
    // The conjugate sheet -c - sqrt(c^2) is nonzero: equal component signs.
    let (extension, _) = hidden_extension_zero();
    let field = extension.base_and_extension_path().0;
    let base = RecursiveQuadraticField::Base(field);
    let c = cube(&base);
    let conjugate = extension
        .element(
            c.scale(&real(-1)).unwrap(),
            base.constant(real(-1)).unwrap(),
        )
        .unwrap();
    assert_sign(
        conjugate.sign_at_projected_zero(&sources, &policy, &mut component),
        RealSign::Negative,
    );
    // An undecided component keeps the replay undecided.
    let mut undecided = |_: &RecursiveQuadraticValue| -> Result<_, TestError> {
        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
    };
    assert_eq!(
        zero.sign_at_projected_zero(&sources, &policy, &mut undecided)
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Predicate)
    );
}

#[test]
fn exact_value_sources_carry_their_witness() {
    // A representation passed through `from_exact_value` keeps its witness.
    let point = AlgebraicRootRepresentation::from_exact_value(&root_of(5));
    let field = RecursiveQuadraticField::base(
        vec![point.clone()],
        constant_tensor(1, real(1)),
        constant_tensor(1, real(1)),
    )
    .unwrap();
    let axis = field.retained_root_value(&point).unwrap();
    assert_value(&axis, &root_of(5));
    assert_sign(
        axis.subtract(&field.constant(fraction(9, 4)).unwrap())
            .unwrap()
            .sign(&TestContext::strict()),
        RealSign::Negative,
    );
}

/// `u` as a constant value: zero, but undecidable under STRICT.
fn undecidable_value(field: &RecursiveQuadraticField) -> RecursiveQuadraticValue {
    field.constant(undecidable_zero()).unwrap()
}

#[test]
fn undecidable_values_stay_undecided_unless_approximation_is_consumed() {
    let field = quadratic_base(2, 3);
    let value = undecidable_value(&field);
    let strict = TestContext::strict();
    assert!(value.sign(&strict).unwrap().is_uncertain());
    assert_eq!(
        strict
            .bounded_exact_predicate_pass(|| value.sign(&strict))
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Predicate)
    );
    // u is zero, so the approximate terminal's answer is correct, and its
    // consumption is recorded rather than silently certified.
    let approximate = TestContext::approximate();
    assert_sign(value.sign(&approximate), RealSign::Zero);
    assert!(approximate.approximations() > 0);
    // An extension value u + u*sqrt(2) stays undecided at its retained part.
    let extension = field.extension(field.constant(real(2)).unwrap()).unwrap();
    let lifted = extension.element(value.clone(), value).unwrap();
    assert!(lifted.sign(&strict).unwrap().is_uncertain());
}

#[test]
fn structural_zero_coefficients_are_recognized_after_interval_failure() {
    let field = quadratic_base(2, 3);
    let value = base_value(&field, [denested_zero(), real(0), real(0), real(0)]);
    assert!(!value.is_coefficientwise_stored_zero());
    assert!(value.is_structurally_zero());
    assert_sign(value.sign(&TestContext::strict()), RealSign::Zero);
}

#[test]
fn extension_signs_combine_exactly_signed_components() {
    let strict = TestContext::strict();
    let field = field_with_dependent_speeds();
    let zero = hidden_zero(&field);
    let extension = field.extension(field.constant(real(2)).unwrap()).unwrap();
    // 0 + 0*sqrt2 with hidden zeros in both parts.
    let both = extension.element(zero.clone(), zero.clone()).unwrap();
    assert_sign(both.sign(&strict), RealSign::Zero);
    // 0 + u*sqrt2: the radical part is undecidable.
    let undecided = extension.element(zero, undecidable_value(&field)).unwrap();
    assert!(undecided.sign(&strict).unwrap().is_uncertain());
    // -c + sqrt(c^2 + u) = 0, but its magnitude -u is undecidable.
    let cube_field = cube_field(&[real(1)], &[real(1)]);
    let c = cube(&cube_field);
    let radicand = c
        .square()
        .unwrap()
        .add(&undecidable_value(&cube_field))
        .unwrap();
    let extension = cube_field.extension(radicand).unwrap();
    let value = extension
        .element(
            c.scale(&real(-1)).unwrap(),
            cube_field.constant(real(1)).unwrap(),
        )
        .unwrap();
    assert!(value.sign(&strict).unwrap().is_uncertain());
}

#[test]
fn sign_of_values_needing_deeper_refinement_over_two_axes() {
    let field = RecursiveQuadraticField::base(
        vec![cube_root_two(), cube_root_three()],
        constant_tensor(2, real(1)),
        constant_tensor(2, real(1)),
    )
    .unwrap();
    let c = field.retained_root_value(&cube_root_two()).unwrap();
    // 2^(1/3) - 1.2599210 ~ 5e-8: separated only after deeper refinement.
    let offset = fraction(12_599_210, 10_000_000);
    let expected = strict_sign(&(cube_root_two_real() - &offset)).unwrap();
    let value = c.subtract(&field.constant(offset).unwrap()).unwrap();
    assert_eq!(value.bounded_interval_sign(0..=16), None);
    // A bounded pass over two unwitnessed axes may decline.
    let strict = TestContext::strict();
    assert_sign_or_undecided(
        strict.bounded_exact_predicate_pass(|| value.sign(&strict)),
        expected,
    );
    assert_sign(value.sign(&strict), expected);
    // Later bounded passes, which may reuse the tuple's retained deeper
    // refinement, still agree with the certified sign.
    assert_sign_or_undecided(
        strict.bounded_exact_predicate_pass(|| value.sign(&strict)),
        expected,
    );
}

#[test]
fn affine_positive_root_signs_of_undecidable_terms() {
    let strict = TestContext::strict();
    let rank_zero = quadratic_base(2, 3);
    let u = undecidable_value(&rank_zero);
    let two = rank_zero.constant(real(2)).unwrap();
    // u + u*sqrt2: the retained term is undecidable.
    assert!(
        RecursiveQuadraticValue::affine_positive_root_sign(&u, &u, &two, None, None, &strict)
            .unwrap()
            .is_uncertain()
    );
    // 0 + u*sqrt2 over the cube-root tuple: the radical term is undecidable.
    let field = field_with_dependent_speeds();
    let zero = hidden_zero(&field);
    let u = undecidable_value(&field);
    let two = field.constant(real(2)).unwrap();
    assert!(
        RecursiveQuadraticValue::affine_positive_root_sign(&zero, &u, &two, None, None, &strict)
            .unwrap()
            .is_uncertain()
    );
    // -c + sqrt(c^2 + u): opposite separated terms, undecidable magnitude.
    let cube_field = cube_field(&[real(1)], &[real(1)]);
    let c = cube(&cube_field);
    let minus_c = c.scale(&real(-1)).unwrap();
    let one = cube_field.constant(real(1)).unwrap();
    let radicand = c
        .square()
        .unwrap()
        .add(&undecidable_value(&cube_field))
        .unwrap();
    assert!(
        RecursiveQuadraticValue::affine_positive_root_sign(
            &minus_c, &one, &radicand, None, None, &strict,
        )
        .unwrap()
        .is_uncertain()
    );
    let approximate = TestContext::approximate();
    assert_sign(
        RecursiveQuadraticValue::affine_positive_root_sign(
            &minus_c,
            &one,
            &radicand,
            None,
            None,
            &approximate,
        ),
        RealSign::Zero,
    );
    assert!(approximate.approximations() > 0);
    // A bounded pass over witnessed transverse terms still decides.
    let rational = quadratic_base(2, 3);
    let sign = strict
        .bounded_exact_predicate_pass(|| {
            RecursiveQuadraticValue::affine_positive_root_sign(
                &int_value(&rational, [-1, 0, 0, 0]),
                &int_value(&rational, [1, 0, 0, 0]),
                &int_value(&rational, [3, 0, 0, 0]),
                None,
                None,
                &strict,
            )
        })
        .unwrap();
    assert_eq!(sign, Classification::Decided(RealSign::Positive));
}

#[test]
fn nested_positive_root_affine_signs_of_undecidable_terms() {
    let strict = TestContext::strict();
    let field = quadratic_base(2, 3);
    let u = undecidable_value(&field);
    let two = field.constant(real(2)).unwrap();
    // sqrt2 * (u + u*sqrt2) + u is zero; STRICT cannot decide it.
    assert!(
        RecursiveQuadraticValue::nested_positive_root_affine_sign(
            &u, &u, &two, &u, &two, None, None, None, &strict,
        )
        .unwrap()
        .is_uncertain()
    );
    let approximate = TestContext::approximate();
    assert_sign(
        RecursiveQuadraticValue::nested_positive_root_affine_sign(
            &u,
            &u,
            &two,
            &u,
            &two,
            None,
            None,
            None,
            &approximate,
        ),
        RealSign::Zero,
    );
    assert!(approximate.approximations() > 0);
    // A witnessed transverse value decides inside a bounded pass.
    let one = field.constant(real(1)).unwrap();
    let sign = strict
        .bounded_exact_predicate_pass(|| {
            RecursiveQuadraticValue::nested_positive_root_affine_sign(
                &one, &one, &two, &one, &two, None, None, None, &strict,
            )
        })
        .unwrap();
    assert_eq!(sign, Classification::Decided(RealSign::Positive));
    // Over the cube-root tuple: c + sqrt2 > 0 with an undecidable constant.
    let cube_field = cube_field(&[real(1)], &[real(1)]);
    let c = cube(&cube_field);
    assert!(
        RecursiveQuadraticValue::nested_positive_root_affine_sign(
            &c,
            &cube_field.constant(real(1)).unwrap(),
            &cube_field.constant(real(2)).unwrap(),
            &undecidable_value(&cube_field),
            &cube_field.constant(real(1)).unwrap(),
            None,
            None,
            None,
            &strict,
        )
        .unwrap()
        .is_uncertain()
    );
}

#[test]
fn nested_positive_root_affine_signs_of_exact_cancellations() {
    let strict = TestContext::strict();
    // Speeds 2 and 3 over c: sqrt(1) * (c + 1*sqrt2) - (c + sqrt2) = 0.
    let field = cube_field(&[real(2)], &[real(3)]);
    let c = cube(&field);
    let minus_sum = axis_value(
        &field,
        &[real(0), real(-1)],
        &[real(-1)],
        &[real(0)],
        &[real(0)],
    )
    .unwrap();
    assert_sign(
        RecursiveQuadraticValue::nested_positive_root_affine_sign(
            &c,
            &field.constant(real(1)).unwrap(),
            &field.constant(real(2)).unwrap(),
            &minus_sum,
            &field.constant(real(1)).unwrap(),
            None,
            None,
            None,
            &strict,
        ),
        RealSign::Zero,
    );
}

#[test]
fn projected_zero_replay_follows_magnitudes_down_the_tower() {
    let policy = TestContext::strict();
    // E = F(sqrt(c^2)): g = c on the authored sheet and -c on its conjugate.
    let field = cube_field(&[real(1)], &[real(1)]);
    let c = cube(&field);
    let extension = field.extension(c.square().unwrap()).unwrap();
    let g = generator(&extension);
    let lifted_c = extension.lift(&c).unwrap();
    let g_plus_c = g.add(&lifted_c).unwrap();
    let constant = |value: i64| extension.constant(real(value)).unwrap();
    let c_real = cube_root_two_real();
    let sources = vec![cube_root_two()];
    let mut component = |value: &RecursiveQuadraticValue| value.sign(&policy);
    // With s = a^2 -+ (g + c), a^2 - s = +-(g + c) vanishes on the conjugate
    // sheet, so the full norm of a - sqrt(s) is zero while its authored
    // value is not.
    for (a, s, expected) in [
        // 2 - sqrt(4 - 2c) > 0: opposite terms, positive magnitude 2c.
        (
            2,
            constant(4).subtract(&g_plus_c).unwrap(),
            real(2) - sqrt(real(4) - real(2) * &c_real),
        ),
        // 1 - sqrt(1 + 2c) < 0: opposite terms, negative magnitude -2c.
        (
            1,
            constant(1).add(&g_plus_c).unwrap(),
            real(1) - sqrt(real(1) + real(2) * &c_real),
        ),
    ] {
        let outer = extension.extension(s).unwrap();
        let value = outer.element(constant(a), constant(-1)).unwrap();
        let expected = strict_sign(&expected).unwrap();
        assert_sign(
            value.sign_at_projected_zero(&sources, &policy, &mut component),
            expected,
        );
        assert_sign(value.sign(&policy), expected);
    }
    // Both components zero: z + z*sqrt(2) with z = g - c.
    let z = g.subtract(&lifted_c).unwrap();
    let outer = extension.extension(constant(2)).unwrap();
    let both = outer.element(z.clone(), z).unwrap();
    assert_sign(
        both.sign_at_projected_zero(&sources, &policy, &mut component),
        RealSign::Zero,
    );
    // Inside a bounded pass the components fall back to their own
    // projected-zero certificates.
    let bounded = policy
        .bounded_exact_predicate_pass(|| both.sign_with_projected_zero_fallback(&policy))
        .unwrap();
    assert_eq!(bounded, Classification::Decided(RealSign::Zero));
    // An undecided radical component keeps the replay undecided.
    let mut calls = 0;
    let mut second_undecided = |value: &RecursiveQuadraticValue| {
        calls += 1;
        if calls == 1 {
            value.sign(&policy)
        } else {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
    };
    let (_, zero) = hidden_extension_zero();
    assert_eq!(
        zero.sign_at_projected_zero(&sources, &policy, &mut second_undecided)
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Predicate)
    );
}
