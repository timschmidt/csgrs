//! Exact values, intervals, ratios and tensor images built from represented
//! algebraic roots.
//!
//! Every decided construction is checked against an independently derived
//! closed form: exact replay of the defining polynomial at that form,
//! certified containment in the isolator, and a test-local Sturm proof that
//! the isolator holds one root. Uncertain outcomes use inputs whose blocker
//! is known by construction.

#[path = "support/represented_roots.rs"]
mod support;

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};
use hypersolve::classification::{Classification, UncertaintyReason};
use hypersolve::work_budget::{self, WorkBudget, WorkKind};
use hypersolve::*;
use support::*;

/// The minimal polynomial `x^3 - 2` of `cbrt(2)`.
const CBRT2: &[i64] = &[-2, 0, 0, 1];

// Univariate coordinates.

#[test]
fn univariate_coordinates_select_the_strictly_monotone_root() {
    let mut provenance = sqrt2();
    provenance.constraint_index = 7;
    provenance.symbol = SymbolId(3);
    provenance.interval_index = 2;

    let positive = decided(represented_univariate_coordinate(
        &ints(&[-2, 0, 1]),
        &int(1),
        &int(2),
        &provenance,
    ));
    assert_selects(&positive, &sqrt(2));
    assert_eq!(positive.constraint_index, 7);
    assert_eq!(positive.symbol, SymbolId(3));
    assert_eq!(positive.interval_index, 2);

    let negative = decided(represented_univariate_coordinate(
        &ints(&[-2, 0, 1]),
        &int(-2),
        &int(-1),
        &provenance,
    ));
    assert_selects(&negative, &-sqrt(2));
}

#[test]
fn univariate_coordinates_reject_intervals_without_a_root() {
    // x^2 - 2 is strictly increasing on [2, 3] but positive at both ends,
    // so the interval holds no root; publishing one would be a wrong answer.
    let provenance = sqrt2();
    assert_eq!(
        uncertain(represented_univariate_coordinate(
            &ints(&[-2, 0, 1]),
            &int(2),
            &int(3),
            &provenance,
        )),
        UncertaintyReason::Unsupported
    );
    // A root at an endpoint still exists: x^2 - 4 on [2, 3] selects 2.
    let endpoint = decided(represented_univariate_coordinate(
        &ints(&[-4, 0, 1]),
        &int(2),
        &int(3),
        &provenance,
    ));
    assert_selects(&endpoint, &int(2));
}

#[test]
fn univariate_point_coordinates_replay_the_polynomial_exactly() {
    let provenance = sqrt2();
    let point = decided(represented_univariate_coordinate(
        &ints(&[-2, 0, 1]),
        &sqrt(2),
        &sqrt(2),
        &provenance,
    ));
    assert_selects(&point, &sqrt(2));
    assert_equal(point.exact_point_witness().unwrap(), &sqrt(2));

    // 3/2 is not a root of x^2 - 2: (3/2)^2 - 2 = 1/4.
    assert_eq!(
        uncertain(represented_univariate_coordinate(
            &ints(&[-2, 0, 1]),
            &q(3, 2),
            &q(3, 2),
            &provenance,
        )),
        UncertaintyReason::Predicate
    );
}

#[test]
fn univariate_coordinates_reject_constant_reversed_and_non_monotone_inputs() {
    let provenance = sqrt2();
    for coefficients in [vec![], ints(&[5])] {
        assert_eq!(
            uncertain(represented_univariate_coordinate(
                &coefficients,
                &int(1),
                &int(2),
                &provenance,
            )),
            UncertaintyReason::Unsupported
        );
    }
    assert_eq!(
        uncertain(represented_univariate_coordinate(
            &ints(&[-2, 0, 1]),
            &int(2),
            &int(1),
            &provenance,
        )),
        UncertaintyReason::Predicate
    );
    // [-2, 2] holds both square roots of two; the derivative 2x changes sign.
    assert_eq!(
        uncertain(represented_univariate_coordinate(
            &ints(&[-2, 0, 1]),
            &int(-2),
            &int(2),
            &provenance,
        )),
        UncertaintyReason::Predicate
    );
    // A stored zero leading coefficient is a malformed defining polynomial,
    // even though the remaining terms are monotone on [1, 2].
    assert_eq!(
        uncertain(represented_univariate_coordinate(
            &ints(&[-2, 0, 1, 0]),
            &int(1),
            &int(2),
            &provenance,
        )),
        UncertaintyReason::Unsupported
    );
}

// Refinement.

#[test]
fn refined_roots_shrink_their_isolator_around_the_same_root() {
    let source = cbrt2();
    assert_eq!(refined_represented_root(&source, 0), source);
    let point = exact(&int(3));
    assert_eq!(refined_represented_root(&point, 16), point);

    let refined = refined_represented_root(&source, 12);
    assert_selects_root_of(&refined, &cbrt(2), CBRT2);
    let width = rational(&refined.interval.upper) - rational(&refined.interval.lower);
    assert!(width < hyperreal::Rational::fraction(1, 64).unwrap());
    // Cubes of the endpoints bracket two.
    let lower = rational(&refined.interval.lower);
    let upper = rational(&refined.interval.upper);
    assert!(&(&lower * &lower) * &lower < hyperreal::Rational::new(2));
    assert!(&(&upper * &upper) * &upper > hyperreal::Rational::new(2));
}

#[test]
fn exhausted_refinement_budgets_return_the_unrefined_root() {
    let source = cbrt2();
    let mut results = Vec::new();
    let outcome = work_budget::with_budget(WorkBudget::ZERO, || {
        results.push(refined_represented_root(&source, 8));
        results.push(refined_represented_root(&source, 8));
    });
    let error = outcome.result.unwrap_err();
    assert_eq!(error.kind, WorkKind::RootRefinement);
    assert_eq!(results, vec![source.clone(), source.clone()]);
}

#[test]
fn incremental_refinement_accumulates_steps_and_retries_after_no_progress() {
    let source = cbrt2();
    let mut refinement = RepresentedRootRefinement::new(&source);
    assert_eq!(refinement.refine_to(0), &source);

    let first = refinement.refine_to(8).clone();
    assert_selects_root_of(&first, &cbrt(2), CBRT2);
    assert_ne!(first, source);
    // A target below the completed count performs no work.
    assert_eq!(refinement.refine_to(4), &first);
    let second = refinement.refine_to(16).clone();
    assert_selects_root_of(&second, &cbrt(2), CBRT2);
    let first_width = rational(&first.interval.upper) - rational(&first.interval.lower);
    let second_width = rational(&second.interval.upper) - rational(&second.interval.lower);
    assert!(second_width < first_width);

    // No budgeted progress keeps the completed count, so a later request retries.
    let mut blocked = RepresentedRootRefinement::new(&source);
    let mut during = None;
    let outcome = work_budget::with_budget(WorkBudget::ZERO, || {
        during = Some(blocked.refine_to(8).clone());
    });
    assert!(outcome.result.is_err());
    assert_eq!(during.unwrap(), source);
    let retried = blocked.refine_to(8).clone();
    assert_ne!(retried, source);
    assert_selects_root_of(&retried, &cbrt(2), CBRT2);
}

#[test]
fn incremental_refinement_finds_exact_dyadic_roots_and_handles_endpoint_roots() {
    // 4x^2 - 1 has the root 1/2, the first bisection midpoint of [0, 1].
    let half = root(&[-1, 0, 4], int(0), int(1));
    let mut refinement = RepresentedRootRefinement::new(&half);
    let refined = refinement.refine_to(4).clone();
    assert_selects(&refined, &q(1, 2));
    assert_equal(refined.exact_point_witness().unwrap(), &q(1, 2));
    // Refining an exact point leaves it unchanged.
    assert_eq!(refinement.refine_to(8), &refined);

    // x - 2 on [1, 2] owns its root at the included upper endpoint, so the
    // endpoint signs are not opposite and the Sturm refinement is used.
    let endpoint = root(&[-2, 1], int(1), int(2));
    let mut refinement = RepresentedRootRefinement::new(&endpoint);
    assert_selects(refinement.refine_to(4), &int(2));

    // (2x - 3)^2 has equal endpoint signs on [1, 2].
    let double = root(&[9, -12, 4], int(1), int(2));
    let mut refinement = RepresentedRootRefinement::new(&double);
    let refined = refinement.refine_to(4).clone();
    assert_exact_zero(&Real::eval_poly(&refined.polynomial_coefficients, &q(3, 2)));
    assert_encloses(
        &RealInterval {
            lower: refined.interval.lower.clone(),
            upper: refined.interval.upper.clone(),
        },
        &q(3, 2),
    );
}

#[test]
fn open_interval_scalars_lie_strictly_inside() {
    assert_equal(&scalar_in_open_interval(&int(1), &int(2)), &q(3, 2));
    for (lower, upper) in [
        (sqrt(2), sqrt(3)),
        (int(1), sqrt(2)),
        (sqrt(5), int(3)),
        (cbrt(2), sqrt(2)),
        // Endpoints 2^-20 apart need finer inner dyadic bounds.
        (sqrt(2), sqrt(2) + q(1, 1 << 20)),
    ] {
        let inside = scalar_in_open_interval(&lower, &upper);
        assert!(inside.exact_rational_normal_form().is_some());
        assert_strictly_between(&inside, &lower, &upper);
    }
}

// Dense tensor enclosures.

/// `x*y + 1` with an appended unit output axis.
fn xy_plus_one() -> DenseTensorPolynomial {
    tensor(&[2, 2, 1], ints(&[1, 0, 0, 1]))
}

#[test]
fn dense_tensor_intervals_enclose_the_value_at_the_selected_tuple() {
    let sources = [sqrt2(), sqrt3()];
    let value = sqrt(6) + int(1);
    let interval = dense_tensor_interval(&xy_plus_one(), &sources).unwrap();
    assert_encloses(&interval, &value);

    // Enclosures follow the supplied isolators as they narrow.
    let refined = sources
        .iter()
        .map(|source| refined_represented_root(source, 24))
        .collect::<Vec<_>>();
    let narrow = dense_tensor_interval(&xy_plus_one(), &refined).unwrap();
    assert_encloses(&narrow, &value);
    // sqrt(6) + 1 = 3.44948974...
    assert_strictly_between(&narrow.lower, &q(34494, 10000), &value);
    assert_strictly_between(&narrow.upper, &value, &q(34495, 10000));
}

#[test]
fn dense_tensor_intervals_round_coefficients_and_witnesses_outward() {
    let sources = [sqrt2()];
    // pi * x at sqrt(2).
    let polynomial = tensor(&[2, 1], vec![Real::zero(), Real::pi()]);
    let value = Real::pi() * sqrt(2);
    let rounded =
        dense_tensor_interval_with_coefficient_precision(&polynomial, &sources, Some(-40)).unwrap();
    assert_encloses(&rounded, &value);
    assert!(rounded.lower.exact_rational_normal_form().is_some());
    assert!(rounded.upper.exact_rational_normal_form().is_some());

    let witnesses = [Some(sqrt(2))];
    let point = dense_tensor_interval_with_coefficient_precision_and_source_witnesses(
        &polynomial,
        &sources,
        Some(&witnesses),
        Some(-40),
    )
    .unwrap();
    assert_encloses(&point, &value);
    // The witness replaces the [1, 2] isolator, so pi*sqrt(2) = 4.44288...
    // is bounded far more tightly than pi*[1, 2].
    assert_strictly_between(&point.lower, &q(4442, 1000), &value);
    assert_strictly_between(&point.upper, &value, &q(4443, 1000));

    let exact_witness = dense_tensor_interval_with_coefficient_precision_and_source_witnesses(
        &xy_plus_one(),
        &[sqrt2(), sqrt3()],
        Some(&[Some(sqrt(2)), None]),
        None,
    )
    .unwrap();
    // sqrt(2)*[1, 2] + 1.
    assert_equal(&exact_witness.lower, &(sqrt(2) + int(1)));
    assert_equal(&exact_witness.upper, &(int(2) * sqrt(2) + int(1)));
}

#[test]
fn dense_tensor_intervals_reject_mismatched_shapes_and_keep_stored_zeroes() {
    let sources = [sqrt2(), sqrt3()];
    // Missing output axis, wrong output degree, and too few sources.
    assert!(dense_tensor_interval(&tensor(&[2, 2], ints(&[1, 0, 0, 1])), &sources).is_none());
    assert!(dense_tensor_interval(&tensor(&[2, 2, 2], ints(&[0; 8])), &sources).is_none());
    assert!(dense_tensor_interval(&xy_plus_one(), &sources[..1]).is_none());
    assert!(
        dense_tensor_interval_with_coefficient_precision_and_source_witnesses(
            &xy_plus_one(),
            &sources,
            Some(&[None]),
            None,
        )
        .is_none()
    );

    let zero = dense_tensor_interval(&tensor(&[2, 2, 1], ints(&[0; 4])), &sources).unwrap();
    assert_exact_zero(&zero.lower);
    assert_exact_zero(&zero.upper);

    // A zero coefficient beside a nonzero one under coefficient rounding.
    let sparse = tensor(&[2, 2, 1], ints(&[0, 0, 0, 3]));
    let interval =
        dense_tensor_interval_with_coefficient_precision(&sparse, &sources, Some(-32)).unwrap();
    assert_encloses(&interval, &(int(3) * sqrt(6)));
}

#[test]
fn dense_polynomial_value_intervals_append_the_output_axis() {
    let sources = [sqrt2()];
    // x^2 - 3 at sqrt(2) is -1.
    let polynomial = tensor(&[3], ints(&[-3, 0, 1]));
    let interval = dense_polynomial_value_interval(&polynomial, &sources).unwrap();
    assert_encloses(&interval, &int(-1));
    let rounded =
        dense_polynomial_value_interval_with_coefficient_precision(&polynomial, &sources, -32)
            .unwrap();
    assert_encloses(&rounded, &int(-1));
    assert!(
        dense_polynomial_value_interval(&tensor(&[3, 1], ints(&[-3, 0, 1])), &sources).is_none()
    );
}

#[test]
fn positive_square_root_intervals_enclose_the_authored_sheet() {
    let sources = [sqrt2()];
    let x = on_axis(1, 0, ints(&[0, 1]));
    // x + sqrt(3) at sqrt(2).
    let interval = dense_positive_square_root_interval(
        &x,
        &constant(1, int(1)),
        &constant(1, int(3)),
        &sources,
    )
    .unwrap();
    assert_encloses(&interval, &(sqrt(2) + sqrt(3)));
    // A negative radicand has no real square root.
    assert!(
        dense_positive_square_root_interval(
            &x,
            &constant(1, int(1)),
            &constant(1, int(-1)),
            &sources
        )
        .is_none()
    );
}

#[test]
fn two_square_root_intervals_enclose_every_radical_term() {
    let sources = [sqrt2()];
    let x = on_axis(1, 0, ints(&[0, 1]));
    let x_squared = on_axis(1, 0, ints(&[0, 0, 1]));
    // 1 + x*sqrt(3) + 0*sqrt(x^2) + 1*sqrt(3)*sqrt(x^2) = 1 + 2*sqrt(6).
    let expression = radical_expression::TwoSquareRootExpression {
        rational: constant(1, int(1)),
        first: x.clone(),
        second: constant(1, Real::zero()),
        product: constant(1, int(1)),
    };
    let value = int(1) + int(2) * sqrt(6);
    let interval = dense_two_positive_square_root_interval(
        &expression,
        &constant(1, int(3)),
        &x_squared,
        &sources,
    )
    .unwrap();
    assert_encloses(&interval, &value);

    let refined = [refined_represented_root(&sources[0], 20)];
    let witnesses = [None];
    let rounded = dense_two_positive_square_root_interval_with_coefficient_precision(
        &expression,
        &constant(1, int(3)),
        &x_squared,
        &refined,
        Some(&witnesses),
        Some(-48),
    )
    .unwrap();
    assert_encloses(&rounded, &value);
    // 1 + 2*sqrt(6) = 5.89897948...
    assert_strictly_between(&rounded.lower, &q(5898, 1000), &value);
    assert_strictly_between(&rounded.upper, &value, &q(5899, 1000));

    assert!(
        dense_two_positive_square_root_interval(
            &expression,
            &constant(1, int(-3)),
            &x_squared,
            &sources,
        )
        .is_none()
    );
}

#[test]
fn nested_intervals_combine_the_retained_and_radical_parts() {
    let sources = [sqrt3()];
    let radical = sqrt2();
    let retained = tensor(&[2, 1], ints(&[0, 1]));
    let candidate = tensor(&[1, 1], ints(&[1]));
    let interval =
        represented_tensor_nested_interval(&retained, &candidate, &sources, &radical).unwrap();
    assert_encloses(&interval, &(sqrt(3) + sqrt(2)));
    assert!(
        represented_tensor_nested_interval(
            &tensor(&[2], ints(&[0, 1])),
            &candidate,
            &sources,
            &radical
        )
        .is_none()
    );
}

// Orders, signs and equality.

#[test]
fn strict_orders_and_equality_follow_the_selected_values() {
    let (s2, s3) = (sqrt2(), sqrt3());
    assert_eq!(represented_strict_order(&s2, &s3), Some(Ordering::Less));
    assert_eq!(represented_strict_order(&s3, &s2), Some(Ordering::Greater));
    assert_eq!(represented_strict_order(&s2, &s2), Some(Ordering::Equal));
    assert_eq!(
        represented_strict_order(&s2, &exact(&sqrt(2))),
        Some(Ordering::Equal)
    );
    assert_eq!(
        represented_strict_order(&cbrt2(), &exact(&q(5, 4))),
        Some(Ordering::Greater)
    );

    assert!(represented_roots_strictly_equal(&s2, &sqrt2_from_cubic()));
    assert!(represented_roots_strictly_equal(&s2, &exact(&sqrt(2))));
    assert!(!represented_roots_strictly_equal(&s2, &s3));
    assert!(!represented_roots_strictly_equal(&s2, &negative_sqrt2()));
}

#[test]
fn strict_signs_use_the_isolator_then_a_certified_comparison() {
    assert_eq!(represented_strict_sign(&cbrt2()), Some(RealSign::Positive));
    assert_eq!(
        represented_strict_sign(&negative_sqrt2()),
        Some(RealSign::Negative)
    );
    assert_eq!(
        represented_strict_sign(&exact(&Real::zero())),
        Some(RealSign::Zero)
    );
    // x^3 + x - 1 and x^3 + x + 1 are increasing; their roots are about
    // +0.6823 and -0.6823, inside isolators that straddle zero.
    assert_eq!(
        represented_strict_sign(&root(&[-1, 1, 0, 1], int(-1), int(1))),
        Some(RealSign::Positive)
    );
    assert_eq!(
        represented_strict_sign(&root(&[1, 1, 0, 1], int(-1), int(1))),
        Some(RealSign::Negative)
    );
}

#[test]
fn undecidable_exact_zeroes_have_no_strict_order_or_sign() {
    let value = exact(&unknown_zero());
    assert_eq!(
        represented_strict_order(&value, &exact(&Real::zero())),
        None
    );
    assert_eq!(represented_strict_sign(&value), None);
    // `false` means no STRICT equality proof, not a proof of inequality.
    assert!(!represented_roots_strictly_equal(
        &value,
        &exact(&Real::zero())
    ));
}

#[test]
fn unit_scales_certify_identity_and_reflection() {
    let both = POSITIVE_UNIT_SCALE | NEGATIVE_UNIT_SCALE;
    // Exact witnesses.
    assert_eq!(
        represented_zero_offset_unit_scales(&exact(&int(2)), &exact(&int(2))),
        POSITIVE_UNIT_SCALE
    );
    assert_eq!(
        represented_zero_offset_unit_scales(&exact(&int(2)), &exact(&int(-2))),
        NEGATIVE_UNIT_SCALE
    );
    assert_eq!(
        represented_zero_offset_unit_scales(&exact(&Real::zero()), &exact(&Real::zero())),
        both
    );
    assert_eq!(
        represented_zero_offset_unit_scales(&exact(&int(2)), &exact(&int(3))),
        0
    );

    // Structural identity, proportional polynomials and reflections.
    let s2 = sqrt2();
    assert_eq!(
        represented_zero_offset_unit_scales(&s2, &s2),
        POSITIVE_UNIT_SCALE
    );
    assert_eq!(
        represented_zero_offset_unit_scales(&s2, &root(&[-4, 0, 2], int(1), int(2))),
        POSITIVE_UNIT_SCALE
    );
    assert_eq!(
        represented_zero_offset_unit_scales(&s2, &negative_sqrt2()),
        NEGATIVE_UNIT_SCALE
    );
    // x^3 + 2 on [-2, -1] selects -cbrt(2); odd powers change sign.
    assert_eq!(
        represented_zero_offset_unit_scales(&cbrt2(), &root(&[2, 0, 0, 1], int(-2), int(-1))),
        NEGATIVE_UNIT_SCALE
    );

    // Different defining polynomials require a certified equality proof.
    assert_eq!(
        represented_zero_offset_unit_scales(&s2, &sqrt2_from_cubic()),
        POSITIVE_UNIT_SCALE
    );
    assert_eq!(
        represented_zero_offset_unit_scales(&s2, &root(&[6, -2, -3, 1], int(-2), int(-1))),
        NEGATIVE_UNIT_SCALE
    );
    assert_eq!(
        represented_zero_offset_unit_scales(&s2, &root(&[-2, 0, 1], int(1), q(3, 2))),
        POSITIVE_UNIT_SCALE
    );
    assert_eq!(represented_zero_offset_unit_scales(&s2, &sqrt3()), 0);
    assert_eq!(
        represented_zero_offset_unit_scales(&exact(&sqrt(2)), &s2),
        POSITIVE_UNIT_SCALE
    );
}

// Affine images.

#[test]
fn affine_images_of_exact_points_stay_exact() {
    let two = exact(&int(2));
    let radical = exact(&sqrt(2));
    let image = decided(represented_affine_coordinate(
        &[(&two, &int(3)), (&radical, &int(1))],
        &int(1),
    ));
    assert_equal(image.exact_point_witness().unwrap(), &(int(7) + sqrt(2)));

    // A zero scale removes the term entirely; no terms leave the offset.
    let image = decided(represented_affine_coordinate(
        &[(&sqrt3(), &Real::zero())],
        &int(5),
    ));
    assert_equal(image.exact_point_witness().unwrap(), &int(5));
    let image = decided(represented_affine_coordinate(&[], &sqrt(2)));
    assert_equal(image.exact_point_witness().unwrap(), &sqrt(2));
}

#[test]
fn affine_images_of_one_selected_root() {
    let s2 = sqrt2();
    assert_eq!(
        decided(represented_affine_coordinate(
            &[(&s2, &int(1))],
            &Real::zero()
        )),
        s2
    );
    let image = decided(represented_affine_coordinate(&[(&s2, &int(2))], &int(1)));
    assert_selects(&image, &(int(2) * sqrt(2) + int(1)));
    let image = decided(represented_affine_coordinate(
        &[(&s2, &int(-1))],
        &Real::zero(),
    ));
    assert_selects(&image, &-sqrt(2));
    let image = decided(represented_affine_coordinate(
        &[(&cbrt2(), &q(-1, 3))],
        &int(2),
    ));
    // x = 2 - c/3 with c^3 = 2: 27(2 - x)^3 = 2.
    assert_selects_root_of(
        &image,
        &(int(2) - cbrt(2) * q(1, 3)),
        &[214, -324, 162, -27],
    );
}

#[test]
fn affine_images_collapse_related_sources() {
    let s2 = sqrt2();
    let image = decided(represented_affine_coordinate(
        &[(&s2, &int(1)), (&s2, &int(1))],
        &Real::zero(),
    ));
    assert_selects(&image, &(int(2) * sqrt(2)));

    // sqrt(2) - sqrt(2) + 5 cancels to an exact point.
    let image = decided(represented_affine_coordinate(
        &[(&s2, &int(1)), (&s2, &int(-1))],
        &int(5),
    ));
    assert_equal(image.exact_point_witness().unwrap(), &int(5));

    // sqrt(2) + (-sqrt(2)) through a certified affine relation.
    let image = decided(represented_affine_coordinate(
        &[(&s2, &int(1)), (&negative_sqrt2(), &int(1))],
        &int(1),
    ));
    assert_selects(&image, &int(1));

    let image = decided(represented_affine_coordinate(
        &[(&s2, &int(1)), (&sqrt3(), &int(1))],
        &Real::zero(),
    ));
    assert_selects(&image, &(sqrt(2) + sqrt(3)));
}

#[test]
fn affine_images_of_independent_sources_use_a_tensor_image() {
    let image = decided(represented_affine_coordinate(
        &[(&sqrt2(), &int(1)), (&cbrt2(), &int(1))],
        &Real::zero(),
    ));
    // With a^2 = 2 and (x - a)^3 = 2: (x^3 + 6x - 2)^2 = 2(3x^2 + 2)^2.
    assert_selects_root_of(&image, &(sqrt(2) + cbrt(2)), &[-4, -24, 12, -4, -6, 0, 1]);

    // (a - x)^3 = 2: (x^3 + 6x + 2)^2 = 2(3x^2 + 2)^2.
    let image = decided(represented_affine_coordinate(
        &[(&sqrt2(), &int(1)), (&cbrt2(), &int(-1))],
        &Real::zero(),
    ));
    assert_selects_root_of(&image, &(sqrt(2) - cbrt(2)), &[-4, 24, 12, 4, -6, 0, 1]);

    // (x - 2a)^3 = 2: (x^3 + 24x - 2)^2 = 2(6x^2 + 16)^2.
    let image = decided(represented_affine_coordinate(
        &[(&sqrt2(), &int(2)), (&cbrt2(), &int(1))],
        &Real::zero(),
    ));
    assert_selects_root_of(
        &image,
        &(int(2) * sqrt(2) + cbrt(2)),
        &[-508, -96, 192, -4, -24, 0, 1],
    );
}

#[test]
fn affine_images_report_undecided_scales_and_invalid_sources() {
    // The scale is exactly zero but not provably so.
    assert_eq!(
        uncertain(represented_affine_coordinate(
            &[(&sqrt2(), &unknown_zero())],
            &Real::zero()
        )),
        UncertaintyReason::Predicate
    );
    assert_eq!(
        uncertain(represented_affine_coordinate(
            &[(&non_unit_root(), &int(2))],
            &Real::zero()
        )),
        UncertaintyReason::Unsupported
    );
}

#[test]
fn exhausted_budgets_stop_tensor_images_before_publication() {
    let (s2, c2) = (sqrt2(), cbrt2());
    let mut first = None;
    let mut after = None;
    let outcome = work_budget::with_budget(WorkBudget::ZERO, || {
        first = Some(represented_affine_coordinate(
            &[(&s2, &int(1)), (&c2, &int(1))],
            &Real::zero(),
        ));
        // The scope is now exhausted; a new construction stops immediately.
        after = Some(represented_affine_coordinate(
            &[(&s2, &int(1)), (&c2, &int(-1))],
            &Real::zero(),
        ));
    });
    assert_eq!(
        outcome.result.unwrap_err().kind,
        WorkKind::AlgebraicPromotion
    );
    assert_eq!(
        first.unwrap(),
        Classification::Uncertain(UncertaintyReason::BudgetExhausted)
    );
    assert_eq!(
        after.unwrap(),
        Classification::Uncertain(UncertaintyReason::BudgetExhausted)
    );

    let mut ratio = None;
    let mut value = None;
    let outcome = work_budget::with_budget(WorkBudget::ZERO, || {
        ratio = Some(represented_ratio(&c2, &s2));
        value = Some(represented_dense_value_refined(
            &tensor(&[2, 2, 1], ints(&[0, 1, 1, 0])),
            &[s2.clone(), c2.clone()],
        ));
    });
    assert!(outcome.result.is_err());
    assert_eq!(
        ratio.unwrap(),
        Classification::Uncertain(UncertaintyReason::BudgetExhausted)
    );
    assert_eq!(
        value.unwrap(),
        Classification::Uncertain(UncertaintyReason::BudgetExhausted)
    );
}

// Ratios.

#[test]
fn ratios_of_selected_roots() {
    let image = decided(represented_ratio(&sqrt2(), &sqrt3()));
    assert_selects(&image, &(sqrt(2) / sqrt(3)).unwrap());
    let image = decided(represented_ratio(&sqrt2(), &sqrt2()));
    assert_selects(&image, &int(1));
    let image = decided(represented_ratio(&cbrt2(), &sqrt2()));
    // cbrt(2)/sqrt(2) = 2^(-1/6): 2x^6 = 1.
    assert_selects_root_of(
        &image,
        &(cbrt(2) / sqrt(2)).unwrap(),
        &[-1, 0, 0, 0, 0, 0, 2],
    );
    let image = decided(represented_ratio(&exact(&Real::zero()), &sqrt2()));
    assert_selects(&image, &Real::zero());
}

#[test]
fn ratios_with_a_denominator_isolator_containing_zero_are_undecided() {
    // The denominator root is about 0.6823, but its isolator [-1, 1]
    // contains zero, so no quotient enclosure exists.
    let denominator = root(&[-1, 1, 0, 1], int(-1), int(1));
    assert_eq!(
        uncertain(represented_ratio(&sqrt2(), &denominator)),
        UncertaintyReason::Predicate
    );
}

// Dense values.

#[test]
fn dense_values_construct_polynomial_images_of_selected_tuples() {
    let sources = [sqrt2(), sqrt3()];
    // x*y with an output axis.
    let product = tensor(&[2, 2, 1], ints(&[0, 0, 0, 1]));
    let image = decided(represented_dense_value_with_coefficient_precision(
        &product, &sources, -64,
    ));
    assert_selects(&image, &sqrt(6));
    let image = decided(represented_dense_value_refined(&product, &sources));
    assert_selects(&image, &sqrt(6));

    // x^2 + y at (sqrt(2), cbrt(2)) is 2 + cbrt(2).
    let sum = tensor(&[3, 2, 1], ints(&[0, 1, 0, 0, 1, 0]));
    let image = decided(represented_dense_value_refined(&sum, &[sqrt2(), cbrt2()]));
    // (x - 2)^3 = 2.
    assert_selects_root_of(&image, &(int(2) + cbrt(2)), &[-10, 12, -6, 1]);

    let constant = tensor(&[1], vec![q(7, 3)]);
    let image = decided(represented_dense_value_refined(&constant, &[]));
    assert_equal(image.exact_point_witness().unwrap(), &q(7, 3));
}

#[test]
fn dense_values_reject_mismatched_shapes() {
    let sources = [sqrt2(), sqrt3()];
    for polynomial in [
        tensor(&[2, 2], ints(&[0, 0, 0, 1])),
        tensor(&[2, 2, 2], ints(&[0; 8])),
    ] {
        assert_eq!(
            uncertain(represented_dense_value_refined(&polynomial, &sources)),
            UncertaintyReason::Unsupported
        );
        assert_eq!(
            uncertain(represented_dense_value_with_coefficient_precision(
                &polynomial,
                &sources,
                -64
            )),
            UncertaintyReason::Unsupported
        );
    }
}

// Tensor bases and source substitution.

#[test]
fn affine_tensor_bases_share_axes_for_related_coordinates() {
    let s2 = sqrt2();
    let two_s2_plus_one = decided(represented_affine_coordinate(&[(&s2, &int(2))], &int(1)));
    let coordinates = [
        exact(&int(5)),
        s2.clone(),
        two_s2_plus_one,
        sqrt2_from_cubic(),
        sqrt3(),
        s2.clone(),
    ];
    let (sources, polynomials) = represented_affine_tensor_basis(&coordinates).unwrap();
    assert_eq!(sources.len(), 2);
    assert_selects(&sources[0], &sqrt(2));
    assert_selects(&sources[1], &sqrt(3));
    let expected = [
        int(5),
        sqrt(2),
        int(2) * sqrt(2) + int(1),
        sqrt(2),
        sqrt(3),
        sqrt(2),
    ];
    let point = [sqrt(2), sqrt(3), Real::zero()];
    assert_eq!(polynomials.len(), expected.len());
    for (polynomial, expected) in polynomials.iter().zip(&expected) {
        assert_eq!(polynomial.dimensions().len(), 3);
        assert_equal(&evaluate_tensor(polynomial, &point), expected);
    }
}

#[test]
fn affinely_related_tensor_sources_collapse() {
    // x + y over equal, reflected and unrelated source pairs.
    let x_plus_y = tensor(&[2, 2], ints(&[0, 1, 1, 0]));
    let (merged, sources) =
        dense_substitute_affinely_related_sources(x_plus_y.clone(), &[sqrt2(), sqrt2_from_cubic()])
            .unwrap();
    assert_eq!(sources.len(), 1);
    assert_equal(&evaluate_tensor(&merged, &[sqrt(2)]), &(int(2) * sqrt(2)));

    let (merged, sources) =
        dense_substitute_affinely_related_sources(x_plus_y.clone(), &[sqrt2(), negative_sqrt2()])
            .unwrap();
    assert_eq!(sources.len(), 1);
    assert_exact_zero(&evaluate_tensor(&merged, &[sqrt(2)]));

    let (unchanged, sources) =
        dense_substitute_affinely_related_sources(x_plus_y.clone(), &[sqrt2(), cbrt2()]).unwrap();
    assert_eq!(sources.len(), 2);
    assert_eq!(unchanged, x_plus_y);

    assert!(dense_substitute_affinely_related_sources(x_plus_y, &[sqrt2()]).is_none());
}

// Vector products.

#[test]
fn exact_vector_products_are_exact() {
    let first = [exact(&int(1)), exact(&int(2))];
    let second = [exact(&int(3)), exact(&int(4))];
    let [dot, cross] = decided(represented_vector_dot_cross(&first, &second));
    assert_equal(dot.exact_point_witness().unwrap(), &int(11));
    assert_equal(cross.exact_point_witness().unwrap(), &int(-2));
}

#[test]
fn vector_products_with_one_exact_vector_are_affine() {
    let radical = [sqrt2(), sqrt3()];
    let rational = [exact(&int(1)), exact(&int(2))];
    let [dot, cross] = decided(represented_vector_dot_cross(&radical, &rational));
    assert_selects(&dot, &(sqrt(2) + int(2) * sqrt(3)));
    assert_selects(&cross, &(int(2) * sqrt(2) - sqrt(3)));

    let [dot, cross] = decided(represented_vector_dot_cross(&rational, &radical));
    assert_selects(&dot, &(sqrt(2) + int(2) * sqrt(3)));
    assert_selects(&cross, &(sqrt(3) - int(2) * sqrt(2)));
}

#[test]
fn general_vector_products_use_one_tensor_basis() {
    let [dot, cross] = decided(represented_vector_dot_cross(
        &[sqrt2(), sqrt3()],
        &[sqrt3(), sqrt2()],
    ));
    assert_selects(&dot, &(int(2) * sqrt(6)));
    assert_selects(&cross, &int(-1));

    let [dot, cross] = decided(represented_vector_dot_cross(
        &[sqrt2(), cbrt2()],
        &[cbrt2(), sqrt2()],
    ));
    // 2*sqrt(2)*cbrt(2) = 2^(11/6), and 2 - cbrt(4) solves (2 - x)^3 = 4.
    assert_selects_root_of(
        &dot,
        &(int(2) * sqrt(2) * cbrt(2)),
        &[-2048, 0, 0, 0, 0, 0, 1],
    );
    assert_selects_root_of(&cross, &(int(2) - cbrt(2) * cbrt(2)), &[-4, 12, -6, 1]);
}

#[test]
fn vector_products_report_undecided_and_unsupported_components() {
    // The exact second vector (u, 0) has u = 0 exactly, but not provably.
    let undecided = [exact(&unknown_zero()), exact(&Real::zero())];
    assert_eq!(
        uncertain(represented_vector_dot_cross(
            &[sqrt2(), sqrt3()],
            &undecided
        )),
        UncertaintyReason::Predicate
    );
    assert_eq!(
        uncertain(represented_vector_dot_cross(
            &[non_unit_root(), sqrt3()],
            &[exact(&int(1)), exact(&int(2))]
        )),
        UncertaintyReason::Unsupported
    );
}

// Tensor ratios.

#[test]
fn rank_one_tensor_ratios_cancel_common_content() {
    let sources = [sqrt2()];
    // (x + 1)(x - 3) / (x - 3) = x + 1.
    let image = decided(represented_tensor_ratio(
        &tensor(&[3, 1], ints(&[-3, -2, 1])),
        &tensor(&[2, 1], ints(&[-3, 1])),
        &sources,
    ));
    assert_selects(&image, &(sqrt(2) + int(1)));

    // (2x + 2) / (x + 1) = 2.
    let image = decided(represented_tensor_ratio(
        &tensor(&[2, 1], ints(&[2, 2])),
        &tensor(&[2, 1], ints(&[1, 1])),
        &sources,
    ));
    assert_selects(&image, &int(2));

    // (x + 1) / (x - 1) = (sqrt(2) + 1)^2 = 3 + 2*sqrt(2).
    let image = decided(represented_tensor_ratio(
        &tensor(&[2, 1], ints(&[1, 1])),
        &tensor(&[2, 1], ints(&[-1, 1])),
        &sources,
    ));
    assert_selects(&image, &(int(3) + int(2) * sqrt(2)));

    // x^2 / (x + 1) at cbrt(2) needs the general image.
    let image = decided(represented_tensor_ratio(
        &tensor(&[3, 1], ints(&[0, 0, 1])),
        &tensor(&[2, 1], ints(&[1, 1])),
        &[cbrt2()],
    ));
    // x = c^2/(c + 1) with c^3 = 2 eliminates to 3x^3 + 6x^2 - 4 = 0.
    assert_selects_root_of(
        &image,
        &(cbrt(2) * cbrt(2) / (cbrt(2) + int(1))).unwrap(),
        &[-4, 0, 6, 3],
    );
}

#[test]
fn tensor_ratios_with_a_vanishing_denominator_are_boundaries() {
    // (x^2 - 2)(x + 1) / (x^2 - 2) at sqrt(2): the common factor vanishes.
    assert_eq!(
        uncertain(represented_tensor_ratio(
            &tensor(&[4, 1], ints(&[-2, -2, 1, 1])),
            &tensor(&[3, 1], ints(&[-2, 0, 1])),
            &[sqrt2()],
        )),
        UncertaintyReason::Boundary
    );
    // (x + 1) / (x - sqrt(2)) over the coefficient field Q(sqrt(2)).
    assert_eq!(
        uncertain(represented_tensor_ratio(
            &tensor(&[2, 1], ints(&[1, 1])),
            &tensor(&[2, 1], vec![-sqrt(2), int(1)]),
            &[sqrt2()],
        )),
        UncertaintyReason::Boundary
    );
    // x / (y^2 - 3) at (sqrt(2), sqrt(3)).
    assert_eq!(
        uncertain(represented_tensor_ratio(
            &tensor(&[2, 1, 1], ints(&[0, 1])),
            &tensor(&[1, 3, 1], ints(&[-3, 0, 1])),
            &[sqrt2(), sqrt3()],
        )),
        UncertaintyReason::Boundary
    );
}

#[test]
fn multi_source_tensor_ratios_refine_until_the_denominator_separates() {
    let sources = [sqrt2(), sqrt3()];
    // x / y.
    let image = decided(represented_tensor_ratio(
        &tensor(&[2, 1, 1], ints(&[0, 1])),
        &tensor(&[1, 2, 1], ints(&[0, 1])),
        &sources,
    ));
    assert_selects(&image, &(sqrt(2) / sqrt(3)).unwrap());

    // 1 / (y - x): the initial denominator enclosure [-1, 1] contains zero.
    let image = decided(represented_tensor_ratio(
        &tensor(&[1, 1, 1], ints(&[1])),
        &tensor(&[2, 2, 1], ints(&[0, 1, -1, 0])),
        &sources,
    ));
    assert_selects(&image, &(sqrt(3) + sqrt(2)));

    assert_eq!(
        uncertain(represented_tensor_ratio(
            &tensor(&[2, 1], ints(&[0, 1])),
            &tensor(&[2, 1, 1], ints(&[0, 1])),
            &sources,
        )),
        UncertaintyReason::Unsupported
    );
}

// Nested ratios `(A + B*sqrt(S)) / (C + D*sqrt(S))`.

#[test]
fn nested_ratios_without_tensor_axes_are_mobius_images_of_the_radical() {
    // (1 + sqrt(2)) / 2.
    let image = decided(represented_tensor_nested_ratio(
        &tensor(&[1], ints(&[1])),
        &tensor(&[1], ints(&[1])),
        &tensor(&[1], ints(&[2])),
        &tensor(&[1], ints(&[0])),
        &tensor(&[1], ints(&[2])),
        &[],
        &sqrt2(),
    ));
    assert_selects(&image, &((int(1) + sqrt(2)) / int(2)).unwrap());
}

#[test]
fn nested_ratios_over_selected_tuples() {
    let sources = [sqrt3()];
    let x = tensor(&[2, 1], ints(&[0, 1]));
    let one = tensor(&[1, 1], ints(&[1]));
    let zero = tensor(&[1, 1], ints(&[0]));
    let minus_one = tensor(&[1, 1], ints(&[-1]));
    let two = tensor(&[1, 1], ints(&[2]));

    // (x + sqrt(2)) / 1.
    let image = decided(represented_tensor_nested_ratio(
        &x,
        &one,
        &one,
        &zero,
        &two,
        &sources,
        &sqrt2(),
    ));
    assert_selects(&image, &(sqrt(3) + sqrt(2)));

    // The negative radical sheet: (x - sqrt(2)) / 1.
    let image = decided(represented_tensor_nested_ratio(
        &x,
        &one,
        &one,
        &zero,
        &two,
        &sources,
        &negative_sqrt2(),
    ));
    assert_selects(&image, &(sqrt(3) - sqrt(2)));

    // (x + sqrt(2)) / (x - sqrt(2)) = (sqrt(3) + sqrt(2))^2 = 5 + 2*sqrt(6).
    let image = decided(represented_tensor_nested_ratio(
        &x,
        &one,
        &x,
        &minus_one,
        &two,
        &sources,
        &sqrt2(),
    ));
    assert_selects(&image, &(int(5) + int(2) * sqrt(6)));
}

#[test]
fn nested_ratios_with_vanishing_denominators_are_boundaries() {
    let sources = [sqrt3()];
    let x = tensor(&[2, 1], ints(&[0, 1]));
    let one = tensor(&[1, 1], ints(&[1]));
    let zero = tensor(&[1, 1], ints(&[0]));
    // Denominator x^2 - 3.
    assert_eq!(
        uncertain(represented_tensor_nested_ratio(
            &x,
            &one,
            &tensor(&[3, 1], ints(&[-3, 0, 1])),
            &zero,
            &tensor(&[1, 1], ints(&[2])),
            &sources,
            &sqrt2(),
        )),
        UncertaintyReason::Boundary
    );
    // Denominator x - sqrt(3) on the positive sheet of sqrt(3).
    assert_eq!(
        uncertain(represented_tensor_nested_ratio(
            &x,
            &one,
            &x,
            &tensor(&[1, 1], ints(&[-1])),
            &tensor(&[1, 1], ints(&[3])),
            &sources,
            &sqrt3(),
        )),
        UncertaintyReason::Boundary
    );
    // Wrong rank.
    assert_eq!(
        uncertain(represented_tensor_nested_ratio(
            &tensor(&[2], ints(&[0, 1])),
            &one,
            &one,
            &zero,
            &one,
            &sources,
            &sqrt2(),
        )),
        UncertaintyReason::Unsupported
    );
}

// Line intersections.

#[test]
fn projective_line_intersections_share_one_determinant() {
    let sources = [sqrt2()];
    let one = tensor(&[1, 1], ints(&[1]));
    let minus_one = tensor(&[1, 1], ints(&[-1]));
    let zero = tensor(&[1, 1], ints(&[0]));
    let minus_x = tensor(&[2, 1], ints(&[0, -1]));
    // x + y - sqrt(2) = 0 and x - y = 0 meet at (sqrt(2)/2, sqrt(2)/2).
    let [x, y] = decided(represented_projective_line_intersection(
        [one.clone(), one.clone(), minus_x.clone()],
        [one.clone(), minus_one.clone(), zero.clone()],
        &sources,
    ));
    let half_sqrt2 = (sqrt(2) / int(2)).unwrap();
    assert_selects(&x, &half_sqrt2);
    assert_selects(&y, &half_sqrt2);

    // Parallel lines x + y = sqrt(2) and x + y = 0 have a zero determinant.
    assert_eq!(
        uncertain(represented_projective_line_intersection(
            [one.clone(), one.clone(), minus_x.clone()],
            [one.clone(), one.clone(), zero.clone()],
            &sources,
        )),
        UncertaintyReason::Predicate
    );
    // Mixed ranks cannot be multiplied.
    assert_eq!(
        uncertain(represented_projective_line_intersection(
            [one.clone(), one.clone(), minus_x],
            [tensor(&[1], ints(&[1])), minus_one, zero],
            &sources,
        )),
        UncertaintyReason::Unsupported
    );
}

// Constants, degenerate isolators and continued refinement.

#[test]
fn tensor_values_without_sources_are_exact_constants() {
    // A dyadic constant survives outward dyadic coefficient rounding exactly.
    let image = decided(represented_dense_value_with_coefficient_precision(
        &tensor(&[1], vec![q(3, 4)]),
        &[],
        -64,
    ));
    assert_equal(image.exact_point_witness().unwrap(), &q(3, 4));
    // 7/3 is rounded to a dyadic enclosure; any decided value is still 7/3.
    match represented_dense_value_with_coefficient_precision(&tensor(&[1], vec![q(7, 3)]), &[], -64)
    {
        Classification::Decided(image) => assert_selects(&image, &q(7, 3)),
        Classification::Uncertain(reason) => assert_eq!(reason, UncertaintyReason::Predicate),
    }

    let image = decided(represented_tensor_ratio(
        &tensor(&[1], ints(&[6])),
        &tensor(&[1], ints(&[3])),
        &[],
    ));
    assert_selects(&image, &int(2));
    assert_eq!(
        uncertain(represented_tensor_ratio(
            &tensor(&[1], ints(&[6])),
            &tensor(&[1], ints(&[0])),
            &[],
        )),
        UncertaintyReason::Boundary
    );
}

#[test]
fn refined_dense_values_separate_conjugate_images() {
    // x - y at (sqrt(2), sqrt(3)). The first enclosure [-1, 1] contains both
    // real conjugates +-(sqrt(3) - sqrt(2)), so the sources must be refined.
    let difference = tensor(&[2, 2, 1], ints(&[0, -1, 1, 0]));
    let sources = [sqrt2(), sqrt3()];
    let image = decided(represented_dense_value_refined(&difference, &sources));
    assert_selects(&image, &(sqrt(2) - sqrt(3)));

    let mut blocked = None;
    let outcome = work_budget::with_budget(
        WorkBudget {
            refinements: 0,
            promotions: 64,
        },
        || blocked = Some(represented_dense_value_refined(&difference, &sources)),
    );
    assert_eq!(outcome.result.unwrap_err().kind, WorkKind::RootRefinement);
    assert_eq!(
        blocked.unwrap(),
        Classification::Uncertain(UncertaintyReason::BudgetExhausted)
    );

    // 1 / (y - x) needs the same refinement before its denominator separates.
    let mut ratio = None;
    let outcome = work_budget::with_budget(
        WorkBudget {
            refinements: 0,
            promotions: 1024,
        },
        || {
            ratio = Some(represented_tensor_ratio(
                &tensor(&[1, 1, 1], ints(&[1])),
                &tensor(&[2, 2, 1], ints(&[0, 1, -1, 0])),
                &sources,
            ))
        },
    );
    assert!(outcome.result.is_err());
    assert_eq!(
        ratio.unwrap(),
        Classification::Uncertain(UncertaintyReason::BudgetExhausted)
    );
}

#[test]
fn point_isolators_without_witnesses_use_certified_dyadic_image_bounds() {
    // Interval arithmetic over [sqrt(2), sqrt(2)] and [cbrt(2), cbrt(2)]
    // collapses to the single irrational endpoint sqrt(2) + cbrt(2).
    let s2 = point_isolator(&[-2, 0, 1], sqrt(2));
    let c2 = point_isolator(&[-2, 0, 0, 1], cbrt(2));
    let image = decided(represented_affine_coordinate(
        &[(&s2, &int(1)), (&c2, &int(1))],
        &Real::zero(),
    ));
    assert_selects_root_of(&image, &(sqrt(2) + cbrt(2)), &[-4, -24, 12, -4, -6, 0, 1]);
}

/// Solutions of `q^2 - 3 p^2 = 1`, so `p*sqrt(3) - q = -1/(q + p*sqrt(3))`.
fn pell_three(iterations: usize) -> (Real, Real) {
    let (mut q, mut p) = (int(2), int(1));
    for _ in 0..iterations {
        let next_q = int(2) * &q + int(3) * &p;
        let next_p = &q + int(2) * &p;
        q = next_q;
        p = next_p;
    }
    assert_exact_zero(&(&q * &q - int(3) * &p * &p - int(1)));
    (q, p)
}

#[test]
fn nearly_vanishing_denominators_are_never_misclassified() {
    // p is about 2^76, so p*sqrt(3) - q = -1/(q + p*sqrt(3)) is about
    // -2^-78: sixty-four bisections of sqrt(3) cannot exclude zero from its
    // enclosure. A decided quotient must be -(q + p*sqrt(3)); otherwise the
    // nonzero proof of the denominator is reported as undecided.
    let (q, p) = pell_three(40);
    let one = tensor(&[1, 1], ints(&[1]));
    let zero = tensor(&[1, 1], ints(&[0]));
    let expected = -(&q + &p * sqrt(3));
    match represented_tensor_nested_ratio(
        &one,
        &zero,
        &tensor(&[2, 1], vec![-q.clone(), p.clone()]),
        &zero,
        &tensor(&[1, 1], ints(&[2])),
        &[sqrt3()],
        &sqrt2(),
    ) {
        Classification::Decided(image) => assert_selects(&image, &expected),
        Classification::Uncertain(reason) => assert_eq!(reason, UncertaintyReason::Predicate),
    }
    match represented_tensor_ratio(
        &tensor(&[1, 1, 1], ints(&[1])),
        &tensor(&[1, 2, 1], vec![-q.clone(), p.clone()]),
        &[sqrt2(), sqrt3()],
    ) {
        Classification::Decided(image) => assert_selects(&image, &expected),
        Classification::Uncertain(reason) => assert_eq!(reason, UncertaintyReason::Predicate),
    }
}

/// Solutions of `q^2 - 2 p^2 = -1` after an even number of steps.
fn pell_two(iterations: usize) -> (Real, Real) {
    assert!(iterations.is_multiple_of(2));
    let (mut q, mut p) = (int(1), int(1));
    for _ in 0..iterations {
        let next_q = &q + int(2) * &p;
        let next_p = &q + &p;
        q = next_q;
        p = next_p;
    }
    assert_exact_zero(&(&q * &q - int(2) * &p * &p + int(1)));
    (q, p)
}

#[test]
fn close_conjugate_images_continue_refinement_after_the_bounded_rounds() {
    // With y = 2^(1/4) and p*sqrt(2) - q = 1/(p*sqrt(2) + q), about 2^-143,
    // the image z = y*(p*y^2 - q) and its real conjugate -z (from -y) are
    // closer than the 64- and 128-step quotient rounds can separate.
    let (pell_q, pell_p) = pell_two(112);
    let fourth_root = root(&[-2, 0, 0, 0, 1], int(1), int(2));
    let numerator = tensor(
        &[4, 1],
        vec![Real::zero(), -pell_q.clone(), Real::zero(), pell_p.clone()],
    );
    let value = sqrt(2).sqrt().unwrap() * (&pell_p * sqrt(2) - &pell_q);
    assert_strictly_between(&value, &Real::zero(), &q(1, 1 << 62));
    // z^2 + 4pq = (2p^2 + q^2) y^2, so (z^2 + 4pq)^2 = 2(2p^2 + q^2)^2.
    let (p_rational, q_rational) = (rational(&pell_p), rational(&pell_q));
    let pq = &p_rational * &q_rational;
    let norm = &(&(&p_rational * &p_rational) * &hyperreal::Rational::new(2))
        + &(&q_rational * &q_rational);
    let minimal = [
        &(&(&pq * &pq) * &hyperreal::Rational::new(16))
            - &(&(&norm * &norm) * &hyperreal::Rational::new(2)),
        hyperreal::Rational::zero(),
        &pq * &hyperreal::Rational::new(8),
        hyperreal::Rational::zero(),
        hyperreal::Rational::one(),
    ];

    let image = decided(represented_tensor_ratio(
        &numerator,
        &tensor(&[1, 1], ints(&[1])),
        std::slice::from_ref(&fourth_root),
    ));
    assert_selects_root_of_rationals(&image, &value, &minimal);

    let zero = tensor(&[1, 1], ints(&[0]));
    let image = decided(represented_tensor_nested_ratio(
        &numerator,
        &zero,
        &tensor(&[1, 1], ints(&[1])),
        &zero,
        &tensor(&[1, 1], ints(&[2])),
        std::slice::from_ref(&fourth_root),
        &sqrt2(),
    ));
    assert_selects_root_of_rationals(&image, &value, &minimal);
}

#[test]
fn constructions_over_invalid_sources_are_unsupported() {
    let invalid = non_unit_root();
    let x = tensor(&[2, 1], ints(&[0, 1]));
    let one = tensor(&[1, 1], ints(&[1]));
    let zero = tensor(&[1, 1], ints(&[0]));
    let minus_one = tensor(&[1, 1], ints(&[-1]));
    assert_eq!(
        uncertain(represented_tensor_nested_ratio(
            &x,
            &one,
            &one,
            &zero,
            &tensor(&[1, 1], ints(&[2])),
            std::slice::from_ref(&invalid),
            &sqrt2(),
        )),
        UncertaintyReason::Unsupported
    );
    assert_eq!(
        uncertain(represented_projective_line_intersection(
            [one.clone(), one.clone(), tensor(&[2, 1], ints(&[0, -1]))],
            [one, minus_one, zero],
            std::slice::from_ref(&invalid),
        )),
        UncertaintyReason::Unsupported
    );
    assert_eq!(
        uncertain(represented_dense_value_refined(
            &tensor(&[2, 1], ints(&[0, 1])),
            &[invalid]
        )),
        UncertaintyReason::Unsupported
    );
}

#[test]
fn unresolved_source_enclosures_have_no_tensor_interval() {
    // An isolator [u, 2u] whose endpoints are both exact zeroes that STRICT
    // cannot order: products over it cannot be bounded.
    let unresolved = unchecked_root(ints(&[0, 1]), unknown_zero(), int(2) * unknown_zero());
    let sources = [sqrt2(), unresolved];
    // x*y + 1 and x + y.
    assert!(dense_tensor_interval(&xy_plus_one(), &sources).is_none());
    assert!(dense_tensor_interval(&tensor(&[2, 2, 1], ints(&[0, 1, 1, 0])), &sources).is_none());
}

#[test]
fn field_invariant_errors_display_their_message() {
    let error = FieldInvariantError("selected sheet lost".to_owned());
    assert_eq!(error.to_string(), "selected sheet lost");
    let source: &dyn std::error::Error = &error;
    assert!(source.source().is_none());
    assert_eq!(
        ContextError::from(error),
        ContextError::Invariant("selected sheet lost".to_owned())
    );
}
