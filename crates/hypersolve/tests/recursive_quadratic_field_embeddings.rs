//! Foreign-base embeddings, base-preserving rebases, ordered-field
//! arithmetic and polynomial projection over recursive quadratic fields.
//!
//! Every mapped value is checked against an independently built `Real`
//! closed form, an exact algebraic identity (such as `root^2 == radicand`),
//! or a certified enclosure of the closed form.

#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

use std::cmp::Ordering;
use std::sync::Arc;

use hyperreal::{Real, RealSign};
use hypersolve::classification::{Classification, UncertaintyReason};
use hypersolve::recursive_quadratic_field::*;
use hypersolve::{
    DenseTensorPolynomial, OrderedFieldPolynomialContext, OrderedFieldRootIsolationConfig,
    OrderedFieldRootIsolationStatus, RealInterval, SelectedAlgebraContext,
    isolate_ordered_field_polynomial_roots, ordered_field_sign_at_selected_root,
};
use support::*;

/// A rank-one base over `x = sqrt2` with constant speeds 2 and 3.
fn sqrt_two_base() -> RecursiveQuadraticField {
    RecursiveQuadraticField::base(
        vec![sqrt_two_root()],
        constant_tensor(1, real(2)),
        constant_tensor(1, real(3)),
    )
    .unwrap()
}

/// `sqrt_two_base()(sqrt(1 + x))`.
fn sqrt_two_tower() -> (RecursiveQuadraticField, RecursiveQuadraticField) {
    let base = sqrt_two_base();
    let x = base.retained_root_value(&sqrt_two_root()).unwrap();
    let radicand = x.add(&base.constant(real(1)).unwrap()).unwrap();
    let extension = base.extension(radicand).unwrap();
    (base, extension)
}

/// A rank-two base over `(2^(1/3), sqrt2)`.
fn pair_base(
    first: DenseTensorPolynomial,
    second: DenseTensorPolynomial,
) -> RecursiveQuadraticField {
    RecursiveQuadraticField::base(vec![cube_root_two(), sqrt_two_root()], first, second).unwrap()
}

/// A rank-one base over `c = 2^(1/3)` with the given speed polynomials.
fn cube_field(first: &[Real], second: &[Real]) -> RecursiveQuadraticField {
    RecursiveQuadraticField::base(
        vec![cube_root_two()],
        axis_tensor(first),
        axis_tensor(second),
    )
    .unwrap()
}

/// `x*sqrt(A) + sqrt(B) + sqrt(A)*sqrt(B)` over `sqrt_two_base()`, whose
/// closed form is `2 + sqrt3 + sqrt6`.
fn mixed_value(base: &RecursiveQuadraticField) -> RecursiveQuadraticValue {
    axis_value(
        base,
        &[real(0)],
        &[real(0), real(1)],
        &[real(1)],
        &[real(1)],
    )
    .unwrap()
}

fn mixed_closed_form() -> Real {
    real(2) + root_of(3) + root_of(6)
}

fn enclosure(lower: &Real, upper: &Real) -> RealInterval {
    RealInterval {
        lower: lower.clone(),
        upper: upper.clone(),
    }
}

#[test]
fn foreign_embeddings_scale_constant_speeds_and_adjoin_generators() {
    let policy = TestContext::strict();
    let (source_base, source) = sqrt_two_tower();
    let target = pair_base(constant_tensor(2, real(8)), constant_tensor(2, real(3)));
    let Classification::Decided(Some((joined, embedding))) =
        recursive_embed_foreign_field(&source, base_arc(&target), vec![1], target.clone(), &policy)
            .unwrap()
    else {
        panic!("the source tower embeds");
    };
    assert!(Arc::ptr_eq(&base_arc(&joined), &base_arc(&target)));
    assert_eq!(joined.base_and_extension_path().1.len(), 1);
    assert_eq!(embedding.axes, vec![1]);
    assert_eq!(embedding.extensions.len(), 1);
    // sqrt(2) = sqrt(1/4) * sqrt(8) and sqrt(3) = sqrt(3/8) * sqrt(8).
    assert_value(&embedding.first_root, &root_of(2));
    assert_value(&embedding.second_root, &root_of(3));

    let value = source
        .element(
            mixed_value(&source_base),
            source_base.constant(real(3)).unwrap(),
        )
        .unwrap();
    let expected = mixed_closed_form() + real(3) * Tower::first_closed_form();
    assert_value(&value, &expected);
    let mapped = embedding.value(&value, &joined).unwrap();
    assert!(mapped.field().same_field(&joined));
    assert_value(&mapped, &expected);
    let polynomial = embedding
        .base_polynomial(&axis_tensor(&[real(1), real(1)]))
        .unwrap();
    assert_value(&polynomial, &(real(1) + root_of(2)));

    // Values outside the embedded source are refused.
    assert!(
        embedding
            .value(&int_value(&quadratic_base(2, 3), [1, 0, 0, 0]), &joined)
            .is_none()
    );
    let (_, other) = sqrt_two_tower();
    assert!(embedding.value(&generator(&other), &joined).is_none());
}

#[test]
fn foreign_embeddings_reuse_an_existing_equal_generator() {
    let policy = TestContext::strict();
    let (_, source) = sqrt_two_tower();
    let target = pair_base(constant_tensor(2, real(2)), constant_tensor(2, real(3)));
    let y = target.retained_root_value(&sqrt_two_root()).unwrap();
    let existing = target
        .extension(y.add(&target.constant(real(1)).unwrap()).unwrap())
        .unwrap();
    let Classification::Decided(Some((joined, embedding))) = recursive_embed_foreign_field(
        &source,
        base_arc(&target),
        vec![1],
        existing.clone(),
        &policy,
    )
    .unwrap() else {
        panic!("the source tower embeds");
    };
    assert!(joined.same_field(&existing));
    assert!(Arc::ptr_eq(
        &embedding.extensions[0].target,
        &extension_arc(&existing)
    ));
    let image = embedding.value(&generator(&source), &joined).unwrap();
    assert!(
        image
            .subtract(&generator(&existing))
            .unwrap()
            .is_structurally_zero()
    );
}

#[test]
fn foreign_embeddings_prove_equal_generators_stored_differently() {
    let policy = TestContext::strict();
    // Source over c = 2^(1/3) with sqrt(1 + c); speeds 2 and 8.
    let source_base = RecursiveQuadraticField::base(
        vec![cube_root_two()],
        constant_tensor(1, real(2)),
        constant_tensor(1, real(8)),
    )
    .unwrap();
    let c = source_base.retained_root_value(&cube_root_two()).unwrap();
    let source = source_base
        .extension(c.add(&source_base.constant(real(1)).unwrap()).unwrap())
        .unwrap();
    // The target already holds sqrt(1 + c + 2*sqrt2 - sqrt8) = sqrt(1 + c).
    let target = RecursiveQuadraticField::base(
        vec![cube_root_two()],
        constant_tensor(1, real(2)),
        constant_tensor(1, real(8)),
    )
    .unwrap();
    let disguised = axis_value(
        &target,
        &[real(1), real(1)],
        &[real(2)],
        &[real(-1)],
        &[real(0)],
    )
    .unwrap();
    let existing = target.extension(disguised).unwrap();
    let Classification::Decided(Some((joined, embedding))) = recursive_embed_foreign_field(
        &source,
        base_arc(&target),
        vec![0],
        existing.clone(),
        &policy,
    )
    .unwrap() else {
        panic!("the source tower embeds");
    };
    assert!(joined.same_field(&existing));
    assert!(Arc::ptr_eq(
        &embedding.extensions[0].target,
        &extension_arc(&existing)
    ));
}

#[test]
fn foreign_embeddings_adjoin_generators_distinct_from_existing_ones() {
    let policy = TestContext::strict();
    let (_, source) = sqrt_two_tower();
    let target = pair_base(constant_tensor(2, real(2)), constant_tensor(2, real(3)));
    let existing = target.extension(target.constant(real(5)).unwrap()).unwrap();
    let Classification::Decided(Some((joined, embedding))) = recursive_embed_foreign_field(
        &source,
        base_arc(&target),
        vec![1],
        existing.clone(),
        &policy,
    )
    .unwrap() else {
        panic!("the source tower embeds");
    };
    assert_eq!(joined.base_and_extension_path().1.len(), 2);
    let image = embedding.value(&generator(&source), &joined).unwrap();
    assert_value(&image, &Tower::first_closed_form());
    let preserved = joined.lift(&generator(&existing)).unwrap();
    assert_value(&preserved, &root_of(5));
}

#[test]
fn foreign_embeddings_select_polynomial_and_constant_roots() {
    let policy = TestContext::strict();
    // Source speeds (1 + c)^2 and 2; target speeds 1 + c^2 and c share no
    // positive scale with them.
    let source = cube_field(&[real(1), real(2), real(1)], &[real(2)]);
    let target = cube_field(&[real(1), real(0), real(1)], &[real(0), real(1)]);
    let Classification::Decided(Some((joined, embedding))) =
        recursive_embed_foreign_field(&source, base_arc(&target), vec![0], target.clone(), &policy)
            .unwrap()
    else {
        panic!("the source base embeds");
    };
    assert!(joined.same_field(&target));
    let c = cube_root_two_real();
    // first_root = 1 + c: its square is the source speed and it is positive.
    let speed = recursive_quadratic_rational_value(
        &base_arc(&target),
        axis_tensor(&[real(1), real(2), real(1)]),
    )
    .unwrap();
    assert!(
        embedding
            .first_root
            .square()
            .unwrap()
            .subtract(&speed)
            .unwrap()
            .is_structurally_zero()
    );
    assert_encloses(&embedding.first_root.interval(16).unwrap(), &(real(1) + &c));
    assert_value(&embedding.second_root, &root_of(2));
    // sqrt(A) + sqrt(B) maps to (1 + c) + sqrt2.
    let value = axis_value(&source, &[real(0)], &[real(1)], &[real(1)], &[real(0)]).unwrap();
    let mapped = embedding.value(&value, &joined).unwrap();
    assert_encloses(&mapped.interval(16).unwrap(), &(real(1) + &c + root_of(2)));
}

#[test]
fn foreign_embeddings_adjoin_irrational_speed_roots_and_reuse_prior_ones() {
    let policy = TestContext::strict();
    // Source speeds c and 4c; target speeds 1 + c^2 and c^2.
    let source = cube_field(&[real(0), real(1)], &[real(0), real(4)]);
    let target = cube_field(&[real(1), real(0), real(1)], &[real(0), real(0), real(1)]);
    let Classification::Decided(Some((joined, embedding))) =
        recursive_embed_foreign_field(&source, base_arc(&target), vec![0], target.clone(), &policy)
            .unwrap()
    else {
        panic!("the source base embeds");
    };
    // sqrt(c) is adjoined once; sqrt(4c) = 2*sqrt(c) reuses it.
    assert_eq!(joined.base_and_extension_path().1.len(), 1);
    let c = joined.lift(&cube(&target)).unwrap();
    let first = joined.lift(&embedding.first_root).unwrap();
    let second = joined.lift(&embedding.second_root).unwrap();
    assert!(
        first
            .square()
            .unwrap()
            .subtract(&c)
            .unwrap()
            .is_structurally_zero()
    );
    assert!(
        second
            .subtract(&first.scale(&real(2)).unwrap())
            .unwrap()
            .is_structurally_zero()
    );
    assert_encloses(&first.interval(16).unwrap(), &sqrt(cube_root_two_real()));

    // A field that already holds sqrt(c) reuses its authored generator.
    let existing = target.extension(cube(&target)).unwrap();
    let Classification::Decided(Some((reused, _))) = recursive_embed_foreign_field(
        &source,
        base_arc(&target),
        vec![0],
        existing.clone(),
        &policy,
    )
    .unwrap() else {
        panic!("the source base embeds");
    };
    assert!(reused.same_field(&existing));
}

fn cube(field: &RecursiveQuadraticField) -> RecursiveQuadraticValue {
    field.retained_root_value(&cube_root_two()).unwrap()
}

#[test]
fn foreign_embeddings_reject_negative_speeds_and_malformed_axes() {
    let policy = TestContext::strict();
    // -c has no positive square root.
    let source = cube_field(&[real(0), real(-1)], &[real(1)]);
    let target = cube_field(&[real(1), real(0), real(1)], &[real(0), real(0), real(1)]);
    assert!(matches!(
        recursive_embed_foreign_field(&source, base_arc(&target), vec![0], target.clone(), &policy),
        Err(TestError::Field(_))
    ));
    // An axis map whose length differs from the source rank, and one that
    // names an axis outside the target tuple.
    let positive = cube_field(&[real(1)], &[real(1)]);
    assert!(matches!(
        recursive_embed_foreign_field(
            &positive,
            base_arc(&target),
            vec![],
            target.clone(),
            &policy
        )
        .unwrap(),
        Classification::Decided(None)
    ));
    assert!(matches!(
        recursive_embed_foreign_field(
            &positive,
            base_arc(&target),
            vec![3],
            target.clone(),
            &policy
        )
        .unwrap(),
        Classification::Decided(None)
    ));
}

#[test]
fn foreign_embeddings_map_a_zero_speed_to_zero() {
    let policy = TestContext::strict();
    let source = quadratic_base(0, 3);
    let target = quadratic_base(2, 3);
    let Classification::Decided(Some((joined, embedding))) = recursive_embed_foreign_field(
        &source,
        base_arc(&target),
        Vec::new(),
        target.clone(),
        &policy,
    )
    .unwrap() else {
        panic!("the source base embeds");
    };
    assert!(joined.same_field(&target));
    assert!(embedding.first_root.is_coefficientwise_stored_zero());
    assert_value(&embedding.second_root, &root_of(3));
    let mapped = embedding
        .value(&int_value(&source, [1, 5, 1, 7]), &joined)
        .unwrap();
    assert_value(&mapped, &(real(1) + root_of(3)));
}

#[test]
fn foreign_base_roots_scale_prior_generators() {
    let policy = TestContext::strict();
    let target = cube_field(&[real(1), real(0), real(1)], &[real(0), real(0), real(1)]);
    let base = base_arc(&target);
    let prior_polynomial = axis_tensor(&[real(0), real(1)]);
    let Classification::Decided(Some((field, prior))) = recursive_foreign_base_root(
        prior_polynomial.clone(),
        &base,
        target.clone(),
        None,
        &policy,
    )
    .unwrap() else {
        panic!("sqrt(c) is selected");
    };
    let Classification::Decided(Some((same, root))) = recursive_foreign_base_root(
        axis_tensor(&[real(0), real(9)]),
        &base,
        field.clone(),
        Some((&prior_polynomial, &prior)),
        &policy,
    )
    .unwrap() else {
        panic!("sqrt(9c) reuses the prior root");
    };
    assert!(same.same_field(&field));
    assert!(
        root.subtract(&prior.scale(&real(3)).unwrap())
            .unwrap()
            .is_structurally_zero()
    );
    // A constant multiple of a target speed reuses that base generator.
    let Classification::Decided(Some((_, scaled))) = recursive_foreign_base_root(
        axis_tensor(&[real(4), real(0), real(4)]),
        &base,
        target.clone(),
        None,
        &policy,
    )
    .unwrap() else {
        panic!("sqrt(4 (1 + c^2)) is 2 sqrt(1 + c^2)");
    };
    let generator = recursive_quadratic_base_generator(&base, true).unwrap();
    assert!(
        scaled
            .subtract(&generator.scale(&real(2)).unwrap())
            .unwrap()
            .is_structurally_zero()
    );
}

#[test]
fn base_preserving_rebases_carry_values_into_a_larger_tuple() {
    let (source_base, source) = sqrt_two_tower();
    let target = pair_base(constant_tensor(2, real(2)), constant_tensor(2, real(3)));
    let (rebased, original, embeddings) =
        recursive_rebase_field_preserving_base(&source, base_arc(&target), &[1]).unwrap();
    assert!(Arc::ptr_eq(&original, &base_arc(&source_base)));
    assert!(Arc::ptr_eq(&base_arc(&rebased), &base_arc(&target)));
    assert_eq!(embeddings.len(), 1);
    let value = source
        .element(
            mixed_value(&source_base),
            source_base.constant(real(3)).unwrap(),
        )
        .unwrap();
    let mapped = recursive_rebase_value_preserving_base(
        &value,
        &original,
        &base_arc(&target),
        &[1],
        &embeddings,
    )
    .unwrap();
    assert!(mapped.field().same_field(&rebased));
    assert_value(
        &mapped,
        &(mixed_closed_form() + real(3) * Tower::first_closed_form()),
    );

    assert!(recursive_rebase_field_preserving_base(&source, base_arc(&target), &[0, 1]).is_none());
    let foreign = int_value(&quadratic_base(2, 3), [1, 0, 0, 0]);
    assert!(
        recursive_rebase_value_preserving_base(
            &foreign,
            &original,
            &base_arc(&target),
            &[1],
            &embeddings
        )
        .is_none()
    );
    assert!(
        recursive_rebase_value_preserving_base(&value, &original, &base_arc(&target), &[1], &[])
            .is_none()
    );
}

fn ordered_context(
    field: &RecursiveQuadraticField,
) -> RecursiveQuadraticOrderedFieldContext<TestContext> {
    RecursiveQuadraticOrderedFieldContext {
        field: field.clone(),
        policy: TestContext::strict(),
    }
}

#[test]
fn ordered_field_context_arithmetic_is_exact_and_typed() {
    let tower = Tower::new();
    let mut context = ordered_context(&tower.first);
    let g = generator(&tower.first);
    let three_halves = context.constant(&fraction(3, 2)).unwrap();
    assert_value(&three_halves, &fraction(3, 2));
    let g_closed = Tower::first_closed_form();
    assert_value(
        &context.add(&g, &three_halves).unwrap(),
        &(&g_closed + fraction(3, 2)),
    );
    assert_value(
        &context.multiply(&g, &three_halves).unwrap(),
        &(&g_closed * fraction(3, 2)),
    );
    assert_value(
        &context.scale(&g, &real(-4)).unwrap(),
        &(&g_closed * real(-4)),
    );
    let foreign = generator(&Tower::new().first);
    assert!(matches!(
        context.add(&g, &foreign),
        Err(RecursiveQuadraticOrderedFieldError::Context(
            TestError::Field(_)
        ))
    ));
    assert!(matches!(
        context.multiply(&g, &foreign),
        Err(RecursiveQuadraticOrderedFieldError::Context(
            TestError::Field(_)
        ))
    ));
    // Signs: g ~ 1.5538 lies between 3/2 and 8/5.
    let below = g.subtract(&three_halves).unwrap();
    let above = g
        .subtract(&context.constant(&fraction(8, 5)).unwrap())
        .unwrap();
    assert_eq!(
        strict_sign(&(&g_closed - fraction(3, 2))),
        Some(RealSign::Positive)
    );
    assert_eq!(
        strict_sign(&(&g_closed - fraction(8, 5))),
        Some(RealSign::Negative)
    );
    assert_eq!(context.sign(&below).unwrap(), Ordering::Greater);
    assert_eq!(context.sign(&above).unwrap(), Ordering::Less);
    let zero = context.constant(&Real::zero()).unwrap();
    assert_eq!(context.sign(&zero).unwrap(), Ordering::Equal);
    assert_eq!(
        context.sign_if_separated(&zero).unwrap(),
        Some(Ordering::Equal)
    );
    assert_eq!(
        context.sign_if_separated(&below).unwrap(),
        Some(Ordering::Greater)
    );
    assert_eq!(
        context.sign_if_separated(&above).unwrap(),
        Some(Ordering::Less)
    );
    // Normalization removes the common positive factor 2.
    let mut coefficients = vec![
        g.scale(&real(6)).unwrap(),
        context.constant(&real(4)).unwrap(),
    ];
    context.normalize_positive_scale(&mut coefficients);
    assert_value(&coefficients[0], &(&g_closed * real(3)));
    assert_value(&coefficients[1], &real(2));
    // Only a bounded pass limits remainder eliminations.
    assert_eq!(context.remainder_elimination_budget(), None);
    let policy = TestContext::strict();
    let bounded = RecursiveQuadraticOrderedFieldContext {
        field: tower.first.clone(),
        policy,
    };
    let budget = bounded
        .policy
        .bounded_exact_predicate_pass(|| bounded.remainder_elimination_budget());
    assert_eq!(budget, Some(10));
}

#[test]
fn ordered_field_context_reports_undecidable_coefficients() {
    let field = quadratic_base(2, 3);
    let mut context = ordered_context(&field);
    let undecided = base_value(&field, [undecidable_zero(), real(0), real(0), real(0)]);
    assert!(matches!(
        context.sign(&undecided),
        Err(RecursiveQuadraticOrderedFieldError::Uncertain)
    ));
}

#[test]
fn ordered_field_selected_root_queries_over_a_tower() {
    let tower = Tower::new();
    let mut context = ordered_context(&tower.first);
    let g = generator(&tower.first);
    let zero = tower.first.constant(Real::zero()).unwrap();
    let one = tower.first.constant(Real::one()).unwrap();
    // t^2 - g has the roots +-g2 with g2 = (1 + sqrt2)^(1/4) ~ 1.2465.
    let defining = vec![g.scale(&real(-1)).unwrap(), zero.clone(), one.clone()];
    let interval = hypersolve::IsolatedRootInterval {
        lower: real(1),
        upper: real(2),
        exact_root: None,
        distinct_root_count: 1,
    };
    let g2 = Tower::second_closed_form();
    for (offset, closed_sign, expected) in [
        (fraction(5, 4), RealSign::Negative, Ordering::Less),
        (fraction(6, 5), RealSign::Positive, Ordering::Greater),
    ] {
        assert_eq!(strict_sign(&(&g2 - &offset)), Some(closed_sign));
        let predicate = vec![tower.first.constant(-offset).unwrap(), one.clone()];
        assert_eq!(
            ordered_field_sign_at_selected_root(&defining, &predicate, &interval, &mut context)
                .unwrap(),
            Some(expected)
        );
    }
    let report = isolate_ordered_field_polynomial_roots(
        defining,
        &real(-2),
        &real(2),
        OrderedFieldRootIsolationConfig::default(),
        &mut context,
    )
    .unwrap();
    assert_eq!(report.status, OrderedFieldRootIsolationStatus::Isolated);
    assert_eq!(report.intervals.len(), 2);
    let mut roots = [-g2.clone(), g2];
    roots.sort_by(|left, right| {
        if strict_sign(&(left - right)) == Some(RealSign::Negative) {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    });
    for (interval, root) in report.intervals.iter().zip(&roots) {
        assert_encloses(&enclosure(&interval.lower, &interval.upper), root);
    }
}

#[test]
fn projections_drop_vanishing_generator_components() {
    let tower = Tower::new();
    let g = generator(&tower.first);
    let zero = tower.first.constant(Real::zero()).unwrap();
    // g*(t - 2) + 0*t^2: the trailing zero is dropped and only the radical
    // part survives, so the projection is t - 2.
    let (_, projection) = recursive_quadratic_polynomial_projection(vec![
        g.scale(&real(-2)).unwrap(),
        g.clone(),
        zero,
    ])
    .unwrap();
    assert_real_eq(
        &Real::eval_poly(projection.coefficients(), &real(2)),
        &Real::zero(),
    );
    // Lifted base coefficients carry no radical part: t - 3.
    let (_, projection) = recursive_quadratic_polynomial_projection(vec![
        tower.first.constant(real(-3)).unwrap(),
        tower.first.constant(real(1)).unwrap(),
    ])
    .unwrap();
    assert_real_eq(
        &Real::eval_poly(projection.coefficients(), &real(3)),
        &Real::zero(),
    );
    // Base coefficients supported on one radical component each.
    for (parts, root) in [
        ([[0, -1, 0, 0], [0, 1, 0, 0]], real(1)),
        ([[0, 0, -4, 0], [0, 0, 2, 0]], real(2)),
        ([[0, 0, 0, 3], [0, 0, 0, 1]], real(-3)),
    ] {
        let (_, projection) = recursive_quadratic_polynomial_projection(
            parts
                .iter()
                .map(|parts| int_value(&tower.base, *parts))
                .collect(),
        )
        .unwrap();
        assert_real_eq(
            &Real::eval_poly(projection.coefficients(), &root),
            &Real::zero(),
        );
    }
}

#[test]
fn polynomial_square_roots_report_undecidable_coefficients() {
    let policy = TestContext::strict();
    let undecided = undecidable_zero();
    for coefficients in [
        // An undecidable leading coefficient.
        vec![real(1), undecided.clone()],
        // An undecidable valuation coefficient.
        vec![undecided.clone(), real(0), real(1)],
        // (1 + t)^2 perturbed by an undecidable leading term.
        vec![real(1), real(2), real(1) + &undecided],
    ] {
        assert_eq!(
            polynomial_square_root(&coefficients, &policy).unwrap(),
            Classification::Uncertain(UncertaintyReason::RealSign)
        );
    }
}

#[test]
fn joins_with_undecidable_radicand_equality_stay_undecided() {
    let field = quadratic_base(2, 3);
    let first = field.extension(field.constant(real(2)).unwrap()).unwrap();
    let second = field
        .extension(base_value(
            &field,
            [real(2) + undecidable_zero(), real(0), real(0), real(0)],
        ))
        .unwrap();
    let strict = TestContext::strict();
    assert!(first.joined_with(&second, &strict).unwrap().is_uncertain());
    // Generator identity is exact evidence: an approximate context must not
    // consume its terminal to merge the generators.
    let approximate = TestContext::approximate();
    assert!(
        first
            .joined_with(&second, &approximate)
            .unwrap()
            .is_uncertain()
    );
    assert_eq!(approximate.approximations(), 0);
}

#[test]
fn projective_scalars_with_undecidable_denominators_stay_undecided() {
    let policy = TestContext::strict();
    let field = quadratic_base(2, 3);
    let scalar = RecursiveQuadraticProjectiveScalar {
        numerator: field.constant(real(1)).unwrap(),
        denominator: base_value(&field, [undecidable_zero(), real(0), real(0), real(0)]),
    };
    assert!(scalar.represented_value(&policy).unwrap().is_uncertain());
}

#[test]
fn projective_scalars_over_mixed_witness_tuples_publish_their_value() {
    let policy = TestContext::strict();
    let field = RecursiveQuadraticField::base(
        vec![sqrt_two_root(), cube_root_two()],
        constant_tensor(2, real(1)),
        constant_tensor(2, real(1)),
    )
    .unwrap();
    // (sqrt2 + c) / 2 with only the sqrt2 axis witnessed.
    let x = field.retained_root_value(&sqrt_two_root()).unwrap();
    let c = field.retained_root_value(&cube_root_two()).unwrap();
    let scalar = RecursiveQuadraticProjectiveScalar {
        numerator: x.add(&c).unwrap(),
        denominator: field.constant(real(2)).unwrap(),
    };
    let expected = ((root_of(2) + cube_root_two_real()) / real(2)).unwrap();
    let Classification::Decided(represented) = scalar.represented_value(&policy).unwrap() else {
        panic!("the projected image is representable");
    };
    assert_eq!(
        hypersolve::validate_algebraic_root_representation(
            &represented,
            hyperlimit::PredicatePolicy::STRICT
        )
        .status,
        hypersolve::AlgebraicRootValidationStatus::Valid
    );
    assert_real_eq(
        &Real::eval_poly(&represented.polynomial_coefficients, &expected),
        &Real::zero(),
    );
    assert_encloses(
        &enclosure(&represented.interval.lower, &represented.interval.upper),
        &expected,
    );
}

#[test]
fn projective_scalars_with_imaginary_numerators_publish_no_real_value() {
    let policy = TestContext::strict();
    let base = quadratic_base(2, 3);
    let imaginary = base.extension(base.constant(real(-1)).unwrap()).unwrap();
    let scalar = RecursiveQuadraticProjectiveScalar {
        numerator: generator(&imaginary),
        denominator: imaginary.constant(real(2)).unwrap(),
    };
    // i/2 has no real value, so no decided real root may be published.
    assert!(!matches!(
        scalar.represented_value(&policy),
        Ok(Classification::Decided(_))
    ));
}

#[test]
fn foreign_embeddings_of_deep_towers_skip_shallower_candidates() {
    let policy = TestContext::strict();
    // Source: sqrt(1 + x) and then sqrt(sqrt(1 + x)) over x = sqrt2.
    let (_, first) = sqrt_two_tower();
    let source = first.extension(generator(&first)).unwrap();
    let target = pair_base(constant_tensor(2, real(2)), constant_tensor(2, real(3)));
    let existing = target.extension(target.constant(real(5)).unwrap()).unwrap();
    let Classification::Decided(Some((joined, embedding))) = recursive_embed_foreign_field(
        &source,
        base_arc(&target),
        vec![1],
        existing.clone(),
        &policy,
    )
    .unwrap() else {
        panic!("the source tower embeds");
    };
    // sqrt5, then both source generators.
    assert_eq!(joined.base_and_extension_path().1.len(), 3);
    assert_eq!(embedding.extensions.len(), 2);
    let image = embedding.value(&generator(&source), &joined).unwrap();
    assert_value(&image, &Tower::second_closed_form());
    assert_value(&joined.lift(&generator(&existing)).unwrap(), &root_of(5));
}

#[test]
fn foreign_embeddings_with_undecidable_generator_equality_stay_undecided() {
    let policy = TestContext::strict();
    let source_base = sqrt_two_base();
    let source = source_base
        .extension(source_base.constant(real(2) + undecidable_zero()).unwrap())
        .unwrap();
    let target = pair_base(constant_tensor(2, real(2)), constant_tensor(2, real(3)));
    let existing = target.extension(target.constant(real(2)).unwrap()).unwrap();
    assert!(
        recursive_embed_foreign_field(
            &source,
            base_arc(&target),
            vec![1],
            existing.clone(),
            &policy
        )
        .unwrap()
        .is_uncertain()
    );
    // Generator identity is exact evidence under either query policy.
    let approximate = TestContext::approximate();
    assert!(
        recursive_embed_foreign_field(&source, base_arc(&target), vec![1], existing, &approximate)
            .unwrap()
            .is_uncertain()
    );
    assert_eq!(approximate.approximations(), 0);
}

#[test]
fn foreign_embeddings_with_undecidable_speed_signs_stay_undecided() {
    let policy = TestContext::strict();
    let target = cube_field(&[real(1), real(0), real(1)], &[real(0), real(0), real(1)]);
    // u*c has an undecidable sign, so no positive root can be selected.
    let undecided = cube_field(&[real(0), undecidable_zero()], &[real(1)]);
    assert!(
        recursive_embed_foreign_field(
            &undecided,
            base_arc(&target),
            vec![0],
            target.clone(),
            &policy
        )
        .unwrap()
        .is_uncertain()
    );
    let second = cube_field(&[real(1)], &[real(0), undecidable_zero()]);
    assert!(
        recursive_embed_foreign_field(&second, base_arc(&target), vec![0], target, &policy)
            .unwrap()
            .is_uncertain()
    );
}

#[test]
fn foreign_base_roots_select_the_positive_polynomial_sheet() {
    let policy = TestContext::strict();
    let target = cube_field(&[real(1), real(0), real(1)], &[real(0), real(0), real(1)]);
    // (1 - c)^2 has the polynomial root 1 - c < 0; the positive root is c - 1.
    let speed = axis_tensor(&[real(1), real(-2), real(1)]);
    let Classification::Decided(Some((field, root))) = recursive_foreign_base_root(
        speed.clone(),
        &base_arc(&target),
        target.clone(),
        None,
        &policy,
    )
    .unwrap() else {
        panic!("a polynomial square root is selected");
    };
    assert!(field.same_field(&target));
    let square = recursive_quadratic_rational_value(&base_arc(&target), speed).unwrap();
    assert!(
        root.square()
            .unwrap()
            .subtract(&square)
            .unwrap()
            .is_structurally_zero()
    );
    assert_encloses(
        &root.interval(16).unwrap(),
        &(cube_root_two_real() - real(1)),
    );
    assert_sign(root.sign(&policy), RealSign::Positive);
}

#[test]
fn joins_in_a_bounded_pass_certify_hidden_generator_equality() {
    let policy = TestContext::strict();
    // E = F(sqrt(c^2)); sqrt(1 + g) and sqrt(1 + c) are the same root.
    let field = cube_field(&[real(1)], &[real(1)]);
    let c = cube(&field);
    let extension = field.extension(c.square().unwrap()).unwrap();
    let one = extension.constant(real(1)).unwrap();
    let first = extension
        .extension(generator(&extension).add(&one).unwrap())
        .unwrap();
    let second = extension
        .extension(extension.lift(&c).unwrap().add(&one).unwrap())
        .unwrap();
    for bounded in [false, true] {
        let joined = if bounded {
            policy.bounded_exact_predicate_pass(|| first.joined_with(&second, &policy))
        } else {
            first.joined_with(&second, &policy)
        }
        .unwrap();
        match joined {
            Classification::Decided(Some((joined, embeddings))) => {
                assert!(joined.same_field(&first));
                assert!(Arc::ptr_eq(&embeddings[0].target, &extension_arc(&first)));
            }
            Classification::Decided(None) => panic!("equal generators must join"),
            // Only a bounded pass may decline; it never adjoins a second
            // copy of the same root.
            Classification::Uncertain(reason) => {
                assert!(bounded, "a complete pass declined: {reason:?}");
            }
        }
    }
}

#[test]
fn ordered_field_context_separates_exact_witness_zeros() {
    let field = quadratic_base(2, 8);
    let mut context = ordered_context(&field);
    // 2*sqrt2 - sqrt8 has no stored cancellation but an exact zero witness.
    let zero = int_value(&field, [0, 2, -1, 0]);
    assert_eq!(
        context.sign_if_separated(&zero).unwrap(),
        Some(Ordering::Equal)
    );
    // A hidden zero without a witness is not separated cheaply.
    let cube_field = RecursiveQuadraticField::base(
        vec![cube_root_two()],
        constant_tensor(1, real(2)),
        constant_tensor(1, real(8)),
    )
    .unwrap();
    let hidden = axis_value(
        &cube_field,
        &[real(0), real(1)],
        &[real(2)],
        &[real(-1)],
        &[real(0)],
    )
    .unwrap()
    .subtract(&cube(&cube_field))
    .unwrap();
    let mut context = ordered_context(&cube_field);
    assert!(matches!(
        context.sign_if_separated(&hidden).unwrap(),
        None | Some(Ordering::Equal)
    ));
    assert_eq!(context.sign(&hidden).unwrap(), Ordering::Equal);
}
