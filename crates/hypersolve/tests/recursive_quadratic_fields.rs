//! Construction, arithmetic, embedding and join contracts of recursive
//! quadratic coefficient fields.
//!
//! Oracles are independent exact closed forms built directly in `Real`
//! (for example `sqrt(1 + sqrt(2))`), compared under STRICT predicates, and
//! algebraic identities such as `generator^2 == radicand` and
//! `(a + b) - b == a`. No expected value is derived from the function under
//! test.

#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

use std::sync::Arc;

use hyperreal::{Real, RealSign};
use hypersolve::classification::{Classification, UncertaintyReason};
use hypersolve::radical_expression::TwoSquareRootExpression;
use hypersolve::recursive_quadratic_field::*;
use hypersolve::{BivariatePolynomial, DenseTensorPolynomial};
use support::*;

#[test]
fn base_rejects_speed_polynomials_whose_rank_differs_from_the_tuple() {
    let one_axis = constant_tensor(1, real(2));
    let no_axis = constant_tensor(0, real(3));
    assert!(RecursiveQuadraticField::base(Vec::new(), one_axis.clone(), no_axis.clone()).is_none());
    assert!(RecursiveQuadraticField::base(Vec::new(), no_axis.clone(), one_axis.clone()).is_none());
    assert!(
        RecursiveQuadraticField::base(vec![sqrt_two_root()], no_axis.clone(), one_axis.clone())
            .is_none()
    );
    let field = RecursiveQuadraticField::base(vec![sqrt_two_root()], one_axis.clone(), one_axis)
        .expect("matching ranks");
    assert!(matches!(field, RecursiveQuadraticField::Base(_)));
}

#[test]
fn base_retains_exact_source_witnesses_only_where_they_are_proved() {
    let point = hypersolve::AlgebraicRootRepresentation::from_exact_value(&fraction(1, 2));
    let field = RecursiveQuadraticField::base(
        vec![point, sqrt_two_root(), cube_root_two()],
        constant_tensor(3, real(1)),
        constant_tensor(3, real(1)),
    )
    .expect("matching ranks");
    let base = base_arc(&field);
    assert_eq!(base.source_real_witnesses.len(), 3);
    assert_real_eq(
        base.source_real_witnesses[0]
            .as_ref()
            .expect("point witness"),
        &fraction(1, 2),
    );
    // The quadratic source has a proved compact witness; the cubic does not.
    assert_real_eq(
        base.source_real_witnesses[1]
            .as_ref()
            .expect("quadratic witness"),
        &root_of(2),
    );
    assert!(base.source_real_witnesses[2].is_none());
}

#[test]
fn same_field_is_identity_not_structure() {
    let first = quadratic_base(2, 3);
    let second = quadratic_base(2, 3);
    assert!(first.same_field(&first.clone()));
    assert!(!first.same_field(&second));
    assert!(recursive_quadratic_bases_equivalent(
        &base_arc(&first),
        &base_arc(&second)
    ));
    assert!(!recursive_quadratic_bases_equivalent(
        &base_arc(&first),
        &base_arc(&quadratic_base(2, 5))
    ));
    assert!(!recursive_quadratic_bases_equivalent(
        &base_arc(&first),
        &base_arc(&quadratic_base(5, 3))
    ));

    let tower = Tower::over(first.clone());
    let other = Tower::over(first.clone());
    assert!(tower.first.same_field(&tower.first.clone()));
    assert!(!tower.first.same_field(&other.first));
    assert!(!tower.first.same_field(&first));
    assert!(!first.same_field(&tower.first));
}

#[test]
fn extension_requires_a_radicand_from_the_parent_field() {
    let base = quadratic_base(2, 3);
    let foreign = quadratic_base(2, 3);
    assert!(base.extension(int_value(&foreign, [1, 1, 0, 0])).is_none());
    let tower = Tower::over(base.clone());
    // A base radicand does not belong to the extension itself.
    assert!(
        tower
            .first
            .extension(int_value(&base, [2, 0, 0, 0]))
            .is_none()
    );
}

#[test]
fn base_and_extension_path_lists_generators_from_the_base_outward() {
    let tower = Tower::new();
    let (base, path) = tower.second.base_and_extension_path();
    assert!(Arc::ptr_eq(&base, &base_arc(&tower.base)));
    assert_eq!(path.len(), 2);
    assert!(Arc::ptr_eq(&path[0], &extension_arc(&tower.first)));
    assert!(Arc::ptr_eq(&path[1], &extension_arc(&tower.second)));
    assert!(tower.base.base_and_extension_path().1.is_empty());
}

#[test]
fn tower_generators_square_to_their_radicands() {
    let tower = Tower::new();
    let first = generator(&tower.first);
    let second = generator(&tower.second);
    assert_value(&first, &Tower::first_closed_form());
    assert_value(&second, &Tower::second_closed_form());

    let radicand = tower
        .first
        .lift(&int_value(&tower.base, [1, 1, 0, 0]))
        .expect("base values lift");
    let square = first.square().expect("square");
    assert!(
        square
            .subtract(&radicand)
            .expect("same field")
            .is_structurally_zero()
    );
    assert_value(&square, &(real(1) + root_of(2)));

    let second_square = second.square().expect("square");
    let lifted_first = tower.second.lift(&first).expect("lift");
    assert!(
        second_square
            .subtract(&lifted_first)
            .expect("same field")
            .is_structurally_zero()
    );
    // g2^4 = 1 + sqrt2.
    let fourth = second_square.square().expect("square");
    assert_value(&fourth, &(real(1) + root_of(2)));
}

#[test]
fn constants_and_lifts_preserve_exact_values() {
    let tower = Tower::new();
    for field in [&tower.base, &tower.first, &tower.second] {
        let zero = field.constant(Real::zero()).expect("zero");
        assert!(zero.is_coefficientwise_stored_zero());
        assert!(zero.is_structurally_zero());
        assert_value(&zero, &Real::zero());
        let one = field.constant(Real::one()).expect("one");
        assert!(one.is_coefficientwise_stored_one());
        assert!(!one.is_coefficientwise_stored_zero());
        let value = field.constant(fraction(-7, 3)).expect("constant");
        assert_value(&value, &fraction(-7, 3));
        assert!(!value.is_coefficientwise_stored_one());
        assert!(!value.is_structurally_zero());
    }

    let base_value = int_value(&tower.base, [1, -2, 3, 1]);
    let expected = base_closed_form(2, 3, [1, -2, 3, 1]);
    let lifted = tower
        .second
        .lift(&base_value)
        .expect("lift through the tower");
    assert!(lifted.field().same_field(&tower.second));
    assert_value(&lifted, &expected);
    // Lifting into the same field is the identity.
    assert!(Arc::ptr_eq(
        &tower.base.lift(&base_value).expect("same field").data,
        &base_value.data
    ));
    // A base field cannot receive a value of its own extension or of a
    // foreign base.
    assert!(tower.base.lift(&generator(&tower.first)).is_none());
    assert!(
        quadratic_base(2, 3)
            .lift(&int_value(&tower.base, [1, 0, 0, 0]))
            .is_none()
    );
    assert!(tower.first.lift(&generator(&Tower::new().first)).is_none());
}

#[test]
fn element_requires_an_extension_and_parent_coefficients() {
    let tower = Tower::new();
    let one = tower.base.constant(Real::one()).expect("one");
    assert!(tower.base.element(one.clone(), one.clone()).is_none());
    let value = tower
        .first
        .element(
            int_value(&tower.base, [3, 0, 0, 0]),
            int_value(&tower.base, [0, 0, 1, 0]),
        )
        .expect("parent coefficients");
    // 3 + sqrt3 * sqrt(1 + sqrt2).
    assert_value(&value, &(real(3) + root_of(3) * Tower::first_closed_form()));
    let foreign = quadratic_base(2, 3).constant(Real::one()).expect("one");
    assert!(tower.first.element(foreign.clone(), one.clone()).is_none());
    assert!(tower.first.element(one, foreign.clone()).is_none());
    assert!(
        RecursiveQuadraticValue::from_extension(
            extension_arc(&tower.first),
            foreign.clone(),
            foreign
        )
        .is_none()
    );
}

#[test]
fn from_base_rejects_rank_mismatches_and_reduces_at_the_tuple() {
    let field = RecursiveQuadraticField::base(
        vec![sqrt_two_root()],
        constant_tensor(1, real(1)),
        constant_tensor(1, real(1)),
    )
    .expect("rank one");
    let base = base_arc(&field);
    let rank_zero = constant_tensor(0, real(1));
    let rank_one = constant_tensor(1, real(1));
    for index in 0..4 {
        let mut parts = [
            rank_one.clone(),
            rank_one.clone(),
            rank_one.clone(),
            rank_one.clone(),
        ];
        parts[index] = rank_zero.clone();
        let [rational, first, second, product] = parts;
        assert!(
            RecursiveQuadraticValue::from_base(
                base.clone(),
                TwoSquareRootExpression {
                    rational,
                    first,
                    second,
                    product,
                },
            )
            .is_none()
        );
    }
    // x^2 at the selected root x = sqrt2 reduces to the exact constant 2.
    let square = axis_value(
        &field,
        &[real(0), real(0), real(1)],
        &[real(0)],
        &[real(0)],
        &[real(0)],
    )
    .expect("value");
    assert_value(&square, &real(2));
    let difference = square
        .subtract(&field.constant(real(2)).expect("constant"))
        .expect("same field");
    assert!(difference.is_structurally_zero());
}

#[test]
fn base_arithmetic_matches_exact_closed_forms() {
    let field = quadratic_base(2, 3);
    let cases = [[1, 2, -1, 3], [0, 1, 1, 0], [-4, 0, 2, -1], [5, -3, 0, 2]];
    for left in cases {
        for right in cases {
            let a = int_value(&field, left);
            let b = int_value(&field, right);
            let expected_a = base_closed_form(2, 3, left);
            let expected_b = base_closed_form(2, 3, right);
            assert_value(&a.add(&b).expect("sum"), &(&expected_a + &expected_b));
            assert_value(
                &a.subtract(&b).expect("difference"),
                &(&expected_a - &expected_b),
            );
            assert_value(
                &a.multiply(&b).expect("product"),
                &(&expected_a * &expected_b),
            );
            // (a + b) - b == a, with an exact structural zero residual.
            let round_trip = a.add(&b).unwrap().subtract(&b).unwrap();
            assert!(round_trip.subtract(&a).unwrap().is_structurally_zero());
        }
        let a = int_value(&field, left);
        let expected = base_closed_form(2, 3, left);
        assert_value(&a.square().expect("square"), &(&expected * &expected));
        assert_value(
            &a.scale(&fraction(-3, 2)).expect("scale"),
            &(&expected * fraction(-3, 2)),
        );
    }
}

#[test]
fn arithmetic_shortcuts_preserve_identity_and_reject_foreign_operands() {
    let field = quadratic_base(2, 3);
    let value = int_value(&field, [1, 2, 0, -1]);
    let zero = field.constant(Real::zero()).unwrap();
    let one = field.constant(Real::one()).unwrap();
    assert!(Arc::ptr_eq(&zero.add(&value).unwrap().data, &value.data));
    assert!(Arc::ptr_eq(&value.add(&zero).unwrap().data, &value.data));
    assert!(Arc::ptr_eq(
        &value.subtract(&zero).unwrap().data,
        &value.data
    ));
    assert!(Arc::ptr_eq(
        &value.multiply(&one).unwrap().data,
        &value.data
    ));
    assert!(Arc::ptr_eq(
        &one.multiply(&value).unwrap().data,
        &value.data
    ));
    assert!(
        value
            .multiply(&zero)
            .unwrap()
            .is_coefficientwise_stored_zero()
    );
    assert!(
        zero.multiply(&value)
            .unwrap()
            .is_coefficientwise_stored_zero()
    );
    assert!(Arc::ptr_eq(
        &value.scale(&Real::one()).unwrap().data,
        &value.data
    ));
    assert!(
        value
            .scale(&Real::zero())
            .unwrap()
            .is_coefficientwise_stored_zero()
    );
    assert!(zero.square().unwrap().is_coefficientwise_stored_zero());
    assert!(one.square().unwrap().is_coefficientwise_stored_one());
    // Multiplying a value by itself is its square.
    let expected = base_closed_form(2, 3, [1, 2, 0, -1]);
    assert_value(&value.multiply(&value).unwrap(), &(&expected * &expected));

    let foreign = int_value(&quadratic_base(2, 3), [1, 0, 0, 0]);
    assert!(value.add(&foreign).is_none());
    assert!(value.subtract(&foreign).is_none());
    assert!(value.multiply(&foreign).is_none());
    assert!(value.extension_cross_term(&value).is_none());
    let tower = Tower::over(field.clone());
    let lifted = tower.first.lift(&value).unwrap();
    assert!(lifted.add(&value).is_none());
    assert!(lifted.multiply(&value).is_none());
}

#[test]
fn extension_arithmetic_matches_nested_radical_closed_forms() {
    let tower = Tower::new();
    let g1 = Tower::first_closed_form();
    let g2 = Tower::second_closed_form();
    // a = 1 + sqrt3 * g1, b = sqrt2 - g1.
    let a = tower
        .first
        .element(
            int_value(&tower.base, [1, 0, 0, 0]),
            int_value(&tower.base, [0, 0, 1, 0]),
        )
        .unwrap();
    let b = tower
        .first
        .element(
            int_value(&tower.base, [0, 1, 0, 0]),
            int_value(&tower.base, [-1, 0, 0, 0]),
        )
        .unwrap();
    let expected_a = real(1) + root_of(3) * &g1;
    let expected_b = root_of(2) - &g1;
    assert_value(&a.add(&b).unwrap(), &(&expected_a + &expected_b));
    assert_value(&a.subtract(&b).unwrap(), &(&expected_a - &expected_b));
    assert_value(&a.multiply(&b).unwrap(), &(&expected_a * &expected_b));
    assert_value(&a.square().unwrap(), &(&expected_a * &expected_a));
    assert_value(&a.scale(&real(-2)).unwrap(), &(&expected_a * real(-2)));
    // The cross term of (r1 + s1 g)(r2 + s2 g) is r1 s2 + s1 r2.
    let cross = a.extension_cross_term(&b).unwrap();
    assert_value(&cross, &(real(-1) + root_of(3) * root_of(2)));
    assert!(
        a.extension_cross_term(&generator(&Tower::new().first))
            .is_none()
    );

    // Depth-two arithmetic: (g2 + 1)(g2 - 1) = g2^2 - 1 = g1 - 1.
    let second = generator(&tower.second);
    let one = tower.second.constant(Real::one()).unwrap();
    let product = second
        .add(&one)
        .unwrap()
        .multiply(&second.subtract(&one).unwrap())
        .unwrap();
    assert_value(&product, &(&g2 * &g2 - real(1)));
    assert_value(&product, &(&g1 - real(1)));
}

#[test]
fn stored_equivalence_is_an_exact_certificate() {
    let field = quadratic_base(2, 3);
    let value = int_value(&field, [1, 2, 3, 4]);
    assert!(value.is_stored_equivalent_to(&value.clone()));
    assert!(value.is_stored_equivalent_to(&int_value(&field, [1, 2, 3, 4])));
    // A rational coefficient written as an unreduced quotient.
    let quotient = base_value(
        &field,
        [(real(2) / real(2)).unwrap(), real(2), real(3), real(4)],
    );
    assert!(value.is_stored_equivalent_to(&quotient));
    assert!(!value.is_stored_equivalent_to(&int_value(&field, [1, 2, 3, 5])));
    // Values of different fields with different values are not equivalent.
    assert!(!value.is_stored_equivalent_to(&int_value(&quadratic_base(2, 3), [1, 2, 3, 5])));

    // Speeds 2 and 8 give two stored forms of 2*sqrt2 = sqrt8; the exact
    // witnesses identify them even though their coefficients differ.
    let dependent = quadratic_base(2, 8);
    let first = int_value(&dependent, [0, 2, 0, 0]);
    let second = int_value(&dependent, [0, 0, 1, 0]);
    assert!(first.is_stored_equivalent_to(&second));
    assert!(!first.is_stored_equivalent_to(&int_value(&dependent, [0, 0, 2, 0])));

    let tower = Tower::over(field.clone());
    let a = tower.first.element(value.clone(), value.clone()).unwrap();
    let b = tower
        .first
        .element(int_value(&field, [1, 2, 3, 4]), quotient.clone())
        .unwrap();
    assert!(a.is_stored_equivalent_to(&b));
    assert!(!a.is_stored_equivalent_to(&tower.first.lift(&value).unwrap()));
    assert!(!a.is_stored_equivalent_to(&value));
    assert!(!value.is_stored_equivalent_to(&a));
}

#[test]
fn stored_equivalence_never_identifies_different_unwitnessed_values() {
    let field = RecursiveQuadraticField::base(
        vec![cube_root_two()],
        constant_tensor(1, real(2)),
        constant_tensor(1, real(8)),
    )
    .unwrap();
    // c + 2*sqrt2 and c + sqrt8 are equal; c + 3*sqrt2 is not. The cube
    // root has no compact witness, so only a positive certificate counts.
    let first = axis_value(
        &field,
        &[real(0), real(1)],
        &[real(2)],
        &[real(0)],
        &[real(0)],
    )
    .unwrap();
    let second = axis_value(
        &field,
        &[real(0), real(1)],
        &[real(0)],
        &[real(1)],
        &[real(0)],
    )
    .unwrap();
    let different = axis_value(
        &field,
        &[real(0), real(1)],
        &[real(3)],
        &[real(0)],
        &[real(0)],
    )
    .unwrap();
    assert!(first.exact_real_value_with_retained_witnesses().is_none());
    assert!(!first.is_stored_equivalent_to(&different));
    assert!(!second.is_stored_equivalent_to(&different));
    assert!(first.is_stored_equivalent_to(&first.clone()));
}

#[test]
fn exact_witnesses_follow_the_tower_and_cache() {
    let tower = Tower::new();
    let value = tower
        .second
        .element(
            tower.first.constant(real(2)).unwrap(),
            tower.first.constant(Real::zero()).unwrap(),
        )
        .unwrap();
    // A zero radical part reuses the retained witness directly.
    assert_value(&value, &real(2));
    assert!(value.real_witness().get().is_some());
    let fresh = generator(&tower.second);
    assert!(fresh.real_witness().get().is_none());
    let computed = fresh.compute_exact_real_witness().unwrap();
    assert_real_eq(&computed, &Tower::second_closed_form());
    // compute does not populate the cache; the retained accessor does.
    assert!(fresh.real_witness().get().is_none());
    assert_value(&fresh, &Tower::second_closed_form());
    assert!(fresh.real_witness().get().is_some());
    assert_value(&fresh, &Tower::second_closed_form());
}

#[test]
fn exact_witnesses_decline_for_negative_radicands_and_unwitnessed_axes() {
    let base = quadratic_base(2, 3);
    // The extension constructor does not certify positivity; the witness
    // evaluator must then refuse the imaginary square root.
    let imaginary = base.extension(int_value(&base, [-1, 0, 0, 0])).unwrap();
    let root = generator(&imaginary);
    assert!(root.exact_real_value_with_retained_witnesses().is_none());
    // A value whose radical part vanishes still has a real witness.
    let rational = imaginary.constant(real(5)).unwrap();
    assert_value(&rational, &real(5));

    let cubic = RecursiveQuadraticField::base(
        vec![cube_root_two()],
        constant_tensor(1, real(1)),
        constant_tensor(1, real(1)),
    )
    .unwrap();
    let axis = cubic.retained_root_value(&cube_root_two()).unwrap();
    assert!(axis.exact_real_value_with_retained_witnesses().is_none());
    // Constants need no source witness.
    assert_value(&cubic.constant(real(3)).unwrap(), &real(3));

    // A negative base speed has no positive real root.
    let negative = quadratic_base(-2, 3);
    assert!(
        int_value(&negative, [0, 1, 0, 0])
            .exact_real_value_with_retained_witnesses()
            .is_none()
    );
    // A zero coefficient for that root never evaluates it.
    assert_value(&int_value(&negative, [1, 0, 1, 0]), &(real(1) + root_of(3)));
}

#[test]
fn retained_root_value_imports_only_existing_axes() {
    let field = RecursiveQuadraticField::base(
        vec![sqrt_two_root(), cube_root_two()],
        constant_tensor(2, real(1)),
        constant_tensor(2, real(1)),
    )
    .unwrap();
    let tower = field.extension(field.constant(real(3)).unwrap()).unwrap();
    let sqrt_two = tower.retained_root_value(&sqrt_two_root()).unwrap();
    assert!(sqrt_two.field().same_field(&tower));
    assert_value(&sqrt_two, &root_of(2));
    // x^2 - 2 vanishes at the imported root.
    let residual = sqrt_two
        .square()
        .unwrap()
        .subtract(&tower.constant(real(2)).unwrap())
        .unwrap();
    assert!(residual.is_structurally_zero());
    let cube = field.retained_root_value(&cube_root_two()).unwrap();
    let cubed = cube.square().unwrap().multiply(&cube).unwrap();
    assert!(
        cubed
            .subtract(&field.constant(real(2)).unwrap())
            .unwrap()
            .is_structurally_zero()
    );
    // A source that is not an axis of the field is not imported.
    assert!(tower.retained_root_value(&cube_root_three()).is_none());
}

#[test]
fn retained_positive_square_root_recovers_authored_generators_only() {
    let tower = Tower::new();
    let radicand = int_value(&tower.base, [1, 1, 0, 0]);
    let root = tower
        .second
        .retained_positive_square_root(&radicand)
        .unwrap();
    assert!(root.field().same_field(&tower.second));
    assert_value(&root, &Tower::first_closed_form());
    // The square of the second generator is the first generator.
    let first = generator(&tower.first);
    let root = tower.second.retained_positive_square_root(&first).unwrap();
    assert_value(&root, &Tower::second_closed_form());
    // A radicand without an authored generator, or from a foreign field.
    assert!(
        tower
            .second
            .retained_positive_square_root(&int_value(&tower.base, [5, 0, 0, 0]))
            .is_none()
    );
    assert!(
        tower
            .first
            .retained_positive_square_root(&int_value(&quadratic_base(2, 3), [1, 1, 0, 0]))
            .is_none()
    );
    assert!(
        tower
            .base
            .retained_positive_square_root(&radicand)
            .is_none()
    );
}

#[test]
fn rebasing_replays_every_generator_over_an_equivalent_base() {
    let tower = Tower::new();
    let target = quadratic_base(2, 3);
    let (rebased, embeddings) = tower
        .second
        .rebased_to_equivalent_base(base_arc(&target))
        .unwrap();
    assert_eq!(embeddings.len(), 2);
    assert!(Arc::ptr_eq(&base_arc(&rebased), &base_arc(&target)));
    assert!(Arc::ptr_eq(
        &embeddings[0].source,
        &extension_arc(&tower.first)
    ));
    assert!(Arc::ptr_eq(
        &embeddings[1].source,
        &extension_arc(&tower.second)
    ));
    assert!(Arc::ptr_eq(&embeddings[1].target, &extension_arc(&rebased)));

    let value = tower
        .second
        .element(
            generator(&tower.first).scale(&real(3)).unwrap(),
            tower
                .first
                .lift(&int_value(&tower.base, [0, 1, 0, 0]))
                .unwrap(),
        )
        .unwrap();
    let expected = real(3) * Tower::first_closed_form() + root_of(2) * Tower::second_closed_form();
    assert_value(&value, &expected);
    let moved = value
        .rebased_to_equivalent_base(base_arc(&target), &embeddings)
        .unwrap();
    assert!(moved.field().same_field(&rebased));
    assert_value(&moved, &expected);

    // Inequivalent bases refuse to rebase.
    let other = quadratic_base(2, 5);
    assert!(
        tower
            .second
            .rebased_to_equivalent_base(base_arc(&other))
            .is_none()
    );
    assert!(
        int_value(&tower.base, [1, 0, 0, 0])
            .rebased_to_equivalent_base(base_arc(&other), &[])
            .is_none()
    );
    // An extension value without its generator correspondence is refused.
    assert!(
        value
            .rebased_to_equivalent_base(base_arc(&target), &[])
            .is_none()
    );
}

#[test]
fn embeddings_into_an_existing_equivalent_tower_require_equal_radicands() {
    let tower = Tower::new();
    let replay = Tower::over(quadratic_base(2, 3));
    let (base, embeddings) = tower
        .first
        .extension_embeddings_to_equivalent_tower(&replay.second)
        .unwrap();
    assert!(Arc::ptr_eq(&base, &base_arc(&replay.base)));
    assert_eq!(embeddings.len(), 1);
    assert!(Arc::ptr_eq(
        &embeddings[0].target,
        &extension_arc(&replay.first)
    ));
    // The rebased generator names the target's authored root and lifts
    // through the deeper target tower.
    let rebased = generator(&tower.first)
        .rebased_to_equivalent_base(base, &embeddings)
        .unwrap();
    assert!(rebased.field().same_field(&replay.first));
    let lifted = replay.second.lift(&rebased).unwrap();
    assert_value(&lifted, &Tower::first_closed_form());
    assert!(
        lifted
            .subtract(&replay.second.lift(&generator(&replay.first)).unwrap())
            .unwrap()
            .is_structurally_zero()
    );

    // A deeper source than target cannot embed.
    assert!(
        tower
            .second
            .extension_embeddings_to_equivalent_tower(&replay.first)
            .is_none()
    );
    // A different radicand names a different generator.
    let other_base = quadratic_base(2, 3);
    let different = other_base
        .extension(int_value(&other_base, [2, 1, 0, 0]))
        .unwrap();
    assert!(
        tower
            .first
            .extension_embeddings_to_equivalent_tower(&different)
            .is_none()
    );
    // An inequivalent base cannot host the tower.
    let inequivalent = Tower::over(quadratic_base(3, 2));
    assert!(
        tower
            .first
            .extension_embeddings_to_equivalent_tower(&inequivalent.first)
            .is_none()
    );
}

#[test]
fn embed_value_declines_values_without_a_correspondence() {
    let tower = Tower::new();
    let foreign = quadratic_base(2, 3);
    assert!(
        tower
            .first
            .embed_value(&int_value(&foreign, [1, 0, 0, 0]), &[])
            .is_none()
    );
    let other = Tower::new();
    assert!(
        tower
            .first
            .embed_value(&generator(&other.first), &[])
            .is_none()
    );
}

#[test]
fn joins_of_foreign_bases_are_decided_absent() {
    let policy = TestContext::strict();
    let first = Tower::new();
    let second = Tower::new();
    let joined = first.first.joined_with(&second.first, &policy).unwrap();
    assert!(matches!(joined, Classification::Decided(None)));
}

#[test]
fn joins_with_shared_prefixes_reuse_or_replay_generators() {
    let policy = TestContext::strict();
    let tower = Tower::new();
    let Classification::Decided(Some((joined, embeddings))) =
        tower.second.joined_with(&tower.first, &policy).unwrap()
    else {
        panic!("a shared prefix joins");
    };
    assert!(joined.same_field(&tower.second));
    assert!(embeddings.is_empty());
    // Joining from the ancestor replays the descendant's generators into a
    // transient tower; it does not adopt the other tower by identity.
    let Classification::Decided(Some((joined, embeddings))) =
        tower.base.joined_with(&tower.second, &policy).unwrap()
    else {
        panic!("a descendant joins");
    };
    assert_eq!(joined.base_and_extension_path().1.len(), 2);
    assert_eq!(embeddings.len(), 2);
    let image = joined
        .embed_value(&generator(&tower.second), &embeddings)
        .unwrap();
    assert_value(&image, &Tower::second_closed_form());
    let first_image = joined
        .embed_value(&generator(&tower.first), &embeddings)
        .unwrap();
    assert!(first_image.field().same_field(&joined));
    assert_value(&first_image, &Tower::first_closed_form());
}

#[test]
fn joins_identify_separately_authored_equal_generators() {
    let policy = TestContext::strict();
    let base = quadratic_base(2, 3);
    let first = base.extension(int_value(&base, [1, 1, 0, 0])).unwrap();
    let second = base.extension(int_value(&base, [1, 1, 0, 0])).unwrap();
    let Classification::Decided(Some((joined, embeddings))) =
        first.joined_with(&second, &policy).unwrap()
    else {
        panic!("equal radicands join");
    };
    // The second generator is the same positive root; nothing is adjoined.
    assert!(joined.same_field(&first));
    assert_eq!(embeddings.len(), 1);
    let image = joined
        .embed_value(&generator(&second), &embeddings)
        .unwrap();
    let difference = image.subtract(&generator(&first)).unwrap();
    assert!(difference.is_structurally_zero());
}

#[test]
fn joins_adjoin_independent_generators_and_keep_both_roots() {
    let policy = TestContext::strict();
    let base = quadratic_base(2, 3);
    let first = base.extension(int_value(&base, [1, 1, 0, 0])).unwrap();
    let second = base.extension(int_value(&base, [2, 0, 1, 0])).unwrap();
    let Classification::Decided(Some((joined, embeddings))) =
        first.joined_with(&second, &policy).unwrap()
    else {
        panic!("independent radicands join");
    };
    assert_eq!(joined.base_and_extension_path().1.len(), 2);
    assert_eq!(embeddings.len(), 1);
    let first_root = joined.lift(&generator(&first)).unwrap();
    let second_root = joined
        .embed_value(&generator(&second), &embeddings)
        .unwrap();
    assert_value(&first_root, &Tower::first_closed_form());
    let expected_second = sqrt(real(2) + root_of(3));
    assert_value(&second_root, &expected_second);
    // Both generators square to their own radicands in the joined field.
    let radicand = joined.lift(&int_value(&base, [2, 0, 1, 0])).unwrap();
    assert!(
        second_root
            .square()
            .unwrap()
            .subtract(&radicand)
            .unwrap()
            .is_structurally_zero()
    );
    let product = first_root.multiply(&second_root).unwrap();
    assert_value(&product, &(Tower::first_closed_form() * expected_second));
}

#[test]
fn joins_prove_equal_generators_whose_stored_radicands_differ() {
    let policy = TestContext::strict();
    let field = RecursiveQuadraticField::base(
        vec![cube_root_two()],
        constant_tensor(1, real(2)),
        constant_tensor(1, real(8)),
    )
    .unwrap();
    // c + 2*sqrt2 == c + sqrt8 with c = 2^(1/3): only an exact sign proof,
    // not stored coefficients, identifies these generators.
    let first_radicand = axis_value(
        &field,
        &[real(0), real(1)],
        &[real(2)],
        &[real(0)],
        &[real(0)],
    )
    .unwrap();
    let second_radicand = axis_value(
        &field,
        &[real(0), real(1)],
        &[real(0)],
        &[real(1)],
        &[real(0)],
    )
    .unwrap();
    let first = field.extension(first_radicand).unwrap();
    let second = field.extension(second_radicand).unwrap();
    let Classification::Decided(Some((joined, embeddings))) =
        first.joined_with(&second, &policy).unwrap()
    else {
        panic!("equal radicands join");
    };
    assert!(joined.same_field(&first));
    assert_eq!(embeddings.len(), 1);
    assert!(Arc::ptr_eq(&embeddings[0].target, &extension_arc(&first)));
}

#[test]
fn bivariate_homogeneous_evaluation_matches_the_closed_form() {
    let tower = Tower::new();
    // P(x, y) = 1 + 2x - 3xy + y^2, homogenized at degree 2:
    // w^2 + 2xw - 3xy + y^2.
    let polynomial = BivariatePolynomial::new(vec![
        vec![real(1), real(0), real(1)],
        vec![real(2), real(-3)],
    ]);
    let x = generator(&tower.first);
    let y = tower
        .first
        .lift(&int_value(&tower.base, [0, 1, 0, 0]))
        .unwrap();
    let w = tower.first.constant(real(2)).unwrap();
    let value = evaluate_bivariate_homogeneous_polynomial(&polynomial, &x, &y, &w, 2).unwrap();
    let gx = Tower::first_closed_form();
    let gy = root_of(2);
    let gw = real(2);
    let expected = &gw * &gw + real(2) * &gx * &gw - real(3) * &gx * &gy + &gy * &gy;
    assert_value(&value, &expected);
    // A homogenizing degree below the total degree is refused.
    assert!(evaluate_bivariate_homogeneous_polynomial(&polynomial, &x, &y, &w, 1).is_none());
    // Coordinates from different fields cannot compose.
    let foreign = Tower::new().first.constant(real(1)).unwrap();
    assert!(evaluate_bivariate_homogeneous_polynomial(&polynomial, &x, &y, &foreign, 2).is_none());
}

#[test]
fn dense_exact_positive_scale_requires_a_positive_coefficientwise_multiple() {
    let reference = axis_tensor(&[real(1), real(2), real(3)]);
    let scaled = axis_tensor(&[real(4), real(8), real(12)]);
    assert_real_eq(
        &dense_exact_positive_scale(&scaled, &reference).unwrap(),
        &real(4),
    );
    assert_real_eq(
        &dense_positive_square_root_scale(&scaled, &reference).unwrap(),
        &real(2),
    );
    let negative = axis_tensor(&[real(-4), real(-8), real(-12)]);
    assert!(dense_exact_positive_scale(&negative, &reference).is_none());
    let skewed = axis_tensor(&[real(4), real(8), real(13)]);
    assert!(dense_exact_positive_scale(&skewed, &reference).is_none());
    let zero = DenseTensorPolynomial::zero(vec![3]).unwrap();
    assert!(dense_exact_positive_scale(&scaled, &zero).is_none());
    assert!(dense_exact_positive_scale(&constant_tensor(0, real(4)), &reference).is_none());
    // The pivot is the first nonzero reference coefficient.
    let shifted = axis_tensor(&[real(0), real(3)]);
    let shifted_scaled = axis_tensor(&[real(0), real(27)]);
    assert_real_eq(
        &dense_positive_square_root_scale(&shifted_scaled, &shifted).unwrap(),
        &real(3),
    );
}

#[test]
fn base_generators_are_the_positive_speed_roots() {
    let field = quadratic_base(5, 7);
    let base = base_arc(&field);
    assert_value(
        &recursive_quadratic_base_generator(&base, true).unwrap(),
        &root_of(5),
    );
    assert_value(
        &recursive_quadratic_base_generator(&base, false).unwrap(),
        &root_of(7),
    );
    let rational = recursive_quadratic_rational_value(&base, constant_tensor(0, real(9))).unwrap();
    assert_value(&rational, &real(9));
    assert!(recursive_quadratic_rational_value(&base, constant_tensor(1, real(9))).is_none());
}

#[test]
fn source_union_deduplicates_equal_sources_in_order() {
    let sqrt_two = sqrt_two_root();
    let cube = cube_root_two();
    let (sources, first_axes, second_axes) = recursive_quadratic_source_union(
        &[sqrt_two.clone(), cube.clone(), sqrt_two.clone()],
        &[cube_root_three(), cube.clone()],
    );
    assert_eq!(sources.len(), 3);
    assert_eq!(first_axes, vec![0, 1, 0]);
    assert_eq!(second_axes, vec![2, 1]);
}

#[test]
fn polynomial_square_roots_are_exact_or_absent() {
    let policy = TestContext::strict();
    // (1 + 2t + 3t^2)^2 = 1 + 4t + 10t^2 + 12t^3 + 9t^4.
    let square = [1, 4, 10, 12, 9].map(real);
    let Classification::Decided(Some(root)) = polynomial_square_root(&square, &policy).unwrap()
    else {
        panic!("perfect square");
    };
    assert_eq!(root.len(), 3);
    for (actual, expected) in root.iter().zip([1, 2, 3]) {
        assert_real_eq(actual, &real(expected));
    }
    // Trailing zero coefficients and an even valuation: t^2 (2 + t)^2 * ...
    // (t * (2 + t))^2 = 4t^2 + 4t^3 + t^4.
    let shifted = [0, 0, 4, 4, 1, 0].map(real);
    let Classification::Decided(Some(root)) = polynomial_square_root(&shifted, &policy).unwrap()
    else {
        panic!("shifted perfect square");
    };
    let replay: Vec<Real> = (0..5)
        .map(|power| {
            let mut sum = Real::zero();
            for left in 0..=power {
                let right = power - left;
                if left < root.len() && right < root.len() {
                    sum += &root[left] * &root[right];
                }
            }
            sum
        })
        .collect();
    for (actual, expected) in replay.iter().zip([0, 0, 4, 4, 1]) {
        assert_real_eq(actual, &real(expected));
    }
    // The zero polynomial has the zero root.
    let Classification::Decided(Some(root)) =
        polynomial_square_root(&[real(0), real(0)], &policy).unwrap()
    else {
        panic!("the zero polynomial is a square");
    };
    assert_eq!(root.len(), 1);
    assert_real_eq(&root[0], &Real::zero());
    // Odd valuation, odd degree, negative constant and non-square inputs.
    for coefficients in [
        vec![real(0), real(1), real(1)],
        vec![real(1), real(1)],
        vec![real(-1), real(0), real(1)],
        vec![real(1), real(0), real(1)],
    ] {
        assert_eq!(
            polynomial_square_root(&coefficients, &policy).unwrap(),
            Classification::Decided(None)
        );
    }
}

#[test]
fn polynomial_projection_combination_and_products_are_exact() {
    let tower = Tower::new();
    let g = generator(&tower.first);
    let one = tower.first.constant(Real::one()).unwrap();
    // (1 + g t)(1 - g t) = 1 - g^2 t^2.
    let left = vec![one.clone(), g.clone()];
    let right = vec![one.clone(), g.scale(&real(-1)).unwrap()];
    let product = recursive_quadratic_polynomial_multiply(&left, &right).unwrap();
    assert_eq!(product.len(), 3);
    assert_value(&product[0], &real(1));
    assert!(product[1].is_structurally_zero());
    assert_value(&product[2], &(-(real(1) + root_of(2))));
    assert!(recursive_quadratic_polynomial_multiply(&[], &right).is_none());
    assert!(recursive_quadratic_polynomial_multiply(&left, &[]).is_none());

    let sum = recursive_quadratic_polynomial_combine(&left, &product, false).unwrap();
    assert_eq!(sum.len(), 3);
    assert_value(&sum[0], &real(2));
    assert_value(&sum[1], &Tower::first_closed_form());
    let difference = recursive_quadratic_polynomial_combine(&product, &left, true).unwrap();
    assert_value(&difference[0], &real(0));
    assert_value(&difference[1], &(-Tower::first_closed_form()));
    assert!(recursive_quadratic_polynomial_combine(&[], &[], false).is_none());

    let scaled = recursive_quadratic_polynomial_scale(&left, &g).unwrap();
    assert_value(&scaled[0], &Tower::first_closed_form());
    assert_value(&scaled[1], &(real(1) + root_of(2)));
}

#[test]
fn polynomial_projection_eliminates_every_generator() {
    let tower = Tower::new();
    // t - g2 has the projected norm whose root set contains g2.
    let g2 = generator(&tower.second);
    let one = tower.second.constant(Real::one()).unwrap();
    let (base, projection) =
        recursive_quadratic_polynomial_projection(vec![g2.scale(&real(-1)).unwrap(), one]).unwrap();
    assert!(Arc::ptr_eq(&base, &base_arc(&tower.base)));
    // Rank-zero base: the projection has one free output axis.
    assert_eq!(projection.dimensions().len(), 1);
    let root = Tower::second_closed_form();
    let value = Real::eval_poly(projection.coefficients(), &root);
    assert_real_eq(&value, &Real::zero());
    // Mixed fields, mixed kinds and empty inputs are refused.
    let lifted = tower.second.lift(&generator(&tower.first)).unwrap();
    assert!(
        recursive_quadratic_polynomial_projection(vec![
            lifted.clone(),
            generator(&Tower::new().second),
        ])
        .is_none()
    );
    assert!(
        recursive_quadratic_polynomial_projection(vec![
            int_value(&tower.base, [1, 0, 0, 0]),
            int_value(&quadratic_base(2, 3), [1, 0, 0, 0]),
        ])
        .is_none()
    );
    assert!(
        recursive_quadratic_polynomial_projection(vec![
            lifted,
            int_value(&tower.base, [1, 0, 0, 0]),
        ])
        .is_none()
    );
    assert!(
        recursive_quadratic_polynomial_projection(vec![
            int_value(&tower.base, [1, 0, 0, 0]),
            tower.first.constant(real(1)).unwrap(),
        ])
        .is_none()
    );
    assert!(recursive_quadratic_polynomial_projection(Vec::new()).is_none());
}

#[test]
fn exact_witnesses_skip_coefficients_that_vanish_at_duplicate_axes() {
    // Two copies of the same selected root x = y = sqrt2: the radical
    // coefficient x*y - 2 is not stored as zero but vanishes at the tuple.
    let field = RecursiveQuadraticField::base(
        vec![sqrt_two_root(), sqrt_two_root()],
        constant_tensor(2, real(3)),
        constant_tensor(2, real(5)),
    )
    .unwrap();
    let vanishing =
        DenseTensorPolynomial::try_new(vec![2, 2], vec![real(-2), real(0), real(0), real(1)])
            .unwrap();
    let value = RecursiveQuadraticValue::from_base(
        base_arc(&field),
        TwoSquareRootExpression {
            rational: constant_tensor(2, real(1)),
            first: vanishing,
            second: constant_tensor(2, real(0)),
            product: constant_tensor(2, real(0)),
        },
    )
    .unwrap();
    assert_value(&value, &real(1));
    assert_sign(value.sign(&TestContext::strict()), RealSign::Positive);
}

#[test]
fn normalize_positive_scale_removes_common_rational_content() {
    let field = quadratic_base(2, 3);
    let mut values = vec![
        int_value(&field, [4, 6, 0, -2]),
        int_value(&field, [8, 0, 10, 0]),
    ];
    let original = values.clone();
    RecursiveQuadraticValue::normalize_positive_scale(&mut values);
    assert_value(&values[0], &base_closed_form(2, 3, [2, 3, 0, -1]));
    assert_value(&values[1], &base_closed_form(2, 3, [4, 0, 5, 0]));
    // The common factor is positive: signs and ratios are preserved.
    let ratio_before = (original[0]
        .exact_real_value_with_retained_witnesses()
        .unwrap()
        * values[1]
            .exact_real_value_with_retained_witnesses()
            .unwrap())
        - original[1]
            .exact_real_value_with_retained_witnesses()
            .unwrap()
            * values[0]
                .exact_real_value_with_retained_witnesses()
                .unwrap();
    assert_real_eq(&ratio_before, &Real::zero());

    // A primitive tuple is left untouched, including its identities.
    let mut primitive = vec![
        int_value(&field, [1, 2, 0, 0]),
        int_value(&field, [3, 0, 0, 1]),
    ];
    let identities: Vec<_> = primitive.iter().map(|value| value.data.clone()).collect();
    RecursiveQuadraticValue::normalize_positive_scale(&mut primitive);
    for (value, identity) in primitive.iter().zip(identities) {
        assert!(Arc::ptr_eq(&value.data, &identity));
    }
    // An all-zero tuple and an irrational coefficient decline atomically.
    let mut zeros = vec![field.constant(Real::zero()).unwrap()];
    RecursiveQuadraticValue::normalize_positive_scale(&mut zeros);
    assert!(zeros[0].is_coefficientwise_stored_zero());
    let irrational = base_value(&field, [root_of(5), real(2), real(0), real(0)]);
    let mut opaque = vec![irrational.clone(), int_value(&field, [2, 0, 0, 0])];
    RecursiveQuadraticValue::normalize_positive_scale(&mut opaque);
    assert!(Arc::ptr_eq(&opaque[0].data, &irrational.data));
}

#[test]
fn normalize_positive_scale_rescales_towers_shared_values_and_witnesses() {
    let tower = Tower::new();
    let shared = tower
        .first
        .element(
            int_value(&tower.base, [6, 0, 0, 0]),
            int_value(&tower.base, [0, 3, 0, 0]),
        )
        .unwrap();
    // Retain a witness before normalization so it is rescaled, not dropped.
    assert_value(
        &shared,
        &(real(6) + real(3) * root_of(2) * Tower::first_closed_form()),
    );
    let mut values = vec![
        shared.clone(),
        shared.clone(),
        tower.first.constant(real(9)).unwrap(),
    ];
    RecursiveQuadraticValue::normalize_positive_scale(&mut values);
    let expected = real(2) + root_of(2) * Tower::first_closed_form();
    assert!(values[0].real_witness().get().is_some());
    assert_value(&values[0], &expected);
    assert_value(&values[1], &expected);
    assert_value(&values[2], &real(3));
    // Shared inputs remain shared after the rebuild.
    assert!(Arc::ptr_eq(&values[0].data, &values[1].data));
}

#[test]
fn projective_scalars_publish_and_order_exact_quotients() {
    let policy = TestContext::strict();
    let tower = Tower::new();
    let scalar = RecursiveQuadraticProjectiveScalar {
        numerator: generator(&tower.first),
        denominator: tower.first.constant(real(2)).unwrap(),
    };
    let expected = (Tower::first_closed_form() / real(2)).unwrap();
    assert_real_eq(&scalar.exact_real_value().unwrap(), &expected);
    let interval = scalar.interval(8).unwrap();
    assert_encloses(&interval, &expected);
    let Classification::Decided(represented) = scalar.represented_value(&policy).unwrap() else {
        panic!("an exact quotient is representable");
    };
    assert_real_eq(represented.exact_point_witness().unwrap(), &expected);
    for (value, ordering) in [
        (fraction(3, 4), std::cmp::Ordering::Greater),
        (fraction(4, 5), std::cmp::Ordering::Less),
    ] {
        assert_eq!(
            scalar.order_to_real(&value, &policy).unwrap(),
            Classification::Decided(ordering)
        );
    }
    // sqrt(1 + sqrt2)/2 compared with itself is equal.
    let self_order = RecursiveQuadraticProjectiveScalar {
        numerator: generator(&tower.first),
        denominator: tower.first.constant(Real::one()).unwrap(),
    };
    assert_eq!(
        self_order
            .order_to_real(&Tower::first_closed_form(), &policy)
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Equal)
    );
}

#[test]
fn projective_scalars_reject_mismatched_fields_and_nonpositive_denominators() {
    let policy = TestContext::strict();
    let tower = Tower::new();
    let mismatched = RecursiveQuadraticProjectiveScalar {
        numerator: generator(&tower.first),
        denominator: tower.base.constant(real(1)).unwrap(),
    };
    assert_eq!(
        mismatched.represented_value(&policy).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    for denominator in [real(0), real(-1)] {
        let invalid = RecursiveQuadraticProjectiveScalar {
            numerator: generator(&tower.first),
            denominator: tower.first.constant(denominator).unwrap(),
        };
        assert!(matches!(
            invalid.represented_value(&policy),
            Err(TestError::Field(_))
        ));
    }
    assert!(mismatched.order_to_real(&real(1), &policy).is_err());
}

#[test]
fn projective_scalars_over_selected_axes_publish_through_projection() {
    let policy = TestContext::strict();
    let field = RecursiveQuadraticField::base(
        vec![cube_root_two()],
        constant_tensor(1, real(1)),
        constant_tensor(1, real(1)),
    )
    .unwrap();
    let cube = field.retained_root_value(&cube_root_two()).unwrap();
    let scalar = RecursiveQuadraticProjectiveScalar {
        numerator: cube.add(&field.constant(real(1)).unwrap()).unwrap(),
        denominator: field.constant(real(3)).unwrap(),
    };
    // A selected axis without a witness cannot bypass root publication.
    assert!(scalar.exact_real_value().is_none());
    let expected = ((cube_root_two_real() + real(1)) / real(3)).unwrap();
    assert_encloses(&scalar.interval(16).unwrap(), &expected);
    let Classification::Decided(represented) = scalar.represented_value(&policy).unwrap() else {
        panic!("the projected image is representable");
    };
    // The published root passes STRICT validation, satisfies its own
    // defining polynomial at the independently computed closed form, and
    // isolates it.
    assert_eq!(
        hypersolve::validate_algebraic_root_representation(
            &represented,
            hyperlimit::PredicatePolicy::STRICT
        )
        .status,
        hypersolve::AlgebraicRootValidationStatus::Valid
    );
    let residual = Real::eval_poly(&represented.polynomial_coefficients, &expected);
    assert_real_eq(&residual, &Real::zero());
    assert_encloses(
        &hypersolve::RealInterval {
            lower: represented.interval.lower.clone(),
            upper: represented.interval.upper.clone(),
        },
        &expected,
    );
    assert_eq!(
        scalar.order_to_real(&fraction(3, 4), &policy).unwrap(),
        Classification::Decided(std::cmp::Ordering::Greater)
    );
    assert_eq!(
        scalar.order_to_real(&fraction(4, 5), &policy).unwrap(),
        Classification::Decided(std::cmp::Ordering::Less)
    );
}

#[test]
fn base_values_over_an_empty_tuple_sign_from_their_closed_forms() {
    let policy = TestContext::strict();
    let field = quadratic_base(2, 3);
    for parts in [[1, 2, -1, 3], [0, 0, 0, 0], [-5, 1, 1, 1], [5, -2, -1, 0]] {
        let value = int_value(&field, parts);
        let expected = strict_sign(&base_closed_form(2, 3, parts)).unwrap();
        assert_sign(value.sign(&policy), expected);
    }
    // sqrt6 - sqrt2*sqrt3 = 0.
    let cancelling = int_value(&field, [0, 0, 0, 1])
        .subtract(
            &int_value(&field, [0, 1, 0, 0])
                .multiply(&int_value(&field, [0, 0, 1, 0]))
                .unwrap(),
        )
        .unwrap();
    assert_sign(cancelling.sign(&policy), RealSign::Zero);
}
