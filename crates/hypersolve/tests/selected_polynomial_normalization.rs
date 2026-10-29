//! Common positive scaling preserves root sets and oriented sign decisions.

#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

#[path = "support/selected_polynomial_replay.rs"]
mod codec;

use hyperreal::{Real, RealSign};
use hypersolve::classification::Classification;
use hypersolve::ordered_field_roots::OrderedFieldPolynomialContext;
use hypersolve::recursive_quadratic_field::RecursiveQuadraticField;
use hypersolve::selected_polynomial_field::{
    SelectedPolynomialContext, SelectedPolynomialDefinition, SelectedPolynomialError,
    SelectedPolynomialField, SelectedPolynomialValue,
};
use support::*;

fn base() -> SelectedPolynomialField {
    SelectedPolynomialField::from_recursive(
        RecursiveQuadraticField::base(
            Vec::new(),
            constant_tensor(0, real(1)),
            constant_tensor(0, real(1)),
        )
        .unwrap(),
    )
}
fn decided<T: std::fmt::Debug>(result: Classification<T>) -> T {
    match result {
        Classification::Decided(value) => value,
        other => panic!("expected certified result: {other:?}"),
    }
}
fn coefficients(field: &SelectedPolynomialField, values: &[i32]) -> Vec<SelectedPolynomialValue> {
    values
        .iter()
        .map(|value| field.constant((*value).into()).unwrap())
        .collect()
}
fn check(field: &SelectedPolynomialField, original: Vec<SelectedPolynomialValue>, roots: usize) {
    let policy = TestContext::strict();
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    let definition = || SelectedPolynomialDefinition::from_values(&original, 10000).unwrap();
    let original_definition = codec::encode(&definition());
    let mut normalized = original.clone();
    context.normalize_positive_scale(&mut normalized);
    let anchor = original
        .iter()
        .position(|value| decided(value.sign(&policy).unwrap()) != RealSign::Zero)
        .unwrap();
    let factor = decided(
        normalized[anchor]
            .divide(&original[anchor], &policy)
            .unwrap(),
    );
    assert_eq!(decided(factor.sign(&policy).unwrap()), RealSign::Positive);
    for (before, after) in original.iter().zip(&normalized) {
        assert_eq!(
            decided(
                after
                    .subtract(&before.multiply(&factor).unwrap())
                    .unwrap()
                    .sign(&policy)
                    .unwrap()
            ),
            RealSign::Zero
        );
        assert_eq!(after.sign(&policy).unwrap(), before.sign(&policy).unwrap());
        assert!(after.field().shares_authority(field));
    }
    for polynomial in [&original, &normalized] {
        assert_eq!(
            hypersolve::ordered_field_count_roots(polynomial, &real(-3), &real(3), &mut context)
                .unwrap(),
            Some(roots)
        );
    }
    // The original relation and selector remain the replay authority.
    assert_eq!(codec::encode(&definition()), original_definition);
    let replayed = decided(
        codec::decode(&original_definition)
            .replay(10000, &policy)
            .unwrap(),
    );
    for (before, after) in original.iter().zip(replayed) {
        let join = decided(before.field().joined_with(after.field(), &policy).unwrap());
        assert_eq!(
            decided(
                join.embed(before)
                    .unwrap()
                    .subtract(&join.embed(&after).unwrap())
                    .unwrap()
                    .sign(&policy)
                    .unwrap()
            ),
            RealSign::Zero
        );
    }
    assert!(matches!(
        normalized[anchor].divide(&field.constant(real(0)).unwrap(), &policy),
        Err(SelectedPolynomialError::ZeroDenominator)
    ));
}

#[test]
fn rational_denominators_preserve_two_roots_and_zero_coefficients() {
    let field = base();
    check(
        &field,
        [(-1, 6), (0, 1), (1, 24), (0, 7)]
            .map(|(n, d)| field.constant((real(n) / real(d)).unwrap()).unwrap())
            .to_vec(),
        2,
    );
}

#[test]
fn dependent_local_denominators_preserve_repeated_roots_and_signs() {
    let q = base();
    let policy = TestContext::strict();
    // (a^3-2)(a-3), selecting the cubic root, retains the other component.
    let first = decided(
        q.select(
            coefficients(&q, &[6, -2, 0, -3, 1]),
            real(1),
            real(2),
            &policy,
        )
        .unwrap(),
    );
    let a = first.generator().unwrap();
    let field = decided(
        first
            .select(
                vec![
                    a.scale(&real(-1)).unwrap(),
                    first.constant(real(0)).unwrap(),
                    first.constant(real(1)).unwrap(),
                ],
                real(1),
                real(2),
                &policy,
            )
            .unwrap(),
    );
    let a = field.lift(&a).unwrap();
    let b = field.generator().unwrap();
    let a2 = a.multiply(&a).unwrap();
    let twice_a = a.scale(&real(2)).unwrap();
    // (t-a)^2(t+1), divided by a known negative selected-local denominator.
    let denominator = a
        .subtract(&field.constant(real(3)).unwrap())
        .unwrap()
        .multiply(&b)
        .unwrap();
    let polynomial = vec![
        a2.clone(),
        a2.subtract(&twice_a).unwrap(),
        field.constant(real(1)).unwrap().subtract(&twice_a).unwrap(),
        field.constant(real(1)).unwrap(),
        field.constant(real(0)).unwrap(),
    ]
    .into_iter()
    .map(|value| decided(value.divide(&denominator, &policy).unwrap()))
    .collect();
    check(&field, polynomial, 2);
}

#[test]
fn selected_zero_coefficients_keep_nearby_signs_and_selector_validation() {
    use hypersolve::{IsolatedRootInterval, ordered_field_sign_at_selected_root};
    use std::cmp::Ordering;

    let q = base();
    let strict = TestContext::strict();
    // Retain (a^3-2)(a-3), selecting a=cuberoot(2). The polynomial
    // a^3-2 is zero on that selection, without being the stored relation.
    let field = decided(
        q.select(
            coefficients(&q, &[6, -2, 0, -3, 1]),
            real(1),
            real(2),
            &strict,
        )
        .unwrap(),
    );
    let a = field.generator().unwrap();
    let zero = a
        .multiply(&a)
        .unwrap()
        .multiply(&a)
        .unwrap()
        .subtract(&field.constant(real(2)).unwrap())
        .unwrap();
    let defining = vec![
        a.scale(&real(-1)).unwrap(),
        field.constant(real(0)).unwrap(),
        field.constant(real(1)).unwrap(),
    ];
    // This admission proves the singleton used by the shared sign query.
    let selected = decided(
        field
            .select(defining.clone(), real(1), real(2), &strict)
            .unwrap(),
    );
    let interval = IsolatedRootInterval {
        lower: selected.isolating_bounds().unwrap()[0].clone(),
        upper: selected.isolating_bounds().unwrap()[1].clone(),
        exact_root: None,
        distinct_root_count: 1,
    };
    let epsilon = (real(1) / Real::from(1_u128 << 100)).unwrap();
    for policy in [TestContext::strict(), TestContext::approximate()] {
        let original: Vec<_> = (1..=5)
            .map(|offset| {
                let denominator = a.add(&field.constant(real(offset)).unwrap()).unwrap();
                decided(zero.divide(&denominator, &policy).unwrap())
            })
            .collect();
        let mut context = SelectedPolynomialContext {
            field: field.clone(),
            policy: &policy,
        };
        for (sign, expected) in [
            (-1, Ordering::Less),
            (0, Ordering::Equal),
            (1, Ordering::Greater),
        ] {
            let mut predicate = original.clone();
            predicate[0] = predicate[0]
                .add(&field.constant(&epsilon * real(sign)).unwrap())
                .unwrap();
            assert_eq!(
                ordered_field_sign_at_selected_root(&defining, &predicate, &interval, &mut context)
                    .unwrap(),
                Some(expected)
            );
        }
        // A zero predicate cannot conceal a defining root at a selector end.
        assert_eq!(
            ordered_field_sign_at_selected_root(
                &coefficients(&field, &[-1, 1]),
                &original,
                &interval,
                &mut context
            )
            .unwrap(),
            None
        );
        assert_eq!(policy.approximations(), 0);
    }
}
