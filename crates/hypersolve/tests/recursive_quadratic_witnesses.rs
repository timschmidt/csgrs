//! Exact zero padding must not require unused selected-source witnesses.

#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::Real;
use hypersolve::radical_expression::TwoSquareRootExpression;
use hypersolve::recursive_quadratic_field::{RecursiveQuadraticField, RecursiveQuadraticValue};
use hypersolve::{AlgebraicRootRepresentation, DenseTensorPolynomial};
use support::{constant_tensor, cube_root_two, real, sqrt_two_root};

fn field(sources: Vec<AlgebraicRootRepresentation>) -> RecursiveQuadraticField {
    let one = constant_tensor(sources.len(), real(1));
    RecursiveQuadraticField::base(sources, one.clone(), one).unwrap()
}

fn value(
    field: &RecursiveQuadraticField,
    dimensions: Vec<usize>,
    coefficients: Vec<Real>,
) -> RecursiveQuadraticValue {
    let RecursiveQuadraticField::Base(base) = field else {
        unreachable!()
    };
    RecursiveQuadraticValue::from_base(
        base.clone(),
        TwoSquareRootExpression::from_rational(
            DenseTensorPolynomial::try_new(dimensions, coefficients).unwrap(),
        )
        .unwrap(),
    )
    .unwrap()
}

fn witness(value: &RecursiveQuadraticValue, expected: Real) {
    let actual = value
        .exact_real_value_with_retained_witnesses()
        .expect("zero-padded unused axes must not require a source witness");
    assert_eq!(
        compare_reals(&actual, &expected, PredicatePolicy::STRICT).value(),
        Some(Ordering::Equal)
    );
    assert_eq!(value.real_witness().get(), Some(&actual));
}

#[test]
fn padded_constants_preserve_the_unwitnessed_selected_source() {
    let field = field(vec![cube_root_two()]);
    let RecursiveQuadraticField::Base(base) = &field else {
        unreachable!()
    };
    assert!(base.source_real_witnesses[0].is_none());
    let quarter = (real(1) / real(4)).unwrap();
    for expected in [real(0), quarter, real(-7)] {
        let value = value(&field, vec![3], vec![expected.clone(), real(0), real(0)]);
        witness(&value, expected);
        assert!(value.field().same_field(&field));
    }
}

#[test]
fn selected_relation_reduction_can_remove_every_generator_dependency() {
    let field = field(vec![cube_root_two()]);
    // The original source relation is a^3=2, so 1+a^3 is exactly 3.
    let value = value(&field, vec![4], vec![real(1), real(0), real(0), real(1)]);
    witness(&value, real(3));
    assert!(value.field().same_field(&field));
}

#[test]
fn mixed_tensor_axis_orders_keep_every_nonzero_dependency() {
    let root = real(2).sqrt().unwrap();
    let outer_unknown = field(vec![cube_root_two(), sqrt_two_root()]);
    witness(
        &value(
            &outer_unknown,
            vec![3, 2],
            vec![real(0), real(1), real(0), real(0), real(0), real(0)],
        ),
        root.clone(),
    );
    let inner_unknown = field(vec![sqrt_two_root(), cube_root_two()]);
    witness(
        &value(
            &inner_unknown,
            vec![2, 3],
            vec![real(1), real(0), real(0), real(2), real(0), real(0)],
        ),
        real(1) + real(2) * root,
    );
    let mut epsilon = real(1);
    for _ in 0..80 {
        epsilon = (epsilon / real(2)).unwrap();
    }
    for coefficients in [
        vec![real(0), real(1), real(0), epsilon.clone(), real(0), real(0)],
        vec![real(0), real(1), real(0), real(0), epsilon, real(0)],
    ] {
        let value = value(&outer_unknown, vec![3, 2], coefficients);
        assert!(value.exact_real_value_with_retained_witnesses().is_none());
        assert!(value.real_witness().get().is_none());
        assert!(value.field().same_field(&outer_unknown));
    }
}
