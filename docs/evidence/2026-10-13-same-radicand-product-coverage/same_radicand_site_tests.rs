use super::*;
use crate::same_radicand_diagnostic::Report;

fn coefficient(rational: i64, radical: i64, radicand: i64) -> RadicalCoefficient {
    RadicalCoefficient::rational(Rational::new(rational))
        .add(RadicalCoefficient::radical(
            Rational::new(radical),
            Rational::new(radicand),
        ))
        .unwrap()
}

#[test]
fn equal_radicands_preserve_affine_products_and_exact_cancellation() {
    let left = coefficient(3, 2, 13);
    let right = coefficient(5, 7, 13);
    let conjugate = coefficient(3, -2, 13);
    let pure_left = coefficient(0, 2, 13);
    let pure_right = coefficient(0, 7, 13);
    let report = Report::begin();
    let affine = left.clone().multiply(&right).unwrap();
    let cancelled = left.multiply(&conjugate).unwrap();
    let pure = pure_left.multiply(&pure_right).unwrap();
    let counts = report.finish();
    assert_eq!(affine.rational, Rational::new(197));
    assert_eq!(
        affine.radicals,
        vec![(Rational::new(13), Rational::new(31))]
    );
    assert_eq!(cancelled.rational, Rational::new(-43));
    assert!(cancelled.radicals.is_empty());
    assert_eq!(pure.rational, Rational::new(182));
    assert!(pure.radicals.is_empty());
    assert_eq!(counts.calls, 3);
    assert_eq!(counts.narrow, 3);
    assert_eq!(counts.branches, [0, 3]);
    assert_eq!(counts.outputs, [[0; 4], [2, 1, 0, 0]]);
}

#[test]
fn unequal_radicands_preserve_original_insertion_order() {
    let left = coefficient(1, 1, 2);
    let right = coefficient(1, 1, 3);
    let report = Report::begin();
    let product = left.multiply(&right).unwrap();
    let counts = report.finish();
    assert_eq!(product.rational, Rational::one());
    assert_eq!(
        product.radicals,
        [2, 3, 6]
            .into_iter()
            .map(|root| (Rational::new(root), Rational::one()))
            .collect::<Vec<_>>()
    );
    assert_eq!(counts.calls, 1);
    assert_eq!(counts.narrow, 1);
    assert_eq!(counts.branches, [1, 0]);
    assert_eq!(counts.outputs, [[0, 0, 0, 1], [0; 4]]);
}

#[test]
fn other_shapes_and_opaque_decline_are_counted_without_new_owners() {
    let root = coefficient(0, 1, 13);
    let rational = RadicalCoefficient::rational(Rational::new(3));
    let mut wide = RadicalCoefficient::rational(Rational::zero());
    for prime in [2, 3, 5, 7, 11, 17, 19, 23] {
        wide.insert_radical(Rational::new(prime), Rational::one())
            .unwrap();
    }
    let opaque = Computable::rational(Rational::new(7)).sin();
    let weak = Arc::downgrade(&opaque.internal);
    let atom = RadicalCoefficient::atom(opaque.clone());
    let owners = Arc::strong_count(&opaque.internal);
    let report = Report::begin();
    assert_eq!(
        rational.clone().multiply(&rational).unwrap().rational,
        Rational::new(9)
    );
    assert_eq!(rational.multiply(&root).unwrap().radicals.len(), 1);
    assert_eq!(wide.multiply(&root).unwrap().radicals.len(), 8);
    assert!(atom.multiply(&root).is_none());
    let counts = report.finish();
    assert_eq!(counts.calls, 4);
    assert_eq!(counts.narrow, 0);
    assert_eq!(counts.branches, [0; 2]);
    assert_eq!(counts.outputs, [[0; 4]; 2]);
    assert_eq!(Arc::strong_count(&opaque.internal), owners - 1);
    assert!(opaque.cached().is_none());
    drop(opaque);
    assert!(weak.upgrade().is_none());
}
