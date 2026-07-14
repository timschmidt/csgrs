#![no_main]

use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    AlgebraicRootKind, AlgebraicRootPolynomialImageStatus, AlgebraicRootRepresentation,
    AlgebraicRootValidationReport, AlgebraicRootValidationStatus, IsolatedRootInterval, SymbolId,
    transform_algebraic_root_polynomial_image,
};
use libfuzzer_sys::fuzz_target;

fn real(value: i64) -> Real {
    Real::from(value)
}

fn represented_rational_root(root: i64) -> AlgebraicRootRepresentation {
    AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![real(-root), Real::one()],
        interval: IsolatedRootInterval {
            lower: real(root),
            upper: real(root),
            exact_root: Some(real(root)),
            distinct_root_count: 1,
        },
        kind: AlgebraicRootKind::ExactRationalWitness,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    }
}

fuzz_target!(|data: [i16; 4]| {
    let root = i64::from(data[0] % 24);
    let constant = i64::from(data[1] % 16);
    let linear = i64::from(data[2] % 16);
    let quadratic = i64::from(data[3] % 16);
    let report = transform_algebraic_root_polynomial_image(
        &represented_rational_root(root),
        &[real(constant), real(linear), real(quadratic)],
        PredicatePolicy,
    );
    if report.status == AlgebraicRootPolynomialImageStatus::Transformed {
        let expected = constant + linear * root + quadratic * root * root;
        let representation = report.representation.as_ref().unwrap();
        assert!(representation.is_valid());
        assert_eq!(representation.exact_rational_witness(), Some(&real(expected)));
    }
});
