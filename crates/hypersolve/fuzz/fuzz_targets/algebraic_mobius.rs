#![no_main]

use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    AlgebraicRootKind, AlgebraicRootMobiusTransformStatus, AlgebraicRootRepresentation,
    AlgebraicRootValidationReport, AlgebraicRootValidationStatus, IsolatedRootInterval, SymbolId,
    transform_algebraic_root_mobius,
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

fuzz_target!(|data: [i16; 5]| {
    let root = i64::from(data[0] % 32);
    let a = i64::from(data[1] % 16);
    let b = i64::from(data[2] % 16);
    let c = i64::from(data[3] % 16);
    let d = i64::from(data[4] % 16);
    let report = transform_algebraic_root_mobius(
        &represented_rational_root(root),
        real(a),
        real(b),
        real(c),
        real(d),
        PredicatePolicy::default(),
    );
    if a * d - b * c == 0 {
        assert_ne!(report.status, AlgebraicRootMobiusTransformStatus::Transformed);
        return;
    }
    if c * root + d == 0 {
        assert_ne!(report.status, AlgebraicRootMobiusTransformStatus::Transformed);
        return;
    }
    if report.status == AlgebraicRootMobiusTransformStatus::Transformed {
        let expected = (real(a * root + b) / real(c * root + d)).unwrap();
        let representation = report.representation.as_ref().unwrap();
        assert!(representation.is_valid());
        assert_eq!(representation.exact_rational_witness(), Some(&expected));
    }
});
