#![no_main]

use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    transform_algebraic_root_rational_image, AlgebraicRootKind, AlgebraicRootRationalImageStatus,
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    IsolatedRootInterval, SymbolId,
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

fn sqrt_two_positive() -> AlgebraicRootRepresentation {
    AlgebraicRootRepresentation {
        constraint_index: 1,
        symbol: SymbolId(1),
        interval_index: 0,
        polynomial_coefficients: vec![real(-2), Real::zero(), Real::one()],
        interval: IsolatedRootInterval {
            lower: real(1),
            upper: real(2),
            exact_root: None,
            distinct_root_count: 1,
        },
        kind: AlgebraicRootKind::IsolatingInterval,
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    }
}

fuzz_target!(|data: [i16; 5]| {
    let root = i64::from(data[0] % 24);
    let numerator_constant = i64::from(data[1] % 16);
    let numerator_linear = i64::from(data[2] % 16);
    let denominator_constant = i64::from(data[3] % 16);
    let denominator_linear = i64::from(data[4] % 16);
    let denominator = denominator_constant + denominator_linear * root;
    let report = transform_algebraic_root_rational_image(
        &represented_rational_root(root),
        &[real(numerator_constant), real(numerator_linear)],
        &[real(denominator_constant), real(denominator_linear)],
        PredicatePolicy::default(),
    );

    if denominator == 0 {
        assert_eq!(
            report.status,
            AlgebraicRootRationalImageStatus::CertifiedZeroDenominator
        );
    } else if report.status == AlgebraicRootRationalImageStatus::Transformed {
        let expected =
            (real(numerator_constant + numerator_linear * root) / real(denominator)).unwrap();
        let representation = report.representation.as_ref().unwrap();
        assert!(representation.is_valid());
        assert_eq!(representation.exact_rational_witness(), Some(&expected));
    }

    let positive_constant = i64::from(data[0].unsigned_abs() % 16) + 9;
    let direct = transform_algebraic_root_rational_image(
        &sqrt_two_positive(),
        &[Real::one(), Real::one()],
        &[real(positive_constant), real(-3), Real::one()],
        PredicatePolicy::default(),
    );
    if direct.status == AlgebraicRootRationalImageStatus::Transformed {
        assert!(direct.numerator_image.is_none());
        assert!(direct.denominator_image.is_none());
        assert!(direct.representation.as_ref().unwrap().is_valid());
    }
});
