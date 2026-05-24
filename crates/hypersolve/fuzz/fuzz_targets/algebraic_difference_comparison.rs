#![no_main]

use hyperreal::Real;
use hypersolve::{
    AlgebraicRootComparisonStatus, AlgebraicRootKind, AlgebraicRootRefinementComparisonConfig,
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    IsolatedRootInterval, SymbolId, compare_algebraic_root_representations_by_difference,
};
use libfuzzer_sys::fuzz_target;

fn real(value: i64) -> Real {
    Real::from(value)
}

fn sqrt_root(square: i64) -> AlgebraicRootRepresentation {
    AlgebraicRootRepresentation {
        constraint_index: square as usize,
        symbol: SymbolId(square as u32),
        interval_index: 0,
        polynomial_coefficients: vec![-real(square), Real::zero(), Real::one()],
        interval: IsolatedRootInterval {
            lower: Real::one(),
            upper: real(8),
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

fuzz_target!(|data: [u8; 2]| {
    let left_square = i64::from(data[0] % 15) + 2;
    let right_square = i64::from(data[1] % 15) + 17;
    let report = compare_algebraic_root_representations_by_difference(
        &sqrt_root(left_square),
        &sqrt_root(right_square),
        AlgebraicRootRefinementComparisonConfig {
            max_refinement_rounds: 0,
            ..AlgebraicRootRefinementComparisonConfig::default()
        },
    );
    if report.comparison.status == AlgebraicRootComparisonStatus::Compared {
        assert_eq!(
            report.comparison.ordering,
            Some(left_square.cmp(&right_square))
        );
    }
});
