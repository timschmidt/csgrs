#![no_main]

use hyperlimit::PredicatePolicy;
use hyperreal::Real;
use hypersolve::{
    AlgebraicRootArithmeticOp, AlgebraicRootBinaryTransformStatus, AlgebraicRootKind,
    AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    IsolatedRootInterval, SymbolId, transform_algebraic_roots_binary,
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

fuzz_target!(|data: [u8; 3]| {
    let left_square = i64::from(data[0] % 15) + 2;
    let right_square = i64::from(data[1] % 15) + 17;
    let operation = match data[2] % 4 {
        0 => AlgebraicRootArithmeticOp::Add,
        1 => AlgebraicRootArithmeticOp::Subtract,
        2 => AlgebraicRootArithmeticOp::Multiply,
        _ => AlgebraicRootArithmeticOp::Divide,
    };
    let report = transform_algebraic_roots_binary(
        &sqrt_root(left_square),
        &sqrt_root(right_square),
        operation,
        PredicatePolicy::default(),
    );
    if report.status == AlgebraicRootBinaryTransformStatus::Transformed {
        let representation = report.representation.as_ref().unwrap();
        assert!(representation.is_valid());
        assert!(representation.exact_rational_witness().is_none());
    }
});
