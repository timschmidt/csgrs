#![no_main]

use hyperreal::Real;
use hypersolve::{
    SparseResidualTerm, solve_sparse_linear_system_bareiss_pattern_preserving,
};
use libfuzzer_sys::fuzz_target;

fn real(value: i64) -> Real {
    Real::from(value)
}

fuzz_target!(|data: [i16; 6]| {
    let a = i64::from((data[0] % 15).abs()) + 1;
    let b = i64::from(data[1] % 15);
    let c = i64::from((data[2] % 15).abs()) + 1;
    let x = i64::from(data[3] % 64);
    let y = i64::from(data[4] % 64);
    let duplicate = i64::from(data[5] % 8);
    let terms = [
        SparseResidualTerm {
            row: 0,
            column: 0,
            coefficient: real(a),
        },
        SparseResidualTerm {
            row: 0,
            column: 1,
            coefficient: real(b),
        },
        SparseResidualTerm {
            row: 1,
            column: 1,
            coefficient: real(c + duplicate),
        },
        SparseResidualTerm {
            row: 1,
            column: 1,
            coefficient: real(-duplicate),
        },
    ];
    let rhs = [real(a * x + b * y), real(c * y)];
    let report = solve_sparse_linear_system_bareiss_pattern_preserving(2, 2, &terms, &rhs, -64)
        .expect("generated triangular system is nonsingular");
    assert_eq!(report.solution, vec![real(x), real(y)]);
    assert!(report.sparse_residual_replay.accepted);
});
