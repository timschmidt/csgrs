#![no_main]

use hyperreal::Real;
use hypersolve::{solve_dense_linear_system_bareiss, solve_dense_linear_system_bareiss_multi_rhs};
use libfuzzer_sys::fuzz_target;

fn real(value: i64) -> Real {
    Real::from(value)
}

fn sample(data: &[u8], index: usize) -> i64 {
    i64::from(i8::from_le_bytes([data
        .get(index)
        .copied()
        .unwrap_or_default()]))
}

fuzz_target!(|data: &[u8]| {
    let a = (sample(data, 0) % 15).abs() + 1;
    let b = sample(data, 1) % 15;
    let c = (sample(data, 2) % 15).abs() + 1;
    let first = [sample(data, 3) % 64, sample(data, 4) % 64];
    let second = [sample(data, 5) % 64, sample(data, 6) % 64];
    let matrix = [vec![real(a), real(b)], vec![real(0), real(c)]];
    let right_hand_sides = [
        vec![real(a * first[0] + b * first[1]), real(c * first[1])],
        vec![real(a * second[0] + b * second[1]), real(c * second[1])],
    ];

    let report = solve_dense_linear_system_bareiss_multi_rhs(&matrix, &right_hand_sides, -64)
        .expect("generated triangular system is nonsingular");
    assert_eq!(
        report.solutions,
        vec![
            vec![real(first[0]), real(first[1])],
            vec![real(second[0]), real(second[1])],
        ]
    );
    assert!(report.residual_replays.iter().all(|replay| replay.accepted));

    for (index, rhs) in right_hand_sides.iter().enumerate() {
        let single = solve_dense_linear_system_bareiss(&matrix, rhs, -64)
            .expect("generated triangular system is nonsingular");
        assert_eq!(report.solutions[index], single.solution);
        assert_eq!(report.numerators[index], single.numerators);
        assert_eq!(report.residual_replays[index], single.residual_replay);
    }
});
