#[path = "support/conic_box.rs"]
mod oracle;

fn replay(coeffs: [i8; 6], rect: [i8; 4], mode: u8) {
    let [a, b, c, d, e, f] = coeffs.map(|n| n as u8);
    let [left, right, bottom, top] = rect.map(|n| n as u8);
    oracle::check([a, b, c, d, e, f, left, right, bottom, top, mode]);
}

#[test]
fn critical_points_include_isolated_interior_edge_and_singular_minima() {
    // (3x+y-1)² + (2y-1)² vanishes only at (1/6, 1/2).
    let q = [9, 6, 5, -6, -6, 2];
    // On [2,4] × [-1,1], 5q - 121 has its minimum at (2, -3/5).
    let edge = [45, 30, 25, -30, -30, -111];
    // The stationary line of (2x-3y-1)² intersects two edges.
    let singular = [4, -12, 9, -4, 6, 1];
    for (coeffs, rect, expected) in [
        (q, [0, 1, 0, 1], (0, 1)),
        (edge, [2, 4, -1, 1], (0, 1)),
        (singular, [0, 2, 0, 1], (0, 1)),
        (q.map(|v| -v), [0, 1, 0, 1], (-1, 0)),
    ] {
        let [left, right, bottom, top] = rect.map(i64::from);
        assert_eq!(
            oracle::extrema_signs(coeffs.map(i64::from), [left, right], [bottom, top]),
            expected
        );
        for kind in 0..4 {
            replay(coeffs, rect, kind);
        }
    }
}

#[test]
fn full_quadratic_receipts_respect_exact_rectangle_extrema() {
    for [a, b, c] in [
        [1, 3, 2],
        [2, 4, 2],
        [2, 1, 3],
        [0, 3, 0],
        [0, 0, 0],
        [-2, 3, -3],
    ] {
        for rect in [
            [-2, 3, -1, 4],
            [1, 1, 2, 2],
            [0, 0, -3, 4],
            [-2, 2, 1, 1],
            [0, 1, 0, 1],
        ] {
            for mode in 0..12 {
                replay([a, b, c, -3, 5, -1], rect, mode);
            }
        }
    }
}

#[test]
fn retained_fuzz_inputs_replay_under_both_policies() {
    for seed in [
        include_bytes!("fixtures/conic/interior.bin"),
        include_bytes!("fixtures/conic/edge.bin"),
        include_bytes!("fixtures/conic/stationary-line.bin"),
        include_bytes!("fixtures/conic/cross-term.bin"),
        include_bytes!("fixtures/conic/point-control.bin"),
        include_bytes!("fixtures/conic/constant-control.bin"),
    ] {
        oracle::check(*seed);
    }
}
