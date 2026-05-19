#![no_main]

use hypermesh::prelude::{Manifold, OpType, compute_boolean_with_report};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let offsets = data
        .chunks_exact(2)
        .take(6)
        .map(|chunk| {
            let raw = i16::from_le_bytes([chunk[0], chunk[1]]) as f64;
            raw / 4096.0
        })
        .collect::<Vec<_>>();
    if offsets.len() < 6 {
        return;
    }

    let left = tetrahedron([offsets[0], offsets[1], offsets[2]], 1.0);
    let right = tetrahedron([offsets[3], offsets[4], offsets[5]], 1.0);
    let Ok(left) = Manifold::new(&left, &[0, 3, 1, 1, 2, 0, 1, 3, 2, 2, 3, 0]) else {
        return;
    };
    let Ok(right) = Manifold::new(&right, &[0, 3, 1, 1, 2, 0, 1, 3, 2, 2, 3, 0]) else {
        return;
    };

    for operation in [OpType::Add, OpType::Subtract, OpType::Intersect] {
        if let Ok(result) = compute_boolean_with_report(&left, &right, operation) {
            let _ = result.validate_against_inputs(&left, &right);
        }
    }
});

fn tetrahedron(offset: [f64; 3], scale: f64) -> Vec<f64> {
    let base = [
        [0.0, 0.0, 0.0],
        [scale, 0.0, 0.0],
        [0.0, scale, 0.0],
        [0.0, 0.0, scale],
    ];
    base.into_iter()
        .flat_map(|point| {
            [
                point[0] + offset[0],
                point[1] + offset[1],
                point[2] + offset[2],
            ]
        })
        .collect()
}
