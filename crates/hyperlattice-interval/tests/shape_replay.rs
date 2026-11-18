#![cfg(all(feature = "fixed", feature = "serde"))]

#[path = "support/shape_oracle.rs"]
mod oracle;

#[test]
fn rectangular_replay_and_action() {
    for shape in 0..7 {
        for matrix in 0..9 {
            for vector in 0..7 {
                for index in [0, 1, 2, 127, 255] {
                    for encoding in [0, 64, 128] {
                        oracle::check(&[
                            shape | encoding,
                            matrix,
                            vector,
                            index,
                            128,
                            3,
                            17,
                            127,
                            0,
                        ]);
                    }
                }
            }
        }
    }
}

#[test]
fn retained_shape_inputs() {
    for input in [
        include_bytes!("fixtures/shapes/prefix-overflow.bin").as_slice(),
        include_bytes!("fixtures/shapes/replayed-overflow.bin").as_slice(),
        include_bytes!("fixtures/shapes/replayed-sum-overflow.bin").as_slice(),
        include_bytes!("fixtures/shapes/wide-quarter-grid.bin").as_slice(),
        include_bytes!("fixtures/shapes/empty-before-invalid.bin").as_slice(),
    ] {
        oracle::check(input);
    }
}
