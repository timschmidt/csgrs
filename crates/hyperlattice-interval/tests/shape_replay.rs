//! Deterministic callers of the Cartesian shape/replay oracle shared with fuzzing.
//!
//! Replay occurs in this process. The bounded parameter sweep and retained byte
//! fixtures test wire validation, empty/unit distinctions and subsequent actions;
//! they do not exhaust all endpoint values or replace the fuzz campaign.

#![cfg(all(feature = "fixed", feature = "serde"))]

/// Independent wire-status and integer-arithmetic expectations for the shared corpus.
#[path = "support/shape_oracle.rs"]
mod oracle;

#[test]
/// Exercise 6,615 byte inputs spanning seven shapes, nine matrix edits, seven
/// vector edits, five selectors and three endpoint encodings. Selectors also
/// affect projection indices; cyclic payload reuse means this is a finite
/// deterministic sweep, not independent coverage of every possible endpoint.
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
/// Replay five unchanged regression byte strings through the full shared oracle.
/// They retain prefix overflow, overflow after replay (product and accumulated
/// sum), a 33-lane quarter-grid case, and malformed bounds after an empty lane.
/// Filename labels identify witnesses; the oracle derives outcomes from bytes.
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
