//! Same-process JSON replay witnesses for fixed Cartesian vectors and matrices.
//!
//! Checks constructor validation, dimensions above Serde's ordinary small-array
//! support, deterministic encoding where asserted, and subsequent interval work.
//! The expectations concern the Cartesian carriers themselves, not preservation
//! of correlations discarded by an earlier enclosure or fresh-process replay.

#![cfg(all(feature = "serde", feature = "fixed"))]
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{IntervalMatrix, IntervalVector};
/// Signed 32-bit integer-grid intervals used by the JSON fixtures.
type I = FixedInterval<i32, 0>;

#[test]
/// Round-trip a 65-lane box and a 40-by-2 matrix, then consume decoded outputs.
/// The wide vector also requires byte-identical re-encoding. The decoded matrix maps
/// `[-1,2]` × `[4,5]` to forty `[6,14]` lanes; replaying that result and taking an independent
/// interval dot product gives `[1440,7840]`. Matrix/output checks assert values, not an
/// additional byte-identity or dependency-aware squared-norm contract.
fn arbitrary_dimensions_and_output_replay() {
    let wide = IntervalVector::new([I::new(-2, 3).unwrap(); 65]).unwrap();
    let bytes = serde_json::to_string(&wide).unwrap();
    let replay: IntervalVector<I, 65> = serde_json::from_str(&bytes).unwrap();
    assert_eq!(wide, replay);
    assert_eq!(bytes, serde_json::to_string(&replay).unwrap());
    let tall = IntervalMatrix::new([[I::point(2).unwrap(); 2]; 40]).unwrap();
    let text = serde_json::to_string(&tall).unwrap();
    let replay: IntervalMatrix<I, 40, 2> = serde_json::from_str(&text).unwrap();
    assert_eq!(tall, replay);
    let vector = IntervalVector::new([I::new(-1, 2).unwrap(), I::new(4, 5).unwrap()]).unwrap();
    let output = replay.mul_vector(&vector, &mut ()).unwrap();
    assert_eq!(output.lanes(), Some(&[I::new(6, 14).unwrap(); 40]));
    let replay: IntervalVector<I, 40> =
        serde_json::from_str(&serde_json::to_string(&output).unwrap()).unwrap();
    assert_eq!(
        replay.dot(&output, &mut ()).unwrap(),
        I::new(1440, 7840).unwrap()
    );
}

#[test]
/// Reject bad shapes, endpoint order and invalid fixed families, even beside emptiness.
/// An empty lane cannot hide a later malformed scalar. Valid empty lanes canonicalize
/// whole products to JSON null; zero-sized arrays and null still validate fixed scale.
/// This is a finite malformed-input corpus, not a complete serialization-format audit.
fn malformed_replay_and_empty_canonicalization() {
    for input in [
        "[]",
        "[[1,2]]",
        "[[1,2],[3,4],[5,6]]",
        "[[2,1],[3,4]]",
        "[null,[4,3]]",
    ] {
        assert!(
            serde_json::from_str::<IntervalVector<I, 2>>(input).is_err(),
            "{input}"
        );
    }
    for input in [
        "[]",
        "[[[0,1]]]",
        "[[[0,1],[1,2]],[[0,1]]]",
        "[[[0,1],[1,2]],[[0,1],[2,1]]]",
        "[[],[]]",
    ] {
        assert!(
            serde_json::from_str::<IntervalMatrix<I, 2, 2>>(input).is_err(),
            "{input}"
        );
    }
    let vector: IntervalVector<I, 2> = serde_json::from_str("[null,[1,2]]").unwrap();
    assert!(vector.is_empty());
    assert_eq!(serde_json::to_string(&vector).unwrap(), "null");
    let matrix: IntervalMatrix<I, 1, 2> = serde_json::from_str("[[[1,2],null]]").unwrap();
    assert!(matrix.is_empty());
    assert_eq!(serde_json::to_string(&matrix).unwrap(), "null");
    assert!(serde_json::from_str::<IntervalVector<FixedInterval<i32, 32>, 0>>("null").is_err());
    assert!(serde_json::from_str::<IntervalVector<FixedInterval<i32, 32>, 0>>("[]").is_err());
    assert!(
        serde_json::from_str::<IntervalMatrix<FixedInterval<i64, 64>, 2, 0>>("[[],[]]").is_err()
    );
    assert!(serde_json::from_str::<IntervalMatrix<FixedInterval<i64, 64>, 2, 0>>("null").is_err());
}

#[test]
/// Decode unit products separately from JSON null and preserve the matrix encodings.
/// The vector `[]` and matrix `[[],[]]` denote nonempty units; null denotes canonical empty.
/// Vector inequality and matrix re-encoding guard against collapsing these cases.
fn unit_and_empty_products_have_distinct_replays() {
    let unit: IntervalVector<I, 0> = serde_json::from_str("[]").unwrap();
    let empty: IntervalVector<I, 0> = serde_json::from_str("null").unwrap();
    assert!(unit.is_singleton() && empty.is_empty());
    assert_ne!(unit, empty);
    let unit: IntervalMatrix<I, 2, 0> = serde_json::from_str("[[],[]]").unwrap();
    let empty: IntervalMatrix<I, 2, 0> = serde_json::from_str("null").unwrap();
    assert!(unit.is_singleton() && empty.is_empty());
    assert_eq!(serde_json::to_string(&unit).unwrap(), "[[],[]]");
    assert_eq!(serde_json::to_string(&empty).unwrap(), "null");
}
