//! Source-preserving native replay of statically nested affine definitions.
//!
//! The fresh-process fixture carries an off-grid exact image; other fixtures check
//! validation despite empty sources, parameter restrictions and structural identity.

#![cfg(all(feature = "fixed", feature = "serde"))]
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{AffineBox, AffineImage, AffineMap, IntervalMatrix, IntervalVector};
/// Half-unit fixed grid used to distinguish an exact quarter from its enclosure.
type I = FixedInterval<i32, 1>;
/// One retained scalar map over the original one-coordinate parameter box.
type First = AffineBox<I, 1, 1>;
/// Two statically nested scalar maps retaining that same original parameter domain.
type Chain = AffineImage<I, First, 1, 1>;

/// Construct the point map x -> (numerator/2)*x with zero translation.
/// The argument is a raw fixed numerator, not an integer-valued coefficient;
/// fixture construction failures panic.
fn scale(numerator: i32) -> AffineMap<I, 1, 1> {
    AffineMap::new(
        IntervalMatrix::new([[I::point(numerator).unwrap()]]).unwrap(),
        IntervalVector::new([I::point(0).unwrap()]).unwrap(),
    )
    .unwrap()
}

#[test]
/// Replay an exact quarter in a child, double it, then replay and double again locally.
/// The child verifies original encoding identity and independent projected half/one
/// values. The parent selects this exact test in the same executable and checks
/// captured exit status without an internal timeout. A readable marker bypasses
/// spawning, so clear it before exercising the parent path.
fn fresh_process_replay_retains_off_grid_point_and_next_operation() {
    /// Off-grid source-image JSON payload and child selector; no projection cache is sent.
    const KEY: &str = "HYPERLATTICE_AFFINE_REPLAY";
    if let Ok(text) = std::env::var(KEY) {
        let image: First = serde_json::from_str(&text).unwrap();
        assert_eq!(serde_json::to_string(&image).unwrap(), text);
        assert_eq!(
            image.enclosure(&mut ()).unwrap().lanes(),
            Some(&[I::new(0, 1).unwrap()])
        );
        let restored = scale(4).image(image);
        assert_eq!(
            restored.enclosure(&mut ()).unwrap().lanes(),
            Some(&[I::point(1).unwrap()])
        );
        let serialized = serde_json::to_string(&restored).unwrap();
        let again: Chain = serde_json::from_str(&serialized).unwrap();
        assert_eq!(again, restored);
        assert_eq!(
            scale(4).image(again).enclosure(&mut ()).unwrap().lanes(),
            Some(&[I::point(2).unwrap()])
        );
        return;
    }
    // The first image is the exact point 1/4, absent from this backend's grid.
    let image = scale(1).image(IntervalVector::new([I::point(1).unwrap()]).unwrap());
    let text = serde_json::to_string(&image).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_replay_retains_off_grid_point_and_next_operation",
            "--nocapture",
        ])
        .env(KEY, text)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
/// Reject invalid coefficients at any decoded map, including with an empty source.
/// Nonsingleton matrix/translation, malformed structure and invalid scalar-scale
/// payloads fail. A valid empty image retains its map and structural identity
/// through replay while its projection remains canonical empty.
fn validation_runs_for_empty_sources_and_all_nested_maps() {
    let image = scale(1).image(IntervalVector::new([I::point(1).unwrap()]).unwrap());
    let valid = serde_json::to_value(image).unwrap();
    for empty in [false, true] {
        for bad_matrix in [false, true] {
            let mut malformed = valid.clone();
            if empty {
                malformed[0] = serde_json::Value::Null;
            }
            if bad_matrix {
                malformed[1][0][0][0] = serde_json::json!([0, 1]);
            } else {
                malformed[1][1][0] = serde_json::json!([0, 1]);
            }
            assert!(serde_json::from_value::<First>(malformed).is_err());
        }
    }
    let chain = scale(4).image(image);
    let mut malformed = serde_json::to_value(chain).unwrap();
    malformed[0][1][0] = serde_json::Value::Null;
    assert!(serde_json::from_value::<Chain>(malformed).is_err());
    for malformed in [
        "null",
        "[]",
        "[null]",
        "[null,null]",
        "[null,[null,null]]",
        "[[],[[],[]],0]",
    ] {
        assert!(
            serde_json::from_str::<First>(malformed).is_err(),
            "{malformed}"
        );
    }
    let empty = scale(1).image(IntervalVector::<I, 1>::empty().unwrap());
    let text = serde_json::to_string(&empty).unwrap();
    let replay: First = serde_json::from_str(&text).unwrap();
    assert!(replay.is_empty() && replay.enclosure(&mut ()).unwrap().is_empty());
    assert_eq!(replay, empty);
    /// Zero-dimensional affine box with an invalid fixed scale, which replay must reject.
    type Invalid = AffineBox<FixedInterval<i32, 32>, 0, 0>;
    assert!(serde_json::from_str::<Invalid>("[[],[[],[]]]").is_err());
}

#[test]
/// Replay a restricted original parameter box without mutating the unrestricted image.
/// Literal projected bounds check restriction; zero maps over different domains
/// then demonstrate that equal projected sets need not have equal definitions.
fn serialized_restrictions_keep_parameters_and_structural_identity() {
    let original =
        scale(4).image(scale(1).image(IntervalVector::new([I::new(-2, 4).unwrap()]).unwrap()));
    let selected = original
        .restrict_parameters(
            &IntervalVector::new([I::new(1, 3).unwrap()]).unwrap(),
            &mut (),
        )
        .unwrap();
    let text = serde_json::to_string(&selected).unwrap();
    let replay: Chain = serde_json::from_str(&text).unwrap();
    assert_eq!(replay, selected);
    assert_eq!(
        replay.enclosure(&mut ()).unwrap().lanes(),
        Some(&[I::new(1, 3).unwrap()])
    );
    assert_eq!(
        original.enclosure(&mut ()).unwrap().lanes(),
        Some(&[I::new(-2, 4).unwrap()])
    );
    // Equal images can have different exact definitions; PartialEq is structural.
    let a = scale(0).image(IntervalVector::new([I::new(-1, 1).unwrap()]).unwrap());
    let b = scale(0).image(IntervalVector::new([I::new(0, 2).unwrap()]).unwrap());
    assert_ne!(a, b);
    assert_eq!(a.enclosure(&mut ()).unwrap(), b.enclosure(&mut ()).unwrap());
}
