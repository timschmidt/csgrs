//! Runtime-chain replay preserving source restrictions, occupied maps and subsequent use.
//!
//! Wire definitions omit capacity and unused slots. Decode into a larger capacity
//! can therefore preserve bytes while every source and map is validated again.

#![cfg(all(feature = "fixed", feature = "serde"))]
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{AffineChain, AffineMap, IntervalMatrix, IntervalVector};
/// Half-unit fixed grid that cannot represent the exact quarter-point intermediate.
type I = FixedInterval<i32, 1>;
/// One-dimensional runtime chain whose receiving capacity is a Rust type parameter.
type Chain<const CAP: usize> = AffineChain<I, IntervalVector<I, 1>, 1, CAP>;
/// Construct multiplication by raw numerator n/2 with zero translation.
/// The fixture unwraps native carrier and singleton-map validation failures.
fn scale(n: i32) -> AffineMap<I, 1, 1> {
    AffineMap::new(
        IntervalMatrix::new([[I::point(n).unwrap()]]).unwrap(),
        IntervalVector::new([I::point(0).unwrap()]).unwrap(),
    )
    .unwrap()
}

#[test]
/// Replay 33 alternating half/two maps from capacity 33 into a capacity-40 child.
/// The restricted half-point source has exact image one quarter. Encoding identity,
/// occupied count and source bounds survive replay; an appended two restores one
/// half, then a static multiplication by two round-trips locally and yields one. The parent
/// selects this exact child test and checks captured status without an internal
/// timeout. Clear the readable marker to exercise spawning rather than the child path.
fn fresh_process_runtime_replay_preserves_off_grid_and_restriction() {
    /// Runtime source/map definition payload and child selector, excluding unused capacity.
    const KEY: &str = "HYPERLATTICE_AFFINE_CHAIN_REPLAY";
    if let Ok(text) = std::env::var(KEY) {
        let mut chain: Chain<40> = serde_json::from_str(&text).unwrap();
        assert_eq!(serde_json::to_string(&chain).unwrap(), text);
        assert_eq!(chain.map_count(), 33);
        assert_eq!(
            chain.parameter_domain().lanes(),
            Some(&[I::point(1).unwrap()])
        );
        assert_eq!(
            chain.enclosure(&mut ()).unwrap().lanes(),
            Some(&[I::new(0, 1).unwrap()])
        );
        chain.push(scale(4)).unwrap();
        assert_eq!(
            chain.enclosure(&mut ()).unwrap().lanes(),
            Some(&[I::point(1).unwrap()])
        );
        // Next static map consumes this runtime result, and replays again.
        let next = scale(4).image(chain);
        let text = serde_json::to_string(&next).unwrap();
        let replay: hyperlattice_interval::AffineImage<I, Chain<40>, 1, 1> =
            serde_json::from_str(&text).unwrap();
        assert_eq!(replay, next);
        assert_eq!(
            replay.enclosure(&mut ()).unwrap().lanes(),
            Some(&[I::point(2).unwrap()])
        );
        return;
    }
    let mut chain = Chain::<33>::new(IntervalVector::new([I::new(-2, 4).unwrap()]).unwrap());
    for index in 0..33 {
        chain
            .push(scale(if index % 2 == 0 { 1 } else { 4 }))
            .unwrap();
    }
    let selected = chain
        .restrict_parameters(
            &IntervalVector::new([I::point(1).unwrap()]).unwrap(),
            &mut (),
        )
        .unwrap();
    let text = serde_json::to_string(&selected).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_runtime_replay_preserves_off_grid_and_restriction",
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
/// Check replay across capacities, malformed shapes and every map position.
/// Both empty and nonempty sources retain definitions from zero through three maps.
/// Larger capacities preserve bytes and projections; smaller capacities reject excess.
/// Nonsingleton coefficients, malformed tuple/array data and invalid fixed scales
/// fail rather than being hidden by empty or zero-dimensional source states.
fn replay_checks_capacity_shapes_and_all_maps_even_on_empty_source() {
    for source in [
        IntervalVector::new([I::new(-1, 1).unwrap()]).unwrap(),
        IntervalVector::empty().unwrap(),
    ] {
        let mut chain = Chain::<3>::new(source);
        for depth in 0..=3 {
            let text = serde_json::to_string(&chain).unwrap();
            let replay: Chain<3> = serde_json::from_str(&text).unwrap();
            assert_eq!(replay, chain);
            let larger: Chain<40> = serde_json::from_str(&text).unwrap();
            assert_eq!(serde_json::to_string(&larger).unwrap(), text);
            assert_eq!(
                larger.enclosure(&mut ()).unwrap(),
                chain.enclosure(&mut ()).unwrap()
            );
            if depth > 0 {
                assert!(serde_json::from_str::<Chain<0>>(&text).is_err());
            }
            if depth < 3 {
                chain.push(scale(1)).unwrap();
            }
        }
        let valid = serde_json::to_value(chain).unwrap();
        for index in 0..3 {
            for bad_matrix in [false, true] {
                let mut bad = valid.clone();
                if bad_matrix {
                    bad[1][index][0][0][0] = serde_json::json!([0, 1]);
                } else {
                    bad[1][index][1][0] = serde_json::json!([0, 1]);
                }
                assert!(serde_json::from_value::<Chain<3>>(bad).is_err());
            }
        }
        let mut excess = valid.clone();
        excess[1]
            .as_array_mut()
            .unwrap()
            .push(serde_json::to_value(scale(1)).unwrap());
        let error = serde_json::from_value::<Chain<3>>(excess)
            .unwrap_err()
            .to_string();
        assert!(
            error.contains("affine chain capacity 3 exhausted"),
            "{error}"
        );
    }
    for bad in [
        "null",
        "[]",
        "[null]",
        "[null,null]",
        "[null,[null]]",
        "[null,[],0]",
        "[[],[]]",
        "[[[0,1],[0,1]],[]]",
    ] {
        assert!(serde_json::from_str::<Chain<3>>(bad).is_err(), "{bad}");
    }
    let empty: Chain<0> = serde_json::from_str("[null,[]]").unwrap();
    assert!(empty.is_empty());
    /// Zero-dimensional, zero-capacity chain whose invalid scalar scale must still fail.
    type Invalid =
        AffineChain<FixedInterval<i32, 32>, IntervalVector<FixedInterval<i32, 32>, 0>, 0, 0>;
    assert!(serde_json::from_str::<Invalid>("[[],[]]").is_err());
}
