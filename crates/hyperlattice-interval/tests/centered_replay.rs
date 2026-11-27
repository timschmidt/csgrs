//! Fresh-process authored-definition replay and malformed native encoding witnesses.
//!
//! The source components, not projected endpoints, cross the process boundary.
//! Optional rational promotion and IEEE projections exercise reuse after decoding.

#![cfg(all(feature = "fixed", feature = "serde"))]
use hyperlattice_interval::{
    CenteredMatrix, CenteredVector, IntervalLatticeError, IntervalVector,
    hyperinterval::{FixedInterval, IntervalError},
};
/// Whole-unit i64 endpoints for the retained overflowing source and JSON fixtures.
type I = FixedInterval<i64, 0>;

#[test]
/// Check exact source encoding identity and retain components after projection overflow.
/// The child optionally promotes fixed components to rational endpoints and checks
/// intersection, and replays an IEEE matrix with subnormal radii before transpose.
/// The parent invokes this exact test and checks captured status without an internal
/// timeout. Clear the marker and companion float payload to exercise parent setup.
fn authored_definition_replays_in_fresh_process() {
    /// Mandatory fixed-source JSON payload and child selector; readable values bypass spawn.
    const KEY: &str = "HYPERLATTICE_AUTHORED_CENTER_RADIUS";
    if let Ok(text) = std::env::var(KEY) {
        let form: CenteredVector<I, 2> = serde_json::from_str(&text).unwrap();
        assert_eq!(serde_json::to_string(&form).unwrap(), text);
        assert_eq!(
            form.enclosure(&mut ()),
            Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
        );
        let (center, radius) = form.into_parts().unwrap();
        assert_eq!(
            center.lanes().unwrap()[0].bounds(),
            Some((i64::MAX, i64::MAX))
        );
        assert_eq!(radius.lanes().unwrap()[0].bounds(), Some((1, 1)));
        #[cfg(feature = "rational")]
        {
            use hyperlattice_interval::hyperinterval::RationalInterval as R;
            let promote = |v: IntervalVector<I, 2>| {
                IntervalVector::new(
                    v.into_lanes()
                        .unwrap()
                        .map(|i| R::point(i.bounds().unwrap().0.into())),
                )
                .unwrap()
            };
            let form = CenteredVector::new(promote(center), promote(radius)).unwrap();
            let result = form.enclosure(&mut ()).unwrap();
            assert_eq!(
                result.lanes().unwrap()[0],
                R::new((i64::MAX - 1).into(), (i64::MAX as u64 + 1).into()).unwrap()
            );
            assert_eq!(
                result.lanes().unwrap()[1],
                R::new((-5).into(), (-1).into()).unwrap()
            );
            assert_eq!(result.intersection(&result, &mut ()).unwrap(), result);
        }
        #[cfg(feature = "float-enclosure")]
        {
            use hyperlattice_interval::hyperinterval::F64Enclosure as F;
            let text = std::env::var("HYPERLATTICE_AUTHORED_CENTER_RADIUS_FLOAT").unwrap();
            let form: CenteredMatrix<F, 1, 2> = serde_json::from_str(&text).unwrap();
            assert_eq!(serde_json::to_string(&form).unwrap(), text);
            let result = form.enclosure(&mut ()).unwrap();
            assert_eq!(
                result.rows().unwrap()[0][0].bounds(),
                Some((
                    f64::from_bits(1.0f64.to_bits() - 1),
                    f64::from_bits(1.0f64.to_bits() + 1)
                ))
            );
            assert_eq!(
                result.rows().unwrap()[0][1].bounds(),
                Some((0.0, f64::from_bits(2)))
            );
            assert_eq!(
                result
                    .transpose(&mut ())
                    .unwrap()
                    .transpose(&mut ())
                    .unwrap(),
                result
            );
        }
        return;
    }
    let form = CenteredVector::new(
        IntervalVector::new([I::point(i64::MAX).unwrap(), I::point(-3).unwrap()]).unwrap(),
        IntervalVector::new([I::point(1).unwrap(), I::point(2).unwrap()]).unwrap(),
    )
    .unwrap();
    let mut child = std::process::Command::new(std::env::current_exe().unwrap());
    child
        .args([
            "--exact",
            "authored_definition_replays_in_fresh_process",
            "--nocapture",
        ])
        .env(KEY, serde_json::to_string(&form).unwrap());
    #[cfg(feature = "float-enclosure")]
    {
        use hyperlattice_interval::{IntervalMatrix, hyperinterval::F64Enclosure as F};
        let tiny = F::point(f64::from_bits(1)).unwrap();
        let form = CenteredMatrix::new(
            IntervalMatrix::new([[F::point(1.0).unwrap(), tiny]]).unwrap(),
            IntervalMatrix::new([[tiny, tiny]]).unwrap(),
        )
        .unwrap();
        child.env(
            "HYPERLATTICE_AUTHORED_CENTER_RADIUS_FLOAT",
            serde_json::to_string(&form).unwrap(),
        );
    }
    let output = child.output().unwrap();
    assert!(
        output.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
/// Reject malformed products, nonsingletons, negative radii and fabricated validity data.
/// Native decoding distinguishes null empty from unit products, rejects invalid
/// scalar scales even for empty, and round-trips a 64-lane authored source before
/// checking all explicit projected bounds.
fn malformed_components_and_zero_dimensions() {
    /// One-coordinate authored vector used to reject malformed center/radius payloads.
    type V = CenteredVector<I, 1>;
    for text in [
        "[null,[[0,0]]]",
        "[[[0,0]],null]",
        "[[[0,1]],[[0,0]]]",
        "[[[0,0]],[[0,1]]]",
        "[[[0,0]],[[-1,-1]]]",
        "[[[1,0]],[[0,0]]]",
        "[[],[]]",
        "[[[0,0]],[[0,0]],true]",
        "{\"valid\":true}",
    ] {
        assert!(serde_json::from_str::<V>(text).is_err(), "{text}");
    }
    let empty: V = serde_json::from_str("null").unwrap();
    assert!(empty.enclosure(&mut ()).unwrap().is_empty());
    let unit: CenteredVector<I, 0> = serde_json::from_str("[[],[]]").unwrap();
    assert!(unit.enclosure(&mut ()).unwrap().is_singleton());
    assert_eq!(serde_json::to_string(&unit).unwrap(), "[[],[]]");
    assert!(serde_json::from_str::<CenteredVector<FixedInterval<i32, 32>, 0>>("null").is_err());
    /// One-row authored matrix used to reject invalid component and sign payloads.
    type M = CenteredMatrix<I, 1, 2>;
    for text in [
        "[null,null]",
        "[[[[0,0],[0,0]]],[[[0,0],[-1,-1]]]]",
        "[[[[0,0],[0,1]]],[[[0,0],[0,0]]]]",
        "[[[[0,0],[0,0]]],[[[0,0],[0,1]]]]",
    ] {
        assert!(serde_json::from_str::<M>(text).is_err(), "{text}");
    }
    let unit: CenteredMatrix<I, 2, 0> = serde_json::from_str("[[[],[]],[[],[]]]").unwrap();
    assert!(unit.enclosure(&mut ()).unwrap().is_singleton());
    let empty: CenteredMatrix<I, 0, 3> = serde_json::from_str("null").unwrap();
    assert!(empty.is_empty());
    assert!(empty.enclosure(&mut ()).unwrap().is_empty());
    assert_eq!(serde_json::to_string(&empty).unwrap(), "null");
    let point = I::point(3).unwrap();
    let center = IntervalVector::new([point; 64]).unwrap();
    let form = CenteredVector::new(center, center).unwrap();
    let text = serde_json::to_string(&form).unwrap();
    let restored: CenteredVector<I, 64> = serde_json::from_str(&text).unwrap();
    assert_eq!(restored, form);
    assert_eq!(
        restored.enclosure(&mut ()).unwrap(),
        IntervalVector::new([I::new(0, 6).unwrap(); 64]).unwrap()
    );
}
