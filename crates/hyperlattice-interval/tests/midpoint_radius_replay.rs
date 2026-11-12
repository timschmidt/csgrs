#![cfg(all(feature = "fixed", feature = "serde"))]
use hyperlattice_interval::{
    IntervalLatticeError, IntervalVector, MidpointRadiusVector,
    hyperinterval::{FixedInterval, IntervalError},
};
type I = FixedInterval<i64, 63>;

#[test]
fn retained_halves_replay_in_a_fresh_process() {
    const KEY: &str = "HYPERLATTICE_MIDPOINT_RADIUS_REPLAY";
    if let Ok(text) = std::env::var(KEY) {
        let form: MidpointRadiusVector<I, 2> = serde_json::from_str(&text).unwrap();
        assert_eq!(serde_json::to_string(&form).unwrap(), text);
        assert_eq!(
            form.midpoint_enclosure(&mut ()).unwrap().lanes().unwrap()[0],
            I::new(0, 1).unwrap()
        );
        assert_eq!(
            form.radius_enclosure(&mut ()),
            Err(IntervalLatticeError::Scalar(IntervalError::Overflow))
        );
        let source = form.into_intervals();
        assert_eq!(source.widest_axis(&mut ()).unwrap(), Some(1));
        assert_eq!(
            source.widths(&mut ()).unwrap().unwrap()[1].numerator(),
            u64::MAX
        );
        #[cfg(feature = "float-enclosure")]
        {
            use hyperlattice_interval::{
                IntervalMatrix, MidpointRadiusMatrix, hyperinterval::F64Enclosure as F,
            };
            let text = std::env::var("HYPERLATTICE_MIDPOINT_RADIUS_FLOAT").unwrap();
            let form: MidpointRadiusMatrix<F, 1, 2> = serde_json::from_str(&text).unwrap();
            assert_eq!(serde_json::to_string(&form).unwrap(), text);
            assert_eq!(
                form.radius_enclosure(&mut ()).unwrap().rows().unwrap()[0][1],
                F::point(f64::MAX).unwrap()
            );
            let source = form.into_intervals();
            let restored: IntervalMatrix<F, 1, 2> =
                serde_json::from_str(&serde_json::to_string(&source).unwrap()).unwrap();
            assert_eq!(restored.intersection(&source, &mut ()).unwrap(), source);
            assert!(
                !restored
                    .midpoint_radius()
                    .midpoint_enclosure(&mut ())
                    .unwrap()
                    .is_singleton()
            );
        }
        #[cfg(feature = "dyadic")]
        {
            use hyperlattice_interval::hyperinterval::{DyadicBudget, DyadicInterval as D};
            let text = std::env::var("HYPERLATTICE_MIDPOINT_RADIUS_DYADIC").unwrap();
            let form: MidpointRadiusVector<D, 1> = serde_json::from_str(&text).unwrap();
            assert_eq!(serde_json::to_string(&form).unwrap(), text);
            let midpoint = form
                .midpoint_enclosure(&mut DyadicBudget::new(32, 1000))
                .unwrap();
            let source = form.into_intervals();
            assert_eq!(midpoint, source);
            assert_eq!(
                source
                    .widths(&mut DyadicBudget::new(32, 1000))
                    .unwrap()
                    .unwrap()[0],
                D::point(1.into(), i32::MIN).unwrap()
            );
        }
        return;
    }
    let form = IntervalVector::new([I::new(0, 1).unwrap(), I::new(i64::MIN, i64::MAX).unwrap()])
        .unwrap()
        .midpoint_radius();
    let mut command = std::process::Command::new(std::env::current_exe().unwrap());
    command
        .args([
            "--exact",
            "retained_halves_replay_in_a_fresh_process",
            "--nocapture",
        ])
        .env(KEY, serde_json::to_string(&form).unwrap());
    #[cfg(feature = "float-enclosure")]
    {
        use hyperlattice_interval::{IntervalMatrix, hyperinterval::F64Enclosure as F};
        let form = IntervalMatrix::new([[
            F::new(0., f64::from_bits(1)).unwrap(),
            F::new(-f64::MAX, f64::MAX).unwrap(),
        ]])
        .unwrap()
        .midpoint_radius();
        command.env(
            "HYPERLATTICE_MIDPOINT_RADIUS_FLOAT",
            serde_json::to_string(&form).unwrap(),
        );
    }
    #[cfg(feature = "dyadic")]
    {
        use hyperlattice_interval::hyperinterval::DyadicInterval as D;
        let form = IntervalVector::new([D::new(1.into(), 2.into(), i32::MIN).unwrap()])
            .unwrap()
            .midpoint_radius();
        command.env(
            "HYPERLATTICE_MIDPOINT_RADIUS_DYADIC",
            serde_json::to_string(&form).unwrap(),
        );
    }
    let output = command.output().unwrap();
    assert!(
        output.status.success(),
        "{} {}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}
