//! Completed continuous-trajectory verification, with preparation measured separately.
use hyperdynamics::field::{
    EnclosureBudget, EnclosurePolicy,
    picard::{PicardProposal, PicardStatus},
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hyperphysics::{ForceAccumulator3, ForceContribution3, Real, Vector3};
use std::{hint::black_box, time::Instant};
fn b() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn main() {
    let iterations: u32 = std::env::var("PHYSICS_FLOW_ITERATIONS")
        .map(|v| v.parse().unwrap())
        .unwrap_or(1_000);
    assert!(iterations > 0);
    let mut forces = ForceAccumulator3::default();
    forces.push(ForceContribution3 {
        source: "actuator".into(),
        force: Vector3::new([Real::from(3), Real::from(-2), Real::zero()]),
    });
    forces.push(ForceContribution3 {
        source: "bias".into(),
        force: Vector3::new([Real::from(1), Real::from(-2), Real::zero()]),
    });
    let prepare = || forces.dynamics(Real::from(2), &mut b()).unwrap();
    let model = prepare();
    let proposal = PicardProposal {
        start: d(0, 0, 0),
        end: d(1, 1, -2),
        initial: BoxN::new(std::array::from_fn(|_| d(0, 0, 0))).unwrap(),
        candidate: BoxN::new([
            d(0, 1, -3),
            d(-1, 0, -3),
            d(0, 0, 0),
            d(0, 1, -1),
            d(-1, 0, -1),
            d(0, 0, 0),
        ])
        .unwrap(),
    };
    let expected = model.certify_step(proposal.clone(), &mut b());
    assert!(matches!(
        expected.native().status(),
        PicardStatus::Certified
    ));
    let end = expected.native().end_enclosure().unwrap().lanes().unwrap();
    assert_eq!(&end[3..], &[d(1, 1, -1), d(-1, -1, -1), d(0, 0, 0)]);
    let mut arithmetic = DyadicBudget::new(4096, 100_000_000);
    assert!(d(1, 1, -4).subset_of(&end[0], &mut arithmetic).unwrap());
    assert!(d(-1, -1, -4).subset_of(&end[1], &mut arithmetic).unwrap());
    for candidate in [prepare(), model.clone()] {
        let result = candidate.certify_step(proposal.clone(), &mut b());
        assert!(matches!(result.native().status(), PicardStatus::Certified));
        assert_eq!(
            result.native().end_enclosure(),
            expected.native().end_enclosure()
        );
        assert_eq!(result.native().tube(), expected.native().tube());
        assert_eq!(
            result.native().evidence().jacobian(),
            expected.native().evidence().jacobian()
        );
        assert_eq!(
            result.native().evidence().contraction(),
            expected.native().evidence().contraction()
        );
        assert_eq!(result.native().identity(), expected.native().identity());
        assert_eq!(result.model().mass_sign(), expected.model().mass_sign());
        assert_eq!(result.model().provenance(), expected.model().provenance());
        assert_eq!(
            result.model().constant_forces().unwrap().contributions(),
            forces.contributions()
        );
    }
    for mode in ["prepare", "reuse", "prepare_and_step"] {
        for _ in 0..16 {
            drop(prepare().certify_step(proposal.clone(), &mut b()));
        }
        let started = Instant::now();
        for _ in 0..iterations {
            if mode == "prepare" {
                drop(black_box(prepare()));
            } else {
                let prepared;
                let current = if mode == "reuse" {
                    &model
                } else {
                    prepared = prepare();
                    &prepared
                };
                let report = current.certify_step(black_box(proposal.clone()), &mut b());
                assert!(matches!(report.native().status(), PicardStatus::Certified));
                assert_eq!(
                    report.native().end_enclosure(),
                    expected.native().end_enclosure()
                );
                assert_eq!(report.native().tube(), expected.native().tube());
                drop(black_box(report));
            }
        }
        let elapsed = started.elapsed();
        println!(
            "{mode}: iterations={iterations} total_ns={} ns_per_operation={}",
            elapsed.as_nanos(),
            elapsed.as_nanos() / u128::from(iterations)
        );
    }
}
