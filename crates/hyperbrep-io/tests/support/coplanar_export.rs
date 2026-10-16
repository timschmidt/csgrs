//! Repeated exact curved Booleans retain explicit exchange-family blockers.
use hyperbrep::{OperationContext, SurfaceKind, WorkBudget, boolean, builder};
use hyperbrep_io::step::{StepBody, StepWriteOptions, WriteErrorKind, write_advanced};
#[allow(dead_code)]
#[path = "../../../hyperbrep/tests/support/seam_cuts.rs"]
mod seam;

pub fn failures_and_recovery() {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    });
    let options = StepWriteOptions::new("coplanar.step", "2026-10-16T00:00:00");
    for kind in 0..3 {
        let (source, cutter, _, _) = seam::fixture(kind);
        let half = boolean::intersection(&source, &cutter).unwrap();
        let other = boolean::difference(&source, &cutter).unwrap();
        for (name, result) in [
            ("recut", boolean::intersection(&half, &cutter).unwrap()),
            ("reconstructed", boolean::union(&half, &other).unwrap()),
        ] {
            let replay = seam::reload(result.model());
            let error = write_advanced(
                &[StepBody {
                    solid: replay.solids().next().unwrap(),
                    name,
                }],
                &options,
                &context,
            )
            .result
            .unwrap()
            .unwrap_err();
            let expected = if kind == 2 {
                SurfaceKind::Cone
            } else {
                SurfaceKind::Torus
            };
            assert!(
                matches!(error.kind, WriteErrorKind::UnsupportedSurface(kind) if kind == expected),
                "{error:?}"
            );
            assert_eq!(error.body, Some(0));
            assert!(error.entity.is_some());
        }
    }
    let cylinder = builder::cylinder(2.into(), 4.into()).unwrap();
    let output = write_advanced(
        &[StepBody {
            solid: cylinder.bodies().next().unwrap(),
            name: "recovery",
        }],
        &options,
        &context,
    )
    .result
    .unwrap()
    .unwrap();
    assert!(!output.text().is_empty());
}
