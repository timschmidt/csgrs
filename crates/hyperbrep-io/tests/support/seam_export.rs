//! Unsupported analytic STEP families stay explicit after successful Booleans.
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
    let options = StepWriteOptions::new("seams.step", "2026-10-16T00:00:00");
    for kind in 0..3 {
        let (source, cutter, _, _) = seam::fixture(kind);
        let half = boolean::intersection(&source, &cutter).unwrap();
        let error = write_advanced(
            &[StepBody {
                solid: half.bodies().next().unwrap(),
                name: "half",
            }],
            &options,
            &context,
        )
        .result
        .unwrap()
        .unwrap_err();
        assert!(
            matches!(
                error.kind,
                WriteErrorKind::UnsupportedSurface(SurfaceKind::Torus | SurfaceKind::Cone)
            ),
            "{error:?}"
        );
        assert_eq!(error.body, Some(0));
        assert!(error.entity.is_some());
        seam::reload(half.model());
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
