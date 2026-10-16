//! Exact sector and band Booleans retain explicit exchange-family blockers.
use hyperbrep::{OperationContext, SurfaceKind, WorkBudget, boolean, builder};
use hyperbrep_io::step::{StepBody, StepWriteOptions, WriteErrorKind, write_advanced};
#[allow(dead_code)]
#[path = "../../../hyperbrep/tests/support/sectors.rs"]
mod sectors;
use sectors::seam;

pub fn failures_and_recovery() {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    });
    let options = StepWriteOptions::new("sectors.step", "2026-10-16T00:00:00");
    for kind in 0..5 {
        let (half, cutter, _, _) = sectors::fixture(kind);
        for (name, result) in [
            (
                "sector-intersection",
                boolean::intersection(&half, &cutter).unwrap(),
            ),
            (
                "sector-difference",
                boolean::difference(&half, &cutter).unwrap(),
            ),
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
            let expected = if matches!(kind, 2 | 4) {
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
