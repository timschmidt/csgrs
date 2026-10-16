//! An oblique Boolean result preserves typed exchange blockers and recovery.
use hyperbrep::{Matrix4, OperationContext, SurfaceKind, WorkBudget, boolean, builder};
use hyperbrep_io::step::{StepBody, StepWriteOptions, WriteErrorKind, write_advanced};
#[allow(dead_code)]
#[path = "../../../hyperbrep/tests/support/oblique.rs"]
mod oblique;

pub fn failures_and_recovery() {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    });
    let options = StepWriteOptions::new("oblique.step", "2026-10-16T00:00:00");
    for map in [
        Matrix4::identity(),
        oblique::similarity(&oblique::q(1, 1000), true),
    ] {
        let (half, band) = oblique::torus_band_operands(&map);
        for result in [
            boolean::intersection(&half, &band).unwrap(),
            boolean::difference(&half, &band).unwrap(),
        ] {
            let json = result.model().to_json().unwrap();
            let replay =
                hyperbrep::RawModel::from_json(&json, &hyperbrep::ArchiveLimits::default())
                    .unwrap()
                    .validate()
                    .unwrap();
            let error = write_advanced(
                &[StepBody {
                    solid: replay.solids().next().unwrap(),
                    name: "oblique result",
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
                    WriteErrorKind::UnsupportedSurface(SurfaceKind::Torus)
                ),
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
