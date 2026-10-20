//! Repeated torus and cone Booleans retain their exact exchange contracts.
use hyperbrep::{OperationContext, Real, WorkBudget, boolean, builder};
use hyperbrep_io::step::{StepBody, StepWriteOptions, write_advanced};
#[allow(dead_code)]
#[path = "torus_exchange.rs"]
mod torus;
use torus::seam;

pub fn exchange_and_recovery() {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    });
    let options = StepWriteOptions::new("coplanar.step", "2026-10-16T00:00:00");
    for kind in 0..3 {
        let (source, cutter, volume, area) = seam::fixture(kind);
        let half = boolean::intersection(&source, &cutter).unwrap();
        let other = boolean::difference(&source, &cutter).unwrap();
        for (name, result) in [
            ("recut", boolean::intersection(&half, &cutter).unwrap()),
            ("reconstructed", boolean::union(&half, &other).unwrap()),
        ] {
            let replay = seam::reload(result.model());
            let (expected_volume, expected_area) = if name == "recut" {
                (volume.clone(), area.clone())
            } else if kind == 2 {
                (
                    Real::from(78) * Real::pi(),
                    (Real::from(29) + Real::from(21) * Real::from(5).sqrt().unwrap()) * Real::pi(),
                )
            } else {
                let full = Real::from(40) * Real::pi() * Real::pi();
                (full.clone(), full)
            };
            torus::round_trip(
                replay.solids().next().unwrap(),
                &options,
                &expected_volume,
                &expected_area,
                &[],
            );
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
