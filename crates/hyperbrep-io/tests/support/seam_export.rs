//! Complete torus and cone exchange on the original seam-cut fixtures.
use hyperbrep::{OperationContext, WorkBudget, boolean, builder};
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
    let options = StepWriteOptions::new("seams.step", "2026-10-16T00:00:00");
    for kind in 0..3 {
        let (source, cutter, volume, area) = seam::fixture(kind);
        let half = boolean::intersection(&source, &cutter).unwrap();
        torus::round_trip(half.bodies().next().unwrap(), &options, &volume, &area, &[]);
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
