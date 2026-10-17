//! Exact partial-depth Boolean results through STEP and continued modeling.
use hyperbrep::*;
use hyperbrep_io::step::*;
use std::error::Error;
#[allow(dead_code)]
#[path = "../../../hyperbrep/tests/support/axial_shells.rs"]
pub mod native;

pub fn round_trip(kind: usize, reflected: bool) -> Result<(String, String), Box<dyn Error>> {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1_000,
    });
    let source = context
        .evaluate(|| native::workflow(kind, reflected))
        .result
        .map_err(|e| format!("native budget: {e:?}"))?;
    let map = native::frame(reflected);
    let output = write_advanced(
        &[StepBody {
            solid: source.bodies().next().unwrap(),
            name: "axial shell",
        }],
        &StepWriteOptions::new("axial-shell.step", "2026-10-17T00:00:00"),
        &context,
    )
    .result
    .map_err(|e| format!("write budget: {e:?}"))??;
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default())?;
    let imported = file
        .import_representation(output.representation(), &ImportLimits::default(), &context)
        .result
        .map_err(|e| format!("import budget: {e:?}"))??;
    assert_eq!(imported.bodies().len(), 1);
    let cold = native::replay(imported.bodies()[0].model());
    native::continued(&cold, kind, &map);
    let round = BodySet::from_solid(cold.solids().next().unwrap());
    for result in [
        boolean::union(&source, &round)?,
        boolean::intersection(&round, &source)?,
    ] {
        native::verify(&native::replay(result.model()), kind, &map);
    }
    let empty = boolean::difference(&source, &round)?;
    assert!(empty.is_empty());
    native::verify(boolean::union(&empty, &round)?.model(), kind, &map);
    Ok((output.text().to_owned(), cold.to_json()?))
}
