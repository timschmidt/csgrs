//! Complete native Boolean -> transform -> STEP -> replay -> Boolean workflows.
use hyperbrep::*;
use hyperbrep_io::step::*;
use std::error::Error;

#[allow(dead_code)]
#[path = "../../../hyperbrep/tests/support/axial_cuts.rs"]
mod native;

pub fn round_trip(disjoint: bool, reflected: bool) -> Result<(String, String), Box<dyn Error>> {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1_000,
    });
    let source = context
        .evaluate(|| native::unchanged_cut(disjoint))
        .result
        .map_err(|error| format!("native budget: {error:?}"))?;
    let sign = if reflected { -Real::one() } else { Real::one() };
    let q = |n| (Real::from(n) / Real::from(5)).unwrap();
    let map = Matrix4::from_row_major([
        0.into(),
        0.into(),
        1.into(),
        7.into(),
        q(3) * &sign,
        -q(4),
        0.into(),
        (-2).into(),
        q(4) * sign,
        q(3),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let moved = source.model().transformed(&map)?;
    let cold = native::replay(&moved);
    native::verify_cylinder(&cold, &map);
    let name = format!("axial-disjoint-{disjoint}-reflected-{reflected}");
    let output = write_advanced(
        &[StepBody {
            solid: cold.solids().next().unwrap(),
            name: &name,
        }],
        &StepWriteOptions::new("axial.step", "2026-10-17T00:00:00"),
        &context,
    )
    .result
    .map_err(|error| format!("write budget: {error:?}"))??;
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default())?;
    let imported = file
        .import_representation(output.representation(), &ImportLimits::default(), &context)
        .result
        .map_err(|error| format!("import budget: {error:?}"))??;
    assert_eq!(imported.bodies().len(), 1);
    let round = native::replay(imported.bodies()[0].model());
    native::verify_cylinder(&round, &map);
    let first = BodySet::from_solid(cold.solids().next().unwrap());
    let second = BodySet::from_solid(round.solids().next().unwrap());
    for result in [
        boolean::union(&first, &second)?,
        boolean::intersection(&second, &first)?,
    ] {
        native::verify_cylinder(result.model(), &map);
        native::verify_cylinder(&native::replay(result.model()), &map);
    }
    let empty = boolean::difference(&first, &second)?;
    assert!(empty.is_empty());
    native::verify_cylinder(boolean::union(&empty, &second)?.model(), &map);
    Ok((output.text().to_owned(), round.to_json()?))
}
