//! Import an exact analytic cylinder, split an edge and replay the native model.
use hyperbrep::{ArchiveLimits, Curve3Kind, OperationContext, RawModel, Real, WorkBudget};
use hyperbrep_io::step::{ImportLimits, StepFile, StepReadOptions};
use hyperlimit::{PredicateOutcome, compare_reals};
use std::cmp::Ordering;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = include_bytes!("../tests/fixtures/exact-cylinder.step");
    let file = StepFile::read(source, &StepReadOptions::default())?;
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1_000,
    });
    let imported = file
        .import_representation(10, &ImportLimits::default(), &context)
        .result??;
    let body = &imported.bodies()[0];
    let edge = body
        .explicit_edges()
        .find(|edge| edge.edge.curve().kind() == Curve3Kind::CircleArc)
        .expect("the fixture has circular edges");
    assert_eq!(edge.edge.start(), edge.edge.end());
    let split = edge
        .edge
        .split((edge.edge.curve().domain().represented_start()? + Real::pi()).into())?
        .into_model();
    let archive = split.to_json()?;
    let replay = RawModel::from_json(&archive, &ArchiveLimits::default())?.validate()?;
    assert_eq!(replay.to_json()?, archive);
    let volume = replay
        .solids()
        .next()
        .expect("the fixture has one solid")
        .volume()?;
    assert!(matches!(
        compare_reals(
            &volume,
            &(Real::from(175) * Real::pi()),
            context.predicate_policy()
        ),
        PredicateOutcome::Decided {
            value: Ordering::Equal,
            ..
        }
    ));
    println!(
        "Imported and replayed an exact cylinder: {} faces, {} edges after splitting; volume 175*pi cubic metres.",
        replay.counts().faces,
        replay.counts().edges
    );
    Ok(())
}
