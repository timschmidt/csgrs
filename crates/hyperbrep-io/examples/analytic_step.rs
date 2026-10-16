//! Construct/import, edit, export and reimport exact analytic STEP.
//! Run with an output directory; the emitted files also serve independent readers.
use hyperbrep::{Matrix4, Model, OperationContext, Real, SolidRef, WorkBudget, builder};
use hyperbrep_io::step::{
    ImportLimits, StepBody, StepFile, StepReadOptions, StepWriteOptions, write_advanced,
};
use std::{error::Error, path::Path};

fn context() -> OperationContext {
    OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    })
}
fn save(directory: &Path, name: &str, solid: SolidRef<'_>) -> Result<(), Box<dyn Error>> {
    let file_name = format!("{name}.step");
    let output = write_advanced(
        &[StepBody { solid, name }],
        &StepWriteOptions::new(&file_name, "2026-10-16T00:00:00"),
        &context(),
    )
    .result
    .map_err(|e| format!("solver budget: {e:?}"))??;
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default())?;
    let imported = file
        .import_representation(
            output.representation(),
            &ImportLimits::default(),
            &context(),
        )
        .result
        .map_err(|e| format!("solver budget: {e:?}"))??;
    let volume = imported.bodies()[0].solid().volume()?;
    assert_eq!(
        (volume - solid.volume()?).exact_rational_normal_form(),
        Some(hyperreal::Rational::zero())
    );
    let counts = imported.bodies()[0].model().counts();
    assert_eq!(counts.vertices, output.bodies()[0].vertices().len());
    assert_eq!(counts.edges, output.bodies()[0].edges().len());
    std::fs::write(directory.join(file_name), output.text())?;
    println!(
        "{name}: vertices={}, edges={}, faces={}, bytes={}",
        counts.vertices,
        counts.edges,
        counts.faces,
        output.text().len()
    );
    Ok(())
}
fn imported(text: &str, id: u64) -> Result<Model, Box<dyn Error>> {
    let file = StepFile::read(text.as_bytes(), &StepReadOptions::default())?;
    let imported = file
        .import_representation(id, &ImportLimits::default(), &context())
        .result
        .map_err(|e| format!("solver budget: {e:?}"))??;
    Ok(imported.bodies()[0].model().clone())
}
fn main() -> Result<(), Box<dyn Error>> {
    let directory = std::env::args_os()
        .nth(1)
        .ok_or("usage: analytic_step OUTPUT_DIRECTORY")?;
    let directory = Path::new(&directory);
    std::fs::create_dir_all(directory)?;
    let native = builder::cylinder(5.into(), 7.into())?;
    save(
        directory,
        "native-cylinder",
        native.bodies().next().unwrap(),
    )?;
    for (name, text, id) in [
        (
            "closed-cylinder",
            include_str!("../tests/fixtures/exact-cylinder.step"),
            10,
        ),
        (
            "supplied-cylinder",
            include_str!("../tests/fixtures/supplied-cylinder.step"),
            10,
        ),
        (
            "oblique-cylinder",
            include_str!("../tests/fixtures/oblique-cylinder.step"),
            10,
        ),
        (
            "radical-cylinder",
            include_str!("../tests/fixtures/radical-frame-cylinder.step"),
            10,
        ),
        (
            "planar-hole",
            include_str!("../tests/fixtures/faceted-through-hole.step"),
            502,
        ),
    ] {
        let model = imported(text, id)?;
        save(directory, name, model.solids().next().unwrap())?;
        if name == "closed-cylinder" {
            let edge = model
                .edges()
                .find(|e| e.curve().circular_geometry().is_some())
                .unwrap();
            let (a, b) = edge.domain().try_real_endpoints().unwrap();
            let model = edge
                .split(((a + b) / Real::from(2))?.into())?
                .into_model()
                .transformed(&Matrix4::affine_nonuniform_scale([
                    -Real::one(),
                    Real::one(),
                    Real::one(),
                ]))?;
            save(
                directory,
                "split-reflected-cylinder",
                model.solids().next().unwrap(),
            )?;
        }
    }
    Ok(())
}
