//! Import independent planar advanced BREP, edit it, and export exact faceted STEP.
//! Run with an output directory. Export is an explicit choice of STEP encoding;
//! these planar line boundaries retain their geometry without tessellation.
use hyperbrep::{Matrix4, OperationContext, Real, SolidRef, WorkBudget};
use hyperbrep_io::step::{
    FacetedBody, ImportLimits, StepFile, StepReadOptions, StepWriteOptions, WriteLengthUnit,
    write_faceted,
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
    let mut options = StepWriteOptions::new(&file_name, "2026-10-15T00:00:00");
    options.length_unit = WriteLengthUnit {
        name: "millimetre",
        metres_per_unit: (Real::one() / Real::from(1000))?,
    };
    options.representation_name = name;
    let output = write_faceted(&[FacetedBody { solid, name }], &options, &context())
        .result
        .map_err(|error| format!("solver budget: {error:?}"))??;
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default())?;
    let replay = file
        .import_representation(
            output.representation(),
            &ImportLimits::default(),
            &context(),
        )
        .result
        .map_err(|error| format!("solver budget: {error:?}"))??;
    let counts = replay.bodies()[0].model().counts();
    assert_eq!(replay.bodies()[0].solid().volume()?, solid.volume()?);
    assert_eq!(counts.solids, 1);
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
fn main() -> Result<(), Box<dyn Error>> {
    let directory = std::env::args_os()
        .nth(1)
        .ok_or("usage: advanced_step OUTPUT_DIRECTORY")?;
    let directory = Path::new(&directory);
    std::fs::create_dir_all(directory)?;
    for (name, text, expected_mm3) in [
        (
            "advanced-box-3d",
            include_str!("../tests/fixtures/occt-advanced-box-3d.step"),
            24,
        ),
        (
            "advanced-box-pcurves",
            include_str!("../tests/fixtures/occt-advanced-box.step"),
            24,
        ),
        (
            "advanced-hole",
            include_str!("../tests/fixtures/occt-advanced-hole.step"),
            20,
        ),
    ] {
        let file = StepFile::read(text.as_bytes(), &StepReadOptions::default())?;
        let imported = file
            .import_representation(10, &ImportLimits::default(), &context())
            .result
            .map_err(|error| format!("solver budget: {error:?}"))??;
        let body = &imported.bodies()[0];
        assert_eq!(
            body.solid().volume()?,
            (Real::from(expected_mm3) / Real::from(1_000_000_000))?
        );
        save(directory, name, body.solid())?;
        if name == "advanced-hole" {
            let edited = body
                .model()
                .edges()
                .next()
                .ok_or("missing hole edge")?
                .split((Real::one() / Real::from(2))?.into())?
                .into_model()
                .transformed(&Matrix4::affine_nonuniform_scale([
                    -Real::one(),
                    Real::one(),
                    Real::one(),
                ]))?;
            let solid = edited.solids().next().ok_or("missing edited solid")?;
            assert_eq!(solid.volume()?, body.solid().volume()?);
            save(directory, "advanced-hole-edited", solid)?;
        }
    }
    Ok(())
}
