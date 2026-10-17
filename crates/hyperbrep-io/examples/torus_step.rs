//! Exact ring-torus STEP exchange followed by native modeling and archive replay.
use hyperbrep::*;
use hyperbrep_io::step::*;
use std::{error::Error, path::Path};

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
fn equal(a: &Real, b: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(a, b, STRICT_PREDICATES).value(),
        Some(std::cmp::Ordering::Equal)
    );
}
fn save(
    directory: &Path,
    name: &str,
    solid: SolidRef<'_>,
    volume: &Real,
    area: &Real,
) -> Result<BodySet, Box<dyn Error>> {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1_000,
    });
    let file_name = format!("{name}.step");
    let output = write_advanced(
        &[StepBody { solid, name }],
        &StepWriteOptions::new(&file_name, "2026-10-16T00:00:00"),
        &context,
    )
    .result
    .map_err(|e| format!("solver budget: {e:?}"))??;
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default())?;
    let imported = file
        .import_representation(output.representation(), &ImportLimits::default(), &context)
        .result
        .map_err(|e| format!("solver budget: {e:?}"))??;
    let model = imported.bodies()[0].model();
    equal(&model.solids().next().unwrap().volume()?, volume);
    equal(
        &model
            .faces()
            .map(|f| f.area().unwrap())
            .fold(Real::zero(), |a, b| a + b),
        area,
    );
    let json = model.to_json()?;
    let replay = RawModel::from_json(&json, &ArchiveLimits::default())?.validate()?;
    assert_eq!(replay.to_json()?, json);
    let counts = model.counts();
    std::fs::write(directory.join(file_name), output.text())?;
    println!(
        "{{\"case\":\"{name}\",\"faces\":{},\"edges\":{},\"vertices\":{},\"bytes\":{}}}",
        counts.faces,
        counts.edges,
        counts.vertices,
        output.text().len()
    );
    Ok(BodySet::from_solid(replay.solids().next().unwrap()))
}

fn main() -> Result<(), Box<dyn Error>> {
    let directory = std::env::args_os()
        .nth(1)
        .ok_or("usage: torus_step OUTPUT_DIRECTORY")?;
    let directory = Path::new(&directory);
    std::fs::create_dir_all(directory)?;
    let pi = Real::pi();
    let pi2 = &pi * &pi;
    let source = builder::torus(3.into(), 1.into())?;
    save(
        directory,
        "whole-torus",
        source.bodies().next().unwrap(),
        &(Real::from(6) * &pi2),
        &(Real::from(12) * &pi2),
    )?;
    let scale = (Real::one() / Real::from(1000))?;
    let reflected = source.transformed(&Matrix4::affine_nonuniform_scale([
        -&scale,
        scale.clone(),
        scale.clone(),
    ]))?;
    save(
        directory,
        "millimetre-reflected-torus",
        reflected.bodies().next().unwrap(),
        &(Real::from(6) * &pi2 * &scale * &scale * &scale),
        &(Real::from(12) * &pi2 * &scale * &scale),
    )?;
    let axial = builder::cuboid(p(-20, -20, 0), p(20, 20, 20))?;
    let half = boolean::intersection(&source, &axial)?;
    save(
        directory,
        "axial-half-torus",
        half.bodies().next().unwrap(),
        &(Real::from(3) * &pi2),
        &(Real::from(6) * &pi2 + Real::from(12) * &pi),
    )?;
    let longitudinal = builder::cuboid(p(0, -20, -20), p(20, 20, 20))?;
    let half = boolean::intersection(&source, &longitudinal)?;
    save(
        directory,
        "longitudinal-half-torus",
        half.bodies().next().unwrap(),
        &(Real::from(3) * &pi2),
        &(Real::from(6) * &pi2 + Real::from(2) * &pi),
    )?;
    let transverse = builder::cuboid(p(-20, 0, -20), p(20, 20, 20))?;
    let quarter = boolean::intersection(&half, &transverse)?;
    save(
        directory,
        "quarter-torus",
        quarter.bodies().next().unwrap(),
        &((Real::from(3) * &pi2) / Real::from(2))?,
        &(Real::from(3) * &pi2 + Real::from(2) * &pi),
    )?;
    let source = builder::torus(10.into(), 5.into())?;
    let cutter = builder::cuboid(p(-40, -40, 3), p(40, 40, 40))?;
    let band = boolean::intersection(&source, &cutter)?;
    let volume = Real::from(20)
        * &pi
        * (Real::from(25) * ((&pi / Real::from(2))? - (Real::from(3) / Real::from(5))?.asin()?)
            - Real::from(12));
    let area =
        Real::from(100) * &pi * (&pi - Real::from(2) * (Real::from(3) / Real::from(4))?.atan()?)
            + Real::from(160) * &pi;
    let imported = save(
        directory,
        "rational-band-torus",
        band.bodies().next().unwrap(),
        &volume,
        &area,
    )?;
    let recut = boolean::intersection(&imported, &cutter)?;
    save(
        directory,
        "recut-band-torus",
        recut.bodies().next().unwrap(),
        &volume,
        &area,
    )?;
    assert!(boolean::difference(&imported, &cutter)?.is_empty());
    Ok(())
}
