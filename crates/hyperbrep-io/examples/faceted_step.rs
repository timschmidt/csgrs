//! Write native-authored, edited and transformed geometry for independent readers.
//! Run with an output directory; every coordinate is authored in exact metres.
use hyperbrep::{Matrix4, OperationContext, Point2, Point3, Real, SolidRef, WorkBudget, builder};
use hyperbrep_io::step::{StepBody, StepWriteOptions, WriteLengthUnit, write_faceted};
use std::{error::Error, path::Path};

fn ratio(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn profile(points: &[(i32, i32)]) -> Vec<Point2> {
    points
        .iter()
        .map(|&(x, y)| Point2::new(ratio(x, 1000), ratio(y, 1000)))
        .collect()
}
fn save(
    directory: &Path,
    name: &str,
    solids: &[SolidRef<'_>],
    scale: Real,
    unit: &str,
) -> Result<(), Box<dyn Error>> {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    });
    let file_name = format!("{name}.step");
    let mut options = StepWriteOptions::new(&file_name, "2026-10-15T00:00:00");
    options.length_unit = WriteLengthUnit {
        name: unit,
        metres_per_unit: scale,
    };
    options.author = "HyperBRep example";
    options.representation_name = name;
    let inputs = solids
        .iter()
        .map(|&solid| StepBody { solid, name })
        .collect::<Vec<_>>();
    let output = write_faceted(&inputs, &options, &context)
        .result
        .map_err(|error| format!("solver budget: {error:?}"))??;
    std::fs::write(directory.join(file_name), output.text())?;
    println!(
        "{name}: bodies={}, bytes={}, representation=#{}",
        output.bodies().len(),
        output.text().len(),
        output.representation()
    );
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let directory = std::env::args_os()
        .nth(1)
        .ok_or("usage: faceted_step OUTPUT_DIRECTORY")?;
    let directory = Path::new(&directory);
    std::fs::create_dir_all(directory)?;
    let box_body = builder::cuboid(
        Point3::origin(),
        Point3::new(ratio(2, 1000), ratio(3, 1000), ratio(4, 1000)),
    )?;
    let solid = box_body.bodies().next().unwrap();
    save(
        directory,
        "native-box",
        &[solid],
        ratio(1, 1000),
        "millimetre",
    )?;
    let translated = box_body.transformed(&Matrix4::affine_translation([
        ratio(10, 1000),
        Real::zero(),
        Real::zero(),
    ]))?;
    save(
        directory,
        "native-two",
        &[solid, translated.bodies().next().unwrap()],
        Real::one(),
        "metre",
    )?;
    let edited = box_body
        .model()
        .edges()
        .next()
        .unwrap()
        .split(ratio(1, 8).into())?;
    save(
        directory,
        "native-edited",
        &[edited.model().solids().next().unwrap()],
        Real::one(),
        "metre",
    )?;
    let hole = builder::extrude_region(
        &profile(&[(0, 0), (4, 0), (4, 3), (0, 3)]),
        &[profile(&[(1, 1), (3, 1), (3, 2), (1, 2)])],
        Real::zero(),
        ratio(2, 1000),
    )?;
    save(
        directory,
        "native-hole",
        &[hole.bodies().next().unwrap()],
        ratio(1, 1000),
        "millimetre",
    )?;
    let reflected = hole.transformed(&Matrix4::affine_nonuniform_scale([
        -Real::one(),
        Real::one(),
        Real::one(),
    ]))?;
    save(
        directory,
        "native-reflected",
        &[reflected.bodies().next().unwrap()],
        ratio(1, 1000),
        "millimetre",
    )?;
    let concave = builder::extrude(
        &profile(&[(0, 0), (3, 0), (3, 1), (1, 1), (1, 3), (0, 3)]),
        Real::zero(),
        ratio(2, 1000),
    )?;
    save(
        directory,
        "native-concave",
        &[concave.bodies().next().unwrap()],
        ratio(1, 1000),
        "millimetre",
    )?;
    let inch = ratio(127, 5000);
    let inch_body = builder::cuboid(
        Point3::origin(),
        Point3::new(inch.clone(), inch.clone(), inch.clone()),
    )?;
    save(
        directory,
        "native-inch",
        &[inch_body.bodies().next().unwrap()],
        inch,
        "inch",
    )?;
    Ok(())
}
