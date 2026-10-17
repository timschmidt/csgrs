//! Exact cone STEP exchange, source parameter maps and repeated native modeling.
use hyperbrep::*;
use hyperbrep_io::step::*;
use std::{error::Error, path::Path};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
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
    body: &BodySet,
    volume: &Real,
    area: &Real,
) -> Result<BodySet, Box<dyn Error>> {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1_000,
    });
    let file_name = format!("{name}.step");
    let output = write_advanced(
        &[StepBody {
            solid: body.bodies().next().unwrap(),
            name,
        }],
        &StepWriteOptions::new(&file_name, "2026-10-17T00:00:00"),
        &context,
    )
    .result
    .map_err(|e| format!("solver budget: {e:?}"))??;
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default())?;
    let imported = file
        .import_representation(output.representation(), &ImportLimits::default(), &context)
        .result
        .map_err(|e| format!("solver budget: {e:?}"))??;
    assert_eq!(imported.bodies().len(), 1);
    let imported_body = &imported.bodies()[0];
    let model = imported_body.model();
    equal(&imported_body.solid().volume()?, volume);
    equal(
        &model
            .faces()
            .map(|f| f.area().unwrap())
            .fold(Real::zero(), |a, b| a + b),
        area,
    );
    // Native cone V is slant distance. STEP V is the referenced LINE's
    // parameter. Retain this exact affine map instead of assuming equality.
    let mut mapped_cones = 0;
    for face in imported_body.faces() {
        if face.face.surface().kind() == SurfaceKind::Cone {
            let uv = Point2::new(Real::zero(), Real::one());
            let source_uv = face.parameters.map(&uv);
            assert!(file.syntax().entity(face.parameters.surface).is_some());
            let expected = face.parameters.origin.clone() + &face.parameters.v;
            equal(&source_uv.x, &expected.x);
            equal(&source_uv.y, &expected.y);
            mapped_cones += 1;
        }
    }
    assert!(mapped_cones > 0);
    let json = model.to_json()?;
    let replay = RawModel::from_json(&json, &ArchiveLimits::default())?.validate()?;
    assert_eq!(replay.to_json()?, json);
    std::fs::write(directory.join(file_name), output.text())?;
    println!(
        "{{\"case\":\"{name}\",\"faces\":{},\"cone_maps\":{mapped_cones},\"bytes\":{}}}",
        model.counts().faces,
        output.text().len()
    );
    Ok(BodySet::from_solid(replay.solids().next().unwrap()))
}

fn main() -> Result<(), Box<dyn Error>> {
    let directory = std::env::args_os()
        .nth(1)
        .ok_or("usage: cone_step OUTPUT_DIRECTORY")?;
    let directory = Path::new(&directory);
    std::fs::create_dir_all(directory)?;
    let pi = Real::pi();
    let root = Real::from(5).sqrt()?;
    let whole_volume = Real::from(78) * &pi;
    let whole_area = (Real::from(29) + Real::from(21) * &root) * &pi;
    let half_volume = Real::from(39) * &pi;
    let half_area = &whole_area * q(1, 2) + Real::from(42);
    let rotation = Matrix4::from_row_major([
        q(3, 5),
        -q(4, 5),
        0.into(),
        0.into(),
        q(4, 5),
        q(3, 5),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let source = builder::cone_frustum(5.into(), 2.into(), 6.into())?;
    let source = save(directory, "whole", &source, &whole_volume, &whole_area)?;
    let cutter = builder::cuboid(p(0, -20, -20), p(20, 20, 20))?.transformed(&rotation)?;
    let positive = save(
        directory,
        "positive",
        &boolean::intersection(&source, &cutter)?,
        &half_volume,
        &half_area,
    )?;
    let negative = save(
        directory,
        "negative",
        &boolean::difference(&source, &cutter)?,
        &half_volume,
        &half_area,
    )?;
    let axial = builder::cuboid(p(-20, -20, 1), p(20, 20, 20))?;
    let band = save(
        directory,
        "band",
        &boolean::intersection(&positive, &axial)?,
        &(q(665, 24) * &pi),
        &((Real::from(97) + Real::from(65) * &root) * &pi * q(1, 8) + q(65, 2)),
    )?;
    let lower = save(
        directory,
        "lower",
        &boolean::difference(&positive, &axial)?,
        &(q(271, 24) * &pi),
        &((Real::from(181) + Real::from(19) * &root) * &pi * q(1, 8) + q(19, 2)),
    )?;
    for (a, b, name, volume, area) in [
        (
            &positive,
            &negative,
            "reunion-first",
            &whole_volume,
            &whole_area,
        ),
        (
            &negative,
            &positive,
            "reunion-second",
            &whole_volume,
            &whole_area,
        ),
        (
            &band,
            &lower,
            "band-reunion-first",
            &half_volume,
            &half_area,
        ),
        (
            &lower,
            &band,
            "band-reunion-second",
            &half_volume,
            &half_area,
        ),
    ] {
        let restored = save(directory, name, &boolean::union(a, b)?, volume, area)?;
        equal(
            &boolean::intersection(&restored, &cutter)?
                .bodies()
                .next()
                .unwrap()
                .volume()?,
            &half_volume,
        );
        let empty = boolean::difference(&restored, &restored)?;
        assert!(empty.is_empty());
        equal(
            &boolean::union(&empty, &restored)?
                .bodies()
                .next()
                .unwrap()
                .volume()?,
            volume,
        );
    }
    // The apex Z=35/3 is not a finite decimal. A finite cap anchor preserves
    // the original cone exactly in the LINE revolution representation.
    save(
        directory,
        "shifted-apex",
        &builder::cone_frustum(5.into(), 2.into(), 7.into())?,
        &(Real::from(91) * &pi),
        &((Real::from(29) + Real::from(7) * Real::from(58).sqrt()?) * &pi),
    )?;
    Ok(())
}
