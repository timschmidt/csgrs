//! Complete public normalized-region extrusion and STEP consumer.
use hyperbrep::*;
use hyperbrep_io::step::*;
use hypercurve::{
    CurvePath2, CurveRegion2, LineSeg2, OffsetCap, OffsetCornerStyle2, Point2 as CurvePoint2,
};
use std::error::Error;

fn q(a: i32, b: i32) -> Real {
    (Real::from(a) / Real::from(b)).unwrap()
}

fn equal(a: Real, b: Real) {
    assert!((a - b).exact_rational_normal_form().unwrap().is_zero());
}

fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let result = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(result.to_json().unwrap(), json);
    result
}

fn verify(model: &Model, cap: OffsetCap, map: &Matrix4) {
    let solids = model.solids().collect::<Vec<_>>();
    assert_eq!(solids.len(), 1);
    let solid = solids[0];
    let volume = if cap == OffsetCap::Round {
        Real::from(30) + Real::from(3) * Real::pi()
    } else {
        Real::from(30)
    };
    let area = if cap == OffsetCap::Round {
        Real::from(50) + Real::from(8) * Real::pi()
    } else {
        Real::from(62)
    };
    equal(solid.volume().unwrap(), volume);
    equal(
        model
            .faces()
            .map(|f| f.area().unwrap())
            .fold(Real::zero(), |a, b| a + b),
        area,
    );
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
    let delta: Real = "1e-12".parse().unwrap();
    let normal_point = |distance: Real| {
        Point3::new(
            q(3, 2) - q(4, 5) * &distance,
            Real::from(2) + q(3, 5) * distance,
            q(3, 2),
        )
    };
    for (point, location) in [
        (normal_point(Real::zero()), SolidPointLocation::Inside),
        (normal_point(Real::one()), SolidPointLocation::Boundary),
        (
            normal_point(Real::one() - &delta),
            SolidPointLocation::Inside,
        ),
        (
            normal_point(Real::one() + &delta),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(Real::zero(), Real::zero(), q(3, 2)),
            if cap == OffsetCap::Round {
                SolidPointLocation::Inside
            } else {
                SolidPointLocation::Boundary
            },
        ),
        (
            Point3::new(-q(3, 5), -q(4, 5), q(3, 2)),
            if cap == OffsetCap::Round {
                SolidPointLocation::Boundary
            } else {
                SolidPointLocation::Outside
            },
        ),
        (
            Point3::new(q(3, 2), Real::from(2), Real::zero()),
            SolidPointLocation::Boundary,
        ),
        (
            Point3::new(q(3, 2), Real::from(2), -delta),
            SolidPointLocation::Outside,
        ),
    ] {
        assert_eq!(
            solid
                .classify_point(&map.transform_point3(&point).unwrap())
                .unwrap(),
            location
        );
    }
}

fn round_trip(cap: OffsetCap, reverse: bool) -> Result<(String, String), Box<dyn Error>> {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1_000,
    });
    let name = format!("{cap:?}-reverse-{reverse}");
    let (a, b) = if reverse {
        ((3, 4), (0, 0))
    } else {
        ((0, 0), (3, 4))
    };
    let path = CurvePath2::try_new(vec![
        LineSeg2::try_new(
            CurvePoint2::new(a.0.into(), a.1.into()),
            CurvePoint2::new(b.0.into(), b.1.into()),
        )?
        .into(),
    ])?;
    let region = hypercurve::evaluate_under(hypercurve::PredicatePolicy::STRICT, || {
        CurveRegion2::stroke_path(&path, Real::one(), &OffsetCornerStyle2::Round, cap)
    })
    .certified()
    .ok_or("stroke did not retain certified evidence")??;
    let expected = if cap == OffsetCap::Round {
        Real::from(10) + Real::pi()
    } else {
        Real::from(10)
    };
    equal(region.filled_area()?.unwrap(), expected);
    let boundaries = region.boundary_paths()?;
    assert_eq!(boundaries.len(), 1);
    let original = builder::extrude_curve_region(&region, Real::zero(), Real::from(3))?;
    verify(original.model(), cap, &Matrix4::identity());
    let map = Matrix4::affine_translation([Real::from(7), Real::from(-2), Real::one()]);
    let moved = original.model().transformed(&map)?;
    let cold = replay(&moved);
    verify(&cold, cap, &map);
    let output = write_advanced(
        &[StepBody {
            solid: cold.solids().next().unwrap(),
            name: &name,
        }],
        &StepWriteOptions::new("stroke.step", "2026-10-17T00:00:00"),
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
    let round = replay(imported.bodies()[0].model());
    verify(&round, cap, &map);
    let first = BodySet::from_solid(cold.solids().next().unwrap());
    let second = BodySet::from_solid(round.solids().next().unwrap());
    for result in [
        boolean::union(&first, &second)?,
        boolean::intersection(&second, &first)?,
    ] {
        verify(result.model(), cap, &map);
        verify(&replay(result.model()), cap, &map);
    }
    let empty = boolean::difference(&first, &second)?;
    assert!(empty.is_empty());
    verify(boolean::union(&empty, &second)?.model(), cap, &map);
    Ok((output.text().to_owned(), round.to_json()?))
}

fn main() -> Result<(), Box<dyn Error>> {
    let directory = std::env::args_os().nth(1).ok_or("output directory")?;
    let directory = std::path::Path::new(&directory);
    std::fs::create_dir_all(directory)?;
    for cap in [OffsetCap::Butt, OffsetCap::Round] {
        for reverse in [false, true] {
            let name = format!("{cap:?}-reverse-{reverse}");
            let (step, native) = round_trip(cap, reverse)?;
            std::fs::write(directory.join(format!("{name}.step")), &step)?;
            std::fs::write(directory.join(format!("{name}.json")), &native)?;
            println!(
                "case={name} outcome=certified step_bytes={} native_bytes={}",
                step.len(),
                native.len()
            );
        }
    }
    Ok(())
}
