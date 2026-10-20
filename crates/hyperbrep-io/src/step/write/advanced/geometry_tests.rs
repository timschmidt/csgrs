use super::*;
use hyperbrep::Curve3;

fn equal(actual: &Real, expected: &Real) {
    assert_eq!(
        compare_reals(actual, expected, hyperbrep::STRICT_PREDICATES).value(),
        Some(Ordering::Equal)
    );
}

fn circle() -> Curve3 {
    Curve3::circle_arc(
        Point3::new(2.into(), 3.into(), 4.into()),
        Vector3::x(),
        Vector3::y(),
        5.into(),
        Real::pi() * Real::from(2),
        Real::pi() * Real::from(4),
    )
    .unwrap()
}

#[test]
fn extrusion_charts_keep_shifted_reversed_angles_and_signed_axial_units() {
    let scale = (Real::one() / Real::from(1000)).unwrap();
    for reversed in [false, true] {
        let profile = if reversed {
            circle().reversed().unwrap()
        } else {
            circle()
        };
        // Subdivision retains the original angular identity and an interior
        // source domain; neither endpoint may be reduced modulo a turn.
        let profile = profile.split_at(&(Real::pi() * Real::from(3))).unwrap().1;
        for negative in [false, true] {
            let direction = Vector3::z() * Real::from(if negative { -3 } else { 3 });
            let source = Surface::extrusion(profile.clone(), direction).unwrap();
            let mut out = Output::new(Default::default());
            let (_, parameters, flips) = surface(&mut out, &source, &scale).unwrap();
            assert!(out.text.contains("CYLINDRICAL_SURFACE"));
            assert_eq!(flips, reversed != negative);
            for t in [
                Real::from(3) * Real::pi(),
                (Real::from(7) * Real::pi() / Real::from(2)).unwrap(),
                Real::from(4) * Real::pi(),
            ] {
                let actual = parameters.map(&Point2::new(t.clone(), 2.into()));
                let expected = if reversed {
                    Real::from(6) * Real::pi() - &t
                } else {
                    t.clone()
                };
                equal(&actual.x, &expected);
                equal(&actual.y, &Real::from(if negative { -6000 } else { 6000 }));
                let point = source
                    .point_at(&{
                        let uv = &Point2::new(t, 2.into());
                        hypercurve::Point2::new(uv.x.clone(), uv.y.clone()).into()
                    })
                    .map(|point| {
                        point
                            .try_coordinates()
                            .expect("represented surface fixture")
                            .clone()
                    })
                    .unwrap();
                equal(
                    &point.x,
                    &(Real::from(2) + Real::from(5) * actual.x.clone().cos()),
                );
                equal(&point.y, &(Real::from(3) + Real::from(5) * actual.x.sin()));
                equal(&point.z, &(Real::from(4) + actual.y * &scale));
            }
        }
    }
}

#[test]
fn line_extrusion_maps_an_oblique_nonunit_chart_exactly() {
    let profile = Curve3::line(
        Point3::new(1.into(), 2.into(), 3.into()),
        Point3::new(3.into(), 2.into(), 3.into()),
    )
    .unwrap();
    let source = Surface::extrusion(profile, Vector3::new([1.into(), 0.into(), 3.into()])).unwrap();
    let mut out = Output::new(Default::default());
    let (_, parameters, reversed) = surface(
        &mut out,
        &source,
        &(Real::one() / Real::from(1000)).unwrap(),
    )
    .unwrap();
    assert!(out.text.contains("PLANE"));
    assert!(!reversed);
    let mapped = parameters.map(&Point2::new(
        (Real::one() / Real::from(4)).unwrap(),
        2.into(),
    ));
    equal(&mapped.x, &Real::from(2500));
    equal(&mapped.y, &Real::from(6000));
}

#[test]
fn other_extrusions_remain_typed_and_leave_a_valid_retry_possible() {
    let ellipse = Curve3::ellipse_arc(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        2.into(),
        1.into(),
        Real::zero(),
        Real::pi(),
    )
    .unwrap();
    for source in [
        Surface::extrusion(circle(), Vector3::new([1.into(), 0.into(), 1.into()])).unwrap(),
        Surface::extrusion(ellipse, Vector3::z()).unwrap(),
    ] {
        let mut out = Output::new(Default::default());
        let error = surface(&mut out, &source, &Real::one()).unwrap_err();
        assert!(matches!(
            error.kind,
            WriteErrorKind::UnsupportedSurface(SurfaceKind::Extrusion)
        ));
        assert!(out.text.is_empty());
        let good = Surface::extrusion(circle(), Vector3::z()).unwrap();
        surface(&mut out, &good, &Real::one()).unwrap();
    }
}

#[test]
fn original_bezier_extrusion_fixture_now_keeps_its_chart_and_complete_solid() {
    use crate::step::{ImportLimits, StepFile, StepReadOptions, StepWriteOptions, write_advanced};
    use hyperbrep::{ArchiveLimits, OperationContext, RawModel, SolidPointLocation, WorkBudget};
    use hypercurve::{CurvePath2, LineSeg2, Point2 as ProfilePoint, QuadraticBezier2};

    // Preserve the exact former rejection fixture as a positive regression.
    let bezier = Curve3::rational_bezier(
        vec![
            Point3::origin(),
            Point3::new(1.into(), 1.into(), 0.into()),
            Point3::new(2.into(), 0.into(), 0.into()),
        ],
        vec![Real::one(); 3],
    )
    .unwrap();
    let source = Surface::extrusion(bezier, Vector3::z()).unwrap();
    for scale in [Real::one(), (Real::one() / Real::from(1000)).unwrap()] {
        let mut out = Output::new(Default::default());
        let (_, parameters, flips) = surface(&mut out, &source, &scale).unwrap();
        assert!(!flips);
        assert!(out.text.contains("B_SPLINE_CURVE(2,"));
        assert!(out.text.contains("B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.,1.)"));
        assert!(out.text.contains("RATIONAL_B_SPLINE_CURVE((1.,1.,1.))"));
        assert!(out.text.contains("SURFACE_OF_LINEAR_EXTRUSION"));
        let uv = Point2::new((Real::one() / Real::from(4)).unwrap(), 2.into());
        let mapped = parameters.map(&uv);
        equal(&mapped.x, &uv.x);
        equal(&mapped.y, &uv.y);
    }

    // x=2t, y=2t(1-t); area integral from x=0 to 2 is 2/3.
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(
            ProfilePoint::from_values(0, 0),
            ProfilePoint::from_values(1, 1),
            ProfilePoint::from_values(2, 0),
        )
        .into(),
        LineSeg2::try_new(
            ProfilePoint::from_values(2, 0),
            ProfilePoint::from_values(0, 0),
        )
        .unwrap()
        .into(),
    ])
    .unwrap();
    let bodies = hyperbrep::builder::extrude_path(&path, 0.into(), 1.into()).unwrap();
    let expected = (Real::from(2) / Real::from(3)).unwrap();
    equal(
        &bodies.bodies().next().unwrap().volume().unwrap(),
        &expected,
    );
    let context = OperationContext::strict(WorkBudget {
        refinements: 100_000,
        promotions: 10_000,
    });
    let output = write_advanced(
        &[super::super::StepBody {
            solid: bodies.bodies().next().unwrap(),
            name: "original Bezier fixture",
        }],
        &StepWriteOptions::new("bezier.step", "2026-10-18T00:00:00"),
        &context,
    )
    .result
    .unwrap()
    .unwrap();
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
    let imported = file
        .import_representation(output.representation(), &ImportLimits::default(), &context)
        .result
        .unwrap()
        .unwrap();
    assert_eq!(imported.bodies().len(), 1);
    let text = imported.bodies()[0].model().to_json().unwrap();
    let replay = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(replay.to_json().unwrap(), text);
    let solid = replay.solids().next().unwrap();
    equal(&solid.volume().unwrap(), &expected);
    for (numerator, location) in [
        (1, SolidPointLocation::Inside),
        (2, SolidPointLocation::Boundary),
        (3, SolidPointLocation::Outside),
    ] {
        assert_eq!(
            solid
                .classify_point(&Point3::new(
                    1.into(),
                    (Real::from(numerator) / Real::from(4)).unwrap(),
                    (Real::one() / Real::from(2)).unwrap(),
                ))
                .unwrap(),
            location
        );
    }
}
