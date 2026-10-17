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
                let point = source.point_at(&Point2::new(t, 2.into())).unwrap();
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
    let bezier = Curve3::rational_bezier(
        vec![
            Point3::origin(),
            Point3::new(1.into(), 1.into(), 0.into()),
            Point3::new(2.into(), 0.into(), 0.into()),
        ],
        vec![Real::one(); 3],
    )
    .unwrap();
    for source in [
        Surface::extrusion(circle(), Vector3::new([1.into(), 0.into(), 1.into()])).unwrap(),
        Surface::extrusion(ellipse, Vector3::z()).unwrap(),
        Surface::extrusion(bezier, Vector3::z()).unwrap(),
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
