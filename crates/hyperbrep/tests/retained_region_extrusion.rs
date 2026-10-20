use hyperbrep::*;
use hypercurve::{
    Curve2, CurveCertainty, CurveFamily2, CurvePath2, CurveRegion2, FillRule, LineSeg2,
    Point2 as CurvePoint2, PredicatePolicy, QuadraticBezier2,
};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn p(x: i32, y: i32) -> CurvePoint2 {
    CurvePoint2::from_values(x, y)
}

fn equal(actual: Real, expected: Real) {
    assert!(
        (actual - expected)
            .exact_rational_normal_form()
            .unwrap()
            .is_zero()
    );
}

fn parabola(offset: i32, reverse: bool) -> CurvePath2 {
    // x = -2 + 4t + offset, y = 4 - 16t + 16t²: y = (x-offset)².
    let path = CurvePath2::try_new(vec![
        QuadraticBezier2::new(p(offset - 2, 4), p(offset, -4), p(offset + 2, 4)).into(),
        LineSeg2::try_new(p(offset + 2, 4), p(offset - 2, 4))
            .unwrap()
            .into(),
    ])
    .unwrap();
    if reverse {
        path.reversed().unwrap()
    } else {
        path
    }
}

fn hole(reverse: bool) -> CurvePath2 {
    let points = [
        CurvePoint2::new(q(-1, 2), 1.into()),
        CurvePoint2::new(q(1, 2), 1.into()),
        CurvePoint2::new(q(1, 2), 2.into()),
        CurvePoint2::new(q(-1, 2), 2.into()),
    ];
    let path = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::from(
                    LineSeg2::try_new(points[i].clone(), points[(i + 1) % 4].clone()).unwrap(),
                )
            })
            .collect(),
    )
    .unwrap();
    if reverse {
        path.reversed().unwrap()
    } else {
        path
    }
}

fn profile(reverse: bool, components: bool, affine: bool) -> CurveRegion2 {
    let mut paths = vec![parabola(0, reverse)];
    if components {
        paths.extend([hole(reverse), parabola(8, reverse)]);
    }
    let region = CurveRegion2::try_from_boundary_paths(&paths, FillRule::EvenOdd).unwrap();
    assert!(region.boundary_loops().iter().any(|boundary| {
        boundary
            .curves()
            .iter()
            .any(|curve| curve.family() == CurveFamily2::Line && curve.geometry().is_none())
    }));
    if affine {
        region
            .transform_affine(
                &(-2).into(),
                &0.into(),
                &0.into(),
                &3.into(),
                &7.into(),
                &(-1).into(),
            )
            .unwrap()
    } else {
        region
    }
}

fn replay(model: &Model) -> Model {
    let text = model.to_json().unwrap();
    let cold = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), text);
    cold
}

fn verify(model: &Model, components: bool, affine: bool) {
    let scale = if affine { 6 } else { 1 };
    equal(
        model.solids().map(|solid| solid.volume().unwrap()).sum(),
        ((if components { 61 } else { 32 }) * scale).into(),
    );
    assert_eq!(model.solids().count(), if components { 2 } else { 1 });
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
    let gap: Real = "1e-12".parse().unwrap();
    let mut cases = vec![
        (1.into(), 2.into(), SolidPointLocation::Inside),
        (1.into(), 1.into(), SolidPointLocation::Boundary),
        (1.into(), Real::one() + &gap, SolidPointLocation::Inside),
        (1.into(), Real::one() - &gap, SolidPointLocation::Outside),
        (0.into(), 4.into(), SolidPointLocation::Boundary),
        (0.into(), Real::from(4) - &gap, SolidPointLocation::Inside),
        (0.into(), Real::from(4) + &gap, SolidPointLocation::Outside),
        (4.into(), 2.into(), SolidPointLocation::Outside),
    ];
    if components {
        cases.extend([
            (0.into(), q(3, 2), SolidPointLocation::Outside),
            (q(1, 2), q(3, 2), SolidPointLocation::Boundary),
            (q(1, 2) + &gap, q(3, 2), SolidPointLocation::Inside),
            (q(1, 2) - &gap, q(3, 2), SolidPointLocation::Outside),
            (8.into(), 2.into(), SolidPointLocation::Inside),
        ]);
    }
    for (x, y, expected) in cases {
        let (x, y) = if affine {
            (
                Real::from(-2) * x + Real::from(7),
                Real::from(3) * y - Real::one(),
            )
        } else {
            (x, y)
        };
        let point = Point3::new(x, y, q(3, 2));
        let mut location = SolidPointLocation::Outside;
        for solid in model.solids() {
            match solid.classify_point(&point).unwrap() {
                SolidPointLocation::Inside => {
                    location = SolidPointLocation::Inside;
                    break;
                }
                SolidPointLocation::Boundary => location = SolidPointLocation::Boundary,
                SolidPointLocation::Outside => {}
            }
        }
        assert_eq!(location, expected, "{point:?}");
    }
}

#[test]
fn retained_chords_extrude_with_exact_holes_components_and_reversed_affine_images() {
    for reverse in [false, true] {
        for components in [false, true] {
            for affine in [false, true] {
                let region = profile(reverse, components, affine);
                // Integral of 4-x² on [-2,2] is 32/3. The hole has area 1;
                // the second cap adds 32/3. Affine determinant has magnitude 6.
                let expected_area =
                    q(if components { 61 } else { 32 }, 3) * Real::from(if affine { 6 } else { 1 });
                equal(region.filled_area().unwrap().unwrap(), expected_area);
                let result = hypercurve::evaluate_under(PredicatePolicy::APPROXIMATE_512, || {
                    builder::extrude_curve_region(&region, 0.into(), 3.into())
                });
                assert_eq!(result.certainty(), CurveCertainty::Certified);
                let bodies = result.into_unverified().unwrap();
                verify(bodies.model(), components, affine);
                verify(&replay(bodies.model()), components, affine);
                let empty = BodySet::empty();
                verify(
                    boolean::union(&empty, &bodies).unwrap().model(),
                    components,
                    affine,
                );
                verify(
                    boolean::difference(&bodies, &empty).unwrap().model(),
                    components,
                    affine,
                );
                assert!(boolean::intersection(&bodies, &empty).unwrap().is_empty());
                assert!(boolean::difference(&empty, &bodies).unwrap().is_empty());
            }
        }
    }
}

#[test]
fn retained_profile_results_continue_through_slab_cuts_and_restoration() {
    let region = profile(false, false, false);
    let original = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
    let cold = replay(original.model());
    let body = BodySet::from_solid(cold.solids().next().unwrap());
    let cutter = builder::extrude_curve_region(&region, 1.into(), 2.into()).unwrap();
    let middle = boolean::intersection(&body, &cutter).unwrap();
    equal(middle.bodies().map(|s| s.volume().unwrap()).sum(), q(32, 3));
    let removed = boolean::difference(&body, &cutter).unwrap();
    assert_eq!(removed.len(), 2);
    equal(
        removed.bodies().map(|s| s.volume().unwrap()).sum(),
        q(64, 3),
    );
    let restored = boolean::union(&removed, &middle).unwrap();
    verify(restored.model(), false, false);
    verify(&replay(restored.model()), false, false);
    assert!(boolean::difference(&restored, &body).unwrap().is_empty());
    assert!(boolean::difference(&body, &restored).unwrap().is_empty());
}

#[test]
fn retained_chord_planar_face_uses_the_same_native_image_proof() {
    let region = profile(false, false, false);
    let paths = region.boundary_paths().unwrap();
    assert!(
        paths[0]
            .curves()
            .iter()
            .any(|curve| curve.geometry().is_none())
    );
    let sheet =
        builder::planar_face(&paths[0], &[], Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    equal(sheet.faces().next().unwrap().area().unwrap(), q(32, 3));
    let cold = replay(sheet.model());
    equal(cold.faces().next().unwrap().area().unwrap(), q(32, 3));
    assert_eq!(cold.solids().count(), 0);
}

#[test]
fn equal_profile_intervals_preserve_exact_thin_components_and_touching_union() {
    let region = profile(true, false, false);
    let body = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
    let gap: Real = "1e-12".parse().unwrap();
    let cutter = builder::extrude_curve_region(&region, gap.clone(), Real::from(3) - &gap).unwrap();
    let thin = boolean::difference(&body, &cutter).unwrap();
    assert_eq!(thin.len(), 2);
    equal(
        thin.bodies().map(|s| s.volume().unwrap()).sum(),
        q(64, 3) * &gap,
    );
    let cold = replay(thin.model());
    equal(
        cold.solids().map(|s| s.volume().unwrap()).sum(),
        q(64, 3) * &gap,
    );
    for (z, expected) in [
        (gap.clone() * q(1, 2), SolidPointLocation::Inside),
        (gap.clone(), SolidPointLocation::Boundary),
        (gap.clone() * Real::from(2), SolidPointLocation::Outside),
        (Real::from(3) - &gap * q(1, 2), SolidPointLocation::Inside),
    ] {
        let point = Point3::new(0.into(), 2.into(), z);
        let locations = cold
            .solids()
            .map(|s| s.classify_point(&point).unwrap())
            .collect::<Vec<_>>();
        assert!(if expected == SolidPointLocation::Outside {
            locations.iter().all(|location| *location == expected)
        } else {
            locations.contains(&expected)
        });
    }
    verify(
        boolean::union(&thin, &cutter).unwrap().model(),
        false,
        false,
    );
    // A single shared cap is a zero-volume intersection and a connected union.
    let first = builder::extrude_curve_region(&region, 0.into(), 1.into()).unwrap();
    let second = builder::extrude_curve_region(&region, 1.into(), 3.into()).unwrap();
    assert!(boolean::intersection(&first, &second).unwrap().is_empty());
    verify(
        boolean::union(&first, &second).unwrap().model(),
        false,
        false,
    );
    verify(
        boolean::union(&second, &first).unwrap().model(),
        false,
        false,
    );
}

#[test]
fn interval_continuation_requires_equal_material_including_holes() {
    let region = profile(false, false, false);
    let body = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
    let holed = CurveRegion2::try_from_boundary_paths(
        &[parabola(0, false), hole(false)],
        FillRule::EvenOdd,
    )
    .unwrap();
    equal(holed.filled_area().unwrap().unwrap(), q(29, 3));
    let cutter = builder::extrude_curve_region(&holed, 1.into(), 2.into()).unwrap();
    let before = body.model().to_json().unwrap();
    // Identical outer bounds and caps do not prove equal filled profiles.
    let error = boolean::difference(&body, &cutter).unwrap_err();
    assert!(
        matches!(error, BooleanError::FallbackFailed { optimized, .. }
        if matches!(*optimized, BooleanError::IncompatibleExtrusionSlabs))
    );
    assert_eq!(body.model().to_json().unwrap(), before);
}

#[test]
fn native_images_do_not_bypass_planar_path_simplicity() {
    let points = [p(0, 0), p(2, 2), p(0, 2), p(2, 0)];
    let path = CurvePath2::try_new(
        (0..4)
            .map(|i| {
                Curve2::try_line(points[i].clone().into(), points[(i + 1) % 4].clone().into())
                    .unwrap()
            })
            .collect(),
    )
    .unwrap();
    assert!(path.curves().iter().all(|curve| curve.geometry().is_none()));
    assert_eq!(
        builder::planar_face(&path, &[], Point3::origin(), Vector3::x(), Vector3::y()).unwrap_err(),
        ConstructionError::SelfIntersectingProfile
    );
}
