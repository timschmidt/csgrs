use hyperbrep::*;
use hypercurve::{
    Contour2, Curve2, CurveCertainty, CurveFamily2, CurvePath2, CurveRegion2, FillRule, LineSeg2,
    OffsetCap, OffsetCornerStyle2, Point2 as CurvePoint2, Segment2,
};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn equal(actual: Real, expected: Real) {
    assert!(
        (actual - expected)
            .exact_rational_normal_form()
            .unwrap()
            .is_zero()
    );
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

fn measures(model: &Model, volume: Real, area: Real) {
    equal(
        model.solids().map(|solid| solid.volume().unwrap()).sum(),
        volume,
    );
    equal(model.faces().map(|face| face.area().unwrap()).sum(), area);
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

fn stroke(cap: OffsetCap, reverse: bool) -> CurveRegion2 {
    let mut points = [
        CurvePoint2::from_values(0, 0),
        CurvePoint2::from_values(3, 4),
    ];
    if reverse {
        points.reverse();
    }
    let path = CurvePath2::try_new(vec![
        LineSeg2::try_new(points[0].clone(), points[1].clone())
            .unwrap()
            .into(),
    ])
    .unwrap();
    CurveRegion2::stroke_path(&path, Real::one(), &OffsetCornerStyle2::Round, cap).unwrap()
}

#[test]
fn normalized_strokes_keep_native_circle_surfaces_and_exact_measures() {
    for cap in [OffsetCap::Butt, OffsetCap::Round] {
        for reverse in [false, true] {
            let region = stroke(cap, reverse);
            if cap == OffsetCap::Round {
                assert!(
                    region.boundary_paths().unwrap()[0]
                        .curves()
                        .iter()
                        .any(|curve| curve.family() == CurveFamily2::RationalQuadraticBezier)
                );
            }
            let bodies = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
            assert_eq!(bodies.len(), 1);
            let circle_sides = bodies
                .model()
                .faces()
                .filter(|face| face.surface().kind() == SurfaceKind::Cylinder)
                .count();
            if cap == OffsetCap::Round {
                assert!(circle_sides >= 2);
            } else {
                assert_eq!(circle_sides, 0);
            }
            for face in bodies.model().faces() {
                assert!(matches!(
                    face.surface().kind(),
                    SurfaceKind::Plane | SurfaceKind::Cylinder
                ));
            }
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
            measures(bodies.model(), volume.clone(), area.clone());
            measures(&replay(bodies.model()), volume, area);
        }
    }
}

fn rectangle(x0: i32, y0: i32, x1: i32, y1: i32, reverse: bool) -> Contour2 {
    let mut points = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
    if reverse {
        points.reverse();
    }
    Contour2::try_new(
        (0..4)
            .map(|index| {
                let a = points[index];
                let b = points[(index + 1) % 4];
                Segment2::Line(
                    LineSeg2::try_new(
                        CurvePoint2::from_values(a.0, a.1),
                        CurvePoint2::from_values(b.0, b.1),
                    )
                    .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap()
}

fn classify(model: &Model, x: Real, y: Real) -> SolidPointLocation {
    let point = Point3::new(x, y, q(3, 2));
    let mut result = SolidPointLocation::Outside;
    for solid in model.solids() {
        match solid.classify_point(&point).unwrap() {
            SolidPointLocation::Inside => return SolidPointLocation::Inside,
            SolidPointLocation::Boundary => result = SolidPointLocation::Boundary,
            SolidPointLocation::Outside => {}
        }
    }
    result
}

#[test]
fn winding_owned_holes_islands_and_components_survive_replay_and_composition() {
    for reverse in [false, true] {
        let region = CurveRegion2::try_from_native_boundary_contours(
            &[
                rectangle(0, 0, 6, 6, reverse),
                rectangle(1, 1, 5, 5, reverse),
                rectangle(2, 2, 4, 4, reverse),
                rectangle(8, 0, 9, 1, reverse),
            ],
            FillRule::EvenOdd,
        )
        .unwrap();
        // Area: 36 - 16 + 4 + 1 = 25. Perimeter: 24 + 16 + 8 + 4 = 52.
        equal(region.filled_area().unwrap().unwrap(), 25.into());
        let bodies = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
        assert_eq!(bodies.len(), 3);
        let cold = replay(bodies.model());
        for model in [bodies.model(), &cold] {
            measures(model, 75.into(), 206.into());
            let delta: Real = "1e-12".parse().unwrap();
            for (x, y, expected) in [
                (q(1, 2), q(1, 2), SolidPointLocation::Inside),
                (q(3, 2), q(3, 2), SolidPointLocation::Outside),
                (q(5, 2), q(5, 2), SolidPointLocation::Inside),
                (q(17, 2), q(1, 2), SolidPointLocation::Inside),
                (Real::one(), 3.into(), SolidPointLocation::Boundary),
                (Real::one() - &delta, 3.into(), SolidPointLocation::Inside),
                (Real::one() + &delta, 3.into(), SolidPointLocation::Outside),
                (Real::from(6) - &delta, 3.into(), SolidPointLocation::Inside),
                (
                    Real::from(6) + &delta,
                    3.into(),
                    SolidPointLocation::Outside,
                ),
            ] {
                assert_eq!(classify(model, x, y), expected);
            }
        }
        let island = BodySet::from_solid(
            bodies
                .bodies()
                .find(|solid| {
                    solid
                        .classify_point(&Point3::new(q(5, 2), q(5, 2), q(3, 2)))
                        .unwrap()
                        == SolidPointLocation::Inside
                })
                .unwrap(),
        );
        measures(island.model(), 75.into(), 206.into());
        equal(island.bodies().next().unwrap().volume().unwrap(), 12.into());
        let removed = boolean::difference(&bodies, &island).unwrap();
        assert_eq!(removed.len(), 2);
        measures(removed.model(), 63.into(), 174.into());
        let restored = boolean::union(&removed, &island).unwrap();
        measures(restored.model(), 75.into(), 206.into());
        let empty = boolean::difference(&restored, &bodies).unwrap();
        assert!(empty.is_empty());
        measures(
            boolean::union(&empty, &bodies).unwrap().model(),
            75.into(),
            206.into(),
        );
    }
}

#[test]
fn empty_material_is_composable_and_does_not_hide_invalid_heights() {
    let empty_region = CurveRegion2::empty();
    let empty = builder::extrude_curve_region(&empty_region, (-2).into(), 1.into()).unwrap();
    assert!(empty.is_empty());
    assert_eq!(
        empty.model().to_json().unwrap(),
        BodySet::empty().model().to_json().unwrap()
    );
    let region = stroke(OffsetCap::Butt, false);
    for source in [&empty_region, &region] {
        for (low, high) in [(0, 0), (3, 0)] {
            assert_eq!(
                builder::extrude_curve_region(source, low.into(), high.into()).unwrap_err(),
                ConstructionError::InvalidBounds(Axis::Z)
            );
        }
    }
    let body = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
    assert!(boolean::intersection(&empty, &body).unwrap().is_empty());
    assert!(boolean::difference(&empty, &body).unwrap().is_empty());
    measures(
        boolean::union(&empty, &body).unwrap().model(),
        30.into(),
        62.into(),
    );
    measures(
        boolean::difference(&body, &empty).unwrap().model(),
        30.into(),
        62.into(),
    );
}

#[test]
fn extrusion_restores_ambient_receipts_and_cannot_refund_exhausted_work() {
    let region = stroke(OffsetCap::Round, false);
    let line = Curve2::from(
        LineSeg2::try_new(
            CurvePoint2::from_values(0, 0),
            CurvePoint2::from_values(1, 0),
        )
        .unwrap(),
    );
    let tiny: Real = "1e-2000".parse().unwrap();
    let parameter = Real::one() + (Real::pi() + &tiny + &tiny).exp().unwrap()
        - (Real::pi() + tiny).exp().unwrap();
    let outer = hypercurve::provisional(|| {
        line.point_at(&parameter.clone().into()).unwrap();
        let bodies = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
        line.point_at(&parameter.clone().into()).unwrap();
        bodies
    });
    // The local strict construction cannot erase the caller's consumed terminal.
    assert_eq!(outer.certainty(), CurveCertainty::Approximate512Consumed);
    assert_eq!(outer.into_unverified().len(), 1);

    let zero = OperationContext::strict(WorkBudget::ZERO);
    let denied = zero.evaluate(|| {
        assert!(hypersolve::work_budget::consume(WorkKind::RootRefinement, 1).is_err());
        builder::extrude_curve_region(&region, 0.into(), 3.into())
    });
    assert_eq!(denied.result.unwrap_err().kind, WorkKind::RootRefinement);
    let retry = OperationContext::strict(WorkBudget {
        refinements: 512,
        promotions: 64,
    })
    .evaluate(|| builder::extrude_curve_region(&region, 0.into(), 3.into()));
    let body = retry.result.unwrap().unwrap();
    measures(
        body.model(),
        Real::from(30) + Real::from(3) * Real::pi(),
        Real::from(50) + Real::from(8) * Real::pi(),
    );
}
