use hyperbrep::*;
use hypercurve::{BooleanOp, CurvePath2, CurveRegion2, FillRule};

#[path = "selected_profile_source.rs"]
mod source;
use source::{line, q};
pub use source::{profile, replay};

fn volume(model: &Model, expected: &Real) {
    let actual: Real = model.solids().map(|s| s.volume().unwrap()).sum();
    assert_eq!(
        hyperlimit::compare_reals(&actual, expected, STRICT_PREDICATES).value(),
        Some(std::cmp::Ordering::Equal),
    );
}

fn location(model: &Model, point: &Point3) -> SolidPointLocation {
    let mut result = SolidPointLocation::Outside;
    for solid in model.solids() {
        match solid.classify_point(point).unwrap() {
            SolidPointLocation::Inside => return SolidPointLocation::Inside,
            SolidPointLocation::Boundary => result = SolidPointLocation::Boundary,
            SolidPointLocation::Outside => {}
        }
    }
    result
}

fn verify(model: &Model, area: &Real, xor: bool, affine: bool) {
    assert_eq!(model.solids().count(), if xor { 2 } else { 1 });
    volume(model, &(area * Real::from(3)));
    let gap: Real = "1e-12".parse().unwrap();
    for (x, y, expected) in [
        (0.into(), 1.into(), SolidPointLocation::Inside),
        (0.into(), 3.into(), SolidPointLocation::Outside),
        (0.into(), 0.into(), SolidPointLocation::Boundary),
        (1.into(), 1.into(), SolidPointLocation::Boundary),
        (1.into(), Real::one() + &gap, SolidPointLocation::Inside),
        (1.into(), Real::one() - &gap, SolidPointLocation::Outside),
        (0.into(), 2.into(), SolidPointLocation::Boundary),
        (0.into(), Real::from(2) - &gap, SolidPointLocation::Inside),
        (0.into(), Real::from(2) + &gap, SolidPointLocation::Outside),
        (
            0.into(),
            q(9, 2),
            if xor {
                SolidPointLocation::Inside
            } else {
                SolidPointLocation::Outside
            },
        ),
        (
            q(5, 2),
            3.into(),
            if xor {
                SolidPointLocation::Inside
            } else {
                SolidPointLocation::Outside
            },
        ),
    ] {
        let (x, y) = if affine {
            (
                Real::from(-2) * x + Real::from(7),
                Real::from(3) * y - Real::one(),
            )
        } else {
            (x, y)
        };
        let point = Point3::new(x, y, q(3, 2));
        assert_eq!(location(model, &point), expected, "{point:?}");
    }
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

pub fn selected_boolean_profiles_extrude_and_replay() {
    for xor in [false, true] {
        for affine in [false, true] {
            for reverse in [false, true] {
                let (region, area) = profile(xor, affine, reverse);
                let measured_area = region.signed_area().unwrap().unwrap();
                assert_eq!(
                    hyperlimit::compare_reals(&measured_area, &area, STRICT_PREDICATES).value(),
                    Some(std::cmp::Ordering::Equal),
                );
                let result =
                    hypercurve::evaluate_under(hypercurve::PredicatePolicy::STRICT, || {
                        builder::extrude_curve_region(&region, 0.into(), 3.into())
                    });
                assert_eq!(result.certainty(), hypercurve::CurveCertainty::Certified);
                let bodies = result.into_unverified().unwrap();
                for model in [bodies.model().clone(), replay(bodies.model())] {
                    verify(&model, &area, xor, affine);
                }
            }
        }
    }
}

pub fn selected_profile_paths_share_native_image_admission() {
    for xor in [false, true] {
        for reverse in [false, true] {
            let (region, area) = profile(xor, false, reverse);
            let regions = region
                .material_components()
                .unwrap()
                .iter()
                .map(|component| {
                    let mut paths = component
                        .boundary_loops()
                        .iter()
                        .map(|boundary| {
                            let path = CurvePath2::try_new(boundary.curves().to_vec()).unwrap();
                            if reverse {
                                path.reversed().unwrap()
                            } else {
                                path
                            }
                        })
                        .collect::<Vec<_>>();
                    (paths.remove(0), paths)
                })
                .collect::<Vec<_>>();
            let bodies = builder::extrude_path_regions(&regions, 0.into(), 3.into()).unwrap();
            verify(&replay(bodies.model()), &area, xor, false);
            if !xor {
                let path = &regions[0].0;
                for bodies in [
                    builder::extrude_path(path, 0.into(), 3.into()).unwrap(),
                    builder::extrude_path_region(path, &[], 0.into(), 3.into()).unwrap(),
                ] {
                    verify(&replay(bodies.model()), &area, false, false);
                }
                let sheet =
                    builder::planar_face(path, &[], Point3::origin(), Vector3::x(), Vector3::y())
                        .unwrap();
                let cold = replay(sheet.model());
                let actual = cold.faces().next().unwrap().area().unwrap();
                assert_eq!(
                    hyperlimit::compare_reals(&actual, &area, STRICT_PREDICATES).value(),
                    Some(std::cmp::Ordering::Equal)
                );
            }
        }
    }
}

pub fn selected_profile_edits_and_slab_booleans_preserve_exact_material() {
    for xor in [false, true] {
        for affine in [false, true] {
            let (region, area) = profile(xor, affine, false);
            let original = builder::extrude_curve_region(&region, 0.into(), 3.into()).unwrap();
            let edge = original
                .model()
                .edges()
                .find(|edge| edge.curve().kind() == Curve3Kind::RationalBezier)
                .unwrap();
            let split = edge.split(q(1, 2).into()).unwrap();
            let cold = replay(split.model());
            verify(&cold, &area, xor, affine);
            let mut current = BodySet::empty();
            for solid in cold.solids() {
                current = boolean::union(&current, &BodySet::from_solid(solid)).unwrap();
            }
            for _ in 0..2 {
                let cutter = builder::extrude_curve_region(&region, 1.into(), 2.into()).unwrap();
                let middle = boolean::intersection(&current, &cutter).unwrap();
                let ends = boolean::difference(&current, &cutter).unwrap_or_else(|error| {
                    panic!("slab difference xor={xor} affine={affine}: {error:?}")
                });
                volume(&replay(middle.model()), &area);
                volume(&replay(ends.model()), &(&area * Real::from(2)));
                assert_eq!(ends.len(), if xor { 4 } else { 2 });
                current = boolean::union(&ends, &middle).unwrap();
                verify(&replay(current.model()), &area, xor, affine);
                assert!(boolean::difference(&current, &original).unwrap().is_empty());
                assert!(boolean::difference(&original, &current).unwrap().is_empty());
                let empty = boolean::difference(&current, &current).unwrap();
                assert!(empty.is_empty());
                assert_eq!(replay(empty.model()).counts().solids, 0);
                current = boolean::union(&empty, &current).unwrap();
            }
            for identity in [
                boolean::union(&current, &BodySet::empty()).unwrap(),
                boolean::difference(&current, &BodySet::empty()).unwrap(),
            ] {
                verify(&replay(identity.model()), &area, xor, affine);
            }
            assert!(
                boolean::intersection(&current, &BodySet::empty())
                    .unwrap()
                    .is_empty()
            );
            assert!(
                boolean::difference(&BodySet::empty(), &current)
                    .unwrap()
                    .is_empty()
            );
        }
    }
}

pub fn selected_profiles_keep_holes_and_disconnected_components() {
    let (region, area) = profile(false, false, false);
    let points = [
        hypercurve::Point2::new(q(-1, 2), 1.into()),
        hypercurve::Point2::new(q(1, 2), 1.into()),
        hypercurve::Point2::new(q(1, 2), q(3, 2)),
        hypercurve::Point2::new(q(-1, 2), q(3, 2)),
    ];
    let hole = CurvePath2::try_new(
        (0..4)
            .map(|i| line(points[i].clone(), points[(i + 1) % 4].clone()))
            .collect(),
    )
    .unwrap();
    let hole = CurveRegion2::try_from_boundary_paths(&[hole], FillRule::EvenOdd).unwrap();
    let shifted = region
        .transform_affine(
            &1.into(),
            &0.into(),
            &0.into(),
            &1.into(),
            &8.into(),
            &0.into(),
        )
        .unwrap();
    let combined = region
        .boolean_region(&hole, BooleanOp::Difference)
        .unwrap()
        .boolean_region(&shifted, BooleanOp::Union)
        .unwrap();
    let bodies = builder::extrude_curve_region(&combined, 0.into(), 3.into()).unwrap();
    let expected = (area * Real::from(2) - q(1, 2)) * Real::from(3);
    for model in [bodies.model().clone(), replay(bodies.model())] {
        assert_eq!(model.solids().count(), 2);
        volume(&model, &expected);
        for (x, y, expected) in [
            (0.into(), q(5, 4), SolidPointLocation::Outside),
            (q(1, 2), q(5, 4), SolidPointLocation::Boundary),
            (8.into(), 1.into(), SolidPointLocation::Inside),
        ] {
            assert_eq!(location(&model, &Point3::new(x, y, 1.into())), expected);
        }
    }
}

pub fn selected_profiles_transport_exactly_and_reject_damaged_archives() {
    let transform = Matrix4::from_row_major([
        (-2).into(),
        1.into(),
        0.into(),
        7.into(),
        0.into(),
        3.into(),
        1.into(),
        (-1).into(),
        0.into(),
        0.into(),
        2.into(),
        5.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let gap: Real = "1e-12".parse().unwrap();
    for xor in [false, true] {
        let (region, area) = profile(xor, false, false);
        let small = OperationContext::strict(WorkBudget::ZERO)
            .evaluate(|| builder::extrude_curve_region(&region, 0.into(), 3.into()));
        if let Ok(result) = small.result {
            verify(result.unwrap().model(), &area, xor, false);
        }
        // Either a sufficient zero-cost proof or typed budget exhaustion is
        // admissible above. Full-budget reconstruction must still succeed.
        let outcome = OperationContext::strict(WorkBudget {
            refinements: 100_000,
            promotions: 10_000,
        })
        .evaluate(|| builder::extrude_curve_region(&region, 0.into(), 3.into()));
        let original = outcome.result.unwrap().unwrap();
        verify(original.model(), &area, xor, false);
        let transformed = original.model().transformed(&transform).unwrap();
        for model in [transformed.clone(), replay(&transformed)] {
            volume(&model, &(&area * Real::from(36)));
            for (y, expected) in [
                (Real::one(), SolidPointLocation::Boundary),
                (Real::one() - &gap, SolidPointLocation::Outside),
                (Real::one() + &gap, SolidPointLocation::Inside),
            ] {
                let point = transform
                    .transform_point3(&Point3::new(1.into(), y, q(3, 2)))
                    .unwrap();
                assert_eq!(location(&model, &point), expected);
            }
        }
        let mut damaged: serde_json::Value =
            serde_json::from_str(&original.model().to_json().unwrap()).unwrap();
        let curve = damaged["curves"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|curve| curve.get("RationalBezier").is_some())
            .unwrap();
        curve["RationalBezier"]["control_points"][1][2] =
            serde_json::to_value(Real::from(10)).unwrap();
        assert!(
            RawModel::from_json(&damaged.to_string(), &ArchiveLimits::default())
                .unwrap()
                .validate()
                .is_err()
        );
        verify(&replay(original.model()), &area, xor, false);
    }
}

pub fn partial_height_profile_contacts_keep_boundary_and_overlap_distinct() {
    let rectangle = |left: Real, right: Real| {
        let points = [
            hypercurve::Point2::new(left.clone(), 0.into()),
            hypercurve::Point2::new(right.clone(), 0.into()),
            hypercurve::Point2::new(right, 2.into()),
            hypercurve::Point2::new(left, 2.into()),
        ];
        let path = CurvePath2::try_new(
            (0..4)
                .map(|i| line(points[i].clone(), points[(i + 1) % 4].clone()))
                .collect(),
        )
        .unwrap();
        CurveRegion2::try_from_boundary_paths(&[path], FillRule::EvenOdd).unwrap()
    };
    let first =
        builder::extrude_curve_region(&rectangle((-2).into(), 0.into()), 0.into(), 3.into())
            .unwrap();
    let gap: Real = "1e-12".parse().unwrap();
    for left in [gap.clone(), Real::zero(), -&gap] {
        let second =
            builder::extrude_curve_region(&rectangle(left.clone(), 2.into()), 1.into(), 2.into())
                .unwrap();
        let difference = boolean::difference(&first, &second).unwrap();
        let expected = if left < Real::zero() {
            Real::from(12) - Real::from(2) * &gap
        } else {
            Real::from(12)
        };
        volume(&replay(difference.model()), &expected);
        let point = Point3::new(-(&gap * q(1, 2)), 1.into(), q(3, 2));
        assert_eq!(
            location(difference.model(), &point),
            if left < Real::zero() {
                SolidPointLocation::Outside
            } else {
                SolidPointLocation::Inside
            }
        );
        let union = boolean::union(&first, &second).unwrap();
        let expected = Real::from(12) + (Real::from(2) - &left) * Real::from(2)
            - if left < Real::zero() {
                Real::from(2) * &gap
            } else {
                Real::zero()
            };
        volume(&replay(union.model()), &expected);
        if left == Real::zero() {
            // A shared face must be stitched; keeping two separate bodies
            // would incorrectly classify this material-interior point as boundary.
            assert_eq!(union.len(), 1);
            assert_eq!(
                location(union.model(), &Point3::new(0.into(), 1.into(), q(3, 2))),
                SolidPointLocation::Inside
            );
        }
    }
}
