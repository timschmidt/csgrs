use hyperbrep::*;
use hypercurve::{CurveRegion2, Similarity2};

#[allow(dead_code)]
#[path = "selected_procedural_profiles.rs"]
mod selected;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn cap(clipped: bool) -> BodySet {
    let region = if clipped {
        selected::clipped_parallel_cap()
    } else {
        selected::original::original_parallel_cap()
    };
    let shift = Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        3.into(),
        0.into(),
    )
    .unwrap();
    let shifted: CurveRegion2 = region.transform_similarity(&shift).unwrap();
    builder::revolve_curve_region(&shifted).unwrap()
}

fn radial_band(min: Real, max: Real) -> BodySet {
    builder::revolve(&[
        Point2::new(min.clone(), (-3).into()),
        Point2::new(max.clone(), (-3).into()),
        Point2::new(max, 3.into()),
        Point2::new(min, 3.into()),
    ])
    .unwrap()
}

fn location(model: &Model, radius: Real, height: Real) -> SolidPointLocation {
    let point = Point3::new(radius, Real::zero(), height);
    let mut boundary = false;
    for solid in model.solids() {
        match solid.classify_point(&point).unwrap() {
            SolidPointLocation::Inside => return SolidPointLocation::Inside,
            SolidPointLocation::Boundary => boundary = true,
            SolidPointLocation::Outside => {}
        }
    }
    if boundary {
        SolidPointLocation::Boundary
    } else {
        SolidPointLocation::Outside
    }
}

fn replay(bodies: &BodySet) -> Model {
    let text = bodies.model().to_json().unwrap();
    let cold = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), text);
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(folder) = std::env::var_os("HYPERBREP_COAXIAL_PROCEDURAL_ARTIFACT_DIR") {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let folder = std::path::PathBuf::from(folder);
        std::fs::create_dir_all(&folder).unwrap();
        std::fs::write(
            folder.join(format!(
                "boolean-{:03}.json",
                NEXT.fetch_add(1, Ordering::Relaxed)
            )),
            &text,
        )
        .unwrap();
    }
    cold
}

fn verify_cut(clipped: bool) {
    let original = cap(clipped);
    let (min, max, middle, left, right) = if clipped {
        (q(13, 4), q(15, 4), q(7, 2), q(25, 8), q(31, 8))
    } else {
        (4.into(), 6.into(), 5.into(), 3.into(), 7.into())
    };
    let tool = radial_band(min.clone(), max.clone());
    let common = boolean::intersection(&original, &tool)
        .expect("a co-axial radial cut must retain the exact procedural profile");
    assert_eq!(common.len(), 1);
    assert!(common.model().curves().any(|curve| {
        curve
            .geometry()
            .planar_geometry()
            .is_some_and(|image| image.source.analytic_parallel().is_some())
    }));
    for model in [common.model(), &replay(&common)] {
        for (radius, expected) in [
            (middle.clone(), SolidPointLocation::Inside),
            (min.clone(), SolidPointLocation::Boundary),
            (max.clone(), SolidPointLocation::Boundary),
            (left.clone(), SolidPointLocation::Outside),
            (right.clone(), SolidPointLocation::Outside),
        ] {
            assert_eq!(location(model, radius, 0.into()), expected);
        }
        assert_eq!(
            location(model, middle.clone(), (-2).into()),
            SolidPointLocation::Boundary
        );
        assert_eq!(
            location(model, middle.clone(), q(5, 2)),
            SolidPointLocation::Outside
        );
    }
    let cut = boolean::difference(&original, &tool).unwrap();
    assert_eq!(cut.len(), 2);
    for model in [cut.model(), &replay(&cut)] {
        assert_eq!(
            location(model, middle.clone(), 0.into()),
            SolidPointLocation::Outside
        );
        assert_eq!(
            location(model, left.clone(), 0.into()),
            SolidPointLocation::Inside
        );
        assert_eq!(
            location(model, right.clone(), 0.into()),
            SolidPointLocation::Inside
        );
    }
    let filled = boolean::union(&cut, &common).unwrap();
    assert_eq!(filled.len(), 1);
    assert_eq!(
        location(&replay(&filled), middle, 0.into()),
        SolidPointLocation::Inside
    );
    assert!(boolean::difference(&filled, &original).unwrap().is_empty());
    assert!(boolean::difference(&original, &filled).unwrap().is_empty());
}

pub fn full_parallel_revolution_boolean_continues() {
    verify_cut(false);
}

pub fn selected_parallel_revolution_boolean_continues() {
    verify_cut(true);
}

pub fn coaxial_parallel_union_and_xor_overlap() {
    let original = cap(false);
    let tool = radial_band(4.into(), 6.into());
    let left = boolean::union(&original, &tool).unwrap();
    let right = boolean::union(&tool, &original).unwrap();
    for joined in [&left, &right] {
        assert_eq!(joined.len(), 1);
        let cold = replay(joined);
        for (r, z, expected) in [
            (3.into(), 0.into(), SolidPointLocation::Inside),
            (7.into(), 0.into(), SolidPointLocation::Inside),
            (5.into(), 0.into(), SolidPointLocation::Inside),
            (5.into(), q(5, 2), SolidPointLocation::Inside),
            (5.into(), q(-5, 2), SolidPointLocation::Inside),
            (5.into(), 3.into(), SolidPointLocation::Boundary),
            (3.into(), q(5, 2), SolidPointLocation::Outside),
            (0.into(), 0.into(), SolidPointLocation::Outside),
        ] {
            assert_eq!(location(&cold, r, z), expected);
        }
    }
    assert!(boolean::difference(&left, &right).unwrap().is_empty());
    assert!(boolean::difference(&right, &left).unwrap().is_empty());
    let xor = boolean::symmetric_difference(&original, &tool).unwrap();
    // The two side lobes and the band's top/bottom each have a connected
    // volume interior; closures meet along boundary circles only.
    assert_eq!(xor.len(), 4);
    let cold = replay(&xor);
    for (r, z, expected) in [
        (3.into(), 0.into(), SolidPointLocation::Inside),
        (7.into(), 0.into(), SolidPointLocation::Inside),
        (5.into(), q(5, 2), SolidPointLocation::Inside),
        (5.into(), q(-5, 2), SolidPointLocation::Inside),
        (5.into(), 0.into(), SolidPointLocation::Outside),
        (0.into(), 0.into(), SolidPointLocation::Outside),
    ] {
        assert_eq!(location(&cold, r, z), expected);
    }
    let common = boolean::intersection(&original, &tool).unwrap();
    assert!(boolean::intersection(&xor, &common).unwrap().is_empty());
    let filled = boolean::union(&xor, &common).unwrap();
    assert!(boolean::difference(&filled, &left).unwrap().is_empty());
    assert!(boolean::difference(&left, &filled).unwrap().is_empty());
}

fn location_at(model: &Model, point: &Point3) -> SolidPointLocation {
    let mut boundary = false;
    for solid in model.solids() {
        match solid.classify_point(point).unwrap() {
            SolidPointLocation::Inside => return SolidPointLocation::Inside,
            SolidPointLocation::Boundary => boundary = true,
            SolidPointLocation::Outside => {}
        }
    }
    if boundary {
        SolidPointLocation::Boundary
    } else {
        SolidPointLocation::Outside
    }
}

pub fn coaxial_native_arc_and_nurbs_profiles_continue() {
    use hypercurve::{CircularArc2, Contour2, Curve2, CurvePath2, LineSeg2, Segment2};
    let p = |x, y| hypercurve::Point2::from_values(x, y);
    let disk = Contour2::try_new(vec![
        Segment2::Arc(CircularArc2::try_from_center(p(4, 0), p(2, 0), p(3, 0), false).unwrap()),
        Segment2::Arc(CircularArc2::try_from_center(p(2, 0), p(4, 0), p(3, 0), false).unwrap()),
    ])
    .unwrap();
    let spline = CurvePath2::try_new(vec![
        Curve2::from(LineSeg2::try_new(p(2, 0), p(4, 0)).unwrap()),
        Curve2::try_nurbs(
            2,
            vec![p(4, 0), p(5, 1), p(4, 2)],
            vec![Real::one(); 3],
            vec![0.into(), 0.into(), 0.into(), 1.into(), 1.into(), 1.into()],
        )
        .unwrap(),
        Curve2::from(LineSeg2::try_new(p(4, 2), p(2, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(p(2, 2), p(2, 0)).unwrap()),
    ])
    .unwrap();
    for (original, min, max, middle, left, right, height, family) in [
        (
            builder::revolve_contour(&disk).unwrap(),
            Real::from(3),
            q(7, 2),
            q(13, 4),
            q(11, 4),
            q(15, 4),
            Real::zero(),
            Curve3Kind::CircleArc,
        ),
        (
            builder::revolve_path(&spline).unwrap(),
            Real::from(4),
            q(17, 4),
            q(33, 8),
            Real::from(3),
            q(35, 8),
            Real::one(),
            Curve3Kind::Nurbs,
        ),
    ] {
        assert!(
            original
                .model()
                .curves()
                .any(|c| c.geometry().kind() == family)
        );
        let tool = radial_band(min.clone(), max.clone());
        let common = boolean::intersection(&original, &tool).unwrap();
        assert_eq!(common.len(), 1);
        let cold = replay(&common);
        for model in [common.model(), &cold] {
            for (r, expected) in [
                (middle.clone(), SolidPointLocation::Inside),
                (min.clone(), SolidPointLocation::Boundary),
                (max.clone(), SolidPointLocation::Boundary),
                (left.clone(), SolidPointLocation::Outside),
                (right.clone(), SolidPointLocation::Outside),
            ] {
                assert_eq!(location(model, r, height.clone()), expected);
            }
        }
        let cut = boolean::difference(&original, &tool).unwrap();
        assert_eq!(cut.len(), 2);
        let cold_cut = replay(&cut);
        assert_eq!(
            location(&cold_cut, left, height.clone()),
            SolidPointLocation::Inside
        );
        assert_eq!(
            location(&cold_cut, right, height.clone()),
            SolidPointLocation::Inside
        );
        assert_eq!(
            location(&cold_cut, middle, height),
            SolidPointLocation::Outside
        );
        let filled =
            boolean::union(&cut, &BodySet::from_solid(cold.solids().next().unwrap())).unwrap();
        assert!(boolean::difference(&filled, &original).unwrap().is_empty());
        assert!(boolean::difference(&original, &filled).unwrap().is_empty());
    }
}

pub fn coaxial_selected_axes_similarities_and_exact_gaps() {
    let original = cap(true);
    let tool = radial_band(q(13, 4), q(15, 4));
    // Opposite unit direction and a different origin on the same axis line.
    // The tool's resulting height interval [-2,4] covers the cap completely.
    let reverse = Matrix4::from_row_major([
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        (-1).into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let tool = tool.transformed(&reverse).unwrap();
    let gap: Real = "1e-12".parse().unwrap();
    for sign in [-2, 2] {
        let transform = Matrix4::from_row_major([
            Real::from(sign) * q(4, 5),
            0.into(),
            q(6, 5),
            7.into(),
            0.into(),
            2.into(),
            0.into(),
            (-3).into(),
            -Real::from(sign) * q(3, 5),
            0.into(),
            q(8, 5),
            5.into(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
        ]);
        let first = original.transformed(&transform).unwrap();
        let second = tool.transformed(&transform).unwrap();
        for result in [
            boolean::intersection(&first, &second).unwrap(),
            boolean::intersection(&second, &first).unwrap(),
        ] {
            let cold = replay(&result);
            for model in [result.model(), &cold] {
                for (r, expected) in [
                    (q(13, 4) - &gap, SolidPointLocation::Outside),
                    (q(13, 4), SolidPointLocation::Boundary),
                    (q(13, 4) + &gap, SolidPointLocation::Inside),
                    (q(7, 2), SolidPointLocation::Inside),
                    (q(15, 4) - &gap, SolidPointLocation::Inside),
                    (q(15, 4), SolidPointLocation::Boundary),
                    (q(15, 4) + &gap, SolidPointLocation::Outside),
                ] {
                    let point = transform
                        .transform_point3(&Point3::new(r, 0.into(), 0.into()))
                        .unwrap();
                    assert_eq!(location_at(model, &point), expected);
                }
            }
        }
    }
}

pub fn coaxial_procedural_cavities_xor_and_empty_recover() {
    let original = cap(false);
    let cavity = builder::revolve(&[
        Point2::new(q(9, 2), (-1).into()),
        Point2::new(q(11, 2), (-1).into()),
        Point2::new(q(11, 2), 0.into()),
        Point2::new(q(9, 2), 0.into()),
    ])
    .unwrap();
    for hollow in [
        boolean::difference(&original, &cavity).unwrap(),
        boolean::symmetric_difference(&original, &cavity).unwrap(),
    ] {
        assert_eq!(hollow.len(), 1);
        let cold = replay(&hollow);
        assert_eq!(cold.solids().next().unwrap().voids().len(), 1);
        let tiny: Real = "1e-12".parse().unwrap();
        for (r, expected) in [
            (q(9, 2) - &tiny, SolidPointLocation::Inside),
            (q(9, 2), SolidPointLocation::Boundary),
            (q(9, 2) + &tiny, SolidPointLocation::Outside),
            (Real::from(5), SolidPointLocation::Outside),
        ] {
            assert_eq!(location(&cold, r, q(-1, 2)), expected);
        }
        let restored =
            boolean::union(&BodySet::from_solid(cold.solids().next().unwrap()), &cavity).unwrap();
        assert!(
            boolean::difference(&restored, &original)
                .unwrap()
                .is_empty()
        );
        assert!(
            boolean::difference(&original, &restored)
                .unwrap()
                .is_empty()
        );
        let empty = boolean::difference(&hollow, &original).unwrap();
        assert!(empty.is_empty());
        assert!(boolean::intersection(&hollow, &empty).unwrap().is_empty());
        assert!(boolean::difference(&empty, &hollow).unwrap().is_empty());
        for retained in [
            boolean::union(&empty, &hollow).unwrap(),
            boolean::difference(&hollow, &empty).unwrap(),
        ] {
            assert_eq!(
                location(&replay(&retained), 5.into(), q(-1, 2)),
                SolidPointLocation::Outside
            );
            assert_eq!(
                location(retained.model(), 3.into(), 0.into()),
                SolidPointLocation::Inside
            );
        }
    }
}

pub fn coaxial_selected_split_output_continues() {
    let original = cap(true);
    let tool = radial_band(q(13, 4), q(15, 4));
    let common = boolean::intersection(&original, &tool).unwrap();
    let cold = replay(&common);
    let meridian = cold
        .edges()
        .find(|e| {
            e.curve()
                .geometry()
                .planar_geometry()
                .is_some_and(|p| p.source.analytic_parallel().is_some() && p.u == &Vector3::x())
        })
        .unwrap();
    let plane = Surface::plane(
        Point3::new(q(7, 2), 0.into(), 0.into()),
        Vector3::y(),
        Vector3::z(),
    )
    .unwrap();
    let CurveSurfaceIntersection::Points(contacts) =
        plane.intersect_curve(meridian.curve().geometry()).unwrap()
    else {
        panic!("one exact selected radial intersection")
    };
    assert_eq!(contacts.len(), 1);
    assert!(contacts[0].parameter.try_real().is_none());
    let split = meridian.split(contacts[0].parameter.clone()).unwrap();
    let text = split.model().to_json().unwrap();
    let replayed = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(replayed.to_json().unwrap(), text);
    let operand = BodySet::from_solid(replayed.solids().next().unwrap());
    let result = boolean::intersection(&operand, &tool).unwrap();
    assert_eq!(
        location(&replay(&result), q(7, 2), 0.into()),
        SolidPointLocation::Inside
    );
    assert!(boolean::difference(&result, &common).unwrap().is_empty());
    assert!(boolean::difference(&common, &result).unwrap().is_empty());
}

pub fn coaxial_procedural_certainty_budget_and_axis_rejection() {
    let original = cap(false);
    let tool = radial_band(4.into(), 6.into());
    let denied = OperationContext::strict(WorkBudget::ZERO)
        .evaluate(|| boolean::intersection(&original, &tool));
    assert_eq!(denied.result.unwrap_err().limit, 0);
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let receipt =
            hypercurve::evaluate_under(policy, || boolean::intersection(&original, &tool));
        assert_eq!(receipt.certainty(), hypercurve::CurveCertainty::Certified);
        let common = receipt.into_unverified().unwrap();
        assert_eq!(
            location(&replay(&common), 5.into(), 0.into()),
            SolidPointLocation::Inside
        );
    }
    let offset = Matrix4::from_row_major([
        1.into(),
        0.into(),
        0.into(),
        "1e-12".parse().unwrap(),
        0.into(),
        1.into(),
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
    let shifted = tool.transformed(&offset).unwrap();
    assert!(matches!(
        boolean::intersection(&original, &shifted),
        Err(boolean::BooleanError::FallbackFailed { .. })
    ));
    let recovered = boolean::intersection(&original, &tool).unwrap();
    assert_eq!(
        location(&replay(&recovered), 5.into(), 0.into()),
        SolidPointLocation::Inside
    );
}
