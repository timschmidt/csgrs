//! Orientation-reversing transforms of models that share one pcurve between
//! surface charts whose first coordinates reflect about different pivots.
//!
//! A plane chart reflects `u` about 0 and a rational Bézier chart about 1.
//! The expected sets are those of the same models built without sharing,
//! and transforms must commute with Booleans: `T(A op B) = T(A) op T(B)`.

use hyperbrep::*;
use std::cmp::Ordering;

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn point(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn exact(left: &Real, right: &Real) -> bool {
    hyperlimit::compare_reals(left, right, hyperlimit::PredicatePolicy::STRICT).value()
        == Some(Ordering::Equal)
}

fn volume(model: &Model) -> Real {
    model
        .solids()
        .fold(Real::zero(), |sum, solid| sum + solid.volume().unwrap())
}

fn locate(model: &Model, sample: &Point3) -> SolidPointLocation {
    let mut location = SolidPointLocation::Outside;
    for solid in model.solids() {
        match solid.classify_point(sample).unwrap() {
            SolidPointLocation::Inside => return SolidPointLocation::Inside,
            SolidPointLocation::Boundary => location = SolidPointLocation::Boundary,
            SolidPointLocation::Outside => {}
        }
    }
    location
}

fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let replayed = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(replayed.to_json().unwrap(), json);
    replayed
}

fn is_unit_segment(pcurve: PcurveRef<'_>) -> bool {
    let curve = pcurve.geometry();
    let start = curve.point_at(&curve.domain_start().unwrap()).unwrap();
    let end = curve.point_at(&curve.domain_end().unwrap()).unwrap();
    curve.kind() == hypercurve::CurveFamily2::Line
        && start.coordinates() == Some(&hypercurve::Point2::new(0.into(), 0.into()))
        && end.coordinates() == Some(&hypercurve::Point2::new(1.into(), 0.into()))
}

/// A unit-square sweep along a curved path. Its top cap (a plane chart) and
/// one side (a rational Bézier chart) each own the segment (0,0)→(1,0).
fn unit_square_sweep() -> BodySet {
    let square = [(0, 0), (1, 0), (1, 1), (0, 1)].map(|(x, y)| Point2::new(x.into(), y.into()));
    let path = Curve3::rational_bezier(
        vec![
            point(Real::zero(), Real::zero(), Real::zero()),
            point(Real::one(), Real::zero(), Real::one()),
            point(Real::zero(), Real::zero(), Real::from(4)),
        ],
        vec![Real::one(), Real::from(2), Real::from(3)],
    )
    .unwrap();
    builder::sweep_curve(&square, Vector3::x(), Vector3::y(), path).unwrap()
}

/// The same model with the top cap's use of that segment pointing at the
/// side's record, through the public edit API.
fn shared_pcurve_sweep(sweep: &BodySet) -> Model {
    let model = sweep.model();
    let mut cap_use = None;
    let mut side_use = None;
    for edge_use in model.edge_uses() {
        if !is_unit_segment(edge_use.pcurve()) {
            continue;
        }
        let face = edge_use.wire().face();
        match face.surface().geometry().kind() {
            SurfaceKind::Plane if face.orientation() == Orientation::Forward => {
                cap_use = Some(edge_use);
            }
            SurfaceKind::RationalBezier if side_use.is_none() => side_use = Some(edge_use),
            _ => {}
        }
    }
    let (cap_use, side_use) = (cap_use.unwrap(), side_use.unwrap());
    let mut edit = model.edit();
    edit.replace_edge_use(
        cap_use,
        cap_use.edge(),
        cap_use.direction(),
        side_use.pcurve(),
        cap_use.parameter_correspondence().clone(),
    )
    .unwrap();
    let shared = edit.commit().unwrap();
    let shared_uses = shared
        .edge_uses()
        .filter(|edge_use| edge_use.pcurve().index() == side_use.pcurve().index())
        .count();
    assert_eq!(shared_uses, 2);
    shared
}

fn samples() -> Vec<Point3> {
    let mut samples = Vec::new();
    for z in [Real::zero(), q(1, 2), Real::from(2), Real::from(4), q(9, 2)] {
        for (x, y) in [
            (q(1, 2), q(1, 2)),
            (Real::from(1), q(1, 2)),
            (Real::from(2), q(1, 2)),
            (q(3, 2), q(1, 4)),
            (q(1, 4), q(1, 4)),
            (-q(1, 2), q(1, 2)),
        ] {
            samples.push(point(x, y, z.clone()));
        }
    }
    samples
}

fn reversing_transforms() -> Vec<Matrix4> {
    vec![
        Matrix4::affine_nonuniform_scale([-Real::one(), Real::one(), Real::one()]),
        Matrix4::affine_nonuniform_scale([-Real::from(2), Real::from(2), Real::from(2)]),
        // A reflection composed with shear and nonuniform scale.
        Matrix4::from_row_major([
            -Real::from(2),
            Real::zero(),
            q(1, 2),
            Real::one(),
            Real::zero(),
            Real::from(3),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            q(1, 3),
            Real::from(4),
            -Real::one(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ]),
    ]
}

#[test]
fn reflections_transport_a_pcurve_shared_across_chart_pivots() {
    let sweep = unit_square_sweep();
    let independent = sweep.model().clone();
    let shared = shared_pcurve_sweep(&sweep);
    let replayed = replay(&shared);
    assert!(exact(&volume(&shared), &volume(&independent)));
    for transform in reversing_transforms() {
        let scale = transform.determinant().abs();
        let expected_volume = &volume(&independent) * &scale;
        let reference = independent.transformed(&transform).unwrap();
        for source in [&shared, &replayed] {
            let moved = source.transformed(&transform).unwrap();
            // One record per pivot: the shared segment splits once.
            assert_eq!(moved.counts().pcurves, source.counts().pcurves + 1);
            assert!(exact(&volume(&moved), &expected_volume));
            let moved_replay = replay(&moved);
            assert!(exact(&volume(&moved_replay), &expected_volume));
            for sample in samples() {
                let image = transform.transform_point3(&sample).unwrap();
                let expected = locate(&independent, &sample);
                assert_eq!(locate(&reference, &image), expected);
                assert_eq!(locate(&moved, &image), expected, "{sample:?}");
                assert_eq!(locate(&moved_replay, &image), expected);
            }
            // Reflecting back restores the set without another split.
            let restored = moved
                .transformed(&transform.clone().inverse().unwrap())
                .unwrap();
            assert_eq!(restored.counts().pcurves, moved.counts().pcurves);
            assert!(exact(&volume(&restored), &volume(&independent)));
            for sample in samples() {
                assert_eq!(locate(&restored, &sample), locate(&independent, &sample));
            }
        }
    }
}

#[test]
fn reflections_of_shared_pcurve_models_commute_with_booleans() {
    let sweep = unit_square_sweep();
    let shared = BodySet::from_solid(shared_pcurve_sweep(&sweep).solids().next().unwrap());
    let slab = builder::cuboid(
        point(-Real::from(2), -Real::from(2), Real::one()),
        point(Real::from(3), Real::from(3), Real::from(3)),
    )
    .unwrap();
    for transform in reversing_transforms() {
        let moved_shared = shared.transformed(&transform).unwrap();
        let moved_slab = slab.transformed(&transform).unwrap();
        for (operation, before, after) in [
            (
                "intersection",
                boolean::intersection(&shared, &slab).unwrap(),
                boolean::intersection(&moved_shared, &moved_slab).unwrap(),
            ),
            (
                "difference",
                boolean::difference(&shared, &slab).unwrap(),
                boolean::difference(&moved_shared, &moved_slab).unwrap(),
            ),
        ] {
            let image = before.transformed(&transform).unwrap();
            let scale = transform.determinant().abs();
            assert!(
                exact(&volume(after.model()), &(&volume(before.model()) * &scale)),
                "{operation}"
            );
            assert!(exact(&volume(image.model()), &volume(after.model())));
            for sample in samples() {
                let mapped = transform.transform_point3(&sample).unwrap();
                let expected = locate(before.model(), &sample);
                assert_eq!(
                    locate(after.model(), &mapped),
                    expected,
                    "{operation} {sample:?}"
                );
                assert_eq!(locate(image.model(), &mapped), expected);
            }
        }
    }
}
