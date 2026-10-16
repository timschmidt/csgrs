//! Seam-aligned complete workflows shared by native, WASM and cost consumers.
use hyperbrep::{
    ArchiveLimits, BodySet, Matrix4, Model, Point3, RawModel, Real, SolidPointLocation, boolean,
    builder,
};
use std::cmp::Ordering;
#[allow(dead_code)]
#[path = "closed_cylinder.rs"]
mod closed_cylinder;

pub fn ratio(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
pub fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
#[track_caller]
pub fn equal(a: &Real, b: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(a, b, hyperbrep::STRICT_PREDICATES).value(),
        Some(Ordering::Equal),
        "actual={a:?}, expected={b:?}"
    );
}
pub fn fixture(kind: usize) -> (BodySet, BodySet, Real, Real) {
    let source = if kind == 2 {
        builder::cone_frustum(5.into(), 2.into(), 6.into()).unwrap()
    } else {
        builder::torus(5.into(), 2.into()).unwrap()
    };
    let cutter = if kind == 0 {
        builder::cuboid(point(-20, -20, 0), point(20, 20, 20))
    } else {
        builder::cuboid(point(0, -20, -20), point(20, 20, 20))
    }
    .unwrap();
    let (volume, area) = if kind == 2 {
        (
            Real::from(39) * Real::pi(),
            Real::from(42)
                + ((Real::from(29) + Real::from(21) * Real::from(5).sqrt().unwrap()) * Real::pi()
                    / Real::from(2))
                .unwrap(),
        )
    } else {
        let half = Real::from(20) * Real::pi() * Real::pi();
        (
            half.clone(),
            half + Real::from(if kind == 0 { 40 } else { 8 }) * Real::pi(),
        )
    };
    (source, cutter, volume, area)
}
pub fn reload(model: &Model) -> Model {
    reload_with_context(model, "standalone replay")
}
fn reload_with_context(model: &Model, context: &str) -> Model {
    let json = model.to_json().unwrap();
    let loaded = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap_or_else(|error| panic!("{context}: archive decoding: {error:?}"))
        .validate()
        .unwrap_or_else(|error| panic!("{context}: {error:?}"));
    assert_eq!(loaded.to_json().unwrap(), json);
    loaded
}
fn measures(model: &Model, volume: &Real, area: &Real, context: &str) {
    equal(&model.solids().next().unwrap().volume().unwrap(), volume);
    let actual_area = model
        .faces()
        .map(|f| f.area().unwrap())
        .fold(Real::zero(), |sum, value| sum + value);
    assert_eq!(
        hyperlimit::compare_reals(&actual_area, area, hyperbrep::STRICT_PREDICATES).value(),
        Some(Ordering::Equal),
        "{context}: exact area, actual={actual_area:?}, expected={area:?}",
    );
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}
pub fn similarity(scale: &Real, reflected: bool) -> Matrix4 {
    let sign = if reflected { -Real::one() } else { Real::one() };
    // A cyclic permutation moves the old axial seam onto a world coordinate
    // plane; the rational rotation also exercises non-coordinate meridians.
    Matrix4::from_row_major([
        0.into(),
        0.into(),
        scale.clone(),
        3.into(),
        scale * ratio(3, 5) * &sign,
        -scale * ratio(4, 5),
        0.into(),
        (-2).into(),
        scale * ratio(4, 5) * sign,
        scale * ratio(3, 5),
        0.into(),
        7.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ])
}
fn samples(kind: usize, difference: bool) -> Vec<(Point3, SolidPointLocation)> {
    use SolidPointLocation::{Boundary, Inside, Outside};
    let sign = if difference {
        -Real::one()
    } else {
        Real::one()
    };
    let delta = ratio(1, 1_000_000) * ratio(1, 1_000_000);
    match kind {
        0 => vec![
            (Point3::new(5.into(), 0.into(), sign.clone()), Inside),
            (Point3::new(5.into(), 0.into(), -&sign), Outside),
            (point(5, 0, 0), Boundary),
            (point(0, 0, 0), Outside),
            (Point3::new(5.into(), 0.into(), &sign * &delta), Inside),
            (Point3::new(5.into(), 0.into(), -(&sign * &delta)), Outside),
            (
                Point3::new(5.into(), 0.into(), &sign * (Real::from(2) - &delta)),
                Inside,
            ),
            (
                Point3::new(5.into(), 0.into(), &sign * Real::from(2)),
                Boundary,
            ),
            (
                Point3::new(5.into(), 0.into(), &sign * (Real::from(2) + &delta)),
                Outside,
            ),
        ],
        1 => vec![
            (
                Point3::new(&sign * Real::from(5), 0.into(), 0.into()),
                Inside,
            ),
            (
                Point3::new(-(&sign * Real::from(5)), 0.into(), 0.into()),
                Outside,
            ),
            (point(0, 5, 0), Boundary),
            (point(0, 0, 0), Outside),
            (Point3::new(&sign * &delta, 5.into(), 0.into()), Inside),
            (Point3::new(-(&sign * &delta), 5.into(), 0.into()), Outside),
            (
                Point3::new(&sign * (Real::from(7) - &delta), 0.into(), 0.into()),
                Inside,
            ),
            (
                Point3::new(&sign * Real::from(7), 0.into(), 0.into()),
                Boundary,
            ),
            (
                Point3::new(&sign * (Real::from(7) + &delta), 0.into(), 0.into()),
                Outside,
            ),
        ],
        2 => vec![
            (Point3::new(sign.clone(), 0.into(), 3.into()), Inside),
            (Point3::new(-&sign, 0.into(), 3.into()), Outside),
            (point(0, 0, 3), Boundary),
            (point(0, 0, 7), Outside),
            (Point3::new(&sign * &delta, 0.into(), 3.into()), Inside),
            (Point3::new(-(&sign * &delta), 0.into(), 3.into()), Outside),
            (
                Point3::new(&sign * (ratio(7, 2) - &delta), 0.into(), 3.into()),
                Inside,
            ),
            (
                Point3::new(&sign * ratio(7, 2), 0.into(), 3.into()),
                Boundary,
            ),
            (
                Point3::new(&sign * (ratio(7, 2) + &delta), 0.into(), 3.into()),
                Outside,
            ),
        ],
        _ => unreachable!(),
    }
}
pub fn check_case(kind: usize, map: &Matrix4, scale: &Real) -> u64 {
    let (source, cutter, volume, area) = fixture(kind);
    let source = source.transformed(map).unwrap();
    let cutter = cutter.transformed(map).unwrap();
    let expected_volume = &volume * scale * scale * scale;
    let expected_area = &area * scale * scale;
    let mut receipt = 0xcbf29ce484222325_u64;
    for (operation_index, (difference, result)) in [
        (false, boolean::intersection(&source, &cutter)),
        (false, boolean::intersection(&cutter, &source)),
        (true, boolean::difference(&source, &cutter)),
    ]
    .into_iter()
    .enumerate()
    {
        let result = result.unwrap_or_else(|error| {
            panic!("case={kind} difference={difference}, scale={scale:?}, map={map:?}: {error:?}")
        });
        assert_eq!(result.len(), 1);
        let context = format!(
            "kind={kind} operation={operation_index} difference={difference} scale={scale:?} map={map:?}"
        );
        let loaded = reload_with_context(result.model(), &format!("{context} original replay"));
        for model in [result.model(), &loaded] {
            measures(
                model,
                &expected_volume,
                &expected_area,
                &format!("{context} original/replay"),
            );
            for (point, expected) in samples(kind, difference) {
                let point = map.transform_point3(&point).unwrap();
                assert_eq!(
                    model
                        .solids()
                        .next()
                        .unwrap()
                        .classify_point(&point)
                        .unwrap(),
                    expected,
                    "case={kind} difference={difference}"
                );
            }
        }
        for byte in loaded.to_json().unwrap().bytes() {
            receipt = (receipt ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
        // Consume the output through a shared-edge edit, untrusted replay and
        // a reflected similarity, retaining exact measures and incidence.
        let edge = loaded.edges().next().unwrap();
        let middle = ((edge.domain().start().try_real().unwrap()
            + edge.domain().end().try_real().unwrap())
            / Real::from(2))
        .unwrap();
        let split = edge.split(middle.into()).unwrap();
        let replay = reload_with_context(split.model(), &format!("{context} split replay"));
        measures(
            &replay,
            &expected_volume,
            &expected_area,
            &format!("{context} split/replay"),
        );
        let moved = replay
            .transformed(&similarity(&ratio(1, 1000), true))
            .unwrap();
        measures(
            &reload_with_context(&moved, &format!("{context} reflected/rescaled replay")),
            &(&expected_volume * ratio(1, 1_000_000_000)),
            &(&expected_area * ratio(1, 1_000_000)),
            &format!("{context} split/reflected/rescaled/replay"),
        );
    }
    receipt
}
pub fn original(kind: usize) -> u64 {
    check_case(kind, &Matrix4::identity(), &Real::one())
}
pub fn transforms() -> usize {
    let mut count = 0;
    for scale in [
        ratio(1, 1000),
        ratio(127, 5000),
        Real::from(2).sqrt().unwrap(),
    ] {
        for reflected in [false, true] {
            for kind in 0..3 {
                check_case(kind, &similarity(&scale, reflected), &scale);
                count += 3;
            }
        }
    }
    count
}

pub fn boundary_provenance() -> usize {
    let (source, cutter, _, _) = fixture(0);
    let graph = boolean::intersection_graph(
        source.bodies().next().unwrap(),
        cutter.bodies().next().unwrap(),
    )
    .unwrap();
    let partition = graph.partition_first_faces().unwrap();
    let mut boundaries = 0;
    for face in partition.partitions() {
        for trace in face.traces() {
            let Some(boundary) = trace.boundary() else {
                continue;
            };
            boundaries += 1;
            assert_eq!(trace.splits().len(), 0);
            assert_eq!(trace.segments().len(), 1);
            assert_eq!(
                boundary.operand(),
                hyperbrep::SurfaceIntersectionOperand::First
            );
            let overlaps = boundary.overlaps().collect::<Vec<_>>();
            assert!(!overlaps.is_empty());
            equal(
                overlaps.first().unwrap().trace_range().start(),
                &Real::zero(),
            );
            equal(overlaps.last().unwrap().trace_range().end(), &Real::one());
            for overlap in overlaps {
                let use_ = overlap.edge_use();
                assert!(use_.model().same_revision(boundary.source_model()));
                assert!(use_.edge().model().same_revision(boundary.source_model()));
                let materialized = boundary
                    .intersection()
                    .first_pcurve()
                    .materialize()
                    .unwrap();
                for (a, b) in [
                    (
                        overlap.trace_range().start(),
                        overlap.boundary_range().start(),
                    ),
                    (overlap.trace_range().end(), overlap.boundary_range().end()),
                ] {
                    let actual = materialized.curve().point_at(&a.clone().into()).unwrap();
                    let expected = use_.pcurve().curve().point_at(&b.clone().into()).unwrap();
                    assert_eq!(actual, expected);
                }
            }
            reload(boundary.source_model());
        }
    }
    assert_eq!(boundaries, 16);
    boundaries
}

pub fn closed_boundary_provenance() -> usize {
    for reversed in [false, true] {
        let model = closed_cylinder::build(Real::zero(), 0, reversed).unwrap();
        let face = model
            .faces()
            .find(|f| f.surface().kind() == hyperbrep::SurfaceKind::Cylinder)
            .unwrap();
        let plane = hyperbrep::Surface::plane(
            point(0, 0, 0),
            hyperbrep::Vector3::x(),
            hyperbrep::Vector3::y(),
        )
        .unwrap();
        let hyperbrep::SurfaceSurfaceIntersection::Curve(trace) =
            face.surface().intersect_surface(&plane).unwrap()
        else {
            panic!("a full cylinder latitude");
        };
        let original = model.to_json().unwrap();
        let partition = face
            .split_by_surface_curves(&[*trace], hyperbrep::SurfaceIntersectionOperand::First)
            .unwrap();
        assert_eq!(partition.model().to_json().unwrap(), original);
        let record = partition.details().traces().next().unwrap();
        assert!(record.splits().next().is_none());
        let boundary = record.boundary().unwrap();
        assert_eq!(boundary.overlaps().len(), 1);
        let edge = boundary.overlaps().next().unwrap().edge_use().edge();
        assert_eq!(edge.start(), edge.end());
        closed_cylinder::check(&reload(partition.model()));
    }
    2
}
