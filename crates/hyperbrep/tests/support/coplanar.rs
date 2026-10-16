//! Repeated curved-cap Booleans with independent exact expectations.
use hyperbrep::{BodySet, Matrix4, Model, Real, SolidPointLocation, boolean};

#[allow(dead_code)]
#[path = "seam_cuts.rs"]
mod seam;

fn measures(model: &Model, volume: &Real, area: &Real) {
    assert_eq!(model.solids().count(), 1);
    seam::equal(&model.solids().next().unwrap().volume().unwrap(), volume);
    seam::equal(
        &model
            .faces()
            .map(|face| face.area().unwrap())
            .fold(Real::zero(), |a, b| a + b),
        area,
    );
    for edge in model.edges() {
        let uses: Vec<_> = edge.uses().collect();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

pub fn duplicate_carriers(kind: usize) {
    let (source, _, _, _) = seam::fixture(kind);
    let mut archive: serde_json::Value =
        serde_json::from_str(&source.model().to_json().unwrap()).unwrap();
    let family = if kind == 2 { "Cone" } else { "Torus" };
    let surfaces = archive["surfaces"].as_array_mut().unwrap();
    let original = surfaces
        .iter()
        .position(|surface| surface.get(family).is_some())
        .unwrap();
    let duplicate = surfaces.len();
    surfaces.push(surfaces[original].clone());
    let mut changed = 0;
    for (index, face) in archive["faces"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .enumerate()
    {
        if index % 2 == 1 && face["surface"].as_u64() == Some(original as u64) {
            face["surface"] = duplicate.into();
            changed += 1;
        }
    }
    assert!(changed > 0);
    let encoded = serde_json::to_string(&archive).unwrap();
    let restored = hyperbrep::RawModel::from_json(&encoded, &hyperbrep::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    check_output(
        &BodySet::from_solid(restored.solids().next().unwrap()),
        kind,
        true,
        &Matrix4::identity(),
        &Real::one(),
    );
    let coordinate = if kind == 2 { "apex" } else { "center" };
    archive["surfaces"][duplicate][family][coordinate][0] =
        serde_json::to_value(seam::ratio(1, 1_000_000) * seam::ratio(1, 1_000_000)).unwrap();
    let malformed = serde_json::to_string(&archive).unwrap();
    assert!(
        hyperbrep::RawModel::from_json(&malformed, &hyperbrep::ArchiveLimits::default())
            .unwrap()
            .validate()
            .is_err()
    );
}

fn samples(kind: usize, restored: bool) -> Vec<(hyperbrep::Point3, SolidPointLocation)> {
    use SolidPointLocation::{Boundary, Inside, Outside};
    let p = seam::point;
    let delta = seam::ratio(1, 1_000_000) * seam::ratio(1, 1_000_000);
    let other = if restored { Inside } else { Outside };
    let seam_location = if restored { Inside } else { Boundary };
    match kind {
        0 => vec![
            (p(5, 0, 1), Inside),
            (p(5, 0, -1), other),
            (p(5, 0, 0), seam_location),
            (p(0, 0, 0), Outside),
            (p(5, 0, 2), Boundary),
            (
                hyperbrep::Point3::new(5.into(), 0.into(), delta.clone()),
                Inside,
            ),
            (hyperbrep::Point3::new(5.into(), 0.into(), -&delta), other),
            (
                hyperbrep::Point3::new(5.into(), 0.into(), Real::from(2) - &delta),
                Inside,
            ),
            (
                hyperbrep::Point3::new(5.into(), 0.into(), Real::from(2) + &delta),
                Outside,
            ),
        ],
        1 => vec![
            (p(5, 0, 0), Inside),
            (p(-5, 0, 0), other),
            (p(0, 5, 0), seam_location),
            (p(0, 0, 0), Outside),
            (p(7, 0, 0), Boundary),
            (
                hyperbrep::Point3::new(delta.clone(), 5.into(), 0.into()),
                Inside,
            ),
            (hyperbrep::Point3::new(-&delta, 5.into(), 0.into()), other),
            (
                hyperbrep::Point3::new(Real::from(7) - &delta, 0.into(), 0.into()),
                Inside,
            ),
            (
                hyperbrep::Point3::new(Real::from(7) + &delta, 0.into(), 0.into()),
                Outside,
            ),
        ],
        2 => vec![
            (p(1, 0, 3), Inside),
            (p(-1, 0, 3), other),
            (p(0, 0, 3), seam_location),
            (p(0, 0, 7), Outside),
            (
                hyperbrep::Point3::new(seam::ratio(7, 2), 0.into(), 3.into()),
                Boundary,
            ),
            (
                hyperbrep::Point3::new(delta.clone(), 0.into(), 3.into()),
                Inside,
            ),
            (hyperbrep::Point3::new(-&delta, 0.into(), 3.into()), other),
            (
                hyperbrep::Point3::new(seam::ratio(7, 2) - &delta, 0.into(), 3.into()),
                Inside,
            ),
            (
                hyperbrep::Point3::new(seam::ratio(7, 2) + &delta, 0.into(), 3.into()),
                Outside,
            ),
        ],
        _ => unreachable!(),
    }
}

fn check_output(result: &BodySet, kind: usize, restored: bool, map: &Matrix4, scale: &Real) -> u64 {
    let (_, _, half_volume, half_area) = seam::fixture(kind);
    let volume = if restored {
        Real::from(2) * half_volume
    } else {
        half_volume
    };
    let area = if restored {
        if kind == 2 {
            (Real::from(29) + Real::from(21) * Real::from(5).sqrt().unwrap()) * Real::pi()
        } else {
            Real::from(40) * Real::pi() * Real::pi()
        }
    } else {
        half_area
    };
    let volume = volume * scale * scale * scale;
    let area = area * scale * scale;
    assert_eq!(result.len(), 1);
    let replay = seam::reload(result.model());
    for model in [result.model(), &replay] {
        measures(model, &volume, &area);
        for (point, expected) in samples(kind, restored) {
            assert_eq!(
                model
                    .solids()
                    .next()
                    .unwrap()
                    .classify_point(&map.transform_point3(&point).unwrap())
                    .unwrap(),
                expected
            );
        }
    }
    let edge = replay.edges().next().unwrap();
    let parameter = ((edge.domain().start().try_real().unwrap()
        + edge.domain().end().try_real().unwrap())
        / Real::from(2))
    .unwrap();
    let split = edge.split(parameter.into()).unwrap();
    let replay = seam::reload(split.model());
    measures(&replay, &volume, &area);
    let moved = replay
        .transformed(&seam::similarity(&seam::ratio(1, 1000), true))
        .unwrap();
    measures(
        &seam::reload(&moved),
        &(volume * seam::ratio(1, 1_000_000_000)),
        &(area * seam::ratio(1, 1_000_000)),
    );
    receipt(result.model())
}

fn receipt(model: &Model) -> u64 {
    model
        .to_json()
        .unwrap()
        .bytes()
        .fold(0xcbf29ce484222325_u64, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
        })
}

/// Operation 0 recuts in both orders; 1 subtracts to empty and recovers; 2
/// reconstructs the complete source from complementary halves in both orders.
pub fn workflow(kind: usize, operation: usize, map: &Matrix4, scale: &Real) -> u64 {
    let (source, cutter, _, _) = seam::fixture(kind);
    let source = source.transformed(map).unwrap();
    let cutter = cutter.transformed(map).unwrap();
    let half = boolean::intersection(&source, &cutter).unwrap();
    let other = boolean::difference(&source, &cutter).unwrap();
    let mut checksum = 0xcbf29ce484222325_u64;
    let mut record = |value: u64| checksum = (checksum ^ value).wrapping_mul(0x100000001b3);
    match operation {
        0 => {
            for (index, result) in [
                boolean::intersection(&half, &cutter),
                boolean::intersection(&cutter, &half),
            ]
            .into_iter()
            .enumerate()
            {
                let result = result.unwrap_or_else(|error| {
                    panic!("kind={kind} recut order={index} scale={scale:?} map={map:?}: {error:?}")
                });
                record(check_output(&result, kind, false, map, scale));
            }
        }
        1 => {
            let empty = boolean::difference(&half, &cutter).unwrap_or_else(|error| {
                panic!("kind={kind} clear scale={scale:?} map={map:?}: {error:?}")
            });
            assert!(empty.is_empty());
            assert_eq!(
                empty.model().to_json().unwrap(),
                BodySet::empty().model().to_json().unwrap()
            );
            let recovered = boolean::union(&empty, &half).unwrap();
            record(receipt(empty.model()));
            record(check_output(&recovered, kind, false, map, scale));
        }
        2 => {
            for (index, result) in [boolean::union(&half, &other), boolean::union(&other, &half)]
                .into_iter()
                .enumerate()
            {
                let result = result.unwrap_or_else(|error| {
                    panic!(
                        "kind={kind} restore order={index} scale={scale:?} map={map:?}: {error:?}"
                    )
                });
                record(check_output(&result, kind, true, map, scale));
            }
        }
        _ => unreachable!(),
    }
    checksum
}

pub fn original(kind: usize, operation: usize) -> u64 {
    workflow(kind, operation, &Matrix4::identity(), &Real::one())
}

pub fn transforms() -> (usize, u64) {
    let mut count = 0;
    let mut checksum = 0xcbf29ce484222325_u64;
    for scale in [
        seam::ratio(1, 1000),
        seam::ratio(127, 5000),
        Real::from(2).sqrt().unwrap(),
    ] {
        for reflected in [false, true] {
            for kind in 0..3 {
                for operation in 0..3 {
                    let value = workflow(
                        kind,
                        operation,
                        &seam::similarity(&scale, reflected),
                        &scale,
                    );
                    checksum = (checksum ^ value).wrapping_mul(0x100000001b3);
                    count += 1;
                }
            }
        }
    }
    (count, checksum)
}

pub fn circular_boundary_provenance() -> usize {
    let source = hyperbrep::builder::cylinder(5.into(), 6.into()).unwrap();
    let other = seam::reload(source.model());
    let mut count = 0;
    for face in source.model().faces() {
        if face.surface().kind() != hyperbrep::SurfaceKind::Plane {
            continue;
        }
        let other_face = other.faces().nth(face.index()).unwrap();
        let pair = boolean::intersect_faces(face, other_face).unwrap().unwrap();
        let hyperbrep::FacePairTrim::CoincidentFaces { first_traces, .. } = pair.details().trim()
        else {
            panic!("coincident circular caps retain both boundary charts")
        };
        let partition = face
            .split_by_surface_curves(first_traces, hyperbrep::SurfaceIntersectionOperand::First)
            .unwrap();
        assert_eq!(
            partition.model().to_json().unwrap(),
            source.model().to_json().unwrap()
        );
        for trace in partition.details().traces() {
            assert_eq!(trace.splits().len(), 0);
            let boundary = trace.boundary().unwrap();
            assert!(boundary.source_model().same_revision(source.model()));
            let materialized = boundary
                .intersection()
                .first_pcurve()
                .materialize()
                .unwrap();
            for overlap in boundary.overlaps() {
                let use_ = overlap.edge_use();
                assert!(use_.model().same_revision(boundary.source_model()));
                let native = overlap.native_overlap().unwrap();
                assert!(native.includes_start() && native.includes_end());
                for (a, b) in [
                    (
                        overlap.trace_range().start(),
                        overlap.boundary_range().start(),
                    ),
                    (overlap.trace_range().end(), overlap.boundary_range().end()),
                ] {
                    let a = materialized.curve().point_at(&a.clone().into()).unwrap();
                    let b = use_.pcurve().curve().point_at(&b.clone().into()).unwrap();
                    let (a, b) = (a.coordinates().unwrap(), b.coordinates().unwrap());
                    seam::equal(a.x(), b.x());
                    seam::equal(a.y(), b.y());
                }
            }
            count += 1;
        }
        seam::reload(partition.model());
    }
    assert_eq!(count, 8);
    count
}
