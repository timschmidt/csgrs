//! Admission-proof lifetime, whole-solid scope and replay regressions.

use super::*;
use crate::construction::{self, SphereVoid};
use std::cmp::Ordering;

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

fn exact(actual: Real, expected: Real) {
    assert_eq!(
        compare_reals(&actual, &expected, crate::STRICT_PREDICATES).value(),
        Some(Ordering::Equal)
    );
}

// Exercise ordinary solid admission using exactly the same geometry as the
// transfer path. This deliberately removes the previous whole-solid proofs,
// while preserving unrelated endpoint and boundary-image certificates.
fn append_admitted(model: &ArenaModel, builder: &mut ArenaModelBuilder) -> Vec<SolidId> {
    let mut model = model.clone();
    Arc::make_mut(&mut model.data)
        .solid_certificates_valid
        .fill(false);
    model.append_to_builder(builder).unwrap()
}

#[test]
fn builder_certificates_survive_interleaved_admission_transfer_and_replay() {
    let (cylinder, _) = construction::cylinder(2.into(), 3.into()).unwrap();
    let (sphere, _) = construction::sphere_with_voids(
        5.into(),
        &[SphereVoid {
            center: Point3::origin(),
            radius: Real::one(),
        }],
    )
    .unwrap();
    let (cuboid, _) = construction::cuboid(point(10, 0, 0), point(12, 3, 4)).unwrap();
    let mut expected = None;
    for mode in 0..4 {
        let mut builder = ArenaModelBuilder::new();
        let mut ids = Vec::new();
        for model in [&cylinder, &sphere, &cuboid] {
            ids.extend(if mode == 1 {
                model.append_to_builder(&mut builder).unwrap()
            } else {
                append_admitted(model, &mut builder)
            });
        }
        if mode == 3 {
            // A partial record with no positive proof must still derive the
            // same whole-solid facts. Unlike a transfer, absence is no veto.
            for record in &mut builder.solid_certificates {
                if let BuilderSolidCertificates::Admitted(proofs) = record {
                    proofs.cylinder = None;
                    proofs.sphere = None;
                }
            }
        }
        let model = builder.finish().unwrap();
        assert!(model.data.certified_cylinders[ids[0].index()].is_some());
        // Retaining a sufficient cylinder proof must not hide the secondary
        // prism proof needed by the common-axis Boolean continuation.
        assert!(model.data.certified_prisms[ids[0].index()].is_some());
        assert!(model.data.certified_prisms[ids[1].index()].is_none());
        let json = model.to_json().unwrap();
        let replayed = crate::ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        for model in [&model, &replayed] {
            exact(
                model.solid_volume(ids[0]).unwrap(),
                Real::from(12) * Real::pi(),
            );
            exact(
                model.solid_volume(ids[1]).unwrap(),
                (Real::from(496) * Real::pi() / Real::from(3)).unwrap(),
            );
            exact(model.solid_volume(ids[2]).unwrap(), Real::from(24));
            for (point, location) in [
                (point(0, 0, 0), SolidPointLocation::Outside),
                (point(1, 0, 0), SolidPointLocation::Boundary),
                (point(2, 0, 0), SolidPointLocation::Inside),
                (point(5, 0, 0), SolidPointLocation::Boundary),
            ] {
                assert_eq!(model.classify_point(ids[1], &point).unwrap(), location);
            }
            let tiny = (Real::one() / Real::from(1_000_000_000_000_i64)).unwrap();
            assert_eq!(
                model
                    .classify_point(
                        ids[1],
                        &Point3::new(Real::from(5) + tiny, 0.into(), 0.into())
                    )
                    .unwrap(),
                SolidPointLocation::Outside
            );
            assert_eq!(model.to_json().unwrap(), json);
        }
        if let Some(previous) = &expected {
            assert_eq!(previous, &json);
        }
        expected = Some(json);
    }
}

fn sphere_shell(
    builder: &mut ArenaModelBuilder,
    center: Point3,
    radius: i32,
    orientation: Orientation,
) -> ShellId {
    let surface = builder
        .surface(
            Surface::sphere(
                center,
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                radius.into(),
            )
            .unwrap(),
        )
        .unwrap();
    let face = builder.whole_face(surface, orientation).unwrap();
    builder.shell(vec![face]).unwrap()
}

#[test]
fn builder_failed_nesting_and_ownership_leave_no_admitted_solid() {
    let mut builder = ArenaModelBuilder::new();
    let outer = sphere_shell(&mut builder, Point3::origin(), 5, Orientation::Forward);
    let outside = sphere_shell(&mut builder, point(10, 0, 0), 1, Orientation::Reversed);
    assert_eq!(
        builder.solid(outer, vec![outside]),
        Err(BuildError::VoidShellOutside(outside))
    );
    assert!(builder.solids.is_empty());
    let first = builder.solid(outer, Vec::new()).unwrap();
    assert_eq!(first.index(), 0);
    assert_eq!(
        builder.solid(outer, Vec::new()),
        Err(BuildError::ShellAlreadyOwned(outer))
    );
    assert_eq!(builder.solids.len(), 1);
    let second_shell = sphere_shell(&mut builder, point(20, 0, 0), 2, Orientation::Forward);
    let second = builder.solid(second_shell, Vec::new()).unwrap();
    let model = builder.finish().unwrap();
    exact(
        model.solid_volume(first).unwrap(),
        (Real::from(500) * Real::pi() / Real::from(3)).unwrap(),
    );
    exact(
        model.solid_volume(second).unwrap(),
        (Real::from(32) * Real::pi() / Real::from(3)).unwrap(),
    );
}

#[test]
fn builder_admission_does_not_hide_later_orphans_or_mutate_shared_pcurves() {
    let (model, _) = construction::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let mut builder = ArenaModelBuilder::new();
    append_admitted(&model, &mut builder);
    let original = builder.wires[0].edge_uses.clone();
    let before = original
        .iter()
        .map(|id| {
            builder.pcurves[builder.edge_uses[id.index()].pcurve.index()]
                .line_segment()
                .unwrap()
        })
        .map(|line| (line.start().clone(), line.end().clone()))
        .collect::<Vec<_>>();
    let duplicate = original
        .iter()
        .map(|id| {
            let edge_use = builder.edge_uses[id.index()].clone();
            builder
                .edge_use(
                    edge_use.edge,
                    edge_use.direction,
                    edge_use.pcurve,
                    edge_use.parameter_correspondence,
                )
                .unwrap()
        })
        .collect();
    let orphan = builder
        .wire_with_simplicity_certificate(duplicate, true)
        .unwrap();
    for (id, (start, end)) in original.iter().zip(before) {
        let line = builder.pcurves[builder.edge_uses[id.index()].pcurve.index()]
            .line_segment()
            .unwrap();
        assert_eq!(line.start(), &start);
        assert_eq!(line.end(), &end);
    }
    assert_eq!(
        builder.finish().unwrap_err().errors(),
        &[BuildError::OrphanWire(orphan)]
    );
}

#[test]
fn builder_replayed_surface_edit_cannot_reuse_a_stale_sphere_proof() {
    let (original, solid) = construction::sphere(5.into()).unwrap();
    let json = original.to_json().unwrap();
    let surface = original.surfaces().next().unwrap().0;
    let mut edit = original.edit();
    edit.replace_surface(
        surface,
        Surface::sphere(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            2.into(),
        )
        .unwrap(),
    )
    .unwrap();
    let changed = edit.commit().unwrap();
    exact(
        changed.solid_volume(solid).unwrap(),
        (Real::from(32) * Real::pi() / Real::from(3)).unwrap(),
    );
    assert_eq!(
        changed.classify_point(solid, &point(3, 0, 0)).unwrap(),
        SolidPointLocation::Outside
    );
    assert_eq!(
        original.classify_point(solid, &point(3, 0, 0)).unwrap(),
        SolidPointLocation::Inside
    );
    assert_eq!(original.to_json().unwrap(), json);
}

#[test]
fn generic_point_classification_rejects_uncertified_curved_faces() {
    let (cylinder, _) = construction::cylinder(2.into(), 3.into()).unwrap();
    let (solid, _) = cylinder.solids().next().unwrap();
    // Without its whole-solid family records, a curved shell reaches the
    // generic planar ray classifier. That path has no curved-face support and
    // must fail typed rather than assume every face is a plane.
    let mut uncertified = cylinder.clone();
    let data = Arc::make_mut(&mut uncertified.data);
    let index = solid.index();
    data.certified_cylinders[index] = None;
    data.certified_spheres[index] = None;
    data.certified_sphere_pairs[index] = None;
    data.certified_cone_frustums[index] = None;
    data.certified_tori[index] = None;
    data.certified_revolutions[index] = None;
    data.certified_lofts[index] = None;
    data.certified_curve_sweeps[index] = None;
    data.certified_prisms[index] = None;
    data.certified_axial_shells[index] = None;
    // An interior point is off both planar caps, so the curved wall decides.
    let interior = point(0, 0, 1);
    assert!(matches!(
        uncertified.classify_point(solid, &interior),
        Err(QueryError::Geometry(GeometryError::UnsupportedIntersection))
    ));
    // The certified model still answers through its family record.
    assert_eq!(
        cylinder.classify_point(solid, &interior).unwrap(),
        SolidPointLocation::Inside
    );
}
