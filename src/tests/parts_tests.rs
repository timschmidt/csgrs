//! Antagonistic tests for CSG part metadata and blueprint visibility.

use crate::{
    csg::CSG,
    mesh::Mesh,
    parts::{
        AssemblyDocumentation, AssemblyFlag, BlueprintEdgeStyle, BlueprintOcclusionStatus,
        BlueprintProjection, CsgPartInterface, ExactVector3, InstallationVector, PartMetadata,
        PartSource, blueprint_from_aabb_parts,
    },
};
use hyperlattice::Real;

fn vec3(x: i64, y: i64, z: i64) -> ExactVector3 {
    ExactVector3::from_i64(x, y, z)
}

fn source() -> PartSource {
    PartSource {
        family: "fixture".into(),
        revision: "test".into(),
    }
}

fn metadata(handle: &str, explode: Option<ExactVector3>) -> PartMetadata {
    let mut interface = CsgPartInterface::exact_csg("fixture-family", handle, source());
    interface.documentation = AssemblyDocumentation {
        installation: explode.map(|offset| {
            InstallationVector::new(vec3(0, 0, 1), offset, true, false)
                .expect("nonzero install direction")
        }),
        pose_hint: Some("front".into()),
        flags: Vec::new(),
    };
    PartMetadata::new(handle, interface)
}

#[test]
fn installation_vector_rejects_exact_zero_direction() {
    let vector = InstallationVector::new(vec3(0, 0, 0), vec3(0, 0, 10), true, false);
    assert!(vector.is_none());
}

#[test]
fn mesh_transform_preserves_part_metadata() {
    let mesh = Mesh::cube(Real::from(2), metadata("cube", Some(vec3(0, 0, 5))));
    let moved = mesh.translate(Real::from(3), Real::from(0), Real::from(0));
    let metadata = &moved.polygons[0].metadata;
    assert_eq!(metadata.handle, "cube");
    assert!(metadata.interface.has_installation_vector());
}

#[test]
fn front_blueprint_hides_fully_covered_rear_box() {
    let rear = Mesh::cube(Real::from(2), metadata("rear", Some(vec3(0, 0, -6)))).translate(
        Real::from(0),
        Real::from(0),
        Real::from(-3),
    );
    let front = Mesh::cube(Real::from(4), metadata("front", Some(vec3(0, 0, 6)))).translate(
        Real::from(0),
        Real::from(0),
        Real::from(3),
    );

    let report = blueprint_from_aabb_parts(&[rear, front], BlueprintProjection::Front, false);
    let rear_edges = report
        .assembled
        .edges
        .iter()
        .filter(|edge| edge.part_handle == "rear")
        .collect::<Vec<_>>();

    assert_eq!(rear_edges.len(), 4);
    assert!(
        rear_edges
            .iter()
            .all(|edge| edge.style == BlueprintEdgeStyle::HiddenDashed)
    );
    assert!(
        rear_edges
            .iter()
            .all(|edge| edge.evidence.status == BlueprintOcclusionStatus::ExactAabb)
    );
}

#[test]
fn suppress_hidden_switch_removes_fully_covered_rear_lines() {
    let rear = Mesh::cube(Real::from(2), metadata("rear", Some(vec3(0, 0, -6)))).translate(
        Real::from(0),
        Real::from(0),
        Real::from(-3),
    );
    let front = Mesh::cube(Real::from(4), metadata("front", Some(vec3(0, 0, 6)))).translate(
        Real::from(0),
        Real::from(0),
        Real::from(3),
    );

    let report = blueprint_from_aabb_parts(&[rear, front], BlueprintProjection::Front, true);
    assert!(
        report
            .assembled
            .edges
            .iter()
            .filter(|edge| edge.part_handle == "rear")
            .all(|edge| edge.style == BlueprintEdgeStyle::Suppressed)
    );
}

#[test]
fn partial_overlap_is_not_falsely_certified_as_hidden() {
    let left = Mesh::cube(Real::from(4), metadata("left", Some(vec3(0, 0, -6)))).translate(
        Real::from(0),
        Real::from(0),
        Real::from(-3),
    );
    let right = Mesh::cube(Real::from(4), metadata("right", Some(vec3(0, 0, 6)))).translate(
        Real::from(2),
        Real::from(0),
        Real::from(3),
    );

    let report = blueprint_from_aabb_parts(&[left, right], BlueprintProjection::Front, false);
    assert!(
        report
            .assembled
            .edges
            .iter()
            .filter(|edge| edge.part_handle == "left")
            .all(|edge| edge.style == BlueprintEdgeStyle::Unknown)
    );
    assert!(
        report
            .assembled
            .edges
            .iter()
            .filter(|edge| edge.part_handle == "left")
            .all(|edge| edge.evidence.status
                == BlueprintOcclusionStatus::PartialOverlapNeedsSplit)
    );
}

#[test]
fn exploded_view_uses_retained_installation_vectors() {
    let part = Mesh::cube(Real::from(2), metadata("moving", Some(vec3(10, 0, 0))));
    let report = blueprint_from_aabb_parts(&[part], BlueprintProjection::Front, false);
    let assembled_x = &report.assembled.edges[0].start.x;
    let exploded_x = &report.exploded.edges[0].start.x;
    assert_eq!(exploded_x.clone() - assembled_x.clone(), Real::from(10));
}

#[test]
fn missing_installation_vector_is_reported_not_guessed() {
    let part = Mesh::cube(Real::from(2), metadata("static", None));
    let report = blueprint_from_aabb_parts(&[part], BlueprintProjection::Front, false);
    assert_eq!(report.blockers, vec!["static missing installation vector"]);
}

#[test]
fn no_explode_flag_suppresses_missing_vector_blocker() {
    let mut meta = metadata("fixed", None);
    meta.interface
        .documentation
        .flags
        .push(AssemblyFlag::NoExplode);
    let part = Mesh::cube(Real::from(2), meta);
    let report = blueprint_from_aabb_parts(&[part], BlueprintProjection::Front, false);
    assert!(report.blockers.is_empty());
}
