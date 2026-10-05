//! Orthographic feature-edge occlusion for posed triangle meshes.
//!
//! A feature edge is a boundary edge, or an interior edge whose dihedral angle
//! is certified away from both zero and pi. Each edge is split where a
//! projected triangle boundary crosses it, then each piece is tested at its
//! midpoint. A strictly nearer triangle hides a piece. An undecided comparison
//! leaves the piece unknown.

use std::collections::BTreeMap;

use hyperlattice::{Point2, Point3, Real};
use hypermesh::TriangleMesh;

use super::assembly::{AssemblyDocument, PosedAssembly};
use super::blueprint::{
    BlueprintEdge, BlueprintEdgeStyle, BlueprintOcclusionStatus, BlueprintProjection,
    BlueprintReport, BlueprintView, ProjectedPoint2, edge, project_point,
};
use super::metadata::GeometryCertainty;
use super::scalar::{lerp_point, real_eq, real_ge, real_gt, real_le, real_lt};

struct WorldTriangle {
    points: [Point3; 3],
}

struct FeatureSegment {
    handle: String,
    start: Point3,
    end: Point3,
    skip: Vec<usize>,
    unknown: bool,
}

enum Hit {
    None,
    Parameter(Real),
    Undecided,
}

/// Projects a posed assembly and classifies feature edges and guide lines.
pub fn blueprint_from_posed_mesh(
    document: &AssemblyDocument,
    posed: &PosedAssembly,
    projection: BlueprintProjection,
    suppress_hidden: bool,
) -> BlueprintReport {
    let mut blockers = posed.blockers.clone();
    let mut worlds = Vec::new();
    let mut certainty = GeometryCertainty::NativeExactCsg;
    for part in &posed.parts {
        let Some(node) = document.node(part.node) else {
            blockers.push(format!("{} is missing from the assembly", part.name));
            continue;
        };
        certainty = weaker(certainty, node.geometry_certainty);
        let Some(mesh) = node.geometry.as_ref() else {
            continue;
        };
        match transform_mesh(mesh, &part.world) {
            Some(mesh) => worlds.push((part.name.clone(), mesh)),
            None => blockers.push(format!("{} placement is undecided", part.name)),
        }
    }

    let mut triangles = Vec::new();
    let mut features = Vec::new();
    for (handle, mesh) in &worlds {
        let extracted = feature_segments(handle, mesh, &mut blockers);
        let mut index_of = Vec::with_capacity(mesh.triangles.len());
        for triangle in mesh.triangles.iter() {
            let [a, b, c] = triangle.indices();
            if let (Some(a), Some(b), Some(c)) = (
                mesh.positions.get(a),
                mesh.positions.get(b),
                mesh.positions.get(c),
            ) {
                index_of.push(Some(triangles.len()));
                triangles.push(WorldTriangle {
                    points: [a.clone(), b.clone(), c.clone()],
                });
            } else {
                index_of.push(None);
                blockers.push(format!("{handle} triangle index is out of range"));
            }
        }
        for mut feature in extracted {
            feature.skip = feature
                .skip
                .into_iter()
                .filter_map(|index| index_of.get(index).copied().flatten())
                .collect();
            features.push(feature);
        }
    }

    let mut edges = Vec::new();
    for feature in &features {
        if feature.unknown {
            let (start, _) = project_point(&feature.start, projection);
            let (end, _) = project_point(&feature.end, projection);
            edges.push(edge(
                &feature.handle,
                &start,
                &end,
                BlueprintEdgeStyle::Unknown,
                BlueprintOcclusionStatus::UndecidedMesh,
                "dihedral comparison is undecided",
            ));
            continue;
        }
        edges.extend(classify_segment(
            &feature.handle,
            &feature.start,
            &feature.end,
            &feature.skip,
            &triangles,
            projection,
            suppress_hidden,
            false,
        ));
    }
    for guide in &posed.guides {
        let Some(node) = document.node(guide.node) else {
            continue;
        };
        edges.extend(classify_segment(
            &node.name,
            &guide.start,
            &guide.end,
            &[],
            &triangles,
            projection,
            suppress_hidden,
            true,
        ));
    }

    let view = BlueprintView {
        projection,
        exploded: posed.exploded,
        edges,
    };
    let empty = BlueprintView {
        projection,
        exploded: !posed.exploded,
        edges: Vec::new(),
    };
    let (assembled, exploded) = if posed.exploded {
        (empty, view)
    } else {
        (view, empty)
    };
    BlueprintReport {
        assembled,
        exploded,
        blockers,
        certainty,
    }
}

fn transform_mesh(mesh: &TriangleMesh, world: &hyperlattice::Matrix4) -> Option<TriangleMesh> {
    let mut positions = Vec::with_capacity(mesh.positions.len());
    for point in mesh.positions.iter() {
        positions.push(world.transform_point3(point).ok()?);
    }
    Some(TriangleMesh::new(positions, mesh.triangles.to_vec()))
}

fn feature_segments(
    handle: &str,
    mesh: &TriangleMesh,
    blockers: &mut Vec<String>,
) -> Vec<FeatureSegment> {
    let mut owners: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
    for (index, triangle) in mesh.triangles.iter().enumerate() {
        let [a, b, c] = triangle.indices();
        for (left, right) in [(a, b), (b, c), (c, a)] {
            if left == right {
                continue;
            }
            let key = if left < right {
                (left, right)
            } else {
                (right, left)
            };
            owners.entry(key).or_default().push(index);
        }
    }
    let context = hypermesh::MeshContext::new(crate::PREDICATE_POLICY);
    let mut segments = Vec::new();
    for ((left, right), triangles) in owners {
        let (Some(start), Some(end)) = (mesh.positions.get(left), mesh.positions.get(right))
        else {
            blockers.push(format!("{handle} feature edge index is out of range"));
            continue;
        };
        if real_eq(&start.x, &end.x) == Some(true)
            && real_eq(&start.y, &end.y) == Some(true)
            && real_eq(&start.z, &end.z) == Some(true)
        {
            continue;
        }
        let unknown = match triangles.as_slice() {
            [_] => false,
            [first, second] => match dihedral_is_sharp(mesh, &context, *first, *second) {
                Some(false) => continue,
                Some(true) => false,
                None => true,
            },
            _ => true,
        };
        segments.push(FeatureSegment {
            handle: handle.to_string(),
            start: start.clone(),
            end: end.clone(),
            skip: triangles,
            unknown,
        });
    }
    segments
}

fn dihedral_is_sharp(
    mesh: &TriangleMesh,
    context: &hypermesh::MeshContext,
    first: usize,
    second: usize,
) -> Option<bool> {
    let angle = dihedral(mesh, context, first, second)?;
    let flat_zero = real_eq(&angle, &Real::zero())?;
    let flat_pi = real_eq(&angle, &Real::pi())?;
    Some(!flat_zero && !flat_pi)
}

fn dihedral(
    mesh: &TriangleMesh,
    context: &hypermesh::MeshContext,
    first: usize,
    second: usize,
) -> Option<Real> {
    let first = *mesh.triangles.get(first)?;
    let second = *mesh.triangles.get(second)?;
    mesh.dihedral_angle(context, first, second)
        .ok()
        .map(|outcome| outcome.into_value())
}

fn classify_segment(
    handle: &str,
    start: &Point3,
    end: &Point3,
    skip: &[usize],
    triangles: &[WorldTriangle],
    projection: BlueprintProjection,
    suppress_hidden: bool,
    guide: bool,
) -> Vec<BlueprintEdge> {
    let (projected_start, _) = project_point(start, projection);
    let (projected_end, _) = project_point(end, projection);
    let mut parameters = vec![Real::zero(), Real::one()];
    for (index, triangle) in triangles.iter().enumerate() {
        if skip.contains(&index) {
            continue;
        }
        for edge_index in 0..3 {
            let a = project_point(&triangle.points[edge_index], projection).0;
            let b = project_point(&triangle.points[(edge_index + 1) % 3], projection).0;
            match intersection_parameter(&projected_start, &projected_end, &a, &b) {
                Hit::None => {},
                Hit::Parameter(parameter) => parameters.push(parameter),
                Hit::Undecided => {
                    return vec![edge(
                        handle,
                        &projected_start,
                        &projected_end,
                        BlueprintEdgeStyle::Unknown,
                        BlueprintOcclusionStatus::UndecidedComparison,
                        "edge intersection is undecided",
                    )];
                },
            }
        }
    }
    if !parameters_are_comparable(&parameters) {
        return vec![edge(
            handle,
            &projected_start,
            &projected_end,
            BlueprintEdgeStyle::Unknown,
            BlueprintOcclusionStatus::UndecidedComparison,
            "split parameters could not be ordered",
        )];
    }
    parameters.sort_by(|left, right| {
        super::scalar::real_cmp(left, right)
            .expect("every split parameter comparison was decided")
    });
    parameters.dedup_by(|left, right| real_eq(left, right) == Some(true));
    let mut edges = Vec::new();
    for pair in parameters.windows(2) {
        if real_eq(&pair[0], &pair[1]) == Some(true) {
            continue;
        }
        let two = Real::from(2);
        let parameter = ((pair[0].clone() + pair[1].clone()) / &two).expect("two is nonzero");
        let sample = lerp_point(start, end, &parameter);
        let (projected, depth) = project_point(&sample, projection);
        let style = classify_sample(
            &projected,
            &depth,
            skip,
            triangles,
            projection,
            suppress_hidden,
            guide,
        );
        let piece_start = lerp_projected(&projected_start, &projected_end, &pair[0]);
        let piece_end = lerp_projected(&projected_start, &projected_end, &pair[1]);
        let status = if style == BlueprintEdgeStyle::Unknown {
            BlueprintOcclusionStatus::UndecidedMesh
        } else {
            BlueprintOcclusionStatus::ExactMesh
        };
        let note = if guide {
            "explode guide"
        } else {
            "mesh feature edge"
        };
        push_merged(
            &mut edges,
            edge(handle, &piece_start, &piece_end, style, status, note),
        );
    }
    edges
}

fn classify_sample(
    point: &ProjectedPoint2,
    depth: &Real,
    skip: &[usize],
    triangles: &[WorldTriangle],
    projection: BlueprintProjection,
    suppress_hidden: bool,
    guide: bool,
) -> BlueprintEdgeStyle {
    let mut undecided = false;
    let mut boundaries = Vec::new();
    for (index, triangle) in triangles.iter().enumerate() {
        if skip.contains(&index) {
            continue;
        }
        match triangle_covers(triangle, point, depth, projection) {
            Cover::Strict => {
                return visibility_from_decision(Some(true), suppress_hidden, guide);
            },
            Cover::Boundary(hit) => boundaries.push(*hit),
            Cover::None => {},
            Cover::Undecided => undecided = true,
        }
    }
    match interior_of_adjacent_boundaries(&boundaries) {
        Some(true) => return visibility_from_decision(Some(true), suppress_hidden, guide),
        None => return visibility_from_decision(None, suppress_hidden, guide),
        Some(false) => {},
    }
    if undecided {
        visibility_from_decision(None, suppress_hidden, guide)
    } else {
        visibility_from_decision(Some(false), suppress_hidden, guide)
    }
}

enum Cover {
    /// Strict projected interior of a strictly nearer triangle.
    Strict,
    /// Relative interior of one edge of a strictly nearer triangle.
    Boundary(Box<BoundaryHit>),
    None,
    Undecided,
}

struct BoundaryHit {
    start: Point2,
    end: Point2,
    opposite: Point2,
}

fn triangle_covers(
    triangle: &WorldTriangle,
    point: &ProjectedPoint2,
    depth: &Real,
    projection: BlueprintProjection,
) -> Cover {
    let projected = triangle.points.each_ref().map(|vertex| {
        let (projected, depth) = project_point(vertex, projection);
        (Point2::new(projected.x, projected.y), depth)
    });
    let query = Point2::new(point.x.clone(), point.y.clone());
    cover_projected(&projected, &query, depth)
}

/// Classifies one projected triangle against a sample.
///
/// A strict interior and a strictly nearer plane hides the sample. A sample on
/// the relative interior of an edge does not, by itself: the silhouette of the
/// occluder is that boundary. The caller hides it only when a second nearer
/// triangle lies across the same edge, which is an internal triangulation
/// diagonal rather than the outline. An equal depth is decided and is not
/// nearer. A comparison `hyperlimit` cannot decide returns [`Cover::Undecided`].
fn cover_projected(triangle: &[(Point2, Real); 3], point: &Point2, depth: &Real) -> Cover {
    let (a, depth_a) = &triangle[0];
    let (b, depth_b) = &triangle[1];
    let (c, depth_c) = &triangle[2];
    let Some(area_sign) = orientation(a, b, c) else {
        return Cover::Undecided;
    };
    if area_sign == hyperlimit::Sign::Zero {
        return Cover::None;
    }
    let vertices = [a, b, c];
    let barycentric = [
        (
            orientation(point, b, c),
            hyperlimit::orient2d_value(point, b, c),
            depth_a,
        ),
        (
            orientation(a, point, c),
            hyperlimit::orient2d_value(a, point, c),
            depth_b,
        ),
        (
            orientation(a, b, point),
            hyperlimit::orient2d_value(a, b, point),
            depth_c,
        ),
    ];
    let mut delta = Real::zero();
    let mut boundary_edge = None;
    let mut zeros = 0usize;
    for (index, (sign, weight, vertex_depth)) in barycentric.into_iter().enumerate() {
        match sign {
            Some(sign) if sign == area_sign => {},
            Some(hyperlimit::Sign::Zero) => {
                zeros += 1;
                boundary_edge = Some(index);
            },
            Some(_) => return Cover::None,
            None => return Cover::Undecided,
        }
        let rise = vertex_depth.clone() - depth.clone();
        delta += weight * rise;
    }
    let nearer = match hyperlimit::classify_real_sign(&delta, crate::PREDICATE_POLICY).value()
    {
        Some(sign) if sign == area_sign => true,
        Some(_) => false,
        None => return Cover::Undecided,
    };
    if !nearer {
        return Cover::None;
    }
    match zeros {
        0 => Cover::Strict,
        1 => {
            let opposite_index =
                boundary_edge.expect("a zero barycentric weight selects its edge");
            let start = vertices[(opposite_index + 1) % 3].clone();
            let end = vertices[(opposite_index + 2) % 3].clone();
            Cover::Boundary(Box::new(BoundaryHit {
                start,
                end,
                opposite: vertices[opposite_index].clone(),
            }))
        },
        _ => Cover::None,
    }
}

/// A sample on a triangulation diagonal is inside the union of the two faces.
fn interior_of_adjacent_boundaries(hits: &[BoundaryHit]) -> Option<bool> {
    for (index, left) in hits.iter().enumerate() {
        for right in hits.iter().skip(index + 1) {
            match opposite_sides_of_shared_edge(left, right) {
                Some(true) => return Some(true),
                None => return None,
                Some(false) => {},
            }
        }
    }
    Some(false)
}

fn opposite_sides_of_shared_edge(left: &BoundaryHit, right: &BoundaryHit) -> Option<bool> {
    let start_on = orientation(&left.start, &left.end, &right.start)?;
    let end_on = orientation(&left.start, &left.end, &right.end)?;
    if start_on != hyperlimit::Sign::Zero || end_on != hyperlimit::Sign::Zero {
        return Some(false);
    }
    let left_side = orientation(&left.start, &left.end, &left.opposite)?;
    let right_side = orientation(&left.start, &left.end, &right.opposite)?;
    Some(
        left_side != hyperlimit::Sign::Zero
            && right_side != hyperlimit::Sign::Zero
            && left_side != right_side,
    )
}

fn orientation(a: &Point2, b: &Point2, c: &Point2) -> Option<hyperlimit::Sign> {
    hyperlimit::orient2(a, b, c, crate::PREDICATE_POLICY).value()
}

fn parameters_are_comparable(parameters: &[Real]) -> bool {
    parameters.iter().enumerate().all(|(index, left)| {
        parameters
            .iter()
            .skip(index + 1)
            .all(|right| super::scalar::real_cmp(left, right).is_some())
    })
}

/// Maps one depth decision to a line style.
///
/// `nearer == None` is an undecided comparison and stays unknown, including
/// when the caller asked to drop hidden lines.
pub(super) const fn visibility_from_decision(
    nearer: Option<bool>,
    suppress_hidden: bool,
    guide: bool,
) -> BlueprintEdgeStyle {
    match nearer {
        Some(true) if suppress_hidden => BlueprintEdgeStyle::Suppressed,
        Some(true) => BlueprintEdgeStyle::HiddenDashed,
        Some(false) if guide => BlueprintEdgeStyle::Guide,
        Some(false) => BlueprintEdgeStyle::Visible,
        None => BlueprintEdgeStyle::Unknown,
    }
}

fn intersection_parameter(
    start: &ProjectedPoint2,
    end: &ProjectedPoint2,
    other_start: &ProjectedPoint2,
    other_end: &ProjectedPoint2,
) -> Hit {
    let dx = &end.x - &start.x;
    let dy = &end.y - &start.y;
    let sx = &other_end.x - &other_start.x;
    let sy = &other_end.y - &other_start.y;
    let denominator = &dx * &sy - &dy * &sx;
    match real_eq(&denominator, &Real::zero()) {
        Some(true) => Hit::None,
        None => Hit::Undecided,
        Some(false) => {
            let rx = &other_start.x - &start.x;
            let ry = &other_start.y - &start.y;
            let t_numerator = &rx * &sy - &ry * &sx;
            let u_numerator = &rx * &dy - &ry * &dx;
            let Ok(t) = t_numerator / &denominator else {
                return Hit::Undecided;
            };
            let Ok(u) = u_numerator / &denominator else {
                return Hit::Undecided;
            };
            match (
                open_unit(&t),
                real_ge(&u, &Real::zero()),
                real_le(&u, &Real::one()),
            ) {
                (Some(true), Some(true), Some(true)) => Hit::Parameter(t),
                (Some(_), Some(_), Some(_)) => Hit::None,
                _ => Hit::Undecided,
            }
        },
    }
}

fn open_unit(parameter: &Real) -> Option<bool> {
    Some(real_gt(parameter, &Real::zero())? && real_lt(parameter, &Real::one())?)
}

fn lerp_projected(
    start: &ProjectedPoint2,
    end: &ProjectedPoint2,
    parameter: &Real,
) -> ProjectedPoint2 {
    ProjectedPoint2 {
        x: &start.x + (&end.x - &start.x) * parameter,
        y: &start.y + (&end.y - &start.y) * parameter,
    }
}

fn push_merged(edges: &mut Vec<BlueprintEdge>, next: BlueprintEdge) {
    if let Some(previous) = edges.last_mut()
        && previous.style == next.style
        && previous.part_handle == next.part_handle
        && real_eq(&previous.end.x, &next.start.x) == Some(true)
        && real_eq(&previous.end.y, &next.start.y) == Some(true)
    {
        previous.end = next.end;
        return;
    }
    edges.push(next);
}

const fn weaker(current: GeometryCertainty, next: GeometryCertainty) -> GeometryCertainty {
    const fn rank(certainty: GeometryCertainty) -> u8 {
        match certainty {
            GeometryCertainty::NativeExactCsg => 0,
            GeometryCertainty::CertifiedImported => 1,
            GeometryCertainty::LossyPreviewMesh => 2,
            GeometryCertainty::DisplayOnly => 3,
            GeometryCertainty::Stale => 4,
            GeometryCertainty::Missing => 5,
        }
    }
    if rank(next) > rank(current) {
        next
    } else {
        current
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parts::assembly::{
        AssemblyDocument, AssemblyNode, AssemblyView, ExplodeSpec, posed_parts,
    };
    use crate::parts::scalar::real_eq;
    use crate::solid::{self, SolidExt};
    use hyperlattice::{Real, Vector3};

    fn document_with(parts: Vec<(&str, TriangleMesh)>) -> (AssemblyDocument, PosedAssembly) {
        let mut document = AssemblyDocument::new("occlusion", "main");
        document.node_mut(document.root()).expect("root").instructions = "n/a".into();
        for (name, mesh) in parts {
            document
                .insert(document.root(), AssemblyNode::printed(name, name, mesh))
                .expect("part");
        }
        let posed = posed_parts(&document, AssemblyView::Assembled);
        (document, posed)
    }

    #[test]
    fn cube_feature_edges_are_visible() {
        let (document, posed) = document_with(vec![("cube", solid::cube(Real::from(2)))]);
        let report =
            blueprint_from_posed_mesh(&document, &posed, BlueprintProjection::Front, false);
        assert!(report.blockers.is_empty(), "{:?}", report.blockers);
        let edges = &report.assembled.edges;
        assert_eq!(edges.len(), 12, "a cube has twelve sharp edges");
        assert!(
            edges
                .iter()
                .all(|edge| edge.style == BlueprintEdgeStyle::Visible)
        );
    }

    #[test]
    fn front_box_hides_rear_feature_edges() {
        let rear =
            solid::cube(Real::from(2)).translated(Real::from(1), Real::from(1), Real::zero());
        let front = solid::cube(Real::from(6)).translated(
            Real::from(-1),
            Real::from(-1),
            Real::from(3),
        );
        let (document, posed) = document_with(vec![("rear", rear), ("front", front)]);
        let report =
            blueprint_from_posed_mesh(&document, &posed, BlueprintProjection::Front, false);
        let rear_edges: Vec<_> = report
            .assembled
            .edges
            .iter()
            .filter(|edge| edge.part_handle == "rear")
            .collect();
        assert!(!rear_edges.is_empty());
        assert!(
            rear_edges
                .iter()
                .all(|edge| edge.style == BlueprintEdgeStyle::HiddenDashed)
        );
    }

    #[test]
    fn half_covered_edge_splits() {
        let rear = solid::cube(Real::from(4));
        let front =
            solid::cube(Real::from(2)).translated(Real::zero(), Real::from(-1), Real::from(3));
        let (document, posed) = document_with(vec![("rear", rear), ("front", front)]);
        let report =
            blueprint_from_posed_mesh(&document, &posed, BlueprintProjection::Front, false);
        let along_y0: Vec<_> = report
            .assembled
            .edges
            .iter()
            .filter(|edge| {
                edge.part_handle == "rear"
                    && real_eq(&edge.start.y, &Real::zero()) == Some(true)
                    && real_eq(&edge.end.y, &Real::zero()) == Some(true)
            })
            .collect();
        assert!(
            along_y0
                .iter()
                .any(|edge| edge.style == BlueprintEdgeStyle::HiddenDashed)
        );
        assert!(
            along_y0
                .iter()
                .any(|edge| edge.style == BlueprintEdgeStyle::Visible)
        );
    }

    #[test]
    fn suppressed_front_box_still_hides_the_rear_box() {
        let rear =
            solid::cube(Real::from(2)).translated(Real::from(1), Real::from(1), Real::zero());
        let front = solid::cube(Real::from(6)).translated(
            Real::from(-1),
            Real::from(-1),
            Real::from(3),
        );
        let mut document = AssemblyDocument::new("occlusion", "main");
        document.node_mut(document.root()).expect("root").instructions = "n/a".into();
        document
            .insert(document.root(), AssemblyNode::printed("rear", "rear", rear))
            .expect("rear");
        let mut front_node = AssemblyNode::printed("front", "front", front);
        front_node.explode = ExplodeSpec::translation(Vector3::new([
            Real::zero(),
            Real::zero(),
            Real::from(30),
        ]));
        front_node.explode.suppressed = true;
        document.insert(document.root(), front_node).expect("front");
        let posed = posed_parts(
            &document,
            AssemblyView::Exploded {
                step: document.root(),
            },
        );
        let report =
            blueprint_from_posed_mesh(&document, &posed, BlueprintProjection::Front, false);
        let rear_edges: Vec<_> = report
            .exploded
            .edges
            .iter()
            .filter(|edge| edge.part_handle == "rear")
            .collect();
        assert!(!rear_edges.is_empty());
        assert!(
            rear_edges
                .iter()
                .all(|edge| edge.style == BlueprintEdgeStyle::HiddenDashed)
        );
    }

    #[test]
    fn undecided_depth_stays_unknown() {
        assert_eq!(
            visibility_from_decision(None, false, false),
            BlueprintEdgeStyle::Unknown
        );
        assert_eq!(
            visibility_from_decision(None, true, true),
            BlueprintEdgeStyle::Unknown
        );
    }

    #[test]
    fn svg_snapshot_mentions_a_rear_coordinate() {
        let rear =
            solid::cube(Real::from(2)).translated(Real::from(1), Real::from(1), Real::zero());
        let front = solid::cube(Real::from(6)).translated(
            Real::from(-1),
            Real::from(-1),
            Real::from(3),
        );
        let (document, posed) = document_with(vec![("rear", rear), ("front", front)]);
        let report =
            blueprint_from_posed_mesh(&document, &posed, BlueprintProjection::Front, false);
        let svg =
            super::super::blueprint::export_blueprint_svg(&report.assembled).expect("svg");
        assert!(svg.contains("stroke-dasharray=\"0.4 0.25\""));
        let sample = super::super::blueprint::export_real(&Real::from(1)).expect("one");
        assert!(
            svg.contains(&format!("x1=\"{sample}\""))
                || svg.contains(&format!("x2=\"{sample}\""))
        );
    }
}
