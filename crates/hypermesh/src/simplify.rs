//! Exact canonicalization for selected cell complexes.
//!
//! Simplification here means deterministic exact topology cleanup: duplicate
//! selected cells are removed, duplicate boundary nodes are collapsed, and cell
//! order is normalized. It deliberately does not introduce epsilon repair or
//! approximate mesh export.

use std::cmp::Ordering;

use super::arrangement3d::{
    ArrangementFaceCellNode, ArrangementLowerDimensionalArtifact, ExactArrangement,
};
use super::boolean::ExactBooleanOperation;
use super::cell_complex::{
    ExactCellComplexFace, ExactCellRegionLabel, ExactOppositeRegionLabel, ExactSelectedCellComplex,
    ExactSelectedFaceOrientation,
};
use super::loop_triangulation::{choose_polygon_projection, triangulate_exact_loop_group};
use super::mesh::{ExactMesh, Triangle};
use super::regularization::{ExactArrangementBlocker, ExactRegularizationPolicy};
use super::validation::ValidationPolicy;
use super::view::{ApproximateMeshF64View, approximate_mesh_f64_view};
use hyperlimit::CoplanarProjection;
use hyperlimit::{ApproximationPolicy, SourceProvenance};
use hyperlimit::{
    Point3, Sign, compare_reals, orient2d_report, point3_equal, project_point3,
    projected_polygon_area2_value,
};
use hyperreal::Real;

/// One simplified selected face-cell.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactSimplifiedFaceCell {
    /// Original selected face index in the labeled complex.
    pub source_face: usize,
    /// Canonicalized face-cell payload.
    pub face: ExactCellComplexFace,
}

/// Exact simplification report and retained output cells.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactSimplifiedCellComplex {
    /// Boolean operation whose selected cells were simplified.
    pub operation: ExactBooleanOperation,
    /// Canonical selected face-cells.
    pub faces: Vec<ExactSimplifiedFaceCell>,
    /// Retained lower-dimensional arrangement artifacts under policy.
    pub lower_dimensional_artifacts: Vec<ArrangementLowerDimensionalArtifact>,
    /// Number of duplicate selected cells removed.
    pub duplicate_cells_removed: usize,
    /// Number of consecutive duplicate boundary nodes removed.
    pub duplicate_boundary_nodes_removed: usize,
    /// Number of exact collinear boundary nodes removed.
    pub collinear_boundary_nodes_removed: usize,
    /// Number of zero-area selected cells dissolved.
    pub zero_area_cells_removed: usize,
    /// Number of exact internal edges removed between same-label cells.
    pub interior_edges_removed: usize,
    /// Blockers inherited or introduced during simplification.
    pub blockers: Vec<ExactArrangementBlocker>,
}

impl ExactSimplifiedCellComplex {
    /// Validate this simplified complex by replaying the full arrangement,
    /// label, selection, and simplification pipeline from source operands.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<(), ExactArrangementBlocker> {
        let replay = ExactArrangement::from_meshes_with_policy(left, right, policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?
            .label_regions(policy)?
            .select_with_policy(self.operation, policy)?
            .simplify_exact_with_policy(policy)?;
        if replay == *self {
            Ok(())
        } else {
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        }
    }

    /// Triangulate selected cells into an exact mesh.
    ///
    /// The retained boundary of each selected face-cell is projected through a
    /// certified nonzero carrier-plane projection and triangulated by
    /// `hypertri` over exact coordinates. No primitive-float tolerance is used.
    pub fn triangulate(&self) -> Result<ExactMesh, ExactArrangementBlocker> {
        triangulate_simplified_cell_complex(self)
    }

    /// Refuse primitive-float export unless the caller names the approximation
    /// policy at the exact-to-view boundary.
    pub fn approximate_f64_view(&self) -> Result<ApproximateMeshF64View, ExactArrangementBlocker> {
        Err(ExactArrangementBlocker::ApproximationPolicyRequired)
    }

    /// Build a primitive-float view only under an explicit export policy.
    ///
    /// Simplification remains exact; the lossy `f64` rows are produced after
    /// exact triangulation and retain the normal exact mesh replay audit.
    pub fn approximate_f64_view_with_policy(
        &self,
        policy: ApproximationPolicy,
    ) -> Result<ApproximateMeshF64View, ExactArrangementBlocker> {
        match policy {
            ApproximationPolicy::ExactOnly => {
                Err(ExactArrangementBlocker::ApproximationPolicyRequired)
            }
            ApproximationPolicy::EdgeOnly | ApproximationPolicy::ExplicitApproximateDecision => {
                let mesh = self.triangulate()?;
                approximate_mesh_f64_view(&mesh)
                    .map_err(|_| ExactArrangementBlocker::ApproximationPolicyRequired)
            }
        }
    }
}

/// Simplify a selected cell complex by exact canonicalization.
pub fn simplify_selected_cell_complex(
    selected: ExactSelectedCellComplex,
    policy: ExactRegularizationPolicy,
) -> Result<ExactSimplifiedCellComplex, ExactArrangementBlocker> {
    let mut blockers = selected.blockers;
    let mut faces = Vec::new();
    let mut duplicate_cells_removed = 0;
    let mut duplicate_boundary_nodes_removed = 0;
    let mut collinear_boundary_nodes_removed = 0;
    let mut zero_area_cells_removed = 0;
    let mut interior_edges_removed = 0;
    let selected_face_orientations = selected.selected_face_orientations.clone();
    let require_volume_orientations = !matches!(
        selected.operation,
        ExactBooleanOperation::SelectedRegions(_)
    ) && !selected.volume_adjacencies.is_empty();
    let volume_adjacency_faces = volume_adjacency_face_membership(
        selected.faces.len(),
        &selected.volume_adjacencies,
        require_volume_orientations,
    );

    for source_face in selected.selected_faces {
        let mut face = selected.faces[source_face].clone();
        let require_volume_orientation = require_volume_orientations
            && volume_adjacency_faces
                .get(source_face)
                .copied()
                .unwrap_or(false);
        match selected_face_reverse_orientation(
            &face,
            source_face,
            selected.operation,
            &selected_face_orientations,
            require_volume_orientation,
        ) {
            Ok(true) => {
                face.cell.boundary.reverse();
                face.cell.boundary_points.reverse();
            }
            Ok(false) => {}
            Err(blocker) => {
                blockers.push(blocker);
                continue;
            }
        }
        duplicate_boundary_nodes_removed += remove_consecutive_duplicate_nodes(&mut face);
        collinear_boundary_nodes_removed +=
            remove_collinear_boundary_nodes(&mut face, &mut blockers);
        if face.cell.boundary.len() < 3 {
            blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
            continue;
        }
        match boundary_has_nonzero_area(&face.cell.boundary_points) {
            Ok(true) => {}
            Ok(false) => {
                zero_area_cells_removed += 1;
                continue;
            }
            Err(blocker) => {
                blockers.push(blocker);
                continue;
            }
        }
        canonicalize_boundary_start(&mut face);
        if faces.iter().any(|existing: &ExactSimplifiedFaceCell| {
            existing.face == face
                || exact_boundary_loops_same_orientation(
                    &existing.face.cell.boundary_points,
                    &face.cell.boundary_points,
                )
        }) {
            duplicate_cells_removed += 1;
            continue;
        }
        if let Some(opposite) = faces.iter().position(|existing: &ExactSimplifiedFaceCell| {
            exact_boundary_loops_opposite_orientation(
                &existing.face.cell.boundary_points,
                &face.cell.boundary_points,
            )
        }) {
            faces.remove(opposite);
            duplicate_cells_removed += 2;
            continue;
        }
        faces.push(ExactSimplifiedFaceCell { source_face, face });
    }

    let merged = merge_same_label_adjacent_faces(faces, &mut blockers);
    let mut faces = merged.faces;
    interior_edges_removed += merged.interior_edges_removed;
    collinear_boundary_nodes_removed += merged.collinear_boundary_nodes_removed;
    zero_area_cells_removed += merged.zero_area_cells_removed;

    faces.sort_by_key(|face| {
        (
            side_key(face.face.cell.carrier.side),
            face.face.cell.carrier.face,
            face.source_face,
        )
    });

    if !blockers.is_empty()
        && policy.unresolved == super::regularization::ExactUnresolvedPolicy::Block
    {
        return Err(blockers[0].clone());
    }

    Ok(ExactSimplifiedCellComplex {
        operation: selected.operation,
        faces,
        lower_dimensional_artifacts: selected.lower_dimensional_artifacts,
        duplicate_cells_removed,
        duplicate_boundary_nodes_removed,
        collinear_boundary_nodes_removed,
        zero_area_cells_removed,
        interior_edges_removed,
        blockers,
    })
}

#[derive(Clone)]
struct DirectedBoundaryEdge {
    from: ArrangementFaceCellNode,
    to: ArrangementFaceCellNode,
    from_point: Point3,
    to_point: Point3,
}

struct MergeSameLabelResult {
    faces: Vec<ExactSimplifiedFaceCell>,
    interior_edges_removed: usize,
    collinear_boundary_nodes_removed: usize,
    zero_area_cells_removed: usize,
}

fn merge_same_label_adjacent_faces(
    faces: Vec<ExactSimplifiedFaceCell>,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> MergeSameLabelResult {
    let mut groups = std::collections::BTreeMap::<_, Vec<ExactSimplifiedFaceCell>>::new();
    for face in faces {
        groups
            .entry(simplified_group_key(&face))
            .or_default()
            .push(face);
    }

    let mut merged_faces = Vec::new();
    let mut interior_edges_removed = 0;
    let mut collinear_boundary_nodes_removed = 0;
    let mut zero_area_cells_removed = 0;
    for (_, group) in groups {
        if group.len() < 2 {
            merged_faces.extend(group);
            continue;
        }
        match merge_same_label_group(group.clone()) {
            Ok((mut merged, removed)) if removed > 0 => {
                interior_edges_removed += removed;
                for mut face in merged.drain(..) {
                    collinear_boundary_nodes_removed +=
                        remove_collinear_boundary_nodes(&mut face.face, blockers);
                    match boundary_has_nonzero_area(&face.face.cell.boundary_points) {
                        Ok(true) => {
                            canonicalize_boundary_start(&mut face.face);
                            merged_faces.push(face);
                        }
                        Ok(false) => zero_area_cells_removed += 1,
                        Err(blocker) => blockers.push(blocker),
                    }
                }
            }
            Ok((group, _)) => merged_faces.extend(group),
            Err(blocker) => {
                blockers.push(blocker);
                merged_faces.extend(group);
            }
        }
    }

    MergeSameLabelResult {
        faces: merged_faces,
        interior_edges_removed,
        collinear_boundary_nodes_removed,
        zero_area_cells_removed,
    }
}

fn simplified_group_key(face: &ExactSimplifiedFaceCell) -> (usize, usize, usize, usize) {
    (
        side_key(face.face.cell.carrier.side),
        face.face.cell.carrier.face,
        region_label_key(face.face.source),
        opposite_label_key(face.face.opposite),
    )
}

const fn region_label_key(label: ExactCellRegionLabel) -> usize {
    match label {
        ExactCellRegionLabel::LeftBoundary => 0,
        ExactCellRegionLabel::RightBoundary => 1,
    }
}

const fn opposite_label_key(label: ExactOppositeRegionLabel) -> usize {
    match label {
        ExactOppositeRegionLabel::Inside => 0,
        ExactOppositeRegionLabel::Outside => 1,
        ExactOppositeRegionLabel::Boundary => 2,
        ExactOppositeRegionLabel::Unknown => 3,
    }
}

fn merge_same_label_group(
    group: Vec<ExactSimplifiedFaceCell>,
) -> Result<(Vec<ExactSimplifiedFaceCell>, usize), ExactArrangementBlocker> {
    let mut boundary_edges = Vec::<DirectedBoundaryEdge>::new();
    let mut interior_edges_removed = 0;
    for face in &group {
        if face.face.cell.boundary.len() != face.face.cell.boundary_points.len() {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        for index in 0..face.face.cell.boundary.len() {
            let next = (index + 1) % face.face.cell.boundary.len();
            let edge = DirectedBoundaryEdge {
                from: face.face.cell.boundary[index].clone(),
                to: face.face.cell.boundary[next].clone(),
                from_point: face.face.cell.boundary_points[index].clone(),
                to_point: face.face.cell.boundary_points[next].clone(),
            };
            if let Some(reverse) = boundary_edges
                .iter()
                .position(|existing| exact_edges_are_reversed(existing, &edge))
            {
                boundary_edges.remove(reverse);
                interior_edges_removed += 1;
            } else {
                boundary_edges.push(edge);
            }
        }
    }

    if interior_edges_removed == 0 {
        return Ok((group, 0));
    }
    if boundary_edges.is_empty() {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }

    let mut loops = Vec::new();
    while !boundary_edges.is_empty() {
        let first = boundary_edges.remove(0);
        let start = first.from.clone();
        let mut current = first.to.clone();
        let start_point = first.from_point.clone();
        let mut current_point = first.to_point.clone();
        let mut boundary = vec![first.from];
        let mut boundary_points = vec![first.from_point];
        let max_steps = boundary_edges.len().saturating_add(1);
        let mut guard = 0usize;
        while !same_node_or_point(&current, &current_point, &start, &start_point) {
            guard += 1;
            if guard > max_steps {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
            let Some(next_index) = boundary_edges.iter().position(|edge| {
                same_node_or_point(&edge.from, &edge.from_point, &current, &current_point)
            }) else {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            };
            let next = boundary_edges.remove(next_index);
            boundary.push(next.from.clone());
            boundary_points.push(next.from_point.clone());
            current = next.to;
            current_point = next.to_point;
        }
        loops.push((boundary, boundary_points));
    }

    let template = &group[0];
    let merged = loops
        .into_iter()
        .map(|(boundary, boundary_points)| {
            let mut face = template.face.clone();
            face.cell.boundary = boundary;
            face.cell.boundary_points = boundary_points;
            ExactSimplifiedFaceCell {
                source_face: template.source_face,
                face,
            }
        })
        .collect();
    Ok((merged, interior_edges_removed))
}

fn exact_edges_are_reversed(left: &DirectedBoundaryEdge, right: &DirectedBoundaryEdge) -> bool {
    (left.from == right.to && left.to == right.from)
        || (point3_equal(&left.from_point, &right.to_point).value() == Some(true)
            && point3_equal(&left.to_point, &right.from_point).value() == Some(true))
}

fn same_node_or_point(
    left_node: &ArrangementFaceCellNode,
    left_point: &Point3,
    right_node: &ArrangementFaceCellNode,
    right_point: &Point3,
) -> bool {
    left_node == right_node || point3_equal(left_point, right_point).value() == Some(true)
}

fn remove_consecutive_duplicate_nodes(face: &mut ExactCellComplexFace) -> usize {
    if face.cell.boundary.is_empty() {
        return 0;
    }
    let mut removed = 0;
    let mut canonical_boundary = Vec::new();
    let mut canonical_points = Vec::new();
    for (index, node) in face.cell.boundary.iter().enumerate() {
        if canonical_boundary.last() == Some(node) {
            removed += 1;
        } else {
            canonical_boundary.push(node.clone());
            if let Some(point) = face.cell.boundary_points.get(index) {
                canonical_points.push(point.clone());
            }
        }
    }
    if canonical_boundary.len() > 1 && canonical_boundary.first() == canonical_boundary.last() {
        canonical_boundary.pop();
        canonical_points.pop();
        removed += 1;
    }
    face.cell.boundary = canonical_boundary;
    if face.cell.boundary_points.len() == face.cell.boundary.len() + removed {
        face.cell.boundary_points = canonical_points;
    }
    removed
}

fn remove_collinear_boundary_nodes(
    face: &mut ExactCellComplexFace,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> usize {
    if face.cell.boundary.len() < 3 || face.cell.boundary_points.len() != face.cell.boundary.len() {
        return 0;
    }
    let Ok(projection) = choose_polygon_projection(&face.cell.boundary_points) else {
        return 0;
    };
    let mut removed = 0;
    let mut index = 0;
    while face.cell.boundary.len() >= 3 && index < face.cell.boundary.len() {
        let len = face.cell.boundary.len();
        let prev = (index + len - 1) % len;
        let next = (index + 1) % len;
        let a = project_point3(&face.cell.boundary_points[prev], projection);
        let b = project_point3(&face.cell.boundary_points[index], projection);
        let c = project_point3(&face.cell.boundary_points[next], projection);
        match orient2d_report(&a, &b, &c).value() {
            Some(Sign::Zero) => {
                face.cell.boundary.remove(index);
                face.cell.boundary_points.remove(index);
                removed += 1;
                index = index.saturating_sub(1);
            }
            Some(Sign::Positive | Sign::Negative) => index += 1,
            None => {
                blockers.push(ExactArrangementBlocker::UndecidableOrdering);
                index += 1;
            }
        }
    }
    removed
}

fn boundary_has_nonzero_area(points: &[Point3]) -> Result<bool, ExactArrangementBlocker> {
    if points.len() < 3 {
        return Ok(false);
    }
    let mut saw_undecidable = false;
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let area = projected_polygon_area2_value(points, projection);
        match compare_reals(&area, &Real::from(0)).value() {
            Some(Ordering::Less | Ordering::Greater) => return Ok(true),
            Some(Ordering::Equal) => {}
            None => saw_undecidable = true,
        }
    }
    if saw_undecidable {
        Err(ExactArrangementBlocker::UndecidableOrdering)
    } else {
        Ok(false)
    }
}

fn canonicalize_boundary_start(face: &mut ExactCellComplexFace) {
    let Some((index, _)) = face
        .cell
        .boundary
        .iter()
        .enumerate()
        .min_by_key(|(_, node)| format!("{node:?}"))
    else {
        return;
    };
    face.cell.boundary.rotate_left(index);
    if face.cell.boundary_points.len() == face.cell.boundary.len() {
        face.cell.boundary_points.rotate_left(index);
    }
}

fn exact_boundary_loops_same_orientation(left: &[Point3], right: &[Point3]) -> bool {
    if left.len() != right.len() {
        return false;
    }
    if left.is_empty() {
        return true;
    }
    (0..right.len()).any(|offset| {
        (0..left.len()).all(|index| {
            point3_equal(&left[index], &right[(offset + index) % right.len()]).value() == Some(true)
        })
    })
}

fn exact_boundary_loops_opposite_orientation(left: &[Point3], right: &[Point3]) -> bool {
    if left.len() != right.len() {
        return false;
    }
    if left.is_empty() {
        return true;
    }
    (0..right.len()).any(|offset| {
        (0..left.len()).all(|index| {
            let right_index = (offset + right.len() - index) % right.len();
            point3_equal(&left[index], &right[right_index]).value() == Some(true)
        })
    })
}

fn selected_face_reverse_orientation(
    face: &ExactCellComplexFace,
    source_face: usize,
    operation: ExactBooleanOperation,
    orientations: &[ExactSelectedFaceOrientation],
    require_volume_orientation: bool,
) -> Result<bool, ExactArrangementBlocker> {
    let mut matches = orientations
        .iter()
        .filter(|orientation| orientation.face == source_face);
    if let Some(first) = matches.next() {
        let reverse = first.reverse;
        let mut has_volume_orientation = first.from_volume_adjacency;
        for orientation in matches {
            if orientation.reverse != reverse {
                return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
            }
            has_volume_orientation |= orientation.from_volume_adjacency;
        }
        if require_volume_orientation && !has_volume_orientation {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        return Ok(reverse);
    }
    if require_volume_orientation {
        return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
    }
    Ok(operation == ExactBooleanOperation::Difference
        && face.source == ExactCellRegionLabel::RightBoundary)
}

fn volume_adjacency_face_membership(
    face_count: usize,
    volume_adjacencies: &[super::arrangement3d::ArrangementVolumeAdjacency],
    enabled: bool,
) -> Vec<bool> {
    let mut membership = vec![false; face_count];
    if !enabled {
        return membership;
    }
    for adjacency in volume_adjacencies {
        for side in &adjacency.oriented_face_sides {
            if let Some(member) = membership.get_mut(side.face_cell) {
                *member = true;
            }
        }
    }
    membership
}

const fn side_key(side: super::graph::MeshSide) -> usize {
    match side {
        super::graph::MeshSide::Left => 0,
        super::graph::MeshSide::Right => 1,
    }
}

fn triangulate_simplified_cell_complex(
    complex: &ExactSimplifiedCellComplex,
) -> Result<ExactMesh, ExactArrangementBlocker> {
    let mut vertices = Vec::<Point3>::new();
    let mut triangles = Vec::<Triangle>::new();

    let mut groups = std::collections::BTreeMap::<_, Vec<usize>>::new();
    for (index, face) in complex.faces.iter().enumerate() {
        groups
            .entry(simplified_group_key(face))
            .or_default()
            .push(index);
    }

    for face_indices in groups.values() {
        triangulate_simplified_face_group(complex, face_indices, &mut vertices, &mut triangles)?;
    }

    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact simplified arrangement cell complex"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .map_err(|_| ExactArrangementBlocker::NonManifoldCellComplex)
}

fn triangulate_simplified_face_group(
    complex: &ExactSimplifiedCellComplex,
    face_indices: &[usize],
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<(), ExactArrangementBlocker> {
    let mut boundaries = Vec::new();
    for &face_index in face_indices {
        let boundary = complex.faces[face_index].face.cell.boundary_points.clone();
        if boundary.len() < 3 {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        boundaries.push(boundary);
    }
    triangulate_exact_loop_group(&boundaries, vertices, triangles)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arrangement3d::{
        ArrangementFaceCarrier, ArrangementFaceCell, ArrangementFaceCellNode,
        ArrangementVolumeAdjacency, ArrangementVolumeFaceSide,
    };
    use crate::cell_complex::{
        ExactCellComplexFace, ExactCellRegionLabel, ExactOppositeRegionLabel,
        ExactSelectedCellComplex, ExactSelectedFaceOrientation,
    };
    use crate::graph::MeshSide;
    use crate::loop_triangulation::{
        emitted_triangle_orientation, projected_loop_interior_witness,
    };
    use hyperlimit::{Point2, RingPointLocation, classify_point_ring_even_odd};

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn source_node_on(side: MeshSide, vertex: usize) -> ArrangementFaceCellNode {
        ArrangementFaceCellNode::SourceVertex { side, vertex }
    }

    fn selected_face(
        _source_face: usize,
        vertices: &[usize],
        points: &[Point3],
    ) -> ExactCellComplexFace {
        selected_face_with_source(
            MeshSide::Left,
            ExactCellRegionLabel::LeftBoundary,
            vertices,
            points,
        )
    }

    fn selected_face_with_source(
        side: MeshSide,
        source: ExactCellRegionLabel,
        vertices: &[usize],
        points: &[Point3],
    ) -> ExactCellComplexFace {
        ExactCellComplexFace {
            cell: ArrangementFaceCell {
                carrier: ArrangementFaceCarrier {
                    side,
                    face: 0,
                    triangle: [0, 1, 2],
                },
                boundary: vertices
                    .iter()
                    .map(|vertex| source_node_on(side, *vertex))
                    .collect(),
                boundary_points: points.to_vec(),
                opposite: None,
            },
            source,
            opposite: ExactOppositeRegionLabel::Outside,
        }
    }

    fn dummy_volume_adjacency(face_cell: usize) -> ArrangementVolumeAdjacency {
        ArrangementVolumeAdjacency {
            shell_region: 0,
            exterior_volume: 0,
            interior_volume: 1,
            separating_face_cells: vec![face_cell],
            oriented_face_sides: vec![ArrangementVolumeFaceSide {
                face_cell,
                source: MeshSide::Right,
                source_face: 0,
                boundary: vec![
                    source_node_on(MeshSide::Right, 0),
                    source_node_on(MeshSide::Right, 1),
                    source_node_on(MeshSide::Right, 2),
                ],
                exterior_volume: 0,
                interior_volume: 1,
            }],
        }
    }

    fn simplified_square() -> ExactSimplifiedCellComplex {
        let v0 = p(0, 0, 0);
        let v1 = p(1, 0, 0);
        let v2 = p(1, 1, 0);
        let v3 = p(0, 1, 0);
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face(0, &[0, 1, 2], &[v0.clone(), v1, v2.clone()]),
                selected_face(1, &[0, 2, 3], &[v0, v2, v3]),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
    }

    #[test]
    fn simplification_removes_internal_edge_between_same_label_cells() {
        let simplified = simplified_square();
        assert_eq!(simplified.faces.len(), 1);
        assert_eq!(simplified.interior_edges_removed, 1);
        assert_eq!(simplified.faces[0].face.cell.boundary.len(), 4);
        let mesh = simplified.triangulate().unwrap();
        assert_eq!(mesh.vertices().len(), 4);
        assert_eq!(mesh.triangles().len(), 2);
    }

    #[test]
    fn simplification_merges_internal_edge_with_distinct_exact_nodes() {
        let left = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let right = [p(0, 1, 0), p(1, 0, 0), p(1, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face_with_source(
                    MeshSide::Left,
                    ExactCellRegionLabel::LeftBoundary,
                    &[0, 1, 2],
                    &left,
                ),
                selected_face_with_source(
                    MeshSide::Left,
                    ExactCellRegionLabel::LeftBoundary,
                    &[3, 4, 5],
                    &right,
                ),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1],
            selected_face_orientations: vec![
                ExactSelectedFaceOrientation {
                    face: 0,
                    reverse: false,
                    from_volume_adjacency: false,
                },
                ExactSelectedFaceOrientation {
                    face: 1,
                    reverse: false,
                    from_volume_adjacency: false,
                },
            ],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(simplified.faces.len(), 1);
        assert_eq!(simplified.interior_edges_removed, 1);
        assert!(simplified.blockers.is_empty());
        assert_eq!(simplified.faces[0].face.cell.boundary_points.len(), 4);
    }

    #[test]
    fn simplification_removes_exact_duplicate_selected_geometry_across_sources() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face_with_source(
                    MeshSide::Left,
                    ExactCellRegionLabel::LeftBoundary,
                    &[0, 1, 2],
                    &points,
                ),
                selected_face_with_source(
                    MeshSide::Right,
                    ExactCellRegionLabel::RightBoundary,
                    &[3, 4, 5],
                    &points,
                ),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1],
            selected_face_orientations: vec![
                ExactSelectedFaceOrientation {
                    face: 0,
                    reverse: false,
                    from_volume_adjacency: true,
                },
                ExactSelectedFaceOrientation {
                    face: 1,
                    reverse: false,
                    from_volume_adjacency: true,
                },
            ],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(simplified.faces.len(), 1);
        assert_eq!(simplified.duplicate_cells_removed, 1);
    }

    #[test]
    fn simplification_cancels_opposite_duplicate_selected_geometry() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let reversed = [points[0].clone(), points[2].clone(), points[1].clone()];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face_with_source(
                    MeshSide::Left,
                    ExactCellRegionLabel::LeftBoundary,
                    &[0, 1, 2],
                    &points,
                ),
                selected_face_with_source(
                    MeshSide::Right,
                    ExactCellRegionLabel::RightBoundary,
                    &[3, 5, 4],
                    &reversed,
                ),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1],
            selected_face_orientations: vec![
                ExactSelectedFaceOrientation {
                    face: 0,
                    reverse: false,
                    from_volume_adjacency: true,
                },
                ExactSelectedFaceOrientation {
                    face: 1,
                    reverse: false,
                    from_volume_adjacency: true,
                },
            ],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();

        assert!(simplified.faces.is_empty());
        assert_eq!(simplified.duplicate_cells_removed, 2);
        assert!(simplified.blockers.is_empty());
    }

    #[test]
    fn simplification_uses_selected_face_orientation_evidence() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face(0, &[0, 1, 2], &points)],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0],
            selected_face_orientations: vec![ExactSelectedFaceOrientation {
                face: 0,
                reverse: true,
                from_volume_adjacency: true,
            }],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(simplified.faces.len(), 1);
        let area = projected_polygon_area2_value(
            &simplified.faces[0].face.cell.boundary_points,
            CoplanarProjection::Xy,
        );
        assert_eq!(
            compare_reals(&area, &Real::from(0)).value(),
            Some(Ordering::Less)
        );
    }

    #[test]
    fn simplification_blocks_missing_volume_orientation_evidence() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face_with_source(
                MeshSide::Right,
                ExactCellRegionLabel::RightBoundary,
                &[0, 1, 2],
                &points,
            )],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![dummy_volume_adjacency(0)],
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Difference,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );
    }

    #[test]
    fn simplification_allows_label_orientation_outside_volume_adjacency() {
        let volume_points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let label_points = [p(2, 0, 0), p(3, 0, 0), p(2, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face_with_source(
                    MeshSide::Left,
                    ExactCellRegionLabel::LeftBoundary,
                    &[0, 1, 2],
                    &volume_points,
                ),
                selected_face_with_source(
                    MeshSide::Right,
                    ExactCellRegionLabel::RightBoundary,
                    &[3, 4, 5],
                    &label_points,
                ),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![dummy_volume_adjacency(0)],
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1],
            selected_face_orientations: vec![
                ExactSelectedFaceOrientation {
                    face: 0,
                    reverse: false,
                    from_volume_adjacency: true,
                },
                ExactSelectedFaceOrientation {
                    face: 1,
                    reverse: false,
                    from_volume_adjacency: false,
                },
            ],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(simplified.faces.len(), 2);
        assert!(simplified.blockers.is_empty());
    }

    #[test]
    fn simplification_prefers_volume_orientation_over_difference_source_rule() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face_with_source(
                MeshSide::Right,
                ExactCellRegionLabel::RightBoundary,
                &[0, 1, 2],
                &points,
            )],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![dummy_volume_adjacency(0)],
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0],
            selected_face_orientations: vec![ExactSelectedFaceOrientation {
                face: 0,
                reverse: false,
                from_volume_adjacency: true,
            }],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Difference,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();
        let area = projected_polygon_area2_value(
            &simplified.faces[0].face.cell.boundary_points,
            CoplanarProjection::Xy,
        );

        assert_eq!(
            compare_reals(&area, &Real::from(0)).value(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn simplification_accepts_agreeing_mixed_orientation_evidence() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face_with_source(
                MeshSide::Right,
                ExactCellRegionLabel::RightBoundary,
                &[0, 1, 2],
                &points,
            )],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![dummy_volume_adjacency(0)],
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0],
            selected_face_orientations: vec![
                ExactSelectedFaceOrientation {
                    face: 0,
                    reverse: false,
                    from_volume_adjacency: true,
                },
                ExactSelectedFaceOrientation {
                    face: 0,
                    reverse: false,
                    from_volume_adjacency: false,
                },
            ],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Difference,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();
        let area = projected_polygon_area2_value(
            &simplified.faces[0].face.cell.boundary_points,
            CoplanarProjection::Xy,
        );

        assert_eq!(
            compare_reals(&area, &Real::from(0)).value(),
            Some(Ordering::Greater)
        );
    }

    #[test]
    fn simplification_rejects_conflicting_mixed_orientation_evidence() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face_with_source(
                MeshSide::Right,
                ExactCellRegionLabel::RightBoundary,
                &[0, 1, 2],
                &points,
            )],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![dummy_volume_adjacency(0)],
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0],
            selected_face_orientations: vec![
                ExactSelectedFaceOrientation {
                    face: 0,
                    reverse: false,
                    from_volume_adjacency: true,
                },
                ExactSelectedFaceOrientation {
                    face: 0,
                    reverse: true,
                    from_volume_adjacency: false,
                },
            ],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Difference,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );
    }

    #[test]
    fn approximate_view_requires_explicit_policy() {
        let simplified = simplified_square();

        assert_eq!(
            simplified.approximate_f64_view(),
            Err(ExactArrangementBlocker::ApproximationPolicyRequired)
        );
        assert_eq!(
            simplified.approximate_f64_view_with_policy(ApproximationPolicy::ExactOnly),
            Err(ExactArrangementBlocker::ApproximationPolicyRequired)
        );

        let view = simplified
            .approximate_f64_view_with_policy(ApproximationPolicy::EdgeOnly)
            .unwrap();
        assert!(view.lossy_view);
        assert_eq!(view.positions.len(), 12);
        assert_eq!(view.indices.len(), 6);
    }

    #[test]
    fn triangulation_preserves_grouped_hole_loop() {
        let outer = [p(0, 0, 0), p(4, 0, 0), p(4, 4, 0), p(0, 4, 0)];
        let hole = [p(1, 1, 0), p(1, 3, 0), p(3, 3, 0), p(3, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face(0, &[0, 1, 2, 3], &outer),
                selected_face(1, &[4, 5, 6, 7], &hole),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();
        let mesh = simplified.triangulate().unwrap();

        assert_eq!(mesh.vertices().len(), 8);
        assert_eq!(mesh.triangles().len(), 8);
    }

    #[test]
    fn triangulation_normalizes_outer_and_hole_orientation() {
        let outer = [p(0, 4, 0), p(4, 4, 0), p(4, 0, 0), p(0, 0, 0)];
        let hole = [p(3, 1, 0), p(3, 3, 0), p(1, 3, 0), p(1, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face(0, &[0, 1, 2, 3], &outer),
                selected_face(1, &[4, 5, 6, 7], &hole),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();
        let mesh = simplified.triangulate().unwrap();

        assert_eq!(mesh.vertices().len(), 8);
        assert_eq!(mesh.triangles().len(), 8);
    }

    #[test]
    fn triangulation_preserves_volume_reversed_holed_orientation() {
        let outer = [p(0, 0, 0), p(4, 0, 0), p(4, 4, 0), p(0, 4, 0)];
        let hole = [p(1, 1, 0), p(1, 3, 0), p(3, 3, 0), p(3, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face(0, &[0, 1, 2, 3], &outer),
                selected_face(1, &[4, 5, 6, 7], &hole),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![dummy_volume_adjacency(0), dummy_volume_adjacency(1)],
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1],
            selected_face_orientations: vec![
                ExactSelectedFaceOrientation {
                    face: 0,
                    reverse: true,
                    from_volume_adjacency: true,
                },
                ExactSelectedFaceOrientation {
                    face: 1,
                    reverse: true,
                    from_volume_adjacency: true,
                },
            ],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Difference,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();
        let mesh = simplified.triangulate().unwrap();

        assert_eq!(mesh.vertices().len(), 8);
        assert_eq!(mesh.triangles().len(), 8);
        assert_eq!(
            compare_reals(
                &mesh_projected_area2(&mesh, CoplanarProjection::Xy),
                &Real::from(0)
            )
            .value(),
            Some(Ordering::Less)
        );
    }

    #[test]
    fn triangulation_rejects_overlapping_same_depth_loops() {
        let left = [p(0, 0, 0), p(4, 0, 0), p(4, 4, 0), p(0, 4, 0)];
        let right = [p(2, 1, 0), p(6, 1, 0), p(6, 3, 0), p(2, 3, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face(0, &[0, 1, 2, 3], &left),
                selected_face(1, &[4, 5, 6, 7], &right),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(
            simplified.triangulate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn triangulation_rejects_point_touching_same_depth_loops() {
        let left = [p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0)];
        let right = [p(2, 2, 0), p(4, 2, 0), p(4, 4, 0), p(2, 4, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face(0, &[0, 1, 2, 3], &left),
                selected_face(1, &[4, 5, 6, 7], &right),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(
            simplified.triangulate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn triangulation_preserves_nested_island_inside_hole() {
        let outer = [p(0, 0, 0), p(8, 0, 0), p(8, 8, 0), p(0, 8, 0)];
        let hole = [p(1, 1, 0), p(1, 7, 0), p(7, 7, 0), p(7, 1, 0)];
        let island = [p(3, 3, 0), p(5, 3, 0), p(5, 5, 0), p(3, 5, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face(0, &[0, 1, 2, 3], &outer),
                selected_face(1, &[4, 5, 6, 7], &hole),
                selected_face(2, &[8, 9, 10, 11], &island),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            selected_faces: vec![0, 1, 2],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();
        let mesh = simplified.triangulate().unwrap();

        assert_eq!(mesh.vertices().len(), 12);
        assert_eq!(mesh.triangles().len(), 10);
    }

    #[test]
    fn triangulation_rejects_degenerate_emitted_triangle() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)];

        assert_eq!(
            emitted_triangle_orientation(&points, CoplanarProjection::Xy, &[0, 1, 2]),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_retains_lower_dimensional_artifacts() {
        let point = p(1, 1, 0);
        let artifact = ArrangementLowerDimensionalArtifact::PointContact {
            left_face: 0,
            right_face: 1,
            point,
        };
        let selected = ExactSelectedCellComplex {
            faces: Vec::new(),
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: vec![artifact.clone()],
            selected_faces: Vec::new(),
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Intersection,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::RETAIN_ARTIFACTS)
                .unwrap();

        assert_eq!(simplified.lower_dimensional_artifacts, vec![artifact]);
    }

    #[test]
    fn triangulation_uses_interior_witness_for_nested_hole_ownership() {
        let outer = vec![
            Point2::new(Real::from(0), Real::from(0)),
            Point2::new(Real::from(8), Real::from(0)),
            Point2::new(Real::from(8), Real::from(8)),
            Point2::new(Real::from(0), Real::from(8)),
        ];
        let hole = vec![
            Point2::new(Real::from(1), Real::from(1)),
            Point2::new(Real::from(1), Real::from(7)),
            Point2::new(Real::from(7), Real::from(7)),
            Point2::new(Real::from(7), Real::from(1)),
        ];

        let witness = projected_loop_interior_witness(&hole).unwrap();

        assert_eq!(
            classify_point_ring_even_odd(&hole, &witness).value(),
            Some(RingPointLocation::Inside)
        );
        assert_eq!(
            classify_point_ring_even_odd(&outer, &witness).value(),
            Some(RingPointLocation::Inside)
        );
    }

    fn mesh_projected_area2(mesh: &ExactMesh, projection: CoplanarProjection) -> Real {
        mesh.triangles()
            .iter()
            .fold(Real::from(0), |area, triangle| {
                let points = [
                    mesh.vertices()[triangle.0[0]].clone(),
                    mesh.vertices()[triangle.0[1]].clone(),
                    mesh.vertices()[triangle.0[2]].clone(),
                ];
                area + &projected_polygon_area2_value(&points, projection)
            })
    }
}
