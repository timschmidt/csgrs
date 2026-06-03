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
use super::coplanar::CoplanarProjection;
use super::mesh::{ExactMesh, Triangle};
use super::provenance::{ApproximationPolicy, SourceProvenance};
use super::regularization::{ExactArrangementBlocker, ExactRegularizationPolicy};
use super::validation::ValidationPolicy;
use super::view::{ApproximateMeshF64View, approximate_mesh_f64_view};
use hyperlimit::{
    Point2, Point3, RingPointLocation, SegmentIntersection, Sign, TriangleLocation,
    classify_point_ring_even_odd, classify_point_triangle, classify_segment_intersection,
    compare_reals, orient2d_report, point3_equal, project_point3, projected_polygon_area2_value,
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

    for source_face in selected.selected_faces {
        let mut face = selected.faces[source_face].clone();
        match selected_face_reverse_orientation(
            &face,
            source_face,
            selected.operation,
            &selected_face_orientations,
            require_volume_orientations,
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
        if faces
            .iter()
            .any(|existing: &ExactSimplifiedFaceCell| existing.face == face)
        {
            duplicate_cells_removed += 1;
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
            };
            if let Some(reverse) = boundary_edges
                .iter()
                .position(|existing| existing.from == edge.to && existing.to == edge.from)
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
        let mut boundary = vec![first.from];
        let mut boundary_points = vec![first.from_point];
        let max_steps = boundary_edges.len().saturating_add(1);
        let mut guard = 0usize;
        while current != start {
            guard += 1;
            if guard > max_steps {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
            let Some(next_index) = boundary_edges.iter().position(|edge| edge.from == current)
            else {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            };
            let next = boundary_edges.remove(next_index);
            boundary.push(next.from.clone());
            boundary_points.push(next.from_point.clone());
            current = next.to;
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
    let first = matches.next();
    if let Some(first) = first {
        for orientation in matches {
            if orientation.reverse != first.reverse
                || orientation.from_volume_adjacency != first.from_volume_adjacency
            {
                return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
            }
        }
        if require_volume_orientation && !first.from_volume_adjacency {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        return Ok(first.reverse);
    }
    if require_volume_orientation {
        return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
    }
    Ok(operation == ExactBooleanOperation::Difference
        && face.source == ExactCellRegionLabel::RightBoundary)
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

#[derive(Clone)]
struct ProjectedFaceLoop {
    boundary: Vec<Point3>,
    projection: CoplanarProjection,
    projected: Vec<Point2>,
    witness: Point2,
    depth: usize,
}

fn triangulate_simplified_face_group(
    complex: &ExactSimplifiedCellComplex,
    face_indices: &[usize],
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<(), ExactArrangementBlocker> {
    let mut loops = Vec::new();
    for &face_index in face_indices {
        let boundary = complex.faces[face_index].face.cell.boundary_points.clone();
        if boundary.len() < 3 {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        let projection = choose_polygon_projection(&boundary)?;
        let projected = boundary
            .iter()
            .map(|point| project_point3(point, projection))
            .collect::<Vec<_>>();
        let witness = projected_loop_interior_witness(&projected)?;
        loops.push(ProjectedFaceLoop {
            boundary,
            projection,
            projected,
            witness,
            depth: 0,
        });
    }
    compute_loop_depths(&mut loops)?;
    validate_loop_topology(&loops)?;
    let mut used_as_hole = vec![false; loops.len()];
    for outer_index in 0..loops.len() {
        if loops[outer_index].depth % 2 != 0 {
            continue;
        }
        let mut hole_indices = Vec::new();
        for hole_index in 0..loops.len() {
            if loops[hole_index].depth == loops[outer_index].depth + 1
                && loop_contains_loop(&loops[outer_index], &loops[hole_index])?
            {
                if used_as_hole[hole_index] {
                    return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                }
                hole_indices.push(hole_index);
                used_as_hole[hole_index] = true;
            }
        }
        triangulate_loop_with_holes(&loops, outer_index, &hole_indices, vertices, triangles)?;
    }
    for (index, loop_) in loops.iter().enumerate() {
        if loop_.depth % 2 != 0 && !used_as_hole[index] {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
    }
    Ok(())
}

fn validate_loop_topology(loops: &[ProjectedFaceLoop]) -> Result<(), ExactArrangementBlocker> {
    for left_index in 0..loops.len() {
        for right_index in (left_index + 1)..loops.len() {
            validate_loop_boundaries_are_disjoint(
                &loops[left_index].projected,
                &loops[right_index].projected,
            )?;
            if loops[left_index].depth == loops[right_index].depth {
                validate_same_depth_loops_are_area_disjoint(
                    &loops[left_index],
                    &loops[right_index],
                )?;
            }
        }
    }
    Ok(())
}

fn validate_loop_boundaries_are_disjoint(
    left: &[Point2],
    right: &[Point2],
) -> Result<(), ExactArrangementBlocker> {
    for left_index in 0..left.len() {
        let left_next = (left_index + 1) % left.len();
        for right_index in 0..right.len() {
            let right_next = (right_index + 1) % right.len();
            match classify_segment_intersection(
                &left[left_index],
                &left[left_next],
                &right[right_index],
                &right[right_next],
            )
            .value()
            {
                Some(SegmentIntersection::Disjoint) => {}
                Some(
                    SegmentIntersection::Proper
                    | SegmentIntersection::EndpointTouch
                    | SegmentIntersection::CollinearOverlap
                    | SegmentIntersection::Identical,
                ) => return Err(ExactArrangementBlocker::NonManifoldCellComplex),
                None => return Err(ExactArrangementBlocker::UndecidableOrdering),
            }
        }
    }
    Ok(())
}

fn validate_same_depth_loops_are_area_disjoint(
    left: &ProjectedFaceLoop,
    right: &ProjectedFaceLoop,
) -> Result<(), ExactArrangementBlocker> {
    validate_same_depth_loop_witness_outside(left, right)?;
    validate_same_depth_loop_witness_outside(right, left)
}

fn validate_same_depth_loop_witness_outside(
    container: &ProjectedFaceLoop,
    candidate: &ProjectedFaceLoop,
) -> Result<(), ExactArrangementBlocker> {
    match classify_point_ring_even_odd(&container.projected, &candidate.witness).value() {
        Some(RingPointLocation::Outside) => Ok(()),
        Some(RingPointLocation::Inside | RingPointLocation::Boundary) => {
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        }
        None => Err(ExactArrangementBlocker::UndecidableOrdering),
    }
}

fn compute_loop_depths(loops: &mut [ProjectedFaceLoop]) -> Result<(), ExactArrangementBlocker> {
    for loop_index in 0..loops.len() {
        let mut depth = 0;
        for container_index in 0..loops.len() {
            if loop_index == container_index {
                continue;
            }
            if loop_contains_loop(&loops[container_index], &loops[loop_index])? {
                depth += 1;
            }
        }
        loops[loop_index].depth = depth;
    }
    Ok(())
}

fn loop_contains_loop(
    container: &ProjectedFaceLoop,
    child: &ProjectedFaceLoop,
) -> Result<bool, ExactArrangementBlocker> {
    for point in &child.projected {
        match classify_point_ring_even_odd(&container.projected, point).value() {
            Some(RingPointLocation::Inside) => {}
            Some(RingPointLocation::Outside) => return Ok(false),
            Some(RingPointLocation::Boundary) => {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    match classify_point_ring_even_odd(&container.projected, &child.witness).value() {
        Some(RingPointLocation::Inside) => {}
        Some(RingPointLocation::Outside) => return Ok(false),
        Some(RingPointLocation::Boundary) => {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        None => return Err(ExactArrangementBlocker::UndecidableOrdering),
    }
    Ok(true)
}

fn projected_loop_interior_witness(points: &[Point2]) -> Result<Point2, ExactArrangementBlocker> {
    if points.len() < 3 {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    let signed_area_twice = signed_area_twice_points(points);
    let orientation = match compare_reals(&signed_area_twice, &Real::from(0)).value() {
        Some(Ordering::Greater) => Sign::Positive,
        Some(Ordering::Less) => Sign::Negative,
        Some(Ordering::Equal) => return Err(ExactArrangementBlocker::NonManifoldCellComplex),
        None => return Err(ExactArrangementBlocker::UndecidableOrdering),
    };

    for index in 0..points.len() {
        let previous = &points[(index + points.len() - 1) % points.len()];
        let current = &points[index];
        let next = &points[(index + 1) % points.len()];
        match orient2d_report(previous, current, next).value() {
            Some(sign) if sign == orientation => {}
            Some(Sign::Zero | Sign::Positive | Sign::Negative) => continue,
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }

        let mut contains_vertex = false;
        for (candidate_index, candidate) in points.iter().enumerate() {
            if candidate_index == index
                || candidate_index == (index + points.len() - 1) % points.len()
                || candidate_index == (index + 1) % points.len()
            {
                continue;
            }
            match classify_point_triangle(previous, current, next, candidate).value() {
                Some(TriangleLocation::Inside) => {
                    contains_vertex = true;
                    break;
                }
                Some(
                    TriangleLocation::Outside
                    | TriangleLocation::OnEdge
                    | TriangleLocation::OnVertex,
                ) => {}
                Some(TriangleLocation::Degenerate) => {
                    contains_vertex = true;
                    break;
                }
                None => return Err(ExactArrangementBlocker::UndecidableOrdering),
            }
        }
        if contains_vertex {
            continue;
        }

        let witness = triangle_centroid_2d(previous, current, next)?;
        match classify_point_ring_even_odd(points, &witness).value() {
            Some(RingPointLocation::Inside) => return Ok(witness),
            Some(RingPointLocation::Outside | RingPointLocation::Boundary) => {}
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }

    Err(ExactArrangementBlocker::NonManifoldCellComplex)
}

fn signed_area_twice_points(points: &[Point2]) -> Real {
    let mut area = Real::from(0);
    for index in 0..points.len() {
        let current = &points[index];
        let next = &points[(index + 1) % points.len()];
        area = area + &(current.x.clone() * &next.y) - &(current.y.clone() * &next.x);
    }
    area
}

fn triangle_centroid_2d(
    a: &Point2,
    b: &Point2,
    c: &Point2,
) -> Result<Point2, ExactArrangementBlocker> {
    let third = (Real::from(1) / &Real::from(3))
        .ok()
        .ok_or(ExactArrangementBlocker::UndecidableOrdering)?;
    Ok(Point2::new(
        (a.x.clone() + &b.x + &c.x) * &third,
        (a.y.clone() + &b.y + &c.y) * &third,
    ))
}

fn triangulate_loop_with_holes(
    loops: &[ProjectedFaceLoop],
    outer_index: usize,
    hole_loop_indices: &[usize],
    vertices: &mut Vec<Point3>,
    triangles: &mut Vec<Triangle>,
) -> Result<(), ExactArrangementBlocker> {
    let projection = loops[outer_index].projection;
    let mut polygon_points = if hole_loop_indices.is_empty() {
        loops[outer_index].boundary.clone()
    } else {
        oriented_loop_points_for_triangulation(
            &loops[outer_index].boundary,
            projection,
            Ordering::Greater,
        )?
    };
    let mut projected = polygon_points
        .iter()
        .map(|point| project_for_hypertri(point, projection))
        .collect::<Vec<_>>();
    let mut hole_indices = Vec::new();
    for &hole_index in hole_loop_indices {
        if loops[hole_index].projection != projection {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        hole_indices.push(projected.len());
        let hole_points = oriented_loop_points_for_triangulation(
            &loops[hole_index].boundary,
            projection,
            Ordering::Less,
        )?;
        polygon_points.extend(hole_points.iter().cloned());
        projected.extend(
            hole_points
                .iter()
                .map(|point| project_for_hypertri(point, projection)),
        );
    }
    let local_to_global = polygon_points
        .iter()
        .map(|point| find_or_insert_vertex(vertices, point.clone()))
        .collect::<Result<Vec<_>, _>>()?;
    if polygon_points.len() == 3 && hole_indices.is_empty() {
        triangles.push(Triangle([
            local_to_global[0],
            local_to_global[1],
            local_to_global[2],
        ]));
        return Ok(());
    }
    let indices = hypertri::earcut(&projected, &hole_indices)
        .map_err(|_| ExactArrangementBlocker::NonManifoldCellComplex)?;
    if indices.is_empty() {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    let mut emitted_triangles = indices.chunks_exact(3);
    for triangle in &mut emitted_triangles {
        validate_emitted_triangle_area(&polygon_points, projection, triangle)?;
        triangles.push(Triangle([
            local_to_global[triangle[0]],
            local_to_global[triangle[1]],
            local_to_global[triangle[2]],
        ]));
    }
    if !emitted_triangles.remainder().is_empty() {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Ok(())
}

fn validate_emitted_triangle_area(
    polygon_points: &[Point3],
    projection: CoplanarProjection,
    triangle: &[usize],
) -> Result<(), ExactArrangementBlocker> {
    let [a, b, c] = triangle else {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    };
    let points = [
        polygon_points
            .get(*a)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?
            .clone(),
        polygon_points
            .get(*b)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?
            .clone(),
        polygon_points
            .get(*c)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?
            .clone(),
    ];
    match compare_reals(
        &projected_polygon_area2_value(&points, projection),
        &Real::from(0),
    )
    .value()
    {
        Some(Ordering::Less | Ordering::Greater) => Ok(()),
        Some(Ordering::Equal) => Err(ExactArrangementBlocker::NonManifoldCellComplex),
        None => Err(ExactArrangementBlocker::UndecidableOrdering),
    }
}

fn oriented_loop_points_for_triangulation(
    points: &[Point3],
    projection: CoplanarProjection,
    expected: Ordering,
) -> Result<Vec<Point3>, ExactArrangementBlocker> {
    let area = projected_polygon_area2_value(points, projection);
    match compare_reals(&area, &Real::from(0)).value() {
        Some(Ordering::Equal) => Err(ExactArrangementBlocker::NonManifoldCellComplex),
        Some(ordering) if ordering == expected => Ok(points.to_vec()),
        Some(Ordering::Less | Ordering::Greater) => {
            let mut reversed = points.to_vec();
            reversed.reverse();
            Ok(reversed)
        }
        None => Err(ExactArrangementBlocker::UndecidableOrdering),
    }
}

fn find_or_insert_vertex(
    vertices: &mut Vec<Point3>,
    point: Point3,
) -> Result<usize, ExactArrangementBlocker> {
    for (index, existing) in vertices.iter().enumerate() {
        match point3_equal(existing, &point).value() {
            Some(true) => return Ok(index),
            Some(false) => {}
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    let index = vertices.len();
    vertices.push(point);
    Ok(index)
}

fn choose_polygon_projection(
    points: &[Point3],
) -> Result<CoplanarProjection, ExactArrangementBlocker> {
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let area = projected_polygon_area2_value(points, projection);
        match compare_reals(&area, &Real::from(0)).value() {
            Some(Ordering::Less | Ordering::Greater) => return Ok(projection),
            Some(Ordering::Equal) => {}
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    Err(ExactArrangementBlocker::NonManifoldCellComplex)
}

fn project_for_hypertri(point: &Point3, projection: CoplanarProjection) -> hypertri::ExactPoint {
    match projection {
        CoplanarProjection::Xy => hypertri::ExactPoint::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => hypertri::ExactPoint::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => hypertri::ExactPoint::new(point.y.clone(), point.z.clone()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::arrangement3d::{
        ArrangementFaceCarrier, ArrangementFaceCell, ArrangementFaceCellNode,
        ArrangementVolumeAdjacency, ArrangementVolumeFaceSide,
    };
    use crate::exact::cell_complex::{
        ExactCellComplexFace, ExactCellRegionLabel, ExactOppositeRegionLabel,
        ExactSelectedCellComplex, ExactSelectedFaceOrientation,
    };
    use crate::exact::graph::MeshSide;

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
            validate_emitted_triangle_area(&points, CoplanarProjection::Xy, &[0, 1, 2]),
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
}
