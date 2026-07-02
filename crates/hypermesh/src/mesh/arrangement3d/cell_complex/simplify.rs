//! Exact canonicalization for selected cell complexes.
//!
//! Simplification here means deterministic exact topology cleanup: duplicate
//! selected cells are removed, duplicate boundary nodes are collapsed, and cell
//! order is normalized. It deliberately does not introduce epsilon repair or
//! approximate mesh export.

use std::cmp::Ordering;

use super::super::super::boolean::ExactBooleanOperation;
use super::super::super::graph::MeshSide;
use super::super::super::validation::MeshValidationMode;
use super::super::super::{
    Mesh, Triangle, orient_paired_triangle_edges, point3_exact_equal,
    remove_duplicate_triangle_vertex_sets,
};
#[cfg(test)]
use super::super::ExactArrangement3d;
use super::super::loop_triangulation::{
    choose_polygon_projection, group_exact_coplanar_loops, triangulate_exact_loop_group,
};
use super::super::regularization::{ExactArrangementBlocker, ExactRegularizationMode};
use super::super::{
    ArrangementFaceCellNode, ArrangementLowerDimensionalArtifact, ExactTopologyAssemblyReport,
    exact_point_loops_match, validate_lower_dimensional_artifacts,
};
#[cfg(test)]
use super::select_arrangement_for_replay;
use super::{
    ExactCellComplexFace, ExactCellRegionLabel, ExactOppositeRegionLabel,
    ExactRegionOwnershipReport, ExactSelectedCellComplex, ExactSelectedFaceOrientation,
    selected_cell_complex_gate_counts, validate_selected_gate_reports,
    validate_selected_gate_reports_against_counts, validate_volume_adjacency_face_provenance,
};
use hyperlimit::CoplanarProjection;
use hyperlimit::SourceProvenance;
use hyperlimit::{
    Point3, Sign, compare_reals, orient2d_report, orient3d_report, point_on_segment3,
    project_point3, projected_polygon_area2_value,
};
use hyperreal::Real;

/// One simplified selected face-cell.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactSimplifiedFaceCell {
    /// Original selected face index in the labeled complex.
    pub(crate) source_face: usize,
    /// Canonicalized face-cell payload.
    pub(crate) face: ExactCellComplexFace,
}

/// Exact simplification report and retained output cells.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExactSimplifiedCellComplex {
    /// Boolean operation whose selected cells were simplified.
    pub(crate) operation: ExactBooleanOperation,
    /// Canonical selected face-cells.
    pub(crate) faces: Vec<ExactSimplifiedFaceCell>,
    /// Retained lower-dimensional arrangement artifacts under mode.
    pub(crate) lower_dimensional_artifacts: Vec<ArrangementLowerDimensionalArtifact>,
    /// Topology assembly report consumed before the selected cells were simplified.
    pub(crate) topology_assembly_report: Option<ExactTopologyAssemblyReport>,
    /// Region ownership report consumed before the selected cells were simplified.
    pub(crate) region_ownership_report: Option<ExactRegionOwnershipReport>,
    /// Selected face count consumed before simplification merged or dissolved cells.
    pub(crate) selected_faces_before_simplification: usize,
    /// Boundary-node count across selected faces before simplification.
    pub(crate) selected_boundary_nodes_before_simplification: usize,
    /// Selected faces with explicit orientation evidence before simplification.
    pub(crate) oriented_selected_faces_before_simplification: usize,
    /// Selected oriented faces whose output orientation was reversed before simplification.
    pub(crate) reversed_selected_faces_before_simplification: usize,
    /// Selected oriented faces justified by volume adjacency evidence before simplification.
    pub(crate) volume_oriented_selected_faces_before_simplification: usize,
    /// Selected oriented faces justified by source-label operation rules before simplification.
    pub(crate) label_oriented_selected_faces_before_simplification: usize,
    /// Number of duplicate selected cells removed.
    pub(crate) duplicate_cells_removed: usize,
    /// Number of consecutive duplicate boundary nodes removed.
    pub(crate) duplicate_boundary_nodes_removed: usize,
    /// Number of exact collinear boundary nodes removed.
    pub(crate) collinear_boundary_nodes_removed: usize,
    /// Number of zero-area selected cells dissolved.
    pub(crate) zero_area_cells_removed: usize,
    /// Number of exact internal edges removed between compatible selected cells.
    pub(crate) interior_edges_removed: usize,
    /// Blockers inherited or introduced during simplification.
    pub(crate) blockers: Vec<ExactArrangementBlocker>,
}

impl ExactSimplifiedCellComplex {
    /// Validate local simplified-cell consistency without replaying source meshes.
    pub(crate) fn validate(&self) -> Result<(), ExactArrangementBlocker> {
        validate_lower_dimensional_artifacts(&self.lower_dimensional_artifacts)?;
        validate_selected_gate_reports(
            self.topology_assembly_report.as_ref(),
            self.region_ownership_report.as_ref(),
            self.operation,
        )?;
        let Some(oriented_selected_faces) = self
            .volume_oriented_selected_faces_before_simplification
            .checked_add(self.label_oriented_selected_faces_before_simplification)
        else {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        };
        let Some(removed_selected_faces) = self
            .duplicate_cells_removed
            .checked_add(self.zero_area_cells_removed)
        else {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        };
        let Some(accounted_selected_faces) = self.faces.len().checked_add(removed_selected_faces)
        else {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        };
        let Some(simplified_boundary_nodes) = self.faces.iter().try_fold(0usize, |total, face| {
            total.checked_add(face.face.cell.boundary.len())
        }) else {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        };
        let Some(removed_boundary_nodes) = self
            .duplicate_boundary_nodes_removed
            .checked_add(self.collinear_boundary_nodes_removed)
        else {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        };
        let Some(accounted_boundary_nodes) =
            simplified_boundary_nodes.checked_add(removed_boundary_nodes)
        else {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        };
        let Some(min_selected_boundary_nodes) =
            self.selected_faces_before_simplification.checked_mul(3)
        else {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        };
        if self.faces.len() > self.selected_faces_before_simplification
            || self.selected_boundary_nodes_before_simplification < min_selected_boundary_nodes
            || simplified_boundary_nodes > self.selected_boundary_nodes_before_simplification
            || removed_boundary_nodes > self.selected_boundary_nodes_before_simplification
            || accounted_boundary_nodes > self.selected_boundary_nodes_before_simplification
            || self.oriented_selected_faces_before_simplification
                > self.selected_faces_before_simplification
            || self.reversed_selected_faces_before_simplification
                > self.oriented_selected_faces_before_simplification
            || oriented_selected_faces != self.oriented_selected_faces_before_simplification
            || removed_selected_faces > self.selected_faces_before_simplification
            || accounted_selected_faces > self.selected_faces_before_simplification
        {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        for face in &self.faces {
            validate_simplified_face(face)?;
        }
        for pair in self.faces.windows(2) {
            let left = (
                pair[0].face.cell.carrier.side,
                pair[0].face.cell.carrier.face,
                pair[0].source_face,
            );
            let right = (
                pair[1].face.cell.carrier.side,
                pair[1].face.cell.carrier.face,
                pair[1].source_face,
            );
            if left > right {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
        }
        for (index, face) in self.faces.iter().enumerate() {
            for other in &self.faces[index + 1..] {
                if exact_point_loops_match(
                    &face.face.cell.boundary_points,
                    &other.face.cell.boundary_points,
                    false,
                )? || exact_point_loops_match(
                    &face.face.cell.boundary_points,
                    &other.face.cell.boundary_points,
                    true,
                )? {
                    return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                }
            }
        }
        Ok(())
    }

    /// Validate this simplified complex by replaying the full arrangement,
    /// label, selection, and simplification pipeline from source operands.
    #[cfg(test)]
    pub(crate) fn validate_against_sources(
        &self,
        left: &Mesh,
        right: &Mesh,
        mode: ExactRegularizationMode,
    ) -> Result<(), ExactArrangementBlocker> {
        self.validate()?;
        let arrangement =
            ExactArrangement3d::from_meshes_with_regularization_mode(left, right, mode)
                .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?;
        let replay = simplify_selected_cell_complex(
            select_arrangement_for_replay(arrangement, left, right, self.operation, mode)?,
            mode,
        )?;
        if self == &replay {
            return Ok(());
        }
        if self.topology_assembly_report.is_some() || self.region_ownership_report.is_some() {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        let mut replay_without_gate_reports = replay;
        replay_without_gate_reports.topology_assembly_report = None;
        replay_without_gate_reports.region_ownership_report = None;
        if self == &replay_without_gate_reports {
            Ok(())
        } else {
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        }
    }
}

fn validate_simplified_face(face: &ExactSimplifiedFaceCell) -> Result<(), ExactArrangementBlocker> {
    if face.face.cell.boundary.len() != face.face.cell.boundary_points.len()
        || face.face.cell.boundary.len() < 3
    {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    if face
        .face
        .cell
        .boundary
        .windows(2)
        .any(|pair| pair[0] == pair[1])
        || face.face.cell.boundary.first() == face.face.cell.boundary.last()
    {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    if !boundary_has_nonzero_area(&face.face.cell.boundary_points)? {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    let Some(first_key) = face
        .face
        .cell
        .boundary
        .first()
        .map(|node| format!("{node:?}"))
    else {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    };
    if face
        .face
        .cell
        .boundary
        .iter()
        .map(|node| format!("{node:?}"))
        .any(|key| key < first_key)
    {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Ok(())
}

/// Simplify a selected cell complex by exact canonicalization.
pub(crate) fn simplify_selected_cell_complex(
    selected: ExactSelectedCellComplex,
    mode: ExactRegularizationMode,
) -> Result<ExactSimplifiedCellComplex, ExactArrangementBlocker> {
    let gate_counts = selected_cell_complex_gate_counts(
        &selected.faces,
        &selected.volume_regions,
        &selected.volume_adjacencies,
        &selected.lower_dimensional_artifacts,
    );
    validate_selected_gate_reports(
        selected.topology_assembly_report.as_ref(),
        selected.region_ownership_report.as_ref(),
        selected.operation,
    )?;
    validate_selected_gate_reports_against_counts(
        selected.topology_assembly_report.as_ref(),
        selected.region_ownership_report.as_ref(),
        &gate_counts,
    )?;
    let selected_counts = selected.counts();
    let ExactSelectedCellComplex {
        faces: cell_faces,
        volume_regions: _,
        volume_adjacencies,
        lower_dimensional_artifacts,
        topology_assembly_report,
        region_ownership_report,
        selected_faces: selected_face_indices,
        selected_face_orientations,
        selected_volume_regions: _,
        operation,
        blockers,
    } = selected;
    let mut blockers = blockers;
    let mut faces = Vec::<ExactSimplifiedFaceCell>::new();
    let mut duplicate_cells_removed = 0;
    let mut duplicate_boundary_nodes_removed = 0;
    let mut collinear_boundary_nodes_removed = 0;
    let mut zero_area_cells_removed = 0;
    let mut interior_edges_removed = 0;
    let selected_faces_before_simplification = selected_counts.selected_faces;
    let mut selected_boundary_nodes_before_simplification = 0usize;
    for &source_face in &selected_face_indices {
        if let Some(face) = cell_faces.get(source_face) {
            let Some(next_count) =
                selected_boundary_nodes_before_simplification.checked_add(face.cell.boundary.len())
            else {
                blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
                continue;
            };
            selected_boundary_nodes_before_simplification = next_count;
        }
    }
    let oriented_selected_faces_before_simplification = selected_counts.oriented_selected_faces;
    let reversed_selected_faces_before_simplification = selected_counts.reversed_selected_faces;
    let volume_oriented_selected_faces_before_simplification =
        selected_counts.volume_oriented_selected_faces;
    let label_oriented_selected_faces_before_simplification =
        selected_counts.label_oriented_selected_faces;
    let require_volume_orientations =
        !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
            && !volume_adjacencies.is_empty();
    let mut volume_adjacency_faces = vec![false; cell_faces.len()];
    if require_volume_orientations {
        for adjacency in &volume_adjacencies {
            if validate_volume_adjacency_face_provenance(&cell_faces, adjacency).is_err() {
                blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
                continue;
            }
            for side in &adjacency.oriented_face_sides {
                match volume_adjacency_faces.get_mut(side.face_cell) {
                    Some(member) => *member = true,
                    None => blockers.push(ExactArrangementBlocker::NonManifoldCellComplex),
                }
            }
        }
    }
    let remove_collinear_nodes = matches!(operation, ExactBooleanOperation::SelectedRegions(_));
    let mut selected_face_set = vec![false; cell_faces.len()];
    for &face in &selected_face_indices {
        match selected_face_set.get_mut(face) {
            Some(member) if *member => {
                blockers.push(ExactArrangementBlocker::NonManifoldCellComplex)
            }
            Some(member) => *member = true,
            None => blockers.push(ExactArrangementBlocker::NonManifoldCellComplex),
        }
    }
    for orientation in &selected_face_orientations {
        if orientation.face >= cell_faces.len() {
            blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
            continue;
        }
        match selected_face_set.get(orientation.face).copied() {
            Some(true) => {}
            Some(false) | None => blockers.push(ExactArrangementBlocker::NonManifoldCellComplex),
        }
    }

    for source_face in selected_face_indices {
        let Some(mut face) = cell_faces.get(source_face).cloned() else {
            blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
            continue;
        };
        let Some(volume_adjacency_face) = volume_adjacency_faces.get(source_face).copied() else {
            blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
            continue;
        };
        let require_volume_orientation = require_volume_orientations && volume_adjacency_face;
        let reverse_orientation = selected_face_output_reversal(
            &selected_face_orientations,
            source_face,
            face.source,
            operation,
            require_volume_orientation,
        );
        match reverse_orientation {
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
        let duplicate_nodes_removed = remove_duplicate_boundary_nodes(&mut face);
        duplicate_boundary_nodes_removed += duplicate_nodes_removed;
        if remove_collinear_nodes {
            collinear_boundary_nodes_removed +=
                remove_collinear_boundary_nodes(&mut face, &mut blockers);
        }
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
        let mut duplicate = false;
        for existing in &faces {
            if existing.face == face {
                duplicate = true;
                break;
            }
            match exact_point_loops_match(
                &existing.face.cell.boundary_points,
                &face.cell.boundary_points,
                false,
            ) {
                Ok(true) => {
                    duplicate = true;
                    break;
                }
                Ok(false) => {}
                Err(blocker) => blockers.push(blocker),
            }
        }
        if duplicate {
            duplicate_cells_removed += 1;
            continue;
        }
        let mut opposite = None;
        for (index, existing) in faces.iter().enumerate() {
            match exact_point_loops_match(
                &existing.face.cell.boundary_points,
                &face.cell.boundary_points,
                true,
            ) {
                Ok(true) => {
                    opposite = Some(index);
                    break;
                }
                Ok(false) => {}
                Err(blocker) => blockers.push(blocker),
            }
        }
        if let Some(opposite) = opposite {
            faces.remove(opposite);
            duplicate_cells_removed += 2;
            continue;
        }
        faces.push(ExactSimplifiedFaceCell { source_face, face });
    }

    let merged = merge_same_label_adjacent_faces(faces, &mut blockers, remove_collinear_nodes);
    let mut faces = merged.faces;
    interior_edges_removed += merged.interior_edges_removed;
    collinear_boundary_nodes_removed += merged.collinear_boundary_nodes_removed;
    zero_area_cells_removed += merged.zero_area_cells_removed;
    let merged = merge_coplanar_face_pairs(
        faces,
        &mut blockers,
        remove_collinear_nodes,
        |left, right, blockers| {
            let left_label = (
                left.face.cell.carrier.side,
                region_label_key(left.face.source),
                opposite_label_key(left.face.opposite),
            );
            let right_label = (
                right.face.cell.carrier.side,
                region_label_key(right.face.source),
                opposite_label_key(right.face.opposite),
            );
            if left_label != right_label
                || left.face.cell.carrier.face == right.face.cell.carrier.face
            {
                return false;
            }
            match faces_share_reversed_exact_edge(left, right) {
                Ok(true) => face_boundaries_are_coplanar(
                    &left.face.cell.boundary_points,
                    &right.face.cell.boundary_points,
                    blockers,
                ),
                Ok(false) => false,
                Err(blocker) => {
                    blockers.push(blocker);
                    false
                }
            }
        },
    );
    faces = merged.faces;
    interior_edges_removed += merged.interior_edges_removed;
    collinear_boundary_nodes_removed += merged.collinear_boundary_nodes_removed;
    zero_area_cells_removed += merged.zero_area_cells_removed;
    if !matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
        let merged = merge_coplanar_face_pairs(
            faces,
            &mut blockers,
            remove_collinear_nodes,
            |left, right, blockers| match faces_share_reversed_exact_edge(left, right) {
                Ok(true) => face_boundaries_are_coplanar(
                    &left.face.cell.boundary_points,
                    &right.face.cell.boundary_points,
                    blockers,
                ),
                Ok(false) => false,
                Err(blocker) => {
                    blockers.push(blocker);
                    false
                }
            },
        );
        faces = merged.faces;
        interior_edges_removed += merged.interior_edges_removed;
        collinear_boundary_nodes_removed += merged.collinear_boundary_nodes_removed;
        zero_area_cells_removed += merged.zero_area_cells_removed;
    }

    faces.sort_by_key(|face| {
        (
            face.face.cell.carrier.side,
            face.face.cell.carrier.face,
            face.source_face,
        )
    });

    if !blockers.is_empty()
        && mode.unresolved == super::super::regularization::ExactUnresolvedMode::Block
    {
        return Err(blockers[0].clone());
    }

    Ok(ExactSimplifiedCellComplex {
        operation,
        faces,
        lower_dimensional_artifacts,
        topology_assembly_report,
        region_ownership_report,
        selected_faces_before_simplification,
        selected_boundary_nodes_before_simplification,
        oriented_selected_faces_before_simplification,
        reversed_selected_faces_before_simplification,
        volume_oriented_selected_faces_before_simplification,
        label_oriented_selected_faces_before_simplification,
        duplicate_cells_removed,
        duplicate_boundary_nodes_removed,
        collinear_boundary_nodes_removed,
        zero_area_cells_removed,
        interior_edges_removed,
        blockers,
    })
}

fn selected_face_output_reversal(
    orientations: &[ExactSelectedFaceOrientation],
    face: usize,
    source: ExactCellRegionLabel,
    operation: ExactBooleanOperation,
    require_volume_orientation: bool,
) -> Result<bool, ExactArrangementBlocker> {
    if let Some(reverse) = consistent_selected_face_reversal(
        orientations
            .iter()
            .filter(|orientation| orientation.face == face && orientation.from_volume_adjacency),
    )? {
        return Ok(reverse);
    }

    let Some(reverse) = consistent_selected_face_reversal(
        orientations
            .iter()
            .filter(|orientation| orientation.face == face),
    )?
    else {
        return if require_volume_orientation {
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        } else {
            Ok(operation == ExactBooleanOperation::Difference
                && source == ExactCellRegionLabel::RightBoundary)
        };
    };

    if require_volume_orientation {
        Err(ExactArrangementBlocker::UnresolvedRegionClassification)
    } else {
        Ok(reverse)
    }
}

fn consistent_selected_face_reversal<'a>(
    mut orientations: impl Iterator<Item = &'a ExactSelectedFaceOrientation>,
) -> Result<Option<bool>, ExactArrangementBlocker> {
    let Some(first) = orientations.next() else {
        return Ok(None);
    };
    for orientation in orientations {
        if orientation.reverse != first.reverse {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
    }
    Ok(Some(first.reverse))
}

fn remove_duplicate_boundary_nodes(face: &mut ExactCellComplexFace) -> usize {
    let mut duplicate_nodes_removed = 0;
    let mut canonical_boundary = Vec::new();
    let mut canonical_points = Vec::new();
    for (index, node) in face.cell.boundary.iter().enumerate() {
        if canonical_boundary.last() == Some(node) {
            duplicate_nodes_removed += 1;
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
        duplicate_nodes_removed += 1;
    }
    let boundary_points_match_original =
        face.cell.boundary_points.len() == canonical_boundary.len() + duplicate_nodes_removed;
    face.cell.boundary = canonical_boundary;
    if boundary_points_match_original {
        face.cell.boundary_points = canonical_points;
    }
    duplicate_nodes_removed
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

struct RetainedMergedFaceCounts {
    collinear_boundary_nodes_removed: usize,
    zero_area_cells_removed: usize,
}

fn merge_same_label_adjacent_faces(
    faces: Vec<ExactSimplifiedFaceCell>,
    blockers: &mut Vec<ExactArrangementBlocker>,
    remove_collinear_nodes: bool,
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
            Ok((merged, removed)) if removed > 0 => {
                interior_edges_removed += removed;
                let retained = retain_merged_faces(
                    &mut merged_faces,
                    merged,
                    blockers,
                    remove_collinear_nodes,
                );
                collinear_boundary_nodes_removed += retained.collinear_boundary_nodes_removed;
                zero_area_cells_removed += retained.zero_area_cells_removed;
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

fn simplified_group_key(face: &ExactSimplifiedFaceCell) -> (MeshSide, usize, usize, usize) {
    (
        face.face.cell.carrier.side,
        face.face.cell.carrier.face,
        region_label_key(face.face.source),
        opposite_label_key(face.face.opposite),
    )
}

fn merge_coplanar_face_pairs(
    mut faces: Vec<ExactSimplifiedFaceCell>,
    blockers: &mut Vec<ExactArrangementBlocker>,
    remove_collinear_nodes: bool,
    mut eligible: impl FnMut(
        &ExactSimplifiedFaceCell,
        &ExactSimplifiedFaceCell,
        &mut Vec<ExactArrangementBlocker>,
    ) -> bool,
) -> MergeSameLabelResult {
    let mut interior_edges_removed = 0;
    let mut collinear_boundary_nodes_removed = 0;
    let mut zero_area_cells_removed = 0;
    let mut changed = true;
    while changed {
        changed = false;
        'pairs: for left in 0..faces.len() {
            for right in (left + 1)..faces.len() {
                if !eligible(&faces[left], &faces[right], blockers) {
                    continue;
                }
                let pair = vec![faces[left].clone(), faces[right].clone()];
                match merge_same_label_group(pair) {
                    Ok((merged, removed)) if removed > 0 => {
                        faces.remove(right);
                        faces.remove(left);
                        interior_edges_removed += removed;
                        let retained = retain_merged_faces(
                            &mut faces,
                            merged,
                            blockers,
                            remove_collinear_nodes,
                        );
                        collinear_boundary_nodes_removed +=
                            retained.collinear_boundary_nodes_removed;
                        zero_area_cells_removed += retained.zero_area_cells_removed;
                        changed = true;
                        break 'pairs;
                    }
                    Ok(_) => {}
                    Err(blocker) => blockers.push(blocker),
                }
            }
        }
    }
    MergeSameLabelResult {
        faces,
        interior_edges_removed,
        collinear_boundary_nodes_removed,
        zero_area_cells_removed,
    }
}

fn retain_merged_faces(
    output: &mut Vec<ExactSimplifiedFaceCell>,
    merged: Vec<ExactSimplifiedFaceCell>,
    blockers: &mut Vec<ExactArrangementBlocker>,
    remove_collinear_nodes: bool,
) -> RetainedMergedFaceCounts {
    let mut collinear_boundary_nodes_removed = 0;
    let mut zero_area_cells_removed = 0;
    for mut face in merged {
        if remove_collinear_nodes {
            collinear_boundary_nodes_removed +=
                remove_collinear_boundary_nodes(&mut face.face, blockers);
        }
        match boundary_has_nonzero_area(&face.face.cell.boundary_points) {
            Ok(true) => {
                canonicalize_boundary_start(&mut face.face);
                output.push(face);
            }
            Ok(false) => zero_area_cells_removed += 1,
            Err(blocker) => blockers.push(blocker),
        }
    }
    RetainedMergedFaceCounts {
        collinear_boundary_nodes_removed,
        zero_area_cells_removed,
    }
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
            let mut reverse = None;
            for (index, existing) in boundary_edges.iter().enumerate() {
                if exact_edges_are_reversed(existing, &edge)? {
                    reverse = Some(index);
                    break;
                }
            }
            if let Some(reverse) = reverse {
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
        while current != start {
            match point3_exact_equal(&current_point, &start_point) {
                Some(true) => break,
                Some(false) => {}
                None => return Err(ExactArrangementBlocker::UndecidableOrdering),
            }
            guard += 1;
            if guard > max_steps {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
            let mut next_index = None;
            for (index, edge) in boundary_edges.iter().enumerate() {
                if edge.from == current {
                    next_index = Some(index);
                    break;
                }
                match point3_exact_equal(&edge.from_point, &current_point) {
                    Some(true) => {
                        next_index = Some(index);
                        break;
                    }
                    Some(false) => {}
                    None => return Err(ExactArrangementBlocker::UndecidableOrdering),
                }
            }
            let Some(next_index) = next_index else {
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

fn exact_edges_are_reversed(
    left: &DirectedBoundaryEdge,
    right: &DirectedBoundaryEdge,
) -> Result<bool, ExactArrangementBlocker> {
    if left.from == right.to && left.to == right.from {
        return Ok(true);
    }
    match (
        point3_exact_equal(&left.from_point, &right.to_point),
        point3_exact_equal(&left.to_point, &right.from_point),
    ) {
        (Some(true), Some(true)) => Ok(true),
        (Some(false), _) | (_, Some(false)) => Ok(false),
        _ => Err(ExactArrangementBlocker::UndecidableOrdering),
    }
}

fn faces_share_reversed_exact_edge(
    left: &ExactSimplifiedFaceCell,
    right: &ExactSimplifiedFaceCell,
) -> Result<bool, ExactArrangementBlocker> {
    let mut left_edges = Vec::new();
    let mut right_edges = Vec::new();
    for (face, edges) in [(left, &mut left_edges), (right, &mut right_edges)] {
        if face.face.cell.boundary.len() != face.face.cell.boundary_points.len()
            || face.face.cell.boundary.len() < 2
        {
            continue;
        }
        for index in 0..face.face.cell.boundary.len() {
            let next = (index + 1) % face.face.cell.boundary.len();
            edges.push(DirectedBoundaryEdge {
                from: face.face.cell.boundary[index].clone(),
                to: face.face.cell.boundary[next].clone(),
                from_point: face.face.cell.boundary_points[index].clone(),
                to_point: face.face.cell.boundary_points[next].clone(),
            });
        }
    }
    for left_edge in &left_edges {
        for right_edge in &right_edges {
            if exact_edges_are_reversed(left_edge, right_edge)? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn face_boundaries_are_coplanar(
    left: &[Point3],
    right: &[Point3],
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> bool {
    let witness = 'witness: {
        for projection in [
            CoplanarProjection::Xy,
            CoplanarProjection::Xz,
            CoplanarProjection::Yz,
        ] {
            for first in 0..left.len() {
                for second in first + 1..left.len() {
                    for third in second + 1..left.len() {
                        let a = project_point3(&left[first], projection);
                        let b = project_point3(&left[second], projection);
                        let c = project_point3(&left[third], projection);
                        match orient2d_report(&a, &b, &c).value() {
                            Some(Sign::Positive | Sign::Negative) => {
                                break 'witness Some([
                                    left[first].clone(),
                                    left[second].clone(),
                                    left[third].clone(),
                                ]);
                            }
                            Some(Sign::Zero) | None => {}
                        }
                    }
                }
            }
        }
        None
    };
    let Some([a, b, c]) = witness else {
        return false;
    };
    for point in left.iter().chain(right.iter()) {
        match orient3d_report(&a, &b, &c, point).value() {
            Some(Sign::Zero) => {}
            Some(Sign::Positive | Sign::Negative) => return false,
            None => {
                blockers.push(ExactArrangementBlocker::UndecidableOrdering);
                return false;
            }
        }
    }
    true
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

/// Triangulate selected cells into an exact mesh.
///
/// The retained boundary of each selected face-cell is projected through a
/// certified nonzero carrier-plane projection and triangulated by `hypertri`
/// over exact coordinates. No primitive-float tolerance is used.
pub(crate) fn triangulate_simplified_cell_complex(
    complex: &ExactSimplifiedCellComplex,
) -> Result<Mesh, ExactArrangementBlocker> {
    complex.validate()?;
    let mut vertices = Vec::<Point3>::new();
    let mut triangles = Vec::<Triangle>::new();

    if matches!(complex.operation, ExactBooleanOperation::SelectedRegions(_)) {
        let mut groups = std::collections::BTreeMap::<_, Vec<usize>>::new();
        for (index, face) in complex.faces.iter().enumerate() {
            groups
                .entry(simplified_group_key(face))
                .or_default()
                .push(index);
        }
        for face_indices in groups.values() {
            let mut boundaries = Vec::new();
            for &face_index in face_indices {
                let face = &complex.faces[face_index].face.cell;
                if face.boundary.len() != face.boundary_points.len() || face.boundary.len() < 3 {
                    return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                }
                boundaries.push(face.boundary_points.clone());
            }
            triangulate_exact_loop_group(&boundaries, &mut vertices, &mut triangles)?;
        }
    } else {
        let boundaries = complex
            .faces
            .iter()
            .map(|face| {
                if face.face.cell.boundary.len() != face.face.cell.boundary_points.len()
                    || face.face.cell.boundary.len() < 3
                {
                    Err(ExactArrangementBlocker::NonManifoldCellComplex)
                } else {
                    Ok(face.face.cell.boundary_points.clone())
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        let boundaries = refine_boundary_segments_with_collinear_points(boundaries)?;
        for group in group_exact_coplanar_loops(boundaries)? {
            triangulate_exact_loop_group(&group, &mut vertices, &mut triangles)?;
        }
    }

    if !matches!(complex.operation, ExactBooleanOperation::SelectedRegions(_)) {
        deduplicate_output_vertices(&mut vertices, &mut triangles)?;
        let original = std::mem::take(&mut triangles);
        triangles = split_triangles_at_collinear_vertices(&vertices, original)?;
    }
    orient_paired_triangle_edges(&mut triangles)
        .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    orient_paired_triangle_edges(&mut triangles)
        .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
    split_disconnected_vertex_fans(&mut vertices, &mut triangles);
    orient_paired_triangle_edges(&mut triangles)
        .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;

    Mesh::new_with_validation_mode_and_version(
        vertices,
        triangles,
        SourceProvenance::exact("exact simplified arrangement cell complex"),
        MeshValidationMode::ALLOW_BOUNDARY,
        1,
    )
    .map_err(|_| ExactArrangementBlocker::NonManifoldCellComplex)
}

fn exact_points_equal(left: &Point3, right: &Point3) -> Result<bool, ExactArrangementBlocker> {
    point3_exact_equal(left, right).ok_or(ExactArrangementBlocker::UndecidableOrdering)
}

fn deduplicate_output_vertices(
    vertices: &mut Vec<Point3>,
    triangles: &mut [Triangle],
) -> Result<(), ExactArrangementBlocker> {
    let mut unique = Vec::<Point3>::new();
    let mut remap = Vec::<usize>::with_capacity(vertices.len());
    for vertex in std::mem::take(vertices) {
        let mut existing = None;
        for (index, point) in unique.iter().enumerate() {
            if exact_points_equal(point, &vertex)? {
                existing = Some(index);
                break;
            }
        }
        if let Some(existing) = existing {
            remap.push(existing);
        } else {
            remap.push(unique.len());
            unique.push(vertex);
        }
    }
    for triangle in triangles {
        for vertex in &mut triangle.0 {
            if let Some(&mapped) = remap.get(*vertex) {
                *vertex = mapped;
            }
        }
    }
    *vertices = unique;
    Ok(())
}

fn split_triangles_at_collinear_vertices(
    vertices: &[Point3],
    triangles: Vec<Triangle>,
) -> Result<Vec<Triangle>, ExactArrangementBlocker> {
    let mut split = Vec::new();
    for triangle in triangles {
        let [a, b, c] = triangle.0;
        let mut boundary = Vec::new();
        for (start, end) in [(a, b), (b, c), (c, a)] {
            boundary.push(start);
            let mut interior = Vec::new();
            for (candidate, point) in vertices.iter().enumerate() {
                if candidate == start || candidate == end || interior.contains(&candidate) {
                    continue;
                }
                if exact_points_equal(point, &vertices[start])?
                    || exact_points_equal(point, &vertices[end])?
                {
                    continue;
                }
                match point_on_segment3(&vertices[start], &vertices[end], point).value() {
                    Some(true) => interior.push(candidate),
                    Some(false) => {}
                    None => return Err(ExactArrangementBlocker::UndecidableOrdering),
                }
            }
            let (axis, forward) = segment_order_axis(&vertices[start], &vertices[end])?;
            let mut ordered = Vec::<usize>::new();
            'interior: for vertex in interior {
                for ordered_index in 0..ordered.len() {
                    let precedes = match compare_reals(
                        point3_axis_value(&vertices[vertex], axis),
                        point3_axis_value(&vertices[ordered[ordered_index]], axis),
                    )
                    .value()
                    {
                        Some(Ordering::Less) => forward,
                        Some(Ordering::Greater) => !forward,
                        Some(Ordering::Equal) => false,
                        None => return Err(ExactArrangementBlocker::UndecidableOrdering),
                    };
                    if precedes {
                        ordered.insert(ordered_index, vertex);
                        continue 'interior;
                    }
                }
                ordered.push(vertex);
            }
            boundary.extend(ordered);
        }
        let mut deduped = Vec::<usize>::new();
        for vertex in boundary {
            if deduped.last().copied() != Some(vertex) {
                deduped.push(vertex);
            }
        }
        if deduped.len() > 1 && deduped.first() == deduped.last() {
            deduped.pop();
        }
        let boundary = deduped;
        if boundary.len() == 3 {
            split.push(triangle);
            continue;
        }
        for index in 1..boundary.len() - 1 {
            let candidate = Triangle([boundary[0], boundary[index], boundary[index + 1]]);
            let points = [
                vertices[candidate.0[0]].clone(),
                vertices[candidate.0[1]].clone(),
                vertices[candidate.0[2]].clone(),
            ];
            if boundary_has_nonzero_area(&points)? {
                split.push(candidate);
            }
        }
    }
    Ok(split)
}

fn split_disconnected_vertex_fans(vertices: &mut Vec<Point3>, triangles: &mut [Triangle]) {
    let original_vertex_count = vertices.len();
    for vertex in 0..original_vertex_count {
        let incident = triangles
            .iter()
            .enumerate()
            .filter_map(|(triangle, vertices)| vertices.0.contains(&vertex).then_some(triangle))
            .collect::<Vec<_>>();
        if incident.len() <= 1 {
            continue;
        }
        let mut adjacency = vec![Vec::<usize>::new(); incident.len()];
        let mut edge_uses = std::collections::BTreeMap::<usize, Vec<(usize, bool)>>::new();
        for (local_triangle, &triangle_index) in incident.iter().enumerate() {
            let triangle = triangles[triangle_index].0;
            for edge in 0..3 {
                let start = triangle[edge];
                let end = triangle[(edge + 1) % 3];
                if start == vertex {
                    edge_uses
                        .entry(end)
                        .or_default()
                        .push((local_triangle, true));
                } else if end == vertex {
                    edge_uses
                        .entry(start)
                        .or_default()
                        .push((local_triangle, false));
                }
            }
        }
        for uses in edge_uses.values() {
            if let [
                (left_triangle, left_forward),
                (right_triangle, right_forward),
            ] = uses.as_slice()
                && left_forward != right_forward
            {
                adjacency[*left_triangle].push(*right_triangle);
                adjacency[*right_triangle].push(*left_triangle);
            }
        }
        let components = connected_incident_triangle_components(&incident, &adjacency);
        if components.len() <= 1 {
            continue;
        }
        for component in components.into_iter().skip(1) {
            let clone_index = vertices.len();
            vertices.push(vertices[vertex].clone());
            for triangle in component {
                for triangle_vertex in &mut triangles[triangle].0 {
                    if *triangle_vertex == vertex {
                        *triangle_vertex = clone_index;
                    }
                }
            }
        }
    }
}

fn connected_incident_triangle_components(
    incident: &[usize],
    adjacency: &[Vec<usize>],
) -> Vec<Vec<usize>> {
    let mut components = Vec::<Vec<usize>>::new();
    let mut visited = vec![false; incident.len()];
    for start in 0..incident.len() {
        if visited[start] {
            continue;
        }
        visited[start] = true;
        let mut stack = vec![start];
        let mut component = Vec::<usize>::new();
        while let Some(local) = stack.pop() {
            component.push(incident[local]);
            for &neighbor in &adjacency[local] {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }
        components.push(component);
    }
    components
}

#[derive(Clone, Copy)]
enum Point3CoordinateAxis {
    X,
    Y,
    Z,
}

fn refine_boundary_segments_with_collinear_points(
    boundaries: Vec<Vec<Point3>>,
) -> Result<Vec<Vec<Point3>>, ExactArrangementBlocker> {
    let all_points = boundaries
        .iter()
        .flat_map(|boundary| boundary.iter().cloned())
        .collect::<Vec<_>>();
    let mut refined = Vec::with_capacity(boundaries.len());
    for boundary in boundaries {
        if boundary.len() < 3 {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        let mut refined_boundary = Vec::new();
        for index in 0..boundary.len() {
            let start = &boundary[index];
            let end = &boundary[(index + 1) % boundary.len()];
            refined_boundary.push(start.clone());
            let mut candidates = Vec::new();
            for point in &all_points {
                if exact_points_equal(point, start)? || exact_points_equal(point, end)? {
                    continue;
                }
                let mut duplicate = false;
                for candidate in &candidates {
                    if exact_points_equal(candidate, point)? {
                        duplicate = true;
                        break;
                    }
                }
                if duplicate {
                    continue;
                }
                match point_on_segment3(start, end, point).value() {
                    Some(true) => candidates.push(point.clone()),
                    Some(false) => {}
                    None => return Err(ExactArrangementBlocker::UndecidableOrdering),
                }
            }
            let (axis, forward) = segment_order_axis(start, end)?;
            let mut ordered = Vec::<Point3>::new();
            'candidates: for point in candidates {
                for index in 0..ordered.len() {
                    let precedes = match compare_reals(
                        point3_axis_value(&point, axis),
                        point3_axis_value(&ordered[index], axis),
                    )
                    .value()
                    {
                        Some(Ordering::Less) => forward,
                        Some(Ordering::Greater) => !forward,
                        Some(Ordering::Equal) => false,
                        None => return Err(ExactArrangementBlocker::UndecidableOrdering),
                    };
                    if precedes {
                        ordered.insert(index, point);
                        continue 'candidates;
                    }
                }
                ordered.push(point);
            }
            refined_boundary.extend(ordered);
        }
        let mut deduped = Vec::<Point3>::new();
        for point in refined_boundary {
            match deduped.last() {
                Some(last) if exact_points_equal(last, &point)? => {}
                _ => deduped.push(point),
            }
        }
        if deduped.len() > 1 {
            let first = deduped.first().expect("checked non-empty deduped boundary");
            let last = deduped.last().expect("checked non-empty deduped boundary");
            if exact_points_equal(first, last)? {
                deduped.pop();
            }
        }
        refined.push(deduped);
    }
    Ok(refined)
}

fn segment_order_axis(
    start: &Point3,
    end: &Point3,
) -> Result<(Point3CoordinateAxis, bool), ExactArrangementBlocker> {
    for axis in [
        Point3CoordinateAxis::X,
        Point3CoordinateAxis::Y,
        Point3CoordinateAxis::Z,
    ] {
        match compare_reals(point3_axis_value(start, axis), point3_axis_value(end, axis)).value() {
            Some(Ordering::Less) => return Ok((axis, true)),
            Some(Ordering::Greater) => return Ok((axis, false)),
            Some(Ordering::Equal) => {}
            None => return Err(ExactArrangementBlocker::UndecidableOrdering),
        }
    }
    Err(ExactArrangementBlocker::NonManifoldCellComplex)
}

fn point3_axis_value(point: &Point3, axis: Point3CoordinateAxis) -> &Real {
    match axis {
        Point3CoordinateAxis::X => &point.x,
        Point3CoordinateAxis::Y => &point.y,
        Point3CoordinateAxis::Z => &point.z,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::arrangement3d::cell_complex::{
        ExactCellComplexFace, ExactCellRegionLabel, ExactOppositeRegionLabel,
        ExactSelectedCellComplex, ExactSelectedFaceOrientation,
    };
    use crate::mesh::arrangement3d::loop_triangulation::{
        emitted_triangle_orientation, projected_loop_interior_witness,
    };
    use crate::mesh::arrangement3d::{
        ArrangementFaceCarrier, ArrangementFaceCell, ArrangementFaceCellNode,
        ArrangementVolumeAdjacency, ArrangementVolumeFaceSide,
    };
    use crate::mesh::graph::MeshSide;
    use hyperlimit::{Point2, RingPointLocation, classify_point_ring_even_odd};

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn source_node_on(side: MeshSide, vertex: usize) -> ArrangementFaceCellNode {
        ArrangementFaceCellNode::Source { side, vertex }
    }

    fn selected_face(
        source_face: usize,
        vertices: &[usize],
        points: &[Point3],
    ) -> ExactCellComplexFace {
        selected_face_with_carrier(
            MeshSide::Left,
            ExactCellRegionLabel::LeftBoundary,
            source_face,
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
        selected_face_with_carrier(side, source, 0, vertices, points)
    }

    fn selected_face_with_carrier(
        side: MeshSide,
        source: ExactCellRegionLabel,
        carrier_face: usize,
        vertices: &[usize],
        points: &[Point3],
    ) -> ExactCellComplexFace {
        ExactCellComplexFace {
            cell: ArrangementFaceCell {
                carrier: ArrangementFaceCarrier {
                    side,
                    face: carrier_face,
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

    fn dummy_volume_adjacency_for(
        face_cell: usize,
        side: MeshSide,
        vertices: &[usize],
    ) -> ArrangementVolumeAdjacency {
        ArrangementVolumeAdjacency {
            shell_region: 0,
            exterior_volume: 0,
            interior_volume: 1,
            separating_face_cells: vec![face_cell],
            oriented_face_sides: vec![ArrangementVolumeFaceSide {
                face_cell,
                source: side,
                source_face: 0,
                boundary: vertices
                    .iter()
                    .map(|vertex| source_node_on(side, *vertex))
                    .collect(),
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
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
            .unwrap()
    }

    #[test]
    fn simplification_removes_internal_edge_between_same_label_cells() {
        let simplified = simplified_square();
        simplified.validate().unwrap();
        assert_eq!(simplified.faces.len(), 1);
        assert_eq!(simplified.interior_edges_removed, 1);
        assert_eq!(simplified.faces[0].face.cell.boundary.len(), 4);
        let mesh = triangulate_simplified_cell_complex(&simplified).unwrap();
        assert_eq!(mesh.vertices().len(), 4);
        assert_eq!(mesh.triangles().len(), 2);
    }

    #[test]
    fn simplified_cell_complex_validate_rejects_mismatched_boundary_rows() {
        let mut simplified = simplified_square();
        simplified.validate().unwrap();
        simplified.faces[0].face.cell.boundary_points.pop();

        assert_eq!(
            simplified.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplified_cell_complex_validate_rejects_duplicate_retained_loop() {
        let mut simplified = simplified_square();
        simplified.validate().unwrap();
        simplified.faces.push(simplified.faces[0].clone());

        assert_eq!(
            simplified.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplified_cell_complex_validate_rejects_noncanonical_boundary_start() {
        let mut simplified = simplified_square();
        simplified.validate().unwrap();
        simplified.faces[0].face.cell.boundary.rotate_left(1);
        simplified.faces[0].face.cell.boundary_points.rotate_left(1);

        assert_eq!(
            simplified.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn triangulation_rejects_stale_simplified_report_counts() {
        let mut simplified = simplified_square();
        simplified.validate().unwrap();
        triangulate_simplified_cell_complex(&simplified).unwrap();
        simplified.selected_faces_before_simplification = 0;

        assert_eq!(
            triangulate_simplified_cell_complex(&simplified),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
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
            topology_assembly_report: None,
            region_ownership_report: None,
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
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(simplified.faces.len(), 1);
        assert_eq!(simplified.interior_edges_removed, 1);
        assert_eq!(simplified.selected_faces_before_simplification, 2);
        assert_eq!(simplified.selected_boundary_nodes_before_simplification, 6);
        assert_eq!(simplified.oriented_selected_faces_before_simplification, 2);
        assert_eq!(
            simplified.volume_oriented_selected_faces_before_simplification,
            0
        );
        assert_eq!(
            simplified.label_oriented_selected_faces_before_simplification,
            2
        );
        assert!(simplified.blockers.is_empty());
        assert_eq!(simplified.faces[0].face.cell.boundary_points.len(), 4);
        let mut stale_orientation_count = simplified.clone();
        stale_orientation_count.label_oriented_selected_faces_before_simplification += 1;
        assert_eq!(
            stale_orientation_count.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
        let mut stale_removed_count = simplified.clone();
        stale_removed_count.duplicate_cells_removed += 2;
        assert_eq!(
            stale_removed_count.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
        let mut stale_boundary_count = simplified.clone();
        stale_boundary_count.collinear_boundary_nodes_removed += 3;
        assert_eq!(
            stale_boundary_count.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
        let mut overflowing_boundary_minimum = simplified;
        overflowing_boundary_minimum.selected_faces_before_simplification = usize::MAX;
        overflowing_boundary_minimum.selected_boundary_nodes_before_simplification = usize::MAX;
        assert_eq!(
            overflowing_boundary_minimum.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_merges_coplanar_cells_across_source_carriers() {
        let left = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let right = [p(0, 1, 0), p(1, 0, 0), p(1, 1, 0)];
        let mut left_face = selected_face_with_source(
            MeshSide::Left,
            ExactCellRegionLabel::LeftBoundary,
            &[0, 1, 2],
            &left,
        );
        left_face.cell.carrier.face = 0;
        let mut right_face = selected_face_with_source(
            MeshSide::Left,
            ExactCellRegionLabel::LeftBoundary,
            &[3, 4, 5],
            &right,
        );
        right_face.cell.carrier.face = 1;
        let selected = ExactSelectedCellComplex {
            faces: vec![left_face, right_face],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(simplified.faces.len(), 1);
        assert_eq!(simplified.interior_edges_removed, 1);
        assert_eq!(simplified.faces[0].face.cell.boundary_points.len(), 4);
    }

    #[test]
    fn simplification_merges_selected_output_faces_across_source_labels() {
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
                    MeshSide::Right,
                    ExactCellRegionLabel::RightBoundary,
                    &[3, 4, 5],
                    &right,
                ),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(simplified.faces.len(), 1);
        assert_eq!(simplified.interior_edges_removed, 1);
        assert_eq!(simplified.faces[0].face.cell.boundary_points.len(), 4);
        let mesh = triangulate_simplified_cell_complex(&simplified).unwrap();
        assert_eq!(mesh.vertices().len(), 4);
        assert_eq!(mesh.triangles().len(), 2);
    }

    #[test]
    fn simplification_keeps_non_coplanar_adjacent_carriers_separate() {
        let floor = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let wall = [p(0, 1, 0), p(1, 0, 0), p(0, 1, 1)];
        let mut floor_face = selected_face_with_source(
            MeshSide::Left,
            ExactCellRegionLabel::LeftBoundary,
            &[0, 1, 2],
            &floor,
        );
        floor_face.cell.carrier.face = 0;
        let mut wall_face = selected_face_with_source(
            MeshSide::Left,
            ExactCellRegionLabel::LeftBoundary,
            &[3, 4, 5],
            &wall,
        );
        wall_face.cell.carrier.face = 1;
        let selected = ExactSelectedCellComplex {
            faces: vec![floor_face, wall_face],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(simplified.faces.len(), 2);
        assert_eq!(simplified.interior_edges_removed, 0);
        assert!(simplified.blockers.is_empty());
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
            topology_assembly_report: None,
            region_ownership_report: None,
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
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
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
            topology_assembly_report: None,
            region_ownership_report: None,
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
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
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
            topology_assembly_report: None,
            region_ownership_report: None,
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
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
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
            volume_adjacencies: vec![dummy_volume_adjacency_for(0, MeshSide::Right, &[0, 1, 2])],
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Difference,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID),
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
            volume_adjacencies: vec![dummy_volume_adjacency_for(0, MeshSide::Left, &[0, 1, 2])],
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
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
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
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
            volume_adjacencies: vec![dummy_volume_adjacency_for(0, MeshSide::Right, &[0, 1, 2])],
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
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
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
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
            volume_adjacencies: vec![dummy_volume_adjacency_for(0, MeshSide::Right, &[0, 1, 2])],
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
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
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
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
    fn simplification_prefers_volume_orientation_over_conflicting_source_evidence() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face_with_source(
                MeshSide::Right,
                ExactCellRegionLabel::RightBoundary,
                &[0, 1, 2],
                &points,
            )],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![dummy_volume_adjacency_for(0, MeshSide::Right, &[0, 1, 2])],
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
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

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(simplified.faces.len(), 1);
        assert!(simplified.blockers.is_empty(), "{:?}", simplified.blockers);
    }

    #[test]
    fn simplification_rejects_conflicting_volume_orientation_evidence() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face_with_source(
                MeshSide::Right,
                ExactCellRegionLabel::RightBoundary,
                &[0, 1, 2],
                &points,
            )],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![dummy_volume_adjacency_for(0, MeshSide::Right, &[0, 1, 2])],
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
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
                    from_volume_adjacency: true,
                },
            ],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Difference,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );
    }

    #[test]
    fn simplification_rejects_out_of_range_selected_face() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face(0, &[0, 1, 2], &points)],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_rejects_duplicate_selected_face() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face(0, &[0, 1, 2], &points)],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 0],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_rejects_orientation_for_unselected_face() {
        let left = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let right = [p(2, 0, 0), p(3, 0, 0), p(2, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face(0, &[0, 1, 2], &left),
                selected_face(1, &[3, 4, 5], &right),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0],
            selected_face_orientations: vec![ExactSelectedFaceOrientation {
                face: 1,
                reverse: false,
                from_volume_adjacency: false,
            }],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_rejects_out_of_range_volume_adjacency_face() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face(0, &[0, 1, 2], &points)],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![dummy_volume_adjacency_for(1, MeshSide::Right, &[0, 1, 2])],
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0],
            selected_face_orientations: vec![ExactSelectedFaceOrientation {
                face: 0,
                reverse: false,
                from_volume_adjacency: false,
            }],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_rejects_out_of_range_volume_adjacency_separating_face() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let mut adjacency = dummy_volume_adjacency_for(0, MeshSide::Right, &[0, 1, 2]);
        adjacency.separating_face_cells = vec![1];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face(0, &[0, 1, 2], &points)],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![adjacency],
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0],
            selected_face_orientations: vec![ExactSelectedFaceOrientation {
                face: 0,
                reverse: false,
                from_volume_adjacency: false,
            }],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_rejects_volume_side_missing_from_separating_faces() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let mut adjacency = dummy_volume_adjacency_for(0, MeshSide::Right, &[0, 1, 2]);
        adjacency.separating_face_cells.clear();
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face(0, &[0, 1, 2], &points)],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![adjacency],
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0],
            selected_face_orientations: vec![ExactSelectedFaceOrientation {
                face: 0,
                reverse: false,
                from_volume_adjacency: false,
            }],
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
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
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();
        let mesh = triangulate_simplified_cell_complex(&simplified).unwrap();

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
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();
        let mesh = triangulate_simplified_cell_complex(&simplified).unwrap();

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
                selected_face_with_carrier(
                    MeshSide::Left,
                    ExactCellRegionLabel::LeftBoundary,
                    0,
                    &[4, 5, 6, 7],
                    &hole,
                ),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![
                dummy_volume_adjacency_for(0, MeshSide::Left, &[0, 1, 2, 3]),
                dummy_volume_adjacency_for(1, MeshSide::Left, &[4, 5, 6, 7]),
            ],
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
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
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();
        let mesh = triangulate_simplified_cell_complex(&simplified).unwrap();

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
    fn triangulation_unions_overlapping_same_depth_loops_via_arrangement() {
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
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();

        let mesh = triangulate_simplified_cell_complex(&simplified).unwrap();
        assert!(!mesh.triangles().is_empty());
        let area = mesh_projected_area2(&mesh, CoplanarProjection::Xy);
        assert!(
            compare_reals(&area, &Real::from(40)).value() == Some(Ordering::Equal)
                || compare_reals(&area, &Real::from(-40)).value() == Some(Ordering::Equal),
            "{area:?}"
        );
    }

    #[test]
    fn triangulation_rejects_boundary_node_point_mismatch() {
        let mut simplified = simplified_square();
        simplified.faces[0].face.cell.boundary.pop();

        assert_eq!(
            triangulate_simplified_cell_complex(&simplified),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn triangulation_splits_disconnected_vertex_fans_across_face_groups() {
        let shared = p(0, 0, 0);
        let selected = ExactSelectedCellComplex {
            faces: vec![
                selected_face_with_carrier(
                    MeshSide::Left,
                    ExactCellRegionLabel::LeftBoundary,
                    0,
                    &[0, 1, 2],
                    &[shared.clone(), p(1, 0, 0), p(0, 1, 0)],
                ),
                selected_face_with_carrier(
                    MeshSide::Left,
                    ExactCellRegionLabel::LeftBoundary,
                    1,
                    &[0, 3, 4],
                    &[shared, p(0, 0, 1), p(0, 1, 1)],
                ),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::RETAIN_ARTIFACTS)
                .unwrap();
        let mesh = triangulate_simplified_cell_complex(&simplified).unwrap();

        assert_eq!(mesh.triangles().len(), 2);
        assert_eq!(mesh.vertices().len(), 6);
        assert_eq!(mesh.facts().mesh.non_manifold_vertices, 0);
    }

    #[test]
    fn triangulation_accepts_point_touching_same_depth_loops() {
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
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 1],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();

        let mesh = triangulate_simplified_cell_complex(&simplified).unwrap();
        assert_eq!(mesh.vertices().len(), 8);
        assert_eq!(mesh.triangles().len(), 4);
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
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: vec![0, 1, 2],
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Union,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::REGULARIZED_SOLID)
                .unwrap();
        let mesh = triangulate_simplified_cell_complex(&simplified).unwrap();

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
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: Vec::new(),
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Intersection,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::RETAIN_ARTIFACTS)
                .unwrap();

        assert_eq!(simplified.lower_dimensional_artifacts, vec![artifact]);
    }

    #[test]
    fn simplification_retains_lower_dimensional_edge_artifacts() {
        let artifact = ArrangementLowerDimensionalArtifact::EdgeContact {
            left_face: 0,
            right_face: 1,
            endpoints: [p(0, 0, 0), p(1, 0, 0)],
        };
        let selected = ExactSelectedCellComplex {
            faces: Vec::new(),
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: vec![artifact.clone()],
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces: Vec::new(),
            selected_face_orientations: Vec::new(),
            selected_volume_regions: Vec::new(),
            operation: ExactBooleanOperation::Intersection,
            blockers: Vec::new(),
        };

        let simplified =
            simplify_selected_cell_complex(selected, ExactRegularizationMode::RETAIN_ARTIFACTS)
                .unwrap();

        assert_eq!(simplified.lower_dimensional_artifacts, vec![artifact]);
    }

    #[test]
    fn simplification_validation_rejects_degenerate_lower_dimensional_edge_artifact() {
        let artifact = ArrangementLowerDimensionalArtifact::EdgeContact {
            left_face: 0,
            right_face: 1,
            endpoints: [p(0, 0, 0), p(0, 0, 0)],
        };
        let simplified = ExactSimplifiedCellComplex {
            operation: ExactBooleanOperation::Intersection,
            faces: Vec::new(),
            lower_dimensional_artifacts: vec![artifact],
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces_before_simplification: 0,
            selected_boundary_nodes_before_simplification: 0,
            oriented_selected_faces_before_simplification: 0,
            reversed_selected_faces_before_simplification: 0,
            volume_oriented_selected_faces_before_simplification: 0,
            label_oriented_selected_faces_before_simplification: 0,
            duplicate_cells_removed: 0,
            duplicate_boundary_nodes_removed: 0,
            collinear_boundary_nodes_removed: 0,
            zero_area_cells_removed: 0,
            interior_edges_removed: 0,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplified.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_validation_rejects_reversed_duplicate_lower_dimensional_edge_artifact() {
        let simplified = ExactSimplifiedCellComplex {
            operation: ExactBooleanOperation::Intersection,
            faces: Vec::new(),
            lower_dimensional_artifacts: vec![
                ArrangementLowerDimensionalArtifact::EdgeContact {
                    left_face: 0,
                    right_face: 1,
                    endpoints: [p(0, 0, 0), p(1, 0, 0)],
                },
                ArrangementLowerDimensionalArtifact::EdgeContact {
                    left_face: 0,
                    right_face: 1,
                    endpoints: [p(1, 0, 0), p(0, 0, 0)],
                },
            ],
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces_before_simplification: 0,
            selected_boundary_nodes_before_simplification: 0,
            oriented_selected_faces_before_simplification: 0,
            reversed_selected_faces_before_simplification: 0,
            volume_oriented_selected_faces_before_simplification: 0,
            label_oriented_selected_faces_before_simplification: 0,
            duplicate_cells_removed: 0,
            duplicate_boundary_nodes_removed: 0,
            collinear_boundary_nodes_removed: 0,
            zero_area_cells_removed: 0,
            interior_edges_removed: 0,
            blockers: Vec::new(),
        };

        assert_eq!(
            simplified.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
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

    fn mesh_projected_area2(mesh: &Mesh, projection: CoplanarProjection) -> Real {
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
