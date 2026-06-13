//! Exact canonicalization for selected cell complexes.
//!
//! Simplification here means deterministic exact topology cleanup: duplicate
//! selected cells are removed, duplicate boundary nodes are collapsed, and cell
//! order is normalized. It deliberately does not introduce epsilon repair or
//! approximate mesh export.

use std::cmp::Ordering;

use super::arrangement3d::{
    ArrangementFaceCellNode, ArrangementLowerDimensionalArtifact, ExactArrangement,
    ExactTopologyAssemblyReport, validate_lower_dimensional_artifacts,
};
use super::boolean::ExactBooleanOperation;
use super::cell_complex::{
    ExactCellComplexFace, ExactCellRegionLabel, ExactOppositeRegionLabel,
    ExactRegionOwnershipReport, ExactSelectedCellComplex, ExactSelectedFaceOrientation,
    select_arrangement_for_replay, selected_cell_complex_gate_counts,
    validate_selected_gate_reports, validate_selected_gate_reports_against_counts,
    validate_volume_adjacency_face_provenance,
};
use super::loop_triangulation::{choose_polygon_projection, triangulate_exact_loop_group};
use super::mesh::{ExactMesh, Triangle};
use super::regularization::{ExactArrangementBlocker, ExactRegularizationPolicy};
use super::validation::ValidationPolicy;
use super::view::{ApproximateMeshF64View, approximate_mesh_f64_view};
use hyperlimit::CoplanarProjection;
use hyperlimit::{ApproximationPolicy, SourceProvenance};
use hyperlimit::{
    Point3, Sign, compare_reals, orient2d_report, orient3d_report, point3_equal, project_point3,
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
    /// Topology assembly report consumed before the selected cells were simplified.
    pub topology_assembly_report: Option<ExactTopologyAssemblyReport>,
    /// Region ownership report consumed before the selected cells were simplified.
    pub region_ownership_report: Option<ExactRegionOwnershipReport>,
    /// Selected face count consumed before simplification merged or dissolved cells.
    pub selected_faces_before_simplification: usize,
    /// Boundary-node count across selected faces before simplification.
    pub selected_boundary_nodes_before_simplification: usize,
    /// Selected faces with explicit orientation evidence before simplification.
    pub oriented_selected_faces_before_simplification: usize,
    /// Selected oriented faces whose output orientation was reversed before simplification.
    pub reversed_selected_faces_before_simplification: usize,
    /// Selected oriented faces justified by volume adjacency evidence before simplification.
    pub volume_oriented_selected_faces_before_simplification: usize,
    /// Selected oriented faces justified by source-label operation rules before simplification.
    pub label_oriented_selected_faces_before_simplification: usize,
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

/// Freshness status for a retained simplified cell complex.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactSimplifiedCellComplexFreshness {
    /// The simplified complex replays exactly from the current source operands.
    Current,
    /// Rebuilding the arrangement from the source operands is currently blocked.
    SourceReplayBlocked,
    /// Arrangement construction replays, but labeling or selection is blocked.
    SelectionReplayBlocked,
    /// Selection replays, but exact simplification is currently blocked.
    SimplificationReplayBlocked,
    /// The source operands simplify, but the retained simplified complex no longer matches.
    StaleSimplifiedCells,
}

impl ExactSimplifiedCellComplex {
    /// Validate local simplified-cell consistency without replaying source meshes.
    pub fn validate(&self) -> Result<(), ExactArrangementBlocker> {
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
            let left = simplified_sort_key(&pair[0]);
            let right = simplified_sort_key(&pair[1]);
            if left > right {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
        }
        for (index, face) in self.faces.iter().enumerate() {
            if self.faces[index + 1..].iter().any(|other| {
                exact_boundary_loops_same_orientation(
                    &face.face.cell.boundary_points,
                    &other.face.cell.boundary_points,
                ) || exact_boundary_loops_opposite_orientation(
                    &face.face.cell.boundary_points,
                    &other.face.cell.boundary_points,
                )
            }) {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
        }
        Ok(())
    }

    /// Validate this simplified complex by replaying the full arrangement,
    /// label, selection, and simplification pipeline from source operands.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<(), ExactArrangementBlocker> {
        self.validate()?;
        let arrangement = ExactArrangement::from_meshes_with_policy(left, right, policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?;
        let replay =
            select_arrangement_for_replay(arrangement, left, right, self.operation, policy)?
                .simplify_exact_with_policy(policy)?;
        if simplified_cell_complex_matches_replay(self, &replay) {
            Ok(())
        } else {
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        }
    }

    pub(crate) fn validate_against_arrangement(
        &self,
        arrangement: ExactArrangement,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<(), ExactArrangementBlocker> {
        self.validate()?;
        let replay =
            select_arrangement_for_replay(arrangement, left, right, self.operation, policy)?
                .simplify_exact_with_policy(policy)?;
        if simplified_cell_complex_matches_replay(self, &replay) {
            Ok(())
        } else {
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        }
    }

    /// Classify whether this retained simplified complex is fresh for the source operands.
    pub fn freshness_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> ExactSimplifiedCellComplexFreshness {
        let arrangement = match ExactArrangement::from_meshes_with_policy(left, right, policy) {
            Ok(arrangement) => arrangement,
            Err(_) => return ExactSimplifiedCellComplexFreshness::SourceReplayBlocked,
        };
        self.freshness_against_arrangement(arrangement, left, right, policy)
    }

    pub(crate) fn freshness_against_arrangement(
        &self,
        arrangement: ExactArrangement,
        left: &ExactMesh,
        right: &ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> ExactSimplifiedCellComplexFreshness {
        if self.validate().is_err() {
            return ExactSimplifiedCellComplexFreshness::StaleSimplifiedCells;
        }
        let selected =
            match select_arrangement_for_replay(arrangement, left, right, self.operation, policy) {
                Ok(selected) => selected,
                Err(_) => return ExactSimplifiedCellComplexFreshness::SelectionReplayBlocked,
            };
        match selected.simplify_exact_with_policy(policy) {
            Ok(replay) if simplified_cell_complex_matches_replay(self, &replay) => {
                ExactSimplifiedCellComplexFreshness::Current
            }
            Ok(_) => ExactSimplifiedCellComplexFreshness::StaleSimplifiedCells,
            Err(_) => ExactSimplifiedCellComplexFreshness::SimplificationReplayBlocked,
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

fn simplified_sort_key(face: &ExactSimplifiedFaceCell) -> (usize, usize, usize) {
    (
        side_key(face.face.cell.carrier.side),
        face.face.cell.carrier.face,
        face.source_face,
    )
}

/// Simplify a selected cell complex by exact canonicalization.
pub fn simplify_selected_cell_complex(
    selected: ExactSelectedCellComplex,
    policy: ExactRegularizationPolicy,
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
    let mut blockers = selected.blockers;
    let mut faces = Vec::new();
    let mut duplicate_cells_removed = 0;
    let mut duplicate_boundary_nodes_removed = 0;
    let mut collinear_boundary_nodes_removed = 0;
    let mut zero_area_cells_removed = 0;
    let mut interior_edges_removed = 0;
    let selected_face_orientations = selected.selected_face_orientations.clone();
    let selected_faces_before_simplification = selected.selected_faces.len();
    let mut selected_boundary_nodes_before_simplification = 0usize;
    for &source_face in &selected.selected_faces {
        if let Some(face) = selected.faces.get(source_face) {
            let Some(next_count) =
                selected_boundary_nodes_before_simplification.checked_add(face.cell.boundary.len())
            else {
                blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
                continue;
            };
            selected_boundary_nodes_before_simplification = next_count;
        }
    }
    let oriented_selected_faces_before_simplification = selected_face_orientations.len();
    let reversed_selected_faces_before_simplification = selected_face_orientations
        .iter()
        .filter(|orientation| orientation.reverse)
        .count();
    let volume_oriented_selected_faces_before_simplification = selected_face_orientations
        .iter()
        .filter(|orientation| orientation.from_volume_adjacency)
        .count();
    let label_oriented_selected_faces_before_simplification =
        oriented_selected_faces_before_simplification
            .saturating_sub(volume_oriented_selected_faces_before_simplification);
    let require_volume_orientations = !matches!(
        selected.operation,
        ExactBooleanOperation::SelectedRegions(_)
    ) && !selected.volume_adjacencies.is_empty();
    let volume_adjacency_faces = volume_adjacency_face_membership(
        &selected.faces,
        &selected.volume_adjacencies,
        require_volume_orientations,
        &mut blockers,
    );
    let selected_face_set = retained_selected_face_membership(
        selected.faces.len(),
        &selected.selected_faces,
        &mut blockers,
    );
    validate_selected_face_orientations(
        selected.faces.len(),
        &selected_face_set,
        &selected_face_orientations,
        &mut blockers,
    );

    for source_face in selected.selected_faces {
        let Some(mut face) = selected.faces.get(source_face).cloned() else {
            blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
            continue;
        };
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
    let merged = merge_coplanar_same_label_faces_across_carriers(faces, &mut blockers);
    faces = merged.faces;
    interior_edges_removed += merged.interior_edges_removed;
    collinear_boundary_nodes_removed += merged.collinear_boundary_nodes_removed;
    zero_area_cells_removed += merged.zero_area_cells_removed;

    faces.sort_by_key(simplified_sort_key);

    if !blockers.is_empty()
        && policy.unresolved == super::regularization::ExactUnresolvedPolicy::Block
    {
        return Err(blockers[0].clone());
    }

    Ok(ExactSimplifiedCellComplex {
        operation: selected.operation,
        faces,
        lower_dimensional_artifacts: selected.lower_dimensional_artifacts,
        topology_assembly_report: selected.topology_assembly_report,
        region_ownership_report: selected.region_ownership_report,
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

fn simplified_cell_complex_matches_replay(
    retained: &ExactSimplifiedCellComplex,
    replay: &ExactSimplifiedCellComplex,
) -> bool {
    if retained == replay {
        return true;
    }
    if retained.topology_assembly_report.is_some() || retained.region_ownership_report.is_some() {
        return false;
    }
    let mut replay_without_gate_reports = replay.clone();
    replay_without_gate_reports.topology_assembly_report = None;
    replay_without_gate_reports.region_ownership_report = None;
    retained == &replay_without_gate_reports
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

fn simplified_label_key(face: &ExactSimplifiedFaceCell) -> (usize, usize, usize) {
    (
        side_key(face.face.cell.carrier.side),
        region_label_key(face.face.source),
        opposite_label_key(face.face.opposite),
    )
}

fn merge_coplanar_same_label_faces_across_carriers(
    mut faces: Vec<ExactSimplifiedFaceCell>,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> MergeSameLabelResult {
    let mut interior_edges_removed = 0;
    let mut collinear_boundary_nodes_removed = 0;
    let mut zero_area_cells_removed = 0;
    let mut changed = true;
    while changed {
        changed = false;
        'pairs: for left in 0..faces.len() {
            for right in (left + 1)..faces.len() {
                if simplified_label_key(&faces[left]) != simplified_label_key(&faces[right])
                    || faces[left].face.cell.carrier.face == faces[right].face.cell.carrier.face
                    || !faces_share_reversed_exact_edge(&faces[left], &faces[right])
                    || !face_boundaries_are_coplanar(
                        &faces[left].face.cell.boundary_points,
                        &faces[right].face.cell.boundary_points,
                        blockers,
                    )
                {
                    continue;
                }
                let pair = vec![faces[left].clone(), faces[right].clone()];
                match merge_same_label_group(pair) {
                    Ok((mut merged, removed)) if removed > 0 => {
                        let right_face = faces.remove(right);
                        let left_face = faces.remove(left);
                        let _ = (left_face, right_face);
                        interior_edges_removed += removed;
                        for mut face in merged.drain(..) {
                            collinear_boundary_nodes_removed +=
                                remove_collinear_boundary_nodes(&mut face.face, blockers);
                            match boundary_has_nonzero_area(&face.face.cell.boundary_points) {
                                Ok(true) => {
                                    canonicalize_boundary_start(&mut face.face);
                                    faces.push(face);
                                }
                                Ok(false) => zero_area_cells_removed += 1,
                                Err(blocker) => blockers.push(blocker),
                            }
                        }
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

fn faces_share_reversed_exact_edge(
    left: &ExactSimplifiedFaceCell,
    right: &ExactSimplifiedFaceCell,
) -> bool {
    face_boundary_edges(left).iter().any(|left_edge| {
        face_boundary_edges(right)
            .iter()
            .any(|right_edge| exact_edges_are_reversed(left_edge, right_edge))
    })
}

fn face_boundary_edges(face: &ExactSimplifiedFaceCell) -> Vec<DirectedBoundaryEdge> {
    let mut edges = Vec::new();
    if face.face.cell.boundary.len() != face.face.cell.boundary_points.len()
        || face.face.cell.boundary.len() < 2
    {
        return edges;
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
    edges
}

fn face_boundaries_are_coplanar(
    left: &[Point3],
    right: &[Point3],
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> bool {
    let Some([a, b, c]) = non_collinear_point_triple(left) else {
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

fn non_collinear_point_triple(points: &[Point3]) -> Option<[Point3; 3]> {
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        for first in 0..points.len() {
            for second in first + 1..points.len() {
                for third in second + 1..points.len() {
                    let a = project_point3(&points[first], projection);
                    let b = project_point3(&points[second], projection);
                    let c = project_point3(&points[third], projection);
                    match orient2d_report(&a, &b, &c).value() {
                        Some(Sign::Positive | Sign::Negative) => {
                            return Some([
                                points[first].clone(),
                                points[second].clone(),
                                points[third].clone(),
                            ]);
                        }
                        Some(Sign::Zero) | None => {}
                    }
                }
            }
        }
    }
    None
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
    let mut volume_matches = orientations
        .iter()
        .filter(|orientation| orientation.face == source_face && orientation.from_volume_adjacency);
    if let Some(first) = volume_matches.next() {
        let reverse = first.reverse;
        for orientation in volume_matches {
            if orientation.reverse != reverse {
                return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
            }
        }
        return Ok(reverse);
    }

    let mut label_matches = orientations
        .iter()
        .filter(|orientation| orientation.face == source_face);
    if let Some(first) = label_matches.next() {
        if require_volume_orientation {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let reverse = first.reverse;
        for orientation in label_matches {
            if orientation.reverse != reverse {
                return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
            }
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
    faces: &[ExactCellComplexFace],
    volume_adjacencies: &[super::arrangement3d::ArrangementVolumeAdjacency],
    enabled: bool,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Vec<bool> {
    let face_count = faces.len();
    let mut membership = vec![false; face_count];
    if !enabled {
        return membership;
    }
    for adjacency in volume_adjacencies {
        if validate_volume_adjacency_face_provenance(faces, adjacency).is_err() {
            blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
            continue;
        }
        for side in &adjacency.oriented_face_sides {
            match membership.get_mut(side.face_cell) {
                Some(member) => *member = true,
                None => blockers.push(ExactArrangementBlocker::NonManifoldCellComplex),
            }
        }
    }
    membership
}

fn retained_selected_face_membership(
    face_count: usize,
    selected_faces: &[usize],
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Vec<bool> {
    let mut membership = vec![false; face_count];
    for &face in selected_faces {
        match membership.get_mut(face) {
            Some(member) if *member => {
                blockers.push(ExactArrangementBlocker::NonManifoldCellComplex)
            }
            Some(member) => *member = true,
            None => blockers.push(ExactArrangementBlocker::NonManifoldCellComplex),
        }
    }
    membership
}

fn validate_selected_face_orientations(
    face_count: usize,
    selected_faces: &[bool],
    orientations: &[ExactSelectedFaceOrientation],
    blockers: &mut Vec<ExactArrangementBlocker>,
) {
    for orientation in orientations {
        if orientation.face >= face_count
            || !selected_faces
                .get(orientation.face)
                .copied()
                .unwrap_or(false)
        {
            blockers.push(ExactArrangementBlocker::NonManifoldCellComplex);
        }
    }
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
    complex.validate()?;
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

    orient_paired_triangle_edges(&mut triangles)?;
    remove_duplicate_triangle_vertex_sets(&mut triangles);
    orient_paired_triangle_edges(&mut triangles)?;
    split_disconnected_triangle_vertex_fans(&mut vertices, &mut triangles);
    orient_paired_triangle_edges(&mut triangles)?;

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
        let face = &complex.faces[face_index].face.cell;
        if face.boundary.len() != face.boundary_points.len() || face.boundary.len() < 3 {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        boundaries.push(face.boundary_points.clone());
    }
    triangulate_exact_loop_group(&boundaries, vertices, triangles)
}

#[derive(Clone, Copy)]
struct SimplifiedTriangleEdgeUse {
    triangle: usize,
    forward_with_key: bool,
}

#[derive(Clone, Copy)]
struct SimplifiedTriangleOrientationConstraint {
    triangle: usize,
    flip_relative_to_current: bool,
}

fn orient_paired_triangle_edges(
    triangles: &mut [Triangle],
) -> Result<usize, ExactArrangementBlocker> {
    let edge_uses = simplified_triangle_edge_uses(triangles);
    let mut adjacency =
        vec![Vec::<SimplifiedTriangleOrientationConstraint>::new(); triangles.len()];
    for uses in edge_uses.values() {
        let [left, right] = uses.as_slice() else {
            continue;
        };
        let same_direction = left.forward_with_key == right.forward_with_key;
        adjacency[left.triangle].push(SimplifiedTriangleOrientationConstraint {
            triangle: right.triangle,
            flip_relative_to_current: same_direction,
        });
        adjacency[right.triangle].push(SimplifiedTriangleOrientationConstraint {
            triangle: left.triangle,
            flip_relative_to_current: same_direction,
        });
    }

    let mut flips = vec![None; triangles.len()];
    for start in 0..triangles.len() {
        if flips[start].is_some() {
            continue;
        }
        flips[start] = Some(false);
        let mut stack = vec![start];
        while let Some(triangle) = stack.pop() {
            let current_flip =
                flips[triangle].ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
            for constraint in &adjacency[triangle] {
                let required = current_flip ^ constraint.flip_relative_to_current;
                match flips[constraint.triangle] {
                    Some(existing) if existing != required => {
                        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                    }
                    Some(_) => {}
                    None => {
                        flips[constraint.triangle] = Some(required);
                        stack.push(constraint.triangle);
                    }
                }
            }
        }
    }

    let mut flipped = 0;
    for (triangle, flip) in triangles.iter_mut().zip(flips) {
        if flip == Some(true) {
            triangle.0.swap(1, 2);
            flipped += 1;
        }
    }
    Ok(flipped)
}

fn simplified_triangle_edge_uses(
    triangles: &[Triangle],
) -> std::collections::BTreeMap<[usize; 2], Vec<SimplifiedTriangleEdgeUse>> {
    let mut edge_uses =
        std::collections::BTreeMap::<[usize; 2], Vec<SimplifiedTriangleEdgeUse>>::new();
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        for edge in [
            [triangle.0[0], triangle.0[1]],
            [triangle.0[1], triangle.0[2]],
            [triangle.0[2], triangle.0[0]],
        ] {
            let mut key = edge;
            key.sort_unstable();
            edge_uses
                .entry(key)
                .or_default()
                .push(SimplifiedTriangleEdgeUse {
                    triangle: triangle_index,
                    forward_with_key: edge == key,
                });
        }
    }
    edge_uses
}

fn remove_duplicate_triangle_vertex_sets(triangles: &mut Vec<Triangle>) -> usize {
    let original_len = triangles.len();
    let mut seen = std::collections::BTreeSet::<[usize; 3]>::new();
    triangles.retain(|triangle| {
        let mut key = triangle.0;
        key.sort_unstable();
        seen.insert(key)
    });
    original_len - triangles.len()
}

#[derive(Clone, Copy)]
struct VertexFanEdgeUse {
    local_triangle: usize,
    other: usize,
    forward_from_vertex: bool,
}

fn split_disconnected_triangle_vertex_fans(
    vertices: &mut Vec<Point3>,
    triangles: &mut [Triangle],
) -> usize {
    let original_vertex_count = vertices.len();
    let mut cloned_vertices = 0;
    for vertex in 0..original_vertex_count {
        let incident = incident_triangle_indices(triangles, vertex);
        if incident.len() <= 1 {
            continue;
        }
        let components = triangle_vertex_fan_components(triangles, vertex, &incident);
        if components.len() <= 1 {
            continue;
        }
        for component in components.into_iter().skip(1) {
            let clone_index = vertices.len();
            vertices.push(vertices[vertex].clone());
            for triangle in component {
                replace_triangle_vertex(&mut triangles[triangle], vertex, clone_index);
            }
            cloned_vertices += 1;
        }
    }
    cloned_vertices
}

fn incident_triangle_indices(triangles: &[Triangle], vertex: usize) -> Vec<usize> {
    triangles
        .iter()
        .enumerate()
        .filter_map(|(triangle, vertices)| vertices.0.contains(&vertex).then_some(triangle))
        .collect()
}

fn triangle_vertex_fan_components(
    triangles: &[Triangle],
    vertex: usize,
    incident: &[usize],
) -> Vec<Vec<usize>> {
    let mut parent = (0..incident.len()).collect::<Vec<_>>();
    let mut edge_uses = std::collections::BTreeMap::<usize, Vec<VertexFanEdgeUse>>::new();
    for (local_triangle, &triangle_index) in incident.iter().enumerate() {
        for use_ in vertex_fan_edge_uses(local_triangle, vertex, triangles[triangle_index].0) {
            edge_uses.entry(use_.other).or_default().push(use_);
        }
    }
    for uses in edge_uses.values() {
        if let [left, right] = uses.as_slice()
            && left.forward_from_vertex != right.forward_from_vertex
        {
            union_vertex_fan(&mut parent, left.local_triangle, right.local_triangle);
        }
    }
    let mut components = std::collections::BTreeMap::<usize, Vec<usize>>::new();
    for (local, &triangle) in incident.iter().enumerate() {
        let root = find_vertex_fan_root(&mut parent, local);
        components.entry(root).or_default().push(triangle);
    }
    components.into_values().collect()
}

fn vertex_fan_edge_uses(
    local_triangle: usize,
    vertex: usize,
    triangle: [usize; 3],
) -> Vec<VertexFanEdgeUse> {
    let mut uses = Vec::new();
    for edge in 0..3 {
        let start = triangle[edge];
        let end = triangle[(edge + 1) % 3];
        if start == vertex {
            uses.push(VertexFanEdgeUse {
                local_triangle,
                other: end,
                forward_from_vertex: true,
            });
        } else if end == vertex {
            uses.push(VertexFanEdgeUse {
                local_triangle,
                other: start,
                forward_from_vertex: false,
            });
        }
    }
    uses
}

fn union_vertex_fan(parent: &mut [usize], left: usize, right: usize) {
    let left_root = find_vertex_fan_root(parent, left);
    let right_root = find_vertex_fan_root(parent, right);
    if left_root != right_root {
        parent[right_root] = left_root;
    }
}

fn find_vertex_fan_root(parent: &mut [usize], index: usize) -> usize {
    if parent[index] != index {
        parent[index] = find_vertex_fan_root(parent, parent[index]);
    }
    parent[index]
}

fn replace_triangle_vertex(triangle: &mut Triangle, old: usize, new: usize) {
    for vertex in &mut triangle.0 {
        if *vertex == old {
            *vertex = new;
        }
    }
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

    fn dummy_volume_adjacency(face_cell: usize) -> ArrangementVolumeAdjacency {
        dummy_volume_adjacency_for(face_cell, MeshSide::Right, &[0, 1, 2])
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

        simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap()
    }

    #[test]
    fn simplification_removes_internal_edge_between_same_label_cells() {
        let simplified = simplified_square();
        simplified.validate().unwrap();
        assert_eq!(simplified.faces.len(), 1);
        assert_eq!(simplified.interior_edges_removed, 1);
        assert_eq!(simplified.faces[0].face.cell.boundary.len(), 4);
        let mesh = simplified.triangulate().unwrap();
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
        simplified.triangulate().unwrap();
        simplified.selected_faces_before_simplification = 0;

        assert_eq!(
            simplified.triangulate(),
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();

        assert_eq!(simplified.faces.len(), 1);
        assert_eq!(simplified.interior_edges_removed, 1);
        assert_eq!(simplified.faces[0].face.cell.boundary_points.len(), 4);
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
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
            topology_assembly_report: None,
            region_ownership_report: None,
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
            volume_adjacencies: vec![dummy_volume_adjacency(0)],
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
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
            volume_adjacencies: vec![dummy_volume_adjacency(0)],
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID),
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID),
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID),
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_rejects_out_of_range_volume_adjacency_face() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let selected = ExactSelectedCellComplex {
            faces: vec![selected_face(0, &[0, 1, 2], &points)],
            volume_regions: Vec::new(),
            volume_adjacencies: vec![dummy_volume_adjacency(1)],
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_rejects_out_of_range_volume_adjacency_separating_face() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let mut adjacency = dummy_volume_adjacency(0);
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn simplification_rejects_volume_side_missing_from_separating_faces() {
        let points = [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
        let mut adjacency = dummy_volume_adjacency(0);
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
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
            topology_assembly_report: None,
            region_ownership_report: None,
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
            topology_assembly_report: None,
            region_ownership_report: None,
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
            topology_assembly_report: None,
            region_ownership_report: None,
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
    fn triangulation_rejects_boundary_node_point_mismatch() {
        let mut simplified = simplified_square();
        simplified.faces[0].face.cell.boundary.pop();

        assert_eq!(
            simplified.triangulate(),
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::RETAIN_ARTIFACTS)
                .unwrap();
        let mesh = simplified.triangulate().unwrap();

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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::REGULARIZED_SOLID)
                .unwrap();

        let mesh = simplified.triangulate().unwrap();
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
            topology_assembly_report: None,
            region_ownership_report: None,
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
            simplify_selected_cell_complex(selected, ExactRegularizationPolicy::RETAIN_ARTIFACTS)
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
