//! Exact cell-complex selection over retained arrangements.
//!
//! The cell complex keeps arrangement face-cells as the primary topological
//! unit. Boolean operations are selection rules over labels; mesh
//! triangulation/export remains a later step with its own approximation or
//! triangulation policy.

use super::arrangement3d::{
    ArrangementFaceCell, ArrangementLowerDimensionalArtifact, ArrangementOppositeClassification,
    ArrangementVolumeAdjacency, ArrangementVolumeRegion, ExactArrangement, ExactArrangement3d,
    ExactTopologyAssemblyReport, exact_node_loops_equivalent, lower_dimensional_artifact_counts,
    sorted_unique_usize_set, validate_arrangement_face_cell, validate_lower_dimensional_artifacts,
};
use super::boolean::ExactBooleanOperation;
use super::graph::MeshSide;
use super::regularization::{
    ExactArrangementBlocker, ExactLowerDimensionalPolicy, ExactRegularizationPolicy,
    ExactUnresolvedPolicy,
};
use super::simplify::{ExactSimplifiedCellComplex, simplify_selected_cell_complex};
use super::solid::ConvexSolidPointRelation;

/// Region label for one arrangement face-cell.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactCellRegionLabel {
    /// Cell belongs to the boundary of the left source mesh.
    LeftBoundary,
    /// Cell belongs to the boundary of the right source mesh.
    RightBoundary,
}

/// Relation of a boundary cell to the opposite closed mesh.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactOppositeRegionLabel {
    /// Strictly inside the opposite mesh.
    Inside,
    /// Strictly outside the opposite mesh.
    Outside,
    /// On the opposite mesh boundary.
    Boundary,
    /// Classification is unresolved or the target is not closed.
    Unknown,
}

/// Retained cell-complex face-cell.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCellComplexFace {
    /// Arrangement face-cell payload.
    pub cell: ArrangementFaceCell,
    /// Source boundary label.
    pub source: ExactCellRegionLabel,
    /// Opposite-region label derived from exact winding evidence.
    pub opposite: ExactOppositeRegionLabel,
}

/// Labeled volume region induced by closed arrangement shells.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactCellComplexVolumeRegion {
    /// Volume-region index from the source arrangement graph.
    pub index: usize,
    /// Whether this region is the unbounded exterior.
    pub exterior: bool,
    /// Shell components bounding this volume.
    pub boundary_shells: Vec<usize>,
    /// Whether the volume is owned by the left source shell graph.
    pub in_left: bool,
    /// Whether the volume is owned by the right source shell graph.
    pub in_right: bool,
}

/// Exact cell complex built from a 3D arrangement.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCellComplex {
    /// Source arrangement.
    pub arrangement: ExactArrangement3d,
    /// Regularization policy used to build this view.
    pub policy: ExactRegularizationPolicy,
}

/// Labeled arrangement cells.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactLabeledCellComplex {
    /// Labeled face-cells.
    pub faces: Vec<ExactCellComplexFace>,
    /// Labeled volume-region graph nodes.
    pub volume_regions: Vec<ExactCellComplexVolumeRegion>,
    /// Volume-region adjacencies through oriented shell face-cells.
    pub volume_adjacencies: Vec<ArrangementVolumeAdjacency>,
    /// Retained lower-dimensional arrangement artifacts under policy.
    pub lower_dimensional_artifacts: Vec<ArrangementLowerDimensionalArtifact>,
    /// Blockers inherited or introduced during labeling.
    pub blockers: Vec<ExactArrangementBlocker>,
}

/// Selected cells for a Boolean operation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactSelectedCellComplex {
    /// Labeled face-cells.
    pub faces: Vec<ExactCellComplexFace>,
    /// Labeled volume-region graph nodes.
    pub volume_regions: Vec<ExactCellComplexVolumeRegion>,
    /// Volume-region adjacencies through oriented shell face-cells.
    pub volume_adjacencies: Vec<ArrangementVolumeAdjacency>,
    /// Retained lower-dimensional arrangement artifacts under policy.
    pub lower_dimensional_artifacts: Vec<ArrangementLowerDimensionalArtifact>,
    /// Topology assembly report consumed before this selection, when retained.
    pub topology_assembly_report: Option<ExactTopologyAssemblyReport>,
    /// Region ownership report consumed before this selection, when retained.
    pub region_ownership_report: Option<ExactRegionOwnershipReport>,
    /// Indices of selected `faces`.
    pub selected_faces: Vec<usize>,
    /// Per-selected-face orientation relative to the exported boundary.
    pub selected_face_orientations: Vec<ExactSelectedFaceOrientation>,
    /// Indices of selected `volume_regions`.
    pub selected_volume_regions: Vec<usize>,
    /// Boolean operation used for selection.
    pub operation: ExactBooleanOperation,
    /// Blockers inherited or introduced during selection.
    pub blockers: Vec<ExactArrangementBlocker>,
}

/// Orientation chosen for one selected face-cell.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExactSelectedFaceOrientation {
    /// Index into [`ExactSelectedCellComplex::faces`].
    pub face: usize,
    /// Whether the selected output boundary should reverse this face-cell.
    pub reverse: bool,
    /// Whether this orientation came from explicit volume adjacency.
    pub from_volume_adjacency: bool,
}

/// Freshness status for a retained labeled cell complex.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactLabeledCellComplexFreshness {
    /// The labeled complex replays exactly from the current source operands.
    Current,
    /// Rebuilding the arrangement from the source operands is currently blocked.
    SourceReplayBlocked,
    /// Arrangement construction replays, but region labeling is blocked.
    LabelingReplayBlocked,
    /// The source operands relabel, but the retained labeled complex no longer matches.
    StaleLabeledCells,
}

/// Freshness status for a retained selected cell complex.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactSelectedCellComplexFreshness {
    /// The selected complex replays exactly from the current source operands.
    Current,
    /// Rebuilding the arrangement from the source operands is currently blocked.
    SourceReplayBlocked,
    /// Arrangement construction replays, but labeling or selection is blocked.
    SelectionReplayBlocked,
    /// The source operands select, but the retained selected complex no longer matches.
    StaleSelectedCells,
}

/// Region-ownership readiness for a retained labeled cell complex.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactRegionOwnershipStatus {
    /// Volume-region ownership resolves selection without per-face winding
    /// ambiguity.
    VolumeResolved,
    /// Every retained face-cell has a known opposite-side label.
    FaceResolved,
    /// Exact winding or equivalent region ownership evidence is still needed.
    RequiresWinding,
    /// A non-ownership arrangement blocker prevents region selection.
    Blocked,
    /// Rebuilding the arrangement from source operands is currently blocked.
    SourceReplayBlocked,
    /// Arrangement construction replays, but region labeling is blocked.
    LabelingReplayBlocked,
    /// Source operands relabel, but the retained ownership report is stale.
    StaleOwnership,
}

impl ExactRegionOwnershipStatus {
    /// Return whether this ownership status can select named Boolean regions
    /// without additional winding evidence.
    pub const fn is_resolved(self) -> bool {
        matches!(self, Self::VolumeResolved | Self::FaceResolved)
    }

    /// Return whether retained volume-region ownership resolves selection.
    pub const fn is_volume_resolved(self) -> bool {
        matches!(self, Self::VolumeResolved)
    }
}

/// Compact exact ownership report for arrangement regions.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactRegionOwnershipReport {
    /// Overall ownership readiness.
    pub status: ExactRegionOwnershipStatus,
    /// Source replay freshness for the labeled cell complex.
    pub freshness: ExactLabeledCellComplexFreshness,
    /// Labeling blockers retained by the cell complex.
    pub blockers: Vec<ExactArrangementBlocker>,
    /// Retained face-cell count.
    pub face_cells: usize,
    /// Boundary nodes across retained face cells.
    pub face_cell_boundary_nodes: usize,
    /// Boundary coordinates across retained face cells.
    pub face_cell_boundary_points: usize,
    /// Face-cells carried by the left source boundary.
    pub left_boundary_faces: usize,
    /// Face-cells carried by the right source boundary.
    pub right_boundary_faces: usize,
    /// Face-cells classified inside the opposite source.
    pub opposite_inside_faces: usize,
    /// Face-cells classified outside the opposite source.
    pub opposite_outside_faces: usize,
    /// Face-cells classified on the opposite boundary.
    pub opposite_boundary_faces: usize,
    /// Face-cells whose opposite ownership is still unknown.
    pub opposite_unknown_faces: usize,
    /// Retained volume-region count.
    pub volume_regions: usize,
    /// Unbounded exterior volume-region count.
    pub exterior_volume_regions: usize,
    /// Volume regions owned by the left source.
    pub left_owned_volumes: usize,
    /// Volume regions owned by the right source.
    pub right_owned_volumes: usize,
    /// Volume regions owned by both sources.
    pub shared_owned_volumes: usize,
    /// Bounded volume regions not owned by either source.
    pub unowned_bounded_volumes: usize,
    /// Retained volume adjacencies through shell components.
    pub volume_adjacencies: usize,
    /// Oriented face-side witnesses carried by retained volume adjacencies.
    pub volume_adjacency_face_sides: usize,
    /// Separating face-cell references carried by retained volume adjacencies.
    pub volume_adjacency_separating_faces: usize,
    /// Whether retained volume adjacency evidence can drive every named
    /// Boolean selection without opposite-face winding labels.
    pub volume_selection_resolved: bool,
    /// Whether retained volume adjacency evidence can drive union selection.
    pub volume_union_resolved: bool,
    /// Whether retained volume adjacency evidence can drive intersection
    /// selection.
    pub volume_intersection_resolved: bool,
    /// Whether retained volume adjacency evidence can drive difference
    /// selection.
    pub volume_difference_resolved: bool,
    /// Retained lower-dimensional artifacts.
    pub lower_dimensional_artifacts: usize,
    /// Retained point-contact lower-dimensional artifacts.
    pub lower_dimensional_point_contacts: usize,
    /// Retained edge-contact lower-dimensional artifacts.
    pub lower_dimensional_edge_contacts: usize,
    /// Endpoints carried by retained edge-contact artifacts.
    pub lower_dimensional_edge_endpoints: usize,
}

impl ExactRegionOwnershipReport {
    /// Return whether retained exact evidence resolves region ownership.
    pub fn is_resolved(&self) -> bool {
        self.status.is_resolved()
    }

    /// Return whether retained volume-region evidence resolves this named
    /// operation even if other named operations still require winding.
    pub fn volume_selection_resolves_operation(&self, operation: ExactBooleanOperation) -> bool {
        match operation {
            ExactBooleanOperation::Union => self.volume_union_resolved,
            ExactBooleanOperation::Intersection => self.volume_intersection_resolved,
            ExactBooleanOperation::Difference => self.volume_difference_resolved,
            ExactBooleanOperation::SelectedRegions(_) => false,
        }
    }

    /// Return whether retained ownership evidence can select the requested
    /// operation without falling back to winding.
    pub fn resolves_operation_selection(&self, operation: ExactBooleanOperation) -> bool {
        self.is_resolved() || self.volume_selection_resolves_operation(operation)
    }

    /// Validate local ownership report shape without source replay.
    pub fn validate(&self) -> Result<(), ExactArrangementBlocker> {
        let expected_status = region_ownership_status(
            self.freshness,
            &self.blockers,
            self.face_cells,
            self.opposite_unknown_faces,
            self.volume_regions,
            self.volume_adjacencies,
            self.volume_selection_resolved,
        );
        if self.status != expected_status {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let all_named_volume_selections_resolved = self.volume_union_resolved
            && self.volume_intersection_resolved
            && self.volume_difference_resolved;
        let any_named_volume_selection_resolved = self.volume_union_resolved
            || self.volume_intersection_resolved
            || self.volume_difference_resolved;
        if self.volume_selection_resolved != all_named_volume_selections_resolved {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let Some(boundary_faces) = self
            .left_boundary_faces
            .checked_add(self.right_boundary_faces)
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        if self.face_cells != boundary_faces {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let Some(min_face_cell_boundary_nodes) = self.face_cells.checked_mul(3) else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        if self.face_cell_boundary_nodes != self.face_cell_boundary_points
            || (self.face_cells == 0 && self.face_cell_boundary_nodes != 0)
            || (self.face_cells != 0
                && self.face_cell_boundary_nodes < min_face_cell_boundary_nodes)
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let Some(opposite_classified_faces) = self
            .opposite_inside_faces
            .checked_add(self.opposite_outside_faces)
            .and_then(|count| count.checked_add(self.opposite_boundary_faces))
            .and_then(|count| count.checked_add(self.opposite_unknown_faces))
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        if self.face_cells != opposite_classified_faces {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let Some(bounded_volume_regions) = self
            .volume_regions
            .checked_sub(self.exterior_volume_regions)
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        let Some(owned_volume_union) = self
            .left_owned_volumes
            .checked_add(self.right_owned_volumes)
            .and_then(|count| count.checked_sub(self.shared_owned_volumes))
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        let Some(classified_bounded_volumes) =
            owned_volume_union.checked_add(self.unowned_bounded_volumes)
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        if self.left_owned_volumes > bounded_volume_regions
            || self.right_owned_volumes > bounded_volume_regions
            || self.shared_owned_volumes > self.left_owned_volumes
            || self.shared_owned_volumes > self.right_owned_volumes
            || self.shared_owned_volumes > bounded_volume_regions
            || self.unowned_bounded_volumes > bounded_volume_regions
            || classified_bounded_volumes > bounded_volume_regions
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        if (self.volume_adjacencies == 0
            && (self.volume_adjacency_face_sides != 0
                || self.volume_adjacency_separating_faces != 0
                || any_named_volume_selection_resolved))
            || (self.volume_adjacencies != 0
                && (self.volume_adjacency_face_sides == 0
                    || self.volume_adjacency_separating_faces == 0))
            || self.volume_adjacency_face_sides < self.volume_adjacencies
            || self.volume_adjacency_separating_faces < self.volume_adjacencies
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let Some(expected_edge_endpoints) = self.lower_dimensional_edge_contacts.checked_mul(2)
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        let Some(expected_lower_dimensional_artifacts) = self
            .lower_dimensional_point_contacts
            .checked_add(self.lower_dimensional_edge_contacts)
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        if self.lower_dimensional_artifacts != expected_lower_dimensional_artifacts
            || self.lower_dimensional_edge_endpoints != expected_edge_endpoints
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        match self.status {
            ExactRegionOwnershipStatus::VolumeResolved => {
                if self.volume_regions == 0
                    || self.volume_adjacencies == 0
                    || self.volume_adjacency_face_sides == 0
                    || self.volume_adjacency_separating_faces == 0
                    || !all_named_volume_selections_resolved
                {
                    return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
                }
            }
            ExactRegionOwnershipStatus::FaceResolved => {
                if self.opposite_unknown_faces != 0 {
                    return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
                }
            }
            ExactRegionOwnershipStatus::RequiresWinding => {
                if self.opposite_unknown_faces == 0
                    && !self
                        .blockers
                        .contains(&ExactArrangementBlocker::UnresolvedRegionClassification)
                {
                    return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
                }
            }
            ExactRegionOwnershipStatus::Blocked => {
                if !self.blockers.iter().any(|blocker| {
                    *blocker != ExactArrangementBlocker::UnresolvedRegionClassification
                }) {
                    return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
                }
            }
            ExactRegionOwnershipStatus::SourceReplayBlocked
            | ExactRegionOwnershipStatus::LabelingReplayBlocked
            | ExactRegionOwnershipStatus::StaleOwnership => {}
        }
        Ok(())
    }

    /// Validate this ownership report by replaying arrangement construction and
    /// region labeling from source operands.
    pub fn validate_against_sources(
        &self,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<(), ExactArrangementBlocker> {
        self.validate()?;
        let arrangement = ExactArrangement::from_meshes_with_policy(left, right, policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?;
        self.validate_against_arrangement(&arrangement, left, right, policy)
    }

    pub fn status_against_sources(
        &self,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> ExactRegionOwnershipStatus {
        let arrangement = match ExactArrangement::from_meshes_with_policy(left, right, policy) {
            Ok(arrangement) => arrangement,
            Err(_) => return ExactRegionOwnershipStatus::SourceReplayBlocked,
        };
        self.status_against_arrangement(&arrangement, left, right, policy)
    }

    pub(crate) fn validate_against_arrangement(
        &self,
        arrangement: &ExactArrangement,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<(), ExactArrangementBlocker> {
        self.validate()?;
        let replay = arrangement.region_ownership_report_with_policy(left, right, policy)?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        }
    }

    pub(crate) fn status_against_arrangement(
        &self,
        arrangement: &ExactArrangement,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> ExactRegionOwnershipStatus {
        if self.validate().is_err() {
            return ExactRegionOwnershipStatus::StaleOwnership;
        }
        match arrangement.region_ownership_report_with_policy(left, right, policy) {
            Ok(replay) if self == &replay => self.status,
            Ok(_) => ExactRegionOwnershipStatus::StaleOwnership,
            Err(_) => ExactRegionOwnershipStatus::LabelingReplayBlocked,
        }
    }
}

impl ExactCellComplex {
    /// Build a cell-complex view over an arrangement.
    pub fn from_arrangement(
        arrangement: ExactArrangement3d,
        policy: ExactRegularizationPolicy,
    ) -> Self {
        Self {
            arrangement,
            policy,
        }
    }

    /// Label arrangement face-cells by source boundary and opposite winding.
    pub fn label_regions(
        self,
        policy: ExactRegularizationPolicy,
    ) -> Result<ExactLabeledCellComplex, ExactArrangementBlocker> {
        let mut blockers = self.arrangement.blockers.clone();
        for blocker in self.arrangement.retained_volume_graph_blockers() {
            if !blockers.contains(&blocker) {
                blockers.push(blocker);
            }
        }
        let faces = self
            .arrangement
            .face_cells
            .iter()
            .cloned()
            .map(label_face_cell)
            .collect::<Vec<_>>();
        let volume_regions = self
            .arrangement
            .volume_regions
            .as_deref()
            .map(labeled_volume_regions_from_arrangement)
            .unwrap_or_default();
        let volume_adjacencies = self
            .arrangement
            .volume_adjacencies
            .clone()
            .unwrap_or_default();
        if !blockers.is_empty()
            && policy.unresolved == super::regularization::ExactUnresolvedPolicy::Block
        {
            return Err(blockers[0].clone());
        }
        Ok(ExactLabeledCellComplex {
            faces,
            volume_regions,
            volume_adjacencies,
            lower_dimensional_artifacts: self.arrangement.lower_dimensional_artifacts,
            blockers,
        })
    }
}

#[allow(dead_code)]
impl ExactLabeledCellComplex {
    /// Validate local labeled-cell consistency without replaying source meshes.
    pub fn validate(&self) -> Result<(), ExactArrangementBlocker> {
        validate_lower_dimensional_artifacts(&self.lower_dimensional_artifacts)?;
        validate_cell_complex_parts(&self.faces, &self.volume_regions, &self.volume_adjacencies)
    }

    /// Validate this labeled complex by replaying arrangement construction and
    /// region labeling from source operands.
    pub fn validate_against_sources(
        &self,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<(), ExactArrangementBlocker> {
        self.validate()?;
        let replay = ExactArrangement::from_meshes_with_policy(left, right, policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?
            .label_regions(policy)?;
        if replay == *self {
            Ok(())
        } else {
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        }
    }

    /// Classify whether this retained labeled complex is fresh for the source operands.
    pub fn freshness_against_sources(
        &self,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> ExactLabeledCellComplexFreshness {
        if self.validate().is_err() {
            return ExactLabeledCellComplexFreshness::StaleLabeledCells;
        }
        let arrangement = match ExactArrangement::from_meshes_with_policy(left, right, policy) {
            Ok(arrangement) => arrangement,
            Err(_) => return ExactLabeledCellComplexFreshness::SourceReplayBlocked,
        };
        match arrangement.label_regions(policy) {
            Ok(replay) if replay == *self => ExactLabeledCellComplexFreshness::Current,
            Ok(_) => ExactLabeledCellComplexFreshness::StaleLabeledCells,
            Err(_) => ExactLabeledCellComplexFreshness::LabelingReplayBlocked,
        }
    }

    /// Report whether retained exact evidence resolves region ownership.
    pub fn region_ownership_report(
        &self,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> ExactRegionOwnershipReport {
        let freshness = self.freshness_against_sources(left, right, policy);
        let left_boundary_faces = self
            .faces
            .iter()
            .filter(|face| face.source == ExactCellRegionLabel::LeftBoundary)
            .count();
        let right_boundary_faces = self
            .faces
            .iter()
            .filter(|face| face.source == ExactCellRegionLabel::RightBoundary)
            .count();
        let opposite_inside_faces = self
            .faces
            .iter()
            .filter(|face| face.opposite == ExactOppositeRegionLabel::Inside)
            .count();
        let opposite_outside_faces = self
            .faces
            .iter()
            .filter(|face| face.opposite == ExactOppositeRegionLabel::Outside)
            .count();
        let opposite_boundary_faces = self
            .faces
            .iter()
            .filter(|face| face.opposite == ExactOppositeRegionLabel::Boundary)
            .count();
        let opposite_unknown_faces = self
            .faces
            .iter()
            .filter(|face| face.opposite == ExactOppositeRegionLabel::Unknown)
            .count();
        let exterior_volume_regions = self
            .volume_regions
            .iter()
            .filter(|region| region.exterior)
            .count();
        let left_owned_volumes = self
            .volume_regions
            .iter()
            .filter(|region| region.in_left)
            .count();
        let right_owned_volumes = self
            .volume_regions
            .iter()
            .filter(|region| region.in_right)
            .count();
        let shared_owned_volumes = self
            .volume_regions
            .iter()
            .filter(|region| region.in_left && region.in_right)
            .count();
        let unowned_bounded_volumes = self
            .volume_regions
            .iter()
            .filter(|region| !region.exterior && !region.in_left && !region.in_right)
            .count();
        let volume_adjacency_face_sides = self
            .volume_adjacencies
            .iter()
            .map(|adjacency| adjacency.oriented_face_sides.len())
            .sum();
        let volume_adjacency_separating_faces = self
            .volume_adjacencies
            .iter()
            .map(|adjacency| adjacency.separating_face_cells.len())
            .sum();
        let volume_resolution = volume_selection_resolution(
            &self.faces,
            &self.volume_regions,
            &self.volume_adjacencies,
        );
        let (
            lower_dimensional_point_contacts,
            lower_dimensional_edge_contacts,
            lower_dimensional_edge_endpoints,
        ) = lower_dimensional_artifact_counts(&self.lower_dimensional_artifacts);
        let face_cell_boundary_nodes = self.faces.iter().map(|face| face.cell.boundary.len()).sum();
        let face_cell_boundary_points = self
            .faces
            .iter()
            .map(|face| face.cell.boundary_points.len())
            .sum();
        let status = region_ownership_status(
            freshness,
            &self.blockers,
            self.faces.len(),
            opposite_unknown_faces,
            self.volume_regions.len(),
            self.volume_adjacencies.len(),
            volume_resolution.all_named,
        );
        ExactRegionOwnershipReport {
            status,
            freshness,
            blockers: self.blockers.clone(),
            face_cells: self.faces.len(),
            face_cell_boundary_nodes,
            face_cell_boundary_points,
            left_boundary_faces,
            right_boundary_faces,
            opposite_inside_faces,
            opposite_outside_faces,
            opposite_boundary_faces,
            opposite_unknown_faces,
            volume_regions: self.volume_regions.len(),
            exterior_volume_regions,
            left_owned_volumes,
            right_owned_volumes,
            shared_owned_volumes,
            unowned_bounded_volumes,
            volume_adjacencies: self.volume_adjacencies.len(),
            volume_adjacency_face_sides,
            volume_adjacency_separating_faces,
            volume_selection_resolved: volume_resolution.all_named,
            volume_union_resolved: volume_resolution.union,
            volume_intersection_resolved: volume_resolution.intersection,
            volume_difference_resolved: volume_resolution.difference,
            lower_dimensional_artifacts: self.lower_dimensional_artifacts.len(),
            lower_dimensional_point_contacts,
            lower_dimensional_edge_contacts,
            lower_dimensional_edge_endpoints,
        }
    }

    /// Select face-cells for a named Boolean operation.
    pub fn select(
        self,
        operation: ExactBooleanOperation,
    ) -> Result<ExactSelectedCellComplex, ExactArrangementBlocker> {
        self.select_with_policy(operation, ExactRegularizationPolicy::default())
    }

    /// Select face-cells for a named Boolean operation with explicit policy.
    pub fn select_with_policy(
        self,
        operation: ExactBooleanOperation,
        policy: ExactRegularizationPolicy,
    ) -> Result<ExactSelectedCellComplex, ExactArrangementBlocker> {
        let mut blockers = self.blockers;
        if matches!(operation, ExactBooleanOperation::SelectedRegions(_)) {
            blockers.retain(|blocker| {
                *blocker != ExactArrangementBlocker::UnresolvedRegionClassification
            });
        }
        let selected_volume_regions = selected_volume_regions(&self.volume_regions, operation);
        let volume_selected = select_faces_from_volume_adjacencies(
            &self.faces,
            &self.volume_regions,
            &self.volume_adjacencies,
            operation,
        );
        let (selected_faces, selected_face_orientations) = if let Some(selected) =
            match volume_selected {
                Ok(selected) => selected,
                Err(blocker) => {
                    blockers.push(blocker);
                    None
                }
            } {
            blockers.retain(|blocker| {
                *blocker != ExactArrangementBlocker::UnresolvedRegionClassification
            });
            selected
        } else {
            let selected_faces =
                select_faces_from_face_labels(&self.faces, operation, policy, &mut blockers);
            let selected_face_orientations =
                selected_face_orientations_from_operation(&self.faces, &selected_faces, operation);
            (selected_faces, selected_face_orientations)
        };
        if !blockers.is_empty()
            && policy.unresolved == super::regularization::ExactUnresolvedPolicy::Block
        {
            return Err(blockers[0].clone());
        }
        Ok(ExactSelectedCellComplex {
            faces: self.faces,
            volume_regions: self.volume_regions,
            volume_adjacencies: self.volume_adjacencies,
            lower_dimensional_artifacts: self.lower_dimensional_artifacts,
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces,
            selected_face_orientations,
            selected_volume_regions,
            operation,
            blockers,
        })
    }

    pub(crate) fn select_volume_resolved_with_policy(
        self,
        operation: ExactBooleanOperation,
        policy: ExactRegularizationPolicy,
    ) -> Result<ExactSelectedCellComplex, ExactArrangementBlocker> {
        if self
            .blockers
            .iter()
            .any(|blocker| *blocker != ExactArrangementBlocker::UnresolvedRegionClassification)
        {
            return Err(self
                .blockers
                .first()
                .cloned()
                .unwrap_or(ExactArrangementBlocker::UnresolvedRegionClassification));
        }
        validate_lower_dimensional_artifacts(&self.lower_dimensional_artifacts)?;
        if !checked_volume_evidence_resolves_named_operation(
            &self.faces,
            &self.volume_regions,
            &self.volume_adjacencies,
            operation,
        )? {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        let Some((selected_faces, selected_face_orientations)) =
            checked_volume_resolved_face_selection(
                &self.faces,
                &self.volume_regions,
                &self.volume_adjacencies,
                operation,
            )?
        else {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        };
        if policy.lower_dimensional == ExactLowerDimensionalPolicy::ReportBlocker
            && selected_faces.iter().any(|&index| {
                self.faces
                    .get(index)
                    .is_some_and(|face| face.opposite == ExactOppositeRegionLabel::Boundary)
            })
        {
            return Err(ExactArrangementBlocker::LowerDimensionalContact);
        }
        let selected_volume_regions = selected_volume_regions(&self.volume_regions, operation);
        Ok(ExactSelectedCellComplex {
            faces: self.faces,
            volume_regions: self.volume_regions,
            volume_adjacencies: self.volume_adjacencies,
            lower_dimensional_artifacts: self.lower_dimensional_artifacts,
            topology_assembly_report: None,
            region_ownership_report: None,
            selected_faces,
            selected_face_orientations,
            selected_volume_regions,
            operation,
            blockers: Vec::new(),
        })
    }
}

#[allow(dead_code)]
impl ExactSelectedCellComplex {
    pub(crate) fn with_gate_reports(
        mut self,
        topology_assembly_report: ExactTopologyAssemblyReport,
        region_ownership_report: ExactRegionOwnershipReport,
    ) -> Self {
        self.topology_assembly_report = Some(topology_assembly_report);
        self.region_ownership_report = Some(region_ownership_report);
        self
    }

    /// Validate local selected-cell consistency without replaying source meshes.
    pub fn validate(&self) -> Result<(), ExactArrangementBlocker> {
        validate_lower_dimensional_artifacts(&self.lower_dimensional_artifacts)?;
        validate_cell_complex_parts(&self.faces, &self.volume_regions, &self.volume_adjacencies)?;
        let gate_counts = selected_cell_complex_gate_counts(
            &self.faces,
            &self.volume_regions,
            &self.volume_adjacencies,
            &self.lower_dimensional_artifacts,
        );
        validate_selected_gate_reports(
            self.topology_assembly_report.as_ref(),
            self.region_ownership_report.as_ref(),
            self.operation,
        )?;
        validate_selected_gate_reports_against_counts(
            self.topology_assembly_report.as_ref(),
            self.region_ownership_report.as_ref(),
            &gate_counts,
        )?;
        validate_selected_indices(&self.selected_faces, self.faces.len())?;
        validate_selected_indices(&self.selected_volume_regions, self.volume_regions.len())?;
        if self.selected_face_orientations.len() != self.selected_faces.len() {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        let orientation_faces = self
            .selected_face_orientations
            .iter()
            .map(|orientation| orientation.face)
            .collect::<Vec<_>>();
        if orientation_faces != self.selected_faces
            || sorted_unique_usize_set(&orientation_faces).is_none()
        {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        if self
            .selected_face_orientations
            .iter()
            .any(|orientation| orientation.face >= self.faces.len())
        {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        let expected_selected_volume_regions =
            selected_volume_regions(&self.volume_regions, self.operation);
        if self.selected_volume_regions != expected_selected_volume_regions {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        if self
            .selected_face_orientations
            .iter()
            .any(|orientation| orientation.from_volume_adjacency)
        {
            let Some((selected_faces, selected_face_orientations)) =
                select_faces_from_volume_adjacencies(
                    &self.faces,
                    &self.volume_regions,
                    &self.volume_adjacencies,
                    self.operation,
                )?
            else {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            };
            if self.selected_faces != selected_faces
                || self.selected_face_orientations != selected_face_orientations
            {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
        } else {
            for orientation in &self.selected_face_orientations {
                if orientation.reverse
                    != operation_reverses_face(&self.faces[orientation.face], self.operation)
                {
                    return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                }
            }
        }
        Ok(())
    }

    /// Validate this selected complex by replaying arrangement construction,
    /// labeling, and selection from source operands.
    pub fn validate_against_sources(
        &self,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<(), ExactArrangementBlocker> {
        self.validate()?;
        let arrangement = ExactArrangement::from_meshes_with_policy(left, right, policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?;
        let replay =
            select_arrangement_for_replay(arrangement, left, right, self.operation, policy)?;
        if selected_cell_complex_matches_replay(self, &replay) {
            Ok(())
        } else {
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        }
    }

    /// Classify whether this retained selected complex is fresh for the source operands.
    pub fn freshness_against_sources(
        &self,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> ExactSelectedCellComplexFreshness {
        let arrangement = match ExactArrangement::from_meshes_with_policy(left, right, policy) {
            Ok(arrangement) => arrangement,
            Err(_) => return ExactSelectedCellComplexFreshness::SourceReplayBlocked,
        };
        self.freshness_against_arrangement(arrangement, left, right, policy)
    }

    pub(crate) fn freshness_against_arrangement(
        &self,
        arrangement: ExactArrangement,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> ExactSelectedCellComplexFreshness {
        if self.validate().is_err() {
            return ExactSelectedCellComplexFreshness::StaleSelectedCells;
        }
        match select_arrangement_for_replay(arrangement, left, right, self.operation, policy) {
            Ok(replay) if selected_cell_complex_matches_replay(self, &replay) => {
                ExactSelectedCellComplexFreshness::Current
            }
            Ok(_) => ExactSelectedCellComplexFreshness::StaleSelectedCells,
            Err(_) => ExactSelectedCellComplexFreshness::SelectionReplayBlocked,
        }
    }

    /// Run exact canonicalization on selected cells.
    pub fn simplify_exact(self) -> Result<ExactSimplifiedCellComplex, ExactArrangementBlocker> {
        self.simplify_exact_with_policy(ExactRegularizationPolicy::default())
    }

    /// Run exact canonicalization on selected cells with explicit policy.
    pub fn simplify_exact_with_policy(
        self,
        policy: ExactRegularizationPolicy,
    ) -> Result<ExactSimplifiedCellComplex, ExactArrangementBlocker> {
        simplify_selected_cell_complex(self, policy)
    }
}

pub(crate) fn select_arrangement_for_replay(
    arrangement: ExactArrangement3d,
    left: &super::mesh::ExactMesh,
    right: &super::mesh::ExactMesh,
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
) -> Result<ExactSelectedCellComplex, ExactArrangementBlocker> {
    let topology_report = arrangement.topology_assembly_report_with_policy(left, right, policy);
    topology_report.validate()?;
    if !topology_report.is_complete() {
        if let Some(blocker) = topology_report
            .blockers
            .iter()
            .find(|blocker| **blocker != ExactArrangementBlocker::UnresolvedRegionClassification)
        {
            return Err(blocker.clone());
        }
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    let labeling_policy =
        arrangement_cell_complex_labeling_policy(&arrangement, Some(operation), policy);
    let labeled = arrangement.label_regions(labeling_policy)?;
    let ownership_report = labeled.region_ownership_report(left, right, labeling_policy);
    ownership_report.validate()?;
    let selected = if ownership_report.volume_selection_resolves_operation(operation) {
        labeled.select_volume_resolved_with_policy(operation, policy)
    } else {
        if !ownership_report.is_resolved()
            && !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
        labeled.select_with_policy(operation, policy)
    }?
    .with_gate_reports(topology_report, ownership_report);
    Ok(selected)
}

#[allow(dead_code)]
fn selected_cell_complex_matches_replay(
    retained: &ExactSelectedCellComplex,
    replay: &ExactSelectedCellComplex,
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

pub(crate) fn validate_selected_gate_reports(
    topology_assembly_report: Option<&ExactTopologyAssemblyReport>,
    region_ownership_report: Option<&ExactRegionOwnershipReport>,
    operation: ExactBooleanOperation,
) -> Result<(), ExactArrangementBlocker> {
    if let Some(topology_report) = topology_assembly_report {
        topology_report.validate()?;
        if !topology_report.is_complete() {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
    }
    if let Some(ownership_report) = region_ownership_report {
        ownership_report.validate()?;
        if topology_assembly_report.is_none() {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        if !ownership_report.resolves_operation_selection(operation)
            && !matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        {
            return Err(ExactArrangementBlocker::UnresolvedRegionClassification);
        }
    }
    Ok(())
}

pub(crate) fn validate_selected_gate_reports_against_counts(
    topology_assembly_report: Option<&ExactTopologyAssemblyReport>,
    region_ownership_report: Option<&ExactRegionOwnershipReport>,
    counts: &SelectedCellComplexGateCounts,
) -> Result<(), ExactArrangementBlocker> {
    if let Some(topology_report) = topology_assembly_report {
        if topology_report.arrangement_face_cells != counts.face_cells
            || topology_report.arrangement_face_cell_boundary_nodes
                != counts.face_cell_boundary_nodes
            || topology_report.arrangement_face_cell_boundary_points
                != counts.face_cell_boundary_points
            || topology_report.volume_regions != counts.volume_regions
            || topology_report.volume_adjacencies != counts.volume_adjacencies
            || topology_report.volume_adjacency_face_sides != counts.volume_adjacency_face_sides
            || topology_report.volume_adjacency_separating_faces
                != counts.volume_adjacency_separating_faces
            || topology_report.lower_dimensional_artifacts != counts.lower_dimensional_artifacts
            || topology_report.lower_dimensional_point_contacts
                != counts.lower_dimensional_point_contacts
            || topology_report.lower_dimensional_edge_contacts
                != counts.lower_dimensional_edge_contacts
            || topology_report.lower_dimensional_edge_endpoints
                != counts.lower_dimensional_edge_endpoints
        {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
    }
    if let Some(ownership_report) = region_ownership_report {
        if ownership_report.face_cells != counts.face_cells
            || ownership_report.face_cell_boundary_nodes != counts.face_cell_boundary_nodes
            || ownership_report.face_cell_boundary_points != counts.face_cell_boundary_points
            || ownership_report.volume_regions != counts.volume_regions
            || ownership_report.exterior_volume_regions != counts.exterior_volume_regions
            || ownership_report.left_owned_volumes != counts.left_owned_volumes
            || ownership_report.right_owned_volumes != counts.right_owned_volumes
            || ownership_report.shared_owned_volumes != counts.shared_owned_volumes
            || ownership_report.unowned_bounded_volumes != counts.unowned_bounded_volumes
            || ownership_report.volume_adjacencies != counts.volume_adjacencies
            || ownership_report.volume_adjacency_face_sides != counts.volume_adjacency_face_sides
            || ownership_report.volume_adjacency_separating_faces
                != counts.volume_adjacency_separating_faces
            || ownership_report.lower_dimensional_artifacts != counts.lower_dimensional_artifacts
            || ownership_report.lower_dimensional_point_contacts
                != counts.lower_dimensional_point_contacts
            || ownership_report.lower_dimensional_edge_contacts
                != counts.lower_dimensional_edge_contacts
            || ownership_report.lower_dimensional_edge_endpoints
                != counts.lower_dimensional_edge_endpoints
        {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
    }
    Ok(())
}

pub(crate) struct SelectedCellComplexGateCounts {
    face_cells: usize,
    face_cell_boundary_nodes: usize,
    face_cell_boundary_points: usize,
    volume_regions: usize,
    exterior_volume_regions: usize,
    left_owned_volumes: usize,
    right_owned_volumes: usize,
    shared_owned_volumes: usize,
    unowned_bounded_volumes: usize,
    volume_adjacencies: usize,
    volume_adjacency_face_sides: usize,
    volume_adjacency_separating_faces: usize,
    lower_dimensional_artifacts: usize,
    lower_dimensional_point_contacts: usize,
    lower_dimensional_edge_contacts: usize,
    lower_dimensional_edge_endpoints: usize,
}

pub(crate) fn selected_cell_complex_gate_counts(
    faces: &[ExactCellComplexFace],
    volume_regions: &[ExactCellComplexVolumeRegion],
    volume_adjacencies: &[ArrangementVolumeAdjacency],
    lower_dimensional_artifacts: &[ArrangementLowerDimensionalArtifact],
) -> SelectedCellComplexGateCounts {
    let face_cell_boundary_nodes = faces.iter().map(|face| face.cell.boundary.len()).sum();
    let face_cell_boundary_points = faces
        .iter()
        .map(|face| face.cell.boundary_points.len())
        .sum();
    let exterior_volume_regions = volume_regions
        .iter()
        .filter(|region| region.exterior)
        .count();
    let left_owned_volumes = volume_regions
        .iter()
        .filter(|region| !region.exterior && region.in_left)
        .count();
    let right_owned_volumes = volume_regions
        .iter()
        .filter(|region| !region.exterior && region.in_right)
        .count();
    let shared_owned_volumes = volume_regions
        .iter()
        .filter(|region| !region.exterior && region.in_left && region.in_right)
        .count();
    let unowned_bounded_volumes = volume_regions
        .iter()
        .filter(|region| !region.exterior && !region.in_left && !region.in_right)
        .count();
    let volume_adjacency_face_sides = volume_adjacencies
        .iter()
        .map(|adjacency| adjacency.oriented_face_sides.len())
        .sum();
    let volume_adjacency_separating_faces = volume_adjacencies
        .iter()
        .map(|adjacency| adjacency.separating_face_cells.len())
        .sum();
    let (
        lower_dimensional_point_contacts,
        lower_dimensional_edge_contacts,
        lower_dimensional_edge_endpoints,
    ) = lower_dimensional_artifact_counts(lower_dimensional_artifacts);
    SelectedCellComplexGateCounts {
        face_cells: faces.len(),
        face_cell_boundary_nodes,
        face_cell_boundary_points,
        volume_regions: volume_regions.len(),
        exterior_volume_regions,
        left_owned_volumes,
        right_owned_volumes,
        shared_owned_volumes,
        unowned_bounded_volumes,
        volume_adjacencies: volume_adjacencies.len(),
        volume_adjacency_face_sides,
        volume_adjacency_separating_faces,
        lower_dimensional_artifacts: lower_dimensional_artifacts.len(),
        lower_dimensional_point_contacts,
        lower_dimensional_edge_contacts,
        lower_dimensional_edge_endpoints,
    }
}

fn labeled_volume_regions_from_arrangement(
    volume_regions: &[ArrangementVolumeRegion],
) -> Vec<ExactCellComplexVolumeRegion> {
    volume_regions
        .iter()
        .map(|region| ExactCellComplexVolumeRegion {
            index: region.index,
            exterior: region.exterior,
            boundary_shells: region.boundary_shells.clone(),
            in_left: region.source_sides.contains(&MeshSide::Left),
            in_right: region.source_sides.contains(&MeshSide::Right),
        })
        .collect()
}

fn arrangement_volume_evidence(
    arrangement: &ExactArrangement3d,
) -> Option<(
    Vec<ExactCellComplexFace>,
    Vec<ExactCellComplexVolumeRegion>,
    &[ArrangementVolumeAdjacency],
)> {
    let faces = arrangement
        .face_cells
        .iter()
        .cloned()
        .map(label_face_cell)
        .collect::<Vec<_>>();
    let Some(volume_regions) = arrangement.volume_regions.as_deref() else {
        return None;
    };
    let Some(volume_adjacencies) = arrangement.volume_adjacencies.as_deref() else {
        return None;
    };
    let volume_regions = labeled_volume_regions_from_arrangement(volume_regions);
    Some((faces, volume_regions, volume_adjacencies))
}

fn arrangement_volume_evidence_resolves_named_selection(arrangement: &ExactArrangement3d) -> bool {
    if !arrangement.retained_volume_graph_blockers().is_empty() {
        return false;
    }
    let Some((faces, volume_regions, volume_adjacencies)) =
        arrangement_volume_evidence(arrangement)
    else {
        return false;
    };
    volume_evidence_resolves_named_selection(&faces, &volume_regions, volume_adjacencies)
}

fn arrangement_volume_evidence_resolves_named_operation(
    arrangement: &ExactArrangement3d,
    operation: ExactBooleanOperation,
) -> bool {
    if !arrangement.retained_volume_graph_blockers().is_empty() {
        return false;
    }
    let Some((faces, volume_regions, volume_adjacencies)) =
        arrangement_volume_evidence(arrangement)
    else {
        return false;
    };
    volume_evidence_resolves_named_operation(&faces, &volume_regions, volume_adjacencies, operation)
}

fn volume_evidence_resolves_named_selection(
    faces: &[ExactCellComplexFace],
    volume_regions: &[ExactCellComplexVolumeRegion],
    volume_adjacencies: &[ArrangementVolumeAdjacency],
) -> bool {
    volume_selection_resolution(faces, volume_regions, volume_adjacencies).all_named
}

fn volume_evidence_resolves_named_operation(
    faces: &[ExactCellComplexFace],
    volume_regions: &[ExactCellComplexVolumeRegion],
    volume_adjacencies: &[ArrangementVolumeAdjacency],
    operation: ExactBooleanOperation,
) -> bool {
    checked_volume_evidence_resolves_named_operation(
        faces,
        volume_regions,
        volume_adjacencies,
        operation,
    )
    .unwrap_or(false)
}

fn checked_volume_evidence_resolves_named_operation(
    faces: &[ExactCellComplexFace],
    volume_regions: &[ExactCellComplexVolumeRegion],
    volume_adjacencies: &[ArrangementVolumeAdjacency],
    operation: ExactBooleanOperation,
) -> Result<bool, ExactArrangementBlocker> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || volume_regions.is_empty()
        || volume_adjacencies.is_empty()
    {
        return Ok(false);
    }
    validate_cell_complex_parts(faces, volume_regions, volume_adjacencies)?;
    Ok(matches!(
        select_faces_from_volume_adjacencies(faces, volume_regions, volume_adjacencies, operation),
        Ok(Some(_))
    ))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ExactVolumeSelectionResolution {
    all_named: bool,
    union: bool,
    intersection: bool,
    difference: bool,
}

fn volume_selection_resolution(
    faces: &[ExactCellComplexFace],
    volume_regions: &[ExactCellComplexVolumeRegion],
    volume_adjacencies: &[ArrangementVolumeAdjacency],
) -> ExactVolumeSelectionResolution {
    let union = volume_evidence_resolves_named_operation(
        faces,
        volume_regions,
        volume_adjacencies,
        ExactBooleanOperation::Union,
    );
    let intersection = volume_evidence_resolves_named_operation(
        faces,
        volume_regions,
        volume_adjacencies,
        ExactBooleanOperation::Intersection,
    );
    let difference = volume_evidence_resolves_named_operation(
        faces,
        volume_regions,
        volume_adjacencies,
        ExactBooleanOperation::Difference,
    );
    ExactVolumeSelectionResolution {
        all_named: union && intersection && difference,
        union,
        intersection,
        difference,
    }
}

fn arrangement_has_only_region_classification_blockers(arrangement: &ExactArrangement3d) -> bool {
    !arrangement.blockers.is_empty()
        && arrangement
            .blockers
            .iter()
            .all(|blocker| *blocker == ExactArrangementBlocker::UnresolvedRegionClassification)
}

pub(crate) fn arrangement_region_classification_blockers_are_volume_resolved(
    arrangement: &ExactArrangement3d,
) -> bool {
    arrangement_has_only_region_classification_blockers(arrangement)
        && arrangement_volume_evidence_resolves_named_selection(arrangement)
}

pub(crate) fn arrangement_region_classification_blockers_resolve_operation(
    arrangement: &ExactArrangement3d,
    operation: ExactBooleanOperation,
) -> bool {
    arrangement_has_only_region_classification_blockers(arrangement)
        && arrangement_volume_evidence_resolves_named_operation(arrangement, operation)
}

pub(crate) fn arrangement_cell_complex_labeling_policy(
    arrangement: &ExactArrangement3d,
    operation: Option<ExactBooleanOperation>,
    policy: ExactRegularizationPolicy,
) -> ExactRegularizationPolicy {
    let volume_resolves_classification = match operation {
        Some(operation) => {
            arrangement_region_classification_blockers_resolve_operation(arrangement, operation)
        }
        None => arrangement_region_classification_blockers_are_volume_resolved(arrangement),
    };
    if volume_resolves_classification
        || operation.is_some_and(|operation| {
            matches!(operation, ExactBooleanOperation::SelectedRegions(_))
                && arrangement.blockers.iter().all(|blocker| {
                    *blocker == ExactArrangementBlocker::UnresolvedRegionClassification
                })
        })
    {
        ExactRegularizationPolicy {
            unresolved: ExactUnresolvedPolicy::RetainArtifacts,
            ..policy
        }
    } else {
        policy
    }
}

pub(crate) fn region_ownership_status(
    freshness: ExactLabeledCellComplexFreshness,
    blockers: &[ExactArrangementBlocker],
    face_cells: usize,
    opposite_unknown_faces: usize,
    volume_regions: usize,
    volume_adjacencies: usize,
    volume_selection_resolved: bool,
) -> ExactRegionOwnershipStatus {
    match freshness {
        ExactLabeledCellComplexFreshness::SourceReplayBlocked => {
            return ExactRegionOwnershipStatus::SourceReplayBlocked;
        }
        ExactLabeledCellComplexFreshness::LabelingReplayBlocked => {
            return ExactRegionOwnershipStatus::LabelingReplayBlocked;
        }
        ExactLabeledCellComplexFreshness::StaleLabeledCells => {
            return ExactRegionOwnershipStatus::StaleOwnership;
        }
        ExactLabeledCellComplexFreshness::Current => {}
    }
    if blockers
        .iter()
        .any(|blocker| *blocker != ExactArrangementBlocker::UnresolvedRegionClassification)
    {
        return ExactRegionOwnershipStatus::Blocked;
    }
    if volume_regions > 0 && volume_adjacencies > 0 && volume_selection_resolved {
        return ExactRegionOwnershipStatus::VolumeResolved;
    }
    if blockers.contains(&ExactArrangementBlocker::UnresolvedRegionClassification) {
        return ExactRegionOwnershipStatus::RequiresWinding;
    }
    if face_cells == 0 && blockers.is_empty() {
        return ExactRegionOwnershipStatus::FaceResolved;
    }
    if opposite_unknown_faces == 0 {
        ExactRegionOwnershipStatus::FaceResolved
    } else {
        ExactRegionOwnershipStatus::RequiresWinding
    }
}

fn label_face_cell(cell: ArrangementFaceCell) -> ExactCellComplexFace {
    let source = match cell.carrier.side {
        MeshSide::Left => ExactCellRegionLabel::LeftBoundary,
        MeshSide::Right => ExactCellRegionLabel::RightBoundary,
    };
    let opposite = cell
        .opposite
        .as_ref()
        .map(opposite_region_label)
        .unwrap_or(ExactOppositeRegionLabel::Unknown);
    ExactCellComplexFace {
        cell,
        source,
        opposite,
    }
}

fn opposite_region_label(opposite: &ArrangementOppositeClassification) -> ExactOppositeRegionLabel {
    match opposite.convex_certified_relation() {
        Some(ConvexSolidPointRelation::Inside) => return ExactOppositeRegionLabel::Inside,
        Some(ConvexSolidPointRelation::Outside) => return ExactOppositeRegionLabel::Outside,
        Some(ConvexSolidPointRelation::Boundary) => return ExactOppositeRegionLabel::Boundary,
        Some(ConvexSolidPointRelation::Unknown | ConvexSolidPointRelation::NotCertifiedConvex)
        | None => {}
    }
    match opposite.winding.relation {
        super::winding::ClosedMeshWindingRelation::Inside => ExactOppositeRegionLabel::Inside,
        super::winding::ClosedMeshWindingRelation::Outside => ExactOppositeRegionLabel::Outside,
        super::winding::ClosedMeshWindingRelation::Boundary => ExactOppositeRegionLabel::Boundary,
        super::winding::ClosedMeshWindingRelation::Unknown
        | super::winding::ClosedMeshWindingRelation::NotClosed => ExactOppositeRegionLabel::Unknown,
    }
}

fn select_face(
    face: &ExactCellComplexFace,
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
) -> Option<bool> {
    if let ExactBooleanOperation::SelectedRegions(selection) = operation {
        return Some(selection.keeps(match face.source {
            ExactCellRegionLabel::LeftBoundary => MeshSide::Left,
            ExactCellRegionLabel::RightBoundary => MeshSide::Right,
        }));
    }
    if face.opposite == ExactOppositeRegionLabel::Boundary {
        return select_boundary_face(face, operation, policy);
    }
    let inside = match face.opposite {
        ExactOppositeRegionLabel::Inside => true,
        ExactOppositeRegionLabel::Outside => false,
        ExactOppositeRegionLabel::Boundary => unreachable!("handled above"),
        ExactOppositeRegionLabel::Unknown => return None,
    };
    match operation {
        ExactBooleanOperation::Union => Some(!inside),
        ExactBooleanOperation::Intersection => Some(inside),
        ExactBooleanOperation::Difference => match face.source {
            ExactCellRegionLabel::LeftBoundary => Some(!inside),
            ExactCellRegionLabel::RightBoundary => Some(inside),
        },
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled above"),
    }
}

fn select_boundary_face(
    face: &ExactCellComplexFace,
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
) -> Option<bool> {
    if policy.lower_dimensional != ExactLowerDimensionalPolicy::Drop {
        return match operation {
            ExactBooleanOperation::Union => Some(false),
            ExactBooleanOperation::Intersection => Some(true),
            ExactBooleanOperation::Difference => match face.source {
                ExactCellRegionLabel::LeftBoundary => Some(false),
                ExactCellRegionLabel::RightBoundary => Some(true),
            },
            ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled above"),
        };
    }
    match operation {
        ExactBooleanOperation::Union | ExactBooleanOperation::Intersection => Some(false),
        ExactBooleanOperation::Difference => match face.source {
            ExactCellRegionLabel::LeftBoundary => Some(true),
            ExactCellRegionLabel::RightBoundary => Some(false),
        },
        ExactBooleanOperation::SelectedRegions(_) => unreachable!("handled above"),
    }
}

fn select_faces_from_face_labels(
    faces: &[ExactCellComplexFace],
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> Vec<usize> {
    let mut selected_faces = Vec::new();
    for (index, face) in faces.iter().enumerate() {
        if face.opposite == ExactOppositeRegionLabel::Boundary
            && policy.lower_dimensional == ExactLowerDimensionalPolicy::ReportBlocker
        {
            blockers.push(ExactArrangementBlocker::LowerDimensionalContact);
        }
        match select_face(face, operation, policy) {
            Some(true) => selected_faces.push(index),
            Some(false) => {}
            None => blockers.push(ExactArrangementBlocker::UnresolvedRegionClassification),
        }
    }
    selected_faces
}

fn select_faces_from_volume_adjacencies(
    faces: &[ExactCellComplexFace],
    volume_regions: &[ExactCellComplexVolumeRegion],
    volume_adjacencies: &[ArrangementVolumeAdjacency],
    operation: ExactBooleanOperation,
) -> Result<Option<(Vec<usize>, Vec<ExactSelectedFaceOrientation>)>, ExactArrangementBlocker> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || volume_regions.is_empty()
        || volume_adjacencies.is_empty()
    {
        return Ok(None);
    }
    validate_volume_regions_for_selection(volume_regions)?;
    let selected_volumes = volume_regions
        .iter()
        .map(|region| select_volume_region(region, operation))
        .collect::<Vec<_>>();
    let face_count = faces.len();
    let mut selected = Vec::<ExactSelectedFaceOrientation>::new();
    for adjacency in volume_adjacencies {
        validate_volume_adjacency_face_provenance(faces, adjacency)?;
        let exterior_selected = *selected_volumes
            .get(adjacency.exterior_volume)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
        let interior_selected = *selected_volumes
            .get(adjacency.interior_volume)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
        if exterior_selected == interior_selected {
            continue;
        }
        for &face_cell in &adjacency.separating_face_cells {
            if face_cell >= face_count {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
            if !adjacency
                .oriented_face_sides
                .iter()
                .any(|side| side.face_cell == face_cell)
                && !oriented_volume_side_covers_face_provenance(&faces[face_cell], adjacency)
            {
                continue;
            }
            let reverse = exterior_selected && !interior_selected;
            match selected
                .iter()
                .position(|orientation| orientation.face == face_cell)
            {
                Some(index) if selected[index].reverse != reverse => {
                    return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                }
                Some(_) => {}
                None => selected.push(ExactSelectedFaceOrientation {
                    face: face_cell,
                    reverse,
                    from_volume_adjacency: true,
                }),
            }
        }
    }
    selected.sort_by_key(|orientation| orientation.face);
    let selected_faces = selected
        .iter()
        .map(|orientation| orientation.face)
        .collect::<Vec<_>>();
    Ok(Some((selected_faces, selected)))
}

fn checked_volume_resolved_face_selection(
    faces: &[ExactCellComplexFace],
    volume_regions: &[ExactCellComplexVolumeRegion],
    volume_adjacencies: &[ArrangementVolumeAdjacency],
    operation: ExactBooleanOperation,
) -> Result<Option<(Vec<usize>, Vec<ExactSelectedFaceOrientation>)>, ExactArrangementBlocker> {
    if matches!(operation, ExactBooleanOperation::SelectedRegions(_))
        || volume_regions.is_empty()
        || volume_adjacencies.is_empty()
    {
        return Ok(None);
    }
    validate_cell_complex_parts(faces, volume_regions, volume_adjacencies)?;
    select_faces_from_volume_adjacencies(faces, volume_regions, volume_adjacencies, operation)
}

fn validate_cell_complex_parts(
    faces: &[ExactCellComplexFace],
    volume_regions: &[ExactCellComplexVolumeRegion],
    volume_adjacencies: &[ArrangementVolumeAdjacency],
) -> Result<(), ExactArrangementBlocker> {
    for face in faces {
        validate_arrangement_face_cell(&face.cell)?;
    }
    if !volume_regions.is_empty() {
        validate_volume_regions_for_selection(volume_regions)?;
    }
    for adjacency in volume_adjacencies {
        if adjacency.exterior_volume >= volume_regions.len()
            || adjacency.interior_volume >= volume_regions.len()
        {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        validate_volume_adjacency_face_provenance(faces, adjacency)?;
    }
    Ok(())
}

fn validate_selected_indices(
    selected: &[usize],
    upper_bound: usize,
) -> Result<(), ExactArrangementBlocker> {
    if selected.iter().any(|&index| index >= upper_bound)
        || sorted_unique_usize_set(selected).is_none()
    {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Ok(())
}

fn validate_volume_regions_for_selection(
    volume_regions: &[ExactCellComplexVolumeRegion],
) -> Result<(), ExactArrangementBlocker> {
    if volume_regions
        .iter()
        .enumerate()
        .any(|(index, region)| region.index != index)
        || volume_regions
            .iter()
            .filter(|region| region.exterior)
            .count()
            != 1
    {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Ok(())
}

pub(crate) fn validate_volume_adjacency_face_provenance(
    faces: &[ExactCellComplexFace],
    adjacency: &ArrangementVolumeAdjacency,
) -> Result<(), ExactArrangementBlocker> {
    let face_count = faces.len();
    if adjacency.exterior_volume == adjacency.interior_volume
        || adjacency.oriented_face_sides.is_empty()
        || adjacency
            .separating_face_cells
            .iter()
            .any(|&face| face >= face_count)
    {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    let mut side_faces = Vec::with_capacity(adjacency.oriented_face_sides.len());
    for side in &adjacency.oriented_face_sides {
        if side.exterior_volume != adjacency.exterior_volume
            || side.interior_volume != adjacency.interior_volume
            || side.face_cell >= face_count
        {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        let face = &faces[side.face_cell];
        if side.source != face.cell.carrier.side
            || side.source_face != face.cell.carrier.face
            || side.boundary != face.cell.boundary
        {
            return Err(ExactArrangementBlocker::NonManifoldCellComplex);
        }
        side_faces.push(side.face_cell);
    }
    side_faces.sort_unstable();
    side_faces.dedup();
    let Some(separating_face_cells) = sorted_unique_usize_set(&adjacency.separating_face_cells)
    else {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    };
    if side_faces
        .iter()
        .any(|face| separating_face_cells.binary_search(face).is_err())
        || separating_face_cells.iter().any(|&face| {
            side_faces.binary_search(&face).is_err()
                && !oriented_volume_side_covers_face_boundary(&faces[face], adjacency)
        })
    {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Ok(())
}

fn oriented_volume_side_covers_face_boundary(
    face: &ExactCellComplexFace,
    adjacency: &ArrangementVolumeAdjacency,
) -> bool {
    adjacency
        .oriented_face_sides
        .iter()
        .any(|side| exact_node_loops_equivalent(&face.cell.boundary, &side.boundary))
}

fn oriented_volume_side_covers_face_provenance(
    face: &ExactCellComplexFace,
    adjacency: &ArrangementVolumeAdjacency,
) -> bool {
    adjacency.oriented_face_sides.iter().any(|side| {
        side.source == face.cell.carrier.side
            && side.source_face == face.cell.carrier.face
            && exact_node_loops_equivalent(&face.cell.boundary, &side.boundary)
    })
}

fn selected_face_orientations_from_operation(
    faces: &[ExactCellComplexFace],
    selected_faces: &[usize],
    operation: ExactBooleanOperation,
) -> Vec<ExactSelectedFaceOrientation> {
    selected_faces
        .iter()
        .copied()
        .map(|face| ExactSelectedFaceOrientation {
            face,
            reverse: faces
                .get(face)
                .is_some_and(|cell| operation_reverses_face(cell, operation)),
            from_volume_adjacency: false,
        })
        .collect()
}

fn operation_reverses_face(face: &ExactCellComplexFace, operation: ExactBooleanOperation) -> bool {
    operation == ExactBooleanOperation::Difference
        && face.source == ExactCellRegionLabel::RightBoundary
}

fn selected_volume_regions(
    volume_regions: &[ExactCellComplexVolumeRegion],
    operation: ExactBooleanOperation,
) -> Vec<usize> {
    volume_regions
        .iter()
        .enumerate()
        .filter_map(|(index, volume)| select_volume_region(volume, operation).then_some(index))
        .collect()
}

fn select_volume_region(
    region: &ExactCellComplexVolumeRegion,
    operation: ExactBooleanOperation,
) -> bool {
    match operation {
        ExactBooleanOperation::Union => region.in_left || region.in_right,
        ExactBooleanOperation::Intersection => region.in_left && region.in_right,
        ExactBooleanOperation::Difference => region.in_left && !region.in_right,
        ExactBooleanOperation::SelectedRegions(selection) => {
            (region.in_left && selection.keeps(MeshSide::Left))
                || (region.in_right && selection.keeps(MeshSide::Right))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arrangement3d::{
        ArrangementFaceCarrier, ArrangementFaceCell, ArrangementFaceCellNode,
        ArrangementOppositeClassification, ArrangementVolumeFaceSide, ExactTopologyAssemblyStatus,
    };
    use crate::mesh::ExactMesh;
    use crate::region::ExactRegionSelection;
    use crate::solid::ConvexSolidPointClassification;
    use crate::winding::{ClosedMeshWindingRelation, PointMeshWindingReport};
    use hyperlimit::Point3;
    use hyperreal::Real;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn labeled_face(side: MeshSide) -> ExactCellComplexFace {
        let source = match side {
            MeshSide::Left => ExactCellRegionLabel::LeftBoundary,
            MeshSide::Right => ExactCellRegionLabel::RightBoundary,
        };
        ExactCellComplexFace {
            cell: ArrangementFaceCell {
                carrier: ArrangementFaceCarrier {
                    side,
                    face: 0,
                    triangle: [0, 1, 2],
                },
                boundary: [0, 1, 2]
                    .into_iter()
                    .map(|vertex| ArrangementFaceCellNode::SourceVertex { side, vertex })
                    .collect(),
                boundary_points: vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)],
                opposite: None,
            },
            source,
            opposite: ExactOppositeRegionLabel::Outside,
        }
    }

    fn winding_report(relation: ClosedMeshWindingRelation) -> PointMeshWindingReport {
        PointMeshWindingReport {
            relation,
            axis: None,
            tested_axes: 0,
            triangle_count: 0,
            crossings: 0,
            boundary_hits: 0,
            degenerate_hits: 0,
            parallel_faces: 0,
            unknown_hits: 0,
        }
    }

    fn convex_classification(relation: ConvexSolidPointRelation) -> ConvexSolidPointClassification {
        ConvexSolidPointClassification {
            relation,
            predicates: Vec::new(),
        }
    }

    fn face_with_opposite(
        winding_relation: ClosedMeshWindingRelation,
        convex_relation: Option<ConvexSolidPointRelation>,
    ) -> ArrangementFaceCell {
        let mut face = labeled_face(MeshSide::Left).cell;
        face.opposite = Some(ArrangementOppositeClassification {
            representative: p(0, 0, 0),
            winding: winding_report(winding_relation),
            convex_fallback: convex_relation.map(convex_classification),
        });
        face
    }

    fn boundary_labeled_face(side: MeshSide) -> ExactCellComplexFace {
        ExactCellComplexFace {
            opposite: ExactOppositeRegionLabel::Boundary,
            ..labeled_face(side)
        }
    }

    fn unoriented_labeled_face(side: MeshSide) -> ExactCellComplexFace {
        let mut face = labeled_face(side);
        face.cell.carrier.face = 1;
        face.cell.carrier.triangle = [1, 2, 3];
        face.cell.boundary = [1, 2, 3]
            .into_iter()
            .map(|vertex| ArrangementFaceCellNode::SourceVertex { side, vertex })
            .collect();
        face
    }

    fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    fn replay_arrangement_with_blocker(
        blocker: ExactArrangementBlocker,
    ) -> (ExactArrangement, ExactMesh, ExactMesh) {
        let left = tetrahedron_i64([0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, 2]);
        let right = tetrahedron_i64([1, 0, 0], [3, 0, 0], [1, 2, 0], [1, 0, 2]);
        let mut arrangement = ExactArrangement::from_meshes(&left, &right).unwrap();
        arrangement.blockers = vec![blocker];
        (arrangement, left, right)
    }

    fn labeled_with_volume_adjacency_face(
        face_cell: usize,
        blockers: Vec<ExactArrangementBlocker>,
    ) -> ExactLabeledCellComplex {
        let face = ExactCellComplexFace {
            opposite: ExactOppositeRegionLabel::Unknown,
            ..labeled_face(MeshSide::Left)
        };
        ExactLabeledCellComplex {
            faces: vec![face],
            volume_regions: vec![
                ExactCellComplexVolumeRegion {
                    index: 0,
                    exterior: true,
                    boundary_shells: vec![0],
                    in_left: false,
                    in_right: false,
                },
                ExactCellComplexVolumeRegion {
                    index: 1,
                    exterior: false,
                    boundary_shells: vec![0],
                    in_left: true,
                    in_right: false,
                },
            ],
            volume_adjacencies: vec![ArrangementVolumeAdjacency {
                shell_region: 0,
                exterior_volume: 0,
                interior_volume: 1,
                separating_face_cells: vec![face_cell],
                oriented_face_sides: vec![ArrangementVolumeFaceSide {
                    face_cell,
                    source: MeshSide::Left,
                    source_face: 0,
                    boundary: [0, 1, 2]
                        .into_iter()
                        .map(|vertex| ArrangementFaceCellNode::SourceVertex {
                            side: MeshSide::Left,
                            vertex,
                        })
                        .collect(),
                    exterior_volume: 0,
                    interior_volume: 1,
                }],
            }],
            lower_dimensional_artifacts: Vec::new(),
            blockers,
        }
    }

    #[test]
    fn selected_region_operation_respects_requested_source_side() {
        let labeled = ExactLabeledCellComplex {
            faces: vec![labeled_face(MeshSide::Left), labeled_face(MeshSide::Right)],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            blockers: Vec::new(),
        };

        let selected = labeled
            .select(ExactBooleanOperation::SelectedRegions(
                ExactRegionSelection::KeepLeft,
            ))
            .unwrap();

        assert_eq!(selected.selected_faces, vec![0]);
        assert_eq!(
            selected.selected_face_orientations,
            vec![ExactSelectedFaceOrientation {
                face: 0,
                reverse: false,
                from_volume_adjacency: false,
            }]
        );
    }

    #[test]
    fn label_face_cell_uses_certified_convex_relation_when_winding_is_unknown() {
        for (convex, expected) in [
            (
                ConvexSolidPointRelation::Inside,
                ExactOppositeRegionLabel::Inside,
            ),
            (
                ConvexSolidPointRelation::Outside,
                ExactOppositeRegionLabel::Outside,
            ),
            (
                ConvexSolidPointRelation::Boundary,
                ExactOppositeRegionLabel::Boundary,
            ),
            (
                ConvexSolidPointRelation::Unknown,
                ExactOppositeRegionLabel::Unknown,
            ),
            (
                ConvexSolidPointRelation::NotCertifiedConvex,
                ExactOppositeRegionLabel::Unknown,
            ),
        ] {
            let face = label_face_cell(face_with_opposite(
                ClosedMeshWindingRelation::Unknown,
                Some(convex),
            ));
            assert_eq!(face.opposite, expected, "{convex:?}");
        }
    }

    #[test]
    fn label_face_cell_prefers_certified_convex_relation_over_winding() {
        for (winding, convex, expected) in [
            (
                ClosedMeshWindingRelation::Inside,
                ConvexSolidPointRelation::Outside,
                ExactOppositeRegionLabel::Outside,
            ),
            (
                ClosedMeshWindingRelation::Outside,
                ConvexSolidPointRelation::Inside,
                ExactOppositeRegionLabel::Inside,
            ),
            (
                ClosedMeshWindingRelation::Inside,
                ConvexSolidPointRelation::Boundary,
                ExactOppositeRegionLabel::Boundary,
            ),
        ] {
            let face = label_face_cell(face_with_opposite(winding, Some(convex)));
            assert_eq!(face.opposite, expected, "{winding:?} {convex:?}");
        }
    }

    #[test]
    fn selected_region_operation_ignores_unneeded_opposite_classification_blockers() {
        let mut left = labeled_face(MeshSide::Left);
        left.opposite = ExactOppositeRegionLabel::Unknown;
        let mut right = labeled_face(MeshSide::Right);
        right.opposite = ExactOppositeRegionLabel::Unknown;
        let labeled = ExactLabeledCellComplex {
            faces: vec![left, right],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            blockers: vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        };

        let selected = labeled
            .select(ExactBooleanOperation::SelectedRegions(
                ExactRegionSelection::KeepLeft,
            ))
            .unwrap();

        assert_eq!(selected.selected_faces, vec![0]);
        assert!(selected.blockers.is_empty());
    }

    #[test]
    fn selected_region_operation_keeps_real_topology_blockers() {
        let labeled = ExactLabeledCellComplex {
            faces: vec![labeled_face(MeshSide::Left)],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            blockers: vec![ExactArrangementBlocker::NonManifoldCellComplex],
        };

        assert_eq!(
            labeled
                .select(ExactBooleanOperation::SelectedRegions(
                    ExactRegionSelection::KeepLeft,
                ))
                .unwrap_err(),
            ExactArrangementBlocker::NonManifoldCellComplex
        );
    }

    #[test]
    fn region_ownership_report_validation_rejects_overflowing_count_partitions() {
        let report = ExactRegionOwnershipReport {
            status: ExactRegionOwnershipStatus::FaceResolved,
            freshness: ExactLabeledCellComplexFreshness::Current,
            blockers: Vec::new(),
            face_cells: 1,
            face_cell_boundary_nodes: 3,
            face_cell_boundary_points: 3,
            left_boundary_faces: 1,
            right_boundary_faces: 0,
            opposite_inside_faces: 0,
            opposite_outside_faces: 1,
            opposite_boundary_faces: 0,
            opposite_unknown_faces: 0,
            volume_regions: 0,
            exterior_volume_regions: 0,
            left_owned_volumes: 0,
            right_owned_volumes: 0,
            shared_owned_volumes: 0,
            unowned_bounded_volumes: 0,
            volume_adjacencies: 0,
            volume_adjacency_face_sides: 0,
            volume_adjacency_separating_faces: 0,
            volume_selection_resolved: false,
            volume_union_resolved: false,
            volume_intersection_resolved: false,
            volume_difference_resolved: false,
            lower_dimensional_artifacts: 0,
            lower_dimensional_point_contacts: 0,
            lower_dimensional_edge_contacts: 0,
            lower_dimensional_edge_endpoints: 0,
        };
        report.validate().unwrap();
        assert!(report.status.is_resolved());
        assert!(!report.status.is_volume_resolved());

        let mut overflowing_boundary_partition = report.clone();
        overflowing_boundary_partition.left_boundary_faces = usize::MAX;
        overflowing_boundary_partition.right_boundary_faces = 1;
        assert_eq!(
            overflowing_boundary_partition.validate(),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );

        let mut overflowing_opposite_partition = report.clone();
        overflowing_opposite_partition.opposite_inside_faces = usize::MAX;
        overflowing_opposite_partition.opposite_outside_faces = 1;
        assert_eq!(
            overflowing_opposite_partition.validate(),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );

        let mut impossible_owned_volume_count = report.clone();
        impossible_owned_volume_count.volume_regions = 1;
        impossible_owned_volume_count.left_owned_volumes = 2;
        assert_eq!(
            impossible_owned_volume_count.validate(),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );

        let mut impossible_volume_partition = report.clone();
        impossible_volume_partition.volume_regions = 1;
        impossible_volume_partition.left_owned_volumes = 1;
        impossible_volume_partition.right_owned_volumes = 1;
        impossible_volume_partition.shared_owned_volumes = 0;
        assert_eq!(
            impossible_volume_partition.validate(),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );

        let mut overflowing_volume_partition = report;
        overflowing_volume_partition.volume_regions = usize::MAX;
        overflowing_volume_partition.left_owned_volumes = usize::MAX;
        overflowing_volume_partition.right_owned_volumes = 1;
        assert_eq!(
            overflowing_volume_partition.validate(),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );
    }

    #[test]
    fn region_ownership_status_requires_selectable_volume_evidence() {
        let status_without_volume_proof = region_ownership_status(
            ExactLabeledCellComplexFreshness::Current,
            &[ExactArrangementBlocker::UnresolvedRegionClassification],
            1,
            1,
            2,
            1,
            false,
        );
        assert_eq!(
            status_without_volume_proof,
            ExactRegionOwnershipStatus::RequiresWinding
        );

        let status_with_volume_proof = region_ownership_status(
            ExactLabeledCellComplexFreshness::Current,
            &[ExactArrangementBlocker::UnresolvedRegionClassification],
            1,
            1,
            2,
            1,
            true,
        );
        assert_eq!(
            status_with_volume_proof,
            ExactRegionOwnershipStatus::VolumeResolved
        );
    }

    #[test]
    fn arrangement_volume_resolution_requires_selectable_adjacency_provenance() {
        let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
        let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
        let mut arrangement = ExactArrangement::from_meshes(&left, &right).unwrap();
        arrangement.blockers = vec![ExactArrangementBlocker::UnresolvedRegionClassification];

        assert!(arrangement_region_classification_blockers_are_volume_resolved(&arrangement));
        assert!(
            arrangement_region_classification_blockers_resolve_operation(
                &arrangement,
                ExactBooleanOperation::Union
            )
        );
        assert_eq!(
            arrangement_cell_complex_labeling_policy(
                &arrangement,
                Some(ExactBooleanOperation::Union),
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )
            .unresolved,
            ExactUnresolvedPolicy::RetainArtifacts
        );

        arrangement.volume_adjacencies.as_mut().unwrap()[0]
            .separating_face_cells
            .clear();
        assert!(!arrangement_region_classification_blockers_are_volume_resolved(&arrangement));
        assert!(
            !arrangement_region_classification_blockers_resolve_operation(
                &arrangement,
                ExactBooleanOperation::Union
            )
        );
        assert_eq!(
            arrangement_cell_complex_labeling_policy(
                &arrangement,
                Some(ExactBooleanOperation::Union),
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )
            .unresolved,
            ExactUnresolvedPolicy::Block
        );
    }

    #[test]
    fn regularized_solid_selection_drops_boundary_contact_intersection() {
        let labeled = ExactLabeledCellComplex {
            faces: vec![
                boundary_labeled_face(MeshSide::Left),
                boundary_labeled_face(MeshSide::Right),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            blockers: Vec::new(),
        };

        let selected = labeled
            .select_with_policy(
                ExactBooleanOperation::Intersection,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )
            .unwrap();

        assert!(selected.selected_faces.is_empty());
        assert!(selected.selected_face_orientations.is_empty());
    }

    #[test]
    fn regularized_solid_difference_keeps_only_left_boundary_contact() {
        let labeled = ExactLabeledCellComplex {
            faces: vec![
                boundary_labeled_face(MeshSide::Left),
                boundary_labeled_face(MeshSide::Right),
            ],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            blockers: Vec::new(),
        };

        let selected = labeled
            .select_with_policy(
                ExactBooleanOperation::Difference,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )
            .unwrap();

        assert_eq!(selected.selected_faces, vec![0]);
        assert_eq!(
            selected.selected_face_orientations,
            vec![ExactSelectedFaceOrientation {
                face: 0,
                reverse: false,
                from_volume_adjacency: false,
            }]
        );
    }

    #[test]
    fn named_operation_can_select_faces_from_volume_adjacency() {
        let labeled = labeled_with_volume_adjacency_face(
            0,
            vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        );

        let selected = labeled.select(ExactBooleanOperation::Union).unwrap();

        assert_eq!(selected.selected_volume_regions, vec![1]);
        assert_eq!(selected.selected_faces, vec![0]);
        assert_eq!(
            selected.selected_face_orientations,
            vec![ExactSelectedFaceOrientation {
                face: 0,
                reverse: false,
                from_volume_adjacency: true,
            }]
        );
        assert!(selected.blockers.is_empty());
    }

    #[test]
    fn named_operation_selects_all_boundary_equivalent_volume_separator_faces() {
        let mut labeled = labeled_with_volume_adjacency_face(
            0,
            vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        );
        let duplicate_separator = labeled.faces[0].clone();
        labeled.faces.push(duplicate_separator);
        labeled.volume_adjacencies[0].separating_face_cells.push(1);

        let selected = labeled.select(ExactBooleanOperation::Union).unwrap();

        assert_eq!(selected.selected_volume_regions, vec![1]);
        assert_eq!(selected.selected_faces, vec![0, 1]);
        assert_eq!(
            selected.selected_face_orientations,
            vec![
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
            ]
        );
        assert!(selected.blockers.is_empty());
        selected.validate().unwrap();
    }

    #[test]
    fn named_operation_rejects_out_of_range_volume_adjacency_face() {
        let labeled = labeled_with_volume_adjacency_face(1, Vec::new());

        assert_eq!(
            labeled.select(ExactBooleanOperation::Union),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn named_operation_rejects_out_of_range_volume_adjacency_separating_face() {
        let mut labeled = labeled_with_volume_adjacency_face(0, Vec::new());
        labeled.volume_adjacencies[0].separating_face_cells = vec![1];

        assert_eq!(
            labeled.select(ExactBooleanOperation::Union),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn named_operation_rejects_duplicate_volume_adjacency_separating_face() {
        let mut labeled = labeled_with_volume_adjacency_face(0, Vec::new());
        labeled.volume_adjacencies[0].separating_face_cells = vec![0, 0];

        assert_eq!(
            labeled.select(ExactBooleanOperation::Union),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn named_operation_rejects_unoriented_separating_face() {
        let mut labeled = labeled_with_volume_adjacency_face(0, Vec::new());
        labeled.faces.push(unoriented_labeled_face(MeshSide::Left));
        labeled.volume_adjacencies[0].separating_face_cells = vec![0, 1];

        assert_eq!(
            labeled.select(ExactBooleanOperation::Union),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn named_operation_rejects_stale_oriented_side_source_face() {
        let mut labeled = labeled_with_volume_adjacency_face(0, Vec::new());
        labeled.volume_adjacencies[0].oriented_face_sides[0].source_face = 1;

        assert_eq!(
            labeled.select(ExactBooleanOperation::Union),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn named_operation_validates_unselected_volume_adjacency_provenance() {
        let mut labeled = labeled_with_volume_adjacency_face(0, Vec::new());
        labeled.volume_regions[0].in_left = true;
        labeled.volume_adjacencies[0].separating_face_cells = vec![1];

        assert_eq!(
            labeled.select(ExactBooleanOperation::Union),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn named_operation_rejects_stale_volume_region_index() {
        let mut labeled = labeled_with_volume_adjacency_face(0, Vec::new());
        labeled.volume_regions[1].index = 7;

        assert_eq!(
            labeled.select(ExactBooleanOperation::Union),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn labeled_cell_complex_validate_rejects_stale_volume_adjacency_provenance() {
        let mut labeled = labeled_with_volume_adjacency_face(0, Vec::new());
        labeled.validate().unwrap();
        labeled.volume_adjacencies[0].oriented_face_sides[0].source_face = 1;

        assert_eq!(
            labeled.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn selected_cell_complex_validate_rejects_duplicate_selected_face() {
        let labeled = ExactLabeledCellComplex {
            faces: vec![labeled_face(MeshSide::Left)],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            blockers: Vec::new(),
        };
        let mut selected = labeled
            .select(ExactBooleanOperation::SelectedRegions(
                ExactRegionSelection::KeepLeft,
            ))
            .unwrap();
        selected.validate().unwrap();
        selected.selected_faces.push(0);
        selected
            .selected_face_orientations
            .push(ExactSelectedFaceOrientation {
                face: 0,
                reverse: false,
                from_volume_adjacency: false,
            });

        assert_eq!(
            selected.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn selected_cell_complex_validate_rejects_stale_volume_adjacency_orientation() {
        let labeled = labeled_with_volume_adjacency_face(
            0,
            vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        );
        let mut selected = labeled.select(ExactBooleanOperation::Union).unwrap();
        selected.validate().unwrap();
        selected.selected_face_orientations[0].reverse = true;

        assert_eq!(
            selected.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn volume_resolved_selection_consumes_only_region_classification_blockers() {
        let labeled = labeled_with_volume_adjacency_face(
            0,
            vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        );

        let selected = labeled
            .select_volume_resolved_with_policy(
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )
            .unwrap();

        assert_eq!(selected.selected_faces, vec![0]);
        assert_eq!(
            selected.selected_face_orientations,
            vec![ExactSelectedFaceOrientation {
                face: 0,
                reverse: false,
                from_volume_adjacency: true,
            }]
        );
        assert!(selected.blockers.is_empty());
    }

    #[test]
    fn volume_selection_evidence_can_resolve_only_requested_operation() {
        let mut labeled = labeled_with_volume_adjacency_face(
            0,
            vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        );
        labeled.volume_regions.push(ExactCellComplexVolumeRegion {
            index: 2,
            exterior: false,
            boundary_shells: vec![1],
            in_left: false,
            in_right: true,
        });
        let mut crossing_adjacency = labeled.volume_adjacencies[0].clone();
        crossing_adjacency.shell_region = 1;
        crossing_adjacency.exterior_volume = 2;
        crossing_adjacency.interior_volume = 0;
        crossing_adjacency.oriented_face_sides[0].exterior_volume = 2;
        crossing_adjacency.oriented_face_sides[0].interior_volume = 0;
        labeled.volume_adjacencies.push(crossing_adjacency);

        assert!(!volume_evidence_resolves_named_selection(
            &labeled.faces,
            &labeled.volume_regions,
            &labeled.volume_adjacencies,
        ));
        assert!(!volume_evidence_resolves_named_operation(
            &labeled.faces,
            &labeled.volume_regions,
            &labeled.volume_adjacencies,
            ExactBooleanOperation::Union,
        ));
        assert!(volume_evidence_resolves_named_operation(
            &labeled.faces,
            &labeled.volume_regions,
            &labeled.volume_adjacencies,
            ExactBooleanOperation::Difference,
        ));
        assert_eq!(
            labeled.clone().select_volume_resolved_with_policy(
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );

        let selected = labeled
            .clone()
            .select_volume_resolved_with_policy(
                ExactBooleanOperation::Difference,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            )
            .unwrap();
        assert_eq!(selected.selected_faces, vec![0]);
        assert_eq!(selected.selected_volume_regions, vec![1]);
        assert!(selected.blockers.is_empty());

        let report = ExactRegionOwnershipReport {
            status: ExactRegionOwnershipStatus::RequiresWinding,
            freshness: ExactLabeledCellComplexFreshness::Current,
            blockers: vec![ExactArrangementBlocker::UnresolvedRegionClassification],
            face_cells: 1,
            face_cell_boundary_nodes: 3,
            face_cell_boundary_points: 3,
            left_boundary_faces: 1,
            right_boundary_faces: 0,
            opposite_inside_faces: 0,
            opposite_outside_faces: 0,
            opposite_boundary_faces: 0,
            opposite_unknown_faces: 1,
            volume_regions: 3,
            exterior_volume_regions: 1,
            left_owned_volumes: 1,
            right_owned_volumes: 1,
            shared_owned_volumes: 0,
            unowned_bounded_volumes: 0,
            volume_adjacencies: 2,
            volume_adjacency_face_sides: 2,
            volume_adjacency_separating_faces: 2,
            volume_selection_resolved: false,
            volume_union_resolved: false,
            volume_intersection_resolved: true,
            volume_difference_resolved: true,
            lower_dimensional_artifacts: 0,
            lower_dimensional_point_contacts: 0,
            lower_dimensional_edge_contacts: 0,
            lower_dimensional_edge_endpoints: 0,
        };
        report.validate().unwrap();
        assert!(!report.is_resolved());
        assert!(report.resolves_operation_selection(ExactBooleanOperation::Difference));
        assert!(!report.resolves_operation_selection(ExactBooleanOperation::Union));
        assert!(report.volume_selection_resolves_operation(ExactBooleanOperation::Difference));
        assert!(!report.volume_selection_resolves_operation(ExactBooleanOperation::Union));
    }

    #[test]
    fn volume_resolved_selection_rejects_out_of_range_volume_adjacency_face() {
        let labeled = labeled_with_volume_adjacency_face(
            1,
            vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        );

        assert_eq!(
            labeled.select_volume_resolved_with_policy(
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn volume_resolved_selection_rejects_mismatched_volume_adjacency_face_sets() {
        let mut labeled = labeled_with_volume_adjacency_face(
            0,
            vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        );
        labeled.volume_adjacencies[0].separating_face_cells.clear();

        assert_eq!(
            labeled.select_volume_resolved_with_policy(
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn volume_resolved_selection_rejects_duplicate_volume_adjacency_separating_face() {
        let mut labeled = labeled_with_volume_adjacency_face(
            0,
            vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        );
        labeled.volume_adjacencies[0].separating_face_cells = vec![0, 0];

        assert_eq!(
            labeled.select_volume_resolved_with_policy(
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn volume_resolved_selection_rejects_unoriented_separating_face() {
        let mut labeled = labeled_with_volume_adjacency_face(
            0,
            vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        );
        labeled.faces.push(unoriented_labeled_face(MeshSide::Left));
        labeled.volume_adjacencies[0].separating_face_cells = vec![0, 1];

        assert_eq!(
            labeled.select_volume_resolved_with_policy(
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn volume_resolved_selection_rejects_stale_volume_region_index() {
        let mut labeled = labeled_with_volume_adjacency_face(
            0,
            vec![ExactArrangementBlocker::UnresolvedRegionClassification],
        );
        labeled.volume_regions[1].index = 7;

        assert_eq!(
            labeled.select_volume_resolved_with_policy(
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn volume_resolved_selection_rejects_non_region_blockers() {
        let labeled = ExactLabeledCellComplex {
            faces: vec![labeled_face(MeshSide::Left)],
            volume_regions: Vec::new(),
            volume_adjacencies: Vec::new(),
            lower_dimensional_artifacts: Vec::new(),
            blockers: vec![ExactArrangementBlocker::UnresolvedIntersection],
        };

        assert_eq!(
            labeled.select_volume_resolved_with_policy(
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::UnresolvedIntersection)
        );
    }

    #[test]
    fn replay_selection_rejects_stale_region_classification_blocker_mutation() {
        let (arrangement, left, right) = replay_arrangement_with_blocker(
            ExactArrangementBlocker::UnresolvedRegionClassification,
        );

        assert_eq!(
            select_arrangement_for_replay(
                arrangement,
                &left,
                &right,
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn replay_selection_retains_topology_and_ownership_reports() {
        let left = tetrahedron_i64([0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, 2]);
        let right = tetrahedron_i64([1, 0, 0], [3, 0, 0], [1, 2, 0], [1, 0, 2]);
        let arrangement = ExactArrangement::from_meshes_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();

        let selected = select_arrangement_for_replay(
            arrangement,
            &left,
            &right,
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();

        selected
            .topology_assembly_report
            .as_ref()
            .expect("replay-selected cells should retain topology assembly")
            .validate()
            .unwrap();
        let ownership = selected
            .region_ownership_report
            .as_ref()
            .expect("replay-selected cells should retain region ownership");
        ownership.validate().unwrap();
        assert!(ownership.is_resolved());
        assert!(ownership.status.is_resolved());
        selected.validate().unwrap();
        selected
            .validate_against_sources(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        let simplified = selected
            .clone()
            .simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
        assert!(simplified.topology_assembly_report.is_some());
        assert!(simplified.region_ownership_report.is_some());
        simplified.validate().unwrap();

        let mut stale_topology = selected.clone();
        stale_topology
            .topology_assembly_report
            .as_mut()
            .unwrap()
            .status = ExactTopologyAssemblyStatus::MissingRegionPlan;
        assert_eq!(
            stale_topology.validate(),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );
        assert_eq!(
            stale_topology.simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
        );

        let mut stale_payload_counts = selected.clone();
        stale_payload_counts
            .faces
            .push(labeled_face(MeshSide::Left));
        assert_eq!(
            stale_payload_counts.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
        assert_eq!(
            stale_payload_counts
                .simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );

        let mut missing_topology = selected;
        missing_topology.topology_assembly_report = None;
        assert_eq!(
            missing_topology.validate(),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
        assert_eq!(
            missing_topology
                .simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID),
            Err(ExactArrangementBlocker::NonManifoldCellComplex)
        );
    }

    #[test]
    fn replay_selection_rejects_non_region_classification_blockers() {
        let (arrangement, left, right) =
            replay_arrangement_with_blocker(ExactArrangementBlocker::UnresolvedIntersection);

        assert_eq!(
            select_arrangement_for_replay(
                arrangement,
                &left,
                &right,
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::UnresolvedIntersection)
        );
    }
}
