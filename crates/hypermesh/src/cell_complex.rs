//! Exact cell-complex selection over retained arrangements.
//!
//! The cell complex keeps arrangement face-cells as the primary topological
//! unit. Boolean operations are selection rules over labels; mesh
//! triangulation/export remains a later step with its own approximation or
//! triangulation policy.

use super::arrangement3d::{
    ArrangementFaceCell, ArrangementLowerDimensionalArtifact, ArrangementVolumeAdjacency,
    ArrangementVolumeRegion, ExactArrangement, ExactArrangement3d,
};
use super::boolean::ExactBooleanOperation;
use super::graph::MeshSide;
use super::regularization::{
    ExactArrangementBlocker, ExactLowerDimensionalPolicy, ExactRegularizationPolicy,
    ExactUnresolvedPolicy,
};
use super::simplify::{ExactSimplifiedCellComplex, simplify_selected_cell_complex};
use super::solid::ConvexSolidPointRelation;
use super::winding::ClosedMeshWindingRelation;

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
            .as_ref()
            .map(|regions| regions.iter().map(label_volume_region).collect())
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

impl ExactLabeledCellComplex {
    /// Validate this labeled complex by replaying arrangement construction and
    /// region labeling from source operands.
    pub fn validate_against_sources(
        &self,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<(), ExactArrangementBlocker> {
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
        if selected_region_selection_ignores_opposite_classification(operation) {
            blockers.retain(|blocker| {
                *blocker != ExactArrangementBlocker::UnresolvedRegionClassification
            });
        }
        let selected_volume_regions = selected_volume_regions(&self.volume_regions, operation);
        let volume_selected = select_faces_from_volume_adjacencies(
            self.faces.len(),
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
        let Some((selected_faces, selected_face_orientations)) =
            select_faces_from_volume_adjacencies(
                self.faces.len(),
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
            selected_faces,
            selected_face_orientations,
            selected_volume_regions,
            operation,
            blockers: Vec::new(),
        })
    }
}

impl ExactSelectedCellComplex {
    /// Validate this selected complex by replaying arrangement construction,
    /// labeling, and selection from source operands.
    pub fn validate_against_sources(
        &self,
        left: &super::mesh::ExactMesh,
        right: &super::mesh::ExactMesh,
        policy: ExactRegularizationPolicy,
    ) -> Result<(), ExactArrangementBlocker> {
        let arrangement = ExactArrangement::from_meshes_with_policy(left, right, policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?;
        let replay = select_arrangement_for_replay(arrangement, self.operation, policy)?;
        if replay == *self {
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
        match select_arrangement_for_replay(arrangement, self.operation, policy) {
            Ok(replay) if replay == *self => ExactSelectedCellComplexFreshness::Current,
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
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
) -> Result<ExactSelectedCellComplex, ExactArrangementBlocker> {
    let volume_resolved =
        arrangement_region_classification_blockers_are_volume_resolved(&arrangement);
    let labeling_policy = if volume_resolved {
        ExactRegularizationPolicy {
            unresolved: ExactUnresolvedPolicy::RetainArtifacts,
            ..policy
        }
    } else {
        policy
    };
    let labeled = arrangement.label_regions(labeling_policy)?;
    if volume_resolved {
        labeled.select_volume_resolved_with_policy(operation, policy)
    } else {
        labeled.select_with_policy(operation, policy)
    }
}

pub(crate) fn arrangement_region_classification_blockers_are_volume_resolved(
    arrangement: &ExactArrangement3d,
) -> bool {
    !arrangement.blockers.is_empty()
        && arrangement
            .blockers
            .iter()
            .all(|blocker| *blocker == ExactArrangementBlocker::UnresolvedRegionClassification)
        && arrangement
            .volume_regions
            .as_ref()
            .is_some_and(|regions| !regions.is_empty())
        && arrangement
            .volume_adjacencies
            .as_ref()
            .is_some_and(|adjacencies| !adjacencies.is_empty())
}

fn label_face_cell(cell: ArrangementFaceCell) -> ExactCellComplexFace {
    let source = match cell.carrier.side {
        MeshSide::Left => ExactCellRegionLabel::LeftBoundary,
        MeshSide::Right => ExactCellRegionLabel::RightBoundary,
    };
    let opposite = cell
        .opposite
        .as_ref()
        .map_or(ExactOppositeRegionLabel::Unknown, label_opposite_region);
    ExactCellComplexFace {
        cell,
        source,
        opposite,
    }
}

fn label_opposite_region(
    opposite: &super::arrangement3d::ArrangementOppositeClassification,
) -> ExactOppositeRegionLabel {
    match opposite.winding.relation {
        ClosedMeshWindingRelation::Inside => ExactOppositeRegionLabel::Inside,
        ClosedMeshWindingRelation::Outside => ExactOppositeRegionLabel::Outside,
        ClosedMeshWindingRelation::Boundary => ExactOppositeRegionLabel::Boundary,
        ClosedMeshWindingRelation::Unknown | ClosedMeshWindingRelation::NotClosed => match opposite
            .convex_fallback
            .as_ref()
            .map(|fallback| fallback.relation)
        {
            Some(ConvexSolidPointRelation::Inside) => ExactOppositeRegionLabel::Inside,
            Some(ConvexSolidPointRelation::Outside) => ExactOppositeRegionLabel::Outside,
            Some(ConvexSolidPointRelation::Boundary) => ExactOppositeRegionLabel::Boundary,
            Some(ConvexSolidPointRelation::Unknown)
            | Some(ConvexSolidPointRelation::NotCertifiedConvex)
            | None => ExactOppositeRegionLabel::Unknown,
        },
    }
}

fn label_volume_region(region: &ArrangementVolumeRegion) -> ExactCellComplexVolumeRegion {
    ExactCellComplexVolumeRegion {
        index: region.index,
        exterior: region.exterior,
        boundary_shells: region.boundary_shells.clone(),
        in_left: region.source_sides.contains(&MeshSide::Left),
        in_right: region.source_sides.contains(&MeshSide::Right),
    }
}

fn select_face(
    face: &ExactCellComplexFace,
    operation: ExactBooleanOperation,
    policy: ExactRegularizationPolicy,
) -> Option<bool> {
    if let ExactBooleanOperation::SelectedRegions(selection) = operation {
        return Some(selection.keeps(mesh_side_for_source(face.source)));
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

pub(crate) fn selected_region_selection_ignores_opposite_classification(
    operation: ExactBooleanOperation,
) -> bool {
    matches!(operation, ExactBooleanOperation::SelectedRegions(_))
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
    face_count: usize,
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
    let mut selected = Vec::<ExactSelectedFaceOrientation>::new();
    for adjacency in volume_adjacencies {
        validate_volume_adjacency_face_provenance(face_count, adjacency)?;
        let exterior_selected = *selected_volumes
            .get(adjacency.exterior_volume)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
        let interior_selected = *selected_volumes
            .get(adjacency.interior_volume)
            .ok_or(ExactArrangementBlocker::NonManifoldCellComplex)?;
        if exterior_selected == interior_selected {
            continue;
        }
        for side in &adjacency.oriented_face_sides {
            if side.exterior_volume != adjacency.exterior_volume
                || side.interior_volume != adjacency.interior_volume
                || side.face_cell >= face_count
            {
                return Err(ExactArrangementBlocker::NonManifoldCellComplex);
            }
            let reverse = exterior_selected && !interior_selected;
            match selected
                .iter()
                .position(|orientation| orientation.face == side.face_cell)
            {
                Some(index) if selected[index].reverse != reverse => {
                    return Err(ExactArrangementBlocker::NonManifoldCellComplex);
                }
                Some(_) => {}
                None => selected.push(ExactSelectedFaceOrientation {
                    face: side.face_cell,
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
    face_count: usize,
    adjacency: &ArrangementVolumeAdjacency,
) -> Result<(), ExactArrangementBlocker> {
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
        side_faces.push(side.face_cell);
    }
    side_faces.sort_unstable();
    side_faces.dedup();
    let mut separating_face_cells = adjacency.separating_face_cells.clone();
    separating_face_cells.sort_unstable();
    separating_face_cells.dedup();
    if side_faces
        .iter()
        .any(|face| separating_face_cells.binary_search(face).is_err())
    {
        return Err(ExactArrangementBlocker::NonManifoldCellComplex);
    }
    Ok(())
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

const fn mesh_side_for_source(source: ExactCellRegionLabel) -> MeshSide {
    match source {
        ExactCellRegionLabel::LeftBoundary => MeshSide::Left,
        ExactCellRegionLabel::RightBoundary => MeshSide::Right,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arrangement3d::{
        ArrangementFaceCarrier, ArrangementFaceCell, ArrangementFaceCellNode,
        ArrangementVolumeFaceSide,
    };
    use crate::mesh::ExactMesh;
    use crate::region::ExactRegionSelection;
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

    fn boundary_labeled_face(side: MeshSide) -> ExactCellComplexFace {
        ExactCellComplexFace {
            opposite: ExactOppositeRegionLabel::Boundary,
            ..labeled_face(side)
        }
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

    fn replay_arrangement_with_blocker(blocker: ExactArrangementBlocker) -> ExactArrangement {
        let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
        let right = tetrahedron_i64([3, 0, 0], [4, 0, 0], [3, 1, 0], [3, 0, 1]);
        let mut arrangement = ExactArrangement::from_meshes(&left, &right).unwrap();
        arrangement.blockers = vec![blocker];
        arrangement
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
    fn replay_selection_uses_volume_resolved_path_for_region_classification_blockers() {
        let arrangement = replay_arrangement_with_blocker(
            ExactArrangementBlocker::UnresolvedRegionClassification,
        );

        let selected = select_arrangement_for_replay(
            arrangement,
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();

        assert_eq!(selected.selected_faces.len(), 8);
        assert_eq!(selected.selected_volume_regions, vec![1, 2]);
        assert!(selected.blockers.is_empty());
        assert!(
            selected
                .selected_face_orientations
                .iter()
                .all(|orientation| orientation.from_volume_adjacency)
        );
    }

    #[test]
    fn replay_selection_rejects_non_region_classification_blockers() {
        let arrangement =
            replay_arrangement_with_blocker(ExactArrangementBlocker::UnresolvedIntersection);

        assert_eq!(
            select_arrangement_for_replay(
                arrangement,
                ExactBooleanOperation::Union,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
            ),
            Err(ExactArrangementBlocker::UnresolvedIntersection)
        );
    }
}
