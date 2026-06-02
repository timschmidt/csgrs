//! Exact cell-complex selection over retained arrangements.
//!
//! The cell complex keeps arrangement face-cells as the primary topological
//! unit. Boolean operations are selection rules over labels; mesh
//! triangulation/export remains a later step with its own approximation or
//! triangulation policy.

use super::arrangement3d::{ArrangementFaceCell, ExactArrangement, ExactArrangement3d};
use super::boolean::ExactBooleanOperation;
use super::graph::MeshSide;
use super::regularization::{
    ExactArrangementBlocker, ExactLowerDimensionalPolicy, ExactRegularizationPolicy,
};
use super::simplify::{ExactSimplifiedCellComplex, simplify_selected_cell_complex};
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
    /// Blockers inherited or introduced during labeling.
    pub blockers: Vec<ExactArrangementBlocker>,
}

/// Selected cells for a Boolean operation.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactSelectedCellComplex {
    /// Labeled face-cells.
    pub faces: Vec<ExactCellComplexFace>,
    /// Indices of selected `faces`.
    pub selected_faces: Vec<usize>,
    /// Boolean operation used for selection.
    pub operation: ExactBooleanOperation,
    /// Blockers inherited or introduced during selection.
    pub blockers: Vec<ExactArrangementBlocker>,
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
        let faces = self
            .arrangement
            .face_cells
            .iter()
            .cloned()
            .map(|cell| label_face_cell(cell, &mut blockers))
            .collect::<Vec<_>>();
        if !blockers.is_empty()
            && policy.unresolved == super::regularization::ExactUnresolvedPolicy::Block
        {
            return Err(blockers[0].clone());
        }
        Ok(ExactLabeledCellComplex { faces, blockers })
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
        let mut selected_faces = Vec::new();
        for (index, face) in self.faces.iter().enumerate() {
            if face.opposite == ExactOppositeRegionLabel::Boundary
                && policy.lower_dimensional == ExactLowerDimensionalPolicy::ReportBlocker
            {
                blockers.push(ExactArrangementBlocker::LowerDimensionalContact);
            }
            match select_face(face, operation) {
                Some(true) => selected_faces.push(index),
                Some(false) => {}
                None => blockers.push(ExactArrangementBlocker::UnresolvedRegionClassification),
            }
        }
        if !blockers.is_empty()
            && policy.unresolved == super::regularization::ExactUnresolvedPolicy::Block
        {
            return Err(blockers[0].clone());
        }
        Ok(ExactSelectedCellComplex {
            faces: self.faces,
            selected_faces,
            operation,
            blockers,
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
        let replay = ExactArrangement::from_meshes_with_policy(left, right, policy)
            .map_err(|_| ExactArrangementBlocker::UnresolvedIntersection)?
            .label_regions(policy)?
            .select_with_policy(self.operation, policy)?;
        if replay == *self {
            Ok(())
        } else {
            Err(ExactArrangementBlocker::UnresolvedRegionClassification)
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

fn label_face_cell(
    cell: ArrangementFaceCell,
    blockers: &mut Vec<ExactArrangementBlocker>,
) -> ExactCellComplexFace {
    let source = match cell.carrier.side {
        MeshSide::Left => ExactCellRegionLabel::LeftBoundary,
        MeshSide::Right => ExactCellRegionLabel::RightBoundary,
    };
    let opposite = match cell
        .opposite
        .as_ref()
        .map(|opposite| opposite.winding.relation)
    {
        Some(ClosedMeshWindingRelation::Inside) => ExactOppositeRegionLabel::Inside,
        Some(ClosedMeshWindingRelation::Outside) => ExactOppositeRegionLabel::Outside,
        Some(ClosedMeshWindingRelation::Boundary) => ExactOppositeRegionLabel::Boundary,
        Some(ClosedMeshWindingRelation::Unknown | ClosedMeshWindingRelation::NotClosed) | None => {
            blockers.push(ExactArrangementBlocker::UnresolvedRegionClassification);
            ExactOppositeRegionLabel::Unknown
        }
    };
    ExactCellComplexFace {
        cell,
        source,
        opposite,
    }
}

fn select_face(face: &ExactCellComplexFace, operation: ExactBooleanOperation) -> Option<bool> {
    let inside = match face.opposite {
        ExactOppositeRegionLabel::Inside | ExactOppositeRegionLabel::Boundary => true,
        ExactOppositeRegionLabel::Outside => false,
        ExactOppositeRegionLabel::Unknown => return None,
    };
    match operation {
        ExactBooleanOperation::Union => Some(!inside),
        ExactBooleanOperation::Intersection => Some(inside),
        ExactBooleanOperation::Difference => match face.source {
            ExactCellRegionLabel::LeftBoundary => Some(!inside),
            ExactCellRegionLabel::RightBoundary => Some(inside),
        },
        ExactBooleanOperation::SelectedRegions(selection) => {
            Some(selection.keeps(mesh_side_for_source(face.source)))
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
    use crate::exact::arrangement3d::{
        ArrangementFaceCarrier, ArrangementFaceCell, ArrangementFaceCellNode,
    };
    use crate::exact::region::ExactRegionSelection;
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

    #[test]
    fn selected_region_operation_respects_requested_source_side() {
        let labeled = ExactLabeledCellComplex {
            faces: vec![labeled_face(MeshSide::Left), labeled_face(MeshSide::Right)],
            blockers: Vec::new(),
        };

        let selected = labeled
            .select(ExactBooleanOperation::SelectedRegions(
                ExactRegionSelection::KeepLeft,
            ))
            .unwrap();

        assert_eq!(selected.selected_faces, vec![0]);
    }
}
