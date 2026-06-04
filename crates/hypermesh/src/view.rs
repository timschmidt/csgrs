//! Report-bearing approximate views of exact meshes.
//!
//! Rendering, file export, previews, and broad-phase diagnostics often need
//! primitive floats. Those values are views, not mesh identity and not topology
//! certificates. [`ApproximateMeshF64View`] lowers exact coordinates through
//! [`hyperreal::Real::to_f64_lossy`] only after replaying retained mesh state,
//! and can validate the retained primitive-float rows back against the exact
//! useful, but exact geometric decisions must remain tied to exact objects and
//! proof-producing predicates.

use super::{ExactMesh, ExactMeshAuditError, ExactMeshAuditReport, audit_exact_mesh};
use hyperlimit::Point3;
use hyperreal::Real;

/// Primitive-float view of an [`ExactMesh`] with replay metadata.
#[derive(Clone, Debug, PartialEq)]
pub struct ApproximateMeshF64View {
    /// Retained exact mesh audit used to build this view.
    pub audit: ExactMeshAuditReport,
    /// Flat `x, y, z` primitive-float coordinate rows.
    pub positions: Vec<f64>,
    /// Flat triangle index rows.
    pub indices: Vec<usize>,
    /// Number of exact coordinates exported to `f64`.
    pub exported_coordinates: usize,
    /// Whether this object is explicitly a lossy approximate view.
    pub lossy_view: bool,
}

/// Error returned when building or replaying an approximate mesh view fails.
#[derive(Clone, Debug, PartialEq)]
pub enum ApproximateMeshF64ViewError {
    /// The source mesh failed retained-state audit.
    Audit(super::ExactMeshValidationError),
    /// The retained audit in the view no longer matches the source mesh.
    AuditReplay(ExactMeshAuditError),
    /// An exact coordinate could not be represented as finite `f64`.
    CoordinateExportFailed {
        /// Vertex index.
        vertex: usize,
        /// Coordinate lane in `[x, y, z]`.
        coordinate: usize,
    },
    /// A retained primitive-float coordinate is not finite.
    NonFiniteCoordinate {
        /// Flat coordinate index.
        coordinate: usize,
    },
    /// Flat coordinate count does not match the source vertex count.
    PositionCountMismatch {
        /// Expected flat coordinate count.
        expected: usize,
        /// Actual flat coordinate count.
        actual: usize,
    },
    /// Flat index count does not match the source triangle count.
    IndexCountMismatch {
        /// Expected flat index count.
        expected: usize,
        /// Actual flat index count.
        actual: usize,
    },
    /// A retained primitive-float coordinate no longer replays from the source.
    CoordinateReplayMismatch {
        /// Flat coordinate index.
        coordinate: usize,
    },
    /// A retained triangle index no longer replays from the source.
    IndexReplayMismatch {
        /// Flat index slot.
        index: usize,
    },
    /// The view was relabeled as non-lossy.
    MissingLossyViewFlag,
    /// Exported coordinate count disagrees with retained rows.
    ExportedCoordinateCountMismatch {
        /// Expected exported coordinate count.
        expected: usize,
        /// Actual retained exported coordinate count.
        actual: usize,
    },
}

/// Freshness status for a retained primitive-float mesh view.
///
/// The status is an adapter diagnostic only. `Current` means the lossy view
/// still replays bit-for-bit from the exact mesh; it does not authorize
/// exact predicates and approximate representatives separate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ApproximateMeshF64ViewFreshness {
    /// The approximate view replays exactly against the current mesh.
    Current,
    /// The source mesh or retained audit is stale/invalid.
    StaleAudit,
    /// The view was relabeled as non-lossy.
    MissingLossyFlag,
    /// Flat coordinate or index row counts are stale.
    StaleRows,
    /// A retained primitive-float coordinate is non-finite.
    NonFiniteCoordinate,
    /// Exact coordinate export is no longer possible for this view.
    CoordinateExportFailed,
    /// A coordinate value no longer replays bit-for-bit.
    StaleCoordinate,
    /// An index value no longer replays.
    StaleIndex,
}

impl ApproximateMeshF64View {
    /// Build a primitive-float view from an exact mesh after retained-state replay.
    pub fn from_mesh(mesh: &ExactMesh) -> Result<Self, ApproximateMeshF64ViewError> {
        let audit = audit_exact_mesh(mesh).map_err(ApproximateMeshF64ViewError::Audit)?;
        let mut positions = Vec::with_capacity(mesh.vertices().len() * 3);
        for (vertex_index, vertex) in mesh.vertices().iter().enumerate() {
            for coordinate in 0..3 {
                let Some(value) = point_coordinate(vertex, coordinate).to_f64_lossy() else {
                    return Err(ApproximateMeshF64ViewError::CoordinateExportFailed {
                        vertex: vertex_index,
                        coordinate,
                    });
                };
                if !value.is_finite() {
                    return Err(ApproximateMeshF64ViewError::CoordinateExportFailed {
                        vertex: vertex_index,
                        coordinate,
                    });
                }
                positions.push(value);
            }
        }
        let indices = mesh
            .triangles()
            .iter()
            .flat_map(|triangle| triangle.0)
            .collect::<Vec<_>>();
        Ok(Self {
            audit,
            exported_coordinates: positions.len(),
            positions,
            indices,
            lossy_view: true,
        })
    }

    /// Validate that this approximate view still replays from `mesh`.
    pub fn validate_against_mesh(
        &self,
        mesh: &ExactMesh,
    ) -> Result<(), ApproximateMeshF64ViewError> {
        self.audit
            .validate_against_mesh(mesh)
            .map_err(ApproximateMeshF64ViewError::AuditReplay)?;
        if !self.lossy_view {
            return Err(ApproximateMeshF64ViewError::MissingLossyViewFlag);
        }
        let expected_positions = mesh.vertices().len() * 3;
        if self.positions.len() != expected_positions {
            return Err(ApproximateMeshF64ViewError::PositionCountMismatch {
                expected: expected_positions,
                actual: self.positions.len(),
            });
        }
        if self.exported_coordinates != self.positions.len() {
            return Err(
                ApproximateMeshF64ViewError::ExportedCoordinateCountMismatch {
                    expected: self.positions.len(),
                    actual: self.exported_coordinates,
                },
            );
        }
        let expected_indices = mesh.triangles().len() * 3;
        if self.indices.len() != expected_indices {
            return Err(ApproximateMeshF64ViewError::IndexCountMismatch {
                expected: expected_indices,
                actual: self.indices.len(),
            });
        }
        for (coordinate, value) in self.positions.iter().copied().enumerate() {
            if !value.is_finite() {
                return Err(ApproximateMeshF64ViewError::NonFiniteCoordinate { coordinate });
            }
            let vertex = coordinate / 3;
            let lane = coordinate % 3;
            let Some(expected) = point_coordinate(&mesh.vertices()[vertex], lane).to_f64_lossy()
            else {
                return Err(ApproximateMeshF64ViewError::CoordinateExportFailed {
                    vertex,
                    coordinate: lane,
                });
            };
            if value.to_bits() != expected.to_bits() {
                return Err(ApproximateMeshF64ViewError::CoordinateReplayMismatch { coordinate });
            }
        }
        for (index, value) in self.indices.iter().copied().enumerate() {
            let triangle = mesh.triangles()[index / 3].0[index % 3];
            if value != triangle {
                return Err(ApproximateMeshF64ViewError::IndexReplayMismatch { index });
            }
        }
        Ok(())
    }

    /// Classify whether this approximate view is fresh for `mesh`.
    pub fn freshness_against_mesh(&self, mesh: &ExactMesh) -> ApproximateMeshF64ViewFreshness {
        match self.validate_against_mesh(mesh) {
            Ok(()) => ApproximateMeshF64ViewFreshness::Current,
            Err(ApproximateMeshF64ViewError::Audit(_))
            | Err(ApproximateMeshF64ViewError::AuditReplay(_)) => {
                ApproximateMeshF64ViewFreshness::StaleAudit
            }
            Err(ApproximateMeshF64ViewError::MissingLossyViewFlag) => {
                ApproximateMeshF64ViewFreshness::MissingLossyFlag
            }
            Err(ApproximateMeshF64ViewError::PositionCountMismatch { .. })
            | Err(ApproximateMeshF64ViewError::IndexCountMismatch { .. })
            | Err(ApproximateMeshF64ViewError::ExportedCoordinateCountMismatch { .. }) => {
                ApproximateMeshF64ViewFreshness::StaleRows
            }
            Err(ApproximateMeshF64ViewError::NonFiniteCoordinate { .. }) => {
                ApproximateMeshF64ViewFreshness::NonFiniteCoordinate
            }
            Err(ApproximateMeshF64ViewError::CoordinateExportFailed { .. }) => {
                ApproximateMeshF64ViewFreshness::CoordinateExportFailed
            }
            Err(ApproximateMeshF64ViewError::CoordinateReplayMismatch { .. }) => {
                ApproximateMeshF64ViewFreshness::StaleCoordinate
            }
            Err(ApproximateMeshF64ViewError::IndexReplayMismatch { .. }) => {
                ApproximateMeshF64ViewFreshness::StaleIndex
            }
        }
    }
}

/// Build a primitive-float approximate view from an exact mesh.
pub fn approximate_mesh_f64_view(
    mesh: &ExactMesh,
) -> Result<ApproximateMeshF64View, ApproximateMeshF64ViewError> {
    ApproximateMeshF64View::from_mesh(mesh)
}

fn point_coordinate(point: &Point3, coordinate: usize) -> &Real {
    match coordinate {
        0 => &point.x,
        1 => &point.y,
        2 => &point.z,
        _ => unreachable!("validated 3D coordinate lane"),
    }
}
