//! Mesh-facing adapters for shared exact support-vertex k-DOP bounds.
//!
//! The witnessed support-DOP carrier lives in `hyperlimit`; this module keeps
//! only the mesh-source expansion policy and the public hypermesh names.

use hyperlimit::Point3;

use super::mesh::ExactMesh;
use super::provenance::MeshSource;

pub use hyperlimit::{
    SupportDopAxis3, SupportDopExpansionKind, SupportDopExpansionReport, SupportDopRefreshReport,
    SupportDopValidationError, SupportWitness3, WitnessedSupportDop3, WitnessedSupportSlab3,
};

/// One min/max support slab of a k-DOP.
pub type SupportSlab3 = WitnessedSupportSlab3;

/// Exact k-DOP bounds with support-vertex witnesses.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDop3 {
    /// Number of points summarized by this object.
    pub vertex_count: usize,
    /// One min/max support slab per retained axis.
    pub slabs: Vec<SupportSlab3>,
    /// Conservative adapter/rounding expansion metadata.
    pub expansion: SupportDopExpansionReport,
}

impl SupportDop3 {
    /// Build a support-DOP from exact points and an explicit expansion report.
    pub fn from_points_with_expansion(
        points: &[Point3],
        axes: &[SupportDopAxis3],
        expansion: SupportDopExpansionReport,
    ) -> Result<Self, SupportDopValidationError> {
        WitnessedSupportDop3::from_points_with_expansion(points, axes, expansion).map(Self::from)
    }

    /// Build an exact no-expansion support-DOP from points.
    pub fn from_points(
        points: &[Point3],
        axes: &[SupportDopAxis3],
    ) -> Result<Self, SupportDopValidationError> {
        WitnessedSupportDop3::from_points(points, axes).map(Self::from)
    }

    /// Build a support-DOP from an [`ExactMesh`] and retain the source-derived
    /// adapter expansion report.
    pub fn from_mesh(
        mesh: &ExactMesh,
        axes: &[SupportDopAxis3],
    ) -> Result<Self, SupportDopValidationError> {
        support_dop_for_mesh(mesh, axes)
    }

    /// Validate this k-DOP against exact source points.
    pub fn validate_against_points(
        &self,
        points: &[Point3],
    ) -> Result<(), SupportDopValidationError> {
        self.to_shared().validate_against_points(points)
    }

    /// Validate this k-DOP against the current exact mesh.
    pub fn validate_against_mesh(&self, mesh: &ExactMesh) -> Result<(), SupportDopValidationError> {
        let points = mesh.vertices().iter().cloned().collect::<Vec<Point3>>();
        self.validate_against_points(&points)
    }

    /// Refresh slabs after a bounded set of point updates.
    pub fn refresh_for_changed_vertices(
        &mut self,
        points: &[Point3],
        changed_vertices: &[usize],
    ) -> Result<SupportDopRefreshReport, SupportDopValidationError> {
        let mut shared = self.to_shared();
        let report = shared.refresh_for_changed_vertices(points, changed_vertices)?;
        *self = Self::from(shared);
        Ok(report)
    }

    fn to_shared(&self) -> WitnessedSupportDop3 {
        WitnessedSupportDop3 {
            vertex_count: self.vertex_count,
            slabs: self.slabs.clone(),
            expansion: self.expansion.clone(),
        }
    }
}

impl From<WitnessedSupportDop3> for SupportDop3 {
    fn from(shared: WitnessedSupportDop3) -> Self {
        Self {
            vertex_count: shared.vertex_count,
            slabs: shared.slabs,
            expansion: shared.expansion,
        }
    }
}

/// Build support-DOP bounds for an exact mesh.
pub fn support_dop_for_mesh(
    mesh: &ExactMesh,
    axes: &[SupportDopAxis3],
) -> Result<SupportDop3, SupportDopValidationError> {
    let points = mesh.vertices().iter().cloned().collect::<Vec<Point3>>();
    let expansion =
        support_dop_expansion_for_mesh_source(mesh.provenance().source.source, axes.len());
    WitnessedSupportDop3::from_points_with_expansion(&points, axes, expansion)
        .map(SupportDop3::from)
}

fn support_dop_expansion_for_mesh_source(
    source: MeshSource,
    axis_count: usize,
) -> SupportDopExpansionReport {
    match source {
        MeshSource::Exact => SupportDopExpansionReport::exact(axis_count),
        MeshSource::LossyF64 | MeshSource::HypermeshAdapter | MeshSource::ExternalAdapter => {
            SupportDopExpansionReport::lossy_adapter(axis_count)
        }
    }
}
