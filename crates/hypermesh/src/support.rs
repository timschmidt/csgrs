//! Mesh-source expansion policy for shared exact support-vertex k-DOP bounds.

use hyperlimit::{
    MeshSource, SupportDopAxis3, SupportDopExpansionReport, SupportDopValidationError,
    WitnessedSupportDop3,
};

use super::mesh::ExactMesh;

/// Build support-DOP bounds for an exact mesh.
pub fn support_dop_for_mesh(
    mesh: &ExactMesh,
    axes: &[SupportDopAxis3],
) -> Result<WitnessedSupportDop3, SupportDopValidationError> {
    let points = mesh.vertices().to_vec();
    let expansion =
        support_dop_expansion_for_mesh_source(mesh.provenance().source.source, axes.len());
    WitnessedSupportDop3::from_points_with_expansion(&points, axes, expansion)
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
