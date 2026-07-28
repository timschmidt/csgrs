//! Direct exact voxelization geometry for retained BREP shells.

use hypervoxel::{
    ExactBox, ExactTriangle3, ExactTriangleSolid, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh,
};

use crate::topology::BrepShell;

/// Failure to construct exact voxel geometry from a retained BREP shell.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BrepVoxelError {
    BoundsUnavailable,
    TriangleMeshUnavailable,
    TriangleSolidConstructionFailed,
}

/// Lean geometry consumed by HyperVoxel broad and narrow phases.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepVoxelGeometry {
    pub exact_aabb: ExactBox,
    pub triangle_solid: ExactTriangleSolid,
}

impl BrepShell {
    /// Returns exact AABB and triangle-solid geometry for voxelization.
    pub fn voxel_geometry(&self) -> Result<BrepVoxelGeometry, BrepVoxelError> {
        let bounds = self.shell_bounds_report();
        if !bounds.exact_bounds_ready {
            return Err(BrepVoxelError::BoundsUnavailable);
        }
        let (min, max) = bounds
            .min
            .zip(bounds.max)
            .ok_or(BrepVoxelError::BoundsUnavailable)?;
        let exact_aabb = ExactBox::new([min.x, min.y, min.z], [max.x, max.y, max.z]);

        let mesh = self.exact_triangle_mesh_handoff_report();
        if !mesh.exact_triangle_mesh_ready {
            return Err(BrepVoxelError::TriangleMeshUnavailable);
        }
        let triangles = mesh
            .triangles
            .into_iter()
            .enumerate()
            .map(|(index, triangle)| {
                ExactTriangle3::new(
                    triangle.vertices.map(|point| [point.x, point.y, point.z]),
                    Some(index as u64),
                )
            })
            .collect();
        let solid = ExactTriangleSolidMesh::new(ExactTriangleSurfaceMesh::new(triangles), true);
        let triangle_solid = ExactTriangleSolid::new(solid)
            .map_err(|_| BrepVoxelError::TriangleSolidConstructionFailed)?;
        Ok(BrepVoxelGeometry {
            exact_aabb,
            triangle_solid,
        })
    }
}
