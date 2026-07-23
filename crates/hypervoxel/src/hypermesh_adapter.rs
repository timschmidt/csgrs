//! Exact `hypermesh` input adapter.

use crate::{
    ExactTriangle3, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, HypervoxelError,
    HypervoxelResult,
};

/// Adapts a validated `hypermesh` closed solid into a triangle-solid carrier.
pub fn adapt_hypermesh_exact_solid(
    mesh: &hypermesh::InputMesh,
) -> HypervoxelResult<ExactTriangleSolidMesh> {
    hypermesh::prepare_input(&[mesh.as_ref()]).map_err(|_| {
        HypervoxelError::InvalidSourceGeometry {
            reason: "hypermesh input is not a validated closed solid",
        }
    })?;

    let mut triangles = Vec::with_capacity(mesh.triangles.len());
    for (face, triangle) in mesh.triangles.iter().enumerate() {
        let [ia, ib, ic] = triangle.indices();
        let a = mesh
            .positions
            .get(ia)
            .ok_or(HypervoxelError::InvalidSourceGeometry {
                reason: "triangle references a missing vertex",
            })?;
        let b = mesh
            .positions
            .get(ib)
            .ok_or(HypervoxelError::InvalidSourceGeometry {
                reason: "triangle references a missing vertex",
            })?;
        let c = mesh
            .positions
            .get(ic)
            .ok_or(HypervoxelError::InvalidSourceGeometry {
                reason: "triangle references a missing vertex",
            })?;
        triangles.push(ExactTriangle3::new(
            [
                [a.x.clone(), a.y.clone(), a.z.clone()],
                [b.x.clone(), b.y.clone(), b.z.clone()],
                [c.x.clone(), c.y.clone(), c.z.clone()],
            ],
            Some(face as u64),
        ));
    }

    if triangles.is_empty() {
        return Err(HypervoxelError::InvalidSourceGeometry {
            reason: "hypermesh input contains no triangles",
        });
    }

    Ok(ExactTriangleSolidMesh::new(
        ExactTriangleSurfaceMesh::new(triangles),
        true,
    ))
}
