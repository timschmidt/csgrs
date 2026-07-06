//! Public boolean operation entry points.

use hyperlattice::{Point3, Real};

use crate::error::HypermeshResult;
use crate::geometry::{Aabb, axis_mut, axis_ref};
use crate::mesh::{MeshRef, prepare_input};
use crate::output::{BooleanResult, triangulate_and_resolve_certified};
use crate::subdivision::{SubdivisionConfig, SubdivisionTask, subdivide_for_operation};
use crate::winding::{BooleanOp, make_indicator};

/// Configuration for boolean operations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EmberConfig {
    /// Maximum recursive subdivision depth.
    pub max_depth: usize,
}

impl Default for EmberConfig {
    fn default() -> Self {
        Self {
            max_depth: crate::subdivision::DEFAULT_MAX_DEPTH,
        }
    }
}

/// Performs a boolean operation on borrowed mesh views.
pub fn boolean_operation(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    validate_mesh_refs(meshes)?;
    let result = boolean_operation_general(meshes, op, config)?;
    triangulate_and_resolve_certified(&result)?;
    Ok(result)
}

fn boolean_operation_general(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    let soup = prepare_input(meshes)?;

    let process_bounds = expanded_bounds(&soup.bounds);
    let ref_point = outside_reference_point(&process_bounds);
    let ref_wnv = vec![0; soup.num_meshes];
    let indicator = make_indicator(op, soup.num_meshes);
    let classified = subdivide_for_operation(
        SubdivisionTask::new(soup.polygons.clone(), process_bounds, ref_point, ref_wnv),
        &indicator,
        SubdivisionConfig {
            max_depth: config.max_depth,
        },
        op,
    )?;

    Ok(BooleanResult::from_classified(soup, classified))
}

fn validate_mesh_refs(meshes: &[MeshRef<'_>]) -> HypermeshResult<()> {
    if meshes.is_empty() {
        return Err(crate::error::HypermeshError::EmptyInput);
    }

    for (mesh_index, mesh) in meshes.iter().enumerate() {
        if mesh.positions.is_empty() || mesh.triangles.is_empty() {
            return Err(crate::error::HypermeshError::EmptyMesh { mesh_index });
        }
        for triangle in mesh.triangles {
            for index in triangle.indices() {
                if index >= mesh.positions.len() {
                    return Err(crate::error::HypermeshError::VertexIndexOutOfBounds {
                        index,
                        vertex_count: mesh.positions.len(),
                    });
                }
            }
        }
    }

    Ok(())
}

/// Union convenience wrapper.
pub fn boolean_union(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation(&[a, b], BooleanOp::Union, config)
}

/// Intersection convenience wrapper.
pub fn boolean_intersection(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation(&[a, b], BooleanOp::Intersection, config)
}

/// Difference convenience wrapper.
pub fn boolean_difference(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation(&[a, b], BooleanOp::Difference, config)
}

fn expanded_bounds(bounds: &Aabb) -> Aabb {
    let one = Real::one();
    Aabb::new(
        Point3::new(
            &bounds.min.x - &one,
            &bounds.min.y - &one,
            &bounds.min.z - &one,
        ),
        Point3::new(
            &bounds.max.x + &one,
            &bounds.max.y + &one,
            &bounds.max.z + &one,
        ),
    )
}

fn outside_reference_point(bounds: &Aabb) -> Point3 {
    let one = Real::one();
    let mut point = bounds.min.clone();
    for axis in 0..3 {
        *axis_mut(&mut point, axis) = axis_ref(&point, axis) - &one;
    }
    point
}
