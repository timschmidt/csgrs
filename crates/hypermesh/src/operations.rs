//! Public boolean operation entry points.

use hyperlattice::{Point3, Real};

use crate::error::HypermeshResult;
use crate::geometry::{Aabb, axis_mut, axis_ref};
use crate::mesh::{MeshRef, prepare_input};
use crate::output::{BooleanResult, certify_output_polygon_closure};
use crate::subdivision::{SubdivisionConfig, SubdivisionTask, subdivide_for_operation};
use crate::winding::{BooleanOp, make_indicator};

/// Configuration for boolean operations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EmberConfig {
    /// Maximum recursive subdivision depth, or `usize::MAX` for no
    /// caller-selected limit.
    ///
    /// Reaching this bound is not treated as implicit success. If the current
    /// task has not certified as a complete leaf and an exact root-basis
    /// arrangement split remains, the operation fails with
    /// `HypermeshError::SubdivisionDepthLimit`.
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
    crate::trace_dispatch!("boolean-operation", "start");
    validate_mesh_refs(meshes)?;
    let result = boolean_operation_general(meshes, op, config)?;
    crate::trace_dispatch!("boolean-operation", "certify-output-closure");
    certify_output_polygon_closure(&result)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(result)
}

fn boolean_operation_general(
    meshes: &[MeshRef<'_>],
    op: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    crate::trace_dispatch!("boolean-operation", "prepare-input");
    let soup = prepare_input(meshes)?;

    let process_bounds = expanded_bounds(&soup.bounds);
    let ref_point = outside_reference_point(&process_bounds);
    let ref_wnv = vec![0; soup.num_meshes];
    let indicator = make_indicator(op, soup.num_meshes);
    crate::trace_dispatch!("boolean-operation", "subdivide");
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
    let mut point = Point3::new(bounds.midpoint(0), bounds.midpoint(1), bounds.midpoint(2));
    *axis_mut(&mut point, 0) = axis_ref(&bounds.min, 0) - &one;
    point
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn outside_reference_point_uses_exterior_face_center() {
        let bounds = Aabb::new(p(0, 2, 4), p(10, 8, 14));
        let point = outside_reference_point(&bounds);

        assert_eq!(point, p(-1, 5, 9));
    }

    #[test]
    fn default_config_uses_finite_split_basis_without_a_depth_budget() {
        assert_eq!(EmberConfig::default().max_depth, usize::MAX);
    }
}
