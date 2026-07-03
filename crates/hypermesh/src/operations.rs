//! Public boolean operation entry points.

use hyperlattice::{Point3, Real};

use crate::error::HypermeshResult;
use crate::geometry::{Aabb, axis_mut, axis_ref};
use crate::mesh::{InputMesh, MeshRef, prepare_input_refs};
use crate::output::{BooleanResult, triangulate_and_resolve_certified};
use crate::subdivision::{SubdivisionConfig, SubdivisionTask, subdivide};
use crate::winding::{BooleanOp, make_indicator};

/// Configuration for boolean operations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EmberConfig {
    /// Polygon-count threshold for leaf processing.
    pub leaf_threshold: usize,
    /// Maximum recursive subdivision depth.
    pub max_depth: usize,
    /// Assume every source mesh has no self-intersections.
    pub assume_nsi: bool,
    /// Assume every source mesh has no nested components.
    pub assume_nnc: bool,
}

impl Default for EmberConfig {
    fn default() -> Self {
        Self {
            leaf_threshold: crate::subdivision::DEFAULT_LEAF_THRESHOLD,
            max_depth: crate::subdivision::DEFAULT_MAX_DEPTH,
            assume_nsi: false,
            assume_nnc: false,
        }
    }
}

/// Performs a boolean operation on borrowed mesh views.
pub fn boolean_operation_refs(
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
    let mut soup = prepare_input_refs(meshes)?;
    for polygon in &mut soup.polygons {
        if config.assume_nsi {
            polygon.no_self_intersections = true;
        }
        if config.assume_nnc {
            polygon.no_nested_components = true;
        }
    }

    let process_bounds = expanded_bounds(&soup.bounds);
    let ref_point = outside_reference_point(&process_bounds);
    let ref_wnv = vec![0; soup.num_meshes];
    let indicator = make_indicator(op, soup.num_meshes);
    let classified = subdivide(
        SubdivisionTask::new(soup.polygons.clone(), process_bounds, ref_point, ref_wnv),
        &indicator,
        SubdivisionConfig {
            leaf_threshold: config.leaf_threshold,
            max_depth: config.max_depth,
        },
    )?;

    Ok(BooleanResult::from_classified(soup, classified))
}

fn validate_mesh_refs(meshes: &[MeshRef<'_>]) -> HypermeshResult<()> {
    if meshes.iter().all(|mesh| mesh.positions.is_empty()) {
        return Err(crate::error::HypermeshError::EmptyInput);
    }

    for mesh in meshes {
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

/// Performs a boolean operation on owned mesh values through the borrowed API.
pub fn boolean_operation(
    meshes: &[InputMesh],
    op: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    let refs = meshes.iter().map(InputMesh::as_ref).collect::<Vec<_>>();
    boolean_operation_refs(&refs, op, config)
}

/// Borrowed union convenience wrapper.
pub fn boolean_union_refs(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation_refs(&[a, b], BooleanOp::Union, config)
}

/// Owned union convenience wrapper.
pub fn boolean_union(
    a: &InputMesh,
    b: &InputMesh,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_union_refs(a.as_ref(), b.as_ref(), config)
}

/// Borrowed intersection convenience wrapper.
pub fn boolean_intersection_refs(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation_refs(&[a, b], BooleanOp::Intersection, config)
}

/// Owned intersection convenience wrapper.
pub fn boolean_intersection(
    a: &InputMesh,
    b: &InputMesh,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_intersection_refs(a.as_ref(), b.as_ref(), config)
}

/// Borrowed difference convenience wrapper.
pub fn boolean_difference_refs(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation_refs(&[a, b], BooleanOp::Difference, config)
}

/// Owned difference convenience wrapper.
pub fn boolean_difference(
    a: &InputMesh,
    b: &InputMesh,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_difference_refs(a.as_ref(), b.as_ref(), config)
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
