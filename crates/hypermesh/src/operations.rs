//! Public boolean operation entry points.

use std::sync::{Arc, OnceLock};

use hyperlattice::{Point3, Real};

use crate::error::HypermeshResult;
use crate::geometry::{Aabb, axis_mut, axis_ref};
use crate::mesh::{MeshRef, prepare_input};
use crate::output::{BooleanResult, certify_output_polygon_closure};
use crate::subdivision::{SubdivisionConfig, SubdivisionTask};
use crate::winding::{BooleanOp, make_indicator};

const ALL_BOOLEAN_OPERATIONS: [BooleanOp; 4] = [
    BooleanOp::Union,
    BooleanOp::Intersection,
    BooleanOp::Difference,
    BooleanOp::SymmetricDifference,
];

/// A certified mesh arrangement that can be extracted for multiple Boolean
/// operations without repeating input preparation, intersection, BSP, or
/// winding classification work.
#[derive(Clone, Debug)]
pub struct BooleanArrangement {
    soup: crate::mesh::PolygonSoup,
    classified: Vec<crate::output::ClassifiedPolygon>,
    supported_operations: Vec<BooleanOp>,
    extraction_cache: Arc<ExtractionCache>,
}

#[derive(Debug, Default)]
struct ExtractionCache {
    results: [OnceLock<HypermeshResult<Arc<BooleanResult>>>; 4],
    triangle_soups: [OnceLock<HypermeshResult<Arc<crate::output::TriangleSoup>>>; 4],
}

impl PartialEq for BooleanArrangement {
    fn eq(&self, other: &Self) -> bool {
        self.soup == other.soup
            && self.classified == other.classified
            && self.supported_operations == other.supported_operations
    }
}

impl BooleanArrangement {
    /// Extracts and closure-certifies one Boolean operation from this
    /// arrangement's stored front/back winding evidence.
    pub fn extract(&self, op: BooleanOp) -> HypermeshResult<BooleanResult> {
        self.cached_extract(op)
            .map(|result| result.as_ref().clone())
    }

    fn cached_extract(&self, op: BooleanOp) -> HypermeshResult<Arc<BooleanResult>> {
        self.extraction_cache.results[boolean_operation_index(op)]
            .get_or_init(|| self.extract_uncached(op).map(Arc::new))
            .clone()
    }

    fn extract_uncached(&self, op: BooleanOp) -> HypermeshResult<BooleanResult> {
        let result = self.select_result(op)?;
        certify_output_polygon_closure(&result)?;
        Ok(result)
    }

    fn select_result(&self, op: BooleanOp) -> HypermeshResult<BooleanResult> {
        if !self.supported_operations.contains(&op) {
            return Err(crate::error::HypermeshError::UnsupportedBooleanExtraction);
        }
        let indicator = make_indicator(op, self.soup.num_meshes);
        let mut selected = Vec::new();
        for polygon in &self.classified {
            let winding = polygon
                .winding()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let classification = crate::winding::classify_polygon_output(
                &winding.w_front,
                &winding.w_back,
                &indicator,
            );
            if classification != 0 {
                let mut polygon = polygon.clone();
                polygon.classification = classification;
                selected.push(polygon);
            }
        }
        Ok(BooleanResult::from_classified(self.soup.clone(), selected))
    }

    /// Extracts one Boolean operation directly as a closure-certified triangle
    /// soup.
    ///
    /// This preserves both polygon-arrangement and final triangle-soup
    /// certification while avoiding a redundant second polygon closure pass
    /// between the two stages.
    pub fn extract_triangle_soup(
        &self,
        op: BooleanOp,
    ) -> HypermeshResult<Arc<crate::output::TriangleSoup>> {
        self.extraction_cache.triangle_soups[boolean_operation_index(op)]
            .get_or_init(|| {
                let result = if let Some(result) =
                    self.extraction_cache.results[boolean_operation_index(op)].get()
                {
                    result.clone()?
                } else {
                    Arc::new(self.select_result(op)?)
                };
                crate::output::triangulate_and_resolve_polygon_certified(&result).map(Arc::new)
            })
            .clone()
    }

    /// Returns the number of certified arrangement fragments retained for
    /// subsequent extraction.
    pub fn fragment_count(&self) -> usize {
        self.classified.len()
    }
}

const fn boolean_operation_index(operation: BooleanOp) -> usize {
    match operation {
        BooleanOp::Union => 0,
        BooleanOp::Intersection => 1,
        BooleanOp::Difference => 2,
        BooleanOp::SymmetricDifference => 3,
    }
}

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
    let prepared = prepare_boolean_operations(meshes, &[op], config)?;
    crate::trace_dispatch!("boolean-operation", "certify-output-closure");
    let result = prepared.extract(op)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(result)
}

/// Builds a certified arrangement once for extraction under multiple Boolean
/// operations.
///
/// This is the all-operation convenience form of
/// [`prepare_boolean_operations`]. [`boolean_operation`] uses the same prepared
/// pipeline with a one-operation scope, retaining its operation-specific
/// pruning without maintaining a separate execution path.
pub fn build_boolean_arrangement(
    meshes: &[MeshRef<'_>],
    config: EmberConfig,
) -> HypermeshResult<BooleanArrangement> {
    prepare_boolean_operations(meshes, &ALL_BOOLEAN_OPERATIONS, config)
}

/// Prepares a certified arrangement for exactly the requested Boolean
/// operations.
///
/// A single-operation preparation retains operation-specific winding
/// reachability pruning. Multi-operation preparation retains the transition
/// evidence needed to extract every requested result without repeating input
/// preparation, intersection, BSP, or winding classification work.
pub fn prepare_boolean_operations(
    meshes: &[MeshRef<'_>],
    operations: &[BooleanOp],
    config: EmberConfig,
) -> HypermeshResult<BooleanArrangement> {
    prepare_boolean_operations_with_certified_convex_inputs(
        meshes,
        operations,
        &vec![false; meshes.len()],
        config,
    )
}

/// Prepares Boolean operations while accepting exact convex-input
/// certificates supplied by the mesh owner.
///
/// A `true` entry certifies that the corresponding input is one closed,
/// non-self-intersecting, outward-oriented convex shell. Its triangulation
/// needs no self-arrangement cuts, its face-front winding is zero, and exact
/// support-plane tests may classify points against it. Cross-input
/// intersections and every output certification remain exact.
pub fn prepare_boolean_operations_with_certified_convex_inputs(
    meshes: &[MeshRef<'_>],
    operations: &[BooleanOp],
    certified_convex_inputs: &[bool],
    config: EmberConfig,
) -> HypermeshResult<BooleanArrangement> {
    if operations.is_empty() {
        return Err(crate::error::HypermeshError::EmptyBooleanOperationSet);
    }
    if certified_convex_inputs.len() != meshes.len() {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    validate_mesh_refs(meshes)?;
    let supported_operations = ALL_BOOLEAN_OPERATIONS
        .into_iter()
        .filter(|operation| operations.contains(operation))
        .collect::<Vec<_>>();
    let mut soup = prepare_input(meshes)?;
    let process_bounds = expanded_bounds(&soup.bounds);
    let ref_point = outside_reference_point(&process_bounds);
    let ref_wnv = vec![0; soup.num_meshes];
    let classified = crate::subdivision::subdivide_prepared_with_certified_convex_inputs(
        SubdivisionTask::new(
            std::mem::take(&mut soup.polygons),
            process_bounds,
            ref_point,
            ref_wnv,
        ),
        &supported_operations,
        certified_convex_inputs,
        SubdivisionConfig {
            max_depth: config.max_depth,
        },
    )?;
    Ok(BooleanArrangement {
        soup,
        classified,
        supported_operations,
        extraction_cache: Arc::new(ExtractionCache::default()),
    })
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
