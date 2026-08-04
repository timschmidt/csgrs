//! Compact exact Boolean output carriers and topology evidence.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use hyperlattice::Point3;

use crate::error::{HypermeshError, HypermeshResult};
use crate::mesh::{Triangle, TriangleMesh};

/// Input triangle that contributed an output triangle.
#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct TriangleSource {
    /// Source mesh index.
    pub mesh: isize,
    /// Global source triangle index across the ordered input mesh streams.
    pub triangle: isize,
    /// `+1` when output orientation matches the source and `-1` when inverted.
    pub orientation: i8,
}

/// One boundary selected from a shared exact surface arrangement.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BooleanMeshResult {
    /// Compact indices into [`BooleanMeshBatch::vertices`].
    pub triangles: Vec<[u32; 3]>,
    /// Source provenance parallel to `triangles`.
    pub sources: Vec<TriangleSource>,
    /// Whether the unbounded exterior cell belongs to this result.
    ///
    /// Such a result has a finite oriented boundary, but cannot be represented
    /// by a standalone finite [`TriangleMesh`].
    pub exterior_inside: bool,
}

/// Boolean boundaries materialized from one shared exact arrangement.
///
/// Every result indexes the same compact vertex arena, so requesting several
/// expressions does not duplicate exact coordinate storage.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BooleanMeshBatch {
    /// Exact vertices used by at least one requested result.
    pub vertices: Vec<Point3>,
    /// Requested results in program-root order.
    pub results: Vec<BooleanMeshResult>,
}

impl BooleanMeshBatch {
    /// Checks carrier dimensions and compact indices without evaluating any
    /// geometric predicates.
    pub fn validate(&self) -> HypermeshResult<()> {
        for result in &self.results {
            if result.triangles.len() != result.sources.len() {
                return Err(HypermeshError::TriangleSourceCountMismatch {
                    triangles: result.triangles.len(),
                    sources: result.sources.len(),
                });
            }
            if let Some(&index) = result
                .triangles
                .iter()
                .flatten()
                .find(|&&index| index as usize >= self.vertices.len())
            {
                return Err(HypermeshError::VertexIndexOutOfBounds {
                    index: index as usize,
                    vertex_count: self.vertices.len(),
                });
            }
        }
        Ok(())
    }

    /// Consumes bounded results as reusable triangle meshes while sharing the
    /// exact position allocation across every returned mesh.
    ///
    /// Source provenance remains available only on this batch and is discarded
    /// by this explicit carrier conversion.
    pub fn into_triangle_meshes(self) -> HypermeshResult<Vec<TriangleMesh>> {
        self.validate()?;
        for (output, result) in self.results.iter().enumerate() {
            if result.exterior_inside {
                return Err(HypermeshError::UnboundedBooleanOutput { output });
            }
        }

        let positions: Arc<[Point3]> = self.vertices.into();
        let mut meshes = Vec::new();
        meshes.try_reserve_exact(self.results.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "Boolean triangle mesh results",
            }
        })?;
        for result in self.results {
            let mut triangles = Vec::new();
            triangles
                .try_reserve_exact(result.triangles.len())
                .map_err(|_| HypermeshError::CapacityOverflow {
                    operation: "Boolean triangle mesh indices",
                })?;
            triangles.extend(
                result
                    .triangles
                    .into_iter()
                    .map(|[a, b, c]| Triangle::new(a as usize, b as usize, c as usize)),
            );
            meshes.push(TriangleMesh::from_shared_positions(
                Arc::clone(&positions),
                triangles,
            ));
        }
        Ok(meshes)
    }
}

/// Exact topological closure summary for one Boolean result.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BooleanMeshClosureEvidence {
    /// Number of undirected edges used by exactly one triangle.
    pub boundary_edges: usize,
    /// Number of edge classes whose forward and reverse uses do not cancel.
    pub unbalanced_edges: usize,
    /// Number of undirected edges used by more than two triangles.
    pub non_manifold_edges: usize,
    /// Number of triangles with repeated vertex indices.
    pub degenerate_triangles: usize,
    /// Number of unoriented triangle-index triples repeated after their first use.
    pub duplicate_triangles: usize,
}

impl BooleanMeshClosureEvidence {
    /// Returns true when the boundary is directionally balanced and every
    /// triangle is a distinct nondegenerate index triple.
    pub const fn has_no_boundary(self) -> bool {
        self.boundary_edges == 0
            && self.unbalanced_edges == 0
            && self.degenerate_triangles == 0
            && self.duplicate_triangles == 0
    }

    /// Returns true when every edge has exactly two opposite uses and every
    /// triangle is a distinct nondegenerate index triple.
    pub const fn is_closed(self) -> bool {
        self.has_no_boundary() && self.non_manifold_edges == 0
    }
}

/// Computes allocation-free-in-the-result closure evidence from compact exact
/// topology. The arrangement materializer performs the stronger predicate-
/// bearing nondegeneracy certificate before constructing this carrier.
pub fn boolean_mesh_closure_evidence(result: &BooleanMeshResult) -> BooleanMeshClosureEvidence {
    let mut evidence = BooleanMeshClosureEvidence::default();
    let mut edge_uses = BTreeMap::<[u32; 2], [usize; 2]>::new();
    let mut triangles = BTreeSet::<[u32; 3]>::new();

    for &triangle in &result.triangles {
        if triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[0] == triangle[2] {
            evidence.degenerate_triangles += 1;
            continue;
        }
        let mut key = triangle;
        key.sort_unstable();
        if !triangles.insert(key) {
            evidence.duplicate_triangles += 1;
        }
        for edge in [
            [triangle[0], triangle[1]],
            [triangle[1], triangle[2]],
            [triangle[2], triangle[0]],
        ] {
            let canonical = if edge[0] < edge[1] {
                edge
            } else {
                [edge[1], edge[0]]
            };
            edge_uses.entry(canonical).or_default()[usize::from(edge != canonical)] += 1;
        }
    }

    for uses in edge_uses.values() {
        let total = uses[0] + uses[1];
        if total == 1 {
            evidence.boundary_edges += 1;
        } else if total > 2 {
            evidence.non_manifold_edges += 1;
        }
        if uses[0] != uses[1] {
            evidence.unbalanced_edges += 1;
        }
    }
    evidence
}

/// Returns whether one Boolean result is an exactly balanced two-manifold
/// index boundary.
pub fn boolean_mesh_is_closed(result: &BooleanMeshResult) -> bool {
    boolean_mesh_closure_evidence(result).is_closed()
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperlattice::Real;

    fn point(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn bounded_results_convert_with_one_shared_position_allocation() {
        let source = TriangleSource {
            mesh: 0,
            triangle: 0,
            orientation: 1,
        };
        let batch = BooleanMeshBatch {
            vertices: vec![
                point(0, 0, 0),
                point(1, 0, 0),
                point(0, 1, 0),
                point(0, 0, 1),
            ],
            results: vec![
                BooleanMeshResult {
                    triangles: vec![[0, 1, 2]],
                    sources: vec![source],
                    exterior_inside: false,
                },
                BooleanMeshResult {
                    triangles: vec![[0, 3, 1]],
                    sources: vec![source],
                    exterior_inside: false,
                },
            ],
        };
        let meshes = batch.into_triangle_meshes().unwrap();
        assert!(Arc::ptr_eq(&meshes[0].positions, &meshes[1].positions));
        assert_eq!(meshes[0].triangles[0], Triangle::new(0, 1, 2));
        assert_eq!(meshes[1].triangles[0], Triangle::new(0, 3, 1));
    }

    #[test]
    fn conversion_rejects_every_malformed_carrier_path() {
        let vertices = vec![point(0, 0, 0), point(1, 0, 0), point(0, 1, 0)];
        let result = |triangles, sources, exterior_inside| BooleanMeshResult {
            triangles,
            sources,
            exterior_inside,
        };
        assert_eq!(
            BooleanMeshBatch {
                vertices: vertices.clone(),
                results: vec![result(Vec::new(), Vec::new(), true)],
            }
            .into_triangle_meshes()
            .unwrap_err(),
            HypermeshError::UnboundedBooleanOutput { output: 0 }
        );
        assert_eq!(
            BooleanMeshBatch {
                vertices: vertices.clone(),
                results: vec![result(vec![[0, 1, 2]], Vec::new(), false)],
            }
            .into_triangle_meshes()
            .unwrap_err(),
            HypermeshError::TriangleSourceCountMismatch {
                triangles: 1,
                sources: 0,
            }
        );
        assert_eq!(
            BooleanMeshBatch {
                vertices,
                results: vec![result(
                    vec![[0, 1, 3]],
                    vec![TriangleSource::default()],
                    false,
                )],
            }
            .into_triangle_meshes()
            .unwrap_err(),
            HypermeshError::VertexIndexOutOfBounds {
                index: 3,
                vertex_count: 3,
            }
        );
    }

    #[test]
    fn closure_evidence_reports_degenerate_duplicate_and_open_topology() {
        let closed = BooleanMeshResult {
            triangles: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
            sources: vec![TriangleSource::default(); 4],
            exterior_inside: false,
        };
        assert!(boolean_mesh_closure_evidence(&closed).is_closed());

        let malformed = BooleanMeshResult {
            triangles: vec![[0, 1, 2], [2, 1, 0], [0, 0, 1]],
            sources: vec![TriangleSource::default(); 3],
            exterior_inside: false,
        };
        let evidence = boolean_mesh_closure_evidence(&malformed);
        assert_eq!(evidence.duplicate_triangles, 1);
        assert_eq!(evidence.degenerate_triangles, 1);
        assert!(!evidence.has_no_boundary());
    }
}
