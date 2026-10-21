//! Compact exact Boolean output carriers and topology evidence.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Weak};

use hyperlattice::Point3;

use crate::ConstructionEvidence;
use crate::error::{HypermeshError, HypermeshResult};
use crate::mesh::{Triangle, TriangleMesh};
use crate::surface_reduction::RetainedPlaneClasses;

/// Input triangle whose plane carries an output triangle.
///
/// Boolean output merges coplanar pieces of the selected boundary and
/// retriangulates them without the vertices the set does not need, so one
/// output triangle can cover parts of several coplanar input triangles. It
/// then names the least of them by `(mesh, triangle)`. An output triangle
/// inside one input triangle names that triangle.
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
#[derive(Clone, Debug)]
pub struct BooleanMeshBatch {
    /// Exact vertices used by at least one requested result. Use `Arc::make_mut`
    /// to edit; doing so invalidates the retained construction identity.
    pub vertices: Arc<[Point3]>,
    /// Requested results in program-root order. Use `Arc::make_mut` to edit.
    pub results: Arc<[BooleanMeshResult]>,
    source_vertices: Weak<[Point3]>,
    source_results: Weak<[BooleanMeshResult]>,
    construction: ConstructionEvidence,
    // Set only for surface-arrangement output and honored only while both
    // buffers are unchanged: each result's distinct triangles then meet only
    // in shared vertex indices or a shared edge.
    embedded_complex: bool,
    // Per result, under the same condition: the plane classes of its
    // triangles, recorded only from certified arrangements.
    plane_classes: Vec<Option<Arc<RetainedPlaneClasses>>>,
}

impl Default for BooleanMeshBatch {
    fn default() -> Self {
        Self::new(Vec::new(), Vec::new())
    }
}

impl PartialEq for BooleanMeshBatch {
    fn eq(&self, other: &Self) -> bool {
        self.vertices == other.vertices && self.results == other.results
    }
}

impl BooleanMeshBatch {
    /// Constructs a raw batch with untracked source history. Carrier validation
    /// is deferred to [`Self::validate`] or [`Self::into_triangle_meshes`].
    pub fn new(vertices: Vec<Point3>, results: Vec<BooleanMeshResult>) -> Self {
        Self::from_construction(vertices, results, ConstructionEvidence::UNTRACKED)
    }

    pub(crate) fn from_construction(
        vertices: Vec<Point3>,
        results: Vec<BooleanMeshResult>,
        construction: ConstructionEvidence,
    ) -> Self {
        Self::from_parts(vertices, results, construction, false, Vec::new())
    }

    /// Output of one exact surface arrangement. Its facets meet only in
    /// shared arrangement vertices or edges, and every result selects a
    /// subset of those facets over one deduplicated vertex arena. Each
    /// result may retain the plane classes of its triangles.
    pub(crate) fn from_arrangement(
        vertices: Vec<Point3>,
        results: Vec<BooleanMeshResult>,
        construction: ConstructionEvidence,
        plane_classes: Vec<Option<Arc<RetainedPlaneClasses>>>,
    ) -> Self {
        Self::from_parts(vertices, results, construction, true, plane_classes)
    }

    fn from_parts(
        vertices: Vec<Point3>,
        results: Vec<BooleanMeshResult>,
        construction: ConstructionEvidence,
        embedded_complex: bool,
        plane_classes: Vec<Option<Arc<RetainedPlaneClasses>>>,
    ) -> Self {
        let vertices: Arc<[Point3]> = vertices.into();
        let results: Arc<[BooleanMeshResult]> = results.into();
        Self {
            source_vertices: Arc::downgrade(&vertices),
            source_results: Arc::downgrade(&results),
            vertices,
            results,
            construction,
            embedded_complex,
            plane_classes,
        }
    }

    fn buffers_unchanged(&self) -> bool {
        std::ptr::eq(self.source_vertices.as_ptr(), Arc::as_ptr(&self.vertices))
            && std::ptr::eq(self.source_results.as_ptr(), Arc::as_ptr(&self.results))
    }

    /// Retained qualifications, with missing history exposed after buffer edits.
    pub fn construction_evidence(&self) -> ConstructionEvidence {
        if self.buffers_unchanged() {
            self.construction
        } else {
            self.construction.after_buffer_mutation()
        }
    }

    pub(crate) fn with_inherited_construction(mut self, evidence: ConstructionEvidence) -> Self {
        self.construction = self.construction_evidence().combine(evidence);
        self
    }

    /// Checks carrier dimensions and compact indices without evaluating any
    /// geometric predicates.
    pub fn validate(&self) -> HypermeshResult<()> {
        for result in self.results.iter() {
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
    /// exact position allocation across the nonempty returned meshes.
    /// Empty results have neither positions nor triangles, so they remain
    /// canonical empty geometry even when other batch rows use vertices.
    ///
    /// Construction qualifications survive this conversion, including empty
    /// rows. Per-triangle source mappings remain available only on the batch;
    /// this summary is not a replacement for those mappings.
    pub fn into_triangle_meshes(self) -> HypermeshResult<Vec<TriangleMesh>> {
        self.validate()?;
        for (output, result) in self.results.iter().enumerate() {
            if result.exterior_inside {
                return Err(HypermeshError::UnboundedBooleanOutput { output });
            }
        }

        let evidence = self.construction_evidence();
        let embedded_complex = self.embedded_complex && self.buffers_unchanged();
        let plane_classes = self.plane_classes;
        let positions = self.vertices;
        let mut meshes = Vec::new();
        meshes.try_reserve_exact(self.results.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "Boolean triangle mesh results",
            }
        })?;
        for (index, result) in self.results.iter().enumerate() {
            if result.triangles.is_empty() {
                meshes.push(TriangleMesh::from_construction(
                    Vec::new(),
                    Vec::new(),
                    evidence,
                ));
                continue;
            }
            let mut triangles = Vec::new();
            triangles
                .try_reserve_exact(result.triangles.len())
                .map_err(|_| HypermeshError::CapacityOverflow {
                    operation: "Boolean triangle mesh indices",
                })?;
            triangles.extend(
                result
                    .triangles
                    .iter()
                    .map(|&[a, b, c]| Triangle::new(a as usize, b as usize, c as usize)),
            );
            let positions = Arc::clone(&positions);
            meshes.push(if embedded_complex {
                let classes = plane_classes
                    .get(index)
                    .cloned()
                    .flatten()
                    .filter(|classes| classes.len() == triangles.len());
                TriangleMesh::from_shared_embedded_complex(positions, triangles, evidence, classes)
            } else {
                TriangleMesh::from_shared_construction(positions, triangles, evidence)
            });
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
        let batch = BooleanMeshBatch::new(
            vec![
                point(0, 0, 0),
                point(1, 0, 0),
                point(0, 1, 0),
                point(0, 0, 1),
            ],
            vec![
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
        );
        let meshes = batch.into_triangle_meshes().unwrap();
        assert!(Arc::ptr_eq(&meshes[0].positions, &meshes[1].positions));
        assert_eq!(meshes[0].triangles[0], Triangle::new(0, 1, 2));
        assert_eq!(meshes[1].triangles[0], Triangle::new(0, 3, 1));
    }

    fn unit_box(offset: i64) -> TriangleMesh {
        let corner = |x: i64, y: i64, z: i64| point(x + offset, y, z);
        TriangleMesh::new(
            vec![
                corner(0, 0, 0),
                corner(1, 0, 0),
                corner(1, 1, 0),
                corner(0, 1, 0),
                corner(0, 0, 1),
                corner(1, 0, 1),
                corner(1, 1, 1),
                corner(0, 1, 1),
            ],
            [
                [4, 5, 6],
                [4, 6, 7],
                [0, 3, 2],
                [0, 2, 1],
                [1, 2, 6],
                [1, 6, 5],
                [0, 4, 7],
                [0, 7, 3],
                [3, 7, 6],
                [3, 6, 2],
                [0, 1, 5],
                [0, 5, 4],
            ]
            .into_iter()
            .map(|[a, b, c]| Triangle::new(a, b, c))
            .collect(),
        )
    }

    fn arrangement_union() -> BooleanMeshBatch {
        let context = crate::MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        crate::boolean(
            &context,
            &[unit_box(0).as_ref(), unit_box(1).as_ref()],
            crate::BooleanProgram::Operation(crate::BooleanOp::Union),
        )
        .unwrap()
        .into_value()
    }

    #[test]
    fn only_unedited_arrangement_output_retains_the_embedded_complex_fact() {
        let meshes = arrangement_union().into_triangle_meshes().unwrap();
        assert!(meshes[0].is_certified_embedded_complex());
        // Clones share the bound fact; transforms and edits build new buffers.
        assert!(meshes[0].clone().is_certified_embedded_complex());
        let mut edited = meshes[0].clone();
        Arc::make_mut(&mut edited.triangles).swap(0, 1);
        assert!(!edited.is_certified_embedded_complex());

        // Caller-assembled carriers make no structural claim, even when they
        // contain exactly the arrangement's rows.
        let batch = arrangement_union();
        let rebuilt = BooleanMeshBatch::new(batch.vertices.to_vec(), batch.results.to_vec());
        assert!(!rebuilt.into_triangle_meshes().unwrap()[0].is_certified_embedded_complex());
        let mut edited_batch = arrangement_union();
        Arc::make_mut(&mut edited_batch.results)[0]
            .triangles
            .swap(0, 1);
        assert!(!edited_batch.into_triangle_meshes().unwrap()[0].is_certified_embedded_complex());
        assert!(!unit_box(0).is_certified_embedded_complex());
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
            BooleanMeshBatch::new(vertices.clone(), vec![result(Vec::new(), Vec::new(), true)],)
                .into_triangle_meshes()
                .unwrap_err(),
            HypermeshError::UnboundedBooleanOutput { output: 0 }
        );
        assert_eq!(
            BooleanMeshBatch::new(
                vertices.clone(),
                vec![result(vec![[0, 1, 2]], Vec::new(), false)],
            )
            .into_triangle_meshes()
            .unwrap_err(),
            HypermeshError::TriangleSourceCountMismatch {
                triangles: 1,
                sources: 0,
            }
        );
        assert_eq!(
            BooleanMeshBatch::new(
                vertices,
                vec![result(
                    vec![[0, 1, 3]],
                    vec![TriangleSource::default()],
                    false,
                )],
            )
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
