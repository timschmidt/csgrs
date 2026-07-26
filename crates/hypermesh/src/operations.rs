//! Public boolean operation entry points.

use hyperlattice::{
    HomogeneousPoint3, Point3, Rational, Real, homogeneous_point_plane_expression,
    intersect_three_planes,
};
use hyperreal::PreparedRationalLinearForm4Query;

use crate::error::HypermeshResult;
use crate::geometry::{
    Aabb, Classification, Plane, axis_mut, axis_ref, classify_point, classify_projective_point,
};
use crate::mesh::{
    MeshRef, build_polygon_soup_with_certified_convex_inputs,
    build_polygon_soup_with_deferred_edges,
};
use crate::output::{
    ARRANGEMENT_CLASSIFICATION, BooleanResult, ClassifiedPolygon, certify_output_polygon_closure,
};
use crate::polygon::{
    ConstructionEdgeIdentity, ConstructionPlaneIdentity, ConstructionVertexIdentity, ConvexPolygon,
    InputTrianglePlanes, KnownEdgeIdentityCycle, edge_plane,
};
use crate::predicate::{PreparedProjectivePoint3, PreparedRationalPlane4};
use crate::storage_hash::StorageHashMap;
use crate::subdivision::{SubdivisionConfig, SubdivisionTask};
use crate::winding::{BooleanOp, WindingPair, make_indicator};

struct BooleanComputation {
    soup: crate::mesh::PolygonSoup,
    classified: Vec<crate::output::ClassifiedPolygon>,
    triangle_soup: Option<crate::output::TriangleSoup>,
    input_edges_deferred: bool,
}

struct ConvexCandidate {
    classified: Vec<ClassifiedPolygon>,
    triangle_soup: crate::output::TriangleSoup,
}

impl BooleanComputation {
    fn into_result(self, operation: BooleanOp) -> HypermeshResult<BooleanResult> {
        let result = self.into_selected_result(operation)?;
        certify_output_polygon_closure(&result)?;
        Ok(result)
    }

    fn into_selected_result(self, operation: BooleanOp) -> HypermeshResult<BooleanResult> {
        let indicator = make_indicator(operation, self.soup.num_meshes);
        let mut selected = Vec::with_capacity(self.classified.len());
        for mut polygon in self.classified {
            let winding = polygon
                .winding()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let classification = crate::winding::classify_polygon_output(
                &winding.w_front,
                &winding.w_back,
                &indicator,
            );
            if classification != 0 {
                polygon.classification = classification;
                if self.input_edges_deferred {
                    polygon.polygon = polygon.polygon.with_rebuilt_edge_planes()?;
                }
                selected.push(polygon);
            }
        }
        Ok(BooleanResult::from_classified(self.soup, selected))
    }

    fn into_triangle_soup(
        self,
        operation: BooleanOp,
    ) -> HypermeshResult<crate::output::TriangleSoup> {
        if let Some(soup) = self.triangle_soup {
            return Ok(soup);
        }
        let result = self.into_selected_result(operation)?;
        crate::output::triangulate_and_resolve_polygon_certified(&result)
    }
}

fn select_triangle_arrangement(
    arrangement: &crate::output::ClassifiedTriangleArrangement,
    op: BooleanOp,
    num_meshes: usize,
) -> HypermeshResult<crate::output::TriangleSoup> {
    if arrangement.soup.triangles.len() != arrangement.windings.len()
        || arrangement.soup.triangles.len() != arrangement.soup.sources.len()
    {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    let indicator = make_indicator(op, num_meshes);
    let mut triangles = Vec::new();
    let mut sources = Vec::new();
    for ((triangle, source), winding) in arrangement
        .soup
        .triangles
        .iter()
        .zip(&arrangement.soup.sources)
        .zip(&arrangement.windings)
    {
        let classification =
            crate::winding::classify_polygon_output(&winding.w_front, &winding.w_back, &indicator);
        if classification == 0 {
            continue;
        }
        let mut triangle = *triangle;
        if classification == -1 {
            triangle.swap(1, 2);
        }
        let mut source = *source;
        source.orientation = classification;
        triangles.push(triangle);
        sources.push(source);
    }
    let soup = crate::output::TriangleSoup {
        vertices: arrangement.soup.vertices.clone(),
        triangles,
        sources,
    };
    certify_triangle_soup_closure(soup)
}

fn certify_triangle_soup_closure(
    soup: crate::output::TriangleSoup,
) -> HypermeshResult<crate::output::TriangleSoup> {
    let closure = crate::output::triangle_soup_closure_evidence(&soup);
    if !closure.has_no_boundary() {
        return Err(crate::error::HypermeshError::OpenOutput {
            boundary_edges: closure.boundary_edges,
            unbalanced_edges: closure.unbalanced_edges,
            non_manifold_edges: closure.non_manifold_edges,
        });
    }
    Ok(soup)
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
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    crate::trace_dispatch!("boolean-operation", "start");
    let computation = compute_boolean(meshes, operation, None, None, config)?;
    crate::trace_dispatch!("boolean-operation", "certify-output-closure");
    let result = computation.into_result(operation)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(result)
}

/// Performs a Boolean operation using exact convex-input facts supplied by
/// the mesh owner.
///
/// A `true` entry certifies that the corresponding input is one closed,
/// non-self-intersecting, outward-oriented convex shell. Cross-input
/// intersections and output closure remain exactly certified.
pub fn boolean_operation_with_certified_convex_inputs(
    meshes: &[MeshRef<'_>],
    operation: BooleanOp,
    certified_convex_inputs: &[bool],
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    crate::trace_dispatch!("boolean-operation", "start");
    let computation = compute_boolean(
        meshes,
        operation,
        Some(certified_convex_inputs),
        None,
        config,
    )?;
    crate::trace_dispatch!("boolean-operation", "certify-output-closure");
    let result = computation.into_result(operation)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(result)
}

/// Performs a Boolean operation and immediately returns a closure-certified
/// triangle soup.
///
/// This avoids materializing an intermediate polygon result when the caller
/// needs indexed triangles.
pub fn boolean_triangle_soup(
    meshes: &[MeshRef<'_>],
    operation: BooleanOp,
    config: EmberConfig,
) -> HypermeshResult<crate::output::TriangleSoup> {
    crate::trace_dispatch!("boolean-operation", "start");
    let computation = compute_boolean(meshes, operation, None, None, config)?;
    crate::trace_dispatch!("boolean-operation", "triangulate-output");
    let soup = computation.into_triangle_soup(operation)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(soup)
}

/// Performs a Boolean operation with exact convex-input facts and immediately
/// returns a closure-certified triangle soup.
///
/// This is the direct triangle-output counterpart of
/// [`boolean_operation_with_certified_convex_inputs`].
pub fn boolean_triangle_soup_with_certified_convex_inputs(
    meshes: &[MeshRef<'_>],
    operation: BooleanOp,
    certified_convex_inputs: &[bool],
    config: EmberConfig,
) -> HypermeshResult<crate::output::TriangleSoup> {
    crate::trace_dispatch!("boolean-operation", "start");
    let computation = compute_boolean(
        meshes,
        operation,
        Some(certified_convex_inputs),
        None,
        config,
    )?;
    crate::trace_dispatch!("boolean-operation", "triangulate-output");
    let soup = computation.into_triangle_soup(operation)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(soup)
}

/// Performs a Boolean operation with exact convex-input facts and exact
/// per-triangle plane certificates.
///
/// Each plane slice must align with the corresponding mesh's triangle slice.
/// The supplied oriented support and boundary planes are used instead of
/// reconstructing the same objects from transformed vertex coordinates,
/// preserving affine-transform identities at the geometric-object boundary.
pub fn boolean_triangle_soup_with_certified_convex_inputs_and_planes(
    meshes: &[MeshRef<'_>],
    operation: BooleanOp,
    certified_convex_inputs: &[bool],
    input_planes: &[&[InputTrianglePlanes]],
    config: EmberConfig,
) -> HypermeshResult<crate::output::TriangleSoup> {
    crate::trace_dispatch!("boolean-operation", "start");
    let computation = compute_boolean(
        meshes,
        operation,
        Some(certified_convex_inputs),
        Some(input_planes),
        config,
    )?;
    crate::trace_dispatch!("boolean-operation", "triangulate-output");
    let soup = computation.into_triangle_soup(operation)?;
    crate::trace_dispatch!("boolean-operation", "complete");
    Ok(soup)
}

fn compute_boolean(
    meshes: &[MeshRef<'_>],
    operation: BooleanOp,
    certified_convex_inputs: Option<&[bool]>,
    input_planes: Option<&[&[InputTrianglePlanes]]>,
    config: EmberConfig,
) -> HypermeshResult<BooleanComputation> {
    if certified_convex_inputs.is_some_and(|certified| certified.len() != meshes.len()) {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    let certified_convex_inputs = certified_convex_inputs.unwrap_or(&[]);
    let use_two_convex_candidate = meshes.len() == 2 && certified_convex_inputs == [true, true];
    let mut soup = if use_two_convex_candidate {
        build_polygon_soup_with_deferred_edges(meshes, certified_convex_inputs, input_planes)?
    } else if certified_convex_inputs.is_empty() {
        crate::mesh::build_polygon_soup(meshes)?
    } else {
        build_polygon_soup_with_certified_convex_inputs(
            meshes,
            certified_convex_inputs,
            input_planes,
        )?
    };
    let convex_candidate = if use_two_convex_candidate {
        match compute_two_convex_inputs_projectively(&soup.polygons, operation) {
            Ok(candidate) => candidate,
            Err(error) => {
                if cfg!(debug_assertions) {
                    eprintln!("[DEBUG] projective convex candidate failed: {error}");
                }
                None
            }
        }
    } else {
        None
    };
    let (classified, triangle_soup, input_edges_deferred) =
        if let Some(candidate) = convex_candidate {
            (candidate.classified, Some(candidate.triangle_soup), true)
        } else {
            if use_two_convex_candidate {
                soup = build_polygon_soup_with_certified_convex_inputs(
                    meshes,
                    certified_convex_inputs,
                    input_planes,
                )?;
            }
            let process_bounds = expanded_bounds(&soup.bounds);
            let ref_point = outside_reference_point(&process_bounds);
            let ref_wnv = vec![0; soup.num_meshes];
            (
                crate::subdivision::subdivide_boolean_with_certified_convex_inputs(
                    SubdivisionTask::new(
                        std::mem::take(&mut soup.polygons),
                        process_bounds,
                        ref_point,
                        ref_wnv,
                    ),
                    operation,
                    certified_convex_inputs,
                    SubdivisionConfig {
                        max_depth: config.max_depth,
                    },
                )?,
                None,
                false,
            )
        };
    Ok(BooleanComputation {
        soup,
        classified,
        triangle_soup,
        input_edges_deferred,
    })
}

#[derive(Clone)]
struct ProjectiveCycle {
    boundary: Vec<ProjectiveBoundaryEntry>,
    support_index: usize,
    source_plane: ConstructionPlaneIdentity,
    source_unchanged: bool,
}

#[derive(Clone)]
struct ProjectiveBoundaryEntry {
    // Clipping moves this complete incidence record atomically. Keeping the
    // point and outgoing edge evidence together also makes length skew between
    // parallel identity arrays unrepresentable.
    point_index: usize,
    preparation: ProjectivePointPreparation,
    point_identity: ConstructionVertexIdentity,
    // Exact edge planes are single-owned by the computation arena. Moving an
    // index with its identity preserves the complete oriented incidence record.
    edge_index: usize,
    edge_identity: ConstructionEdgeIdentity,
}

#[derive(Clone, Copy)]
struct ProjectivePointPreparation {
    approximate: Option<[f64; 3]>,
    rational_filter_query: Option<PreparedRationalLinearForm4Query>,
}

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
struct ProjectiveBoundaryPlaneIdentity {
    support: ConstructionPlaneIdentity,
    edge: ConstructionEdgeIdentity,
}

struct ProjectiveClip {
    negative: ProjectiveCycle,
    positive: ProjectiveCycle,
    side: ProjectiveClipSide,
}

struct ProjectiveBoundary {
    entries: Vec<ProjectiveBoundaryEntry>,
}

impl ProjectiveBoundary {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            entries: Vec::with_capacity(capacity),
        }
    }

    fn push(
        &mut self,
        point_index: usize,
        preparation: ProjectivePointPreparation,
        point_identity: ConstructionVertexIdentity,
        edge_index: usize,
        edge_identity: ConstructionEdgeIdentity,
        point_cache: &ProjectivePointCache,
    ) {
        // Approximation inequality is only a rejection filter: equal exact
        // points have the same retained finite view. Deduplication still
        // requires exact homogeneous equality below.
        let approximations_differ = self
            .entries
            .last()
            .and_then(|last| last.preparation.approximate)
            .zip(preparation.approximate)
            .is_some_and(|(last, current)| last != current);
        if !approximations_differ
            && self.entries.last().is_some_and(|last| {
                point_cache.point(last.point_index) == point_cache.point(point_index)
            })
        {
            let last = self
                .entries
                .last_mut()
                .expect("nonempty boundary has a last entry");
            last.edge_index = edge_index;
            last.edge_identity = edge_identity;
            return;
        }
        self.entries.push(ProjectiveBoundaryEntry {
            point_index,
            preparation,
            point_identity,
            edge_index,
            edge_identity,
        });
    }

    fn into_cycle(
        mut self,
        support_index: usize,
        source_plane: ConstructionPlaneIdentity,
        point_cache: &ProjectivePointCache,
    ) -> ProjectiveCycle {
        if self.entries.len() > 1
            && self
                .entries
                .first()
                .zip(self.entries.last())
                .is_some_and(|(first, last)| {
                    point_cache.point(first.point_index) == point_cache.point(last.point_index)
                })
        {
            self.entries.pop();
        }
        ProjectiveCycle {
            boundary: self.entries,
            support_index,
            source_plane,
            source_unchanged: false,
        }
    }
}

#[derive(Default)]
struct ProjectiveAffineCache {
    points: StorageHashMap<[usize; 4], ProjectiveAffineCacheEntry>,
    identities: StorageHashMap<ConstructionVertexIdentity, Point3>,
}

struct ProjectiveAffineCacheEntry {
    _coordinates: [Rational; 4],
    affine: Point3,
}

type SourceEdgeKey = [usize; 3];

struct ProjectiveSourceEdge {
    definition_planes: [usize; 2],
    supports: [ConstructionPlaneIdentity; 2],
    support_count: u8,
}

impl ProjectiveSourceEdge {
    fn new(definition_planes: [usize; 2], support: ConstructionPlaneIdentity) -> Self {
        Self {
            definition_planes,
            supports: [support; 2],
            support_count: 1,
        }
    }

    fn supports(&self) -> &[ConstructionPlaneIdentity] {
        &self.supports[..usize::from(self.support_count)]
    }

    fn insert_support(&mut self, support: ConstructionPlaneIdentity) -> bool {
        if self.supports().contains(&support) {
            return true;
        }
        if self.support_count == 2 {
            return false;
        }
        self.supports[usize::from(self.support_count)] = support;
        self.support_count += 1;
        true
    }
}

#[derive(Default)]
struct ProjectivePointCache {
    // Cycles retain stable indices instead of cloning 192-byte exact points
    // and planes. Identity remapping can therefore update a complete
    // coincidence class atomically, while split cycles share exact geometry.
    point_storage: Vec<HomogeneousPoint3>,
    plane_storage: Vec<Plane>,
    points: StorageHashMap<ConstructionVertexIdentity, CachedProjectivePoint>,
    canonical_identities: StorageHashMap<ConstructionVertexIdentity, ConstructionVertexIdentity>,
    canonical_planes: [Vec<ConstructionPlaneIdentity>; 2],
    planes: StorageHashMap<ConstructionPlaneIdentity, usize>,
    source_edges: StorageHashMap<SourceEdgeKey, ProjectiveSourceEdge>,
    boundary_planes: StorageHashMap<ProjectiveBoundaryPlaneIdentity, Vec<usize>>,
    inverted_planes: StorageHashMap<ConstructionPlaneIdentity, usize>,
    source_vertices: StorageHashMap<ConstructionVertexIdentity, [ConstructionPlaneIdentity; 3]>,
    point_incidences: StorageHashMap<ConstructionVertexIdentity, Vec<ConstructionPlaneIdentity>>,
}

struct CachedProjectivePoint {
    point_index: usize,
    approximate: Option<[f64; 3]>,
}

const PROJECTIVE_CROSSING_CACHE_MIN_POINTS: usize = 128;

struct AtomicDisjointSets {
    parents: Vec<usize>,
}

impl AtomicDisjointSets {
    fn new(len: usize) -> Self {
        Self {
            parents: (0..len).collect(),
        }
    }

    fn representative(&mut self, mut index: usize) -> usize {
        while self.parents[index] != index {
            let parent = self.parents[index];
            self.parents[index] = self.parents[parent];
            index = self.parents[index];
        }
        index
    }

    fn merge(&mut self, left: usize, right: usize) {
        let left = self.representative(left);
        let right = self.representative(right);
        if left == right {
            return;
        }
        let (representative, merged) = if left < right {
            (left, right)
        } else {
            (right, left)
        };
        self.parents[merged] = representative;
    }
}

impl ConstructionEdgeIdentity {
    fn intersection_identity(
        &self,
        plane: ConstructionPlaneIdentity,
    ) -> ConstructionVertexIdentity {
        match self {
            Self::Source { mesh, endpoints } => ConstructionVertexIdentity::SourceEdgePlane {
                mesh: *mesh,
                endpoints: *endpoints,
                plane,
            },
            Self::Split { planes: existing } => {
                let mut planes = [existing[0], existing[1], plane];
                planes.sort_unstable();
                ConstructionVertexIdentity::PlaneTriple { planes }
            }
        }
    }
}

impl ProjectivePointCache {
    fn source_edge_key(edge: &ConstructionEdgeIdentity) -> Option<SourceEdgeKey> {
        let ConstructionEdgeIdentity::Source { mesh, endpoints } = edge else {
            return None;
        };
        Some([*mesh, endpoints[0], endpoints[1]])
    }

    fn point(&self, index: usize) -> &HomogeneousPoint3 {
        self.point_storage
            .get(index)
            .expect("projective point index is retained for the computation")
    }

    fn plane(&self, index: usize) -> &Plane {
        self.plane_storage
            .get(index)
            .expect("projective plane index is retained for the computation")
    }

    fn support_plane_index(&mut self, identity: ConstructionPlaneIdentity, plane: &Plane) -> usize {
        let identity = self.canonical_plane_identity(identity);
        if let Some(&index) = self.planes.get(&identity) {
            return index;
        }
        let index = self.plane_storage.len();
        self.plane_storage.push(plane.clone());
        self.planes.insert(identity, index);
        index
    }

    fn boundary_plane_index(
        &mut self,
        support: ConstructionPlaneIdentity,
        edge: &ConstructionEdgeIdentity,
        plane: &Plane,
    ) -> usize {
        let identity = ProjectiveBoundaryPlaneIdentity {
            support: self.canonical_plane_identity(support),
            edge: edge.clone(),
        };
        if let Some(indices) = self.boundary_planes.get(&identity) {
            for &index in indices {
                if self.plane(index) == plane {
                    return index;
                }
            }
        }
        let index = self.plane_storage.len();
        self.plane_storage.push(plane.clone());
        self.boundary_planes
            .entry(identity)
            .or_default()
            .push(index);
        index
    }

    fn inverted_plane_index(&mut self, identity: ConstructionPlaneIdentity) -> usize {
        let identity = self.canonical_plane_identity(identity);
        if let Some(&index) = self.inverted_planes.get(&identity) {
            return index;
        }
        let plane = self
            .planes
            .get(&identity)
            .map(|&index| self.plane(index).inverted())
            .expect("canonical support plane is retained for projective clipping");
        let index = self.plane_storage.len();
        self.plane_storage.push(plane);
        self.inverted_planes.insert(identity, index);
        index
    }

    fn canonical_plane_identity(
        &self,
        identity: ConstructionPlaneIdentity,
    ) -> ConstructionPlaneIdentity {
        self.canonical_planes[identity.mesh]
            .get(identity.plane)
            .copied()
            .unwrap_or(identity)
    }

    fn edge_plane_intersection_identity(
        &self,
        edge: &ConstructionEdgeIdentity,
        plane: ConstructionPlaneIdentity,
    ) -> ConstructionVertexIdentity {
        let plane = self.canonical_plane_identity(plane);
        if let Some(key) = Self::source_edge_key(edge)
            && let Some(source_edge) = self.source_edges.get(&key)
            && source_edge.support_count >= 2
        {
            let supports = source_edge.supports();
            let mut planes = [supports[0], supports[1], plane];
            planes.sort_unstable();
            return ConstructionVertexIdentity::PlaneTriple { planes };
        }
        edge.intersection_identity(plane)
    }

    fn record_incidence(
        &mut self,
        identity: &ConstructionVertexIdentity,
        plane: ConstructionPlaneIdentity,
    ) {
        let plane = self.canonical_plane_identity(plane);
        let incidences = self.point_incidences.entry(identity.clone()).or_default();
        if !incidences.contains(&plane) {
            incidences.push(plane);
        }
    }

    fn has_incidence(
        &self,
        identity: &ConstructionVertexIdentity,
        plane: ConstructionPlaneIdentity,
    ) -> bool {
        let plane = self.canonical_plane_identity(plane);
        if self
            .point_incidences
            .get(identity)
            .is_some_and(|incidences| incidences.contains(&plane))
        {
            return true;
        }
        let canonical = self.canonical_vertex_identity(identity);
        canonical != *identity
            && self
                .point_incidences
                .get(&canonical)
                .is_some_and(|incidences| incidences.contains(&plane))
    }

    fn canonical_vertex_identity(
        &self,
        identity: &ConstructionVertexIdentity,
    ) -> ConstructionVertexIdentity {
        self.canonical_identities
            .get(identity)
            .cloned()
            .unwrap_or_else(|| identity.clone())
    }

    fn record_definition_incidences(&mut self, identity: &ConstructionVertexIdentity) {
        match identity {
            ConstructionVertexIdentity::Source { .. } => {}
            ConstructionVertexIdentity::SourceEdgePlane {
                mesh,
                endpoints,
                plane,
            } => {
                let key = [*mesh, endpoints[0], endpoints[1]];
                let supports = self
                    .source_edges
                    .get(&key)
                    .map(|source_edge| (source_edge.supports, source_edge.support_count));
                if let Some((supports, support_count)) = supports {
                    for support in &supports[..usize::from(support_count)] {
                        self.record_incidence(identity, *support);
                    }
                }
                self.record_incidence(identity, *plane);
            }
            ConstructionVertexIdentity::PlaneTriple { planes } => {
                for plane in planes {
                    self.record_incidence(identity, *plane);
                }
            }
        }
    }

    fn definition_planes(&self, identity: &ConstructionVertexIdentity) -> Option<[&Plane; 3]> {
        match identity {
            ConstructionVertexIdentity::Source { .. } => {
                let planes = self.source_vertices.get(identity)?;
                Some([
                    self.plane(*self.planes.get(&planes[0])?),
                    self.plane(*self.planes.get(&planes[1])?),
                    self.plane(*self.planes.get(&planes[2])?),
                ])
            }
            ConstructionVertexIdentity::SourceEdgePlane {
                mesh,
                endpoints,
                plane,
            } => {
                let source_edge = self
                    .source_edges
                    .get(&[*mesh, endpoints[0], endpoints[1]])?;
                let [support, boundary] = source_edge.definition_planes;
                Some([
                    self.plane(support),
                    self.plane(boundary),
                    self.plane(*self.planes.get(plane)?),
                ])
            }
            ConstructionVertexIdentity::PlaneTriple { planes } => Some([
                self.plane(*self.planes.get(&planes[0])?),
                self.plane(*self.planes.get(&planes[1])?),
                self.plane(*self.planes.get(&planes[2])?),
            ]),
        }
    }

    fn identities_certifiably_equal(
        &self,
        left_identity: &ConstructionVertexIdentity,
        left: &HomogeneousPoint3,
        right_identity: &ConstructionVertexIdentity,
        right: &HomogeneousPoint3,
    ) -> bool {
        let identity_on_triple =
            |identity: &ConstructionVertexIdentity, triple: &ConstructionVertexIdentity| {
                let ConstructionVertexIdentity::PlaneTriple { planes } = triple else {
                    return false;
                };
                self.point_incidences
                    .get(identity)
                    .is_some_and(|incidences| planes.iter().all(|plane| incidences.contains(plane)))
            };
        if (identity_on_triple(left_identity, right_identity)
            || identity_on_triple(right_identity, left_identity))
            && projective_points_certifiably_equal(left, right)
        {
            return true;
        }
        let left_definition = self.definition_planes(left_identity);
        let right_definition = self.definition_planes(right_identity);
        if let (Some(left_definition), Some(right_definition)) =
            (&left_definition, &right_definition)
        {
            let definitions_equal = right_definition.iter().all(|plane| {
                let determinant = crate::intersection::four_plane_determinant(
                    left_definition[0],
                    left_definition[1],
                    left_definition[2],
                    plane,
                );
                crate::predicate::classify_real(&determinant) == Ok(Classification::On)
            }) && left_definition.iter().all(|plane| {
                let determinant = crate::intersection::four_plane_determinant(
                    right_definition[0],
                    right_definition[1],
                    right_definition[2],
                    plane,
                );
                crate::predicate::classify_real(&determinant) == Ok(Classification::On)
            });
            if definitions_equal {
                return true;
            }
        }
        let point_satisfies = |point: &HomogeneousPoint3, definition: &[&Plane; 3]| {
            definition.iter().all(|plane| {
                crate::predicate::classify_real(&homogeneous_point_plane_expression(point, *plane))
                    == Ok(Classification::On)
            })
        };
        match (left_definition.as_ref(), right_definition.as_ref()) {
            (Some(definition), None) => point_satisfies(right, definition),
            (None, Some(definition)) => point_satisfies(left, definition),
            (None, None) => projective_points_certifiably_equal(left, right),
            (Some(_), Some(_)) => projective_points_certifiably_equal(left, right),
        }
    }

    fn resolve_vertex_coincidences(&mut self) {
        let mut entries = self
            .points
            .drain()
            .map(|(identity, cached)| (identity, cached.point_index, cached.approximate))
            .collect::<Vec<_>>();
        entries.sort_unstable_by(|left, right| left.0.cmp(&right.0));

        let mut sets = AtomicDisjointSets::new(entries.len());
        let mut finite = entries
            .iter()
            .enumerate()
            .filter_map(|(index, entry)| entry.2.map(|point| (index, point)))
            .collect::<Vec<_>>();
        let sort_axis = (0..3)
            .max_by(|&left, &right| {
                let normalized_span = |axis| {
                    let (minimum, maximum) = finite.iter().fold(
                        (f64::INFINITY, f64::NEG_INFINITY),
                        |(minimum, maximum), entry| {
                            (minimum.min(entry.1[axis]), maximum.max(entry.1[axis]))
                        },
                    );
                    (maximum - minimum) / minimum.abs().max(maximum.abs()).max(1.0)
                };
                normalized_span(left).total_cmp(&normalized_span(right))
            })
            .unwrap_or(0);
        finite.sort_unstable_by(|left, right| left.1[sort_axis].total_cmp(&right.1[sort_axis]));
        for right in 0..finite.len() {
            let (right_index, right_point) = finite[right];
            for &(left_index, left_point) in finite[..right].iter().rev() {
                let axis_scale = left_point[sort_axis]
                    .abs()
                    .max(right_point[sort_axis].abs())
                    .max(1.0);
                if right_point[sort_axis] - left_point[sort_axis] > axis_scale * 1.0e-9 {
                    break;
                }
                if left_point.iter().zip(right_point).all(|(left, right)| {
                    let scale = left.abs().max(right.abs()).max(1.0);
                    (left - right).abs() <= scale * 1.0e-9
                }) && self.identities_certifiably_equal(
                    &entries[left_index].0,
                    self.point(entries[left_index].1),
                    &entries[right_index].0,
                    self.point(entries[right_index].1),
                ) {
                    sets.merge(left_index, right_index);
                }
            }
        }
        for right in 0..entries.len() {
            for left in 0..right {
                if (entries[left].2.is_none() || entries[right].2.is_none())
                    && self.identities_certifiably_equal(
                        &entries[left].0,
                        self.point(entries[left].1),
                        &entries[right].0,
                        self.point(entries[right].1),
                    )
                {
                    sets.merge(left, right);
                }
            }
        }

        let representatives = (0..entries.len())
            .map(|index| sets.representative(index))
            .collect::<Vec<_>>();
        let mut merged_classes = vec![false; entries.len()];
        for (index, &representative) in representatives.iter().enumerate() {
            if index != representative {
                merged_classes[representative] = true;
            }
        }
        let mut class_incidences = vec![Vec::new(); entries.len()];
        for (index, (identity, _, _)) in entries.iter().enumerate() {
            let representative = representatives[index];
            if !merged_classes[representative] {
                continue;
            }
            if let Some(incidences) = self.point_incidences.get(identity) {
                for &incidence in incidences {
                    if !class_incidences[representative].contains(&incidence) {
                        class_incidences[representative].push(incidence);
                    }
                }
            }
        }
        for incidences in &mut class_incidences {
            incidences.sort_unstable();
        }

        self.canonical_identities.clear();
        for (index, (identity, _, _)) in entries.iter().enumerate() {
            let representative = representatives[index];
            let canonical_identity = entries[representative].0.clone();
            if *identity != canonical_identity {
                self.canonical_identities
                    .insert(identity.clone(), canonical_identity);
            }
            if merged_classes[representative] {
                self.point_incidences
                    .insert(identity.clone(), class_incidences[representative].clone());
            }
        }
        for (identity, point_index, _) in entries {
            self.points.insert(
                identity,
                CachedProjectivePoint {
                    point_index,
                    approximate: None,
                },
            );
        }
        for (identity, canonical_identity) in &self.canonical_identities {
            let canonical_point_index = self
                .points
                .get(canonical_identity)
                .expect("canonical projective point is available")
                .point_index;
            self.points.insert(
                identity.clone(),
                CachedProjectivePoint {
                    point_index: canonical_point_index,
                    approximate: None,
                },
            );
        }
    }

    fn intern_with_approximation(
        &mut self,
        identity: ConstructionVertexIdentity,
        point: HomogeneousPoint3,
    ) -> (usize, Option<[f64; 3]>, ConstructionVertexIdentity) {
        self.intern_with_approximation_by(identity, || point)
    }

    fn intern_with_approximation_by(
        &mut self,
        identity: ConstructionVertexIdentity,
        make_point: impl FnOnce() -> HomogeneousPoint3,
    ) -> (usize, Option<[f64; 3]>, ConstructionVertexIdentity) {
        self.intern_with_optional_approximation_by(identity, None, make_point)
    }

    fn intern_with_known_approximation_by(
        &mut self,
        identity: ConstructionVertexIdentity,
        approximate: Option<[f64; 3]>,
        make_point: impl FnOnce() -> HomogeneousPoint3,
    ) -> (usize, Option<[f64; 3]>, ConstructionVertexIdentity) {
        self.intern_with_optional_approximation_by(identity, Some(approximate), make_point)
    }

    fn intern_with_optional_approximation_by(
        &mut self,
        identity: ConstructionVertexIdentity,
        known_approximate: Option<Option<[f64; 3]>>,
        make_point: impl FnOnce() -> HomogeneousPoint3,
    ) -> (usize, Option<[f64; 3]>, ConstructionVertexIdentity) {
        self.record_definition_incidences(&identity);
        if let Some(existing) = self.points.get(&identity) {
            return (existing.point_index, existing.approximate, identity);
        }
        let point = make_point();
        let approximate = known_approximate.unwrap_or_else(|| projective_point_f64(&point));
        let point_index = self.point_storage.len();
        self.point_storage.push(point);
        self.points.insert(
            identity.clone(),
            CachedProjectivePoint {
                point_index,
                approximate,
            },
        );
        (point_index, approximate, identity)
    }
}

fn projective_point_f64(point: &HomogeneousPoint3) -> Option<[f64; 3]> {
    let weight = point.w.to_f64_lossy()?;
    if weight == 0.0 || !weight.is_finite() {
        return None;
    }
    let coordinates = [&point.x, &point.y, &point.z].map(|coordinate| {
        let value = coordinate.to_f64_lossy()? / weight;
        value.is_finite().then_some(value)
    });
    let [Some(x), Some(y), Some(z)] = coordinates else {
        return None;
    };
    Some([x, y, z])
}

fn affine_point_f64(point: &Point3) -> Option<[f64; 3]> {
    let point = [
        point.x.to_f64_lossy()?,
        point.y.to_f64_lossy()?,
        point.z.to_f64_lossy()?,
    ];
    point.into_iter().all(f64::is_finite).then_some(point)
}

fn normalized_point_plane_may_be_on(point: [f64; 3], plane: Option<[f64; 4]>) -> bool {
    match plane {
        Some(plane) => {
            let value = plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3];
            let scale = point.into_iter().map(f64::abs).fold(1.0_f64, f64::max);
            value.abs() <= scale * 1.0e-9
        }
        None => true,
    }
}

fn projective_points_certifiably_equal(
    left: &HomogeneousPoint3,
    right: &HomogeneousPoint3,
) -> bool {
    let left = [&left.x, &left.y, &left.z, &left.w];
    let right = [&right.x, &right.y, &right.z, &right.w];
    for first in 0..left.len() {
        for second in (first + 1)..left.len() {
            let minor = Real::signed_product_sum(
                [true, false],
                [[left[first], right[second]], [left[second], right[first]]],
            );
            if crate::predicate::classify_real(&minor) != Ok(Classification::On) {
                return false;
            }
        }
    }
    true
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum ProjectiveClipSide {
    Negative,
    Positive,
    Both,
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum SourcePlaneRelation {
    Inside,
    Outside,
    Crossing,
}

#[derive(Clone, Copy, Eq, Hash, PartialEq)]
struct PointClassificationKey([usize; 3]);

#[derive(Default)]
struct PointPlaneClassificationCache {
    source_queries: Vec<Option<Option<PreparedRationalLinearForm4Query>>>,
    source_classifications: Vec<Option<Classification>>,
    source_plane_count: Option<usize>,
    points: StorageHashMap<PointClassificationKey, CachedPointPlaneClassifications>,
}

struct CachedPointPlaneClassifications {
    prepared_query: Option<PreparedRationalLinearForm4Query>,
    classifications: Vec<Option<Classification>>,
}

impl PointPlaneClassificationCache {
    fn source_relation(
        &mut self,
        polygon: &ConvexPolygon,
        plane: &Plane,
        plane_index: usize,
        plane_count: usize,
    ) -> HypermeshResult<(SourcePlaneRelation, Vec<usize>)> {
        if certifiably_same_unoriented_plane(&polygon.support, plane) {
            let on_source_vertices = polygon
                .known_vertex_identities()
                .into_iter()
                .flatten()
                .filter_map(|identity| match identity {
                    ConstructionVertexIdentity::Source { vertex, .. } => Some(vertex),
                    _ => None,
                })
                .collect();
            return Ok((SourcePlaneRelation::Inside, on_source_vertices));
        }
        let mut has_negative = false;
        let mut has_positive = false;
        let mut on_source_vertices = Vec::new();
        let edge_identities = polygon.known_edge_identities();
        for (point_index, point) in polygon
            .known_vertices
            .as_ref()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?
            .iter()
            .enumerate()
        {
            let source_vertex =
                edge_identities.and_then(|identities| source_vertex_index(identities, point_index));
            match self.classify(point, source_vertex, plane, plane_index, plane_count)? {
                Classification::Negative => has_negative = true,
                Classification::Positive => has_positive = true,
                Classification::On => {
                    if let Some(source_vertex) = source_vertex {
                        on_source_vertices.push(source_vertex);
                    }
                }
            }
            if has_positive && has_negative {
                return Ok((SourcePlaneRelation::Crossing, on_source_vertices));
            }
        }
        Ok((
            if has_positive {
                SourcePlaneRelation::Outside
            } else {
                SourcePlaneRelation::Inside
            },
            on_source_vertices,
        ))
    }

    fn classify(
        &mut self,
        point: &Point3,
        source_vertex: Option<usize>,
        plane: &Plane,
        plane_index: usize,
        plane_count: usize,
    ) -> HypermeshResult<Classification> {
        let [Some(x), Some(y), Some(z)] = [
            point.x.exact_rational_ref(),
            point.y.exact_rational_ref(),
            point.z.exact_rational_ref(),
        ] else {
            return classify_point(point, plane);
        };
        let make_cached = || CachedPointPlaneClassifications {
            prepared_query: Real::prepare_rational_affine_point3_query([x, y, z]),
            classifications: vec![None; plane_count],
        };
        if let Some(source_vertex) = source_vertex {
            if let Some(cached_plane_count) = self.source_plane_count {
                if cached_plane_count != plane_count {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
            } else {
                self.source_plane_count = Some(plane_count);
            }
            let source_count = source_vertex
                .checked_add(1)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            if self.source_queries.len() < source_count {
                self.source_queries.resize(source_count, None);
            }
            let classification_count = source_count
                .checked_mul(plane_count)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            if self.source_classifications.len() < classification_count {
                self.source_classifications
                    .resize(classification_count, None);
            }
            let classification_index = source_vertex * plane_count + plane_index;
            if let Some(classification) = self.source_classifications[classification_index] {
                return Ok(classification);
            }
            let prepared_query = *self.source_queries[source_vertex]
                .get_or_insert_with(|| Real::prepare_rational_affine_point3_query([x, y, z]));
            let classification = crate::predicate::classify_point_with_prepared_query(
                point,
                plane,
                prepared_query.as_ref(),
            )?;
            self.source_classifications[classification_index] = Some(classification);
            return Ok(classification);
        }

        let key = PointClassificationKey([x, y, z].map(hyperlattice::Rational::storage_identity));
        let cached = self.points.entry(key).or_insert_with(make_cached);
        if let Some(classification) = cached.classifications[plane_index] {
            return Ok(classification);
        }
        let classification = crate::predicate::classify_point_with_prepared_query(
            point,
            plane,
            cached.prepared_query.as_ref(),
        )?;
        cached.classifications[plane_index] = Some(classification);
        Ok(classification)
    }
}

fn source_vertex_index(
    edge_identities: KnownEdgeIdentityCycle<'_>,
    point_index: usize,
) -> Option<usize> {
    let current = edge_identities.get(point_index)?;
    let previous_index = if point_index == 0 {
        edge_identities.len().checked_sub(1)?
    } else {
        point_index - 1
    };
    let previous = edge_identities.get(previous_index)?;
    let (
        ConstructionEdgeIdentity::Source {
            mesh: current_mesh,
            endpoints: current_endpoints,
        },
        ConstructionEdgeIdentity::Source {
            mesh: previous_mesh,
            endpoints: previous_endpoints,
        },
    ) = (&current, &previous)
    else {
        return None;
    };
    if current_mesh != previous_mesh {
        return None;
    }
    if previous_endpoints.contains(&current_endpoints[0]) {
        Some(current_endpoints[0])
    } else if previous_endpoints.contains(&current_endpoints[1]) {
        Some(current_endpoints[1])
    } else {
        None
    }
}

impl ProjectiveCycle {
    fn point_has_plane_incidence(
        &self,
        point_index: usize,
        plane_identity: ConstructionPlaneIdentity,
        plane: &Plane,
        plane_f64: Option<[f64; 4]>,
        point_cache: &ProjectivePointCache,
    ) -> bool {
        if self.source_plane == plane_identity {
            return true;
        }
        if self.boundary.is_empty() {
            return false;
        }
        let previous = if point_index == 0 {
            self.boundary.len() - 1
        } else {
            point_index - 1
        };
        [previous, point_index].into_iter().any(|edge_index| {
            matches!(
                self.boundary.get(edge_index).map(|entry| &entry.edge_identity),
                Some(ConstructionEdgeIdentity::Split { planes })
                    if planes.contains(&plane_identity)
            )
        }) || (self.boundary[point_index]
            .preparation
            .approximate
            .is_none_or(|point| normalized_point_plane_may_be_on(point, plane_f64))
            && crate::intersection::four_plane_determinant(
                point_cache.plane(self.support_index),
                point_cache.plane(self.boundary[previous].edge_index),
                point_cache.plane(self.boundary[point_index].edge_index),
                plane,
            )
            .definitely_zero())
    }

    fn from_polygon(
        polygon: &ConvexPolygon,
        source_plane: ConstructionPlaneIdentity,
        point_cache: &mut ProjectivePointCache,
    ) -> HypermeshResult<Self> {
        let source_points = polygon
            .known_vertices
            .as_ref()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?;
        let source_edge_identities = polygon
            .known_edge_identities()
            .ok_or(crate::error::HypermeshError::UnknownClassification)?;
        if source_edge_identities.len() != source_points.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        match polygon.edges.len() {
            len if len == source_points.len() || len <= 1 => {}
            _ => return Err(crate::error::HypermeshError::UnknownClassification),
        }
        let source_plane = point_cache.canonical_plane_identity(source_plane);
        let support_index = point_cache.support_plane_index(source_plane, &polygon.support);
        let mut boundary = Vec::with_capacity(source_points.len());
        for (point_index, point) in source_points.iter().enumerate() {
            let approximate = affine_point_f64(point);
            let vertex = source_vertex_index(source_edge_identities, point_index)
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let edge_identity = source_edge_identities
                .get(point_index)
                .expect("source edge identity indices are aligned");
            let mesh = match &edge_identity {
                ConstructionEdgeIdentity::Source { mesh, .. } => *mesh,
                ConstructionEdgeIdentity::Split { .. } => {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
            };
            let identity = ConstructionVertexIdentity::Source { mesh, vertex };
            let (projective_point_index, approximate, identity) = point_cache
                .intern_with_known_approximation_by(identity, approximate, || {
                    HomogeneousPoint3::new(
                        point.x.clone(),
                        point.y.clone(),
                        point.z.clone(),
                        Real::one(),
                    )
                });
            let edge = match polygon.edges.len() {
                0 => &polygon.support,
                1 => &polygon.edges[0],
                _ => &polygon.edges[point_index],
            };
            let edge_index = point_cache.boundary_plane_index(source_plane, &edge_identity, edge);
            boundary.push(ProjectiveBoundaryEntry {
                point_index: projective_point_index,
                preparation: ProjectivePointPreparation {
                    approximate,
                    // Retain prepared queries only for constructed crossings that
                    // survive into later clips; source vertices use the one-shot path.
                    rational_filter_query: None,
                },
                point_identity: identity,
                edge_index,
                edge_identity,
            });
        }
        Ok(Self {
            boundary,
            support_index,
            source_plane,
            source_unchanged: true,
        })
    }

    fn crossing_points(
        &self,
        classifications: &[Classification],
        plane: &Plane,
        plane_identity: ConstructionPlaneIdentity,
        point_cache: &mut ProjectivePointCache,
    ) -> Vec<(
        usize,
        usize,
        ProjectivePointPreparation,
        ConstructionVertexIdentity,
    )> {
        let mut crossings = Vec::with_capacity(2);
        for index in 0..self.boundary.len() {
            let next = (index + 1) % self.boundary.len();
            let current_classification = classifications[index];
            let next_classification = classifications[next];
            let crossing = (current_classification.is_negative()
                && next_classification.is_positive())
                || (current_classification.is_positive() && next_classification.is_negative());
            if crossing {
                let (point, approximate_point, identity) = self.cached_crossing_point(
                    index,
                    plane_identity,
                    current_classification,
                    next,
                    plane,
                    point_cache,
                );
                let rational_filter_query =
                    PreparedProjectivePoint3::new(point_cache.point(point)).rational_filter_query();
                crossings.push((
                    index,
                    point,
                    ProjectivePointPreparation {
                        approximate: approximate_point,
                        rational_filter_query,
                    },
                    identity,
                ));
            }
        }
        crossings
    }

    fn clip(
        self,
        plane: &Plane,
        plane_f64: Option<[f64; 4]>,
        plane_identity: ConstructionPlaneIdentity,
        point_cache: &mut ProjectivePointCache,
    ) -> HypermeshResult<ProjectiveClip> {
        let plane_identity = point_cache.canonical_plane_identity(plane_identity);
        let prepared_plane = PreparedRationalPlane4::new(plane);
        let classifications = self
            .boundary
            .iter()
            .enumerate()
            .map(|(point_index, entry)| {
                let point = point_cache.point(entry.point_index);
                if self.point_has_plane_incidence(
                    point_index,
                    plane_identity,
                    plane,
                    plane_f64,
                    point_cache,
                ) {
                    Ok(Classification::On)
                } else if let Some(plane_filter) = &prepared_plane {
                    let prepared = PreparedProjectivePoint3::with_rational_filter_query(
                        point,
                        entry.preparation.rational_filter_query,
                    );
                    match prepared
                        .classify_rational_plane_filter(plane_filter)
                        .or_else(|| prepared.classify_rational_plane_exact(plane_filter))
                    {
                        Some(classification) => Ok(classification),
                        None => prepared.classify(plane),
                    }
                } else {
                    classify_projective_point(point, plane).inspect_err(|_error| {
                        if cfg!(debug_assertions) {
                            eprintln!(
                                "[DEBUG] projective clip point failed: source={:?} target={:?} point={point_index} value={:?}",
                                self.source_plane,
                                plane_identity,
                                hyperlattice::homogeneous_point_plane_expression(
                                    point,
                                    plane,
                                )
                                    .to_f64_lossy(),
                            );
                        }
                    })
                }
            })
            .collect::<HypermeshResult<Vec<_>>>()?;
        for (point_index, classification) in classifications.iter().enumerate() {
            if *classification == Classification::On {
                point_cache
                    .record_incidence(&self.boundary[point_index].point_identity, plane_identity);
            }
        }
        let has_negative = classifications
            .iter()
            .any(|classification| classification.is_negative());
        let has_positive = classifications
            .iter()
            .any(|classification| classification.is_positive());
        if !has_positive {
            crate::trace_dispatch!("projective-clip", "negative-only");
            return Ok(ProjectiveClip {
                negative: self,
                positive: Self::empty(),
                side: ProjectiveClipSide::Negative,
            });
        }
        if !has_negative {
            crate::trace_dispatch!("projective-clip", "positive-only");
            return Ok(ProjectiveClip {
                negative: Self::empty(),
                positive: self,
                side: ProjectiveClipSide::Positive,
            });
        }

        crate::trace_dispatch!("projective-clip", "split");
        let clipping_plane_index = point_cache.support_plane_index(plane_identity, plane);
        let inverted_plane_index = point_cache.inverted_plane_index(plane_identity);
        let intersections =
            self.crossing_points(&classifications, plane, plane_identity, point_cache);
        let mut negative = ProjectiveBoundary::with_capacity(self.boundary.len() + 1);
        let mut positive = ProjectiveBoundary::with_capacity(self.boundary.len() + 1);
        let mut split_planes = [self.source_plane, plane_identity];
        split_planes.sort_unstable();
        let split_identity = ConstructionEdgeIdentity::Split {
            planes: split_planes,
        };
        let Self {
            boundary,
            support_index,
            source_plane,
            ..
        } = self;
        let mut intersections = intersections.into_iter();
        for (index, entry) in boundary.into_iter().enumerate() {
            let ProjectiveBoundaryEntry {
                point_index: point,
                preparation: point_preparation,
                point_identity,
                edge_index,
                edge_identity,
            } = entry;
            let next = (index + 1) % classifications.len();
            let current_classification = classifications[index];
            let next_classification = classifications[next];
            match (current_classification, next_classification) {
                (Classification::Negative, Classification::Negative | Classification::On) => {
                    negative.push(
                        point,
                        point_preparation,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::Negative, Classification::Positive) => {
                    let (
                        crossing_index,
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                    ) = intersections
                        .next()
                        .expect("strict side transition has an intersection");
                    debug_assert_eq!(crossing_index, index);
                    negative.push(
                        point,
                        point_preparation,
                        point_identity,
                        edge_index,
                        edge_identity.clone(),
                        point_cache,
                    );
                    negative.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity.clone(),
                        clipping_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                    positive.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::On, Classification::Negative) => {
                    negative.push(
                        point,
                        point_preparation,
                        point_identity.clone(),
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                    positive.push(
                        point,
                        point_preparation,
                        point_identity,
                        inverted_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                }
                (Classification::On, Classification::On) => {
                    negative.push(
                        point,
                        point_preparation,
                        point_identity.clone(),
                        edge_index,
                        edge_identity.clone(),
                        point_cache,
                    );
                    positive.push(
                        point,
                        point_preparation,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::On, Classification::Positive) => {
                    negative.push(
                        point,
                        point_preparation,
                        point_identity.clone(),
                        clipping_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                    positive.push(
                        point,
                        point_preparation,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::Positive, Classification::Negative) => {
                    let (
                        crossing_index,
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                    ) = intersections
                        .next()
                        .expect("strict side transition has an intersection");
                    debug_assert_eq!(crossing_index, index);
                    negative.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity.clone(),
                        edge_index,
                        edge_identity.clone(),
                        point_cache,
                    );
                    positive.push(
                        point,
                        point_preparation,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                    positive.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                        inverted_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                }
                (Classification::Positive, Classification::On | Classification::Positive) => {
                    positive.push(
                        point,
                        point_preparation,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
            }
        }
        debug_assert!(intersections.next().is_none());
        Ok(ProjectiveClip {
            negative: negative.into_cycle(support_index, source_plane, point_cache),
            positive: positive.into_cycle(support_index, source_plane, point_cache),
            side: ProjectiveClipSide::Both,
        })
    }

    fn clip_negative(
        self,
        plane: &Plane,
        plane_f64: Option<[f64; 4]>,
        plane_identity: ConstructionPlaneIdentity,
        point_cache: &mut ProjectivePointCache,
    ) -> HypermeshResult<Self> {
        let plane_identity = point_cache.canonical_plane_identity(plane_identity);
        let prepared_plane = PreparedRationalPlane4::new(plane);
        let classifications = self
            .boundary
            .iter()
            .enumerate()
            .map(|(point_index, entry)| {
                let point = point_cache.point(entry.point_index);
                if self.point_has_plane_incidence(
                    point_index,
                    plane_identity,
                    plane,
                    plane_f64,
                    point_cache,
                ) {
                    Ok(Classification::On)
                } else if let Some(plane_filter) = &prepared_plane {
                    let prepared = PreparedProjectivePoint3::with_rational_filter_query(
                        point,
                        entry.preparation.rational_filter_query,
                    );
                    match prepared
                        .classify_rational_plane_filter(plane_filter)
                        .or_else(|| prepared.classify_rational_plane_exact(plane_filter))
                    {
                        Some(classification) => Ok(classification),
                        None => prepared.classify(plane),
                    }
                } else {
                    classify_projective_point(point, plane).inspect_err(|_error| {
                        if cfg!(debug_assertions) {
                            eprintln!(
                                "[DEBUG] projective negative clip point failed: source={:?} target={:?} point={point_index} identity={:?} adjacent={:?} point_xyz={:?} plane={:?} exact={:?} value={:?}",
                                self.source_plane,
                                plane_identity,
                                self.boundary.get(point_index).map(|entry| &entry.point_identity),
                                [
                                    self.boundary.get(if point_index == 0 { self.boundary.len() - 1 } else { point_index - 1 }).map(|entry| &entry.edge_identity),
                                    self.boundary.get(point_index).map(|entry| &entry.edge_identity),
                                ],
                                [
                                    point.x.to_f64_lossy(),
                                    point.y.to_f64_lossy(),
                                    point.z.to_f64_lossy(),
                                    point.w.to_f64_lossy(),
                                ],
                                [
                                    plane.normal.x.to_f64_lossy(),
                                    plane.normal.y.to_f64_lossy(),
                                    plane.normal.z.to_f64_lossy(),
                                    plane.offset.to_f64_lossy(),
                                ],
                                [
                                    plane.normal.x.exact_rational_ref().is_some(),
                                    plane.normal.y.exact_rational_ref().is_some(),
                                    plane.normal.z.exact_rational_ref().is_some(),
                                    plane.offset.exact_rational_ref().is_some(),
                                ],
                                hyperlattice::homogeneous_point_plane_expression(point, plane)
                                    .to_f64_lossy(),
                            );
                        }
                    })
                }
            })
            .collect::<HypermeshResult<Vec<_>>>()?;
        for (point_index, classification) in classifications.iter().enumerate() {
            if *classification == Classification::On {
                point_cache
                    .record_incidence(&self.boundary[point_index].point_identity, plane_identity);
            }
        }
        let has_negative = classifications
            .iter()
            .any(|classification| classification.is_negative());
        let has_positive = classifications
            .iter()
            .any(|classification| classification.is_positive());
        if !has_positive {
            crate::trace_dispatch!("projective-clip-negative", "kept");
            return Ok(self);
        }
        if !has_negative {
            crate::trace_dispatch!("projective-clip-negative", "empty");
            return Ok(Self::empty());
        }
        crate::trace_dispatch!("projective-clip-negative", "split");
        let clipping_plane_index = point_cache.support_plane_index(plane_identity, plane);
        let intersections =
            self.crossing_points(&classifications, plane, plane_identity, point_cache);
        let mut negative = ProjectiveBoundary::with_capacity(self.boundary.len() + 1);
        let mut split_planes = [self.source_plane, plane_identity];
        split_planes.sort_unstable();
        let split_identity = ConstructionEdgeIdentity::Split {
            planes: split_planes,
        };
        let Self {
            boundary,
            support_index,
            source_plane,
            ..
        } = self;
        let mut intersections = intersections.into_iter();
        for (index, entry) in boundary.into_iter().enumerate() {
            let ProjectiveBoundaryEntry {
                point_index: point,
                preparation: point_preparation,
                point_identity,
                edge_index,
                edge_identity,
            } = entry;
            let next = (index + 1) % classifications.len();
            let current_classification = classifications[index];
            let next_classification = classifications[next];
            match (current_classification, next_classification) {
                (
                    Classification::Negative | Classification::On,
                    Classification::Negative | Classification::On,
                ) => {
                    negative.push(
                        point,
                        point_preparation,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::Negative, Classification::Positive) => {
                    let (
                        crossing_index,
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                    ) = intersections
                        .next()
                        .expect("strict side transition has an intersection");
                    debug_assert_eq!(crossing_index, index);
                    negative.push(
                        point,
                        point_preparation,
                        point_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                    negative.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                        clipping_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                }
                (Classification::On, Classification::Positive) => {
                    negative.push(
                        point,
                        point_preparation,
                        point_identity,
                        clipping_plane_index,
                        split_identity.clone(),
                        point_cache,
                    );
                }
                (Classification::Positive, Classification::Negative) => {
                    let (
                        crossing_index,
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                    ) = intersections
                        .next()
                        .expect("strict side transition has an intersection");
                    debug_assert_eq!(crossing_index, index);
                    negative.push(
                        intersection,
                        intersection_preparation,
                        intersection_identity,
                        edge_index,
                        edge_identity,
                        point_cache,
                    );
                }
                (Classification::Positive, Classification::On | Classification::Positive) => {}
            }
        }
        debug_assert!(intersections.next().is_none());
        Ok(negative.into_cycle(support_index, source_plane, point_cache))
    }

    #[allow(clippy::too_many_arguments)]
    fn cached_crossing_point(
        &self,
        edge_index: usize,
        plane_identity: ConstructionPlaneIdentity,
        current_classification: Classification,
        next_index: usize,
        plane: &Plane,
        point_cache: &mut ProjectivePointCache,
    ) -> (usize, Option<[f64; 3]>, ConstructionVertexIdentity) {
        let identity = point_cache.edge_plane_intersection_identity(
            &self.boundary[edge_index].edge_identity,
            plane_identity,
        );
        if point_cache.points.len() >= PROJECTIVE_CROSSING_CACHE_MIN_POINTS
            && let Some(existing) = point_cache.points.get(&identity)
        {
            return (existing.point_index, existing.approximate, identity);
        }
        let point = if let Some(point) = point_cache
            .definition_planes(&identity)
            .and_then(positive_weight_plane_intersection)
        {
            point
        } else {
            let current = point_cache.point(self.boundary[edge_index].point_index);
            let next = point_cache.point(self.boundary[next_index].point_index);
            let current_value = homogeneous_point_plane_expression(current, plane);
            let next_value = homogeneous_point_plane_expression(next, plane);
            projective_crossing_point(
                current,
                &current_value,
                current_classification,
                next,
                &next_value,
            )
        };
        point_cache.intern_with_approximation(identity, point)
    }

    fn materialize(
        self,
        source: &ConvexPolygon,
        affine_cache: &mut ProjectiveAffineCache,
        point_cache: &ProjectivePointCache,
    ) -> HypermeshResult<ConvexPolygon> {
        if self.source_unchanged {
            return Ok(source.clone());
        }
        let mut vertices = Vec::with_capacity(self.boundary.len());
        let mut point_identities = Vec::with_capacity(self.boundary.len());
        let mut edges = Vec::with_capacity(self.boundary.len());
        let mut edge_identities = Vec::with_capacity(self.boundary.len());
        for entry in self.boundary {
            vertices.push(affine_cache.resolve(
                point_cache.point(entry.point_index),
                Some(&entry.point_identity),
            )?);
            point_identities.push(entry.point_identity);
            edges.push(point_cache.plane(entry.edge_index).clone());
            edge_identities.push(entry.edge_identity);
        }
        Ok(source.with_known_vertex_cycle_and_edges(
            vertices,
            point_identities,
            edges,
            edge_identities,
        ))
    }

    fn empty() -> Self {
        Self {
            boundary: Vec::new(),
            support_index: usize::MAX,
            source_plane: ConstructionPlaneIdentity {
                mesh: usize::MAX,
                plane: usize::MAX,
            },
            source_unchanged: false,
        }
    }
}

impl ProjectiveAffineCache {
    fn resolve(
        &mut self,
        point: &HomogeneousPoint3,
        identity: Option<&ConstructionVertexIdentity>,
    ) -> HypermeshResult<Point3> {
        if let Some(identity) = identity
            && let Some(affine) = self.identities.get(identity)
        {
            return Ok(affine.clone());
        }
        let coordinates = [
            point.x.exact_rational_ref(),
            point.y.exact_rational_ref(),
            point.z.exact_rational_ref(),
            point.w.exact_rational_ref(),
        ];
        if let [Some(x), Some(y), Some(z), Some(w)] = coordinates {
            let key = [x, y, z, w].map(Rational::storage_identity);
            if let Some(entry) = self.points.get(&key) {
                return Ok(entry.affine.clone());
            }
            let affine = affine_projective_point(point)?;
            self.points.insert(
                key,
                ProjectiveAffineCacheEntry {
                    _coordinates: [x.clone(), y.clone(), z.clone(), w.clone()],
                    affine: affine.clone(),
                },
            );
            if let Some(identity) = identity {
                self.identities.insert(identity.clone(), affine.clone());
            }
            return Ok(affine);
        }
        let affine = affine_projective_point(point)?;
        if let Some(identity) = identity {
            self.identities.insert(identity.clone(), affine.clone());
        }
        Ok(affine)
    }
}

fn affine_projective_point(point: &HomogeneousPoint3) -> HypermeshResult<Point3> {
    point.to_affine_point().map_err(|_| {
        if point.w.definitely_zero() {
            crate::error::HypermeshError::PointAtInfinity
        } else {
            crate::error::HypermeshError::UnknownClassification
        }
    })
}

fn compute_two_convex_inputs_projectively(
    polygons: &[ConvexPolygon],
    operation: BooleanOp,
) -> HypermeshResult<Option<ConvexCandidate>> {
    let mut support_planes: [Vec<&Plane>; 2] = std::array::from_fn(|_| Vec::new());
    let mut storage_support_planes: [StorageHashMap<[usize; 4], usize>; 2] =
        std::array::from_fn(|_| StorageHashMap::default());
    let mut approximate_support_planes: [StorageHashMap<[u64; 4], Vec<usize>>; 2] =
        std::array::from_fn(|_| StorageHashMap::default());
    let mut non_exact_support_planes: [Vec<usize>; 2] = std::array::from_fn(|_| Vec::new());
    let mut support_plane_f64_values: [Vec<Option<[f64; 4]>>; 2] =
        std::array::from_fn(|_| Vec::new());
    let mut normalized_support_plane_f64_values: [Vec<Option<[f64; 4]>>; 2] =
        std::array::from_fn(|_| Vec::new());
    let mut polygon_support_planes = Vec::with_capacity(polygons.len());
    for polygon in polygons {
        let mesh = usize::try_from(polygon.mesh_index)
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
        if mesh >= support_planes.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let storage_key = exact_plane_storage_key(&polygon.support);
        let stored_plane =
            storage_key.and_then(|key| storage_support_planes[mesh].get(&key).copied());
        let plane = if let Some(index) = stored_plane {
            index
        } else if let Some(values) = exact_plane_f64(&polygon.support) {
            let key = values.map(f64::to_bits);
            if let Some(index) = approximate_support_planes[mesh]
                .get(&key)
                .into_iter()
                .flatten()
                .copied()
                .find(|&index| support_planes[mesh][index] == &polygon.support)
            {
                index
            } else {
                let index = support_planes[mesh].len();
                support_planes[mesh].push(&polygon.support);
                support_plane_f64_values[mesh].push(Some(values));
                normalized_support_plane_f64_values[mesh].push(normalize_plane_f64(values));
                approximate_support_planes[mesh]
                    .entry(key)
                    .or_default()
                    .push(index);
                index
            }
        } else if let Some(index) = support_planes[mesh].iter().position(|existing| {
            planes_may_be_same(existing, &polygon.support)
                && certifiably_same_oriented_plane(existing, &polygon.support).unwrap_or(false)
        }) {
            index
        } else if let Some(index) = support_planes[mesh]
            .iter()
            .position(|plane| *plane == &polygon.support)
        {
            index
        } else {
            let index = support_planes[mesh].len();
            support_planes[mesh].push(&polygon.support);
            support_plane_f64_values[mesh].push(None);
            normalized_support_plane_f64_values[mesh].push(plane_f64(&polygon.support));
            non_exact_support_planes[mesh].push(index);
            index
        };
        if let Some(key) = storage_key
            && stored_plane.is_none()
        {
            storage_support_planes[mesh].insert(key, plane);
        }
        polygon_support_planes.push(ConstructionPlaneIdentity { mesh, plane });
    }
    let support_planes_f64 =
        support_plane_f64_values.map(|planes| planes.into_iter().collect::<Option<Vec<_>>>());
    let canonical_plane_identities =
        canonical_plane_identities(&support_planes, &normalized_support_plane_f64_values);
    let (projective_polygons, mut projective_polygon_support_planes) =
        match collapse_certified_convex_faces(polygons, &polygon_support_planes, &support_planes) {
            Ok(collapsed) => collapsed,
            Err(crate::error::HypermeshError::UnknownClassification) => {
                (polygons.to_vec(), polygon_support_planes)
            }
            Err(error) => return Err(error),
        };
    for identity in &mut projective_polygon_support_planes {
        *identity = canonical_plane_identities[identity.mesh][identity.plane];
    }
    let polygons = projective_polygons.as_slice();
    let polygon_support_planes = projective_polygon_support_planes;
    let mut classified = Vec::new();
    let mut point_plane_caches: [PointPlaneClassificationCache; 2] =
        std::array::from_fn(|_| PointPlaneClassificationCache::default());
    let mut affine_cache = ProjectiveAffineCache::default();
    let mut projective_point_cache = ProjectivePointCache {
        canonical_planes: canonical_plane_identities,
        ..ProjectivePointCache::default()
    };
    for (mesh, planes) in support_planes.iter().enumerate() {
        for (plane, value) in planes.iter().enumerate() {
            let identity = ConstructionPlaneIdentity { mesh, plane };
            let canonical = projective_point_cache.canonical_plane_identity(identity);
            projective_point_cache.support_plane_index(canonical, value);
        }
    }
    let mut source_vertex_points: StorageHashMap<ConstructionVertexIdentity, &Point3> =
        StorageHashMap::default();
    for (polygon, support_identity) in polygons.iter().zip(&polygon_support_planes) {
        if let Some(vertex_identities) = polygon.known_vertex_identities() {
            let retained_vertices = polygon.known_vertices.as_ref();
            for (vertex_index, vertex_identity) in vertex_identities.iter().enumerate() {
                if matches!(vertex_identity, ConstructionVertexIdentity::Source { .. }) {
                    let incidences = projective_point_cache
                        .point_incidences
                        .entry(vertex_identity.clone())
                        .or_default();
                    if !incidences.contains(support_identity) {
                        incidences.push(*support_identity);
                    }
                    if let Some(point) =
                        retained_vertices.and_then(|vertices| vertices.get(vertex_index))
                    {
                        source_vertex_points
                            .entry(vertex_identity.clone())
                            .or_insert(point);
                    }
                }
            }
        }
        let Some(edge_identities) = polygon.known_edge_identities() else {
            continue;
        };
        if edge_identities.len() != polygon.edges.len() {
            continue;
        }
        let Some(&support) = projective_point_cache.planes.get(support_identity) else {
            continue;
        };
        for (edge_identity, edge_plane) in edge_identities.iter().zip(polygon.edges.iter()) {
            if matches!(edge_identity, ConstructionEdgeIdentity::Source { .. }) {
                let key = ProjectivePointCache::source_edge_key(&edge_identity)
                    .expect("source edge identity was matched above");
                if let Some(source_edge) = projective_point_cache.source_edges.get_mut(&key) {
                    if !source_edge.insert_support(*support_identity) {
                        return Err(crate::error::HypermeshError::UnknownClassification);
                    }
                } else {
                    let boundary = projective_point_cache.boundary_plane_index(
                        *support_identity,
                        &edge_identity,
                        edge_plane,
                    );
                    projective_point_cache.source_edges.insert(
                        key,
                        ProjectiveSourceEdge::new([support, boundary], *support_identity),
                    );
                }
            }
        }
    }
    {
        let ProjectivePointCache {
            plane_storage,
            planes,
            source_vertices,
            point_incidences,
            ..
        } = &mut projective_point_cache;
        for (identity, supports) in point_incidences.iter() {
            if !matches!(identity, ConstructionVertexIdentity::Source { .. }) {
                continue;
            }
            'definition: for first in 0..supports.len() {
                for second in (first + 1)..supports.len() {
                    for third in (second + 1)..supports.len() {
                        let definition = [
                            &plane_storage[planes[&supports[first]]],
                            &plane_storage[planes[&supports[second]]],
                            &plane_storage[planes[&supports[third]]],
                        ];
                        if plane_normals_are_independent(definition)? {
                            source_vertices.insert(
                                identity.clone(),
                                [supports[first], supports[second], supports[third]],
                            );
                            break 'definition;
                        }
                    }
                }
            }
        }
    }
    for (identity, point) in &source_vertex_points {
        let ConstructionVertexIdentity::Source { mesh, .. } = identity else {
            continue;
        };
        let other = 1 - *mesh;
        let approximate_point = affine_point_f64(point);
        for (plane, value) in support_planes[other].iter().enumerate() {
            if approximate_point.is_none_or(|point| {
                normalized_point_plane_may_be_on(
                    point,
                    normalized_support_plane_f64_values[other][plane],
                )
            }) && classify_point(point, value) == Ok(Classification::On)
            {
                let plane_identity = projective_point_cache
                    .canonical_plane_identity(ConstructionPlaneIdentity { mesh: other, plane });
                let incidences = projective_point_cache
                    .point_incidences
                    .entry(identity.clone())
                    .or_default();
                if !incidences.contains(&plane_identity) {
                    incidences.push(plane_identity);
                }
            }
        }
    }
    let mut prepared_verification_planes = Vec::new();
    let mut candidate_planes = Vec::new();
    let mut active_plane_proposal_scratch = ActivePlaneProposalScratch::default();
    for (polygon, source_plane) in polygons.iter().zip(polygon_support_planes) {
        let host = usize::try_from(polygon.mesh_index)
            .map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
        let other = 1 - host;
        let emit_outside = projective_transition_is_emitted(host, false, operation);
        let default_emit_inside = projective_transition_is_emitted(host, true, operation);
        candidate_planes.clear();
        let mut excluded = false;
        let mut has_cooriented_coincident_support = false;
        for (plane_index, &plane) in support_planes[other].iter().enumerate() {
            if !has_cooriented_coincident_support
                && source_plane
                    == projective_point_cache.canonical_plane_identity(ConstructionPlaneIdentity {
                        mesh: other,
                        plane: plane_index,
                    })
                && certifiably_same_oriented_plane(&polygon.support, plane).unwrap_or(false)
            {
                has_cooriented_coincident_support = true;
            }
            let (relation, on_source_vertices) = point_plane_caches[host]
                .source_relation(
                polygon,
                plane,
                plane_index,
                support_planes[other].len(),
            )
            .inspect_err(|_error| {
                if cfg!(debug_assertions) {
                    eprintln!(
                        "[DEBUG] projective source relation failed: host={host} polygon={} other_plane={plane_index}",
                        polygon.polygon_index,
                    );
                }
            })?;
            let plane_identity =
                projective_point_cache.canonical_plane_identity(ConstructionPlaneIdentity {
                    mesh: other,
                    plane: plane_index,
                });
            for vertex in on_source_vertices {
                let incidences = projective_point_cache
                    .point_incidences
                    .entry(ConstructionVertexIdentity::Source { mesh: host, vertex })
                    .or_default();
                if !incidences.contains(&plane_identity) {
                    incidences.push(plane_identity);
                }
            }
            match relation {
                SourcePlaneRelation::Inside => {}
                SourcePlaneRelation::Outside => {
                    excluded = true;
                    break;
                }
                SourcePlaneRelation::Crossing => candidate_planes.push(plane_index),
            }
        }
        let (emit_inside, inside_winding) = if has_cooriented_coincident_support {
            match operation {
                BooleanOp::Union => (host == 0, false),
                BooleanOp::Intersection => (host == 0, true),
                BooleanOp::Difference | BooleanOp::SymmetricDifference => (false, true),
            }
        } else {
            (default_emit_inside, true)
        };
        if !emit_outside && !emit_inside {
            continue;
        }
        if excluded {
            if emit_outside {
                push_source_transition(&mut classified, polygon, host, other, false)?;
            }
            continue;
        }
        if candidate_planes.is_empty() {
            if emit_inside {
                push_source_transition(&mut classified, polygon, host, other, inside_winding)?;
            }
            continue;
        }
        let source =
            ProjectiveCycle::from_polygon(polygon, source_plane, &mut projective_point_cache)
                .inspect_err(|_error| {
                    if cfg!(debug_assertions) {
                        eprintln!(
                            "[DEBUG] projective source cycle failed: host={host} polygon={}",
                            polygon.polygon_index,
                        );
                    }
                })?;

        let clipped_result = exact_inside_and_outside_cycles(
            source,
            polygon,
            source_plane,
            &support_planes[other],
            support_planes_f64[other].as_deref(),
            &normalized_support_plane_f64_values[other],
            &candidate_planes,
            other,
            emit_outside,
            &mut prepared_verification_planes,
            &mut active_plane_proposal_scratch,
            &mut projective_point_cache,
        )
        .inspect_err(|_error| {
            if cfg!(debug_assertions) {
                eprintln!(
                    "[DEBUG] projective active planes failed: host={host} polygon={}",
                    polygon.polygon_index,
                );
            }
        })?;
        let Some((inside, outside_cycles)) = clipped_result else {
            if emit_outside {
                let source = ProjectiveCycle::from_polygon(
                    polygon,
                    source_plane,
                    &mut projective_point_cache,
                )?;
                push_projective_transition(
                    &mut classified,
                    source,
                    polygon,
                    &mut affine_cache,
                    &projective_point_cache,
                    host,
                    other,
                    false,
                    operation,
                )?;
            }
            continue;
        };
        if emit_outside {
            let outside_cycles =
                outside_cycles.expect("outside cycles are retained when outside is emitted");
            for outside in outside_cycles {
                push_projective_transition(
                    &mut classified,
                    outside,
                    polygon,
                    &mut affine_cache,
                    &projective_point_cache,
                    host,
                    other,
                    false,
                    operation,
                )?;
            }
            if emit_inside {
                push_projective_transition(
                    &mut classified,
                    inside,
                    polygon,
                    &mut affine_cache,
                    &projective_point_cache,
                    host,
                    other,
                    inside_winding,
                    operation,
                )?;
            }
            continue;
        }
        if emit_inside {
            push_projective_transition(
                &mut classified,
                inside,
                polygon,
                &mut affine_cache,
                &projective_point_cache,
                host,
                other,
                inside_winding,
                operation,
            )?;
        }
    }
    projective_point_cache.resolve_vertex_coincidences();
    if !projective_point_cache.canonical_identities.is_empty() {
        affine_cache.identities.clear();
        // Hash-probing each cycle amortizes only for a substantial fragment
        // family. Small candidates retain the direct rebuild loop.
        const SELECTIVE_REBUILD_MIN_FRAGMENTS: usize = 32;
        let select_changed_fragments = classified.len() >= SELECTIVE_REBUILD_MIN_FRAGMENTS;
        for fragment in &mut classified {
            if let Some(vertex_identities) = fragment.polygon.known_vertex_identities() {
                // Coincidence resolution records only identities whose complete
                // equivalence class selected a different representative. A cycle
                // containing none of those identities already has both its final
                // identities and the affine points materialized from them.
                if select_changed_fragments
                    && !vertex_identities.iter().any(|identity| {
                        projective_point_cache
                            .canonical_identities
                            .contains_key(&identity)
                    })
                {
                    continue;
                }
                let canonical_identities = vertex_identities
                    .iter()
                    .map(|identity| projective_point_cache.canonical_vertex_identity(&identity))
                    .collect::<Vec<_>>();
                let original_vertices = fragment
                    .polygon
                    .known_vertices
                    .as_ref()
                    .ok_or(crate::error::HypermeshError::UnknownClassification)?
                    .iter()
                    .cloned()
                    .collect::<Vec<_>>();
                let vertices = canonical_identities
                    .iter()
                    .zip(original_vertices)
                    .map(|(identity, original)| {
                        let Some(point) = projective_point_cache.points.get(identity) else {
                            return Ok(original);
                        };
                        affine_cache.resolve(
                            projective_point_cache.point(point.point_index),
                            Some(identity),
                        )
                    })
                    .collect::<HypermeshResult<Vec<_>>>()?;
                fragment.polygon = fragment
                    .polygon
                    .with_known_vertex_cycle_and_identities(vertices, canonical_identities);
            }
        }
    }

    let triangle_soup = {
        if operation != BooleanOp::SymmetricDifference {
            let indicator = make_indicator(operation, support_planes.len());
            for fragment in &mut classified {
                let winding = fragment
                    .winding()
                    .ok_or(crate::error::HypermeshError::UnknownClassification)?;
                fragment.classification = crate::winding::classify_polygon_output(
                    &winding.w_front,
                    &winding.w_back,
                    &indicator,
                );
            }
        }
        let triangulate_fallback = || {
            crate::output::triangulate_classified_arrangement_precomputed_f64_scan(&classified)
                .and_then(|triangles| {
                    select_triangle_arrangement(&triangles, operation, support_planes.len())
                })
        };
        let soup = if matches!(operation, BooleanOp::Difference | BooleanOp::Intersection) {
            if classified
                .iter()
                .any(|fragment| !matches!(fragment.classification, -1 | 1))
            {
                return Ok(None);
            }
            let triangulate = |recover| {
                crate::output::triangulate_preclassified_arrangement_construction_candidates(
                    &classified,
                    recover,
                )
                .and_then(certify_triangle_soup_closure)
            };
            triangulate(false)
                .or_else(|_| triangulate_fallback())
                .or_else(|_| triangulate(true))
        } else if operation == BooleanOp::Union {
            let triangulate = |recover| {
                crate::output::triangulate_selected_preclassified_arrangement_construction_candidates(
                    &classified,
                    recover,
                )
                .and_then(certify_triangle_soup_closure)
            };
            triangulate(false)
                .or_else(|_| triangulate_fallback())
                .or_else(|_| triangulate(true))
        } else {
            let triangulate = |recover| {
                crate::output::triangulate_classified_arrangement_construction_candidates(
                    &classified,
                    recover,
                )
                .and_then(|triangles| {
                        select_triangle_arrangement(&triangles, operation, support_planes.len())
                    })
            };
            triangulate(false)
                .or_else(|_| triangulate_fallback())
                .or_else(|_| triangulate(true))
        }
        .inspect_err(|error| {
            if cfg!(debug_assertions) {
                eprintln!("[DEBUG] projective triangulation failed: {error}");
            }
        });
        match soup {
            Ok(soup) => soup,
            Err(_) => return Ok(None),
        }
    };
    Ok(Some(ConvexCandidate {
        classified,
        triangle_soup,
    }))
}

fn plane_normals_are_independent(planes: [&Plane; 3]) -> HypermeshResult<bool> {
    let rows = planes.map(|plane| [&plane.normal.x, &plane.normal.y, &plane.normal.z]);
    if let [
        [Some(a), Some(b), Some(c)],
        [Some(d), Some(e), Some(f)],
        [Some(g), Some(h), Some(i)],
    ] = rows.map(|row| row.map(Real::exact_rational_ref))
    {
        return Ok(!Rational::signed_product_sum_ordering(
            [true, true, true, false, false, false],
            [
                [a, e, i],
                [b, f, g],
                [c, d, h],
                [c, e, g],
                [b, d, i],
                [a, f, h],
            ],
        )
        .is_eq());
    }
    let determinant = Real::signed_product_sum(
        [true, true, true, false, false, false],
        [
            [rows[0][0], rows[1][1], rows[2][2]],
            [rows[0][1], rows[1][2], rows[2][0]],
            [rows[0][2], rows[1][0], rows[2][1]],
            [rows[0][2], rows[1][1], rows[2][0]],
            [rows[0][1], rows[1][0], rows[2][2]],
            [rows[0][0], rows[1][2], rows[2][1]],
        ],
    );
    Ok(crate::predicate::classify_real(&determinant)? != Classification::On)
}

fn exact_rational_product_sum_classification<const TERMS: usize, const FACTORS: usize>(
    positive_terms: [bool; TERMS],
    terms: [[&Real; FACTORS]; TERMS],
) -> Option<Classification> {
    let rational_terms = terms.map(|term| term.map(Real::exact_rational_ref));
    if rational_terms.iter().flatten().any(Option::is_none) {
        return None;
    }
    let rational_terms = rational_terms.map(|term| {
        term.map(|factor| factor.expect("all product-sum factors were checked exact rational"))
    });
    Some(
        match Rational::signed_product_sum_ordering(positive_terms, rational_terms) {
            std::cmp::Ordering::Less => Classification::Negative,
            std::cmp::Ordering::Equal => Classification::On,
            std::cmp::Ordering::Greater => Classification::Positive,
        },
    )
}

fn certifiably_proportional_plane(left: &Plane, right: &Plane) -> HypermeshResult<bool> {
    let left_coefficients = [&left.normal.x, &left.normal.y, &left.normal.z, &left.offset];
    let right_coefficients = [
        &right.normal.x,
        &right.normal.y,
        &right.normal.z,
        &right.offset,
    ];
    let mut unknown_minor = false;
    for first in 0..left_coefficients.len() {
        for second in (first + 1)..left_coefficients.len() {
            let terms = [
                [left_coefficients[first], right_coefficients[second]],
                [left_coefficients[second], right_coefficients[first]],
            ];
            let classification = exact_rational_product_sum_classification([true, false], terms)
                .map(Ok)
                .unwrap_or_else(|| {
                    let minor = Real::signed_product_sum([true, false], terms);
                    crate::predicate::classify_real(&minor)
                });
            match classification {
                Ok(Classification::On) => {}
                Ok(Classification::Negative | Classification::Positive) => return Ok(false),
                Err(crate::error::HypermeshError::UnknownClassification) => unknown_minor = true,
                Err(error) => return Err(error),
            }
        }
    }
    if unknown_minor {
        return Ok(false);
    }
    Ok(true)
}

fn certifiably_same_oriented_plane(left: &Plane, right: &Plane) -> HypermeshResult<bool> {
    if !certifiably_proportional_plane(left, right)? {
        return Ok(false);
    }
    let terms = [
        [&left.normal.x, &right.normal.x],
        [&left.normal.y, &right.normal.y],
        [&left.normal.z, &right.normal.z],
    ];
    let classification = exact_rational_product_sum_classification([true, true, true], terms)
        .map(Ok)
        .unwrap_or_else(|| {
            let orientation = Real::signed_product_sum([true, true, true], terms);
            crate::predicate::classify_real(&orientation)
        });
    Ok(classification? == Classification::Positive)
}

fn certifiably_same_unoriented_plane(left: &Plane, right: &Plane) -> bool {
    certifiably_proportional_plane(left, right).unwrap_or(false)
}

fn plane_f64(plane: &Plane) -> Option<[f64; 4]> {
    normalize_plane_f64([
        plane.normal.x.to_f64_lossy()?,
        plane.normal.y.to_f64_lossy()?,
        plane.normal.z.to_f64_lossy()?,
        plane.offset.to_f64_lossy()?,
    ])
}

fn normalize_plane_f64(mut values: [f64; 4]) -> Option<[f64; 4]> {
    if !values.into_iter().all(f64::is_finite) {
        return None;
    }
    let norm = values[..3]
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    if norm == 0.0 || !norm.is_finite() {
        return None;
    }
    let orientation = if values[..3]
        .iter()
        .max_by(|left, right| left.abs().total_cmp(&right.abs()))
        .copied()?
        .is_sign_negative()
    {
        -1.0
    } else {
        1.0
    };
    for value in &mut values {
        *value *= orientation / norm;
    }
    Some(values)
}

fn planes_may_be_same(left: &Plane, right: &Plane) -> bool {
    normalized_planes_may_be_same(plane_f64(left), plane_f64(right))
}

fn normalized_planes_may_be_same(left: Option<[f64; 4]>, right: Option<[f64; 4]>) -> bool {
    match (left, right) {
        (Some(left), Some(right)) => left.iter().zip(right).all(|(left, right)| {
            let scale = left.abs().max(right.abs()).max(1.0);
            (left - right).abs() <= scale * 1.0e-9
        }),
        _ => true,
    }
}

fn canonical_plane_identities(
    support_planes: &[Vec<&Plane>; 2],
    normalized_support_plane_f64_values: &[Vec<Option<[f64; 4]>>; 2],
) -> [Vec<ConstructionPlaneIdentity>; 2] {
    let first = (0..support_planes[0].len())
        .map(|plane| ConstructionPlaneIdentity { mesh: 0, plane })
        .collect();
    let second = support_planes[1]
        .iter()
        .enumerate()
        .map(|(plane, value)| {
            let approximate = normalized_support_plane_f64_values[1][plane];
            support_planes[0]
                .iter()
                .enumerate()
                .find_map(|(candidate, candidate_value)| {
                    let approximate_match = normalized_planes_may_be_same(
                        normalized_support_plane_f64_values[0][candidate],
                        approximate,
                    );
                    (approximate_match && certifiably_same_unoriented_plane(candidate_value, value))
                        .then_some(ConstructionPlaneIdentity {
                            mesh: 0,
                            plane: candidate,
                        })
                })
                .unwrap_or(ConstructionPlaneIdentity { mesh: 1, plane })
        })
        .collect();
    [first, second]
}

fn collapse_certified_convex_faces(
    polygons: &[ConvexPolygon],
    polygon_support_planes: &[ConstructionPlaneIdentity],
    support_planes: &[Vec<&Plane>; 2],
) -> HypermeshResult<(Vec<ConvexPolygon>, Vec<ConstructionPlaneIdentity>)> {
    // Counting source-vertex uses pays off for small subdivision patches. On
    // large coplanar groups the few provable corners do not amortize a second
    // sort, so those groups retain the ordinary exact collinearity scan.
    const MAX_SINGLE_USE_CERTIFICATE_TRIANGLES: usize = 16;
    if polygon_support_planes.len() != polygons.len() {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }

    let first_mesh_planes = support_planes[0].len();
    let group_count = first_mesh_planes
        .checked_add(support_planes[1].len())
        .ok_or(crate::error::HypermeshError::UnknownClassification)?;
    // Support identities are dense indices. Stable counting partitioning
    // preserves source order within each face while avoiding one tree node
    // and one growable index vector per distinct support plane.
    let mut group_offsets = vec![0usize; group_count + 1];
    for &support in polygon_support_planes {
        if support.mesh >= support_planes.len()
            || support.plane >= support_planes[support.mesh].len()
        {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let group = support.mesh * first_mesh_planes + support.plane;
        group_offsets[group + 1] += 1;
    }
    for group in 0..group_count {
        group_offsets[group + 1] += group_offsets[group];
    }
    let mut grouped_polygon_indices = vec![0usize; polygons.len()];
    // Fill each partition backward so its cumulative end becomes its start.
    // Reverse source traversal keeps indices in their original order and lets
    // the offset buffer double as the insertion cursors.
    for (polygon_index, &support) in polygon_support_planes.iter().enumerate().rev() {
        let group = support.mesh * first_mesh_planes + support.plane;
        group_offsets[group + 1] -= 1;
        grouped_polygon_indices[group_offsets[group + 1]] = polygon_index;
    }

    let mut faces = Vec::with_capacity(group_count);
    let mut face_supports = Vec::with_capacity(group_count);
    for group in 0..group_count {
        let start = group_offsets[group + 1];
        let end = if group + 1 == group_count {
            grouped_polygon_indices.len()
        } else {
            group_offsets[group + 2]
        };
        let polygon_indices = &grouped_polygon_indices[start..end];
        if polygon_indices.is_empty() {
            continue;
        }
        let support_identity = if group < first_mesh_planes {
            ConstructionPlaneIdentity {
                mesh: 0,
                plane: group,
            }
        } else {
            ConstructionPlaneIdentity {
                mesh: 1,
                plane: group - first_mesh_planes,
            }
        };
        if let [polygon_index] = polygon_indices {
            let source = &polygons[*polygon_index];
            let mesh_index = usize::try_from(source.mesh_index)
                .ok()
                .filter(|&mesh| mesh < support_planes.len())
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let mut face = source.clone();
            face.support = support_planes[support_identity.mesh][support_identity.plane].clone();
            face.delta_w = vec![0; support_planes.len()];
            face.delta_w[mesh_index] = 1;
            faces.push(face);
            face_supports.push(support_identity);
            continue;
        }
        let source_edge_count = polygon_indices.len().saturating_mul(3);
        let mut source_edges = SourceEdgeKeys::with_capacity(source_edge_count);
        for &polygon_index in polygon_indices {
            let polygon = &polygons[polygon_index];
            let edge_identities = polygon
                .known_edge_identities()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let vertex_identities = polygon
                .known_vertex_identities()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            let points = polygon
                .known_vertices
                .as_ref()
                .ok_or(crate::error::HypermeshError::UnknownClassification)?;
            if edge_identities.len() != vertex_identities.len()
                || points.len() != vertex_identities.len()
            {
                return Err(crate::error::HypermeshError::UnknownClassification);
            }
            for identity in vertex_identities {
                let ConstructionVertexIdentity::Source { mesh, .. } = identity else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                if mesh != support_identity.mesh {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
            }
            for identity in edge_identities.iter() {
                let ConstructionEdgeIdentity::Source { mesh, endpoints } = identity else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                if mesh != support_identity.mesh {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
                source_edges.push(endpoints);
            }
        }

        let mut single_use_vertices = Vec::new();
        if polygon_indices.len() <= MAX_SINGLE_USE_CERTIFICATE_TRIANGLES {
            let mut source_vertices = Vec::with_capacity(source_edge_count);
            for &polygon_index in polygon_indices {
                for identity in polygons[polygon_index]
                    .known_vertex_identities()
                    .expect("validated above")
                {
                    let ConstructionVertexIdentity::Source { vertex, .. } = identity else {
                        unreachable!("validated source vertex identity");
                    };
                    source_vertices.push(vertex);
                }
            }
            source_vertices.sort_unstable();
            let mut vertex_index = 0;
            while vertex_index < source_vertices.len() {
                let mut next = vertex_index + 1;
                while next < source_vertices.len()
                    && source_vertices[next] == source_vertices[vertex_index]
                {
                    next += 1;
                }
                // Its sole incident source triangle contains both boundary
                // neighbors. The supported input model excludes degenerate
                // source triangles, so this vertex is certifiably a corner.
                if next == vertex_index + 1 {
                    single_use_vertices.push(source_vertices[vertex_index]);
                }
                vertex_index = next;
            }
        }
        let boundary_edges = source_edges.into_unique();
        let mut outgoing = Vec::with_capacity(boundary_edges.len());
        for &polygon_index in polygon_indices {
            let polygon = &polygons[polygon_index];
            let edge_identities = polygon.known_edge_identities().expect("validated above");
            let vertex_identities = polygon.known_vertex_identities().expect("validated above");
            let points = polygon.known_vertices.as_ref().expect("validated above");
            for edge_index in 0..edge_identities.len() {
                let edge_identity = edge_identities
                    .get(edge_index)
                    .expect("validated edge identity index");
                let ConstructionEdgeIdentity::Source { mesh, endpoints } = &edge_identity else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                if *mesh != support_identity.mesh {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                }
                if boundary_edges.binary_search(endpoints).is_err() {
                    continue;
                }
                let ConstructionVertexIdentity::Source { vertex: start, .. } = vertex_identities
                    .get(edge_index)
                    .expect("validated vertex identity index")
                else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                let ConstructionVertexIdentity::Source { vertex: end, .. } = vertex_identities
                    .get((edge_index + 1) % vertex_identities.len())
                    .expect("validated vertex identity index")
                else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                let edge_plane = if polygon.edges.len() == edge_identities.len() {
                    Some(polygon.edges[edge_index].clone())
                } else if polygon.vertex_count() == 3 {
                    None
                } else {
                    return Err(crate::error::HypermeshError::UnknownClassification);
                };
                outgoing.push((
                    start,
                    end,
                    points.get(edge_index).expect("aligned vertex").clone(),
                    edge_plane,
                    edge_identity,
                ));
            }
        }
        outgoing.sort_unstable_by_key(|entry| entry.0);
        if outgoing.windows(2).any(|pair| pair[0].0 == pair[1].0) {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let Some(mut current) = outgoing.first().map(|entry| entry.0) else {
            return Err(crate::error::HypermeshError::UnknownClassification);
        };
        let start = current;
        let mut face_vertices = Vec::with_capacity(outgoing.len());
        let mut vertex_identities = Vec::with_capacity(outgoing.len());
        let mut optional_edge_planes = Vec::with_capacity(outgoing.len());
        let mut edge_identities = Vec::with_capacity(outgoing.len());
        while face_vertices.len() < outgoing.len() {
            let Ok(outgoing_index) = outgoing.binary_search_by_key(&current, |entry| entry.0)
            else {
                return Err(crate::error::HypermeshError::UnknownClassification);
            };
            let (_, next, point, edge_plane, edge_identity) = &outgoing[outgoing_index];
            face_vertices.push(point.clone());
            vertex_identities.push(ConstructionVertexIdentity::Source {
                mesh: support_identity.mesh,
                vertex: current,
            });
            optional_edge_planes.push(edge_plane.clone());
            edge_identities.push(edge_identity.clone());
            current = *next;
            if current == start {
                break;
            }
        }
        if current != start || face_vertices.len() != outgoing.len() {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let mut edge_planes = optional_edge_planes
            .into_iter()
            .collect::<Option<Vec<_>>>()
            .unwrap_or_default();
        let certified_noncollinear_vertices = (!single_use_vertices.is_empty()).then(|| {
            vertex_identities
                .iter()
                .map(|identity| {
                    let ConstructionVertexIdentity::Source { vertex, .. } = identity else {
                        unreachable!("validated source vertex identity");
                    };
                    single_use_vertices.binary_search(vertex).is_ok()
                })
                .collect::<Vec<_>>()
        });
        collapse_certified_collinear_face_vertices(
            support_identity.mesh,
            support_planes[support_identity.mesh][support_identity.plane],
            &mut face_vertices,
            &mut vertex_identities,
            &mut edge_planes,
            &mut edge_identities,
            certified_noncollinear_vertices.as_deref(),
        )?;
        let source = &polygons[polygon_indices[0]];
        let mesh_index = usize::try_from(source.mesh_index)
            .ok()
            .filter(|&mesh| mesh < support_planes.len())
            .ok_or(crate::error::HypermeshError::UnknownClassification)?;
        let mut delta_w = vec![0; support_planes.len()];
        delta_w[mesh_index] = 1;
        faces.push(ConvexPolygon::from_certified_convex_face(
            support_planes[support_identity.mesh][support_identity.plane].clone(),
            face_vertices,
            vertex_identities,
            edge_planes,
            edge_identities,
            source.mesh_index,
            source.polygon_index,
            delta_w,
        ));
        face_supports.push(support_identity);
    }
    Ok((faces, face_supports))
}

enum SourceEdgeKeys {
    Packed(Vec<u64>),
    Wide(Vec<[usize; 2]>),
}

impl SourceEdgeKeys {
    const PACKED_SORT_MIN_EDGES: usize = 128;

    fn with_capacity(capacity: usize) -> Self {
        if capacity >= Self::PACKED_SORT_MIN_EDGES {
            Self::Packed(Vec::with_capacity(capacity))
        } else {
            Self::Wide(Vec::with_capacity(capacity))
        }
    }

    fn push(&mut self, edge: [usize; 2]) {
        if let Self::Packed(packed) = self {
            if let [Ok(start), Ok(end)] = edge.map(u32::try_from) {
                packed.push((u64::from(start) << 32) | u64::from(end));
                return;
            }
            let mut wide = Vec::with_capacity(packed.capacity());
            wide.extend(
                packed
                    .drain(..)
                    .map(|edge| [(edge >> 32) as usize, (edge as u32) as usize]),
            );
            wide.push(edge);
            *self = Self::Wide(wide);
            return;
        }
        let Self::Wide(wide) = self else {
            unreachable!("packed edge path returns above");
        };
        wide.push(edge);
    }

    fn into_unique(self) -> Vec<[usize; 2]> {
        match self {
            Self::Packed(mut edges) => {
                edges.sort_unstable();
                unique_sorted_runs(&edges)
                    .map(|&edge| [(edge >> 32) as usize, (edge as u32) as usize])
                    .collect()
            }
            Self::Wide(mut edges) => {
                edges.sort_unstable();
                unique_sorted_runs(&edges).copied().collect()
            }
        }
    }
}

fn unique_sorted_runs<T: Eq>(values: &[T]) -> impl Iterator<Item = &T> {
    let mut index = 0;
    std::iter::from_fn(move || {
        while index < values.len() {
            let current = index;
            index += 1;
            while index < values.len() && values[index] == values[current] {
                index += 1;
            }
            if index == current + 1 {
                return Some(&values[current]);
            }
        }
        None
    })
}

fn collapse_certified_collinear_face_vertices(
    mesh: usize,
    support: &Plane,
    vertices: &mut Vec<Point3>,
    vertex_identities: &mut Vec<ConstructionVertexIdentity>,
    edges: &mut Vec<Plane>,
    edge_identities: &mut Vec<ConstructionEdgeIdentity>,
    certified_noncollinear_vertices: Option<&[bool]>,
) -> HypermeshResult<()> {
    let len = vertices.len();
    if vertex_identities.len() != len
        || (!edges.is_empty() && edges.len() != len)
        || edge_identities.len() != len
        || certified_noncollinear_vertices.is_some_and(|certified| certified.len() != len)
    {
        return Err(crate::error::HypermeshError::UnknownClassification);
    }
    if len <= 3 {
        return Ok(());
    }
    let rebuild_edge_planes = !edges.is_empty();
    if len > 3 {
        let retained = (0..len)
            .filter(|&index| {
                if certified_noncollinear_vertices.is_some_and(|certified| certified[index]) {
                    return true;
                }
                !support.points_are_collinear_on_support(
                    &vertices[(index + len - 1) % len],
                    &vertices[index],
                    &vertices[(index + 1) % len],
                )
            })
            .collect::<Vec<_>>();
        if retained.len() < 3 {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        if retained.len() == len {
            return Ok(());
        }
        if retained.len() != len {
            *vertices = retained
                .iter()
                .map(|&index| vertices[index].clone())
                .collect();
            *vertex_identities = retained
                .iter()
                .map(|&index| vertex_identities[index].clone())
                .collect();
        }
    }

    edges.clear();
    edge_identities.clear();
    if rebuild_edge_planes {
        edges.reserve(vertices.len());
    }
    edge_identities.reserve(vertices.len());
    for index in 0..vertices.len() {
        let next = (index + 1) % vertices.len();
        if rebuild_edge_planes {
            let after_next = (index + 2) % vertices.len();
            edges.push(edge_plane(
                &vertices[index],
                &vertices[next],
                &vertices[after_next],
                support,
            ));
        }
        let ConstructionVertexIdentity::Source {
            mesh: start_mesh,
            vertex: start,
        } = vertex_identities[index]
        else {
            return Err(crate::error::HypermeshError::UnknownClassification);
        };
        let ConstructionVertexIdentity::Source {
            mesh: end_mesh,
            vertex: end,
        } = vertex_identities[next]
        else {
            return Err(crate::error::HypermeshError::UnknownClassification);
        };
        if start_mesh != mesh || end_mesh != mesh {
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        let mut endpoints = [start, end];
        endpoints.sort_unstable();
        edge_identities.push(ConstructionEdgeIdentity::Source { mesh, endpoints });
    }
    Ok(())
}

fn exact_plane_storage_key(plane: &Plane) -> Option<[usize; 4]> {
    let [Some(a), Some(b), Some(c), Some(d)] = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ]
    .map(Real::exact_rational_ref) else {
        return None;
    };
    Some([a, b, c, d].map(Rational::storage_identity))
}

fn exact_plane_f64(plane: &Plane) -> Option<[f64; 4]> {
    let coefficients = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ];
    if coefficients
        .iter()
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
    {
        return None;
    }
    let [Some(a), Some(b), Some(c), Some(d)] = coefficients.map(Real::to_f64_lossy) else {
        return None;
    };
    Some([a, b, c, d])
}

fn exact_inside_and_outside_cycles<'a>(
    source: ProjectiveCycle,
    source_polygon: &ConvexPolygon,
    source_plane: ConstructionPlaneIdentity,
    support_planes: &[&'a Plane],
    support_planes_f64: Option<&[[f64; 4]]>,
    normalized_support_planes_f64: &[Option<[f64; 4]>],
    candidate_planes: &[usize],
    support_plane_mesh: usize,
    retain_outside: bool,
    prepared_verification_planes: &mut Vec<(usize, Option<PreparedRationalPlane4<'a>>)>,
    active_plane_proposal_scratch: &mut ActivePlaneProposalScratch,
    point_cache: &mut ProjectivePointCache,
) -> HypermeshResult<Option<(ProjectiveCycle, Option<Vec<ProjectiveCycle>>)>> {
    let mut source = Some(source);
    if let Some(proposed_planes) = support_planes_f64.and_then(|planes| {
        propose_active_planes_f64(
            source
                .as_ref()
                .expect("projective source is available for proposal"),
            planes,
            candidate_planes,
            active_plane_proposal_scratch,
        )
    }) {
        crate::trace_dispatch!("projective-active-planes", "proposed");
        let (inside, outside) = clip_inside_cycle_for_output(
            source
                .take()
                .expect("projective source is available for proposal"),
            support_planes,
            normalized_support_planes_f64,
            proposed_planes,
            support_plane_mesh,
            retain_outside,
            point_cache,
        )
        .inspect_err(|_error| {
            if cfg!(debug_assertions) {
                eprintln!("[DEBUG] proposed projective clipping failed");
            }
        })?;
        if inside.boundary.len() < 3 {
            crate::trace_dispatch!("projective-active-planes", "proposed-empty");
            return Ok(None);
        }
        if cycle_satisfies_planes(
            &inside,
            support_planes,
            candidate_planes,
            proposed_planes,
            support_plane_mesh,
            prepared_verification_planes,
            point_cache,
        )
        .inspect_err(|_error| {
            if cfg!(debug_assertions) {
                eprintln!("[DEBUG] proposed projective verification failed");
            }
        })? {
            crate::trace_dispatch!("projective-active-planes", "proposed-certified");
            return Ok(Some((inside, outside)));
        }
        crate::trace_dispatch!("projective-active-planes", "proposed-rejected");
        source = Some(ProjectiveCycle::from_polygon(
            source_polygon,
            source_plane,
            point_cache,
        )?);
    }

    crate::trace_dispatch!("projective-active-planes", "full");
    let (inside, outside) = clip_inside_cycle_for_output(
        source
            .take()
            .expect("projective source is available for full clipping"),
        support_planes,
        normalized_support_planes_f64,
        candidate_planes,
        support_plane_mesh,
        retain_outside,
        point_cache,
    )
    .inspect_err(|_error| {
        if cfg!(debug_assertions) {
            eprintln!("[DEBUG] full projective clipping failed");
        }
    })?;
    if inside.boundary.len() < 3 {
        crate::trace_dispatch!("projective-active-planes", "full-empty");
        return Ok(None);
    }
    crate::trace_dispatch!("projective-active-planes", "full-retained");
    Ok(Some((inside, outside)))
}

fn clip_inside_cycle_for_output(
    source: ProjectiveCycle,
    support_planes: &[&Plane],
    normalized_support_planes_f64: &[Option<[f64; 4]>],
    plane_indices: &[usize],
    support_plane_mesh: usize,
    retain_outside: bool,
    point_cache: &mut ProjectivePointCache,
) -> HypermeshResult<(ProjectiveCycle, Option<Vec<ProjectiveCycle>>)> {
    if retain_outside {
        let (inside, outside) = partition_inside_cycle(
            source,
            support_planes,
            normalized_support_planes_f64,
            plane_indices,
            support_plane_mesh,
            point_cache,
        )?;
        Ok((inside, Some(outside)))
    } else {
        Ok((
            clip_inside_cycle(
                source,
                support_planes,
                normalized_support_planes_f64,
                plane_indices,
                support_plane_mesh,
                point_cache,
            )?,
            None,
        ))
    }
}

fn clip_inside_cycle(
    source: ProjectiveCycle,
    support_planes: &[&Plane],
    normalized_support_planes_f64: &[Option<[f64; 4]>],
    plane_indices: &[usize],
    support_plane_mesh: usize,
    point_cache: &mut ProjectivePointCache,
) -> HypermeshResult<ProjectiveCycle> {
    let mut inside = source;
    for &plane_index in plane_indices {
        inside = inside.clip_negative(
            support_planes[plane_index],
            normalized_support_planes_f64[plane_index],
            ConstructionPlaneIdentity {
                mesh: support_plane_mesh,
                plane: plane_index,
            },
            point_cache,
        )?;
        if inside.boundary.len() < 3 {
            return Ok(ProjectiveCycle::empty());
        }
    }
    Ok(inside)
}

fn partition_inside_cycle(
    source: ProjectiveCycle,
    support_planes: &[&Plane],
    normalized_support_planes_f64: &[Option<[f64; 4]>],
    plane_indices: &[usize],
    support_plane_mesh: usize,
    point_cache: &mut ProjectivePointCache,
) -> HypermeshResult<(ProjectiveCycle, Vec<ProjectiveCycle>)> {
    let mut inside = source;
    let mut outside = Vec::new();
    for &plane_index in plane_indices {
        let clipped = inside.clip(
            support_planes[plane_index],
            normalized_support_planes_f64[plane_index],
            ConstructionPlaneIdentity {
                mesh: support_plane_mesh,
                plane: plane_index,
            },
            point_cache,
        )?;
        match clipped.side {
            ProjectiveClipSide::Negative => inside = clipped.negative,
            ProjectiveClipSide::Positive => {
                outside.push(clipped.positive);
                return Ok((ProjectiveCycle::empty(), outside));
            }
            ProjectiveClipSide::Both => {
                outside.push(clipped.positive);
                inside = clipped.negative;
            }
        }
        if inside.boundary.len() < 3 {
            return Ok((ProjectiveCycle::empty(), outside));
        }
    }
    Ok((inside, outside))
}

fn cycle_satisfies_planes<'a>(
    cycle: &ProjectiveCycle,
    support_planes: &[&'a Plane],
    plane_indices: &[usize],
    excluded_planes: &[usize],
    support_plane_mesh: usize,
    prepared_planes: &mut Vec<(usize, Option<PreparedRationalPlane4<'a>>)>,
    point_cache: &ProjectivePointCache,
) -> HypermeshResult<bool> {
    debug_assert!(plane_indices.is_sorted());
    debug_assert!(excluded_planes.is_sorted());
    prepared_planes.clear();
    let mut excluded_index = 0;
    for &plane_index in plane_indices {
        while excluded_planes
            .get(excluded_index)
            .is_some_and(|&excluded| excluded < plane_index)
        {
            excluded_index += 1;
        }
        if excluded_planes.get(excluded_index) == Some(&plane_index) {
            excluded_index += 1;
            continue;
        }
        prepared_planes.push((
            plane_index,
            PreparedRationalPlane4::new(support_planes[plane_index]),
        ));
    }
    for entry in &cycle.boundary {
        let prepared = PreparedProjectivePoint3::with_rational_filter_query(
            point_cache.point(entry.point_index),
            entry.preparation.rational_filter_query,
        );
        for (plane_index, prepared_plane) in prepared_planes.iter() {
            let classification = match prepared_plane {
                Some(plane) => match prepared.classify_rational_plane_filter(plane) {
                    Some(classification) => classification,
                    None if point_cache.has_incidence(
                        &entry.point_identity,
                        ConstructionPlaneIdentity {
                            mesh: support_plane_mesh,
                            plane: *plane_index,
                        },
                    ) =>
                    {
                        Classification::On
                    }
                    None => match prepared.classify_rational_plane_exact(plane) {
                        Some(classification) => classification,
                        None => prepared.classify(support_planes[*plane_index])?,
                    },
                },
                None => prepared.classify(support_planes[*plane_index])?,
            };
            if classification.is_positive() {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

#[derive(Default)]
struct ActivePlaneProposalScratch {
    cycle: Vec<[f64; 3]>,
    clipped: Vec<[f64; 3]>,
    crossed_planes: Vec<usize>,
    active: Vec<usize>,
}

fn propose_active_planes_f64<'a>(
    source: &ProjectiveCycle,
    planes: &[[f64; 4]],
    candidate_planes: &[usize],
    scratch: &'a mut ActivePlaneProposalScratch,
) -> Option<&'a [usize]> {
    scratch.cycle.clear();
    for entry in &source.boundary {
        scratch.cycle.push(entry.preparation.approximate?);
    }
    scratch.crossed_planes.clear();
    scratch.active.clear();
    for &plane_index in candidate_planes {
        let crossed =
            clip_f64_cycle_into(&scratch.cycle, planes[plane_index], &mut scratch.clipped);
        std::mem::swap(&mut scratch.cycle, &mut scratch.clipped);
        if scratch.cycle.len() < 3 {
            return Some(&scratch.active);
        }
        if crossed {
            scratch.crossed_planes.push(plane_index);
        }
    }
    for &plane_index in &scratch.crossed_planes {
        let plane = planes[plane_index];
        let points_on_plane = scratch
            .cycle
            .iter()
            .filter(|point| {
                let value = f64_plane_value(**point, plane);
                let scale = (plane[0] * point[0]).abs()
                    + (plane[1] * point[1]).abs()
                    + (plane[2] * point[2]).abs()
                    + plane[3].abs();
                value.abs() <= 1.0e-8 * scale.max(1.0)
            })
            .take(2)
            .count();
        if points_on_plane == 2 {
            scratch.active.push(plane_index);
        }
    }
    Some(&scratch.active)
}

fn clip_f64_cycle_into(points: &[[f64; 3]], plane: [f64; 4], clipped: &mut Vec<[f64; 3]>) -> bool {
    clipped.clear();
    clipped.reserve(points.len() + 1);
    let mut crossed = false;
    let Some(first) = points.first() else {
        return crossed;
    };
    let first_value = f64_plane_value(*first, plane);
    let mut current_value = first_value;
    for index in 0..points.len() {
        let next = (index + 1) % points.len();
        let next_value = if next == 0 {
            first_value
        } else {
            f64_plane_value(points[next], plane)
        };
        let current_inside = current_value <= 0.0;
        let next_inside = next_value <= 0.0;
        match (current_inside, next_inside) {
            (true, true) => clipped.push(points[next]),
            (true, false) => {
                crossed = true;
                clipped.push(f64_segment_plane_intersection(
                    points[index],
                    points[next],
                    current_value,
                    next_value,
                ));
            }
            (false, true) => {
                crossed = true;
                clipped.push(f64_segment_plane_intersection(
                    points[index],
                    points[next],
                    current_value,
                    next_value,
                ));
                clipped.push(points[next]);
            }
            (false, false) => {}
        }
        current_value = next_value;
    }
    crossed
}

fn f64_segment_plane_intersection(
    start: [f64; 3],
    end: [f64; 3],
    start_value: f64,
    end_value: f64,
) -> [f64; 3] {
    let parameter = start_value / (start_value - end_value);
    std::array::from_fn(|axis| start[axis] + parameter * (end[axis] - start[axis]))
}

fn f64_plane_value(point: [f64; 3], plane: [f64; 4]) -> f64 {
    plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3]
}

fn positive_weight_plane_intersection(planes: [&Plane; 3]) -> Option<HomogeneousPoint3> {
    let point = intersect_three_planes(planes[0], planes[1], planes[2]);
    let weight = point.w.exact_rational_ref()?;
    if weight.is_positive() {
        Some(point)
    } else if weight.is_negative() {
        Some(HomogeneousPoint3::new(
            -point.x, -point.y, -point.z, -point.w,
        ))
    } else {
        None
    }
}

fn projective_crossing_point(
    current: &HomogeneousPoint3,
    current_value: &Real,
    current_classification: Classification,
    next: &HomogeneousPoint3,
    next_value: &Real,
) -> HomogeneousPoint3 {
    let (negative, negative_value, positive, positive_value) =
        if current_classification.is_negative() {
            (current, current_value, next, next_value)
        } else {
            (next, next_value, current, current_value)
        };
    let coordinate = |negative_coordinate: &Real, positive_coordinate: &Real| {
        Real::signed_product_sum(
            [true, false],
            [
                [positive_value, negative_coordinate],
                [negative_value, positive_coordinate],
            ],
        )
    };
    HomogeneousPoint3::new(
        coordinate(&negative.x, &positive.x),
        coordinate(&negative.y, &positive.y),
        coordinate(&negative.z, &positive.z),
        coordinate(&negative.w, &positive.w),
    )
}

fn push_projective_transition(
    classified: &mut Vec<ClassifiedPolygon>,
    cycle: ProjectiveCycle,
    source: &ConvexPolygon,
    affine_cache: &mut ProjectiveAffineCache,
    point_cache: &ProjectivePointCache,
    host: usize,
    other: usize,
    inside_other: bool,
    operation: BooleanOp,
) -> HypermeshResult<()> {
    if cycle.boundary.len() < 3 {
        return Ok(());
    }
    let winding = projective_transition_winding(host, other, inside_other);
    if !projective_transition_is_emitted(host, inside_other, operation) {
        return Ok(());
    }
    let polygon = cycle.materialize(source, affine_cache, point_cache)?;
    let mut fragment = ClassifiedPolygon::new(polygon, ARRANGEMENT_CLASSIFICATION);
    fragment.winding = Some(winding);
    fragment.is_bsp_fragment = true;
    classified.push(fragment);
    Ok(())
}

fn push_source_transition(
    classified: &mut Vec<ClassifiedPolygon>,
    source: &ConvexPolygon,
    host: usize,
    other: usize,
    inside_other: bool,
) -> HypermeshResult<()> {
    if source.vertex_count() < 3 {
        return Ok(());
    }
    let mut fragment = ClassifiedPolygon::new(source.clone(), ARRANGEMENT_CLASSIFICATION);
    fragment.winding = Some(projective_transition_winding(host, other, inside_other));
    fragment.is_bsp_fragment = true;
    classified.push(fragment);
    Ok(())
}

fn projective_transition_winding(host: usize, other: usize, inside_other: bool) -> WindingPair {
    let mut w_front = vec![0; 2];
    w_front[other] = i32::from(inside_other);
    let mut w_back = w_front.clone();
    w_back[host] = 1;
    WindingPair { w_front, w_back }
}

fn projective_transition_is_emitted(host: usize, inside_other: bool, operation: BooleanOp) -> bool {
    match operation {
        BooleanOp::Union => !inside_other,
        BooleanOp::Intersection => inside_other,
        BooleanOp::Difference => (host == 0 && !inside_other) || (host == 1 && inside_other),
        BooleanOp::SymmetricDifference => true,
    }
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

/// Symmetric-difference convenience wrapper.
pub fn boolean_symmetric_difference(
    a: MeshRef<'_>,
    b: MeshRef<'_>,
    config: EmberConfig,
) -> HypermeshResult<BooleanResult> {
    boolean_operation(&[a, b], BooleanOp::SymmetricDifference, config)
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
    fn projective_source_edge_supports_are_unique_and_bounded() {
        let first = ConstructionPlaneIdentity { mesh: 0, plane: 4 };
        let second = ConstructionPlaneIdentity { mesh: 0, plane: 7 };
        let third = ConstructionPlaneIdentity { mesh: 0, plane: 9 };
        let mut source_edge = ProjectiveSourceEdge::new([2, 3], first);

        assert_eq!(source_edge.supports(), &[first]);
        assert!(source_edge.insert_support(first));
        assert!(source_edge.insert_support(second));
        assert_eq!(source_edge.supports(), &[first, second]);
        assert!(!source_edge.insert_support(third));
        assert_eq!(source_edge.supports(), &[first, second]);
    }

    #[test]
    fn source_edge_keys_preserve_unique_runs_across_packed_and_wide_storage() {
        let edges = [[1, 2], [4, 5], [2, 3], [1, 2], [4, 5], [4, 5]];
        let collect = |capacity| {
            let mut keys = SourceEdgeKeys::with_capacity(capacity);
            for edge in edges {
                keys.push(edge);
            }
            keys.into_unique()
        };

        assert_eq!(collect(edges.len()), vec![[2, 3]]);
        assert_eq!(collect(SourceEdgeKeys::PACKED_SORT_MIN_EDGES), vec![[2, 3]]);

        if usize::BITS > u32::BITS {
            let oversized = usize::try_from(u64::from(u32::MAX) + 1).unwrap();
            let mut fallback = SourceEdgeKeys::with_capacity(SourceEdgeKeys::PACKED_SORT_MIN_EDGES);
            fallback.push([1, 2]);
            fallback.push([oversized, oversized + 1]);
            fallback.push([1, 2]);
            assert_eq!(fallback.into_unique(), vec![[oversized, oversized + 1]]);
        }
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

    #[test]
    fn plane_equivalence_separates_proportionality_from_orientation() {
        let plane =
            Plane::from_coefficients(Real::from(2), Real::from(4), Real::from(6), Real::from(8));
        let same =
            Plane::from_coefficients(Real::from(3), Real::from(6), Real::from(9), Real::from(12));
        let opposite = Plane::from_coefficients(
            Real::from(-3),
            Real::from(-6),
            Real::from(-9),
            Real::from(-12),
        );
        let distinct =
            Plane::from_coefficients(Real::from(3), Real::from(6), Real::from(9), Real::from(13));

        assert!(certifiably_same_oriented_plane(&plane, &same).unwrap());
        assert!(!certifiably_same_oriented_plane(&plane, &opposite).unwrap());
        assert!(certifiably_same_unoriented_plane(&plane, &opposite));
        assert!(!certifiably_same_unoriented_plane(&plane, &distinct));

        let symbolic =
            Plane::from_coefficients(Real::pi(), Real::one(), Real::zero(), Real::from(2));
        assert!(certifiably_same_oriented_plane(&symbolic, &symbolic).unwrap());
    }

    #[test]
    fn projective_cycle_expands_deferred_source_edges_on_demand() {
        let polygon = crate::polygon::make_triangle_with_deferred_edges(
            &p(0, 0, 0),
            &p(1, 0, 0),
            &p(0, 1, 0),
            0,
            0,
        )
        .with_source_triangle_edge_identities(0, [0, 1, 2]);
        assert!(polygon.edges.is_empty());
        assert_eq!(polygon.vertex_count(), 3);

        let mut point_cache = ProjectivePointCache::default();
        let cycle = ProjectiveCycle::from_polygon(
            &polygon,
            ConstructionPlaneIdentity { mesh: 0, plane: 0 },
            &mut point_cache,
        )
        .unwrap();
        assert_eq!(cycle.boundary.len(), 3);
        assert!(
            cycle
                .boundary
                .iter()
                .all(|entry| point_cache.plane(entry.edge_index) == &polygon.support)
        );
    }

    #[test]
    fn consuming_projective_split_preserves_shared_boundary_identities() {
        let polygon = crate::polygon::make_triangle(&p(0, 0, -1), &p(2, 0, 1), &p(0, 2, 0), 0, 0)
            .with_source_triangle_edge_identities(0, [0, 1, 2]);
        let mut point_cache = ProjectivePointCache::default();
        let cycle = ProjectiveCycle::from_polygon(
            &polygon,
            ConstructionPlaneIdentity { mesh: 0, plane: 0 },
            &mut point_cache,
        )
        .unwrap();
        let plane = Plane::axis_aligned(2, Real::zero());
        let negative_only = cycle
            .clone()
            .clip_negative(
                &plane,
                None,
                ConstructionPlaneIdentity { mesh: 1, plane: 0 },
                &mut point_cache,
            )
            .unwrap();
        let split = cycle
            .clip(
                &plane,
                None,
                ConstructionPlaneIdentity { mesh: 1, plane: 0 },
                &mut point_cache,
            )
            .unwrap();
        let affine = |cycle: &ProjectiveCycle| {
            cycle
                .boundary
                .iter()
                .map(|entry| {
                    point_cache
                        .point(entry.point_index)
                        .to_affine_point()
                        .unwrap()
                })
                .collect::<Vec<_>>()
        };

        assert!(matches!(split.side, ProjectiveClipSide::Both));
        assert_eq!(
            affine(&split.negative),
            vec![p(0, 0, -1), p(1, 0, 0), p(0, 2, 0)]
        );
        assert_eq!(
            affine(&split.positive),
            vec![p(1, 0, 0), p(2, 0, 1), p(0, 2, 0)]
        );
        assert_eq!(
            split.negative.boundary[1].point_identity,
            split.positive.boundary[0].point_identity
        );
        assert!(
            split.negative.boundary[1]
                .preparation
                .rational_filter_query
                .is_some()
        );
        assert!(
            split.positive.boundary[0]
                .preparation
                .rational_filter_query
                .is_some()
        );
        assert_eq!(
            split.negative.boundary[2].point_identity,
            split.positive.boundary[2].point_identity
        );
        assert_eq!(
            split.negative.boundary[0].edge_identity,
            split.positive.boundary[0].edge_identity
        );
        assert_eq!(
            split.negative.boundary[1].edge_identity,
            split.positive.boundary[2].edge_identity
        );
        assert_eq!(affine(&negative_only), affine(&split.negative));
        assert!(
            negative_only
                .boundary
                .iter()
                .zip(&split.negative.boundary)
                .all(|(negative, split)| {
                    negative.point_identity == split.point_identity
                        && negative.edge_index == split.edge_index
                        && negative.edge_identity == split.edge_identity
                })
        );
    }

    #[test]
    fn singleton_certified_face_preserves_deferred_edges() {
        let polygon = crate::polygon::make_triangle_with_deferred_edges(
            &p(0, 0, 0),
            &p(1, 0, 0),
            &p(0, 1, 0),
            0,
            0,
        )
        .with_source_triangle_edge_identities(0, [0, 1, 2]);
        let support_identity = ConstructionPlaneIdentity { mesh: 0, plane: 0 };
        let polygons = [polygon];
        let supports = [vec![&polygons[0].support], Vec::new()];

        let (faces, face_supports) =
            collapse_certified_convex_faces(&polygons, &[support_identity], &supports).unwrap();

        assert_eq!(faces.len(), 1);
        assert!(faces[0].edges.is_empty());
        assert_eq!(faces[0].vertex_count(), 3);
        assert_eq!(faces[0].delta_w, vec![1, 0]);
        assert_eq!(face_supports, vec![support_identity]);
    }

    #[test]
    fn certified_face_grouping_rejects_unaligned_or_out_of_range_supports() {
        let polygon = crate::polygon::make_triangle_with_deferred_edges(
            &p(0, 0, 0),
            &p(1, 0, 0),
            &p(0, 1, 0),
            0,
            0,
        )
        .with_source_triangle_edge_identities(0, [0, 1, 2]);
        let polygons = [polygon];
        let supports = [vec![&polygons[0].support], Vec::new()];

        assert!(collapse_certified_convex_faces(&polygons, &[], &supports).is_err());
        assert!(
            collapse_certified_convex_faces(
                &polygons,
                &[ConstructionPlaneIdentity { mesh: 0, plane: 1 }],
                &supports,
            )
            .is_err()
        );
        assert!(
            collapse_certified_convex_faces(
                &polygons,
                &[ConstructionPlaneIdentity { mesh: 2, plane: 0 }],
                &supports,
            )
            .is_err()
        );
    }

    #[test]
    fn merged_certified_face_cycle_is_independent_of_triangle_order() {
        let positions: std::sync::Arc<[Point3]> =
            std::sync::Arc::from([p(0, 0, 0), p(1, 0, 0), p(1, 1, 0), p(0, 1, 0)]);
        let first = crate::polygon::make_indexed_triangle_with_deferred_edges(
            positions.clone(),
            [0, 1, 2],
            None,
            std::sync::Arc::new(Vec::new()),
            0,
            0,
        )
        .with_source_triangle_edge_identities(0, [0, 1, 2]);
        let second = crate::polygon::make_indexed_triangle_with_deferred_edges(
            positions,
            [0, 2, 3],
            Some(first.support.clone()),
            std::sync::Arc::new(Vec::new()),
            0,
            1,
        )
        .with_source_triangle_edge_identities(0, [0, 2, 3]);
        let support_identity = ConstructionPlaneIdentity { mesh: 0, plane: 0 };

        let collapse = |polygons: &[ConvexPolygon]| {
            let supports = [vec![&polygons[0].support], Vec::new()];
            collapse_certified_convex_faces(
                polygons,
                &[support_identity, support_identity],
                &supports,
            )
            .unwrap()
            .0
            .pop()
            .unwrap()
        };
        let forward = collapse(&[first.clone(), second.clone()]);
        let reverse = collapse(&[second, first]);

        let expected_vertices =
            [0, 1, 2, 3].map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex });
        assert_eq!(
            forward
                .known_vertex_identities()
                .unwrap()
                .iter()
                .collect::<Vec<_>>(),
            expected_vertices
        );
        assert_eq!(
            reverse
                .known_vertex_identities()
                .unwrap()
                .iter()
                .collect::<Vec<_>>(),
            expected_vertices
        );
        assert!(forward.approx_bounds.is_none());
        assert!(reverse.approx_bounds.is_none());
        assert_eq!(
            forward.known_edge_identities(),
            reverse.known_edge_identities()
        );
    }

    #[test]
    fn certified_collinear_face_vertices_collapse_to_composite_source_edges() {
        let support = Plane::axis_aligned(2, Real::zero());
        let mut vertices = vec![
            p(0, 0, 0),
            p(1, 0, 0),
            p(2, 0, 0),
            p(2, 1, 0),
            p(2, 2, 0),
            p(1, 2, 0),
            p(0, 2, 0),
            p(0, 1, 0),
        ];
        let mut vertex_identities = (0..vertices.len())
            .map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
            .collect::<Vec<_>>();
        let mut edges = (0..vertices.len())
            .map(|index| {
                edge_plane(
                    &vertices[index],
                    &vertices[(index + 1) % vertices.len()],
                    &vertices[(index + 2) % vertices.len()],
                    &support,
                )
            })
            .collect::<Vec<_>>();
        let mut edge_identities = (0..vertices.len())
            .map(|start| ConstructionEdgeIdentity::Source {
                mesh: 0,
                endpoints: [start, (start + 1) % vertices.len()],
            })
            .collect::<Vec<_>>();

        collapse_certified_collinear_face_vertices(
            0,
            &support,
            &mut vertices,
            &mut vertex_identities,
            &mut edges,
            &mut edge_identities,
            None,
        )
        .unwrap();

        assert_eq!(
            vertices,
            vec![p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0)]
        );
        assert_eq!(
            vertex_identities,
            [0, 2, 4, 6].map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
        );
        assert_eq!(edges.len(), 4);
        assert_eq!(
            edge_identities,
            [[0, 2], [2, 4], [4, 6], [0, 6]]
                .map(|endpoints| { ConstructionEdgeIdentity::Source { mesh: 0, endpoints } })
        );
    }

    #[test]
    fn certified_nondegenerate_face_preserves_supplied_boundaries() {
        let support = Plane::axis_aligned(2, Real::zero());
        let mut vertices = vec![p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0)];
        let mut vertex_identities = (0..vertices.len())
            .map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
            .collect::<Vec<_>>();
        let mut edges = (0..vertices.len())
            .map(|index| Plane::axis_aligned(index % 3, Real::from(index as i64 + 7)))
            .collect::<Vec<_>>();
        let mut edge_identities = (0..vertices.len())
            .map(|start| ConstructionEdgeIdentity::Source {
                mesh: 0,
                endpoints: [start, (start + 1) % vertices.len()],
            })
            .collect::<Vec<_>>();
        let expected_vertices = vertices.clone();
        let expected_vertex_identities = vertex_identities.clone();
        let expected_edges = edges.clone();
        let expected_edge_identities = edge_identities.clone();

        collapse_certified_collinear_face_vertices(
            0,
            &support,
            &mut vertices,
            &mut vertex_identities,
            &mut edges,
            &mut edge_identities,
            None,
        )
        .unwrap();

        assert_eq!(vertices, expected_vertices);
        assert_eq!(vertex_identities, expected_vertex_identities);
        assert_eq!(edges, expected_edges);
        assert_eq!(edge_identities, expected_edge_identities);
    }

    #[test]
    fn certified_face_preserves_deferred_boundaries_after_validation() {
        let support = Plane::axis_aligned(2, Real::zero());
        let mut vertices = vec![p(0, 0, 0), p(2, 0, 0), p(2, 2, 0), p(0, 2, 0)];
        let mut vertex_identities = (0..vertices.len())
            .map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
            .collect::<Vec<_>>();
        let mut edges = Vec::new();
        let mut edge_identities = [[0, 1], [1, 2], [2, 3], [0, 3]]
            .map(|endpoints| ConstructionEdgeIdentity::Source { mesh: 0, endpoints })
            .to_vec();
        let expected_edge_identities = edge_identities.clone();

        collapse_certified_collinear_face_vertices(
            0,
            &support,
            &mut vertices,
            &mut vertex_identities,
            &mut edges,
            &mut edge_identities,
            None,
        )
        .unwrap();

        assert!(edges.is_empty());
        assert_eq!(edge_identities, expected_edge_identities);
    }

    #[test]
    fn exact_support_projection_certifies_collinear_face_vertices() {
        let support = Plane::from_points(&p(0, 0, 0), &p(1, 0, 1), &p(0, 1, 1));

        assert!(support.points_are_collinear_on_support(&p(0, 0, 0), &p(1, 1, 2), &p(2, 2, 4),));
        assert!(!support.points_are_collinear_on_support(&p(0, 0, 0), &p(1, 0, 1), &p(0, 1, 1),));
    }

    #[test]
    fn projective_cycle_verification_reuses_exact_plane_incidences() {
        let polygon = crate::polygon::make_triangle_with_deferred_edges(
            &p(0, 0, 0),
            &p(1, 0, 0),
            &p(0, 1, 0),
            0,
            0,
        )
        .with_source_triangle_edge_identities(0, [0, 1, 2]);
        let mut point_cache = ProjectivePointCache::default();
        let cycle = ProjectiveCycle::from_polygon(
            &polygon,
            ConstructionPlaneIdentity { mesh: 0, plane: 0 },
            &mut point_cache,
        )
        .unwrap();
        for entry in &cycle.boundary {
            point_cache.record_incidence(
                &entry.point_identity,
                ConstructionPlaneIdentity { mesh: 0, plane: 0 },
            );
        }

        let mut prepared_planes = Vec::new();
        assert!(
            cycle_satisfies_planes(
                &cycle,
                &[&polygon.support],
                &[0],
                &[],
                0,
                &mut prepared_planes,
                &point_cache,
            )
            .unwrap()
        );
    }

    #[test]
    fn source_relation_stops_after_exact_crossing_is_certified() {
        let polygon = crate::polygon::make_triangle(&p(0, 0, 1), &p(0, 0, -1), &p(1, 0, 0), 0, 0);
        let plane = Plane::axis_aligned(2, Real::zero());
        let mut cache = PointPlaneClassificationCache::default();

        assert!(matches!(
            cache.source_relation(&polygon, &plane, 0, 1).unwrap(),
            (SourcePlaneRelation::Crossing, _)
        ));
        assert_eq!(cache.points.len(), 2);
    }

    #[test]
    fn source_relation_indexes_certified_source_vertices_without_coordinate_hashing() {
        let polygon = crate::polygon::make_triangle(&p(0, 0, 1), &p(0, 0, -1), &p(1, 0, 0), 0, 0)
            .with_source_triangle_edge_identities(0, [7, 9, 11]);
        let plane = Plane::axis_aligned(2, Real::zero());
        let mut cache = PointPlaneClassificationCache::default();

        assert!(matches!(
            cache.source_relation(&polygon, &plane, 0, 1).unwrap(),
            (SourcePlaneRelation::Crossing, _)
        ));
        assert!(cache.points.is_empty());
        assert_eq!(
            cache
                .source_queries
                .iter()
                .filter(|cached| cached.is_some())
                .count(),
            2
        );
        assert_eq!(
            cache
                .source_classifications
                .iter()
                .filter(|classification| classification.is_some())
                .count(),
            2
        );
        assert_eq!(cache.source_plane_count, Some(1));
    }

    #[test]
    fn canonical_plane_identities_unify_cross_mesh_geometric_planes() {
        let bottom = Plane::axis_aligned(2, Real::zero());
        let opposite_bottom = bottom.inverted();
        let top = Plane::axis_aligned(2, Real::one());
        let support_planes = [vec![&bottom, &top], vec![&opposite_bottom, &bottom]];
        let normalized = support_planes
            .each_ref()
            .map(|planes| planes.iter().map(|plane| plane_f64(plane)).collect());

        let identities = canonical_plane_identities(&support_planes, &normalized);
        assert_eq!(identities[0][0], identities[1][0]);
        assert_eq!(identities[0][0], identities[1][1]);
        assert_ne!(identities[0][0], identities[0][1]);
    }

    #[test]
    fn cached_exact_plane_normalization_matches_direct_conversion() {
        let plane =
            Plane::from_coefficients(Real::from(3), Real::from(-4), Real::from(12), Real::from(7));
        let raw = exact_plane_f64(&plane).unwrap();

        assert_eq!(normalize_plane_f64(raw), plane_f64(&plane));
    }

    #[test]
    fn plane_intersection_normalizes_negative_homogeneous_weight() {
        let planes = [
            Plane::axis_aligned(1, Real::from(2)),
            Plane::axis_aligned(0, Real::from(1)),
            Plane::axis_aligned(2, Real::from(3)),
        ];

        let point = positive_weight_plane_intersection(planes.each_ref()).unwrap();
        assert!(
            point
                .w
                .exact_rational_ref()
                .is_some_and(Rational::is_positive)
        );
        assert_eq!(point.to_affine_point().unwrap(), p(1, 2, 3));
    }

    #[test]
    fn vertex_coincidences_resolve_atomically_to_order_independent_identity() {
        let planes = [
            Plane::axis_aligned(0, Real::from(1)),
            Plane::axis_aligned(1, Real::from(2)),
            Plane::axis_aligned(2, Real::from(3)),
            Plane::from_coefficients(Real::one(), Real::zero(), Real::one(), Real::from(-4)),
        ];
        let plane_ids: [ConstructionPlaneIdentity; 4] =
            std::array::from_fn(|plane| ConstructionPlaneIdentity { mesh: 0, plane });
        let identities = [
            ConstructionVertexIdentity::PlaneTriple {
                planes: [plane_ids[0], plane_ids[1], plane_ids[2]],
            },
            ConstructionVertexIdentity::PlaneTriple {
                planes: [plane_ids[0], plane_ids[1], plane_ids[3]],
            },
        ];

        let resolve = |order: [usize; 2]| {
            let mut cache = ProjectivePointCache::default();
            for (identity, plane) in plane_ids.into_iter().zip(planes.iter().cloned()) {
                cache.support_plane_index(identity, &plane);
            }
            for index in order {
                let definition = cache.definition_planes(&identities[index]).unwrap();
                let point = positive_weight_plane_intersection(definition).unwrap();
                let (_, _, interned) =
                    cache.intern_with_approximation_by(identities[index].clone(), || point);
                assert_eq!(interned, identities[index]);
            }

            cache.resolve_vertex_coincidences();
            let canonical = identities
                .each_ref()
                .map(|identity| cache.canonical_vertex_identity(identity));
            let canonical_point_indices = identities
                .each_ref()
                .map(|identity| cache.points[identity].point_index);
            assert_eq!(cache.canonical_identities.len(), 1);
            assert_eq!(canonical_point_indices[0], canonical_point_indices[1]);
            for identity in &identities {
                if *identity == canonical[0] {
                    assert!(!cache.canonical_identities.contains_key(identity));
                } else {
                    assert_eq!(
                        cache.canonical_identities.get(identity),
                        Some(&canonical[0])
                    );
                }
            }
            canonical
        };

        let forward = resolve([0, 1]);
        let reverse = resolve([1, 0]);
        assert_eq!(forward, reverse);
        assert_eq!(forward[0], forward[1]);
        assert_eq!(
            forward[0],
            std::cmp::min(identities[0].clone(), identities[1].clone())
        );
    }
}
