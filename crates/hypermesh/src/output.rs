//! Boolean result extraction and triangulation helpers.

use std::borrow::Borrow;
use std::collections::{BTreeMap, BTreeSet};

use crate::context::{DecisionContext, MeshCertainty, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, classify_real, compare_real_decision};
use crate::mesh::{OutputVertex, PolygonSoup, Triangle, TriangleMesh};
use crate::point_interner::{PointCoordinates, PointInterner};
use crate::polygon::{
    ConstructionEdgeIdentity, ConstructionPlaneIdentity, ConstructionVertexIdentity, ConvexPolygon,
};
use crate::storage_hash::StorageHashMap;
use crate::winding::WindingPair;
use hyperlattice::{Point3, Rational, Real};
use hyperreal::{RationalLine2Filter, RationalPoint3Query, RealSign};

pub(crate) const ARRANGEMENT_CLASSIFICATION: i8 = 2;

type SplitEdgeCache = StorageHashMap<[usize; 2], SplitEdgeChain>;
type ApproximateOutputVertex = [[f64; 2]; 3];
type ApproximateEdgeBounds = [[f64; 2]; 3];
const MIN_ADAPTIVE_CROSSING_SWEEP_EDGES: usize = 256;
// Small sweeps repay one left-bound computation, but not a full side cache.
const MIN_CACHED_CROSSING_BOUNDS_EDGES: usize = 1024;

struct SplitEdgeChain(Vec<usize>);

impl SplitEdgeChain {
    fn subedges(&self) -> impl DoubleEndedIterator<Item = [usize; 2]> + '_ {
        self.0
            .windows(2)
            .filter_map(|pair| (pair[0] != pair[1]).then_some([pair[0], pair[1]]))
    }
}

enum SplitEdgeSearch<'a> {
    Approximate(&'a [ApproximateOutputVertex]),
    AxisOrder(&'a [Vec<usize>; 3]),
}

/// Polygon plus its boolean output classification.
#[derive(Clone, Debug, PartialEq)]
pub struct ClassifiedPolygon {
    /// Classified polygon.
    pub(crate) polygon: ConvexPolygon,
    /// `+1` emits as-is, `-1` emits inverted.
    pub(crate) classification: i8,
    /// Optional front/back winding evidence.
    pub(crate) winding: Option<WindingPair>,
    /// Whether this polygon came from face-local BSP splitting.
    pub(crate) is_bsp_fragment: bool,
}

impl ClassifiedPolygon {
    /// Constructs a classified polygon.
    pub(crate) fn new(polygon: ConvexPolygon, classification: i8) -> Self {
        Self {
            polygon,
            classification,
            winding: None,
            is_bsp_fragment: false,
        }
    }

    /// Returns the classified polygon.
    pub fn polygon(&self) -> &ConvexPolygon {
        &self.polygon
    }

    /// Returns the output classification sign.
    pub const fn classification(&self) -> i8 {
        self.classification
    }

    /// Returns the certified front/back winding evidence, when available.
    pub const fn winding(&self) -> Option<&WindingPair> {
        self.winding.as_ref()
    }

    /// Returns whether this polygon came from face-local BSP splitting.
    pub const fn is_bsp_fragment(&self) -> bool {
        self.is_bsp_fragment
    }
}

#[cfg(test)]
pub(crate) fn push_unique_classified_polygon(
    classified: &mut Vec<ClassifiedPolygon>,
    candidate: ClassifiedPolygon,
) {
    if let Some(existing) = classified.iter_mut().find(|existing| {
        existing.classification == candidate.classification
            && polygons_match_output_geometry(&existing.polygon, &candidate.polygon)
    }) {
        if existing.winding.is_none() {
            existing.winding = candidate.winding;
        }
        existing.is_bsp_fragment |= candidate.is_bsp_fragment;
        return;
    }
    classified.push(candidate);
}

#[derive(Clone)]
struct ClassifiedPolygonBucket {
    classification: i8,
    support: crate::geometry::Plane,
    edge_count: usize,
    indices: Vec<usize>,
}

pub(crate) struct ClassifiedPolygonBucketState {
    buckets: Vec<ClassifiedPolygonBucket>,
}

#[derive(Clone)]
struct ClassifiedOutputBucket {
    classification: i8,
    support: crate::geometry::Plane,
    edge_count: usize,
    edge_profile: Vec<usize>,
    indices: Vec<usize>,
}

#[derive(Clone, Copy, Eq, Hash, PartialEq)]
struct ClassifiedOutputBucketFingerprint {
    classification: i8,
    edge_count: usize,
    support: Option<[u64; 4]>,
    edge_profile: u64,
}

struct PlaneProfileInterner {
    planes: Vec<Plane>,
    approximate: StorageHashMap<[u64; 4], Vec<usize>>,
}

pub(crate) fn merge_unique_classified_polygons(
    classified: &mut Vec<ClassifiedPolygon>,
    incoming: Vec<ClassifiedPolygon>,
) {
    let mut buckets = ClassifiedPolygonBucketState::from_classified(classified);
    merge_unique_classified_polygons_with_bucket_state(classified, &mut buckets, incoming);
}

/// Result of a boolean operation.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanResult {
    /// Output polygon soup.
    output: PolygonSoup,
    /// Per-output-polygon classifications.
    classifications: Vec<i8>,
    /// Per-output-polygon front/back winding evidence, when produced by the
    /// general subdivision classifier.
    winding_pairs: Vec<Option<WindingPair>>,
}

#[derive(Clone, Debug)]
pub(crate) struct ClassifiedTriangleArrangement {
    pub(crate) soup: BooleanMesh,
    pub(crate) windings: Vec<WindingPair>,
}

impl BooleanResult {
    /// Constructs a result from an output soup and classifications.
    #[cfg(test)]
    fn new(output: PolygonSoup, classifications: Vec<i8>) -> Self {
        let winding_pairs = vec![None; classifications.len()];
        Self {
            output,
            classifications,
            winding_pairs,
        }
    }

    /// Builds a result by applying classification orientation to owned
    /// classified polygons.
    pub(crate) fn from_classified(
        mut output: PolygonSoup,
        classified: Vec<ClassifiedPolygon>,
    ) -> Self {
        output.polygons.clear();
        let mut classifications = Vec::with_capacity(classified.len());
        let mut winding_pairs: Vec<Option<WindingPair>> = Vec::with_capacity(classified.len());
        let mut buckets: Vec<ClassifiedOutputBucket> = Vec::new();
        let mut bucket_fingerprints: StorageHashMap<ClassifiedOutputBucketFingerprint, Vec<usize>> =
            StorageHashMap::default();
        let mut plane_interner = PlaneProfileInterner::new();

        for classified_polygon in classified {
            let classification = classified_polygon.classification;
            let winding = classified_polygon.winding;
            let polygon = if classification == -1 {
                classified_polygon.polygon.inverted()
            } else {
                classified_polygon.polygon
            };
            let edge_profile = plane_interner.edge_profile(&polygon.edges);
            let bucket_fingerprint = ClassifiedOutputBucketFingerprint {
                classification,
                edge_count: polygon.edges.len(),
                support: plane_f64_fingerprint(&polygon.support),
                edge_profile: edge_profile_fingerprint(&edge_profile),
            };
            if let Some(existing_index) = find_matching_output_polygon_index(
                &buckets,
                bucket_fingerprints
                    .get(&bucket_fingerprint)
                    .map(Vec::as_slice),
                &output.polygons,
                classification,
                &edge_profile,
                &polygon,
            ) {
                if winding_pairs[existing_index].is_none() {
                    winding_pairs[existing_index] = winding;
                }
                continue;
            }
            let edge_count = polygon.edges.len();
            let support = polygon.support.clone();
            output.polygons.push(polygon);
            classifications.push(classification);
            winding_pairs.push(winding);
            let new_index = output.polygons.len() - 1;
            if let Some(bucket_index) =
                bucket_fingerprints
                    .get(&bucket_fingerprint)
                    .and_then(|candidate_buckets| {
                        candidate_buckets.iter().copied().find(|&bucket_index| {
                            let bucket = &buckets[bucket_index];
                            bucket.classification == classification
                                && bucket.edge_count == edge_count
                                && bucket.support == support
                                && bucket.edge_profile == edge_profile
                        })
                    })
            {
                buckets[bucket_index].indices.push(new_index);
            } else {
                let bucket_index = buckets.len();
                buckets.push(ClassifiedOutputBucket {
                    classification,
                    support,
                    edge_count,
                    edge_profile,
                    indices: vec![new_index],
                });
                bucket_fingerprints
                    .entry(bucket_fingerprint)
                    .or_default()
                    .push(bucket_index);
            }
        }

        Self {
            output,
            classifications,
            winding_pairs,
        }
    }

    /// Returns the output polygon soup.
    pub const fn output(&self) -> &PolygonSoup {
        &self.output
    }

    pub(crate) fn into_output(self) -> PolygonSoup {
        self.output
    }

    /// Returns per-output-polygon classifications.
    pub fn classifications(&self) -> &[i8] {
        &self.classifications
    }

    /// Returns per-output-polygon front/back winding evidence.
    pub fn winding_pairs(&self) -> &[Option<WindingPair>] {
        &self.winding_pairs
    }
}

fn polygons_match_output_geometry(left: &ConvexPolygon, right: &ConvexPolygon) -> bool {
    left.support == right.support && edge_cycles_match_up_to_rotation(&left.edges, &right.edges)
}

fn build_classified_polygon_buckets(
    classified: &[ClassifiedPolygon],
) -> Vec<ClassifiedPolygonBucket> {
    let mut buckets: Vec<ClassifiedPolygonBucket> = Vec::new();
    for (index, polygon) in classified.iter().enumerate() {
        let classification = polygon.classification;
        let edge_count = polygon.polygon.edges.len();
        let support = polygon.polygon.support.clone();
        if let Some(bucket) = buckets.iter_mut().find(|bucket| {
            bucket.classification == classification
                && bucket.edge_count == edge_count
                && bucket.support == support
        }) {
            bucket.indices.push(index);
        } else {
            buckets.push(ClassifiedPolygonBucket {
                classification,
                support,
                edge_count,
                indices: vec![index],
            });
        }
    }
    buckets
}

impl ClassifiedPolygonBucketState {
    pub(crate) fn new() -> Self {
        Self {
            buckets: Vec::new(),
        }
    }

    pub(crate) fn from_classified(classified: &[ClassifiedPolygon]) -> Self {
        Self {
            buckets: build_classified_polygon_buckets(classified),
        }
    }
}

pub(crate) fn merge_unique_classified_polygons_with_bucket_state(
    classified: &mut Vec<ClassifiedPolygon>,
    buckets: &mut ClassifiedPolygonBucketState,
    incoming: Vec<ClassifiedPolygon>,
) {
    for candidate in incoming {
        push_unique_classified_polygon_with_bucket_state(classified, buckets, candidate);
    }
}

pub(crate) fn push_unique_classified_polygon_with_bucket_state(
    classified: &mut Vec<ClassifiedPolygon>,
    buckets: &mut ClassifiedPolygonBucketState,
    candidate: ClassifiedPolygon,
) {
    push_unique_classified_polygon_with_buckets(classified, &mut buckets.buckets, candidate);
}

fn push_unique_classified_polygon_with_buckets(
    classified: &mut Vec<ClassifiedPolygon>,
    buckets: &mut Vec<ClassifiedPolygonBucket>,
    candidate: ClassifiedPolygon,
) {
    if let Some(existing_index) =
        find_matching_classified_polygon_index(buckets, classified, &candidate)
    {
        let existing = &mut classified[existing_index];
        if existing.winding.is_none() {
            existing.winding = candidate.winding;
        }
        existing.is_bsp_fragment |= candidate.is_bsp_fragment;
        return;
    }

    let classification = candidate.classification;
    let edge_count = candidate.polygon.edges.len();
    let support = candidate.polygon.support.clone();
    classified.push(candidate);
    let new_index = classified.len() - 1;
    if let Some(bucket) = buckets.iter_mut().find(|bucket| {
        bucket.classification == classification
            && bucket.edge_count == edge_count
            && bucket.support == support
    }) {
        bucket.indices.push(new_index);
    } else {
        buckets.push(ClassifiedPolygonBucket {
            classification,
            support,
            edge_count,
            indices: vec![new_index],
        });
    }
}

fn find_matching_classified_polygon_index(
    buckets: &[ClassifiedPolygonBucket],
    classified: &[ClassifiedPolygon],
    candidate: &ClassifiedPolygon,
) -> Option<usize> {
    let bucket = buckets.iter().find(|bucket| {
        bucket.classification == candidate.classification
            && bucket.edge_count == candidate.polygon.edges.len()
            && bucket.support == candidate.polygon.support
    })?;
    bucket.indices.iter().copied().find(|index| {
        polygons_match_output_geometry(&classified[*index].polygon, &candidate.polygon)
            && (candidate.classification != ARRANGEMENT_CLASSIFICATION
                || classified[*index].winding == candidate.winding)
    })
}

fn find_matching_output_polygon_index(
    buckets: &[ClassifiedOutputBucket],
    candidate_buckets: Option<&[usize]>,
    polygons: &[ConvexPolygon],
    classification: i8,
    edge_profile: &[usize],
    candidate: &ConvexPolygon,
) -> Option<usize> {
    candidate_buckets?.iter().copied().find_map(|bucket_index| {
        let bucket = &buckets[bucket_index];
        (bucket.classification == classification
            && bucket.edge_count == candidate.edges.len()
            && bucket.support == candidate.support
            && bucket.edge_profile == edge_profile)
            .then(|| {
                bucket
                    .indices
                    .iter()
                    .copied()
                    .find(|index| polygons_match_output_geometry(&polygons[*index], candidate))
            })
            .flatten()
    })
}

impl PlaneProfileInterner {
    fn new() -> Self {
        Self {
            planes: Vec::new(),
            approximate: StorageHashMap::default(),
        }
    }

    fn edge_profile(&mut self, edges: &[Plane]) -> Vec<usize> {
        let mut profile = edges
            .iter()
            .map(|edge| self.plane_id(edge))
            .collect::<Vec<_>>();
        profile.sort_unstable();
        profile
    }

    fn plane_id(&mut self, plane: &Plane) -> usize {
        if let Some(key) = plane_f64_fingerprint(plane) {
            if let Some(index) = self.approximate.get(&key).and_then(|candidates| {
                candidates
                    .iter()
                    .copied()
                    .find(|&index| self.planes[index] == *plane)
            }) {
                return index;
            }
            let index = self.planes.len();
            self.planes.push(plane.clone());
            self.approximate.entry(key).or_default().push(index);
            return index;
        }
        if let Some(index) = self.planes.iter().position(|existing| existing == plane) {
            return index;
        }
        let index = self.planes.len();
        self.planes.push(plane.clone());
        index
    }
}

fn plane_f64_fingerprint(plane: &Plane) -> Option<[u64; 4]> {
    let coordinates = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ];
    if coordinates
        .iter()
        .any(|coordinate| coordinate.exact_rational_ref().is_none())
    {
        return None;
    }
    let [Some(a), Some(b), Some(c), Some(d)] = coordinates.map(Real::to_f64_lossy) else {
        return None;
    };
    [a, b, c, d].into_iter().all(f64::is_finite).then(|| {
        [a, b, c, d].map(|value| {
            if value == 0.0 {
                0.0_f64.to_bits()
            } else {
                value.to_bits()
            }
        })
    })
}

fn edge_profile_fingerprint(profile: &[usize]) -> u64 {
    profile
        .iter()
        .fold(profile.len() as u64, |fingerprint, &id| {
            fingerprint
                .rotate_left(17)
                .wrapping_mul(0x9e37_79b9_7f4a_7c15)
                .wrapping_add(id as u64)
        })
}

fn edge_cycles_match_up_to_rotation(
    left: &[crate::geometry::Plane],
    right: &[crate::geometry::Plane],
) -> bool {
    if left.len() != right.len() {
        return false;
    }
    if left.is_empty() {
        return true;
    }

    for offset in 0..left.len() {
        let mut all_match = true;
        for index in 0..left.len() {
            if left[index] != right[(index + offset) % right.len()] {
                all_match = false;
                break;
            }
        }
        if all_match {
            return true;
        }
    }

    false
}

/// Extracted output polygon with explicit vertices.
#[derive(Clone, Debug, PartialEq)]
pub struct OutputPolygon {
    /// Vertices in polygon winding order.
    pub vertices: Vec<OutputVertex>,
    /// Source mesh index.
    pub source_mesh: isize,
    /// Source polygon index.
    pub source_polygon: isize,
}

/// Input triangle that contributed an output triangle.
#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct TriangleSource {
    /// Source mesh index.
    pub mesh: isize,
    /// Global source triangle index across the ordered input mesh streams.
    ///
    /// `-1` denotes a triangle synthesized from certified analytic source
    /// geometry, such as an exact axis-aligned-box cell boundary.
    pub triangle: isize,
    /// `+1` when output orientation matches the source and `-1` when inverted.
    ///
    /// Zero is reserved for callers constructing source records without
    /// orientation provenance.
    pub orientation: i8,
}

/// Boolean output geometry with source provenance.
///
/// Call [`Self::into_triangle_mesh`] when provenance has been consumed or is
/// not needed. Keeping the source rows here leaves [`TriangleMesh`] as the
/// canonical reusable geometry carrier.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BooleanMesh {
    /// Output vertices.
    pub vertices: Vec<OutputVertex>,
    /// Triangle vertex indices.
    pub triangles: Vec<[usize; 3]>,
    /// Source polygon for each triangle, parallel to `triangles`.
    pub sources: Vec<TriangleSource>,
}

impl BooleanMesh {
    /// Materializes the reusable native triangle geometry while retaining this
    /// Boolean result and its provenance.
    pub fn to_triangle_mesh(&self) -> TriangleMesh {
        TriangleMesh::new(
            self.vertices
                .iter()
                .map(|vertex| {
                    hyperlattice::Point3::new(vertex.x.clone(), vertex.y.clone(), vertex.z.clone())
                })
                .collect(),
            self.triangles
                .iter()
                .map(|triangle| Triangle::new(triangle[0], triangle[1], triangle[2]))
                .collect(),
        )
    }

    /// Consumes this Boolean result and returns its reusable native triangle
    /// geometry, discarding source provenance explicitly.
    pub fn into_triangle_mesh(self) -> TriangleMesh {
        TriangleMesh::new(
            self.vertices
                .into_iter()
                .map(|vertex| hyperlattice::Point3::new(vertex.x, vertex.y, vertex.z))
                .collect(),
            self.triangles
                .into_iter()
                .map(|triangle| Triangle::new(triangle[0], triangle[1], triangle[2]))
                .collect(),
        )
    }

    /// Borrows the per-triangle provenance rows.
    pub fn sources(&self) -> &[TriangleSource] {
        &self.sources
    }

    /// Returns true when materialization contains no invalid, degenerate, or
    /// exact duplicate triangle geometry.
    ///
    /// Independently indexed exact duplicate position rows are canonicalized
    /// before triangle keys are compared.
    pub fn has_unique_nondegenerate_triangles(
        &self,
        context: &MeshContext,
    ) -> HypermeshResult<MeshOutcome<bool>> {
        let decisions = DecisionContext::new(context);
        let valid = self.has_unique_nondegenerate_triangles_decision(&decisions)?;
        Ok(decisions.finish(valid))
    }

    pub(crate) fn has_unique_nondegenerate_triangles_decision(
        &self,
        decisions: &DecisionContext,
    ) -> HypermeshResult<bool> {
        if self.sources.len() != self.triangles.len() {
            return Ok(false);
        }
        if self
            .triangles
            .iter()
            .flatten()
            .any(|index| *index >= self.vertices.len())
        {
            return Ok(false);
        }
        let canonical = merge_duplicate_vertices(decisions, self)?;
        let mut seen = BTreeSet::new();
        for triangle in &canonical.triangles {
            let [Some(a), Some(b), Some(c)] = triangle.map(|index| canonical.vertices.get(index))
            else {
                return Ok(false);
            };
            if triangle[0] == triangle[1]
                || triangle[1] == triangle[2]
                || triangle[0] == triangle[2]
                || !crate::geometry::Plane::decide_points_are_nondegenerate(
                    decisions,
                    &Point3::new(a.x.clone(), a.y.clone(), a.z.clone()),
                    &Point3::new(b.x.clone(), b.y.clone(), b.z.clone()),
                    &Point3::new(c.x.clone(), c.y.clone(), c.z.clone()),
                )?
            {
                return Ok(false);
            }
            let mut key = *triangle;
            key.sort_unstable();
            if !seen.insert(key) {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

/// Exact closure summary for Boolean output geometry.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BooleanMeshClosureEvidence {
    /// Number of undirected edges used by exactly one triangle.
    pub boundary_edges: usize,
    /// Number of geometric edge classes whose forward and reverse uses do not
    /// cancel.
    pub unbalanced_edges: usize,
    /// Number of undirected edges used by more than two triangles.
    pub non_manifold_edges: usize,
}

impl BooleanMeshClosureEvidence {
    /// Returns true when there are no singleton edges and every directed edge
    /// use cancels. Balanced non-manifold edge valence is allowed for closed
    /// PWN outputs.
    pub const fn has_no_boundary(self) -> bool {
        self.boundary_edges == 0 && self.unbalanced_edges == 0
    }

    /// Returns true when every undirected edge has exactly two oppositely
    /// directed uses.
    pub const fn is_closed(self) -> bool {
        self.boundary_edges == 0 && self.unbalanced_edges == 0 && self.non_manifold_edges == 0
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct DirectedEdgeUses {
    forward: usize,
    reverse: usize,
}

impl DirectedEdgeUses {
    const fn total(self) -> usize {
        self.forward + self.reverse
    }

    const fn is_balanced(self) -> bool {
        self.forward == self.reverse
    }
}

/// Extracts output polygons from a boolean result.
pub fn extract_output(
    context: &MeshContext,
    result: &BooleanResult,
) -> HypermeshResult<MeshOutcome<Vec<OutputPolygon>>> {
    extract_output_polygons(context, &result.output.polygons)
}

/// Extracts output polygons from a borrowed polygon slice.
pub fn extract_output_polygons(
    context: &MeshContext,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<MeshOutcome<Vec<OutputPolygon>>> {
    let decisions = DecisionContext::new(context);
    let output = extract_output_polygons_decision(&decisions, polygons)?;
    Ok(decisions.finish(output))
}

fn extract_output_polygons_decision(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<Vec<OutputPolygon>> {
    let mut out = Vec::with_capacity(polygons.len());
    for polygon in polygons {
        let mut vertices = Vec::with_capacity(polygon.vertex_count());
        append_polygon_output_vertices(decisions, &mut vertices, polygon)?;
        out.push(OutputPolygon {
            vertices,
            source_mesh: polygon.mesh_index,
            source_polygon: polygon.polygon_index,
        });
    }
    Ok(out)
}

fn append_polygon_output_vertices(
    decisions: &DecisionContext,
    vertices: &mut Vec<OutputVertex>,
    polygon: &ConvexPolygon,
) -> HypermeshResult<()> {
    if let Some(points) = polygon.known_vertices.as_ref() {
        vertices.extend(points.iter().map(|point| OutputVertex {
            x: point.x.clone(),
            y: point.y.clone(),
            z: point.z.clone(),
        }));
    } else {
        vertices.extend(
            polygon
                .vertices_decision(decisions)?
                .into_iter()
                .map(|point| OutputVertex {
                    x: point.x,
                    y: point.y,
                    z: point.z,
                }),
        );
    }
    Ok(())
}

fn triangulate_output(
    decisions: &DecisionContext,
    result: &BooleanResult,
) -> HypermeshResult<BooleanMesh> {
    triangulate_polygons(
        decisions,
        &result.output.polygons,
        Some(&result.classifications),
    )
}

/// Fan-triangulates and resolves exact duplicate/T-junction artifacts.
///
/// This is useful for tests and callers that need evidence that the classified
/// arrangement is already a closed regularized PWN surface. Non-manifold edge
/// valence is allowed, but non-empty open, reversed, or zero-volume soups are
/// reported as uncertified.
pub fn triangulate_and_resolve_certified(
    context: &MeshContext,
    result: &BooleanResult,
) -> HypermeshResult<MeshOutcome<BooleanMesh>> {
    let decisions = DecisionContext::new(context);
    certify_output_polygon_closure_decision(&decisions, result)?;
    let soup = triangulate_and_resolve_polygon_certified(&decisions, result)?;
    Ok(decisions.finish(soup))
}

pub(crate) fn triangulate_and_resolve_polygon_certified(
    decisions: &DecisionContext,
    result: &BooleanResult,
) -> HypermeshResult<BooleanMesh> {
    let mut soup = match triangulate_closed_polygon_arrangement(
        decisions,
        &result.output.polygons,
        &result.classifications,
        None,
        false,
        false,
        false,
    ) {
        Ok((soup, _)) => soup,
        Err(HypermeshError::PredicateUndecided { .. } | HypermeshError::UnknownClassification) => {
            resolve_tjunctions(decisions, &triangulate_output(decisions, result)?)?
        }
        Err(err) => return Err(err),
    };
    if soup.triangles.is_empty() {
        return Ok(soup);
    }
    let mut closure = boolean_mesh_closure_evidence(&soup);
    if !closure.has_no_boundary() {
        soup = resolve_tjunctions(decisions, &triangulate_output(decisions, result)?)?;
        closure = boolean_mesh_closure_evidence(&soup);
    }
    if !closure.has_no_boundary() {
        return Err(HypermeshError::OpenOutput {
            boundary_edges: closure.boundary_edges,
            unbalanced_edges: closure.unbalanced_edges,
            non_manifold_edges: closure.non_manifold_edges,
        });
    }
    if !boolean_result_has_complete_orientation_evidence(result) {
        certify_positive_signed_volume(decisions, &soup)?;
    }
    Ok(soup)
}

fn boolean_result_has_complete_orientation_evidence(result: &BooleanResult) -> bool {
    result.output.polygons.len() == result.classifications.len()
        && result.output.polygons.len() == result.winding_pairs.len()
        && result
            .classifications
            .iter()
            .all(|classification| matches!(classification, -1 | 1))
        && result.winding_pairs.iter().all(Option::is_some)
}

fn triangulate_closed_polygon_arrangement<P>(
    decisions: &DecisionContext,
    polygons: &[P],
    orientations: &[i8],
    polygon_windings: Option<&[WindingPair]>,
    prefer_precomputed_f64_scan: bool,
    prefer_construction_candidates: bool,
    filter_recovery_candidates: bool,
) -> HypermeshResult<(BooleanMesh, Vec<WindingPair>)>
where
    P: Borrow<ConvexPolygon>,
{
    if polygons.len() != orientations.len() {
        return Err(HypermeshError::UnknownClassification);
    }
    if polygon_windings.is_some_and(|windings| windings.len() != polygons.len()) {
        return Err(HypermeshError::UnknownClassification);
    }
    let (mut vertices, indexed_polygons) =
        merge_duplicate_convex_polygon_vertices(decisions, polygons)?;
    let rational_vertex_queries = filter_recovery_candidates.then(|| {
        vertices
            .iter()
            .map(|vertex| {
                let [Some(x), Some(y), Some(z)] = [
                    vertex.x.exact_rational_ref(),
                    vertex.y.exact_rational_ref(),
                    vertex.z.exact_rational_ref(),
                ] else {
                    return None;
                };
                RationalPoint3Query::from_rationals([x, y, z])
            })
            .collect::<Vec<_>>()
    });
    let construction_candidates = prefer_construction_candidates
        .then(|| build_construction_edge_candidates(polygons, &indexed_polygons, vertices.len()))
        .transpose()?;
    let approximate_vertices = prefer_precomputed_f64_scan
        .then(|| exact_output_vertex_enclosures(&vertices))
        .flatten();
    let axis_order = (approximate_vertices.is_none() && construction_candidates.is_none())
        .then(|| sorted_vertex_indices_by_axis(decisions, &vertices))
        .transpose()?;
    let triangle_capacity = indexed_polygons
        .iter()
        .zip(orientations)
        .filter(|(_, orientation)| **orientation != 0)
        .map(|(polygon, _)| polygon.len().saturating_sub(2))
        .sum();
    let mut split_edge_cache = SplitEdgeCache::default();
    let mut triangles = Vec::with_capacity(triangle_capacity);
    let mut sources = Vec::with_capacity(triangle_capacity);
    let mut triangle_windings = Vec::with_capacity(if polygon_windings.is_some() {
        triangle_capacity
    } else {
        0
    });
    let mut boundary = Vec::new();

    for (polygon_index, ((polygon, indexed), orientation)) in polygons
        .iter()
        .zip(indexed_polygons)
        .zip(orientations.iter().copied())
        .enumerate()
    {
        let polygon = polygon.borrow();
        if orientation == 0 {
            continue;
        }
        if indexed.len() < 3 {
            continue;
        }
        boundary.clear();
        boundary.reserve(indexed.len());
        for edge_index in 0..indexed.len() {
            let start = indexed[edge_index];
            let end = indexed[(edge_index + 1) % indexed.len()];
            if start == end {
                continue;
            }
            let canonical = sorted_edge([start, end]);
            let chain = if let Some(candidates) = &construction_candidates {
                split_segment_subedges_exact_candidates(
                    decisions,
                    &mut split_edge_cache,
                    &vertices,
                    canonical,
                    &candidates.groups[candidates.polygon_edges[polygon_index][edge_index]],
                    &candidates.recovery_vertices,
                    rational_vertex_queries.as_deref(),
                    filter_recovery_candidates,
                )
                .inspect_err(|error| {
                    if cfg!(debug_assertions) {
                        eprintln!(
                            "[DEBUG] construction edge split failed: polygon={polygon_index} edge={edge_index}: {error}"
                        );
                    }
                })?
            } else if let Some(approximate_vertices) = &approximate_vertices {
                split_segment_subedges_exact(
                    decisions,
                    &mut split_edge_cache,
                    &vertices,
                    SplitEdgeSearch::Approximate(approximate_vertices),
                    canonical,
                )?
            } else {
                split_segment_subedges_exact(
                    decisions,
                    &mut split_edge_cache,
                    &vertices,
                    SplitEdgeSearch::AxisOrder(
                        axis_order
                            .as_ref()
                            .expect("axis order exists without an approximate scan"),
                    ),
                    canonical,
                )?
            };
            if start == canonical[0] {
                boundary.extend(chain.subedges().map(|edge| edge[0]));
            } else {
                boundary.extend(chain.subedges().rev().map(|edge| edge[1]));
            }
        }
        boundary.dedup();
        if boundary.len() > 1 && boundary.first() == boundary.last() {
            boundary.pop();
        }
        if boundary.len() < 3 {
            continue;
        }
        let triangle_start = triangles.len();
        if boundary.len() > indexed.len() {
            if append_split_boundary_fan_from_unsplit_corner(
                decisions,
                polygon,
                &indexed,
                &boundary,
                &vertices,
                &mut triangles,
            )? {
                crate::trace_dispatch!("output-triangulation", "split-boundary-corner-fan");
            } else {
                crate::trace_dispatch!("output-triangulation", "split-boundary-centroid");
                let center = append_output_polygon_centroid(&mut vertices, &indexed)?;
                for index in 0..boundary.len() {
                    triangles.push([
                        center,
                        boundary[index],
                        boundary[(index + 1) % boundary.len()],
                    ]);
                }
            }
        } else {
            let appended_construction = if construction_candidates.is_some() {
                append_exact_corner_boundary_triangles(
                    decisions,
                    polygon,
                    &indexed,
                    &boundary,
                    &vertices,
                    &mut triangles,
                )?
                .is_some()
            } else {
                false
            };
            if !appended_construction {
                match triangulate_weakly_convex_boundary(
                    decisions,
                    &boundary,
                    &vertices,
                    &polygon.support,
                ) {
                    Ok(polygon_triangles) => triangles.extend(polygon_triangles),
                    Err(
                        HypermeshError::PredicateUndecided { .. }
                        | HypermeshError::UnknownClassification,
                    ) => {
                        let center = append_output_polygon_centroid(&mut vertices, &boundary)?;
                        for index in 0..boundary.len() {
                            triangles.push([
                                center,
                                boundary[index],
                                boundary[(index + 1) % boundary.len()],
                            ]);
                        }
                    }
                    Err(err) => return Err(err),
                }
            }
        }
        let triangle_count = triangles.len() - triangle_start;
        for _ in 0..triangle_count {
            sources.push(TriangleSource {
                mesh: polygon.mesh_index,
                triangle: polygon.polygon_index,
                orientation,
            });
            if let Some(windings) = polygon_windings {
                triangle_windings.push(windings[polygon_index].clone());
            }
        }
    }

    let mut soup = BooleanMesh {
        vertices,
        triangles,
        sources,
    };
    remove_unused_vertices(&mut soup);
    Ok((soup, triangle_windings))
}

fn remove_unused_vertices(soup: &mut BooleanMesh) {
    let mut used = vec![false; soup.vertices.len()];
    for triangle in &soup.triangles {
        for &vertex in triangle {
            used[vertex] = true;
        }
    }
    if used.iter().all(|is_used| *is_used) {
        return;
    }
    let mut remap = vec![0; soup.vertices.len()];
    let mut next = 0;
    for (index, &is_used) in used.iter().enumerate() {
        if is_used {
            remap[index] = next;
            next += 1;
        }
    }
    for triangle in &mut soup.triangles {
        *triangle = triangle.map(|vertex| remap[vertex]);
    }
    let mut index = 0;
    soup.vertices.retain(|_| {
        let retain = used[index];
        index += 1;
        retain
    });
}

pub(crate) fn triangulate_classified_arrangement_precomputed_f64_scan(
    decisions: &DecisionContext,
    classified: &[ClassifiedPolygon],
) -> HypermeshResult<ClassifiedTriangleArrangement> {
    triangulate_classified_arrangement_with_strategy(decisions, classified, true, false, false)
}

pub(crate) fn triangulate_classified_arrangement_construction_candidates(
    decisions: &DecisionContext,
    classified: &[ClassifiedPolygon],
    filter_recovery_candidates: bool,
) -> HypermeshResult<ClassifiedTriangleArrangement> {
    triangulate_classified_arrangement_with_strategy(
        decisions,
        classified,
        false,
        true,
        filter_recovery_candidates,
    )
}

pub(crate) fn triangulate_preclassified_arrangement_construction_candidates(
    decisions: &DecisionContext,
    classified: &[ClassifiedPolygon],
    filter_recovery_candidates: bool,
) -> HypermeshResult<BooleanMesh> {
    triangulate_preclassified_arrangement_with_strategy(
        decisions,
        classified,
        false,
        true,
        filter_recovery_candidates,
        false,
    )
}

pub(crate) fn triangulate_preclassified_arrangement_precomputed_f64_scan(
    decisions: &DecisionContext,
    classified: &[ClassifiedPolygon],
) -> HypermeshResult<BooleanMesh> {
    triangulate_preclassified_arrangement_with_strategy(
        decisions, classified, true, false, false, false,
    )
}

pub(crate) fn triangulate_selected_preclassified_arrangement_construction_candidates(
    decisions: &DecisionContext,
    classified: &[ClassifiedPolygon],
    filter_recovery_candidates: bool,
) -> HypermeshResult<BooleanMesh> {
    triangulate_preclassified_arrangement_with_strategy(
        decisions,
        classified,
        false,
        true,
        filter_recovery_candidates,
        true,
    )
}

fn triangulate_preclassified_arrangement_with_strategy(
    decisions: &DecisionContext,
    classified: &[ClassifiedPolygon],
    prefer_precomputed_f64_scan: bool,
    prefer_construction_candidates: bool,
    filter_recovery_candidates: bool,
    retain_unselected_recovery: bool,
) -> HypermeshResult<BooleanMesh> {
    let polygons = classified
        .iter()
        .map(|classified| &classified.polygon)
        .collect::<Vec<_>>();
    let orientations = classified
        .iter()
        .map(|classified| {
            let orientation = classified.classification;
            if matches!(orientation, -1 | 1) || (retain_unselected_recovery && orientation == 0) {
                Ok(orientation)
            } else {
                Err(HypermeshError::UnknownClassification)
            }
        })
        .collect::<HypermeshResult<Vec<_>>>()?;
    let (mut soup, _) = triangulate_closed_polygon_arrangement(
        decisions,
        &polygons,
        &orientations,
        None,
        prefer_precomputed_f64_scan,
        prefer_construction_candidates,
        filter_recovery_candidates,
    )?;
    for (triangle, source) in soup.triangles.iter_mut().zip(&soup.sources) {
        if source.orientation == -1 {
            triangle.swap(1, 2);
        }
    }
    Ok(soup)
}

fn triangulate_classified_arrangement_with_strategy(
    decisions: &DecisionContext,
    classified: &[ClassifiedPolygon],
    prefer_precomputed_f64_scan: bool,
    prefer_construction_candidates: bool,
    filter_recovery_candidates: bool,
) -> HypermeshResult<ClassifiedTriangleArrangement> {
    let polygons = classified
        .iter()
        .map(|classified| &classified.polygon)
        .collect::<Vec<_>>();
    let windings = classified
        .iter()
        .map(|classified| {
            classified
                .winding
                .clone()
                .ok_or(HypermeshError::UnknownClassification)
        })
        .collect::<HypermeshResult<Vec<_>>>()?;
    let orientations = vec![1; polygons.len()];
    let (soup, triangle_windings) = triangulate_closed_polygon_arrangement(
        decisions,
        &polygons,
        &orientations,
        Some(&windings),
        prefer_precomputed_f64_scan,
        prefer_construction_candidates,
        filter_recovery_candidates,
    )?;
    Ok(ClassifiedTriangleArrangement {
        soup,
        windings: triangle_windings,
    })
}

fn append_split_boundary_fan_from_unsplit_corner(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    indexed: &[usize],
    boundary: &[usize],
    vertices: &[OutputVertex],
    triangles: &mut Vec<[usize; 3]>,
) -> HypermeshResult<bool> {
    let Some((anchor_position, &anchor)) =
        boundary.iter().enumerate().find(|(position, vertex)| {
            indexed.contains(vertex)
                && indexed.contains(&boundary[(position + boundary.len() - 1) % boundary.len()])
                && indexed.contains(&boundary[(position + 1) % boundary.len()])
        })
    else {
        return Ok(false);
    };

    let mut fan = Vec::with_capacity(boundary.len().saturating_sub(2));
    for offset in 1..(boundary.len() - 1) {
        let triangle = [
            anchor,
            boundary[(anchor_position + offset) % boundary.len()],
            boundary[(anchor_position + offset + 1) % boundary.len()],
        ];
        if !output_triangle_is_nondegenerate(decisions, triangle, vertices, &polygon.support)? {
            return Ok(false);
        }
        fan.push(triangle);
    }
    triangles.extend(fan);
    Ok(true)
}

fn append_output_polygon_centroid(
    vertices: &mut Vec<OutputVertex>,
    boundary: &[usize],
) -> HypermeshResult<usize> {
    let center = OutputVertex {
        x: mean_output_coordinate(vertices, boundary, |vertex| &vertex.x)?,
        y: mean_output_coordinate(vertices, boundary, |vertex| &vertex.y)?,
        z: mean_output_coordinate(vertices, boundary, |vertex| &vertex.z)?,
    };
    let index = vertices.len();
    vertices.push(center);
    Ok(index)
}

fn mean_output_coordinate<'a>(
    vertices: &'a [OutputVertex],
    boundary: &[usize],
    coordinate: impl Fn(&'a OutputVertex) -> &'a Real,
) -> HypermeshResult<Real> {
    if boundary.is_empty() {
        return Err(HypermeshError::UnknownClassification);
    }
    if let &[first, second, third] = boundary
        && let (Some(first), Some(second), Some(third)) = (
            coordinate(&vertices[first]).exact_rational_ref(),
            coordinate(&vertices[second]).exact_rational_ref(),
            coordinate(&vertices[third]).exact_rational_ref(),
        )
    {
        return Ok(Real::from(Rational::mean3_refs([first, second, third])));
    }
    if let Some(rationals) = boundary
        .iter()
        .map(|&index| coordinate(&vertices[index]).exact_rational_ref())
        .collect::<Option<Vec<_>>>()
    {
        return Rational::mean_refs(&rationals)
            .map(Real::from)
            .ok_or(HypermeshError::UnknownClassification);
    }
    let count = Real::from(
        u64::try_from(boundary.len()).map_err(|_| HypermeshError::UnknownClassification)?,
    );
    (Real::sum_refs(boundary.iter().map(|&index| coordinate(&vertices[index]))) / count)
        .map_err(|_| HypermeshError::UnknownClassification)
}

fn append_exact_corner_boundary_triangles(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    indexed: &[usize],
    boundary: &[usize],
    vertices: &[OutputVertex],
    triangles: &mut Vec<[usize; 3]>,
) -> HypermeshResult<Option<()>> {
    if indexed != boundary {
        return Ok(None);
    }
    if let &[a, b, c] = boundary {
        // Construction candidates exist only for retained, certified vertex
        // cycles. An unchanged three-vertex boundary is therefore the
        // certified source polygon itself; merging or splitting would have
        // made `indexed` and `boundary` differ above.
        triangles.push([a, b, c]);
        return Ok(Some(()));
    }
    let mut fan = Vec::with_capacity(boundary.len().saturating_sub(2));
    // A corner fan is complete only when every boundary edge belongs to one
    // nondegenerate wedge. Silently omitting a collinear wedge would also
    // omit its boundary edge and turn an otherwise closed surface into an
    // open one. Let the caller choose a different triangulation in that case.
    for index in 1..boundary.len() - 1 {
        let triangle = [boundary[0], boundary[index], boundary[index + 1]];
        if !output_triangle_is_nondegenerate(decisions, triangle, vertices, &polygon.support)? {
            return Ok(None);
        }
        fan.push(triangle);
    }
    triangles.extend(fan);
    Ok(Some(()))
}

fn triangulate_weakly_convex_boundary(
    decisions: &DecisionContext,
    boundary: &[usize],
    vertices: &[OutputVertex],
    support: &Plane,
) -> HypermeshResult<Vec<[usize; 3]>> {
    let mut remaining = boundary.to_vec();
    let mut triangles = Vec::with_capacity(remaining.len().saturating_sub(2));
    while remaining.len() > 3 {
        let mut ear = None;
        for index in 0..remaining.len() {
            let triangle = [
                remaining[(index + remaining.len() - 1) % remaining.len()],
                remaining[index],
                remaining[(index + 1) % remaining.len()],
            ];
            if output_triangle_is_nondegenerate(decisions, triangle, vertices, support)? {
                ear = Some((index, triangle));
                break;
            }
        }
        let Some((index, triangle)) = ear else {
            return Err(HypermeshError::UnknownClassification);
        };
        triangles.push(triangle);
        remaining.remove(index);
    }
    let triangle = [remaining[0], remaining[1], remaining[2]];
    if !output_triangle_is_nondegenerate(decisions, triangle, vertices, support)? {
        return Err(HypermeshError::UnknownClassification);
    }
    triangles.push(triangle);
    Ok(triangles)
}

fn output_triangle_is_nondegenerate(
    decisions: &DecisionContext,
    triangle: [usize; 3],
    vertices: &[OutputVertex],
    support: &Plane,
) -> HypermeshResult<bool> {
    // Boundary vertices lie on `support`. Their edge cross product is
    // therefore parallel to its normal, so an exactly nonzero normal
    // component certifies that the complementary 2D projection is degenerate
    // exactly when the original triangle is degenerate.
    let normal = [&support.normal.x, &support.normal.y, &support.normal.z];
    if let Some(projection_axis) = normal.iter().position(|component| {
        component
            .exact_rational_ref()
            .is_some_and(|value| !value.is_zero())
    }) {
        let (u_axis, v_axis) = match projection_axis {
            0 => (1, 2),
            1 => (0, 2),
            2 => (0, 1),
            _ => unreachable!("projection axis is in 0..3"),
        };
        let origin = &vertices[triangle[0]];
        let left = &vertices[triangle[1]];
        let right = &vertices[triangle[2]];
        let left_u = vertex_axis(left, u_axis) - vertex_axis(origin, u_axis);
        let left_v = vertex_axis(left, v_axis) - vertex_axis(origin, v_axis);
        let right_u = vertex_axis(right, u_axis) - vertex_axis(origin, u_axis);
        let right_v = vertex_axis(right, v_axis) - vertex_axis(origin, v_axis);
        if let ([Some(left_u), Some(left_v)], [Some(right_u), Some(right_v)]) = (
            [&left_u, &left_v].map(Real::exact_rational_ref),
            [&right_u, &right_v].map(Real::exact_rational_ref),
        ) {
            return Ok(!Rational::signed_product_sum_ordering(
                [true, false],
                [[left_u, right_v], [left_v, right_u]],
            )
            .is_eq());
        }
        let projected_area =
            Real::signed_product_sum([true, false], [[&left_u, &right_v], [&left_v, &right_u]]);
        return Ok(classify_real(decisions, &projected_area)? != Classification::On);
    }

    let left = sub_vertex(&vertices[triangle[1]], &vertices[triangle[0]]);
    let right = sub_vertex(&vertices[triangle[2]], &vertices[triangle[0]]);
    if let Some(sign) = Real::exact_rational_det3_word_sign(
        [&left[0], &left[1], &left[2]],
        [&right[0], &right[1], &right[2]],
        [&support.normal.x, &support.normal.y, &support.normal.z],
    ) {
        return Ok(sign != RealSign::Zero);
    }
    let cross = cross_arrays(&left, &right);
    let oriented_area = Real::signed_product_sum(
        [true, true, true],
        [
            [&cross[0], &support.normal.x],
            [&cross[1], &support.normal.y],
            [&cross[2], &support.normal.z],
        ],
    );
    Ok(classify_real(decisions, &oriented_area)? != Classification::On)
}

/// Certifies that the classified polygon arrangement is already closed before
/// triangulation cleanup runs.
///
/// Balanced non-manifold edge valence is allowed, but any singleton edge or
/// directed edge imbalance is reported as [`HypermeshError::OpenOutput`]
/// instead of being left for triangle cleanup to repair.
pub fn certify_output_polygon_closure(
    context: &MeshContext,
    result: &BooleanResult,
) -> HypermeshResult<MeshOutcome<BooleanMeshClosureEvidence>> {
    let decisions = DecisionContext::new(context);
    let evidence = certify_output_polygon_closure_decision(&decisions, result)?;
    Ok(decisions.finish(evidence))
}

pub(crate) fn certify_output_polygon_closure_decision(
    decisions: &DecisionContext,
    result: &BooleanResult,
) -> HypermeshResult<BooleanMeshClosureEvidence> {
    let polygon_closure =
        output_polygon_closure_evidence_from_convex_polygons(decisions, &result.output.polygons)?;
    if !polygon_closure.has_no_boundary() {
        return Err(HypermeshError::OpenOutput {
            boundary_edges: polygon_closure.boundary_edges,
            unbalanced_edges: polygon_closure.unbalanced_edges,
            non_manifold_edges: polygon_closure.non_manifold_edges,
        });
    }
    Ok(polygon_closure)
}

#[cfg(test)]
fn output_polygon_closure_evidence(
    decisions: &DecisionContext,
    polygons: &[OutputPolygon],
) -> HypermeshResult<BooleanMeshClosureEvidence> {
    let (vertices, indexed_polygons) = merge_duplicate_polygon_vertices(decisions, polygons)?;
    output_polygon_closure_evidence_from_indexed_vertices(decisions, &vertices, &indexed_polygons)
}

fn output_polygon_closure_evidence_from_convex_polygons(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<BooleanMeshClosureEvidence> {
    let (vertices, indexed_polygons) =
        merge_duplicate_convex_polygon_vertices(decisions, polygons)?;
    output_polygon_closure_evidence_from_indexed_vertices(decisions, &vertices, &indexed_polygons)
}

fn output_polygon_closure_evidence_from_indexed_vertices(
    decisions: &DecisionContext,
    vertices: &[OutputVertex],
    indexed_polygons: &[Vec<usize>],
) -> HypermeshResult<BooleanMeshClosureEvidence> {
    let axis_order = sorted_vertex_indices_by_axis(decisions, vertices)?;
    let edge_counts = polygon_edge_counts(decisions, vertices, indexed_polygons, &axis_order)?;
    let mut evidence = BooleanMeshClosureEvidence::default();
    for uses in edge_counts.values().copied() {
        if uses.total() == 1 {
            evidence.boundary_edges += 1;
        } else if uses.total() > 2 {
            evidence.non_manifold_edges += 1;
        }
        if !uses.is_balanced() {
            evidence.unbalanced_edges += 1;
        }
    }
    Ok(evidence)
}

#[cfg(test)]
fn merge_duplicate_polygon_vertices(
    decisions: &DecisionContext,
    polygons: &[OutputPolygon],
) -> HypermeshResult<(Vec<OutputVertex>, Vec<Vec<usize>>)> {
    let mut positions = Vec::new();
    let mut indexed_polygons: Vec<Vec<usize>> = polygons
        .iter()
        .map(|polygon| vec![0; polygon.vertices.len()])
        .collect();

    for (polygon_index, polygon) in polygons.iter().enumerate() {
        for vertex_index in 0..polygon.vertices.len() {
            positions.push((polygon_index, vertex_index, positions.len()));
        }
    }

    positions.sort_by(
        |(left_polygon, left_vertex, _), (right_polygon, right_vertex, _)| {
            compare_output_vertices_lexicographic(
                decisions,
                &polygons[*left_polygon].vertices[*left_vertex],
                &polygons[*right_polygon].vertices[*right_vertex],
            )
            .expect("exact output vertex ordering should compare")
        },
    );

    let mut groups: Vec<(usize, OutputVertex, Vec<(usize, usize)>)> = Vec::new();
    for (polygon_index, vertex_index, flat_index) in positions {
        let vertex = &polygons[polygon_index].vertices[vertex_index];
        match groups.last_mut() {
            Some((first_flat_index, existing, members)) if *existing == *vertex => {
                *first_flat_index = (*first_flat_index).min(flat_index);
                members.push((polygon_index, vertex_index));
            }
            _ => groups.push((
                flat_index,
                vertex.clone(),
                vec![(polygon_index, vertex_index)],
            )),
        }
    }
    groups.sort_by_key(|(first_flat_index, _, _)| *first_flat_index);

    let mut vertices = Vec::with_capacity(groups.len());
    for (_, vertex, members) in groups {
        let merged_index = vertices.len();
        vertices.push(vertex);
        for (polygon_index, vertex_index) in members {
            indexed_polygons[polygon_index][vertex_index] = merged_index;
        }
    }

    Ok((vertices, indexed_polygons))
}

fn merge_duplicate_convex_polygon_vertices<P>(
    decisions: &DecisionContext,
    polygons: &[P],
) -> HypermeshResult<(Vec<OutputVertex>, Vec<Vec<usize>>)>
where
    P: Borrow<ConvexPolygon>,
{
    // Retained rational cycles can be canonicalized while borrowed, so only
    // vertices admitted to the merged output clone their exact coordinates.
    if polygons.iter().all(|polygon| {
        let polygon = polygon.borrow();
        polygon.known_vertices.as_ref().is_some_and(|points| {
            points.len() == polygon.vertex_count()
                && points.iter().all(|point| {
                    point.x.exact_rational_ref().is_some()
                        && point.y.exact_rational_ref().is_some()
                        && point.z.exact_rational_ref().is_some()
                })
        })
    }) {
        let position_count = polygons
            .iter()
            .map(|polygon| polygon.borrow().vertex_count())
            .sum();
        let mut interner = PointInterner::<()>::try_with_capacity(position_count, true, false)?;
        let mut vertices = Vec::new();
        vertices.try_reserve_exact(position_count).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "convex output vertices",
            }
        })?;
        let mut indexed_polygons = Vec::with_capacity(polygons.len());
        for polygon in polygons {
            let polygon = polygon.borrow();
            let points = polygon
                .known_vertices
                .as_ref()
                .expect("the retained exact path was validated above");
            let mut indexed = Vec::with_capacity(points.len());
            for point in points.iter() {
                indexed.push(interner.intern_with(
                    decisions,
                    &mut vertices,
                    point,
                    None,
                    || OutputVertex {
                        x: point.x.clone(),
                        y: point.y.clone(),
                        z: point.z.clone(),
                    },
                )?);
            }
            indexed_polygons.push(indexed);
        }
        return Ok((vertices, indexed_polygons));
    }

    let position_count = polygons
        .iter()
        .map(|polygon| polygon.borrow().vertex_count())
        .sum();
    let mut positions = Vec::with_capacity(position_count);
    let mut indexed_polygons = Vec::with_capacity(polygons.len());

    for (polygon_index, polygon) in polygons.iter().enumerate() {
        let polygon = polygon.borrow();
        indexed_polygons.push(vec![0; polygon.vertex_count()]);
        let vertex_identities = polygon.known_vertex_identities();
        if let Some(points) = polygon.known_vertices.as_ref() {
            for (vertex_index, point) in points.iter().enumerate() {
                positions.push((
                    polygon_index,
                    vertex_index,
                    OutputVertex {
                        x: point.x.clone(),
                        y: point.y.clone(),
                        z: point.z.clone(),
                    },
                    vertex_identities.and_then(|identities| identities.get(vertex_index)),
                ));
            }
        } else {
            for (vertex_index, point) in polygon
                .vertices_decision(decisions)?
                .into_iter()
                .enumerate()
            {
                positions.push((
                    polygon_index,
                    vertex_index,
                    OutputVertex {
                        x: point.x,
                        y: point.y,
                        z: point.z,
                    },
                    vertex_identities.and_then(|identities| identities.get(vertex_index)),
                ));
            }
        }
    }

    if positions
        .iter()
        .all(|(_, _, vertex, _)| vertex.has_exact_rational_coordinates())
    {
        let mut interner = PointInterner::<()>::try_with_capacity(positions.len(), true, false)?;
        let mut vertices = Vec::new();
        vertices.try_reserve_exact(positions.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "convex output vertices",
            }
        })?;
        for (polygon_index, vertex_index, vertex, _) in positions {
            indexed_polygons[polygon_index][vertex_index] =
                interner.intern_owned(decisions, &mut vertices, vertex, None)?;
        }
        return Ok((vertices, indexed_polygons));
    }

    // Projective construction identities have already been canonicalized with
    // exact plane-incidence certificates. The shared interner therefore avoids
    // policy-aware equality between two distinct retained identities, while
    // certified interval misses remain the only numeric pruning proof.
    let mut interner = PointInterner::<ConstructionVertexIdentity>::try_with_capacity(
        positions.len(),
        false,
        true,
    )?;
    let mut vertices = Vec::new();
    vertices
        .try_reserve_exact(positions.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "convex output vertices",
        })?;
    for (polygon_index, vertex_index, vertex, identity) in positions {
        indexed_polygons[polygon_index][vertex_index] =
            interner.intern_owned(decisions, &mut vertices, vertex, identity)?;
    }

    Ok((vertices, indexed_polygons))
}

#[cfg(test)]
fn compare_output_vertices_lexicographic(
    decisions: &DecisionContext,
    left: &OutputVertex,
    right: &OutputVertex,
) -> HypermeshResult<std::cmp::Ordering> {
    let x = compare_real_decision(decisions, &left.x, &right.x)?;
    if !x.is_eq() {
        return Ok(x);
    }
    let y = compare_real_decision(decisions, &left.y, &right.y)?;
    if !y.is_eq() {
        return Ok(y);
    }
    compare_real_decision(decisions, &left.z, &right.z)
}

struct ConstructionEdgeCandidates {
    groups: Vec<ConstructionEdgeCandidateGroup>,
    polygon_edges: Vec<Vec<usize>>,
    recovery_vertices: Vec<usize>,
}

struct ConstructionEdgeCandidateGroup {
    collinear: Vec<usize>,
}

fn build_construction_edge_candidates<P>(
    polygons: &[P],
    indexed_polygons: &[Vec<usize>],
    vertex_count: usize,
) -> HypermeshResult<ConstructionEdgeCandidates>
where
    P: Borrow<ConvexPolygon>,
{
    if polygons.len() != indexed_polygons.len() {
        return Err(HypermeshError::UnknownClassification);
    }
    let mut group_indices: StorageHashMap<ConstructionEdgeIdentity, usize> =
        StorageHashMap::default();
    let mut groups: Vec<ConstructionEdgeCandidateGroup> = Vec::new();
    let mut polygon_edges = Vec::with_capacity(polygons.len());
    for (polygon, indexed) in polygons.iter().zip(indexed_polygons) {
        let polygon = polygon.borrow();
        let identities = polygon
            .known_edge_identities()
            .ok_or(HypermeshError::UnknownClassification)?;
        let vertex_identities = polygon
            .known_vertex_identities()
            .ok_or(HypermeshError::UnknownClassification)?;
        if indexed.len() != identities.len() {
            return Err(HypermeshError::UnknownClassification);
        }
        if indexed.len() != vertex_identities.len() {
            return Err(HypermeshError::UnknownClassification);
        }
        let mut edge_groups = Vec::with_capacity(indexed.len());
        for (edge_index, identity) in identities.iter().enumerate() {
            let group_index = match group_indices.entry(identity.clone()) {
                std::collections::hash_map::Entry::Occupied(entry) => *entry.get(),
                std::collections::hash_map::Entry::Vacant(entry) => {
                    let index = groups.len();
                    groups.push(ConstructionEdgeCandidateGroup {
                        collinear: Vec::new(),
                    });
                    entry.insert(index);
                    index
                }
            };
            groups[group_index].collinear.push(indexed[edge_index]);
            groups[group_index]
                .collinear
                .push(indexed[(edge_index + 1) % indexed.len()]);
            edge_groups.push(group_index);
        }
        polygon_edges.push(edge_groups);
    }
    let mut split_group_indices: StorageHashMap<[ConstructionPlaneIdentity; 2], usize> =
        StorageHashMap::default();
    for (identity, &group_index) in &group_indices {
        if let ConstructionEdgeIdentity::Split { planes } = identity {
            let mut planes = *planes;
            planes.sort_unstable();
            split_group_indices.insert(planes, group_index);
        }
    }
    if !split_group_indices.is_empty() {
        for (polygon, indexed) in polygons.iter().zip(indexed_polygons) {
            let vertex_identities = polygon
                .borrow()
                .known_vertex_identities()
                .ok_or(HypermeshError::UnknownClassification)?;
            for (&vertex, identity) in indexed.iter().zip(vertex_identities) {
                let ConstructionVertexIdentity::PlaneTriple { mut planes } = identity else {
                    continue;
                };
                planes.sort_unstable();
                let pairs = [
                    [planes[0], planes[1]],
                    [planes[0], planes[2]],
                    [planes[1], planes[2]],
                ];
                for (pair_index, pair) in pairs.into_iter().enumerate() {
                    if pair_index > 0 && pairs[..pair_index].contains(&pair) {
                        continue;
                    }
                    if let Some(&group_index) = split_group_indices.get(&pair) {
                        groups[group_index].collinear.push(vertex);
                    }
                }
            }
        }
    }
    // Construction labels give the cheapest candidate set, but a retained
    // source edge and a newly split edge can describe the same exact line
    // without sharing a label. Include all arrangement vertices as recovery
    // proposals; the inexpensive approximate segment filter narrows this set,
    // and exact containment plus final closure certification remain mandatory.
    // Vertex merging constructs a compact pool from these polygon positions,
    // so its complete index set is exactly this contiguous range.
    let recovery_vertices = (0..vertex_count).collect();
    for group in &mut groups {
        group.collinear.sort_unstable();
        group.collinear.dedup();
    }
    Ok(ConstructionEdgeCandidates {
        groups,
        polygon_edges,
        recovery_vertices,
    })
}

fn polygon_edge_counts(
    decisions: &DecisionContext,
    vertices: &[OutputVertex],
    polygons: &[Vec<usize>],
    axis_order: &[Vec<usize>; 3],
) -> HypermeshResult<StorageHashMap<[usize; 2], DirectedEdgeUses>> {
    let mut counts: StorageHashMap<[usize; 2], DirectedEdgeUses> = StorageHashMap::default();
    let mut split_edge_cache = SplitEdgeCache::default();

    for polygon in polygons {
        if polygon.len() < 2 {
            continue;
        }

        for edge_index in 0..polygon.len() {
            let start = polygon[edge_index];
            let end = polygon[(edge_index + 1) % polygon.len()];
            if start == end {
                continue;
            }
            let canonical_edge = sorted_edge([start, end]);
            let follows_canonical_edge = start == canonical_edge[0];
            for canonical_subedge in split_segment_subedges_exact(
                decisions,
                &mut split_edge_cache,
                vertices,
                SplitEdgeSearch::AxisOrder(axis_order),
                canonical_edge,
            )?
            .subedges()
            {
                let subedge = if follows_canonical_edge {
                    canonical_subedge
                } else {
                    [canonical_subedge[1], canonical_subedge[0]]
                };
                let key = sorted_edge(subedge);
                let uses = counts.entry(key).or_default();
                if subedge == key {
                    uses.forward += 1;
                } else {
                    uses.reverse += 1;
                }
            }
        }
    }

    Ok(counts)
}

fn split_segment_subedges_exact<'a>(
    decisions: &DecisionContext,
    cache: &'a mut SplitEdgeCache,
    vertices: &[OutputVertex],
    search: SplitEdgeSearch<'_>,
    edge: [usize; 2],
) -> HypermeshResult<&'a SplitEdgeChain> {
    let edge = sorted_edge(edge);
    match cache.entry(edge) {
        std::collections::hash_map::Entry::Occupied(entry) => Ok(entry.into_mut()),
        std::collections::hash_map::Entry::Vacant(entry) => {
            Ok(entry.insert(build_split_edge_chain(decisions, vertices, search, edge)?))
        }
    }
}

fn split_segment_subedges_exact_candidates<'a>(
    decisions: &DecisionContext,
    cache: &'a mut SplitEdgeCache,
    vertices: &[OutputVertex],
    edge: [usize; 2],
    candidates: &ConstructionEdgeCandidateGroup,
    recovery_vertices: &[usize],
    rational_vertex_queries: Option<&[Option<RationalPoint3Query>]>,
    filter_recovery_candidates: bool,
) -> HypermeshResult<&'a SplitEdgeChain> {
    let edge = sorted_edge(edge);
    match cache.entry(edge) {
        std::collections::hash_map::Entry::Occupied(entry) => Ok(entry.into_mut()),
        std::collections::hash_map::Entry::Vacant(entry) => {
            let axis =
                inexpensive_nonzero_segment_axis(decisions, &vertices[edge[0]], &vertices[edge[1]])
                    .inspect_err(|error| {
                        if cfg!(debug_assertions) {
                            eprintln!("[DEBUG] construction edge axis failed: {error}");
                        }
                    })?;
            let projection_filters = filter_recovery_candidates
                .then(|| {
                    let queries = rational_vertex_queries?;
                    let from = queries.get(edge[0])?.as_ref()?;
                    let to = queries.get(edge[1])?.as_ref()?;
                    (0..3)
                        .filter(|&other_axis| other_axis != axis)
                        .map(|other_axis| {
                            Some((
                                other_axis,
                                RationalLine2Filter::from_point3(from, to, [axis, other_axis])?,
                            ))
                        })
                        .collect::<Option<Vec<(usize, RationalLine2Filter)>>>()
                })
                .flatten();
            let mut on_edge = Vec::new();
            for &vertex_index in &candidates.collinear {
                if vertex_index == edge[0] || vertex_index == edge[1] {
                    continue;
                }
                if point_on_segment_exact(
                    decisions,
                    &vertices[vertex_index],
                    &vertices[edge[0]],
                    &vertices[edge[1]],
                )? {
                    on_edge.push(vertex_index);
                }
            }
            for &vertex_index in recovery_vertices
                .iter()
                .take(if filter_recovery_candidates {
                    usize::MAX
                } else {
                    0
                })
            {
                if vertex_index == edge[0]
                    || vertex_index == edge[1]
                    || on_edge.contains(&vertex_index)
                {
                    continue;
                }
                if projection_filters.as_ref().is_some_and(|filters| {
                    let Some(point) = rational_vertex_queries
                        .and_then(|queries| queries.get(vertex_index))
                        .and_then(Option::as_ref)
                    else {
                        return false;
                    };
                    filters.iter().any(|(other_axis, filter)| {
                        filter.sign_point3(point, [axis, *other_axis]).is_some()
                    })
                }) {
                    continue;
                }
                if point_on_segment_exact(
                    decisions,
                    &vertices[vertex_index],
                    &vertices[edge[0]],
                    &vertices[edge[1]],
                )? {
                    on_edge.push(vertex_index);
                }
            }
            on_edge.sort_unstable();
            on_edge.dedup();
            let mut chain = Vec::with_capacity(on_edge.len() + 2);
            chain.push(edge[0]);
            chain.extend(sort_along_segment_on_axis(
                decisions, &on_edge, edge[0], edge[1], vertices, axis,
            )?);
            chain.push(edge[1]);
            Ok(entry.insert(SplitEdgeChain(chain)))
        }
    }
}

fn exact_output_vertex_enclosures(
    vertices: &[OutputVertex],
) -> Option<Vec<ApproximateOutputVertex>> {
    let mut approximate = Vec::with_capacity(vertices.len());
    for vertex in vertices {
        let coordinates = [&vertex.x, &vertex.y, &vertex.z];
        let [Some(x), Some(y), Some(z)] =
            coordinates.map(|coordinate| coordinate.exact_rational_ref()?.to_f64_enclosure())
        else {
            return None;
        };
        approximate.push([x, y, z]);
    }
    Some(approximate)
}

fn build_split_edge_chain(
    decisions: &DecisionContext,
    vertices: &[OutputVertex],
    search: SplitEdgeSearch<'_>,
    edge: [usize; 2],
) -> HypermeshResult<SplitEdgeChain> {
    let edge = sorted_edge(edge);
    let mut on_edge = Vec::new();
    match search {
        SplitEdgeSearch::Approximate(approximate_vertices) => {
            let start = approximate_vertices[edge[0]];
            let end = approximate_vertices[edge[1]];
            for (vertex_index, point) in approximate_vertices.iter().enumerate() {
                if vertex_index == edge[0] || vertex_index == edge[1] {
                    continue;
                }
                if (0..3).all(|axis| {
                    point[axis][1] >= start[axis][0].min(end[axis][0])
                        && point[axis][0] <= start[axis][1].max(end[axis][1])
                }) && point_on_segment_exact(
                    decisions,
                    &vertices[vertex_index],
                    &vertices[edge[0]],
                    &vertices[edge[1]],
                )? {
                    on_edge.push(vertex_index);
                }
            }
        }
        SplitEdgeSearch::AxisOrder(axis_order) => {
            let axis = dominant_segment_axis(decisions, &vertices[edge[0]], &vertices[edge[1]])?;
            let bounds = exact_edge_bounds(decisions, edge, vertices, None)?;
            let (start, end) =
                candidate_vertex_index_range_for_edge(decisions, axis_order, vertices, edge, axis)?;
            for &vertex_index in &axis_order[axis][start..end] {
                if vertex_index == edge[0] || vertex_index == edge[1] {
                    continue;
                }
                if point_within_edge_bounds_except_axis_exact(
                    decisions,
                    &vertices[vertex_index],
                    &bounds,
                    vertices,
                    axis,
                )? && point_collinear_with_segment_exact(
                    decisions,
                    &vertices[vertex_index],
                    &vertices[edge[0]],
                    &vertices[edge[1]],
                )? {
                    on_edge.push(vertex_index);
                }
            }
        }
    }

    let mut chain = Vec::with_capacity(on_edge.len() + 2);
    chain.push(edge[0]);
    chain.extend(sort_along_segment(
        decisions, &on_edge, edge[0], edge[1], vertices,
    )?);
    chain.push(edge[1]);
    Ok(SplitEdgeChain(chain))
}

fn sorted_vertex_indices_by_axis(
    decisions: &DecisionContext,
    vertices: &[OutputVertex],
) -> HypermeshResult<[Vec<usize>; 3]> {
    const COMPARISON_SORT_MIN_VERTICES: usize = 32;
    let mut order = [
        (0..vertices.len()).collect::<Vec<_>>(),
        (0..vertices.len()).collect::<Vec<_>>(),
        (0..vertices.len()).collect::<Vec<_>>(),
    ];
    for (axis, axis_order) in order.iter_mut().enumerate() {
        if axis_order.len() >= COMPARISON_SORT_MIN_VERTICES {
            let mut error = None;
            axis_order.sort_unstable_by(|&left, &right| {
                if error.is_some() {
                    return std::cmp::Ordering::Equal;
                }
                match compare_real_decision(
                    decisions,
                    vertex_axis(&vertices[left], axis),
                    vertex_axis(&vertices[right], axis),
                ) {
                    Ok(ordering) => ordering,
                    Err(sort_error) => {
                        error = Some(sort_error);
                        std::cmp::Ordering::Equal
                    }
                }
            });
            if let Some(error) = error {
                return Err(error);
            }
            continue;
        }
        for index in 1..axis_order.len() {
            let mut current = index;
            while current > 0
                && compare_real_decision(
                    decisions,
                    vertex_axis(&vertices[axis_order[current]], axis),
                    vertex_axis(&vertices[axis_order[current - 1]], axis),
                )?
                .is_lt()
            {
                axis_order.swap(current, current - 1);
                current -= 1;
            }
        }
    }
    Ok(order)
}

fn candidate_vertex_index_range_for_edge(
    decisions: &DecisionContext,
    axis_order: &[Vec<usize>; 3],
    vertices: &[OutputVertex],
    edge: [usize; 2],
    axis: usize,
) -> HypermeshResult<(usize, usize)> {
    let start_value = vertex_axis(&vertices[edge[0]], axis);
    let end_value = vertex_axis(&vertices[edge[1]], axis);
    let (min_value, max_value) =
        if compare_real_decision(decisions, start_value, end_value)?.is_le() {
            (start_value, end_value)
        } else {
            (end_value, start_value)
        };

    let ordered = &axis_order[axis];
    let start = lower_bound_vertex_axis(decisions, ordered, vertices, axis, min_value)?;
    let end = upper_bound_vertex_axis(decisions, ordered, vertices, axis, max_value)?;
    Ok((start, end))
}

fn lower_bound_vertex_axis(
    decisions: &DecisionContext,
    ordered: &[usize],
    vertices: &[OutputVertex],
    axis: usize,
    value: &Real,
) -> HypermeshResult<usize> {
    let mut low = 0;
    let mut high = ordered.len();
    while low < high {
        let mid = (low + high) / 2;
        if compare_real_decision(decisions, vertex_axis(&vertices[ordered[mid]], axis), value)?
            .is_lt()
        {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    Ok(low)
}

fn upper_bound_vertex_axis(
    decisions: &DecisionContext,
    ordered: &[usize],
    vertices: &[OutputVertex],
    axis: usize,
    value: &Real,
) -> HypermeshResult<usize> {
    let mut low = 0;
    let mut high = ordered.len();
    while low < high {
        let mid = (low + high) / 2;
        if compare_real_decision(decisions, vertex_axis(&vertices[ordered[mid]], axis), value)?
            .is_gt()
        {
            high = mid;
        } else {
            low = mid + 1;
        }
    }
    Ok(low)
}

fn triangulate_polygons(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    orientations: Option<&[i8]>,
) -> HypermeshResult<BooleanMesh> {
    if orientations.is_some_and(|orientations| orientations.len() != polygons.len()) {
        return Err(HypermeshError::UnknownClassification);
    }
    let mut soup = BooleanMesh::default();

    for (polygon_index, polygon) in polygons.iter().enumerate() {
        let vertex_count = polygon.vertex_count();
        if vertex_count < 3 {
            continue;
        }

        let base = soup.vertices.len();
        append_polygon_output_vertices(decisions, &mut soup.vertices, polygon)?;

        for index in 1..(vertex_count - 1) {
            soup.triangles.push([base, base + index, base + index + 1]);
            soup.sources.push(TriangleSource {
                mesh: polygon.mesh_index,
                triangle: polygon.polygon_index,
                orientation: orientations
                    .and_then(|orientations| orientations.get(polygon_index))
                    .copied()
                    .unwrap_or(0),
            });
        }
    }

    Ok(soup)
}

/// Resolves exact duplicate vertices, duplicate faces, and exact T-junctions.
///
/// This pass deliberately uses no tolerance. Primitive floating-point values
/// only schedule or conservatively reject candidates; exact hyperreal
/// predicates prove every merge, split, crossing, and containment decision.
pub(crate) fn resolve_tjunctions(
    decisions: &DecisionContext,
    input: &BooleanMesh,
) -> HypermeshResult<BooleanMesh> {
    let mut soup = merge_duplicate_vertices(decisions, input)?;
    remove_degenerate_and_duplicate_triangles(decisions, &mut soup)?;
    let mut approximate_vertices = exact_output_vertex_enclosures(&soup.vertices);

    loop {
        if split_one_tjunction_pass(decisions, &mut soup, approximate_vertices.as_deref())? {
            remove_degenerate_and_duplicate_triangles(decisions, &mut soup)?;
            continue;
        }
        if split_edge_crossing_events(decisions, &mut soup, approximate_vertices.as_deref())? {
            remove_degenerate_and_duplicate_triangles(decisions, &mut soup)?;
            approximate_vertices = exact_output_vertex_enclosures(&soup.vertices);
            continue;
        }
        return Ok(soup);
    }
}

fn output_vertices_equal(
    decisions: &DecisionContext,
    left: &OutputVertex,
    right: &OutputVertex,
) -> HypermeshResult<bool> {
    crate::predicate::coordinates3_equal(
        decisions,
        [&left.x, &left.y, &left.z],
        [&right.x, &right.y, &right.z],
    )
}

fn merge_duplicate_vertices(
    decisions: &DecisionContext,
    input: &BooleanMesh,
) -> HypermeshResult<BooleanMesh> {
    let exact_only = input
        .vertices
        .iter()
        .all(PointCoordinates::has_exact_rational_coordinates);
    let mut interner =
        PointInterner::<()>::try_with_capacity(input.vertices.len(), exact_only, false)?;
    let mut vertices = Vec::new();
    vertices
        .try_reserve_exact(input.vertices.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "output vertices",
        })?;
    let mut remap = Vec::new();
    remap.try_reserve_exact(input.vertices.len()).map_err(|_| {
        HypermeshError::CapacityOverflow {
            operation: "output vertex remap",
        }
    })?;
    for vertex in &input.vertices {
        remap.push(interner.intern_cloned(decisions, &mut vertices, vertex, None)?);
    }

    let triangles = remap_triangle_indices(&input.triangles, &remap)?;

    Ok(BooleanMesh {
        vertices,
        triangles,
        sources: input.sources.clone(),
    })
}

fn remap_triangle_indices(
    triangles: &[[usize; 3]],
    remap: &[usize],
) -> HypermeshResult<Vec<[usize; 3]>> {
    triangles
        .iter()
        .map(|triangle| {
            triangle.map(|index| {
                remap
                    .get(index)
                    .copied()
                    .ok_or(HypermeshError::VertexIndexOutOfBounds {
                        index,
                        vertex_count: remap.len(),
                    })
            })
        })
        .map(|triangle| {
            let [a, b, c] = triangle;
            Ok([a?, b?, c?])
        })
        .collect()
}

fn remove_degenerate_and_duplicate_triangles(
    decisions: &DecisionContext,
    soup: &mut BooleanMesh,
) -> HypermeshResult<()> {
    if soup.triangles.len() != soup.sources.len() {
        return Err(HypermeshError::TriangleSourceCountMismatch {
            triangles: soup.triangles.len(),
            sources: soup.sources.len(),
        });
    }
    if let Some(index) = soup
        .triangles
        .iter()
        .flatten()
        .find(|index| **index >= soup.vertices.len())
        .copied()
    {
        return Err(HypermeshError::VertexIndexOutOfBounds {
            index,
            vertex_count: soup.vertices.len(),
        });
    }
    let mut seen = BTreeSet::new();
    let mut triangles = Vec::with_capacity(soup.triangles.len());
    let mut sources = Vec::with_capacity(soup.sources.len());
    for (triangle, source) in soup.triangles.drain(..).zip(soup.sources.drain(..)) {
        let [a, b, c] = triangle.map(|index| &soup.vertices[index]);
        if triangle[0] == triangle[1]
            || triangle[1] == triangle[2]
            || triangle[0] == triangle[2]
            || !Plane::decide_points_are_nondegenerate(
                decisions,
                &Point3::new(a.x.clone(), a.y.clone(), a.z.clone()),
                &Point3::new(b.x.clone(), b.y.clone(), b.z.clone()),
                &Point3::new(c.x.clone(), c.y.clone(), c.z.clone()),
            )?
        {
            continue;
        }
        let mut key = triangle;
        key.sort();
        if seen.insert(key) {
            triangles.push(triangle);
            sources.push(source);
        }
    }
    soup.triangles = triangles;
    soup.sources = sources;
    Ok(())
}

fn triangle_edge_counts(triangles: &[[usize; 3]]) -> StorageHashMap<[usize; 2], DirectedEdgeUses> {
    let mut counts: StorageHashMap<[usize; 2], DirectedEdgeUses> = StorageHashMap::default();
    counts.reserve(triangles.len().saturating_mul(3) / 2);
    for triangle in triangles {
        for edge in triangle_edges(*triangle) {
            let key = sorted_edge(edge);
            let uses = counts.entry(key).or_default();
            if edge == key {
                uses.forward += 1;
            } else {
                uses.reverse += 1;
            }
        }
    }
    counts
}

/// Returns true when every undirected triangle edge has exactly two
/// oppositely directed uses.
pub fn boolean_mesh_is_closed(soup: &BooleanMesh) -> bool {
    boolean_mesh_closure_evidence(soup).is_closed()
}

/// Counts exact singleton, directed-imbalance, and non-manifold edges in a
/// triangle soup.
pub fn boolean_mesh_closure_evidence(soup: &BooleanMesh) -> BooleanMeshClosureEvidence {
    let mut evidence = BooleanMeshClosureEvidence::default();
    for uses in triangle_edge_counts(&soup.triangles).values().copied() {
        update_closure_evidence(&mut evidence, uses);
    }
    evidence
}

fn update_closure_evidence(evidence: &mut BooleanMeshClosureEvidence, uses: DirectedEdgeUses) {
    if uses.total() == 1 {
        evidence.boundary_edges += 1;
    } else if uses.total() > 2 {
        evidence.non_manifold_edges += 1;
    }
    if !uses.is_balanced() {
        evidence.unbalanced_edges += 1;
    }
}

fn split_one_tjunction_pass(
    decisions: &DecisionContext,
    soup: &mut BooleanMesh,
    approximate_vertices: Option<&[ApproximateOutputVertex]>,
) -> HypermeshResult<bool> {
    let directed_uses = triangle_edge_counts(&soup.triangles);
    if directed_uses.values().all(|uses| uses.is_balanced()) {
        return Ok(false);
    }
    let axis_order = approximate_vertices
        .is_none()
        .then(|| sorted_vertex_indices_by_axis(decisions, &soup.vertices))
        .transpose()?;
    let mut edge_faces: BTreeMap<[usize; 2], Vec<usize>> = BTreeMap::new();
    for (face_index, triangle) in soup.triangles.iter().enumerate() {
        for edge in triangle_edges(*triangle) {
            edge_faces
                .entry(sorted_edge(edge))
                .or_default()
                .push(face_index);
        }
    }

    let mut to_remove = BTreeSet::new();
    let mut new_triangles = Vec::new();

    for (edge, faces) in edge_faces {
        // A T-junction can repair an open boundary only when the long edge's
        // directed uses do not already cancel. Skipping balanced internal
        // diagonals avoids testing every vertex against centroid spokes and
        // other triangulation-only edges.
        if directed_uses
            .get(&edge)
            .is_some_and(|uses| uses.is_balanced())
        {
            continue;
        }
        let chain = build_split_edge_chain(
            decisions,
            &soup.vertices,
            if let Some(approximate_vertices) = approximate_vertices {
                SplitEdgeSearch::Approximate(approximate_vertices)
            } else {
                SplitEdgeSearch::AxisOrder(
                    axis_order
                        .as_ref()
                        .expect("axis order exists without an approximate scan"),
                )
            },
            edge,
        )?;
        if chain.0.len() <= 2 {
            continue;
        }

        for face_index in faces {
            if to_remove.contains(&face_index) {
                continue;
            }

            let triangle = soup.triangles[face_index];
            for edge_index in 0..3 {
                let ea = triangle[edge_index];
                let eb = triangle[(edge_index + 1) % 3];
                let ec = triangle[(edge_index + 2) % 3];
                if sorted_edge([ea, eb]) != edge {
                    continue;
                }

                let follows_canonical_edge = ea == edge[0];
                let subedge_count = chain.0.len() - 1;
                for offset in 0..subedge_count {
                    let index = if follows_canonical_edge {
                        offset
                    } else {
                        subedge_count - offset - 1
                    };
                    let pair = if follows_canonical_edge {
                        [chain.0[index], chain.0[index + 1]]
                    } else {
                        [chain.0[index + 1], chain.0[index]]
                    };
                    if pair[0] != pair[1] && pair[0] != ec && pair[1] != ec {
                        new_triangles.push(([pair[0], pair[1], ec], soup.sources[face_index]));
                    }
                }
                to_remove.insert(face_index);
                break;
            }
        }
    }

    if to_remove.is_empty() {
        return Ok(false);
    }

    let mut kept = Vec::with_capacity(soup.triangles.len() + new_triangles.len());
    let mut kept_sources = Vec::with_capacity(soup.sources.len() + new_triangles.len());
    for (index, triangle) in soup.triangles.iter().enumerate() {
        if !to_remove.contains(&index) {
            kept.push(*triangle);
            kept_sources.push(soup.sources[index]);
        }
    }
    for (triangle, source) in new_triangles {
        kept.push(triangle);
        kept_sources.push(source);
    }
    soup.triangles = kept;
    soup.sources = kept_sources;
    Ok(true)
}

fn split_edge_crossing_events(
    decisions: &DecisionContext,
    soup: &mut BooleanMesh,
    approximate_vertices: Option<&[ApproximateOutputVertex]>,
) -> HypermeshResult<bool> {
    let edge_capacity =
        soup.triangles
            .len()
            .checked_mul(3)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "output edge crossing discovery",
            })?;
    let mut edges = Vec::new();
    edges
        .try_reserve_exact(edge_capacity)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "output edge crossing discovery",
        })?;
    for triangle in &soup.triangles {
        for edge in triangle_edges(*triangle) {
            edges.push(sorted_edge(edge));
        }
    }
    edges.sort();
    edges.dedup();

    let mut bounded_edges = edges
        .into_iter()
        .map(|edge| exact_edge_bounds(decisions, edge, &soup.vertices, approximate_vertices))
        .collect::<HypermeshResult<Vec<_>>>()?;
    let approximate_sweep_axis = approximate_vertices
        .map(|vertices| approximate_crossing_sweep_axis(vertices, &bounded_edges))
        .unwrap_or(0);
    if let Some(approximate_vertices) = approximate_vertices {
        bounded_edges.sort_unstable_by(|left, right| {
            approximate_edge_min(approximate_vertices, left.edge, approximate_sweep_axis)
                .total_cmp(&approximate_edge_min(
                    approximate_vertices,
                    right.edge,
                    approximate_sweep_axis,
                ))
                .then_with(|| {
                    vertex_axis(
                        &soup.vertices[left.min[approximate_sweep_axis]],
                        approximate_sweep_axis,
                    )
                    .exact_rational_ref()
                    .expect("the approximate sweep contains exact rationals")
                    .partial_cmp(
                        vertex_axis(
                            &soup.vertices[right.min[approximate_sweep_axis]],
                            approximate_sweep_axis,
                        )
                        .exact_rational_ref()
                        .expect("the approximate sweep contains exact rationals"),
                    )
                    .expect("rational ordering is total")
                })
                .then_with(|| left.edge.cmp(&right.edge))
        });
    } else {
        for index in 1..bounded_edges.len() {
            let mut current = index;
            while current > 0 {
                let ordering = compare_real_decision(
                    decisions,
                    vertex_axis(&soup.vertices[bounded_edges[current - 1].min[0]], 0),
                    vertex_axis(&soup.vertices[bounded_edges[current].min[0]], 0),
                )?
                .then_with(|| {
                    bounded_edges[current - 1]
                        .edge
                        .cmp(&bounded_edges[current].edge)
                });
                if !ordering.is_gt() {
                    break;
                }
                bounded_edges.swap(current - 1, current);
                current -= 1;
            }
        }
    }
    let approximate_bounds = match approximate_vertices {
        Some(vertices) if bounded_edges.len() >= MIN_CACHED_CROSSING_BOUNDS_EDGES => {
            let mut bounds = Vec::new();
            if bounds.try_reserve_exact(bounded_edges.len()).is_err() {
                None
            } else {
                for edge in &bounded_edges {
                    bounds.push(approximate_edge_bounds(vertices, edge.edge));
                }
                Some(bounds)
            }
        }
        _ => None,
    };

    let mut interner = None;
    let mut events = Vec::new();
    #[derive(Clone, Copy)]
    enum ApproximateLeftBounds<'a> {
        Cached(&'a [ApproximateEdgeBounds], &'a ApproximateEdgeBounds),
        Direct(&'a [ApproximateOutputVertex], ApproximateEdgeBounds),
    }
    for left_index in 0..bounded_edges.len() {
        let left = &bounded_edges[left_index];
        let approximate_left_bounds = match (approximate_vertices, approximate_bounds.as_deref()) {
            (Some(_), Some(bounds)) => {
                Some(ApproximateLeftBounds::Cached(bounds, &bounds[left_index]))
            }
            (Some(vertices), None) => Some(ApproximateLeftBounds::Direct(
                vertices,
                approximate_edge_bounds(vertices, left.edge),
            )),
            (None, _) => None,
        };
        for right_index in (left_index + 1)..bounded_edges.len() {
            let right = &bounded_edges[right_index];
            if let Some(ApproximateLeftBounds::Cached(approximate_bounds, left_bounds)) =
                approximate_left_bounds
            {
                let right_bounds = &approximate_bounds[right_index];
                // Disjoint outward-rounded enclosures prove exact separation;
                // every survivor is checked against the exact bounds below.
                if right_bounds[approximate_sweep_axis][0] > left_bounds[approximate_sweep_axis][1]
                {
                    break;
                }
                if !approximate_bounds_overlap(left_bounds, right_bounds) {
                    continue;
                }
            } else if let Some(ApproximateLeftBounds::Direct(vertices, left_bounds)) =
                approximate_left_bounds
            {
                if approximate_edge_min(vertices, right.edge, approximate_sweep_axis)
                    > left_bounds[approximate_sweep_axis][1]
                {
                    break;
                }
                if !approximate_edge_overlaps_bounds(vertices, right.edge, &left_bounds) {
                    continue;
                }
            } else if compare_real_decision(
                decisions,
                vertex_axis(&soup.vertices[right.min[0]], 0),
                vertex_axis(&soup.vertices[left.max[0]], 0),
            )?
            .is_gt()
            {
                break;
            }
            let left_edge = left.edge;
            let right_edge = right.edge;
            if left_edge.iter().any(|vertex| right_edge.contains(vertex)) {
                continue;
            }
            if !edge_bounds_overlap_exact(
                decisions,
                left,
                right,
                &soup.vertices,
                usize::from(approximate_vertices.is_none()),
            )? {
                continue;
            }

            let rational_queries = approximate_vertices.and_then(|vertices| {
                Some([
                    RationalPoint3Query::from_certified_enclosures(vertices[left_edge[0]])?,
                    RationalPoint3Query::from_certified_enclosures(vertices[left_edge[1]])?,
                    RationalPoint3Query::from_certified_enclosures(vertices[right_edge[0]])?,
                    RationalPoint3Query::from_certified_enclosures(vertices[right_edge[1]])?,
                ])
            });
            let Some(projection_axis) = proper_segment_intersection_after_bounds_overlap(
                decisions,
                &soup.vertices[left_edge[0]],
                &soup.vertices[left_edge[1]],
                &soup.vertices[right_edge[0]],
                &soup.vertices[right_edge[1]],
                approximate_vertices.and_then(|vertices| {
                    approximate_projection_axis(vertices, left_edge, right_edge)
                }),
                rational_queries.as_ref(),
            )?
            else {
                continue;
            };
            let point = proper_output_segment_intersection_point(
                decisions,
                &soup.vertices,
                left_edge,
                right_edge,
                projection_axis,
            )?;
            if interner.is_none() {
                let exact_only = soup
                    .vertices
                    .iter()
                    .all(PointCoordinates::has_exact_rational_coordinates);
                interner = Some(PointInterner::<()>::try_from_unique(
                    &soup.vertices,
                    exact_only,
                )?);
            }
            let vertex = interner
                .as_mut()
                .expect("the crossing point interner was initialized")
                .intern_owned(decisions, &mut soup.vertices, point, None)?;
            events
                .try_reserve(1)
                .map_err(|_| HypermeshError::CapacityOverflow {
                    operation: "output edge crossing events",
                })?;
            events.push(OutputCrossingEvent {
                left_edge,
                right_edge,
                vertex,
            });
        }
    }
    if events.is_empty() {
        return Ok(false);
    }

    let mut repairs: Vec<CoplanarOutputRepair> = Vec::new();
    for event in &events {
        for edge in [event.left_edge, event.right_edge] {
            let other_edge = if edge == event.left_edge {
                event.right_edge
            } else {
                event.left_edge
            };
            for triangle in soup.triangles.iter().copied().filter(|triangle| {
                triangle_edges(*triangle)
                    .into_iter()
                    .map(sorted_edge)
                    .any(|candidate| candidate == edge)
            }) {
                let [a, b, c] = triangle.map(|vertex| output_vertex_point3(&soup.vertices[vertex]));
                let plane = Plane::from_points(&a, &b, &c);
                if !output_vertex_on_plane(decisions, &plane, &soup.vertices[other_edge[0]])?
                    || !output_vertex_on_plane(decisions, &plane, &soup.vertices[other_edge[1]])?
                {
                    continue;
                }
                let mut matching_repair = None;
                for (index, repair) in repairs.iter().enumerate() {
                    if output_triangle_on_plane(decisions, &repair.plane, &soup.vertices, triangle)?
                    {
                        matching_repair = Some(index);
                        break;
                    }
                }
                let repair_index = match matching_repair {
                    Some(index) => index,
                    None => {
                        repairs
                            .try_reserve(1)
                            .map_err(|_| HypermeshError::CapacityOverflow {
                                operation: "coplanar output repairs",
                            })?;
                        repairs.push(CoplanarOutputRepair {
                            plane,
                            event_vertices: BTreeSet::new(),
                        });
                        repairs.len() - 1
                    }
                };
                repairs[repair_index].event_vertices.insert(event.vertex);
            }
        }
    }
    if std::env::var_os("HYPERMESH_OUTPUT_DIAGNOSTIC").is_some() {
        eprintln!(
            "output crossing batch: events={}, planes={}",
            events.len(),
            repairs.len()
        );
    }
    for repair in repairs {
        retriangulate_coplanar_output_plane(
            decisions,
            soup,
            &repair.plane,
            &repair.event_vertices,
        )?;
    }
    let mut edge_splits: BTreeMap<[usize; 2], BTreeSet<usize>> = BTreeMap::new();
    for event in events {
        edge_splits
            .entry(event.left_edge)
            .or_default()
            .insert(event.vertex);
        edge_splits
            .entry(event.right_edge)
            .or_default()
            .insert(event.vertex);
    }
    split_output_edges_at_vertices(decisions, soup, &edge_splits)?;
    Ok(true)
}

#[derive(Clone, Copy)]
struct OutputCrossingEvent {
    left_edge: [usize; 2],
    right_edge: [usize; 2],
    vertex: usize,
}

struct CoplanarOutputRepair {
    plane: Plane,
    event_vertices: BTreeSet<usize>,
}

#[derive(Clone, Copy)]
struct PlanarCoverageTriangle {
    vertices: [usize; 3],
    source: TriangleSource,
    orientation: i8,
}

fn split_output_edges_at_vertices(
    decisions: &DecisionContext,
    soup: &mut BooleanMesh,
    edge_splits: &BTreeMap<[usize; 2], BTreeSet<usize>>,
) -> HypermeshResult<()> {
    if soup.triangles.len() != soup.sources.len() {
        return Err(HypermeshError::TriangleSourceCountMismatch {
            triangles: soup.triangles.len(),
            sources: soup.sources.len(),
        });
    }
    for (&edge, split_vertices) in edge_splits {
        if edge[1] >= soup.vertices.len() {
            return Err(HypermeshError::VertexIndexOutOfBounds {
                index: edge[1],
                vertex_count: soup.vertices.len(),
            });
        }
        let mut interior = split_vertices.iter().copied().collect::<Vec<_>>();
        for &vertex in &interior {
            if vertex >= soup.vertices.len() {
                return Err(HypermeshError::VertexIndexOutOfBounds {
                    index: vertex,
                    vertex_count: soup.vertices.len(),
                });
            }
            if !point_on_segment_exact(
                decisions,
                &soup.vertices[vertex],
                &soup.vertices[edge[0]],
                &soup.vertices[edge[1]],
            )? {
                return Err(HypermeshError::OutputPlanarizationFailed {
                    reason: "crossing split vertex is not in the edge interior",
                });
            }
        }
        interior = sort_along_segment(decisions, &interior, edge[0], edge[1], &soup.vertices)?;
        let mut chain = Vec::with_capacity(interior.len() + 2);
        chain.push(edge[0]);
        chain.extend(interior);
        chain.push(edge[1]);

        let mut triangles = Vec::with_capacity(
            soup.triangles
                .len()
                .saturating_add(chain.len().saturating_sub(2)),
        );
        let mut sources = Vec::with_capacity(triangles.capacity());
        for (face_index, triangle) in soup.triangles.iter().copied().enumerate() {
            let mut split = false;
            for edge_index in 0..3 {
                let from = triangle[edge_index];
                let to = triangle[(edge_index + 1) % 3];
                let opposite = triangle[(edge_index + 2) % 3];
                if sorted_edge([from, to]) != edge {
                    continue;
                }
                let follows_canonical = from == edge[0];
                for pair in chain.windows(2) {
                    let pair = if follows_canonical {
                        [pair[0], pair[1]]
                    } else {
                        [pair[1], pair[0]]
                    };
                    if pair[0] != opposite && pair[1] != opposite {
                        triangles.push([pair[0], pair[1], opposite]);
                        sources.push(soup.sources[face_index]);
                    }
                }
                split = true;
                break;
            }
            if !split {
                triangles.push(triangle);
                sources.push(soup.sources[face_index]);
            }
        }
        soup.triangles = triangles;
        soup.sources = sources;
    }
    Ok(())
}

fn retriangulate_coplanar_output_plane(
    decisions: &DecisionContext,
    soup: &mut BooleanMesh,
    plane: &Plane,
    event_vertices: &BTreeSet<usize>,
) -> HypermeshResult<()> {
    let diagnostic = std::env::var_os("HYPERMESH_OUTPUT_DIAGNOSTIC").is_some();
    let started = std::time::Instant::now();
    if soup.triangles.len() != soup.sources.len() {
        return Err(HypermeshError::TriangleSourceCountMismatch {
            triangles: soup.triangles.len(),
            sources: soup.sources.len(),
        });
    }
    let projection_axis = output_plane_projection_axis(decisions, plane)?;
    let [u_axis, v_axis] =
        projection_axes(projection_axis).ok_or(HypermeshError::OutputPlanarizationFailed {
            reason: "crossing projection axis is invalid",
        })?;

    let mut group = Vec::new();
    group
        .try_reserve(soup.triangles.len())
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "coplanar output triangle group",
        })?;
    for (index, triangle) in soup.triangles.iter().enumerate() {
        if output_triangle_on_plane(decisions, plane, &soup.vertices, *triangle)? {
            group.push(index);
        }
    }
    if group.is_empty() {
        return Err(HypermeshError::OutputPlanarizationFailed {
            reason: "crossing repair plane has no coplanar triangles",
        });
    }

    let mut global_to_local = vec![usize::MAX; soup.vertices.len()];
    let mut local_to_global = Vec::new();
    let mut constraints = BTreeSet::new();
    for &triangle_index in &group {
        let triangle = soup.triangles[triangle_index];
        let local = triangle
            .map(|global| local_output_vertex(global, &mut global_to_local, &mut local_to_global));
        for edge in triangle_edges(local) {
            if edge[0] != edge[1] {
                constraints.insert(sorted_edge(edge));
            }
        }
    }
    for &global in event_vertices {
        if global >= soup.vertices.len() {
            return Err(HypermeshError::VertexIndexOutOfBounds {
                index: global,
                vertex_count: soup.vertices.len(),
            });
        }
        if !output_vertex_on_plane(decisions, plane, &soup.vertices[global])? {
            return Err(HypermeshError::OutputPlanarizationFailed {
                reason: "crossing event does not lie on an incident triangle plane",
            });
        }
        local_output_vertex(global, &mut global_to_local, &mut local_to_global);
    }
    let mut output_edges = BTreeSet::new();
    for triangle in &soup.triangles {
        output_edges.extend(triangle_edges(*triangle).map(sorted_edge));
    }
    for edge in output_edges {
        if output_vertex_on_plane(decisions, plane, &soup.vertices[edge[0]])?
            && output_vertex_on_plane(decisions, plane, &soup.vertices[edge[1]])?
        {
            constraints.insert(sorted_edge(edge.map(|global| {
                local_output_vertex(global, &mut global_to_local, &mut local_to_global)
            })));
        }
    }

    let mut points = local_to_global
        .iter()
        .map(|&global| {
            let vertex = &soup.vertices[global];
            hypertri::ExactPoint::new(
                vertex_axis(vertex, u_axis).clone(),
                vertex_axis(vertex, v_axis).clone(),
            )
        })
        .collect::<Vec<_>>();
    let mut point_to_global = local_to_global;
    let planar_edges = planarize_output_constraints(
        decisions,
        soup,
        plane,
        projection_axis,
        [u_axis, v_axis],
        &mut points,
        &mut point_to_global,
        &constraints.into_iter().collect::<Vec<_>>(),
    )?;
    if diagnostic {
        eprintln!(
            "output planarize: {:?}, group={}, points={}, edges={}",
            started.elapsed(),
            group.len(),
            points.len(),
            planar_edges.len()
        );
    }
    let mut coverage = Vec::new();
    for &triangle_index in &group {
        let vertices = soup.triangles[triangle_index].map(|global| global_to_local[global]);
        let orientation = match planar_orientation(
            decisions,
            &points[vertices[0]],
            &points[vertices[1]],
            &points[vertices[2]],
        )? {
            Classification::Negative => -1,
            Classification::Positive => 1,
            Classification::On => {
                return Err(HypermeshError::OutputPlanarizationFailed {
                    reason: "coplanar output group contains a degenerate triangle",
                });
            }
        };
        coverage.push(PlanarCoverageTriangle {
            vertices,
            source: soup.sources[triangle_index],
            orientation,
        });
    }
    let faces = bounded_planar_faces(decisions, &points, &planar_edges)?;
    if diagnostic {
        eprintln!(
            "output planar faces: {:?}, triangles={}, faces={}",
            started.elapsed(),
            group.len(),
            faces.len()
        );
    }
    let approximate_points = exact_planar_points_f64(&points);
    let approximate_bounds = approximate_points.as_ref().map(|points| {
        coverage
            .iter()
            .map(|triangle| approximate_planar_triangle_bounds(points, triangle.vertices))
            .collect::<Vec<_>>()
    });

    let mut replacement_triangles = Vec::new();
    let mut replacement_sources = Vec::new();
    for (face_index, face) in faces.into_iter().enumerate() {
        let face_triangles = match triangulate_planar_face(decisions, &points, &face) {
            Ok(triangles) => triangles,
            Err(error) => {
                if diagnostic {
                    eprintln!(
                        "output planar face failure: index={face_index}, vertices={}, unique={}, face={face:?}, error={error:?}",
                        face.len(),
                        face.iter().copied().collect::<BTreeSet<_>>().len(),
                    );
                }
                return Err(error);
            }
        };
        let Some(&sample_triangle) = face_triangles.first() else {
            continue;
        };
        let sample = planar_triangle_centroid(&points, sample_triangle)?;
        let approximate_sample = approximate_points.as_ref().and_then(|_| {
            let [Some(u), Some(v)] = [sample.x.to_f64_lossy(), sample.y.to_f64_lossy()] else {
                return None;
            };
            (u.is_finite() && v.is_finite()).then_some([u, v])
        });
        let mut winding = 0_i32;
        let mut positive_source = None;
        let mut negative_source = None;
        for (coverage_index, candidate) in coverage.iter().enumerate() {
            if approximate_sample.is_some_and(|sample| {
                !approximate_point_within_planar_bounds(
                    sample,
                    approximate_bounds
                        .as_ref()
                        .expect("approximate points have triangle bounds")[coverage_index],
                )
            }) {
                continue;
            }
            if !planar_triangle_contains_point(
                decisions,
                &points,
                candidate.vertices,
                candidate.orientation,
                &sample,
            )? {
                continue;
            }
            winding = winding
                .checked_add(i32::from(candidate.orientation))
                .ok_or(HypermeshError::WindingOverflow)?;
            if candidate.orientation > 0 {
                positive_source.get_or_insert(candidate.source);
            } else {
                negative_source.get_or_insert(candidate.source);
            }
        }
        let source = if winding > 0 {
            positive_source.ok_or(HypermeshError::OutputPlanarizationFailed {
                reason: "positive planar coverage has no source triangle",
            })?
        } else if winding < 0 {
            negative_source.ok_or(HypermeshError::OutputPlanarizationFailed {
                reason: "negative planar coverage has no source triangle",
            })?
        } else {
            continue;
        };
        for mut triangle in face_triangles {
            match planar_orientation(
                decisions,
                &points[triangle[0]],
                &points[triangle[1]],
                &points[triangle[2]],
            )? {
                Classification::Negative => triangle.swap(1, 2),
                Classification::Positive => {}
                Classification::On => {
                    return Err(HypermeshError::OutputPlanarizationFailed {
                        reason: "planar face triangulation emitted a degenerate triangle",
                    });
                }
            }
            if winding < 0 {
                triangle.swap(1, 2);
            }
            replacement_triangles.push(triangle.map(|vertex| point_to_global[vertex]));
            replacement_sources.push(source);
        }
    }

    let mut in_group = vec![false; soup.triangles.len()];
    for index in group {
        in_group[index] = true;
    }
    let retained = soup.triangles.len() - in_group.iter().filter(|member| **member).count();
    let capacity = retained.checked_add(replacement_triangles.len()).ok_or(
        HypermeshError::CapacityOverflow {
            operation: "planarized output soup",
        },
    )?;
    let mut triangles = Vec::with_capacity(capacity);
    let mut sources = Vec::with_capacity(capacity);
    for (index, (&triangle, &source)) in soup.triangles.iter().zip(&soup.sources).enumerate() {
        if !in_group[index] {
            triangles.push(triangle);
            sources.push(source);
        }
    }
    triangles.extend(replacement_triangles);
    sources.extend(replacement_sources);
    soup.triangles = triangles;
    soup.sources = sources;
    if diagnostic {
        eprintln!(
            "output planar replacement: {:?}, triangles={}",
            started.elapsed(),
            soup.triangles.len()
        );
    }
    Ok(())
}

fn local_output_vertex(
    global: usize,
    global_to_local: &mut [usize],
    local_to_global: &mut Vec<usize>,
) -> usize {
    let local = global_to_local[global];
    if local != usize::MAX {
        local
    } else {
        let local = local_to_global.len();
        global_to_local[global] = local;
        local_to_global.push(global);
        local
    }
}

fn planarize_output_constraints(
    decisions: &DecisionContext,
    soup: &mut BooleanMesh,
    plane: &Plane,
    projection_axis: usize,
    axes: [usize; 2],
    points: &mut Vec<hypertri::ExactPoint>,
    point_to_global: &mut Vec<usize>,
    constraints: &[[usize; 2]],
) -> HypermeshResult<BTreeSet<[usize; 2]>> {
    let diagnostic = std::env::var_os("HYPERMESH_OUTPUT_DIAGNOSTIC").is_some();
    let started = std::time::Instant::now();
    let approximate_points = exact_planar_points_f64(points);
    let mut ordered = constraints.to_vec();
    if let Some(approximate_points) = &approximate_points {
        ordered.sort_unstable_by(|left, right| {
            approximate_planar_edge_min(approximate_points, *left, 0)
                .total_cmp(&approximate_planar_edge_min(approximate_points, *right, 0))
                .then_with(|| left.cmp(right))
        });
    }

    let mut interner = None;
    let mut crossing_count = 0_usize;
    let mut existing_intersection_count = 0_usize;
    for left_index in 0..ordered.len() {
        let left = ordered[left_index];
        for &right in &ordered[(left_index + 1)..] {
            if left.iter().any(|vertex| right.contains(vertex)) {
                continue;
            }
            if let Some(approximate_points) = &approximate_points {
                if approximate_planar_edge_min(approximate_points, right, 0)
                    > approximate_planar_edge_max(approximate_points, left, 0)
                {
                    break;
                }
                if !approximate_planar_edge_bounds_overlap(approximate_points, left, right) {
                    continue;
                }
            } else if !planar_edge_bounds_overlap_exact(decisions, points, left, right)? {
                continue;
            }
            if !planar_segments_properly_cross(decisions, points, left, right)? {
                continue;
            }
            crossing_count += 1;

            let intersection = decisions
                .decide(
                    hyperlimit::proper_segment_intersection_point(
                        &hyperlimit_planar_point(&points[left[0]]),
                        &hyperlimit_planar_point(&points[left[1]]),
                        &hyperlimit_planar_point(&points[right[0]]),
                        &hyperlimit_planar_point(&points[right[1]]),
                        decisions.policy(),
                    ),
                    "proper planar output edge intersection",
                )?
                .ok_or(HypermeshError::OutputPlanarizationFailed {
                    reason: "proper output edge crossing has no intersection point",
                })?;
            let intersection = hypertri::ExactPoint::new(intersection.x, intersection.y);
            if planar_point_index(decisions, points, &intersection)?.is_some() {
                existing_intersection_count += 1;
                continue;
            }

            if interner.is_none() {
                let exact_only = soup
                    .vertices
                    .iter()
                    .all(PointCoordinates::has_exact_rational_coordinates);
                interner = Some(PointInterner::<()>::try_from_unique(
                    &soup.vertices,
                    exact_only,
                )?);
            }
            let lifted = lift_planar_point(&intersection, plane, projection_axis, axes)?;
            let global = interner
                .as_mut()
                .expect("the planar point interner was initialized")
                .intern_owned(decisions, &mut soup.vertices, lifted, None)?;
            points
                .try_reserve(1)
                .map_err(|_| HypermeshError::CapacityOverflow {
                    operation: "planar output intersection points",
                })?;
            point_to_global
                .try_reserve(1)
                .map_err(|_| HypermeshError::CapacityOverflow {
                    operation: "planar output point mapping",
                })?;
            points.push(intersection);
            point_to_global.push(global);
        }
    }

    if diagnostic {
        eprintln!(
            "output planar crossings: {:?}, crossings={}, existing={}, points={}",
            started.elapsed(),
            crossing_count,
            existing_intersection_count,
            points.len()
        );
    }
    let approximate_points = exact_planar_points_f64(points);
    let mut planar_edges = BTreeSet::new();
    let mut split_constraint_count = 0_usize;
    let mut max_constraint_points = 0_usize;
    for &constraint in constraints {
        let mut on_segment = Vec::new();
        for point in 0..points.len() {
            if approximate_points.as_ref().is_some_and(|points| {
                !approximate_planar_point_within_edge_bounds(points, constraint, point)
            }) {
                continue;
            }
            if planar_point_on_segment(decisions, points, constraint, point)? {
                on_segment.push(point);
            }
        }
        split_constraint_count += usize::from(on_segment.len() > 2);
        max_constraint_points = max_constraint_points.max(on_segment.len());
        sort_planar_indices_on_segment(decisions, points, constraint, &mut on_segment)?;
        for pair in on_segment.windows(2) {
            if pair[0] != pair[1] {
                planar_edges.insert(sorted_edge([pair[0], pair[1]]));
            }
        }
    }
    if diagnostic {
        eprintln!(
            "output planar split constraints: {:?}, split={}, max_points={}, edges={}",
            started.elapsed(),
            split_constraint_count,
            max_constraint_points,
            planar_edges.len()
        );
    }
    Ok(planar_edges)
}

fn hyperlimit_planar_point(point: &hypertri::ExactPoint) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x.clone(), point.y.clone())
}

fn planar_point_index(
    decisions: &DecisionContext,
    points: &[hypertri::ExactPoint],
    point: &hypertri::ExactPoint,
) -> HypermeshResult<Option<usize>> {
    for (index, candidate) in points.iter().enumerate() {
        if let ([Some(cx), Some(cy)], [Some(px), Some(py)]) = (
            [
                candidate.x.exact_rational_ref(),
                candidate.y.exact_rational_ref(),
            ],
            [point.x.exact_rational_ref(), point.y.exact_rational_ref()],
        ) {
            if cx == px && cy == py {
                return Ok(Some(index));
            }
            continue;
        }
        if compare_real_decision(decisions, &candidate.x, &point.x)?.is_eq()
            && compare_real_decision(decisions, &candidate.y, &point.y)?.is_eq()
        {
            return Ok(Some(index));
        }
    }
    Ok(None)
}

fn planar_segments_properly_cross(
    decisions: &DecisionContext,
    points: &[hypertri::ExactPoint],
    left: [usize; 2],
    right: [usize; 2],
) -> HypermeshResult<bool> {
    let opposite = |first, second| {
        matches!(
            (first, second),
            (Classification::Negative, Classification::Positive)
                | (Classification::Positive, Classification::Negative)
        )
    };
    let right_from = planar_orientation(
        decisions,
        &points[left[0]],
        &points[left[1]],
        &points[right[0]],
    )?;
    let right_to = planar_orientation(
        decisions,
        &points[left[0]],
        &points[left[1]],
        &points[right[1]],
    )?;
    if !opposite(right_from, right_to) {
        return Ok(false);
    }
    let left_from = planar_orientation(
        decisions,
        &points[right[0]],
        &points[right[1]],
        &points[left[0]],
    )?;
    let left_to = planar_orientation(
        decisions,
        &points[right[0]],
        &points[right[1]],
        &points[left[1]],
    )?;
    Ok(opposite(left_from, left_to))
}

fn planar_edge_bounds_overlap_exact(
    decisions: &DecisionContext,
    points: &[hypertri::ExactPoint],
    left: [usize; 2],
    right: [usize; 2],
) -> HypermeshResult<bool> {
    for axis in 0..2 {
        let left_order = compare_real_decision(
            decisions,
            planar_coordinate(&points[left[0]], axis),
            planar_coordinate(&points[left[1]], axis),
        )?;
        let right_order = compare_real_decision(
            decisions,
            planar_coordinate(&points[right[0]], axis),
            planar_coordinate(&points[right[1]], axis),
        )?;
        let (left_min, left_max) = if left_order.is_gt() {
            (left[1], left[0])
        } else {
            (left[0], left[1])
        };
        let (right_min, right_max) = if right_order.is_gt() {
            (right[1], right[0])
        } else {
            (right[0], right[1])
        };
        if compare_real_decision(
            decisions,
            planar_coordinate(&points[left_max], axis),
            planar_coordinate(&points[right_min], axis),
        )?
        .is_lt()
            || compare_real_decision(
                decisions,
                planar_coordinate(&points[right_max], axis),
                planar_coordinate(&points[left_min], axis),
            )?
            .is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn planar_point_on_segment(
    decisions: &DecisionContext,
    points: &[hypertri::ExactPoint],
    edge: [usize; 2],
    point: usize,
) -> HypermeshResult<bool> {
    if planar_orientation(
        decisions,
        &points[edge[0]],
        &points[edge[1]],
        &points[point],
    )? != Classification::On
    {
        return Ok(false);
    }
    for axis in 0..2 {
        let from = planar_coordinate(&points[edge[0]], axis);
        let to = planar_coordinate(&points[edge[1]], axis);
        let point = planar_coordinate(&points[point], axis);
        let (min, max) = if compare_real_decision(decisions, from, to)?.is_gt() {
            (to, from)
        } else {
            (from, to)
        };
        if compare_real_decision(decisions, point, min)?.is_lt()
            || compare_real_decision(decisions, point, max)?.is_gt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn planar_coordinate(point: &hypertri::ExactPoint, axis: usize) -> &Real {
    if axis == 0 { &point.x } else { &point.y }
}

fn sort_planar_indices_on_segment(
    decisions: &DecisionContext,
    points: &[hypertri::ExactPoint],
    edge: [usize; 2],
    indices: &mut [usize],
) -> HypermeshResult<()> {
    let use_x = !compare_real_decision(decisions, &points[edge[0]].x, &points[edge[1]].x)?.is_eq();
    for index in 1..indices.len() {
        let mut cursor = index;
        while cursor > 0 {
            let ordering = if use_x {
                compare_real_decision(
                    decisions,
                    &points[indices[cursor]].x,
                    &points[indices[cursor - 1]].x,
                )?
            } else {
                compare_real_decision(
                    decisions,
                    &points[indices[cursor]].y,
                    &points[indices[cursor - 1]].y,
                )?
            };
            if !ordering.is_lt() {
                break;
            }
            indices.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    Ok(())
}

fn approximate_planar_edge_min(points: &[[f64; 2]], edge: [usize; 2], axis: usize) -> f64 {
    points[edge[0]][axis].min(points[edge[1]][axis])
}

fn approximate_planar_edge_max(points: &[[f64; 2]], edge: [usize; 2], axis: usize) -> f64 {
    points[edge[0]][axis].max(points[edge[1]][axis])
}

fn approximate_planar_edge_bounds_overlap(
    points: &[[f64; 2]],
    left: [usize; 2],
    right: [usize; 2],
) -> bool {
    (0..2).all(|axis| {
        approximate_planar_edge_max(points, left, axis)
            >= approximate_planar_edge_min(points, right, axis)
            && approximate_planar_edge_max(points, right, axis)
                >= approximate_planar_edge_min(points, left, axis)
    })
}

fn approximate_planar_point_within_edge_bounds(
    points: &[[f64; 2]],
    edge: [usize; 2],
    point: usize,
) -> bool {
    (0..2).all(|axis| {
        points[point][axis] >= approximate_planar_edge_min(points, edge, axis)
            && points[point][axis] <= approximate_planar_edge_max(points, edge, axis)
    })
}

fn bounded_planar_faces(
    decisions: &DecisionContext,
    points: &[hypertri::ExactPoint],
    edges: &BTreeSet<[usize; 2]>,
) -> HypermeshResult<Vec<Vec<usize>>> {
    let mut adjacency = vec![Vec::new(); points.len()];
    for &[from, to] in edges {
        adjacency[from].push(to);
        adjacency[to].push(from);
    }
    for (origin, neighbors) in adjacency.iter_mut().enumerate() {
        for index in 1..neighbors.len() {
            let mut cursor = index;
            while cursor > 0
                && compare_planar_directions(
                    decisions,
                    points,
                    origin,
                    neighbors[cursor],
                    neighbors[cursor - 1],
                )?
                .is_lt()
            {
                neighbors.swap(cursor, cursor - 1);
                cursor -= 1;
            }
        }
    }

    let mut visited = BTreeSet::new();
    let mut faces = Vec::new();
    let mut positions = vec![usize::MAX; points.len()];
    let halfedge_count = edges
        .len()
        .checked_mul(2)
        .ok_or(HypermeshError::CapacityOverflow {
            operation: "planar output half-edges",
        })?;
    for &[a, b] in edges {
        for start in [(a, b), (b, a)] {
            if visited.contains(&start) {
                continue;
            }
            let mut face = Vec::new();
            let mut edge = start;
            for _ in 0..=halfedge_count {
                if edge == start && !face.is_empty() {
                    break;
                }
                if !visited.insert(edge) {
                    return Err(HypermeshError::OutputPlanarizationFailed {
                        reason: "planar face traversal revisited a foreign half-edge",
                    });
                }
                face.push(edge.0);
                let neighbors = &adjacency[edge.1];
                let reverse = neighbors
                    .iter()
                    .position(|&vertex| vertex == edge.0)
                    .ok_or(HypermeshError::OutputPlanarizationFailed {
                        reason: "planar face traversal found a missing reverse edge",
                    })?;
                let next = neighbors[(reverse + neighbors.len() - 1) % neighbors.len()];
                edge = (edge.1, next);
            }
            if edge != start {
                return Err(HypermeshError::OutputPlanarizationFailed {
                    reason: "planar face traversal did not close",
                });
            }
            for simple_face in split_planar_face_walk(&face, &mut positions) {
                if simple_face.len() >= 3
                    && planar_polygon_area_classification(decisions, points, &simple_face)?
                        == Classification::Positive
                {
                    faces.push(simple_face);
                }
            }
        }
    }
    Ok(faces)
}

fn split_planar_face_walk(face: &[usize], positions: &mut [usize]) -> Vec<Vec<usize>> {
    let mut path = Vec::new();
    let mut cycles = Vec::new();
    for &vertex in face.iter().chain(face.first()) {
        let position = positions[vertex];
        if position == usize::MAX {
            positions[vertex] = path.len();
            path.push(vertex);
            continue;
        }

        let cycle = path[position..].to_vec();
        if cycle.len() >= 3 {
            cycles.push(cycle);
        }
        for removed in path.drain((position + 1)..) {
            positions[removed] = usize::MAX;
        }
    }
    for vertex in path {
        positions[vertex] = usize::MAX;
    }
    cycles
}

fn compare_planar_directions(
    decisions: &DecisionContext,
    points: &[hypertri::ExactPoint],
    origin: usize,
    left: usize,
    right: usize,
) -> HypermeshResult<std::cmp::Ordering> {
    let left_half = planar_direction_half(decisions, &points[origin], &points[left])?;
    let right_half = planar_direction_half(decisions, &points[origin], &points[right])?;
    if left_half != right_half {
        return Ok(left_half.cmp(&right_half));
    }
    Ok(
        match planar_orientation(decisions, &points[origin], &points[left], &points[right])? {
            Classification::Positive => std::cmp::Ordering::Less,
            Classification::Negative => std::cmp::Ordering::Greater,
            Classification::On => left.cmp(&right),
        },
    )
}

fn planar_direction_half(
    decisions: &DecisionContext,
    origin: &hypertri::ExactPoint,
    point: &hypertri::ExactPoint,
) -> HypermeshResult<u8> {
    Ok(
        match compare_real_decision(decisions, &point.y, &origin.y)? {
            std::cmp::Ordering::Greater => 0,
            std::cmp::Ordering::Less => 1,
            std::cmp::Ordering::Equal => {
                u8::from(compare_real_decision(decisions, &point.x, &origin.x)?.is_lt())
            }
        },
    )
}

fn planar_polygon_area_classification(
    decisions: &DecisionContext,
    points: &[hypertri::ExactPoint],
    polygon: &[usize],
) -> HypermeshResult<Classification> {
    if points.iter().all(|point| {
        point.x.exact_rational_ref().is_some() && point.y.exact_rational_ref().is_some()
    }) {
        let mut signs = Vec::with_capacity(polygon.len() * 2);
        let mut terms = Vec::with_capacity(polygon.len() * 2);
        for index in 0..polygon.len() {
            let from = &points[polygon[index]];
            let to = &points[polygon[(index + 1) % polygon.len()]];
            signs.extend([true, false]);
            terms.push([
                from.x
                    .exact_rational_ref()
                    .expect("the polygon coordinates are exact rationals"),
                to.y.exact_rational_ref()
                    .expect("the polygon coordinates are exact rationals"),
            ]);
            terms.push([
                from.y
                    .exact_rational_ref()
                    .expect("the polygon coordinates are exact rationals"),
                to.x.exact_rational_ref()
                    .expect("the polygon coordinates are exact rationals"),
            ]);
        }
        return Ok(classification_from_ordering(
            Rational::signed_product_sum2_ordering_slice(&signs, &terms),
        ));
    }

    let mut twice_area = Real::zero();
    for index in 0..polygon.len() {
        let from = &points[polygon[index]];
        let to = &points[polygon[(index + 1) % polygon.len()]];
        twice_area += Real::signed_product_sum([true, false], [[&from.x, &to.y], [&from.y, &to.x]]);
    }
    classify_real(decisions, &twice_area)
}

fn triangulate_planar_face(
    decisions: &DecisionContext,
    points: &[hypertri::ExactPoint],
    face: &[usize],
) -> HypermeshResult<Vec<[usize; 3]>> {
    if let &[a, b, c] = face {
        return Ok(vec![[a, b, c]]);
    }
    let vertices = face
        .iter()
        .map(|&vertex| points[vertex].clone())
        .collect::<Vec<_>>();
    let context = hypertri::TriangulationContext::new(decisions.policy());
    let outcome =
        hypertri::earcut(&context, &vertices, &[]).map_err(map_planar_triangulation_error)?;
    decisions.absorb(match outcome.certainty {
        hypertri::TriangulationCertainty::Certified => MeshCertainty::Certified,
        hypertri::TriangulationCertainty::Approximate512Consumed => {
            MeshCertainty::Approximate512Consumed
        }
    });
    if !outcome.value.len().is_multiple_of(3) {
        return Err(HypermeshError::OutputPlanarizationFailed {
            reason: "planar face triangulation returned an incomplete index triple",
        });
    }
    outcome
        .value
        .chunks_exact(3)
        .map(|triangle| {
            let [a, b, c] = *triangle else {
                unreachable!("chunks_exact returned a non-triple")
            };
            let [Some(a), Some(b), Some(c)] = [face.get(a), face.get(b), face.get(c)] else {
                return Err(HypermeshError::OutputPlanarizationFailed {
                    reason: "planar face triangulation index is out of bounds",
                });
            };
            Ok([*a, *b, *c])
        })
        .collect()
}

fn projection_axes(projection_axis: usize) -> Option<[usize; 2]> {
    match projection_axis {
        0 => Some([1, 2]),
        1 => Some([0, 2]),
        2 => Some([0, 1]),
        _ => None,
    }
}

fn output_plane_projection_axis(
    decisions: &DecisionContext,
    plane: &Plane,
) -> HypermeshResult<usize> {
    let normal = [&plane.normal.x, &plane.normal.y, &plane.normal.z];
    if let Some(axis) = normal.iter().position(|component| {
        component
            .exact_rational_ref()
            .is_some_and(|value| !value.is_zero())
    }) {
        return Ok(axis);
    }
    for (axis, component) in normal.into_iter().enumerate() {
        if classify_real(decisions, component)? != Classification::On {
            return Ok(axis);
        }
    }
    Err(HypermeshError::OutputPlanarizationFailed {
        reason: "crossing repair plane has a zero normal",
    })
}

fn proper_output_segment_intersection_point(
    decisions: &DecisionContext,
    vertices: &[OutputVertex],
    left: [usize; 2],
    right: [usize; 2],
    projection_axis: usize,
) -> HypermeshResult<OutputVertex> {
    let [u_axis, v_axis] =
        projection_axes(projection_axis).ok_or(HypermeshError::OutputPlanarizationFailed {
            reason: "crossing projection axis is invalid",
        })?;
    let point = |index| {
        hyperlimit::Point2::new(
            vertex_axis(&vertices[index], u_axis).clone(),
            vertex_axis(&vertices[index], v_axis).clone(),
        )
    };
    let intersection = decisions
        .decide(
            hyperlimit::proper_segment_intersection_point(
                &point(left[0]),
                &point(left[1]),
                &point(right[0]),
                &point(right[1]),
                decisions.policy(),
            ),
            "proper output edge intersection",
        )?
        .ok_or(HypermeshError::OutputPlanarizationFailed {
            reason: "proper output edge crossing has no intersection point",
        })?;
    let plane = Plane::from_points(
        &output_vertex_point3(&vertices[left[0]]),
        &output_vertex_point3(&vertices[left[1]]),
        &output_vertex_point3(&vertices[right[0]]),
    );
    let intersection = lift_planar_point(
        &hypertri::ExactPoint::new(intersection.x, intersection.y),
        &plane,
        projection_axis,
        [u_axis, v_axis],
    )?;
    if !point_on_segment_exact(
        decisions,
        &intersection,
        &vertices[left[0]],
        &vertices[left[1]],
    )? || !point_on_segment_exact(
        decisions,
        &intersection,
        &vertices[right[0]],
        &vertices[right[1]],
    )? {
        return Err(HypermeshError::OutputPlanarizationFailed {
            reason: "constructed crossing point is not on both output edges",
        });
    }
    Ok(intersection)
}

fn output_vertex_point3(vertex: &OutputVertex) -> Point3 {
    Point3::new(vertex.x.clone(), vertex.y.clone(), vertex.z.clone())
}

fn output_triangle_on_plane(
    decisions: &DecisionContext,
    plane: &Plane,
    vertices: &[OutputVertex],
    triangle: [usize; 3],
) -> HypermeshResult<bool> {
    triangle.iter().try_fold(true, |on_plane, &vertex| {
        Ok(on_plane && output_vertex_on_plane(decisions, plane, &vertices[vertex])?)
    })
}

fn output_vertex_on_plane(
    decisions: &DecisionContext,
    plane: &Plane,
    point: &OutputVertex,
) -> HypermeshResult<bool> {
    if let [
        Some(nx),
        Some(ny),
        Some(nz),
        Some(offset),
        Some(x),
        Some(y),
        Some(z),
    ] = [
        plane.normal.x.exact_rational_ref(),
        plane.normal.y.exact_rational_ref(),
        plane.normal.z.exact_rational_ref(),
        plane.offset.exact_rational_ref(),
        point.x.exact_rational_ref(),
        point.y.exact_rational_ref(),
        point.z.exact_rational_ref(),
    ] {
        let one = Rational::one();
        return Ok(Rational::signed_product_sum_ordering(
            [true; 4],
            [[nx, x], [ny, y], [nz, z], [offset, &one]],
        )
        .is_eq());
    }
    let one = Real::one();
    Ok(classify_real(
        decisions,
        &Real::signed_product_sum(
            [true; 4],
            [
                [&plane.normal.x, &point.x],
                [&plane.normal.y, &point.y],
                [&plane.normal.z, &point.z],
                [&plane.offset, &one],
            ],
        ),
    )? == Classification::On)
}

fn planar_orientation(
    decisions: &DecisionContext,
    from: &hypertri::ExactPoint,
    to: &hypertri::ExactPoint,
    point: &hypertri::ExactPoint,
) -> HypermeshResult<Classification> {
    orientation2(
        decisions, &from.x, &from.y, &to.x, &to.y, &point.x, &point.y, None, None,
    )
}

fn exact_planar_points_f64(points: &[hypertri::ExactPoint]) -> Option<Vec<[f64; 2]>> {
    points
        .iter()
        .map(|point| {
            if point.x.exact_rational_ref().is_none() || point.y.exact_rational_ref().is_none() {
                return None;
            }
            let [Some(x), Some(y)] = [point.x.to_f64_lossy(), point.y.to_f64_lossy()] else {
                return None;
            };
            (x.is_finite() && y.is_finite()).then_some([x, y])
        })
        .collect()
}

fn approximate_planar_triangle_bounds(points: &[[f64; 2]], triangle: [usize; 3]) -> [f64; 4] {
    let [a, b, c] = triangle.map(|vertex| points[vertex]);
    [
        a[0].min(b[0]).min(c[0]),
        a[1].min(b[1]).min(c[1]),
        a[0].max(b[0]).max(c[0]),
        a[1].max(b[1]).max(c[1]),
    ]
}

fn approximate_point_within_planar_bounds(point: [f64; 2], bounds: [f64; 4]) -> bool {
    point[0] >= bounds[0] && point[1] >= bounds[1] && point[0] <= bounds[2] && point[1] <= bounds[3]
}

fn planar_triangle_centroid(
    points: &[hypertri::ExactPoint],
    triangle: [usize; 3],
) -> HypermeshResult<hypertri::ExactPoint> {
    let [a, b, c] = triangle.map(|vertex| &points[vertex]);
    Ok(hypertri::ExactPoint::new(
        mean_real3([&a.x, &b.x, &c.x])?,
        mean_real3([&a.y, &b.y, &c.y])?,
    ))
}

fn mean_real3(values: [&Real; 3]) -> HypermeshResult<Real> {
    if let [Some(a), Some(b), Some(c)] = values.map(Real::exact_rational_ref) {
        return Ok(Real::from(Rational::mean3_refs([a, b, c])));
    }
    (Real::sum_refs(values) / Real::from(3_u8)).map_err(|_| {
        HypermeshError::OutputPlanarizationFailed {
            reason: "could not construct a planar triangle interior sample",
        }
    })
}

fn planar_triangle_contains_point(
    decisions: &DecisionContext,
    points: &[hypertri::ExactPoint],
    triangle: [usize; 3],
    orientation: i8,
    point: &hypertri::ExactPoint,
) -> HypermeshResult<bool> {
    for edge in triangle_edges(triangle) {
        let side = planar_orientation(decisions, &points[edge[0]], &points[edge[1]], point)?;
        if (orientation > 0 && side == Classification::Negative)
            || (orientation < 0 && side == Classification::Positive)
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn lift_planar_point(
    point: &hypertri::ExactPoint,
    plane: &Plane,
    projection_axis: usize,
    [u_axis, v_axis]: [usize; 2],
) -> HypermeshResult<OutputVertex> {
    let normal = [&plane.normal.x, &plane.normal.y, &plane.normal.z];
    let one = Real::one();
    let numerator = Real::signed_product_sum(
        [false; 3],
        [
            [normal[u_axis], &point.x],
            [normal[v_axis], &point.y],
            [&plane.offset, &one],
        ],
    );
    let dropped = (numerator / normal[projection_axis]).map_err(|_| {
        HypermeshError::OutputPlanarizationFailed {
            reason: "crossing plane has a zero projection coefficient",
        }
    })?;
    Ok(match projection_axis {
        0 => OutputVertex {
            x: dropped,
            y: point.x.clone(),
            z: point.y.clone(),
        },
        1 => OutputVertex {
            x: point.x.clone(),
            y: dropped,
            z: point.y.clone(),
        },
        2 => OutputVertex {
            x: point.x.clone(),
            y: point.y.clone(),
            z: dropped,
        },
        _ => {
            return Err(HypermeshError::OutputPlanarizationFailed {
                reason: "crossing projection axis is invalid",
            });
        }
    })
}

fn map_planar_triangulation_error(error: hypertri::Error) -> HypermeshError {
    match error {
        hypertri::Error::PredicateUndecided { predicate } => {
            HypermeshError::PredicateUndecided { predicate }
        }
        hypertri::Error::InvalidInput { reason } => {
            HypermeshError::OutputPlanarizationFailed { reason }
        }
        hypertri::Error::NoEarFound => HypermeshError::OutputPlanarizationFailed {
            reason: "no planar triangulation ear was found",
        },
        hypertri::Error::UnsupportedFeature { feature } => {
            HypermeshError::OutputPlanarizationFailed { reason: feature }
        }
    }
}

fn approximate_edge_min(
    vertices: &[ApproximateOutputVertex],
    edge: [usize; 2],
    axis: usize,
) -> f64 {
    vertices[edge[0]][axis][0].min(vertices[edge[1]][axis][0])
}

fn approximate_edge_max(
    vertices: &[ApproximateOutputVertex],
    edge: [usize; 2],
    axis: usize,
) -> f64 {
    vertices[edge[0]][axis][1].max(vertices[edge[1]][axis][1])
}

fn approximate_edge_bounds(
    vertices: &[ApproximateOutputVertex],
    edge: [usize; 2],
) -> ApproximateEdgeBounds {
    [0, 1, 2].map(|axis| {
        [
            approximate_edge_min(vertices, edge, axis),
            approximate_edge_max(vertices, edge, axis),
        ]
    })
}

fn approximate_bounds_overlap(left: &ApproximateEdgeBounds, right: &ApproximateEdgeBounds) -> bool {
    (0..3).all(|axis| left[axis][1] >= right[axis][0] && right[axis][1] >= left[axis][0])
}

fn approximate_edge_overlaps_bounds(
    vertices: &[ApproximateOutputVertex],
    edge: [usize; 2],
    bounds: &ApproximateEdgeBounds,
) -> bool {
    (0..3).all(|axis| {
        bounds[axis][1] >= approximate_edge_min(vertices, edge, axis)
            && approximate_edge_max(vertices, edge, axis) >= bounds[axis][0]
    })
}

// Every axis is exactness-equivalent because the sweep rejects a pair only
// after outward intervals prove separation. Sampling changes work order and
// candidate volume only; all survivors still take the exact predicate path.
fn least_sampled_interval_overlap_axis(
    vertices: &[ApproximateOutputVertex],
    edges: &[ExactEdgeBounds],
) -> usize {
    const MAX_SAMPLED_EDGES: usize = 32;

    let sample_count = edges.len().min(MAX_SAMPLED_EDGES);
    let mut overlap_counts = [0_u16; 3];
    for left_sample in 0..sample_count {
        let left = edges[left_sample.saturating_mul(edges.len()) / sample_count].edge;
        for right_sample in (left_sample + 1)..sample_count {
            let right = edges[right_sample.saturating_mul(edges.len()) / sample_count].edge;
            for (axis, overlap_count) in overlap_counts.iter_mut().enumerate() {
                if approximate_edge_max(vertices, left, axis)
                    >= approximate_edge_min(vertices, right, axis)
                    && approximate_edge_max(vertices, right, axis)
                        >= approximate_edge_min(vertices, left, axis)
                {
                    *overlap_count += 1;
                }
            }
        }
    }

    (1..3).fold(0, |best, axis| {
        if overlap_counts[axis] < overlap_counts[best] {
            axis
        } else {
            best
        }
    })
}

fn approximate_crossing_sweep_axis(
    vertices: &[ApproximateOutputVertex],
    edges: &[ExactEdgeBounds],
) -> usize {
    if edges.len() < MIN_ADAPTIVE_CROSSING_SWEEP_EDGES {
        0
    } else {
        least_sampled_interval_overlap_axis(vertices, edges)
    }
}

fn approximate_projection_axis(
    vertices: &[ApproximateOutputVertex],
    left: [usize; 2],
    right: [usize; 2],
) -> Option<usize> {
    let left = [0, 1, 2].map(|axis| vertices[left[1]][axis][0] - vertices[left[0]][axis][0]);
    let right = [0, 1, 2].map(|axis| vertices[right[1]][axis][0] - vertices[right[0]][axis][0]);
    let normal = [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ];
    (0..3)
        .filter(|&axis| normal[axis].is_finite() && normal[axis] != 0.0)
        .max_by(|&left, &right| normal[left].abs().total_cmp(&normal[right].abs()))
}

struct ExactEdgeBounds {
    edge: [usize; 2],
    min: [usize; 3],
    max: [usize; 3],
}

fn exact_edge_bounds(
    decisions: &DecisionContext,
    edge: [usize; 2],
    vertices: &[OutputVertex],
    approximate_vertices: Option<&[ApproximateOutputVertex]>,
) -> HypermeshResult<ExactEdgeBounds> {
    let mut min = [edge[0]; 3];
    let mut max = [edge[1]; 3];
    for axis in 0..3 {
        let approximate_order = approximate_vertices.and_then(|vertices| {
            let left = vertices[edge[0]][axis];
            let right = vertices[edge[1]][axis];
            if left[1] < right[0] {
                Some(std::cmp::Ordering::Less)
            } else if right[1] < left[0] {
                Some(std::cmp::Ordering::Greater)
            } else {
                None
            }
        });
        let ordering = match approximate_order {
            None => compare_real_decision(
                decisions,
                vertex_axis(&vertices[edge[0]], axis),
                vertex_axis(&vertices[edge[1]], axis),
            )?,
            Some(ordering) => ordering,
        };
        if ordering.is_gt() {
            min[axis] = edge[1];
            max[axis] = edge[0];
        }
    }
    Ok(ExactEdgeBounds { edge, min, max })
}

fn edge_bounds_overlap_exact(
    decisions: &DecisionContext,
    left: &ExactEdgeBounds,
    right: &ExactEdgeBounds,
    vertices: &[OutputVertex],
    first_axis: usize,
) -> HypermeshResult<bool> {
    for axis in first_axis..3 {
        if compare_real_decision(
            decisions,
            vertex_axis(&vertices[left.max[axis]], axis),
            vertex_axis(&vertices[right.min[axis]], axis),
        )?
        .is_lt()
            || compare_real_decision(
                decisions,
                vertex_axis(&vertices[right.max[axis]], axis),
                vertex_axis(&vertices[left.min[axis]], axis),
            )?
            .is_lt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn proper_segment_intersection_after_bounds_overlap(
    decisions: &DecisionContext,
    a: &OutputVertex,
    b: &OutputVertex,
    c: &OutputVertex,
    d: &OutputVertex,
    preferred_projection_axis: Option<usize>,
    rational_queries: Option<&[RationalPoint3Query; 4]>,
) -> HypermeshResult<Option<usize>> {
    let mut projections = [[1, 2], [0, 2], [0, 1]];
    if let Some(axis) = preferred_projection_axis {
        projections.swap(0, axis);
    }
    let mut projection = None;
    let mut saw_unknown = false;
    for [u_axis, v_axis] in projections {
        match projected_segment_crossing(decisions, a, b, c, d, [u_axis, v_axis], rational_queries)
        {
            Ok(Some(true)) => {
                projection = Some((u_axis, v_axis));
                break;
            }
            Ok(Some(false)) => return Ok(None),
            Ok(None) => {}
            Err(
                HypermeshError::PredicateUndecided { .. } | HypermeshError::UnknownClassification,
            ) => saw_unknown = true,
            Err(error) => return Err(error),
        }
    }
    let (u_axis, v_axis) = match projection {
        Some(projection) => projection,
        None if saw_unknown => {
            return Err(HypermeshError::PredicateUndecided {
                predicate: "proper projected edge crossing",
            });
        }
        None => return Ok(None),
    };

    let ab = sub_vertex(b, a);
    let cd = sub_vertex(d, c);
    let ac = sub_vertex(c, a);
    if !vectors_are_coplanar(decisions, &ab, &cd, &ac)? {
        return Ok(None);
    }
    Ok(Some(3 - u_axis - v_axis))
}

fn vectors_are_coplanar(
    decisions: &DecisionContext,
    left: &[Real; 3],
    right: &[Real; 3],
    third: &[Real; 3],
) -> HypermeshResult<bool> {
    let exact = [left, right, third].map(|vector| vector.each_ref().map(Real::exact_rational_ref));
    if let (
        [Some(lx), Some(ly), Some(lz)],
        [Some(rx), Some(ry), Some(rz)],
        [Some(tx), Some(ty), Some(tz)],
    ) = (exact[0], exact[1], exact[2])
    {
        return Ok(Rational::signed_product_sum_ordering(
            [true, true, true, false, false, false],
            [
                [lx, ry, tz],
                [ly, rz, tx],
                [lz, rx, ty],
                [lz, ry, tx],
                [ly, rx, tz],
                [lx, rz, ty],
            ],
        )
        .is_eq());
    }

    Ok(classify_real(
        decisions,
        &Real::signed_product_sum(
            [true, true, true, false, false, false],
            [
                [&left[0], &right[1], &third[2]],
                [&left[1], &right[2], &third[0]],
                [&left[2], &right[0], &third[1]],
                [&left[2], &right[1], &third[0]],
                [&left[1], &right[0], &third[2]],
                [&left[0], &right[2], &third[1]],
            ],
        ),
    )? == Classification::On)
}

fn projected_segment_crossing(
    decisions: &DecisionContext,
    a: &OutputVertex,
    b: &OutputVertex,
    c: &OutputVertex,
    d: &OutputVertex,
    axes: [usize; 2],
    rational_queries: Option<&[RationalPoint3Query; 4]>,
) -> HypermeshResult<Option<bool>> {
    let opposite = |left, right| {
        matches!(
            (left, right),
            (Classification::Negative, Classification::Positive)
                | (Classification::Positive, Classification::Negative)
        )
    };
    let same_side = |left, right| {
        matches!(
            (left, right),
            (Classification::Negative, Classification::Negative)
                | (Classification::Positive, Classification::Positive)
        )
    };

    let ab_filter = rational_queries
        .and_then(|queries| RationalLine2Filter::from_point3(&queries[0], &queries[1], axes))
        .or_else(|| projected_rational_line_filter(a, b, axes));
    let c_side = projected_orientation(
        decisions,
        a,
        b,
        c,
        axes,
        ab_filter.as_ref(),
        rational_queries.map(|queries| &queries[2]),
    )?;
    let d_side = projected_orientation(
        decisions,
        a,
        b,
        d,
        axes,
        ab_filter.as_ref(),
        rational_queries.map(|queries| &queries[3]),
    )?;
    if same_side(c_side, d_side) {
        return Ok(Some(false));
    }
    let cd_filter = rational_queries
        .and_then(|queries| RationalLine2Filter::from_point3(&queries[2], &queries[3], axes))
        .or_else(|| projected_rational_line_filter(c, d, axes));
    let a_side = projected_orientation(
        decisions,
        c,
        d,
        a,
        axes,
        cd_filter.as_ref(),
        rational_queries.map(|queries| &queries[0]),
    )?;
    let b_side = projected_orientation(
        decisions,
        c,
        d,
        b,
        axes,
        cd_filter.as_ref(),
        rational_queries.map(|queries| &queries[1]),
    )?;
    if same_side(a_side, b_side) {
        return Ok(Some(false));
    }
    if opposite(c_side, d_side) && opposite(a_side, b_side) {
        Ok(Some(true))
    } else {
        Ok(None)
    }
}

#[inline]
fn projected_rational_line_filter(
    from: &OutputVertex,
    to: &OutputVertex,
    [u_axis, v_axis]: [usize; 2],
) -> Option<RationalLine2Filter> {
    RationalLine2Filter::from_rationals(
        [
            vertex_axis(from, u_axis).exact_rational_ref()?,
            vertex_axis(from, v_axis).exact_rational_ref()?,
        ],
        [
            vertex_axis(to, u_axis).exact_rational_ref()?,
            vertex_axis(to, v_axis).exact_rational_ref()?,
        ],
    )
}

fn projected_orientation(
    decisions: &DecisionContext,
    from: &OutputVertex,
    to: &OutputVertex,
    point: &OutputVertex,
    [u_axis, v_axis]: [usize; 2],
    filter: Option<&RationalLine2Filter>,
    query: Option<&RationalPoint3Query>,
) -> HypermeshResult<Classification> {
    let [from_u, from_v, to_u, to_v, point_u, point_v] = [
        vertex_axis(from, u_axis),
        vertex_axis(from, v_axis),
        vertex_axis(to, u_axis),
        vertex_axis(to, v_axis),
        vertex_axis(point, u_axis),
        vertex_axis(point, v_axis),
    ];
    orientation2(
        decisions,
        from_u,
        from_v,
        to_u,
        to_v,
        point_u,
        point_v,
        filter,
        query.map(|query| filter.and_then(|filter| filter.sign_point3(query, [u_axis, v_axis]))),
    )
}

fn orientation2(
    decisions: &DecisionContext,
    from_u: &Real,
    from_v: &Real,
    to_u: &Real,
    to_v: &Real,
    point_u: &Real,
    point_v: &Real,
    filter: Option<&RationalLine2Filter>,
    precomputed_sign: Option<Option<RealSign>>,
) -> HypermeshResult<Classification> {
    if let [
        Some(from_u),
        Some(from_v),
        Some(to_u),
        Some(to_v),
        Some(point_u),
        Some(point_v),
    ] = [
        from_u.exact_rational_ref(),
        from_v.exact_rational_ref(),
        to_u.exact_rational_ref(),
        to_v.exact_rational_ref(),
        point_u.exact_rational_ref(),
        point_v.exact_rational_ref(),
    ] {
        let sign = precomputed_sign.unwrap_or_else(|| match filter {
            Some(filter) => filter.sign_rationals([point_u, point_v]),
            None => Real::certified_rational_line2_sign(
                [from_u, from_v],
                [to_u, to_v],
                [point_u, point_v],
            ),
        });
        if let Some(sign) = sign {
            return Ok(match sign {
                RealSign::Negative => Classification::Negative,
                RealSign::Zero => Classification::On,
                RealSign::Positive => Classification::Positive,
            });
        }
        return Ok(classification_from_ordering(
            Rational::signed_product_sum_ordering(
                [true, true, true, false, false, false],
                [
                    [to_u, point_v],
                    [from_u, to_v],
                    [from_v, point_u],
                    [from_v, to_u],
                    [to_v, point_u],
                    [from_u, point_v],
                ],
            ),
        ));
    }

    let direction_u = to_u - from_u;
    let direction_v = to_v - from_v;
    let point_u = point_u - from_u;
    let point_v = point_v - from_v;
    classify_real(
        decisions,
        &Real::signed_product_sum(
            [true, false],
            [[&direction_u, &point_v], [&direction_v, &point_u]],
        ),
    )
}

fn classification_from_ordering(ordering: std::cmp::Ordering) -> Classification {
    match ordering {
        std::cmp::Ordering::Less => Classification::Negative,
        std::cmp::Ordering::Equal => Classification::On,
        std::cmp::Ordering::Greater => Classification::Positive,
    }
}

fn triangle_edges(triangle: [usize; 3]) -> [[usize; 2]; 3] {
    [
        [triangle[0], triangle[1]],
        [triangle[1], triangle[2]],
        [triangle[2], triangle[0]],
    ]
}

fn sorted_edge(edge: [usize; 2]) -> [usize; 2] {
    if edge[0] <= edge[1] {
        edge
    } else {
        [edge[1], edge[0]]
    }
}

fn point_on_segment_exact(
    decisions: &DecisionContext,
    point: &OutputVertex,
    start: &OutputVertex,
    end: &OutputVertex,
) -> HypermeshResult<bool> {
    if !point_within_segment_bounds_exact(decisions, point, start, end)? {
        return Ok(false);
    }
    point_collinear_with_segment_exact(decisions, point, start, end)
}

fn point_collinear_with_segment_exact(
    decisions: &DecisionContext,
    point: &OutputVertex,
    start: &OutputVertex,
    end: &OutputVertex,
) -> HypermeshResult<bool> {
    let ab = sub_vertex(end, start);
    let av = sub_vertex(point, start);
    let cross = cross_arrays(&ab, &av);
    for component in &cross {
        if classify_real(decisions, component)? != Classification::On {
            return Ok(false);
        }
    }

    Ok(!output_vertices_equal(decisions, point, start)?
        && !output_vertices_equal(decisions, point, end)?)
}

fn point_within_edge_bounds_except_axis_exact(
    decisions: &DecisionContext,
    point: &OutputVertex,
    bounds: &ExactEdgeBounds,
    vertices: &[OutputVertex],
    excluded_axis: usize,
) -> HypermeshResult<bool> {
    for axis in 0..3 {
        if axis == excluded_axis {
            continue;
        }
        let coordinate = vertex_axis(point, axis);
        if compare_real_decision(
            decisions,
            coordinate,
            vertex_axis(&vertices[bounds.min[axis]], axis),
        )?
        .is_lt()
            || compare_real_decision(
                decisions,
                coordinate,
                vertex_axis(&vertices[bounds.max[axis]], axis),
            )?
            .is_gt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn point_within_segment_bounds_exact(
    decisions: &DecisionContext,
    point: &OutputVertex,
    start: &OutputVertex,
    end: &OutputVertex,
) -> HypermeshResult<bool> {
    for axis in 0..3 {
        let p = vertex_axis(point, axis);
        let a = vertex_axis(start, axis);
        let b = vertex_axis(end, axis);
        let (min, max) = ordered_reals(decisions, a, b)?;
        if compare_real_decision(decisions, p, min)?.is_lt()
            || compare_real_decision(decisions, p, max)?.is_gt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn ordered_reals<'a>(
    decisions: &DecisionContext,
    left: &'a Real,
    right: &'a Real,
) -> HypermeshResult<(&'a Real, &'a Real)> {
    if compare_real_decision(decisions, left, right)?.is_le() {
        Ok((left, right))
    } else {
        Ok((right, left))
    }
}

fn sort_along_segment(
    decisions: &DecisionContext,
    indices: &[usize],
    start: usize,
    end: usize,
    vertices: &[OutputVertex],
) -> HypermeshResult<Vec<usize>> {
    let axis = dominant_segment_axis(decisions, &vertices[start], &vertices[end])?;
    sort_along_segment_on_axis(decisions, indices, start, end, vertices, axis)
}

fn sort_along_segment_on_axis(
    decisions: &DecisionContext,
    indices: &[usize],
    start: usize,
    end: usize,
    vertices: &[OutputVertex],
    axis: usize,
) -> HypermeshResult<Vec<usize>> {
    let ascending = compare_real_decision(
        decisions,
        vertex_axis(&vertices[start], axis),
        vertex_axis(&vertices[end], axis),
    )?
    .is_lt();
    let mut sorted: Vec<usize> = Vec::new();

    for index in indices {
        let mut insert_at = sorted.len();
        for (position, existing) in sorted.iter().enumerate() {
            let order = compare_real_decision(
                decisions,
                vertex_axis(&vertices[*index], axis),
                vertex_axis(&vertices[*existing], axis),
            )
            .inspect_err(|error| {
                if cfg!(debug_assertions) {
                    eprintln!(
                        "[DEBUG] segment ordering failed: axis={axis} left={} right={} left-f64={:?} right-f64={:?} left-point={:?} right-point={:?}: {error}",
                        *index,
                        *existing,
                        vertex_axis(&vertices[*index], axis).to_f64_lossy(),
                        vertex_axis(&vertices[*existing], axis).to_f64_lossy(),
                        [
                            vertices[*index].x.to_f64_lossy(),
                            vertices[*index].y.to_f64_lossy(),
                            vertices[*index].z.to_f64_lossy(),
                        ],
                        [
                            vertices[*existing].x.to_f64_lossy(),
                            vertices[*existing].y.to_f64_lossy(),
                            vertices[*existing].z.to_f64_lossy(),
                        ],
                    );
                }
            })?;
            if (ascending && order.is_lt()) || (!ascending && order.is_gt()) {
                insert_at = position;
                break;
            }
        }
        sorted.insert(insert_at, *index);
    }

    Ok(sorted)
}

fn inexpensive_nonzero_segment_axis(
    decisions: &DecisionContext,
    start: &OutputVertex,
    end: &OutputVertex,
) -> HypermeshResult<usize> {
    let approximate = (|| {
        Some([
            (end.x.to_f64_lossy()? - start.x.to_f64_lossy()?).abs(),
            (end.y.to_f64_lossy()? - start.y.to_f64_lossy()?).abs(),
            (end.z.to_f64_lossy()? - start.z.to_f64_lossy()?).abs(),
        ])
    })();
    if let Some(approximate) = approximate {
        let mut best = None;
        for axis in 0..3 {
            let delta = approximate[axis];
            if delta.is_finite()
                && delta != 0.0
                && best.is_none_or(|best_axis| delta >= approximate[best_axis])
            {
                best = Some(axis);
            }
        }
        if let Some(axis) = best
            && compare_real_decision(decisions, vertex_axis(start, axis), vertex_axis(end, axis))?
                .is_ne()
        {
            return Ok(axis);
        }
    }
    dominant_segment_axis(decisions, start, end)
}

fn dominant_segment_axis(
    decisions: &DecisionContext,
    start: &OutputVertex,
    end: &OutputVertex,
) -> HypermeshResult<usize> {
    let delta = sub_vertex(end, start);
    let abs = [
        delta[0].clone().abs(),
        delta[1].clone().abs(),
        delta[2].clone().abs(),
    ];
    let mut best = 0;
    for axis in 1..3 {
        if compare_real_decision(decisions, &abs[axis], &abs[best])?.is_gt() {
            best = axis;
        }
    }
    Ok(best)
}

fn certify_positive_signed_volume(
    decisions: &DecisionContext,
    soup: &BooleanMesh,
) -> HypermeshResult<()> {
    let volume = signed_volume_numerator(soup);
    if classify_real(decisions, &volume)? != Classification::Positive {
        return Err(HypermeshError::UnknownClassification);
    }
    Ok(())
}

fn signed_volume_numerator(soup: &BooleanMesh) -> Real {
    let mut volume = Real::zero();
    for triangle in &soup.triangles {
        let v0 = &soup.vertices[triangle[0]];
        let v1 = &soup.vertices[triangle[1]];
        let v2 = &soup.vertices[triangle[2]];
        let term = Real::signed_product_sum(
            [true, true, true, false, false, false],
            [
                [&v0.x, &v1.y, &v2.z],
                [&v0.y, &v1.z, &v2.x],
                [&v0.z, &v1.x, &v2.y],
                [&v0.x, &v1.z, &v2.y],
                [&v0.y, &v1.x, &v2.z],
                [&v0.z, &v1.y, &v2.x],
            ],
        );
        volume += term;
    }
    volume
}

fn sub_vertex(left: &OutputVertex, right: &OutputVertex) -> [Real; 3] {
    [&left.x - &right.x, &left.y - &right.y, &left.z - &right.z]
}

fn cross_arrays(left: &[Real; 3], right: &[Real; 3]) -> [Real; 3] {
    [
        Real::signed_product_sum(
            [true, false],
            [[&left[1], &right[2]], [&left[2], &right[1]]],
        ),
        Real::signed_product_sum(
            [true, false],
            [[&left[2], &right[0]], [&left[0], &right[2]]],
        ),
        Real::signed_product_sum(
            [true, false],
            [[&left[0], &right[1]], [&left[1], &right[0]]],
        ),
    ]
}

fn vertex_axis(vertex: &OutputVertex, axis: usize) -> &Real {
    match axis {
        0 => &vertex.x,
        1 => &vertex.y,
        2 => &vertex.z,
        _ => panic!("axis must be 0, 1, or 2"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Aabb;
    use crate::test_support::approximate_convex_triangle;
    use crate::winding::WindingPair;
    use hyperlattice::Point3;

    fn r(value: i32) -> Real {
        value.into()
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    fn ov(x: i32, y: i32, z: i32) -> OutputVertex {
        OutputVertex {
            x: r(x),
            y: r(y),
            z: r(z),
        }
    }

    fn ovx(x: Real, y: i32, z: i32) -> OutputVertex {
        OutputVertex {
            x,
            y: r(y),
            z: r(z),
        }
    }

    fn for_each_policy(mut test: impl FnMut(&DecisionContext)) {
        for policy in [
            crate::PredicatePolicy::STRICT,
            crate::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            test(&DecisionContext::new(&context));
        }
    }

    fn op(vertices: Vec<OutputVertex>) -> OutputPolygon {
        OutputPolygon {
            vertices,
            source_mesh: 0,
            source_polygon: 0,
        }
    }

    fn positive_tetra_soup() -> BooleanMesh {
        BooleanMesh {
            vertices: vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 1, 0), ov(0, 0, 1)],
            triangles: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
            sources: vec![TriangleSource::default(); 4],
        }
    }

    #[test]
    fn large_axis_orders_remain_exactly_monotone() {
        let vertices = (0..64)
            .rev()
            .map(|index| ov(index, 63 - index, index % 7))
            .collect::<Vec<_>>();
        let orders =
            sorted_vertex_indices_by_axis(&crate::test_support::approximate_decisions(), &vertices)
                .unwrap();

        for (axis, order) in orders.iter().enumerate() {
            assert!(order.windows(2).all(|pair| {
                crate::predicate::compare_real_decision(
                    &crate::test_support::approximate_decisions(),
                    vertex_axis(&vertices[pair[0]], axis),
                    vertex_axis(&vertices[pair[1]], axis),
                )
                .is_ok_and(|ordering| ordering.is_le())
            }));
        }
    }

    #[test]
    fn plane_profile_fingerprint_collisions_remain_exactly_disambiguated() {
        let plane = |coefficient: &str| {
            Plane::from_coefficients(
                coefficient.parse::<Real>().unwrap(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
            )
        };
        let first = plane("9007199254740992");
        let equal = plane("9007199254740992");
        let rounded_collision = plane("9007199254740993");
        assert_eq!(
            plane_f64_fingerprint(&first),
            plane_f64_fingerprint(&rounded_collision),
        );

        let mut interner = PlaneProfileInterner::new();
        let first_id = interner.plane_id(&first);
        assert_eq!(interner.plane_id(&equal), first_id);
        assert_ne!(interner.plane_id(&rounded_collision), first_id);
    }

    #[test]
    fn exact_corner_boundary_appends_a_convex_fan() {
        let polygon = approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let vertices = vec![ov(0, 0, 0), ov(1, 0, 0), ov(1, 1, 0), ov(0, 1, 0)];
        let mut triangles = Vec::new();

        assert_eq!(
            append_exact_corner_boundary_triangles(
                &crate::test_support::approximate_decisions(),
                &polygon,
                &[0, 1, 2, 3],
                &[0, 1, 2, 3],
                &vertices,
                &mut triangles,
            )
            .unwrap(),
            Some(())
        );
        assert_eq!(triangles, vec![[0, 1, 2], [0, 2, 3]]);
    }

    #[test]
    fn exact_corner_boundary_rejects_incomplete_collinear_fan() {
        let polygon = approximate_convex_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 1, 0), 0, 0);
        let vertices = vec![ov(0, 0, 0), ov(1, 0, 0), ov(2, 0, 0), ov(0, 1, 0)];
        let mut triangles = Vec::new();

        assert_eq!(
            append_exact_corner_boundary_triangles(
                &crate::test_support::approximate_decisions(),
                &polygon,
                &[0, 1, 2, 3],
                &[0, 1, 2, 3],
                &vertices,
                &mut triangles,
            )
            .unwrap(),
            None
        );
        assert!(triangles.is_empty());
    }

    #[test]
    fn exact_corner_fan_preserves_collinear_vertices_opposite_the_anchor() {
        let polygon = approximate_convex_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let vertices = vec![
            ov(0, 0, 0),
            ov(2, 0, 0),
            ov(2, 2, 0),
            ov(1, 2, 0),
            ov(0, 2, 0),
        ];
        let mut triangles = Vec::new();

        assert_eq!(
            append_exact_corner_boundary_triangles(
                &crate::test_support::approximate_decisions(),
                &polygon,
                &[0, 1, 2, 3, 4],
                &[0, 1, 2, 3, 4],
                &vertices,
                &mut triangles,
            )
            .unwrap(),
            Some(())
        );
        assert_eq!(triangles, vec![[0, 1, 2], [0, 2, 3], [0, 3, 4]]);
    }

    #[test]
    fn output_triangle_nondegeneracy_projects_every_normal_axis_exactly() {
        for points in [
            [p(0, 0, 0), p(1, 0, 0), p(0, 1, 0), p(2, 0, 0)],
            [p(0, 0, 0), p(0, 1, 0), p(0, 0, 1), p(0, 2, 0)],
            [p(0, 0, 0), p(1, 0, 0), p(0, 0, 1), p(2, 0, 0)],
            [p(0, 0, 0), p(1, 0, -1), p(0, 1, -1), p(2, 0, -2)],
        ] {
            let support = Plane::from_points(&points[0], &points[1], &points[2]);
            let vertices = points
                .into_iter()
                .map(|point| OutputVertex {
                    x: point.x,
                    y: point.y,
                    z: point.z,
                })
                .collect::<Vec<_>>();
            assert!(
                output_triangle_is_nondegenerate(
                    &crate::test_support::approximate_decisions(),
                    [0, 1, 2],
                    &vertices,
                    &support
                )
                .unwrap()
            );
            assert!(
                !output_triangle_is_nondegenerate(
                    &crate::test_support::approximate_decisions(),
                    [0, 1, 3],
                    &vertices,
                    &support
                )
                .unwrap()
            );
        }

        let symbolic_support =
            Plane::from_coefficients(Real::pi(), Real::zero(), Real::zero(), Real::zero());
        let vertices = vec![ov(0, 0, 0), ov(0, 1, 0), ov(0, 0, 1), ov(0, 2, 0)];
        assert!(
            output_triangle_is_nondegenerate(
                &crate::test_support::approximate_decisions(),
                [0, 1, 2],
                &vertices,
                &symbolic_support
            )
            .unwrap()
        );
        assert!(
            !output_triangle_is_nondegenerate(
                &crate::test_support::approximate_decisions(),
                [0, 1, 3],
                &vertices,
                &symbolic_support
            )
            .unwrap()
        );
    }

    #[test]
    fn unused_output_vertices_are_compacted_and_remapped() {
        let mut soup = BooleanMesh {
            vertices: vec![
                ov(9, 9, 9),
                ov(0, 0, 0),
                ov(8, 8, 8),
                ov(1, 0, 0),
                ov(0, 1, 0),
            ],
            triangles: vec![[1, 3, 4]],
            sources: vec![TriangleSource::default()],
        };

        remove_unused_vertices(&mut soup);

        assert_eq!(soup.vertices, vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 1, 0)]);
        assert_eq!(soup.triangles, vec![[0, 1, 2]]);
    }

    #[test]
    fn internal_resolution_merges_duplicate_vertices_and_faces_exactly() {
        let soup = BooleanMesh {
            vertices: vec![
                ov(0, 0, 0),
                ov(1, 0, 0),
                ov(0, 1, 0),
                ov(1, 0, 0),
                ov(2, 0, 0),
            ],
            triangles: vec![[0, 1, 2], [0, 3, 2], [0, 1, 4]],
            sources: vec![
                TriangleSource {
                    mesh: 0,
                    triangle: 3,
                    orientation: 0,
                },
                TriangleSource {
                    mesh: 1,
                    triangle: 9,
                    orientation: 0,
                },
                TriangleSource {
                    mesh: 1,
                    triangle: 10,
                    orientation: 0,
                },
            ],
        };

        let resolved =
            resolve_tjunctions(&crate::test_support::approximate_decisions(), &soup).unwrap();

        assert_eq!(resolved.vertices.len(), 4);
        assert_eq!(resolved.triangles.len(), 1);
        assert_eq!(
            resolved.sources,
            vec![TriangleSource {
                mesh: 0,
                triangle: 3,
                orientation: 0,
            }]
        );
    }

    #[test]
    fn internal_resolution_splits_exact_boundary_tjunction() {
        let soup = BooleanMesh {
            vertices: vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0), ov(1, 0, 0)],
            triangles: vec![[0, 1, 2]],
            sources: vec![TriangleSource {
                mesh: 1,
                triangle: 7,
                orientation: 0,
            }],
        };

        let resolved =
            resolve_tjunctions(&crate::test_support::approximate_decisions(), &soup).unwrap();

        assert_eq!(resolved.vertices.len(), 4);
        assert_eq!(resolved.triangles.len(), 2);
        assert_eq!(
            resolved.sources,
            vec![
                TriangleSource {
                    mesh: 1,
                    triangle: 7,
                    orientation: 0,
                };
                2
            ]
        );
        assert!(
            resolved
                .triangles
                .iter()
                .any(|triangle| triangle.contains(&3))
        );
    }

    #[test]
    fn tjunction_scan_keeps_exact_points_collapsed_by_f64_and_reverses_the_chain() {
        let base = Real::from(1_i64 << 60);
        let soup = BooleanMesh {
            vertices: vec![
                ovx(base.clone(), 0, 0),
                ovx(&base + &Real::from(2), 0, 0),
                ovx(base.clone(), 1, 0),
                ovx(&base + &Real::one(), 0, 0),
            ],
            triangles: vec![[1, 0, 2]],
            sources: vec![TriangleSource::default()],
        };
        let approximate = exact_output_vertex_enclosures(&soup.vertices).unwrap();
        assert!(approximate[0][0][1] >= approximate[1][0][0]);
        assert!(approximate[1][0][1] >= approximate[0][0][0]);
        assert!(approximate[0][0][1] >= approximate[3][0][0]);
        assert!(approximate[3][0][1] >= approximate[0][0][0]);

        for_each_policy(|decisions| {
            let mut resolved = soup.clone();

            assert!(
                split_one_tjunction_pass(decisions, &mut resolved, Some(&approximate)).unwrap()
            );
            assert_eq!(resolved.triangles, vec![[1, 3, 2], [3, 0, 2]]);
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        });
    }

    #[test]
    fn tjunction_chain_uses_policy_aware_axis_order_without_rational_coordinates() {
        let sqrt_two = Real::from(2).sqrt().unwrap();
        let vertices = vec![
            ov(0, 0, 0),
            ovx(&sqrt_two * &Real::from(2), 0, 0),
            ovx(sqrt_two, 0, 0),
        ];
        assert!(exact_output_vertex_enclosures(&vertices).is_none());

        for_each_policy(|decisions| {
            let axis_order = sorted_vertex_indices_by_axis(decisions, &vertices).unwrap();
            let chain = build_split_edge_chain(
                decisions,
                &vertices,
                SplitEdgeSearch::AxisOrder(&axis_order),
                [0, 1],
            )
            .unwrap();

            assert_eq!(chain.subedges().collect::<Vec<_>>(), vec![[0, 2], [2, 1]]);
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        });
    }

    #[test]
    fn tjunction_chain_uses_exact_axis_order_outside_f64_enclosure_range() {
        let huge = (Real::from(2).powi_i64(1025).unwrap() / Real::from(3)).unwrap();
        let vertices = vec![
            ov(0, 0, 0),
            ovx(&huge * &Real::from(2), 0, 0),
            ovx(huge, 0, 0),
        ];
        assert!(exact_output_vertex_enclosures(&vertices).is_none());

        for_each_policy(|decisions| {
            let axis_order = sorted_vertex_indices_by_axis(decisions, &vertices).unwrap();
            let chain = build_split_edge_chain(
                decisions,
                &vertices,
                SplitEdgeSearch::AxisOrder(&axis_order),
                [0, 1],
            )
            .unwrap();

            assert_eq!(chain.subedges().collect::<Vec<_>>(), vec![[0, 2], [2, 1]]);
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        });
    }

    #[test]
    fn internal_resolution_runs_until_the_finite_event_set_is_empty() {
        let soup = BooleanMesh {
            vertices: vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0), ov(1, 0, 0)],
            triangles: vec![[0, 1, 2]],
            sources: vec![TriangleSource::default()],
        };

        let resolved =
            resolve_tjunctions(&crate::test_support::approximate_decisions(), &soup).unwrap();

        assert_eq!(resolved.triangles.len(), 2);
    }

    #[test]
    fn crossing_discovery_batches_independent_events() {
        let soup = BooleanMesh {
            vertices: vec![
                ov(-2, 0, 0),
                ov(2, 0, 0),
                ov(-2, -1, 0),
                ov(0, -2, 0),
                ov(0, 2, 0),
                ov(1, -2, 0),
                ov(8, 0, 0),
                ov(12, 0, 0),
                ov(8, -1, 0),
                ov(10, -2, 0),
                ov(10, 2, 0),
                ov(11, -2, 0),
            ],
            triangles: vec![[0, 1, 2], [3, 4, 5], [6, 7, 8], [9, 10, 11]],
            sources: vec![TriangleSource::default(); 4],
        };

        for_each_policy(|decisions| {
            let mut resolved = soup.clone();
            let approximate = exact_output_vertex_enclosures(&resolved.vertices);
            assert!(
                split_edge_crossing_events(decisions, &mut resolved, approximate.as_deref())
                    .unwrap()
            );
            assert!(resolved.vertices.contains(&ov(0, 0, 0)));
            assert!(resolved.vertices.contains(&ov(10, 0, 0)));
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        });
    }

    #[test]
    fn adaptive_crossing_sweep_selects_the_least_overlapping_sampled_axis() {
        let mut vertices = Vec::with_capacity(MIN_ADAPTIVE_CROSSING_SWEEP_EDGES * 2);
        let mut edges = Vec::with_capacity(MIN_ADAPTIVE_CROSSING_SWEEP_EDGES);
        for index in 0..MIN_ADAPTIVE_CROSSING_SWEEP_EDGES {
            let coordinate = index as f64;
            let start = vertices.len();
            vertices.push([[0.0, 0.0], [coordinate, coordinate], [0.0, 0.0]]);
            vertices.push([[1.0, 1.0], [coordinate, coordinate], [0.0, 0.0]]);
            edges.push(ExactEdgeBounds {
                edge: [start, start + 1],
                min: [start; 3],
                max: [start + 1; 3],
            });
        }

        assert_eq!(least_sampled_interval_overlap_axis(&vertices, &edges), 1);
        assert_eq!(
            approximate_crossing_sweep_axis(&vertices, &edges[..edges.len() - 1]),
            0,
        );
        assert_eq!(approximate_crossing_sweep_axis(&vertices, &edges), 1);
    }

    #[test]
    fn cached_crossing_bounds_preserve_separated_exact_sweep() {
        let triangle_count = MIN_CACHED_CROSSING_BOUNDS_EDGES.div_ceil(3);
        let mut soup = BooleanMesh::default();
        for index in 0..triangle_count {
            let x = i32::try_from(index * 4).unwrap();
            let base = soup.vertices.len();
            soup.vertices
                .extend([ov(x, 0, 0), ov(x + 1, 0, 0), ov(x, 1, 0)]);
            soup.triangles.push([base, base + 1, base + 2]);
            soup.sources.push(TriangleSource::default());
        }
        assert!(soup.triangles.len() * 3 >= MIN_CACHED_CROSSING_BOUNDS_EDGES);
        let approximate = exact_output_vertex_enclosures(&soup.vertices).unwrap();

        for_each_policy(|decisions| {
            let mut resolved = soup.clone();
            assert!(
                !split_edge_crossing_events(decisions, &mut resolved, Some(&approximate)).unwrap()
            );
            assert_eq!(resolved, soup);
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        });
    }

    #[test]
    fn internal_resolution_exhausts_tjunctions_before_crossing_batches() {
        let soup = BooleanMesh {
            vertices: vec![
                ov(-10, 0, 0),
                ov(-8, 0, 0),
                ov(-10, 2, 0),
                ov(-9, 0, 0),
                ov(0, 0, 1),
                ov(4, 0, 1),
                ov(0, -1, 1),
                ov(2, -2, 1),
                ov(2, 2, 1),
                ov(3, -2, 1),
            ],
            triangles: vec![[0, 1, 2], [4, 5, 6], [7, 8, 9]],
            sources: vec![TriangleSource::default(); 3],
        };

        for_each_policy(|decisions| {
            let resolved = resolve_tjunctions(decisions, &soup).unwrap();
            assert!(
                resolved
                    .triangles
                    .iter()
                    .any(|triangle| triangle.contains(&3))
            );
            assert!(resolved.vertices.contains(&ov(2, 0, 1)));
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        });
    }

    #[test]
    fn crossing_discovery_keeps_exact_events_collapsed_by_f64() {
        let base = Real::from(1_i64 << 60);
        let soup = BooleanMesh {
            vertices: vec![
                ovx(base.clone(), 0, 0),
                ovx(&base + &Real::from(2), 0, 0),
                ovx(base.clone(), -1, 0),
                ovx(&base + &Real::one(), -1, 0),
                ovx(&base + &Real::one(), 1, 0),
                ovx(&base + &Real::from(2), -1, 0),
            ],
            triangles: vec![[0, 1, 2], [3, 4, 5]],
            sources: vec![TriangleSource::default(); 2],
        };
        let approximate = exact_output_vertex_enclosures(&soup.vertices).unwrap();
        assert!(approximate[0][0][1] >= approximate[1][0][0]);
        assert!(approximate[1][0][1] >= approximate[0][0][0]);
        assert!(approximate[0][0][1] >= approximate[3][0][0]);
        assert!(approximate[3][0][1] >= approximate[0][0][0]);

        for_each_policy(|decisions| {
            let mut resolved = soup.clone();
            assert!(
                split_edge_crossing_events(decisions, &mut resolved, Some(&approximate)).unwrap()
            );
            let expected = ovx(&base + &Real::one(), 0, 0);
            assert!(
                resolved
                    .vertices
                    .iter()
                    .any(|vertex| { output_vertices_equal(decisions, vertex, &expected).unwrap() })
            );
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        });
    }

    #[test]
    fn projected_crossing_rejects_skew_symbolic_edges_under_both_policies() {
        let sqrt_two = Real::from(2).sqrt().unwrap();
        let [a, b, c, d] = [
            ovx(-sqrt_two.clone(), 0, 0),
            ovx(sqrt_two, 0, 0),
            ov(0, -1, -1),
            ov(0, 1, 2),
        ];

        for_each_policy(|decisions| {
            assert!(
                proper_segment_intersection_after_bounds_overlap(
                    decisions, &a, &b, &c, &d, None, None,
                )
                .unwrap()
                .is_none()
            );
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        });
    }

    #[test]
    fn projected_crossing_reuses_rational_line_filters_for_every_topology() {
        let vertex = |x_num, x_den, y_num, y_den| OutputVertex {
            x: Real::from(Rational::fraction(x_num, x_den).unwrap()),
            y: Real::from(Rational::fraction(y_num, y_den).unwrap()),
            z: Real::zero(),
        };
        let a = vertex(0, 1, 1, 3);
        let b = vertex(1, 1, 2, 3);
        let c = vertex(0, 1, 2, 3);
        let d = vertex(1, 1, 1, 3);
        let same_side_start = vertex(0, 1, 3, 4);
        let same_side_end = vertex(1, 1, 4, 5);
        let query = |vertex: &OutputVertex| {
            RationalPoint3Query::from_certified_enclosures([&vertex.x, &vertex.y, &vertex.z].map(
                |coordinate| {
                    coordinate
                        .exact_rational_ref()
                        .unwrap()
                        .to_f64_enclosure()
                        .unwrap()
                },
            ))
            .unwrap()
        };
        let crossing_queries = [query(&a), query(&b), query(&c), query(&d)];
        let same_side_queries = [
            query(&a),
            query(&b),
            query(&same_side_start),
            query(&same_side_end),
        ];
        let endpoint_queries = [query(&a), query(&b), query(&b), query(&c)];

        for_each_policy(|decisions| {
            assert_eq!(
                projected_segment_crossing(
                    decisions,
                    &a,
                    &b,
                    &c,
                    &d,
                    [0, 1],
                    Some(&crossing_queries),
                )
                .unwrap(),
                Some(true)
            );
            assert_eq!(
                projected_segment_crossing(
                    decisions,
                    &a,
                    &b,
                    &same_side_start,
                    &same_side_end,
                    [0, 1],
                    Some(&same_side_queries),
                )
                .unwrap(),
                Some(false)
            );
            assert_eq!(
                projected_segment_crossing(
                    decisions,
                    &a,
                    &b,
                    &b,
                    &c,
                    [0, 1],
                    Some(&endpoint_queries),
                )
                .unwrap(),
                None
            );
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        });
    }

    #[test]
    fn crossing_discovery_batches_more_than_the_historical_pass_limit() {
        let mut soup = BooleanMesh {
            vertices: Vec::new(),
            triangles: Vec::new(),
            sources: Vec::new(),
        };
        for coordinate in 0..17_i32 {
            let y = coordinate * 3;
            let base = soup.vertices.len();
            soup.vertices
                .extend([ov(-1, y, 0), ov(49, y, 0), ov(-1, y - 1, 0)]);
            soup.triangles.push([base, base + 1, base + 2]);
            soup.sources.push(TriangleSource::default());

            let x = coordinate * 3;
            let base = soup.vertices.len();
            soup.vertices
                .extend([ov(x, -1, 0), ov(x, 49, 0), ov(x + 1, -1, 0)]);
            soup.triangles.push([base, base + 1, base + 2]);
            soup.sources.push(TriangleSource::default());
        }

        let approximate = exact_output_vertex_enclosures(&soup.vertices);
        assert!(
            split_edge_crossing_events(
                &crate::test_support::approximate_decisions(),
                &mut soup,
                approximate.as_deref(),
            )
            .unwrap()
        );
        for x in 0..17_i32 {
            for y in 0..17_i32 {
                assert!(soup.vertices.contains(&ov(x * 3, y * 3, 0)));
            }
        }
    }

    #[test]
    fn internal_resolution_rejects_invalid_indices_and_provenance_lengths() {
        let malformed_index = BooleanMesh {
            vertices: vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 1, 0)],
            triangles: vec![[0, 1, 3]],
            sources: vec![TriangleSource::default()],
        };
        assert_eq!(
            resolve_tjunctions(
                &crate::test_support::approximate_decisions(),
                &malformed_index
            ),
            Err(HypermeshError::VertexIndexOutOfBounds {
                index: 3,
                vertex_count: 3,
            })
        );

        let missing_source = BooleanMesh {
            vertices: vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 1, 0)],
            triangles: vec![[0, 1, 2]],
            sources: Vec::new(),
        };
        assert_eq!(
            resolve_tjunctions(
                &crate::test_support::approximate_decisions(),
                &missing_source
            ),
            Err(HypermeshError::TriangleSourceCountMismatch {
                triangles: 1,
                sources: 0,
            })
        );
    }

    #[test]
    fn output_extraction_uses_real_vertices() {
        let polygon = approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let result = BooleanResult::new(
            PolygonSoup {
                polygons: vec![polygon],
                bounds: Aabb::new(p(0, 0, 0), p(1, 1, 0)),
                num_meshes: 1,
            },
            vec![1],
        );

        let polygons = extract_output(&crate::test_support::APPROXIMATE_CONTEXT, &result)
            .unwrap()
            .into_value();
        assert_eq!(polygons.len(), 1);
        assert_eq!(polygons[0].vertices.len(), 3);
        assert!(polygons[0].vertices.iter().any(|vertex| vertex.x == r(1)));
    }

    #[test]
    fn output_polygon_closure_evidence_accepts_closed_tetrahedron() {
        let polygons = vec![
            op(vec![ov(0, 0, 0), ov(0, 1, 0), ov(1, 0, 0)]),
            op(vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 0, 1)]),
            op(vec![ov(0, 0, 0), ov(0, 0, 1), ov(0, 1, 0)]),
            op(vec![ov(1, 0, 0), ov(0, 1, 0), ov(0, 0, 1)]),
        ];

        let evidence = output_polygon_closure_evidence(
            &crate::test_support::approximate_decisions(),
            &polygons,
        )
        .unwrap();

        assert_eq!(
            evidence,
            BooleanMeshClosureEvidence {
                boundary_edges: 0,
                unbalanced_edges: 0,
                non_manifold_edges: 0,
            }
        );
    }

    #[test]
    fn output_polygon_closure_evidence_rejects_reversed_tetrahedron_face() {
        let mut polygons = vec![
            op(vec![ov(0, 0, 0), ov(0, 1, 0), ov(1, 0, 0)]),
            op(vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 0, 1)]),
            op(vec![ov(0, 0, 0), ov(0, 0, 1), ov(0, 1, 0)]),
            op(vec![ov(1, 0, 0), ov(0, 1, 0), ov(0, 0, 1)]),
        ];
        polygons[0].vertices.swap(1, 2);

        let evidence = output_polygon_closure_evidence(
            &crate::test_support::approximate_decisions(),
            &polygons,
        )
        .unwrap();

        assert_eq!(evidence.boundary_edges, 0);
        assert_eq!(evidence.unbalanced_edges, 3);
        assert_eq!(evidence.non_manifold_edges, 0);
        assert!(!evidence.has_no_boundary());
    }

    #[test]
    fn output_polygon_closure_evidence_accepts_balanced_non_manifold_multiplicity() {
        let mut polygons = vec![
            op(vec![ov(0, 0, 0), ov(0, 1, 0), ov(1, 0, 0)]),
            op(vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 0, 1)]),
            op(vec![ov(0, 0, 0), ov(0, 0, 1), ov(0, 1, 0)]),
            op(vec![ov(1, 0, 0), ov(0, 1, 0), ov(0, 0, 1)]),
        ];
        polygons.extend(polygons.clone());

        let evidence = output_polygon_closure_evidence(
            &crate::test_support::approximate_decisions(),
            &polygons,
        )
        .unwrap();

        assert_eq!(evidence.boundary_edges, 0);
        assert_eq!(evidence.unbalanced_edges, 0);
        assert_eq!(evidence.non_manifold_edges, 6);
        assert!(evidence.has_no_boundary());
        assert!(!evidence.is_closed());
    }

    #[test]
    fn boolean_mesh_closure_evidence_requires_directed_balance() {
        let mut reversed_face = positive_tetra_soup();
        reversed_face.triangles[0].swap(1, 2);
        let reversed_report = boolean_mesh_closure_evidence(&reversed_face);

        assert_eq!(reversed_report.boundary_edges, 0);
        assert_eq!(reversed_report.unbalanced_edges, 3);
        assert_eq!(reversed_report.non_manifold_edges, 0);
        assert!(!reversed_report.has_no_boundary());

        let mut doubled = positive_tetra_soup();
        doubled.triangles.extend(doubled.triangles.clone());
        doubled.sources.extend(doubled.sources.clone());
        let doubled_report = boolean_mesh_closure_evidence(&doubled);

        assert_eq!(doubled_report.boundary_edges, 0);
        assert_eq!(doubled_report.unbalanced_edges, 0);
        assert_eq!(doubled_report.non_manifold_edges, 6);
        assert!(doubled_report.has_no_boundary());
        assert!(
            !doubled
                .has_unique_nondegenerate_triangles_decision(
                    &crate::test_support::approximate_decisions()
                )
                .unwrap()
        );
    }

    #[test]
    fn output_vertex_merging_uses_numeric_equality() {
        let left = Real::pi() + Real::e();
        let equivalent_left = Real::e() + Real::pi();
        assert_ne!(left, equivalent_left);
        let soup = BooleanMesh {
            vertices: vec![
                OutputVertex {
                    x: left,
                    y: Real::zero(),
                    z: Real::zero(),
                },
                OutputVertex {
                    x: equivalent_left,
                    y: Real::zero(),
                    z: Real::zero(),
                },
            ],
            triangles: Vec::new(),
            sources: Vec::new(),
        };

        let merged =
            merge_duplicate_vertices(&crate::test_support::approximate_decisions(), &soup).unwrap();

        assert_eq!(merged.vertices.len(), 1);
    }

    #[test]
    fn output_vertex_merging_rejects_invalid_indices() {
        let soup = BooleanMesh {
            vertices: vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 1, 0)],
            triangles: vec![[0, 1, 3]],
            sources: vec![TriangleSource::default()],
        };

        assert!(
            !soup
                .has_unique_nondegenerate_triangles_decision(
                    &crate::test_support::approximate_decisions()
                )
                .unwrap()
        );
        assert!(matches!(
            merge_duplicate_vertices(&crate::test_support::approximate_decisions(), &soup),
            Err(HypermeshError::VertexIndexOutOfBounds {
                index: 3,
                vertex_count: 3
            })
        ));
    }

    #[test]
    fn merge_duplicate_polygon_vertices_reuses_exact_vertex_keys() {
        let polygons = vec![
            op(vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0)]),
            op(vec![ov(2, 0, 0), ov(0, 0, 0), ov(0, -1, 0)]),
        ];

        let (vertices, indexed) = merge_duplicate_polygon_vertices(
            &crate::test_support::approximate_decisions(),
            &polygons,
        )
        .unwrap();

        assert_eq!(vertices.len(), 4);
        assert_eq!(indexed[0], vec![0, 1, 2]);
        assert_eq!(indexed[1], vec![1, 0, 3]);
    }

    #[test]
    fn polygon_edge_counts_split_partial_shared_edges_exactly() {
        let polygons = vec![
            op(vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0)]),
            op(vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, -1, 0)]),
            op(vec![ov(1, 0, 0), ov(2, 0, 0), ov(2, -1, 0)]),
        ];
        let (vertices, indexed) = merge_duplicate_polygon_vertices(
            &crate::test_support::approximate_decisions(),
            &polygons,
        )
        .unwrap();
        let axis_order =
            sorted_vertex_indices_by_axis(&crate::test_support::approximate_decisions(), &vertices)
                .unwrap();
        let counts = polygon_edge_counts(
            &crate::test_support::approximate_decisions(),
            &vertices,
            &indexed,
            &axis_order,
        )
        .unwrap();

        assert_eq!(
            counts.get(&[0, 3]),
            Some(&DirectedEdgeUses {
                forward: 2,
                reverse: 0,
            })
        );
        assert_eq!(
            counts.get(&[1, 3]),
            Some(&DirectedEdgeUses {
                forward: 0,
                reverse: 2,
            })
        );
    }

    #[test]
    fn expanded_boundary_uses_unsplit_opposite_corner_fan() {
        let polygons = vec![
            approximate_convex_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0),
            approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, -1, 0), 0, 1),
            approximate_convex_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(2, -1, 0), 0, 2),
        ];

        let (soup, _) = triangulate_closed_polygon_arrangement(
            &crate::test_support::approximate_decisions(),
            &polygons,
            &[1, 1, 1],
            None,
            false,
            false,
            false,
        )
        .unwrap();
        let two_thirds = Real::from(Rational::fraction(2, 3).unwrap());

        assert_eq!(soup.vertices.len(), 6);
        assert_eq!(soup.triangles.len(), 4);
        assert!(!soup.vertices.contains(&OutputVertex {
            x: two_thirds.clone(),
            y: two_thirds,
            z: Real::zero(),
        }));
    }

    #[test]
    fn split_boundary_corner_fan_requires_both_incident_edges_unsplit() {
        let polygon = approximate_convex_triangle(&p(2, 0, 0), &p(0, 0, 0), &p(0, 2, 0), 0, 0);
        let indexed = [0, 1, 2];
        let vertices = vec![
            ov(2, 0, 0),
            ov(0, 0, 0),
            ov(0, 2, 0),
            ov(1, 0, 0),
            ov(1, 1, 0),
            ov(0, 1, 0),
        ];
        let mut triangles = Vec::new();

        assert!(
            append_split_boundary_fan_from_unsplit_corner(
                &crate::test_support::approximate_decisions(),
                &polygon,
                &indexed,
                &[0, 3, 1, 2],
                &vertices,
                &mut triangles,
            )
            .unwrap()
        );
        assert_eq!(triangles, vec![[2, 0, 3], [2, 3, 1]]);

        triangles.clear();
        assert!(
            !append_split_boundary_fan_from_unsplit_corner(
                &crate::test_support::approximate_decisions(),
                &polygon,
                &indexed,
                &[0, 3, 1, 4, 2, 5],
                &vertices,
                &mut triangles,
            )
            .unwrap()
        );
        assert!(triangles.is_empty());
    }

    #[test]
    fn split_boundary_corner_fan_rejects_a_degenerate_wedge_atomically() {
        let polygon = approximate_convex_triangle(&p(2, 0, 0), &p(0, 0, 0), &p(0, 2, 0), 0, 0);
        let vertices = vec![ov(2, 0, 0), ov(0, 0, 0), ov(0, 2, 0), ov(0, 1, 0)];
        let mut triangles = Vec::new();

        assert!(
            !append_split_boundary_fan_from_unsplit_corner(
                &crate::test_support::approximate_decisions(),
                &polygon,
                &[0, 1, 2],
                &[0, 3, 1, 2],
                &vertices,
                &mut triangles,
            )
            .unwrap()
        );
        assert!(triangles.is_empty());
    }

    #[test]
    fn split_segment_subedges_exact_reuses_undirected_edge_cache() {
        let polygons = vec![
            op(vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0)]),
            op(vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, -1, 0)]),
            op(vec![ov(2, 0, 0), ov(0, 0, 0), ov(2, -1, 0)]),
        ];
        let (vertices, _indexed) = merge_duplicate_polygon_vertices(
            &crate::test_support::approximate_decisions(),
            &polygons,
        )
        .unwrap();
        let axis_order =
            sorted_vertex_indices_by_axis(&crate::test_support::approximate_decisions(), &vertices)
                .unwrap();
        let mut cache = SplitEdgeCache::default();

        let forward = split_segment_subedges_exact(
            &crate::test_support::approximate_decisions(),
            &mut cache,
            &vertices,
            SplitEdgeSearch::AxisOrder(&axis_order),
            [0, 1],
        )
        .unwrap()
        .subedges()
        .collect::<Vec<_>>();
        let reversed = split_segment_subedges_exact(
            &crate::test_support::approximate_decisions(),
            &mut cache,
            &vertices,
            SplitEdgeSearch::AxisOrder(&axis_order),
            [1, 0],
        )
        .unwrap()
        .subedges()
        .collect::<Vec<_>>();

        assert_eq!(forward, vec![[0, 3], [3, 1]]);
        assert_eq!(reversed, forward);
        assert_eq!(cache.len(), 1);
    }

    #[test]
    fn construction_edge_candidates_index_incident_plane_triples_order_independently() {
        let plane = |mesh, plane| ConstructionPlaneIdentity { mesh, plane };
        let a = plane(0, 0);
        let b = plane(0, 1);
        let c = plane(1, 0);
        let source_edge = |endpoints| ConstructionEdgeIdentity::Source { mesh: 0, endpoints };
        let first = approximate_convex_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let first = first
            .with_known_vertex_cycle_and_edges(
                &crate::test_support::approximate_decisions(),
                first
                    .vertices(&crate::test_support::APPROXIMATE_CONTEXT)
                    .unwrap()
                    .into_value(),
                vec![
                    ConstructionVertexIdentity::PlaneTriple { planes: [a, b, c] },
                    ConstructionVertexIdentity::Source { mesh: 0, vertex: 1 },
                    ConstructionVertexIdentity::Source { mesh: 0, vertex: 2 },
                ],
                first.edges.as_ref().clone(),
                vec![
                    ConstructionEdgeIdentity::Split { planes: [b, a] },
                    source_edge([1, 2]),
                    source_edge([0, 2]),
                ],
            )
            .unwrap();
        let second = approximate_convex_triangle(&p(1, 0, 0), &p(3, 0, 0), &p(1, 2, 0), 0, 1);
        let second = second
            .with_known_vertex_cycle_and_edges(
                &crate::test_support::approximate_decisions(),
                second
                    .vertices(&crate::test_support::APPROXIMATE_CONTEXT)
                    .unwrap()
                    .into_value(),
                vec![
                    ConstructionVertexIdentity::PlaneTriple { planes: [c, b, a] },
                    ConstructionVertexIdentity::Source { mesh: 0, vertex: 4 },
                    ConstructionVertexIdentity::Source { mesh: 0, vertex: 5 },
                ],
                second.edges.as_ref().clone(),
                vec![
                    source_edge([3, 4]),
                    source_edge([4, 5]),
                    source_edge([3, 5]),
                ],
            )
            .unwrap();

        let candidates = build_construction_edge_candidates(
            &[first, second],
            &[vec![0, 1, 2], vec![3, 4, 5]],
            6,
        )
        .unwrap();
        let split_group = candidates.polygon_edges[0][0];

        assert_eq!(candidates.groups[split_group].collinear, vec![0, 1, 3]);
    }

    #[test]
    fn construction_labels_do_not_admit_off_segment_vertices() {
        let vertices = vec![ov(0, 0, 0), ov(2, 0, 0), ov(1, 1, 0), ov(1, 0, 0)];
        let candidates = ConstructionEdgeCandidateGroup {
            collinear: vec![0, 1, 2, 3],
        };
        let mut cache = SplitEdgeCache::default();

        let subedges = split_segment_subedges_exact_candidates(
            &crate::test_support::approximate_decisions(),
            &mut cache,
            &vertices,
            [0, 1],
            &candidates,
            &[],
            None,
            false,
        )
        .unwrap()
        .subedges()
        .collect::<Vec<_>>();

        assert_eq!(subedges, vec![[0, 3], [3, 1]]);
    }

    #[test]
    fn inexpensive_segment_axis_uses_largest_finite_approximation() {
        assert_eq!(
            inexpensive_nonzero_segment_axis(
                &crate::test_support::approximate_decisions(),
                &ov(1, 2, 3),
                &ov(0, 0, 0)
            )
            .unwrap(),
            2
        );
        assert_eq!(
            inexpensive_nonzero_segment_axis(
                &crate::test_support::approximate_decisions(),
                &ov(3, -3, 2),
                &ov(0, 0, 0)
            )
            .unwrap(),
            1
        );
        assert_eq!(
            inexpensive_nonzero_segment_axis(
                &crate::test_support::approximate_decisions(),
                &ov(1, 1, 1),
                &ov(1, 1, 1)
            )
            .unwrap(),
            0
        );
    }

    #[test]
    fn candidate_vertex_index_range_for_edge_matches_full_vertex_scan() {
        let polygons = vec![
            op(vec![ov(0, 0, 0), ov(3, 0, 0), ov(0, 2, 0)]),
            op(vec![ov(1, 0, 0), ov(2, 0, 0), ov(1, -1, 0)]),
            op(vec![ov(3, 0, 0), ov(0, 0, 0), ov(3, -1, 0)]),
            op(vec![ov(0, 1, 0), ov(0, 3, 0), ov(-1, 1, 0)]),
        ];
        let (vertices, _indexed) = merge_duplicate_polygon_vertices(
            &crate::test_support::approximate_decisions(),
            &polygons,
        )
        .unwrap();
        let axis_order =
            sorted_vertex_indices_by_axis(&crate::test_support::approximate_decisions(), &vertices)
                .unwrap();
        let edge = [0, 1];
        let axis = dominant_segment_axis(
            &crate::test_support::approximate_decisions(),
            &vertices[edge[0]],
            &vertices[edge[1]],
        )
        .unwrap();

        let (start, end) = candidate_vertex_index_range_for_edge(
            &crate::test_support::approximate_decisions(),
            &axis_order,
            &vertices,
            edge,
            axis,
        )
        .unwrap();
        let filtered = axis_order[axis][start..end].to_vec();
        let full_scan = (0..vertices.len()).collect::<Vec<_>>();

        let filtered_on_edge = filtered
            .into_iter()
            .filter(|index| {
                *index != edge[0]
                    && *index != edge[1]
                    && point_on_segment_exact(
                        &crate::test_support::approximate_decisions(),
                        &vertices[*index],
                        &vertices[edge[0]],
                        &vertices[edge[1]],
                    )
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let full_on_edge = full_scan
            .into_iter()
            .filter(|index| {
                *index != edge[0]
                    && *index != edge[1]
                    && point_on_segment_exact(
                        &crate::test_support::approximate_decisions(),
                        &vertices[*index],
                        &vertices[edge[0]],
                        &vertices[edge[1]],
                    )
                    .unwrap()
            })
            .collect::<Vec<_>>();

        assert_eq!(filtered_on_edge, full_on_edge);
    }

    #[test]
    fn certified_triangulation_rejects_duplicate_open_faces_exactly() {
        let polygon = approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let result = BooleanResult::new(
            PolygonSoup {
                polygons: vec![polygon.clone(), polygon],
                bounds: Aabb::new(p(0, 0, 0), p(1, 1, 0)),
                num_meshes: 1,
            },
            vec![1, 1],
        );

        let err = certify_output_polygon_closure_decision(
            &crate::test_support::approximate_decisions(),
            &result,
        )
        .unwrap_err();
        assert_eq!(
            err,
            HypermeshError::OpenOutput {
                boundary_edges: 0,
                unbalanced_edges: 3,
                non_manifold_edges: 0,
            }
        );
    }

    #[test]
    fn certified_triangulation_rejects_open_output() {
        let polygon = approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let result = BooleanResult::new(
            PolygonSoup {
                polygons: vec![polygon],
                bounds: Aabb::new(p(0, 0, 0), p(1, 1, 0)),
                num_meshes: 1,
            },
            vec![1],
        );

        let err = triangulate_and_resolve_polygon_certified(
            &crate::test_support::approximate_decisions(),
            &result,
        )
        .unwrap_err();
        assert_eq!(
            err,
            HypermeshError::OpenOutput {
                boundary_edges: 3,
                unbalanced_edges: 3,
                non_manifold_edges: 0,
            }
        );
    }

    #[test]
    fn boolean_result_preserves_classified_winding_evidence() {
        let polygon = approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let mut classified = ClassifiedPolygon::new(polygon, 1);
        classified.winding = Some(WindingPair {
            w_front: vec![0],
            w_back: vec![1],
        });

        let result = BooleanResult::from_classified(
            PolygonSoup {
                polygons: Vec::new(),
                bounds: Aabb::new(p(0, 0, 0), p(1, 1, 0)),
                num_meshes: 1,
            },
            vec![classified],
        );

        assert_eq!(result.winding_pairs().len(), 1);
        assert_eq!(
            result.winding_pairs()[0],
            Some(WindingPair {
                w_front: vec![0],
                w_back: vec![1],
            })
        );
    }

    #[test]
    fn boolean_result_dedupes_exact_duplicate_oriented_classified_polygons() {
        let mut first = ClassifiedPolygon::new(
            approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        );
        first.winding = Some(WindingPair {
            w_front: vec![0],
            w_back: vec![1],
        });
        let second = ClassifiedPolygon::new(
            approximate_convex_triangle(&p(1, 0, 0), &p(0, 1, 0), &p(0, 0, 0), 1, 7),
            1,
        );

        let result = BooleanResult::from_classified(
            PolygonSoup {
                polygons: Vec::new(),
                bounds: Aabb::new(p(0, 0, 0), p(1, 1, 0)),
                num_meshes: 2,
            },
            vec![first, second],
        );

        assert_eq!(result.output().polygons.len(), 1);
        assert_eq!(result.classifications(), &[1]);
        assert_eq!(
            result.winding_pairs(),
            &[Some(WindingPair {
                w_front: vec![0],
                w_back: vec![1],
            })]
        );
    }

    #[test]
    fn boolean_result_keeps_distinct_same_support_polygons() {
        let first = ClassifiedPolygon::new(
            approximate_convex_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0),
            1,
        );
        let second = ClassifiedPolygon::new(
            approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 1),
            1,
        );

        let result = BooleanResult::from_classified(
            PolygonSoup {
                polygons: Vec::new(),
                bounds: Aabb::new(p(0, 0, 0), p(2, 2, 0)),
                num_meshes: 1,
            },
            vec![first, second],
        );

        assert_eq!(result.output().polygons.len(), 2);
        assert_eq!(result.classifications(), &[1, 1]);
    }

    #[test]
    fn push_unique_classified_polygon_merges_duplicate_classified_output() {
        let mut output = Vec::new();
        let first = ClassifiedPolygon::new(
            approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        );
        let mut second = ClassifiedPolygon::new(
            approximate_convex_triangle(&p(1, 0, 0), &p(0, 1, 0), &p(0, 0, 0), 1, 3),
            1,
        );
        second.winding = Some(WindingPair {
            w_front: vec![2],
            w_back: vec![3],
        });
        second.is_bsp_fragment = true;

        push_unique_classified_polygon(&mut output, first);
        push_unique_classified_polygon(&mut output, second);

        assert_eq!(output.len(), 1);
        assert_eq!(
            output[0].winding,
            Some(WindingPair {
                w_front: vec![2],
                w_back: vec![3],
            })
        );
        assert!(output[0].is_bsp_fragment);
    }

    #[test]
    fn merge_unique_classified_polygons_dedupes_exact_duplicate_output() {
        let mut output = vec![ClassifiedPolygon::new(
            approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        )];
        let mut duplicate = ClassifiedPolygon::new(
            approximate_convex_triangle(&p(1, 0, 0), &p(0, 1, 0), &p(0, 0, 0), 1, 4),
            1,
        );
        duplicate.winding = Some(WindingPair {
            w_front: vec![5],
            w_back: vec![6],
        });
        duplicate.is_bsp_fragment = true;

        merge_unique_classified_polygons(&mut output, vec![duplicate]);

        assert_eq!(output.len(), 1);
        assert_eq!(
            output[0].winding,
            Some(WindingPair {
                w_front: vec![5],
                w_back: vec![6],
            })
        );
        assert!(output[0].is_bsp_fragment);
    }

    #[test]
    fn merge_unique_classified_polygons_keeps_distinct_same_support_polygons() {
        let mut output = vec![ClassifiedPolygon::new(
            approximate_convex_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0),
            1,
        )];
        let incoming = vec![ClassifiedPolygon::new(
            approximate_convex_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 1),
            1,
        )];

        merge_unique_classified_polygons(&mut output, incoming);

        assert_eq!(output.len(), 2);
    }

    #[test]
    fn certified_triangulation_rejects_open_surface_after_boundary_tjunction_cleanup() {
        let lower = approximate_convex_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let upper = approximate_convex_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1);
        let result = BooleanResult::new(
            PolygonSoup {
                polygons: vec![lower, upper],
                bounds: Aabb::new(p(0, 0, 0), p(2, 2, 0)),
                num_meshes: 1,
            },
            vec![1, 1],
        );

        let err = triangulate_and_resolve_polygon_certified(
            &crate::test_support::approximate_decisions(),
            &result,
        )
        .unwrap_err();
        assert!(matches!(err, HypermeshError::OpenOutput { .. }));
    }

    #[test]
    fn signed_volume_certification_accepts_only_positive_orientation() {
        let positive = positive_tetra_soup();
        certify_positive_signed_volume(&crate::test_support::approximate_decisions(), &positive)
            .unwrap();

        let mut reversed = positive.clone();
        for triangle in &mut reversed.triangles {
            triangle.swap(0, 1);
        }
        assert_eq!(
            certify_positive_signed_volume(
                &crate::test_support::approximate_decisions(),
                &reversed
            ),
            Err(HypermeshError::UnknownClassification)
        );

        let flat = BooleanMesh {
            vertices: vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 1, 0)],
            triangles: vec![[0, 1, 2]],
            sources: vec![TriangleSource::default()],
        };
        assert_eq!(
            certify_positive_signed_volume(&crate::test_support::approximate_decisions(), &flat),
            Err(HypermeshError::UnknownClassification)
        );
    }
}
