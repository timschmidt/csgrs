//! Boolean result extraction and triangulation helpers.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, compare_real};
use crate::mesh::{OutputVertex, PolygonSoup};
use crate::polygon::{ConstructionEdgeIdentity, ConvexPolygon};
use crate::storage_hash::StorageHashMap;
use crate::winding::WindingPair;
use hyperlattice::{Rational, Real};
use hyperreal::PreparedRationalLine2Filter;

const RESOLVE_TJUNCTION_MAX_PASSES: usize = 256;

pub(crate) const ARRANGEMENT_CLASSIFICATION: i8 = 2;

type SplitEdgeCache = StorageHashMap<[usize; 2], Vec<[usize; 2]>>;

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

struct PlaneProfileInterner {
    planes: Vec<Plane>,
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
    pub(crate) soup: TriangleSoup,
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
            if let Some(existing_index) = find_matching_output_polygon_index(
                &buckets,
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
            if let Some(bucket) = buckets.iter_mut().find(|bucket| {
                bucket.classification == classification
                    && bucket.edge_count == edge_count
                    && bucket.support == support
                    && bucket.edge_profile == edge_profile
            }) {
                bucket.indices.push(new_index);
            } else {
                buckets.push(ClassifiedOutputBucket {
                    classification,
                    support,
                    edge_count,
                    edge_profile,
                    indices: vec![new_index],
                });
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
    polygons: &[ConvexPolygon],
    classification: i8,
    edge_profile: &[usize],
    candidate: &ConvexPolygon,
) -> Option<usize> {
    let bucket = buckets.iter().find(|bucket| {
        bucket.classification == classification
            && bucket.edge_count == candidate.edges.len()
            && bucket.support == candidate.support
            && bucket.edge_profile == edge_profile
    })?;
    bucket
        .indices
        .iter()
        .copied()
        .find(|index| polygons_match_output_geometry(&polygons[*index], candidate))
}

impl PlaneProfileInterner {
    fn new() -> Self {
        Self { planes: Vec::new() }
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
        if let Some(index) = self.planes.iter().position(|existing| existing == plane) {
            return index;
        }
        let index = self.planes.len();
        self.planes.push(plane.clone());
        index
    }
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
    pub triangle: isize,
    /// `+1` when output orientation matches the source and `-1` when inverted.
    ///
    /// Zero is reserved for callers constructing source records without
    /// orientation provenance.
    pub orientation: i8,
}

/// Indexed triangle soup using hyperreal output vertices.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TriangleSoup {
    /// Output vertices.
    pub vertices: Vec<OutputVertex>,
    /// Triangle vertex indices.
    pub triangles: Vec<[usize; 3]>,
    /// Source polygon for each triangle, parallel to `triangles`.
    pub sources: Vec<TriangleSource>,
}

/// Exact closure summary for an indexed triangle soup.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TriangleSoupClosureReport {
    /// Number of undirected edges used by exactly one triangle.
    pub boundary_edges: usize,
    /// Number of geometric edge classes whose forward and reverse uses do not
    /// cancel.
    pub unbalanced_edges: usize,
    /// Number of undirected edges used by more than two triangles.
    pub non_manifold_edges: usize,
}

impl TriangleSoupClosureReport {
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
pub fn extract_output(result: &BooleanResult) -> HypermeshResult<Vec<OutputPolygon>> {
    extract_output_polygons(&result.output.polygons)
}

/// Extracts output polygons from a borrowed polygon slice.
pub fn extract_output_polygons(polygons: &[ConvexPolygon]) -> HypermeshResult<Vec<OutputPolygon>> {
    let mut out = Vec::with_capacity(polygons.len());
    for polygon in polygons {
        let mut vertices = Vec::with_capacity(polygon.vertex_count());
        for point in polygon.vertices()? {
            vertices.push(OutputVertex {
                x: point.x,
                y: point.y,
                z: point.z,
            });
        }
        out.push(OutputPolygon {
            vertices,
            source_mesh: polygon.mesh_index,
            source_polygon: polygon.polygon_index,
        });
    }
    Ok(out)
}

fn triangulate_output(result: &BooleanResult) -> HypermeshResult<TriangleSoup> {
    triangulate_polygons(&result.output.polygons, Some(&result.classifications))
}

/// Fan-triangulates and resolves exact duplicate/T-junction artifacts.
///
/// This is useful for tests and callers that need evidence that the classified
/// arrangement is already a closed regularized PWN surface. Non-manifold edge
/// valence is allowed, but non-empty open, reversed, or zero-volume soups are
/// reported as uncertified.
pub fn triangulate_and_resolve_certified(result: &BooleanResult) -> HypermeshResult<TriangleSoup> {
    certify_output_polygon_closure(result)?;
    triangulate_and_resolve_polygon_certified(result)
}

pub(crate) fn triangulate_and_resolve_polygon_certified(
    result: &BooleanResult,
) -> HypermeshResult<TriangleSoup> {
    let mut soup = match triangulate_closed_polygon_arrangement(
        &result.output.polygons,
        &result.classifications,
        None,
        false,
        false,
        false,
    ) {
        Ok((soup, _)) => soup,
        Err(HypermeshError::UnknownClassification) => {
            resolve_tjunctions(&triangulate_output(result)?)?
        }
        Err(err) => return Err(err),
    };
    if soup.triangles.is_empty() {
        return Ok(soup);
    }
    let mut closure = triangle_soup_closure_report(&soup);
    if !closure.has_no_boundary() {
        soup = resolve_tjunctions(&triangulate_output(result)?)?;
        closure = triangle_soup_closure_report(&soup);
    }
    if !closure.has_no_boundary() {
        return Err(HypermeshError::OpenOutput {
            boundary_edges: closure.boundary_edges,
            unbalanced_edges: closure.unbalanced_edges,
            non_manifold_edges: closure.non_manifold_edges,
        });
    }
    if !boolean_result_has_complete_orientation_evidence(result) {
        certify_positive_signed_volume(&soup)?;
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

fn triangulate_closed_polygon_arrangement(
    polygons: &[ConvexPolygon],
    orientations: &[i8],
    polygon_windings: Option<&[WindingPair]>,
    prefer_precomputed_f64_scan: bool,
    prefer_construction_candidates: bool,
    filter_recovery_candidates: bool,
) -> HypermeshResult<(TriangleSoup, Vec<WindingPair>)> {
    if polygons.len() != orientations.len() {
        return Err(HypermeshError::UnknownClassification);
    }
    if polygon_windings.is_some_and(|windings| windings.len() != polygons.len()) {
        return Err(HypermeshError::UnknownClassification);
    }
    let (mut vertices, indexed_polygons) = merge_duplicate_convex_polygon_vertices(polygons)?;
    let construction_candidates = prefer_construction_candidates
        .then(|| build_construction_edge_candidates(polygons, &indexed_polygons))
        .transpose()?;
    let approximate_vertices = prefer_precomputed_f64_scan
        .then(|| exact_output_vertices_f64(&vertices))
        .flatten();
    let axis_order = (approximate_vertices.is_none() && construction_candidates.is_none())
        .then(|| sorted_vertex_indices_by_axis(&vertices))
        .transpose()?;
    let mut split_edge_cache = SplitEdgeCache::default();
    let mut triangles = Vec::new();
    let mut sources = Vec::new();
    let mut triangle_windings = Vec::new();

    for (polygon_index, ((polygon, indexed), orientation)) in polygons
        .iter()
        .zip(indexed_polygons)
        .zip(orientations.iter().copied())
        .enumerate()
    {
        if indexed.len() < 3 {
            continue;
        }
        let mut boundary = Vec::new();
        for edge_index in 0..indexed.len() {
            let start = indexed[edge_index];
            let end = indexed[(edge_index + 1) % indexed.len()];
            if start == end {
                continue;
            }
            let canonical = sorted_edge([start, end]);
            let subedges = if let Some(candidates) = &construction_candidates {
                split_segment_subedges_exact_candidates(
                    &mut split_edge_cache,
                    &vertices,
                    canonical,
                    &candidates.groups[candidates.polygon_edges[polygon_index][edge_index]],
                    filter_recovery_candidates,
                )?
            } else if let Some(approximate_vertices) = &approximate_vertices {
                split_segment_subedges_exact_precomputed_f64_scan(
                    &mut split_edge_cache,
                    &vertices,
                    approximate_vertices,
                    canonical,
                )?
            } else {
                split_segment_subedges_exact(
                    &mut split_edge_cache,
                    &vertices,
                    axis_order
                        .as_ref()
                        .expect("axis order exists without an approximate scan"),
                    canonical,
                )?
            };
            if start == canonical[0] {
                boundary.extend(subedges.iter().map(|edge| edge[0]));
            } else {
                boundary.extend(subedges.iter().rev().map(|edge| edge[1]));
            }
        }
        boundary.dedup();
        if boundary.len() > 1 && boundary.first() == boundary.last() {
            boundary.pop();
        }
        if boundary.len() < 3 {
            continue;
        }
        let polygon_triangles = if boundary.len() > indexed.len() {
            let center = append_output_polygon_centroid(&mut vertices, &boundary)?;
            (0..boundary.len())
                .map(|index| {
                    [
                        center,
                        boundary[index],
                        boundary[(index + 1) % boundary.len()],
                    ]
                })
                .collect()
        } else if let Some(polygon_triangles) = construction_candidates
            .as_ref()
            .and_then(|_| triangulate_construction_boundary(polygon, &indexed, &boundary))
        {
            polygon_triangles
        } else {
            match triangulate_weakly_convex_boundary(&boundary, &vertices, &polygon.support) {
                Ok(triangles) => triangles,
                Err(HypermeshError::UnknownClassification) => (1..(indexed.len() - 1))
                    .map(|index| [indexed[0], indexed[index], indexed[index + 1]])
                    .collect(),
                Err(err) => return Err(err),
            }
        };
        for triangle in polygon_triangles {
            triangles.push(triangle);
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

    Ok((
        TriangleSoup {
            vertices,
            triangles,
            sources,
        },
        triangle_windings,
    ))
}

pub(crate) fn triangulate_classified_arrangement_precomputed_f64_scan(
    classified: &[ClassifiedPolygon],
) -> HypermeshResult<ClassifiedTriangleArrangement> {
    triangulate_classified_arrangement_with_strategy(classified, true, false, false)
}

pub(crate) fn triangulate_classified_arrangement_construction_candidates(
    classified: &[ClassifiedPolygon],
    filter_recovery_candidates: bool,
) -> HypermeshResult<ClassifiedTriangleArrangement> {
    triangulate_classified_arrangement_with_strategy(
        classified,
        false,
        true,
        filter_recovery_candidates,
    )
}

pub(crate) fn triangulate_preclassified_arrangement_construction_candidates(
    classified: &[ClassifiedPolygon],
    filter_recovery_candidates: bool,
) -> HypermeshResult<TriangleSoup> {
    let polygons = classified
        .iter()
        .map(|classified| classified.polygon.clone())
        .collect::<Vec<_>>();
    let orientations = classified
        .iter()
        .map(|classified| match classified.classification {
            orientation @ (-1 | 1) => Ok(orientation),
            _ => Err(HypermeshError::UnknownClassification),
        })
        .collect::<HypermeshResult<Vec<_>>>()?;
    let (mut soup, _) = triangulate_closed_polygon_arrangement(
        &polygons,
        &orientations,
        None,
        false,
        true,
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
    classified: &[ClassifiedPolygon],
    prefer_precomputed_f64_scan: bool,
    prefer_construction_candidates: bool,
    filter_recovery_candidates: bool,
) -> HypermeshResult<ClassifiedTriangleArrangement> {
    let polygons = classified
        .iter()
        .map(|classified| classified.polygon.clone())
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

fn append_output_polygon_centroid(
    vertices: &mut Vec<OutputVertex>,
    boundary: &[usize],
) -> HypermeshResult<usize> {
    let mut x = Real::zero();
    let mut y = Real::zero();
    let mut z = Real::zero();
    for &index in boundary {
        x += vertices[index].x.clone();
        y += vertices[index].y.clone();
        z += vertices[index].z.clone();
    }
    let count = Real::from(
        u64::try_from(boundary.len()).map_err(|_| HypermeshError::UnknownClassification)?,
    );
    let center = OutputVertex {
        x: (x / &count).map_err(|_| HypermeshError::UnknownClassification)?,
        y: (y / &count).map_err(|_| HypermeshError::UnknownClassification)?,
        z: (z / count).map_err(|_| HypermeshError::UnknownClassification)?,
    };
    let index = vertices.len();
    vertices.push(center);
    Ok(index)
}

fn triangulate_construction_boundary(
    polygon: &ConvexPolygon,
    indexed: &[usize],
    boundary: &[usize],
) -> Option<Vec<[usize; 3]>> {
    if indexed != boundary {
        return None;
    }
    let identities = polygon.known_edge_identities.as_ref()?;
    if identities.len() != boundary.len() {
        return None;
    }
    let strictly_convex = boundary
        .iter()
        .enumerate()
        .filter_map(|(index, &vertex)| {
            let incoming = (index + identities.len() - 1) % identities.len();
            (identities[incoming] != identities[index]).then_some(vertex)
        })
        .collect::<Vec<_>>();
    if strictly_convex.len() < 3 {
        return None;
    }
    Some(
        (1..(strictly_convex.len() - 1))
            .map(|index| {
                [
                    strictly_convex[0],
                    strictly_convex[index],
                    strictly_convex[index + 1],
                ]
            })
            .collect(),
    )
}

fn triangulate_weakly_convex_boundary(
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
            if output_triangle_is_nondegenerate(triangle, vertices, support)? {
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
    if !output_triangle_is_nondegenerate(triangle, vertices, support)? {
        return Err(HypermeshError::UnknownClassification);
    }
    triangles.push(triangle);
    Ok(triangles)
}

fn output_triangle_is_nondegenerate(
    triangle: [usize; 3],
    vertices: &[OutputVertex],
    support: &Plane,
) -> HypermeshResult<bool> {
    let left = sub_vertex(&vertices[triangle[1]], &vertices[triangle[0]]);
    let right = sub_vertex(&vertices[triangle[2]], &vertices[triangle[0]]);
    let cross = cross_arrays(&left, &right);
    let oriented_area = Real::signed_product_sum(
        [true, true, true],
        [
            [&cross[0], &support.normal.x],
            [&cross[1], &support.normal.y],
            [&cross[2], &support.normal.z],
        ],
    );
    Ok(crate::geometry::classify_real(&oriented_area)? != Classification::On)
}

/// Certifies that the classified polygon arrangement is already closed before
/// triangulation cleanup runs.
///
/// Balanced non-manifold edge valence is allowed, but any singleton edge or
/// directed edge imbalance is reported as [`HypermeshError::OpenOutput`]
/// instead of being left for triangle cleanup to repair.
pub fn certify_output_polygon_closure(
    result: &BooleanResult,
) -> HypermeshResult<TriangleSoupClosureReport> {
    let polygon_closure =
        output_polygon_closure_report_from_convex_polygons(&result.output.polygons)?;
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
fn output_polygon_closure_report(
    polygons: &[OutputPolygon],
) -> HypermeshResult<TriangleSoupClosureReport> {
    let (vertices, indexed_polygons) = merge_duplicate_polygon_vertices(polygons);
    output_polygon_closure_report_from_indexed_vertices(&vertices, &indexed_polygons)
}

fn output_polygon_closure_report_from_convex_polygons(
    polygons: &[ConvexPolygon],
) -> HypermeshResult<TriangleSoupClosureReport> {
    let (vertices, indexed_polygons) = merge_duplicate_convex_polygon_vertices(polygons)?;
    output_polygon_closure_report_from_indexed_vertices(&vertices, &indexed_polygons)
}

fn output_polygon_closure_report_from_indexed_vertices(
    vertices: &[OutputVertex],
    indexed_polygons: &[Vec<usize>],
) -> HypermeshResult<TriangleSoupClosureReport> {
    let axis_order = sorted_vertex_indices_by_axis(vertices)?;
    let edge_counts = polygon_edge_counts(vertices, indexed_polygons, &axis_order)?;
    let mut report = TriangleSoupClosureReport::default();
    for uses in edge_counts.values().copied() {
        if uses.total() == 1 {
            report.boundary_edges += 1;
        } else if uses.total() > 2 {
            report.non_manifold_edges += 1;
        }
        if !uses.is_balanced() {
            report.unbalanced_edges += 1;
        }
    }
    Ok(report)
}

#[cfg(test)]
fn merge_duplicate_polygon_vertices(
    polygons: &[OutputPolygon],
) -> (Vec<OutputVertex>, Vec<Vec<usize>>) {
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

    (vertices, indexed_polygons)
}

fn merge_duplicate_convex_polygon_vertices(
    polygons: &[ConvexPolygon],
) -> HypermeshResult<(Vec<OutputVertex>, Vec<Vec<usize>>)> {
    let mut positions = Vec::new();
    let mut indexed_polygons = Vec::with_capacity(polygons.len());
    let mut flat_index = 0usize;

    for (polygon_index, polygon) in polygons.iter().enumerate() {
        let points = polygon.vertices()?;
        indexed_polygons.push(vec![0; points.len()]);
        for (vertex_index, point) in points.into_iter().enumerate() {
            positions.push((
                polygon_index,
                vertex_index,
                flat_index,
                OutputVertex {
                    x: point.x,
                    y: point.y,
                    z: point.z,
                },
            ));
            flat_index += 1;
        }
    }

    if positions.iter().all(|(_, _, _, vertex)| {
        vertex.x.exact_rational_ref().is_some()
            && vertex.y.exact_rational_ref().is_some()
            && vertex.z.exact_rational_ref().is_some()
    }) {
        let mut vertices: Vec<OutputVertex> = Vec::with_capacity(positions.len());
        let mut storage_vertices: StorageHashMap<[usize; 3], usize> = StorageHashMap::default();
        storage_vertices.reserve(positions.len());
        // Rational's retained arithmetic caches use interior mutability, but
        // its Eq and Hash implementations depend only on the immutable exact
        // value. This map intentionally canonicalizes by that value.
        #[allow(clippy::mutable_key_type)]
        let mut exact_vertices: HashMap<ExactOutputVertexKey, usize> =
            HashMap::with_capacity(positions.len());
        for (polygon_index, vertex_index, _, vertex) in positions {
            let storage_key = exact_output_vertex_storage_key(&vertex)
                .expect("all output vertices were certified exact rational");
            let merged_index = if let Some(&index) = storage_vertices.get(&storage_key) {
                index
            } else {
                let key = exact_output_vertex_key(&vertex)
                    .expect("all output vertices were certified exact rational");
                let index = match exact_vertices.entry(key) {
                    std::collections::hash_map::Entry::Occupied(entry) => *entry.get(),
                    std::collections::hash_map::Entry::Vacant(entry) => {
                        let index = vertices.len();
                        vertices.push(vertex);
                        entry.insert(index);
                        index
                    }
                };
                storage_vertices.insert(storage_key, index);
                index
            };
            indexed_polygons[polygon_index][vertex_index] = merged_index;
        }
        return Ok((vertices, indexed_polygons));
    }

    positions.sort_by(|(_, _, _, left), (_, _, _, right)| {
        compare_output_vertices_lexicographic(left, right)
            .expect("exact output vertex ordering should compare")
    });

    let mut groups: Vec<(usize, OutputVertex, Vec<(usize, usize)>)> = Vec::new();
    for (polygon_index, vertex_index, flat_index, vertex) in positions {
        match groups.last_mut() {
            Some((first_flat_index, existing, members)) if *existing == vertex => {
                *first_flat_index = (*first_flat_index).min(flat_index);
                members.push((polygon_index, vertex_index));
            }
            _ => groups.push((flat_index, vertex, vec![(polygon_index, vertex_index)])),
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

#[derive(Eq, Hash, PartialEq)]
struct ExactOutputVertexKey([Rational; 3]);

fn exact_output_vertex_storage_key(vertex: &OutputVertex) -> Option<[usize; 3]> {
    Some([
        vertex.x.exact_rational_ref()?.storage_identity(),
        vertex.y.exact_rational_ref()?.storage_identity(),
        vertex.z.exact_rational_ref()?.storage_identity(),
    ])
}

fn exact_output_vertex_key(vertex: &OutputVertex) -> Option<ExactOutputVertexKey> {
    Some(ExactOutputVertexKey([
        vertex.x.exact_rational_ref()?.clone(),
        vertex.y.exact_rational_ref()?.clone(),
        vertex.z.exact_rational_ref()?.clone(),
    ]))
}

fn compare_output_vertices_lexicographic(
    left: &OutputVertex,
    right: &OutputVertex,
) -> HypermeshResult<std::cmp::Ordering> {
    let x = compare_real(&left.x, &right.x)?;
    if !x.is_eq() {
        return Ok(x);
    }
    let y = compare_real(&left.y, &right.y)?;
    if !y.is_eq() {
        return Ok(y);
    }
    compare_real(&left.z, &right.z)
}

struct ConstructionEdgeCandidates {
    groups: Vec<ConstructionEdgeCandidateGroup>,
    polygon_edges: Vec<Vec<usize>>,
}

struct ConstructionEdgeCandidateGroup {
    collinear: Vec<usize>,
    same_plane: Vec<usize>,
}

fn build_construction_edge_candidates(
    polygons: &[ConvexPolygon],
    indexed_polygons: &[Vec<usize>],
) -> HypermeshResult<ConstructionEdgeCandidates> {
    if polygons.len() != indexed_polygons.len() {
        return Err(HypermeshError::UnknownClassification);
    }
    let mut group_indices: StorageHashMap<ConstructionEdgeIdentity, usize> =
        StorageHashMap::default();
    let mut plane_vertices: StorageHashMap<crate::polygon::ConstructionPlaneIdentity, Vec<usize>> =
        StorageHashMap::default();
    let mut groups: Vec<ConstructionEdgeCandidateGroup> = Vec::new();
    let mut polygon_edges = Vec::with_capacity(polygons.len());
    for (polygon, indexed) in polygons.iter().zip(indexed_polygons) {
        let identities = polygon
            .known_edge_identities
            .as_ref()
            .ok_or(HypermeshError::UnknownClassification)?;
        if indexed.len() != identities.len() {
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
                        same_plane: Vec::new(),
                    });
                    entry.insert(index);
                    index
                }
            };
            groups[group_index].collinear.push(indexed[edge_index]);
            groups[group_index]
                .collinear
                .push(indexed[(edge_index + 1) % indexed.len()]);
            if let ConstructionEdgeIdentity::Split { planes } = identity {
                for plane in planes {
                    plane_vertices.entry(*plane).or_default().extend([
                        indexed[edge_index],
                        indexed[(edge_index + 1) % indexed.len()],
                    ]);
                }
            }
            edge_groups.push(group_index);
        }
        polygon_edges.push(edge_groups);
    }
    for (identity, &group_index) in &group_indices {
        if let ConstructionEdgeIdentity::Split { planes } = identity {
            for plane in planes {
                if let Some(vertices) = plane_vertices.get(plane) {
                    groups[group_index].same_plane.extend(vertices);
                }
            }
        }
    }
    for group in &mut groups {
        group.collinear.sort_unstable();
        group.collinear.dedup();
        group.same_plane.sort_unstable();
        group.same_plane.dedup();
    }
    Ok(ConstructionEdgeCandidates {
        groups,
        polygon_edges,
    })
}

fn polygon_edge_counts(
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
            for &canonical_subedge in split_segment_subedges_exact(
                &mut split_edge_cache,
                vertices,
                axis_order,
                canonical_edge,
            )? {
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
    cache: &'a mut SplitEdgeCache,
    vertices: &[OutputVertex],
    axis_order: &[Vec<usize>; 3],
    edge: [usize; 2],
) -> HypermeshResult<&'a [[usize; 2]]> {
    let edge = sorted_edge(edge);
    if let std::collections::hash_map::Entry::Vacant(e) = cache.entry(edge) {
        let axis = dominant_segment_axis(&vertices[edge[0]], &vertices[edge[1]])?;
        let bounds = exact_edge_bounds(edge, vertices)?;
        let mut on_edge = Vec::new();
        let (start, end) = candidate_vertex_index_range_for_edge(axis_order, vertices, edge, axis)?;
        for &vertex_index in &axis_order[axis][start..end] {
            if vertex_index == edge[0] || vertex_index == edge[1] {
                continue;
            }
            if point_within_edge_bounds_except_axis_exact(
                &vertices[vertex_index],
                &bounds,
                vertices,
                axis,
            )? && point_collinear_with_segment_exact(
                &vertices[vertex_index],
                &vertices[edge[0]],
                &vertices[edge[1]],
            )? {
                on_edge.push(vertex_index);
            }
        }

        let mut chain = Vec::with_capacity(on_edge.len() + 2);
        chain.push(edge[0]);
        chain.extend(sort_along_segment(&on_edge, edge[0], edge[1], vertices)?);
        chain.push(edge[1]);

        let subedges = chain
            .windows(2)
            .filter_map(|pair| (pair[0] != pair[1]).then_some([pair[0], pair[1]]))
            .collect();
        e.insert(subedges);
    }
    Ok(cache.get(&edge).expect("cached edge was just inserted"))
}

fn split_segment_subedges_exact_candidates<'a>(
    cache: &'a mut SplitEdgeCache,
    vertices: &[OutputVertex],
    edge: [usize; 2],
    candidates: &ConstructionEdgeCandidateGroup,
    filter_recovery_candidates: bool,
) -> HypermeshResult<&'a [[usize; 2]]> {
    let edge = sorted_edge(edge);
    if let std::collections::hash_map::Entry::Vacant(entry) = cache.entry(edge) {
        let axis = inexpensive_nonzero_segment_axis(&vertices[edge[0]], &vertices[edge[1]])?;
        let (min, max) = ordered_reals(
            vertex_axis(&vertices[edge[0]], axis),
            vertex_axis(&vertices[edge[1]], axis),
        )?;
        let projection_filters = filter_recovery_candidates
            .then(|| {
                (0..3)
                    .filter(|&other_axis| other_axis != axis)
                    .map(|other_axis| {
                        let start = [
                            vertex_axis(&vertices[edge[0]], axis).exact_rational_ref()?,
                            vertex_axis(&vertices[edge[0]], other_axis).exact_rational_ref()?,
                        ];
                        let end = [
                            vertex_axis(&vertices[edge[1]], axis).exact_rational_ref()?,
                            vertex_axis(&vertices[edge[1]], other_axis).exact_rational_ref()?,
                        ];
                        Some((other_axis, Real::prepare_rational_line2_filter(start, end)?))
                    })
                    .collect::<Option<Vec<(usize, PreparedRationalLine2Filter)>>>()
            })
            .flatten();
        let mut on_edge = Vec::new();
        for &vertex_index in &candidates.collinear {
            if vertex_index == edge[0] || vertex_index == edge[1] {
                continue;
            }
            let coordinate = vertex_axis(&vertices[vertex_index], axis);
            if !compare_real(coordinate, min)?.is_lt() && !compare_real(coordinate, max)?.is_gt() {
                on_edge.push(vertex_index);
            }
        }
        for &vertex_index in &candidates.same_plane {
            if vertex_index == edge[0] || vertex_index == edge[1] || on_edge.contains(&vertex_index)
            {
                continue;
            }
            if projection_filters.as_ref().is_some_and(|filters| {
                filters.iter().any(|(other_axis, filter)| {
                    let point = [
                        vertex_axis(&vertices[vertex_index], axis).exact_rational_ref(),
                        vertex_axis(&vertices[vertex_index], *other_axis).exact_rational_ref(),
                    ];
                    let [Some(first), Some(second)] = point else {
                        return false;
                    };
                    filter.sign_rational([first, second]).is_some()
                })
            }) {
                continue;
            }
            if point_on_segment_exact(
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
            &on_edge, edge[0], edge[1], vertices, axis,
        )?);
        chain.push(edge[1]);
        entry.insert(
            chain
                .windows(2)
                .filter_map(|pair| (pair[0] != pair[1]).then_some([pair[0], pair[1]]))
                .collect(),
        );
    }
    Ok(cache.get(&edge).expect("candidate edge was just cached"))
}

fn exact_output_vertices_f64(vertices: &[OutputVertex]) -> Option<Vec<[f64; 3]>> {
    vertices
        .iter()
        .map(|vertex| {
            let coordinates = [&vertex.x, &vertex.y, &vertex.z];
            if coordinates
                .iter()
                .any(|coordinate| coordinate.exact_rational_ref().is_none())
            {
                return None;
            }
            let point = coordinates
                .map(|coordinate| coordinate.to_f64_lossy())
                .map(|coordinate| coordinate.filter(|coordinate| coordinate.is_finite()));
            let [Some(x), Some(y), Some(z)] = point else {
                return None;
            };
            Some([x, y, z])
        })
        .collect()
}

fn split_segment_subedges_exact_precomputed_f64_scan<'a>(
    cache: &'a mut SplitEdgeCache,
    vertices: &[OutputVertex],
    approximate_vertices: &[[f64; 3]],
    edge: [usize; 2],
) -> HypermeshResult<&'a [[usize; 2]]> {
    let edge = sorted_edge(edge);
    if let std::collections::hash_map::Entry::Vacant(entry) = cache.entry(edge) {
        let start = approximate_vertices[edge[0]];
        let end = approximate_vertices[edge[1]];
        let mut on_edge = Vec::new();
        for (vertex_index, point) in approximate_vertices.iter().enumerate() {
            if vertex_index == edge[0] || vertex_index == edge[1] {
                continue;
            }
            if (0..3).all(|axis| {
                point[axis] >= start[axis].min(end[axis])
                    && point[axis] <= start[axis].max(end[axis])
            }) && point_on_segment_exact(
                &vertices[vertex_index],
                &vertices[edge[0]],
                &vertices[edge[1]],
            )? {
                on_edge.push(vertex_index);
            }
        }
        let mut chain = Vec::with_capacity(on_edge.len() + 2);
        chain.push(edge[0]);
        chain.extend(sort_along_segment(&on_edge, edge[0], edge[1], vertices)?);
        chain.push(edge[1]);
        entry.insert(
            chain
                .windows(2)
                .filter_map(|pair| (pair[0] != pair[1]).then_some([pair[0], pair[1]]))
                .collect(),
        );
    }
    Ok(cache.get(&edge).expect("scanned edge was just cached"))
}

fn sorted_vertex_indices_by_axis(vertices: &[OutputVertex]) -> HypermeshResult<[Vec<usize>; 3]> {
    let mut order = [
        (0..vertices.len()).collect::<Vec<_>>(),
        (0..vertices.len()).collect::<Vec<_>>(),
        (0..vertices.len()).collect::<Vec<_>>(),
    ];
    for (axis, axis_order) in order.iter_mut().enumerate() {
        axis_order.sort_by(|left, right| {
            compare_real(
                vertex_axis(&vertices[*left], axis),
                vertex_axis(&vertices[*right], axis),
            )
            .expect("exact vertex ordering should compare")
        });
    }
    Ok(order)
}

fn candidate_vertex_index_range_for_edge(
    axis_order: &[Vec<usize>; 3],
    vertices: &[OutputVertex],
    edge: [usize; 2],
    axis: usize,
) -> HypermeshResult<(usize, usize)> {
    let start_value = vertex_axis(&vertices[edge[0]], axis);
    let end_value = vertex_axis(&vertices[edge[1]], axis);
    let (min_value, max_value) = if compare_real(start_value, end_value)?.is_le() {
        (start_value, end_value)
    } else {
        (end_value, start_value)
    };

    let ordered = &axis_order[axis];
    let start = lower_bound_vertex_axis(ordered, vertices, axis, min_value)?;
    let end = upper_bound_vertex_axis(ordered, vertices, axis, max_value)?;
    Ok((start, end))
}

fn lower_bound_vertex_axis(
    ordered: &[usize],
    vertices: &[OutputVertex],
    axis: usize,
    value: &Real,
) -> HypermeshResult<usize> {
    let mut low = 0;
    let mut high = ordered.len();
    while low < high {
        let mid = (low + high) / 2;
        if compare_real(vertex_axis(&vertices[ordered[mid]], axis), value)?.is_lt() {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    Ok(low)
}

fn upper_bound_vertex_axis(
    ordered: &[usize],
    vertices: &[OutputVertex],
    axis: usize,
    value: &Real,
) -> HypermeshResult<usize> {
    let mut low = 0;
    let mut high = ordered.len();
    while low < high {
        let mid = (low + high) / 2;
        if compare_real(vertex_axis(&vertices[ordered[mid]], axis), value)?.is_gt() {
            high = mid;
        } else {
            low = mid + 1;
        }
    }
    Ok(low)
}

fn triangulate_polygons(
    polygons: &[ConvexPolygon],
    orientations: Option<&[i8]>,
) -> HypermeshResult<TriangleSoup> {
    if orientations.is_some_and(|orientations| orientations.len() != polygons.len()) {
        return Err(HypermeshError::UnknownClassification);
    }
    let mut soup = TriangleSoup::default();

    for (polygon_index, polygon) in polygons.iter().enumerate() {
        let vertex_count = polygon.vertex_count();
        if vertex_count < 3 {
            continue;
        }

        let base = soup.vertices.len();
        for point in polygon.vertices()? {
            soup.vertices.push(OutputVertex {
                x: point.x,
                y: point.y,
                z: point.z,
            });
        }

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
/// This pass deliberately uses no tolerance and no primitive floating-point
/// arithmetic. It only merges or splits when exact hyperreal predicates prove
/// equality, collinearity, and segment containment.
fn resolve_tjunctions(input: &TriangleSoup) -> HypermeshResult<TriangleSoup> {
    resolve_tjunctions_with_pass_limit(input, RESOLVE_TJUNCTION_MAX_PASSES)
}

fn resolve_tjunctions_with_pass_limit(
    input: &TriangleSoup,
    pass_limit: usize,
) -> HypermeshResult<TriangleSoup> {
    let mut soup = merge_duplicate_vertices(input);
    remove_degenerate_and_duplicate_triangles(&mut soup);

    let mut passes = 0;
    loop {
        if passes >= pass_limit {
            return Err(HypermeshError::OutputResolutionLimit { pass_limit });
        }
        let split_tjunction = split_one_tjunction_pass(&mut soup)?;
        let split_crossing = split_one_edge_crossing_pass(&mut soup)?;
        if !split_tjunction && !split_crossing {
            return Ok(soup);
        }
        passes += 1;
        remove_degenerate_and_duplicate_triangles(&mut soup);
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct OutputVertexBucket([Option<u64>; 3]);

fn merge_duplicate_vertices(input: &TriangleSoup) -> TriangleSoup {
    let mut vertices: Vec<OutputVertex> = Vec::with_capacity(input.vertices.len());
    let mut buckets: HashMap<OutputVertexBucket, Vec<usize>> = HashMap::new();
    let mut remap = vec![0; input.vertices.len()];

    for (index, vertex) in input.vertices.iter().enumerate() {
        let key = OutputVertexBucket([
            vertex.x.to_f64_lossy().map(f64::to_bits),
            vertex.y.to_f64_lossy().map(f64::to_bits),
            vertex.z.to_f64_lossy().map(f64::to_bits),
        ]);
        let candidates = buckets.entry(key).or_default();
        if let Some(existing) = candidates
            .iter()
            .copied()
            .find(|&candidate| vertices[candidate] == *vertex)
        {
            remap[index] = existing;
        } else {
            remap[index] = vertices.len();
            vertices.push(vertex.clone());
            candidates.push(vertices.len() - 1);
        }
    }

    let triangles = input
        .triangles
        .iter()
        .map(|triangle| [remap[triangle[0]], remap[triangle[1]], remap[triangle[2]]])
        .collect();

    TriangleSoup {
        vertices,
        triangles,
        sources: input.sources.clone(),
    }
}

fn remove_degenerate_and_duplicate_triangles(soup: &mut TriangleSoup) {
    let mut seen = BTreeSet::new();
    let mut triangles = Vec::with_capacity(soup.triangles.len());
    let mut sources = Vec::with_capacity(soup.sources.len());
    for (triangle, source) in soup.triangles.drain(..).zip(soup.sources.drain(..)) {
        if triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[0] == triangle[2] {
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
pub fn triangle_soup_is_closed(soup: &TriangleSoup) -> bool {
    triangle_soup_closure_report(soup).is_closed()
}

/// Counts exact singleton, directed-imbalance, and non-manifold edges in a
/// triangle soup.
pub fn triangle_soup_closure_report(soup: &TriangleSoup) -> TriangleSoupClosureReport {
    let mut report = TriangleSoupClosureReport::default();
    for uses in triangle_edge_counts(&soup.triangles).values().copied() {
        update_closure_report(&mut report, uses);
    }
    report
}

fn update_closure_report(report: &mut TriangleSoupClosureReport, uses: DirectedEdgeUses) {
    if uses.total() == 1 {
        report.boundary_edges += 1;
    } else if uses.total() > 2 {
        report.non_manifold_edges += 1;
    }
    if !uses.is_balanced() {
        report.unbalanced_edges += 1;
    }
}

fn split_one_tjunction_pass(soup: &mut TriangleSoup) -> HypermeshResult<bool> {
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
        let mut on_edge = Vec::new();
        for vertex_index in 0..soup.vertices.len() {
            if vertex_index == edge[0] || vertex_index == edge[1] {
                continue;
            }
            if point_on_segment_exact(
                &soup.vertices[vertex_index],
                &soup.vertices[edge[0]],
                &soup.vertices[edge[1]],
            )? {
                on_edge.push(vertex_index);
            }
        }
        if on_edge.is_empty() {
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

                let mut chain = Vec::with_capacity(on_edge.len() + 2);
                chain.push(ea);
                chain.extend(sort_along_segment(&on_edge, ea, eb, &soup.vertices)?);
                chain.push(eb);

                for pair in chain.windows(2) {
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

fn split_one_edge_crossing_pass(soup: &mut TriangleSoup) -> HypermeshResult<bool> {
    let mut edges = Vec::new();
    for triangle in &soup.triangles {
        for edge in triangle_edges(*triangle) {
            edges.push(sorted_edge(edge));
        }
    }
    edges.sort();
    edges.dedup();

    let mut bounded_edges = edges
        .into_iter()
        .map(|edge| exact_edge_bounds(edge, &soup.vertices))
        .collect::<HypermeshResult<Vec<_>>>()?;
    bounded_edges.sort_by(|left, right| {
        compare_real(
            vertex_axis(&soup.vertices[left.min[0]], 0),
            vertex_axis(&soup.vertices[right.min[0]], 0),
        )
        .expect("exact edge-bound ordering should compare")
        .then_with(|| left.edge.cmp(&right.edge))
    });

    for left_index in 0..bounded_edges.len() {
        let left = &bounded_edges[left_index];
        for right in &bounded_edges[(left_index + 1)..] {
            if compare_real(
                vertex_axis(&soup.vertices[right.min[0]], 0),
                vertex_axis(&soup.vertices[left.max[0]], 0),
            )?
            .is_gt()
            {
                break;
            }
            if !edge_bounds_overlap_exact(left, right, &soup.vertices)? {
                continue;
            }
            let left = left.edge;
            let right = right.edge;
            if left.iter().any(|vertex| right.contains(vertex)) {
                continue;
            }

            let Some(point) = proper_segment_intersection_after_bounds_overlap(
                &soup.vertices[left[0]],
                &soup.vertices[left[1]],
                &soup.vertices[right[0]],
                &soup.vertices[right[1]],
            )?
            else {
                continue;
            };

            let new_index = insert_or_find_vertex(soup, point);
            split_edges_at_vertex(soup, &[left, right], new_index);
            return Ok(true);
        }
    }

    Ok(false)
}

struct ExactEdgeBounds {
    edge: [usize; 2],
    min: [usize; 3],
    max: [usize; 3],
}

fn exact_edge_bounds(
    edge: [usize; 2],
    vertices: &[OutputVertex],
) -> HypermeshResult<ExactEdgeBounds> {
    let mut min = [edge[0]; 3];
    let mut max = [edge[1]; 3];
    for axis in 0..3 {
        if compare_real(
            vertex_axis(&vertices[edge[0]], axis),
            vertex_axis(&vertices[edge[1]], axis),
        )?
        .is_gt()
        {
            min[axis] = edge[1];
            max[axis] = edge[0];
        }
    }
    Ok(ExactEdgeBounds { edge, min, max })
}

fn edge_bounds_overlap_exact(
    left: &ExactEdgeBounds,
    right: &ExactEdgeBounds,
    vertices: &[OutputVertex],
) -> HypermeshResult<bool> {
    for axis in 1..3 {
        if compare_real(
            vertex_axis(&vertices[left.max[axis]], axis),
            vertex_axis(&vertices[right.min[axis]], axis),
        )?
        .is_lt()
            || compare_real(
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
    a: &OutputVertex,
    b: &OutputVertex,
    c: &OutputVertex,
    d: &OutputVertex,
) -> HypermeshResult<Option<OutputVertex>> {
    let ab = sub_vertex(b, a);
    let cd = sub_vertex(d, c);
    let normal = cross_arrays(&ab, &cd);
    if normal
        .iter()
        .all(|component| crate::geometry::classify_real(component) == Ok(Classification::On))
    {
        return Ok(None);
    }

    let ac = sub_vertex(c, a);
    if crate::geometry::classify_real(&dot_arrays(&ac, &normal))? != Classification::On {
        return Ok(None);
    }

    let projection_axis = dominant_component_axis(&normal)?;
    let (u_axis, v_axis) = match projection_axis {
        0 => (1, 2),
        1 => (0, 2),
        2 => (0, 1),
        _ => unreachable!("axis must be in 0..3"),
    };
    let denom = Real::signed_product_sum(
        [true, false],
        [[&ab[u_axis], &cd[v_axis]], [&ab[v_axis], &cd[u_axis]]],
    );
    if crate::geometry::classify_real(&denom)? == Classification::On {
        return Ok(None);
    }
    let t_num = Real::signed_product_sum(
        [true, false],
        [[&ac[u_axis], &cd[v_axis]], [&ac[v_axis], &cd[u_axis]]],
    );
    let t = (t_num / denom).map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
    let point = OutputVertex {
        x: &a.x + &(t.clone() * &ab[0]),
        y: &a.y + &(t.clone() * &ab[1]),
        z: &a.z + &(t * &ab[2]),
    };

    if point == *a || point == *b || point == *c || point == *d {
        return Ok(None);
    }
    if point_on_segment_exact(&point, a, b)? && point_on_segment_exact(&point, c, d)? {
        Ok(Some(point))
    } else {
        Ok(None)
    }
}

fn insert_or_find_vertex(soup: &mut TriangleSoup, vertex: OutputVertex) -> usize {
    if let Some(index) = soup
        .vertices
        .iter()
        .position(|existing| existing == &vertex)
    {
        index
    } else {
        let index = soup.vertices.len();
        soup.vertices.push(vertex);
        index
    }
}

fn split_edges_at_vertex(soup: &mut TriangleSoup, edges: &[[usize; 2]], vertex: usize) {
    let mut new_triangles = Vec::new();
    let mut new_sources = Vec::new();
    let mut kept = Vec::new();
    let mut kept_sources = Vec::new();
    for (face_index, triangle) in soup.triangles.iter().enumerate() {
        let mut split = false;
        for edge_index in 0..3 {
            let ea = triangle[edge_index];
            let eb = triangle[(edge_index + 1) % 3];
            let ec = triangle[(edge_index + 2) % 3];
            if edges.contains(&sorted_edge([ea, eb]))
                && vertex != ea
                && vertex != eb
                && vertex != ec
            {
                new_triangles.push([ea, vertex, ec]);
                new_triangles.push([vertex, eb, ec]);
                new_sources.push(soup.sources[face_index]);
                new_sources.push(soup.sources[face_index]);
                split = true;
                break;
            }
        }
        if !split {
            kept.push(*triangle);
            kept_sources.push(soup.sources[face_index]);
        }
    }
    kept.extend(new_triangles);
    kept_sources.extend(new_sources);
    soup.triangles = kept;
    soup.sources = kept_sources;
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
    point: &OutputVertex,
    start: &OutputVertex,
    end: &OutputVertex,
) -> HypermeshResult<bool> {
    if !point_within_segment_bounds_exact(point, start, end)? {
        return Ok(false);
    }
    point_collinear_with_segment_exact(point, start, end)
}

fn point_collinear_with_segment_exact(
    point: &OutputVertex,
    start: &OutputVertex,
    end: &OutputVertex,
) -> HypermeshResult<bool> {
    let ab = sub_vertex(end, start);
    let av = sub_vertex(point, start);
    let cross = cross_arrays(&ab, &av);
    if cross
        .iter()
        .any(|component| crate::geometry::classify_real(component) != Ok(Classification::On))
    {
        return Ok(false);
    }

    Ok(point != start && point != end)
}

fn point_within_edge_bounds_except_axis_exact(
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
        if compare_real(coordinate, vertex_axis(&vertices[bounds.min[axis]], axis))?.is_lt()
            || compare_real(coordinate, vertex_axis(&vertices[bounds.max[axis]], axis))?.is_gt()
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn point_within_segment_bounds_exact(
    point: &OutputVertex,
    start: &OutputVertex,
    end: &OutputVertex,
) -> HypermeshResult<bool> {
    for axis in 0..3 {
        let p = vertex_axis(point, axis);
        let a = vertex_axis(start, axis);
        let b = vertex_axis(end, axis);
        let (min, max) = ordered_reals(a, b)?;
        if compare_real(p, min)?.is_lt() || compare_real(p, max)?.is_gt() {
            return Ok(false);
        }
    }
    Ok(true)
}

fn ordered_reals<'a>(left: &'a Real, right: &'a Real) -> HypermeshResult<(&'a Real, &'a Real)> {
    if compare_real(left, right)?.is_le() {
        Ok((left, right))
    } else {
        Ok((right, left))
    }
}

fn sort_along_segment(
    indices: &[usize],
    start: usize,
    end: usize,
    vertices: &[OutputVertex],
) -> HypermeshResult<Vec<usize>> {
    let axis = dominant_segment_axis(&vertices[start], &vertices[end])?;
    sort_along_segment_on_axis(indices, start, end, vertices, axis)
}

fn sort_along_segment_on_axis(
    indices: &[usize],
    start: usize,
    end: usize,
    vertices: &[OutputVertex],
    axis: usize,
) -> HypermeshResult<Vec<usize>> {
    let ascending = compare_real(
        vertex_axis(&vertices[start], axis),
        vertex_axis(&vertices[end], axis),
    )?
    .is_lt();
    let mut sorted = Vec::new();

    for index in indices {
        let mut insert_at = sorted.len();
        for (position, existing) in sorted.iter().enumerate() {
            let order = compare_real(
                vertex_axis(&vertices[*index], axis),
                vertex_axis(&vertices[*existing], axis),
            )?;
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
    start: &OutputVertex,
    end: &OutputVertex,
) -> HypermeshResult<usize> {
    let approximate = (0..3)
        .map(|axis| {
            Some(
                (vertex_axis(end, axis).to_f64_lossy()?
                    - vertex_axis(start, axis).to_f64_lossy()?)
                .abs(),
            )
        })
        .collect::<Option<Vec<_>>>();
    if let Some(approximate) = approximate
        && let Some((axis, _)) = approximate
            .iter()
            .enumerate()
            .filter(|(_, delta)| delta.is_finite() && **delta != 0.0)
            .max_by(|(_, left), (_, right)| {
                left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal)
            })
    {
        return Ok(axis);
    }
    dominant_segment_axis(start, end)
}

fn dominant_segment_axis(start: &OutputVertex, end: &OutputVertex) -> HypermeshResult<usize> {
    let delta = sub_vertex(end, start);
    let abs = [
        delta[0].clone().abs(),
        delta[1].clone().abs(),
        delta[2].clone().abs(),
    ];
    let mut best = 0;
    for axis in 1..3 {
        if compare_real(&abs[axis], &abs[best])?.is_gt() {
            best = axis;
        }
    }
    Ok(best)
}

fn dominant_component_axis(values: &[Real; 3]) -> HypermeshResult<usize> {
    let abs = [
        values[0].clone().abs(),
        values[1].clone().abs(),
        values[2].clone().abs(),
    ];
    let mut best = 0;
    for axis in 1..3 {
        if compare_real(&abs[axis], &abs[best])?.is_gt() {
            best = axis;
        }
    }
    Ok(best)
}

fn certify_positive_signed_volume(soup: &TriangleSoup) -> HypermeshResult<()> {
    let volume = signed_volume_numerator(soup);
    if crate::geometry::classify_real(&volume)? != Classification::Positive {
        return Err(HypermeshError::UnknownClassification);
    }
    Ok(())
}

fn signed_volume_numerator(soup: &TriangleSoup) -> Real {
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

fn dot_arrays(left: &[Real; 3], right: &[Real; 3]) -> Real {
    Real::signed_product_sum(
        [true; 3],
        [
            [&left[0], &right[0]],
            [&left[1], &right[1]],
            [&left[2], &right[2]],
        ],
    )
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
    use crate::polygon::make_triangle;
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

    fn op(vertices: Vec<OutputVertex>) -> OutputPolygon {
        OutputPolygon {
            vertices,
            source_mesh: 0,
            source_polygon: 0,
        }
    }

    fn positive_tetra_soup() -> TriangleSoup {
        TriangleSoup {
            vertices: vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 1, 0), ov(0, 0, 1)],
            triangles: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
            sources: vec![TriangleSource::default(); 4],
        }
    }

    #[test]
    fn internal_resolution_merges_duplicate_vertices_and_faces_exactly() {
        let soup = TriangleSoup {
            vertices: vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 1, 0), ov(1, 0, 0)],
            triangles: vec![[0, 1, 2], [0, 3, 2]],
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
            ],
        };

        let resolved = resolve_tjunctions(&soup).unwrap();

        assert_eq!(resolved.vertices.len(), 3);
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
        let soup = TriangleSoup {
            vertices: vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0), ov(1, 0, 0)],
            triangles: vec![[0, 1, 2]],
            sources: vec![TriangleSource {
                mesh: 1,
                triangle: 7,
                orientation: 0,
            }],
        };

        let resolved = resolve_tjunctions(&soup).unwrap();

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
    fn internal_resolution_reports_pass_limit_exhaustion() {
        let soup = TriangleSoup {
            vertices: vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0), ov(1, 0, 0)],
            triangles: vec![[0, 1, 2]],
            sources: vec![TriangleSource::default()],
        };

        let err = resolve_tjunctions_with_pass_limit(&soup, 1).unwrap_err();

        assert_eq!(err, HypermeshError::OutputResolutionLimit { pass_limit: 1 });
    }

    #[test]
    fn internal_resolution_accepts_budget_covering_split_and_certification_passes() {
        let soup = TriangleSoup {
            vertices: vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0), ov(1, 0, 0)],
            triangles: vec![[0, 1, 2]],
            sources: vec![TriangleSource::default()],
        };

        let resolved = resolve_tjunctions_with_pass_limit(&soup, 2).unwrap();

        assert_eq!(resolved.triangles.len(), 2);
    }

    #[test]
    fn output_extraction_uses_real_vertices() {
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let result = BooleanResult::new(
            PolygonSoup {
                polygons: vec![polygon],
                bounds: Aabb::new(p(0, 0, 0), p(1, 1, 0)),
                num_meshes: 1,
            },
            vec![1],
        );

        let polygons = extract_output(&result).unwrap();
        assert_eq!(polygons.len(), 1);
        assert_eq!(polygons[0].vertices.len(), 3);
        assert!(polygons[0].vertices.iter().any(|vertex| vertex.x == r(1)));
    }

    #[test]
    fn output_polygon_closure_report_accepts_closed_tetrahedron() {
        let polygons = vec![
            op(vec![ov(0, 0, 0), ov(0, 1, 0), ov(1, 0, 0)]),
            op(vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 0, 1)]),
            op(vec![ov(0, 0, 0), ov(0, 0, 1), ov(0, 1, 0)]),
            op(vec![ov(1, 0, 0), ov(0, 1, 0), ov(0, 0, 1)]),
        ];

        let report = output_polygon_closure_report(&polygons).unwrap();

        assert_eq!(
            report,
            TriangleSoupClosureReport {
                boundary_edges: 0,
                unbalanced_edges: 0,
                non_manifold_edges: 0,
            }
        );
    }

    #[test]
    fn output_polygon_closure_report_rejects_reversed_tetrahedron_face() {
        let mut polygons = vec![
            op(vec![ov(0, 0, 0), ov(0, 1, 0), ov(1, 0, 0)]),
            op(vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 0, 1)]),
            op(vec![ov(0, 0, 0), ov(0, 0, 1), ov(0, 1, 0)]),
            op(vec![ov(1, 0, 0), ov(0, 1, 0), ov(0, 0, 1)]),
        ];
        polygons[0].vertices.swap(1, 2);

        let report = output_polygon_closure_report(&polygons).unwrap();

        assert_eq!(report.boundary_edges, 0);
        assert_eq!(report.unbalanced_edges, 3);
        assert_eq!(report.non_manifold_edges, 0);
        assert!(!report.has_no_boundary());
    }

    #[test]
    fn output_polygon_closure_report_accepts_balanced_non_manifold_multiplicity() {
        let mut polygons = vec![
            op(vec![ov(0, 0, 0), ov(0, 1, 0), ov(1, 0, 0)]),
            op(vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 0, 1)]),
            op(vec![ov(0, 0, 0), ov(0, 0, 1), ov(0, 1, 0)]),
            op(vec![ov(1, 0, 0), ov(0, 1, 0), ov(0, 0, 1)]),
        ];
        polygons.extend(polygons.clone());

        let report = output_polygon_closure_report(&polygons).unwrap();

        assert_eq!(report.boundary_edges, 0);
        assert_eq!(report.unbalanced_edges, 0);
        assert_eq!(report.non_manifold_edges, 6);
        assert!(report.has_no_boundary());
        assert!(!report.is_closed());
    }

    #[test]
    fn triangle_soup_closure_report_requires_directed_balance() {
        let mut reversed_face = positive_tetra_soup();
        reversed_face.triangles[0].swap(1, 2);
        let reversed_report = triangle_soup_closure_report(&reversed_face);

        assert_eq!(reversed_report.boundary_edges, 0);
        assert_eq!(reversed_report.unbalanced_edges, 3);
        assert_eq!(reversed_report.non_manifold_edges, 0);
        assert!(!reversed_report.has_no_boundary());

        let mut doubled = positive_tetra_soup();
        doubled.triangles.extend(doubled.triangles.clone());
        let doubled_report = triangle_soup_closure_report(&doubled);

        assert_eq!(doubled_report.boundary_edges, 0);
        assert_eq!(doubled_report.unbalanced_edges, 0);
        assert_eq!(doubled_report.non_manifold_edges, 6);
        assert!(doubled_report.has_no_boundary());
    }

    #[test]
    fn merge_duplicate_polygon_vertices_reuses_exact_vertex_keys() {
        let polygons = vec![
            op(vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0)]),
            op(vec![ov(2, 0, 0), ov(0, 0, 0), ov(0, -1, 0)]),
        ];

        let (vertices, indexed) = merge_duplicate_polygon_vertices(&polygons);

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
        let (vertices, indexed) = merge_duplicate_polygon_vertices(&polygons);
        let axis_order = sorted_vertex_indices_by_axis(&vertices).unwrap();
        let counts = polygon_edge_counts(&vertices, &indexed, &axis_order).unwrap();

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
    fn split_segment_subedges_exact_reuses_undirected_edge_cache() {
        let polygons = vec![
            op(vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0)]),
            op(vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, -1, 0)]),
            op(vec![ov(2, 0, 0), ov(0, 0, 0), ov(2, -1, 0)]),
        ];
        let (vertices, _indexed) = merge_duplicate_polygon_vertices(&polygons);
        let axis_order = sorted_vertex_indices_by_axis(&vertices).unwrap();
        let mut cache = SplitEdgeCache::default();

        let forward = split_segment_subedges_exact(&mut cache, &vertices, &axis_order, [0, 1])
            .unwrap()
            .to_vec();
        let reversed =
            split_segment_subedges_exact(&mut cache, &vertices, &axis_order, [1, 0]).unwrap();

        assert_eq!(forward, vec![[0, 3], [3, 1]]);
        assert_eq!(reversed, forward);
        assert_eq!(cache.len(), 1);
    }

    #[test]
    fn candidate_vertex_index_range_for_edge_matches_full_vertex_scan() {
        let polygons = vec![
            op(vec![ov(0, 0, 0), ov(3, 0, 0), ov(0, 2, 0)]),
            op(vec![ov(1, 0, 0), ov(2, 0, 0), ov(1, -1, 0)]),
            op(vec![ov(3, 0, 0), ov(0, 0, 0), ov(3, -1, 0)]),
            op(vec![ov(0, 1, 0), ov(0, 3, 0), ov(-1, 1, 0)]),
        ];
        let (vertices, _indexed) = merge_duplicate_polygon_vertices(&polygons);
        let axis_order = sorted_vertex_indices_by_axis(&vertices).unwrap();
        let edge = [0, 1];
        let axis = dominant_segment_axis(&vertices[edge[0]], &vertices[edge[1]]).unwrap();

        let (start, end) =
            candidate_vertex_index_range_for_edge(&axis_order, &vertices, edge, axis).unwrap();
        let filtered = axis_order[axis][start..end].to_vec();
        let full_scan = (0..vertices.len()).collect::<Vec<_>>();

        let filtered_on_edge = filtered
            .into_iter()
            .filter(|index| {
                *index != edge[0]
                    && *index != edge[1]
                    && point_on_segment_exact(
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
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let result = BooleanResult::new(
            PolygonSoup {
                polygons: vec![polygon.clone(), polygon],
                bounds: Aabb::new(p(0, 0, 0), p(1, 1, 0)),
                num_meshes: 1,
            },
            vec![1, 1],
        );

        let err = certify_output_polygon_closure(&result).unwrap_err();
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
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
        let result = BooleanResult::new(
            PolygonSoup {
                polygons: vec![polygon],
                bounds: Aabb::new(p(0, 0, 0), p(1, 1, 0)),
                num_meshes: 1,
            },
            vec![1],
        );

        let err = triangulate_and_resolve_certified(&result).unwrap_err();
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
        let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
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
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        );
        first.winding = Some(WindingPair {
            w_front: vec![0],
            w_back: vec![1],
        });
        let second = ClassifiedPolygon::new(
            make_triangle(&p(1, 0, 0), &p(0, 1, 0), &p(0, 0, 0), 1, 7),
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
            make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0),
            1,
        );
        let second = ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 1),
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
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        );
        let mut second = ClassifiedPolygon::new(
            make_triangle(&p(1, 0, 0), &p(0, 1, 0), &p(0, 0, 0), 1, 3),
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
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        )];
        let mut duplicate = ClassifiedPolygon::new(
            make_triangle(&p(1, 0, 0), &p(0, 1, 0), &p(0, 0, 0), 1, 4),
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
            make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0),
            1,
        )];
        let incoming = vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 1),
            1,
        )];

        merge_unique_classified_polygons(&mut output, incoming);

        assert_eq!(output.len(), 2);
    }

    #[test]
    fn certified_triangulation_rejects_open_surface_after_boundary_tjunction_cleanup() {
        let lower = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
        let upper = make_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1);
        let result = BooleanResult::new(
            PolygonSoup {
                polygons: vec![lower, upper],
                bounds: Aabb::new(p(0, 0, 0), p(2, 2, 0)),
                num_meshes: 1,
            },
            vec![1, 1],
        );

        let err = triangulate_and_resolve_certified(&result).unwrap_err();
        assert!(matches!(err, HypermeshError::OpenOutput { .. }));
    }

    #[test]
    fn signed_volume_certification_accepts_only_positive_orientation() {
        let positive = positive_tetra_soup();
        certify_positive_signed_volume(&positive).unwrap();

        let mut reversed = positive.clone();
        for triangle in &mut reversed.triangles {
            triangle.swap(0, 1);
        }
        assert_eq!(
            certify_positive_signed_volume(&reversed),
            Err(HypermeshError::UnknownClassification)
        );

        let flat = TriangleSoup {
            vertices: vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 1, 0)],
            triangles: vec![[0, 1, 2]],
            sources: vec![TriangleSource::default()],
        };
        assert_eq!(
            certify_positive_signed_volume(&flat),
            Err(HypermeshError::UnknownClassification)
        );
    }
}
