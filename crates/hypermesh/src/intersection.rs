//! Pairwise convex polygon intersection primitives.

use hyperlattice::{
    HomogeneousPoint3, Point3, Real, intersect_homogeneous_line_plane, intersect_two_planes,
};

use crate::bvh::ExactBvh;
use crate::clip::{ClipSide, clip_polygon_decision};
use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, compare_real_decision};
use crate::point_interner::PointInterner;
use crate::polygon::ConvexPolygon;
use crate::predicate::{
    classify_point_decision, classify_projective_point_decision, classify_real,
};

/// Intersection segment between two polygons.
#[derive(Clone, Debug, PartialEq)]
pub struct IntersectionSegment {
    /// First segment endpoint.
    pub v0: Point3,
    /// Second segment endpoint.
    pub v1: Point3,
    /// Local index of the other polygon.
    pub other_polygon_idx: usize,
}

/// Coplanar overlap information.
#[derive(Clone, Debug, PartialEq)]
pub struct OverlapInfo {
    /// Local index of the other polygon.
    pub other_polygon_idx: usize,
}

/// Type of pairwise polygon intersection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PairwiseIntersectionType {
    /// No intersection.
    None,
    /// Single point intersection.
    Point,
    /// Non-degenerate segment intersection.
    Segment,
    /// Coplanar non-empty overlap.
    Overlap,
}

/// Result of intersecting two convex polygons.
#[derive(Clone, Debug, PartialEq)]
pub struct PairwiseIntersection {
    /// Intersection kind.
    pub kind: PairwiseIntersectionType,
    /// Segment payload when `kind == Segment`.
    pub segment: Option<IntersectionSegment>,
    /// Overlap payload when `kind == Overlap`.
    pub overlap: Option<OverlapInfo>,
}

impl PairwiseIntersection {
    /// Creates a no-intersection result.
    pub const fn none() -> Self {
        Self {
            kind: PairwiseIntersectionType::None,
            segment: None,
            overlap: None,
        }
    }

    /// Creates a point-intersection result.
    pub const fn point() -> Self {
        Self {
            kind: PairwiseIntersectionType::Point,
            segment: None,
            overlap: None,
        }
    }
}

const NO_INTERSECTION_NODE: u32 = u32::MAX;
const NO_INTERSECTION_SEGMENT: u32 = u32::MAX;

#[derive(Clone, Debug, PartialEq)]
struct PairwiseIntersectionNode {
    next: u32,
    other_polygon: u32,
    segment: u32,
}

#[derive(Clone, Debug, PartialEq)]
struct PairwiseIntersectionSegment {
    endpoints: [u32; 2],
}

#[derive(Clone, Copy)]
pub(crate) enum PairwiseIntersectionEventRef<'a> {
    Segment {
        segment: PairwiseIntersectionSegmentRef<'a>,
        other_polygon_idx: usize,
    },
    Overlap {
        other_polygon_idx: usize,
    },
}

#[derive(Clone, Copy)]
pub(crate) struct PairwiseIntersectionSegmentRef<'a> {
    pub(crate) v0: &'a Point3,
    pub(crate) v1: &'a Point3,
}

/// Compact face-indexed intersection adjacency backed by one node arena.
///
/// Empty faces cost eight bytes rather than a separately allocated `Vec`
/// header. The two directed events for a non-coplanar cut share one endpoint
/// record. Events are appended directly from the BVH stream and retain their
/// deterministic discovery order without a global candidate-pair buffer.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PairwiseIntersectionGraph {
    heads: Box<[u32]>,
    counts: Box<[u32]>,
    points: Vec<Point3>,
    segments: Vec<PairwiseIntersectionSegment>,
    nodes: Vec<PairwiseIntersectionNode>,
}

impl PairwiseIntersectionGraph {
    pub(crate) fn len(&self) -> usize {
        self.heads.len()
    }

    pub(crate) fn event_count(&self) -> usize {
        self.nodes.len()
    }

    pub(crate) fn row(&self, face: usize) -> PairwiseIntersectionRow<'_> {
        debug_assert!(face < self.len());
        let next = self
            .heads
            .get(face)
            .copied()
            .unwrap_or(NO_INTERSECTION_NODE);
        let remaining = self.counts.get(face).copied().unwrap_or(0);
        PairwiseIntersectionRow {
            graph: self,
            next,
            remaining,
        }
    }

    pub(crate) fn iter(&self) -> impl ExactSizeIterator<Item = PairwiseIntersectionRow<'_>> + '_ {
        (0..self.len()).map(|face| self.row(face))
    }

    pub(crate) fn remap_polygon_order(&self, query_to_cached: &[usize]) -> HypermeshResult<Self> {
        if self.len() != query_to_cached.len() {
            return Err(HypermeshError::UnknownClassification);
        }
        let mut cached_to_query = vec![usize::MAX; query_to_cached.len()];
        for (query_index, &cached_index) in query_to_cached.iter().enumerate() {
            if cached_index >= cached_to_query.len() || cached_to_query[cached_index] != usize::MAX
            {
                return Err(HypermeshError::UnknownClassification);
            }
            cached_to_query[cached_index] = query_index;
        }

        let mut remapped = PairwiseIntersectionGraphBuilder::new(query_to_cached.len());
        remapped
            .points
            .try_reserve(self.points.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection point remapping",
            })?;
        remapped
            .point_interner
            .register_unindexed_existing(self.points.len())?;
        remapped.points.extend(self.points.iter().cloned());
        remapped
            .segments
            .try_reserve(self.segments.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection segment remapping",
            })?;
        remapped.segments.extend(self.segments.iter().cloned());
        for segment in &remapped.segments {
            if segment
                .endpoints
                .iter()
                .any(|&point| remapped.points.get(point as usize).is_none())
            {
                return Err(HypermeshError::UnknownClassification);
            }
        }
        remapped.reserve_nodes(self.nodes.len())?;
        for (query_index, &cached_index) in query_to_cached.iter().enumerate() {
            let mut node = self.heads[cached_index];
            while node != NO_INTERSECTION_NODE {
                let entry = self
                    .nodes
                    .get(node as usize)
                    .ok_or(HypermeshError::UnknownClassification)?;
                if entry.segment != NO_INTERSECTION_SEGMENT
                    && self.segments.get(entry.segment as usize).is_none()
                {
                    return Err(HypermeshError::UnknownClassification);
                }
                let other_polygon = remapped_face_id(&cached_to_query, entry.other_polygon)?;
                remapped.append(query_index, other_polygon, entry.segment)?;
                node = entry.next;
            }
        }
        Ok(remapped.finish())
    }
}

fn remapped_face_id(cached_to_query: &[usize], cached: u32) -> HypermeshResult<u32> {
    let query = cached_to_query
        .get(cached as usize)
        .copied()
        .filter(|&query| query != usize::MAX)
        .ok_or(HypermeshError::UnknownClassification)?;
    u32::try_from(query).map_err(|_| HypermeshError::CapacityOverflow {
        operation: "pairwise intersection face remapping",
    })
}

#[derive(Clone, Copy)]
pub(crate) struct PairwiseIntersectionRow<'a> {
    graph: &'a PairwiseIntersectionGraph,
    next: u32,
    remaining: u32,
}

impl PairwiseIntersectionRow<'_> {
    pub(crate) fn is_empty(&self) -> bool {
        self.remaining == 0
    }

    pub(crate) fn iter(&self) -> Self {
        *self
    }
}

impl<'a> Iterator for PairwiseIntersectionRow<'a> {
    type Item = PairwiseIntersectionEventRef<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.next == NO_INTERSECTION_NODE {
            debug_assert_eq!(self.remaining, 0);
            return None;
        }
        let node = &self.graph.nodes[self.next as usize];
        self.next = node.next;
        self.remaining -= 1;
        Some(if node.segment != NO_INTERSECTION_SEGMENT {
            let segment = &self.graph.segments[node.segment as usize];
            PairwiseIntersectionEventRef::Segment {
                segment: PairwiseIntersectionSegmentRef {
                    v0: &self.graph.points[segment.endpoints[0] as usize],
                    v1: &self.graph.points[segment.endpoints[1] as usize],
                },
                other_polygon_idx: node.other_polygon as usize,
            }
        } else {
            PairwiseIntersectionEventRef::Overlap {
                other_polygon_idx: node.other_polygon as usize,
            }
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.remaining as usize;
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for PairwiseIntersectionRow<'_> {}

pub(crate) struct PairwiseIntersectionGraphBuilder {
    heads: Box<[u32]>,
    tails: Box<[u32]>,
    counts: Box<[u32]>,
    points: Vec<Point3>,
    point_interner: PointInterner<()>,
    segments: Vec<PairwiseIntersectionSegment>,
    nodes: Vec<PairwiseIntersectionNode>,
}

impl PairwiseIntersectionGraphBuilder {
    pub(crate) fn new(face_count: usize) -> Self {
        Self {
            heads: vec![NO_INTERSECTION_NODE; face_count].into_boxed_slice(),
            tails: vec![NO_INTERSECTION_NODE; face_count].into_boxed_slice(),
            counts: vec![0; face_count].into_boxed_slice(),
            points: Vec::new(),
            point_interner: PointInterner::new_exact_unreserved(),
            segments: Vec::new(),
            nodes: Vec::new(),
        }
    }

    fn face_id(&self, face: usize) -> HypermeshResult<u32> {
        if face >= self.heads.len() {
            return Err(HypermeshError::CapacityOverflow {
                operation: "pairwise intersection graph face index",
            });
        }
        u32::try_from(face).map_err(|_| HypermeshError::CapacityOverflow {
            operation: "pairwise intersection face ID",
        })
    }

    fn reserve_nodes(&mut self, additional: usize) -> HypermeshResult<()> {
        let new_len = self
            .nodes
            .len()
            .checked_add(additional)
            .filter(|&len| len <= NO_INTERSECTION_NODE as usize)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "pairwise intersection graph",
            })?;
        debug_assert!(new_len >= self.nodes.len());
        self.nodes
            .try_reserve(additional)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection graph",
            })
    }

    fn reserve_segments(&mut self, additional: usize) -> HypermeshResult<()> {
        self.segments
            .len()
            .checked_add(additional)
            .filter(|&len| len <= NO_INTERSECTION_NODE as usize)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "pairwise intersection segment arena",
            })?;
        self.segments
            .try_reserve(additional)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection segment arena",
            })
    }

    fn check_row_capacity(&self, face: usize, additional: u32) -> HypermeshResult<()> {
        self.counts[face]
            .checked_add(additional)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "pairwise intersection graph row",
            })?;
        Ok(())
    }

    fn append_prechecked(&mut self, face: usize, other_polygon: u32, segment: u32) {
        let node_index = self.nodes.len() as u32;
        let head = &mut self.heads[face];
        let tail = &mut self.tails[face];
        self.counts[face] += 1;
        self.nodes.push(PairwiseIntersectionNode {
            next: NO_INTERSECTION_NODE,
            other_polygon,
            segment,
        });
        if *tail == NO_INTERSECTION_NODE {
            *head = node_index;
        } else {
            self.nodes[*tail as usize].next = node_index;
        }
        *tail = node_index;
    }

    fn append(&mut self, face: usize, other_polygon: u32, segment: u32) -> HypermeshResult<()> {
        self.face_id(face)?;
        self.check_row_capacity(face, 1)?;
        self.reserve_nodes(1)?;
        self.append_prechecked(face, other_polygon, segment);
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn append_overlap(
        &mut self,
        face: usize,
        other_polygon_idx: usize,
    ) -> HypermeshResult<()> {
        let other_polygon = self.face_id(other_polygon_idx)?;
        self.append(face, other_polygon, NO_INTERSECTION_SEGMENT)
    }

    pub(crate) fn append_segment_pair(
        &mut self,
        left: usize,
        right: usize,
        v0: Point3,
        v1: Point3,
    ) -> HypermeshResult<()> {
        if left == right {
            return Err(HypermeshError::UnknownClassification);
        }
        let left_id = self.face_id(left)?;
        let right_id = self.face_id(right)?;
        self.check_row_capacity(left, 1)?;
        self.check_row_capacity(right, 1)?;
        self.reserve_segments(1)?;
        self.reserve_nodes(2)?;
        self.points
            .len()
            .checked_add(2)
            .filter(|&len| len <= u32::MAX as usize)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "pairwise intersection point arena",
            })?;
        let endpoints = self
            .point_interner
            .intern_exact_pair_or_append(&mut self.points, [v0, v1])?;
        debug_assert!(endpoints.iter().all(|&point| point < u32::MAX as usize));
        let endpoints = endpoints.map(|point| {
            u32::try_from(point).expect("intersection point capacity was checked before insertion")
        });
        let segment_id = self.segments.len() as u32;
        self.segments
            .push(PairwiseIntersectionSegment { endpoints });
        self.append_prechecked(left, right_id, segment_id);
        self.append_prechecked(right, left_id, segment_id);
        Ok(())
    }

    pub(crate) fn append_overlap_pair(&mut self, left: usize, right: usize) -> HypermeshResult<()> {
        if left == right {
            return Err(HypermeshError::UnknownClassification);
        }
        let left_id = self.face_id(left)?;
        let right_id = self.face_id(right)?;
        self.check_row_capacity(left, 1)?;
        self.check_row_capacity(right, 1)?;
        self.reserve_nodes(2)?;
        self.append_prechecked(left, right_id, NO_INTERSECTION_SEGMENT);
        self.append_prechecked(right, left_id, NO_INTERSECTION_SEGMENT);
        Ok(())
    }

    pub(crate) fn finish(self) -> PairwiseIntersectionGraph {
        PairwiseIntersectionGraph {
            heads: self.heads,
            counts: self.counts,
            points: self.points,
            segments: self.segments,
            nodes: self.nodes,
        }
    }
}

/// Computes the pairwise intersection between two convex polygons.
pub fn intersect_polygons(
    context: &MeshContext,
    polygon: &ConvexPolygon,
    other: &ConvexPolygon,
    other_polygon_idx: usize,
) -> HypermeshResult<MeshOutcome<PairwiseIntersection>> {
    let decisions = DecisionContext::new(context);
    let polygon_vertices = polygon.vertices_decision(&decisions)?;
    let other_vertices = other.vertices_decision(&decisions)?;
    let intersection = intersect_polygons_with_vertices(
        &decisions,
        polygon,
        &polygon_vertices,
        other,
        &other_vertices,
        other_polygon_idx,
    )?;
    Ok(decisions.finish(intersection))
}

/// Computes a pairwise intersection from affine vertices already materialized
/// for both polygons. Subdivision compares each polygon with many candidates,
/// so retaining these exact points at that boundary avoids repeatedly solving
/// the same adjacent plane triples.
pub(crate) fn intersect_polygons_with_vertices(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    polygon_vertices: &[Point3],
    other: &ConvexPolygon,
    other_vertices: &[Point3],
    other_polygon_idx: usize,
) -> HypermeshResult<PairwiseIntersection> {
    if polygon.vertex_count() == 0 || other.vertex_count() == 0 {
        return Ok(PairwiseIntersection::none());
    }

    let supports_parallel = supports_are_parallel(decisions, &polygon.support, &other.support)?;
    if supports_parallel {
        crate::trace_dispatch!("intersect-polygons", "parallel-supports");
        let other_vertex = other_vertices
            .first()
            .ok_or(HypermeshError::UnknownClassification)?;
        return if classify_point_decision(decisions, other_vertex, &polygon.support)?
            == Classification::On
        {
            intersect_coplanar(decisions, polygon, other, other_polygon_idx)
        } else {
            Ok(PairwiseIntersection::none())
        };
    }

    let mut points = Vec::new();
    crate::trace_dispatch!("intersect-polygons", "edge-crossings-forward");
    collect_edge_plane_crossings(decisions, polygon, polygon_vertices, other, &mut points)?;
    crate::trace_dispatch!("intersect-polygons", "edge-crossings-reverse");
    collect_edge_plane_crossings(decisions, other, other_vertices, polygon, &mut points)?;
    dedup_points(decisions, &mut points)?;

    match points.len() {
        0 => Ok(PairwiseIntersection::none()),
        1 => Ok(PairwiseIntersection::point()),
        _ => Ok(PairwiseIntersection {
            kind: PairwiseIntersectionType::Segment,
            segment: Some(IntersectionSegment {
                v0: points[0].clone(),
                v1: points[1].clone(),
                other_polygon_idx,
            }),
            overlap: None,
        }),
    }
}

/// Builds exact symmetric intersection rows for one polygon arrangement.
///
/// The BVH callback is consumed directly so the broad phase never materializes
/// a global candidate-pair vector. Rows remain in deterministic polygon order
/// and contain only segment or positive-area overlap events that can change the
/// arrangement.
pub(crate) fn pairwise_intersections_by_polygon_with_certified_embedded_inputs(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    certified_embedded_inputs: &[bool],
) -> HypermeshResult<PairwiseIntersectionGraph> {
    let mut graph = PairwiseIntersectionGraphBuilder::new(polygons.len());
    let bvh = ExactBvh::build_decision(decisions, polygons)?;
    let vertices = polygons
        .iter()
        .map(|polygon| polygon.vertices_decision(decisions))
        .collect::<HypermeshResult<Vec<_>>>()?;
    let mut failure = None;

    bvh.intersect_pairs_decision(decisions, &bvh, |global_i, global_j| {
        if global_i >= global_j || failure.is_some() {
            return;
        }
        if let Err(error) = append_pairwise_intersection(
            decisions,
            polygons,
            &vertices,
            certified_embedded_inputs,
            &mut graph,
            global_i,
            global_j,
        ) {
            failure = Some(error);
        }
    })?;
    if let Some(error) = failure {
        return Err(error);
    }
    Ok(graph.finish())
}

#[cfg(test)]
pub(crate) fn pairwise_intersections_by_polygon(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
) -> HypermeshResult<PairwiseIntersectionGraph> {
    pairwise_intersections_by_polygon_with_certified_embedded_inputs(decisions, polygons, &[])
}

fn append_pairwise_intersection(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    vertices: &[Vec<Point3>],
    certified_embedded_inputs: &[bool],
    graph: &mut PairwiseIntersectionGraphBuilder,
    global_i: usize,
    global_j: usize,
) -> HypermeshResult<()> {
    crate::trace_dispatch!("pairwise-intersection", "bvh-candidate");
    if polygons[global_i].mesh_index == polygons[global_j].mesh_index
        && usize::try_from(polygons[global_i].mesh_index)
            .ok()
            .and_then(|mesh_index| certified_embedded_inputs.get(mesh_index))
            .copied()
            .unwrap_or(false)
    {
        crate::trace_dispatch!("pairwise-intersection", "certified-embedded-input");
        return Ok(());
    }
    let same_mesh = polygons[global_i].mesh_index == polygons[global_j].mesh_index;
    let shares_manifold_edge = if same_mesh {
        polygon_cycles_share_reversed_noncoplanar_triangle_edge(
            decisions,
            &vertices[global_i],
            &polygons[global_i].support,
            &vertices[global_j],
            &polygons[global_j].support,
        )?
    } else {
        false
    };
    if same_mesh && shares_manifold_edge {
        crate::trace_dispatch!("pairwise-intersection", "known-manifold-edge");
        return Ok(());
    }
    crate::trace_dispatch!(
        "pairwise-intersection",
        if same_mesh {
            "same-mesh-polygon-test"
        } else {
            "cross-mesh-polygon-test"
        }
    );
    let intersection = intersect_polygons_with_vertices(
        decisions,
        &polygons[global_i],
        &vertices[global_i],
        &polygons[global_j],
        &vertices[global_j],
        global_j,
    )
    .inspect_err(|_error| {
        crate::trace_dispatch!("pairwise-intersection", "polygon-test-failed");
        if cfg!(debug_assertions) {
            eprintln!(
                "[DEBUG] pairwise failure: left={global_i}/mesh{} right={global_j}/mesh{}",
                polygons[global_i].mesh_index, polygons[global_j].mesh_index,
            );
        }
    })?;
    if !matches!(
        intersection.kind,
        PairwiseIntersectionType::Segment | PairwiseIntersectionType::Overlap
    ) {
        return Ok(());
    }
    if same_mesh
        && intersection.kind == PairwiseIntersectionType::Segment
        && let Some(segment) = intersection.segment.as_ref()
        && !segment_has_strict_interior_point_in_both(
            decisions,
            &segment.v0,
            &segment.v1,
            &polygons[global_i],
            &polygons[global_j],
        )?
    {
        crate::trace_dispatch!("pairwise-intersection", "same-mesh-boundary-only");
        return Ok(());
    }
    crate::trace_dispatch!("pairwise-intersection", "nonempty-cut");
    match intersection.kind {
        PairwiseIntersectionType::Segment => {
            let segment = intersection
                .segment
                .ok_or(HypermeshError::UnknownClassification)?;
            graph.append_segment_pair(global_i, global_j, segment.v0, segment.v1)
        }
        PairwiseIntersectionType::Overlap => {
            intersection
                .overlap
                .ok_or(HypermeshError::UnknownClassification)?;
            graph.append_overlap_pair(global_i, global_j)
        }
        PairwiseIntersectionType::None | PairwiseIntersectionType::Point => {
            Err(HypermeshError::UnknownClassification)
        }
    }
}

fn polygon_cycles_share_reversed_noncoplanar_triangle_edge(
    decisions: &DecisionContext,
    left: &[Point3],
    left_support: &Plane,
    right: &[Point3],
    right_support: &Plane,
) -> HypermeshResult<bool> {
    if left.len() != 3 || right.len() != 3 {
        return Ok(false);
    }
    for left_index in 0..3 {
        let left_start = &left[left_index];
        let left_end = &left[(left_index + 1) % 3];
        for right_index in 0..3 {
            if left_start != &right[(right_index + 1) % 3] || left_end != &right[right_index] {
                continue;
            }
            let left_opposite = &left[(left_index + 2) % 3];
            let right_opposite = &right[(right_index + 2) % 3];
            return Ok(
                classify_point_decision(decisions, right_opposite, left_support)?
                    != Classification::On
                    || classify_point_decision(decisions, left_opposite, right_support)?
                        != Classification::On,
            );
        }
    }
    Ok(false)
}

pub(crate) fn segment_has_strict_interior_point_in_both(
    decisions: &DecisionContext,
    a: &Point3,
    b: &Point3,
    left: &ConvexPolygon,
    right: &ConvexPolygon,
) -> HypermeshResult<bool> {
    let mut lower = Real::zero();
    let mut upper = Real::one();
    Ok(
        constrain_open_segment_interval_to_polygon(decisions, a, b, left, &mut lower, &mut upper)?
            && constrain_open_segment_interval_to_polygon(
                decisions, a, b, right, &mut lower, &mut upper,
            )?
            && compare_real_decision(decisions, &lower, &upper)?.is_lt(),
    )
}

fn constrain_open_segment_interval_to_polygon(
    decisions: &DecisionContext,
    a: &Point3,
    b: &Point3,
    polygon: &ConvexPolygon,
    lower: &mut Real,
    upper: &mut Real,
) -> HypermeshResult<bool> {
    for edge in polygon.edges.iter() {
        if !constrain_open_segment_interval_to_plane_negative(decisions, a, b, edge, lower, upper)?
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn constrain_open_segment_interval_to_plane_negative(
    decisions: &DecisionContext,
    a: &Point3,
    b: &Point3,
    plane: &Plane,
    lower: &mut Real,
    upper: &mut Real,
) -> HypermeshResult<bool> {
    let start = plane.expression_at_point(a);
    let end = plane.expression_at_point(b);
    let start_class = classify_real(decisions, &start)?;
    let end_class = classify_real(decisions, &end)?;

    match (start_class, end_class) {
        (Classification::Negative, Classification::Negative)
        | (Classification::Negative, Classification::On)
        | (Classification::On, Classification::Negative) => Ok(true),
        (Classification::Positive, Classification::Negative) => {
            let cut = (start.clone() / (&start - &end))
                .map_err(|_| HypermeshError::UnknownClassification)?;
            update_open_segment_lower(decisions, lower, &cut)
        }
        (Classification::Negative, Classification::Positive) => {
            let cut = (start.clone() / (&start - &end))
                .map_err(|_| HypermeshError::UnknownClassification)?;
            update_open_segment_upper(decisions, upper, &cut)
        }
        (Classification::On, Classification::On)
        | (Classification::Positive, Classification::Positive)
        | (Classification::Positive, Classification::On)
        | (Classification::On, Classification::Positive) => Ok(false),
    }
}

fn update_open_segment_lower(
    decisions: &DecisionContext,
    lower: &mut Real,
    candidate: &Real,
) -> HypermeshResult<bool> {
    if compare_real_decision(decisions, candidate, lower)?.is_gt() {
        *lower = candidate.clone();
    }
    Ok(compare_real_decision(decisions, lower, &Real::one())?.is_lt())
}

fn update_open_segment_upper(
    decisions: &DecisionContext,
    upper: &mut Real,
    candidate: &Real,
) -> HypermeshResult<bool> {
    if compare_real_decision(decisions, candidate, upper)?.is_lt() {
        *upper = candidate.clone();
    }
    Ok(compare_real_decision(decisions, &Real::zero(), upper)?.is_lt())
}

fn intersect_coplanar(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    other: &ConvexPolygon,
    other_polygon_idx: usize,
) -> HypermeshResult<PairwiseIntersection> {
    if polygons_share_area(decisions, polygon, other)? {
        Ok(PairwiseIntersection {
            kind: PairwiseIntersectionType::Overlap,
            segment: None,
            overlap: Some(OverlapInfo { other_polygon_idx }),
        })
    } else {
        Ok(PairwiseIntersection::none())
    }
}

fn polygons_share_area(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    other: &ConvexPolygon,
) -> HypermeshResult<bool> {
    let mut intersection = polygon.clone();
    for edge in other.edges.iter() {
        let clipped = clip_polygon_decision(decisions, &intersection, edge)?;
        intersection = match clipped.side {
            ClipSide::Left | ClipSide::Both => clipped.left,
            ClipSide::Right => return Ok(false),
        };
    }

    let vertices = intersection.vertices_decision(decisions)?;
    let Some(first) = vertices.first() else {
        return Ok(false);
    };
    for index in 1..vertices.len().saturating_sub(1) {
        if Plane::decide_points_are_nondegenerate(
            decisions,
            first,
            &vertices[index],
            &vertices[index + 1],
        )? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn collect_edge_plane_crossings(
    decisions: &DecisionContext,
    edge_polygon: &ConvexPolygon,
    vertices: &[Point3],
    plane_polygon: &ConvexPolygon,
    points: &mut Vec<Point3>,
) -> HypermeshResult<()> {
    if let [v0, v1, v2] = vertices {
        let c0 = classify_point_decision(decisions, v0, &plane_polygon.support)?;
        let c1 = classify_point_decision(decisions, v1, &plane_polygon.support)?;
        let c2 = classify_point_decision(decisions, v2, &plane_polygon.support)?;
        collect_edge_plane_crossing(
            decisions,
            edge_polygon,
            0,
            v0,
            v1,
            c0,
            c1,
            plane_polygon,
            points,
        )?;
        collect_edge_plane_crossing(
            decisions,
            edge_polygon,
            1,
            v1,
            v2,
            c1,
            c2,
            plane_polygon,
            points,
        )?;
        collect_edge_plane_crossing(
            decisions,
            edge_polygon,
            2,
            v2,
            v0,
            c2,
            c0,
            plane_polygon,
            points,
        )?;
        return Ok(());
    }

    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        let start_class = classify_point_decision(decisions, start, &plane_polygon.support)?;
        let end_class = classify_point_decision(decisions, end, &plane_polygon.support)?;
        collect_edge_plane_crossing(
            decisions,
            edge_polygon,
            index,
            start,
            end,
            start_class,
            end_class,
            plane_polygon,
            points,
        )?;
    }
    Ok(())
}

#[inline]
fn collect_edge_plane_crossing(
    decisions: &DecisionContext,
    edge_polygon: &ConvexPolygon,
    edge_index: usize,
    start: &Point3,
    end: &Point3,
    start_class: Classification,
    end_class: Classification,
    plane_polygon: &ConvexPolygon,
    points: &mut Vec<Point3>,
) -> HypermeshResult<()> {
    let candidate = match (start_class, end_class) {
        (Classification::On, _) => {
            affine_point_in_polygon_on_support(decisions, start, plane_polygon)?
                .then(|| start.clone())
        }
        (_, Classification::On) => {
            affine_point_in_polygon_on_support(decisions, end, plane_polygon)?.then(|| end.clone())
        }
        (Classification::Negative, Classification::Positive)
        | (Classification::Positive, Classification::Negative) => {
            let point = intersect_segment_plane(start, end, &plane_polygon.support)?;
            let contained =
                match affine_point_in_polygon_on_support(decisions, &point, plane_polygon) {
                    Ok(contained) => contained,
                    Err(HypermeshError::PredicateUndecided { .. }) => {
                        match projective_edge_plane_intersection_in_polygon(
                            decisions,
                            edge_polygon,
                            edge_index,
                            plane_polygon,
                        ) {
                            Ok(contained) => contained,
                            Err(HypermeshError::PredicateUndecided { .. }) => {
                                segment_plane_intersection_in_polygon(
                                    decisions,
                                    start,
                                    end,
                                    start_class,
                                    plane_polygon,
                                )?
                            }
                            Err(error) => return Err(error),
                        }
                    }
                    Err(error) => return Err(error),
                };
            contained.then_some(point)
        }
        _ => None,
    };

    if let Some(point) = candidate {
        points.push(point);
    }
    Ok(())
}

fn projective_edge_plane_intersection_in_polygon(
    decisions: &DecisionContext,
    edge_polygon: &ConvexPolygon,
    edge_index: usize,
    plane_polygon: &ConvexPolygon,
) -> HypermeshResult<bool> {
    let edge_plane = edge_polygon
        .edges
        .get(edge_index)
        .ok_or(HypermeshError::UnknownClassification)?;
    let line = intersect_two_planes(&edge_polygon.support, edge_plane);
    let point = intersect_homogeneous_line_plane(&line, &plane_polygon.support);
    let mut saw_unknown = false;
    for edge in plane_polygon.edges.iter() {
        match classify_projective_point_decision(decisions, &point, edge) {
            Ok(Classification::Positive) => return Ok(false),
            Ok(Classification::Negative | Classification::On) => {}
            Err(HypermeshError::PredicateUndecided { .. }) => {
                if homogeneous_point_certifiably_nonzero(decisions, &point)
                    && crate::predicate::classify_real(
                        decisions,
                        &four_plane_determinant(
                            &edge_polygon.support,
                            edge_plane,
                            &plane_polygon.support,
                            edge,
                        ),
                    ) == Ok(Classification::On)
                {
                    continue;
                }
                saw_unknown = true;
            }
            Err(error) => return Err(error),
        }
    }
    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "projective edge/polygon containment",
        })
    } else {
        Ok(true)
    }
}

fn homogeneous_point_certifiably_nonzero(
    decisions: &DecisionContext,
    point: &HomogeneousPoint3,
) -> bool {
    [&point.x, &point.y, &point.z, &point.w]
        .into_iter()
        .any(|coordinate| {
            matches!(
                crate::predicate::classify_real(decisions, coordinate),
                Ok(Classification::Negative | Classification::Positive)
            )
        })
}

pub(crate) fn four_plane_determinant(
    a: &Plane,
    b: &Plane,
    c: &Plane,
    d: &Plane,
) -> hyperlattice::Real {
    const PERMUTATIONS: [[usize; 4]; 24] = [
        [0, 1, 2, 3],
        [0, 1, 3, 2],
        [0, 2, 1, 3],
        [0, 2, 3, 1],
        [0, 3, 1, 2],
        [0, 3, 2, 1],
        [1, 0, 2, 3],
        [1, 0, 3, 2],
        [1, 2, 0, 3],
        [1, 2, 3, 0],
        [1, 3, 0, 2],
        [1, 3, 2, 0],
        [2, 0, 1, 3],
        [2, 0, 3, 1],
        [2, 1, 0, 3],
        [2, 1, 3, 0],
        [2, 3, 0, 1],
        [2, 3, 1, 0],
        [3, 0, 1, 2],
        [3, 0, 2, 1],
        [3, 1, 0, 2],
        [3, 1, 2, 0],
        [3, 2, 0, 1],
        [3, 2, 1, 0],
    ];
    const POSITIVE: [bool; 24] = [
        true, false, false, true, true, false, false, true, true, false, false, true, true, false,
        false, true, true, false, false, true, true, false, false, true,
    ];
    let rows = [
        [&a.normal.x, &a.normal.y, &a.normal.z, &a.offset],
        [&b.normal.x, &b.normal.y, &b.normal.z, &b.offset],
        [&c.normal.x, &c.normal.y, &c.normal.z, &c.offset],
        [&d.normal.x, &d.normal.y, &d.normal.z, &d.offset],
    ];
    let terms: [[&hyperlattice::Real; 4]; 24] =
        std::array::from_fn(|term| std::array::from_fn(|row| rows[row][PERMUTATIONS[term][row]]));
    hyperlattice::Real::signed_product_sum(POSITIVE, terms)
}

/// Certifies containment of a proper segment/plane intersection without first
/// expanding the affine intersection point.
///
/// For support values `a`, `b` at the segment endpoints and edge-plane values
/// `q0`, `q1`, the edge value at the intersection is
/// `(a*q1 - b*q0) / (a - b)`. The endpoints are known to be on opposite sides,
/// so the denominator sign is already certified by `start_class`. Keeping this
/// predicate as a two-term determinant preserves cancellations that can become
/// opaque after all three affine coordinates are materialized.
fn segment_plane_intersection_in_polygon(
    decisions: &DecisionContext,
    start: &Point3,
    end: &Point3,
    start_class: Classification,
    polygon: &ConvexPolygon,
) -> HypermeshResult<bool> {
    debug_assert!(matches!(
        start_class,
        Classification::Negative | Classification::Positive
    ));

    let start_support = polygon.support.expression_at_point(start);
    let end_support = polygon.support.expression_at_point(end);
    let denominator_is_positive = start_class == Classification::Positive;
    let mut saw_unknown = false;

    for edge in polygon.edges.iter() {
        let start_edge = edge.expression_at_point(start);
        let end_edge = edge.expression_at_point(end);
        let numerator = hyperlattice::Real::signed_product_sum(
            [true, false],
            [[&start_support, &end_edge], [&end_support, &start_edge]],
        );
        let candidate_class = match classify_real(decisions, &numerator) {
            Ok(classification) if denominator_is_positive => classification,
            Ok(Classification::Negative) => Classification::Positive,
            Ok(Classification::Positive) => Classification::Negative,
            Ok(Classification::On) => Classification::On,
            Err(HypermeshError::PredicateUndecided { .. }) => {
                saw_unknown = true;
                continue;
            }
            Err(error) => return Err(error),
        };
        if candidate_class == Classification::Positive {
            return Ok(false);
        }
    }

    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "segment-plane intersection containment",
        })
    } else {
        Ok(true)
    }
}

fn intersect_segment_plane(start: &Point3, end: &Point3, plane: &Plane) -> HypermeshResult<Point3> {
    let start_value = plane.expression_at_point(start);
    let end_value = plane.expression_at_point(end);
    let denom = &start_value - &end_value;
    let t = (start_value / denom).map_err(|_| HypermeshError::UnknownClassification)?;

    Ok(Point3::new(
        &start.x + &(t.clone() * (&end.x - &start.x)),
        &start.y + &(t.clone() * (&end.y - &start.y)),
        &start.z + &(t * (&end.z - &start.z)),
    ))
}

fn affine_point_in_polygon_on_support(
    decisions: &DecisionContext,
    point: &Point3,
    polygon: &ConvexPolygon,
) -> HypermeshResult<bool> {
    if polygon.has_retained_vertex(point) {
        return Ok(true);
    }
    let mut saw_unknown = false;
    for edge in polygon.edges.iter() {
        match classify_point_decision(decisions, point, edge) {
            Ok(Classification::Positive) => return Ok(false),
            Ok(Classification::Negative | Classification::On) => {}
            Err(HypermeshError::PredicateUndecided { .. }) => saw_unknown = true,
            Err(error) => return Err(error),
        }
    }
    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "affine point/polygon containment",
        })
    } else {
        Ok(true)
    }
}

fn supports_are_parallel(
    decisions: &DecisionContext,
    left: &Plane,
    right: &Plane,
) -> HypermeshResult<bool> {
    let cross = Point3::new(
        hyperlattice::Real::signed_product_sum(
            [true, false],
            [
                [&left.normal.y, &right.normal.z],
                [&left.normal.z, &right.normal.y],
            ],
        ),
        hyperlattice::Real::signed_product_sum(
            [true, false],
            [
                [&left.normal.z, &right.normal.x],
                [&left.normal.x, &right.normal.z],
            ],
        ),
        hyperlattice::Real::signed_product_sum(
            [true, false],
            [
                [&left.normal.x, &right.normal.y],
                [&left.normal.y, &right.normal.x],
            ],
        ),
    );
    let mut saw_unknown = false;
    for component in [&cross.x, &cross.y, &cross.z] {
        match classify_real(decisions, component) {
            Ok(Classification::On) => {}
            Ok(Classification::Negative | Classification::Positive) => return Ok(false),
            Err(HypermeshError::PredicateUndecided { .. }) => saw_unknown = true,
            Err(error) => return Err(error),
        }
    }
    if saw_unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "polygon support-plane parallelism",
        })
    } else {
        Ok(true)
    }
}

fn dedup_points(decisions: &DecisionContext, points: &mut Vec<Point3>) -> HypermeshResult<()> {
    let mut unique = Vec::with_capacity(points.len());
    for point in points.drain(..) {
        let mut duplicate = false;
        for existing in &unique {
            if existing == &point || crate::predicate::points_equal(decisions, existing, &point)? {
                duplicate = true;
                break;
            }
        }
        if !duplicate {
            unique.push(point);
        }
    }
    *points = unique;
    Ok(())
}

#[cfg(test)]
mod tests {
    use hyperlattice::{Point3, Real};

    use super::{
        PairwiseIntersection, PairwiseIntersectionEventRef, PairwiseIntersectionGraphBuilder,
        PairwiseIntersectionNode,
    };

    #[test]
    fn compact_graph_preserves_stream_order_without_per_face_vectors() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(4);
        graph.append_overlap(2, 0).unwrap();
        graph.append_overlap(0, 2).unwrap();
        graph.append_overlap(2, 1).unwrap();
        let graph = graph.finish();

        assert_eq!(graph.len(), 4);
        assert_eq!(graph.event_count(), 3);
        assert!(graph.row(1).is_empty());
        assert!(graph.row(3).is_empty());
        assert_eq!(
            graph
                .row(2)
                .map(|event| match event {
                    PairwiseIntersectionEventRef::Overlap { other_polygon_idx } => {
                        other_polygon_idx
                    }
                    PairwiseIntersectionEventRef::Segment { .. } => unreachable!(),
                })
                .collect::<Vec<_>>(),
            vec![0, 1]
        );
    }

    #[test]
    fn empty_face_index_uses_two_u32_words_per_face() {
        let graph = PairwiseIntersectionGraphBuilder::new(64).finish();
        assert_eq!(graph.heads.len() * size_of::<u32>(), 256);
        assert_eq!(graph.counts.len() * size_of::<u32>(), 256);
        assert!(graph.nodes.is_empty());
        assert!(64 * 2 * size_of::<u32>() < 64 * size_of::<Vec<PairwiseIntersection>>());
        assert!(size_of::<PairwiseIntersectionNode>() < size_of::<PairwiseIntersection>());
    }

    #[test]
    fn symmetric_segment_events_share_one_segment_and_endpoint_record() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(2);
        graph
            .append_segment_pair(
                0,
                1,
                Point3::origin(),
                Point3::new(Real::one(), Real::zero(), Real::zero()),
            )
            .unwrap();
        let graph = graph.finish();

        assert_eq!(graph.points.len(), 2);
        assert_eq!(graph.segments.len(), 1);
        assert_eq!(size_of::<super::PairwiseIntersectionSegment>(), 8);
        assert_eq!(graph.event_count(), 2);
        assert!(matches!(
            graph.row(0).next(),
            Some(PairwiseIntersectionEventRef::Segment {
                other_polygon_idx: 1,
                ..
            })
        ));
        assert!(matches!(
            graph.row(1).next(),
            Some(PairwiseIntersectionEventRef::Segment {
                other_polygon_idx: 0,
                ..
            })
        ));
    }

    #[test]
    fn exact_segment_endpoints_share_one_compact_point_arena() {
        let origin = Point3::origin();
        let mut graph = PairwiseIntersectionGraphBuilder::new(3);
        graph
            .append_segment_pair(
                0,
                1,
                origin.clone(),
                Point3::new(Real::one(), Real::zero(), Real::zero()),
            )
            .unwrap();
        graph
            .append_segment_pair(
                0,
                2,
                origin,
                Point3::new(Real::zero(), Real::one(), Real::zero()),
            )
            .unwrap();
        let graph = graph.finish();

        assert_eq!(graph.points.len(), 3);
        assert_eq!(graph.segments.len(), 2);
        assert_eq!(
            graph.segments[0].endpoints[0],
            graph.segments[1].endpoints[0]
        );
    }

    #[test]
    fn symbolic_segment_endpoints_do_not_add_an_equality_decision() {
        let symbolic = Point3::new(Real::from(2).sqrt().unwrap(), Real::zero(), Real::zero());
        let mut graph = PairwiseIntersectionGraphBuilder::new(3);
        graph
            .append_segment_pair(0, 1, symbolic.clone(), Point3::origin())
            .unwrap();
        graph
            .append_segment_pair(0, 2, symbolic, Point3::origin())
            .unwrap();
        let graph = graph.finish();

        assert_eq!(graph.points.len(), 4);
        assert_ne!(
            graph.segments[0].endpoints[0],
            graph.segments[1].endpoints[0]
        );
    }

    #[test]
    fn polygon_order_remap_preserves_compact_endpoint_ids() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(3);
        graph
            .append_segment_pair(
                0,
                2,
                Point3::origin(),
                Point3::new(Real::one(), Real::zero(), Real::zero()),
            )
            .unwrap();
        let graph = graph.finish().remap_polygon_order(&[2, 1, 0]).unwrap();

        assert_eq!(graph.points.len(), 2);
        let Some(PairwiseIntersectionEventRef::Segment {
            segment,
            other_polygon_idx,
        }) = graph.row(2).next()
        else {
            panic!("remapped source face must retain its segment");
        };
        assert_eq!(other_polygon_idx, 0);
        assert_eq!(segment.v0, &Point3::origin());
        assert_eq!(
            segment.v1,
            &Point3::new(Real::one(), Real::zero(), Real::zero())
        );
    }

    #[test]
    fn invalid_face_append_fails_without_mutating_the_arena() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(0);
        assert!(graph.append_overlap(0, 0).is_err());
        assert_eq!(graph.finish().event_count(), 0);
    }

    #[test]
    fn pair_append_failures_leave_no_half_edge_or_orphan_segment() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(2);
        graph.counts[1] = u32::MAX;

        assert!(
            graph
                .append_segment_pair(
                    0,
                    1,
                    Point3::origin(),
                    Point3::new(Real::one(), Real::zero(), Real::zero()),
                )
                .is_err()
        );
        assert!(graph.append_overlap_pair(0, 1).is_err());
        assert!(graph.nodes.is_empty());
        assert!(graph.points.is_empty());
        assert!(graph.segments.is_empty());
        assert_eq!(graph.counts[0], 0);
        assert_eq!(graph.heads[0], super::NO_INTERSECTION_NODE);
        assert_eq!(graph.tails[0], super::NO_INTERSECTION_NODE);
    }

    #[test]
    fn self_pair_is_rejected_without_mutation() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(1);
        assert!(graph.append_overlap_pair(0, 0).is_err());
        assert!(
            graph
                .append_segment_pair(0, 0, Point3::origin(), Point3::origin())
                .is_err()
        );
        let graph = graph.finish();
        assert_eq!(graph.event_count(), 0);
        assert!(graph.points.is_empty());
        assert!(graph.segments.is_empty());
    }
}
