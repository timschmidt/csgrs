//! Pairwise convex polygon intersection primitives.

use hyperlattice::{
    HomogeneousPoint3, Point3, Real, intersect_homogeneous_line_plane, intersect_two_planes,
};

use crate::bvh::ExactBvh;
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

/// Single exact contact point between two polygons.
#[derive(Clone, Debug, PartialEq)]
pub struct IntersectionPoint {
    /// Exact contact point.
    pub point: Point3,
    /// Local index of the other polygon.
    pub other_polygon_idx: usize,
}

/// Coplanar overlap information.
#[derive(Clone, Debug, PartialEq)]
pub struct OverlapInfo {
    /// Local index of the other polygon.
    pub other_polygon_idx: usize,
}

/// Exact closed-set intersection of two convex polygons.
///
/// Separate variants make support-plane incidence and intersection dimension
/// explicit and make contradictory kind/payload combinations unrepresentable.
#[derive(Clone, Debug, PartialEq)]
pub enum PairwiseIntersection {
    /// The closed polygons are disjoint.
    Disjoint,
    /// One-point intersection between non-coplanar support planes.
    NonCoplanarPoint(IntersectionPoint),
    /// Positive-length intersection between non-coplanar support planes.
    NonCoplanarSegment(IntersectionSegment),
    /// One-point boundary contact between coplanar polygons.
    CoplanarPoint(IntersectionPoint),
    /// Positive-length, zero-area boundary contact between coplanar polygons.
    CoplanarSegment(IntersectionSegment),
    /// Positive-area intersection between coplanar polygons.
    CoplanarOverlap(OverlapInfo),
}

const INTERSECTION_EVENT_POINT: u32 = 1 << 31;
const INTERSECTION_EVENT_COPLANAR: u32 = 1 << 30;
const INTERSECTION_EVENT_INDEX_MASK: u32 = INTERSECTION_EVENT_COPLANAR - 1;
const INTERSECTION_EVENT_INDEX_LIMIT: usize = INTERSECTION_EVENT_INDEX_MASK as usize;
const COPLANAR_OVERLAP_EVENT: u32 = u32::MAX;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum StoredIntersectionKind {
    NonCoplanarSegment,
    CoplanarSegment,
    NonCoplanarPoint,
    CoplanarPoint,
    CoplanarOverlap,
}

fn encode_intersection_geometry(
    kind: StoredIntersectionKind,
    index: usize,
) -> HypermeshResult<u32> {
    if kind == StoredIntersectionKind::CoplanarOverlap {
        return Ok(COPLANAR_OVERLAP_EVENT);
    }
    let index = u32::try_from(index)
        .ok()
        .filter(|&index| index < INTERSECTION_EVENT_INDEX_MASK)
        .ok_or(HypermeshError::CapacityOverflow {
            operation: "pairwise intersection geometry arena",
        })?;
    let tag = match kind {
        StoredIntersectionKind::NonCoplanarSegment => 0,
        StoredIntersectionKind::CoplanarSegment => INTERSECTION_EVENT_COPLANAR,
        StoredIntersectionKind::NonCoplanarPoint => INTERSECTION_EVENT_POINT,
        StoredIntersectionKind::CoplanarPoint => {
            INTERSECTION_EVENT_POINT | INTERSECTION_EVENT_COPLANAR
        }
        StoredIntersectionKind::CoplanarOverlap => unreachable!(),
    };
    Ok(tag | index)
}

fn decode_intersection_geometry(geometry: u32) -> (StoredIntersectionKind, Option<usize>) {
    if geometry == COPLANAR_OVERLAP_EVENT {
        return (StoredIntersectionKind::CoplanarOverlap, None);
    }
    let kind = match geometry & (INTERSECTION_EVENT_POINT | INTERSECTION_EVENT_COPLANAR) {
        0 => StoredIntersectionKind::NonCoplanarSegment,
        INTERSECTION_EVENT_COPLANAR => StoredIntersectionKind::CoplanarSegment,
        INTERSECTION_EVENT_POINT => StoredIntersectionKind::NonCoplanarPoint,
        _ => StoredIntersectionKind::CoplanarPoint,
    };
    (
        kind,
        Some((geometry & INTERSECTION_EVENT_INDEX_MASK) as usize),
    )
}

fn intersection_geometry_exists(
    geometry: u32,
    points: &[Point3],
    segments: &[PairwiseIntersectionSegment],
) -> bool {
    let (kind, index) = decode_intersection_geometry(geometry);
    match kind {
        StoredIntersectionKind::NonCoplanarPoint | StoredIntersectionKind::CoplanarPoint => {
            index.is_some_and(|index| points.get(index).is_some())
        }
        StoredIntersectionKind::NonCoplanarSegment | StoredIntersectionKind::CoplanarSegment => {
            index
                .and_then(|index| segments.get(index))
                .is_some_and(|segment| {
                    segment
                        .endpoints
                        .iter()
                        .all(|&point| points.get(point as usize).is_some())
                })
        }
        StoredIntersectionKind::CoplanarOverlap => true,
    }
}

#[derive(Clone, Debug, PartialEq)]
struct PendingIntersectionEvent {
    face: u32,
    other_polygon: u32,
    geometry: u32,
}

#[derive(Clone, Debug, PartialEq)]
struct PairwiseIntersectionEvent {
    other_polygon: u32,
    geometry: u32,
}

#[derive(Clone, Debug, PartialEq)]
struct PairwiseIntersectionSegment {
    endpoints: [u32; 2],
}

#[derive(Clone, Copy)]
pub(crate) enum PairwiseIntersectionEventRef<'a> {
    NonCoplanarPoint {
        point: &'a Point3,
        other_polygon_idx: usize,
    },
    NonCoplanarSegment {
        segment: PairwiseIntersectionSegmentRef<'a>,
        other_polygon_idx: usize,
    },
    CoplanarPoint {
        point: &'a Point3,
        other_polygon_idx: usize,
    },
    CoplanarSegment {
        segment: PairwiseIntersectionSegmentRef<'a>,
        other_polygon_idx: usize,
    },
    CoplanarOverlap {
        other_polygon_idx: usize,
    },
}

impl PairwiseIntersectionEventRef<'_> {
    pub(crate) const fn other_polygon_idx(self) -> usize {
        match self {
            Self::NonCoplanarPoint {
                other_polygon_idx, ..
            }
            | Self::NonCoplanarSegment {
                other_polygon_idx, ..
            }
            | Self::CoplanarPoint {
                other_polygon_idx, ..
            }
            | Self::CoplanarSegment {
                other_polygon_idx, ..
            }
            | Self::CoplanarOverlap { other_polygon_idx } => other_polygon_idx,
        }
    }

    pub(crate) const fn is_coplanar_overlap(self) -> bool {
        matches!(self, Self::CoplanarOverlap { .. })
    }

    pub(crate) const fn changes_open_face_partition(self) -> bool {
        match self {
            Self::NonCoplanarSegment { segment, .. } => {
                let _ = segment;
                true
            }
            Self::CoplanarOverlap { .. } => true,
            Self::NonCoplanarPoint { point, .. } | Self::CoplanarPoint { point, .. } => {
                let _ = point;
                false
            }
            Self::CoplanarSegment { segment, .. } => {
                let _ = segment;
                false
            }
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct PairwiseIntersectionSegmentRef<'a> {
    pub(crate) v0: &'a Point3,
    pub(crate) v1: &'a Point3,
}

/// Compact face-indexed intersection adjacency backed by contiguous rows.
///
/// Empty faces cost one 32-bit offset rather than a separately allocated `Vec`
/// header. The two directed events for a non-coplanar cut share one endpoint
/// record. Events retain their deterministic BVH discovery order within each
/// row without retaining a global candidate-pair buffer.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PairwiseIntersectionGraph {
    offsets: Box<[u32]>,
    points: Vec<Point3>,
    segments: Vec<PairwiseIntersectionSegment>,
    events: Vec<PairwiseIntersectionEvent>,
}

impl PairwiseIntersectionGraph {
    pub(crate) fn len(&self) -> usize {
        self.offsets.len().saturating_sub(1)
    }

    pub(crate) fn event_count(&self) -> usize {
        self.events.len()
    }

    pub(crate) fn row(&self, face: usize) -> PairwiseIntersectionRow<'_> {
        debug_assert!(face < self.len());
        let next_face = face.checked_add(1);
        let next = self.offsets.get(face).copied().unwrap_or(0);
        let end = next_face
            .and_then(|next_face| self.offsets.get(next_face))
            .copied()
            .unwrap_or(next);
        PairwiseIntersectionRow {
            graph: self,
            next,
            end,
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

        let mut remapped = PairwiseIntersectionGraphBuilder::new(query_to_cached.len())?;
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
        remapped.reserve_events(self.events.len())?;
        for (query_index, &cached_index) in query_to_cached.iter().enumerate() {
            let start = *self
                .offsets
                .get(cached_index)
                .ok_or(HypermeshError::UnknownClassification)? as usize;
            let end = *self
                .offsets
                .get(cached_index + 1)
                .ok_or(HypermeshError::UnknownClassification)? as usize;
            let row = self
                .events
                .get(start..end)
                .ok_or(HypermeshError::UnknownClassification)?;
            for entry in row {
                if !intersection_geometry_exists(entry.geometry, &self.points, &self.segments) {
                    return Err(HypermeshError::UnknownClassification);
                }
                let other_polygon = remapped_face_id(&cached_to_query, entry.other_polygon)?;
                remapped.append(query_index, other_polygon, entry.geometry)?;
            }
        }
        remapped.finish()
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
    end: u32,
}

impl PairwiseIntersectionRow<'_> {
    #[cfg(test)]
    pub(crate) fn is_empty(&self) -> bool {
        self.next == self.end
    }

    pub(crate) fn iter(&self) -> Self {
        *self
    }

    pub(crate) fn open_face_partition_count(self) -> usize {
        self.filter(|event| event.changes_open_face_partition())
            .count()
    }
}

impl<'a> Iterator for PairwiseIntersectionRow<'a> {
    type Item = PairwiseIntersectionEventRef<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.next == self.end {
            return None;
        }
        let event = &self.graph.events[self.next as usize];
        self.next += 1;
        let other_polygon_idx = event.other_polygon as usize;
        let (kind, index) = decode_intersection_geometry(event.geometry);
        Some(match kind {
            StoredIntersectionKind::NonCoplanarPoint => {
                PairwiseIntersectionEventRef::NonCoplanarPoint {
                    point: &self.graph.points[index.expect("point events carry an index")],
                    other_polygon_idx,
                }
            }
            StoredIntersectionKind::NonCoplanarSegment => {
                let segment = &self.graph.segments[index.expect("segment events carry an index")];
                PairwiseIntersectionEventRef::NonCoplanarSegment {
                    segment: PairwiseIntersectionSegmentRef {
                        v0: &self.graph.points[segment.endpoints[0] as usize],
                        v1: &self.graph.points[segment.endpoints[1] as usize],
                    },
                    other_polygon_idx,
                }
            }
            StoredIntersectionKind::CoplanarPoint => PairwiseIntersectionEventRef::CoplanarPoint {
                point: &self.graph.points[index.expect("point events carry an index")],
                other_polygon_idx,
            },
            StoredIntersectionKind::CoplanarSegment => {
                let segment = &self.graph.segments[index.expect("segment events carry an index")];
                PairwiseIntersectionEventRef::CoplanarSegment {
                    segment: PairwiseIntersectionSegmentRef {
                        v0: &self.graph.points[segment.endpoints[0] as usize],
                        v1: &self.graph.points[segment.endpoints[1] as usize],
                    },
                    other_polygon_idx,
                }
            }
            StoredIntersectionKind::CoplanarOverlap => {
                PairwiseIntersectionEventRef::CoplanarOverlap { other_polygon_idx }
            }
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = (self.end - self.next) as usize;
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for PairwiseIntersectionRow<'_> {}

pub(crate) struct PairwiseIntersectionGraphBuilder {
    counts: Box<[u32]>,
    points: Vec<Point3>,
    point_interner: PointInterner<()>,
    segments: Vec<PairwiseIntersectionSegment>,
    events: Vec<PendingIntersectionEvent>,
}

impl PairwiseIntersectionGraphBuilder {
    pub(crate) fn new(face_count: usize) -> HypermeshResult<Self> {
        u32::try_from(face_count).map_err(|_| HypermeshError::CapacityOverflow {
            operation: "pairwise intersection face arena",
        })?;
        let mut counts = Vec::new();
        counts
            .try_reserve_exact(face_count)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection face arena",
            })?;
        counts.resize(face_count, 0);
        Ok(Self {
            counts: counts.into_boxed_slice(),
            points: Vec::new(),
            point_interner: PointInterner::new_exact_unreserved(),
            segments: Vec::new(),
            events: Vec::new(),
        })
    }

    fn face_id(&self, face: usize) -> HypermeshResult<u32> {
        if face >= self.counts.len() {
            return Err(HypermeshError::CapacityOverflow {
                operation: "pairwise intersection graph face index",
            });
        }
        u32::try_from(face).map_err(|_| HypermeshError::CapacityOverflow {
            operation: "pairwise intersection face ID",
        })
    }

    fn reserve_events(&mut self, additional: usize) -> HypermeshResult<()> {
        let new_len = self
            .events
            .len()
            .checked_add(additional)
            .filter(|&len| len <= u32::MAX as usize)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "pairwise intersection graph",
            })?;
        debug_assert!(new_len >= self.events.len());
        self.events
            .try_reserve(additional)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection graph",
            })
    }

    fn reserve_segments(&mut self, additional: usize) -> HypermeshResult<()> {
        self.segments
            .len()
            .checked_add(additional)
            .filter(|&len| len <= INTERSECTION_EVENT_INDEX_LIMIT)
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

    fn append_prechecked(&mut self, face: usize, face_id: u32, other_polygon: u32, geometry: u32) {
        self.counts[face] += 1;
        self.events.push(PendingIntersectionEvent {
            face: face_id,
            other_polygon,
            geometry,
        });
    }

    fn append(&mut self, face: usize, other_polygon: u32, geometry: u32) -> HypermeshResult<()> {
        let face_id = self.face_id(face)?;
        self.check_row_capacity(face, 1)?;
        self.reserve_events(1)?;
        self.append_prechecked(face, face_id, other_polygon, geometry);
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn append_coplanar_overlap(
        &mut self,
        face: usize,
        other_polygon_idx: usize,
    ) -> HypermeshResult<()> {
        let other_polygon = self.face_id(other_polygon_idx)?;
        self.append(face, other_polygon, COPLANAR_OVERLAP_EVENT)
    }

    pub(crate) fn append_non_coplanar_segment_pair(
        &mut self,
        left: usize,
        right: usize,
        v0: Point3,
        v1: Point3,
    ) -> HypermeshResult<()> {
        self.append_segment_pair(
            left,
            right,
            v0,
            v1,
            StoredIntersectionKind::NonCoplanarSegment,
        )
    }

    pub(crate) fn append_coplanar_segment_pair(
        &mut self,
        left: usize,
        right: usize,
        v0: Point3,
        v1: Point3,
    ) -> HypermeshResult<()> {
        self.append_segment_pair(left, right, v0, v1, StoredIntersectionKind::CoplanarSegment)
    }

    fn append_segment_pair(
        &mut self,
        left: usize,
        right: usize,
        v0: Point3,
        v1: Point3,
        kind: StoredIntersectionKind,
    ) -> HypermeshResult<()> {
        debug_assert!(matches!(
            kind,
            StoredIntersectionKind::NonCoplanarSegment | StoredIntersectionKind::CoplanarSegment
        ));
        if left == right {
            return Err(HypermeshError::UnknownClassification);
        }
        let left_id = self.face_id(left)?;
        let right_id = self.face_id(right)?;
        self.check_row_capacity(left, 1)?;
        self.check_row_capacity(right, 1)?;
        self.reserve_segments(1)?;
        self.reserve_events(2)?;
        self.points
            .len()
            .checked_add(2)
            .filter(|&len| len <= INTERSECTION_EVENT_INDEX_LIMIT)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "pairwise intersection point arena",
            })?;
        let endpoints = self
            .point_interner
            .intern_exact_pair_or_append(&mut self.points, [v0, v1])?;
        debug_assert!(
            endpoints
                .iter()
                .all(|&point| point < INTERSECTION_EVENT_INDEX_LIMIT)
        );
        let endpoints = endpoints.map(|point| {
            u32::try_from(point).expect("intersection point capacity was checked before insertion")
        });
        let geometry = encode_intersection_geometry(kind, self.segments.len())?;
        self.segments
            .push(PairwiseIntersectionSegment { endpoints });
        self.append_prechecked(left, left_id, right_id, geometry);
        self.append_prechecked(right, right_id, left_id, geometry);
        Ok(())
    }

    pub(crate) fn append_non_coplanar_point_pair(
        &mut self,
        left: usize,
        right: usize,
        point: Point3,
    ) -> HypermeshResult<()> {
        self.append_point_pair(left, right, point, StoredIntersectionKind::NonCoplanarPoint)
    }

    pub(crate) fn append_coplanar_point_pair(
        &mut self,
        left: usize,
        right: usize,
        point: Point3,
    ) -> HypermeshResult<()> {
        self.append_point_pair(left, right, point, StoredIntersectionKind::CoplanarPoint)
    }

    fn append_point_pair(
        &mut self,
        left: usize,
        right: usize,
        point: Point3,
        kind: StoredIntersectionKind,
    ) -> HypermeshResult<()> {
        debug_assert!(matches!(
            kind,
            StoredIntersectionKind::NonCoplanarPoint | StoredIntersectionKind::CoplanarPoint
        ));
        if left == right {
            return Err(HypermeshError::UnknownClassification);
        }
        let left_id = self.face_id(left)?;
        let right_id = self.face_id(right)?;
        self.check_row_capacity(left, 1)?;
        self.check_row_capacity(right, 1)?;
        self.reserve_events(2)?;
        self.points
            .len()
            .checked_add(1)
            .filter(|&len| len <= INTERSECTION_EVENT_INDEX_LIMIT)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "pairwise intersection point arena",
            })?;
        let point = self
            .point_interner
            .intern_exact_or_append(&mut self.points, point)?;
        let geometry = encode_intersection_geometry(kind, point)?;
        self.append_prechecked(left, left_id, right_id, geometry);
        self.append_prechecked(right, right_id, left_id, geometry);
        Ok(())
    }

    pub(crate) fn append_coplanar_overlap_pair(
        &mut self,
        left: usize,
        right: usize,
    ) -> HypermeshResult<()> {
        if left == right {
            return Err(HypermeshError::UnknownClassification);
        }
        let left_id = self.face_id(left)?;
        let right_id = self.face_id(right)?;
        self.check_row_capacity(left, 1)?;
        self.check_row_capacity(right, 1)?;
        self.reserve_events(2)?;
        self.append_prechecked(left, left_id, right_id, COPLANAR_OVERLAP_EVENT);
        self.append_prechecked(right, right_id, left_id, COPLANAR_OVERLAP_EVENT);
        Ok(())
    }

    pub(crate) fn finish(self) -> HypermeshResult<PairwiseIntersectionGraph> {
        let Self {
            mut counts,
            points,
            point_interner,
            segments,
            events,
        } = self;
        drop(point_interner);

        let offset_capacity =
            counts
                .len()
                .checked_add(1)
                .ok_or(HypermeshError::CapacityOverflow {
                    operation: "pairwise intersection graph offsets",
                })?;
        let mut offsets = Vec::new();
        offsets.try_reserve_exact(offset_capacity).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "pairwise intersection graph offsets",
            }
        })?;
        offsets.push(0u32);
        for &count in counts.iter() {
            let next = offsets.last().copied().unwrap().checked_add(count).ok_or(
                HypermeshError::CapacityOverflow {
                    operation: "pairwise intersection graph offsets",
                },
            )?;
            offsets.push(next);
        }
        if offsets.last().copied().unwrap_or(0) as usize != events.len() {
            return Err(HypermeshError::UnknownClassification);
        }

        let mut ordered = Vec::new();
        ordered
            .try_reserve_exact(events.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection graph rows",
            })?;
        ordered.resize(
            events.len(),
            PairwiseIntersectionEvent {
                other_polygon: 0,
                geometry: COPLANAR_OVERLAP_EVENT,
            },
        );
        for (face, cursor) in counts.iter_mut().enumerate() {
            *cursor = offsets[face];
        }
        for event in events {
            let face = event.face as usize;
            if face >= counts.len()
                || event.other_polygon as usize >= counts.len()
                || !intersection_geometry_exists(event.geometry, &points, &segments)
            {
                return Err(HypermeshError::UnknownClassification);
            }
            let cursor = &mut counts[face];
            if *cursor >= offsets[face + 1] {
                return Err(HypermeshError::UnknownClassification);
            }
            ordered[*cursor as usize] = PairwiseIntersectionEvent {
                other_polygon: event.other_polygon,
                geometry: event.geometry,
            };
            *cursor += 1;
        }
        if counts
            .iter()
            .enumerate()
            .any(|(face, &cursor)| cursor != offsets[face + 1])
        {
            return Err(HypermeshError::UnknownClassification);
        }

        Ok(PairwiseIntersectionGraph {
            offsets: offsets.into_boxed_slice(),
            points,
            segments,
            events: ordered,
        })
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
        return Ok(PairwiseIntersection::Disjoint);
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
            intersect_coplanar(
                decisions,
                polygon,
                polygon_vertices,
                other,
                other_vertices,
                other_polygon_idx,
            )
        } else {
            Ok(PairwiseIntersection::Disjoint)
        };
    }

    let mut points = Vec::new();
    crate::trace_dispatch!("intersect-polygons", "edge-crossings-forward");
    collect_edge_plane_crossings(decisions, polygon, polygon_vertices, other, &mut points)?;
    crate::trace_dispatch!("intersect-polygons", "edge-crossings-reverse");
    collect_edge_plane_crossings(decisions, other, other_vertices, polygon, &mut points)?;
    dedup_points(decisions, &mut points)?;

    match exact_intersection_span(decisions, &polygon.support, &points)? {
        IntersectionSpan::Empty => Ok(PairwiseIntersection::Disjoint),
        IntersectionSpan::Point(point) => {
            Ok(PairwiseIntersection::NonCoplanarPoint(IntersectionPoint {
                point,
                other_polygon_idx,
            }))
        }
        IntersectionSpan::Segment { v0, v1 } => Ok(PairwiseIntersection::NonCoplanarSegment(
            IntersectionSegment {
                v0,
                v1,
                other_polygon_idx,
            },
        )),
    }
}

/// Operation-local affine vertices in one checked face-indexed range arena.
///
/// Retained points are cloned structurally and derived points use the caller's
/// policy-aware decision context. The arena records no topology conclusion.
struct PolygonVertexArena {
    offsets: Vec<u32>,
    points: Vec<Point3>,
}

impl PolygonVertexArena {
    fn build(decisions: &DecisionContext, polygons: &[ConvexPolygon]) -> HypermeshResult<Self> {
        u32::try_from(polygons.len()).map_err(|_| HypermeshError::CapacityOverflow {
            operation: "pairwise polygon vertex face arena",
        })?;
        let offset_capacity =
            polygons
                .len()
                .checked_add(1)
                .ok_or(HypermeshError::CapacityOverflow {
                    operation: "pairwise polygon vertex offsets",
                })?;
        let point_capacity = polygons
            .iter()
            .try_fold(0usize, |total, polygon| {
                total.checked_add(polygon.vertex_count())
            })
            .filter(|&total| total <= u32::MAX as usize)
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "pairwise polygon vertex arena",
            })?;

        let mut offsets = Vec::new();
        offsets.try_reserve_exact(offset_capacity).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "pairwise polygon vertex offsets",
            }
        })?;
        let mut points = Vec::new();
        points
            .try_reserve_exact(point_capacity)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise polygon vertex arena",
            })?;
        offsets.push(0);

        for polygon in polygons {
            if let Some(vertices) = polygon.known_vertices.as_ref() {
                points.extend(vertices.iter().cloned());
            } else {
                for index in 0..polygon.vertex_count() {
                    points.push(polygon.vertex_point_decision(decisions, index)?);
                }
            }
            offsets.push(
                u32::try_from(points.len())
                    .expect("pairwise polygon vertex capacity was checked before construction"),
            );
        }
        debug_assert_eq!(points.len(), point_capacity);
        Ok(Self { offsets, points })
    }

    fn row(&self, polygon: usize) -> HypermeshResult<&[Point3]> {
        let next = polygon
            .checked_add(1)
            .ok_or(HypermeshError::UnknownClassification)?;
        let start = self
            .offsets
            .get(polygon)
            .copied()
            .ok_or(HypermeshError::UnknownClassification)? as usize;
        let end = self
            .offsets
            .get(next)
            .copied()
            .ok_or(HypermeshError::UnknownClassification)? as usize;
        self.points
            .get(start..end)
            .ok_or(HypermeshError::UnknownClassification)
    }
}

/// Builds exact symmetric intersection rows for one polygon arrangement.
///
/// The BVH callback is consumed directly so the broad phase never materializes
/// a global candidate-pair vector. Rows remain in deterministic polygon order
/// and retain every exact point, segment, and positive-area overlap incidence
/// required by a complete face arrangement.
pub(crate) fn pairwise_intersections_by_polygon_with_certified_embedded_inputs(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    certified_embedded_inputs: &[bool],
) -> HypermeshResult<PairwiseIntersectionGraph> {
    let mut graph = PairwiseIntersectionGraphBuilder::new(polygons.len())?;
    let bvh = ExactBvh::build_decision(decisions, polygons)?;
    let vertices = PolygonVertexArena::build(decisions, polygons)?;
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
    graph.finish()
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
    vertices: &PolygonVertexArena,
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
    let left_vertices = vertices.row(global_i)?;
    let right_vertices = vertices.row(global_j)?;
    let shares_manifold_edge = if same_mesh {
        polygon_cycles_share_reversed_noncoplanar_triangle_edge(
            decisions,
            left_vertices,
            &polygons[global_i].support,
            right_vertices,
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
        left_vertices,
        &polygons[global_j],
        right_vertices,
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
    if same_mesh
        && pairwise_intersection_is_shared_input_feature(
            decisions,
            &intersection,
            &polygons[global_i],
            left_vertices,
            &polygons[global_j],
            right_vertices,
        )?
    {
        crate::trace_dispatch!("pairwise-intersection", "same-mesh-shared-feature");
        return Ok(());
    }
    match intersection {
        PairwiseIntersection::NonCoplanarPoint(point) => {
            crate::trace_dispatch!("pairwise-intersection", "nonempty-contact");
            graph.append_non_coplanar_point_pair(global_i, global_j, point.point)
        }
        PairwiseIntersection::NonCoplanarSegment(segment) => {
            if same_mesh
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
            graph.append_non_coplanar_segment_pair(global_i, global_j, segment.v0, segment.v1)
        }
        PairwiseIntersection::CoplanarPoint(point) => {
            crate::trace_dispatch!("pairwise-intersection", "nonempty-contact");
            graph.append_coplanar_point_pair(global_i, global_j, point.point)
        }
        PairwiseIntersection::CoplanarSegment(segment) => {
            crate::trace_dispatch!("pairwise-intersection", "nonempty-contact");
            graph.append_coplanar_segment_pair(global_i, global_j, segment.v0, segment.v1)
        }
        PairwiseIntersection::CoplanarOverlap(_) => {
            crate::trace_dispatch!("pairwise-intersection", "nonempty-cut");
            graph.append_coplanar_overlap_pair(global_i, global_j)
        }
        PairwiseIntersection::Disjoint => Ok(()),
    }
}

fn pairwise_intersection_is_shared_input_feature(
    decisions: &DecisionContext,
    intersection: &PairwiseIntersection,
    left: &ConvexPolygon,
    left_vertices: &[Point3],
    right: &ConvexPolygon,
    right_vertices: &[Point3],
) -> HypermeshResult<bool> {
    match intersection {
        PairwiseIntersection::NonCoplanarPoint(contact)
        | PairwiseIntersection::CoplanarPoint(contact) => shared_vertex_identity_at_point(
            decisions,
            left,
            left_vertices,
            right,
            right_vertices,
            &contact.point,
        ),
        PairwiseIntersection::NonCoplanarSegment(segment)
        | PairwiseIntersection::CoplanarSegment(segment) => shared_edge_identity_for_segment(
            decisions,
            left,
            left_vertices,
            right,
            right_vertices,
            &segment.v0,
            &segment.v1,
        ),
        PairwiseIntersection::Disjoint | PairwiseIntersection::CoplanarOverlap(_) => Ok(false),
    }
}

fn shared_vertex_identity_at_point(
    decisions: &DecisionContext,
    left: &ConvexPolygon,
    left_vertices: &[Point3],
    right: &ConvexPolygon,
    right_vertices: &[Point3],
    point: &Point3,
) -> HypermeshResult<bool> {
    let (Some(left_identities), Some(right_identities)) = (
        left.known_vertex_identities(),
        right.known_vertex_identities(),
    ) else {
        return Ok(false);
    };
    for (left_index, left_point) in left_vertices.iter().enumerate() {
        if left_point != point && !crate::predicate::points_equal(decisions, left_point, point)? {
            continue;
        }
        let Some(left_identity) = left_identities.get(left_index) else {
            return Err(HypermeshError::UnknownClassification);
        };
        for (right_index, right_point) in right_vertices.iter().enumerate() {
            let Some(right_identity) = right_identities.get(right_index) else {
                return Err(HypermeshError::UnknownClassification);
            };
            if left_identity == right_identity
                && (right_point == point
                    || crate::predicate::points_equal(decisions, right_point, point)?)
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn shared_edge_identity_for_segment(
    decisions: &DecisionContext,
    left: &ConvexPolygon,
    left_vertices: &[Point3],
    right: &ConvexPolygon,
    right_vertices: &[Point3],
    v0: &Point3,
    v1: &Point3,
) -> HypermeshResult<bool> {
    let (Some(left_identities), Some(right_identities)) =
        (left.known_edge_identities(), right.known_edge_identities())
    else {
        return Ok(false);
    };
    for left_index in 0..left_identities.len() {
        let Some(left_identity) = left_identities.get(left_index) else {
            return Err(HypermeshError::UnknownClassification);
        };
        for right_index in 0..right_identities.len() {
            let Some(right_identity) = right_identities.get(right_index) else {
                return Err(HypermeshError::UnknownClassification);
            };
            if right_identity != left_identity {
                continue;
            }
            if segment_matches_polygon_edge(decisions, left_vertices, left_index, v0, v1)?
                && segment_matches_polygon_edge(decisions, right_vertices, right_index, v0, v1)?
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn segment_matches_polygon_edge(
    decisions: &DecisionContext,
    vertices: &[Point3],
    edge: usize,
    v0: &Point3,
    v1: &Point3,
) -> HypermeshResult<bool> {
    let Some(start) = vertices.get(edge) else {
        return Err(HypermeshError::UnknownClassification);
    };
    let Some(end) = edge
        .checked_add(1)
        .map(|next| next % vertices.len())
        .and_then(|next| vertices.get(next))
    else {
        return Err(HypermeshError::UnknownClassification);
    };
    Ok(
        (points_match(decisions, start, v0)? && points_match(decisions, end, v1)?)
            || (points_match(decisions, start, v1)? && points_match(decisions, end, v0)?),
    )
}

fn points_match(
    decisions: &DecisionContext,
    left: &Point3,
    right: &Point3,
) -> HypermeshResult<bool> {
    Ok(left == right || crate::predicate::points_equal(decisions, left, right)?)
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
    polygon_vertices: &[Point3],
    other: &ConvexPolygon,
    other_vertices: &[Point3],
    other_polygon_idx: usize,
) -> HypermeshResult<PairwiseIntersection> {
    if polygons_share_area(decisions, polygon, polygon_vertices, other, other_vertices)? {
        return Ok(PairwiseIntersection::CoplanarOverlap(OverlapInfo {
            other_polygon_idx,
        }));
    }

    // The intersection of two closed convex polygons is convex. Once the exact
    // separating-axis proof has rejected positive area, any remaining
    // intersection is one point or one collinear segment. Every endpoint of
    // that intersection is an input vertex contained in the other closed
    // polygon, so no projected edge/edge construction or tolerance path is
    // required.
    let capacity = polygon_vertices
        .len()
        .checked_add(other_vertices.len())
        .ok_or(HypermeshError::CapacityOverflow {
            operation: "coplanar polygon contact candidates",
        })?;
    let mut points = Vec::new();
    points
        .try_reserve_exact(capacity)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "coplanar polygon contact candidates",
        })?;
    for point in polygon_vertices {
        if affine_point_in_polygon_on_support(decisions, point, other)? {
            points.push(point.clone());
        }
    }
    for point in other_vertices {
        if affine_point_in_polygon_on_support(decisions, point, polygon)? {
            points.push(point.clone());
        }
    }
    dedup_points(decisions, &mut points)?;

    match exact_intersection_span(decisions, &polygon.support, &points)? {
        IntersectionSpan::Empty => Ok(PairwiseIntersection::Disjoint),
        IntersectionSpan::Point(point) => {
            Ok(PairwiseIntersection::CoplanarPoint(IntersectionPoint {
                point,
                other_polygon_idx,
            }))
        }
        IntersectionSpan::Segment { v0, v1 } => {
            Ok(PairwiseIntersection::CoplanarSegment(IntersectionSegment {
                v0,
                v1,
                other_polygon_idx,
            }))
        }
    }
}

enum IntersectionSpan {
    Empty,
    Point(Point3),
    Segment { v0: Point3, v1: Point3 },
}

/// Canonicalizes any nonempty zero- or one-dimensional convex intersection.
///
/// Narrow-phase edge walks can encounter more than two distinct points when
/// an otherwise valid polygon retains collinear boundary vertices. Selecting
/// exact lexicographic extrema preserves the whole interval instead of
/// silently truncating it to the first two discovery-order points.
fn exact_intersection_span(
    decisions: &DecisionContext,
    support: &Plane,
    points: &[Point3],
) -> HypermeshResult<IntersectionSpan> {
    let Some(first) = points.first() else {
        return Ok(IntersectionSpan::Empty);
    };
    match points {
        [_] => return Ok(IntersectionSpan::Point(first.clone())),
        [v0, v1] => {
            return Ok(IntersectionSpan::Segment {
                v0: v0.clone(),
                v1: v1.clone(),
            });
        }
        _ => {}
    }

    let mut minimum = first;
    let mut maximum = first;
    for point in &points[1..] {
        if compare_points_lexicographically(decisions, point, minimum)?.is_lt() {
            minimum = point;
        }
        if compare_points_lexicographically(decisions, point, maximum)?.is_gt() {
            maximum = point;
        }
    }
    for point in points {
        if !support.points_are_collinear_on_support(decisions, minimum, maximum, point)? {
            return Err(HypermeshError::UnknownClassification);
        }
    }
    Ok(IntersectionSpan::Segment {
        v0: minimum.clone(),
        v1: maximum.clone(),
    })
}

fn compare_points_lexicographically(
    decisions: &DecisionContext,
    left: &Point3,
    right: &Point3,
) -> HypermeshResult<std::cmp::Ordering> {
    for (left, right) in [
        (&left.x, &right.x),
        (&left.y, &right.y),
        (&left.z, &right.z),
    ] {
        let ordering = compare_real_decision(decisions, left, right)?;
        if !ordering.is_eq() {
            return Ok(ordering);
        }
    }
    Ok(std::cmp::Ordering::Equal)
}

fn polygons_share_area(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    polygon_vertices: &[Point3],
    other: &ConvexPolygon,
    other_vertices: &[Point3],
) -> HypermeshResult<bool> {
    Ok(
        !polygon_has_open_interior_separator(decisions, polygon, other_vertices)?
            && !polygon_has_open_interior_separator(decisions, other, polygon_vertices)?,
    )
}

/// Exact convex separating-axis test for positive-area coplanar overlap.
///
/// Every edge plane is an axis whose negative halfspace contains its polygon.
/// If every vertex of the other polygon is on or outside one edge, the closed
/// sets are disjoint or touch only in dimension zero or one. Conversely, if no
/// edge of either convex polygon separates their open interiors, their planar
/// intersection has positive area. This avoids constructing and repeatedly
/// cloning an intermediate clipped polygon.
fn polygon_has_open_interior_separator(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    other_vertices: &[Point3],
) -> HypermeshResult<bool> {
    if other_vertices.is_empty() {
        return Ok(true);
    }
    for edge in polygon.edges.iter() {
        let mut reaches_negative_halfspace = false;
        for point in other_vertices {
            if classify_point_decision(decisions, point, edge)? == Classification::Negative {
                reaches_negative_halfspace = true;
                break;
            }
        }
        if !reaches_negative_halfspace {
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
        PairwiseIntersection, PairwiseIntersectionEvent, PairwiseIntersectionEventRef,
        PairwiseIntersectionGraphBuilder, PolygonVertexArena,
    };

    #[test]
    fn polygon_vertex_arena_flattens_known_empty_and_constructed_rows() {
        let decisions = crate::test_support::approximate_decisions();
        let first = crate::test_support::approximate_convex_triangle(
            &Point3::origin(),
            &Point3::new(Real::one(), Real::zero(), Real::zero()),
            &Point3::new(Real::zero(), Real::one(), Real::zero()),
            0,
            0,
        );
        let mut constructed = crate::test_support::approximate_convex_triangle(
            &Point3::new(Real::zero(), Real::zero(), Real::one()),
            &Point3::new(Real::one(), Real::zero(), Real::one()),
            &Point3::new(Real::zero(), Real::one(), Real::one()),
            1,
            1,
        );
        constructed.known_vertices = None;
        let expected_constructed = constructed.vertices_decision(&decisions).unwrap();

        let arena = PolygonVertexArena::build(
            &decisions,
            &[
                first.clone(),
                crate::polygon::ConvexPolygon::empty(),
                constructed,
            ],
        )
        .unwrap();

        assert_eq!(arena.offsets, [0, 3, 3, 6]);
        assert_eq!(
            arena.row(0).unwrap(),
            first.vertices_decision(&decisions).unwrap()
        );
        assert!(arena.row(1).unwrap().is_empty());
        assert!(arena.row(2).unwrap().iter().zip(expected_constructed).all(
            |(actual, expected)| {
                crate::predicate::points_equal(&decisions, actual, &expected).unwrap()
            }
        ));
        assert!(arena.row(3).is_err());
        assert!(arena.row(usize::MAX).is_err());
        assert_eq!(arena.offsets.len() * size_of::<u32>(), 16);
        assert!(arena.offsets.len() * size_of::<u32>() < 3 * size_of::<Vec<Point3>>());

        let invalid = PolygonVertexArena {
            offsets: vec![0, 2],
            points: vec![Point3::origin()],
        };
        assert!(invalid.row(0).is_err());
    }

    #[test]
    fn compact_graph_preserves_stream_order_without_per_face_vectors() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(4).unwrap();
        graph.append_coplanar_overlap(2, 0).unwrap();
        graph.append_coplanar_overlap(0, 2).unwrap();
        graph.append_coplanar_overlap(2, 1).unwrap();
        let graph = graph.finish().unwrap();

        assert_eq!(graph.len(), 4);
        assert_eq!(graph.event_count(), 3);
        assert_eq!(&*graph.offsets, &[0, 1, 1, 3, 3]);
        assert!(graph.row(1).is_empty());
        assert!(graph.row(3).is_empty());
        assert_eq!(
            graph
                .row(2)
                .map(|event| match event {
                    PairwiseIntersectionEventRef::CoplanarOverlap { other_polygon_idx } => {
                        other_polygon_idx
                    }
                    _ => unreachable!(),
                })
                .collect::<Vec<_>>(),
            vec![0, 1]
        );
    }

    #[test]
    fn empty_face_index_uses_one_u32_offset_per_face_and_a_terminal() {
        let graph = PairwiseIntersectionGraphBuilder::new(64)
            .unwrap()
            .finish()
            .unwrap();
        assert_eq!(graph.offsets.len() * size_of::<u32>(), 260);
        assert!(graph.events.is_empty());
        assert!(65 * size_of::<u32>() < 64 * size_of::<Vec<PairwiseIntersection>>());
        assert!(size_of::<PairwiseIntersectionEvent>() < size_of::<PairwiseIntersection>());
    }

    #[test]
    fn symmetric_segment_events_share_one_segment_and_endpoint_record() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(2).unwrap();
        graph
            .append_non_coplanar_segment_pair(
                0,
                1,
                Point3::origin(),
                Point3::new(Real::one(), Real::zero(), Real::zero()),
            )
            .unwrap();
        let graph = graph.finish().unwrap();

        assert_eq!(graph.points.len(), 2);
        assert_eq!(graph.segments.len(), 1);
        assert_eq!(size_of::<super::PairwiseIntersectionSegment>(), 8);
        assert_eq!(graph.event_count(), 2);
        assert!(matches!(
            graph.row(0).next(),
            Some(PairwiseIntersectionEventRef::NonCoplanarSegment {
                other_polygon_idx: 1,
                ..
            })
        ));
        assert!(matches!(
            graph.row(1).next(),
            Some(PairwiseIntersectionEventRef::NonCoplanarSegment {
                other_polygon_idx: 0,
                ..
            })
        ));
    }

    #[test]
    fn compact_graph_retains_every_pairwise_intersection_dimension() {
        let p2 = |x, y| Point3::new(Real::from(x), Real::from(y), Real::zero());
        let mut graph = PairwiseIntersectionGraphBuilder::new(6).unwrap();
        graph
            .append_non_coplanar_point_pair(0, 1, p2(0, 0))
            .unwrap();
        graph
            .append_non_coplanar_segment_pair(1, 2, p2(1, 0), p2(2, 0))
            .unwrap();
        graph.append_coplanar_point_pair(2, 3, p2(3, 0)).unwrap();
        graph
            .append_coplanar_segment_pair(3, 4, p2(4, 0), p2(5, 0))
            .unwrap();
        graph.append_coplanar_overlap_pair(4, 5).unwrap();
        let graph = graph.finish().unwrap();

        assert_eq!(graph.event_count(), 10);
        assert_eq!(size_of::<PairwiseIntersectionEvent>(), 8);
        assert_eq!(size_of::<super::PendingIntersectionEvent>(), 12);
        assert!(matches!(
            graph.row(0).next(),
            Some(PairwiseIntersectionEventRef::NonCoplanarPoint {
                point,
                other_polygon_idx: 1,
            }) if point == &Point3::origin()
        ));
        assert!(matches!(
            graph.row(1).nth(1),
            Some(PairwiseIntersectionEventRef::NonCoplanarSegment {
                other_polygon_idx: 2,
                ..
            })
        ));
        assert!(matches!(
            graph.row(2).nth(1),
            Some(PairwiseIntersectionEventRef::CoplanarPoint {
                point: contact,
                other_polygon_idx: 3,
            }) if contact == &p2(3, 0)
        ));
        assert!(matches!(
            graph.row(3).nth(1),
            Some(PairwiseIntersectionEventRef::CoplanarSegment {
                segment,
                other_polygon_idx: 4,
            }) if segment.v0 == &p2(4, 0) && segment.v1 == &p2(5, 0)
        ));
        assert!(matches!(
            graph.row(5).next(),
            Some(PairwiseIntersectionEventRef::CoplanarOverlap {
                other_polygon_idx: 4,
            })
        ));
        assert_eq!(graph.row(0).open_face_partition_count(), 0);
        assert_eq!(graph.row(1).open_face_partition_count(), 1);
        assert_eq!(graph.row(3).open_face_partition_count(), 0);
        assert_eq!(graph.row(4).open_face_partition_count(), 1);
    }

    #[test]
    fn compact_event_tag_reserves_are_checked_and_round_trip() {
        for kind in [
            super::StoredIntersectionKind::NonCoplanarSegment,
            super::StoredIntersectionKind::CoplanarSegment,
            super::StoredIntersectionKind::NonCoplanarPoint,
            super::StoredIntersectionKind::CoplanarPoint,
        ] {
            let encoded = super::encode_intersection_geometry(kind, 17).unwrap();
            assert_eq!(
                super::decode_intersection_geometry(encoded),
                (kind, Some(17))
            );
            assert!(
                super::encode_intersection_geometry(kind, super::INTERSECTION_EVENT_INDEX_LIMIT)
                    .is_err()
            );
        }
        assert_eq!(
            super::decode_intersection_geometry(super::COPLANAR_OVERLAP_EVENT),
            (super::StoredIntersectionKind::CoplanarOverlap, None)
        );
    }

    #[test]
    fn intrinsic_source_features_are_not_duplicated_as_intersection_events() {
        let decisions = crate::test_support::approximate_decisions();
        let p = |x, y| Point3::new(Real::from(x), Real::from(y), Real::zero());
        let mut host =
            crate::test_support::approximate_convex_triangle(&p(0, 0), &p(2, 0), &p(0, 2), 0, 0);
        host.set_source_triangle_edge_identities(0, [0, 1, 2])
            .unwrap();
        let mut shared_edge =
            crate::test_support::approximate_convex_triangle(&p(2, 0), &p(0, 0), &p(1, -2), 0, 1);
        shared_edge
            .set_source_triangle_edge_identities(0, [1, 0, 3])
            .unwrap();
        let mut shared_vertex =
            crate::test_support::approximate_convex_triangle(&p(2, 0), &p(3, -1), &p(3, 0), 0, 2);
        shared_vertex
            .set_source_triangle_edge_identities(0, [1, 4, 5])
            .unwrap();

        let graph = super::pairwise_intersections_by_polygon(
            &decisions,
            &[host.clone(), shared_edge, shared_vertex],
        )
        .unwrap();
        assert_eq!(graph.event_count(), 0);
        assert!(graph.points.is_empty());
        assert!(graph.segments.is_empty());

        let mut distinct_vertex =
            crate::test_support::approximate_convex_triangle(&p(2, 0), &p(3, -1), &p(3, 0), 0, 3);
        distinct_vertex
            .set_source_triangle_edge_identities(0, [10, 11, 12])
            .unwrap();
        let graph =
            super::pairwise_intersections_by_polygon(&decisions, &[host, distinct_vertex]).unwrap();
        assert_eq!(graph.event_count(), 2);
        assert!(matches!(
            graph.row(0).next(),
            Some(PairwiseIntersectionEventRef::CoplanarPoint {
                other_polygon_idx: 1,
                ..
            })
        ));
    }

    #[test]
    fn exact_segment_endpoints_share_one_compact_point_arena() {
        let origin = Point3::origin();
        let mut graph = PairwiseIntersectionGraphBuilder::new(3).unwrap();
        graph
            .append_non_coplanar_segment_pair(
                0,
                1,
                origin.clone(),
                Point3::new(Real::one(), Real::zero(), Real::zero()),
            )
            .unwrap();
        graph
            .append_non_coplanar_segment_pair(
                0,
                2,
                origin,
                Point3::new(Real::zero(), Real::one(), Real::zero()),
            )
            .unwrap();
        let graph = graph.finish().unwrap();

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
        let mut graph = PairwiseIntersectionGraphBuilder::new(3).unwrap();
        graph
            .append_non_coplanar_segment_pair(0, 1, symbolic.clone(), Point3::origin())
            .unwrap();
        graph
            .append_non_coplanar_segment_pair(0, 2, symbolic, Point3::origin())
            .unwrap();
        let graph = graph.finish().unwrap();

        assert_eq!(graph.points.len(), 4);
        assert_ne!(
            graph.segments[0].endpoints[0],
            graph.segments[1].endpoints[0]
        );
    }

    #[test]
    fn polygon_order_remap_preserves_compact_endpoint_ids() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(3).unwrap();
        graph
            .append_non_coplanar_segment_pair(
                0,
                2,
                Point3::origin(),
                Point3::new(Real::one(), Real::zero(), Real::zero()),
            )
            .unwrap();
        let graph = graph
            .finish()
            .unwrap()
            .remap_polygon_order(&[2, 1, 0])
            .unwrap();

        assert_eq!(graph.points.len(), 2);
        let Some(PairwiseIntersectionEventRef::NonCoplanarSegment {
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
        #[cfg(target_pointer_width = "64")]
        assert!(PairwiseIntersectionGraphBuilder::new(usize::MAX).is_err());
        let mut graph = PairwiseIntersectionGraphBuilder::new(0).unwrap();
        assert!(graph.append_coplanar_overlap(0, 0).is_err());
        assert_eq!(graph.finish().unwrap().event_count(), 0);
    }

    #[test]
    fn pair_append_failures_leave_no_half_edge_or_orphan_segment() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(2).unwrap();
        graph.counts[1] = u32::MAX;

        assert!(
            graph
                .append_non_coplanar_segment_pair(
                    0,
                    1,
                    Point3::origin(),
                    Point3::new(Real::one(), Real::zero(), Real::zero()),
                )
                .is_err()
        );
        assert!(
            graph
                .append_coplanar_segment_pair(
                    0,
                    1,
                    Point3::origin(),
                    Point3::new(Real::one(), Real::zero(), Real::zero()),
                )
                .is_err()
        );
        assert!(
            graph
                .append_non_coplanar_point_pair(0, 1, Point3::origin())
                .is_err()
        );
        assert!(
            graph
                .append_coplanar_point_pair(0, 1, Point3::origin())
                .is_err()
        );
        assert!(graph.append_coplanar_overlap_pair(0, 1).is_err());
        assert!(graph.events.is_empty());
        assert!(graph.points.is_empty());
        assert!(graph.segments.is_empty());
        assert_eq!(graph.counts[0], 0);
        assert!(graph.finish().is_err());
    }

    #[test]
    fn self_pair_is_rejected_without_mutation() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(1).unwrap();
        assert!(graph.append_coplanar_overlap_pair(0, 0).is_err());
        assert!(
            graph
                .append_non_coplanar_segment_pair(0, 0, Point3::origin(), Point3::origin(),)
                .is_err()
        );
        assert!(
            graph
                .append_coplanar_segment_pair(0, 0, Point3::origin(), Point3::origin())
                .is_err()
        );
        assert!(
            graph
                .append_non_coplanar_point_pair(0, 0, Point3::origin())
                .is_err()
        );
        assert!(
            graph
                .append_coplanar_point_pair(0, 0, Point3::origin())
                .is_err()
        );
        let graph = graph.finish().unwrap();
        assert_eq!(graph.event_count(), 0);
        assert!(graph.points.is_empty());
        assert!(graph.segments.is_empty());
    }
}
