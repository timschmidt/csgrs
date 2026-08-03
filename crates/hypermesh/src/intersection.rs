//! Pairwise convex polygon intersection primitives.

use std::hash::{Hash, Hasher};

use hyperlattice::{
    HomogeneousPoint3, Point3, Real, intersect_homogeneous_line_plane, intersect_two_planes,
};

use crate::bvh::ExactBvh;
use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, compare_real_decision};
use crate::point_interner::PointInterner;
use crate::polygon::{ConstructionPlaneIdentity, ConstructionVertexIdentity, ConvexPolygon};
use crate::predicate::{
    classify_point_decision, classify_projective_point_decision, classify_real,
};
use crate::storage_hash::{StorageHashMap, StorageIdentityHasher};

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

#[derive(Clone, Debug)]
struct ConstructedIntersectionPoint {
    point: Point3,
    identity: Option<ConstructionVertexIdentity>,
}

#[derive(Clone, Debug)]
struct ConstructedIntersectionSegment {
    v0: ConstructedIntersectionPoint,
    v1: ConstructedIntersectionPoint,
}

#[derive(Clone, Debug)]
enum ConstructedPairwiseIntersection {
    Disjoint,
    NonCoplanarPoint(ConstructedIntersectionPoint),
    NonCoplanarSegment(ConstructedIntersectionSegment),
    CoplanarPoint(ConstructedIntersectionPoint),
    CoplanarSegment(ConstructedIntersectionSegment),
    CoplanarOverlap,
}

impl ConstructedPairwiseIntersection {
    fn into_public(self, other_polygon_idx: usize) -> PairwiseIntersection {
        match self {
            Self::Disjoint => PairwiseIntersection::Disjoint,
            Self::NonCoplanarPoint(point) => {
                PairwiseIntersection::NonCoplanarPoint(IntersectionPoint {
                    point: point.point,
                    other_polygon_idx,
                })
            }
            Self::NonCoplanarSegment(segment) => {
                PairwiseIntersection::NonCoplanarSegment(IntersectionSegment {
                    v0: segment.v0.point,
                    v1: segment.v1.point,
                    other_polygon_idx,
                })
            }
            Self::CoplanarPoint(point) => PairwiseIntersection::CoplanarPoint(IntersectionPoint {
                point: point.point,
                other_polygon_idx,
            }),
            Self::CoplanarSegment(segment) => {
                PairwiseIntersection::CoplanarSegment(IntersectionSegment {
                    v0: segment.v0.point,
                    v1: segment.v1.point,
                    other_polygon_idx,
                })
            }
            Self::CoplanarOverlap => {
                PairwiseIntersection::CoplanarOverlap(OverlapInfo { other_polygon_idx })
            }
        }
    }
}

const INTERSECTION_EVENT_POINT: u32 = 1 << 31;
const INTERSECTION_EVENT_COPLANAR: u32 = 1 << 30;
const INTERSECTION_EVENT_INDEX_MASK: u32 = INTERSECTION_EVENT_COPLANAR - 1;
const INTERSECTION_EVENT_INDEX_LIMIT: usize = INTERSECTION_EVENT_INDEX_MASK as usize;
const COPLANAR_OVERLAP_EVENT: u32 = u32::MAX;
// A source mesh cannot occupy this index in a realizable in-memory operand
// list. Pairwise construction recipes use it to distinguish face-indexed
// support planes from persistent source/projective plane namespaces.
const PAIRWISE_FACE_PLANE_NAMESPACE: u32 = u32::MAX;

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

#[cfg(test)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum PairwiseIntersectionEventIds {
    NonCoplanarPoint {
        point: u32,
        other_polygon: u32,
    },
    NonCoplanarSegment {
        endpoints: [u32; 2],
        other_polygon: u32,
    },
    CoplanarPoint {
        point: u32,
        other_polygon: u32,
    },
    CoplanarSegment {
        endpoints: [u32; 2],
        other_polygon: u32,
    },
    CoplanarOverlap {
        other_polygon: u32,
    },
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
    point_identities: Vec<Option<ConstructionVertexIdentity>>,
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

    #[cfg(test)]
    pub(crate) fn construction_point_count(&self) -> usize {
        self.points.len()
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

    #[cfg(test)]
    pub(crate) fn event_ids(
        &self,
        face: usize,
    ) -> HypermeshResult<impl ExactSizeIterator<Item = PairwiseIntersectionEventIds> + '_> {
        let end_face = face
            .checked_add(1)
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "source-face intersection row index overflowed",
            })?;
        let start =
            self.offsets
                .get(face)
                .copied()
                .ok_or(HypermeshError::SurfaceArrangementFailed {
                    reason: "source-face intersection row is absent",
                })?;
        let end = self.offsets.get(end_face).copied().ok_or(
            HypermeshError::SurfaceArrangementFailed {
                reason: "source-face intersection row terminal is absent",
            },
        )?;
        Ok(self.events[start as usize..end as usize]
            .iter()
            .map(|event| {
                let (kind, index) = decode_intersection_geometry(event.geometry);
                match kind {
                    StoredIntersectionKind::NonCoplanarPoint => {
                        PairwiseIntersectionEventIds::NonCoplanarPoint {
                            point: u32::try_from(index.expect("point events carry an index"))
                                .expect("intersection point indices fit the compact arena"),
                            other_polygon: event.other_polygon,
                        }
                    }
                    StoredIntersectionKind::NonCoplanarSegment => {
                        PairwiseIntersectionEventIds::NonCoplanarSegment {
                            endpoints: self.segments[index.expect("segment events carry an index")]
                                .endpoints,
                            other_polygon: event.other_polygon,
                        }
                    }
                    StoredIntersectionKind::CoplanarPoint => {
                        PairwiseIntersectionEventIds::CoplanarPoint {
                            point: u32::try_from(index.expect("point events carry an index"))
                                .expect("intersection point indices fit the compact arena"),
                            other_polygon: event.other_polygon,
                        }
                    }
                    StoredIntersectionKind::CoplanarSegment => {
                        PairwiseIntersectionEventIds::CoplanarSegment {
                            endpoints: self.segments[index.expect("segment events carry an index")]
                                .endpoints,
                            other_polygon: event.other_polygon,
                        }
                    }
                    StoredIntersectionKind::CoplanarOverlap => {
                        PairwiseIntersectionEventIds::CoplanarOverlap {
                            other_polygon: event.other_polygon,
                        }
                    }
                }
            }))
    }

    #[cfg(test)]
    pub(crate) fn construction_point(
        &self,
        point: u32,
    ) -> HypermeshResult<(&Point3, &ConstructionVertexIdentity)> {
        let index = point as usize;
        let point = self
            .points
            .get(index)
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "intersection event references an absent construction point",
            })?;
        let identity = self
            .point_identities
            .get(index)
            .and_then(Option::as_ref)
            .ok_or(HypermeshError::SurfaceArrangementFailed {
                reason: "intersection construction point has no canonical recipe",
            })?;
        Ok((point, identity))
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
            .point_identities
            .try_reserve_exact(self.point_identities.len())
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection construction remapping",
            })?;
        for identity in &self.point_identities {
            remapped.point_identities.push(
                identity
                    .as_ref()
                    .map(|identity| {
                        remap_pairwise_construction_identity(identity, &cached_to_query)
                    })
                    .transpose()?,
            );
        }
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

fn remap_pairwise_construction_plane(
    plane: ConstructionPlaneIdentity,
    cached_to_query: &[usize],
) -> HypermeshResult<ConstructionPlaneIdentity> {
    if plane.mesh != PAIRWISE_FACE_PLANE_NAMESPACE {
        return Ok(plane);
    }
    Ok(ConstructionPlaneIdentity {
        mesh: PAIRWISE_FACE_PLANE_NAMESPACE,
        plane: remapped_face_id(cached_to_query, plane.plane)?,
    })
}

fn remap_pairwise_construction_identity(
    identity: &ConstructionVertexIdentity,
    cached_to_query: &[usize],
) -> HypermeshResult<ConstructionVertexIdentity> {
    Ok(match identity {
        ConstructionVertexIdentity::Source { mesh, vertex } => ConstructionVertexIdentity::Source {
            mesh: *mesh,
            vertex: *vertex,
        },
        ConstructionVertexIdentity::SourceEdgePlane {
            mesh,
            endpoints,
            plane,
        } => ConstructionVertexIdentity::SourceEdgePlane {
            mesh: *mesh,
            endpoints: *endpoints,
            plane: remap_pairwise_construction_plane(*plane, cached_to_query)?,
        },
        ConstructionVertexIdentity::PlaneTriple { planes } => {
            let mut planes = [
                remap_pairwise_construction_plane(planes[0], cached_to_query)?,
                remap_pairwise_construction_plane(planes[1], cached_to_query)?,
                remap_pairwise_construction_plane(planes[2], cached_to_query)?,
            ];
            planes.sort_unstable();
            ConstructionVertexIdentity::PlaneTriple { planes }
        }
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
                let index = index.expect("point events carry an index");
                PairwiseIntersectionEventRef::NonCoplanarPoint {
                    point: &self.graph.points[index],
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
            StoredIntersectionKind::CoplanarPoint => {
                let index = index.expect("point events carry an index");
                PairwiseIntersectionEventRef::CoplanarPoint {
                    point: &self.graph.points[index],
                    other_polygon_idx,
                }
            }
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

struct ConstructionPointAlias {
    identity: ConstructionVertexIdentity,
    point: u32,
    next: u32,
}

pub(crate) struct PairwiseIntersectionGraphBuilder {
    counts: Box<[u32]>,
    points: Vec<Point3>,
    point_identities: Vec<Option<ConstructionVertexIdentity>>,
    construction_heads: StorageHashMap<u64, usize>,
    construction_aliases: Vec<ConstructionPointAlias>,
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
            point_identities: Vec::new(),
            construction_heads: StorageHashMap::default(),
            construction_aliases: Vec::new(),
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

    #[cfg(test)]
    pub(crate) fn append_non_coplanar_segment_pair(
        &mut self,
        left: usize,
        right: usize,
        v0: Point3,
        v1: Point3,
    ) -> HypermeshResult<()> {
        self.append_constructed_segment_pair(
            left,
            right,
            ConstructedIntersectionSegment {
                v0: ConstructedIntersectionPoint {
                    point: v0,
                    identity: None,
                },
                v1: ConstructedIntersectionPoint {
                    point: v1,
                    identity: None,
                },
            },
            StoredIntersectionKind::NonCoplanarSegment,
        )
    }

    #[cfg(test)]
    pub(crate) fn append_coplanar_segment_pair(
        &mut self,
        left: usize,
        right: usize,
        v0: Point3,
        v1: Point3,
    ) -> HypermeshResult<()> {
        self.append_constructed_segment_pair(
            left,
            right,
            ConstructedIntersectionSegment {
                v0: ConstructedIntersectionPoint {
                    point: v0,
                    identity: None,
                },
                v1: ConstructedIntersectionPoint {
                    point: v1,
                    identity: None,
                },
            },
            StoredIntersectionKind::CoplanarSegment,
        )
    }

    fn append_constructed_segment_pair(
        &mut self,
        left: usize,
        right: usize,
        segment: ConstructedIntersectionSegment,
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
        let endpoints = self.intern_constructed_point_pair([segment.v0, segment.v1])?;
        if endpoints[0] == endpoints[1] {
            return Err(HypermeshError::UnknownClassification);
        }
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

    #[cfg(test)]
    pub(crate) fn append_non_coplanar_point_pair(
        &mut self,
        left: usize,
        right: usize,
        point: Point3,
    ) -> HypermeshResult<()> {
        self.append_constructed_point_pair(
            left,
            right,
            ConstructedIntersectionPoint {
                point,
                identity: None,
            },
            StoredIntersectionKind::NonCoplanarPoint,
        )
    }

    #[cfg(test)]
    pub(crate) fn append_coplanar_point_pair(
        &mut self,
        left: usize,
        right: usize,
        point: Point3,
    ) -> HypermeshResult<()> {
        self.append_constructed_point_pair(
            left,
            right,
            ConstructedIntersectionPoint {
                point,
                identity: None,
            },
            StoredIntersectionKind::CoplanarPoint,
        )
    }

    fn append_constructed_point_pair(
        &mut self,
        left: usize,
        right: usize,
        point: ConstructedIntersectionPoint,
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
        let point = self.intern_constructed_point(point)?;
        let geometry = encode_intersection_geometry(kind, point)?;
        self.append_prechecked(left, left_id, right_id, geometry);
        self.append_prechecked(right, right_id, left_id, geometry);
        Ok(())
    }

    fn reserve_construction_points(
        &mut self,
        additional_points: usize,
        additional_identities: usize,
    ) -> HypermeshResult<()> {
        self.point_identities
            .try_reserve(additional_points)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection construction arena",
            })?;
        if additional_identities == 0 {
            return Ok(());
        }
        self.construction_heads
            .try_reserve(additional_identities)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection construction arena",
            })?;
        self.construction_aliases
            .try_reserve(additional_identities)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection construction arena",
            })
    }

    fn structurally_interned_point(
        &self,
        point: &ConstructedIntersectionPoint,
    ) -> HypermeshResult<Option<usize>> {
        let Some(identity) = point.identity.as_ref() else {
            return Ok(None);
        };
        let fingerprint = construction_identity_fingerprint(identity);
        let mut alias = self.construction_heads.get(&fingerprint).copied();
        while let Some(index) = alias {
            let entry = self
                .construction_aliases
                .get(index)
                .ok_or(HypermeshError::UnknownClassification)?;
            if &entry.identity == identity {
                let point_index = entry.point as usize;
                let existing = self
                    .points
                    .get(point_index)
                    .ok_or(HypermeshError::UnknownClassification)?;
                if exact_rational_points_contradict(existing, &point.point) {
                    return Err(HypermeshError::UnknownClassification);
                }
                return Ok(Some(point_index));
            }
            alias = (entry.next != u32::MAX).then_some(entry.next as usize);
        }
        Ok(None)
    }

    fn record_constructed_point(
        &mut self,
        index: usize,
        identity: Option<ConstructionVertexIdentity>,
    ) -> HypermeshResult<()> {
        if index >= self.point_identities.len() {
            if index != self.point_identities.len() || index >= self.points.len() {
                return Err(HypermeshError::UnknownClassification);
            }
            self.point_identities.push(identity.clone());
        } else if let Some(identity) = identity.as_ref() {
            let canonical = &mut self.point_identities[index];
            if canonical
                .as_ref()
                .is_none_or(|existing| identity < existing)
            {
                *canonical = Some(identity.clone());
            }
        }
        if let Some(identity) = identity {
            let compact = u32::try_from(index).map_err(|_| HypermeshError::CapacityOverflow {
                operation: "pairwise intersection construction arena",
            })?;
            let alias = u32::try_from(self.construction_aliases.len()).map_err(|_| {
                HypermeshError::CapacityOverflow {
                    operation: "pairwise intersection construction arena",
                }
            })?;
            let fingerprint = construction_identity_fingerprint(&identity);
            let next = self
                .construction_heads
                .insert(fingerprint, alias as usize)
                .map_or(u32::MAX, |next| {
                    u32::try_from(next)
                        .expect("construction alias capacity is checked before insertion")
                });
            self.construction_aliases.push(ConstructionPointAlias {
                identity,
                point: compact,
                next,
            });
        }
        Ok(())
    }

    fn intern_constructed_point(
        &mut self,
        point: ConstructedIntersectionPoint,
    ) -> HypermeshResult<usize> {
        if let Some(index) = self.structurally_interned_point(&point)? {
            return Ok(index);
        }
        self.reserve_construction_points(1, point.identity.is_some() as usize)?;
        let identity = point.identity;
        let index = self
            .point_interner
            .intern_exact_or_append(&mut self.points, point.point)?;
        self.record_constructed_point(index, identity)?;
        Ok(index)
    }

    fn intern_constructed_point_pair(
        &mut self,
        points: [ConstructedIntersectionPoint; 2],
    ) -> HypermeshResult<[usize; 2]> {
        let structural = [
            self.structurally_interned_point(&points[0])?,
            self.structurally_interned_point(&points[1])?,
        ];
        let [first_point, second_point] = points;
        match structural {
            [Some(first), Some(second)] => return Ok([first, second]),
            [Some(first), None] => {
                return Ok([first, self.intern_constructed_point(second_point)?]);
            }
            [None, Some(second)] => {
                return Ok([self.intern_constructed_point(first_point)?, second]);
            }
            [None, None] => {}
        }
        if first_point.identity.is_some() && first_point.identity == second_point.identity {
            let second_materialization = second_point.point;
            let first = self.intern_constructed_point(first_point)?;
            if exact_rational_points_contradict(&self.points[first], &second_materialization) {
                return Err(HypermeshError::UnknownClassification);
            }
            return Ok([first, first]);
        }

        let identity_count =
            first_point.identity.is_some() as usize + second_point.identity.is_some() as usize;
        self.reserve_construction_points(2, identity_count)?;
        let ConstructedIntersectionPoint {
            point: first_materialization,
            identity: first_identity,
        } = first_point;
        let ConstructedIntersectionPoint {
            point: second_materialization,
            identity: second_identity,
        } = second_point;
        let endpoints = self.point_interner.intern_exact_pair_or_append(
            &mut self.points,
            [first_materialization, second_materialization],
        )?;
        self.record_constructed_point(endpoints[0], first_identity)?;
        self.record_constructed_point(endpoints[1], second_identity)?;
        Ok(endpoints)
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
            point_identities,
            construction_heads,
            construction_aliases,
            point_interner,
            segments,
            events,
        } = self;
        drop(point_interner);
        drop(construction_heads);
        drop(construction_aliases);
        if point_identities.len() != points.len() {
            return Err(HypermeshError::UnknownClassification);
        }

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
            point_identities,
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
    intersect_polygons_with_vertices_constructed(
        decisions,
        polygon,
        polygon_vertices,
        None,
        other,
        other_vertices,
        None,
    )
    .map(|intersection| intersection.into_public(other_polygon_idx))
}

fn intersect_polygons_with_vertices_constructed(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    polygon_vertices: &[Point3],
    polygon_support_identity: Option<ConstructionPlaneIdentity>,
    other: &ConvexPolygon,
    other_vertices: &[Point3],
    other_support_identity: Option<ConstructionPlaneIdentity>,
) -> HypermeshResult<ConstructedPairwiseIntersection> {
    if polygon.vertex_count() == 0 || other.vertex_count() == 0 {
        return Ok(ConstructedPairwiseIntersection::Disjoint);
    }
    let retain_construction =
        polygon_support_identity.is_some() && other_support_identity.is_some();

    let supports_parallel = supports_are_parallel(decisions, &polygon.support, &other.support)?;
    if supports_parallel {
        crate::trace_dispatch!("intersect-polygons", "parallel-supports");
        let other_vertex = other_vertices
            .first()
            .ok_or(HypermeshError::UnknownClassification)?;
        return if classify_point_decision(decisions, other_vertex, &polygon.support)?
            == Classification::On
        {
            intersect_coplanar_constructed(
                decisions,
                polygon,
                polygon_vertices,
                other,
                other_vertices,
                retain_construction,
            )
        } else {
            Ok(ConstructedPairwiseIntersection::Disjoint)
        };
    }

    let mut points = Vec::new();
    crate::trace_dispatch!("intersect-polygons", "edge-crossings-forward");
    collect_edge_plane_crossings(
        decisions,
        polygon,
        polygon_vertices,
        other,
        other_support_identity,
        &mut points,
    )?;
    crate::trace_dispatch!("intersect-polygons", "edge-crossings-reverse");
    collect_edge_plane_crossings(
        decisions,
        other,
        other_vertices,
        polygon,
        polygon_support_identity,
        &mut points,
    )?;
    dedup_constructed_points(decisions, &mut points)?;

    match exact_constructed_intersection_span(decisions, &polygon.support, &points)? {
        ConstructedIntersectionSpan::Empty => Ok(ConstructedPairwiseIntersection::Disjoint),
        ConstructedIntersectionSpan::Point(point) => {
            Ok(ConstructedPairwiseIntersection::NonCoplanarPoint(point))
        }
        ConstructedIntersectionSpan::Segment { v0, v1 } => {
            Ok(ConstructedPairwiseIntersection::NonCoplanarSegment(
                ConstructedIntersectionSegment { v0, v1 },
            ))
        }
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
    let bvh = ExactBvh::build_decision(decisions, polygons)?;
    pairwise_intersections_by_polygon_from_bvh(decisions, polygons, certified_embedded_inputs, &bvh)
}

pub(crate) fn pairwise_intersections_by_polygon_from_bvh(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    certified_embedded_inputs: &[bool],
    bvh: &ExactBvh,
) -> HypermeshResult<PairwiseIntersectionGraph> {
    if bvh.len() != polygons.len() {
        return Err(HypermeshError::SurfaceArrangementFailed {
            reason: "intersection hierarchy and source-face counts differ",
        });
    }
    let mut graph = PairwiseIntersectionGraphBuilder::new(polygons.len())?;
    let vertices = PolygonVertexArena::build(decisions, polygons)?;
    let mut failure = None;

    bvh.intersect_pairs_decision(decisions, bvh, |global_i, global_j| {
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
    let left_support_identity = pairwise_support_identity(global_i)?;
    let right_support_identity = pairwise_support_identity(global_j)?;
    let intersection = intersect_polygons_with_vertices_constructed(
        decisions,
        &polygons[global_i],
        left_vertices,
        left_support_identity,
        &polygons[global_j],
        right_vertices,
        right_support_identity,
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
        ConstructedPairwiseIntersection::NonCoplanarPoint(point) => {
            crate::trace_dispatch!("pairwise-intersection", "nonempty-contact");
            graph.append_constructed_point_pair(
                global_i,
                global_j,
                point,
                StoredIntersectionKind::NonCoplanarPoint,
            )
        }
        ConstructedPairwiseIntersection::NonCoplanarSegment(segment) => {
            if same_mesh
                && !segment_has_strict_interior_point_in_both(
                    decisions,
                    &segment.v0.point,
                    &segment.v1.point,
                    &polygons[global_i],
                    &polygons[global_j],
                )?
            {
                crate::trace_dispatch!("pairwise-intersection", "same-mesh-boundary-only");
                return Ok(());
            }
            crate::trace_dispatch!("pairwise-intersection", "nonempty-cut");
            graph.append_constructed_segment_pair(
                global_i,
                global_j,
                segment,
                StoredIntersectionKind::NonCoplanarSegment,
            )
        }
        ConstructedPairwiseIntersection::CoplanarPoint(point) => {
            crate::trace_dispatch!("pairwise-intersection", "nonempty-contact");
            graph.append_constructed_point_pair(
                global_i,
                global_j,
                point,
                StoredIntersectionKind::CoplanarPoint,
            )
        }
        ConstructedPairwiseIntersection::CoplanarSegment(segment) => {
            crate::trace_dispatch!("pairwise-intersection", "nonempty-contact");
            graph.append_constructed_segment_pair(
                global_i,
                global_j,
                segment,
                StoredIntersectionKind::CoplanarSegment,
            )
        }
        ConstructedPairwiseIntersection::CoplanarOverlap => {
            crate::trace_dispatch!("pairwise-intersection", "nonempty-cut");
            graph.append_coplanar_overlap_pair(global_i, global_j)
        }
        ConstructedPairwiseIntersection::Disjoint => Ok(()),
    }
}

pub(crate) fn pairwise_support_identity(
    face: usize,
) -> HypermeshResult<Option<ConstructionPlaneIdentity>> {
    Ok(Some(ConstructionPlaneIdentity {
        mesh: PAIRWISE_FACE_PLANE_NAMESPACE,
        plane: u32::try_from(face).map_err(|_| HypermeshError::CapacityOverflow {
            operation: "pairwise construction support plane",
        })?,
    }))
}

fn pairwise_intersection_is_shared_input_feature(
    decisions: &DecisionContext,
    intersection: &ConstructedPairwiseIntersection,
    left: &ConvexPolygon,
    left_vertices: &[Point3],
    right: &ConvexPolygon,
    right_vertices: &[Point3],
) -> HypermeshResult<bool> {
    match intersection {
        ConstructedPairwiseIntersection::NonCoplanarPoint(contact)
        | ConstructedPairwiseIntersection::CoplanarPoint(contact) => {
            shared_vertex_identity_at_point(
                decisions,
                left,
                left_vertices,
                right,
                right_vertices,
                &contact.point,
            )
        }
        ConstructedPairwiseIntersection::NonCoplanarSegment(segment)
        | ConstructedPairwiseIntersection::CoplanarSegment(segment) => {
            shared_edge_identity_for_segment(
                decisions,
                left,
                left_vertices,
                right,
                right_vertices,
                &segment.v0.point,
                &segment.v1.point,
            )
        }
        ConstructedPairwiseIntersection::Disjoint
        | ConstructedPairwiseIntersection::CoplanarOverlap => Ok(false),
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

fn intersect_coplanar_constructed(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    polygon_vertices: &[Point3],
    other: &ConvexPolygon,
    other_vertices: &[Point3],
    retain_construction: bool,
) -> HypermeshResult<ConstructedPairwiseIntersection> {
    if polygons_share_area(decisions, polygon, polygon_vertices, other, other_vertices)? {
        return Ok(ConstructedPairwiseIntersection::CoplanarOverlap);
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
    for (index, point) in polygon_vertices.iter().enumerate() {
        if affine_point_in_polygon_on_support(decisions, point, other)? {
            points.push(ConstructedIntersectionPoint {
                point: point.clone(),
                identity: retain_construction
                    .then(|| polygon_vertex_identity(polygon, index))
                    .flatten(),
            });
        }
    }
    for (index, point) in other_vertices.iter().enumerate() {
        if affine_point_in_polygon_on_support(decisions, point, polygon)? {
            points.push(ConstructedIntersectionPoint {
                point: point.clone(),
                identity: retain_construction
                    .then(|| polygon_vertex_identity(other, index))
                    .flatten(),
            });
        }
    }
    dedup_constructed_points(decisions, &mut points)?;

    match exact_constructed_intersection_span(decisions, &polygon.support, &points)? {
        ConstructedIntersectionSpan::Empty => Ok(ConstructedPairwiseIntersection::Disjoint),
        ConstructedIntersectionSpan::Point(point) => {
            Ok(ConstructedPairwiseIntersection::CoplanarPoint(point))
        }
        ConstructedIntersectionSpan::Segment { v0, v1 } => {
            Ok(ConstructedPairwiseIntersection::CoplanarSegment(
                ConstructedIntersectionSegment { v0, v1 },
            ))
        }
    }
}

enum ConstructedIntersectionSpan {
    Empty,
    Point(ConstructedIntersectionPoint),
    Segment {
        v0: ConstructedIntersectionPoint,
        v1: ConstructedIntersectionPoint,
    },
}

/// Canonicalizes any nonempty zero- or one-dimensional convex intersection.
///
/// Narrow-phase edge walks can encounter more than two distinct points when
/// an otherwise valid polygon retains collinear boundary vertices. Selecting
/// exact lexicographic extrema preserves the whole interval instead of
/// silently truncating it to the first two discovery-order points.
fn exact_constructed_intersection_span(
    decisions: &DecisionContext,
    support: &Plane,
    points: &[ConstructedIntersectionPoint],
) -> HypermeshResult<ConstructedIntersectionSpan> {
    let Some(first) = points.first() else {
        return Ok(ConstructedIntersectionSpan::Empty);
    };
    match points {
        [_] => return Ok(ConstructedIntersectionSpan::Point(first.clone())),
        [v0, v1] => {
            return Ok(ConstructedIntersectionSpan::Segment {
                v0: v0.clone(),
                v1: v1.clone(),
            });
        }
        _ => {}
    }

    let mut minimum = first;
    let mut maximum = first;
    for point in &points[1..] {
        if compare_points_lexicographically(decisions, &point.point, &minimum.point)?.is_lt() {
            minimum = point;
        }
        if compare_points_lexicographically(decisions, &point.point, &maximum.point)?.is_gt() {
            maximum = point;
        }
    }
    for point in points {
        if !support.points_are_collinear_on_support(
            decisions,
            &minimum.point,
            &maximum.point,
            &point.point,
        )? {
            return Err(HypermeshError::UnknownClassification);
        }
    }
    Ok(ConstructedIntersectionSpan::Segment {
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
    plane_identity: Option<ConstructionPlaneIdentity>,
    points: &mut Vec<ConstructedIntersectionPoint>,
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
            plane_identity,
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
            plane_identity,
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
            plane_identity,
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
            plane_identity,
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
    plane_identity: Option<ConstructionPlaneIdentity>,
    points: &mut Vec<ConstructedIntersectionPoint>,
) -> HypermeshResult<()> {
    let candidate = match (start_class, end_class) {
        (Classification::On, _) => {
            affine_point_in_polygon_on_support(decisions, start, plane_polygon)?.then(|| {
                ConstructedIntersectionPoint {
                    point: start.clone(),
                    identity: plane_identity
                        .and_then(|_| polygon_vertex_identity(edge_polygon, edge_index)),
                }
            })
        }
        (_, Classification::On) => {
            affine_point_in_polygon_on_support(decisions, end, plane_polygon)?.then(|| {
                ConstructedIntersectionPoint {
                    point: end.clone(),
                    identity: plane_identity.and_then(|_| {
                        polygon_vertex_identity(
                            edge_polygon,
                            (edge_index + 1) % edge_polygon.vertex_count(),
                        )
                    }),
                }
            })
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
            contained.then(|| ConstructedIntersectionPoint {
                point,
                identity: edge_plane_intersection_identity(
                    edge_polygon,
                    edge_index,
                    plane_identity,
                ),
            })
        }
        _ => None,
    };

    if let Some(point) = candidate {
        points.push(point);
    }
    Ok(())
}

fn polygon_vertex_identity(
    polygon: &ConvexPolygon,
    vertex: usize,
) -> Option<ConstructionVertexIdentity> {
    polygon.known_vertex_identities()?.get(vertex)
}

fn edge_plane_intersection_identity(
    polygon: &ConvexPolygon,
    edge: usize,
    plane: Option<ConstructionPlaneIdentity>,
) -> Option<ConstructionVertexIdentity> {
    Some(
        polygon
            .known_edge_identities()?
            .get(edge)?
            .intersection_identity(plane?),
    )
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

fn exact_rational_points_contradict(left: &Point3, right: &Point3) -> bool {
    let left = [&left.x, &left.y, &left.z].map(Real::exact_rational_ref);
    let right = [&right.x, &right.y, &right.z].map(Real::exact_rational_ref);
    left.into_iter()
        .zip(right)
        .any(|(left, right)| matches!((left, right), (Some(left), Some(right)) if left != right))
}

fn construction_identity_fingerprint(identity: &ConstructionVertexIdentity) -> u64 {
    let mut hasher = StorageIdentityHasher::default();
    identity.hash(&mut hasher);
    hasher.finish()
}

fn dedup_constructed_points(
    decisions: &DecisionContext,
    points: &mut Vec<ConstructedIntersectionPoint>,
) -> HypermeshResult<()> {
    let mut unique: Vec<ConstructedIntersectionPoint> = Vec::with_capacity(points.len());
    for point in points.drain(..) {
        let mut duplicate = None;
        for (index, existing) in unique.iter().enumerate() {
            if existing.point == point.point
                || crate::predicate::points_equal(decisions, &existing.point, &point.point)?
            {
                duplicate = Some(index);
                break;
            }
        }
        if let Some(index) = duplicate {
            if point.identity.as_ref().is_some_and(|identity| {
                unique[index]
                    .identity
                    .as_ref()
                    .is_none_or(|existing| identity < existing)
            }) {
                unique[index].identity = point.identity;
            }
        } else {
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
        ConstructedIntersectionPoint, ConstructedIntersectionSegment, PairwiseIntersection,
        PairwiseIntersectionEvent, PairwiseIntersectionEventRef, PairwiseIntersectionGraphBuilder,
        PolygonVertexArena, StoredIntersectionKind, pairwise_intersections_by_polygon_from_bvh,
    };
    use crate::bvh::ExactBvh;
    use crate::error::HypermeshError;
    use crate::polygon::{ConstructionPlaneIdentity, ConstructionVertexIdentity};

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

        let bvh = ExactBvh::build_decision(&decisions, &[first]).unwrap();
        assert_eq!(
            pairwise_intersections_by_polygon_from_bvh(&decisions, &[], &[], &bvh).unwrap_err(),
            HypermeshError::SurfaceArrangementFailed {
                reason: "intersection hierarchy and source-face counts differ",
            }
        );
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
                ..
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
                ..
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
    fn retained_graph_points_carry_canonical_crossing_recipes() {
        let decisions = crate::test_support::approximate_decisions();
        let p = |x, y, z| Point3::new(Real::from(x), Real::from(y), Real::from(z));
        let mut horizontal = crate::test_support::approximate_convex_triangle(
            &p(0, 0, 0),
            &p(2, 0, 0),
            &p(0, 2, 0),
            0,
            0,
        );
        horizontal
            .set_source_triangle_edge_identities(0, [0, 1, 2])
            .unwrap();
        let mut vertical = crate::test_support::approximate_convex_triangle(
            &p(1, -1, -1),
            &p(1, 3, -1),
            &p(1, -1, 3),
            1,
            1,
        );
        vertical
            .set_source_triangle_edge_identities(1, [0, 1, 2])
            .unwrap();

        let graph =
            super::pairwise_intersections_by_polygon(&decisions, &[horizontal, vertical]).unwrap();

        assert_eq!(graph.points.len(), 2);
        assert_eq!(graph.point_identities.len(), graph.points.len());
        let support = ConstructionPlaneIdentity {
            mesh: super::PAIRWISE_FACE_PLANE_NAMESPACE,
            plane: 1,
        };
        let mut identities = graph
            .point_identities
            .iter()
            .cloned()
            .collect::<Option<Vec<_>>>()
            .expect("normalized source triangles give every crossing a recipe");
        identities.sort_unstable();
        assert_eq!(
            identities,
            vec![
                ConstructionVertexIdentity::SourceEdgePlane {
                    mesh: 0,
                    endpoints: [0, 1],
                    plane: support,
                },
                ConstructionVertexIdentity::SourceEdgePlane {
                    mesh: 0,
                    endpoints: [1, 2],
                    plane: support,
                },
            ]
        );

        let remapped = graph.remap_polygon_order(&[1, 0]).unwrap();
        let remapped_support = ConstructionPlaneIdentity {
            mesh: super::PAIRWISE_FACE_PLANE_NAMESPACE,
            plane: 0,
        };
        assert!(remapped.point_identities.iter().all(|identity| matches!(
            identity,
            Some(ConstructionVertexIdentity::SourceEdgePlane { plane, .. })
                if *plane == remapped_support
        )));

        let persistent = ConstructionPlaneIdentity { mesh: 3, plane: 7 };
        let mut planes = [
            persistent,
            ConstructionPlaneIdentity { mesh: 4, plane: 2 },
            support,
        ];
        planes.sort_unstable();
        let remapped_triple = super::remap_pairwise_construction_identity(
            &ConstructionVertexIdentity::PlaneTriple { planes },
            &[1, 0],
        )
        .unwrap();
        let mut expected = [
            persistent,
            ConstructionPlaneIdentity { mesh: 4, plane: 2 },
            remapped_support,
        ];
        expected.sort_unstable();
        assert_eq!(
            remapped_triple,
            ConstructionVertexIdentity::PlaneTriple { planes: expected }
        );
    }

    #[test]
    fn structural_recipe_interning_shares_symbolic_points_without_equality() {
        let symbolic = Point3::new(Real::from(2).sqrt().unwrap(), Real::zero(), Real::zero());
        let symbolic_identity = ConstructionVertexIdentity::Source { mesh: 0, vertex: 7 };
        let origin_identity = ConstructionVertexIdentity::Source { mesh: 0, vertex: 8 };
        let segment = |origin_vertex| ConstructedIntersectionSegment {
            v0: ConstructedIntersectionPoint {
                point: symbolic.clone(),
                identity: Some(symbolic_identity.clone()),
            },
            v1: ConstructedIntersectionPoint {
                point: Point3::origin(),
                identity: Some(ConstructionVertexIdentity::Source {
                    mesh: 0,
                    vertex: origin_vertex,
                }),
            },
        };
        let mut graph = PairwiseIntersectionGraphBuilder::new(3).unwrap();
        graph
            .append_constructed_segment_pair(
                0,
                1,
                segment(8),
                StoredIntersectionKind::NonCoplanarSegment,
            )
            .unwrap();
        graph
            .append_constructed_segment_pair(
                0,
                2,
                segment(8),
                StoredIntersectionKind::NonCoplanarSegment,
            )
            .unwrap();
        let graph = graph.finish().unwrap();

        assert_eq!(graph.points.len(), 2);
        assert_eq!(
            graph.segments[0].endpoints[0],
            graph.segments[1].endpoints[0]
        );
        assert_eq!(
            graph.point_identities[graph.segments[0].endpoints[0] as usize],
            Some(symbolic_identity)
        );
        assert_eq!(
            graph.point_identities[graph.segments[0].endpoints[1] as usize],
            Some(origin_identity)
        );
    }

    #[test]
    fn structural_recipe_interning_disambiguates_fingerprint_collisions() {
        let first_identity = ConstructionVertexIdentity::Source { mesh: 0, vertex: 4 };
        let second_identity = ConstructionVertexIdentity::Source { mesh: 0, vertex: 9 };
        let point = |coordinate, identity| ConstructedIntersectionPoint {
            point: Point3::new(Real::from(coordinate), Real::zero(), Real::zero()),
            identity: Some(identity),
        };
        let mut graph = PairwiseIntersectionGraphBuilder::new(3).unwrap();
        graph
            .append_constructed_point_pair(
                0,
                1,
                point(0, first_identity.clone()),
                StoredIntersectionKind::NonCoplanarPoint,
            )
            .unwrap();

        let first_fingerprint = super::construction_identity_fingerprint(&first_identity);
        let second_fingerprint = super::construction_identity_fingerprint(&second_identity);
        assert_ne!(first_fingerprint, second_fingerprint);
        assert_eq!(graph.construction_heads.insert(second_fingerprint, 0), None);
        graph
            .append_constructed_point_pair(
                0,
                2,
                point(1, second_identity.clone()),
                StoredIntersectionKind::CoplanarPoint,
            )
            .unwrap();
        let graph = graph.finish().unwrap();

        assert_eq!(graph.points.len(), 2);
        assert_eq!(
            graph.point_identities,
            [Some(first_identity), Some(second_identity)]
        );
    }

    #[test]
    fn structural_recipe_interning_rejects_exact_materialization_contradictions() {
        let identity = ConstructionVertexIdentity::Source { mesh: 0, vertex: 4 };
        let point = |coordinate| ConstructedIntersectionPoint {
            point: Point3::new(Real::from(coordinate), Real::zero(), Real::zero()),
            identity: Some(identity.clone()),
        };
        let mut graph = PairwiseIntersectionGraphBuilder::new(3).unwrap();
        graph
            .append_constructed_point_pair(0, 1, point(0), StoredIntersectionKind::NonCoplanarPoint)
            .unwrap();

        assert!(matches!(
            graph.append_constructed_point_pair(
                0,
                2,
                point(1),
                StoredIntersectionKind::CoplanarPoint,
            ),
            Err(crate::HypermeshError::UnknownClassification)
        ));
        let graph = graph.finish().unwrap();
        assert_eq!(graph.points.len(), 1);
        assert_eq!(graph.event_count(), 2);
    }

    #[test]
    fn exact_aliases_choose_order_independent_canonical_recipe() {
        let point = |vertex| ConstructedIntersectionPoint {
            point: Point3::origin(),
            identity: Some(ConstructionVertexIdentity::Source { mesh: 0, vertex }),
        };
        let build = |first, second| {
            let mut graph = PairwiseIntersectionGraphBuilder::new(3).unwrap();
            graph
                .append_constructed_point_pair(
                    0,
                    1,
                    point(first),
                    StoredIntersectionKind::NonCoplanarPoint,
                )
                .unwrap();
            graph
                .append_constructed_point_pair(
                    0,
                    2,
                    point(second),
                    StoredIntersectionKind::CoplanarPoint,
                )
                .unwrap();
            graph.finish().unwrap()
        };
        let graph = build(9, 2);
        let reversed = build(2, 9);

        assert_eq!(graph.points.len(), 1);
        assert_eq!(
            graph.point_identities,
            [Some(ConstructionVertexIdentity::Source {
                mesh: 0,
                vertex: 2,
            })]
        );
        assert_eq!(reversed.point_identities, graph.point_identities);
        let remapped = graph.remap_polygon_order(&[2, 1, 0]).unwrap();
        assert_eq!(
            remapped.point_identities,
            [Some(ConstructionVertexIdentity::Source {
                mesh: 0,
                vertex: 2,
            })]
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
