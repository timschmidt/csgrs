//! Pairwise convex polygon intersection primitives.

use std::hash::{Hash, Hasher};

use hyperlattice::{Point3, Real};

use crate::bvh::ExactBvh;
use crate::context::{DecisionContext, MeshCertainty, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, Plane, compare_real_decision};
use crate::point_interner::PointInterner;
use crate::polygon::{ConstructionPlaneIdentity, ConstructionVertexIdentity, ConvexPolygon};
use crate::predicate::{
    Point3PredicateQuery, classify_point_decision, classify_real, exact_rational_points_contradict,
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

#[derive(Clone, Copy, Debug)]
enum SupportLineAxis {
    X,
    Y,
    Z,
}

impl SupportLineAxis {
    fn coordinate(self, point: &Point3) -> &Real {
        match self {
            Self::X => &point.x,
            Self::Y => &point.y,
            Self::Z => &point.z,
        }
    }
}

#[derive(Clone, Debug)]
enum DeferredIntersectionGeometry<'a> {
    Affine {
        point: Point3,
        axis: SupportLineAxis,
        enclosure: Option<[f64; 2]>,
    },
    SegmentPlane {
        coordinate_numerator: Real,
        denominator: Real,
        denominator_is_positive: bool,
        enclosure: Option<[f64; 2]>,
        start: &'a Point3,
        end: &'a Point3,
        parameter_numerator: Real,
    },
}

#[derive(Clone, Debug)]
struct DeferredIntersectionPoint<'a> {
    geometry: DeferredIntersectionGeometry<'a>,
    identity: Option<ConstructionVertexIdentity>,
    discovery_order: (bool, usize),
}

#[derive(Default)]
struct DeferredIntersectionSpan<'a> {
    minimum: Option<DeferredIntersectionPoint<'a>>,
    maximum: Option<DeferredIntersectionPoint<'a>>,
}

struct DeferredIntersectionEndpoint<'span, 'point> {
    source: &'span DeferredIntersectionPoint<'point>,
    identity: Option<ConstructionVertexIdentity>,
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

#[derive(Default)]
struct PairwiseIntersectionScratch {
    points: Vec<ConstructedIntersectionPoint>,
    coplanar_classifications: Vec<Option<Classification>>,
    coplanar_queries: Vec<Option<Point3PredicateQuery>>,
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

fn compact_intersection_index(value: usize, operation: &'static str) -> HypermeshResult<u32> {
    u32::try_from(value).map_err(|_| HypermeshError::CapacityOverflow { operation })
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
    // Compact retained facts from authored adjacencies. Each pair proves that
    // its two source triangles can meet only on distinct radial rays along
    // their shared edge.
    radially_separated_face_pair_keys: Box<[u64]>,
}

pub(crate) fn source_face_pair_key(left: u32, right: u32) -> Option<u64> {
    if left == right {
        return None;
    }
    let [first, second] = if left < right {
        [left, right]
    } else {
        [right, left]
    };
    Some((u64::from(first) << u32::BITS) | u64::from(second))
}

impl PairwiseIntersectionGraph {
    pub(crate) fn len(&self) -> usize {
        self.offsets.len().saturating_sub(1)
    }

    pub(crate) fn construction_point_count(&self) -> usize {
        self.points.len()
    }

    pub(crate) fn event_ids(
        &self,
        face: usize,
    ) -> HypermeshResult<
        impl ExactSizeIterator<Item = HypermeshResult<PairwiseIntersectionEventIds>> + '_,
    > {
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
        let events = self.events.get(start as usize..end as usize).ok_or(
            HypermeshError::SurfaceArrangementFailed {
                reason: "source-face intersection row range is invalid",
            },
        )?;
        Ok(events.iter().map(|event| {
            let (kind, index) = decode_intersection_geometry(event.geometry);
            Ok(match kind {
                StoredIntersectionKind::NonCoplanarPoint => {
                    PairwiseIntersectionEventIds::NonCoplanarPoint {
                        point: compact_intersection_index(
                            index.ok_or(HypermeshError::SurfaceArrangementFailed {
                                reason: "point intersection event has no point index",
                            })?,
                            "intersection point event IDs",
                        )?,
                        other_polygon: event.other_polygon,
                    }
                }
                StoredIntersectionKind::NonCoplanarSegment => {
                    PairwiseIntersectionEventIds::NonCoplanarSegment {
                        endpoints: self
                            .segments
                            .get(index.ok_or(HypermeshError::SurfaceArrangementFailed {
                                reason: "segment intersection event has no segment index",
                            })?)
                            .ok_or(HypermeshError::SurfaceArrangementFailed {
                                reason: "intersection event references an absent segment",
                            })?
                            .endpoints,
                        other_polygon: event.other_polygon,
                    }
                }
                StoredIntersectionKind::CoplanarPoint => {
                    PairwiseIntersectionEventIds::CoplanarPoint {
                        point: compact_intersection_index(
                            index.ok_or(HypermeshError::SurfaceArrangementFailed {
                                reason: "point intersection event has no point index",
                            })?,
                            "intersection point event IDs",
                        )?,
                        other_polygon: event.other_polygon,
                    }
                }
                StoredIntersectionKind::CoplanarSegment => {
                    PairwiseIntersectionEventIds::CoplanarSegment {
                        endpoints: self
                            .segments
                            .get(index.ok_or(HypermeshError::SurfaceArrangementFailed {
                                reason: "segment intersection event has no segment index",
                            })?)
                            .ok_or(HypermeshError::SurfaceArrangementFailed {
                                reason: "intersection event references an absent segment",
                            })?
                            .endpoints,
                        other_polygon: event.other_polygon,
                    }
                }
                StoredIntersectionKind::CoplanarOverlap => {
                    PairwiseIntersectionEventIds::CoplanarOverlap {
                        other_polygon: event.other_polygon,
                    }
                }
            })
        }))
    }

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

    /// Consumes the graph and retains only source-face pairs whose authored
    /// reversed edge was proved to separate two distinct radial rays.
    pub(crate) fn into_radially_separated_face_pair_keys(self) -> Box<[u64]> {
        self.radially_separated_face_pair_keys
    }
}

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
    radially_separated_face_pair_keys: Vec<u64>,
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
        // A consistently oriented triangle surface ordinarily contributes
        // three face-edge incidences for every two authored adjacencies. This
        // is only a storage seed: nonmanifold inputs may exceed it and retain
        // the same checked growth path.
        let radial_pair_capacity =
            face_count
                .checked_add(face_count / 2)
                .ok_or(HypermeshError::CapacityOverflow {
                    operation: "radially separated source-face pairs",
                })?;
        let mut radially_separated_face_pair_keys = Vec::new();
        radially_separated_face_pair_keys
            .try_reserve_exact(radial_pair_capacity)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "radially separated source-face pairs",
            })?;
        Ok(Self {
            counts: counts.into_boxed_slice(),
            points: Vec::new(),
            point_identities: Vec::new(),
            construction_heads: StorageHashMap::default(),
            construction_aliases: Vec::new(),
            point_interner: PointInterner::new_exact_unreserved(),
            segments: Vec::new(),
            events: Vec::new(),
            radially_separated_face_pair_keys,
        })
    }

    fn append_radially_separated_face_pair(
        &mut self,
        left: usize,
        right: usize,
    ) -> HypermeshResult<()> {
        let pair = source_face_pair_key(self.face_id(left)?, self.face_id(right)?)
            .ok_or(HypermeshError::UnknownClassification)?;
        self.radially_separated_face_pair_keys
            .try_reserve(1)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "radially separated source-face pairs",
            })?;
        self.radially_separated_face_pair_keys.push(pair);
        Ok(())
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

    #[cfg(test)]
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
            mut radially_separated_face_pair_keys,
        } = self;
        drop(point_interner);
        drop(construction_heads);
        drop(construction_aliases);
        if point_identities.len() != points.len() {
            return Err(HypermeshError::UnknownClassification);
        }
        radially_separated_face_pair_keys.sort_unstable();
        radially_separated_face_pair_keys.dedup();

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
            radially_separated_face_pair_keys: radially_separated_face_pair_keys.into_boxed_slice(),
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
    let mut scratch = PairwiseIntersectionScratch::default();
    intersect_polygons_with_vertices_constructed(
        decisions,
        polygon,
        polygon_vertices,
        None,
        other,
        other_vertices,
        None,
        &mut scratch,
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
    scratch: &mut PairwiseIntersectionScratch,
) -> HypermeshResult<ConstructedPairwiseIntersection> {
    scratch.points.clear();
    scratch.coplanar_classifications.clear();
    scratch.coplanar_queries.clear();
    if polygon.vertex_count() == 0 || other.vertex_count() == 0 {
        return Ok(ConstructedPairwiseIntersection::Disjoint);
    }
    let Some(support_line_axis) =
        support_line_order_axis(decisions, &polygon.support, &other.support)?
    else {
        let retain_construction =
            polygon_support_identity.is_some() && other_support_identity.is_some();
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
                scratch,
            )
        } else {
            Ok(ConstructedPairwiseIntersection::Disjoint)
        };
    };

    intersect_nonparallel_polygons_constructed(
        decisions,
        polygon,
        polygon_vertices,
        polygon_support_identity,
        other,
        other_vertices,
        other_support_identity,
        support_line_axis,
    )
}

fn intersect_nonparallel_polygons_constructed<'point>(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    polygon_vertices: &'point [Point3],
    polygon_support_identity: Option<ConstructionPlaneIdentity>,
    other: &ConvexPolygon,
    other_vertices: &'point [Point3],
    other_support_identity: Option<ConstructionPlaneIdentity>,
    support_line_axis: SupportLineAxis,
) -> HypermeshResult<ConstructedPairwiseIntersection> {
    // A closed triangle reaches a plane only through an on-plane vertex or a
    // pair of vertices on opposite sides. Certifying this symmetric support
    // separator before constructing crossings avoids exact points that the
    // other direction would later reject. A successful triangle pair always
    // needs all six classifications, so retaining them changes no successful
    // policy-decision set.
    let triangle_classes = if let ([p0, p1, p2], [q0, q1, q2]) = (polygon_vertices, other_vertices)
    {
        let polygon_classes = [
            classify_point_decision(decisions, p0, &other.support)?,
            classify_point_decision(decisions, p1, &other.support)?,
            classify_point_decision(decisions, p2, &other.support)?,
        ];
        if !triangle_reaches_plane(polygon_classes) {
            crate::trace_dispatch!("intersect-polygons", "separating-support-plane");
            return Ok(ConstructedPairwiseIntersection::Disjoint);
        }
        let other_classes = [
            classify_point_decision(decisions, q0, &polygon.support)?,
            classify_point_decision(decisions, q1, &polygon.support)?,
            classify_point_decision(decisions, q2, &polygon.support)?,
        ];
        if !triangle_reaches_plane(other_classes) {
            crate::trace_dispatch!("intersect-polygons", "separating-support-plane");
            return Ok(ConstructedPairwiseIntersection::Disjoint);
        }
        Some([polygon_classes, other_classes])
    } else {
        None
    };

    crate::trace_dispatch!("intersect-polygons", "support-line-slice-forward");
    let polygon_span = collect_polygon_plane_slice(
        decisions,
        polygon,
        polygon_vertices,
        triangle_classes.as_ref().map(|classes| &classes[0]),
        &other.support,
        other_support_identity,
        support_line_axis,
        false,
    )?;

    crate::trace_dispatch!("intersect-polygons", "support-line-slice-reverse");
    let other_span = collect_polygon_plane_slice(
        decisions,
        other,
        other_vertices,
        triangle_classes.as_ref().map(|classes| &classes[1]),
        &polygon.support,
        polygon_support_identity,
        support_line_axis,
        true,
    )?;
    intersect_deferred_spans(decisions, polygon_span, other_span)
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
    let mut scratch = PairwiseIntersectionScratch::default();
    let mut failure = None;

    bvh.intersect_self_candidates_decision(decisions, |global_i, global_j| {
        if failure.is_some() {
            return;
        }
        if let Err(error) = append_pairwise_intersection(
            decisions,
            polygons,
            &vertices,
            certified_embedded_inputs,
            &mut graph,
            &mut scratch,
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
    let bvh = ExactBvh::build_decision(decisions, polygons)?;
    pairwise_intersections_by_polygon_from_bvh(decisions, polygons, &[], &bvh)
}

fn append_pairwise_intersection(
    decisions: &DecisionContext,
    polygons: &[ConvexPolygon],
    vertices: &PolygonVertexArena,
    certified_embedded_inputs: &[bool],
    graph: &mut PairwiseIntersectionGraphBuilder,
    scratch: &mut PairwiseIntersectionScratch,
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
        polygon_cycles_share_reversed_manifold_triangle_edge(
            decisions,
            left_vertices,
            &polygons[global_i],
            right_vertices,
            &polygons[global_j],
        )?
    } else {
        false
    };
    if same_mesh && shares_manifold_edge {
        crate::trace_dispatch!("pairwise-intersection", "known-manifold-edge");
        graph.append_radially_separated_face_pair(global_i, global_j)?;
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
        scratch,
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

fn polygon_cycles_share_reversed_manifold_triangle_edge(
    decisions: &DecisionContext,
    left: &[Point3],
    left_polygon: &ConvexPolygon,
    right: &[Point3],
    right_polygon: &ConvexPolygon,
) -> HypermeshResult<bool> {
    if left.len() != 3 || right.len() != 3 {
        return Ok(false);
    }
    let (Some(left_vertices), Some(left_edges), Some(right_vertices), Some(right_edges)) = (
        left_polygon.known_vertex_identities(),
        left_polygon.known_edge_identities(),
        right_polygon.known_vertex_identities(),
        right_polygon.known_edge_identities(),
    ) else {
        return Ok(false);
    };
    for left_index in 0..3 {
        let Some(left_edge) = left_edges.get(left_index) else {
            return Err(HypermeshError::UnknownClassification);
        };
        let Some(left_start) = left_vertices.get(left_index) else {
            return Err(HypermeshError::UnknownClassification);
        };
        let Some(left_end) = left_vertices.get((left_index + 1) % 3) else {
            return Err(HypermeshError::UnknownClassification);
        };
        for right_index in 0..3 {
            let Some(right_edge) = right_edges.get(right_index) else {
                return Err(HypermeshError::UnknownClassification);
            };
            if left_edge != right_edge {
                continue;
            }
            let Some(right_start) = right_vertices.get(right_index) else {
                return Err(HypermeshError::UnknownClassification);
            };
            let Some(right_end) = right_vertices.get((right_index + 1) % 3) else {
                return Err(HypermeshError::UnknownClassification);
            };
            if left_start != right_end || left_end != right_start {
                continue;
            }
            let left_opposite = &left[(left_index + 2) % 3];
            let right_opposite = &right[(right_index + 2) % 3];
            if classify_point_decision(decisions, right_opposite, &left_polygon.support)?
                != Classification::On
                || classify_point_decision(decisions, left_opposite, &right_polygon.support)?
                    != Classification::On
            {
                return Ok(true);
            }
            // Coplanar PWN neighbors meet only at their authored edge when
            // each opposite vertex lies strictly outside the other's edge
            // half-space. Same-side or collinear folds still reach the full
            // coplanar-overlap path.
            return Ok(classify_point_decision(
                decisions,
                right_opposite,
                &left_polygon.edges[left_index],
            )? == Classification::Positive
                && classify_point_decision(
                    decisions,
                    left_opposite,
                    &right_polygon.edges[right_index],
                )? == Classification::Positive);
        }
    }
    Ok(false)
}

fn intersect_coplanar_constructed(
    decisions: &DecisionContext,
    polygon: &ConvexPolygon,
    polygon_vertices: &[Point3],
    other: &ConvexPolygon,
    other_vertices: &[Point3],
    retain_construction: bool,
    scratch: &mut PairwiseIntersectionScratch,
) -> HypermeshResult<ConstructedPairwiseIntersection> {
    let PairwiseIntersectionScratch {
        points,
        coplanar_classifications,
        coplanar_queries,
    } = scratch;
    let mut relation = CoplanarClassificationCache::new(
        decisions,
        polygon,
        polygon_vertices,
        other,
        other_vertices,
        coplanar_classifications,
        coplanar_queries,
    )?;
    if relation.polygons_share_area()? {
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
    points
        .try_reserve_exact(capacity)
        .map_err(|_| HypermeshError::CapacityOverflow {
            operation: "coplanar polygon contact candidates",
        })?;
    for (index, point) in polygon_vertices.iter().enumerate() {
        if relation.left_vertex_is_contained(index)? {
            points.push(ConstructedIntersectionPoint {
                point: point.clone(),
                identity: retain_construction
                    .then(|| polygon_vertex_identity(polygon, index))
                    .flatten(),
            });
        }
    }
    for (index, point) in other_vertices.iter().enumerate() {
        if relation.right_vertex_is_contained(index)? {
            points.push(ConstructedIntersectionPoint {
                point: point.clone(),
                identity: retain_construction
                    .then(|| polygon_vertex_identity(other, index))
                    .flatten(),
            });
        }
    }
    dedup_constructed_points(decisions, points)?;

    match exact_constructed_intersection_span(decisions, &polygon.support, points)? {
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

/// Lazily retains the exact point/edge classifications shared by the
/// separating-axis and lower-dimensional-contact passes for one coplanar
/// polygon pair. The original predicate order and short-circuit behavior stay
/// intact, so this cache cannot consume a terminal policy decision that the
/// geometric result did not already require.
struct CoplanarClassificationCache<'geometry, 'scratch> {
    right_vertices_against_left: CoplanarClassificationMatrix<'geometry, 'scratch>,
    left_vertices_against_right: CoplanarClassificationMatrix<'geometry, 'scratch>,
}

struct CoplanarClassificationMatrix<'geometry, 'scratch> {
    decisions: &'geometry DecisionContext,
    container: &'geometry ConvexPolygon,
    vertices: &'geometry [Point3],
    values: &'scratch mut [Option<Classification>],
    queries: &'scratch mut [Option<Point3PredicateQuery>],
}

impl<'geometry, 'scratch> CoplanarClassificationCache<'geometry, 'scratch> {
    fn new(
        decisions: &'geometry DecisionContext,
        left: &'geometry ConvexPolygon,
        left_vertices: &'geometry [Point3],
        right: &'geometry ConvexPolygon,
        right_vertices: &'geometry [Point3],
        values: &'scratch mut Vec<Option<Classification>>,
        queries: &'scratch mut Vec<Option<Point3PredicateQuery>>,
    ) -> HypermeshResult<Self> {
        let left_matrix_len = left.edges.len().checked_mul(right_vertices.len()).ok_or(
            HypermeshError::CapacityOverflow {
                operation: "coplanar classification matrix",
            },
        )?;
        let right_matrix_len = right.edges.len().checked_mul(left_vertices.len()).ok_or(
            HypermeshError::CapacityOverflow {
                operation: "coplanar classification matrix",
            },
        )?;
        let len = left_matrix_len.checked_add(right_matrix_len).ok_or(
            HypermeshError::CapacityOverflow {
                operation: "coplanar classification matrix",
            },
        )?;
        values.clear();
        values
            .try_reserve_exact(len)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "coplanar classification matrix",
            })?;
        values.resize(len, None);
        let (left_values, right_values) = values.split_at_mut(left_matrix_len);
        let query_len = right_vertices
            .len()
            .checked_add(left_vertices.len())
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "coplanar point query scratch",
            })?;
        queries.clear();
        queries
            .try_reserve_exact(query_len)
            .map_err(|_| HypermeshError::CapacityOverflow {
                operation: "coplanar point query scratch",
            })?;
        queries.resize(query_len, None);
        let (right_queries, left_queries) = queries.split_at_mut(right_vertices.len());
        Ok(Self {
            right_vertices_against_left: CoplanarClassificationMatrix {
                decisions,
                container: left,
                vertices: right_vertices,
                values: left_values,
                queries: right_queries,
            },
            left_vertices_against_right: CoplanarClassificationMatrix {
                decisions,
                container: right,
                vertices: left_vertices,
                values: right_values,
                queries: left_queries,
            },
        })
    }

    fn polygons_share_area(&mut self) -> HypermeshResult<bool> {
        Ok(!self
            .right_vertices_against_left
            .has_open_interior_separator()?
            && !self
                .left_vertices_against_right
                .has_open_interior_separator()?)
    }

    fn left_vertex_is_contained(&mut self, vertex: usize) -> HypermeshResult<bool> {
        self.left_vertices_against_right.vertex_is_contained(vertex)
    }

    fn right_vertex_is_contained(&mut self, vertex: usize) -> HypermeshResult<bool> {
        self.right_vertices_against_left.vertex_is_contained(vertex)
    }
}

impl CoplanarClassificationMatrix<'_, '_> {
    /// Exact convex separating-axis test for positive-area coplanar overlap.
    ///
    /// Every edge plane bounds the polygon's negative halfspace. If every
    /// vertex of the other polygon is on or outside one edge, their open
    /// interiors are separated; if neither polygon supplies such an edge,
    /// their planar intersection has positive area.
    fn has_open_interior_separator(&mut self) -> HypermeshResult<bool> {
        if self.vertices.is_empty() {
            return Ok(true);
        }
        for edge in 0..self.container.edges.len() {
            let mut reaches_negative_halfspace = false;
            for vertex in 0..self.vertices.len() {
                if self.classification(edge, vertex)? == Classification::Negative {
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

    fn vertex_is_contained(&mut self, vertex: usize) -> HypermeshResult<bool> {
        let cached = self.cached_vertex_containment(vertex)?;
        if cached == Some(true) {
            return Ok(true);
        }
        if cached == Some(false) && self.decisions.certainty() == MeshCertainty::Certified {
            return Ok(false);
        }
        let point = self
            .vertices
            .get(vertex)
            .ok_or(HypermeshError::UnknownClassification)?;
        if self.container.has_retained_vertex(point) {
            return Ok(true);
        }
        if cached == Some(false) {
            return Ok(false);
        }

        let mut saw_unknown = false;
        for edge in 0..self.container.edges.len() {
            match self.classification(edge, vertex) {
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

    /// Reuses only predicates already demanded by the separating-axis walk.
    /// A fully populated non-positive column proves containment without
    /// another predicate. A certified cached positive proves exclusion;
    /// after any approximate terminal, retained vertex identity keeps its
    /// stronger original priority.
    fn cached_vertex_containment(&self, vertex: usize) -> HypermeshResult<Option<bool>> {
        if vertex >= self.vertices.len() {
            return Err(HypermeshError::UnknownClassification);
        }
        let mut missing = false;
        for edge in 0..self.container.edges.len() {
            match self
                .values
                .get(self.index(edge, vertex)?)
                .copied()
                .flatten()
            {
                Some(Classification::Positive) => return Ok(Some(false)),
                Some(Classification::Negative | Classification::On) => {}
                None => missing = true,
            }
        }
        Ok((!missing).then_some(true))
    }

    fn classification(&mut self, edge: usize, vertex: usize) -> HypermeshResult<Classification> {
        let index = self.index(edge, vertex)?;
        if let Some(classification) = self.values.get(index).copied().flatten() {
            return Ok(classification);
        }
        let point = self
            .vertices
            .get(vertex)
            .ok_or(HypermeshError::UnknownClassification)?;
        let plane = self
            .container
            .edges
            .get(edge)
            .ok_or(HypermeshError::UnknownClassification)?;
        let query = self
            .queries
            .get_mut(vertex)
            .ok_or(HypermeshError::UnknownClassification)?
            .get_or_insert_with(|| Point3PredicateQuery::new(point));
        let classification = query.classify(self.decisions, point, plane)?;
        *self
            .values
            .get_mut(index)
            .ok_or(HypermeshError::UnknownClassification)? = Some(classification);
        Ok(classification)
    }

    fn index(&self, edge: usize, vertex: usize) -> HypermeshResult<usize> {
        if edge >= self.container.edges.len() || vertex >= self.vertices.len() {
            return Err(HypermeshError::UnknownClassification);
        }
        edge.checked_mul(self.vertices.len())
            .and_then(|index| index.checked_add(vertex))
            .ok_or(HypermeshError::CapacityOverflow {
                operation: "coplanar classification matrix index",
            })
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

/// Intersects two closed convex slices on the line shared by nonparallel
/// polygon supports, then materializes only the surviving endpoints.
///
/// Any coordinate with a certified nonzero support-line direction is a
/// one-to-one affine parameter on that line. Comparing the retained rational
/// numerators and denominators therefore orders each complete closed slice
/// without performing three coordinate divisions for candidates that the
/// interval intersection will reject.
fn intersect_deferred_spans<'point>(
    decisions: &DecisionContext,
    left: DeferredIntersectionSpan<'point>,
    right: DeferredIntersectionSpan<'point>,
) -> HypermeshResult<ConstructedPairwiseIntersection> {
    let Some([left_minimum, left_maximum]) = deferred_span_interval(&left) else {
        return Ok(ConstructedPairwiseIntersection::Disjoint);
    };
    let Some([right_minimum, right_maximum]) = deferred_span_interval(&right) else {
        return Ok(ConstructedPairwiseIntersection::Disjoint);
    };

    let minimum = deferred_point_maximum(decisions, left_minimum, right_minimum)?;
    let maximum = deferred_point_minimum(decisions, left_maximum, right_maximum)?;
    match compare_deferred_points(decisions, minimum.source, maximum.source)? {
        std::cmp::Ordering::Greater => Ok(ConstructedPairwiseIntersection::Disjoint),
        std::cmp::Ordering::Equal => {
            let point =
                materialize_deferred_point(merge_equal_deferred_endpoints(minimum, maximum))?;
            Ok(ConstructedPairwiseIntersection::NonCoplanarPoint(point))
        }
        std::cmp::Ordering::Less => {
            let (v0, v1) = if minimum.source.discovery_order <= maximum.source.discovery_order {
                (minimum, maximum)
            } else {
                (maximum, minimum)
            };
            Ok(ConstructedPairwiseIntersection::NonCoplanarSegment(
                ConstructedIntersectionSegment {
                    v0: materialize_deferred_point(v0)?,
                    v1: materialize_deferred_point(v1)?,
                },
            ))
        }
    }
}

fn deferred_span_interval<'span, 'point>(
    span: &'span DeferredIntersectionSpan<'point>,
) -> Option<[&'span DeferredIntersectionPoint<'point>; 2]> {
    let minimum = span.minimum.as_ref()?;
    let maximum = span.maximum.as_ref().unwrap_or(minimum);
    Some([minimum, maximum])
}

fn deferred_point_maximum<'span, 'point>(
    decisions: &DecisionContext,
    left: &'span DeferredIntersectionPoint<'point>,
    right: &'span DeferredIntersectionPoint<'point>,
) -> HypermeshResult<DeferredIntersectionEndpoint<'span, 'point>> {
    Ok(match compare_deferred_points(decisions, left, right)? {
        std::cmp::Ordering::Less => deferred_endpoint(right),
        std::cmp::Ordering::Greater => deferred_endpoint(left),
        std::cmp::Ordering::Equal => merge_equal_deferred_sources(left, right),
    })
}

fn deferred_point_minimum<'span, 'point>(
    decisions: &DecisionContext,
    left: &'span DeferredIntersectionPoint<'point>,
    right: &'span DeferredIntersectionPoint<'point>,
) -> HypermeshResult<DeferredIntersectionEndpoint<'span, 'point>> {
    Ok(match compare_deferred_points(decisions, left, right)? {
        std::cmp::Ordering::Less => deferred_endpoint(left),
        std::cmp::Ordering::Greater => deferred_endpoint(right),
        std::cmp::Ordering::Equal => merge_equal_deferred_sources(left, right),
    })
}

fn extend_deferred_span<'point>(
    decisions: &DecisionContext,
    span: &mut DeferredIntersectionSpan<'point>,
    candidate: DeferredIntersectionPoint<'point>,
) -> HypermeshResult<()> {
    let Some(minimum) = span.minimum.as_ref() else {
        span.minimum = Some(candidate);
        return Ok(());
    };
    if span.maximum.is_none() {
        match compare_deferred_points(decisions, &candidate, minimum)? {
            std::cmp::Ordering::Less => {
                span.maximum = span.minimum.take();
                span.minimum = Some(candidate);
            }
            std::cmp::Ordering::Equal => merge_equal_deferred_point(
                span.minimum
                    .as_mut()
                    .expect("the deferred point span has a minimum"),
                candidate,
            ),
            std::cmp::Ordering::Greater => span.maximum = Some(candidate),
        }
        return Ok(());
    }

    match compare_deferred_points(decisions, &candidate, minimum)? {
        std::cmp::Ordering::Less => span.minimum = Some(candidate),
        std::cmp::Ordering::Equal => merge_equal_deferred_point(
            span.minimum
                .as_mut()
                .expect("the deferred segment has a minimum"),
            candidate,
        ),
        std::cmp::Ordering::Greater => {
            let maximum = span
                .maximum
                .as_mut()
                .expect("the deferred segment has a maximum");
            match compare_deferred_points(decisions, &candidate, maximum)? {
                std::cmp::Ordering::Less => {}
                std::cmp::Ordering::Equal => merge_equal_deferred_point(maximum, candidate),
                std::cmp::Ordering::Greater => {
                    *maximum = candidate;
                }
            }
        }
    }
    Ok(())
}

fn compare_deferred_points(
    decisions: &DecisionContext,
    left: &DeferredIntersectionPoint<'_>,
    right: &DeferredIntersectionPoint<'_>,
) -> HypermeshResult<std::cmp::Ordering> {
    match (&left.geometry, &right.geometry) {
        (
            DeferredIntersectionGeometry::Affine {
                point: left,
                axis: left_axis,
                enclosure: left_enclosure,
            },
            DeferredIntersectionGeometry::Affine {
                point: right,
                axis: right_axis,
                enclosure: right_enclosure,
            },
        ) => {
            if let (Some(left), Some(right)) = (left_enclosure, right_enclosure)
                && let Some(ordering) = certified_enclosure_ordering(*left, *right)
            {
                return Ok(ordering);
            }
            compare_real_decision(
                decisions,
                left_axis.coordinate(left),
                right_axis.coordinate(right),
            )
        }
        (
            DeferredIntersectionGeometry::SegmentPlane {
                coordinate_numerator: left_numerator,
                denominator: left_denominator,
                denominator_is_positive: left_positive,
                enclosure: left_enclosure,
                ..
            },
            DeferredIntersectionGeometry::SegmentPlane {
                coordinate_numerator: right_numerator,
                denominator: right_denominator,
                denominator_is_positive: right_positive,
                enclosure: right_enclosure,
                ..
            },
        ) => {
            if let (Some(left), Some(right)) = (left_enclosure, right_enclosure)
                && let Some(ordering) = certified_enclosure_ordering(*left, *right)
            {
                return Ok(ordering);
            }
            let ordering = classification_ordering(classify_two_product_difference(
                decisions,
                left_numerator,
                right_denominator,
                right_numerator,
                left_denominator,
            )?);
            Ok(if left_positive == right_positive {
                ordering
            } else {
                ordering.reverse()
            })
        }
        (
            DeferredIntersectionGeometry::SegmentPlane {
                coordinate_numerator,
                denominator,
                denominator_is_positive,
                enclosure,
                ..
            },
            DeferredIntersectionGeometry::Affine {
                point: affine,
                axis,
                enclosure: affine_enclosure,
            },
        ) => {
            if let (Some(left), Some(right)) = (*enclosure, *affine_enclosure)
                && let Some(ordering) = certified_enclosure_ordering(left, right)
            {
                return Ok(ordering);
            }
            compare_ratio_to_affine(
                decisions,
                coordinate_numerator,
                denominator,
                *denominator_is_positive,
                axis.coordinate(affine),
            )
        }
        (
            DeferredIntersectionGeometry::Affine {
                point: affine,
                axis,
                enclosure: affine_enclosure,
            },
            DeferredIntersectionGeometry::SegmentPlane {
                coordinate_numerator,
                denominator,
                denominator_is_positive,
                enclosure,
                ..
            },
        ) => {
            if let (Some(left), Some(right)) = (*affine_enclosure, *enclosure)
                && let Some(ordering) = certified_enclosure_ordering(left, right)
            {
                return Ok(ordering);
            }
            compare_ratio_to_affine(
                decisions,
                coordinate_numerator,
                denominator,
                *denominator_is_positive,
                axis.coordinate(affine),
            )
            .map(std::cmp::Ordering::reverse)
        }
    }
}

fn certified_enclosure_ordering(left: [f64; 2], right: [f64; 2]) -> Option<std::cmp::Ordering> {
    if left[1] < right[0] {
        Some(std::cmp::Ordering::Less)
    } else if left[0] > right[1] {
        Some(std::cmp::Ordering::Greater)
    } else if left[0] == left[1] && right[0] == right[1] && left[0] == right[0] {
        Some(std::cmp::Ordering::Equal)
    } else {
        None
    }
}

fn certified_real_enclosure(value: &Real) -> Option<[f64; 2]> {
    if let Some(exact) = value.to_f64_exact_dyadic() {
        return Some([exact, exact]);
    }
    value.exact_rational_ref()?.to_f64_enclosure()
}

fn affine_deferred_geometry(
    point: Point3,
    axis: SupportLineAxis,
) -> DeferredIntersectionGeometry<'static> {
    let enclosure = certified_real_enclosure(axis.coordinate(&point));
    DeferredIntersectionGeometry::Affine {
        point,
        axis,
        enclosure,
    }
}

fn certified_ratio_enclosure(
    numerator: &Real,
    denominator: &Real,
    denominator_is_positive: bool,
) -> Option<[f64; 2]> {
    let numerator = certified_real_enclosure(numerator)?;
    let denominator = certified_real_enclosure(denominator)?;
    if (denominator_is_positive && denominator[0] <= 0.0)
        || (!denominator_is_positive && denominator[1] >= 0.0)
    {
        return None;
    }
    let quotients = [
        numerator[0] / denominator[0],
        numerator[0] / denominator[1],
        numerator[1] / denominator[0],
        numerator[1] / denominator[1],
    ];
    if quotients.iter().any(|value| value.is_nan()) {
        return None;
    }
    let lower = quotients.into_iter().fold(f64::INFINITY, f64::min);
    let upper = quotients.into_iter().fold(f64::NEG_INFINITY, f64::max);
    Some([lower.next_down(), upper.next_up()])
}

fn compare_ratio_to_affine(
    decisions: &DecisionContext,
    numerator: &Real,
    denominator: &Real,
    denominator_is_positive: bool,
    affine: &Real,
) -> HypermeshResult<std::cmp::Ordering> {
    let classification = if let [Some(numerator), Some(denominator), Some(affine)] =
        [numerator, denominator, affine].map(Real::exact_rational_ref)
    {
        let one = hyperlattice::Rational::one_ref();
        match hyperlattice::Rational::signed_product_sum_ordering(
            [true, false],
            [[numerator, one], [denominator, affine]],
        ) {
            std::cmp::Ordering::Less => Classification::Negative,
            std::cmp::Ordering::Equal => Classification::On,
            std::cmp::Ordering::Greater => Classification::Positive,
        }
    } else {
        classify_real(decisions, &(numerator - &(denominator * affine)))?
    };
    let ordering = classification_ordering(classification);
    Ok(if denominator_is_positive {
        ordering
    } else {
        ordering.reverse()
    })
}

fn classification_ordering(classification: Classification) -> std::cmp::Ordering {
    match classification {
        Classification::Negative => std::cmp::Ordering::Less,
        Classification::On => std::cmp::Ordering::Equal,
        Classification::Positive => std::cmp::Ordering::Greater,
    }
}

fn deferred_endpoint<'span, 'point>(
    point: &'span DeferredIntersectionPoint<'point>,
) -> DeferredIntersectionEndpoint<'span, 'point> {
    DeferredIntersectionEndpoint {
        source: point,
        identity: point.identity.clone(),
    }
}

fn merge_equal_deferred_sources<'span, 'point>(
    left: &'span DeferredIntersectionPoint<'point>,
    right: &'span DeferredIntersectionPoint<'point>,
) -> DeferredIntersectionEndpoint<'span, 'point> {
    DeferredIntersectionEndpoint {
        source: if right.discovery_order < left.discovery_order {
            right
        } else {
            left
        },
        identity: canonical_deferred_identity(left.identity.as_ref(), right.identity.as_ref()),
    }
}

fn merge_equal_deferred_endpoints<'span, 'point>(
    mut left: DeferredIntersectionEndpoint<'span, 'point>,
    mut right: DeferredIntersectionEndpoint<'span, 'point>,
) -> DeferredIntersectionEndpoint<'span, 'point> {
    let identity = canonical_deferred_identity(left.identity.as_ref(), right.identity.as_ref());
    if right.source.discovery_order < left.source.discovery_order {
        right.identity = identity;
        right
    } else {
        left.identity = identity;
        left
    }
}

fn merge_equal_deferred_point<'point>(
    existing: &mut DeferredIntersectionPoint<'point>,
    mut candidate: DeferredIntersectionPoint<'point>,
) {
    let identity =
        canonical_deferred_identity(existing.identity.as_ref(), candidate.identity.as_ref());
    if candidate.discovery_order < existing.discovery_order {
        candidate.identity = identity;
        *existing = candidate;
    } else {
        existing.identity = identity;
    }
}

fn canonical_deferred_identity(
    left: Option<&ConstructionVertexIdentity>,
    right: Option<&ConstructionVertexIdentity>,
) -> Option<ConstructionVertexIdentity> {
    match (left, right) {
        (Some(left), Some(right)) => Some(left.min(right).clone()),
        (Some(left), None) => Some(left.clone()),
        (None, Some(right)) => Some(right.clone()),
        (None, None) => None,
    }
}

fn materialize_deferred_point(
    point: DeferredIntersectionEndpoint<'_, '_>,
) -> HypermeshResult<ConstructedIntersectionPoint> {
    let affine = match &point.source.geometry {
        DeferredIntersectionGeometry::Affine { point, .. } => point.clone(),
        DeferredIntersectionGeometry::SegmentPlane {
            start,
            end,
            parameter_numerator,
            denominator,
            ..
        } => intersect_segment_plane_from_ratio(start, end, parameter_numerator, denominator)?,
    };
    Ok(ConstructedIntersectionPoint {
        point: affine,
        identity: point.identity,
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

fn triangle_reaches_plane([c0, c1, c2]: [Classification; 3]) -> bool {
    c0 == Classification::On || c0 != c1 || c1 != c2
}

fn triangle_has_two_proper_plane_crossings([c0, c1, c2]: [Classification; 3]) -> bool {
    c0 != Classification::On
        && c1 != Classification::On
        && c2 != Classification::On
        && (c0 != c1 || c1 != c2)
}

/// Computes the closed slice of one convex polygon by a nonparallel plane.
///
/// Every emitted point lies on a polygon boundary edge and on `plane`. The
/// slice is therefore complete without testing those points against a second
/// polygon; the caller intersects the two convex slice intervals afterward.
fn collect_polygon_plane_slice<'point>(
    decisions: &DecisionContext,
    edge_polygon: &ConvexPolygon,
    vertices: &'point [Point3],
    retained_classifications: Option<&[Classification; 3]>,
    plane: &Plane,
    plane_identity: Option<ConstructionPlaneIdentity>,
    axis: SupportLineAxis,
    reverse_slice: bool,
) -> HypermeshResult<DeferredIntersectionSpan<'point>> {
    let mut span = DeferredIntersectionSpan::default();
    if let [v0, v1, v2] = vertices {
        let [c0, c1, c2] = match retained_classifications {
            Some(classifications) => *classifications,
            None => [
                classify_point_decision(decisions, v0, plane)?,
                classify_point_decision(decisions, v1, plane)?,
                classify_point_decision(decisions, v2, plane)?,
            ],
        };
        let support_values = triangle_has_two_proper_plane_crossings([c0, c1, c2]).then(|| {
            [
                plane.expression_at_point(v0),
                plane.expression_at_point(v1),
                plane.expression_at_point(v2),
            ]
        });
        extend_polygon_plane_slice_edge(
            decisions,
            &mut span,
            edge_polygon,
            0,
            0,
            v0,
            v1,
            c0,
            c1,
            support_values
                .as_ref()
                .map(|values| (&values[0], &values[1])),
            plane,
            plane_identity,
            axis,
            reverse_slice,
        )?;
        extend_polygon_plane_slice_edge(
            decisions,
            &mut span,
            edge_polygon,
            1,
            usize::from(c0 == Classification::On),
            v1,
            v2,
            c1,
            c2,
            support_values
                .as_ref()
                .map(|values| (&values[1], &values[2])),
            plane,
            plane_identity,
            axis,
            reverse_slice,
        )?;
        extend_polygon_plane_slice_edge(
            decisions,
            &mut span,
            edge_polygon,
            2,
            if c1 == Classification::On { 2 } else { 1 },
            v2,
            v0,
            c2,
            c0,
            support_values
                .as_ref()
                .map(|values| (&values[2], &values[0])),
            plane,
            plane_identity,
            axis,
            reverse_slice,
        )?;
        return Ok(span);
    }

    let Some(first) = vertices.first() else {
        return Ok(span);
    };
    let first_class = classify_point_decision(decisions, first, plane)?;
    let last_class = if vertices.len() == 1 {
        first_class
    } else {
        classify_point_decision(
            decisions,
            vertices
                .last()
                .expect("the nonempty polygon slice has a final vertex"),
            plane,
        )?
    };
    let mut previous_class = last_class;
    let mut start_class = first_class;
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        let end_class = if index + 1 == vertices.len() {
            first_class
        } else if index + 2 == vertices.len() {
            last_class
        } else {
            classify_point_decision(decisions, end, plane)?
        };
        let vertex_discovery_edge = if index == 0 || previous_class == Classification::On {
            index
        } else {
            index - 1
        };
        extend_polygon_plane_slice_edge(
            decisions,
            &mut span,
            edge_polygon,
            index,
            vertex_discovery_edge,
            start,
            end,
            start_class,
            end_class,
            None,
            plane,
            plane_identity,
            axis,
            reverse_slice,
        )?;
        previous_class = start_class;
        start_class = end_class;
    }
    Ok(span)
}

#[inline]
fn extend_polygon_plane_slice_edge<'point>(
    decisions: &DecisionContext,
    span: &mut DeferredIntersectionSpan<'point>,
    edge_polygon: &ConvexPolygon,
    edge_index: usize,
    vertex_discovery_edge: usize,
    start: &'point Point3,
    end: &'point Point3,
    start_class: Classification,
    end_class: Classification,
    support_values: Option<(&Real, &Real)>,
    plane: &Plane,
    plane_identity: Option<ConstructionPlaneIdentity>,
    axis: SupportLineAxis,
    reverse_slice: bool,
) -> HypermeshResult<()> {
    if start_class == Classification::On {
        extend_deferred_span(
            decisions,
            span,
            DeferredIntersectionPoint {
                geometry: affine_deferred_geometry(start.clone(), axis),
                identity: plane_identity
                    .and_then(|_| polygon_vertex_identity(edge_polygon, edge_index)),
                discovery_order: (reverse_slice, vertex_discovery_edge),
            },
        )?;
    }

    if !matches!(
        (start_class, end_class),
        (Classification::Negative, Classification::Positive)
            | (Classification::Positive, Classification::Negative)
    ) {
        return Ok(());
    }

    let owned_support_values;
    let (start_support, end_support) = match support_values {
        Some(values) => values,
        None => {
            owned_support_values = (
                plane.expression_at_point(start),
                plane.expression_at_point(end),
            );
            (&owned_support_values.0, &owned_support_values.1)
        }
    };
    let parameter_denominator = start_support - end_support;
    let coordinate_numerator = Real::signed_product_sum(
        [true, false],
        [
            [start_support, axis.coordinate(end)],
            [end_support, axis.coordinate(start)],
        ],
    );
    let denominator_is_positive = start_class == Classification::Positive;
    let enclosure = certified_ratio_enclosure(
        &coordinate_numerator,
        &parameter_denominator,
        denominator_is_positive,
    );
    extend_deferred_span(
        decisions,
        span,
        DeferredIntersectionPoint {
            geometry: DeferredIntersectionGeometry::SegmentPlane {
                coordinate_numerator,
                denominator: parameter_denominator,
                denominator_is_positive,
                enclosure,
                start,
                end,
                parameter_numerator: start_support.clone(),
            },
            identity: edge_plane_intersection_identity(edge_polygon, edge_index, plane_identity),
            discovery_order: (reverse_slice, edge_index),
        },
    )
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

fn intersect_segment_plane_from_ratio(
    start: &Point3,
    end: &Point3,
    numerator: &Real,
    denominator: &Real,
) -> HypermeshResult<Point3> {
    if [
        &start.x,
        &start.y,
        &start.z,
        &end.x,
        &end.y,
        &end.z,
        numerator,
        denominator,
    ]
    .into_iter()
    .all(Real::is_exact_dyadic_rational)
    {
        let [x, y, z] = Real::exact_rational_interpolate_point3_known_dyadic(
            [&start.x, &start.y, &start.z],
            [&end.x, &end.y, &end.z],
            numerator,
            denominator,
        )
        .map_err(|_| HypermeshError::UnknownClassification)?;
        return Ok(Point3::new(x, y, z));
    }
    let t = (numerator / denominator).map_err(|_| HypermeshError::UnknownClassification)?;

    Ok(Point3::new(
        &start.x + &(t.clone() * (&end.x - &start.x)),
        &start.y + &(t.clone() * (&end.y - &start.y)),
        &start.z + &(t * (&end.z - &start.z)),
    ))
}

fn support_line_order_axis(
    decisions: &DecisionContext,
    left: &Plane,
    right: &Plane,
) -> HypermeshResult<Option<SupportLineAxis>> {
    let components = [
        (
            SupportLineAxis::X,
            &left.normal.y,
            &right.normal.z,
            &left.normal.z,
            &right.normal.y,
        ),
        (
            SupportLineAxis::Y,
            &left.normal.z,
            &right.normal.x,
            &left.normal.x,
            &right.normal.z,
        ),
        (
            SupportLineAxis::Z,
            &left.normal.x,
            &right.normal.y,
            &left.normal.y,
            &right.normal.x,
        ),
    ];
    let mut unresolved = 0_u8;
    for (index, &(axis, a, b, c, d)) in components.iter().enumerate() {
        match probe_two_product_difference_strict(decisions, a, b, c, d) {
            Some(Classification::Negative | Classification::Positive) => return Ok(Some(axis)),
            Some(Classification::On) => {}
            None => unresolved |= 1 << index,
        }
    }
    if unresolved == 0 {
        return Ok(None);
    }
    if decisions.policy() == hyperlimit::PredicatePolicy::STRICT {
        return Err(HypermeshError::PredicateUndecided {
            predicate: "polygon support-plane parallelism",
        });
    }

    // Parallelism is a composite predicate. Only after every component has
    // exhausted its strict proof path may APPROXIMATE_512 resolve the still
    // unknown components. A later certified nonzero component therefore keeps
    // the whole support-line decision certified.
    for (index, &(axis, a, b, c, d)) in components.iter().enumerate() {
        if unresolved & (1 << index) != 0
            && classify_two_product_difference(decisions, a, b, c, d)? != Classification::On
        {
            return Ok(Some(axis));
        }
    }
    Ok(None)
}

fn probe_two_product_difference_strict(
    decisions: &DecisionContext,
    a: &Real,
    b: &Real,
    c: &Real,
    d: &Real,
) -> Option<Classification> {
    if let Some(classification) = classify_exact_two_product_difference(a, b, c, d) {
        return Some(classification);
    }
    let value = Real::signed_product_sum([true, false], [[a, b], [c, d]]);
    decisions
        .probe(hyperlimit::classify_real_sign(
            &value,
            hyperlimit::PredicatePolicy::STRICT,
        ))
        .map(|sign| match sign {
            hyperlimit::Sign::Negative => Classification::Negative,
            hyperlimit::Sign::Zero => Classification::On,
            hyperlimit::Sign::Positive => Classification::Positive,
        })
}

fn classify_exact_two_product_difference(
    a: &Real,
    b: &Real,
    c: &Real,
    d: &Real,
) -> Option<Classification> {
    let [Some(a), Some(b), Some(c), Some(d)] = [a, b, c, d].map(Real::exact_rational_ref) else {
        return None;
    };
    Some(
        match hyperlattice::Rational::signed_product_sum_ordering([true, false], [[a, b], [c, d]]) {
            std::cmp::Ordering::Less => Classification::Negative,
            std::cmp::Ordering::Equal => Classification::On,
            std::cmp::Ordering::Greater => Classification::Positive,
        },
    )
}

fn classify_two_product_difference(
    decisions: &DecisionContext,
    a: &Real,
    b: &Real,
    c: &Real,
    d: &Real,
) -> HypermeshResult<Classification> {
    if let Some(classification) = classify_exact_two_product_difference(a, b, c, d) {
        return Ok(classification);
    }
    classify_real(
        decisions,
        &Real::signed_product_sum([true, false], [[a, b], [c, d]]),
    )
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
    let mut candidate = 0;
    while candidate < points.len() {
        let mut duplicate = None;
        for existing in 0..candidate {
            if points[existing].point == points[candidate].point
                || crate::predicate::points_equal(
                    decisions,
                    &points[existing].point,
                    &points[candidate].point,
                )?
            {
                duplicate = Some(existing);
                break;
            }
        }
        if let Some(existing) = duplicate {
            let point = points.remove(candidate);
            if point.identity.as_ref().is_some_and(|identity| {
                points[existing]
                    .identity
                    .as_ref()
                    .is_none_or(|existing| identity < existing)
            }) {
                points[existing].identity = point.identity;
            }
        } else {
            candidate += 1;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use hyperlattice::{Point3, Real};

    use super::{
        ConstructedIntersectionPoint, ConstructedIntersectionSegment,
        ConstructedPairwiseIntersection, CoplanarClassificationMatrix,
        DeferredIntersectionGeometry, DeferredIntersectionPoint, DeferredIntersectionSpan,
        PairwiseIntersection, PairwiseIntersectionEvent, PairwiseIntersectionEventIds,
        PairwiseIntersectionGraphBuilder, PairwiseIntersectionScratch, PolygonVertexArena,
        StoredIntersectionKind, affine_deferred_geometry, certified_enclosure_ordering,
        certified_ratio_enclosure, classify_two_product_difference, compare_deferred_points,
        dedup_constructed_points, intersect_deferred_spans,
        intersect_polygons_with_vertices_constructed, pairwise_intersections_by_polygon_from_bvh,
        polygon_cycles_share_reversed_manifold_triangle_edge, source_face_pair_key,
        support_line_order_axis, triangle_has_two_proper_plane_crossings, triangle_reaches_plane,
    };
    use crate::bvh::ExactBvh;
    use crate::context::{DecisionContext, MeshContext};
    use crate::error::HypermeshError;
    use crate::geometry::{Classification, Plane};
    use crate::polygon::{ConstructionPlaneIdentity, ConstructionVertexIdentity};

    fn deferred_point_span<'point>(
        point: DeferredIntersectionPoint<'point>,
    ) -> DeferredIntersectionSpan<'point> {
        DeferredIntersectionSpan {
            minimum: Some(point),
            maximum: None,
        }
    }

    fn deferred_segment_span<'point>(
        minimum: DeferredIntersectionPoint<'point>,
        maximum: DeferredIntersectionPoint<'point>,
    ) -> DeferredIntersectionSpan<'point> {
        DeferredIntersectionSpan {
            minimum: Some(minimum),
            maximum: Some(maximum),
        }
    }

    #[test]
    fn deferred_support_line_carrier_stays_stack_compact() {
        let point_bytes = core::mem::size_of::<DeferredIntersectionPoint<'static>>();
        let span_bytes = core::mem::size_of::<DeferredIntersectionSpan<'static>>();
        assert!(
            point_bytes <= 256,
            "deferred point grew to {point_bytes} bytes"
        );
        assert!(
            span_bytes <= 528,
            "deferred span grew to {span_bytes} bytes"
        );
    }

    #[test]
    fn triangle_plane_prepass_rejects_only_one_open_halfspace() {
        let values = [
            Classification::Negative,
            Classification::On,
            Classification::Positive,
        ];
        for c0 in values {
            for c1 in values {
                for c2 in values {
                    let classifications = [c0, c1, c2];
                    let expected = classifications.contains(&Classification::On)
                        || (classifications.contains(&Classification::Negative)
                            && classifications.contains(&Classification::Positive));
                    assert_eq!(triangle_reaches_plane(classifications), expected);
                }
            }
        }
    }

    #[test]
    fn triangle_support_values_are_shared_only_across_two_proper_crossings() {
        let values = [
            Classification::Negative,
            Classification::On,
            Classification::Positive,
        ];
        for c0 in values {
            for c1 in values {
                for c2 in values {
                    let classifications = [c0, c1, c2];
                    let proper_crossings = [(c0, c1), (c1, c2), (c2, c0)]
                        .into_iter()
                        .filter(|(start, end)| {
                            matches!(
                                (start, end),
                                (Classification::Negative, Classification::Positive)
                                    | (Classification::Positive, Classification::Negative)
                            )
                        })
                        .count();
                    assert_eq!(
                        triangle_has_two_proper_plane_crossings(classifications),
                        proper_crossings == 2,
                    );
                }
            }
        }
    }

    #[test]
    fn sign_only_two_product_difference_matches_materialized_value() {
        let decisions = crate::test_support::approximate_decisions();
        for seed in 0_i64..512 {
            let value = |multiplier: i64, addend: i64| {
                Real::from((seed * multiplier + addend).rem_euclid(43) - 21)
            };
            let [a, b, c, d] = [value(3, 1), value(5, 2), value(7, 3), value(11, 4)];
            let materialized = Real::signed_product_sum([true, false], [[&a, &b], [&c, &d]]);
            let expected = crate::predicate::classify_real(&decisions, &materialized).unwrap();
            assert_eq!(
                classify_two_product_difference(&decisions, &a, &b, &c, &d).unwrap(),
                expected,
                "seed={seed}",
            );
        }
    }

    #[test]
    fn certified_deferred_ratio_order_matches_materialized_exact_rationals() {
        let decisions = crate::test_support::approximate_decisions();
        let origin = Point3::origin();
        let endpoint = |numerator: i64, denominator: i64| {
            let denominator_is_positive = denominator > 0;
            let numerator = Real::from(numerator);
            let denominator = Real::from(denominator);
            let enclosure =
                certified_ratio_enclosure(&numerator, &denominator, denominator_is_positive);
            DeferredIntersectionPoint {
                geometry: DeferredIntersectionGeometry::SegmentPlane {
                    coordinate_numerator: numerator,
                    denominator,
                    denominator_is_positive,
                    enclosure,
                    start: &origin,
                    end: &origin,
                    parameter_numerator: Real::zero(),
                },
                identity: None,
                discovery_order: (false, 0),
            }
        };

        let mut enclosure_decisions = 0;
        for seed in 0_i64..1_024 {
            let nonzero = |value: i64| if value == 0 { 1 } else { value };
            let left_numerator = (seed * 5 + 3).rem_euclid(47) - 23;
            let left_denominator = nonzero((seed * 7 + 1).rem_euclid(37) - 18);
            let right_numerator = (seed * 11 + 5).rem_euclid(43) - 21;
            let right_denominator = nonzero((seed * 13 + 2).rem_euclid(41) - 20);
            let left_value = (&Real::from(left_numerator) / &Real::from(left_denominator)).unwrap();
            let right_value =
                (&Real::from(right_numerator) / &Real::from(right_denominator)).unwrap();
            let expected = left_value
                .exact_rational_ref()
                .unwrap()
                .partial_cmp(right_value.exact_rational_ref().unwrap())
                .unwrap();
            let left = endpoint(left_numerator, left_denominator);
            let right = endpoint(right_numerator, right_denominator);
            if let (
                DeferredIntersectionGeometry::SegmentPlane {
                    enclosure: Some(left),
                    ..
                },
                DeferredIntersectionGeometry::SegmentPlane {
                    enclosure: Some(right),
                    ..
                },
            ) = (&left.geometry, &right.geometry)
                && let Some(filtered) = certified_enclosure_ordering(*left, *right)
            {
                enclosure_decisions += 1;
                assert_eq!(filtered, expected, "enclosure seed={seed}");
            }

            assert_eq!(
                compare_deferred_points(&decisions, &left, &right).unwrap(),
                expected,
                "seed={seed}",
            );
        }
        assert!(enclosure_decisions > 900);
        assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
    }

    #[test]
    fn symbolic_support_parallelism_obeys_terminal_policy() {
        let left_value = Real::pi() + Real::e();
        let right_value = Real::e() + Real::pi();
        let left = Plane::from_coefficients(Real::zero(), left_value, right_value, Real::zero());
        let right = Plane::from_coefficients(Real::zero(), Real::one(), Real::one(), Real::zero());

        let strict_context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let strict = DecisionContext::new(&strict_context);
        assert!(matches!(
            support_line_order_axis(&strict, &left, &right),
            Err(HypermeshError::PredicateUndecided { .. })
        ));
        assert_eq!(strict.certainty(), crate::MeshCertainty::Certified);

        let approximate_context = MeshContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);
        let approximate = DecisionContext::new(&approximate_context);
        assert!(
            support_line_order_axis(&approximate, &left, &right)
                .unwrap()
                .is_none()
        );
        assert_eq!(
            approximate.certainty(),
            crate::MeshCertainty::Approximate512Consumed
        );
    }

    #[test]
    fn later_certified_support_component_avoids_terminal_approximation() {
        let first = Real::pi() + Real::e();
        let equivalent = Real::e() + Real::pi();
        let left = Plane::from_coefficients(Real::one(), first, equivalent, Real::zero());
        let right = Plane::from_coefficients(Real::zero(), Real::one(), Real::one(), Real::zero());

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            assert!(matches!(
                support_line_order_axis(&decisions, &left, &right),
                Ok(Some(super::SupportLineAxis::Y))
            ));
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        }
    }

    #[test]
    fn exact_closed_slice_intervals_cover_every_overlap_dimension_and_order() {
        fn point<'point>(
            positions: &'point [Point3; 6],
            x: i64,
            negative_denominator: bool,
        ) -> DeferredIntersectionPoint<'point> {
            let point = &positions[x as usize];
            DeferredIntersectionPoint {
                geometry: if negative_denominator {
                    DeferredIntersectionGeometry::SegmentPlane {
                        coordinate_numerator: Real::from(-x),
                        denominator: Real::from(-1),
                        denominator_is_positive: false,
                        enclosure: None,
                        start: point,
                        end: point,
                        parameter_numerator: Real::zero(),
                    }
                } else {
                    affine_deferred_geometry(point.clone(), super::SupportLineAxis::X)
                },
                identity: None,
                discovery_order: (false, x as usize),
            }
        }

        fn span<'point>(
            positions: &'point [Point3; 6],
            interval: Option<(i64, i64)>,
            negative_denominator: bool,
        ) -> DeferredIntersectionSpan<'point> {
            match interval {
                None => DeferredIntersectionSpan::default(),
                Some((minimum, maximum)) if minimum == maximum => {
                    deferred_point_span(point(positions, minimum, negative_denominator))
                }
                Some((minimum, maximum)) => deferred_segment_span(
                    point(positions, minimum, negative_denominator),
                    point(positions, maximum, negative_denominator),
                ),
            }
        }

        let cases = [
            (None, Some((0, 1)), None),
            (Some((0, 1)), Some((2, 3)), None),
            (Some((1, 1)), Some((1, 1)), Some((1, 1))),
            (Some((1, 1)), Some((0, 2)), Some((1, 1))),
            (Some((0, 1)), Some((1, 2)), Some((1, 1))),
            (Some((0, 3)), Some((1, 2)), Some((1, 2))),
            (Some((0, 3)), Some((2, 5)), Some((2, 3))),
            (Some((4, 4)), Some((0, 3)), None),
        ];
        let positions =
            std::array::from_fn(|x| Point3::new(Real::from(x as i64), Real::zero(), Real::zero()));

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            for &(left, right, expected) in &cases {
                for (left, right) in [(left, right), (right, left)] {
                    for (left_negative, right_negative) in
                        [(false, false), (false, true), (true, false), (true, true)]
                    {
                        let context = MeshContext::new(policy);
                        let decisions = DecisionContext::new(&context);
                        let actual = intersect_deferred_spans(
                            &decisions,
                            span(&positions, left, left_negative),
                            span(&positions, right, right_negative),
                        )
                        .unwrap();
                        let actual = match actual {
                            ConstructedPairwiseIntersection::Disjoint => None,
                            ConstructedPairwiseIntersection::NonCoplanarPoint(point) => {
                                Some((point.point.x.clone(), point.point.x))
                            }
                            ConstructedPairwiseIntersection::NonCoplanarSegment(segment) => {
                                Some((segment.v0.point.x, segment.v1.point.x))
                            }
                            _ => {
                                panic!(
                                    "closed support-line slices cannot produce a coplanar result"
                                )
                            }
                        };
                        assert_eq!(
                            actual,
                            expected.map(|(minimum, maximum)| {
                                (Real::from(minimum), Real::from(maximum))
                            }),
                            "left={left:?}, right={right:?}, policy={policy:?}",
                        );
                        assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
                    }
                }
            }
        }
    }

    #[test]
    fn surviving_deferred_crossing_materializes_from_its_retained_ratio() {
        let start = Point3::origin();
        let end = Point3::new(Real::from(10), Real::zero(), Real::zero());
        let crossing = DeferredIntersectionPoint {
            geometry: DeferredIntersectionGeometry::SegmentPlane {
                coordinate_numerator: Real::from(-50),
                denominator: Real::from(-10),
                denominator_is_positive: false,
                enclosure: None,
                start: &start,
                end: &end,
                parameter_numerator: Real::from(-5),
            },
            identity: None,
            discovery_order: (false, 0),
        };
        let affine = DeferredIntersectionPoint {
            geometry: affine_deferred_geometry(
                Point3::new(Real::from(5), Real::zero(), Real::zero()),
                super::SupportLineAxis::X,
            ),
            identity: None,
            discovery_order: (true, 0),
        };
        let decisions = crate::test_support::approximate_decisions();

        let result = intersect_deferred_spans(
            &decisions,
            deferred_point_span(crossing),
            deferred_point_span(affine),
        )
        .unwrap();
        let ConstructedPairwiseIntersection::NonCoplanarPoint(point) = result else {
            panic!("equal deferred endpoints must produce one materialized point");
        };
        assert_eq!(
            point.point,
            Point3::new(Real::from(5), Real::zero(), Real::zero())
        );
        assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
    }

    #[test]
    fn equal_slice_endpoints_choose_the_canonical_construction_identity() {
        let identity = |vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex };
        let point = |vertex| DeferredIntersectionPoint {
            geometry: affine_deferred_geometry(Point3::origin(), super::SupportLineAxis::X),
            identity: Some(identity(vertex)),
            discovery_order: (false, 0),
        };
        let decisions = crate::test_support::approximate_decisions();

        for (left, right) in [(9, 2), (2, 9)] {
            let result = intersect_deferred_spans(
                &decisions,
                deferred_point_span(point(left)),
                deferred_point_span(point(right)),
            )
            .unwrap();
            let ConstructedPairwiseIntersection::NonCoplanarPoint(result) = result else {
                panic!("equal point slices must intersect in one point");
            };
            assert_eq!(result.identity, Some(identity(2)));
        }
        assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
    }

    #[test]
    fn symbolic_slice_endpoint_equality_obeys_terminal_policy() {
        let origin = Point3::origin();
        let point = |x: Real, ratio: bool| {
            deferred_point_span(DeferredIntersectionPoint {
                geometry: if ratio {
                    DeferredIntersectionGeometry::SegmentPlane {
                        coordinate_numerator: x.clone(),
                        denominator: Real::one(),
                        denominator_is_positive: true,
                        enclosure: None,
                        start: &origin,
                        end: &origin,
                        parameter_numerator: Real::zero(),
                    }
                } else {
                    affine_deferred_geometry(
                        Point3::new(x, Real::zero(), Real::zero()),
                        super::SupportLineAxis::X,
                    )
                },
                identity: None,
                discovery_order: (false, 0),
            })
        };

        for (left_ratio, right_ratio) in
            [(false, false), (false, true), (true, false), (true, true)]
        {
            let strict_context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
            let strict = DecisionContext::new(&strict_context);
            assert!(matches!(
                intersect_deferred_spans(
                    &strict,
                    point(Real::pi() + Real::e(), left_ratio),
                    point(Real::e() + Real::pi(), right_ratio),
                ),
                Err(HypermeshError::PredicateUndecided { .. })
            ));
            assert_eq!(strict.certainty(), crate::MeshCertainty::Certified);

            let approximate_context =
                MeshContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);
            let approximate = DecisionContext::new(&approximate_context);
            assert!(matches!(
                intersect_deferred_spans(
                    &approximate,
                    point(Real::pi() + Real::e(), left_ratio),
                    point(Real::e() + Real::pi(), right_ratio),
                )
                .unwrap(),
                ConstructedPairwiseIntersection::NonCoplanarPoint(_)
            ));
            assert_eq!(
                approximate.certainty(),
                crate::MeshCertainty::Approximate512Consumed
            );
        }
    }

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
    fn pairwise_scratch_drops_prior_points_and_resets_coplanar_classifications() {
        let decisions = crate::test_support::approximate_decisions();
        let triangle = crate::test_support::approximate_convex_triangle(
            &Point3::origin(),
            &Point3::new(Real::from(4), Real::zero(), Real::zero()),
            &Point3::new(Real::zero(), Real::from(4), Real::zero()),
            0,
            0,
        );
        let disjoint = crate::test_support::approximate_convex_triangle(
            &Point3::new(Real::from(8), Real::zero(), Real::zero()),
            &Point3::new(Real::from(9), Real::zero(), Real::zero()),
            &Point3::new(Real::from(8), Real::one(), Real::zero()),
            1,
            0,
        );
        let triangle_vertices = triangle.vertices_decision(&decisions).unwrap();
        let disjoint_vertices = disjoint.vertices_decision(&decisions).unwrap();
        let mut scratch = PairwiseIntersectionScratch::default();
        scratch.points.push(ConstructedIntersectionPoint {
            point: Point3::new(Real::pi(), Real::e(), Real::one()),
            identity: None,
        });

        assert!(matches!(
            intersect_polygons_with_vertices_constructed(
                &decisions,
                &triangle,
                &triangle_vertices,
                None,
                &triangle,
                &triangle_vertices,
                None,
                &mut scratch,
            )
            .unwrap(),
            ConstructedPairwiseIntersection::CoplanarOverlap
        ));
        assert!(scratch.points.is_empty());
        assert!(scratch.coplanar_classifications.iter().any(Option::is_some));
        assert!(scratch.coplanar_queries.iter().any(Option::is_some));

        assert!(matches!(
            intersect_polygons_with_vertices_constructed(
                &decisions,
                &triangle,
                &triangle_vertices,
                None,
                &disjoint,
                &disjoint_vertices,
                None,
                &mut scratch,
            )
            .unwrap(),
            ConstructedPairwiseIntersection::Disjoint
        ));
        assert!(scratch.points.is_empty());
        assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
    }

    #[test]
    fn approximate_cached_exclusion_does_not_override_retained_vertex_identity() {
        let decisions = crate::test_support::approximate_decisions();
        decisions.absorb(crate::MeshCertainty::Approximate512Consumed);
        let triangle = crate::test_support::approximate_convex_triangle(
            &Point3::origin(),
            &Point3::new(Real::from(4), Real::zero(), Real::zero()),
            &Point3::new(Real::zero(), Real::from(4), Real::zero()),
            0,
            0,
        );
        let point = triangle.vertices_decision(&decisions).unwrap()[0].clone();
        let vertices = [point];
        let mut values = vec![None; triangle.edges.len()];
        values[0] = Some(Classification::Positive);
        let mut queries = vec![None];
        let mut matrix = CoplanarClassificationMatrix {
            decisions: &decisions,
            container: &triangle,
            vertices: &vertices,
            values: &mut values,
            queries: &mut queries,
        };

        assert!(matrix.vertex_is_contained(0).unwrap());
        assert_eq!(
            decisions.certainty(),
            crate::MeshCertainty::Approximate512Consumed
        );
    }

    #[test]
    fn constructed_point_deduplication_retains_capacity_and_canonical_recipe() {
        let decisions = crate::test_support::approximate_decisions();
        let identity = |vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex };
        let point = |x, vertex| ConstructedIntersectionPoint {
            point: Point3::new(Real::from(x), Real::zero(), Real::zero()),
            identity: Some(identity(vertex)),
        };
        let mut points = Vec::with_capacity(8);
        points.extend([point(0, 9), point(1, 4), point(0, 2)]);
        let capacity = points.capacity();

        dedup_constructed_points(&decisions, &mut points).unwrap();

        assert_eq!(points.len(), 2);
        assert_eq!(points.capacity(), capacity);
        assert_eq!(points[0].identity, Some(identity(2)));
        assert_eq!(points[1].identity, Some(identity(4)));
        assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
    }

    #[test]
    fn compact_graph_preserves_stream_order_without_per_face_vectors() {
        let mut graph = PairwiseIntersectionGraphBuilder::new(4).unwrap();
        graph.append_coplanar_overlap(2, 0).unwrap();
        graph.append_coplanar_overlap(0, 2).unwrap();
        graph.append_coplanar_overlap(2, 1).unwrap();
        let graph = graph.finish().unwrap();

        assert_eq!(graph.len(), 4);
        assert_eq!(graph.events.len(), 3);
        assert_eq!(&*graph.offsets, &[0, 1, 1, 3, 3]);
        assert_eq!(graph.event_ids(1).unwrap().len(), 0);
        assert_eq!(graph.event_ids(3).unwrap().len(), 0);
        assert_eq!(
            graph
                .event_ids(2)
                .unwrap()
                .map(|event| match event.unwrap() {
                    PairwiseIntersectionEventIds::CoplanarOverlap { other_polygon } => {
                        other_polygon
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
        assert_eq!(graph.events.len(), 2);
        assert!(matches!(
            graph.event_ids(0).unwrap().next(),
            Some(Ok(PairwiseIntersectionEventIds::NonCoplanarSegment {
                other_polygon: 1,
                ..
            }))
        ));
        assert!(matches!(
            graph.event_ids(1).unwrap().next(),
            Some(Ok(PairwiseIntersectionEventIds::NonCoplanarSegment {
                other_polygon: 0,
                ..
            }))
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

        assert_eq!(graph.events.len(), 10);
        assert_eq!(size_of::<PairwiseIntersectionEvent>(), 8);
        assert_eq!(size_of::<super::PendingIntersectionEvent>(), 12);
        assert!(matches!(
            graph.event_ids(0).unwrap().next(),
            Some(Ok(PairwiseIntersectionEventIds::NonCoplanarPoint {
                point,
                other_polygon: 1,
            })) if graph.points[point as usize] == Point3::origin()
        ));
        assert!(matches!(
            graph.event_ids(1).unwrap().nth(1),
            Some(Ok(PairwiseIntersectionEventIds::NonCoplanarSegment {
                other_polygon: 2,
                ..
            }))
        ));
        assert!(matches!(
            graph.event_ids(2).unwrap().nth(1),
            Some(Ok(PairwiseIntersectionEventIds::CoplanarPoint {
                point,
                other_polygon: 3,
            })) if graph.points[point as usize] == p2(3, 0)
        ));
        assert!(matches!(
            graph.event_ids(3).unwrap().nth(1),
            Some(Ok(PairwiseIntersectionEventIds::CoplanarSegment {
                endpoints,
                other_polygon: 4,
            })) if graph.points[endpoints[0] as usize] == p2(4, 0)
                && graph.points[endpoints[1] as usize] == p2(5, 0)
        ));
        assert!(matches!(
            graph.event_ids(5).unwrap().next(),
            Some(Ok(PairwiseIntersectionEventIds::CoplanarOverlap {
                other_polygon: 4,
            }))
        ));
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
        assert_eq!(graph.events.len(), 0);
        assert!(graph.points.is_empty());
        assert!(graph.segments.is_empty());
        assert_eq!(
            graph.radially_separated_face_pair_keys.as_ref(),
            [source_face_pair_key(0, 1).unwrap()]
        );

        let mut distinct_vertex =
            crate::test_support::approximate_convex_triangle(&p(2, 0), &p(3, -1), &p(3, 0), 0, 3);
        distinct_vertex
            .set_source_triangle_edge_identities(0, [10, 11, 12])
            .unwrap();
        let graph =
            super::pairwise_intersections_by_polygon(&decisions, &[host, distinct_vertex]).unwrap();
        assert_eq!(graph.events.len(), 2);
        assert!(matches!(
            graph.event_ids(0).unwrap().next(),
            Some(Ok(PairwiseIntersectionEventIds::CoplanarPoint {
                other_polygon: 1,
                ..
            }))
        ));
        assert!(graph.radially_separated_face_pair_keys.is_empty());
    }

    #[test]
    fn conservative_binary32_bvh_candidate_is_rejected_by_exact_narrow_phase() {
        let base = 1_i64 << 30;
        let p = |x, y| Point3::new(Real::from(x), Real::from(y), Real::zero());
        let polygons = [
            crate::test_support::approximate_convex_triangle(
                &p(base, 0),
                &p(base + 1, 0),
                &p(base, 1),
                0,
                0,
            ),
            crate::test_support::approximate_convex_triangle(
                &p(base + 2, 0),
                &p(base + 3, 0),
                &p(base + 2, 1),
                1,
                0,
            ),
        ];

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            let graph = super::pairwise_intersections_by_polygon(&decisions, &polygons).unwrap();
            assert!(graph.events.is_empty());
            assert!(graph.points.is_empty());
            assert!(graph.segments.is_empty());
            assert!(graph.radially_separated_face_pair_keys.is_empty());
            assert_eq!(
                decisions.certainty(),
                crate::context::MeshCertainty::Certified
            );
        }
    }

    #[test]
    fn manifold_edge_skip_requires_shared_construction_identity() {
        let decisions = crate::test_support::approximate_decisions();
        let p = |x, y| Point3::new(Real::from(x), Real::from(y), Real::zero());
        let mut host =
            crate::test_support::approximate_convex_triangle(&p(0, 0), &p(2, 0), &p(0, 2), 0, 0);
        host.set_source_triangle_edge_identities(0, [0, 1, 2])
            .unwrap();
        let mut authored_neighbor =
            crate::test_support::approximate_convex_triangle(&p(2, 0), &p(0, 0), &p(1, -2), 0, 1);
        authored_neighbor
            .set_source_triangle_edge_identities(0, [1, 0, 3])
            .unwrap();
        let mut coincident_component_edge = authored_neighbor.clone();
        coincident_component_edge
            .set_source_triangle_edge_identities(0, [10, 11, 12])
            .unwrap();
        let host_vertices = host.vertices_decision(&decisions).unwrap();
        let neighbor_vertices = authored_neighbor.vertices_decision(&decisions).unwrap();
        let component_vertices = coincident_component_edge
            .vertices_decision(&decisions)
            .unwrap();

        assert!(
            polygon_cycles_share_reversed_manifold_triangle_edge(
                &decisions,
                &host_vertices,
                &host,
                &neighbor_vertices,
                &authored_neighbor,
            )
            .unwrap()
        );
        assert!(
            !polygon_cycles_share_reversed_manifold_triangle_edge(
                &decisions,
                &host_vertices,
                &host,
                &component_vertices,
                &coincident_component_edge,
            )
            .unwrap()
        );
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
        assert_eq!(graph.events.len(), 2);
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
    fn invalid_face_append_fails_without_mutating_the_arena() {
        #[cfg(target_pointer_width = "64")]
        assert!(PairwiseIntersectionGraphBuilder::new(usize::MAX).is_err());
        let mut graph = PairwiseIntersectionGraphBuilder::new(0).unwrap();
        assert!(graph.append_coplanar_overlap(0, 0).is_err());
        assert_eq!(graph.finish().unwrap().events.len(), 0);
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
        assert_eq!(graph.events.len(), 0);
        assert!(graph.points.is_empty());
        assert!(graph.segments.is_empty());
    }
}
