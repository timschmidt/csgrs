//! Convex polygon representation backed by hyperreal planes.

use hyperlattice::{HomogeneousPoint3, Point3, Rational, Real, intersect_three_planes};
use hyperreal::RationalLinearForm4Query;
use std::sync::{Arc, OnceLock};

use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{
    Classification, Plane, affine_projective_point_decision, axis_ref, cross_arrays, sub_points,
};
use crate::predicate::{
    Point3PredicateQuery, classify_projective_point_decision, compare_real_decision,
};
use crate::winding::WindingNumberTransitionVector;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub(crate) struct ConstructionPlaneIdentity {
    pub(crate) mesh: u32,
    pub(crate) plane: u32,
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub(crate) enum ConstructionEdgeIdentity {
    Source {
        mesh: u32,
        endpoints: [u32; 2],
    },
    Split {
        planes: [ConstructionPlaneIdentity; 2],
    },
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub(crate) enum ConstructionVertexIdentity {
    Source {
        mesh: u32,
        vertex: u32,
    },
    SourceEdgePlane {
        mesh: u32,
        endpoints: [u32; 2],
        plane: ConstructionPlaneIdentity,
    },
    PlaneTriple {
        planes: [ConstructionPlaneIdentity; 3],
    },
}

impl ConstructionEdgeIdentity {
    pub(crate) fn intersection_identity(
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

fn compact_construction_index(value: usize, operation: &'static str) -> HypermeshResult<u32> {
    u32::try_from(value).map_err(|_| crate::error::HypermeshError::CapacityOverflow { operation })
}

/// Approximate exact-coordinate bounds for fast spatial rejection.
#[derive(Clone, Debug, PartialEq)]
pub struct ApproxBounds {
    /// Minimum coordinate by axis.
    pub min: Point3,
    /// Maximum coordinate by axis.
    pub max: Point3,
}

/// Borrowed exact extrema whose coordinates may share another geometry owner.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ApproxBoundsRef<'a> {
    pub(crate) min: [&'a Real; 3],
    pub(crate) max: [&'a Real; 3],
}

impl ApproxBoundsRef<'_> {
    pub(crate) fn to_owned(self) -> ApproxBounds {
        ApproxBounds::new(
            Point3::new(
                self.min[0].clone(),
                self.min[1].clone(),
                self.min[2].clone(),
            ),
            Point3::new(
                self.max[0].clone(),
                self.max[1].clone(),
                self.max[2].clone(),
            ),
        )
    }
}

const NO_RETAINED_SOURCE_PREDICATE_QUERY: u32 = u32::MAX;

#[derive(Debug)]
enum RetainedSourcePredicateQueries {
    /// Every source position is referenced, so its source index is the query
    /// index and no indirection table is needed.
    Dense(Box<[RationalLinearForm4Query]>),
    /// Unused source positions retain no query. The compact index preserves
    /// direct source-ID lookup without storing a 32-byte query for each one.
    Indexed {
        indices: Box<[u32]>,
        queries: Box<[RationalLinearForm4Query]>,
    },
}

#[derive(Debug)]
/// One exact position owner shared by every retained face of a source mesh.
///
/// The sized wrapper keeps the `Arc` stored in each indexed face thin while
/// the inner slice remains the canonical native or copied borrowed owner.
pub(crate) struct RetainedSourcePositions {
    positions: Arc<[Point3]>,
    predicate_queries: Option<RetainedSourcePredicateQueries>,
}

impl RetainedSourcePositions {
    #[cfg(test)]
    pub(crate) fn shared(positions: Arc<[Point3]>) -> Arc<Self> {
        Arc::new(Self {
            positions,
            predicate_queries: None,
        })
    }

    /// Builds one compact certified query per referenced source position.
    ///
    /// This schedule is representation-driven: if any referenced point cannot
    /// supply Hyperreal's certified rational filter query, it retains no
    /// partial schedule and every predicate uses the ordinary exact cascade.
    pub(crate) fn shared_with_predicate_queries(
        positions: Arc<[Point3]>,
        referenced: &[bool],
    ) -> HypermeshResult<Arc<Self>> {
        if positions.len() != referenced.len() {
            return Err(HypermeshError::SurfaceArrangementFailed {
                reason: "source position and usage schedules differ",
            });
        }
        let referenced_count = referenced.iter().filter(|&&used| used).count();
        let mut queries = Vec::new();
        queries.try_reserve_exact(referenced_count).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "retained source point predicate queries",
            }
        })?;
        for (point, &used) in positions.iter().zip(referenced) {
            if !used {
                continue;
            }
            let Some(query) = Point3PredicateQuery::new(point).rational_filter_query() else {
                return Ok(Arc::new(Self {
                    positions,
                    predicate_queries: None,
                }));
            };
            queries.push(query);
        }
        let predicate_queries = if queries.is_empty() {
            None
        } else if queries.len() == positions.len() {
            Some(RetainedSourcePredicateQueries::Dense(
                queries.into_boxed_slice(),
            ))
        } else {
            let mut indices = Vec::new();
            indices.try_reserve_exact(positions.len()).map_err(|_| {
                HypermeshError::CapacityOverflow {
                    operation: "retained source point predicate query indices",
                }
            })?;
            indices.resize(positions.len(), NO_RETAINED_SOURCE_PREDICATE_QUERY);
            let mut query = 0_usize;
            for (position, &used) in referenced.iter().enumerate() {
                if !used {
                    continue;
                }
                indices[position] =
                    u32::try_from(query).map_err(|_| HypermeshError::CapacityOverflow {
                        operation: "retained source point predicate query index",
                    })?;
                query += 1;
            }
            debug_assert_eq!(query, queries.len());
            Some(RetainedSourcePredicateQueries::Indexed {
                indices: indices.into_boxed_slice(),
                queries: queries.into_boxed_slice(),
            })
        };
        Ok(Arc::new(Self {
            positions,
            predicate_queries,
        }))
    }

    pub(crate) fn get(&self, index: usize) -> Option<&Point3> {
        self.positions.get(index)
    }

    pub(crate) fn len(&self) -> usize {
        self.positions.len()
    }

    fn predicate_query(&self, index: usize) -> Option<&RationalLinearForm4Query> {
        match self.predicate_queries.as_ref()? {
            RetainedSourcePredicateQueries::Dense(queries) => queries.get(index),
            RetainedSourcePredicateQueries::Indexed { indices, queries } => {
                let query = *indices.get(index)?;
                (query != NO_RETAINED_SOURCE_PREDICATE_QUERY)
                    .then(|| queries.get(query as usize))
                    .flatten()
            }
        }
    }

    #[cfg(test)]
    pub(crate) fn owner(&self) -> &Arc<[Point3]> {
        &self.positions
    }
}

/// Lossless primitive encodings of one source triangle's support and edges.
///
/// These are exact representations, not predicate results: admission requires
/// every coefficient to survive a lossless binary round trip.
#[derive(Clone, Copy, Debug)]
pub(crate) struct CompactSourcePolygon<T> {
    pub(crate) planes: [[T; 4]; 4],
}

impl CompactSourcePolygon<f64> {
    pub(crate) fn from_planes(planes: [&Plane; 4]) -> Option<Self> {
        let [support, first, second, third] = planes.map(exact_dyadic_plane_coefficients);
        Some(Self {
            planes: [support?, first?, second?, third?],
        })
    }
}

impl CompactSourcePolygon<f32> {
    fn from_binary64(wide: CompactSourcePolygon<f64>) -> Option<Self> {
        let [support, first, second, third] = wide.planes.map(|plane| {
            let [a, b, c, d] = plane.map(exact_binary32);
            Some([a?, b?, c?, d?])
        });
        Some(Self {
            planes: [support?, first?, second?, third?],
        })
    }
}

#[derive(Debug)]
pub(crate) enum CompactSourcePolygons {
    Binary32(Box<[CompactSourcePolygon<f32>]>),
    Binary64(Box<[CompactSourcePolygon<f64>]>),
}

impl CompactSourcePolygons {
    pub(crate) fn from_binary64_rows(rows: Vec<CompactSourcePolygon<f64>>) -> Self {
        let binary32 = rows
            .iter()
            .copied()
            .map(CompactSourcePolygon::<f32>::from_binary64)
            .collect::<Option<Vec<_>>>();
        match binary32 {
            Some(rows) => Self::Binary32(rows.into_boxed_slice()),
            None => Self::Binary64(rows.into_boxed_slice()),
        }
    }

    pub(crate) fn len(&self) -> usize {
        match self {
            Self::Binary32(polygons) => polygons.len(),
            Self::Binary64(polygons) => polygons.len(),
        }
    }

    fn plane(&self, face: usize, plane: usize) -> Plane {
        let coefficients = match self {
            Self::Binary32(polygons) => polygons[face].planes[plane].map(f64::from),
            Self::Binary64(polygons) => polygons[face].planes[plane],
        };
        plane_from_exact_dyadic_coefficients(coefficients)
    }
}

fn exact_binary32(value: f64) -> Option<f32> {
    let narrowed = value as f32;
    (f64::from(narrowed) == value).then_some(narrowed)
}

pub(crate) fn exact_dyadic_plane_coefficients(plane: &Plane) -> Option<[f64; 4]> {
    let [a, b, c, d] = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ]
    .map(Real::to_f64_exact_dyadic);
    Some([a?, b?, c?, d?])
}

fn plane_from_exact_dyadic_coefficients([a, b, c, d]: [f64; 4]) -> Plane {
    Plane::from_coefficients(
        Real::try_from(a).expect("retained exact-dyadic coefficient is finite"),
        Real::try_from(b).expect("retained exact-dyadic coefficient is finite"),
        Real::try_from(c).expect("retained exact-dyadic coefficient is finite"),
        Real::try_from(d).expect("retained exact-dyadic coefficient is finite"),
    )
}

#[derive(Debug)]
struct SourceTrianglePlaneCache {
    support: OnceLock<Box<Plane>>,
    edges: OnceLock<Box<[Plane; 3]>>,
}

#[derive(Debug)]
struct SourceTrianglePlanes {
    support: Plane,
    edges: [Plane; 3],
}

#[derive(Debug)]
enum SourcePlaneStorage {
    /// Lossless primitive rows are cheap to reconstruct and serve dense exact
    /// consumers without an atomic lazy-cache probe on every plane access.
    Eager(Box<[SourceTrianglePlanes]>),
    /// General `Real` planes retain source expressions and are materialized
    /// only for faces reached by the conservative candidate schedule.
    Lazy(Box<[SourceTrianglePlaneCache]>),
}

/// One operation-local plane owner shared by every face of a source mesh.
///
/// Source positions remain the canonical geometry. General `Real` support and
/// edge planes are constructed only when a consuming arrangement path demands
/// them. Native lossless primitive rows eagerly reconstruct the same exact
/// planes without repeating source arithmetic or adding lazy-cache probes to
/// their dense fast path.
#[derive(Debug)]
pub(crate) struct RetainedSourcePlanes {
    storage: SourcePlaneStorage,
    normalize_wide_dyadic: bool,
}

impl RetainedSourcePlanes {
    pub(crate) fn new(
        face_count: usize,
        compact: Option<Arc<CompactSourcePolygons>>,
        normalize_wide_dyadic: bool,
    ) -> HypermeshResult<Arc<Self>> {
        if compact
            .as_deref()
            .is_some_and(|compact| compact.len() != face_count)
        {
            return Err(crate::error::HypermeshError::SurfaceArrangementFailed {
                reason: "retained source-plane and triangle counts differ",
            });
        }
        let storage = compact.map_or_else(
            || {
                SourcePlaneStorage::Lazy(
                    (0..face_count)
                        .map(|_| SourceTrianglePlaneCache {
                            support: OnceLock::new(),
                            edges: OnceLock::new(),
                        })
                        .collect(),
                )
            },
            |compact| {
                SourcePlaneStorage::Eager(
                    (0..face_count)
                        .map(|face| SourceTrianglePlanes {
                            support: compact.plane(face, 0),
                            edges: [
                                compact.plane(face, 1),
                                compact.plane(face, 2),
                                compact.plane(face, 3),
                            ],
                        })
                        .collect(),
                )
            },
        );
        Ok(Arc::new(Self {
            storage,
            normalize_wide_dyadic,
        }))
    }

    fn len(&self) -> usize {
        match &self.storage {
            SourcePlaneStorage::Eager(planes) => planes.len(),
            SourcePlaneStorage::Lazy(cache) => cache.len(),
        }
    }

    fn has_lossless_primitive_storage(&self) -> bool {
        matches!(&self.storage, SourcePlaneStorage::Eager(_))
    }

    #[inline]
    fn support<'point>(&self, face: usize, points: impl FnOnce() -> [&'point Point3; 3]) -> &Plane {
        match &self.storage {
            SourcePlaneStorage::Eager(planes) => &planes[face].support,
            SourcePlaneStorage::Lazy(cache) => {
                let support = &cache[face].support;
                if let Some(support) = support.get() {
                    return support;
                }
                let [first, second, third] = points();
                support
                    .get_or_init(|| Box::new(Plane::from_points(first, second, third)))
                    .as_ref()
            }
        }
    }

    #[inline]
    fn edges<'point>(&self, face: usize, points: impl FnOnce() -> [&'point Point3; 3]) -> &[Plane] {
        match &self.storage {
            SourcePlaneStorage::Eager(planes) => &planes[face].edges,
            SourcePlaneStorage::Lazy(cache) => {
                let edges = &cache[face].edges;
                if let Some(edges) = edges.get() {
                    return edges.as_slice();
                }
                let points @ [first, second, third] = points();
                edges
                    .get_or_init(|| {
                        Box::new(source_triangle_edge_planes(
                            [first, second, third],
                            self.support(face, || points),
                            self.normalize_wide_dyadic,
                        ))
                    })
                    .as_slice()
            }
        }
    }

    #[cfg(test)]
    fn materialization_counts(&self) -> (bool, usize, usize) {
        match &self.storage {
            SourcePlaneStorage::Eager(planes) => (true, planes.len(), planes.len()),
            SourcePlaneStorage::Lazy(cache) => (
                false,
                cache
                    .iter()
                    .filter(|face| face.support.get().is_some())
                    .count(),
                cache
                    .iter()
                    .filter(|face| face.edges.get().is_some())
                    .count(),
            ),
        }
    }

    #[cfg(debug_assertions)]
    fn face_is_materialized(&self, face: usize) -> bool {
        match &self.storage {
            SourcePlaneStorage::Eager(planes) => face < planes.len(),
            SourcePlaneStorage::Lazy(cache) => cache
                .get(face)
                .is_some_and(|face| face.support.get().is_some() && face.edges.get().is_some()),
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) enum RetainedVertexCycle {
    /// Standalone polygon vertices owned by this cycle.
    Owned(Arc<[Point3]>),
    /// One indexed triangle in a shared source-mesh position owner.
    SourceTriangle {
        positions: Arc<RetainedSourcePositions>,
        vertices: [u32; 3],
        extrema: SourceTriangleExtrema,
    },
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct SourceTriangleExtrema(u16);

impl SourceTriangleExtrema {
    fn new(vertices: [u8; 6]) -> Self {
        let mut packed = 0_u16;
        for (slot, vertex) in vertices.into_iter().enumerate() {
            debug_assert!(vertex < 3);
            packed |= u16::from(vertex) << (slot * 2);
        }
        Self(packed)
    }

    fn get(self, slot: usize) -> usize {
        usize::from((self.0 >> (slot * 2)) & 3)
    }

    fn reversed(self) -> Self {
        Self::new(std::array::from_fn(|slot| 2 - self.get(slot) as u8))
    }
}

impl RetainedVertexCycle {
    pub(crate) fn len(&self) -> usize {
        match self {
            Self::Owned(vertices) => vertices.len(),
            Self::SourceTriangle { .. } => 3,
        }
    }

    pub(crate) fn get(&self, index: usize) -> Option<&Point3> {
        match self {
            Self::Owned(vertices) => vertices.get(index),
            Self::SourceTriangle {
                positions,
                vertices,
                ..
            } => positions.get(usize::try_from(*vertices.get(index)?).ok()?),
        }
    }

    pub(crate) fn iter(&self) -> impl DoubleEndedIterator<Item = &Point3> + ExactSizeIterator {
        (0..self.len()).map(|index| {
            self.get(index)
                .expect("retained vertex indices are validated at input preparation")
        })
    }

    fn to_vec(&self) -> Vec<Point3> {
        self.iter().cloned().collect()
    }

    fn reversed(&self) -> Self {
        match self {
            Self::Owned(vertices) => Self::Owned(Arc::from(
                vertices.iter().rev().cloned().collect::<Vec<_>>(),
            )),
            Self::SourceTriangle {
                positions,
                vertices: [first, second, third],
                extrema,
            } => Self::SourceTriangle {
                positions: Arc::clone(positions),
                vertices: [*third, *second, *first],
                extrema: extrema.reversed(),
            },
        }
    }

    fn source_bounds(&self) -> Option<ApproxBoundsRef<'_>> {
        let Self::SourceTriangle { .. } = self else {
            return None;
        };
        Some(ApproxBoundsRef {
            min: std::array::from_fn(|axis| {
                self.source_bound(axis, false)
                    .expect("source triangles retain every minimum extremum")
            }),
            max: std::array::from_fn(|axis| {
                self.source_bound(axis, true)
                    .expect("source triangles retain every maximum extremum")
            }),
        })
    }

    fn source_bound(&self, axis: usize, maximum: bool) -> Option<&Real> {
        let Self::SourceTriangle {
            positions,
            vertices,
            extrema,
        } = self
        else {
            return None;
        };
        let slot = axis + usize::from(maximum) * 3;
        let triangle_vertex = extrema.get(slot);
        let source_vertex = *vertices.get(triangle_vertex)? as usize;
        positions
            .get(source_vertex)
            .map(|point| axis_ref(point, axis))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum RetainedIdentityCycles {
    SourceTriangle {
        mesh: u32,
        vertices: [u32; 3],
    },
    Owned {
        vertices: Arc<[ConstructionVertexIdentity]>,
        edges: Arc<[ConstructionEdgeIdentity]>,
    },
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct KnownVertexIdentityCycle<'a>(&'a RetainedIdentityCycles);

impl<'a> KnownVertexIdentityCycle<'a> {
    pub(crate) fn len(self) -> usize {
        match self.0 {
            RetainedIdentityCycles::SourceTriangle { .. } => 3,
            RetainedIdentityCycles::Owned { vertices, .. } => vertices.len(),
        }
    }

    pub(crate) fn get(self, index: usize) -> Option<ConstructionVertexIdentity> {
        match self.0 {
            RetainedIdentityCycles::SourceTriangle { mesh, vertices } => {
                Some(ConstructionVertexIdentity::Source {
                    mesh: *mesh,
                    vertex: *vertices.get(index)?,
                })
            }
            RetainedIdentityCycles::Owned { vertices, .. } => vertices.get(index).cloned(),
        }
    }

    pub(crate) fn iter(self) -> KnownVertexIdentityIter<'a> {
        KnownVertexIdentityIter {
            cycle: self,
            indices: 0..self.len(),
        }
    }
}

pub(crate) struct KnownVertexIdentityIter<'a> {
    cycle: KnownVertexIdentityCycle<'a>,
    indices: std::ops::Range<usize>,
}

impl Iterator for KnownVertexIdentityIter<'_> {
    type Item = ConstructionVertexIdentity;

    fn next(&mut self) -> Option<Self::Item> {
        self.cycle.get(self.indices.next()?)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.indices.size_hint()
    }
}

impl DoubleEndedIterator for KnownVertexIdentityIter<'_> {
    fn next_back(&mut self) -> Option<Self::Item> {
        self.cycle.get(self.indices.next_back()?)
    }
}

impl ExactSizeIterator for KnownVertexIdentityIter<'_> {}

impl<'a> IntoIterator for KnownVertexIdentityCycle<'a> {
    type Item = ConstructionVertexIdentity;
    type IntoIter = KnownVertexIdentityIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct KnownEdgeIdentityCycle<'a>(&'a RetainedIdentityCycles);

impl<'a> KnownEdgeIdentityCycle<'a> {
    pub(crate) fn len(self) -> usize {
        match self.0 {
            RetainedIdentityCycles::SourceTriangle { .. } => 3,
            RetainedIdentityCycles::Owned { edges, .. } => edges.len(),
        }
    }

    pub(crate) fn get(self, index: usize) -> Option<ConstructionEdgeIdentity> {
        match self.0 {
            RetainedIdentityCycles::SourceTriangle { mesh, vertices } => {
                let mut endpoints = [
                    *vertices.get(index)?,
                    vertices[(index + 1) % vertices.len()],
                ];
                endpoints.sort_unstable();
                Some(ConstructionEdgeIdentity::Source {
                    mesh: *mesh,
                    endpoints,
                })
            }
            RetainedIdentityCycles::Owned { edges, .. } => edges.get(index).cloned(),
        }
    }

    pub(crate) fn iter(self) -> KnownEdgeIdentityIter<'a> {
        KnownEdgeIdentityIter {
            cycle: self,
            indices: 0..self.len(),
        }
    }
}

pub(crate) struct KnownEdgeIdentityIter<'a> {
    cycle: KnownEdgeIdentityCycle<'a>,
    indices: std::ops::Range<usize>,
}

impl Iterator for KnownEdgeIdentityIter<'_> {
    type Item = ConstructionEdgeIdentity;

    fn next(&mut self) -> Option<Self::Item> {
        self.cycle.get(self.indices.next()?)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.indices.size_hint()
    }
}

impl DoubleEndedIterator for KnownEdgeIdentityIter<'_> {
    fn next_back(&mut self) -> Option<Self::Item> {
        self.cycle.get(self.indices.next_back()?)
    }
}

impl ExactSizeIterator for KnownEdgeIdentityIter<'_> {}

impl<'a> IntoIterator for KnownEdgeIdentityCycle<'a> {
    type Item = ConstructionEdgeIdentity;
    type IntoIter = KnownEdgeIdentityIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl PartialEq for KnownEdgeIdentityCycle<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.len() == other.len() && self.iter().eq(other.iter())
    }
}

impl RetainedIdentityCycles {
    fn vertices(&self) -> KnownVertexIdentityCycle<'_> {
        KnownVertexIdentityCycle(self)
    }

    fn edges(&self) -> KnownEdgeIdentityCycle<'_> {
        KnownEdgeIdentityCycle(self)
    }
}

impl ApproxBounds {
    /// Constructs bounds from min/max points.
    pub const fn new(min: Point3, max: Point3) -> Self {
        Self { min, max }
    }

    pub(crate) fn borrowed(&self) -> ApproxBoundsRef<'_> {
        ApproxBoundsRef {
            min: [&self.min.x, &self.min.y, &self.min.z],
            max: [&self.max.x, &self.max.y, &self.max.z],
        }
    }

    /// Computes bounds for a non-empty borrowed point slice.
    pub fn for_points(
        context: &MeshContext,
        points: &[&Point3],
    ) -> HypermeshResult<MeshOutcome<Self>> {
        let decisions = DecisionContext::new(context);
        let bounds = Self::for_points_decision(&decisions, points)?;
        Ok(decisions.finish(bounds))
    }

    pub(crate) fn for_points_decision(
        decisions: &DecisionContext,
        points: &[&Point3],
    ) -> HypermeshResult<Self> {
        bounds_for_points(decisions, points)
    }
}

#[derive(Clone, Debug)]
struct OwnedPolygonPlanes {
    support: Plane,
    edges: Vec<Plane>,
}

#[derive(Clone, Debug)]
enum PolygonPlanes {
    Owned(Arc<OwnedPolygonPlanes>),
    SourceTriangle {
        owner: Arc<RetainedSourcePlanes>,
        face: u32,
    },
}

impl PolygonPlanes {
    fn owned(support: Plane, edges: Vec<Plane>) -> Self {
        Self::Owned(Arc::new(OwnedPolygonPlanes { support, edges }))
    }
}

/// Plane-bounded convex polygon.
#[derive(Clone, Debug)]
pub struct ConvexPolygon {
    planes: PolygonPlanes,
    /// Source mesh index.
    pub mesh_index: isize,
    /// Source polygon index.
    pub polygon_index: isize,
    /// Winding transition vector.
    pub delta_w: WindingNumberTransitionVector,
    /// Optional approximate bounds.
    pub approx_bounds: Option<Box<ApproxBounds>>,
    /// Exact vertices retained when supplied directly by the input owner.
    ///
    /// Arrangement-derived polygons clear this cache when their edge cycle
    /// changes.
    pub(crate) known_vertices: Option<RetainedVertexCycle>,
    pub(crate) known_identities: Option<RetainedIdentityCycles>,
}

impl PartialEq for ConvexPolygon {
    fn eq(&self, other: &Self) -> bool {
        self.support_plane() == other.support_plane()
            && self.edge_planes() == other.edge_planes()
            && self.mesh_index == other.mesh_index
            && self.polygon_index == other.polygon_index
            && self.delta_w == other.delta_w
            && self.retained_bounds() == other.retained_bounds()
    }
}

impl ConvexPolygon {
    /// Returns the exact supporting plane, constructing a retained source
    /// plane on first demand when necessary.
    #[inline]
    pub fn support_plane(&self) -> &Plane {
        match &self.planes {
            PolygonPlanes::Owned(planes) => &planes.support,
            PolygonPlanes::SourceTriangle { owner, face } => owner.support(*face as usize, || {
                self.retained_source_triangle_points()
                    .expect("an unmaterialized source plane retains its checked source vertices")
            }),
        }
    }

    pub(crate) fn support_plane_has_lossless_primitive_storage(&self) -> bool {
        match &self.planes {
            PolygonPlanes::SourceTriangle { owner, .. } => owner.has_lossless_primitive_storage(),
            PolygonPlanes::Owned(_) => false,
        }
    }

    /// Returns the exact interior-facing edge planes, constructing a retained
    /// source edge cycle on first demand when necessary.
    #[inline]
    pub fn edge_planes(&self) -> &[Plane] {
        match &self.planes {
            PolygonPlanes::Owned(planes) => planes.edges.as_slice(),
            PolygonPlanes::SourceTriangle { owner, face } => owner.edges(*face as usize, || {
                self.retained_source_triangle_points().expect(
                    "an unmaterialized source edge cycle retains its checked source vertices",
                )
            }),
        }
    }

    fn retained_source_triangle_points(&self) -> Option<[&Point3; 3]> {
        let vertices = self.known_vertices.as_ref()?;
        Some([vertices.get(0)?, vertices.get(1)?, vertices.get(2)?])
    }

    #[cfg(test)]
    pub(crate) fn source_plane_materialization_counts(&self) -> Option<(bool, usize, usize)> {
        match &self.planes {
            PolygonPlanes::SourceTriangle { owner, .. } => Some(owner.materialization_counts()),
            PolygonPlanes::Owned(_) => None,
        }
    }

    fn replace_planes(&mut self, support: Plane, edges: Vec<Plane>) {
        self.planes = PolygonPlanes::owned(support, edges);
    }

    pub(crate) fn replace_edge_planes(&mut self, edges: Vec<Plane>) {
        self.replace_planes(self.support_plane().clone(), edges);
    }

    pub(crate) fn clear_edge_planes(&mut self) {
        self.replace_edge_planes(Vec::new());
    }

    /// Releases an exact retained vertex cycle after its plane carrier has
    /// been made self-sufficient by `edge_planes` or `replace_edge_planes`.
    pub(crate) fn clear_known_vertices(&mut self) {
        #[cfg(debug_assertions)]
        if let PolygonPlanes::SourceTriangle { owner, face } = &self.planes {
            debug_assert!(owner.face_is_materialized(*face as usize));
        }
        self.known_vertices = None;
    }

    pub(crate) fn retained_bounds(&self) -> Option<ApproxBoundsRef<'_>> {
        self.approx_bounds
            .as_deref()
            .map(ApproxBounds::borrowed)
            .or_else(|| {
                self.known_vertices
                    .as_ref()
                    .and_then(RetainedVertexCycle::source_bounds)
            })
    }

    pub(crate) fn retained_bound(&self, axis: usize, maximum: bool) -> Option<&Real> {
        self.approx_bounds
            .as_deref()
            .map(|bounds| axis_ref(if maximum { &bounds.max } else { &bounds.min }, axis))
            .or_else(|| {
                self.known_vertices
                    .as_ref()
                    .and_then(|vertices| vertices.source_bound(axis, maximum))
            })
    }

    /// Constructs an empty polygon carrier.
    pub fn empty() -> Self {
        Self {
            planes: PolygonPlanes::owned(
                Plane::from_coefficients(Real::zero(), Real::zero(), Real::zero(), Real::zero()),
                Vec::new(),
            ),
            mesh_index: -1,
            polygon_index: -1,
            delta_w: Vec::new(),
            approx_bounds: None,
            known_vertices: None,
            known_identities: None,
        }
    }

    /// Returns the number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.known_vertices
            .as_ref()
            .map_or_else(|| self.edge_planes().len(), RetainedVertexCycle::len)
    }

    pub(crate) fn has_retained_vertex(&self, point: &Point3) -> bool {
        self.known_vertices
            .as_ref()
            .is_some_and(|vertices| vertices.iter().any(|vertex| vertex == point))
    }

    pub(crate) fn known_vertex_identities(&self) -> Option<KnownVertexIdentityCycle<'_>> {
        self.known_identities
            .as_ref()
            .map(RetainedIdentityCycles::vertices)
    }

    pub(crate) fn known_source_triangle_identity(&self) -> Option<(u32, [u32; 3])> {
        match self.known_identities.as_ref()? {
            RetainedIdentityCycles::SourceTriangle { mesh, vertices } => Some((*mesh, *vertices)),
            RetainedIdentityCycles::Owned { .. } => None,
        }
    }

    /// Returns the certified rational filter query retained by an authored
    /// source vertex. Owned/constructed cycles keep using local predicate
    /// scheduling because they have no shared dense source-ID domain.
    pub(crate) fn known_vertex_predicate_query(
        &self,
        index: usize,
    ) -> Option<&RationalLinearForm4Query> {
        let RetainedVertexCycle::SourceTriangle {
            positions,
            vertices,
            ..
        } = self.known_vertices.as_ref()?
        else {
            return None;
        };
        let source = *vertices.get(index)? as usize;
        positions.predicate_query(source)
    }

    pub(crate) fn known_edge_identities(&self) -> Option<KnownEdgeIdentityCycle<'_>> {
        self.known_identities
            .as_ref()
            .map(RetainedIdentityCycles::edges)
    }

    /// Returns true when this polygon has at least three vertices and a
    /// non-zero support normal.
    pub fn is_valid(&self, context: &MeshContext) -> HypermeshResult<MeshOutcome<bool>> {
        let decisions = DecisionContext::new(context);
        let valid = self.vertex_count() >= 3 && self.support_plane().decide_is_valid(&decisions)?;
        Ok(decisions.finish(valid))
    }

    /// Computes vertex `i` as a homogeneous intersection of support and two
    /// adjacent edge planes.
    pub fn vertex(&self, i: usize) -> HomogeneousPoint3 {
        let n = self.vertex_count();
        let support = self.support_plane();
        let edges = self.edge_planes();
        intersect_three_planes(support, &edges[i], &edges[(i + 1) % n])
    }

    /// Computes an affine vertex.
    pub fn vertex_point(
        &self,
        context: &MeshContext,
        i: usize,
    ) -> HypermeshResult<MeshOutcome<Point3>> {
        let decisions = DecisionContext::new(context);
        let point = self.vertex_point_decision(&decisions, i)?;
        Ok(decisions.finish(point))
    }

    pub(crate) fn vertex_point_decision(
        &self,
        decisions: &DecisionContext,
        i: usize,
    ) -> HypermeshResult<Point3> {
        affine_projective_point_decision(decisions, &self.vertex(i))
    }

    /// Computes all affine vertices.
    pub fn vertices(&self, context: &MeshContext) -> HypermeshResult<MeshOutcome<Vec<Point3>>> {
        let decisions = DecisionContext::new(context);
        let vertices = self.vertices_decision(&decisions)?;
        Ok(decisions.finish(vertices))
    }

    pub(crate) fn vertices_decision(
        &self,
        decisions: &DecisionContext,
    ) -> HypermeshResult<Vec<Point3>> {
        if let Some(vertices) = &self.known_vertices {
            return Ok(vertices.to_vec());
        }
        (0..self.vertex_count())
            .map(|index| self.vertex_point_decision(decisions, index))
            .collect()
    }

    /// Returns a polygon with inverted support orientation and vertex winding.
    ///
    /// Edge planes retain their interior-facing halfspaces: polygon interior
    /// remains on every edge's non-positive side after orientation reversal.
    pub fn inverted(&self) -> Self {
        let mut result = self.clone();
        result.replace_planes(
            self.support_plane().inverted(),
            self.edge_planes().iter().rev().cloned().collect(),
        );
        result.known_vertices = self
            .known_vertices
            .as_ref()
            .map(RetainedVertexCycle::reversed);
        result.known_identities = self.known_identities.as_ref().map(|identities| {
            let vertices = Arc::from(identities.vertices().iter().rev().collect::<Vec<_>>());
            let edges = identities.edges();
            let count = edges.len();
            let edges = Arc::from(
                (0..count)
                    .map(|index| {
                        edges
                            .get((count + count - 2 - index) % count)
                            .expect("known edge identity indices are retained")
                    })
                    .collect::<Vec<_>>(),
            );
            RetainedIdentityCycles::Owned { vertices, edges }
        });
        result
    }

    #[inline]
    #[cfg(test)]
    pub(crate) fn set_source_triangle_edge_identities(
        &mut self,
        mesh: usize,
        vertices: [usize; 3],
    ) -> HypermeshResult<()> {
        let [first, second, third] = vertices;
        let mesh = compact_construction_index(mesh, "source triangle mesh ID")?;
        let vertices = [
            compact_construction_index(first, "source triangle vertex ID")?,
            compact_construction_index(second, "source triangle vertex ID")?,
            compact_construction_index(third, "source triangle vertex ID")?,
        ];
        self.known_identities = Some(RetainedIdentityCycles::SourceTriangle { mesh, vertices });
        Ok(())
    }

    /// Returns true if a homogeneous point lies on or inside the polygon.
    pub fn contains_point(
        &self,
        context: &MeshContext,
        point: &HomogeneousPoint3,
    ) -> HypermeshResult<MeshOutcome<bool>> {
        let decisions = DecisionContext::new(context);
        let contains = self.contains_point_decision(&decisions, point)?;
        Ok(decisions.finish(contains))
    }

    pub(crate) fn contains_point_decision(
        &self,
        decisions: &DecisionContext,
        point: &HomogeneousPoint3,
    ) -> HypermeshResult<bool> {
        let support = self.support_plane();
        if classify_projective_point_decision(decisions, point, support)? != Classification::On {
            return Ok(false);
        }
        let edges = self.edge_planes();
        for edge in edges {
            if classify_projective_point_decision(decisions, point, edge)?.is_positive() {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Returns true if a homogeneous point lies strictly inside the polygon.
    pub fn contains_point_strictly(
        &self,
        context: &MeshContext,
        point: &HomogeneousPoint3,
    ) -> HypermeshResult<MeshOutcome<bool>> {
        let decisions = DecisionContext::new(context);
        let contains = self.contains_point_strictly_decision(&decisions, point)?;
        Ok(decisions.finish(contains))
    }

    pub(crate) fn contains_point_strictly_decision(
        &self,
        decisions: &DecisionContext,
        point: &HomogeneousPoint3,
    ) -> HypermeshResult<bool> {
        let support = self.support_plane();
        if classify_projective_point_decision(decisions, point, support)? != Classification::On {
            return Ok(false);
        }
        let edges = self.edge_planes();
        for edge in edges {
            if classify_projective_point_decision(decisions, point, edge)?.is_non_negative() {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

/// Returns a convex triangle from three exact positions.
pub fn convex_triangle(
    context: &MeshContext,
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    mesh_index: isize,
    polygon_index: isize,
) -> HypermeshResult<MeshOutcome<ConvexPolygon>> {
    let decisions = DecisionContext::new(context);
    let normalize_wide_dyadic = points_require_wide_dyadic_plane_normalization([p0, p1, p2]);
    let polygon = convex_triangle_decision(
        &decisions,
        p0,
        p1,
        p2,
        mesh_index,
        polygon_index,
        normalize_wide_dyadic,
    )?;
    Ok(decisions.finish(polygon))
}

pub(crate) fn convex_triangle_decision(
    decisions: &DecisionContext,
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    mesh_index: isize,
    polygon_index: isize,
    normalize_wide_dyadic: bool,
) -> HypermeshResult<ConvexPolygon> {
    let support = Plane::from_points(p0, p1, p2);
    let edges = Vec::from([
        edge_plane(decisions, p0, p1, p2, &support, normalize_wide_dyadic)?,
        edge_plane(decisions, p1, p2, p0, &support, normalize_wide_dyadic)?,
        edge_plane(decisions, p2, p0, p1, &support, normalize_wide_dyadic)?,
    ]);
    ConvexPolygon::from_triangle_planes(
        decisions,
        [p0, p1, p2],
        support,
        edges,
        mesh_index,
        polygon_index,
    )
}

impl ConvexPolygon {
    pub(crate) fn from_triangle_planes(
        decisions: &DecisionContext,
        [p0, p1, p2]: [&Point3; 3],
        support: Plane,
        edges: Vec<Plane>,
        mesh_index: isize,
        polygon_index: isize,
    ) -> HypermeshResult<Self> {
        debug_assert_eq!(edges.len(), 3);
        Ok(Self {
            planes: PolygonPlanes::owned(support, edges),
            mesh_index,
            polygon_index,
            delta_w: Vec::new(),
            approx_bounds: Some(Box::new(bounds_for_points(decisions, &[p0, p1, p2])?)),
            known_vertices: Some(RetainedVertexCycle::Owned(Arc::new([
                p0.clone(),
                p1.clone(),
                p2.clone(),
            ]))),
            known_identities: None,
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn from_source_triangle(
        decisions: &DecisionContext,
        source_planes: Arc<RetainedSourcePlanes>,
        positions: Arc<RetainedSourcePositions>,
        vertices: [u32; 3],
        face: usize,
        mesh_index: usize,
        signed_mesh_index: isize,
        polygon_index: isize,
    ) -> HypermeshResult<Self> {
        let mesh = compact_construction_index(mesh_index, "source triangle mesh ID")?;
        let compact_face = compact_construction_index(face, "source triangle plane-cache ID")?;
        if face >= source_planes.len() {
            return Err(crate::error::HypermeshError::SurfaceArrangementFailed {
                reason: "source triangle has no retained plane-cache row",
            });
        }
        let points = vertices.map(|vertex| {
            positions.get(vertex as usize).ok_or(
                crate::error::HypermeshError::VertexIndexOutOfBounds {
                    index: vertex as usize,
                    vertex_count: positions.len(),
                },
            )
        });
        let [p0, p1, p2] = points;
        let [p0, p1, p2] = [p0?, p1?, p2?];
        let extrema = bounds_extrema_for_triangle(decisions, [p0, p1, p2])?;
        Ok(Self {
            planes: PolygonPlanes::SourceTriangle {
                owner: Arc::clone(&source_planes),
                face: compact_face,
            },
            mesh_index: signed_mesh_index,
            polygon_index,
            delta_w: Vec::new(),
            approx_bounds: None,
            known_vertices: Some(RetainedVertexCycle::SourceTriangle {
                positions,
                vertices,
                extrema,
            }),
            known_identities: Some(RetainedIdentityCycles::SourceTriangle { mesh, vertices }),
        })
    }
}

/// Returns a convex quad from four coplanar exact positions in winding order.
pub fn convex_quad(
    context: &MeshContext,
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    p3: &Point3,
    mesh_index: isize,
    polygon_index: isize,
) -> HypermeshResult<MeshOutcome<ConvexPolygon>> {
    let decisions = DecisionContext::new(context);
    let normalize_wide_dyadic = points_require_wide_dyadic_plane_normalization([p0, p1, p2, p3]);
    let polygon = convex_quad_decision(
        &decisions,
        [p0, p1, p2, p3],
        mesh_index,
        polygon_index,
        normalize_wide_dyadic,
    )?;
    Ok(decisions.finish(polygon))
}

pub(crate) fn convex_quad_decision(
    decisions: &DecisionContext,
    [p0, p1, p2, p3]: [&Point3; 4],
    mesh_index: isize,
    polygon_index: isize,
    normalize_wide_dyadic: bool,
) -> HypermeshResult<ConvexPolygon> {
    let support = Plane::from_points(p0, p1, p2);
    let edges = Vec::from([
        edge_plane(decisions, p0, p1, p2, &support, normalize_wide_dyadic)?,
        edge_plane(decisions, p1, p2, p3, &support, normalize_wide_dyadic)?,
        edge_plane(decisions, p2, p3, p0, &support, normalize_wide_dyadic)?,
        edge_plane(decisions, p3, p0, p1, &support, normalize_wide_dyadic)?,
    ]);

    Ok(ConvexPolygon {
        planes: PolygonPlanes::owned(support, edges),
        mesh_index,
        polygon_index,
        delta_w: Vec::new(),
        approx_bounds: Some(Box::new(bounds_for_points(decisions, &[p0, p1, p2, p3])?)),
        known_vertices: Some(RetainedVertexCycle::Owned(Arc::new([
            p0.clone(),
            p1.clone(),
            p2.clone(),
            p3.clone(),
        ]))),
        known_identities: None,
    })
}

pub(crate) fn source_triangle_planes(
    points @ [first, second, third]: [&Point3; 3],
    normalize_wide_dyadic: bool,
) -> [Plane; 4] {
    let support = Plane::from_points(first, second, third);
    let edges = source_triangle_edge_planes(points, &support, normalize_wide_dyadic);
    let [first, second, third] = edges;
    [support, first, second, third]
}

fn source_triangle_edge_planes(
    [first, second, third]: [&Point3; 3],
    support: &Plane,
    normalize_wide_dyadic: bool,
) -> [Plane; 3] {
    // For n = (b-a) x (c-a), ((b-a) x n) . (c-a) = -(n . n).
    // A validated source triangle therefore puts its opposite vertex on the
    // negative side of each cyclic edge plane by construction. This algebraic
    // invariant fixes orientation without a second numeric predicate.
    [
        oriented_edge_plane(first, second, support, normalize_wide_dyadic),
        oriented_edge_plane(second, third, support, normalize_wide_dyadic),
        oriented_edge_plane(third, first, support, normalize_wide_dyadic),
    ]
}

pub(crate) fn edge_plane(
    decisions: &DecisionContext,
    a: &Point3,
    b: &Point3,
    opposite: &Point3,
    support: &Plane,
    normalize_wide_dyadic: bool,
) -> HypermeshResult<Plane> {
    let mut plane = oriented_edge_plane(a, b, support, normalize_wide_dyadic);
    if crate::predicate::classify_point_decision(decisions, opposite, &plane)?
        == Classification::Positive
    {
        plane = plane.inverted();
    }
    Ok(plane)
}

fn oriented_edge_plane(
    a: &Point3,
    b: &Point3,
    support: &Plane,
    normalize_wide_dyadic: bool,
) -> Plane {
    let edge = sub_points(b, a);
    let support_normal = [
        support.normal.x.clone(),
        support.normal.y.clone(),
        support.normal.z.clone(),
    ];
    let normal = cross_arrays(&edge, &support_normal);
    Plane::from_normal_and_point(normal, a, normalize_wide_dyadic)
}

pub(crate) fn points_require_wide_dyadic_plane_normalization<'a>(
    points: impl IntoIterator<Item = &'a Point3>,
) -> bool {
    crate::trace_dispatch!("wide-plane-normalization", "affine-content-scan");
    let mut points = points.into_iter();
    let Some(anchor) = points.next() else {
        return false;
    };
    let [Some(anchor_x), Some(anchor_y), Some(anchor_z)] =
        [&anchor.x, &anchor.y, &anchor.z].map(Real::exact_rational_ref)
    else {
        return false;
    };
    let anchors = [anchor_x, anchor_y, anchor_z];
    // A common wide numerator factor in exact dyadic displacements is an
    // affine, translation-invariant scale. Removing it from derived edge
    // planes pays back across many predicates. A wide denominator by itself
    // is merely coordinate resolution and does not justify normalization.
    let mut content: Option<Rational> = None;
    for point in points {
        let [Some(x), Some(y), Some(z)] =
            [&point.x, &point.y, &point.z].map(Real::exact_rational_ref)
        else {
            return false;
        };
        for (coordinate, anchor) in [x, y, z].into_iter().zip(anchors) {
            let Some(displacement_numerator) =
                coordinate.dyadic_difference_numerator_magnitude(anchor)
            else {
                return false;
            };
            if displacement_numerator.is_zero() {
                continue;
            }
            let common = match content {
                Some(content) => content.numerator_magnitude_gcd(&displacement_numerator),
                None => displacement_numerator,
            };
            if common.numerator().bits() <= u64::from(usize::BITS) {
                return false;
            }
            content = Some(common);
        }
    }
    content.is_some()
}

fn bounds_extrema_for_triangle(
    decisions: &DecisionContext,
    points: [&Point3; 3],
) -> HypermeshResult<SourceTriangleExtrema> {
    let mut extrema = [0_u8; 6];
    for axis in 0..3 {
        let values = points.map(|point| axis_ref(point, axis));
        let (mut minimum, maximum) = match compare_real_decision(decisions, values[1], values[0])? {
            std::cmp::Ordering::Less => (1, 0),
            std::cmp::Ordering::Greater => (0, 1),
            std::cmp::Ordering::Equal => (0, 0),
        };
        if compare_real_decision(decisions, values[2], values[minimum])?.is_lt() {
            minimum = 2;
        }
        let mut maximum = maximum;
        if compare_real_decision(decisions, values[2], values[maximum])?.is_gt() {
            maximum = 2;
        }
        extrema[axis] = minimum as u8;
        extrema[axis + 3] = maximum as u8;
    }
    Ok(SourceTriangleExtrema::new(extrema))
}

fn bounds_for_points(
    decisions: &DecisionContext,
    points: &[&Point3],
) -> HypermeshResult<ApproxBounds> {
    let (min_x, max_x) = min_max_real(decisions, points.iter().map(|point| &point.x))?;
    let (min_y, max_y) = min_max_real(decisions, points.iter().map(|point| &point.y))?;
    let (min_z, max_z) = min_max_real(decisions, points.iter().map(|point| &point.z))?;
    let min = Point3::new(min_x, min_y, min_z);
    let max = Point3::new(max_x, max_y, max_z);
    Ok(ApproxBounds::new(min, max))
}

fn min_max_real<'a>(
    decisions: &DecisionContext,
    mut values: impl Iterator<Item = &'a Real>,
) -> HypermeshResult<(Real, Real)> {
    let first = values
        .next()
        .expect("bounds need at least one point")
        .clone();
    let Some(second) = values.next() else {
        return Ok((first.clone(), first));
    };
    let (mut min, mut max) = match compare_real_decision(decisions, second, &first)? {
        std::cmp::Ordering::Less => (second.clone(), first),
        std::cmp::Ordering::Greater => (first, second.clone()),
        std::cmp::Ordering::Equal => (first.clone(), first),
    };
    while let Some(left) = values.next() {
        let Some(right) = values.next() else {
            update_min_max(decisions, left, &mut min, &mut max)?;
            break;
        };
        match compare_real_decision(decisions, right, left)? {
            std::cmp::Ordering::Less => {
                update_min(decisions, right, &mut min)?;
                update_max(decisions, left, &mut max)?;
            }
            std::cmp::Ordering::Greater => {
                update_min(decisions, left, &mut min)?;
                update_max(decisions, right, &mut max)?;
            }
            std::cmp::Ordering::Equal => {
                update_min_max(decisions, left, &mut min, &mut max)?;
            }
        }
    }
    Ok((min, max))
}

fn update_min(decisions: &DecisionContext, value: &Real, min: &mut Real) -> HypermeshResult<()> {
    if compare_real_decision(decisions, value, min)?.is_lt() {
        *min = value.clone();
    }
    Ok(())
}

fn update_max(decisions: &DecisionContext, value: &Real, max: &mut Real) -> HypermeshResult<()> {
    if compare_real_decision(decisions, value, max)?.is_gt() {
        *max = value.clone();
    }
    Ok(())
}

fn update_min_max(
    decisions: &DecisionContext,
    value: &Real,
    min: &mut Real,
    max: &mut Real,
) -> HypermeshResult<()> {
    update_min(decisions, value, min)?;
    update_max(decisions, value, max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::approximate_convex_triangle;
    fn point(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn source_triangle_identities_expand_from_compact_descriptor() {
        let mut polygon =
            approximate_convex_triangle(&point(0, 0, 0), &point(1, 0, 0), &point(0, 1, 0), 3, 7);
        polygon
            .set_source_triangle_edge_identities(3, [9, 2, 5])
            .unwrap();

        assert_eq!(std::mem::size_of::<ConstructionPlaneIdentity>(), 8);
        assert_eq!(std::mem::size_of::<ConstructionEdgeIdentity>(), 20);
        assert_eq!(std::mem::size_of::<ConstructionVertexIdentity>(), 28);
        assert_eq!(std::mem::size_of::<RetainedIdentityCycles>(), 32);
        assert_eq!(std::mem::size_of::<SourceTriangleExtrema>(), 2);
        assert_eq!(std::mem::size_of::<RetainedVertexCycle>(), 24);
        #[cfg(target_pointer_width = "64")]
        {
            assert_eq!(std::mem::size_of::<PolygonPlanes>(), 16);
            assert_eq!(std::mem::size_of::<SourceTrianglePlaneCache>(), 32);
            assert_eq!(std::mem::size_of::<ConvexPolygon>(), 128);
        }
        assert_eq!(
            polygon
                .known_vertex_identities()
                .unwrap()
                .iter()
                .collect::<Vec<_>>(),
            [9, 2, 5].map(|vertex| ConstructionVertexIdentity::Source { mesh: 3, vertex })
        );
        assert_eq!(
            polygon
                .known_edge_identities()
                .unwrap()
                .iter()
                .collect::<Vec<_>>(),
            [[2, 9], [2, 5], [5, 9]]
                .map(|endpoints| { ConstructionEdgeIdentity::Source { mesh: 3, endpoints } })
        );
    }

    #[test]
    fn retained_source_predicate_queries_follow_referenced_vertex_ids() {
        let positions: Arc<[Point3]> = Arc::from([point(0, 0, 0), point(1, 0, 0), point(0, 1, 0)]);
        let dense = RetainedSourcePositions::shared_with_predicate_queries(
            Arc::clone(&positions),
            &[true, true, true],
        )
        .unwrap();
        assert!(matches!(
            dense.predicate_queries,
            Some(RetainedSourcePredicateQueries::Dense(ref queries)) if queries.len() == 3
        ));
        assert!((0..3).all(|vertex| dense.predicate_query(vertex).is_some()));

        let indexed = RetainedSourcePositions::shared_with_predicate_queries(
            Arc::clone(&positions),
            &[true, false, true],
        )
        .unwrap();
        assert!(matches!(
            indexed.predicate_queries,
            Some(RetainedSourcePredicateQueries::Indexed {
                ref indices,
                ref queries,
            }) if **indices == [0, NO_RETAINED_SOURCE_PREDICATE_QUERY, 1]
                && queries.len() == 2
        ));
        assert!(indexed.predicate_query(0).is_some());
        assert!(indexed.predicate_query(1).is_none());
        assert!(indexed.predicate_query(2).is_some());
        assert!(indexed.predicate_query(3).is_none());

        let unavailable = RetainedSourcePositions::shared_with_predicate_queries(
            Arc::from([
                point(0, 0, 0),
                Point3::new(Real::pi(), Real::zero(), Real::zero()),
            ]),
            &[true, true],
        )
        .unwrap();
        assert!(unavailable.predicate_queries.is_none());
        assert!(unavailable.predicate_query(0).is_none());

        assert!(matches!(
            RetainedSourcePositions::shared_with_predicate_queries(positions, &[true, false]),
            Err(crate::HypermeshError::SurfaceArrangementFailed {
                reason: "source position and usage schedules differ"
            })
        ));
    }

    #[test]
    fn inverted_polygon_preserves_interior_edge_halfspaces() {
        let context = crate::test_support::APPROXIMATE_CONTEXT;
        let polygon = convex_quad(
            &context,
            &point(-2, -1, 0),
            &point(3, -1, 0),
            &point(3, 2, 0),
            &point(-2, 2, 0),
            0,
            0,
        )
        .unwrap()
        .into_value();
        let inverted = polygon.inverted();
        let interior =
            HomogeneousPoint3::new(Real::zero(), Real::zero(), Real::zero(), Real::one());

        assert!(
            inverted
                .contains_point_strictly(&context, &interior)
                .unwrap()
                .into_value()
        );
        assert_eq!(inverted.inverted(), polygon);
    }

    #[test]
    fn source_edge_orientation_is_fixed_by_the_valid_triangle_identity() {
        let points = [point(-3, 2, 5), point(7, -1, 4), point(2, 9, -6)];
        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let context = MeshContext::new(policy);
            let decisions = DecisionContext::new(&context);
            assert!(
                Plane::decide_points_are_nondegenerate(
                    &decisions, &points[0], &points[1], &points[2],
                )
                .unwrap()
            );
            let planes = source_triangle_planes([&points[0], &points[1], &points[2]], false);
            for edge in 0..3 {
                let first = &points[edge];
                let second = &points[(edge + 1) % 3];
                let opposite = &points[(edge + 2) % 3];
                assert_eq!(
                    crate::predicate::classify_point_decision(
                        &decisions,
                        opposite,
                        &planes[edge + 1],
                    )
                    .unwrap(),
                    Classification::Negative
                );
                assert_eq!(
                    edge_plane(&decisions, first, second, opposite, &planes[0], false).unwrap(),
                    planes[edge + 1]
                );
            }
            assert_eq!(decisions.certainty(), crate::MeshCertainty::Certified);
        }
    }

    #[test]
    fn retained_source_planes_reject_misaligned_compact_rows() {
        let positions = RetainedSourcePositions::shared(Arc::from([
            point(0, 0, 0),
            point(1, 0, 0),
            point(0, 1, 0),
        ]));
        let compact = Arc::new(CompactSourcePolygons::Binary64(
            vec![CompactSourcePolygon {
                planes: [[0.0; 4]; 4],
            }]
            .into_boxed_slice(),
        ));
        let error = RetainedSourcePlanes::new(2, Some(compact), false).unwrap_err();
        assert!(matches!(
            error,
            crate::HypermeshError::SurfaceArrangementFailed {
                reason: "retained source-plane and triangle counts differ"
            }
        ));

        let source_planes = RetainedSourcePlanes::new(1, None, false).unwrap();
        let context = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);
        let decisions = DecisionContext::new(&context);
        let error = ConvexPolygon::from_source_triangle(
            &decisions,
            source_planes,
            positions,
            [0, 1, 2],
            1,
            0,
            0,
            0,
        )
        .unwrap_err();
        assert!(matches!(
            error,
            crate::HypermeshError::SurfaceArrangementFailed {
                reason: "source triangle has no retained plane-cache row"
            }
        ));
    }

    #[test]
    fn wide_dyadic_triangle_uses_primitive_support_and_edge_planes() {
        let denominator = Rational::new(2)
            .powi(2048_i64.into())
            .expect("fixture exponent is positive");
        let scale = Real::from((&denominator + Rational::one()) / denominator);
        let p0 = Point3::new(Real::zero(), Real::zero(), Real::zero());
        let p1 = Point3::new(scale.clone(), Real::zero(), Real::zero());
        let p2 = Point3::new(Real::zero(), scale.clone(), Real::zero());

        for policy in [
            hyperlimit::PredicatePolicy::STRICT,
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ] {
            let outcome = convex_triangle(&MeshContext::new(policy), &p0, &p1, &p2, 0, 0)
                .expect("the exact triangle is nondegenerate");
            assert_eq!(outcome.certainty, crate::MeshCertainty::Certified);
            let polygon = outcome.value;
            assert_eq!(
                polygon.support_plane().clone(),
                Plane::new(
                    Point3::new(Real::zero(), Real::zero(), Real::one()),
                    Real::zero(),
                )
            );
            assert_eq!(
                polygon.edge_planes(),
                [
                    Plane::new(
                        Point3::new(Real::zero(), Real::from(-1), Real::zero()),
                        Real::zero(),
                    ),
                    Plane::new(
                        Point3::new(Real::one(), Real::one(), Real::zero()),
                        -scale.clone(),
                    ),
                    Plane::new(
                        Point3::new(Real::from(-1), Real::zero(), Real::zero()),
                        Real::zero(),
                    ),
                ]
            );
        }
    }

    #[test]
    fn wide_plane_schedule_requires_affine_numerator_content() {
        let denominator = Rational::new(2)
            .powi(2048_i64.into())
            .expect("fixture exponent is positive");
        let scale = Real::from((&denominator + Rational::one()) / &denominator);
        let translation = Real::from(
            (&denominator + Rational::from(3_u8)) / (&denominator * Rational::from(2_u8)),
        );
        let p0 = Point3::new(translation.clone(), Real::zero(), Real::zero());
        let p1 = Point3::new(
            translation.clone() + scale.clone(),
            Real::zero(),
            Real::zero(),
        );
        let p2 = Point3::new(
            translation + scale.clone() * Real::from(3_u8),
            scale,
            Real::zero(),
        );
        assert!(points_require_wide_dyadic_plane_normalization([
            &p0, &p1, &p2
        ]));

        let resolution_only = Real::from(Rational::one() / &denominator);
        let unrelated = Point3::new(resolution_only.clone(), Real::zero(), Real::zero());
        let unrelated_2 = Point3::new(
            resolution_only * Real::from(3_u8),
            Real::one(),
            Real::zero(),
        );
        assert!(!points_require_wide_dyadic_plane_normalization([
            &Point3::origin(),
            &unrelated,
            &unrelated_2,
        ]));
    }

    #[cfg(target_pointer_width = "64")]
    #[test]
    fn compact_source_triangle_identity_overflow_is_rejected_before_mutation() {
        let mut polygon = ConvexPolygon::empty();
        assert!(
            polygon
                .set_source_triangle_edge_identities(0, [0, 1, usize::MAX])
                .is_err()
        );
        assert!(polygon.known_identities.is_none());
    }
}
