//! Convex polygon representation backed by hyperreal planes.

use hyperlattice::{HomogeneousPoint3, Point3, Rational, Real, intersect_three_planes};
use hyperreal::RealSign;
use std::sync::Arc;

use crate::error::HypermeshResult;
use crate::geometry::{
    Classification, Plane, classify_projective_point, cross_arrays, dot_point, sub_points,
};
use crate::winding::WindingNumberTransitionVector;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub(crate) struct ConstructionPlaneIdentity {
    pub(crate) mesh: usize,
    pub(crate) plane: usize,
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub(crate) enum ConstructionEdgeIdentity {
    Source {
        mesh: usize,
        endpoints: [usize; 2],
    },
    Split {
        planes: [ConstructionPlaneIdentity; 2],
    },
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub(crate) enum ConstructionVertexIdentity {
    Source {
        mesh: usize,
        vertex: usize,
    },
    SourceEdgePlane {
        mesh: usize,
        endpoints: [usize; 2],
        plane: ConstructionPlaneIdentity,
    },
    PlaneTriple {
        planes: [ConstructionPlaneIdentity; 3],
    },
}

/// Exact oriented support and boundary planes for one input triangle.
///
/// Mesh owners that retain affine-transform provenance can transform these
/// geometric objects directly instead of reconstructing them from expanded
/// transformed coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct InputTrianglePlanes {
    /// Oriented triangle support plane.
    pub support: Plane,
    /// Oriented edge planes in triangle winding order.
    pub edges: [Plane; 3],
}

impl InputTrianglePlanes {
    /// Constructs the support and three boundary planes from source points.
    pub fn from_points(p0: &Point3, p1: &Point3, p2: &Point3) -> Self {
        let support = Plane::from_points(p0, p1, p2);
        let points = [p0, p1, p2];
        let edges = std::array::from_fn(|i| {
            edge_plane(
                points[i],
                points[(i + 1) % 3],
                points[(i + 2) % 3],
                &support,
            )
        });
        Self { support, edges }
    }
}

/// Approximate exact-coordinate bounds for fast spatial rejection.
#[derive(Clone, Debug, PartialEq)]
pub struct ApproxBounds {
    /// Minimum coordinate by axis.
    pub min: Point3,
    /// Maximum coordinate by axis.
    pub max: Point3,
}

#[derive(Clone, Debug)]
pub(crate) enum RetainedVertexCycle {
    Owned(Arc<[Point3]>),
    IndexedTriangle {
        positions: Arc<[Point3]>,
        indices: [usize; 3],
    },
    SourceIndexed {
        positions: Arc<[Point3]>,
        identities: Arc<[ConstructionVertexIdentity]>,
    },
}

impl RetainedVertexCycle {
    pub(crate) fn len(&self) -> usize {
        match self {
            Self::Owned(vertices) => vertices.len(),
            Self::IndexedTriangle { .. } => 3,
            Self::SourceIndexed { identities, .. } => identities.len(),
        }
    }

    pub(crate) fn get(&self, index: usize) -> Option<&Point3> {
        match self {
            Self::Owned(vertices) => vertices.get(index),
            Self::IndexedTriangle { positions, indices } => positions.get(*indices.get(index)?),
            Self::SourceIndexed {
                positions,
                identities,
            } => {
                let ConstructionVertexIdentity::Source { vertex, .. } = identities.get(index)?
                else {
                    return None;
                };
                positions.get(*vertex)
            }
        }
    }

    pub(crate) fn source_positions(&self) -> Option<&Arc<[Point3]>> {
        match self {
            Self::IndexedTriangle { positions, .. } | Self::SourceIndexed { positions, .. } => {
                Some(positions)
            }
            Self::Owned(_) => None,
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
}

#[derive(Clone, Debug)]
pub(crate) enum RetainedIdentityCycles {
    SourceTriangle {
        mesh: usize,
        vertices: [usize; 3],
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

    /// Computes bounds for a non-empty borrowed point slice.
    pub fn for_points(points: &[&Point3]) -> Self {
        bounds_for_points(points)
    }
}

/// Plane-bounded convex polygon.
#[derive(Clone, Debug)]
pub struct ConvexPolygon {
    /// Supporting plane.
    pub support: Plane,
    /// Edge planes. Interior is on the non-positive side of each edge.
    pub edges: Arc<Vec<Plane>>,
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
    /// Derived clipping and BSP polygons clear this cache when their edge
    /// cycle changes.
    pub(crate) known_vertices: Option<RetainedVertexCycle>,
    pub(crate) known_identities: Option<RetainedIdentityCycles>,
}

impl PartialEq for ConvexPolygon {
    fn eq(&self, other: &Self) -> bool {
        self.support == other.support
            && self.edges == other.edges
            && self.mesh_index == other.mesh_index
            && self.polygon_index == other.polygon_index
            && self.delta_w == other.delta_w
            && self.approx_bounds == other.approx_bounds
    }
}

impl ConvexPolygon {
    /// Constructs an empty polygon carrier.
    pub fn empty() -> Self {
        Self {
            support: Plane::from_coefficients(
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
            ),
            edges: Arc::new(Vec::new()),
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
            .map_or(self.edges.len(), |vertices| vertices.len())
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

    pub(crate) fn known_edge_identities(&self) -> Option<KnownEdgeIdentityCycle<'_>> {
        self.known_identities
            .as_ref()
            .map(RetainedIdentityCycles::edges)
    }

    /// Returns true when this polygon has at least three vertices and a
    /// non-zero support normal.
    pub fn is_valid(&self) -> bool {
        self.vertex_count() >= 3 && self.support.is_valid()
    }

    /// Computes vertex `i` as a homogeneous intersection of support and two
    /// adjacent edge planes.
    pub fn vertex(&self, i: usize) -> HomogeneousPoint3 {
        let n = self.vertex_count();
        intersect_three_planes(&self.support, &self.edges[i], &self.edges[(i + 1) % n])
    }

    /// Computes an affine vertex.
    pub fn vertex_point(&self, i: usize) -> HypermeshResult<Point3> {
        self.vertex(i).to_affine_point().map_err(|_| {
            if self.vertex(i).w.definitely_zero() {
                crate::error::HypermeshError::PointAtInfinity
            } else {
                crate::error::HypermeshError::UnknownClassification
            }
        })
    }

    /// Computes all affine vertices.
    pub fn vertices(&self) -> HypermeshResult<Vec<Point3>> {
        if let Some(vertices) = &self.known_vertices {
            return Ok(vertices.to_vec());
        }
        (0..self.vertex_count())
            .map(|index| self.vertex_point(index))
            .collect()
    }

    /// Returns an inverted polygon with reversed edge winding.
    pub fn inverted(&self) -> Self {
        let mut result = self.clone();
        result.support = self.support.inverted();
        result.edges = Arc::new(
            self.edges
                .iter()
                .rev()
                .map(Plane::inverted)
                .collect::<Vec<_>>(),
        );
        result.known_vertices = self.known_vertices.as_ref().map(|vertices| {
            RetainedVertexCycle::Owned(Arc::from(
                vertices.iter().rev().cloned().collect::<Vec<_>>(),
            ))
        });
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

    pub(crate) fn with_known_vertex_cycle_and_edges(
        &self,
        vertices: Vec<Point3>,
        vertex_identities: Vec<ConstructionVertexIdentity>,
        edges: Vec<Plane>,
        edge_identities: Vec<ConstructionEdgeIdentity>,
    ) -> Self {
        debug_assert_eq!(vertices.len(), edges.len());
        debug_assert_eq!(vertices.len(), vertex_identities.len());
        debug_assert_eq!(vertices.len(), edge_identities.len());
        let approx_bounds =
            (!vertices.is_empty()).then(|| Box::new(bounds_for_owned_points(vertices.as_slice())));
        let mut result = self.clone();
        result.edges = Arc::new(edges);
        result.approx_bounds = approx_bounds;
        result.known_vertices = Some(RetainedVertexCycle::Owned(Arc::from(vertices)));
        result.known_identities = Some(RetainedIdentityCycles::Owned {
            vertices: Arc::from(vertex_identities),
            edges: Arc::from(edge_identities),
        });
        result
    }

    pub(crate) fn with_known_vertex_cycle_and_identities(
        &self,
        vertices: Vec<Point3>,
        vertex_identities: Vec<ConstructionVertexIdentity>,
    ) -> Self {
        let edge_identities = self
            .known_edge_identities()
            .expect("known vertex identities have an aligned edge cycle");
        debug_assert_eq!(vertices.len(), vertex_identities.len());
        debug_assert_eq!(vertices.len(), edge_identities.len());
        let approx_bounds =
            (!vertices.is_empty()).then(|| Box::new(bounds_for_owned_points(vertices.as_slice())));
        let mut result = self.clone();
        result.approx_bounds = approx_bounds;
        result.known_vertices = Some(RetainedVertexCycle::Owned(Arc::from(vertices)));
        result.known_identities = Some(RetainedIdentityCycles::Owned {
            vertices: Arc::from(vertex_identities),
            edges: Arc::from(edge_identities.iter().collect::<Vec<_>>()),
        });
        result
    }

    pub(crate) fn with_source_triangle_edge_identities(
        mut self,
        mesh: usize,
        vertices: [usize; 3],
    ) -> Self {
        self.known_identities = Some(RetainedIdentityCycles::SourceTriangle { mesh, vertices });
        self
    }

    pub(crate) fn from_certified_convex_face(
        support: Plane,
        vertices: Vec<Point3>,
        indexed_positions: Option<Arc<[Point3]>>,
        vertex_identities: Vec<ConstructionVertexIdentity>,
        edges: Vec<Plane>,
        edge_identities: Vec<ConstructionEdgeIdentity>,
        mesh_index: isize,
        polygon_index: isize,
        delta_w: WindingNumberTransitionVector,
    ) -> Self {
        debug_assert_eq!(vertices.len(), vertex_identities.len());
        debug_assert!(edges.is_empty() || vertices.len() == edges.len());
        debug_assert_eq!(vertices.len(), edge_identities.len());
        debug_assert!(indexed_positions.as_ref().is_none_or(|positions| {
            vertices
                .iter()
                .zip(&vertex_identities)
                .all(|(point, identity)| {
                    let ConstructionVertexIdentity::Source { vertex, .. } = identity else {
                        return false;
                    };
                    positions.get(*vertex) == Some(point)
                })
        }));
        let vertex_identities = Arc::from(vertex_identities);
        let known_vertices = match indexed_positions {
            Some(positions) => RetainedVertexCycle::SourceIndexed {
                positions,
                identities: Arc::clone(&vertex_identities),
            },
            None => RetainedVertexCycle::Owned(Arc::from(vertices)),
        };
        Self {
            support,
            edges: Arc::new(edges),
            mesh_index,
            polygon_index,
            delta_w,
            // Certified convex faces are consumed only by the projective
            // two-input candidate, which classifies directly against support
            // planes. A failed candidate rebuilds ordinary input polygons
            // before any BVH or subdivision query.
            approx_bounds: None,
            known_vertices: Some(known_vertices),
            known_identities: Some(RetainedIdentityCycles::Owned {
                vertices: vertex_identities,
                edges: Arc::from(edge_identities),
            }),
        }
    }

    pub(crate) fn with_rebuilt_edge_planes(&self) -> HypermeshResult<Self> {
        let vertices = self.vertices()?;
        if vertices.len() < 3 {
            return Ok(self.clone());
        }
        let edges = (0..vertices.len())
            .map(|index| {
                edge_plane(
                    &vertices[index],
                    &vertices[(index + 1) % vertices.len()],
                    &vertices[(index + 2) % vertices.len()],
                    &self.support,
                )
            })
            .collect();
        let mut result = self.clone();
        result.edges = Arc::new(edges);
        Ok(result)
    }

    /// Returns true if a homogeneous point lies on or inside the polygon.
    pub fn contains_point(&self, point: &HomogeneousPoint3) -> HypermeshResult<bool> {
        if classify_projective_point(point, &self.support)? != Classification::On {
            return Ok(false);
        }
        for edge in self.edges.iter() {
            if classify_projective_point(point, edge)?.is_positive() {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Returns true if a homogeneous point lies strictly inside the polygon.
    pub fn contains_point_strictly(&self, point: &HomogeneousPoint3) -> HypermeshResult<bool> {
        if classify_projective_point(point, &self.support)? != Classification::On {
            return Ok(false);
        }
        for edge in self.edges.iter() {
            if classify_projective_point(point, edge)?.is_non_negative() {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

/// Creates a triangle polygon from three exact positions.
pub fn make_triangle(
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    let support = Plane::from_points(p0, p1, p2);
    let points = [p0, p1, p2];
    let edges = (0..3)
        .map(|i| {
            edge_plane(
                points[i],
                points[(i + 1) % 3],
                points[(i + 2) % 3],
                &support,
            )
        })
        .collect();

    ConvexPolygon {
        support,
        edges: Arc::new(edges),
        mesh_index,
        polygon_index,
        delta_w: Vec::new(),
        approx_bounds: Some(Box::new(bounds_for_points(&[p0, p1, p2]))),
        known_vertices: Some(RetainedVertexCycle::Owned(Arc::new([
            p0.clone(),
            p1.clone(),
            p2.clone(),
        ]))),
        known_identities: None,
    }
}

pub(crate) fn make_triangle_with_input_planes(
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    planes: InputTrianglePlanes,
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    ConvexPolygon {
        support: planes.support,
        edges: Arc::new(Vec::from(planes.edges)),
        mesh_index,
        polygon_index,
        delta_w: Vec::new(),
        approx_bounds: Some(Box::new(bounds_for_points(&[p0, p1, p2]))),
        known_vertices: Some(RetainedVertexCycle::Owned(Arc::new([
            p0.clone(),
            p1.clone(),
            p2.clone(),
        ]))),
        known_identities: None,
    }
}

#[cfg(test)]
pub(crate) fn make_triangle_with_deferred_edges(
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    let support = Plane::from_points(p0, p1, p2);
    ConvexPolygon {
        // Certified two-convex preparation needs only aligned placeholders
        // for source edges that actually reach projective clipping. The
        // support already carries that deferred plane, so keep this empty and
        // expand it at the narrower projective boundary.
        edges: Arc::new(Vec::new()),
        support,
        mesh_index,
        polygon_index,
        delta_w: Vec::new(),
        approx_bounds: Some(Box::new(bounds_for_points(&[p0, p1, p2]))),
        known_vertices: Some(RetainedVertexCycle::Owned(Arc::new([
            p0.clone(),
            p1.clone(),
            p2.clone(),
        ]))),
        known_identities: None,
    }
}

pub(crate) fn make_indexed_triangle_with_deferred_edges(
    positions: Arc<[Point3]>,
    indices: [usize; 3],
    support_hint: Option<Plane>,
    deferred_edges: Arc<Vec<Plane>>,
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    debug_assert!(deferred_edges.is_empty());
    let [i0, i1, i2] = indices;
    let p0 = &positions[i0];
    let p1 = &positions[i1];
    let p2 = &positions[i2];
    let support = support_hint.unwrap_or_else(|| Plane::from_points(p0, p1, p2));
    ConvexPolygon {
        edges: deferred_edges,
        support,
        mesh_index,
        polygon_index,
        delta_w: Vec::new(),
        // The indexed carrier is used only by the certified two-convex
        // projective candidate, which classifies directly against support
        // planes and never queries polygon AABBs. A failed candidate rebuilds
        // ordinary input polygons before entering BVH/subdivision code.
        approx_bounds: None,
        known_vertices: Some(RetainedVertexCycle::IndexedTriangle { positions, indices }),
        known_identities: None,
    }
}

pub(crate) fn exact_axis_aligned_triangle_support(
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    axis: usize,
    orientation_hint: Option<RealSign>,
) -> Option<Plane> {
    let points = [
        [&p0.x, &p0.y, &p0.z],
        [&p1.x, &p1.y, &p1.z],
        [&p2.x, &p2.y, &p2.z],
    ];
    let [Some(value), Some(second), Some(third)] =
        points.map(|point| point.get(axis)?.exact_rational_ref())
    else {
        return None;
    };
    if value != second || value != third {
        return None;
    }
    // For a triangle in an axis plane, the cyclic complementary-coordinate
    // determinant is exactly the corresponding component of its cross
    // product. Its sign therefore supplies the original support orientation
    // without materializing any coordinate differences or plane scale.
    let u = (axis + 1) % 3;
    let v = (axis + 2) % 3;
    let orientation = match orientation_hint.or_else(|| {
        Real::certified_affine_det2_sign(
            [points[0][u], points[0][v]],
            [points[1][u], points[1][v]],
            [points[2][u], points[2][v]],
        )
    }) {
        Some(RealSign::Negative) => std::cmp::Ordering::Less,
        Some(RealSign::Positive) => std::cmp::Ordering::Greater,
        Some(RealSign::Zero) | None => {
            let [Some(p0u), Some(p1u), Some(p2u)] =
                points.map(|point| point[u].exact_rational_ref())
            else {
                return None;
            };
            let [Some(p0v), Some(p1v), Some(p2v)] =
                points.map(|point| point[v].exact_rational_ref())
            else {
                return None;
            };
            Rational::signed_product_sum_ordering(
                [true, true, true, false, false, false],
                [
                    [p0u, p1v],
                    [p1u, p2v],
                    [p2u, p0v],
                    [p0u, p2v],
                    [p1u, p0v],
                    [p2u, p1v],
                ],
            )
        }
    };
    match orientation {
        std::cmp::Ordering::Less => {
            Some(Plane::axis_aligned(axis, points[0][axis].clone()).inverted())
        }
        std::cmp::Ordering::Equal => None,
        std::cmp::Ordering::Greater => Some(Plane::axis_aligned(axis, points[0][axis].clone())),
    }
}

pub(crate) fn make_indexed_triangle_with_deferred_edges_and_input_planes(
    positions: Arc<[Point3]>,
    indices: [usize; 3],
    planes: InputTrianglePlanes,
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    ConvexPolygon {
        edges: Arc::new(Vec::from(planes.edges)),
        support: planes.support,
        mesh_index,
        polygon_index,
        delta_w: Vec::new(),
        approx_bounds: None,
        known_vertices: Some(RetainedVertexCycle::IndexedTriangle { positions, indices }),
        known_identities: None,
    }
}

/// Creates a quad polygon from four coplanar exact positions in winding order.
pub fn make_quad(
    p0: &Point3,
    p1: &Point3,
    p2: &Point3,
    p3: &Point3,
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    let support = Plane::from_points(p0, p1, p2);
    let points = [p0, p1, p2, p3];
    let edges = (0..4)
        .map(|i| {
            edge_plane(
                points[i],
                points[(i + 1) % 4],
                points[(i + 2) % 4],
                &support,
            )
        })
        .collect();

    ConvexPolygon {
        support,
        edges: Arc::new(edges),
        mesh_index,
        polygon_index,
        delta_w: Vec::new(),
        approx_bounds: Some(Box::new(bounds_for_points(&[p0, p1, p2, p3]))),
        known_vertices: Some(RetainedVertexCycle::Owned(Arc::new([
            p0.clone(),
            p1.clone(),
            p2.clone(),
            p3.clone(),
        ]))),
        known_identities: None,
    }
}

pub(crate) fn edge_plane(a: &Point3, b: &Point3, opposite: &Point3, support: &Plane) -> Plane {
    let mut plane = oriented_edge_plane(a, b, support);
    if matches!(
        crate::geometry::classify_point(opposite, &plane),
        Ok(Classification::Positive)
    ) {
        plane = plane.inverted();
    }
    plane
}

fn oriented_edge_plane(a: &Point3, b: &Point3, support: &Plane) -> Plane {
    let edge = sub_points(b, a);
    let support_normal = [
        support.normal.x.clone(),
        support.normal.y.clone(),
        support.normal.z.clone(),
    ];
    let normal = cross_arrays(&edge, &support_normal);
    let offset = -dot_point(&normal, a);
    Plane::new(normal, offset)
}

fn bounds_for_points(points: &[&Point3]) -> ApproxBounds {
    let (min_x, max_x) = min_max_real(points.iter().map(|point| &point.x));
    let (min_y, max_y) = min_max_real(points.iter().map(|point| &point.y));
    let (min_z, max_z) = min_max_real(points.iter().map(|point| &point.z));
    let min = Point3::new(min_x, min_y, min_z);
    let max = Point3::new(max_x, max_y, max_z);
    ApproxBounds::new(min, max)
}

fn bounds_for_owned_points(points: &[Point3]) -> ApproxBounds {
    let (min_x, max_x) = min_max_real(points.iter().map(|point| &point.x));
    let (min_y, max_y) = min_max_real(points.iter().map(|point| &point.y));
    let (min_z, max_z) = min_max_real(points.iter().map(|point| &point.z));
    let min = Point3::new(min_x, min_y, min_z);
    let max = Point3::new(max_x, max_y, max_z);
    ApproxBounds::new(min, max)
}

fn min_max_real<'a>(mut values: impl Iterator<Item = &'a Real>) -> (Real, Real) {
    let first = values
        .next()
        .expect("bounds need at least one point")
        .clone();
    let Some(second) = values.next() else {
        return (first.clone(), first);
    };
    let (mut min, mut max) = match crate::geometry::compare_real(second, &first) {
        Ok(std::cmp::Ordering::Less) => (second.clone(), first),
        Ok(std::cmp::Ordering::Greater) => (first, second.clone()),
        Ok(std::cmp::Ordering::Equal) | Err(_) => (first.clone(), first),
    };
    while let Some(left) = values.next() {
        let Some(right) = values.next() else {
            update_min_max(left, &mut min, &mut max);
            break;
        };
        match crate::geometry::compare_real(right, left) {
            Ok(std::cmp::Ordering::Less) => {
                update_min(right, &mut min);
                update_max(left, &mut max);
            }
            Ok(std::cmp::Ordering::Greater) => {
                update_min(left, &mut min);
                update_max(right, &mut max);
            }
            Ok(std::cmp::Ordering::Equal) => update_min_max(left, &mut min, &mut max),
            Err(_) => {
                update_min_max(left, &mut min, &mut max);
                update_min_max(right, &mut min, &mut max);
            }
        }
    }
    (min, max)
}

fn update_min(value: &Real, min: &mut Real) {
    if matches!(
        crate::geometry::compare_real(value, min),
        Ok(std::cmp::Ordering::Less)
    ) {
        *min = value.clone();
    }
}

fn update_max(value: &Real, max: &mut Real) {
    if matches!(
        crate::geometry::compare_real(value, max),
        Ok(std::cmp::Ordering::Greater)
    ) {
        *max = value.clone();
    }
}

fn update_min_max(value: &Real, min: &mut Real, max: &mut Real) {
    update_min(value, min);
    update_max(value, max);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn source_triangle_identities_expand_from_compact_descriptor() {
        let polygon = make_triangle(&point(0, 0, 0), &point(1, 0, 0), &point(0, 1, 0), 3, 7)
            .with_source_triangle_edge_identities(3, [9, 2, 5]);

        assert!(std::mem::size_of::<RetainedIdentityCycles>() <= 5 * std::mem::size_of::<usize>());
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
    fn certified_source_face_reuses_position_and_identity_arenas() {
        let positions: Arc<[Point3]> = Arc::new([
            point(0, 0, 0),
            point(1, 0, 0),
            point(1, 1, 0),
            point(0, 1, 0),
        ]);
        let indices = [0, 1, 2, 3];
        let vertex_identities = indices
            .map(|vertex| ConstructionVertexIdentity::Source { mesh: 0, vertex })
            .to_vec();
        let edge_identities = [[0, 1], [1, 2], [2, 3], [0, 3]]
            .map(|endpoints| ConstructionEdgeIdentity::Source { mesh: 0, endpoints })
            .to_vec();
        let polygon = ConvexPolygon::from_certified_convex_face(
            Plane::axis_aligned(2, Real::zero()),
            indices.map(|index| positions[index].clone()).to_vec(),
            Some(Arc::clone(&positions)),
            vertex_identities,
            Vec::new(),
            edge_identities,
            0,
            0,
            vec![1],
        );

        let Some(RetainedVertexCycle::SourceIndexed {
            positions: retained_positions,
            identities: vertex_positions,
        }) = &polygon.known_vertices
        else {
            panic!("certified source face should retain indexed positions");
        };
        let Some(RetainedIdentityCycles::Owned { vertices, edges: _ }) = &polygon.known_identities
        else {
            panic!("certified source face should retain expanded identities");
        };
        assert!(Arc::ptr_eq(retained_positions, &positions));
        assert!(Arc::ptr_eq(vertex_positions, vertices));
        assert_eq!(polygon.vertices().unwrap(), positions.as_ref());
    }

    #[test]
    fn pairwise_bounds_preserve_exact_extrema_for_odd_even_and_equal_coordinates() {
        for points in [
            vec![
                point(3, -2, 7),
                point(-4, 9, 7),
                point(3, 1, -5),
                point(8, 9, 2),
            ],
            vec![
                point(3, -2, 7),
                point(-4, 9, 7),
                point(3, 1, -5),
                point(8, 9, 2),
                point(0, -2, 4),
            ],
        ] {
            let bounds = bounds_for_owned_points(&points);
            assert_eq!(bounds.min, point(-4, -2, -5));
            assert_eq!(bounds.max, point(8, 9, 7));
        }
    }

    #[test]
    fn exact_axis_aligned_triangle_support_preserves_every_normal_orientation() {
        for (axis, points, expected) in [
            (
                0,
                [point(2, 0, 0), point(2, 1, 0), point(2, 0, 1)],
                Plane::axis_aligned(0, Real::from(2)),
            ),
            (
                1,
                [point(0, 2, 0), point(0, 2, 1), point(1, 2, 0)],
                Plane::axis_aligned(1, Real::from(2)),
            ),
            (
                2,
                [point(0, 0, 2), point(1, 0, 2), point(0, 1, 2)],
                Plane::axis_aligned(2, Real::from(2)),
            ),
        ] {
            assert_eq!(
                exact_axis_aligned_triangle_support(&points[0], &points[1], &points[2], axis, None),
                Some(expected.clone())
            );
            assert_eq!(
                exact_axis_aligned_triangle_support(&points[0], &points[2], &points[1], axis, None),
                Some(expected.inverted())
            );
        }

        let non_axis = [point(0, 0, 0), point(1, 0, 0), point(0, 1, 1)];
        assert_eq!(
            exact_axis_aligned_triangle_support(&non_axis[0], &non_axis[1], &non_axis[2], 0, None),
            None
        );
        let degenerate = [point(0, 0, 0), point(0, 0, 1), point(0, 0, 2)];
        assert_eq!(
            exact_axis_aligned_triangle_support(
                &degenerate[0],
                &degenerate[1],
                &degenerate[2],
                0,
                None,
            ),
            None
        );
    }
}
