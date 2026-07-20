//! Convex polygon representation backed by hyperreal planes.

use hyperlattice::{HomogeneousPoint3, Point3, Real, intersect_three_planes};
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

#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub(crate) enum ConstructionEdgeIdentity {
    Source {
        mesh: usize,
        endpoints: [usize; 2],
    },
    Split {
        planes: [ConstructionPlaneIdentity; 2],
    },
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
}

impl RetainedVertexCycle {
    pub(crate) fn len(&self) -> usize {
        match self {
            Self::Owned(vertices) => vertices.len(),
            Self::IndexedTriangle { .. } => 3,
        }
    }

    pub(crate) fn get(&self, index: usize) -> Option<&Point3> {
        match self {
            Self::Owned(vertices) => vertices.get(index),
            Self::IndexedTriangle { positions, indices } => positions.get(*indices.get(index)?),
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
    pub approx_bounds: Option<ApproxBounds>,
    /// Exact vertices retained when supplied directly by the input owner.
    ///
    /// Derived clipping and BSP polygons clear this cache when their edge
    /// cycle changes.
    pub(crate) known_vertices: Option<RetainedVertexCycle>,
    pub(crate) known_edge_identities: Option<Arc<[ConstructionEdgeIdentity]>>,
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
            known_edge_identities: None,
        }
    }

    /// Returns the number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.known_vertices
            .as_ref()
            .map_or(self.edges.len(), |vertices| vertices.len())
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
        result.known_edge_identities = self.known_edge_identities.as_ref().map(|identities| {
            let count = identities.len();
            Arc::from(
                (0..count)
                    .map(|index| identities[(count + count - 2 - index) % count].clone())
                    .collect::<Vec<_>>(),
            )
        });
        result
    }

    pub(crate) fn with_known_vertex_cycle_and_edges(
        &self,
        vertices: Vec<Point3>,
        edges: Vec<Plane>,
        edge_identities: Vec<ConstructionEdgeIdentity>,
    ) -> Self {
        debug_assert_eq!(vertices.len(), edges.len());
        debug_assert_eq!(vertices.len(), edge_identities.len());
        let approx_bounds = (!vertices.is_empty()).then(|| {
            let points = vertices.iter().collect::<Vec<_>>();
            bounds_for_points(&points)
        });
        let mut result = self.clone();
        result.edges = Arc::new(edges);
        result.approx_bounds = approx_bounds;
        result.known_vertices = Some(RetainedVertexCycle::Owned(Arc::from(vertices)));
        result.known_edge_identities = Some(Arc::from(edge_identities));
        result
    }

    pub(crate) fn with_source_triangle_edge_identities(
        mut self,
        mesh: usize,
        vertices: [usize; 3],
    ) -> Self {
        let identities: [ConstructionEdgeIdentity; 3] = std::array::from_fn(|index| {
            let mut endpoints = [vertices[index], vertices[(index + 1) % 3]];
            endpoints.sort_unstable();
            ConstructionEdgeIdentity::Source { mesh, endpoints }
        });
        self.known_edge_identities = Some(Arc::new(identities));
        self
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
        approx_bounds: Some(bounds_for_points(&[p0, p1, p2])),
        known_vertices: Some(RetainedVertexCycle::Owned(Arc::new([
            p0.clone(),
            p1.clone(),
            p2.clone(),
        ]))),
        known_edge_identities: None,
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
        // for source edges that actually reach projective clipping. Retain one
        // shared value here and expand it at that narrower boundary.
        edges: Arc::new(vec![support.clone()]),
        support,
        mesh_index,
        polygon_index,
        delta_w: Vec::new(),
        approx_bounds: Some(bounds_for_points(&[p0, p1, p2])),
        known_vertices: Some(RetainedVertexCycle::Owned(Arc::new([
            p0.clone(),
            p1.clone(),
            p2.clone(),
        ]))),
        known_edge_identities: None,
    }
}

pub(crate) fn make_indexed_triangle_with_deferred_edges(
    positions: Arc<[Point3]>,
    indices: [usize; 3],
    mesh_index: isize,
    polygon_index: isize,
) -> ConvexPolygon {
    let [i0, i1, i2] = indices;
    let p0 = &positions[i0];
    let p1 = &positions[i1];
    let p2 = &positions[i2];
    let support = Plane::from_points(p0, p1, p2);
    ConvexPolygon {
        edges: Arc::new(vec![support.clone()]),
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
        known_edge_identities: None,
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
        approx_bounds: Some(bounds_for_points(&[p0, p1, p2, p3])),
        known_vertices: Some(RetainedVertexCycle::Owned(Arc::new([
            p0.clone(),
            p1.clone(),
            p2.clone(),
            p3.clone(),
        ]))),
        known_edge_identities: None,
    }
}

fn edge_plane(a: &Point3, b: &Point3, opposite: &Point3, support: &Plane) -> Plane {
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
    let min = Point3::new(
        min_real(points.iter().map(|point| &point.x)),
        min_real(points.iter().map(|point| &point.y)),
        min_real(points.iter().map(|point| &point.z)),
    );
    let max = Point3::new(
        max_real(points.iter().map(|point| &point.x)),
        max_real(points.iter().map(|point| &point.y)),
        max_real(points.iter().map(|point| &point.z)),
    );
    ApproxBounds::new(min, max)
}

fn min_real<'a>(mut values: impl Iterator<Item = &'a Real>) -> Real {
    let first = values
        .next()
        .expect("bounds need at least one point")
        .clone();
    values.fold(first, |current, value| {
        if matches!(
            crate::geometry::compare_real(value, &current),
            Ok(std::cmp::Ordering::Less)
        ) {
            value.clone()
        } else {
            current
        }
    })
}

fn max_real<'a>(mut values: impl Iterator<Item = &'a Real>) -> Real {
    let first = values
        .next()
        .expect("bounds need at least one point")
        .clone();
    values.fold(first, |current, value| {
        if matches!(
            crate::geometry::compare_real(value, &current),
            Ok(std::cmp::Ordering::Greater)
        ) {
            value.clone()
        } else {
            current
        }
    })
}
