//! Convex polygon representation backed by hyperreal planes.

use hyperlattice::{HomogeneousPoint3, Point3, Rational, Real, intersect_three_planes};
use std::sync::Arc;

use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::HypermeshResult;
use crate::geometry::{
    Classification, Plane, affine_projective_point_decision, cross_arrays, sub_points,
};
use crate::predicate::{classify_projective_point_decision, compare_real_decision};
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

#[derive(Clone, Debug)]
pub(crate) struct RetainedVertexCycle(Arc<[Point3]>);

impl RetainedVertexCycle {
    pub(crate) fn len(&self) -> usize {
        self.0.len()
    }

    pub(crate) fn get(&self, index: usize) -> Option<&Point3> {
        self.0.get(index)
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
    pub fn is_valid(&self, context: &MeshContext) -> HypermeshResult<MeshOutcome<bool>> {
        let decisions = DecisionContext::new(context);
        let valid = self.vertex_count() >= 3 && self.support.decide_is_valid(&decisions)?;
        Ok(decisions.finish(valid))
    }

    /// Computes vertex `i` as a homogeneous intersection of support and two
    /// adjacent edge planes.
    pub fn vertex(&self, i: usize) -> HomogeneousPoint3 {
        let n = self.vertex_count();
        intersect_three_planes(&self.support, &self.edges[i], &self.edges[(i + 1) % n])
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
        result.support = self.support.inverted();
        result.edges = Arc::new(self.edges.iter().rev().cloned().collect::<Vec<_>>());
        result.known_vertices = self.known_vertices.as_ref().map(|vertices| {
            RetainedVertexCycle(Arc::from(
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

    #[inline]
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
        if classify_projective_point_decision(decisions, point, &self.support)?
            != Classification::On
        {
            return Ok(false);
        }
        for edge in self.edges.iter() {
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
        if classify_projective_point_decision(decisions, point, &self.support)?
            != Classification::On
        {
            return Ok(false);
        }
        for edge in self.edges.iter() {
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
            support,
            edges: Arc::new(edges),
            mesh_index,
            polygon_index,
            delta_w: Vec::new(),
            approx_bounds: Some(Box::new(bounds_for_points(decisions, &[p0, p1, p2])?)),
            known_vertices: Some(RetainedVertexCycle(Arc::new([
                p0.clone(),
                p1.clone(),
                p2.clone(),
            ]))),
            known_identities: None,
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
        support,
        edges: Arc::new(edges),
        mesh_index,
        polygon_index,
        delta_w: Vec::new(),
        approx_bounds: Some(Box::new(bounds_for_points(decisions, &[p0, p1, p2, p3])?)),
        known_vertices: Some(RetainedVertexCycle(Arc::new([
            p0.clone(),
            p1.clone(),
            p2.clone(),
            p3.clone(),
        ]))),
        known_identities: None,
    })
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
                polygon.support,
                Plane::new(
                    Point3::new(Real::zero(), Real::zero(), Real::one()),
                    Real::zero(),
                )
            );
            assert_eq!(
                polygon.edges.as_slice(),
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
