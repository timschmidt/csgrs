//! Polygon source lowering for exact mesh booleans.
//!
//! Rectangular prisms are not enough for PCB copper zones, chamfered pads, and
//! polygonal CAM fixtures. This module admits retained straight-edge polygons
//! extruded over one exact Z interval, first as strictly convex polygons and
//! then as simple orthogonal polygons. No curve flattening or tolerance repair
//! is performed. The design follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): the polygon vertices
//! remain the authoritative object, while the triangulated prism is a
//! replayable handoff to `hypermesh`. The solid interpretation remains
//! Requicha-style regularized set geometry.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hypermesh::exact::ExactMesh;
use hyperreal::{Real, RealExactSetFacts};

use crate::mesh_boolean::PathMeshBooleanError;
use crate::provenance::PathProvenance;

/// Certified winding of a retained convex polygon.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexPolygonWinding {
    /// Vertices wind counter-clockwise in the XY plane.
    CounterClockwise,
    /// Vertices wind clockwise in the XY plane.
    Clockwise,
}

/// Exact prism swept from a strictly convex retained polygon.
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexPolygonPrism {
    vertices: Vec<Point2>,
    winding: ConvexPolygonWinding,
    z_min: Real,
    z_max: Real,
    provenance: PathProvenance,
    exact: RealExactSetFacts,
}

/// Exact prism swept from a simple orthogonal retained polygon.
#[derive(Clone, Debug, PartialEq)]
pub struct OrthogonalPolygonPrism {
    vertices: Vec<Point2>,
    winding: ConvexPolygonWinding,
    z_min: Real,
    z_max: Real,
    provenance: PathProvenance,
    exact: RealExactSetFacts,
    cap_triangles: Vec<[usize; 3]>,
}

impl ConvexPolygonPrism {
    /// Construct a positive-height prism from retained strictly convex vertices.
    pub fn new(
        vertices: Vec<Point2>,
        z_min: Real,
        z_max: Real,
        provenance: PathProvenance,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        if vertices.len() < 3 {
            return Err(PathMeshBooleanError::TooFewPolygonVertices);
        }
        if compare_reals_with_policy(&z_min, &z_max, policy).value() != Some(Ordering::Less) {
            return Err(PathMeshBooleanError::DegenerateHeight);
        }
        let area_twice = polygon_signed_area_twice(&vertices);
        let winding = match compare_reals_with_policy(&area_twice, &Real::zero(), policy).value() {
            Some(Ordering::Greater) => ConvexPolygonWinding::CounterClockwise,
            Some(Ordering::Less) => ConvexPolygonWinding::Clockwise,
            Some(Ordering::Equal) => return Err(PathMeshBooleanError::DegeneratePolygon),
            None => return Err(PathMeshBooleanError::UnknownPolygonOrientation),
        };
        validate_strict_convexity(&vertices, winding, policy)?;
        let mut refs = vertices
            .iter()
            .flat_map(|point| [&point.x, &point.y])
            .collect::<Vec<_>>();
        refs.extend([&z_min, &z_max]);
        let exact = Real::exact_set_facts(refs);
        Ok(Self {
            vertices,
            winding,
            z_min,
            z_max,
            provenance,
            exact,
        })
    }

    /// Return retained polygon vertices in source order.
    pub fn vertices(&self) -> &[Point2] {
        &self.vertices
    }

    /// Return certified source winding.
    pub const fn winding(&self) -> ConvexPolygonWinding {
        self.winding
    }

    /// Return exact minimum Z.
    pub const fn z_min(&self) -> &Real {
        &self.z_min
    }

    /// Return exact maximum Z.
    pub const fn z_max(&self) -> &Real {
        &self.z_max
    }

    /// Return retained source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }

    /// Return exact-set facts for polygon coordinates and Z bounds.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Derive the exact `hypermesh` solid used for boolean certification.
    ///
    /// The triangulation is a fan because strict convexity is already proven.
    /// This is intentionally not a general polygon triangulator: Held's FIST
    /// and other robust polygon triangulators are appropriate once nonconvex
    /// arrangements are retained, but a convex fan keeps this source on the
    /// exact object side of Yap's EGC boundary.
    pub fn to_exact_mesh(&self) -> Result<ExactMesh, PathMeshBooleanError> {
        let mut ring = self.vertices.clone();
        if self.winding == ConvexPolygonWinding::Clockwise {
            ring.reverse();
        }
        let n = ring.len();
        let mut positions = Vec::with_capacity(n * 6);
        for point in &ring {
            positions.extend([point.x.clone(), point.y.clone(), self.z_min.clone()]);
        }
        for point in &ring {
            positions.extend([point.x.clone(), point.y.clone(), self.z_max.clone()]);
        }

        let mut indices = Vec::with_capacity((n - 2) * 6 + n * 6);
        for i in 1..n - 1 {
            indices.extend([0, i + 1, i]);
            indices.extend([n, n + i, n + i + 1]);
        }
        for i in 0..n {
            let j = (i + 1) % n;
            indices.extend([i, j, n + j]);
            indices.extend([i, n + j, n + i]);
        }
        ExactMesh::from_real_triangles(&positions, &indices)
            .map_err(|error| PathMeshBooleanError::MeshConstruction(format!("{error:?}")))
    }
}

impl OrthogonalPolygonPrism {
    /// Construct a positive-height prism from retained simple orthogonal vertices.
    ///
    /// The constructor validates exact area, exact orthogonal edge shape, and a
    /// complete triangulation. It intentionally does not accept holes; holed
    /// copper pours need retained arrangement cells before becoming exact mesh
    /// operands.
    pub fn new(
        vertices: Vec<Point2>,
        z_min: Real,
        z_max: Real,
        provenance: PathProvenance,
        policy: PredicatePolicy,
    ) -> Result<Self, PathMeshBooleanError> {
        if vertices.len() < 3 {
            return Err(PathMeshBooleanError::TooFewPolygonVertices);
        }
        if compare_reals_with_policy(&z_min, &z_max, policy).value() != Some(Ordering::Less) {
            return Err(PathMeshBooleanError::DegenerateHeight);
        }
        let area_twice = polygon_signed_area_twice(&vertices);
        let winding = match compare_reals_with_policy(&area_twice, &Real::zero(), policy).value() {
            Some(Ordering::Greater) => ConvexPolygonWinding::CounterClockwise,
            Some(Ordering::Less) => ConvexPolygonWinding::Clockwise,
            Some(Ordering::Equal) => return Err(PathMeshBooleanError::DegeneratePolygon),
            None => return Err(PathMeshBooleanError::UnknownPolygonOrientation),
        };
        validate_orthogonal_edges(&vertices, policy)?;
        validate_simple_orthogonal_polygon_edges(&vertices, policy)?;
        let cap_triangles = triangulate_simple_polygon(&vertices, winding, policy)?;
        let mut refs = vertices
            .iter()
            .flat_map(|point| [&point.x, &point.y])
            .collect::<Vec<_>>();
        refs.extend([&z_min, &z_max]);
        let exact = Real::exact_set_facts(refs);
        Ok(Self {
            vertices,
            winding,
            z_min,
            z_max,
            provenance,
            exact,
            cap_triangles,
        })
    }

    /// Return retained polygon vertices in source order.
    pub fn vertices(&self) -> &[Point2] {
        &self.vertices
    }

    /// Return certified source winding.
    pub const fn winding(&self) -> ConvexPolygonWinding {
        self.winding
    }

    /// Return exact minimum Z.
    pub const fn z_min(&self) -> &Real {
        &self.z_min
    }

    /// Return exact maximum Z.
    pub const fn z_max(&self) -> &Real {
        &self.z_max
    }

    /// Return retained source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }

    /// Return exact-set facts for polygon coordinates and Z bounds.
    pub const fn exact_facts(&self) -> &RealExactSetFacts {
        &self.exact
    }

    /// Return retained cap triangulation over [`Self::vertices`].
    pub fn cap_triangles(&self) -> &[[usize; 3]] {
        &self.cap_triangles
    }

    /// Derive the exact `hypermesh` solid used for boolean certification.
    ///
    /// Cap triangulation uses the two-ears theorem of Meisters, "Polygons Have
    /// Ears," *The American Mathematical Monthly* 82.6 (1975), with exact
    /// orientation tests. This is deliberately smaller than Held's FIST class
    /// of industrial polygon triangulators: the retained source is simple,
    /// orthogonal, and hole-free, so every accepted triangle can be replayed
    /// from exact vertices without a repair tolerance.
    pub fn to_exact_mesh(&self) -> Result<ExactMesh, PathMeshBooleanError> {
        let n = self.vertices.len();
        let mut positions = Vec::with_capacity(n * 6);
        for point in &self.vertices {
            positions.extend([point.x.clone(), point.y.clone(), self.z_min.clone()]);
        }
        for point in &self.vertices {
            positions.extend([point.x.clone(), point.y.clone(), self.z_max.clone()]);
        }

        let mut indices = Vec::with_capacity(self.cap_triangles.len() * 6 + n * 6);
        for [a, b, c] in &self.cap_triangles {
            indices.extend([*a, *c, *b]);
            indices.extend([n + *a, n + *b, n + *c]);
        }
        for i in 0..n {
            let j = (i + 1) % n;
            match self.winding {
                ConvexPolygonWinding::CounterClockwise => {
                    indices.extend([i, j, n + j]);
                    indices.extend([i, n + j, n + i]);
                }
                ConvexPolygonWinding::Clockwise => {
                    indices.extend([i, n + j, j]);
                    indices.extend([i, n + i, n + j]);
                }
            }
        }
        ExactMesh::from_real_triangles(&positions, &indices)
            .map_err(|error| PathMeshBooleanError::MeshConstruction(format!("{error:?}")))
    }
}

/// Convenience constructor for an exact integer convex-polygon prism.
pub fn convex_polygon_prism_from_i64_vertices(
    vertices: Vec<[i64; 2]>,
    z_min: i64,
    z_max: i64,
    policy: PredicatePolicy,
) -> Result<ConvexPolygonPrism, PathMeshBooleanError> {
    let vertices = vertices
        .into_iter()
        .map(|point| Point2::new(Real::from(point[0]), Real::from(point[1])))
        .collect::<Vec<_>>();
    ConvexPolygonPrism::new(
        vertices,
        Real::from(z_min),
        Real::from(z_max),
        PathProvenance::native(),
        policy,
    )
}

/// Convenience constructor for an exact integer orthogonal-polygon prism.
pub fn orthogonal_polygon_prism_from_i64_vertices(
    vertices: Vec<[i64; 2]>,
    z_min: i64,
    z_max: i64,
    policy: PredicatePolicy,
) -> Result<OrthogonalPolygonPrism, PathMeshBooleanError> {
    let vertices = vertices
        .into_iter()
        .map(|point| Point2::new(Real::from(point[0]), Real::from(point[1])))
        .collect::<Vec<_>>();
    OrthogonalPolygonPrism::new(
        vertices,
        Real::from(z_min),
        Real::from(z_max),
        PathProvenance::native(),
        policy,
    )
}

fn polygon_signed_area_twice(vertices: &[Point2]) -> Real {
    let mut area = Real::zero();
    for index in 0..vertices.len() {
        let current = &vertices[index];
        let next = &vertices[(index + 1) % vertices.len()];
        area = area + current.x.clone() * next.y.clone() - next.x.clone() * current.y.clone();
    }
    area
}

fn validate_orthogonal_edges(
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        let same_x = compare_reals_with_policy(&start.x, &end.x, policy)
            .value()
            .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
            == Ordering::Equal;
        let same_y = compare_reals_with_policy(&start.y, &end.y, policy)
            .value()
            .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
            == Ordering::Equal;
        match (same_x, same_y) {
            (true, true) => return Err(PathMeshBooleanError::DegeneratePolygon),
            (true, false) | (false, true) => {}
            (false, false) => return Err(PathMeshBooleanError::NonConvexPolygon),
        }
    }
    Ok(())
}

fn validate_simple_orthogonal_polygon_edges(
    vertices: &[Point2],
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    for left in 0..vertices.len() {
        let left_next = (left + 1) % vertices.len();
        for right in left + 1..vertices.len() {
            let right_next = (right + 1) % vertices.len();
            if left == right_next || left_next == right {
                continue;
            }
            if orthogonal_segments_intersect(
                &vertices[left],
                &vertices[left_next],
                &vertices[right],
                &vertices[right_next],
                policy,
            )? {
                return Err(PathMeshBooleanError::PolygonTriangulationFailed);
            }
        }
    }
    Ok(())
}

fn orthogonal_segments_intersect(
    a0: &Point2,
    a1: &Point2,
    b0: &Point2,
    b1: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let a_vertical = compare_reals_with_policy(&a0.x, &a1.x, policy)
        .value()
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
        == Ordering::Equal;
    let b_vertical = compare_reals_with_policy(&b0.x, &b1.x, policy)
        .value()
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)?
        == Ordering::Equal;
    match (a_vertical, b_vertical) {
        (true, true) => Ok(real_equal(&a0.x, &b0.x, policy)?
            && ranges_overlap(&a0.y, &a1.y, &b0.y, &b1.y, policy)?),
        (false, false) => Ok(real_equal(&a0.y, &b0.y, policy)?
            && ranges_overlap(&a0.x, &a1.x, &b0.x, &b1.x, policy)?),
        (true, false) => Ok(real_between_closed(&a0.x, &b0.x, &b1.x, policy)?
            && real_between_closed(&b0.y, &a0.y, &a1.y, policy)?),
        (false, true) => Ok(real_between_closed(&b0.x, &a0.x, &a1.x, policy)?
            && real_between_closed(&a0.y, &b0.y, &b1.y, policy)?),
    }
}

fn ranges_overlap(
    a0: &Real,
    a1: &Real,
    b0: &Real,
    b1: &Real,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let a_min = real_min(a0, a1, policy)?;
    let a_max = real_max(a0, a1, policy)?;
    let b_min = real_min(b0, b1, policy)?;
    let b_max = real_max(b0, b1, policy)?;
    Ok(
        compare_reals_with_policy(a_min, b_max, policy).value() != Some(Ordering::Greater)
            && compare_reals_with_policy(b_min, a_max, policy).value() != Some(Ordering::Greater),
    )
}

fn real_between_closed(
    value: &Real,
    a: &Real,
    b: &Real,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let min = real_min(a, b, policy)?;
    let max = real_max(a, b, policy)?;
    Ok(
        compare_reals_with_policy(min, value, policy).value() != Some(Ordering::Greater)
            && compare_reals_with_policy(value, max, policy).value() != Some(Ordering::Greater),
    )
}

fn real_min<'a>(
    a: &'a Real,
    b: &'a Real,
    policy: PredicatePolicy,
) -> Result<&'a Real, PathMeshBooleanError> {
    match compare_reals_with_policy(a, b, policy).value() {
        Some(Ordering::Less | Ordering::Equal) => Ok(a),
        Some(Ordering::Greater) => Ok(b),
        None => Err(PathMeshBooleanError::UnknownPolygonOrientation),
    }
}

fn real_max<'a>(
    a: &'a Real,
    b: &'a Real,
    policy: PredicatePolicy,
) -> Result<&'a Real, PathMeshBooleanError> {
    match compare_reals_with_policy(a, b, policy).value() {
        Some(Ordering::Greater | Ordering::Equal) => Ok(a),
        Some(Ordering::Less) => Ok(b),
        None => Err(PathMeshBooleanError::UnknownPolygonOrientation),
    }
}

fn real_equal(a: &Real, b: &Real, policy: PredicatePolicy) -> Result<bool, PathMeshBooleanError> {
    compare_reals_with_policy(a, b, policy)
        .value()
        .map(|ordering| ordering == Ordering::Equal)
        .ok_or(PathMeshBooleanError::UnknownPolygonOrientation)
}

fn triangulate_simple_polygon(
    vertices: &[Point2],
    winding: ConvexPolygonWinding,
    policy: PredicatePolicy,
) -> Result<Vec<[usize; 3]>, PathMeshBooleanError> {
    let mut remaining = match winding {
        ConvexPolygonWinding::CounterClockwise => (0..vertices.len()).collect::<Vec<_>>(),
        ConvexPolygonWinding::Clockwise => (0..vertices.len()).rev().collect::<Vec<_>>(),
    };
    let mut triangles = Vec::with_capacity(vertices.len() - 2);
    while remaining.len() > 3 {
        let mut ear = None;
        for index in 0..remaining.len() {
            let previous = remaining[(index + remaining.len() - 1) % remaining.len()];
            let current = remaining[index];
            let next = remaining[(index + 1) % remaining.len()];
            if is_ear(vertices, &remaining, previous, current, next, policy)? {
                ear = Some((index, [previous, current, next]));
                break;
            }
        }
        let Some((index, triangle)) = ear else {
            return Err(PathMeshBooleanError::PolygonTriangulationFailed);
        };
        triangles.push(triangle);
        remaining.remove(index);
    }
    triangles.push([remaining[0], remaining[1], remaining[2]]);
    Ok(triangles)
}

fn is_ear(
    vertices: &[Point2],
    remaining: &[usize],
    a: usize,
    b: usize,
    c: usize,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    let cross = orient2(&vertices[a], &vertices[b], &vertices[c]);
    match compare_reals_with_policy(&cross, &Real::zero(), policy).value() {
        Some(Ordering::Greater) => {}
        Some(Ordering::Equal | Ordering::Less) => return Ok(false),
        None => return Err(PathMeshBooleanError::UnknownPolygonOrientation),
    }
    for &candidate in remaining {
        if candidate == a || candidate == b || candidate == c {
            continue;
        }
        if point_in_closed_ccw_triangle(
            &vertices[candidate],
            &vertices[a],
            &vertices[b],
            &vertices[c],
            policy,
        )? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn point_in_closed_ccw_triangle(
    point: &Point2,
    a: &Point2,
    b: &Point2,
    c: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, PathMeshBooleanError> {
    for edge in [(a, b), (b, c), (c, a)] {
        match compare_reals_with_policy(&orient2(edge.0, edge.1, point), &Real::zero(), policy)
            .value()
        {
            Some(Ordering::Greater | Ordering::Equal) => {}
            Some(Ordering::Less) => return Ok(false),
            None => return Err(PathMeshBooleanError::UnknownPolygonOrientation),
        }
    }
    Ok(true)
}

fn orient2(a: &Point2, b: &Point2, c: &Point2) -> Real {
    (b.x.clone() - a.x.clone()) * (c.y.clone() - a.y.clone())
        - (b.y.clone() - a.y.clone()) * (c.x.clone() - a.x.clone())
}

fn validate_strict_convexity(
    vertices: &[Point2],
    winding: ConvexPolygonWinding,
    policy: PredicatePolicy,
) -> Result<(), PathMeshBooleanError> {
    let expected = match winding {
        ConvexPolygonWinding::CounterClockwise => Ordering::Greater,
        ConvexPolygonWinding::Clockwise => Ordering::Less,
    };
    for index in 0..vertices.len() {
        let previous = &vertices[index];
        let current = &vertices[(index + 1) % vertices.len()];
        let next = &vertices[(index + 2) % vertices.len()];
        let cross = (current.x.clone() - previous.x.clone()) * (next.y.clone() - current.y.clone())
            - (current.y.clone() - previous.y.clone()) * (next.x.clone() - current.x.clone());
        match compare_reals_with_policy(&cross, &Real::zero(), policy).value() {
            Some(Ordering::Equal) => return Err(PathMeshBooleanError::DegeneratePolygon),
            Some(ordering) if ordering == expected => {}
            Some(_) => return Err(PathMeshBooleanError::NonConvexPolygon),
            None => return Err(PathMeshBooleanError::UnknownPolygonOrientation),
        }
    }
    Ok(())
}
