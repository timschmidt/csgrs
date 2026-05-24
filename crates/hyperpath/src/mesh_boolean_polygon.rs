//! Convex polygon source lowering for exact mesh booleans.
//!
//! Rectangular prisms are not enough for PCB copper zones, chamfered pads, and
//! polygonal CAM fixtures. This module admits the next exact subset: retained
//! strictly convex straight-edge polygons extruded over one exact Z interval.
//! No curve flattening or tolerance repair is performed. The design follows
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): the polygon vertices remain the authoritative object, while the
//! triangulated prism is a replayable handoff to `hypermesh`. The solid
//! interpretation remains Requicha-style regularized set geometry.

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

fn polygon_signed_area_twice(vertices: &[Point2]) -> Real {
    let mut area = Real::zero();
    for index in 0..vertices.len() {
        let current = &vertices[index];
        let next = &vertices[(index + 1) % vertices.len()];
        area = area + current.x.clone() * next.y.clone() - next.x.clone() * current.y.clone();
    }
    area
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
