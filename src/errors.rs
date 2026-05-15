//! Shared error types used by geometry validation and file conversion APIs.

use crate::float_types::Real;
use nalgebra::Point3;

/// Coordinate validation failure for a single point.
#[derive(Clone, Debug, thiserror::Error, PartialEq)]
pub enum PointError {
    #[error("point {0:?} has NaN fields")]
    NaN(Point3<Real>),
    #[error("point {0:?} has infinite fields")]
    Infinite(Point3<Real>),
}

/// All the possible validation issues we might encounter,
#[derive(Debug, Clone, thiserror::Error, PartialEq)]
#[non_exhaustive]
pub enum ValidationError {
    /// (RepeatedPoint) Two consecutive coords are identical
    #[error("point {0:?} is repeated consecutively")]
    RepeatedPoint(Point3<Real>),
    /// (HoleOutsideShell) A hole is *not* contained by its outer shell
    #[error("hole is not contained by its outer shell near {0:?}")]
    HoleOutsideShell(Point3<Real>),
    /// (NestedHoles) A hole is nested inside another hole
    #[error("hole is nested inside another hole near {0:?}")]
    NestedHoles(Point3<Real>),
    /// (DisconnectedInterior) The interior is disconnected
    #[error("interior is disconnected near {0:?}")]
    DisconnectedInterior(Point3<Real>),
    /// (SelfIntersection) A polygon self‐intersects
    #[error("polygon self-intersects near {0:?}")]
    SelfIntersection(Point3<Real>),
    /// (RingSelfIntersection) A linear ring has a self‐intersection
    #[error("linear ring self-intersects near {0:?}")]
    RingSelfIntersection(Point3<Real>),
    /// (NestedShells) Two outer shells are nested incorrectly
    #[error("outer shells are nested incorrectly near {0:?}")]
    NestedShells(Point3<Real>),
    /// (TooFewPoints) A ring or line has fewer than the minimal #points
    #[error("ring or line has too few points near {0:?}")]
    TooFewPoints(Point3<Real>),
    /// (InvalidCoordinate) The coordinate has a NaN or infinite
    #[error("invalid coordinate {0:?}")]
    InvalidCoordinate(Point3<Real>),
    /// A more precise invalid-coordinate report.
    #[error(transparent)]
    PointError(#[from] PointError),
    /// (RingNotClosed) The ring's first/last points differ
    #[error("ring is not closed near {0:?}")]
    RingNotClosed(Point3<Real>),
    /// (MismatchedVertices) operation requires polygons with same number of vertices
    #[error("operation requires polygons with the same number of vertices")]
    MismatchedVertices,
    /// Operation requires polygons with the same number of vertices.
    #[error("operation requires polygons with the same number of vertices, {left} != {right}")]
    MismatchedVertexCount { left: usize, right: usize },
    /// (IndexOutOfRange) operation requires polygons with same number of vertices
    #[error("index out of range")]
    IndexOutOfRange,
    /// A required index is outside a collection.
    #[error("index {index} is out of range for length {len}")]
    IndexOutOfRangeWithLen { index: usize, len: usize },
    /// (InvalidArguments) operation requires polygons with same number of vertices
    #[error("invalid arguments")]
    InvalidArguments,
    /// A named integer field is below the supported minimum.
    #[error("{name} must not be less than {min}")]
    FieldLessThan { name: &'static str, min: i32 },
    /// A named real field is below the supported minimum.
    #[error("{name} must not be less than {min}")]
    FieldLessThanFloat { name: &'static str, min: Real },
    /// An inconsistency while building a triangle mesh.
    #[error("triangle mesh builder error: {0}")]
    TriMeshError(String),
    /// In general, anything else
    #[error("{0}")]
    Other(String, Option<Point3<Real>>),
}

// Plane::from_points "Degenerate polygon: vertices do not define a plane"
// Mesh::polyhedron "Face index {} is out of range (points.len = {})."
// Sketch::rotate_extrude "rotate_extrude requires at least 2 segments"
// Sketch::extrude_between "extrude_between: both polygons must have the same number of vertices"
