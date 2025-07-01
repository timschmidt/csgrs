use crate::float_types::Real;
use nalgebra::Point3;
use crate::float_types::rapier3d::prelude::TriMeshBuilderError;

/// The coordinate has a NaN or infinite
#[derive(Clone, Debug, thiserror::Error, PartialEq)]
pub enum PointError {
    #[error("Point({}) has NaN fields", .0)]
    NaN(Point3<Real>),
    #[error("Point({}) has Infinite fields", .0)]
    Infinite(Point3<Real>),
}

/// All the possible validation issues we might encounter, with genaric errors and wrappers for suberrors
#[derive(Clone, Debug, thiserror::Error, PartialEq)]
pub enum ValidationError {
    /// A [`PlaneError`](crate::plane::PlaneError)
    #[error(transparent)]
    PlaneError(#[from] crate::mesh::plane::PlaneError),
    /// `name` must not be less then `min`
    #[error("{} must be not be less then {}", .name, .min)]
    FieldLessThen { name: &'static str, min: i32 },
    /// `name` must not be less then `min`
    #[error("{} must be not be less then {}", .name, .min)]
    FieldLessThenFloat { name: &'static str, min: Real },
    /// If a required index is higher then len
    /// `name` must not be less or equal to 0.0
    #[error("{} must be not be >= 0", .name)]
    Zero { name: &'static str },
    #[error("Face index {} is out of range (points.len = {})", .index, .len)]
    IndexOutOfRange { index: usize, len: usize },
    /// A `Polygon` is non-planar or not on it's plane
    #[error("A Polygon is non-planar or not on it's plane")]
    NonPlanar,
    /// Two consecutive coords are identical
    #[error("Point({}) is repeated consecutively", .0)]
    RepeatedPoint(Point3<Real>),
    /// A `Polygon`'s first/last points differ
    #[error("Polygon not closed first({}) and last({}) points differ", .first, .last)]
    NotClosed { first: Point3<Real>, last: Point3<Real> },
    /// A operation requires polygons with same number of vertices
    #[error("A operation requires polygons with same number of vertices, {} != {}", .0, .1)]
    MismatchedVertices(usize, usize),
    /// The coordinate has a NaN or infinite
    #[error(transparent)]
    InvalidCoordinate(#[from] PointError),

    // FIXME: Uncomment when https://github.com/georust/geo/pull/1375 is merged
    // /// An error from spade triangulation
    // #[cfg(feature = "delaunay")]
    // #[error(transparent)]
    // TriangulationError(#[from] geo::triangulate_spade::TriangulationError),
    /// An inconsistency while building a triangle mesh
    #[error(transparent)]
    TriMeshError(#[from] TriMeshBuilderError),
}

// Plane::from_points "Degenerate polygon: vertices do not define a plane"
// Mesh::polyhedron "Face index {} is out of range (points.len = {})."
// Sketch::rotate_extrude "rotate_extrude requires at least 2 segments"
// Sketch::extrude_between "extrude_between: both polygons must have the same number of vertices"
