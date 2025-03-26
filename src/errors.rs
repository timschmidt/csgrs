//! Validation errors

use crate::float_types::Real;
use nalgebra::Point3;
use std::fmt::Display;

/// All the possible validation issues we might encounter
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ValidationError {
    /// (RepeatedPoint) Two consecutive coords are identical
    RepeatedPoint(Point3<Real>),
    /// (HoleOutsideShell) A hole is *not* contained by its outer shell
    HoleOutsideShell(Point3<Real>),
    /// (NestedHoles) A hole is nested inside another hole
    NestedHoles(Point3<Real>),
    /// (DisconnectedInterior) The interior is disconnected
    DisconnectedInterior(Point3<Real>),
    /// (SelfIntersection) A polygon self‐intersects
    SelfIntersection(Point3<Real>),
    /// (RingSelfIntersection) A linear ring has a self‐intersection
    RingSelfIntersection(Point3<Real>),
    /// (NestedShells) Two outer shells are nested incorrectly
    NestedShells(Point3<Real>),
    /// (TooFewPoints) A ring or line has fewer than the minimal #points
    TooFewPoints(Point3<Real>),
    /// (InvalidCoordinate) The coordinate has a NaN or infinite
    InvalidCoordinate(Point3<Real>),
    /// (RingNotClosed) The ring’s first/last points differ
    RingNotClosed { first: Point3<Real>, last: Point3<Real> },
    /// In general, anything else
    Other(String, Option<Point3<Real>>),
    /// Indicates an inconsistency while building a triangle mesh
    TriMesh(#[from] crate::float_types::parry3d::shape::TriMeshBuilderError),
}

impl Display for ValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ValidationError::RepeatedPoint(opoint) => write!(f, "(RepeatedPoint) Two consecutive coords are identical at: {}", opoint),
            ValidationError::HoleOutsideShell(opoint) => write!(f, "(HoleOutsideShell) A hole is *not* contained by its outer shell at: {}", opoint),
            ValidationError::NestedHoles(opoint) => write!(f, "(NestedHoles) A hole is nested inside another hole at: {}", opoint),
            ValidationError::DisconnectedInterior(opoint) => write!(f, "(DisconnectedInterior) The interior is disconnected at: {}", opoint),
            ValidationError::SelfIntersection(opoint) => write!(f, "(SelfIntersection) A polygon self-intersects at: {}", opoint),
            ValidationError::RingSelfIntersection(opoint) => write!(f, "(RingSelfIntersection) A linear ring has a self-intersection at: {}", opoint),
            ValidationError::NestedShells(opoint) => write!(f, "(NestedShells) Two outer shells are nested incorrectly at: {}", opoint),
            ValidationError::TooFewPoints(opoint) => write!(f, "(TooFewPoints) A ring or line has fewer than the minimal #points at: {}", opoint),
            ValidationError::InvalidCoordinate(opoint) => write!(f, "(InvalidCoordinate) The coordinate ({}) has a NaN or infinite", opoint),
            ValidationError::RingNotClosed { first, last } => write!(f, "(RingNotClosed) The ring's first({}) and last({}) points differ", first, last),
            ValidationError::Other(str, opoint) => {
                if let Some(opoint) = opoint {
                    write!(f, "{} at: {}", str, opoint)
                } else {
                    write!(f, "{}", str)
                }
            },
            ValidationError::TriMesh(tri_mesh_builder_error) => tri_mesh_builder_error.fmt(f),
        }
    }
}

// Plane::from_points "Degenerate polygon: vertices do not define a plane"
// CSG::polyhedron "Face index {} is out of range (points.len = {})."
// CSG::rotate_extrude "rotate_extrude requires at least 2 segments"
// CSG::extrude_between "extrude_between: both polygons must have the same number of vertices"
