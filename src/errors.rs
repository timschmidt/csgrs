use crate::float_types::Real;
use nalgebra::Point3;

/// All the possible validation issues we might encounter,
#[derive(Debug, Clone, PartialEq)]
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
    RingNotClosed(Point3<Real>),
    /// In general, anything else
    Other(String, Option<Point3<Real>>),
}

// Plane::from_points "Degenerate polygon: vertices do not define a plane"
// CSG::polyhedron "Face index {} is out of range (points.len = {})."
// CSG::rotate_extrude "rotate_extrude requires at least 2 segments"
// CSG::extrude_between "extrude_between: both polygons must have the same number of vertices"
