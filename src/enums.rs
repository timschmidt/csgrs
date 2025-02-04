use nalgebra::Point3;

pub enum Axis {
    X,
    Y,
    Z,
}

/// All the possible validation issues we might encounter,
#[derive(Debug, Clone, PartialEq)]
pub enum ValidationError {
    /// (RepeatedPoint) Two consecutive coords are identical
    RepeatedPoint(Point3<f64>),
    /// (HoleOutsideShell) A hole is *not* contained by its outer shell
    HoleOutsideShell(Point3<f64>),
    /// (NestedHoles) A hole is nested inside another hole
    NestedHoles(Point3<f64>),
    /// (DisconnectedInterior) The interior is disconnected
    DisconnectedInterior(Point3<f64>),
    /// (SelfIntersection) A polygon self‐intersects
    SelfIntersection(Point3<f64>),
    /// (RingSelfIntersection) A linear ring has a self‐intersection
    RingSelfIntersection(Point3<f64>),
    /// (NestedShells) Two outer shells are nested incorrectly
    NestedShells(Point3<f64>),
    /// (TooFewPoints) A ring or line has fewer than the minimal #points
    TooFewPoints(Point3<f64>),
    /// (InvalidCoordinate) The coordinate has a NaN or infinite
    InvalidCoordinate(Point3<f64>),
    /// (RingNotClosed) The ring’s first/last points differ
    RingNotClosed(Point3<f64>),
    /// In general, anything else
    Other(String, Option<Point3<f64>>),
}
