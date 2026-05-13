/// 2D point with scalar coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point2<S> {
    /// X coordinate.
    pub x: S,
    /// Y coordinate.
    pub y: S,
}

impl<S> Point2<S> {
    /// Construct a 2D point from coordinates.
    pub const fn new(x: S, y: S) -> Self {
        Self { x, y }
    }
}

/// 3D point with scalar coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3<S> {
    /// X coordinate.
    pub x: S,
    /// Y coordinate.
    pub y: S,
    /// Z coordinate.
    pub z: S,
}

impl<S> Point3<S> {
    /// Construct a 3D point from coordinates.
    pub const fn new(x: S, y: S, z: S) -> Self {
        Self { x, y, z }
    }
}
