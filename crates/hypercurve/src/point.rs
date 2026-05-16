//! Two-dimensional points backed by [`hyperreal::Real`].

use hyperreal::Real;

/// A two-dimensional point.
#[derive(Clone, Debug, PartialEq)]
pub struct Point2 {
    x: Real,
    y: Real,
}

impl Point2 {
    /// Constructs a point from Real coordinates.
    pub const fn new(x: Real, y: Real) -> Self {
        Self { x, y }
    }

    /// Constructs a point from values convertible into Real coordinates.
    pub fn from_values<X, Y>(x: X, y: Y) -> Self
    where
        X: Into<Real>,
        Y: Into<Real>,
    {
        Self {
            x: x.into(),
            y: y.into(),
        }
    }

    /// Returns the x coordinate.
    pub const fn x(&self) -> &Real {
        &self.x
    }

    /// Returns the y coordinate.
    pub const fn y(&self) -> &Real {
        &self.y
    }

    /// Returns `self - other` as a coordinate pair.
    pub fn delta_from(&self, other: &Self) -> (Real, Real) {
        (&self.x - &other.x, &self.y - &other.y)
    }

    /// Returns squared Euclidean distance to another point.
    pub fn distance_squared(&self, other: &Self) -> Real {
        let (dx, dy) = self.delta_from(other);
        &dx * &dx + &dy * &dy
    }

    /// Linearly interpolates between two points.
    pub fn lerp(&self, other: &Self, t: Real) -> Self {
        let one_minus_t = Real::one() - &t;
        Self {
            x: (&self.x * &one_minus_t) + (&other.x * &t),
            y: (&self.y * &one_minus_t) + (&other.y * &t),
        }
    }

    /// Translates the point by the given Real delta.
    pub fn translated(&self, dx: Real, dy: Real) -> Self {
        Self {
            x: &self.x + dx,
            y: &self.y + dy,
        }
    }

    /// Returns conservative structural facts for this point's coordinates.
    ///
    /// The facts expose exact-rational schedule eligibility and symbolic
    /// dependency families without exposing scalar internals. They are intended
    /// for object-level dispatch in the style described by Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn structural_facts(&self) -> crate::Point2Facts {
        crate::facts::point2_facts(self)
    }
}
