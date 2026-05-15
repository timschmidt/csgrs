//! Two-dimensional points backed by `hyperlattice::Scalar`.

use hyperlattice::{Backend, DefaultBackend, Scalar};

/// A two-dimensional point.
#[derive(Clone, Debug, PartialEq)]
pub struct Point2<B: Backend = DefaultBackend> {
    x: Scalar<B>,
    y: Scalar<B>,
}

impl<B: Backend> Point2<B> {
    /// Constructs a point from scalar coordinates.
    pub const fn new(x: Scalar<B>, y: Scalar<B>) -> Self {
        Self { x, y }
    }

    /// Constructs a point from values convertible into scalars.
    pub fn from_values<X, Y>(x: X, y: Y) -> Self
    where
        X: Into<Scalar<B>>,
        Y: Into<Scalar<B>>,
    {
        Self {
            x: x.into(),
            y: y.into(),
        }
    }

    /// Returns the x coordinate.
    pub const fn x(&self) -> &Scalar<B> {
        &self.x
    }

    /// Returns the y coordinate.
    pub const fn y(&self) -> &Scalar<B> {
        &self.y
    }

    /// Returns `self - other` as a coordinate pair.
    pub fn delta_from(&self, other: &Self) -> (Scalar<B>, Scalar<B>) {
        (&self.x - &other.x, &self.y - &other.y)
    }

    /// Returns squared Euclidean distance to another point.
    pub fn distance_squared(&self, other: &Self) -> Scalar<B> {
        let (dx, dy) = self.delta_from(other);
        &dx * &dx + &dy * &dy
    }

    /// Linearly interpolates between two points.
    pub fn lerp(&self, other: &Self, t: Scalar<B>) -> Self {
        let one_minus_t = Scalar::<B>::one() - &t;
        Self {
            x: (&self.x * &one_minus_t) + (&other.x * &t),
            y: (&self.y * &one_minus_t) + (&other.y * &t),
        }
    }

    /// Translates the point by the given scalar delta.
    pub fn translated(&self, dx: Scalar<B>, dy: Scalar<B>) -> Self {
        Self {
            x: &self.x + dx,
            y: &self.y + dy,
        }
    }
}
