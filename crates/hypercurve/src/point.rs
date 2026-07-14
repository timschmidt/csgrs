//! Two-dimensional points backed by [`hyperreal::Real`].

use hyperreal::{Real, ZeroKnowledge as ZeroStatus};
use std::sync::atomic::{AtomicU64, Ordering};

static NEXT_POINT_ID: AtomicU64 = AtomicU64::new(1);

fn fresh_point_id() -> u64 {
    NEXT_POINT_ID.fetch_add(1, Ordering::Relaxed)
}

/// A two-dimensional point.
#[derive(Clone, Debug)]
pub struct Point2 {
    x: Real,
    y: Real,
    identity: u64,
}

impl PartialEq for Point2 {
    fn eq(&self, other: &Self) -> bool {
        self.identity == other.identity
            || ((&self.x - &other.x).zero_status() == ZeroStatus::Zero
                && (&self.y - &other.y).zero_status() == ZeroStatus::Zero)
    }
}

impl Point2 {
    /// Constructs a point from Real coordinates.
    pub fn new(x: Real, y: Real) -> Self {
        Self {
            x,
            y,
            identity: fresh_point_id(),
        }
    }

    /// Constructs a point from values convertible into Real coordinates.
    pub fn from_values<X, Y>(x: X, y: Y) -> Self
    where
        X: Into<Real>,
        Y: Into<Real>,
    {
        Self::new(x.into(), y.into())
    }

    /// Returns the x coordinate.
    pub const fn x(&self) -> &Real {
        &self.x
    }

    /// Returns the y coordinate.
    pub const fn y(&self) -> &Real {
        &self.y
    }

    pub(crate) const fn identity(&self) -> u64 {
        self.identity
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
        Self::new(
            (&self.x * &one_minus_t) + (&other.x * &t),
            (&self.y * &one_minus_t) + (&other.y * &t),
        )
    }

    /// Translates the point by the given Real delta.
    pub fn translated(&self, dx: Real, dy: Real) -> Self {
        Self::new(&self.x + dx, &self.y + dy)
    }

    /// Returns conservative structural facts for this point's coordinates.
    ///
    /// The facts expose exact-rational schedule eligibility and symbolic
    /// dependency families without exposing scalar internals. They are intended
    /// for object-level dispatch in the style described by exact-computation discipline.
    pub fn structural_facts(&self) -> crate::Point2Facts {
        crate::facts::point2_facts(self)
    }
}
