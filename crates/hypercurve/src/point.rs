//! Two-dimensional points backed by [`hyperreal::Real`].

use hyperreal::{Real, ZeroKnowledge as ZeroStatus};
use std::sync::Arc;

/// A two-dimensional point.
#[derive(Clone, Debug)]
pub struct Point2(Arc<Point2Data>);

#[derive(Debug)]
struct Point2Data {
    x: Real,
    y: Real,
}

impl PartialEq for Point2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
            || ((&self.0.x - &other.0.x).zero_status() == ZeroStatus::Zero
                && (&self.0.y - &other.0.y).zero_status() == ZeroStatus::Zero)
    }
}

impl Point2 {
    /// Constructs a point from Real coordinates.
    pub fn new(x: Real, y: Real) -> Self {
        Self(Arc::new(Point2Data { x, y }))
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
    pub fn x(&self) -> &Real {
        &self.0.x
    }

    /// Returns the y coordinate.
    pub fn y(&self) -> &Real {
        &self.0.y
    }

    pub(crate) fn identity(&self) -> u64 {
        Arc::as_ptr(&self.0) as usize as u64
    }

    /// Returns `self - other` as a coordinate pair.
    pub fn delta_from(&self, other: &Self) -> (Real, Real) {
        (self.x() - other.x(), self.y() - other.y())
    }

    /// Returns squared Euclidean distance to another point.
    pub fn distance_squared(&self, other: &Self) -> Real {
        let (dx, dy) = self.delta_from(other);
        &dx * &dx + &dy * &dy
    }

    /// Linearly interpolates between two points.
    pub fn lerp(&self, other: &Self, t: Real) -> Self {
        let one_minus_t = Real::one() - &t;
        self.lerp_with_weights(other, &one_minus_t, &t)
    }

    /// Interpolates with caller-provided affine weights that are shared by one
    /// de Casteljau level or triangle.
    pub(crate) fn lerp_with_weights(&self, other: &Self, one_minus_t: &Real, t: &Real) -> Self {
        Self::new(
            (self.x() * one_minus_t) + (other.x() * t),
            (self.y() * one_minus_t) + (other.y() * t),
        )
    }

    /// Translates the point by the given Real delta.
    pub fn translated(&self, dx: Real, dy: Real) -> Self {
        Self::new(self.x() + dx, self.y() + dy)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clones_share_exact_coordinate_storage_and_identity() {
        let point = Point2::from_values(3_i8, -5_i8);
        let clone = point.clone();

        assert!(Arc::ptr_eq(&point.0, &clone.0));
        assert_eq!(point.identity(), clone.identity());
        assert_eq!(point.x(), clone.x());
        assert_eq!(point.y(), clone.y());
    }

    #[test]
    fn point_handle_remains_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Point2>();
    }
}
