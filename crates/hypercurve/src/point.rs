//! Two-dimensional points backed by [`hyperreal::Real`].

use hyperreal::{Real, ZeroKnowledge as ZeroStatus};
use std::{fmt, sync::Arc};

/// A two-dimensional point.
#[derive(Clone)]
pub struct Point2(Arc<Point2Data>);

struct Point2Data {
    coordinates: [Real; 2],
}

impl fmt::Debug for Point2 {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("Point2")
            .field("x", self.x())
            .field("y", self.y())
            .finish()
    }
}

impl PartialEq for Point2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
            || ((self.x() - other.x()).zero_status() == ZeroStatus::Zero
                && (self.y() - other.y()).zero_status() == ZeroStatus::Zero)
    }
}

impl Point2 {
    /// Constructs a point from Real coordinates.
    pub fn new(x: Real, y: Real) -> Self {
        Self(Arc::new(Point2Data {
            coordinates: [x, y],
        }))
    }

    #[inline]
    pub(crate) fn shares_storage(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
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
    #[inline]
    pub fn x(&self) -> &Real {
        &self.0.coordinates[0]
    }

    /// Returns the y coordinate.
    #[inline]
    pub fn y(&self) -> &Real {
        &self.0.coordinates[1]
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
        if dx.exact_rational_ref().is_some() && dy.exact_rational_ref().is_some() {
            return Real::exact_rational_signed_product_sum_known_exact(
                [true; 2],
                [[&dx, &dx], [&dy, &dy]],
            );
        }
        Real::signed_product_sum([true; 2], [[&dx, &dx], [&dy, &dy]])
    }

    pub(crate) fn cross_product(&self, other: &Self) -> Real {
        Real::diff_of_products(self.x(), other.y(), self.y(), other.x())
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
    use std::mem::size_of;

    #[test]
    fn clones_share_exact_coordinate_storage_and_identity() {
        let point = Point2::from_values(3_i8, -5_i8);
        let clone = point.clone();

        assert!(point.shares_storage(&clone));
        assert_eq!(point.identity(), clone.identity());
        assert_eq!(point.x(), clone.x());
        assert_eq!(point.y(), clone.y());
    }

    #[test]
    fn point_handle_remains_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Point2>();
    }

    #[test]
    fn point_storage_preserves_one_word_handle_layout() {
        assert_eq!(size_of::<Point2>(), size_of::<usize>());
        assert_eq!(size_of::<Point2Data>(), 2 * size_of::<Real>());
    }

    #[test]
    fn cloned_point_outlives_its_original_owner() {
        let clone = {
            let original = Point2::from_values(7_i8, -9_i8);
            original.clone()
        };
        assert_eq!(clone, Point2::from_values(7_i8, -9_i8));
    }

    #[test]
    fn distance_squared_fuses_exact_deltas_and_preserves_symbolic_expression() {
        let exact_start = Point2::new(
            (Real::from(-7) / Real::from(3)).unwrap(),
            (Real::from(5) / Real::from(2)).unwrap(),
        );
        let exact_end = Point2::new(
            (Real::from(11) / Real::from(5)).unwrap(),
            (Real::from(-13) / Real::from(7)).unwrap(),
        );
        let (exact_dx, exact_dy) = exact_start.delta_from(&exact_end);
        assert_eq!(
            exact_start.distance_squared(&exact_end),
            &exact_dx * &exact_dx + &exact_dy * &exact_dy
        );

        let sqrt_two = Real::from(2).sqrt().unwrap();
        let symbolic_start = Point2::new(sqrt_two.clone(), Real::from(3));
        let symbolic_end = Point2::new(Real::from(1), -sqrt_two);
        let (symbolic_dx, symbolic_dy) = symbolic_start.delta_from(&symbolic_end);
        assert_eq!(
            symbolic_start.distance_squared(&symbolic_end),
            &symbolic_dx * &symbolic_dx + &symbolic_dy * &symbolic_dy
        );
    }

    #[test]
    fn cross_product_fuses_exact_coordinates_and_preserves_symbolic_expression() {
        let exact_left = Point2::from_values(-7, 5);
        let exact_right = Point2::from_values(11, -13);
        assert_eq!(
            exact_left.cross_product(&exact_right),
            exact_left.x() * exact_right.y() - exact_left.y() * exact_right.x()
        );

        let sqrt_two = Real::from(2).sqrt().unwrap();
        let symbolic_left = Point2::new(sqrt_two.clone(), Real::from(3));
        let symbolic_right = Point2::new(Real::from(5), -sqrt_two);
        assert_eq!(
            symbolic_left.cross_product(&symbolic_right),
            symbolic_left.x() * symbolic_right.y() - symbolic_left.y() * symbolic_right.x()
        );
    }
}
