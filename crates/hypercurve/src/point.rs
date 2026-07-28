//! Two-dimensional points backed by [`hyperreal::Real`].

#[cfg(test)]
use hyperreal::ExactDyadicLine2;
use hyperreal::{
    ExactDyadicLinePoint2, ExactDyadicWideLinePoint2, Real, ZeroKnowledge as ZeroStatus,
};
use std::{
    fmt,
    ptr::NonNull,
    sync::{Arc, OnceLock},
};

/// A two-dimensional point.
pub struct Point2(NonNull<()>);

struct Point2Data {
    coordinates: [Real; 2],
}

struct DeferredPoint2<P> {
    point: P,
    coordinates: OnceLock<Box<[Real; 2]>>,
}

const POINT_TAG_MASK: usize = 0b11;
const MATERIALIZED_POINT_TAG: usize = 0;
const EXACT_DYADIC_POINT_TAG: usize = 1;
const EXACT_DYADIC_WIDE_POINT_TAG: usize = 2;
const _: () = {
    assert!(std::mem::align_of::<Point2Data>() > POINT_TAG_MASK);
    assert!(std::mem::align_of::<DeferredPoint2<ExactDyadicLinePoint2>>() > POINT_TAG_MASK);
    assert!(std::mem::align_of::<DeferredPoint2<ExactDyadicWideLinePoint2>>() > POINT_TAG_MASK);
};

// SAFETY: every tagged pointer owns one strong `Arc` reference to one of the
// three Send + Sync payloads below. Clone and Drop dispatch on the immutable
// tag and use the corresponding `Arc` raw-pointer operation.
unsafe impl Send for Point2 {}
// SAFETY: coordinate access is immutable, and deferred initialization uses
// `OnceLock`; every possible payload is Sync.
unsafe impl Sync for Point2 {}

impl Clone for Point2 {
    #[inline]
    fn clone(&self) -> Self {
        if self.tag() == MATERIALIZED_POINT_TAG {
            // SAFETY: the materialized tag is assigned only from
            // Arc<Point2Data>.
            unsafe {
                Arc::increment_strong_count(self.materialized_pointer::<Point2Data>());
            }
        } else {
            self.increment_deferred_strong_count();
        }
        Self(self.0)
    }
}

impl Drop for Point2 {
    #[inline]
    fn drop(&mut self) {
        if self.tag() == MATERIALIZED_POINT_TAG {
            // SAFETY: this Point2 owns one strong Arc<Point2Data> reference.
            unsafe {
                Arc::decrement_strong_count(self.materialized_pointer::<Point2Data>());
            }
        } else {
            self.decrement_deferred_strong_count();
        }
    }
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
        self.0 == other.0
            || ((self.x() - other.x()).zero_status() == ZeroStatus::Zero
                && (self.y() - other.y()).zero_status() == ZeroStatus::Zero)
    }
}

impl Point2 {
    /// Constructs a point from Real coordinates.
    pub fn new(x: Real, y: Real) -> Self {
        Self::from_arc(
            Arc::new(Point2Data {
                coordinates: [x, y],
            }),
            MATERIALIZED_POINT_TAG,
        )
    }

    pub(crate) fn from_exact_dyadic_line_point(point: ExactDyadicLinePoint2) -> Self {
        Self::from_arc(
            Arc::new(DeferredPoint2 {
                point,
                coordinates: OnceLock::new(),
            }),
            EXACT_DYADIC_POINT_TAG,
        )
    }

    pub(crate) fn from_exact_dyadic_wide_line_point(point: ExactDyadicWideLinePoint2) -> Self {
        Self::from_arc(
            Arc::new(DeferredPoint2 {
                point,
                coordinates: OnceLock::new(),
            }),
            EXACT_DYADIC_WIDE_POINT_TAG,
        )
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
        &self.coordinates()[0]
    }

    /// Returns the y coordinate.
    #[inline]
    pub fn y(&self) -> &Real {
        &self.coordinates()[1]
    }

    pub(crate) fn identity(&self) -> u64 {
        self.pointer::<()>().addr() as u64
    }

    fn from_arc<T>(point: Arc<T>, tag: usize) -> Self {
        debug_assert!(tag <= POINT_TAG_MASK);
        let pointer = Arc::into_raw(point).cast_mut().cast::<()>();
        debug_assert_eq!(pointer.addr() & POINT_TAG_MASK, 0);
        let tagged = pointer.map_addr(|address| address | tag);
        // SAFETY: Arc never returns a null data pointer.
        Self(unsafe { NonNull::new_unchecked(tagged) })
    }

    #[inline(always)]
    fn tag(&self) -> usize {
        self.0.as_ptr().addr() & POINT_TAG_MASK
    }

    #[inline(always)]
    fn pointer<T>(&self) -> *const T {
        self.0
            .as_ptr()
            .map_addr(|address| address & !POINT_TAG_MASK)
            .cast()
    }

    #[inline(always)]
    fn materialized_pointer<T>(&self) -> *const T {
        debug_assert_eq!(self.tag(), MATERIALIZED_POINT_TAG);
        self.0.as_ptr().cast()
    }

    #[inline(always)]
    fn coordinates(&self) -> &[Real; 2] {
        if self.tag() == MATERIALIZED_POINT_TAG {
            // SAFETY: the tag is assigned only from Arc<Point2Data>.
            &unsafe { &*self.materialized_pointer::<Point2Data>() }.coordinates
        } else {
            self.deferred_coordinates()
        }
    }

    #[cold]
    #[inline(never)]
    fn deferred_coordinates(&self) -> &[Real; 2] {
        match self.tag() {
            EXACT_DYADIC_POINT_TAG => {
                // SAFETY: the tag is assigned only from this deferred Arc type.
                let deferred = unsafe { &*self.pointer::<DeferredPoint2<ExactDyadicLinePoint2>>() };
                deferred
                    .coordinates
                    .get_or_init(|| Box::new(deferred.point.materialize()))
            }
            EXACT_DYADIC_WIDE_POINT_TAG => {
                // SAFETY: the tag is assigned only from this deferred Arc type.
                let deferred =
                    unsafe { &*self.pointer::<DeferredPoint2<ExactDyadicWideLinePoint2>>() };
                deferred
                    .coordinates
                    .get_or_init(|| Box::new(deferred.point.materialize()))
            }
            _ => unreachable!("two-bit point tag has a reserved value"),
        }
    }

    #[inline(always)]
    fn increment_deferred_strong_count(&self) {
        // SAFETY: constructors preserve the tag/type correspondence for the
        // full lifetime of the raw Arc pointer.
        unsafe {
            match self.tag() {
                EXACT_DYADIC_POINT_TAG => {
                    Arc::increment_strong_count(
                        self.pointer::<DeferredPoint2<ExactDyadicLinePoint2>>(),
                    );
                }
                EXACT_DYADIC_WIDE_POINT_TAG => {
                    Arc::increment_strong_count(
                        self.pointer::<DeferredPoint2<ExactDyadicWideLinePoint2>>(),
                    );
                }
                _ => unreachable!("deferred point has an exact carrier tag"),
            }
        }
    }

    #[inline(always)]
    fn decrement_deferred_strong_count(&self) {
        // SAFETY: this Point2 owns one strong reference of the type selected
        // by its immutable deferred tag.
        unsafe {
            match self.tag() {
                EXACT_DYADIC_POINT_TAG => {
                    Arc::decrement_strong_count(
                        self.pointer::<DeferredPoint2<ExactDyadicLinePoint2>>(),
                    );
                }
                EXACT_DYADIC_WIDE_POINT_TAG => {
                    Arc::decrement_strong_count(
                        self.pointer::<DeferredPoint2<ExactDyadicWideLinePoint2>>(),
                    );
                }
                _ => unreachable!("deferred point has an exact carrier tag"),
            }
        }
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

        assert_eq!(point.0, clone.0);
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
    fn exact_line_coordinates_materialize_on_first_observation() {
        let line = ExactDyadicLine2::from_f64([0.0, 0.0], [2.0, 2.0]).unwrap();
        let (_, retained) = line
            .retained_intersection_point_f64([0.0, 2.0], [2.0, 0.0])
            .unwrap();
        let point = Point2::from_exact_dyadic_line_point(retained);
        // SAFETY: the constructor above assigns this exact tag/type pair.
        let deferred = unsafe { &*point.pointer::<DeferredPoint2<ExactDyadicLinePoint2>>() };
        assert_eq!(point.tag(), EXACT_DYADIC_POINT_TAG);
        assert!(deferred.coordinates.get().is_none());

        let clone = point.clone();
        assert_eq!(point.x(), &Real::one());
        assert!(deferred.coordinates.get().is_some());
        assert_eq!(clone.y(), &Real::one());
        assert_eq!(point, Point2::from_values(1_i8, 1_i8));
    }

    #[test]
    fn tagged_storage_preserves_materialized_point_layout() {
        assert_eq!(size_of::<Point2>(), size_of::<usize>());
        assert_eq!(size_of::<Point2Data>(), 2 * size_of::<Real>());
        assert!(size_of::<DeferredPoint2<ExactDyadicLinePoint2>>() <= 176);
        assert!(size_of::<DeferredPoint2<ExactDyadicWideLinePoint2>>() <= 192);
    }

    #[test]
    fn tagged_clones_outlive_their_original_owner() {
        let materialized = {
            let original = Point2::from_values(7_i8, -9_i8);
            original.clone()
        };
        assert_eq!(materialized, Point2::from_values(7_i8, -9_i8));

        let line = ExactDyadicLine2::from_f64([0.0, 0.0], [2.0, 2.0]).unwrap();
        let deferred = {
            let (_, retained) = line
                .retained_intersection_point_f64([0.0, 2.0], [2.0, 0.0])
                .unwrap();
            Point2::from_exact_dyadic_line_point(retained).clone()
        };
        assert_eq!(deferred, Point2::from_values(1_i8, 1_i8));
    }

    #[test]
    fn wide_deferred_point_materializes_and_drops() {
        let extent = 2_f64.powi(100);
        let near_extent = f64::from_bits(extent.to_bits() - 1);
        let line = ExactDyadicLine2::from_f64([0.0, 0.0], [extent, near_extent]).unwrap();
        let (_, retained) = line
            .wide_retained_intersection_point_f64([0.0, near_extent], [extent, 0.0])
            .unwrap();
        let point = Point2::from_exact_dyadic_wide_line_point(retained);

        assert_eq!(point.tag(), EXACT_DYADIC_WIDE_POINT_TAG);
        assert_eq!(point.x(), &Real::try_from(extent / 2.0).unwrap());
        assert_eq!(point.y(), &Real::try_from(near_extent / 2.0).unwrap());
    }

    #[test]
    fn deferred_point_can_materialize_from_another_thread() {
        let line = ExactDyadicLine2::from_f64([0.0, 0.0], [2.0, 2.0]).unwrap();
        let (_, retained) = line
            .retained_intersection_point_f64([0.0, 2.0], [2.0, 0.0])
            .unwrap();
        let point = Point2::from_exact_dyadic_line_point(retained);
        let clone = point.clone();
        std::thread::spawn(move || {
            assert_eq!(clone.x(), &Real::one());
            assert_eq!(clone.y(), &Real::one());
        })
        .join()
        .unwrap();
        assert_eq!(point, Point2::from_values(1_i8, 1_i8));
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
