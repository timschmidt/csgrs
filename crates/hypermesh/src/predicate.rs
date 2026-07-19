//! Certified scalar and point-plane predicate dispatch.

use std::cell::RefCell;
use std::cmp::Ordering;

use hyperlattice::{HomogeneousPoint3, Point3, Rational, Real, homogeneous_point_plane_expression};
use hyperlimit::{PredicateOutcome, Sign, classify_real_sign};
use hyperreal::{PreparedRationalLinearForm4Filter, PreparedRationalLinearForm4Query, RealSign};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::Plane;
use crate::storage_hash::StorageHashMap;

const LINEAR_FORM_FILTER_CACHE_CAPACITY: usize = 8_192;

struct CachedLinearForm3Filter {
    _owners: [Rational; 4],
    filter: Option<PreparedRationalLinearForm4Filter>,
}

thread_local! {
    static LINEAR_FORM_FILTERS: RefCell<
        StorageHashMap<[usize; 4], CachedLinearForm3Filter>
    > = RefCell::new(StorageHashMap::default());
}

fn prepared_linear_form3_filter(
    plane: &Plane,
    coefficients: [&Rational; 4],
) -> Option<PreparedRationalLinearForm4Filter> {
    let key = coefficients.map(Rational::storage_identity);
    LINEAR_FORM_FILTERS.with_borrow_mut(|cache| {
        if let Some(cached) = cache.get(&key) {
            return cached.filter;
        }
        if cache.len() >= LINEAR_FORM_FILTER_CACHE_CAPACITY {
            cache.clear();
        }
        let filter = Real::prepare_rational_linear_form4_filter([
            &plane.normal.x,
            &plane.normal.y,
            &plane.normal.z,
            &plane.offset,
        ]);
        cache.insert(
            key,
            CachedLinearForm3Filter {
                _owners: coefficients.map(Clone::clone),
                filter,
            },
        );
        filter
    })
}

/// Certified point-vs-plane classification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Classification {
    /// Point is on the negative side of the plane.
    Negative,
    /// Point lies exactly on the plane.
    On,
    /// Point is on the positive side of the plane.
    Positive,
}

impl Classification {
    /// Returns true when the classification is positive.
    pub const fn is_positive(self) -> bool {
        matches!(self, Self::Positive)
    }

    /// Returns true when the classification is negative.
    pub const fn is_negative(self) -> bool {
        matches!(self, Self::Negative)
    }

    /// Returns true when the point is on the negative side or on the plane.
    pub const fn is_non_positive(self) -> bool {
        !self.is_positive()
    }

    /// Returns true when the point is on the positive side or on the plane.
    pub const fn is_non_negative(self) -> bool {
        !self.is_negative()
    }
}

/// Classifies an affine point against a plane.
pub fn classify_point(point: &Point3, plane: &Plane) -> HypermeshResult<Classification> {
    PreparedPoint3::new(point).classify(plane)
}

/// Borrowed point coordinates prepared for repeated plane predicates.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PreparedPoint3<'a> {
    point: &'a Point3,
    exact_coordinates: Option<[&'a Rational; 3]>,
    rational_filter_query: Option<PreparedRationalLinearForm4Query>,
}

impl<'a> PreparedPoint3<'a> {
    /// Retains exact-rational coordinate facts without cloning scalar storage.
    pub(crate) fn new(point: &'a Point3) -> Self {
        let exact_coordinates = match (
            point.x.exact_rational_ref(),
            point.y.exact_rational_ref(),
            point.z.exact_rational_ref(),
        ) {
            (Some(x), Some(y), Some(z)) => Some([x, y, z]),
            _ => None,
        };
        crate::trace_dispatch!(
            "prepared-point3",
            if exact_coordinates.is_some() {
                "exact-rational"
            } else {
                "general-real"
            }
        );
        let rational_filter_query =
            exact_coordinates.and_then(Real::prepare_rational_affine_point3_query);
        if rational_filter_query.is_some() {
            crate::trace_dispatch!("prepared-point3", "rational-filter-query");
        }
        Self {
            point,
            exact_coordinates,
            rational_filter_query,
        }
    }

    /// Classifies the retained point against one plane.
    pub(crate) fn classify(&self, plane: &Plane) -> HypermeshResult<Classification> {
        if let Some(coordinates) = self.exact_coordinates
            && let Some(classification) = classify_exact_rational_coordinates(
                plane,
                coordinates,
                Rational::one_ref(),
                self.rational_filter_query.as_ref(),
            )
        {
            crate::trace_dispatch!("classify-point", "affine-exact-rational");
            return Ok(classification);
        }

        crate::trace_dispatch!("classify-point", "affine-real-fallback");
        classify_real(&plane.expression_at_point(self.point))
    }
}

pub(crate) fn classify_point_with_prepared_query(
    point: &Point3,
    plane: &Plane,
    rational_filter_query: Option<&PreparedRationalLinearForm4Query>,
) -> HypermeshResult<Classification> {
    let exact_coordinates = match (
        point.x.exact_rational_ref(),
        point.y.exact_rational_ref(),
        point.z.exact_rational_ref(),
    ) {
        (Some(x), Some(y), Some(z)) => Some([x, y, z]),
        _ => None,
    };
    PreparedPoint3 {
        point,
        exact_coordinates,
        rational_filter_query: rational_filter_query.copied(),
    }
    .classify(plane)
}

/// Classifies a homogeneous point against a plane.
pub fn classify_projective_point(
    point: &HomogeneousPoint3,
    plane: &Plane,
) -> HypermeshResult<Classification> {
    if let Some(weight) = point.w.exact_rational_ref()
        && let Some(classification) =
            classify_exact_rational_terms(plane, [&point.x, &point.y, &point.z], weight)
    {
        crate::trace_dispatch!("classify-point", "projective-exact-rational");
        return Ok(classification);
    }
    crate::trace_dispatch!("classify-point", "projective-real-fallback");
    classify_real(&homogeneous_point_plane_expression(point, plane))
}

/// Borrowed homogeneous coordinates prepared for repeated plane predicates.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PreparedProjectivePoint3<'a> {
    point: &'a HomogeneousPoint3,
    exact_coordinates: Option<[&'a Rational; 4]>,
    rational_filter_query: Option<PreparedRationalLinearForm4Query>,
}

impl<'a> PreparedProjectivePoint3<'a> {
    pub(crate) fn new(point: &'a HomogeneousPoint3) -> Self {
        let exact_coordinates = match (
            point.x.exact_rational_ref(),
            point.y.exact_rational_ref(),
            point.z.exact_rational_ref(),
            point.w.exact_rational_ref(),
        ) {
            (Some(x), Some(y), Some(z), Some(w)) => Some([x, y, z, w]),
            _ => None,
        };
        let rational_filter_query =
            exact_coordinates.and_then(Real::prepare_rational_linear_form4_query);
        Self {
            point,
            exact_coordinates,
            rational_filter_query,
        }
    }

    pub(crate) fn classify(&self, plane: &Plane) -> HypermeshResult<Classification> {
        if let Some([x, y, z, weight]) = self.exact_coordinates
            && let Some(classification) = classify_exact_rational_coordinates(
                plane,
                [x, y, z],
                weight,
                self.rational_filter_query.as_ref(),
            )
        {
            crate::trace_dispatch!("classify-point", "projective-exact-rational");
            return Ok(classification);
        }
        crate::trace_dispatch!("classify-point", "projective-real-fallback");
        classify_real(&homogeneous_point_plane_expression(self.point, plane))
    }
}

fn classify_exact_rational_terms(
    plane: &Plane,
    coordinates: [&Real; 3],
    homogeneous_weight: &Rational,
) -> Option<Classification> {
    let [Some(x), Some(y), Some(z)] = coordinates.map(Real::exact_rational_ref) else {
        return None;
    };
    classify_exact_rational_coordinates(plane, [x, y, z], homogeneous_weight, None)
}

fn classify_exact_rational_coordinates(
    plane: &Plane,
    [x, y, z]: [&Rational; 3],
    homogeneous_weight: &Rational,
    prepared_query: Option<&PreparedRationalLinearForm4Query>,
) -> Option<Classification> {
    let [Some(a), Some(b), Some(c), Some(d)] = [
        &plane.normal.x,
        &plane.normal.y,
        &plane.normal.z,
        &plane.offset,
    ]
    .map(Real::exact_rational_ref) else {
        return None;
    };
    let filtered_sign =
        prepared_linear_form3_filter(plane, [a, b, c, d]).and_then(|filter| match prepared_query {
            Some(query) => filter.sign_prepared(query),
            None => filter.sign_rational([x, y, z, homogeneous_weight]),
        });
    if let Some(sign) = filtered_sign {
        crate::trace_dispatch!(
            "classify-point",
            if homogeneous_weight.is_one() {
                "affine-rational-floating-filter"
            } else {
                "projective-rational-floating-filter"
            }
        );
        return Some(match sign {
            RealSign::Negative => Classification::Negative,
            RealSign::Zero => Classification::On,
            RealSign::Positive => Classification::Positive,
        });
    }

    Some(
        match Rational::signed_product_sum_ordering(
            [true; 4],
            [[a, x], [b, y], [c, z], [d, homogeneous_weight]],
        ) {
            Ordering::Less => Classification::Negative,
            Ordering::Equal => Classification::On,
            Ordering::Greater => Classification::Positive,
        },
    )
}

/// Returns a certified ordering for two exact reals.
pub fn compare_real(left: &Real, right: &Real) -> HypermeshResult<Ordering> {
    if let (Some(left), Some(right)) = (left.exact_rational_ref(), right.exact_rational_ref()) {
        crate::trace_dispatch!("compare-real", "exact-rational");
        return Ok(left
            .partial_cmp(right)
            .expect("exact rationals are totally ordered"));
    }
    crate::trace_dispatch!("compare-real", "hyperlimit");
    match hyperlimit::compare_reals(left, right) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { .. } => Err(HypermeshError::UnknownClassification),
    }
}

pub(crate) fn classify_real(value: &Real) -> HypermeshResult<Classification> {
    crate::trace_dispatch!("classify-real", "hyperlimit");
    match classify_real_sign(value) {
        PredicateOutcome::Decided {
            value: Sign::Negative,
            ..
        } => Ok(Classification::Negative),
        PredicateOutcome::Decided {
            value: Sign::Zero, ..
        } => Ok(Classification::On),
        PredicateOutcome::Decided {
            value: Sign::Positive,
            ..
        } => Ok(Classification::Positive),
        PredicateOutcome::Unknown { .. } => Err(HypermeshError::UnknownClassification),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: Real, y: Real, z: Real) -> Point3 {
        Point3::new(x, y, z)
    }

    #[test]
    fn prepared_point_matches_direct_exact_classification() {
        let point = point(Real::from(2), Real::from(3), Real::from(5));
        let prepared = PreparedPoint3::new(&point);
        let planes = [
            Plane::from_coefficients(Real::one(), Real::zero(), Real::zero(), Real::from(-2)),
            Plane::from_coefficients(Real::zero(), Real::one(), Real::zero(), Real::from(-4)),
        ];

        assert_eq!(prepared.classify(&planes[0]).unwrap(), Classification::On);
        assert_eq!(
            prepared.classify(&planes[1]).unwrap(),
            Classification::Negative
        );
        for plane in &planes {
            assert_eq!(prepared.classify(plane), classify_point(&point, plane));
        }
    }

    #[test]
    fn symbolic_coefficients_preserve_general_exact_fallback() {
        let point = point(Real::one(), Real::zero(), Real::zero());
        let plane =
            Plane::from_coefficients(Real::pi(), Real::zero(), Real::zero(), Real::from(-3));

        assert_eq!(
            classify_point(&point, &plane).unwrap(),
            Classification::Positive
        );
        assert_eq!(
            PreparedPoint3::new(&point).classify(&plane).unwrap(),
            Classification::Positive
        );
    }

    #[test]
    fn projective_exact_dispatch_respects_homogeneous_weight() {
        let plane =
            Plane::from_coefficients(Real::one(), Real::zero(), Real::zero(), Real::from(-2));
        let point =
            HomogeneousPoint3::new(Real::from(6), Real::zero(), Real::zero(), Real::from(3));
        assert_eq!(
            classify_projective_point(&point, &plane).unwrap(),
            Classification::On
        );
    }

    #[test]
    fn prepared_projective_point_matches_repeated_plane_classification() {
        let point =
            HomogeneousPoint3::new(Real::from(6), Real::from(3), Real::zero(), Real::from(3));
        let prepared = PreparedProjectivePoint3::new(&point);
        let planes = [
            Plane::from_coefficients(Real::one(), Real::zero(), Real::zero(), Real::from(-2)),
            Plane::from_coefficients(Real::zero(), Real::one(), Real::zero(), Real::from(-2)),
            Plane::from_coefficients(Real::pi(), Real::zero(), Real::zero(), Real::from(-3)),
        ];

        assert_eq!(prepared.classify(&planes[0]).unwrap(), Classification::On);
        assert_eq!(
            prepared.classify(&planes[1]).unwrap(),
            Classification::Negative
        );
        assert_eq!(
            prepared.classify(&planes[2]).unwrap(),
            Classification::Positive
        );
        for plane in &planes {
            assert_eq!(
                prepared.classify(plane),
                classify_projective_point(&point, plane)
            );
        }
    }

    #[test]
    fn exact_real_comparison_matches_rational_ordering() {
        assert_eq!(
            compare_real(&Real::from(-3), &Real::from(2)).unwrap(),
            Ordering::Less,
        );
        assert_eq!(
            compare_real(&Real::from(5), &Real::from(5)).unwrap(),
            Ordering::Equal,
        );
    }
}
