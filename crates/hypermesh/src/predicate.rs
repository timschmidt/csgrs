//! Certified scalar and point-plane predicate dispatch.

use std::cmp::Ordering;

use hyperlattice::{HomogeneousPoint3, Point3, Rational, Real, homogeneous_point_plane_expression};
use hyperlimit::{PredicateOutcome, Sign, classify_real_sign};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::Plane;

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
        Self {
            point,
            exact_coordinates,
        }
    }

    /// Classifies the retained point against one plane.
    pub(crate) fn classify(&self, plane: &Plane) -> HypermeshResult<Classification> {
        if let Some(coordinates) = self.exact_coordinates
            && let Some(classification) =
                classify_exact_rational_coordinates(plane, coordinates, Rational::one_ref())
        {
            crate::trace_dispatch!("classify-point", "affine-exact-rational");
            return Ok(classification);
        }

        crate::trace_dispatch!("classify-point", "affine-real-fallback");
        classify_real(&plane.expression_at_point(self.point))
    }
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

fn classify_exact_rational_terms(
    plane: &Plane,
    coordinates: [&Real; 3],
    homogeneous_weight: &Rational,
) -> Option<Classification> {
    let [Some(x), Some(y), Some(z)] = coordinates.map(Real::exact_rational_ref) else {
        return None;
    };
    classify_exact_rational_coordinates(plane, [x, y, z], homogeneous_weight)
}

fn classify_exact_rational_coordinates(
    plane: &Plane,
    [x, y, z]: [&Rational; 3],
    homogeneous_weight: &Rational,
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
