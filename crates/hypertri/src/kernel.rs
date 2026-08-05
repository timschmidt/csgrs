//! Numeric kernels used by triangulation algorithms.
//!
//! All irreversible topology decisions in exact code should pass through this
//! layer. Cheap structural facts and filters are used only when they certify a
//! sign; otherwise the exact/refinement path must decide or report uncertainty.

use crate::context::{TriangulationCertainty, TriangulationContext, TriangulationOutcome};
#[cfg(any(feature = "earcut", feature = "cdt", feature = "nd"))]
use crate::error::{Error, Result};
#[cfg(any(feature = "earcut", feature = "cdt"))]
use crate::types::{Point2, Real, Sign, TriangleLocation};
#[cfg(any(feature = "earcut", feature = "cdt", feature = "nd"))]
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy};
use std::cell::Cell;
#[cfg(any(feature = "earcut", feature = "cdt"))]
use std::cmp::Ordering;

/// Operation-local exact kernel backed by [`hyperreal::Real`].
///
/// A fresh kernel is created for every public operation. Its interior cell
/// aggregates whether any Hyperlimit decision consumed APPROXIMATE_512 without
/// making the caller's immutable [`TriangulationContext`] stateful.
#[derive(Debug)]
pub(crate) struct ExactKernel {
    #[cfg(any(feature = "earcut", feature = "cdt", feature = "nd"))]
    policy: PredicatePolicy,
    certainty: Cell<TriangulationCertainty>,
}

impl ExactKernel {
    pub(crate) fn new(context: &TriangulationContext) -> Self {
        #[cfg(not(any(feature = "earcut", feature = "cdt", feature = "nd")))]
        let _ = context;
        Self {
            #[cfg(any(feature = "earcut", feature = "cdt", feature = "nd"))]
            policy: context.predicate_policy(),
            certainty: Cell::new(TriangulationCertainty::Certified),
        }
    }

    #[cfg(any(feature = "earcut", feature = "cdt", feature = "nd"))]
    pub(crate) const fn policy(&self) -> PredicatePolicy {
        self.policy
    }

    pub(crate) fn finish<T>(&self, value: T) -> TriangulationOutcome<T> {
        TriangulationOutcome::new(value, self.certainty.get())
    }

    #[cfg(any(feature = "earcut", feature = "cdt", feature = "nd"))]
    pub(crate) fn decide<T>(
        &self,
        outcome: PredicateOutcome<T>,
        predicate: &'static str,
    ) -> Result<T> {
        match outcome {
            PredicateOutcome::Decided {
                value, certainty, ..
            } => {
                if certainty == Certainty::Approximate {
                    self.certainty
                        .set(TriangulationCertainty::Approximate512Consumed);
                }
                Ok(value)
            }
            PredicateOutcome::Unknown { .. } => Err(Error::PredicateUndecided { predicate }),
        }
    }

    #[cfg(any(feature = "earcut", feature = "cdt"))]
    pub(crate) fn from_i64(value: i64) -> Real {
        Real::from(value)
    }

    #[cfg(any(feature = "earcut", feature = "cdt"))]
    pub(crate) fn add(left: &Real, right: &Real) -> Real {
        left + right
    }

    #[cfg(any(feature = "earcut", feature = "cdt"))]
    pub(crate) fn sub(left: &Real, right: &Real) -> Real {
        left - right
    }

    #[cfg(feature = "cdt")]
    pub(crate) fn mul(left: &Real, right: &Real) -> Real {
        left * right
    }

    #[cfg(any(feature = "earcut", feature = "cdt"))]
    pub(crate) fn div(left: &Real, right: &Real) -> Result<Real> {
        (left / right).map_err(|_| Error::InvalidInput {
            reason: "Real division failed",
        })
    }

    #[cfg(feature = "earcut")]
    pub(crate) fn midpoint(left: &Point2, right: &Point2) -> Result<Point2> {
        let two = Self::from_i64(2);
        Ok(Point2::new(
            Self::div(&Self::add(&left.x, &right.x), &two)?,
            Self::div(&Self::add(&left.y, &right.y), &two)?,
        ))
    }

    #[cfg(any(feature = "earcut", feature = "cdt"))]
    pub(crate) fn cmp(&self, left: &Real, right: &Real) -> Result<Ordering> {
        self.decide(
            hyperlimit::compare_reals(left, right, self.policy),
            "exact Real ordering",
        )
    }

    #[cfg(any(feature = "earcut", feature = "cdt"))]
    pub(crate) fn orient2(&self, a: &Point2, b: &Point2, c: &Point2) -> Result<Sign> {
        // Triangulation topology consumes `hyperlimit`'s certified predicate
        // pipeline rather than rebuilding a private determinant expression.
        // Keeping the determinant owner in the predicate crate separates
        // application topology from arithmetic policy. The predicate
        // implementation keeps robust-orientation logic and exact
        // rational/common-scale schedules near their use.
        self.decide(hyperlimit::orient2(a, b, c, self.policy), "orient2")
            .map(map_hyperlimit_sign)
    }

    #[cfg(feature = "cdt")]
    pub(crate) fn incircle2(&self, a: &Point2, b: &Point2, c: &Point2, d: &Point2) -> Result<Sign> {
        // In-circle legality is the CDT edge-flip predicate. Route it through
        // `hyperlimit` so exact lifted-determinant certificates remain
        // centralized. Hypertri consumes only the certified empty-circle
        // result.
        self.decide(hyperlimit::incircle2(a, b, c, d, self.policy), "incircle2")
            .map(map_hyperlimit_sign)
    }

    #[cfg(any(feature = "earcut", feature = "cdt"))]
    pub(crate) fn classify_point_triangle(
        &self,
        a: &Point2,
        b: &Point2,
        c: &Point2,
        point: &Point2,
    ) -> Result<TriangleLocation> {
        classify_point_triangle(self, a, b, c, point)
    }
}

#[cfg(any(feature = "earcut", feature = "cdt"))]
fn classify_point_triangle(
    kernel: &ExactKernel,
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
) -> Result<TriangleLocation> {
    let orientation = kernel.orient2(a, b, c)?;
    if orientation == Sign::Zero {
        return Ok(TriangleLocation::Degenerate);
    }

    if points_equal(kernel, point, a)?
        || points_equal(kernel, point, b)?
        || points_equal(kernel, point, c)?
    {
        return Ok(TriangleLocation::OnVertex);
    }

    let ab = kernel.orient2(a, b, point)?;
    let bc = kernel.orient2(b, c, point)?;
    let ca = kernel.orient2(c, a, point)?;
    let signs = [ab, bc, ca];

    let has_negative = signs.contains(&Sign::Negative);
    let has_positive = signs.contains(&Sign::Positive);
    if has_negative && has_positive {
        return Ok(TriangleLocation::Outside);
    }
    if signs.contains(&Sign::Zero) {
        Ok(TriangleLocation::OnEdge)
    } else {
        Ok(TriangleLocation::Inside)
    }
}

#[cfg(any(feature = "earcut", feature = "cdt"))]
fn points_equal(kernel: &ExactKernel, left: &Point2, right: &Point2) -> Result<bool> {
    Ok(kernel.cmp(&left.x, &right.x)? == Ordering::Equal
        && kernel.cmp(&left.y, &right.y)? == Ordering::Equal)
}

#[cfg(any(feature = "earcut", feature = "cdt"))]
fn map_hyperlimit_sign(sign: hyperlimit::Sign) -> Sign {
    match sign {
        hyperlimit::Sign::Negative => Sign::Negative,
        hyperlimit::Sign::Zero => Sign::Zero,
        hyperlimit::Sign::Positive => Sign::Positive,
    }
}

#[cfg(all(test, any(feature = "earcut", feature = "cdt")))]
mod tests {
    use super::*;

    const APPROX: TriangulationContext =
        TriangulationContext::new(PredicatePolicy::APPROXIMATE_512);

    fn kernel() -> ExactKernel {
        ExactKernel::new(&APPROX)
    }

    fn p(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn exact_kernel_routes_orientation_through_predicate_layer() {
        let kernel = kernel();
        assert_eq!(
            kernel.orient2(&p(0, 0), &p(2, 0), &p(1, 1)).unwrap(),
            Sign::Positive
        );
        assert_eq!(
            kernel.orient2(&p(0, 0), &p(1, 1), &p(2, 2)).unwrap(),
            Sign::Zero
        );
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn exact_kernel_compares_rationals_without_materializing_a_difference() {
        let context = TriangulationContext::new(PredicatePolicy::STRICT);
        let kernel = ExactKernel::new(&context);
        let left = Real::from(hyperreal::Rational::fraction(7, 13).unwrap());
        let right = Real::from(hyperreal::Rational::fraction(8, 13).unwrap());

        hyperreal::dispatch_trace::reset();
        let ordering = hyperreal::dispatch_trace::with_recording(|| kernel.cmp(&left, &right));

        assert_eq!(ordering, Ok(Ordering::Less));
        assert_eq!(
            kernel.finish(()).certainty,
            TriangulationCertainty::Certified
        );
        let trace = hyperreal::dispatch_trace::take_trace();
        assert_eq!(
            trace.path_count("hyperlimit", "compare_reals", "exact-rational"),
            1
        );
        assert_eq!(trace.operation_count("rational", "sub"), 0);
        assert_eq!(trace.rational.gcds, 0);
    }

    #[cfg(feature = "cdt")]
    #[test]
    fn exact_kernel_routes_incircle_through_predicate_layer() {
        let kernel = kernel();
        let a = p(0, 0);
        let b = p(2, 0);
        let c = p(0, 2);
        assert_eq!(
            kernel.incircle2(&a, &b, &c, &p(1, 1)).unwrap(),
            Sign::Positive
        );
        assert_eq!(kernel.incircle2(&a, &b, &c, &p(2, 2)).unwrap(), Sign::Zero);
        assert_eq!(
            kernel.incircle2(&a, &b, &c, &p(3, 3)).unwrap(),
            Sign::Negative
        );
    }

    #[cfg(feature = "earcut")]
    #[test]
    fn exact_kernel_uses_central_terminal_equality_policy() {
        let left = &Real::pi() + &Real::e();
        let right = &Real::e() + &Real::pi();
        assert_ne!(
            left, right,
            "the fixture must use distinct Real representations"
        );

        let left_point = Point2::new(left, Real::zero());
        let right_point = Point2::new(right, Real::zero());
        let kernel = kernel();
        assert_eq!(points_equal(&kernel, &left_point, &right_point), Ok(true));
        assert_eq!(
            kernel.finish(()).certainty,
            TriangulationCertainty::Approximate512Consumed
        );
        assert!(matches!(
            hyperlimit::classify_real_sign(
                &(&left_point.x - &right_point.x),
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let strict_context = TriangulationContext::new(PredicatePolicy::STRICT);
        let strict_kernel = ExactKernel::new(&strict_context);
        assert!(matches!(
            strict_kernel.cmp(&left_point.x, &right_point.x),
            Err(Error::PredicateUndecided {
                predicate: "exact Real ordering"
            })
        ));
        assert_eq!(
            strict_kernel.finish(()).certainty,
            TriangulationCertainty::Certified
        );
    }

    #[cfg(feature = "earcut")]
    #[test]
    fn triangle_vertex_classification_uses_kernel_coordinate_equality() {
        let left = Real::pi() + Real::e();
        let right = Real::e() + Real::pi();
        let a = Point2::new(left.clone(), Real::zero());
        let b = Point2::new(&left + &Real::from(2), Real::zero());
        let c = Point2::new(left, Real::from(2));
        let representation_distinct_query = Point2::new(right, Real::zero());
        assert_ne!(representation_distinct_query, a);
        assert_eq!(
            kernel()
                .classify_point_triangle(&a, &b, &c, &representation_distinct_query,)
                .unwrap(),
            TriangleLocation::OnVertex
        );
    }
}
