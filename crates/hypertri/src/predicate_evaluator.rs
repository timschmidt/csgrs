//! Operation-local predicate evaluation used by triangulation algorithms.
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
#[cfg(feature = "cdt")]
use std::cmp::Ordering;

/// Operation-local predicate evaluator backed by [`hyperreal::Real`].
///
/// A fresh evaluator is created for every public operation. Its interior cell
/// aggregates whether any Hyperlimit decision consumed APPROXIMATE_512 without
/// making the caller's immutable [`TriangulationContext`] stateful.
#[derive(Debug)]
pub(crate) struct PredicateEvaluator {
    #[cfg(any(feature = "earcut", feature = "cdt", feature = "nd"))]
    policy: PredicatePolicy,
    certainty: Cell<TriangulationCertainty>,
}

impl PredicateEvaluator {
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

    #[cfg(feature = "cdt")]
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

    #[cfg(feature = "cdt")]
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
        let orientation = self.orient2(a, b, c)?;
        self.classify_point_triangle_with_orientation(a, b, c, point, orientation)
    }

    /// Classify a point after the ordered triangle orientation has already
    /// been decided by this operation's predicate evaluator.
    ///
    /// Incremental triangulation retains positive winding for every active
    /// triangle, and cavity ear selection decides the same turn immediately
    /// before testing contained vertices. Reusing that topological fact avoids
    /// rebuilding the fixed determinant while every query-edge orientation
    /// still enters the normal policy-aware predicate cascade.
    #[cfg(any(feature = "earcut", feature = "cdt"))]
    pub(crate) fn classify_point_triangle_with_orientation(
        &self,
        a: &Point2,
        b: &Point2,
        c: &Point2,
        point: &Point2,
        orientation: Sign,
    ) -> Result<TriangleLocation> {
        classify_point_triangle_with_orientation(self, a, b, c, point, orientation)
    }
}

#[cfg(any(feature = "earcut", feature = "cdt"))]
fn classify_point_triangle_with_orientation(
    evaluator: &PredicateEvaluator,
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
    orientation: Sign,
) -> Result<TriangleLocation> {
    if orientation == Sign::Zero {
        return Ok(TriangleLocation::Degenerate);
    }

    let ab = evaluator.orient2(a, b, point)?;
    let bc = evaluator.orient2(b, c, point)?;
    let ca = evaluator.orient2(c, a, point)?;
    let signs = [ab, bc, ca];

    if signs.contains(&orientation.reversed()) {
        return Ok(TriangleLocation::Outside);
    }
    match signs.iter().filter(|&&sign| sign == Sign::Zero).count() {
        0 => Ok(TriangleLocation::Inside),
        1 => Ok(TriangleLocation::OnEdge),
        _ => Ok(TriangleLocation::OnVertex),
    }
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

    fn evaluator() -> PredicateEvaluator {
        PredicateEvaluator::new(&APPROX)
    }

    fn p(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn predicate_evaluator_routes_orientation_through_predicate_layer() {
        let evaluator = evaluator();
        assert_eq!(
            evaluator.orient2(&p(0, 0), &p(2, 0), &p(1, 1)).unwrap(),
            Sign::Positive
        );
        assert_eq!(
            evaluator.orient2(&p(0, 0), &p(1, 1), &p(2, 2)).unwrap(),
            Sign::Zero
        );
    }

    #[test]
    fn predicate_evaluator_maps_real_division_failure() {
        assert_eq!(
            PredicateEvaluator::div(&Real::one(), &Real::zero()),
            Err(Error::InvalidInput {
                reason: "Real division failed"
            })
        );
    }

    #[cfg(all(feature = "dispatch-trace", feature = "cdt"))]
    #[test]
    fn predicate_evaluator_compares_rationals_without_materializing_a_difference() {
        let context = TriangulationContext::new(PredicatePolicy::STRICT);
        let evaluator = PredicateEvaluator::new(&context);
        let left = Real::from(hyperreal::Rational::fraction(7, 13).unwrap());
        let right = Real::from(hyperreal::Rational::fraction(8, 13).unwrap());

        hyperreal::dispatch_trace::reset();
        let ordering = hyperreal::dispatch_trace::with_recording(|| evaluator.cmp(&left, &right));

        assert_eq!(ordering, Ok(Ordering::Less));
        assert_eq!(
            evaluator.finish(()).certainty,
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
    fn predicate_evaluator_routes_incircle_through_predicate_layer() {
        let evaluator = evaluator();
        let a = p(0, 0);
        let b = p(2, 0);
        let c = p(0, 2);
        assert_eq!(
            evaluator.incircle2(&a, &b, &c, &p(1, 1)).unwrap(),
            Sign::Positive
        );
        assert_eq!(
            evaluator.incircle2(&a, &b, &c, &p(2, 2)).unwrap(),
            Sign::Zero
        );
        assert_eq!(
            evaluator.incircle2(&a, &b, &c, &p(3, 3)).unwrap(),
            Sign::Negative
        );
    }

    #[cfg(feature = "earcut")]
    #[test]
    fn predicate_evaluator_uses_central_terminal_equality_policy() {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let left = &sine * &sine + &cosine * &cosine + Real::from(2);
        let right = Real::from(3);
        assert_ne!(
            left, right,
            "the fixture must use distinct Real representations"
        );

        let left_point = Point2::new(left, Real::zero());
        let right_point = Point2::new(right, Real::zero());
        let evaluator = evaluator();
        assert_eq!(
            crate::predicates::points_equal(&evaluator, &left_point, &right_point),
            Ok(true)
        );
        assert_eq!(
            evaluator.finish(()).certainty,
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
        let strict_evaluator = PredicateEvaluator::new(&strict_context);
        assert!(matches!(
            crate::predicates::points_equal(&strict_evaluator, &left_point, &right_point),
            Err(Error::PredicateUndecided {
                predicate: "point2_equal"
            })
        ));
        assert_eq!(
            strict_evaluator.finish(()).certainty,
            TriangulationCertainty::Certified
        );
    }

    #[cfg(feature = "earcut")]
    #[test]
    fn triangle_vertex_classification_uses_exact_edge_incidence() {
        let left = Real::pi() + Real::e();
        let right = Real::e() + Real::pi();
        let a = Point2::new(left.clone(), Real::zero());
        let b = Point2::new(&left + &Real::from(2), Real::zero());
        let c = Point2::new(left, Real::from(2));
        let representation_distinct_query = Point2::new(right, Real::zero());
        assert_ne!(representation_distinct_query, a);
        assert_eq!(
            evaluator()
                .classify_point_triangle(&a, &b, &c, &representation_distinct_query,)
                .unwrap(),
            TriangleLocation::OnVertex
        );
    }

    #[test]
    fn retained_triangle_orientation_matches_immediate_classification() {
        let triangle = [p(0, 0), p(4, 0), p(0, 4)];
        let queries = [
            (p(1, 1), TriangleLocation::Inside),
            (p(2, 0), TriangleLocation::OnEdge),
            (p(0, 0), TriangleLocation::OnVertex),
            (p(3, 3), TriangleLocation::Outside),
        ];

        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            let context = TriangulationContext::new(policy);
            for (indices, orientation) in [([0, 1, 2], Sign::Positive), ([0, 2, 1], Sign::Negative)]
            {
                let [a, b, c] = indices.map(|index| &triangle[index]);
                for (query, expected) in &queries {
                    let immediate = PredicateEvaluator::new(&context);
                    assert_eq!(
                        immediate.classify_point_triangle(a, b, c, query),
                        Ok(*expected)
                    );

                    let retained = PredicateEvaluator::new(&context);
                    assert_eq!(
                        retained.classify_point_triangle_with_orientation(
                            a,
                            b,
                            c,
                            query,
                            orientation,
                        ),
                        Ok(*expected)
                    );
                    assert_eq!(
                        retained.finish(()).certainty,
                        TriangulationCertainty::Certified
                    );
                }
            }

            let degenerate = PredicateEvaluator::new(&context);
            assert_eq!(
                degenerate.classify_point_triangle_with_orientation(
                    &triangle[0],
                    &triangle[1],
                    &triangle[0],
                    &queries[0].0,
                    Sign::Zero,
                ),
                Ok(TriangleLocation::Degenerate)
            );
        }
    }

    #[cfg(feature = "dispatch-trace")]
    #[test]
    fn retained_triangle_orientation_omits_the_fixed_determinant() {
        let context = TriangulationContext::new(PredicatePolicy::STRICT);
        let [a, b, c, query] = [p(0, 0), p(4, 0), p(0, 4), p(1, 1)];

        hyperreal::dispatch_trace::reset();
        let immediate = hyperreal::dispatch_trace::with_recording(|| {
            PredicateEvaluator::new(&context).classify_point_triangle(&a, &b, &c, &query)
        });
        assert_eq!(immediate, Ok(TriangleLocation::Inside));
        let immediate_trace = hyperreal::dispatch_trace::take_trace();

        hyperreal::dispatch_trace::reset();
        let retained = hyperreal::dispatch_trace::with_recording(|| {
            PredicateEvaluator::new(&context).classify_point_triangle_with_orientation(
                &a,
                &b,
                &c,
                &query,
                Sign::Positive,
            )
        });
        assert_eq!(retained, Ok(TriangleLocation::Inside));
        let retained_trace = hyperreal::dispatch_trace::take_trace();

        assert_eq!(immediate_trace.operation_count("hyperlimit", "orient2d"), 4);
        assert_eq!(retained_trace.operation_count("hyperlimit", "orient2d"), 3);
    }
}
