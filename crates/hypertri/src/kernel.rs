//! Numeric kernels used by triangulation algorithms.
//!
//! All irreversible topology decisions in exact code should pass through this
//! layer. The predicate pipeline follows the exact-geometric-computation model
//! described by Yap and the adaptive predicate discipline described by Shewchuk:
//! cheap structural facts and filters are used only when they certify a sign;
//! otherwise the exact/refinement path must decide or report uncertainty.

use crate::error::{Error, Result};
use crate::types::{Point2, Real, Sign, TriangleLocation};
use hyperlimit::PredicateOutcome;
use hyperreal::{RealSign, ZeroKnowledge};
use std::cmp::Ordering;

/// Numeric operations required by triangulation algorithms.
pub trait Kernel {
    /// Additive identity.
    fn zero() -> Real;

    /// Construct a Real from a small integer.
    fn from_i64(value: i64) -> Real;

    /// Add two Real values.
    fn add(left: &Real, right: &Real) -> Real;

    /// Subtract two Real values.
    fn sub(left: &Real, right: &Real) -> Real;

    /// Multiply two Real values.
    fn mul(left: &Real, right: &Real) -> Real;

    /// Divide two Real values.
    fn div(left: &Real, right: &Real) -> Result<Real>;

    /// Return the exact midpoint between two points.
    fn midpoint(left: &Point2, right: &Point2) -> Result<Point2> {
        let two = Self::from_i64(2);
        Ok(Point2::new(
            Self::div(&Self::add(&left.x, &right.x), &two)?,
            Self::div(&Self::add(&left.y, &right.y), &two)?,
        ))
    }

    /// Decide a Real sign.
    fn real_sign(value: &Real) -> Result<Sign>;

    /// Compare two Real values by deciding the sign of `left - right`.
    fn cmp(left: &Real, right: &Real) -> Result<Ordering> {
        match Self::real_sign(&Self::sub(left, right))? {
            Sign::Negative => Ok(Ordering::Less),
            Sign::Zero => Ok(Ordering::Equal),
            Sign::Positive => Ok(Ordering::Greater),
        }
    }

    /// Decide the orientation of three 2D points.
    fn orient2d(a: &Point2, b: &Point2, c: &Point2) -> Result<Sign>;

    /// Decide the in-circle relation of four 2D points.
    fn incircle2d(a: &Point2, b: &Point2, c: &Point2, d: &Point2) -> Result<Sign>;

    /// Classify a point relative to a triangle.
    fn classify_point_triangle(
        a: &Point2,
        b: &Point2,
        c: &Point2,
        point: &Point2,
    ) -> Result<TriangleLocation>;
}

/// Exact kernel backed by [`hyperreal::Real`].
#[derive(Clone, Copy, Debug, Default)]
pub struct ExactKernel;

impl Kernel for ExactKernel {
    fn zero() -> Real {
        Real::zero()
    }

    fn from_i64(value: i64) -> Real {
        Real::from(value)
    }

    fn add(left: &Real, right: &Real) -> Real {
        left + right
    }

    fn sub(left: &Real, right: &Real) -> Real {
        left - right
    }

    fn mul(left: &Real, right: &Real) -> Real {
        left * right
    }

    fn div(left: &Real, right: &Real) -> Result<Real> {
        (left / right).map_err(|_| Error::InvalidInput {
            reason: "Real division failed",
        })
    }

    fn real_sign(value: &Real) -> Result<Sign> {
        let facts = value.structural_facts();

        if let Some(sign) = facts.sign {
            return Ok(map_real_sign(sign));
        }

        match facts.zero {
            ZeroKnowledge::Zero => return Ok(Sign::Zero),
            ZeroKnowledge::NonZero | ZeroKnowledge::Unknown => {}
        }

        // Structural-dispatch note: this Real path currently consumes only
        // sign and zero status. Future predicate kernels can avoid expression
        // growth by carrying exact-rational kind, dyadic denominator shape, and
        // Real-local magnitude-bit classes through determinant construction, then
        // selecting a fraction-free or sparse product-sum path before asking
        // hyperreal to refine the final sign.
        if let Some(sign) = value.refine_sign_until(-4096) {
            return Ok(map_real_sign(sign));
        }

        Err(Error::PredicateUndecided {
            predicate: "exact Real sign",
        })
    }

    fn orient2d(a: &Point2, b: &Point2, c: &Point2) -> Result<Sign> {
        // Triangulation topology consumes `hyperlimit`'s certified predicate
        // pipeline rather than rebuilding a private determinant expression.
        // Keeping the determinant owner in the predicate crate follows Yap's
        // exact-geometric-computation separation between application topology
        // and arithmetic packages; see Yap, "Towards Exact Geometric
        // Computation," *Computational Geometry* 7.1-2 (1997). The predicate
        // implementation carries Shewchuk-style robust-orientation discipline
        // and exact rational/common-scale schedules near their use.
        decide_hyperlimit_sign(
            hyperlimit::orient2d(
                &predicate_point(a),
                &predicate_point(b),
                &predicate_point(c),
            ),
            "orient2d",
        )
    }

    fn incircle2d(a: &Point2, b: &Point2, c: &Point2, d: &Point2) -> Result<Sign> {
        // In-circle legality is the CDT edge-flip predicate. Route it through
        // `hyperlimit` so exact lifted-determinant certificates and future
        // prepared incircle facts remain centralized, matching Delaunay's empty
        // circle test while preserving Yap's object/predicate boundary.
        decide_hyperlimit_sign(
            hyperlimit::incircle2d(
                &predicate_point(a),
                &predicate_point(b),
                &predicate_point(c),
                &predicate_point(d),
            ),
            "incircle2d",
        )
    }

    fn classify_point_triangle(
        a: &Point2,
        b: &Point2,
        c: &Point2,
        point: &Point2,
    ) -> Result<TriangleLocation> {
        classify_point_triangle::<Self>(a, b, c, point)
    }
}

fn classify_point_triangle<K>(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
) -> Result<TriangleLocation>
where
    K: Kernel,
{
    let orientation = K::orient2d(a, b, c)?;
    if orientation == Sign::Zero {
        return Ok(TriangleLocation::Degenerate);
    }

    if point == a || point == b || point == c {
        return Ok(TriangleLocation::OnVertex);
    }

    let ab = K::orient2d(a, b, point)?;
    let bc = K::orient2d(b, c, point)?;
    let ca = K::orient2d(c, a, point)?;
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

fn map_real_sign(sign: RealSign) -> Sign {
    match sign {
        RealSign::Negative => Sign::Negative,
        RealSign::Zero => Sign::Zero,
        RealSign::Positive => Sign::Positive,
    }
}

fn predicate_point(point: &Point2) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x.clone(), point.y.clone())
}

fn decide_hyperlimit_sign(
    outcome: PredicateOutcome<hyperlimit::Sign>,
    predicate: &'static str,
) -> Result<Sign> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Ok(map_hyperlimit_sign(value)),
        PredicateOutcome::Unknown { .. } => Err(Error::PredicateUndecided { predicate }),
    }
}

fn map_hyperlimit_sign(sign: hyperlimit::Sign) -> Sign {
    match sign {
        hyperlimit::Sign::Negative => Sign::Negative,
        hyperlimit::Sign::Zero => Sign::Zero,
        hyperlimit::Sign::Positive => Sign::Positive,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn exact_kernel_routes_orientation_through_predicate_layer() {
        assert_eq!(
            ExactKernel::orient2d(&p(0, 0), &p(2, 0), &p(1, 1)).unwrap(),
            Sign::Positive
        );
        assert_eq!(
            ExactKernel::orient2d(&p(0, 0), &p(1, 1), &p(2, 2)).unwrap(),
            Sign::Zero
        );
    }

    #[test]
    fn exact_kernel_routes_incircle_through_predicate_layer() {
        let a = p(0, 0);
        let b = p(2, 0);
        let c = p(0, 2);
        assert_eq!(
            ExactKernel::incircle2d(&a, &b, &c, &p(1, 1)).unwrap(),
            Sign::Positive
        );
        assert_eq!(
            ExactKernel::incircle2d(&a, &b, &c, &p(2, 2)).unwrap(),
            Sign::Zero
        );
        assert_eq!(
            ExactKernel::incircle2d(&a, &b, &c, &p(3, 3)).unwrap(),
            Sign::Negative
        );
    }
}
