//! Exact oriented 2D angle-equality packages for sketch constraints.
//!
//! A retained oriented angle is represented by the two scalar invariants of a
//! line pair, `(dot(a, b), cross(a, b))`. For nondegenerate line carriers this
//! vector is a positive scale of `(cos(theta), sin(theta))`, so equality of two
//! oriented angles can be certified without `atan2`, inverse trigonometry, or
//! primitive-float normalization. That keeps the decision boundary at exact
//! replay, as required by Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997). The explicit angle-vector and branch
//! split follows the retained-constraint vocabulary surveyed by Bouma, Fudos,
//! Hoffmann, Cai, and Paige, "A Geometric Constraint Solver,"
//! *Computer-Aided Design* 27.6 (1995).

use crate::symbolic::Expr;

/// Exact residual expressions for a retained oriented 2D equal-angle relation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct OrientedAngleExprs {
    /// Equation certifying that the two angle vectors are collinear:
    /// `dot(a,b) * cross(c,d) - cross(a,b) * dot(c,d) == 0`.
    pub(crate) angle_vector_collinear: Expr,
    /// Inequality certifying that the collinear angle vectors point along the
    /// same oriented branch rather than the supplemental branch.
    pub(crate) same_branch: Expr,
}

/// Build exact rows for equality of two oriented 2D angles.
///
/// The inputs are the four unnormalized line directions for the angles
/// `(a,b)` and `(c,d)`. The proof package is:
///
/// - angle-vector collinearity, `(dot_ab, cross_ab) x (dot_cd, cross_cd) == 0`;
/// - same-branch predicate, `(dot_ab, cross_ab) . (dot_cd, cross_cd) >= 0`.
///
/// No row divides by a line length. Zero-length line carriers therefore remain
/// explicit entity-domain obligations instead of disappearing behind a
/// normalized angle.
pub(crate) fn oriented_angle_exprs(
    a: &[Expr; 2],
    b: &[Expr; 2],
    c: &[Expr; 2],
    d: &[Expr; 2],
) -> OrientedAngleExprs {
    let first_dot = dot2(a, b);
    let first_cross = cross2(a, b);
    let second_dot = dot2(c, d);
    let second_cross = cross2(c, d);

    OrientedAngleExprs {
        angle_vector_collinear: first_dot.clone() * second_cross.clone()
            - first_cross.clone() * second_dot.clone(),
        same_branch: first_dot * second_dot + first_cross * second_cross,
    }
}

fn dot2(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    a[0].clone() * b[0].clone() + a[1].clone() * b[1].clone()
}

fn cross2(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    a[0].clone() * b[1].clone() - a[1].clone() * b[0].clone()
}
