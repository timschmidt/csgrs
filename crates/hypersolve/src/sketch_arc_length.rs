//! Exact retained circular-arc length packages for sketch constraints.
//!
//! Circular arc length is not a polynomial predicate in the general retained
//! endpoint model: it is `r * theta`, and `theta` is recovered from the
//! endpoint radius vectors. Following Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997), this module keeps that
//! semantic boundary explicit instead of disguising the relation as an
//! algebraic row. The endpoint-on-radius equations remain polynomial proof
//! obligations, while the line/arc length equality is retained as an exact
//! symbolic transcendental residual. The decomposition matches the
//! constraint-solver modeling split surveyed by Bouma, Fudos, Hoffmann, Cai,
//! and Paige, "A Geometric Constraint Solver," *Computer-Aided Design* 27.6
//! (1995): construction objects are retained, and replay checks the exact rows
//! appropriate to each object.

use crate::symbolic::Expr;

/// Exact residual expressions for a retained 2D line/minor-arc length relation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct LineArcLengthExprs {
    /// Equations certifying that the start and end points lie on the retained
    /// radius circle.
    pub(crate) endpoint_radius: [Expr; 2],
    /// Exact symbolic residual `|line|^2 - (radius * acos(cos(theta)))^2`.
    ///
    /// The angle is the principal unsigned angle between the retained start
    /// and end radius vectors, so this is the minor arc length in `[0, pi]`.
    /// Major or oriented arc lengths require an explicit branch carrier and
    /// are intentionally not inferred from floating sweep data.
    pub(crate) length_equality: Expr,
}

/// Build exact rows for a retained line length equal to a circular minor arc.
///
/// Let `s = start - center`, `e = end - center`, and `d` be the line direction.
/// The package contains:
///
/// - `|s|^2 - radius^2 == 0`;
/// - `|e|^2 - radius^2 == 0`;
/// - `|d|^2 - (radius * acos((s . e) / (|s| |e|)))^2 == 0`.
///
/// The final residual deliberately contains `acos`: exact replay may evaluate
/// it through `hyperreal`, but callers should not treat it as a polynomial
/// proof row. The line length is squared to avoid introducing a second
/// square-root branch for a segment whose length is already nonnegative by
/// construction. Zero-radius arcs and degenerate endpoints remain explicit
/// retained-domain obligations because the cosine expression divides by
/// `|s| |e|`.
pub(crate) fn line_arc_length_exprs(
    center: &[Expr; 2],
    start: &[Expr; 2],
    end: &[Expr; 2],
    radius: Expr,
    line_direction: &[Expr; 2],
) -> LineArcLengthExprs {
    let start_radius = [
        start[0].clone() - center[0].clone(),
        start[1].clone() - center[1].clone(),
    ];
    let end_radius = [
        end[0].clone() - center[0].clone(),
        end[1].clone() - center[1].clone(),
    ];
    let cosine = angle_cosine_expr(&start_radius, &end_radius);
    let arc_length = radius.clone() * cosine.acos();

    LineArcLengthExprs {
        endpoint_radius: [
            squared_norm2(&start_radius) - radius.clone() * radius.clone(),
            squared_norm2(&end_radius) - radius.clone() * radius,
        ],
        length_equality: squared_norm2(line_direction) - arc_length.clone() * arc_length,
    }
}

fn angle_cosine_expr(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    dot2(a, b) / (squared_norm2(a).sqrt() * squared_norm2(b).sqrt())
}

fn dot2(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    a[0].clone() * b[0].clone() + a[1].clone() * b[1].clone()
}

fn squared_norm2(v: &[Expr; 2]) -> Expr {
    dot2(v, v)
}
