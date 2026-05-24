//! Exact cubic-Bezier point and tangent expressions for sketch constraints.
//!
//! This module keeps the differential part of retained cubic constraints out
//! of the general sketch handle/diagnostic layer. The formulas are Bernstein
//! polynomials: point evaluation uses the cubic basis and tangent evaluation
//! uses the derivative control net
//! `3(P1-P0), 3(P2-P1), 3(P3-P2)`. Keeping both expressions symbolic follows
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7.1-2 (1997): curve construction and parameter choice remain retained
//! objects, while exact replay certifies the incidence and tangent rows. The
//! Bernstein/de Casteljau model follows Farin, *Curves and Surfaces for CAGD*,
//! 5th ed. (2002).

use crate::symbolic::Expr;

/// Exact point and derivative expressions for a retained cubic Bezier curve.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CubicPointTangentExprs {
    /// Cubic Bezier point `B(t)`.
    pub(crate) point: [Expr; 2],
    /// Cubic Bezier derivative `B'(t)`.
    pub(crate) tangent: [Expr; 2],
}

/// Build exact symbolic point and tangent expressions for a 2D cubic Bezier.
///
/// The parameter is accepted as an expression, not a primitive float. This
/// function deliberately does not impose `0 <= t <= 1`; segment-domain policy
/// belongs to explicit sketch parameter domains or range constraints.
pub(crate) fn cubic_point_tangent_exprs(
    p0: &[Expr; 2],
    p1: &[Expr; 2],
    p2: &[Expr; 2],
    p3: &[Expr; 2],
    parameter: Expr,
) -> CubicPointTangentExprs {
    let one_minus_t = Expr::int(1) - parameter.clone();
    let b0 = one_minus_t.clone() * one_minus_t.clone() * one_minus_t.clone();
    let b1 = Expr::int(3) * one_minus_t.clone() * one_minus_t.clone() * parameter.clone();
    let b2 = Expr::int(3) * one_minus_t.clone() * parameter.clone() * parameter.clone();
    let b3 = parameter.clone() * parameter.clone() * parameter.clone();
    let q0 = [
        Expr::int(3) * (p1[0].clone() - p0[0].clone()),
        Expr::int(3) * (p1[1].clone() - p0[1].clone()),
    ];
    let q1 = [
        Expr::int(3) * (p2[0].clone() - p1[0].clone()),
        Expr::int(3) * (p2[1].clone() - p1[1].clone()),
    ];
    let q2 = [
        Expr::int(3) * (p3[0].clone() - p2[0].clone()),
        Expr::int(3) * (p3[1].clone() - p2[1].clone()),
    ];
    let d0 = one_minus_t.clone() * one_minus_t.clone();
    let d1 = Expr::int(2) * one_minus_t * parameter.clone();
    let d2 = parameter.clone() * parameter;

    CubicPointTangentExprs {
        point: [
            p0[0].clone() * b0.clone()
                + p1[0].clone() * b1.clone()
                + p2[0].clone() * b2.clone()
                + p3[0].clone() * b3.clone(),
            p0[1].clone() * b0 + p1[1].clone() * b1 + p2[1].clone() * b2 + p3[1].clone() * b3,
        ],
        tangent: [
            q0[0].clone() * d0.clone() + q1[0].clone() * d1.clone() + q2[0].clone() * d2.clone(),
            q0[1].clone() * d0 + q1[1].clone() * d1 + q2[1].clone() * d2,
        ],
    }
}
