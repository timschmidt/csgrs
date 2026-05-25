//! Exact circle/circle tangency packages for retained sketch constraints.
//!
//! The sketch layer resolves circle handles, radius handles, and branch flags;
//! this module owns only the symbolic proof rows. This is the retained-object
//! split advocated by Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): a caller chooses the topological
//! tangency branch, and exact replay accepts or rejects the polynomial
//! evidence. The branch vocabulary mirrors the explicit geometric-constraint
//! solver objects described by Bouma, Fudos, Hoffmann, Cai, and Paige,
//! "A Geometric Constraint Solver," *Computer-Aided Design* 27.6 (1995).

use crate::symbolic::Expr;

/// Exact residual expressions for retained 2D circle/circle tangency.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CircleCircleTangentExprs {
    /// Branch-specific polynomial row:
    /// `|center_b-center_a|^2 - (r_a+r_b)^2` for external tangency, or
    /// `|center_b-center_a|^2 - (r_a-r_b)^2` for internal tangency.
    pub(crate) tangency: Expr,
}

/// Build the exact circle/circle tangency row for a retained branch.
///
/// Let `d = center_b - center_a`. External tangency replays
/// `|d|^2 == (r_a + r_b)^2`; internal tangency replays
/// `|d|^2 == (r_a - r_b)^2`. The row deliberately avoids square roots,
/// normalization, and absolute values. Radius positivity, noncoincident
/// centers, and containment policy are retained domain/preflight obligations
/// rather than hidden tolerance checks, following Yap's exact replay boundary.
pub(crate) fn circle_circle_tangent_exprs(
    first_center: &[Expr; 2],
    first_radius: Expr,
    second_center: &[Expr; 2],
    second_radius: Expr,
    external: bool,
) -> CircleCircleTangentExprs {
    let center_delta = [
        second_center[0].clone() - first_center[0].clone(),
        second_center[1].clone() - first_center[1].clone(),
    ];
    let branch_radius = if external {
        first_radius + second_radius
    } else {
        first_radius - second_radius
    };

    CircleCircleTangentExprs {
        tangency: squared_norm2(&center_delta) - branch_radius.clone() * branch_radius,
    }
}

fn squared_norm2(v: &[Expr; 2]) -> Expr {
    v[0].clone() * v[0].clone() + v[1].clone() * v[1].clone()
}
