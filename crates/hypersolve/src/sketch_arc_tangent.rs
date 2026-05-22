//! Exact 2D circular-arc/line tangency packages for sketch constraints.
//!
//! The retained sketch layer resolves arc, endpoint, and line handles; this
//! module owns only the symbolic proof rows. A certified arc-line tangent
//! package contains endpoint incidence, arc endpoint-on-radius validation,
//! radius/tangent perpendicularity, and a signed orientation branch. This is
//! the exact-computation boundary advocated by Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7.1-2 (1997): construction objects
//! and endpoint flags stay explicit, and exact replay decides acceptance. The
//! endpoint/orientation split follows the geometric-constraint-solver practice
//! surveyed by Bouma, Fudos, Hoffmann, Cai, and Paige, "A Geometric Constraint
//! Solver," *Computer-Aided Design* 27.6 (1995).

use crate::symbolic::Expr;

/// Exact residual expressions for a retained 2D arc-line tangency relation.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ArcLineTangentExprs {
    /// Equation certifying that the selected arc endpoint lies on the retained
    /// radius circle.
    pub(crate) arc_endpoint_radius: Expr,
    /// Coordinate incidence rows between the selected line endpoint and the
    /// selected arc endpoint.
    pub(crate) endpoint_incidence: [Expr; 2],
    /// Equation certifying that the radius vector is perpendicular to the
    /// selected outgoing line tangent.
    pub(crate) radius_perpendicular: Expr,
    /// Signed orientation row. Callers lower this as `>= 0`; clockwise callers
    /// pass the negated cross product so the same inequality convention works.
    pub(crate) orientation: Expr,
}

/// Build exact rows for a selected arc endpoint tangent to a selected line
/// endpoint.
///
/// Let `r = endpoint - center` and `t` be the outgoing line direction from the
/// selected line endpoint. The tangent proof rows are:
///
/// - `|r|^2 - radius^2 == 0`;
/// - selected line endpoint equals selected arc endpoint;
/// - `r . t == 0`;
/// - signed `r x t >= 0` for the selected orientation branch.
///
/// None of these rows divides by a radius or line length. Zero-radius arcs and
/// zero-length line tangents are therefore not hidden by normalization; callers
/// should attach the existing retained entity-domain preflight obligations.
pub(crate) fn arc_line_tangent_exprs(
    center: &[Expr; 2],
    arc_endpoint: &[Expr; 2],
    radius: Expr,
    line_endpoint: &[Expr; 2],
    line_tangent: &[Expr; 2],
    orientation_sign: i8,
) -> ArcLineTangentExprs {
    let radius_vector = [
        arc_endpoint[0].clone() - center[0].clone(),
        arc_endpoint[1].clone() - center[1].clone(),
    ];
    let endpoint_delta = [
        line_endpoint[0].clone() - arc_endpoint[0].clone(),
        line_endpoint[1].clone() - arc_endpoint[1].clone(),
    ];
    let orientation = cross2(&radius_vector, line_tangent);
    let orientation = if orientation_sign < 0 {
        Expr::zero() - orientation
    } else {
        orientation
    };

    ArcLineTangentExprs {
        arc_endpoint_radius: squared_norm2(&radius_vector) - radius.clone() * radius,
        endpoint_incidence: endpoint_delta,
        radius_perpendicular: dot2(&radius_vector, line_tangent),
        orientation,
    }
}

fn dot2(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    a[0].clone() * b[0].clone() + a[1].clone() * b[1].clone()
}

fn cross2(a: &[Expr; 2], b: &[Expr; 2]) -> Expr {
    a[0].clone() * b[1].clone() - a[1].clone() * b[0].clone()
}

fn squared_norm2(direction: &[Expr; 2]) -> Expr {
    direction[0].clone() * direction[0].clone() + direction[1].clone() * direction[1].clone()
}
