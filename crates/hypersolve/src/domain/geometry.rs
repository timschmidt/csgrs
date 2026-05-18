//! Exact geometry residual builders for domain crates.
//!
//! These helpers are intentionally small expression constructors, not a
//! geometry kernel. Domain crates such as `hyperpath` should continue to ask
//! `hyperlimit` for topological predicates, while `hypersolve` receives only
//! continuous residuals that a numerical proposal engine may adjust and exact
//! replay may certify. This follows Yap, "Towards Exact Geometric Computation"
//! (1997): proposed numeric values are separate from certified geometric
//! decisions. The tangent residuals also mirror the vector cross/dot tests used
//! by exact curve arrangement kernels such as CGAL's arrangement traits.

use hyperreal::Real;

use crate::model::{Constraint, ConstraintKind, VariableId};
use crate::symbolic::Expr;

/// Pair of solver variables representing an exact 2D point candidate.
///
/// The point is a residual-model handle, not a geometric predicate object.
/// Its coordinates lower to [`Expr`] leaves so callers can build exact
/// incidence, distance, and tangent residuals without losing the variable
/// identity needed by prepared Jacobian and affine passes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Point2 {
    /// X-coordinate variable.
    pub x: VariableId,
    /// Y-coordinate variable.
    pub y: VariableId,
}

impl Point2 {
    /// Construct a point handle from two solver variables.
    pub fn new(x: VariableId, y: VariableId) -> Self {
        Self { x, y }
    }

    /// Return the x-coordinate as a symbolic expression.
    pub fn x_expr(self) -> Expr {
        Expr::symbol(self.x.into(), "point_x")
    }

    /// Return the y-coordinate as a symbolic expression.
    pub fn y_expr(self) -> Expr {
        Expr::symbol(self.y.into(), "point_y")
    }
}

impl From<VariableId> for crate::symbolic::SymbolId {
    fn from(value: VariableId) -> Self {
        Self(value.0)
    }
}

/// Build a named equality constraint.
pub fn named_equation(name: impl Into<String>, residual: Expr) -> Constraint {
    Constraint::equality(name, residual)
}

/// Return the exact squared-distance residual expression between two points.
///
/// Squared distance avoids square-root normalization and keeps the residual in
/// the polynomial fragment whenever both points are variables or exact
/// constants. That is the same representation used by exact circle/pad
/// clearance predicates before they delegate topology to `hyperlimit`.
pub fn squared_distance_expr(first: Point2, second: Point2) -> Expr {
    let dx = first.x_expr() - second.x_expr();
    let dy = first.y_expr() - second.y_expr();
    dx.clone() * dx + dy.clone() * dy
}

/// Build `|first - second|^2 - target_squared = 0`.
pub fn squared_distance_equation(
    name: impl Into<String>,
    first: Point2,
    second: Point2,
    target_squared: Real,
) -> Constraint {
    named_equation(
        name,
        squared_distance_expr(first, second) - Expr::real(target_squared),
    )
}

/// Build exact coordinate coincidence equations for two point handles.
///
/// The result is two equality rows, `first.x - second.x = 0` and
/// `first.y - second.y = 0`. Keeping the rows split lets affine preparation
/// and direct substitution see each coordinate independently before any dense
/// nonlinear backend is asked for a proposal.
pub fn point_coincidence_equations(
    name_prefix: impl AsRef<str>,
    first: Point2,
    second: Point2,
) -> Vec<Constraint> {
    let prefix = name_prefix.as_ref();
    vec![
        named_equation(
            format!("{prefix} x coincidence"),
            first.x_expr() - second.x_expr(),
        ),
        named_equation(
            format!("{prefix} y coincidence"),
            first.y_expr() - second.y_expr(),
        ),
    ]
}

/// Return the 2D cross-product expression `a.x * b.y - a.y * b.x`.
pub fn tangent_cross_expr(first: Point2, second: Point2) -> Expr {
    first.x_expr() * second.y_expr() - first.y_expr() * second.x_expr()
}

/// Return the 2D dot-product expression `a.x * b.x + a.y * b.y`.
pub fn tangent_dot_expr(first: Point2, second: Point2) -> Expr {
    first.x_expr() * second.x_expr() + first.y_expr() * second.y_expr()
}

/// Build a parallel-tangent equality using the exact cross product.
///
/// This is the residual form of the G1 tangent predicate: a candidate tangent
/// may be produced by a smoother or fillet solver, but exact replay must prove
/// the cross product is zero before the candidate is trusted.
pub fn tangent_parallel_equation(
    name: impl Into<String>,
    candidate: Point2,
    target: Point2,
) -> Constraint {
    named_equation(name, tangent_cross_expr(candidate, target))
}

/// Build a same-direction tangent inequality using the exact dot product.
///
/// The residual is `candidate · target >= 0`, rejecting reversed tangents
/// without normalizing either vector. Degenerate zero tangents should still be
/// handled by the domain predicate layer.
pub fn tangent_same_direction_constraint(
    name: impl Into<String>,
    candidate: Point2,
    target: Point2,
) -> Constraint {
    Constraint {
        name: name.into(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: tangent_dot_expr(candidate, target),
        weight: Real::one(),
        active: true,
    }
}
