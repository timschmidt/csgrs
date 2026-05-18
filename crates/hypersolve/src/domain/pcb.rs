//! PCB residual builders for solver proposal stages.
//!
//! PCB autorouting uses search to choose topology, then continuous solves for
//! spacing, length matching, and tuning. These helpers build exact residuals
//! for that second stage only. Final no-short, pad, via, board-edge, and
//! clearance decisions still belong to exact `hyperpath`/`hyperlimit`
//! predicates. This mirrors Yap's proposed-object/certified-decision boundary
//! and the PCB routing separation surveyed by Yan, Ma, and Wong, "Advances in
//! PCB Routing" (2014).

use hyperreal::Real;

use crate::domain::geometry::{Point2, squared_distance_expr};
use crate::model::{Constraint, ConstraintKind};
use crate::symbolic::Expr;

/// Collection of residuals produced by a PCB-specific model builder.
#[derive(Clone, Debug, Default)]
pub struct PcbConstraintSet {
    /// Constraints in stable construction order.
    pub constraints: Vec<Constraint>,
}

impl PcbConstraintSet {
    /// Append one constraint.
    pub fn push(&mut self, constraint: Constraint) {
        self.constraints.push(constraint);
    }

    /// Return whether no constraints have been added.
    pub fn is_empty(&self) -> bool {
        self.constraints.is_empty()
    }
}

/// Build a squared center-clearance inequality.
///
/// The residual is `|first - second|^2 - required_clearance^2 >= 0`. Squared
/// clearance avoids square roots in solver replay and matches the exact
/// distance-squared style used by `hyperpath` pad/via clearance predicates.
/// This constraint is only a continuous proposal check; topology and copper
/// intersection still require the domain predicates after solving.
pub fn center_clearance_squared_constraint(
    name: impl Into<String>,
    first: Point2,
    second: Point2,
    required_clearance: Real,
) -> Constraint {
    let required_squared = required_clearance.clone() * required_clearance;
    Constraint {
        name: name.into(),
        kind: ConstraintKind::GreaterOrEqual,
        residual: squared_distance_expr(first, second) - Expr::real(required_squared),
        weight: Real::one(),
        active: true,
    }
}

/// Build an exact differential-pair length-skew equality.
///
/// The residual is `first_length - second_length - target_skew = 0`. PCB
/// routers often treat differential-pair matching as a continuous tuning step
/// after topology has been proposed; Yan, Ma, and Wong's PCB routing survey
/// frames that separation between routing search and later timing/length
/// adjustment. This helper keeps that adjustment in `hypersolve` as exact
/// residual replay while `hyperpath` remains responsible for the meander,
/// clearance, pad, via, and board predicates that make the proposed geometry
/// acceptable.
pub fn differential_pair_skew_equation(
    name: impl Into<String>,
    first_length: Expr,
    second_length: Expr,
    target_skew: Real,
) -> Constraint {
    Constraint::equality(name, first_length - second_length - Expr::real(target_skew))
}
