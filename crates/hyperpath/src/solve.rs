//! Domain residual builders lowered into `hypersolve`.
//!
//! `hyperpath` owns PCB, routing, and CAM semantics. These helpers build exact
//! solver residuals for proposal/replay stages while leaving accepted geometry
//! and topology certification in path and predicate modules.

use hyperlimit::Point2;
use hyperreal::Real;
use hypersolve::{
    Constraint, ConstraintKind, Expr, SolverPoint2, VariableId, squared_distance_expr,
};

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
pub fn center_clearance_squared_constraint(
    name: impl Into<String>,
    first: SolverPoint2,
    second: SolverPoint2,
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
pub fn differential_pair_skew_equation(
    name: impl Into<String>,
    first_length: Expr,
    second_length: Expr,
    target_skew: Real,
) -> Constraint {
    Constraint::equality(name, first_length - second_length - Expr::real(target_skew))
}

/// Exact rectangular region carrier for solver replay.
#[derive(Clone, Debug, PartialEq)]
pub struct RectangularRegion {
    /// Exact minimum corner.
    pub min: Point2,
    /// Exact maximum corner.
    pub max: Point2,
}

impl RectangularRegion {
    /// Construct a retained rectangular region from exact corners.
    pub const fn new(min: Point2, max: Point2) -> Self {
        Self { min, max }
    }

    /// Return the exact symbolic area expression `(max.x-min.x)*(max.y-min.y)`.
    pub fn area_expr(&self) -> Expr {
        rectangle_area_expr(&self.min, &self.max)
    }
}

/// Collection of residuals produced by a toolpath model builder.
#[derive(Clone, Debug, Default)]
pub struct ToolpathConstraintSet {
    /// Constraints in stable construction order.
    pub constraints: Vec<Constraint>,
}

impl ToolpathConstraintSet {
    /// Append one constraint.
    pub fn push(&mut self, constraint: Constraint) {
        self.constraints.push(constraint);
    }

    /// Return whether no constraints have been added.
    pub fn is_empty(&self) -> bool {
        self.constraints.is_empty()
    }
}

/// Build an exact path-length matching residual.
pub fn length_match_equation(
    name: impl Into<String>,
    current_length: Real,
    extra_length: VariableId,
    target_length: Real,
) -> Constraint {
    Constraint::equality(
        name,
        Expr::real(current_length) + Expr::symbol(extra_length.into(), "extra_length")
            - Expr::real(target_length),
    )
}

/// Build a linear feed/time residual `path_length - feed_rate * time = 0`.
pub fn constant_feed_time_equation(
    name: impl Into<String>,
    path_length: Real,
    feed_rate: Real,
    time: VariableId,
) -> Constraint {
    Constraint::equality(
        name,
        Expr::real(path_length) - Expr::real(feed_rate) * Expr::symbol(time.into(), "time"),
    )
}

/// Build the four-phase jerk-limited S-curve residual.
///
/// The retained profile is the symmetric rest-to-rest sequence `+j, -j, -j,
/// +j` with equal quarter-duration phases. For total time `T`, path length
/// `L`, and jerk magnitude `j`, exact integration gives `L = j*T^3/32`.
/// Returning the denominator-free residual `j*T^3 - 32*L = 0` keeps the
/// certification in Yap's exact predicate layer and avoids sampled controller
/// traces.
pub fn symmetric_jerk_limited_feed_time_equation(
    name: impl Into<String>,
    path_length: Real,
    jerk: Real,
    time: VariableId,
) -> Constraint {
    let time_expr = Expr::symbol(time.into(), "time");
    Constraint::equality(
        name,
        Expr::real(jerk) * time_expr.clone() * time_expr.clone() * time_expr
            - Expr::real(Real::from(32) * path_length),
    )
}

/// Build exact residuals for replaying one local curve-offset sample.
pub fn bezier_offset_sample_constraints(
    name_prefix: impl AsRef<str>,
    candidate: SolverPoint2,
    source_point: Point2,
    tangent: Point2,
    side_normal: Point2,
    distance_squared: Real,
) -> ToolpathConstraintSet {
    let prefix = name_prefix.as_ref();
    let dx = candidate.x_expr() - Expr::real(source_point.x);
    let dy = candidate.y_expr() - Expr::real(source_point.y);
    let tangent_x = Expr::real(tangent.x);
    let tangent_y = Expr::real(tangent.y);
    let normal_x = Expr::real(side_normal.x);
    let normal_y = Expr::real(side_normal.y);
    let mut set = ToolpathConstraintSet::default();
    set.push(Constraint::equality(
        format!("{prefix} offset distance"),
        dx.clone() * dx.clone() + dy.clone() * dy.clone() - Expr::real(distance_squared),
    ));
    set.push(Constraint::equality(
        format!("{prefix} tangent perpendicular"),
        dx.clone() * tangent_x + dy.clone() * tangent_y,
    ));
    set.push(Constraint {
        name: format!("{prefix} retained side"),
        kind: ConstraintKind::GreaterOrEqual,
        residual: dx * normal_x + dy * normal_y,
        weight: Real::one(),
        active: true,
    });
    set
}

/// Build an exact rectangular area replay residual.
pub fn rectangular_region_area_equation(
    name: impl Into<String>,
    region: RectangularRegion,
    expected_area: Real,
) -> Constraint {
    Constraint::equality(name, region.area_expr() - Expr::real(expected_area))
}

/// Build exact containment residuals for one rectangle inside another.
pub fn rectangular_region_containment_constraints(
    name_prefix: impl AsRef<str>,
    inner: RectangularRegion,
    outer: RectangularRegion,
) -> ToolpathConstraintSet {
    let prefix = name_prefix.as_ref();
    let mut set = ToolpathConstraintSet::default();
    set.push(greater_or_equal(
        format!("{prefix} min x inside"),
        Expr::real(inner.min.x) - Expr::real(outer.min.x),
    ));
    set.push(greater_or_equal(
        format!("{prefix} min y inside"),
        Expr::real(inner.min.y) - Expr::real(outer.min.y),
    ));
    set.push(greater_or_equal(
        format!("{prefix} max x inside"),
        Expr::real(outer.max.x) - Expr::real(inner.max.x),
    ));
    set.push(greater_or_equal(
        format!("{prefix} max y inside"),
        Expr::real(outer.max.y) - Expr::real(inner.max.y),
    ));
    set
}

/// Build an exact area-conservation residual for rectangular subtraction.
pub fn rectangular_difference_area_equation(
    name: impl Into<String>,
    subject: RectangularRegion,
    removed: Option<RectangularRegion>,
    remainder: impl IntoIterator<Item = RectangularRegion>,
) -> Constraint {
    let mut residual = subject.area_expr();
    if let Some(removed) = removed {
        residual = residual - removed.area_expr();
    }
    for piece in remainder {
        residual = residual - piece.area_expr();
    }
    Constraint::equality(name, residual)
}

fn rectangle_area_expr(min: &Point2, max: &Point2) -> Expr {
    (Expr::real(max.x.clone()) - Expr::real(min.x.clone()))
        * (Expr::real(max.y.clone()) - Expr::real(min.y.clone()))
}

fn greater_or_equal(name: impl Into<String>, residual: Expr) -> Constraint {
    Constraint {
        name: name.into(),
        kind: ConstraintKind::GreaterOrEqual,
        residual,
        weight: Real::one(),
        active: true,
    }
}
