//! Toolpath residual builders.
//!
//! CAM and EDA path planners often use heuristic or graph search to propose a
//! topology, then solve continuous parameters such as length extension, fillet
//! radius, feed constraints, or bead spacing. This module keeps the
//! `hypersolve` side to exact residual construction and replay. The accepted
//! geometry still belongs to domain predicates, following Yap's exact
//! geometric computation boundary and the path-planning separation used by
//! modern toolpath/routing literature.

use hyperlimit::Point2;
use hyperreal::Real;

use crate::domain::geometry::Point2 as SolverPoint2;
use crate::model::{Constraint, ConstraintKind, VariableId};
use crate::symbolic::Expr;

/// Exact rectangular region carrier for solver replay.
///
/// `hyperpath` owns path/CAM objects, but `hypersolve` should not depend on it.
/// This minimal value type lets callers replay rectangular CAM booleans through
/// exact residuals without introducing a crate cycle. The design follows Yap,
/// "Towards Exact Geometric Computation": keep exact geometric objects and
/// predicate results visible at package boundaries instead of reducing them to
/// lossy scalar samples.
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
///
/// The residual is `current_length + extra_length - target_length = 0`, where
/// `extra_length` is a solver variable and the path crate remains responsible
/// for proving that emitted geometry really has the requested length. This is
/// the same proposal/replay boundary used by `hyperpath` rectangular meanders.
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
///
/// This is a conservative exact placeholder for feed/speed stages: it keeps
/// the common constant-feed relation affine in the time variable, while later
/// PH-curve or acceleration-limited models can add richer polynomial blocks
/// only when their exact curve facts justify them.
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

/// Build exact residuals for replaying one local curve-offset sample.
///
/// General Bezier offsets are not polynomial Beziers, so `hyperpath` records a
/// local candidate with an exact source point, hodograph, side normal, and
/// squared distance. These residuals replay that candidate without sampling:
/// the offset point must be at the requested squared distance, perpendicular
/// to the tangent, and on the retained side-normal half-plane. This follows
/// Yap's exact predicate boundary and the staged Bezier-offset approaches of
/// Tiller-Hanson, Levien, and Blend2D/Yzerman: proposal and fitting are
/// separate from exact acceptance.
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
///
/// The residual is `width * height - expected_area = 0`. It is intentionally
/// not a geometric validity proof for arbitrary polygons; it is a cheap exact
/// replay row for rectangular support/infill clipping records produced by
/// `hyperpath`. This mirrors CGAL-style arrangement pipelines: topology
/// construction and area/accounting predicates remain separate, inspectable
/// stages.
pub fn rectangular_region_area_equation(
    name: impl Into<String>,
    region: RectangularRegion,
    expected_area: Real,
) -> Constraint {
    Constraint::equality(name, region.area_expr() - Expr::real(expected_area))
}

/// Build exact containment residuals for one rectangle inside another.
///
/// The generated inequalities are:
/// `inner.min.x - outer.min.x >= 0`,
/// `inner.min.y - outer.min.y >= 0`,
/// `outer.max.x - inner.max.x >= 0`, and
/// `outer.max.y - inner.max.y >= 0`.
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
///
/// For a retained rectangular boolean record, the residual is:
/// `area(subject) - area(removed) - sum(area(remainder_i)) = 0`.
/// It certifies accounting for the emitted pieces without requiring `hypersolve`
/// to understand the full arrangement topology.
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
