//! Exact rational-parameter Bezier/conic split scheduling.
//!
//! This module is an arrangement cleanup layer, not an intersection finder.
//! It accepts already-certified rational event parameters and emits retained
//! exact fragments. Polynomial Beziers become native sub-curves; rational
//! quadratic conics become homogeneous sub-curve records because restricting a
//! rational Bezier interval does not generally preserve the endpoint-weight
//! normalization used by [`crate::bezier::RationalQuadraticBezier`].

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal_with_policy};
use hyperreal::{Real, RealExactSetFacts};
use hypersolve::{
    AlgebraicRootPolynomialImageReport, AlgebraicRootPolynomialImageStatus,
    AlgebraicRootRepresentation, Constraint, Expr, PreparedProblem, Problem, RootIsolationConfig,
    represent_univariate_algebraic_roots, transform_algebraic_root_polynomial_image,
};

use crate::bezier::{BezierParameter, CubicBezier, QuadraticBezier, RationalQuadraticBezier};
use crate::provenance::PathProvenance;
use crate::segment::{Axis, LinePathSegment};

/// Errors while building retained Bezier arrangement fragments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierArrangementError {
    /// No source curve was supplied to arrange.
    EmptyInput,
    /// Parameter comparison could not be decided exactly.
    UndecidableParameterOrder,
    /// A rational conic homogeneous endpoint had zero weight.
    HomogeneousDenominatorFailure,
}

/// Certified class for a retained line segment against a quadratic Bezier.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineQuadraticBezierIntersectionClass {
    /// The segment and curve are certified disjoint.
    Disjoint,
    /// The line is tangent to the curve at one certified point.
    Tangent,
    /// The line crosses the curve at one certified point inside the segment bounds.
    OnePoint,
    /// The line crosses the curve at two certified points inside the segment bounds.
    TwoPoints,
    /// The segment and a degree-elevated linear quadratic Bezier overlap over a
    /// positive-length interval with certified endpoint witnesses.
    Overlap,
    /// The exact predicate package cannot certify the relation.
    Unknown,
}

/// Exact line/quadratic-Bezier event witness.
#[derive(Clone, Debug, PartialEq)]
pub struct LineQuadraticBezierIntersection {
    /// Exact Bezier parameter in `[0, 1]`.
    pub parameter: Real,
    /// Exact point on the retained Bezier and line segment.
    pub point: Point2,
}

/// Exact event report for an axis-aligned line segment and quadratic Bezier.
///
/// This is a discovered-event predicate for the mixed line/Bezier arrangement
/// work. For an axis-aligned retained line, one Bezier coordinate gives an
/// exact scalar quadratic `a t^2 + b t + c = 0`; roots are accepted only after
/// exact parameter-domain and segment-bound replay. This is the standard
/// implicit-line/substitution step used by Bezier arrangement algorithms, with
/// the Yap exact-computation rule applied directly: the report returns exact
/// witnesses or `Unknown`, never a tolerance-polyline approximation. See Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), and de Casteljau subdivision as used in Farouki, *Pythagorean
/// Hodograph Curves* (2008), for the retained-curve object discipline.
#[derive(Clone, Debug, PartialEq)]
pub struct LineQuadraticBezierIntersectionReport {
    /// Certified intersection class.
    pub class: LineQuadraticBezierIntersectionClass,
    /// Certified witnesses in increasing Bezier-parameter order.
    pub intersections: Vec<LineQuadraticBezierIntersection>,
}

/// Certified class for a retained line segment against a cubic Bezier.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierIntersectionClass {
    /// The segment and curve are certified disjoint.
    Disjoint,
    /// The line is tangent to the curve at one certified point.
    Tangent,
    /// The line crosses the curve at one certified point inside the segment bounds.
    OnePoint,
    /// The line crosses the curve at two certified points inside the segment bounds.
    TwoPoints,
    /// The line crosses the curve at three certified points inside the segment bounds.
    ThreePoints,
    /// The segment and a degree-elevated linear cubic Bezier overlap over a
    /// positive-length interval with certified endpoint witnesses.
    Overlap,
    /// The exact predicate package cannot certify the relation.
    Unknown,
}

/// Exact line/cubic-Bezier event witness.
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierIntersection {
    /// Exact cubic Bezier parameter in `[0, 1]`.
    pub parameter: Real,
    /// Exact point on the retained Bezier and line segment.
    pub point: Point2,
}

/// Exact interval-domain status for a represented line/cubic support root.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicAlgebraicRootDomain {
    /// The isolating interval is wholly inside the retained Bezier parameter domain `[0, 1]`.
    InsideUnitInterval,
    /// The isolating interval is wholly outside `[0, 1]`.
    OutsideUnitInterval,
    /// Exact interval comparison could not decide or the interval straddles a boundary.
    Unknown,
}

/// Exact segment-domain status for the algebraic image of a line/cubic root.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicAlgebraicPointDomain {
    /// The support equation and varying-coordinate image certify the point inside segment bounds.
    InsideSegmentBounds,
    /// The varying-coordinate image is certified outside the retained segment bounds.
    OutsideSegmentBounds,
    /// Image construction or exact interval comparison did not decide.
    Unknown,
}

/// Represented algebraic point image for a line/cubic support root.
///
/// The `x` and `y` fields are `hypersolve` polynomial-image reports for
/// `B_x(alpha)` and `B_y(alpha)`, where `alpha` is the represented cubic
/// support root. They are retained even when topology remains
/// [`LineCubicBezierIntersectionClass::Unknown`]. This is the Yap EGC
/// separation in concrete form: the exact algebraic object is carried forward,
/// while only certified order predicates may later turn it into a split event.
/// The image construction uses the resultant-based algebraic image described
/// by Sylvester (1853), Collins and Loos, "Real Zeros of Polynomials" (1982),
/// and Yap, "Towards Exact Geometric Computation" (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierAlgebraicPointImage {
    /// Exact represented image for the curve's x-coordinate at the support root.
    pub x: AlgebraicRootPolynomialImageReport,
    /// Exact represented image for the curve's y-coordinate at the support root.
    pub y: AlgebraicRootPolynomialImageReport,
    /// Certified relation of the algebraic point image to the retained line segment bounds.
    pub segment_domain: LineCubicAlgebraicPointDomain,
}

/// Represented algebraic root of the line/cubic support equation.
///
/// This is retained event evidence, not a split breakpoint. The parameter is
/// an algebraic object represented by its exact support polynomial and a
/// Sturm-isolated interval from `hypersolve`. Its point image is also retained
/// as exact resultant evidence; until a downstream scheduler consumes those
/// image intervals as topology, these roots keep the topological relation
/// [`LineCubicBezierIntersectionClass::Unknown`].
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierAlgebraicSupportRoot {
    /// Represented algebraic parameter root for the cubic support equation.
    pub parameter: AlgebraicRootRepresentation,
    /// Whether the root's isolating interval is certified inside `[0, 1]`.
    pub parameter_domain: LineCubicAlgebraicRootDomain,
    /// Represented algebraic point image for the root.
    pub point_image: LineCubicBezierAlgebraicPointImage,
}

/// Exact event report for an axis-aligned line segment and cubic Bezier.
///
/// The retained line is substituted into one cubic Bezier coordinate. Constant,
/// linear, and quadratic support polynomials are solved exactly and replayed
/// against the parameter and segment domains. True cubic support roots are
/// retained as represented algebraic parameters with exact algebraic point
/// images, but the topological class remains
/// [`LineCubicBezierIntersectionClass::Unknown`] until the mixed scheduler
/// consumes that image evidence as concrete breakpoints. Same-support
/// overlaps are promoted only for degree-elevated straight cubic images, where
/// endpoint coordinates invert to exact affine parameters.
///
/// This follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): unsupported algebraic discovery is reported instead
/// of sampled. The Bezier restriction/replay rows use the retained polynomial
/// object discipline described by de Casteljau subdivision and by Farouki,
/// *Pythagorean Hodograph Curves* (2008).
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierIntersectionReport {
    /// Certified intersection class.
    pub class: LineCubicBezierIntersectionClass,
    /// Certified witnesses in increasing cubic-parameter order.
    pub intersections: Vec<LineCubicBezierIntersection>,
    /// Represented algebraic support roots for true cubic equations.
    pub algebraic_support_roots: Vec<LineCubicBezierAlgebraicSupportRoot>,
}

/// Certified class for a retained line segment against a rational quadratic conic.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierIntersectionClass {
    /// The segment and conic are certified disjoint.
    Disjoint,
    /// The line is tangent to the conic at one certified point.
    Tangent,
    /// The line crosses the conic at one certified point inside the segment bounds.
    OnePoint,
    /// The line crosses the conic at two certified points inside the segment bounds.
    TwoPoints,
    /// The segment and a monotone rational-quadratic line image overlap over a
    /// positive-length interval with certified endpoint witnesses.
    Overlap,
    /// The exact predicate package cannot certify the relation.
    Unknown,
}

/// Exact line/rational-quadratic event witness.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierIntersection {
    /// Exact conic parameter in `[0, 1]`.
    pub parameter: Real,
    /// Exact affine point on the retained conic and line segment.
    pub point: Point2,
}

/// Exact event report for an axis-aligned line segment and rational quadratic conic.
///
/// The predicate substitutes the retained line coordinate into the homogeneous
/// conic equation before dividing by weight: for a horizontal line, for example,
/// it solves `Y(t) - y_line W(t) = 0` as an exact scalar quadratic. Candidate
/// roots are accepted only after parameter-domain, nonzero-weight, and
/// segment-bound replay. Same-support overlaps are promoted only for certified
/// monotone rational line images, where exact endpoint parameters can be
/// recovered by replaying the one-dimensional rational equation. This follows
/// Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), by keeping the exact
/// conic object and returning `Unknown` instead of flattening or dividing
/// undecidable denominators. The homogeneous construction is the standard
/// rational Bezier/conic representation described by Farouki, *Pythagorean
/// Hodograph Curves* (2008).
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierIntersectionReport {
    /// Certified intersection class.
    pub class: LineRationalQuadraticBezierIntersectionClass,
    /// Certified witnesses in increasing conic-parameter order.
    pub intersections: Vec<LineRationalQuadraticBezierIntersection>,
}

/// Exact breakpoint on one retained Bezier/conic source.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierArrangementBreakpoint {
    /// Source curve index.
    pub source: usize,
    /// Exact rational source parameter.
    pub parameter: BezierParameter,
}

/// Exact quadratic Bezier sub-curve fragment.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezierArrangementFragment {
    /// Source curve index.
    pub source: usize,
    /// Start breakpoint.
    pub start: BezierArrangementBreakpoint,
    /// End breakpoint.
    pub end: BezierArrangementBreakpoint,
    /// Retained exact sub-curve.
    pub curve: QuadraticBezier,
}

/// Exact cubic Bezier sub-curve fragment.
#[derive(Clone, Debug, PartialEq)]
pub struct CubicBezierArrangementFragment {
    /// Source curve index.
    pub source: usize,
    /// Start breakpoint.
    pub start: BezierArrangementBreakpoint,
    /// End breakpoint.
    pub end: BezierArrangementBreakpoint,
    /// Retained exact sub-curve.
    pub curve: CubicBezier,
}

/// Homogeneous control point for a rational quadratic sub-curve.
#[derive(Clone, Debug, PartialEq)]
pub struct HomogeneousPoint2 {
    /// Weighted x coordinate.
    pub x: Real,
    /// Weighted y coordinate.
    pub y: Real,
    /// Homogeneous weight.
    pub w: Real,
}

/// Exact homogeneous rational-quadratic sub-curve fragment.
///
/// The fragment stores homogeneous Bernstein controls `(X, Y, W)` directly.
/// That is the exact conic object produced by de Casteljau restriction. It
/// avoids pretending every restricted conic can be represented by the
/// normalized endpoint weights of [`RationalQuadraticBezier`].
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBezierArrangementFragment {
    /// Source curve index.
    pub source: usize,
    /// Start breakpoint.
    pub start: BezierArrangementBreakpoint,
    /// End breakpoint.
    pub end: BezierArrangementBreakpoint,
    /// Homogeneous start control.
    pub start_control: HomogeneousPoint2,
    /// Homogeneous middle control.
    pub control: HomogeneousPoint2,
    /// Homogeneous end control.
    pub end_control: HomogeneousPoint2,
}

/// Exact split report for a set of quadratic Beziers.
///
/// The construction uses de Casteljau's affine subdivision identities. For a
/// subinterval `[a,b]`, the fragment is `B(a + (b-a)u)`. Its control points are
/// recovered from exact endpoint and derivative data. This is the same
/// retained-object discipline described by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): event parameters are
/// exact objects, and no sampled polyline is introduced before topology is
/// certified.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezierArrangementReport {
    /// Retained source curves.
    pub curves: Vec<QuadraticBezier>,
    /// Sorted breakpoints per source curve.
    pub breakpoints: Vec<Vec<BezierArrangementBreakpoint>>,
    /// Positive-length exact fragments.
    pub fragments: Vec<QuadraticBezierArrangementFragment>,
    /// Exact-set facts across emitted fragment control points.
    pub fragment_exact: RealExactSetFacts,
    /// Source provenance for the schedule.
    pub provenance: PathProvenance,
}

/// Exact split report for a set of cubic Beziers.
#[derive(Clone, Debug, PartialEq)]
pub struct CubicBezierArrangementReport {
    /// Retained source curves.
    pub curves: Vec<CubicBezier>,
    /// Sorted breakpoints per source curve.
    pub breakpoints: Vec<Vec<BezierArrangementBreakpoint>>,
    /// Positive-length exact fragments.
    pub fragments: Vec<CubicBezierArrangementFragment>,
    /// Exact-set facts across emitted fragment control points.
    pub fragment_exact: RealExactSetFacts,
    /// Source provenance for the schedule.
    pub provenance: PathProvenance,
}

/// Exact homogeneous split report for a set of rational quadratic Beziers.
///
/// Homogeneous de Casteljau subdivision is the standard exact carrier used by
/// rational Bezier/conic arrangement kernels, including the CGAL-style
/// object/predicate split. Farouki's rational-curve treatment similarly works
/// in homogeneous coordinates before quotient evaluation. This report exposes
/// those controls directly so later conic overlap promotion does not lose
/// exactness through endpoint-weight renormalization.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBezierArrangementReport {
    /// Retained source curves.
    pub curves: Vec<RationalQuadraticBezier>,
    /// Sorted breakpoints per source curve.
    pub breakpoints: Vec<Vec<BezierArrangementBreakpoint>>,
    /// Positive-length exact homogeneous fragments.
    pub fragments: Vec<RationalQuadraticBezierArrangementFragment>,
    /// Exact-set facts across emitted homogeneous controls.
    pub fragment_exact: RealExactSetFacts,
    /// Source provenance for the schedule.
    pub provenance: PathProvenance,
}

/// Arrange quadratic Beziers at exact rational event parameters.
pub fn arrange_quadratic_beziers(
    curves: &[QuadraticBezier],
    events: &[Vec<BezierParameter>],
    policy: PredicatePolicy,
) -> Result<QuadraticBezierArrangementReport, BezierArrangementError> {
    arrange_quadratic_beziers_with_provenance(curves, events, policy, PathProvenance::native())
}

/// Arrange quadratic Beziers at exact rational parameters with provenance.
pub fn arrange_quadratic_beziers_with_provenance(
    curves: &[QuadraticBezier],
    events: &[Vec<BezierParameter>],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<QuadraticBezierArrangementReport, BezierArrangementError> {
    validate_inputs(curves.len(), events.len())?;
    let breakpoints = sorted_breakpoints(events, policy)?;
    let fragments = build_quadratic_fragments(curves, &breakpoints, policy)?;
    let fragment_exact = quadratic_fragment_facts(&fragments);
    Ok(QuadraticBezierArrangementReport {
        curves: curves.to_vec(),
        breakpoints,
        fragments,
        fragment_exact,
        provenance,
    })
}

/// Arrange cubic Beziers at exact rational event parameters.
pub fn arrange_cubic_beziers(
    curves: &[CubicBezier],
    events: &[Vec<BezierParameter>],
    policy: PredicatePolicy,
) -> Result<CubicBezierArrangementReport, BezierArrangementError> {
    arrange_cubic_beziers_with_provenance(curves, events, policy, PathProvenance::native())
}

/// Arrange cubic Beziers at exact rational parameters with provenance.
pub fn arrange_cubic_beziers_with_provenance(
    curves: &[CubicBezier],
    events: &[Vec<BezierParameter>],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<CubicBezierArrangementReport, BezierArrangementError> {
    validate_inputs(curves.len(), events.len())?;
    let breakpoints = sorted_breakpoints(events, policy)?;
    let fragments = build_cubic_fragments(curves, &breakpoints, policy)?;
    let fragment_exact = cubic_fragment_facts(&fragments);
    Ok(CubicBezierArrangementReport {
        curves: curves.to_vec(),
        breakpoints,
        fragments,
        fragment_exact,
        provenance,
    })
}

/// Arrange rational quadratic Beziers at exact rational event parameters.
pub fn arrange_rational_quadratic_beziers(
    curves: &[RationalQuadraticBezier],
    events: &[Vec<BezierParameter>],
    policy: PredicatePolicy,
) -> Result<RationalQuadraticBezierArrangementReport, BezierArrangementError> {
    arrange_rational_quadratic_beziers_with_provenance(
        curves,
        events,
        policy,
        PathProvenance::native(),
    )
}

/// Arrange rational quadratic Beziers at exact rational parameters with provenance.
pub fn arrange_rational_quadratic_beziers_with_provenance(
    curves: &[RationalQuadraticBezier],
    events: &[Vec<BezierParameter>],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<RationalQuadraticBezierArrangementReport, BezierArrangementError> {
    validate_inputs(curves.len(), events.len())?;
    let breakpoints = sorted_breakpoints(events, policy)?;
    let fragments = build_rational_quadratic_fragments(curves, &breakpoints, policy)?;
    let fragment_exact = rational_quadratic_fragment_facts(&fragments);
    Ok(RationalQuadraticBezierArrangementReport {
        curves: curves.to_vec(),
        breakpoints,
        fragments,
        fragment_exact,
        provenance,
    })
}

/// Intersect an axis-aligned line segment with a quadratic Bezier exactly.
///
/// The returned witnesses are exact `Real` parameter/point objects. A retained
/// horizontal segment substitutes `B_y(t) = y_line`; a retained vertical segment
/// substitutes `B_x(t) = x_line`. The resulting scalar quadratic is solved in
/// the object layer and every candidate root is replayed against `[0, 1]` and
/// the closed segment bounds before it becomes topology. If a support-line
/// overlap is a degree-elevated line segment, the overlap interval is promoted
/// to exact endpoint witnesses. Nonlinear collinear Bezier images still return
/// [`LineQuadraticBezierIntersectionClass::Unknown`] because they need a later
/// exact inverse-parameter construction.
///
/// This follows Yap's "Towards Exact Geometric Computation" rule that
/// geometric decisions should be made by exact predicates over retained
/// objects, not sampled approximations. The Bezier substitution is the standard
/// Bernstein-polynomial line incidence test used in curve arrangement kernels;
/// see also Farouki, *Pythagorean Hodograph Curves* (2008), for preserving
/// polynomial curve objects through exact downstream processing.
pub fn intersect_axis_aligned_line_quadratic_bezier(
    segment: &LinePathSegment,
    curve: &QuadraticBezier,
    policy: PredicatePolicy,
) -> LineQuadraticBezierIntersectionReport {
    let Some(axis) = segment.facts().axis_aligned else {
        return line_quadratic_unknown_report();
    };
    let fixed = match axis {
        Axis::X => segment.start().y.clone(),
        Axis::Y => segment.start().x.clone(),
    };
    let roots = match solve_quadratic_coordinate_roots(curve, axis, fixed.clone(), policy) {
        Some(roots) => roots,
        None => {
            return quadratic_line_overlap_report(segment, curve, axis, fixed, policy)
                .unwrap_or_else(line_quadratic_unknown_report);
        }
    };
    let mut intersections = Vec::new();
    for parameter in roots {
        match parameter_in_unit_interval(&parameter, policy) {
            Some(true) => {}
            Some(false) => continue,
            None => return line_quadratic_unknown_report(),
        }
        let point = eval_quadratic_at_real(curve, &parameter);
        match point_inside_segment_bounds(&point, segment, policy) {
            Some(true) => {}
            Some(false) => continue,
            None => return line_quadratic_unknown_report(),
        }
        if push_unique_intersection(&mut intersections, parameter, point, policy).is_none() {
            return line_quadratic_unknown_report();
        }
    }
    if sort_line_quadratic_intersections(&mut intersections, policy).is_none() {
        return line_quadratic_unknown_report();
    }
    let class = match intersections.len() {
        0 => LineQuadraticBezierIntersectionClass::Disjoint,
        1 => match roots_are_tangent(curve, axis, segment, policy) {
            Some(true) => LineQuadraticBezierIntersectionClass::Tangent,
            Some(false) => LineQuadraticBezierIntersectionClass::OnePoint,
            None => return line_quadratic_unknown_report(),
        },
        2 => LineQuadraticBezierIntersectionClass::TwoPoints,
        _ => LineQuadraticBezierIntersectionClass::Unknown,
    };
    LineQuadraticBezierIntersectionReport {
        class,
        intersections,
    }
}

/// Intersect an axis-aligned line segment with a cubic Bezier exactly where
/// the retained support equation has degree at most two.
pub fn intersect_axis_aligned_line_cubic_bezier(
    segment: &LinePathSegment,
    curve: &CubicBezier,
    policy: PredicatePolicy,
) -> LineCubicBezierIntersectionReport {
    let Some(axis) = segment.facts().axis_aligned else {
        return line_cubic_unknown_report();
    };
    let fixed = match axis {
        Axis::X => segment.start().y.clone(),
        Axis::Y => segment.start().x.clone(),
    };
    let roots =
        match solve_cubic_coordinate_roots_up_to_quadratic(curve, axis, fixed.clone(), policy) {
            Some(roots) => roots,
            None => {
                return degree_elevated_cubic_line_overlap_report(
                    segment,
                    curve,
                    axis,
                    fixed.clone(),
                    policy,
                )
                .unwrap_or_else(|| {
                    true_cubic_algebraic_support_report(segment, curve, axis, fixed, policy)
                });
            }
        };
    let mut intersections = Vec::new();
    for parameter in roots {
        match parameter_in_unit_interval(&parameter, policy) {
            Some(true) => {}
            Some(false) => continue,
            None => return line_cubic_unknown_report(),
        }
        let point = eval_cubic_at_real(curve, &parameter);
        match point_inside_segment_bounds(&point, segment, policy) {
            Some(true) => {}
            Some(false) => continue,
            None => return line_cubic_unknown_report(),
        }
        if push_unique_cubic_intersection(&mut intersections, parameter, point, policy).is_none() {
            return line_cubic_unknown_report();
        }
    }
    if sort_cubic_intersections(&mut intersections, policy).is_none() {
        return line_cubic_unknown_report();
    }
    let class = match intersections.len() {
        0 => LineCubicBezierIntersectionClass::Disjoint,
        1 => match cubic_roots_are_tangent_up_to_quadratic(curve, axis, segment, policy) {
            Some(true) => LineCubicBezierIntersectionClass::Tangent,
            Some(false) => LineCubicBezierIntersectionClass::OnePoint,
            None => return line_cubic_unknown_report(),
        },
        2 => LineCubicBezierIntersectionClass::TwoPoints,
        3 => LineCubicBezierIntersectionClass::ThreePoints,
        _ => LineCubicBezierIntersectionClass::Unknown,
    };
    LineCubicBezierIntersectionReport {
        class,
        intersections,
        algebraic_support_roots: Vec::new(),
    }
}

/// Intersect an axis-aligned line segment with a rational quadratic conic exactly.
pub fn intersect_axis_aligned_line_rational_quadratic_bezier(
    segment: &LinePathSegment,
    curve: &RationalQuadraticBezier,
    policy: PredicatePolicy,
) -> LineRationalQuadraticBezierIntersectionReport {
    let Some(axis) = segment.facts().axis_aligned else {
        return line_rational_quadratic_unknown_report();
    };
    let fixed = match axis {
        Axis::X => segment.start().y.clone(),
        Axis::Y => segment.start().x.clone(),
    };
    let roots = match solve_rational_quadratic_coordinate_roots(curve, axis, fixed.clone(), policy)
    {
        Some(roots) => roots,
        None => {
            return rational_quadratic_line_overlap_report(segment, curve, axis, fixed, policy)
                .unwrap_or_else(line_rational_quadratic_unknown_report);
        }
    };
    let mut intersections = Vec::new();
    for parameter in roots {
        match parameter_in_unit_interval(&parameter, policy) {
            Some(true) => {}
            Some(false) => continue,
            None => return line_rational_quadratic_unknown_report(),
        }
        let Some(point) = eval_rational_quadratic_at_real(curve, &parameter, policy) else {
            return line_rational_quadratic_unknown_report();
        };
        match point_inside_segment_bounds(&point, segment, policy) {
            Some(true) => {}
            Some(false) => continue,
            None => return line_rational_quadratic_unknown_report(),
        }
        if push_unique_rational_quadratic_intersection(&mut intersections, parameter, point, policy)
            .is_none()
        {
            return line_rational_quadratic_unknown_report();
        }
    }
    if sort_rational_quadratic_intersections(&mut intersections, policy).is_none() {
        return line_rational_quadratic_unknown_report();
    }
    let class = match intersections.len() {
        0 => LineRationalQuadraticBezierIntersectionClass::Disjoint,
        1 => match rational_quadratic_roots_are_tangent(curve, axis, segment, policy) {
            Some(true) => LineRationalQuadraticBezierIntersectionClass::Tangent,
            Some(false) => LineRationalQuadraticBezierIntersectionClass::OnePoint,
            None => return line_rational_quadratic_unknown_report(),
        },
        2 => LineRationalQuadraticBezierIntersectionClass::TwoPoints,
        _ => LineRationalQuadraticBezierIntersectionClass::Unknown,
    };
    LineRationalQuadraticBezierIntersectionReport {
        class,
        intersections,
    }
}

fn validate_inputs(curves_len: usize, events_len: usize) -> Result<(), BezierArrangementError> {
    if curves_len == 0 {
        return Err(BezierArrangementError::EmptyInput);
    }
    if curves_len != events_len {
        return Err(BezierArrangementError::EmptyInput);
    }
    Ok(())
}

fn sorted_breakpoints(
    events: &[Vec<BezierParameter>],
    policy: PredicatePolicy,
) -> Result<Vec<Vec<BezierArrangementBreakpoint>>, BezierArrangementError> {
    events
        .iter()
        .enumerate()
        .map(|(source, source_events)| {
            let mut points = vec![
                BezierArrangementBreakpoint {
                    source,
                    parameter: BezierParameter::new(0, 1).expect("valid zero parameter"),
                },
                BezierArrangementBreakpoint {
                    source,
                    parameter: BezierParameter::new(1, 1).expect("valid one parameter"),
                },
            ];
            for parameter in source_events {
                insert_breakpoint(
                    &mut points,
                    BezierArrangementBreakpoint {
                        source,
                        parameter: *parameter,
                    },
                    policy,
                )?;
            }
            Ok(points)
        })
        .collect()
}

fn insert_breakpoint(
    points: &mut Vec<BezierArrangementBreakpoint>,
    point: BezierArrangementBreakpoint,
    policy: PredicatePolicy,
) -> Result<(), BezierArrangementError> {
    for index in 0..points.len() {
        match compare_parameters(point.parameter, points[index].parameter, policy)? {
            Ordering::Less => {
                points.insert(index, point);
                return Ok(());
            }
            Ordering::Equal => return Ok(()),
            Ordering::Greater => {}
        }
    }
    points.push(point);
    Ok(())
}

fn compare_parameters(
    left: BezierParameter,
    right: BezierParameter,
    policy: PredicatePolicy,
) -> Result<Ordering, BezierArrangementError> {
    compare_reals_with_policy(&left.to_real(), &right.to_real(), policy)
        .value()
        .ok_or(BezierArrangementError::UndecidableParameterOrder)
}

fn build_quadratic_fragments(
    curves: &[QuadraticBezier],
    breakpoints: &[Vec<BezierArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<Vec<QuadraticBezierArrangementFragment>, BezierArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            if compare_parameters(window[0].parameter, window[1].parameter, policy)?
                == Ordering::Equal
            {
                continue;
            }
            let source = &curves[window[0].source];
            fragments.push(QuadraticBezierArrangementFragment {
                source: window[0].source,
                start: window[0].clone(),
                end: window[1].clone(),
                curve: quadratic_subcurve(source, window[0].parameter, window[1].parameter)?,
            });
        }
    }
    Ok(fragments)
}

fn build_cubic_fragments(
    curves: &[CubicBezier],
    breakpoints: &[Vec<BezierArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<Vec<CubicBezierArrangementFragment>, BezierArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            if compare_parameters(window[0].parameter, window[1].parameter, policy)?
                == Ordering::Equal
            {
                continue;
            }
            let source = &curves[window[0].source];
            fragments.push(CubicBezierArrangementFragment {
                source: window[0].source,
                start: window[0].clone(),
                end: window[1].clone(),
                curve: cubic_subcurve(source, window[0].parameter, window[1].parameter)?,
            });
        }
    }
    Ok(fragments)
}

fn build_rational_quadratic_fragments(
    curves: &[RationalQuadraticBezier],
    breakpoints: &[Vec<BezierArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<Vec<RationalQuadraticBezierArrangementFragment>, BezierArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            if compare_parameters(window[0].parameter, window[1].parameter, policy)?
                == Ordering::Equal
            {
                continue;
            }
            let source = &curves[window[0].source];
            fragments.push(rational_quadratic_subcurve(source, &window[0], &window[1])?);
        }
    }
    Ok(fragments)
}

fn quadratic_subcurve(
    curve: &QuadraticBezier,
    start: BezierParameter,
    end: BezierParameter,
) -> Result<QuadraticBezier, BezierArrangementError> {
    let start_point = curve.eval(start);
    let end_point = curve.eval(end);
    let delta = end.to_real() - start.to_real();
    let start_derivative = curve.derivative(start);
    let half_dx = div_real(delta.clone() * start_derivative.x, Real::from(2))?;
    let half_dy = div_real(delta * start_derivative.y, Real::from(2))?;
    let control = Point2::new(
        start_point.x.clone() + half_dx,
        start_point.y.clone() + half_dy,
    );
    Ok(QuadraticBezier::with_provenance(
        start_point,
        control,
        end_point,
        curve.provenance(),
    ))
}

fn cubic_subcurve(
    curve: &CubicBezier,
    start: BezierParameter,
    end: BezierParameter,
) -> Result<CubicBezier, BezierArrangementError> {
    let start_point = curve.eval(start);
    let end_point = curve.eval(end);
    let delta = end.to_real() - start.to_real();
    let start_derivative = curve.derivative(start);
    let end_derivative = curve.derivative(end);
    let third_start_dx = div_real(delta.clone() * start_derivative.x, Real::from(3))?;
    let third_start_dy = div_real(delta.clone() * start_derivative.y, Real::from(3))?;
    let third_end_dx = div_real(delta.clone() * end_derivative.x, Real::from(3))?;
    let third_end_dy = div_real(delta * end_derivative.y, Real::from(3))?;
    let control0 = Point2::new(
        start_point.x.clone() + third_start_dx,
        start_point.y.clone() + third_start_dy,
    );
    let control1 = Point2::new(
        end_point.x.clone() - third_end_dx,
        end_point.y.clone() - third_end_dy,
    );
    Ok(CubicBezier::with_provenance(
        start_point,
        control0,
        control1,
        end_point,
        curve.provenance(),
    ))
}

fn rational_quadratic_subcurve(
    curve: &RationalQuadraticBezier,
    start: &BezierArrangementBreakpoint,
    end: &BezierArrangementBreakpoint,
) -> Result<RationalQuadraticBezierArrangementFragment, BezierArrangementError> {
    let start_control = homogeneous_eval(curve, start.parameter);
    let end_control = homogeneous_eval(curve, end.parameter);
    let delta = end.parameter.to_real() - start.parameter.to_real();
    let derivative = homogeneous_derivative(curve, start.parameter);
    let half_dx = div_real(delta.clone() * derivative.x, Real::from(2))?;
    let half_dy = div_real(delta.clone() * derivative.y, Real::from(2))?;
    let half_dw = div_real(delta * derivative.w, Real::from(2))?;
    let control = HomogeneousPoint2 {
        x: start_control.x.clone() + half_dx,
        y: start_control.y.clone() + half_dy,
        w: start_control.w.clone() + half_dw,
    };
    Ok(RationalQuadraticBezierArrangementFragment {
        source: start.source,
        start: start.clone(),
        end: end.clone(),
        start_control,
        control,
        end_control,
    })
}

fn homogeneous_eval(
    curve: &RationalQuadraticBezier,
    parameter: BezierParameter,
) -> HomogeneousPoint2 {
    let t = parameter.to_real();
    let one_minus_t = Real::one() - t.clone();
    let b0 = one_minus_t.clone() * one_minus_t.clone();
    let b1 = Real::from(2) * one_minus_t * t.clone();
    let b2 = t.clone() * t;
    let weighted_b1 = b1 * curve.control_weight().clone();
    HomogeneousPoint2 {
        x: curve.start().x.clone() * b0.clone()
            + curve.control().x.clone() * weighted_b1.clone()
            + curve.end().x.clone() * b2.clone(),
        y: curve.start().y.clone() * b0.clone()
            + curve.control().y.clone() * weighted_b1.clone()
            + curve.end().y.clone() * b2.clone(),
        w: b0 + weighted_b1 + b2,
    }
}

fn homogeneous_derivative(
    curve: &RationalQuadraticBezier,
    parameter: BezierParameter,
) -> HomogeneousPoint2 {
    let t = parameter.to_real();
    let db0 = -Real::from(2) * (Real::one() - t.clone());
    let db1 = Real::from(2) * (Real::one() - Real::from(2) * t.clone());
    let db2 = Real::from(2) * t;
    let weighted_db1 = db1 * curve.control_weight().clone();
    HomogeneousPoint2 {
        x: curve.start().x.clone() * db0.clone()
            + curve.control().x.clone() * weighted_db1.clone()
            + curve.end().x.clone() * db2.clone(),
        y: curve.start().y.clone() * db0.clone()
            + curve.control().y.clone() * weighted_db1.clone()
            + curve.end().y.clone() * db2.clone(),
        w: db0 + weighted_db1 + db2,
    }
}

fn quadratic_fragment_facts(fragments: &[QuadraticBezierArrangementFragment]) -> RealExactSetFacts {
    Real::exact_set_facts(
        fragments
            .iter()
            .flat_map(|fragment| {
                [
                    &fragment.curve.start().x,
                    &fragment.curve.start().y,
                    &fragment.curve.control().x,
                    &fragment.curve.control().y,
                    &fragment.curve.end().x,
                    &fragment.curve.end().y,
                ]
            })
            .collect::<Vec<_>>(),
    )
}

fn cubic_fragment_facts(fragments: &[CubicBezierArrangementFragment]) -> RealExactSetFacts {
    Real::exact_set_facts(
        fragments
            .iter()
            .flat_map(|fragment| {
                [
                    &fragment.curve.start().x,
                    &fragment.curve.start().y,
                    &fragment.curve.control0().x,
                    &fragment.curve.control0().y,
                    &fragment.curve.control1().x,
                    &fragment.curve.control1().y,
                    &fragment.curve.end().x,
                    &fragment.curve.end().y,
                ]
            })
            .collect::<Vec<_>>(),
    )
}

fn rational_quadratic_fragment_facts(
    fragments: &[RationalQuadraticBezierArrangementFragment],
) -> RealExactSetFacts {
    Real::exact_set_facts(
        fragments
            .iter()
            .flat_map(|fragment| {
                [
                    &fragment.start_control.x,
                    &fragment.start_control.y,
                    &fragment.start_control.w,
                    &fragment.control.x,
                    &fragment.control.y,
                    &fragment.control.w,
                    &fragment.end_control.x,
                    &fragment.end_control.y,
                    &fragment.end_control.w,
                ]
            })
            .collect::<Vec<_>>(),
    )
}

fn div_real(numerator: Real, denominator: Real) -> Result<Real, BezierArrangementError> {
    (numerator / denominator).map_err(|_| BezierArrangementError::HomogeneousDenominatorFailure)
}

fn solve_quadratic_coordinate_roots(
    curve: &QuadraticBezier,
    axis: Axis,
    fixed: Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let p0 = coordinate(curve.start(), axis);
    let p1 = coordinate(curve.control(), axis);
    let p2 = coordinate(curve.end(), axis);
    let a = p0.clone() - Real::from(2) * p1.clone() + p2.clone();
    let b = Real::from(2) * (p1 - p0.clone());
    let c = p0 - fixed;
    match compare_reals_with_policy(&a, &Real::zero(), policy).value()? {
        Ordering::Equal => solve_linear_root(b, c, policy),
        Ordering::Less | Ordering::Greater => solve_quadratic_roots(a, b, c, policy),
    }
}

fn solve_rational_quadratic_coordinate_roots(
    curve: &RationalQuadraticBezier,
    axis: Axis,
    fixed: Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let q0 = rational_conic_support_coefficient(curve.start(), &Real::one(), axis, &fixed);
    let q1 =
        rational_conic_support_coefficient(curve.control(), curve.control_weight(), axis, &fixed);
    let q2 = rational_conic_support_coefficient(curve.end(), &Real::one(), axis, &fixed);
    let a = q0.clone() - Real::from(2) * q1.clone() + q2.clone();
    let b = Real::from(2) * (q1 - q0.clone());
    let c = q0;
    match compare_reals_with_policy(&a, &Real::zero(), policy).value()? {
        Ordering::Equal => solve_linear_root(b, c, policy),
        Ordering::Less | Ordering::Greater => solve_quadratic_roots(a, b, c, policy),
    }
}

fn solve_cubic_coordinate_roots_up_to_quadratic(
    curve: &CubicBezier,
    axis: Axis,
    fixed: Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let (a, b, c, d) = cubic_coordinate_polynomial(curve, axis, fixed);
    match compare_reals_with_policy(&a, &Real::zero(), policy).value()? {
        Ordering::Equal => match compare_reals_with_policy(&b, &Real::zero(), policy).value()? {
            Ordering::Equal => solve_linear_root(c, d, policy),
            Ordering::Less | Ordering::Greater => solve_quadratic_roots(b, c, d, policy),
        },
        Ordering::Less | Ordering::Greater => None,
    }
}

fn cubic_coordinate_polynomial(
    curve: &CubicBezier,
    axis: Axis,
    fixed: Real,
) -> (Real, Real, Real, Real) {
    let p0 = coordinate(curve.start(), axis);
    let p1 = coordinate(curve.control0(), axis);
    let p2 = coordinate(curve.control1(), axis);
    let p3 = coordinate(curve.end(), axis);
    let a = -p0.clone() + Real::from(3) * p1.clone() - Real::from(3) * p2.clone() + p3;
    let b = Real::from(3) * p0.clone() - Real::from(6) * p1.clone() + Real::from(3) * p2;
    let c = Real::from(3) * (p1 - p0.clone());
    let d = p0 - fixed;
    (a, b, c, d)
}

#[derive(Clone, Copy)]
enum PointCoordinate {
    X,
    Y,
}

fn cubic_point_coordinate_polynomial(
    curve: &CubicBezier,
    coordinate: PointCoordinate,
) -> Vec<Real> {
    let p0 = point_coordinate(curve.start(), coordinate);
    let p1 = point_coordinate(curve.control0(), coordinate);
    let p2 = point_coordinate(curve.control1(), coordinate);
    let p3 = point_coordinate(curve.end(), coordinate);
    let a = -p0.clone() + Real::from(3) * p1.clone() - Real::from(3) * p2.clone() + p3;
    let b = Real::from(3) * p0.clone() - Real::from(6) * p1.clone() + Real::from(3) * p2;
    let c = Real::from(3) * (p1 - p0.clone());
    vec![p0, c, b, a]
}

fn point_coordinate(point: &Point2, coordinate: PointCoordinate) -> Real {
    match coordinate {
        PointCoordinate::X => point.x.clone(),
        PointCoordinate::Y => point.y.clone(),
    }
}

fn true_cubic_algebraic_support_report(
    segment: &LinePathSegment,
    curve: &CubicBezier,
    axis: Axis,
    fixed: Real,
    policy: PredicatePolicy,
) -> LineCubicBezierIntersectionReport {
    // True cubic support equations cross the boundary where a `Real`
    // parameter witness may not be orderable enough for arrangement splitting.
    // We still retain the exact event evidence by delegating to `hypersolve`'s
    // Sturm isolator and algebraic-root representation. Sturm's theorem
    // (1835) supplies the distinct-root interval proof; Yap (1997) supplies the
    // discipline used here: report the exact algebraic object, but do not turn
    // it into topology until later predicates can order its point image.
    let (a, b, c, d) = cubic_coordinate_polynomial(curve, axis, fixed);
    match compare_reals_with_policy(&a, &Real::zero(), policy).value() {
        Some(Ordering::Less | Ordering::Greater) => {}
        Some(Ordering::Equal) | None => return line_cubic_unknown_report(),
    }
    let mut problem = Problem::default();
    let parameter = problem.add_variable("line_cubic_parameter", Real::zero());
    let t = Expr::symbol(parameter.into(), "line_cubic_parameter");
    let residual = Expr::real(d)
        + Expr::real(c) * t.clone()
        + Expr::real(b) * t.clone().powi(2)
        + Expr::real(a) * t.powi(3);
    problem.add_constraint(Constraint::equality("line cubic support root", residual));
    let prepared = PreparedProblem::new(&problem);
    let roots = represent_univariate_algebraic_roots(
        &prepared,
        RootIsolationConfig {
            policy,
            max_interval_width: Some((Real::one() / Real::from(1024)).expect("nonzero width")),
            max_refinement_steps: 64,
            ..RootIsolationConfig::default()
        },
    )
    .into_iter()
    .flat_map(|report| report.roots)
    .map(|root| {
        let point_image = cubic_algebraic_point_image(curve, segment, axis, &root, policy);
        LineCubicBezierAlgebraicSupportRoot {
            parameter_domain: classify_algebraic_root_unit_domain(&root, policy),
            parameter: root,
            point_image,
        }
    })
    .collect();
    LineCubicBezierIntersectionReport {
        class: LineCubicBezierIntersectionClass::Unknown,
        intersections: Vec::new(),
        algebraic_support_roots: roots,
    }
}

fn cubic_algebraic_point_image(
    curve: &CubicBezier,
    segment: &LinePathSegment,
    axis: Axis,
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> LineCubicBezierAlgebraicPointImage {
    // `hypersolve` constructs `q(alpha)` by eliminating the source parameter
    // with a Sylvester resultant and then validating the mapped isolating
    // interval. We use it independently for x and y so later path scheduling
    // can replay bound checks without sampling the cubic. See Sylvester
    // (1853), Collins and Loos (1982), and Yap (1997).
    let x = transform_algebraic_root_polynomial_image(
        root,
        &cubic_point_coordinate_polynomial(curve, PointCoordinate::X),
        policy,
    );
    let y = transform_algebraic_root_polynomial_image(
        root,
        &cubic_point_coordinate_polynomial(curve, PointCoordinate::Y),
        policy,
    );
    let segment_domain = classify_algebraic_point_segment_domain(&x, &y, segment, axis, policy);
    LineCubicBezierAlgebraicPointImage {
        x,
        y,
        segment_domain,
    }
}

fn classify_algebraic_point_segment_domain(
    x: &AlgebraicRootPolynomialImageReport,
    y: &AlgebraicRootPolynomialImageReport,
    segment: &LinePathSegment,
    axis: Axis,
    policy: PredicatePolicy,
) -> LineCubicAlgebraicPointDomain {
    // The fixed coordinate is certified by the support equation itself. Its
    // transformed image may still be a narrow isolating interval around the
    // line constant, so segment-bound clipping uses the varying coordinate
    // image and the retained axis-aligned segment bounds.
    let varying_image = match axis {
        Axis::X => x,
        Axis::Y => y,
    };
    let Some(varying_representation) = transformed_image_representation(varying_image) else {
        return LineCubicAlgebraicPointDomain::Unknown;
    };
    let (bound_min, bound_max) = match axis {
        Axis::X => (&segment.bounds_min().x, &segment.bounds_max().x),
        Axis::Y => (&segment.bounds_min().y, &segment.bounds_max().y),
    };
    match algebraic_interval_against_closed_bounds(
        &varying_representation.interval.lower,
        &varying_representation.interval.upper,
        bound_min,
        bound_max,
        policy,
    ) {
        Some(true) => LineCubicAlgebraicPointDomain::InsideSegmentBounds,
        Some(false) => LineCubicAlgebraicPointDomain::OutsideSegmentBounds,
        None => LineCubicAlgebraicPointDomain::Unknown,
    }
}

fn transformed_image_representation(
    image: &AlgebraicRootPolynomialImageReport,
) -> Option<&AlgebraicRootRepresentation> {
    (image.status == AlgebraicRootPolynomialImageStatus::Transformed)
        .then_some(image.representation.as_ref())
        .flatten()
}

fn algebraic_interval_against_closed_bounds(
    lower: &Real,
    upper: &Real,
    bound_min: &Real,
    bound_max: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    let lower_inside = compare_reals_with_policy(lower, bound_min, policy).value()?;
    let upper_inside = compare_reals_with_policy(upper, bound_max, policy).value()?;
    if matches!(lower_inside, Ordering::Equal | Ordering::Greater)
        && matches!(upper_inside, Ordering::Equal | Ordering::Less)
    {
        return Some(true);
    }
    let upper_before_min = compare_reals_with_policy(upper, bound_min, policy).value()?;
    let lower_after_max = compare_reals_with_policy(lower, bound_max, policy).value()?;
    if matches!(upper_before_min, Ordering::Less) || matches!(lower_after_max, Ordering::Greater) {
        Some(false)
    } else {
        None
    }
}

fn classify_algebraic_root_unit_domain(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> LineCubicAlgebraicRootDomain {
    if let Some(exact) = &root.interval.exact_root {
        return match parameter_in_unit_interval(exact, policy) {
            Some(true) => LineCubicAlgebraicRootDomain::InsideUnitInterval,
            Some(false) => LineCubicAlgebraicRootDomain::OutsideUnitInterval,
            None => LineCubicAlgebraicRootDomain::Unknown,
        };
    }
    let lower_zero = compare_reals_with_policy(&root.interval.lower, &Real::zero(), policy).value();
    let upper_one = compare_reals_with_policy(&root.interval.upper, &Real::one(), policy).value();
    let upper_zero = compare_reals_with_policy(&root.interval.upper, &Real::zero(), policy).value();
    let lower_one = compare_reals_with_policy(&root.interval.lower, &Real::one(), policy).value();
    if matches!(lower_zero, Some(Ordering::Equal | Ordering::Greater))
        && matches!(upper_one, Some(Ordering::Equal | Ordering::Less))
    {
        LineCubicAlgebraicRootDomain::InsideUnitInterval
    } else if matches!(upper_zero, Some(Ordering::Less))
        || matches!(lower_one, Some(Ordering::Greater))
    {
        LineCubicAlgebraicRootDomain::OutsideUnitInterval
    } else {
        LineCubicAlgebraicRootDomain::Unknown
    }
}

fn rational_conic_support_coefficient(
    point: &Point2,
    weight: &Real,
    axis: Axis,
    fixed: &Real,
) -> Real {
    weight.clone() * (coordinate(point, axis) - fixed.clone())
}

fn solve_linear_root(b: Real, c: Real, policy: PredicatePolicy) -> Option<Vec<Real>> {
    match compare_reals_with_policy(&b, &Real::zero(), policy).value()? {
        Ordering::Equal => match compare_reals_with_policy(&c, &Real::zero(), policy).value()? {
            Ordering::Equal => None,
            Ordering::Less | Ordering::Greater => Some(Vec::new()),
        },
        Ordering::Less | Ordering::Greater => Some(vec![(-c / b).ok()?]),
    }
}

fn solve_quadratic_roots(a: Real, b: Real, c: Real, policy: PredicatePolicy) -> Option<Vec<Real>> {
    let discriminant = b.clone() * b.clone() - Real::from(4) * a.clone() * c;
    match compare_reals_with_policy(&discriminant, &Real::zero(), policy).value()? {
        Ordering::Less => Some(Vec::new()),
        Ordering::Equal => Some(vec![((-b) / (Real::from(2) * a)).ok()?]),
        Ordering::Greater => {
            let root = discriminant.sqrt().ok()?;
            let denominator = Real::from(2) * a;
            let first = ((-b.clone() - root.clone()) / denominator.clone()).ok()?;
            let second = ((-b + root) / denominator).ok()?;
            Some(vec![first, second])
        }
    }
}

fn quadratic_line_overlap_report(
    segment: &LinePathSegment,
    curve: &QuadraticBezier,
    axis: Axis,
    fixed: Real,
    policy: PredicatePolicy,
) -> Option<LineQuadraticBezierIntersectionReport> {
    if compare_reals_with_policy(&support_coordinate(curve.start(), axis), &fixed, policy)
        .value()?
        != Ordering::Equal
        || compare_reals_with_policy(&support_coordinate(curve.end(), axis), &fixed, policy)
            .value()?
            != Ordering::Equal
    {
        return Some(LineQuadraticBezierIntersectionReport {
            class: LineQuadraticBezierIntersectionClass::Disjoint,
            intersections: Vec::new(),
        });
    }
    if !quadratic_line_image_monotone(curve, axis, policy)? {
        return None;
    }

    let curve_a = varying_coordinate(curve.start(), axis);
    let curve_b = varying_coordinate(curve.end(), axis);
    let segment_a = varying_coordinate(segment.start(), axis);
    let segment_b = varying_coordinate(segment.end(), axis);
    let overlap_min = max_real(
        &min_real(&curve_a, &curve_b, policy)?,
        &min_real(&segment_a, &segment_b, policy)?,
        policy,
    )?;
    let overlap_max = min_real(
        &max_real(&curve_a, &curve_b, policy)?,
        &max_real(&segment_a, &segment_b, policy)?,
        policy,
    )?;
    match compare_reals_with_policy(&overlap_min, &overlap_max, policy).value()? {
        Ordering::Greater => Some(LineQuadraticBezierIntersectionReport {
            class: LineQuadraticBezierIntersectionClass::Disjoint,
            intersections: Vec::new(),
        }),
        Ordering::Equal => {
            let parameter = quadratic_line_image_parameter(curve, axis, &overlap_min, policy)?;
            let point = point_from_axis(axis, fixed, overlap_min);
            Some(LineQuadraticBezierIntersectionReport {
                class: LineQuadraticBezierIntersectionClass::OnePoint,
                intersections: vec![LineQuadraticBezierIntersection { parameter, point }],
            })
        }
        Ordering::Less => {
            let mut intersections = vec![
                LineQuadraticBezierIntersection {
                    parameter: quadratic_line_image_parameter(curve, axis, &overlap_min, policy)?,
                    point: point_from_axis(axis, fixed.clone(), overlap_min),
                },
                LineQuadraticBezierIntersection {
                    parameter: quadratic_line_image_parameter(curve, axis, &overlap_max, policy)?,
                    point: point_from_axis(axis, fixed, overlap_max),
                },
            ];
            sort_line_quadratic_intersections(&mut intersections, policy)?;
            Some(LineQuadraticBezierIntersectionReport {
                class: LineQuadraticBezierIntersectionClass::Overlap,
                intersections,
            })
        }
    }
}

fn degree_elevated_cubic_line_overlap_report(
    segment: &LinePathSegment,
    curve: &CubicBezier,
    axis: Axis,
    fixed: Real,
    policy: PredicatePolicy,
) -> Option<LineCubicBezierIntersectionReport> {
    // The exact cubic line-support equation may vanish identically for both
    // true nonlinear line images and degree-elevated straight segments. We
    // promote only the degree-elevated straight case:
    // `p1 = (2*p0+p3)/3` and `p2 = (p0+2*p3)/3`. Then `B(t)` is exactly the
    // affine line image and endpoint overlap values invert without solving a
    // cubic. This is the same Yap-style retained-object boundary used for the
    // quadratic overlap branch above.
    if !is_degree_elevated_cubic_line(curve, policy)? {
        return None;
    }
    if compare_reals_with_policy(&support_coordinate(curve.start(), axis), &fixed, policy)
        .value()?
        != Ordering::Equal
        || compare_reals_with_policy(&support_coordinate(curve.end(), axis), &fixed, policy)
            .value()?
            != Ordering::Equal
    {
        return Some(LineCubicBezierIntersectionReport {
            class: LineCubicBezierIntersectionClass::Disjoint,
            intersections: Vec::new(),
            algebraic_support_roots: Vec::new(),
        });
    }

    let curve_a = varying_coordinate(curve.start(), axis);
    let curve_b = varying_coordinate(curve.end(), axis);
    let segment_a = varying_coordinate(segment.start(), axis);
    let segment_b = varying_coordinate(segment.end(), axis);
    let overlap_min = max_real(
        &min_real(&curve_a, &curve_b, policy)?,
        &min_real(&segment_a, &segment_b, policy)?,
        policy,
    )?;
    let overlap_max = min_real(
        &max_real(&curve_a, &curve_b, policy)?,
        &max_real(&segment_a, &segment_b, policy)?,
        policy,
    )?;
    match compare_reals_with_policy(&overlap_min, &overlap_max, policy).value()? {
        Ordering::Greater => Some(LineCubicBezierIntersectionReport {
            class: LineCubicBezierIntersectionClass::Disjoint,
            intersections: Vec::new(),
            algebraic_support_roots: Vec::new(),
        }),
        Ordering::Equal => {
            let parameter = cubic_line_image_parameter(curve, axis, &overlap_min, policy)?;
            let point = point_from_axis(axis, fixed, overlap_min);
            Some(LineCubicBezierIntersectionReport {
                class: LineCubicBezierIntersectionClass::OnePoint,
                intersections: vec![LineCubicBezierIntersection { parameter, point }],
                algebraic_support_roots: Vec::new(),
            })
        }
        Ordering::Less => {
            let mut intersections = vec![
                LineCubicBezierIntersection {
                    parameter: cubic_line_image_parameter(curve, axis, &overlap_min, policy)?,
                    point: point_from_axis(axis, fixed.clone(), overlap_min),
                },
                LineCubicBezierIntersection {
                    parameter: cubic_line_image_parameter(curve, axis, &overlap_max, policy)?,
                    point: point_from_axis(axis, fixed, overlap_max),
                },
            ];
            sort_cubic_intersections(&mut intersections, policy)?;
            Some(LineCubicBezierIntersectionReport {
                class: LineCubicBezierIntersectionClass::Overlap,
                intersections,
                algebraic_support_roots: Vec::new(),
            })
        }
    }
}

fn rational_quadratic_line_overlap_report(
    segment: &LinePathSegment,
    curve: &RationalQuadraticBezier,
    axis: Axis,
    fixed: Real,
    policy: PredicatePolicy,
) -> Option<LineRationalQuadraticBezierIntersectionReport> {
    // A rational quadratic can lie exactly on the retained line support while
    // using a nonlinear projective parameterization. Yap's exact-computation
    // model requires us to keep that distinction: we promote overlap only when
    // a Bernstein-sign monotonicity certificate proves that each retained
    // coordinate value has a single replayable source parameter. The derivative
    // sign test uses the rational Bezier hodograph numerator
    // `N'(t)W(t)-N(t)W'(t)` in Bernstein form; see Farouki, *Pythagorean
    // Hodograph Curves* (2008), for the homogeneous rational-curve derivative.
    if !rational_quadratic_same_support(curve, axis, &fixed, policy)? {
        return None;
    }
    if !rational_quadratic_line_image_monotone(curve, axis, policy)? {
        return None;
    }

    let curve_a = varying_coordinate(curve.start(), axis);
    let curve_b = varying_coordinate(curve.end(), axis);
    let segment_a = varying_coordinate(segment.start(), axis);
    let segment_b = varying_coordinate(segment.end(), axis);
    let overlap_min = max_real(
        &min_real(&curve_a, &curve_b, policy)?,
        &min_real(&segment_a, &segment_b, policy)?,
        policy,
    )?;
    let overlap_max = min_real(
        &max_real(&curve_a, &curve_b, policy)?,
        &max_real(&segment_a, &segment_b, policy)?,
        policy,
    )?;
    match compare_reals_with_policy(&overlap_min, &overlap_max, policy).value()? {
        Ordering::Greater => Some(LineRationalQuadraticBezierIntersectionReport {
            class: LineRationalQuadraticBezierIntersectionClass::Disjoint,
            intersections: Vec::new(),
        }),
        Ordering::Equal => {
            let parameter =
                rational_quadratic_line_image_parameter(curve, axis, &overlap_min, policy)?;
            let point = point_from_axis(axis, fixed, overlap_min);
            Some(LineRationalQuadraticBezierIntersectionReport {
                class: LineRationalQuadraticBezierIntersectionClass::OnePoint,
                intersections: vec![LineRationalQuadraticBezierIntersection { parameter, point }],
            })
        }
        Ordering::Less => {
            let mut intersections = vec![
                LineRationalQuadraticBezierIntersection {
                    parameter: rational_quadratic_line_image_parameter(
                        curve,
                        axis,
                        &overlap_min,
                        policy,
                    )?,
                    point: point_from_axis(axis, fixed.clone(), overlap_min),
                },
                LineRationalQuadraticBezierIntersection {
                    parameter: rational_quadratic_line_image_parameter(
                        curve,
                        axis,
                        &overlap_max,
                        policy,
                    )?,
                    point: point_from_axis(axis, fixed, overlap_max),
                },
            ];
            sort_rational_quadratic_intersections(&mut intersections, policy)?;
            Some(LineRationalQuadraticBezierIntersectionReport {
                class: LineRationalQuadraticBezierIntersectionClass::Overlap,
                intersections,
            })
        }
    }
}

fn is_degree_elevated_line(curve: &QuadraticBezier, policy: PredicatePolicy) -> Option<bool> {
    let x_mid = Real::from(2) * curve.control().x.clone();
    let y_mid = Real::from(2) * curve.control().y.clone();
    let x_sum = curve.start().x.clone() + curve.end().x.clone();
    let y_sum = curve.start().y.clone() + curve.end().y.clone();
    Some(
        compare_reals_with_policy(&x_mid, &x_sum, policy).value()? == Ordering::Equal
            && compare_reals_with_policy(&y_mid, &y_sum, policy).value()? == Ordering::Equal,
    )
}

fn quadratic_line_image_monotone(
    curve: &QuadraticBezier,
    axis: Axis,
    policy: PredicatePolicy,
) -> Option<bool> {
    // For a same-support quadratic Bezier, the varying coordinate has
    // derivative `2((p1-p0)(1-t) + (p2-p1)t)`. Certifying the two Bernstein
    // derivative controls have a common nonzero sign proves a one-dimensional
    // monotone image. This is the exact inverse condition used by
    // Farouki-Rajan Bernstein sign reasoning and the object/predicate split in
    // Yap (1997): a nonlinear overlap is accepted only when each retained
    // coordinate has a unique replayable source parameter.
    if is_degree_elevated_line(curve, policy)? {
        return Some(true);
    }
    let first = varying_coordinate(curve.control(), axis) - varying_coordinate(curve.start(), axis);
    let second = varying_coordinate(curve.end(), axis) - varying_coordinate(curve.control(), axis);
    let signs = [
        compare_reals_with_policy(&first, &Real::zero(), policy).value()?,
        compare_reals_with_policy(&second, &Real::zero(), policy).value()?,
    ];
    let nonnegative = signs.iter().all(|sign| *sign != Ordering::Less);
    let nonpositive = signs.iter().all(|sign| *sign != Ordering::Greater);
    let nonconstant = signs.iter().any(|sign| *sign != Ordering::Equal);
    Some(nonconstant && (nonnegative || nonpositive))
}

fn is_degree_elevated_cubic_line(curve: &CubicBezier, policy: PredicatePolicy) -> Option<bool> {
    let three_x1 = Real::from(3) * curve.control0().x.clone();
    let three_y1 = Real::from(3) * curve.control0().y.clone();
    let three_x2 = Real::from(3) * curve.control1().x.clone();
    let three_y2 = Real::from(3) * curve.control1().y.clone();
    let first_x = Real::from(2) * curve.start().x.clone() + curve.end().x.clone();
    let first_y = Real::from(2) * curve.start().y.clone() + curve.end().y.clone();
    let second_x = curve.start().x.clone() + Real::from(2) * curve.end().x.clone();
    let second_y = curve.start().y.clone() + Real::from(2) * curve.end().y.clone();
    Some(
        compare_reals_with_policy(&three_x1, &first_x, policy).value()? == Ordering::Equal
            && compare_reals_with_policy(&three_y1, &first_y, policy).value()? == Ordering::Equal
            && compare_reals_with_policy(&three_x2, &second_x, policy).value()? == Ordering::Equal
            && compare_reals_with_policy(&three_y2, &second_y, policy).value()? == Ordering::Equal,
    )
}

fn line_image_parameter(
    curve: &QuadraticBezier,
    axis: Axis,
    value: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    let start = varying_coordinate(curve.start(), axis);
    let end = varying_coordinate(curve.end(), axis);
    let denominator = end - start.clone();
    match compare_reals_with_policy(&denominator, &Real::zero(), policy).value()? {
        Ordering::Equal => None,
        Ordering::Less | Ordering::Greater => ((value.clone() - start) / denominator).ok(),
    }
}

fn solve_quadratic_varying_roots(
    curve: &QuadraticBezier,
    axis: Axis,
    fixed: Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let p0 = varying_coordinate(curve.start(), axis);
    let p1 = varying_coordinate(curve.control(), axis);
    let p2 = varying_coordinate(curve.end(), axis);
    let a = p0.clone() - Real::from(2) * p1.clone() + p2.clone();
    let b = Real::from(2) * (p1 - p0.clone());
    let c = p0 - fixed;
    match compare_reals_with_policy(&a, &Real::zero(), policy).value()? {
        Ordering::Equal => solve_linear_root(b, c, policy),
        Ordering::Less | Ordering::Greater => solve_quadratic_roots(a, b, c, policy),
    }
}

fn quadratic_line_image_parameter(
    curve: &QuadraticBezier,
    axis: Axis,
    value: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    if is_degree_elevated_line(curve, policy)? {
        return line_image_parameter(curve, axis, value, policy);
    }
    let roots = solve_quadratic_varying_roots(curve, axis, value.clone(), policy)?;
    let mut accepted: Vec<Real> = Vec::new();
    for root in roots {
        match parameter_in_unit_interval(&root, policy) {
            Some(true) => {}
            Some(false) => continue,
            None => return None,
        }
        let mut duplicate = false;
        for existing in &accepted {
            match compare_reals_with_policy(existing, &root, policy).value()? {
                Ordering::Equal => {
                    duplicate = true;
                    break;
                }
                Ordering::Less | Ordering::Greater => {}
            }
        }
        if !duplicate {
            accepted.push(root);
        }
    }
    match accepted.len() {
        1 => accepted.pop(),
        _ => None,
    }
}

fn cubic_line_image_parameter(
    curve: &CubicBezier,
    axis: Axis,
    value: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    let start = varying_coordinate(curve.start(), axis);
    let end = varying_coordinate(curve.end(), axis);
    let denominator = end - start.clone();
    match compare_reals_with_policy(&denominator, &Real::zero(), policy).value()? {
        Ordering::Equal => None,
        Ordering::Less | Ordering::Greater => ((value.clone() - start) / denominator).ok(),
    }
}

fn rational_quadratic_same_support(
    curve: &RationalQuadraticBezier,
    axis: Axis,
    fixed: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    let q0 = rational_conic_support_coefficient(curve.start(), &Real::one(), axis, fixed);
    let q1 =
        rational_conic_support_coefficient(curve.control(), curve.control_weight(), axis, fixed);
    let q2 = rational_conic_support_coefficient(curve.end(), &Real::one(), axis, fixed);
    Some(
        compare_reals_with_policy(&q0, &Real::zero(), policy).value()? == Ordering::Equal
            && compare_reals_with_policy(&q1, &Real::zero(), policy).value()? == Ordering::Equal
            && compare_reals_with_policy(&q2, &Real::zero(), policy).value()? == Ordering::Equal,
    )
}

fn rational_quadratic_line_image_monotone(
    curve: &RationalQuadraticBezier,
    axis: Axis,
    policy: PredicatePolicy,
) -> Option<bool> {
    let start = varying_coordinate(curve.start(), axis);
    let control = varying_coordinate(curve.control(), axis);
    let end = varying_coordinate(curve.end(), axis);
    let first = curve.control_weight().clone() * (control.clone() - start.clone());
    let second = end.clone() - start;
    let third = curve.control_weight().clone() * (end - control);
    let signs = [
        compare_reals_with_policy(&first, &Real::zero(), policy).value()?,
        compare_reals_with_policy(&second, &Real::zero(), policy).value()?,
        compare_reals_with_policy(&third, &Real::zero(), policy).value()?,
    ];
    let nonnegative = signs.iter().all(|sign| *sign != Ordering::Less);
    let nonpositive = signs.iter().all(|sign| *sign != Ordering::Greater);
    let nonconstant = signs.iter().any(|sign| *sign != Ordering::Equal);
    Some(nonconstant && (nonnegative || nonpositive))
}

fn solve_rational_quadratic_varying_roots(
    curve: &RationalQuadraticBezier,
    axis: Axis,
    fixed: Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let q0 = rational_conic_varying_coefficient(curve.start(), &Real::one(), axis, &fixed);
    let q1 =
        rational_conic_varying_coefficient(curve.control(), curve.control_weight(), axis, &fixed);
    let q2 = rational_conic_varying_coefficient(curve.end(), &Real::one(), axis, &fixed);
    let a = q0.clone() - Real::from(2) * q1.clone() + q2.clone();
    let b = Real::from(2) * (q1 - q0.clone());
    let c = q0;
    match compare_reals_with_policy(&a, &Real::zero(), policy).value()? {
        Ordering::Equal => solve_linear_root(b, c, policy),
        Ordering::Less | Ordering::Greater => solve_quadratic_roots(a, b, c, policy),
    }
}

fn rational_conic_varying_coefficient(
    point: &Point2,
    weight: &Real,
    axis: Axis,
    fixed: &Real,
) -> Real {
    weight.clone() * (varying_coordinate(point, axis) - fixed.clone())
}

fn rational_quadratic_line_image_parameter(
    curve: &RationalQuadraticBezier,
    axis: Axis,
    value: &Real,
    policy: PredicatePolicy,
) -> Option<Real> {
    let roots = solve_rational_quadratic_varying_roots(curve, axis, value.clone(), policy)?;
    let mut accepted: Vec<Real> = Vec::new();
    for root in roots {
        match parameter_in_unit_interval(&root, policy) {
            Some(true) => {}
            Some(false) => continue,
            None => return None,
        }
        let point = eval_rational_quadratic_at_real(curve, &root, policy)?;
        match compare_reals_with_policy(&varying_coordinate(&point, axis), value, policy).value()? {
            Ordering::Equal => {}
            Ordering::Less | Ordering::Greater => continue,
        }
        let mut duplicate = false;
        for existing in &accepted {
            match compare_reals_with_policy(existing, &root, policy).value()? {
                Ordering::Equal => {
                    duplicate = true;
                    break;
                }
                Ordering::Less | Ordering::Greater => {}
            }
        }
        if !duplicate {
            accepted.push(root);
        }
    }
    match accepted.len() {
        1 => accepted.pop(),
        _ => None,
    }
}

fn roots_are_tangent(
    curve: &QuadraticBezier,
    axis: Axis,
    segment: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<bool> {
    let fixed = match axis {
        Axis::X => segment.start().y.clone(),
        Axis::Y => segment.start().x.clone(),
    };
    let p0 = coordinate(curve.start(), axis);
    let p1 = coordinate(curve.control(), axis);
    let p2 = coordinate(curve.end(), axis);
    let a = p0.clone() - Real::from(2) * p1.clone() + p2.clone();
    let b = Real::from(2) * (p1 - p0.clone());
    let c = p0 - fixed;
    if compare_reals_with_policy(&a, &Real::zero(), policy).value()? == Ordering::Equal {
        return Some(false);
    }
    let discriminant = b.clone() * b - Real::from(4) * a * c;
    Some(
        compare_reals_with_policy(&discriminant, &Real::zero(), policy).value()? == Ordering::Equal,
    )
}

fn rational_quadratic_roots_are_tangent(
    curve: &RationalQuadraticBezier,
    axis: Axis,
    segment: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<bool> {
    let fixed = match axis {
        Axis::X => segment.start().y.clone(),
        Axis::Y => segment.start().x.clone(),
    };
    let q0 = rational_conic_support_coefficient(curve.start(), &Real::one(), axis, &fixed);
    let q1 =
        rational_conic_support_coefficient(curve.control(), curve.control_weight(), axis, &fixed);
    let q2 = rational_conic_support_coefficient(curve.end(), &Real::one(), axis, &fixed);
    let a = q0.clone() - Real::from(2) * q1.clone() + q2.clone();
    let b = Real::from(2) * (q1 - q0.clone());
    let c = q0;
    if compare_reals_with_policy(&a, &Real::zero(), policy).value()? == Ordering::Equal {
        return Some(false);
    }
    let discriminant = b.clone() * b - Real::from(4) * a * c;
    Some(
        compare_reals_with_policy(&discriminant, &Real::zero(), policy).value()? == Ordering::Equal,
    )
}

fn cubic_roots_are_tangent_up_to_quadratic(
    curve: &CubicBezier,
    axis: Axis,
    segment: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<bool> {
    let fixed = match axis {
        Axis::X => segment.start().y.clone(),
        Axis::Y => segment.start().x.clone(),
    };
    let p0 = coordinate(curve.start(), axis);
    let p1 = coordinate(curve.control0(), axis);
    let p2 = coordinate(curve.control1(), axis);
    let p3 = coordinate(curve.end(), axis);
    let a = -p0.clone() + Real::from(3) * p1.clone() - Real::from(3) * p2.clone() + p3;
    let b = Real::from(3) * p0.clone() - Real::from(6) * p1.clone() + Real::from(3) * p2;
    let c = Real::from(3) * (p1 - p0.clone());
    let d = p0 - fixed;
    if compare_reals_with_policy(&a, &Real::zero(), policy).value()? != Ordering::Equal {
        return None;
    }
    if compare_reals_with_policy(&b, &Real::zero(), policy).value()? == Ordering::Equal {
        return Some(false);
    }
    let discriminant = c.clone() * c - Real::from(4) * b * d;
    Some(
        compare_reals_with_policy(&discriminant, &Real::zero(), policy).value()? == Ordering::Equal,
    )
}

fn parameter_in_unit_interval(parameter: &Real, policy: PredicatePolicy) -> Option<bool> {
    let lower = compare_reals_with_policy(parameter, &Real::zero(), policy).value()?;
    let upper = compare_reals_with_policy(parameter, &Real::one(), policy).value()?;
    Some(!matches!(lower, Ordering::Less) && !matches!(upper, Ordering::Greater))
}

fn point_inside_segment_bounds(
    point: &Point2,
    segment: &LinePathSegment,
    policy: PredicatePolicy,
) -> Option<bool> {
    let x_min = min_real(&segment.start().x, &segment.end().x, policy)?;
    let x_max = max_real(&segment.start().x, &segment.end().x, policy)?;
    let y_min = min_real(&segment.start().y, &segment.end().y, policy)?;
    let y_max = max_real(&segment.start().y, &segment.end().y, policy)?;
    Some(
        compare_reals_with_policy(&point.x, &x_min, policy).value()? != Ordering::Less
            && compare_reals_with_policy(&point.x, &x_max, policy).value()? != Ordering::Greater
            && compare_reals_with_policy(&point.y, &y_min, policy).value()? != Ordering::Less
            && compare_reals_with_policy(&point.y, &y_max, policy).value()? != Ordering::Greater,
    )
}

fn min_real(first: &Real, second: &Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals_with_policy(first, second, policy).value()? {
        Ordering::Less | Ordering::Equal => Some(first.clone()),
        Ordering::Greater => Some(second.clone()),
    }
}

fn max_real(first: &Real, second: &Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals_with_policy(first, second, policy).value()? {
        Ordering::Less | Ordering::Equal => Some(second.clone()),
        Ordering::Greater => Some(first.clone()),
    }
}

fn push_unique_intersection(
    intersections: &mut Vec<LineQuadraticBezierIntersection>,
    parameter: Real,
    point: Point2,
    policy: PredicatePolicy,
) -> Option<()> {
    for existing in intersections.iter() {
        match point2_equal_with_policy(&existing.point, &point, policy).value()? {
            true => return Some(()),
            false => {}
        }
    }
    intersections.push(LineQuadraticBezierIntersection { parameter, point });
    Some(())
}

fn sort_line_quadratic_intersections(
    intersections: &mut [LineQuadraticBezierIntersection],
    policy: PredicatePolicy,
) -> Option<()> {
    for left in 0..intersections.len() {
        for right in (left + 1)..intersections.len() {
            compare_reals_with_policy(
                &intersections[left].parameter,
                &intersections[right].parameter,
                policy,
            )
            .value()?;
        }
    }
    intersections.sort_by(|left, right| {
        compare_reals_with_policy(&left.parameter, &right.parameter, policy)
            .value()
            .expect("pairwise line/quadratic parameter order was certified before sorting")
    });
    Some(())
}

fn eval_quadratic_at_real(curve: &QuadraticBezier, parameter: &Real) -> Point2 {
    let one_minus_t = Real::one() - parameter.clone();
    let start_weight = one_minus_t.clone() * one_minus_t.clone();
    let control_weight = Real::from(2) * one_minus_t * parameter.clone();
    let end_weight = parameter.clone() * parameter.clone();
    Point2::new(
        curve.start().x.clone() * start_weight.clone()
            + curve.control().x.clone() * control_weight.clone()
            + curve.end().x.clone() * end_weight.clone(),
        curve.start().y.clone() * start_weight
            + curve.control().y.clone() * control_weight
            + curve.end().y.clone() * end_weight,
    )
}

fn eval_cubic_at_real(curve: &CubicBezier, parameter: &Real) -> Point2 {
    let one_minus_t = Real::one() - parameter.clone();
    let omt2 = one_minus_t.clone() * one_minus_t.clone();
    let omt3 = omt2.clone() * one_minus_t.clone();
    let t2 = parameter.clone() * parameter.clone();
    let t3 = t2.clone() * parameter.clone();
    let control0_weight = Real::from(3) * omt2 * parameter.clone();
    let control1_weight = Real::from(3) * one_minus_t * t2;
    Point2::new(
        curve.start().x.clone() * omt3.clone()
            + curve.control0().x.clone() * control0_weight.clone()
            + curve.control1().x.clone() * control1_weight.clone()
            + curve.end().x.clone() * t3.clone(),
        curve.start().y.clone() * omt3
            + curve.control0().y.clone() * control0_weight
            + curve.control1().y.clone() * control1_weight
            + curve.end().y.clone() * t3,
    )
}

fn eval_rational_quadratic_at_real(
    curve: &RationalQuadraticBezier,
    parameter: &Real,
    policy: PredicatePolicy,
) -> Option<Point2> {
    let one_minus_t = Real::one() - parameter.clone();
    let b0 = one_minus_t.clone() * one_minus_t.clone();
    let b1 = Real::from(2) * one_minus_t * parameter.clone();
    let b2 = parameter.clone() * parameter.clone();
    let weighted_b1 = b1 * curve.control_weight().clone();
    let denominator = b0.clone() + weighted_b1.clone() + b2.clone();
    if compare_reals_with_policy(&denominator, &Real::zero(), policy).value()? == Ordering::Equal {
        return None;
    }
    let x = curve.start().x.clone() * b0.clone()
        + curve.control().x.clone() * weighted_b1.clone()
        + curve.end().x.clone() * b2.clone();
    let y = curve.start().y.clone() * b0
        + curve.control().y.clone() * weighted_b1
        + curve.end().y.clone() * b2;
    Some(Point2::new(
        (x / denominator.clone()).ok()?,
        (y / denominator).ok()?,
    ))
}

fn push_unique_rational_quadratic_intersection(
    intersections: &mut Vec<LineRationalQuadraticBezierIntersection>,
    parameter: Real,
    point: Point2,
    policy: PredicatePolicy,
) -> Option<()> {
    for existing in intersections.iter() {
        match point2_equal_with_policy(&existing.point, &point, policy).value()? {
            true => return Some(()),
            false => {}
        }
    }
    intersections.push(LineRationalQuadraticBezierIntersection { parameter, point });
    Some(())
}

fn sort_rational_quadratic_intersections(
    intersections: &mut [LineRationalQuadraticBezierIntersection],
    policy: PredicatePolicy,
) -> Option<()> {
    for left in 0..intersections.len() {
        for right in (left + 1)..intersections.len() {
            compare_reals_with_policy(
                &intersections[left].parameter,
                &intersections[right].parameter,
                policy,
            )
            .value()?;
        }
    }
    intersections.sort_by(|left, right| {
        compare_reals_with_policy(&left.parameter, &right.parameter, policy)
            .value()
            .expect("pairwise line/rational-quadratic parameter order was certified before sorting")
    });
    Some(())
}

fn push_unique_cubic_intersection(
    intersections: &mut Vec<LineCubicBezierIntersection>,
    parameter: Real,
    point: Point2,
    policy: PredicatePolicy,
) -> Option<()> {
    for existing in intersections.iter() {
        match point2_equal_with_policy(&existing.point, &point, policy).value()? {
            true => return Some(()),
            false => {}
        }
    }
    intersections.push(LineCubicBezierIntersection { parameter, point });
    Some(())
}

fn sort_cubic_intersections(
    intersections: &mut [LineCubicBezierIntersection],
    policy: PredicatePolicy,
) -> Option<()> {
    for left in 0..intersections.len() {
        for right in (left + 1)..intersections.len() {
            compare_reals_with_policy(
                &intersections[left].parameter,
                &intersections[right].parameter,
                policy,
            )
            .value()?;
        }
    }
    intersections.sort_by(|left, right| {
        compare_reals_with_policy(&left.parameter, &right.parameter, policy)
            .value()
            .expect("pairwise line/cubic parameter order was certified before sorting")
    });
    Some(())
}

fn coordinate(point: &Point2, axis: Axis) -> Real {
    match axis {
        Axis::X => point.y.clone(),
        Axis::Y => point.x.clone(),
    }
}

fn support_coordinate(point: &Point2, axis: Axis) -> Real {
    coordinate(point, axis)
}

fn varying_coordinate(point: &Point2, axis: Axis) -> Real {
    match axis {
        Axis::X => point.x.clone(),
        Axis::Y => point.y.clone(),
    }
}

fn point_from_axis(axis: Axis, fixed: Real, varying: Real) -> Point2 {
    match axis {
        Axis::X => Point2::new(varying, fixed),
        Axis::Y => Point2::new(fixed, varying),
    }
}

fn line_quadratic_unknown_report() -> LineQuadraticBezierIntersectionReport {
    LineQuadraticBezierIntersectionReport {
        class: LineQuadraticBezierIntersectionClass::Unknown,
        intersections: Vec::new(),
    }
}

fn line_cubic_unknown_report() -> LineCubicBezierIntersectionReport {
    LineCubicBezierIntersectionReport {
        class: LineCubicBezierIntersectionClass::Unknown,
        intersections: Vec::new(),
        algebraic_support_roots: Vec::new(),
    }
}

fn line_rational_quadratic_unknown_report() -> LineRationalQuadraticBezierIntersectionReport {
    LineRationalQuadraticBezierIntersectionReport {
        class: LineRationalQuadraticBezierIntersectionClass::Unknown,
        intersections: Vec::new(),
    }
}
