//! Exact rational-parameter Bezier/conic split scheduling.
//!
//! This module is an arrangement cleanup layer, not an intersection finder.
//! It accepts already-certified rational event parameters and emits retained
//! exact fragments. Polynomial Beziers become native sub-curves; rational
//! quadratic conics become homogeneous sub-curve records because restricting a
//! rational Bezier interval does not generally preserve the endpoint-weight
//! normalization used by [`crate::bezier::RationalQuadraticBezier`].

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealExactSetFacts};

use crate::bezier::{BezierParameter, CubicBezier, QuadraticBezier, RationalQuadraticBezier};
use crate::provenance::PathProvenance;

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
