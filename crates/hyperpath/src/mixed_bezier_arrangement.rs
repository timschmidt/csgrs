//! Mixed exact arrangement cleanup for retained lines and quadratic Beziers.
//!
//! This module schedules the first retained mixed line/Bezier split set. It is
//! not a planar-cell extractor and it does not perform a boolean operation.
//! Its job is to promote certified line/quadratic-Bezier events into exact
//! breakpoints on both source families, including events whose Bezier
//! parameters are algebraic `Real` roots rather than rational
//! [`crate::bezier::BezierParameter`] values.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal_with_policy};
use hyperreal::{Real, RealExactSetFacts};

use crate::bezier::QuadraticBezier;
use crate::bezier_arrangement::{
    LineQuadraticBezierIntersection, LineQuadraticBezierIntersectionClass,
    LineQuadraticBezierIntersectionReport, intersect_axis_aligned_line_quadratic_bezier,
};
use crate::provenance::PathProvenance;
use crate::segment::LinePathSegment;

/// Errors that prevent a trusted line/quadratic-Bezier split schedule.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineQuadraticBezierArrangementError {
    /// A retained line segment is degenerate and cannot carry an ordered split set.
    DegenerateLine { line: usize },
    /// Exact comparison of line split parameters was undecidable.
    UndecidableLineOrder { line: usize },
    /// Exact comparison of Bezier split parameters was undecidable.
    UndecidableBezierOrder { curve: usize },
    /// The same geometric point could not be de-duplicated exactly.
    UndecidablePointEquality,
}

/// Exact event between one retained line segment and one quadratic Bezier.
#[derive(Clone, Debug, PartialEq)]
pub struct LineQuadraticBezierArrangementEvent {
    /// Line segment index.
    pub line: usize,
    /// Quadratic Bezier index.
    pub curve: usize,
    /// Certified intersection class.
    pub class: LineQuadraticBezierIntersectionClass,
    /// Raw exact line/quadratic-Bezier predicate report.
    pub intersection: LineQuadraticBezierIntersectionReport,
}

/// Exact breakpoint on one arranged quadratic Bezier.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezierRealBreakpoint {
    /// Quadratic Bezier index.
    pub curve: usize,
    /// Exact source parameter in `[0, 1]`.
    pub parameter: Real,
    /// Exact point image at `parameter`.
    pub point: Point2,
}

/// Exact line breakpoint used by the mixed line/quadratic-Bezier scheduler.
#[derive(Clone, Debug, PartialEq)]
pub struct MixedLineArrangementBreakpoint {
    /// Line segment index.
    pub line: usize,
    /// Exact point on the retained line segment.
    pub point: Point2,
    /// Numerator of the retained parameter `dot(point-start, end-start) / |end-start|^2`.
    pub parameter_numerator: Real,
    /// Positive denominator of the retained line parameter.
    pub parameter_denominator: Real,
}

/// Exact retained line fragment induced by mixed line/quadratic-Bezier events.
#[derive(Clone, Debug, PartialEq)]
pub struct MixedLineArrangementFragment {
    /// Source line segment index.
    pub source_line: usize,
    /// Fragment start witness.
    pub start: MixedLineArrangementBreakpoint,
    /// Fragment end witness.
    pub end: MixedLineArrangementBreakpoint,
    /// Retained exact line fragment.
    pub segment: LinePathSegment,
}

/// Exact quadratic Bezier fragment induced by mixed line/quadratic-Bezier events.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticBezierRealFragment {
    /// Source quadratic Bezier index.
    pub source_curve: usize,
    /// Fragment start witness.
    pub start: QuadraticBezierRealBreakpoint,
    /// Fragment end witness.
    pub end: QuadraticBezierRealBreakpoint,
    /// Retained exact quadratic sub-curve.
    pub curve: QuadraticBezier,
}

/// Cached exact facts for a mixed line/quadratic-Bezier arrangement schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct LineQuadraticBezierArrangementFacts {
    /// Exact-set facts across retained line endpoints and Bezier controls.
    pub input_exact: RealExactSetFacts,
    /// Exact-set facts across emitted line and Bezier fragment controls.
    pub fragment_exact: RealExactSetFacts,
    /// Source provenance for the arrangement schedule.
    pub provenance: PathProvenance,
}

/// Retained mixed line/quadratic-Bezier arrangement schedule.
///
/// The report promotes exact event witnesses from
/// [`intersect_axis_aligned_line_quadratic_bezier`] into split fragments on
/// both participating source families. Unlike
/// [`crate::bezier_arrangement::arrange_quadratic_beziers`], Bezier
/// breakpoints here are arbitrary exact [`Real`] roots, so secants at
/// irrational parameters can still become retained quadratic sub-curves. The
/// sub-curves are reconstructed from exact endpoint and derivative data via de
/// Casteljau's affine restriction identities.
///
/// This is a Yap-style exact object package: uncertain line/Bezier relations
/// remain explicit `Unknown` events and never create topology. Certified
/// events are replayed into ordered line and curve parameters before fragments
/// are emitted. See Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), and Farouki, *Pythagorean Hodograph
/// Curves* (2008), for the retained polynomial-curve discipline used here.
#[derive(Clone, Debug, PartialEq)]
pub struct LineQuadraticBezierArrangementReport {
    /// Retained input line segments.
    pub lines: Vec<LinePathSegment>,
    /// Retained input quadratic Beziers.
    pub curves: Vec<QuadraticBezier>,
    /// Certified or unknown pairwise events.
    pub events: Vec<LineQuadraticBezierArrangementEvent>,
    /// Sorted line breakpoints induced by line endpoints and certified events.
    pub line_breakpoints: Vec<Vec<MixedLineArrangementBreakpoint>>,
    /// Sorted Bezier breakpoints induced by curve endpoints and certified events.
    pub bezier_breakpoints: Vec<Vec<QuadraticBezierRealBreakpoint>>,
    /// Positive-length line fragments.
    pub line_fragments: Vec<MixedLineArrangementFragment>,
    /// Positive-length quadratic Bezier fragments.
    pub bezier_fragments: Vec<QuadraticBezierRealFragment>,
    /// Cached exact facts for the retained schedule.
    pub facts: LineQuadraticBezierArrangementFacts,
}

/// Arrange retained line segments against retained quadratic Beziers.
pub fn arrange_line_segments_with_quadratic_beziers(
    lines: &[LinePathSegment],
    curves: &[QuadraticBezier],
    policy: PredicatePolicy,
) -> Result<LineQuadraticBezierArrangementReport, LineQuadraticBezierArrangementError> {
    arrange_line_segments_with_quadratic_beziers_and_provenance(
        lines,
        curves,
        policy,
        PathProvenance::native(),
    )
}

/// Arrange retained line segments against retained quadratic Beziers with provenance.
pub fn arrange_line_segments_with_quadratic_beziers_and_provenance(
    lines: &[LinePathSegment],
    curves: &[QuadraticBezier],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<LineQuadraticBezierArrangementReport, LineQuadraticBezierArrangementError> {
    reject_degenerate_lines(lines, policy)?;
    let mut line_breakpoints = seed_line_breakpoints(lines);
    let mut bezier_breakpoints = seed_bezier_breakpoints(curves);
    let mut events = Vec::new();

    for (line_index, line) in lines.iter().enumerate() {
        for (curve_index, curve) in curves.iter().enumerate() {
            let intersection = intersect_axis_aligned_line_quadratic_bezier(line, curve, policy);
            if intersection.class != LineQuadraticBezierIntersectionClass::Unknown {
                for event in &intersection.intersections {
                    insert_line_breakpoint(
                        &mut line_breakpoints[line_index],
                        line_index,
                        line,
                        event.point.clone(),
                        policy,
                    )?;
                    insert_bezier_breakpoint(
                        &mut bezier_breakpoints[curve_index],
                        curve_index,
                        event,
                        policy,
                    )?;
                }
            }
            events.push(LineQuadraticBezierArrangementEvent {
                line: line_index,
                curve: curve_index,
                class: intersection.class,
                intersection,
            });
        }
    }

    sort_and_dedup_line_breakpoints(&mut line_breakpoints, policy)?;
    sort_and_dedup_bezier_breakpoints(&mut bezier_breakpoints, policy)?;
    let line_fragments = build_line_fragments(&line_breakpoints, policy)?;
    let bezier_fragments = build_bezier_fragments(&bezier_breakpoints, curves, policy)?;
    let facts = LineQuadraticBezierArrangementFacts {
        input_exact: input_exact_facts(lines, curves),
        fragment_exact: fragment_exact_facts(&line_fragments, &bezier_fragments),
        provenance,
    };

    Ok(LineQuadraticBezierArrangementReport {
        lines: lines.to_vec(),
        curves: curves.to_vec(),
        events,
        line_breakpoints,
        bezier_breakpoints,
        line_fragments,
        bezier_fragments,
        facts,
    })
}

fn reject_degenerate_lines(
    lines: &[LinePathSegment],
    policy: PredicatePolicy,
) -> Result<(), LineQuadraticBezierArrangementError> {
    for (index, line) in lines.iter().enumerate() {
        if line.facts().known_degenerate == Some(true)
            || compare_reals_with_policy(&line.length_squared(), &Real::zero(), policy).value()
                == Some(Ordering::Equal)
        {
            return Err(LineQuadraticBezierArrangementError::DegenerateLine { line: index });
        }
    }
    Ok(())
}

fn seed_line_breakpoints(lines: &[LinePathSegment]) -> Vec<Vec<MixedLineArrangementBreakpoint>> {
    lines
        .iter()
        .enumerate()
        .map(|(line_index, line)| {
            vec![
                line_breakpoint(line_index, line, line.start().clone()),
                line_breakpoint(line_index, line, line.end().clone()),
            ]
        })
        .collect()
}

fn seed_bezier_breakpoints(curves: &[QuadraticBezier]) -> Vec<Vec<QuadraticBezierRealBreakpoint>> {
    curves
        .iter()
        .enumerate()
        .map(|(curve_index, curve)| {
            vec![
                QuadraticBezierRealBreakpoint {
                    curve: curve_index,
                    parameter: Real::zero(),
                    point: curve.start().clone(),
                },
                QuadraticBezierRealBreakpoint {
                    curve: curve_index,
                    parameter: Real::one(),
                    point: curve.end().clone(),
                },
            ]
        })
        .collect()
}

fn insert_line_breakpoint(
    breakpoints: &mut Vec<MixedLineArrangementBreakpoint>,
    line_index: usize,
    line: &LinePathSegment,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<(), LineQuadraticBezierArrangementError> {
    for existing in breakpoints.iter() {
        match point2_equal_with_policy(&existing.point, &point, policy).value() {
            Some(true) => return Ok(()),
            Some(false) => {}
            None => return Err(LineQuadraticBezierArrangementError::UndecidablePointEquality),
        }
    }
    breakpoints.push(line_breakpoint(line_index, line, point));
    Ok(())
}

fn insert_bezier_breakpoint(
    breakpoints: &mut Vec<QuadraticBezierRealBreakpoint>,
    curve_index: usize,
    event: &LineQuadraticBezierIntersection,
    policy: PredicatePolicy,
) -> Result<(), LineQuadraticBezierArrangementError> {
    for existing in breakpoints.iter() {
        match compare_reals_with_policy(&existing.parameter, &event.parameter, policy).value() {
            Some(Ordering::Equal) => return Ok(()),
            Some(Ordering::Less | Ordering::Greater) => {}
            None => {
                return Err(
                    LineQuadraticBezierArrangementError::UndecidableBezierOrder {
                        curve: curve_index,
                    },
                );
            }
        }
    }
    breakpoints.push(QuadraticBezierRealBreakpoint {
        curve: curve_index,
        parameter: event.parameter.clone(),
        point: event.point.clone(),
    });
    Ok(())
}

fn line_breakpoint(
    line_index: usize,
    line: &LinePathSegment,
    point: Point2,
) -> MixedLineArrangementBreakpoint {
    let dx = line.end().x.clone() - line.start().x.clone();
    let dy = line.end().y.clone() - line.start().y.clone();
    let px = point.x.clone() - line.start().x.clone();
    let py = point.y.clone() - line.start().y.clone();
    let parameter_numerator = px * dx.clone() + py * dy.clone();
    let parameter_denominator = dx.clone() * dx + dy.clone() * dy;
    MixedLineArrangementBreakpoint {
        line: line_index,
        point,
        parameter_numerator,
        parameter_denominator,
    }
}

fn sort_and_dedup_line_breakpoints(
    breakpoints: &mut [Vec<MixedLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineQuadraticBezierArrangementError> {
    for (line_index, points) in breakpoints.iter_mut().enumerate() {
        certify_line_orders(points, line_index, policy)?;
        points.sort_by(|left, right| {
            compare_line_parameters(left, right, policy)
                .expect("line breakpoint order was certified before sorting")
        });
        let mut deduped: Vec<MixedLineArrangementBreakpoint> = Vec::new();
        for point in points.drain(..) {
            if let Some(last) = deduped.last() {
                match point2_equal_with_policy(&last.point, &point.point, policy).value() {
                    Some(true) => continue,
                    Some(false) => {}
                    None => {
                        return Err(LineQuadraticBezierArrangementError::UndecidablePointEquality);
                    }
                }
            }
            deduped.push(point);
        }
        *points = deduped;
    }
    Ok(())
}

fn certify_line_orders(
    points: &[MixedLineArrangementBreakpoint],
    line_index: usize,
    policy: PredicatePolicy,
) -> Result<(), LineQuadraticBezierArrangementError> {
    for left in 0..points.len() {
        for right in (left + 1)..points.len() {
            compare_line_parameters(&points[left], &points[right], policy).ok_or(
                LineQuadraticBezierArrangementError::UndecidableLineOrder { line: line_index },
            )?;
        }
    }
    Ok(())
}

fn compare_line_parameters(
    left: &MixedLineArrangementBreakpoint,
    right: &MixedLineArrangementBreakpoint,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    compare_reals_with_policy(
        &(left.parameter_numerator.clone() * right.parameter_denominator.clone()),
        &(right.parameter_numerator.clone() * left.parameter_denominator.clone()),
        policy,
    )
    .value()
}

fn sort_and_dedup_bezier_breakpoints(
    breakpoints: &mut [Vec<QuadraticBezierRealBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineQuadraticBezierArrangementError> {
    for (curve_index, points) in breakpoints.iter_mut().enumerate() {
        certify_bezier_orders(points, curve_index, policy)?;
        points.sort_by(|left, right| {
            compare_reals_with_policy(&left.parameter, &right.parameter, policy)
                .value()
                .expect("Bezier breakpoint order was certified before sorting")
        });
        let mut deduped: Vec<QuadraticBezierRealBreakpoint> = Vec::new();
        for point in points.drain(..) {
            if let Some(last) = deduped.last() {
                match compare_reals_with_policy(&last.parameter, &point.parameter, policy).value() {
                    Some(Ordering::Equal) => continue,
                    Some(Ordering::Less | Ordering::Greater) => {}
                    None => {
                        return Err(
                            LineQuadraticBezierArrangementError::UndecidableBezierOrder {
                                curve: curve_index,
                            },
                        );
                    }
                }
            }
            deduped.push(point);
        }
        *points = deduped;
    }
    Ok(())
}

fn certify_bezier_orders(
    points: &[QuadraticBezierRealBreakpoint],
    curve_index: usize,
    policy: PredicatePolicy,
) -> Result<(), LineQuadraticBezierArrangementError> {
    for left in 0..points.len() {
        for right in (left + 1)..points.len() {
            compare_reals_with_policy(&points[left].parameter, &points[right].parameter, policy)
                .value()
                .ok_or(
                    LineQuadraticBezierArrangementError::UndecidableBezierOrder {
                        curve: curve_index,
                    },
                )?;
        }
    }
    Ok(())
}

fn build_line_fragments(
    breakpoints: &[Vec<MixedLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<Vec<MixedLineArrangementFragment>, LineQuadraticBezierArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            if compare_line_parameters(&window[0], &window[1], policy) == Some(Ordering::Equal) {
                continue;
            }
            fragments.push(MixedLineArrangementFragment {
                source_line: window[0].line,
                start: window[0].clone(),
                end: window[1].clone(),
                segment: LinePathSegment::new(window[0].point.clone(), window[1].point.clone()),
            });
        }
    }
    Ok(fragments)
}

fn build_bezier_fragments(
    breakpoints: &[Vec<QuadraticBezierRealBreakpoint>],
    curves: &[QuadraticBezier],
    policy: PredicatePolicy,
) -> Result<Vec<QuadraticBezierRealFragment>, LineQuadraticBezierArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            match compare_reals_with_policy(&window[0].parameter, &window[1].parameter, policy)
                .value()
            {
                Some(Ordering::Equal) => continue,
                Some(Ordering::Less | Ordering::Greater) => {}
                None => {
                    return Err(
                        LineQuadraticBezierArrangementError::UndecidableBezierOrder {
                            curve: window[0].curve,
                        },
                    );
                }
            }
            let source = &curves[window[0].curve];
            fragments.push(QuadraticBezierRealFragment {
                source_curve: window[0].curve,
                start: window[0].clone(),
                end: window[1].clone(),
                curve: quadratic_subcurve_real(source, &window[0].parameter, &window[1].parameter),
            });
        }
    }
    Ok(fragments)
}

fn quadratic_subcurve_real(curve: &QuadraticBezier, start: &Real, end: &Real) -> QuadraticBezier {
    let start_point = eval_quadratic_real(curve, start);
    let end_point = eval_quadratic_real(curve, end);
    let delta = end.clone() - start.clone();
    let derivative = derivative_quadratic_real(curve, start);
    let half = Real::from(2);
    let control = Point2::new(
        start_point.x.clone() + (delta.clone() * derivative.x / half.clone()).expect("nonzero two"),
        start_point.y.clone() + (delta * derivative.y / half).expect("nonzero two"),
    );
    QuadraticBezier::with_provenance(start_point, control, end_point, curve.provenance())
}

fn eval_quadratic_real(curve: &QuadraticBezier, parameter: &Real) -> Point2 {
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

fn derivative_quadratic_real(curve: &QuadraticBezier, parameter: &Real) -> Point2 {
    let one_minus_t = Real::one() - parameter.clone();
    let start_weight = Real::from(2) * one_minus_t;
    let end_weight = Real::from(2) * parameter.clone();
    Point2::new(
        (curve.control().x.clone() - curve.start().x.clone()) * start_weight.clone()
            + (curve.end().x.clone() - curve.control().x.clone()) * end_weight.clone(),
        (curve.control().y.clone() - curve.start().y.clone()) * start_weight
            + (curve.end().y.clone() - curve.control().y.clone()) * end_weight,
    )
}

fn input_exact_facts(lines: &[LinePathSegment], curves: &[QuadraticBezier]) -> RealExactSetFacts {
    let mut values = Vec::new();
    for line in lines {
        values.extend([
            &line.start().x,
            &line.start().y,
            &line.end().x,
            &line.end().y,
        ]);
    }
    for curve in curves {
        values.extend([
            &curve.start().x,
            &curve.start().y,
            &curve.control().x,
            &curve.control().y,
            &curve.end().x,
            &curve.end().y,
        ]);
    }
    Real::exact_set_facts(values)
}

fn fragment_exact_facts(
    lines: &[MixedLineArrangementFragment],
    curves: &[QuadraticBezierRealFragment],
) -> RealExactSetFacts {
    let mut values = Vec::new();
    for fragment in lines {
        values.extend([
            &fragment.segment.start().x,
            &fragment.segment.start().y,
            &fragment.segment.end().x,
            &fragment.segment.end().y,
        ]);
    }
    for fragment in curves {
        values.extend([
            &fragment.curve.start().x,
            &fragment.curve.start().y,
            &fragment.curve.control().x,
            &fragment.curve.control().y,
            &fragment.curve.end().x,
            &fragment.curve.end().y,
        ]);
    }
    Real::exact_set_facts(values)
}
