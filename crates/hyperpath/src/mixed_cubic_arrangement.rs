//! Mixed exact arrangement cleanup for retained lines and cubic Beziers.
//!
//! This module is a retained split scheduler, not a planar-cell extractor and
//! not a boolean operation. It promotes certified line/cubic Bezier events into
//! exact line breakpoints and exact cubic `Real`-parameter breakpoints, then
//! emits positive-length fragments. True cubic support roots are retained by
//! the predicate layer as represented algebraic parameters and point images,
//! but remain explicit `Unknown` events here until this scheduler can consume
//! algebraic image ordering as concrete breakpoints.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal_with_policy};
use hyperreal::{Real, RealExactSetFacts};

use crate::bezier::CubicBezier;
use crate::bezier_arrangement::{
    LineCubicBezierIntersection, LineCubicBezierIntersectionClass,
    LineCubicBezierIntersectionReport, intersect_axis_aligned_line_cubic_bezier,
};
use crate::provenance::PathProvenance;
use crate::segment::LinePathSegment;

/// Errors that prevent a trusted line/cubic-Bezier split schedule.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierArrangementError {
    /// A retained line segment is degenerate and cannot carry an ordered split set.
    DegenerateLine { line: usize },
    /// Exact comparison of line split parameters was undecidable.
    UndecidableLineOrder { line: usize },
    /// Exact comparison of cubic Bezier split parameters was undecidable.
    UndecidableCubicOrder { curve: usize },
    /// The same geometric point could not be de-duplicated exactly.
    UndecidablePointEquality,
}

/// Exact event between one retained line segment and one cubic Bezier.
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierArrangementEvent {
    /// Line segment index.
    pub line: usize,
    /// Cubic Bezier index.
    pub curve: usize,
    /// Certified intersection class.
    pub class: LineCubicBezierIntersectionClass,
    /// Raw exact line/cubic-Bezier predicate report.
    pub intersection: LineCubicBezierIntersectionReport,
}

/// Exact breakpoint on one arranged cubic Bezier.
#[derive(Clone, Debug, PartialEq)]
pub struct CubicBezierRealBreakpoint {
    /// Cubic Bezier index.
    pub curve: usize,
    /// Exact source parameter in `[0, 1]`.
    pub parameter: Real,
    /// Exact point image at `parameter`.
    pub point: Point2,
}

/// Exact line breakpoint used by the mixed line/cubic-Bezier scheduler.
#[derive(Clone, Debug, PartialEq)]
pub struct MixedCubicLineArrangementBreakpoint {
    /// Line segment index.
    pub line: usize,
    /// Exact point on the retained line segment.
    pub point: Point2,
    /// Numerator of the retained parameter `dot(point-start, end-start) / |end-start|^2`.
    pub parameter_numerator: Real,
    /// Positive denominator of the retained line parameter.
    pub parameter_denominator: Real,
}

/// Exact retained line fragment induced by mixed line/cubic-Bezier events.
#[derive(Clone, Debug, PartialEq)]
pub struct MixedCubicLineArrangementFragment {
    /// Source line segment index.
    pub source_line: usize,
    /// Fragment start witness.
    pub start: MixedCubicLineArrangementBreakpoint,
    /// Fragment end witness.
    pub end: MixedCubicLineArrangementBreakpoint,
    /// Retained exact line fragment.
    pub segment: LinePathSegment,
}

/// Exact cubic Bezier fragment induced by mixed line/cubic-Bezier events.
#[derive(Clone, Debug, PartialEq)]
pub struct CubicBezierRealFragment {
    /// Source cubic Bezier index.
    pub source_curve: usize,
    /// Fragment start witness.
    pub start: CubicBezierRealBreakpoint,
    /// Fragment end witness.
    pub end: CubicBezierRealBreakpoint,
    /// Retained exact cubic sub-curve.
    pub curve: CubicBezier,
}

/// Cached exact facts for a mixed line/cubic-Bezier arrangement schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierArrangementFacts {
    /// Exact-set facts across retained line endpoints and cubic controls.
    pub input_exact: RealExactSetFacts,
    /// Exact-set facts across emitted line and cubic fragment controls.
    pub fragment_exact: RealExactSetFacts,
    /// Source provenance for the arrangement schedule.
    pub provenance: PathProvenance,
}

/// Retained mixed line/cubic-Bezier arrangement schedule.
///
/// Certified events are replayed into sorted split parameters before fragments
/// are emitted. Unknown relations do not add breakpoints. Cubic fragments are
/// reconstructed from exact endpoint and derivative data on each retained
/// subinterval. This follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): exact object construction and exact
/// predicate replay are separated, and unsupported roots remain report states.
/// The cubic restriction formula is de Casteljau's affine subdivision written
/// in endpoint/derivative form; see Farouki, *Pythagorean Hodograph Curves*
/// (2008), for the same retained polynomial-curve discipline.
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierArrangementReport {
    /// Retained input line segments.
    pub lines: Vec<LinePathSegment>,
    /// Retained input cubic Beziers.
    pub curves: Vec<CubicBezier>,
    /// Certified or unknown pairwise events.
    pub events: Vec<LineCubicBezierArrangementEvent>,
    /// Sorted line breakpoints induced by line endpoints and certified events.
    pub line_breakpoints: Vec<Vec<MixedCubicLineArrangementBreakpoint>>,
    /// Sorted cubic breakpoints induced by curve endpoints and certified events.
    pub cubic_breakpoints: Vec<Vec<CubicBezierRealBreakpoint>>,
    /// Positive-length line fragments.
    pub line_fragments: Vec<MixedCubicLineArrangementFragment>,
    /// Positive-length cubic Bezier fragments.
    pub cubic_fragments: Vec<CubicBezierRealFragment>,
    /// Cached exact facts for the retained schedule.
    pub facts: LineCubicBezierArrangementFacts,
}

/// Arrange retained line segments against retained cubic Beziers.
pub fn arrange_line_segments_with_cubic_beziers(
    lines: &[LinePathSegment],
    curves: &[CubicBezier],
    policy: PredicatePolicy,
) -> Result<LineCubicBezierArrangementReport, LineCubicBezierArrangementError> {
    arrange_line_segments_with_cubic_beziers_and_provenance(
        lines,
        curves,
        policy,
        PathProvenance::native(),
    )
}

/// Arrange retained line segments against retained cubic Beziers with provenance.
pub fn arrange_line_segments_with_cubic_beziers_and_provenance(
    lines: &[LinePathSegment],
    curves: &[CubicBezier],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<LineCubicBezierArrangementReport, LineCubicBezierArrangementError> {
    reject_degenerate_lines(lines, policy)?;
    let mut line_breakpoints = seed_line_breakpoints(lines);
    let mut cubic_breakpoints = seed_cubic_breakpoints(curves);
    let mut events = Vec::new();

    for (line_index, line) in lines.iter().enumerate() {
        for (curve_index, curve) in curves.iter().enumerate() {
            let intersection = intersect_axis_aligned_line_cubic_bezier(line, curve, policy);
            if intersection.class != LineCubicBezierIntersectionClass::Unknown {
                for event in &intersection.intersections {
                    insert_line_breakpoint(
                        &mut line_breakpoints[line_index],
                        line_index,
                        line,
                        event.point.clone(),
                        policy,
                    )?;
                    insert_cubic_breakpoint(
                        &mut cubic_breakpoints[curve_index],
                        curve_index,
                        event,
                        policy,
                    )?;
                }
            }
            events.push(LineCubicBezierArrangementEvent {
                line: line_index,
                curve: curve_index,
                class: intersection.class,
                intersection,
            });
        }
    }

    sort_and_dedup_line_breakpoints(&mut line_breakpoints, policy)?;
    sort_and_dedup_cubic_breakpoints(&mut cubic_breakpoints, policy)?;
    let line_fragments = build_line_fragments(&line_breakpoints, policy)?;
    let cubic_fragments = build_cubic_fragments(&cubic_breakpoints, curves, policy)?;
    let facts = LineCubicBezierArrangementFacts {
        input_exact: input_exact_facts(lines, curves),
        fragment_exact: fragment_exact_facts(&line_fragments, &cubic_fragments),
        provenance,
    };

    Ok(LineCubicBezierArrangementReport {
        lines: lines.to_vec(),
        curves: curves.to_vec(),
        events,
        line_breakpoints,
        cubic_breakpoints,
        line_fragments,
        cubic_fragments,
        facts,
    })
}

fn reject_degenerate_lines(
    lines: &[LinePathSegment],
    policy: PredicatePolicy,
) -> Result<(), LineCubicBezierArrangementError> {
    for (index, line) in lines.iter().enumerate() {
        if line.facts().known_degenerate == Some(true)
            || compare_reals_with_policy(&line.length_squared(), &Real::zero(), policy).value()
                == Some(Ordering::Equal)
        {
            return Err(LineCubicBezierArrangementError::DegenerateLine { line: index });
        }
    }
    Ok(())
}

fn seed_line_breakpoints(
    lines: &[LinePathSegment],
) -> Vec<Vec<MixedCubicLineArrangementBreakpoint>> {
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

fn seed_cubic_breakpoints(curves: &[CubicBezier]) -> Vec<Vec<CubicBezierRealBreakpoint>> {
    curves
        .iter()
        .enumerate()
        .map(|(curve_index, curve)| {
            vec![
                CubicBezierRealBreakpoint {
                    curve: curve_index,
                    parameter: Real::zero(),
                    point: curve.start().clone(),
                },
                CubicBezierRealBreakpoint {
                    curve: curve_index,
                    parameter: Real::one(),
                    point: curve.end().clone(),
                },
            ]
        })
        .collect()
}

fn insert_line_breakpoint(
    breakpoints: &mut Vec<MixedCubicLineArrangementBreakpoint>,
    line_index: usize,
    line: &LinePathSegment,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<(), LineCubicBezierArrangementError> {
    for existing in breakpoints.iter() {
        match point2_equal_with_policy(&existing.point, &point, policy).value() {
            Some(true) => return Ok(()),
            Some(false) => {}
            None => return Err(LineCubicBezierArrangementError::UndecidablePointEquality),
        }
    }
    breakpoints.push(line_breakpoint(line_index, line, point));
    Ok(())
}

fn insert_cubic_breakpoint(
    breakpoints: &mut Vec<CubicBezierRealBreakpoint>,
    curve_index: usize,
    event: &LineCubicBezierIntersection,
    policy: PredicatePolicy,
) -> Result<(), LineCubicBezierArrangementError> {
    for existing in breakpoints.iter() {
        match compare_reals_with_policy(&existing.parameter, &event.parameter, policy).value() {
            Some(Ordering::Equal) => return Ok(()),
            Some(Ordering::Less | Ordering::Greater) => {}
            None => {
                return Err(LineCubicBezierArrangementError::UndecidableCubicOrder {
                    curve: curve_index,
                });
            }
        }
    }
    breakpoints.push(CubicBezierRealBreakpoint {
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
) -> MixedCubicLineArrangementBreakpoint {
    let dx = line.end().x.clone() - line.start().x.clone();
    let dy = line.end().y.clone() - line.start().y.clone();
    let px = point.x.clone() - line.start().x.clone();
    let py = point.y.clone() - line.start().y.clone();
    let parameter_numerator = px * dx.clone() + py * dy.clone();
    let parameter_denominator = dx.clone() * dx + dy.clone() * dy;
    MixedCubicLineArrangementBreakpoint {
        line: line_index,
        point,
        parameter_numerator,
        parameter_denominator,
    }
}

fn sort_and_dedup_line_breakpoints(
    breakpoints: &mut [Vec<MixedCubicLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineCubicBezierArrangementError> {
    for (line_index, points) in breakpoints.iter_mut().enumerate() {
        certify_line_orders(points, line_index, policy)?;
        points.sort_by(|left, right| {
            compare_line_parameters(left, right, policy)
                .expect("line breakpoint order was certified before sorting")
        });
        let mut deduped: Vec<MixedCubicLineArrangementBreakpoint> = Vec::new();
        for point in points.drain(..) {
            if let Some(last) = deduped.last() {
                match point2_equal_with_policy(&last.point, &point.point, policy).value() {
                    Some(true) => continue,
                    Some(false) => {}
                    None => {
                        return Err(LineCubicBezierArrangementError::UndecidablePointEquality);
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
    points: &[MixedCubicLineArrangementBreakpoint],
    line_index: usize,
    policy: PredicatePolicy,
) -> Result<(), LineCubicBezierArrangementError> {
    for left in 0..points.len() {
        for right in (left + 1)..points.len() {
            compare_line_parameters(&points[left], &points[right], policy).ok_or(
                LineCubicBezierArrangementError::UndecidableLineOrder { line: line_index },
            )?;
        }
    }
    Ok(())
}

fn compare_line_parameters(
    left: &MixedCubicLineArrangementBreakpoint,
    right: &MixedCubicLineArrangementBreakpoint,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    compare_reals_with_policy(
        &(left.parameter_numerator.clone() * right.parameter_denominator.clone()),
        &(right.parameter_numerator.clone() * left.parameter_denominator.clone()),
        policy,
    )
    .value()
}

fn sort_and_dedup_cubic_breakpoints(
    breakpoints: &mut [Vec<CubicBezierRealBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineCubicBezierArrangementError> {
    for (curve_index, points) in breakpoints.iter_mut().enumerate() {
        certify_cubic_orders(points, curve_index, policy)?;
        points.sort_by(|left, right| {
            compare_reals_with_policy(&left.parameter, &right.parameter, policy)
                .value()
                .expect("cubic breakpoint order was certified before sorting")
        });
        let mut deduped: Vec<CubicBezierRealBreakpoint> = Vec::new();
        for point in points.drain(..) {
            if let Some(last) = deduped.last() {
                match compare_reals_with_policy(&last.parameter, &point.parameter, policy).value() {
                    Some(Ordering::Equal) => continue,
                    Some(Ordering::Less | Ordering::Greater) => {}
                    None => {
                        return Err(LineCubicBezierArrangementError::UndecidableCubicOrder {
                            curve: curve_index,
                        });
                    }
                }
            }
            deduped.push(point);
        }
        *points = deduped;
    }
    Ok(())
}

fn certify_cubic_orders(
    points: &[CubicBezierRealBreakpoint],
    curve_index: usize,
    policy: PredicatePolicy,
) -> Result<(), LineCubicBezierArrangementError> {
    for left in 0..points.len() {
        for right in (left + 1)..points.len() {
            compare_reals_with_policy(&points[left].parameter, &points[right].parameter, policy)
                .value()
                .ok_or(LineCubicBezierArrangementError::UndecidableCubicOrder {
                    curve: curve_index,
                })?;
        }
    }
    Ok(())
}

fn build_line_fragments(
    breakpoints: &[Vec<MixedCubicLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<Vec<MixedCubicLineArrangementFragment>, LineCubicBezierArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            if compare_line_parameters(&window[0], &window[1], policy) == Some(Ordering::Equal) {
                continue;
            }
            fragments.push(MixedCubicLineArrangementFragment {
                source_line: window[0].line,
                start: window[0].clone(),
                end: window[1].clone(),
                segment: LinePathSegment::new(window[0].point.clone(), window[1].point.clone()),
            });
        }
    }
    Ok(fragments)
}

fn build_cubic_fragments(
    breakpoints: &[Vec<CubicBezierRealBreakpoint>],
    curves: &[CubicBezier],
    policy: PredicatePolicy,
) -> Result<Vec<CubicBezierRealFragment>, LineCubicBezierArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            match compare_reals_with_policy(&window[0].parameter, &window[1].parameter, policy)
                .value()
            {
                Some(Ordering::Equal) => continue,
                Some(Ordering::Less | Ordering::Greater) => {}
                None => {
                    return Err(LineCubicBezierArrangementError::UndecidableCubicOrder {
                        curve: window[0].curve,
                    });
                }
            }
            let source = &curves[window[0].curve];
            fragments.push(CubicBezierRealFragment {
                source_curve: window[0].curve,
                start: window[0].clone(),
                end: window[1].clone(),
                curve: cubic_subcurve_real(source, &window[0].parameter, &window[1].parameter),
            });
        }
    }
    Ok(fragments)
}

fn cubic_subcurve_real(curve: &CubicBezier, start: &Real, end: &Real) -> CubicBezier {
    let start_point = eval_cubic_real(curve, start);
    let end_point = eval_cubic_real(curve, end);
    let delta = end.clone() - start.clone();
    let start_derivative = derivative_cubic_real(curve, start);
    let end_derivative = derivative_cubic_real(curve, end);
    let third = Real::from(3);
    let control0 = Point2::new(
        start_point.x.clone()
            + (delta.clone() * start_derivative.x / third.clone()).expect("nonzero three"),
        start_point.y.clone()
            + (delta.clone() * start_derivative.y / third.clone()).expect("nonzero three"),
    );
    let control1 = Point2::new(
        end_point.x.clone()
            - (delta.clone() * end_derivative.x / third.clone()).expect("nonzero three"),
        end_point.y.clone() - (delta * end_derivative.y / third).expect("nonzero three"),
    );
    CubicBezier::with_provenance(
        start_point,
        control0,
        control1,
        end_point,
        curve.provenance(),
    )
}

fn eval_cubic_real(curve: &CubicBezier, parameter: &Real) -> Point2 {
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

fn derivative_cubic_real(curve: &CubicBezier, parameter: &Real) -> Point2 {
    let one_minus_t = Real::one() - parameter.clone();
    let omt2 = one_minus_t.clone() * one_minus_t.clone();
    let t2 = parameter.clone() * parameter.clone();
    let middle = Real::from(6) * one_minus_t * parameter.clone();
    Point2::new(
        (curve.control0().x.clone() - curve.start().x.clone()) * Real::from(3) * omt2.clone()
            + (curve.control1().x.clone() - curve.control0().x.clone()) * middle.clone()
            + (curve.end().x.clone() - curve.control1().x.clone()) * Real::from(3) * t2.clone(),
        (curve.control0().y.clone() - curve.start().y.clone()) * Real::from(3) * omt2
            + (curve.control1().y.clone() - curve.control0().y.clone()) * middle
            + (curve.end().y.clone() - curve.control1().y.clone()) * Real::from(3) * t2,
    )
}

fn input_exact_facts(lines: &[LinePathSegment], curves: &[CubicBezier]) -> RealExactSetFacts {
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
            &curve.control0().x,
            &curve.control0().y,
            &curve.control1().x,
            &curve.control1().y,
            &curve.end().x,
            &curve.end().y,
        ]);
    }
    Real::exact_set_facts(values)
}

fn fragment_exact_facts(
    lines: &[MixedCubicLineArrangementFragment],
    curves: &[CubicBezierRealFragment],
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
            &fragment.curve.control0().x,
            &fragment.curve.control0().y,
            &fragment.curve.control1().x,
            &fragment.curve.control1().y,
            &fragment.curve.end().x,
            &fragment.curve.end().y,
        ]);
    }
    Real::exact_set_facts(values)
}
