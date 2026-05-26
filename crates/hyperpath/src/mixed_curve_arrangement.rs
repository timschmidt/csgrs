//! Bounded mixed line/arc/Bezier/conic arrangement scheduling.
//!
//! This module closes the first mixed-family gap left by the pairwise
//! line/arc, line/quadratic, line/cubic, and line/conic schedulers: one retained line can
//! now receive exact breakpoints from all four curve families before the
//! shared cell graph is built. It deliberately does **not** claim a general
//! curve-curve arrangement. Non-line fragments are admitted together only when
//! their exact convex-hull boxes are strictly separated, so any possible
//! curve-curve intersection remains an explicit unsupported state instead of
//! sampled topology.
//!
//! The design follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): numerical proposal and exact object
//! construction are separated from topology acceptance. The polynomial
//! Bezier hodograph/convex-hull facts are the standard curve-carrier
//! discipline described by Farouki, *Pythagorean Hodograph Curves* (2008).
//! Explicit circular arcs use the same exact curve-object/predicate split as
//! circular-arc arrangement packages such as CGAL Arrangement_on_surface_2.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal_with_policy};
use hyperreal::{Real, RealExactSetFacts};

use crate::arc::{ExplicitArcPointClassification, ExplicitCircularArc};
use crate::arrangement::{
    ExplicitArcArrangementFragment, LineArcArrangementEvent, LineArrangementBreakpoint,
    LineArrangementError, arrange_line_segments_with_explicit_arcs_and_provenance,
};
use crate::bezier::{CubicBezier, QuadraticBezier, RationalQuadraticBezier};
use crate::curve_cell::{
    CurveArrangementCellError, CurveArrangementCellGraph, build_line_mixed_bezier_cell_graph,
};
use crate::mixed_bezier_arrangement::{
    LineQuadraticBezierArrangementError, LineQuadraticBezierArrangementEvent,
    MixedLineArrangementBreakpoint, MixedLineArrangementFragment, QuadraticBezierRealFragment,
    arrange_line_segments_with_quadratic_beziers_and_provenance,
};
use crate::mixed_conic_arrangement::{
    LineRationalQuadraticBezierArrangementError, LineRationalQuadraticBezierArrangementEvent,
    MixedConicLineArrangementBreakpoint, RationalQuadraticBezierRealFragment,
    arrange_line_segments_with_rational_quadratic_beziers_and_provenance,
};
use crate::mixed_cubic_arrangement::{
    CubicBezierRealFragment, LineCubicBezierArrangementError, LineCubicBezierArrangementEvent,
    MixedCubicLineArrangementBreakpoint, arrange_line_segments_with_cubic_beziers_and_provenance,
};
use crate::provenance::PathProvenance;
use crate::segment::LinePathSegment;

/// Errors that prevent the bounded mixed-family scheduler from certifying topology.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LineMixedBezierArrangementError {
    /// A line/quadratic sub-scheduler rejected exact replay.
    Quadratic(LineQuadraticBezierArrangementError),
    /// A line/explicit-arc sub-scheduler rejected exact replay.
    Arc(LineArrangementError),
    /// A line/cubic sub-scheduler rejected exact replay.
    Cubic(LineCubicBezierArrangementError),
    /// A line/conic sub-scheduler rejected exact replay.
    RationalQuadratic(LineRationalQuadraticBezierArrangementError),
    /// Exact comparison of merged line split parameters was undecidable.
    UndecidableLineOrder { line: usize },
    /// Exact endpoint de-duplication was undecidable.
    UndecidablePointEquality,
    /// Two non-line curve fragments could not be certified disjoint by exact hull boxes.
    UnsupportedCurveCurveInteraction {
        left: MixedCurveFragmentRef,
        right: MixedCurveFragmentRef,
    },
    /// Exact tangent ordering around a retained cell vertex was undecidable.
    UndecidableCellOrder { vertex: usize },
    /// Exact Green-integral face-area replay was unavailable for a retained cell edge.
    UndecidableCellArea { edge: usize },
}

/// Source identity for a non-line fragment in the bounded mixed scheduler.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MixedCurveFragmentRef {
    /// Explicit circular-arc fragment index.
    ExplicitArc(usize),
    /// Quadratic Bezier fragment index.
    Quadratic(usize),
    /// Cubic Bezier fragment index.
    Cubic(usize),
    /// Rational quadratic conic fragment index.
    RationalQuadratic(usize),
}

/// Cached exact facts for a bounded mixed line/arc/Bezier/conic schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct LineMixedBezierArrangementFacts {
    /// Exact-set facts across all retained input line and curve controls.
    pub input_exact: RealExactSetFacts,
    /// Exact-set facts across emitted line and curve fragment controls.
    pub fragment_exact: RealExactSetFacts,
    /// Source provenance for this schedule.
    pub provenance: PathProvenance,
}

/// Bounded mixed line plus explicit-arc/quadratic/cubic/conic arrangement schedule.
///
/// Pairwise exact line/curve schedulers discover events and native curve
/// fragments. This report then merges every line breakpoint into one retained
/// line split set and validates that non-line fragments are mutually separated
/// by exact convex-hull boxes before building a shared
/// [`CurveArrangementCellGraph`]. If hull separation cannot be certified, the
/// function returns [`LineMixedBezierArrangementError::UnsupportedCurveCurveInteraction`]
/// rather than accepting a possible unsplit curve-curve crossing.
#[derive(Clone, Debug, PartialEq)]
pub struct LineMixedBezierArrangementReport {
    /// Retained input line segments.
    pub lines: Vec<LinePathSegment>,
    /// Retained input explicit circular arcs.
    pub arcs: Vec<ExplicitCircularArc>,
    /// Retained input quadratic Beziers.
    pub quadratic_curves: Vec<QuadraticBezier>,
    /// Retained input cubic Beziers.
    pub cubic_curves: Vec<CubicBezier>,
    /// Retained input rational quadratic conics.
    pub rational_quadratic_curves: Vec<RationalQuadraticBezier>,
    /// Certified or unknown line/arc events.
    pub arc_events: Vec<LineArcArrangementEvent>,
    /// Certified or unknown line/quadratic events.
    pub quadratic_events: Vec<LineQuadraticBezierArrangementEvent>,
    /// Certified or unknown line/cubic events.
    pub cubic_events: Vec<LineCubicBezierArrangementEvent>,
    /// Certified or unknown line/conic events.
    pub rational_quadratic_events: Vec<LineRationalQuadraticBezierArrangementEvent>,
    /// Merged line breakpoints induced by every retained curve family.
    pub line_breakpoints: Vec<Vec<MixedLineArrangementBreakpoint>>,
    /// Positive-length merged line fragments.
    pub line_fragments: Vec<MixedLineArrangementFragment>,
    /// Positive-length explicit-arc fragments from the pairwise exact scheduler.
    pub arc_fragments: Vec<ExplicitArcArrangementFragment>,
    /// Positive-length quadratic fragments from the pairwise exact scheduler.
    pub quadratic_fragments: Vec<QuadraticBezierRealFragment>,
    /// Positive-length cubic fragments from the pairwise exact scheduler.
    pub cubic_fragments: Vec<CubicBezierRealFragment>,
    /// Positive-length homogeneous conic fragments from the pairwise exact scheduler.
    pub rational_quadratic_fragments: Vec<RationalQuadraticBezierRealFragment>,
    /// Shared retained topology graph over merged line and separated curve fragments.
    pub cell_graph: CurveArrangementCellGraph,
    /// Cached exact facts for the retained schedule.
    pub facts: LineMixedBezierArrangementFacts,
}

/// Arrange retained line segments against separated quadratic/cubic/conic families.
pub fn arrange_line_segments_with_mixed_beziers(
    lines: &[LinePathSegment],
    quadratic_curves: &[QuadraticBezier],
    cubic_curves: &[CubicBezier],
    rational_quadratic_curves: &[RationalQuadraticBezier],
    policy: PredicatePolicy,
) -> Result<LineMixedBezierArrangementReport, LineMixedBezierArrangementError> {
    arrange_line_segments_with_mixed_curves_and_provenance(
        lines,
        &[],
        quadratic_curves,
        cubic_curves,
        rational_quadratic_curves,
        policy,
        PathProvenance::native(),
    )
}

/// Arrange retained line segments against separated quadratic/cubic/conic families with provenance.
pub fn arrange_line_segments_with_mixed_beziers_and_provenance(
    lines: &[LinePathSegment],
    quadratic_curves: &[QuadraticBezier],
    cubic_curves: &[CubicBezier],
    rational_quadratic_curves: &[RationalQuadraticBezier],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<LineMixedBezierArrangementReport, LineMixedBezierArrangementError> {
    arrange_line_segments_with_mixed_curves_and_provenance(
        lines,
        &[],
        quadratic_curves,
        cubic_curves,
        rational_quadratic_curves,
        policy,
        provenance,
    )
}

/// Arrange retained line segments against separated explicit arcs and Bezier/conic families.
pub fn arrange_line_segments_with_mixed_curves(
    lines: &[LinePathSegment],
    arcs: &[ExplicitCircularArc],
    quadratic_curves: &[QuadraticBezier],
    cubic_curves: &[CubicBezier],
    rational_quadratic_curves: &[RationalQuadraticBezier],
    policy: PredicatePolicy,
) -> Result<LineMixedBezierArrangementReport, LineMixedBezierArrangementError> {
    arrange_line_segments_with_mixed_curves_and_provenance(
        lines,
        arcs,
        quadratic_curves,
        cubic_curves,
        rational_quadratic_curves,
        policy,
        PathProvenance::native(),
    )
}

/// Arrange retained line segments against separated explicit arcs and Bezier/conic families with provenance.
pub fn arrange_line_segments_with_mixed_curves_and_provenance(
    lines: &[LinePathSegment],
    arcs: &[ExplicitCircularArc],
    quadratic_curves: &[QuadraticBezier],
    cubic_curves: &[CubicBezier],
    rational_quadratic_curves: &[RationalQuadraticBezier],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<LineMixedBezierArrangementReport, LineMixedBezierArrangementError> {
    let arc_report = arrange_line_segments_with_explicit_arcs_and_provenance(
        lines,
        arcs,
        policy,
        provenance.clone(),
    )
    .map_err(LineMixedBezierArrangementError::Arc)?;
    let quadratic_report = arrange_line_segments_with_quadratic_beziers_and_provenance(
        lines,
        quadratic_curves,
        policy,
        provenance.clone(),
    )
    .map_err(LineMixedBezierArrangementError::Quadratic)?;
    let cubic_report = arrange_line_segments_with_cubic_beziers_and_provenance(
        lines,
        cubic_curves,
        policy,
        provenance.clone(),
    )
    .map_err(LineMixedBezierArrangementError::Cubic)?;
    let rational_quadratic_report =
        arrange_line_segments_with_rational_quadratic_beziers_and_provenance(
            lines,
            rational_quadratic_curves,
            policy,
            provenance.clone(),
        )
        .map_err(LineMixedBezierArrangementError::RationalQuadratic)?;

    let mut line_breakpoints = seed_line_breakpoints(lines);
    merge_arc_line_breakpoints(&mut line_breakpoints, &arc_report.line_breakpoints, policy)?;
    merge_quadratic_line_breakpoints(
        &mut line_breakpoints,
        &quadratic_report.line_breakpoints,
        policy,
    )?;
    merge_cubic_line_breakpoints(
        &mut line_breakpoints,
        &cubic_report.line_breakpoints,
        policy,
    )?;
    merge_conic_line_breakpoints(
        &mut line_breakpoints,
        &rational_quadratic_report.line_breakpoints,
        policy,
    )?;
    sort_and_dedup_line_breakpoints(&mut line_breakpoints, policy)?;

    let line_fragments = build_line_fragments(&line_breakpoints, policy)?;
    validate_curve_fragment_separation(
        &arc_report.arc_fragments,
        &quadratic_report.bezier_fragments,
        &cubic_report.cubic_fragments,
        &rational_quadratic_report.conic_fragments,
        policy,
    )?;
    let cell_graph = build_line_mixed_bezier_cell_graph(
        &line_fragments,
        &arc_report.arc_fragments,
        &quadratic_report.bezier_fragments,
        &cubic_report.cubic_fragments,
        &rational_quadratic_report.conic_fragments,
        policy,
    )
    .map_err(mixed_error_from_curve_cell_error)?;
    let facts = LineMixedBezierArrangementFacts {
        input_exact: input_exact_facts(
            lines,
            arcs,
            quadratic_curves,
            cubic_curves,
            rational_quadratic_curves,
        ),
        fragment_exact: fragment_exact_facts(
            &line_fragments,
            &arc_report.arc_fragments,
            &quadratic_report.bezier_fragments,
            &cubic_report.cubic_fragments,
            &rational_quadratic_report.conic_fragments,
        ),
        provenance,
    };

    Ok(LineMixedBezierArrangementReport {
        lines: lines.to_vec(),
        arcs: arcs.to_vec(),
        quadratic_curves: quadratic_curves.to_vec(),
        cubic_curves: cubic_curves.to_vec(),
        rational_quadratic_curves: rational_quadratic_curves.to_vec(),
        arc_events: arc_report.events,
        quadratic_events: quadratic_report.events,
        cubic_events: cubic_report.events,
        rational_quadratic_events: rational_quadratic_report.events,
        line_breakpoints,
        line_fragments,
        arc_fragments: arc_report.arc_fragments,
        quadratic_fragments: quadratic_report.bezier_fragments,
        cubic_fragments: cubic_report.cubic_fragments,
        rational_quadratic_fragments: rational_quadratic_report.conic_fragments,
        cell_graph,
        facts,
    })
}

fn mixed_error_from_curve_cell_error(
    error: CurveArrangementCellError,
) -> LineMixedBezierArrangementError {
    match error {
        CurveArrangementCellError::UndecidablePointEquality => {
            LineMixedBezierArrangementError::UndecidablePointEquality
        }
        CurveArrangementCellError::UndecidableCellOrder { vertex } => {
            LineMixedBezierArrangementError::UndecidableCellOrder { vertex }
        }
        CurveArrangementCellError::UndecidableCellArea { edge } => {
            LineMixedBezierArrangementError::UndecidableCellArea { edge }
        }
    }
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

fn merge_arc_line_breakpoints(
    target: &mut [Vec<MixedLineArrangementBreakpoint>],
    source: &[Vec<LineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineMixedBezierArrangementError> {
    for (line, points) in source.iter().enumerate() {
        for point in points {
            insert_line_breakpoint(
                &mut target[line],
                MixedLineArrangementBreakpoint {
                    line: point.segment,
                    point: point.point.clone(),
                    parameter_numerator: point.parameter_numerator.clone(),
                    parameter_denominator: point.parameter_denominator.clone(),
                },
                policy,
            )?;
        }
    }
    Ok(())
}

fn merge_quadratic_line_breakpoints(
    target: &mut [Vec<MixedLineArrangementBreakpoint>],
    source: &[Vec<MixedLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineMixedBezierArrangementError> {
    for (line, points) in source.iter().enumerate() {
        for point in points {
            insert_line_breakpoint(&mut target[line], point.clone(), policy)?;
        }
    }
    Ok(())
}

fn merge_cubic_line_breakpoints(
    target: &mut [Vec<MixedLineArrangementBreakpoint>],
    source: &[Vec<MixedCubicLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineMixedBezierArrangementError> {
    for (line, points) in source.iter().enumerate() {
        for point in points {
            insert_line_breakpoint(
                &mut target[line],
                MixedLineArrangementBreakpoint {
                    line: point.line,
                    point: point.point.clone(),
                    parameter_numerator: point.parameter_numerator.clone(),
                    parameter_denominator: point.parameter_denominator.clone(),
                },
                policy,
            )?;
        }
    }
    Ok(())
}

fn merge_conic_line_breakpoints(
    target: &mut [Vec<MixedLineArrangementBreakpoint>],
    source: &[Vec<MixedConicLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineMixedBezierArrangementError> {
    for (line, points) in source.iter().enumerate() {
        for point in points {
            insert_line_breakpoint(
                &mut target[line],
                MixedLineArrangementBreakpoint {
                    line: point.line,
                    point: point.point.clone(),
                    parameter_numerator: point.parameter_numerator.clone(),
                    parameter_denominator: point.parameter_denominator.clone(),
                },
                policy,
            )?;
        }
    }
    Ok(())
}

fn insert_line_breakpoint(
    breakpoints: &mut Vec<MixedLineArrangementBreakpoint>,
    point: MixedLineArrangementBreakpoint,
    policy: PredicatePolicy,
) -> Result<(), LineMixedBezierArrangementError> {
    for existing in breakpoints.iter() {
        match point2_equal_with_policy(&existing.point, &point.point, policy).value() {
            Some(true) => return Ok(()),
            Some(false) => {}
            None => return Err(LineMixedBezierArrangementError::UndecidablePointEquality),
        }
    }
    breakpoints.push(point);
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
) -> Result<(), LineMixedBezierArrangementError> {
    for (line_index, points) in breakpoints.iter_mut().enumerate() {
        for left in 0..points.len() {
            for right in (left + 1)..points.len() {
                compare_line_parameters(&points[left], &points[right], policy).ok_or(
                    LineMixedBezierArrangementError::UndecidableLineOrder { line: line_index },
                )?;
            }
        }
        points.sort_by(|left, right| {
            compare_line_parameters(left, right, policy)
                .expect("merged line breakpoint order was certified before sorting")
        });
        let mut deduped: Vec<MixedLineArrangementBreakpoint> = Vec::new();
        for point in points.drain(..) {
            if let Some(last) = deduped.last() {
                match point2_equal_with_policy(&last.point, &point.point, policy).value() {
                    Some(true) => continue,
                    Some(false) => {}
                    None => return Err(LineMixedBezierArrangementError::UndecidablePointEquality),
                }
            }
            deduped.push(point);
        }
        *points = deduped;
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

fn build_line_fragments(
    breakpoints: &[Vec<MixedLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<Vec<MixedLineArrangementFragment>, LineMixedBezierArrangementError> {
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

#[derive(Clone, Debug)]
struct FragmentBox {
    source: MixedCurveFragmentRef,
    x_min: Real,
    x_max: Real,
    y_min: Real,
    y_max: Real,
}

fn validate_curve_fragment_separation(
    arcs: &[ExplicitArcArrangementFragment],
    quadratics: &[QuadraticBezierRealFragment],
    cubics: &[CubicBezierRealFragment],
    conics: &[RationalQuadraticBezierRealFragment],
    policy: PredicatePolicy,
) -> Result<(), LineMixedBezierArrangementError> {
    let mut boxes = Vec::new();
    for (index, fragment) in arcs.iter().enumerate() {
        boxes.push(FragmentBox {
            source: MixedCurveFragmentRef::ExplicitArc(index),
            ..box_from_explicit_arc(fragment, policy)?
        });
    }
    for (index, fragment) in quadratics.iter().enumerate() {
        boxes.push(FragmentBox {
            source: MixedCurveFragmentRef::Quadratic(index),
            ..box_from_points(
                [
                    fragment.curve.start(),
                    fragment.curve.control(),
                    fragment.curve.end(),
                ],
                policy,
            )?
        });
    }
    for (index, fragment) in cubics.iter().enumerate() {
        boxes.push(FragmentBox {
            source: MixedCurveFragmentRef::Cubic(index),
            ..box_from_points(
                [
                    fragment.curve.start(),
                    fragment.curve.control0(),
                    fragment.curve.control1(),
                    fragment.curve.end(),
                ],
                policy,
            )?
        });
    }
    for (index, fragment) in conics.iter().enumerate() {
        let start = affine_homogeneous_point(&fragment.start_control, policy)?;
        let control = affine_homogeneous_point(&fragment.control, policy)?;
        let end = affine_homogeneous_point(&fragment.end_control, policy)?;
        boxes.push(FragmentBox {
            source: MixedCurveFragmentRef::RationalQuadratic(index),
            ..box_from_points([&start, &control, &end], policy)?
        });
    }
    for left in 0..boxes.len() {
        for right in (left + 1)..boxes.len() {
            if !boxes_strictly_separated(&boxes[left], &boxes[right], policy)? {
                return Err(
                    LineMixedBezierArrangementError::UnsupportedCurveCurveInteraction {
                        left: boxes[left].source,
                        right: boxes[right].source,
                    },
                );
            }
        }
    }
    Ok(())
}

/// Build a sweep-aware exact hull box for an explicit circular-arc fragment.
///
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), draws the boundary used here: topology is accepted only from
/// exact predicates over retained objects. The only interior extrema of an
/// axis-aligned circular-arc box occur at the four cardinal points of the
/// retained circle, so this routine asks the arc sweep predicate whether each
/// cardinal witness is on the fragment. This is the same predicate-carrier
/// discipline used by circular-arc arrangements such as CGAL
/// `Arrangement_on_surface_2`; no sampled angle or approximate tessellation is
/// allowed to shrink the box.
fn box_from_explicit_arc(
    fragment: &ExplicitArcArrangementFragment,
    policy: PredicatePolicy,
) -> Result<FragmentBox, LineMixedBezierArrangementError> {
    let arc = &fragment.arc;
    let mut hull = box_from_points([arc.start(), arc.end()], policy)?;
    let center = arc.center();
    let radius = arc.radius();
    let cardinal_points = [
        Point2::new(center.x.clone() + radius.clone(), center.y.clone()),
        Point2::new(center.x.clone() - radius.clone(), center.y.clone()),
        Point2::new(center.x.clone(), center.y.clone() + radius.clone()),
        Point2::new(center.x.clone(), center.y.clone() - radius.clone()),
    ];

    for point in cardinal_points {
        match arc.classify_point(&point, policy) {
            ExplicitArcPointClassification::OnArc => {
                update_min_max(&mut hull.x_min, &mut hull.x_max, &point.x, policy)?;
                update_min_max(&mut hull.y_min, &mut hull.y_max, &point.y, policy)?;
            }
            ExplicitArcPointClassification::OnCircleOutsideSweep
            | ExplicitArcPointClassification::OffCircle => {}
            ExplicitArcPointClassification::Unknown => {
                return Err(LineMixedBezierArrangementError::UndecidablePointEquality);
            }
        }
    }
    hull.source = MixedCurveFragmentRef::ExplicitArc(usize::MAX);
    Ok(hull)
}

fn box_from_points<'a, const N: usize>(
    points: [&'a Point2; N],
    policy: PredicatePolicy,
) -> Result<FragmentBox, LineMixedBezierArrangementError> {
    let mut x_min = points[0].x.clone();
    let mut x_max = points[0].x.clone();
    let mut y_min = points[0].y.clone();
    let mut y_max = points[0].y.clone();
    for point in points.iter().skip(1) {
        update_min_max(&mut x_min, &mut x_max, &point.x, policy)?;
        update_min_max(&mut y_min, &mut y_max, &point.y, policy)?;
    }
    Ok(FragmentBox {
        source: MixedCurveFragmentRef::Quadratic(usize::MAX),
        x_min,
        x_max,
        y_min,
        y_max,
    })
}

fn update_min_max(
    min: &mut Real,
    max: &mut Real,
    value: &Real,
    policy: PredicatePolicy,
) -> Result<(), LineMixedBezierArrangementError> {
    match compare_reals_with_policy(value, min, policy).value() {
        Some(Ordering::Less) => *min = value.clone(),
        Some(Ordering::Equal | Ordering::Greater) => {}
        None => return Err(LineMixedBezierArrangementError::UndecidablePointEquality),
    }
    match compare_reals_with_policy(value, max, policy).value() {
        Some(Ordering::Greater) => *max = value.clone(),
        Some(Ordering::Equal | Ordering::Less) => {}
        None => return Err(LineMixedBezierArrangementError::UndecidablePointEquality),
    }
    Ok(())
}

fn boxes_strictly_separated(
    left: &FragmentBox,
    right: &FragmentBox,
    policy: PredicatePolicy,
) -> Result<bool, LineMixedBezierArrangementError> {
    Ok(is_less(&left.x_max, &right.x_min, policy)?
        || is_less(&right.x_max, &left.x_min, policy)?
        || is_less(&left.y_max, &right.y_min, policy)?
        || is_less(&right.y_max, &left.y_min, policy)?)
}

fn is_less(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> Result<bool, LineMixedBezierArrangementError> {
    match compare_reals_with_policy(left, right, policy).value() {
        Some(Ordering::Less) => Ok(true),
        Some(Ordering::Equal | Ordering::Greater) => Ok(false),
        None => Err(LineMixedBezierArrangementError::UndecidablePointEquality),
    }
}

fn affine_homogeneous_point(
    point: &crate::bezier_arrangement::HomogeneousPoint2,
    policy: PredicatePolicy,
) -> Result<Point2, LineMixedBezierArrangementError> {
    match compare_reals_with_policy(&point.w, &Real::zero(), policy).value() {
        Some(Ordering::Less | Ordering::Greater) => Ok(Point2::new(
            (point.x.clone() / point.w.clone())
                .map_err(|_| LineMixedBezierArrangementError::UndecidablePointEquality)?,
            (point.y.clone() / point.w.clone())
                .map_err(|_| LineMixedBezierArrangementError::UndecidablePointEquality)?,
        )),
        Some(Ordering::Equal) | None => {
            Err(LineMixedBezierArrangementError::UndecidablePointEquality)
        }
    }
}

fn input_exact_facts(
    lines: &[LinePathSegment],
    arcs: &[ExplicitCircularArc],
    quadratics: &[QuadraticBezier],
    cubics: &[CubicBezier],
    conics: &[RationalQuadraticBezier],
) -> RealExactSetFacts {
    let mut values = Vec::new();
    for line in lines {
        values.extend([
            &line.start().x,
            &line.start().y,
            &line.end().x,
            &line.end().y,
        ]);
    }
    for arc in arcs {
        values.extend([
            &arc.center().x,
            &arc.center().y,
            arc.radius(),
            &arc.start().x,
            &arc.start().y,
            &arc.end().x,
            &arc.end().y,
        ]);
    }
    for curve in quadratics {
        values.extend([
            &curve.start().x,
            &curve.start().y,
            &curve.control().x,
            &curve.control().y,
            &curve.end().x,
            &curve.end().y,
        ]);
    }
    for curve in cubics {
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
    for curve in conics {
        values.extend([
            &curve.start().x,
            &curve.start().y,
            &curve.control().x,
            &curve.control().y,
            curve.control_weight(),
            &curve.end().x,
            &curve.end().y,
        ]);
    }
    Real::exact_set_facts(values)
}

fn fragment_exact_facts(
    lines: &[MixedLineArrangementFragment],
    arcs: &[ExplicitArcArrangementFragment],
    quadratics: &[QuadraticBezierRealFragment],
    cubics: &[CubicBezierRealFragment],
    conics: &[RationalQuadraticBezierRealFragment],
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
    for fragment in arcs {
        values.extend([
            &fragment.arc.center().x,
            &fragment.arc.center().y,
            fragment.arc.radius(),
            &fragment.arc.start().x,
            &fragment.arc.start().y,
            &fragment.arc.end().x,
            &fragment.arc.end().y,
        ]);
    }
    for fragment in quadratics {
        values.extend([
            &fragment.curve.start().x,
            &fragment.curve.start().y,
            &fragment.curve.control().x,
            &fragment.curve.control().y,
            &fragment.curve.end().x,
            &fragment.curve.end().y,
        ]);
    }
    for fragment in cubics {
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
    for fragment in conics {
        values.extend([
            &fragment.start_control.x,
            &fragment.start_control.y,
            &fragment.start_control.w,
            &fragment.control.x,
            &fragment.control.y,
            &fragment.control.w,
            &fragment.end_control.x,
            &fragment.end_control.y,
            &fragment.end_control.w,
        ]);
    }
    Real::exact_set_facts(values)
}
