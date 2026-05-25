//! Mixed exact arrangement cleanup for retained lines and cubic Beziers.
//!
//! This module is a retained split scheduler, not a planar-cell extractor and
//! not a boolean operation. It promotes certified line/cubic Bezier events into
//! exact line breakpoints and exact cubic `Real`-parameter breakpoints, then
//! emits positive-length fragments. True cubic support roots are retained by
//! the predicate layer as represented algebraic parameters and point images,
//! then copied here as separate algebraic breakpoint candidates with exact line
//! parameter images. They remain out of the rational fragment lists until this
//! scheduler can order and materialize algebraic split parameters directly.

use std::cmp::Ordering;
use std::collections::BTreeMap;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal_with_policy};
use hyperreal::{Real, RealExactSetFacts};
use hypersolve::{
    AlgebraicRootPolynomialImageReport, AlgebraicRootPolynomialImageStatus,
    AlgebraicRootRepresentation, transform_algebraic_root_polynomial_image,
};

use crate::bezier::CubicBezier;
use crate::bezier_arrangement::{
    LineCubicAlgebraicPointDomain, LineCubicAlgebraicRootDomain,
    LineCubicBezierAlgebraicPointImage, LineCubicBezierAlgebraicSupportRoot,
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

/// Certified domain status for a retained algebraic line/cubic breakpoint candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicBreakpointDomain {
    /// Cubic parameter, point image, and line parameter are inside the retained pair domains.
    InsideLineAndCurve,
    /// At least one retained parameter/image is certified outside the pair domains.
    OutsideLineOrCurve,
    /// Exact image construction or interval comparison did not decide.
    Unknown,
}

/// Retained algebraic breakpoint candidate for a true line/cubic support root.
///
/// This is the mixed-scheduler counterpart to
/// [`LineCubicBezierAlgebraicSupportRoot`]. It keeps the represented cubic
/// parameter, the exact algebraic point image, and a normalized line-parameter
/// image `dot(B(alpha)-line.start, line.end-line.start) / |line|^2`.
///
/// The line-parameter image is constructed with `hypersolve`'s resultant-based
/// algebraic polynomial image. This directly follows Yap, "Towards Exact
/// Geometric Computation" (1997): the scheduler retains exact algebraic
/// objects with replayable evidence, but it does not insert them into the
/// rational breakpoint/fragments lists until ordering and construction are
/// supported. The elimination step is the Sylvester resultant construction
/// used by Sylvester (1853) and the certified-root discipline of Collins and
/// Loos, "Real Zeros of Polynomials" (1982).
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierAlgebraicBreakpoint {
    /// Line segment index.
    pub line: usize,
    /// Cubic Bezier index.
    pub curve: usize,
    /// Represented algebraic cubic parameter.
    pub cubic_parameter: AlgebraicRootRepresentation,
    /// Exact represented point image on the cubic.
    pub point_image: LineCubicBezierAlgebraicPointImage,
    /// Exact represented normalized line parameter image.
    pub line_parameter: AlgebraicRootPolynomialImageReport,
    /// Certified relation of the retained algebraic candidate to both source domains.
    pub domain: LineCubicBezierAlgebraicBreakpointDomain,
}

/// Certified order relation between two represented line/cubic breakpoint candidates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicBreakpointOrderClass {
    /// The left breakpoint parameter is certified before the right parameter.
    Before,
    /// The represented parameters are certified equal from exact root witnesses.
    Equal,
    /// The left breakpoint parameter is certified after the right parameter.
    After,
    /// The isolating intervals overlap or exact comparison did not decide.
    Unknown,
}

/// Pairwise ordering evidence for retained algebraic line/cubic breakpoints.
///
/// A candidate carries two relevant represented values: the cubic source
/// parameter and the normalized line parameter image. The scheduler records a
/// curve order only when two candidates share a cubic source, and a line order
/// only when they share a retained line. Orders are certified from exact root
/// witnesses or separated Sturm/resultant isolating intervals; overlapping
/// intervals stay [`LineCubicBezierAlgebraicBreakpointOrderClass::Unknown`].
///
/// This follows Yap, "Towards Exact Geometric Computation" (1997): exact
/// algebraic order evidence is retained as a report, but it does not mutate
/// the concrete `Real` breakpoint lists until construction can consume the
/// represented roots without sampling. The polynomial images are the
/// Sylvester-resultant construction used by Sylvester (1853) and Collins and
/// Loos, "Real Zeros of Polynomials" (1982).
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierAlgebraicBreakpointOrder {
    /// Index in [`LineCubicBezierArrangementReport::algebraic_breakpoints`].
    pub left: usize,
    /// Index in [`LineCubicBezierArrangementReport::algebraic_breakpoints`].
    pub right: usize,
    /// Same-curve order, when both candidates came from the same cubic.
    pub cubic_order: Option<LineCubicBezierAlgebraicBreakpointOrderClass>,
    /// Same-line order, when both candidates came from the same line.
    pub line_order: Option<LineCubicBezierAlgebraicBreakpointOrderClass>,
}

/// Source parameter space for an ordered retained algebraic breakpoint sequence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicBreakpointSequenceSource {
    /// Breakpoints ordered by normalized parameter on a retained line segment.
    Line(usize),
    /// Breakpoints ordered by source parameter on a retained cubic Bezier.
    Curve(usize),
}

/// Sequence readiness for represented algebraic line/cubic breakpoints.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicBreakpointSequenceClass {
    /// All pairwise comparisons for this source were certified, so `breakpoints` is sorted.
    Ordered,
    /// At least one pair was equal, missing, or undecidable; insertion order is retained.
    Ambiguous,
}

/// Exact blocker that prevents a retained algebraic breakpoint sequence from being sorted.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicBreakpointSequenceBlocker {
    /// Pairwise order evidence was not emitted for this same-source pair.
    MissingOrder { left: usize, right: usize },
    /// Pairwise order evidence exists but the isolated algebraic intervals still overlap.
    UnknownOrder { left: usize, right: usize },
    /// Distinct retained candidates collapsed to the same represented source parameter.
    EqualOrder { left: usize, right: usize },
}

/// Ordered retained algebraic breakpoint indices for one line or cubic source.
///
/// This is a readiness report for future algebraic split materialization, not
/// a fragment list. The indices address
/// [`LineCubicBezierArrangementReport::algebraic_breakpoints`]. When the
/// sequence is [`LineCubicBezierAlgebraicBreakpointSequenceClass::Ordered`],
/// every pair on the same source has exact order evidence and `breakpoints` is
/// sorted in that source parameter. When it is ambiguous, blockers describe the
/// missing or undecidable comparisons and the original discovery order is
/// preserved.
///
/// The design follows Yap, "Towards Exact Geometric Computation" (1997): exact
/// algebraic decisions are retained as first-class certificates and uncertain
/// decisions remain explicit. The pairwise certificates consumed here are
/// Sturm-isolated root comparisons in the sense of Collins and Loos, "Real
/// Zeros of Polynomials" (1982), with line-parameter images constructed by the
/// Sylvester resultant of Sylvester (1853).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LineCubicBezierAlgebraicBreakpointSequence {
    /// Source whose parameter orders this sequence.
    pub source: LineCubicBezierAlgebraicBreakpointSequenceSource,
    /// Breakpoint indices, sorted only when `class == Ordered`.
    pub breakpoints: Vec<usize>,
    /// Whether this source sequence is ready for exact algebraic split construction.
    pub class: LineCubicBezierAlgebraicBreakpointSequenceClass,
    /// Exact reasons that prevented sorting.
    pub blockers: Vec<LineCubicBezierAlgebraicBreakpointSequenceBlocker>,
}

/// Boundary of a retained algebraic source span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicSourceSpanBoundary {
    /// The exact source parameter `0`.
    SourceStart,
    /// An index in [`LineCubicBezierArrangementReport::algebraic_breakpoints`].
    Breakpoint(usize),
    /// The exact source parameter `1`.
    SourceEnd,
}

/// Conservative source-parameter interval between ordered algebraic breakpoints.
///
/// Spans are emitted only from
/// [`LineCubicBezierAlgebraicBreakpointSequenceClass::Ordered`] sequences.
/// They are not curve fragments. A span records the smallest retained interval
/// that is guaranteed to contain the true source subrange between two adjacent
/// ordered boundaries. For represented algebraic endpoints this uses the
/// Sturm isolating interval retained by `hypersolve`; for line sources it uses
/// the transformed normalized line-parameter image.
///
/// This is the Yap-style certificate/object separation from Yap, "Towards
/// Exact Geometric Computation" (1997): exact construction can later replay
/// these intervals and represented roots, while this scheduler avoids sampling
/// or pretending the nonlinear algebraic boundary is a concrete `Real`
/// breakpoint. The isolating-interval discipline is the Collins-Loos model
/// from Collins and Loos, "Real Zeros of Polynomials" (1982); line source
/// images rely on the Sylvester resultant construction of Sylvester (1853).
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierAlgebraicSourceSpan {
    /// Source whose parameter space owns this span.
    pub source: LineCubicBezierAlgebraicBreakpointSequenceSource,
    /// Left adjacent boundary.
    pub left: LineCubicBezierAlgebraicSourceSpanBoundary,
    /// Right adjacent boundary.
    pub right: LineCubicBezierAlgebraicSourceSpanBoundary,
    /// Conservative lower source parameter bound.
    pub parameter_lower: Real,
    /// Conservative upper source parameter bound.
    pub parameter_upper: Real,
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
    /// Algebraic breakpoint candidates retained from true cubic support roots.
    pub algebraic_breakpoints: Vec<LineCubicBezierAlgebraicBreakpoint>,
    /// Pairwise exact order evidence for retained algebraic breakpoints.
    pub algebraic_breakpoint_orders: Vec<LineCubicBezierAlgebraicBreakpointOrder>,
    /// Per-source retained algebraic breakpoint sequences derived from exact order evidence.
    pub algebraic_breakpoint_sequences: Vec<LineCubicBezierAlgebraicBreakpointSequence>,
    /// Conservative source spans induced by certified algebraic breakpoint sequences.
    pub algebraic_source_spans: Vec<LineCubicBezierAlgebraicSourceSpan>,
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
    let mut algebraic_breakpoints = Vec::new();

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
            algebraic_breakpoints.extend(
                retained_algebraic_breakpoints(
                    line_index,
                    line,
                    curve_index,
                    curve,
                    &intersection,
                    policy,
                )
                .into_iter()
                .filter(|candidate| {
                    candidate.domain == LineCubicBezierAlgebraicBreakpointDomain::InsideLineAndCurve
                }),
            );
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
    let algebraic_breakpoint_orders =
        algebraic_cubic_breakpoint_orders(&algebraic_breakpoints, policy);
    let algebraic_breakpoint_sequences =
        algebraic_cubic_breakpoint_sequences(&algebraic_breakpoints, &algebraic_breakpoint_orders);
    let algebraic_source_spans =
        algebraic_cubic_source_spans(&algebraic_breakpoints, &algebraic_breakpoint_sequences);
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
        algebraic_breakpoints,
        algebraic_breakpoint_orders,
        algebraic_breakpoint_sequences,
        algebraic_source_spans,
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

fn retained_algebraic_breakpoints(
    line_index: usize,
    line: &LinePathSegment,
    curve_index: usize,
    curve: &CubicBezier,
    intersection: &LineCubicBezierIntersectionReport,
    policy: PredicatePolicy,
) -> Vec<LineCubicBezierAlgebraicBreakpoint> {
    intersection
        .algebraic_support_roots
        .iter()
        .filter_map(|root| {
            let line_parameter =
                algebraic_line_parameter_image(line, curve, &root.parameter, policy)?;
            let domain = classify_algebraic_breakpoint_domain(root, &line_parameter, policy);
            Some(LineCubicBezierAlgebraicBreakpoint {
                line: line_index,
                curve: curve_index,
                cubic_parameter: root.parameter.clone(),
                point_image: root.point_image.clone(),
                line_parameter,
                domain,
            })
        })
        .collect()
}

fn algebraic_line_parameter_image(
    line: &LinePathSegment,
    curve: &CubicBezier,
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<AlgebraicRootPolynomialImageReport> {
    // The normalized line parameter is a polynomial image of the same cubic
    // source parameter:
    //
    //     s(t) = dot(B(t)-L0, L1-L0) / |L1-L0|^2.
    //
    // Keeping this as a represented algebraic image gives the next scheduler
    // stage an exact line-order witness without inserting an unorderable
    // algebraic value into the existing rational breakpoint list. The image
    // construction is the same Sylvester-resultant/Yap retained-object step
    // used by the predicate layer.
    let coefficients = cubic_line_parameter_polynomial(line, curve)?;
    Some(transform_algebraic_root_polynomial_image(
        root,
        &coefficients,
        policy,
    ))
}

fn cubic_line_parameter_polynomial(
    line: &LinePathSegment,
    curve: &CubicBezier,
) -> Option<Vec<Real>> {
    let dx = line.end().x.clone() - line.start().x.clone();
    let dy = line.end().y.clone() - line.start().y.clone();
    let denominator = dx.clone() * dx.clone() + dy.clone() * dy.clone();
    let x = cubic_coordinate_power_coefficients(
        &curve.start().x,
        &curve.control0().x,
        &curve.control1().x,
        &curve.end().x,
    );
    let y = cubic_coordinate_power_coefficients(
        &curve.start().y,
        &curve.control0().y,
        &curve.control1().y,
        &curve.end().y,
    );
    let mut numerator = Vec::with_capacity(4);
    for index in 0..4 {
        let x_coefficient = if index == 0 {
            x[index].clone() - line.start().x.clone()
        } else {
            x[index].clone()
        };
        let y_coefficient = if index == 0 {
            y[index].clone() - line.start().y.clone()
        } else {
            y[index].clone()
        };
        numerator.push(x_coefficient * dx.clone() + y_coefficient * dy.clone());
    }
    numerator
        .into_iter()
        .map(|coefficient| coefficient / denominator.clone())
        .collect::<Result<Vec<_>, _>>()
        .ok()
}

fn cubic_coordinate_power_coefficients(p0: &Real, p1: &Real, p2: &Real, p3: &Real) -> [Real; 4] {
    [
        p0.clone(),
        Real::from(3) * (p1.clone() - p0.clone()),
        Real::from(3) * p0.clone() - Real::from(6) * p1.clone() + Real::from(3) * p2.clone(),
        -p0.clone() + Real::from(3) * p1.clone() - Real::from(3) * p2.clone() + p3.clone(),
    ]
}

fn classify_algebraic_breakpoint_domain(
    root: &LineCubicBezierAlgebraicSupportRoot,
    line_parameter: &AlgebraicRootPolynomialImageReport,
    policy: PredicatePolicy,
) -> LineCubicBezierAlgebraicBreakpointDomain {
    let line_domain = classify_line_parameter_image(line_parameter, policy);
    match (
        root.parameter_domain,
        root.point_image.segment_domain,
        line_domain,
    ) {
        (
            LineCubicAlgebraicRootDomain::InsideUnitInterval,
            LineCubicAlgebraicPointDomain::InsideSegmentBounds,
            Some(true),
        ) => LineCubicBezierAlgebraicBreakpointDomain::InsideLineAndCurve,
        (LineCubicAlgebraicRootDomain::OutsideUnitInterval, _, _)
        | (_, LineCubicAlgebraicPointDomain::OutsideSegmentBounds, _)
        | (_, _, Some(false)) => LineCubicBezierAlgebraicBreakpointDomain::OutsideLineOrCurve,
        _ => LineCubicBezierAlgebraicBreakpointDomain::Unknown,
    }
}

fn classify_line_parameter_image(
    image: &AlgebraicRootPolynomialImageReport,
    policy: PredicatePolicy,
) -> Option<bool> {
    if image.status != AlgebraicRootPolynomialImageStatus::Transformed {
        return None;
    }
    let representation = image.representation.as_ref()?;
    interval_inside_unit(
        &representation.interval.lower,
        &representation.interval.upper,
        policy,
    )
}

fn interval_inside_unit(lower: &Real, upper: &Real, policy: PredicatePolicy) -> Option<bool> {
    let lower_zero = compare_reals_with_policy(lower, &Real::zero(), policy).value()?;
    let upper_one = compare_reals_with_policy(upper, &Real::one(), policy).value()?;
    if matches!(lower_zero, Ordering::Equal | Ordering::Greater)
        && matches!(upper_one, Ordering::Equal | Ordering::Less)
    {
        return Some(true);
    }
    let upper_zero = compare_reals_with_policy(upper, &Real::zero(), policy).value()?;
    let lower_one = compare_reals_with_policy(lower, &Real::one(), policy).value()?;
    if matches!(upper_zero, Ordering::Less) || matches!(lower_one, Ordering::Greater) {
        Some(false)
    } else {
        None
    }
}

fn algebraic_cubic_breakpoint_orders(
    breakpoints: &[LineCubicBezierAlgebraicBreakpoint],
    policy: PredicatePolicy,
) -> Vec<LineCubicBezierAlgebraicBreakpointOrder> {
    let mut orders = Vec::new();
    for left in 0..breakpoints.len() {
        for right in (left + 1)..breakpoints.len() {
            let cubic_order = (breakpoints[left].curve == breakpoints[right].curve).then(|| {
                compare_algebraic_cubic_parameters(
                    &breakpoints[left].cubic_parameter,
                    &breakpoints[right].cubic_parameter,
                    policy,
                )
            });
            let line_order = (breakpoints[left].line == breakpoints[right].line).then(|| {
                compare_algebraic_polynomial_images(
                    &breakpoints[left].line_parameter,
                    &breakpoints[right].line_parameter,
                    policy,
                )
            });
            if cubic_order.is_some() || line_order.is_some() {
                orders.push(LineCubicBezierAlgebraicBreakpointOrder {
                    left,
                    right,
                    cubic_order,
                    line_order,
                });
            }
        }
    }
    orders
}

fn algebraic_cubic_breakpoint_sequences(
    breakpoints: &[LineCubicBezierAlgebraicBreakpoint],
    orders: &[LineCubicBezierAlgebraicBreakpointOrder],
) -> Vec<LineCubicBezierAlgebraicBreakpointSequence> {
    let mut curve_breakpoints: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut line_breakpoints: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (index, breakpoint) in breakpoints.iter().enumerate() {
        curve_breakpoints
            .entry(breakpoint.curve)
            .or_default()
            .push(index);
        line_breakpoints
            .entry(breakpoint.line)
            .or_default()
            .push(index);
    }

    let mut sequences = Vec::new();
    for (curve, indices) in curve_breakpoints {
        sequences.push(algebraic_cubic_breakpoint_sequence_for_source(
            LineCubicBezierAlgebraicBreakpointSequenceSource::Curve(curve),
            indices,
            orders,
        ));
    }
    for (line, indices) in line_breakpoints {
        sequences.push(algebraic_cubic_breakpoint_sequence_for_source(
            LineCubicBezierAlgebraicBreakpointSequenceSource::Line(line),
            indices,
            orders,
        ));
    }
    sequences
}

fn algebraic_cubic_breakpoint_sequence_for_source(
    source: LineCubicBezierAlgebraicBreakpointSequenceSource,
    mut indices: Vec<usize>,
    orders: &[LineCubicBezierAlgebraicBreakpointOrder],
) -> LineCubicBezierAlgebraicBreakpointSequence {
    let mut blockers = Vec::new();
    for left_index in 0..indices.len() {
        for right_index in (left_index + 1)..indices.len() {
            let left = indices[left_index];
            let right = indices[right_index];
            match algebraic_cubic_order_between(source, left, right, orders) {
                Some(LineCubicBezierAlgebraicBreakpointOrderClass::Before)
                | Some(LineCubicBezierAlgebraicBreakpointOrderClass::After) => {}
                Some(LineCubicBezierAlgebraicBreakpointOrderClass::Equal) => {
                    blockers.push(
                        LineCubicBezierAlgebraicBreakpointSequenceBlocker::EqualOrder {
                            left,
                            right,
                        },
                    );
                }
                Some(LineCubicBezierAlgebraicBreakpointOrderClass::Unknown) => {
                    blockers.push(
                        LineCubicBezierAlgebraicBreakpointSequenceBlocker::UnknownOrder {
                            left,
                            right,
                        },
                    );
                }
                None => {
                    blockers.push(
                        LineCubicBezierAlgebraicBreakpointSequenceBlocker::MissingOrder {
                            left,
                            right,
                        },
                    );
                }
            }
        }
    }

    let class = if blockers.is_empty() {
        indices.sort_by(|left, right| {
            algebraic_cubic_ordering_for_sort(source, *left, *right, orders)
                .expect("algebraic source order was certified before sorting")
        });
        LineCubicBezierAlgebraicBreakpointSequenceClass::Ordered
    } else {
        LineCubicBezierAlgebraicBreakpointSequenceClass::Ambiguous
    };

    LineCubicBezierAlgebraicBreakpointSequence {
        source,
        breakpoints: indices,
        class,
        blockers,
    }
}

fn algebraic_cubic_ordering_for_sort(
    source: LineCubicBezierAlgebraicBreakpointSequenceSource,
    left: usize,
    right: usize,
    orders: &[LineCubicBezierAlgebraicBreakpointOrder],
) -> Option<Ordering> {
    if left == right {
        return Some(Ordering::Equal);
    }
    match algebraic_cubic_order_between(source, left, right, orders)? {
        LineCubicBezierAlgebraicBreakpointOrderClass::Before => Some(Ordering::Less),
        LineCubicBezierAlgebraicBreakpointOrderClass::After => Some(Ordering::Greater),
        LineCubicBezierAlgebraicBreakpointOrderClass::Equal => Some(Ordering::Equal),
        LineCubicBezierAlgebraicBreakpointOrderClass::Unknown => None,
    }
}

fn algebraic_cubic_order_between(
    source: LineCubicBezierAlgebraicBreakpointSequenceSource,
    left: usize,
    right: usize,
    orders: &[LineCubicBezierAlgebraicBreakpointOrder],
) -> Option<LineCubicBezierAlgebraicBreakpointOrderClass> {
    let direct = orders
        .iter()
        .find(|order| order.left == left && order.right == right)
        .and_then(|order| algebraic_cubic_order_for_source(source, order));
    if direct.is_some() {
        return direct;
    }
    orders
        .iter()
        .find(|order| order.left == right && order.right == left)
        .and_then(|order| algebraic_cubic_order_for_source(source, order))
        .map(reverse_algebraic_cubic_order)
}

fn algebraic_cubic_order_for_source(
    source: LineCubicBezierAlgebraicBreakpointSequenceSource,
    order: &LineCubicBezierAlgebraicBreakpointOrder,
) -> Option<LineCubicBezierAlgebraicBreakpointOrderClass> {
    match source {
        LineCubicBezierAlgebraicBreakpointSequenceSource::Line(_) => order.line_order,
        LineCubicBezierAlgebraicBreakpointSequenceSource::Curve(_) => order.cubic_order,
    }
}

fn reverse_algebraic_cubic_order(
    order: LineCubicBezierAlgebraicBreakpointOrderClass,
) -> LineCubicBezierAlgebraicBreakpointOrderClass {
    match order {
        LineCubicBezierAlgebraicBreakpointOrderClass::Before => {
            LineCubicBezierAlgebraicBreakpointOrderClass::After
        }
        LineCubicBezierAlgebraicBreakpointOrderClass::After => {
            LineCubicBezierAlgebraicBreakpointOrderClass::Before
        }
        LineCubicBezierAlgebraicBreakpointOrderClass::Equal => {
            LineCubicBezierAlgebraicBreakpointOrderClass::Equal
        }
        LineCubicBezierAlgebraicBreakpointOrderClass::Unknown => {
            LineCubicBezierAlgebraicBreakpointOrderClass::Unknown
        }
    }
}

fn algebraic_cubic_source_spans(
    breakpoints: &[LineCubicBezierAlgebraicBreakpoint],
    sequences: &[LineCubicBezierAlgebraicBreakpointSequence],
) -> Vec<LineCubicBezierAlgebraicSourceSpan> {
    let mut spans = Vec::new();
    for sequence in sequences {
        if sequence.class != LineCubicBezierAlgebraicBreakpointSequenceClass::Ordered {
            continue;
        }
        let mut boundaries = Vec::with_capacity(sequence.breakpoints.len() + 2);
        boundaries.push(LineCubicBezierAlgebraicSourceSpanBoundary::SourceStart);
        boundaries.extend(
            sequence
                .breakpoints
                .iter()
                .copied()
                .map(LineCubicBezierAlgebraicSourceSpanBoundary::Breakpoint),
        );
        boundaries.push(LineCubicBezierAlgebraicSourceSpanBoundary::SourceEnd);

        for pair in boundaries.windows(2) {
            let Some((parameter_lower, _)) =
                algebraic_cubic_boundary_interval(sequence.source, pair[0], breakpoints)
            else {
                continue;
            };
            let Some((_, parameter_upper)) =
                algebraic_cubic_boundary_interval(sequence.source, pair[1], breakpoints)
            else {
                continue;
            };
            spans.push(LineCubicBezierAlgebraicSourceSpan {
                source: sequence.source,
                left: pair[0],
                right: pair[1],
                parameter_lower,
                parameter_upper,
            });
        }
    }
    spans
}

fn algebraic_cubic_boundary_interval(
    source: LineCubicBezierAlgebraicBreakpointSequenceSource,
    boundary: LineCubicBezierAlgebraicSourceSpanBoundary,
    breakpoints: &[LineCubicBezierAlgebraicBreakpoint],
) -> Option<(Real, Real)> {
    match boundary {
        LineCubicBezierAlgebraicSourceSpanBoundary::SourceStart => {
            Some((Real::zero(), Real::zero()))
        }
        LineCubicBezierAlgebraicSourceSpanBoundary::SourceEnd => Some((Real::one(), Real::one())),
        LineCubicBezierAlgebraicSourceSpanBoundary::Breakpoint(index) => match source {
            LineCubicBezierAlgebraicBreakpointSequenceSource::Curve(_) => {
                let interval = &breakpoints.get(index)?.cubic_parameter.interval;
                Some((interval.lower.clone(), interval.upper.clone()))
            }
            LineCubicBezierAlgebraicBreakpointSequenceSource::Line(_) => {
                let representation =
                    transformed_image_representation(&breakpoints.get(index)?.line_parameter)?;
                Some((
                    representation.interval.lower.clone(),
                    representation.interval.upper.clone(),
                ))
            }
        },
    }
}

fn compare_algebraic_cubic_parameters(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> LineCubicBezierAlgebraicBreakpointOrderClass {
    compare_algebraic_intervals(
        left.interval.exact_root.as_ref(),
        &left.interval.lower,
        &left.interval.upper,
        right.interval.exact_root.as_ref(),
        &right.interval.lower,
        &right.interval.upper,
        policy,
    )
}

fn compare_algebraic_polynomial_images(
    left: &AlgebraicRootPolynomialImageReport,
    right: &AlgebraicRootPolynomialImageReport,
    policy: PredicatePolicy,
) -> LineCubicBezierAlgebraicBreakpointOrderClass {
    let Some(left_representation) = transformed_image_representation(left) else {
        return LineCubicBezierAlgebraicBreakpointOrderClass::Unknown;
    };
    let Some(right_representation) = transformed_image_representation(right) else {
        return LineCubicBezierAlgebraicBreakpointOrderClass::Unknown;
    };
    compare_algebraic_intervals(
        left_representation.interval.exact_root.as_ref(),
        &left_representation.interval.lower,
        &left_representation.interval.upper,
        right_representation.interval.exact_root.as_ref(),
        &right_representation.interval.lower,
        &right_representation.interval.upper,
        policy,
    )
}

fn transformed_image_representation(
    image: &AlgebraicRootPolynomialImageReport,
) -> Option<&AlgebraicRootRepresentation> {
    (image.status == AlgebraicRootPolynomialImageStatus::Transformed)
        .then_some(image.representation.as_ref())
        .flatten()
}

fn compare_algebraic_intervals(
    left_exact: Option<&Real>,
    left_lower: &Real,
    left_upper: &Real,
    right_exact: Option<&Real>,
    right_lower: &Real,
    right_upper: &Real,
    policy: PredicatePolicy,
) -> LineCubicBezierAlgebraicBreakpointOrderClass {
    if let (Some(left_exact), Some(right_exact)) = (left_exact, right_exact) {
        return match compare_reals_with_policy(left_exact, right_exact, policy).value() {
            Some(Ordering::Less) => LineCubicBezierAlgebraicBreakpointOrderClass::Before,
            Some(Ordering::Equal) => LineCubicBezierAlgebraicBreakpointOrderClass::Equal,
            Some(Ordering::Greater) => LineCubicBezierAlgebraicBreakpointOrderClass::After,
            None => LineCubicBezierAlgebraicBreakpointOrderClass::Unknown,
        };
    }
    match compare_reals_with_policy(left_upper, right_lower, policy).value() {
        Some(Ordering::Less) => return LineCubicBezierAlgebraicBreakpointOrderClass::Before,
        Some(Ordering::Equal | Ordering::Greater) | None => {}
    }
    match compare_reals_with_policy(right_upper, left_lower, policy).value() {
        Some(Ordering::Less) => LineCubicBezierAlgebraicBreakpointOrderClass::After,
        Some(Ordering::Equal | Ordering::Greater) | None => {
            LineCubicBezierAlgebraicBreakpointOrderClass::Unknown
        }
    }
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
