//! Mixed exact arrangement cleanup for retained lines and rational quadratics.
//!
//! This module is the conic companion to the mixed line/quadratic-Bezier
//! scheduler. It promotes certified line/conic event witnesses into exact line
//! breakpoints and exact rational-quadratic breakpoints, then emits retained
//! homogeneous conic fragments plus a tangent-sorted topology graph. It does
//! not perform boolean materialization; those are downstream responsibilities.

use std::cmp::Ordering;
use std::collections::BTreeMap;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal};
use hyperreal::{Real, RealExactSetFacts};
use hypersolve::AlgebraicRootRepresentation;

use crate::bezier::RationalQuadraticBezier;
use crate::bezier_arrangement::{
    HomogeneousPoint2, LineRationalQuadraticBezierIntersection,
    LineRationalQuadraticBezierIntersectionClass, LineRationalQuadraticBezierIntersectionReport,
    LineRationalQuadraticBezierInverseBoundarySource, LineRationalQuadraticBezierInverseRootDomain,
    LineRationalQuadraticBezierSupportOverlap, intersect_line_rational_quadratic_bezier,
};
use crate::curve_cell::{
    CurveArrangementCellError, CurveArrangementCellGraph, build_line_rational_quadratic_cell_graph,
};
use crate::provenance::PathProvenance;
use crate::segment::{Axis, LinePathSegment};

/// Errors that prevent a trusted line/rational-quadratic split schedule.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierArrangementError {
    /// A retained line segment is degenerate and cannot carry an ordered split set.
    DegenerateLine { line: usize },
    /// Exact comparison of line split parameters was undecidable.
    UndecidableLineOrder { line: usize },
    /// Exact comparison of conic split parameters was undecidable.
    UndecidableConicOrder { curve: usize },
    /// The same geometric point could not be de-duplicated exactly.
    UndecidablePointEquality,
    /// Exact tangent ordering around a retained cell vertex was undecidable.
    UndecidableCellOrder { vertex: usize },
    /// Exact curved face-area replay was unavailable for a retained edge.
    UndecidableCellArea { edge: usize },
}

/// Exact event between one retained line segment and one rational quadratic.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierArrangementEvent {
    /// Line segment index.
    pub line: usize,
    /// Rational quadratic index.
    pub curve: usize,
    /// Certified intersection class.
    pub class: LineRationalQuadraticBezierIntersectionClass,
    /// Raw exact line/conic predicate report.
    pub intersection: LineRationalQuadraticBezierIntersectionReport,
}

/// Retained same-support line/conic overlap candidate.
///
/// These candidates are copied from the predicate report whenever a rational
/// quadratic conic is certified to lie on an axis-aligned line support. They
/// are retained even when monotonicity is non-certified and the event remains
/// [`LineRationalQuadraticBezierIntersectionClass::Unknown`]. This keeps the
/// exact homogeneous support and hodograph-numerator evidence available to
/// later algebraic ordering work, following Yap, "Towards Exact Geometric
/// Computation" (1997), instead of collapsing nonmonotone conic overlaps into
/// a lossy sampled approximation.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierSupportOverlapCandidate {
    /// Line segment index.
    pub line: usize,
    /// Rational quadratic conic index.
    pub curve: usize,
    /// Retained same-support overlap evidence.
    pub overlap: LineRationalQuadraticBezierSupportOverlap,
}

/// Certified domain status for a retained algebraic line/conic breakpoint candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierAlgebraicBreakpointDomain {
    /// Conic parameter and line boundary source are certified inside the retained pair domains.
    InsideLineAndCurve,
    /// The retained conic parameter is certified outside `[0, 1]`.
    OutsideConic,
    /// Exact interval comparison did not decide.
    Unknown,
}

/// Retained algebraic breakpoint candidate for a nonmonotone line/conic overlap boundary.
///
/// This is the mixed-scheduler counterpart to
/// [`crate::bezier_arrangement::LineRationalQuadraticBezierAlgebraicInverseRoot`].
/// The predicate layer retains represented roots of
/// `N_v(t) - value * W(t) == 0`; this scheduler attaches each finite root to
/// the exact line endpoint that induced the boundary value.
///
/// These candidates are exact replay evidence, not inserted topology. They remain
/// separate from [`RationalQuadraticBezierRealBreakpoint`] until a later
/// ordering/materialization pass can compare represented conic parameters and
/// split homogeneous rational quadratics without sampling. The homogeneous
/// rational equation is homogeneous, while `hypersolve` isolates represented
/// roots with a Sturm sequence.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierAlgebraicBreakpoint {
    /// Line segment index.
    pub line: usize,
    /// Rational quadratic conic index.
    pub curve: usize,
    /// Line endpoint that supplied the retained boundary value.
    pub boundary_source: LineRationalQuadraticBezierInverseBoundarySource,
    /// Exact varying-coordinate boundary value on the line support.
    pub boundary_value: Real,
    /// Exact point on the retained line support.
    pub point: Point2,
    /// Exact line parameter for the retained endpoint boundary (`0` or `1`).
    pub line_parameter: Real,
    /// Represented algebraic conic parameter.
    pub conic_parameter: AlgebraicRootRepresentation,
    /// Certified relation of the represented conic parameter to `[0, 1]`.
    pub conic_parameter_domain: LineRationalQuadraticBezierInverseRootDomain,
    /// Certified relation of the candidate to both source domains.
    pub domain: LineRationalQuadraticBezierAlgebraicBreakpointDomain,
}

/// Certified order relation between two represented conic breakpoint candidates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierAlgebraicBreakpointOrderClass {
    /// The left breakpoint parameter is certified before the right parameter.
    Before,
    /// The represented parameters are certified equal.
    Equal,
    /// The left breakpoint parameter is certified after the right parameter.
    After,
    /// The isolating intervals overlap or exact comparison did not decide.
    Unknown,
}

/// Pairwise ordering evidence for retained algebraic conic breakpoint candidates.
///
/// The order is certified only from exact root witnesses or from separated
/// Sturm isolating intervals. It is deliberately not used to mutate
/// [`LineRationalQuadraticBezierArrangementReport::conic_breakpoints`], because
/// represented conic parameters still need exact homogeneous subcurve
/// materialization before they can become topology. This is the same
/// object/predicate boundary advocated by Yap, "Towards Exact Geometric
/// Computation" (1997): report exact ordering evidence when available, and
/// keep uncertainty explicit. The isolating-interval comparison follows the
/// Sturm/Collins-Loos univariate root model used by `hypersolve`.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierAlgebraicBreakpointOrder {
    /// Rational quadratic conic index shared by both candidates.
    pub curve: usize,
    /// Index in [`LineRationalQuadraticBezierArrangementReport::algebraic_breakpoints`].
    pub left: usize,
    /// Index in [`LineRationalQuadraticBezierArrangementReport::algebraic_breakpoints`].
    pub right: usize,
    /// Certified order relation between the represented conic parameters.
    pub order: LineRationalQuadraticBezierAlgebraicBreakpointOrderClass,
}

/// Source parameter space for a retained line/conic algebraic breakpoint sequence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource {
    /// Breakpoints ordered by exact retained line parameter.
    Line(usize),
    /// Breakpoints ordered by represented rational-quadratic source parameter.
    Curve(usize),
}

/// Sequence readiness for retained algebraic line/conic breakpoints.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierAlgebraicBreakpointSequenceClass {
    /// Every same-source pair had certified strict order, so `breakpoints` is sorted.
    Ordered,
    /// A pair was equal, missing, or undecidable; insertion order is retained.
    Ambiguous,
}

/// Exact blocker that prevents a retained conic algebraic sequence from being sorted.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierAlgebraicBreakpointSequenceBlocker {
    /// Same-curve represented-parameter order was not emitted for this pair.
    MissingOrder { left: usize, right: usize },
    /// Same-curve represented-parameter intervals overlap or comparison did not decide.
    UnknownOrder { left: usize, right: usize },
    /// Distinct retained candidates have the same source parameter on this sequence.
    EqualOrder { left: usize, right: usize },
}

/// Ordered retained algebraic line/conic breakpoint indices for one source.
///
/// The indices address
/// [`LineRationalQuadraticBezierArrangementReport::algebraic_breakpoints`].
/// Curve sequences consume represented conic-parameter order evidence; line
/// sequences use the exact retained endpoint parameter (`0` or `1`) attached
/// to each nonmonotone overlap boundary. This deliberately remains evidence:
/// it does not mutate [`LineRationalQuadraticBezierArrangementReport::conic_breakpoints`]
/// or emit homogeneous fragments at algebraic parameters.
///
/// Exact retained objects carry replayable certificates, while equality and
/// undecidable order remain explicit blockers. The represented conic roots use
/// a Sturm sequence, and the rational conic boundary equation stays homogeneous.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LineRationalQuadraticBezierAlgebraicBreakpointSequence {
    /// Source whose parameter orders this sequence.
    pub source: LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource,
    /// Breakpoint indices, sorted only when `class == Ordered`.
    pub breakpoints: Vec<usize>,
    /// Whether this source sequence is ready for exact algebraic split construction.
    pub class: LineRationalQuadraticBezierAlgebraicBreakpointSequenceClass,
    /// Exact reasons that prevented sorting.
    pub blockers: Vec<LineRationalQuadraticBezierAlgebraicBreakpointSequenceBlocker>,
}

/// Boundary of a retained line/conic algebraic source span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineRationalQuadraticBezierAlgebraicSourceSpanBoundary {
    /// The exact source parameter `0`.
    SourceStart,
    /// An index in [`LineRationalQuadraticBezierArrangementReport::algebraic_breakpoints`].
    Breakpoint(usize),
    /// The exact source parameter `1`.
    SourceEnd,
}

/// Conservative source-parameter interval between ordered conic algebraic breakpoints.
///
/// Spans are emitted only from ordered retained algebraic sequences. A span is
/// not a homogeneous conic fragment and does not insert represented roots into
/// [`LineRationalQuadraticBezierArrangementReport::conic_breakpoints`]. It
/// stores a conservative interval hull between adjacent certified boundaries:
/// exact `0`/`1` endpoints or the retained Sturm isolating interval of a
/// represented conic root.
///
/// This keeps Yap's object/predicate separation from "Towards Exact Geometric
/// Computation" (1997): the scheduler reports exact replay evidence for later
/// construction without sampling a nonlinear algebraic boundary. The root
/// intervals follow Collins and Loos, "Real Zeros of Polynomials" (1982), and
/// the rational quadratic remains in the homogeneous model described by
/// Farouki, *Pythagorean Hodograph Curves* (2008).
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierAlgebraicSourceSpan {
    /// Source whose parameter space owns this span.
    pub source: LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource,
    /// Left adjacent boundary.
    pub left: LineRationalQuadraticBezierAlgebraicSourceSpanBoundary,
    /// Right adjacent boundary.
    pub right: LineRationalQuadraticBezierAlgebraicSourceSpanBoundary,
    /// Conservative lower source parameter bound.
    pub parameter_lower: Real,
    /// Conservative upper source parameter bound.
    pub parameter_upper: Real,
}

/// Conservative coordinate envelope for a line/conic algebraic source span.
///
/// The envelope is indexed by
/// [`LineRationalQuadraticBezierArrangementReport::algebraic_source_spans`].
/// It encloses retained span endpoints: exact source endpoints and exact
/// same-support inverse-boundary points. For curve-owned spans it also
/// includes any exact rational quadratic coordinate extrema whose quotient
/// derivative roots are certified inside the retained source-parameter
/// interval. It does not evaluate or split the rational quadratic at a
/// represented root and it does not sample the conic interior.
///
/// This is Yap's retained-object boundary from "Towards Exact Geometric
/// Computation" (1997): represented roots remain exact evidence until a later
/// homogeneous materializer can consume them. The source intervals are
/// Sturm/Collins-Loos isolators, following Collins and Loos, "Real Zeros of
/// Polynomials" (1982), and the conic itself remains in the homogeneous
/// rational model described by Farouki, *Pythagorean Hodograph Curves* (2008).
/// Extrema replay uses the exact quotient derivative numerator `N'W - NW'`,
/// the standard rational Bezier derivative construction in that model.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierAlgebraicEndpointEnvelope {
    /// Index in [`LineRationalQuadraticBezierArrangementReport::algebraic_source_spans`].
    pub span: usize,
    /// Conservative lower x-coordinate bound for the span endpoints.
    pub x_lower: Real,
    /// Conservative upper x-coordinate bound for the span endpoints.
    pub x_upper: Real,
    /// Conservative lower y-coordinate bound for the span endpoints.
    pub y_lower: Real,
    /// Conservative upper y-coordinate bound for the span endpoints.
    pub y_upper: Real,
}

/// Exact native conic breakpoint promoted from a retained algebraic root.
///
/// A retained nonmonotone line/conic overlap boundary may still carry an
/// exact rational root witness in its Sturm isolator. In that case the mixed
/// scheduler can replay the root as an ordinary homogeneous conic split
/// parameter while keeping non-rational represented roots retained-only.
///
/// This is a narrow materialization step under Yap, "Towards Exact Geometric
/// Computation" (1997): the conversion is allowed only when the represented
/// root already includes an exact rational witness and is certified inside the
/// conic domain. The root witness itself is produced by the Collins-Loos real
/// root isolation discipline used by `hypersolve`; the emitted fragment is
/// still the homogeneous rational quadratic object described by Farouki,
/// *Pythagorean Hodograph Curves* (2008).
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierExactAlgebraicBreakpointPromotion {
    /// Index in [`LineRationalQuadraticBezierArrangementReport::algebraic_breakpoints`].
    pub algebraic_breakpoint: usize,
    /// Rational quadratic conic index.
    pub curve: usize,
    /// Exact promoted source parameter.
    pub parameter: Real,
    /// Exact point attached to the retained overlap boundary.
    pub point: Point2,
}

/// Exact breakpoint on one arranged rational quadratic conic.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBezierRealBreakpoint {
    /// Rational quadratic index.
    pub curve: usize,
    /// Exact source parameter in `[0, 1]`.
    pub parameter: Real,
    /// Exact affine point image at `parameter`.
    pub point: Point2,
}

/// Exact line breakpoint used by the mixed line/conic scheduler.
#[derive(Clone, Debug, PartialEq)]
pub struct MixedConicLineArrangementBreakpoint {
    /// Line segment index.
    pub line: usize,
    /// Exact point on the retained line segment.
    pub point: Point2,
    /// Numerator of the retained parameter `dot(point-start, end-start) / |end-start|^2`.
    pub parameter_numerator: Real,
    /// Positive denominator of the retained line parameter.
    pub parameter_denominator: Real,
}

/// Exact retained line fragment induced by mixed line/conic events.
#[derive(Clone, Debug, PartialEq)]
pub struct MixedConicLineArrangementFragment {
    /// Source line segment index.
    pub source_line: usize,
    /// Fragment start witness.
    pub start: MixedConicLineArrangementBreakpoint,
    /// Fragment end witness.
    pub end: MixedConicLineArrangementBreakpoint,
    /// Retained exact line fragment.
    pub segment: LinePathSegment,
}

/// Exact homogeneous rational-quadratic fragment induced by mixed line/conic events.
///
/// The fragment stores homogeneous Bernstein controls `(X, Y, W)` directly.
/// That is the exact object produced by rational de Casteljau restriction.
/// Topology is certified before any affine denominator division is trusted.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBezierRealFragment {
    /// Source rational quadratic index.
    pub source_curve: usize,
    /// Fragment start witness.
    pub start: RationalQuadraticBezierRealBreakpoint,
    /// Fragment end witness.
    pub end: RationalQuadraticBezierRealBreakpoint,
    /// Homogeneous start control.
    pub start_control: HomogeneousPoint2,
    /// Homogeneous middle control.
    pub control: HomogeneousPoint2,
    /// Homogeneous end control.
    pub end_control: HomogeneousPoint2,
}

/// Cached exact facts for a mixed line/conic arrangement schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierArrangementFacts {
    /// Exact-set facts across retained line endpoints and conic controls/weights.
    pub input_exact: RealExactSetFacts,
    /// Exact-set facts across emitted line and homogeneous conic fragment controls.
    pub fragment_exact: RealExactSetFacts,
    /// Source provenance for the arrangement schedule.
    pub provenance: PathProvenance,
}

/// Retained mixed line/rational-quadratic arrangement schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct LineRationalQuadraticBezierArrangementReport {
    /// Retained input line segments.
    pub lines: Vec<LinePathSegment>,
    /// Retained input rational quadratic conics.
    pub curves: Vec<RationalQuadraticBezier>,
    /// Certified or unknown pairwise events.
    pub events: Vec<LineRationalQuadraticBezierArrangementEvent>,
    /// Retained same-support conic overlap candidates.
    pub support_overlaps: Vec<LineRationalQuadraticBezierSupportOverlapCandidate>,
    /// Algebraic conic breakpoint candidates retained from nonmonotone overlap boundaries.
    pub algebraic_breakpoints: Vec<LineRationalQuadraticBezierAlgebraicBreakpoint>,
    /// Pairwise exact order evidence for retained algebraic conic breakpoints.
    pub algebraic_breakpoint_orders: Vec<LineRationalQuadraticBezierAlgebraicBreakpointOrder>,
    /// Per-source retained algebraic breakpoint sequences derived from exact order evidence.
    pub algebraic_breakpoint_sequences: Vec<LineRationalQuadraticBezierAlgebraicBreakpointSequence>,
    /// Conservative source spans induced by certified algebraic breakpoint sequences.
    pub algebraic_source_spans: Vec<LineRationalQuadraticBezierAlgebraicSourceSpan>,
    /// Conservative endpoint coordinate envelopes for retained algebraic source spans.
    pub algebraic_endpoint_envelopes: Vec<LineRationalQuadraticBezierAlgebraicEndpointEnvelope>,
    /// Exact algebraic roots promoted into native conic breakpoints.
    pub exact_algebraic_breakpoint_promotions:
        Vec<LineRationalQuadraticBezierExactAlgebraicBreakpointPromotion>,
    /// Sorted line breakpoints induced by line endpoints and certified events.
    pub line_breakpoints: Vec<Vec<MixedConicLineArrangementBreakpoint>>,
    /// Sorted conic breakpoints induced by endpoints and certified events.
    pub conic_breakpoints: Vec<Vec<RationalQuadraticBezierRealBreakpoint>>,
    /// Positive-length line fragments.
    pub line_fragments: Vec<MixedConicLineArrangementFragment>,
    /// Positive-length homogeneous conic fragments.
    pub conic_fragments: Vec<RationalQuadraticBezierRealFragment>,
    /// Retained topology graph over line and homogeneous conic fragments.
    ///
    /// Vertices are exact endpoints, edges retain source fragment provenance,
    /// and half-edges are sorted by exact line tangents or homogeneous conic
    /// endpoint derivatives. This follows Yap, "Towards Exact Geometric
    /// Computation" (1997), by reporting certified topology without sampling.
    /// Rational conic face walks are emitted only when the homogeneous weight
    /// denominator and Green-integral branch are certified exactly; unsupported
    /// quotient branches remain explicit unavailable evidence.
    pub cell_graph: CurveArrangementCellGraph,
    /// Cached exact facts for the retained schedule.
    pub facts: LineRationalQuadraticBezierArrangementFacts,
}

/// Arrange retained line segments against retained rational quadratic conics.
pub fn arrange_line_segments_with_rational_quadratic_beziers(
    lines: &[LinePathSegment],
    curves: &[RationalQuadraticBezier],
    policy: PredicatePolicy,
) -> Result<LineRationalQuadraticBezierArrangementReport, LineRationalQuadraticBezierArrangementError>
{
    arrange_line_segments_with_rational_quadratic_beziers_and_provenance(
        lines,
        curves,
        policy,
        PathProvenance::native(),
    )
}

/// Arrange retained line segments against retained rational quadratics with provenance.
pub fn arrange_line_segments_with_rational_quadratic_beziers_and_provenance(
    lines: &[LinePathSegment],
    curves: &[RationalQuadraticBezier],
    policy: PredicatePolicy,
    provenance: PathProvenance,
) -> Result<LineRationalQuadraticBezierArrangementReport, LineRationalQuadraticBezierArrangementError>
{
    reject_degenerate_lines(lines, policy)?;
    let mut line_breakpoints = seed_line_breakpoints(lines);
    let mut conic_breakpoints = seed_conic_breakpoints(curves);
    let mut events = Vec::new();
    let mut support_overlaps = Vec::new();
    let mut algebraic_breakpoints = Vec::new();

    for (line_index, line) in lines.iter().enumerate() {
        for (curve_index, curve) in curves.iter().enumerate() {
            let intersection = intersect_line_rational_quadratic_bezier(line, curve, policy);
            if intersection.class != LineRationalQuadraticBezierIntersectionClass::Unknown {
                for event in &intersection.intersections {
                    insert_line_breakpoint(
                        &mut line_breakpoints[line_index],
                        line_index,
                        line,
                        event.point.clone(),
                        policy,
                    )?;
                    insert_conic_breakpoint(
                        &mut conic_breakpoints[curve_index],
                        curve_index,
                        event,
                        policy,
                    )?;
                }
            }
            if let Some(overlap) = &intersection.support_overlap {
                algebraic_breakpoints.extend(retained_algebraic_conic_breakpoints(
                    line_index,
                    curve_index,
                    overlap,
                ));
                support_overlaps.push(LineRationalQuadraticBezierSupportOverlapCandidate {
                    line: line_index,
                    curve: curve_index,
                    overlap: overlap.clone(),
                });
            }
            events.push(LineRationalQuadraticBezierArrangementEvent {
                line: line_index,
                curve: curve_index,
                class: intersection.class,
                intersection,
            });
        }
    }

    let exact_algebraic_breakpoint_promotions = promote_exact_algebraic_conic_breakpoints(
        &mut conic_breakpoints,
        &algebraic_breakpoints,
        policy,
    )?;
    sort_and_dedup_line_breakpoints(&mut line_breakpoints, policy)?;
    sort_and_dedup_conic_breakpoints(&mut conic_breakpoints, policy)?;
    let algebraic_breakpoint_orders =
        algebraic_conic_breakpoint_orders(&algebraic_breakpoints, policy);
    let algebraic_breakpoint_sequences = algebraic_conic_breakpoint_sequences(
        &algebraic_breakpoints,
        &algebraic_breakpoint_orders,
        policy,
    );
    let algebraic_source_spans =
        algebraic_conic_source_spans(&algebraic_breakpoints, &algebraic_breakpoint_sequences);
    let algebraic_endpoint_envelopes = algebraic_conic_endpoint_envelopes(
        lines,
        curves,
        &algebraic_breakpoints,
        &algebraic_source_spans,
        policy,
    );
    let line_fragments = build_line_fragments(&line_breakpoints, policy)?;
    let conic_fragments = build_conic_fragments(&conic_breakpoints, curves, policy)?;
    let cell_graph =
        build_line_rational_quadratic_cell_graph(&line_fragments, &conic_fragments, policy)
            .map_err(line_rational_quadratic_error_from_curve_cell_error)?;
    let facts = LineRationalQuadraticBezierArrangementFacts {
        input_exact: input_exact_facts(lines, curves),
        fragment_exact: fragment_exact_facts(&line_fragments, &conic_fragments),
        provenance,
    };

    Ok(LineRationalQuadraticBezierArrangementReport {
        lines: lines.to_vec(),
        curves: curves.to_vec(),
        events,
        support_overlaps,
        algebraic_breakpoints,
        algebraic_breakpoint_orders,
        algebraic_breakpoint_sequences,
        algebraic_source_spans,
        algebraic_endpoint_envelopes,
        exact_algebraic_breakpoint_promotions,
        line_breakpoints,
        conic_breakpoints,
        line_fragments,
        conic_fragments,
        cell_graph,
        facts,
    })
}

fn line_rational_quadratic_error_from_curve_cell_error(
    error: CurveArrangementCellError,
) -> LineRationalQuadraticBezierArrangementError {
    match error {
        CurveArrangementCellError::UndecidablePointEquality => {
            LineRationalQuadraticBezierArrangementError::UndecidablePointEquality
        }
        CurveArrangementCellError::UndecidableCellOrder { vertex } => {
            LineRationalQuadraticBezierArrangementError::UndecidableCellOrder { vertex }
        }
        CurveArrangementCellError::UndecidableCellArea { edge } => {
            LineRationalQuadraticBezierArrangementError::UndecidableCellArea { edge }
        }
    }
}

fn reject_degenerate_lines(
    lines: &[LinePathSegment],
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for (index, line) in lines.iter().enumerate() {
        if line.facts().known_degenerate == Some(true)
            || compare_reals_with_policy(&line.length_squared(), &Real::zero(), policy).value()
                == Some(Ordering::Equal)
        {
            return Err(
                LineRationalQuadraticBezierArrangementError::DegenerateLine { line: index },
            );
        }
    }
    Ok(())
}

fn seed_line_breakpoints(
    lines: &[LinePathSegment],
) -> Vec<Vec<MixedConicLineArrangementBreakpoint>> {
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

fn seed_conic_breakpoints(
    curves: &[RationalQuadraticBezier],
) -> Vec<Vec<RationalQuadraticBezierRealBreakpoint>> {
    curves
        .iter()
        .enumerate()
        .map(|(curve_index, curve)| {
            vec![
                RationalQuadraticBezierRealBreakpoint {
                    curve: curve_index,
                    parameter: Real::zero(),
                    point: curve.start().clone(),
                },
                RationalQuadraticBezierRealBreakpoint {
                    curve: curve_index,
                    parameter: Real::one(),
                    point: curve.end().clone(),
                },
            ]
        })
        .collect()
}

fn retained_algebraic_conic_breakpoints(
    line_index: usize,
    curve_index: usize,
    overlap: &LineRationalQuadraticBezierSupportOverlap,
) -> Vec<LineRationalQuadraticBezierAlgebraicBreakpoint> {
    let mut retained = Vec::new();
    for boundary in &overlap.inverse_boundary_roots {
        let point = point_from_axis(overlap.axis, overlap.fixed.clone(), boundary.value.clone());
        let line_parameter = match boundary.source {
            LineRationalQuadraticBezierInverseBoundarySource::SegmentStart => Real::zero(),
            LineRationalQuadraticBezierInverseBoundarySource::SegmentEnd => Real::one(),
        };
        for root in &boundary.roots {
            retained.push(LineRationalQuadraticBezierAlgebraicBreakpoint {
                line: line_index,
                curve: curve_index,
                boundary_source: boundary.source,
                boundary_value: boundary.value.clone(),
                point: point.clone(),
                line_parameter: line_parameter.clone(),
                conic_parameter: root.parameter.clone(),
                conic_parameter_domain: root.parameter_domain,
                domain: classify_algebraic_conic_breakpoint_domain(root.parameter_domain),
            });
        }
    }
    retained
}

fn classify_algebraic_conic_breakpoint_domain(
    conic_domain: LineRationalQuadraticBezierInverseRootDomain,
) -> LineRationalQuadraticBezierAlgebraicBreakpointDomain {
    match conic_domain {
        LineRationalQuadraticBezierInverseRootDomain::InsideUnitInterval => {
            LineRationalQuadraticBezierAlgebraicBreakpointDomain::InsideLineAndCurve
        }
        LineRationalQuadraticBezierInverseRootDomain::OutsideUnitInterval => {
            LineRationalQuadraticBezierAlgebraicBreakpointDomain::OutsideConic
        }
        LineRationalQuadraticBezierInverseRootDomain::Unknown => {
            LineRationalQuadraticBezierAlgebraicBreakpointDomain::Unknown
        }
    }
}

fn point_from_axis(axis: Axis, fixed: Real, varying: Real) -> Point2 {
    match axis {
        Axis::X => Point2::new(varying, fixed),
        Axis::Y => Point2::new(fixed, varying),
    }
}

fn promote_exact_algebraic_conic_breakpoints(
    conic_breakpoints: &mut [Vec<RationalQuadraticBezierRealBreakpoint>],
    algebraic_breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
    policy: PredicatePolicy,
) -> Result<
    Vec<LineRationalQuadraticBezierExactAlgebraicBreakpointPromotion>,
    LineRationalQuadraticBezierArrangementError,
> {
    let mut promotions = Vec::new();
    for (index, breakpoint) in algebraic_breakpoints.iter().enumerate() {
        if breakpoint.domain
            != LineRationalQuadraticBezierAlgebraicBreakpointDomain::InsideLineAndCurve
        {
            continue;
        }
        let Some(parameter) = breakpoint.conic_parameter.interval.exact_root.clone() else {
            continue;
        };
        insert_exact_conic_breakpoint(
            &mut conic_breakpoints[breakpoint.curve],
            breakpoint.curve,
            parameter.clone(),
            breakpoint.point.clone(),
            policy,
        )?;
        promotions.push(
            LineRationalQuadraticBezierExactAlgebraicBreakpointPromotion {
                algebraic_breakpoint: index,
                curve: breakpoint.curve,
                parameter,
                point: breakpoint.point.clone(),
            },
        );
    }
    Ok(promotions)
}

fn algebraic_conic_breakpoint_orders(
    breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
    policy: PredicatePolicy,
) -> Vec<LineRationalQuadraticBezierAlgebraicBreakpointOrder> {
    let mut orders = Vec::new();
    for left in 0..breakpoints.len() {
        for right in (left + 1)..breakpoints.len() {
            if breakpoints[left].curve != breakpoints[right].curve {
                continue;
            }
            orders.push(LineRationalQuadraticBezierAlgebraicBreakpointOrder {
                curve: breakpoints[left].curve,
                left,
                right,
                order: compare_algebraic_conic_parameters(
                    &breakpoints[left].conic_parameter,
                    &breakpoints[right].conic_parameter,
                    policy,
                ),
            });
        }
    }
    orders
}

fn algebraic_conic_breakpoint_sequences(
    breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
    orders: &[LineRationalQuadraticBezierAlgebraicBreakpointOrder],
    policy: PredicatePolicy,
) -> Vec<LineRationalQuadraticBezierAlgebraicBreakpointSequence> {
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
        sequences.push(algebraic_conic_breakpoint_sequence_for_source(
            LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Curve(curve),
            indices,
            breakpoints,
            orders,
            policy,
        ));
    }
    for (line, indices) in line_breakpoints {
        sequences.push(algebraic_conic_breakpoint_sequence_for_source(
            LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Line(line),
            indices,
            breakpoints,
            orders,
            policy,
        ));
    }
    sequences
}

fn algebraic_conic_breakpoint_sequence_for_source(
    source: LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource,
    mut indices: Vec<usize>,
    breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
    orders: &[LineRationalQuadraticBezierAlgebraicBreakpointOrder],
    policy: PredicatePolicy,
) -> LineRationalQuadraticBezierAlgebraicBreakpointSequence {
    let mut blockers = Vec::new();
    for left_index in 0..indices.len() {
        for right_index in (left_index + 1)..indices.len() {
            let left = indices[left_index];
            let right = indices[right_index];
            match algebraic_conic_order_between(source, left, right, breakpoints, orders, policy) {
                Some(LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Before)
                | Some(LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::After) => {}
                Some(LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Equal) => {
                    blockers.push(
                        LineRationalQuadraticBezierAlgebraicBreakpointSequenceBlocker::EqualOrder {
                            left,
                            right,
                        },
                    );
                }
                Some(LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Unknown) => {
                    blockers.push(
                        LineRationalQuadraticBezierAlgebraicBreakpointSequenceBlocker::UnknownOrder {
                            left,
                            right,
                        },
                    );
                }
                None => {
                    blockers.push(
                        LineRationalQuadraticBezierAlgebraicBreakpointSequenceBlocker::MissingOrder {
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
            algebraic_conic_ordering_for_sort(source, *left, *right, breakpoints, orders, policy)
                .expect("algebraic conic source order was certified before sorting")
        });
        LineRationalQuadraticBezierAlgebraicBreakpointSequenceClass::Ordered
    } else {
        LineRationalQuadraticBezierAlgebraicBreakpointSequenceClass::Ambiguous
    };

    LineRationalQuadraticBezierAlgebraicBreakpointSequence {
        source,
        breakpoints: indices,
        class,
        blockers,
    }
}

fn algebraic_conic_ordering_for_sort(
    source: LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource,
    left: usize,
    right: usize,
    breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
    orders: &[LineRationalQuadraticBezierAlgebraicBreakpointOrder],
    policy: PredicatePolicy,
) -> Option<Ordering> {
    if left == right {
        return Some(Ordering::Equal);
    }
    match algebraic_conic_order_between(source, left, right, breakpoints, orders, policy)? {
        LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Before => Some(Ordering::Less),
        LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::After => Some(Ordering::Greater),
        LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Equal => Some(Ordering::Equal),
        LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Unknown => None,
    }
}

fn algebraic_conic_order_between(
    source: LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource,
    left: usize,
    right: usize,
    breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
    orders: &[LineRationalQuadraticBezierAlgebraicBreakpointOrder],
    policy: PredicatePolicy,
) -> Option<LineRationalQuadraticBezierAlgebraicBreakpointOrderClass> {
    match source {
        LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Line(_) => {
            compare_exact_conic_line_parameters(
                &breakpoints[left].line_parameter,
                &breakpoints[right].line_parameter,
                policy,
            )
        }
        LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Curve(curve) => {
            algebraic_conic_curve_order_between(curve, left, right, orders)
        }
    }
}

fn compare_exact_conic_line_parameters(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> Option<LineRationalQuadraticBezierAlgebraicBreakpointOrderClass> {
    Some(
        match compare_reals_with_policy(left, right, policy).value()? {
            Ordering::Less => LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Before,
            Ordering::Equal => LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Equal,
            Ordering::Greater => LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::After,
        },
    )
}

fn algebraic_conic_curve_order_between(
    curve: usize,
    left: usize,
    right: usize,
    orders: &[LineRationalQuadraticBezierAlgebraicBreakpointOrder],
) -> Option<LineRationalQuadraticBezierAlgebraicBreakpointOrderClass> {
    let direct = orders
        .iter()
        .find(|order| order.curve == curve && order.left == left && order.right == right)
        .map(|order| order.order);
    if direct.is_some() {
        return direct;
    }
    orders
        .iter()
        .find(|order| order.curve == curve && order.left == right && order.right == left)
        .map(|order| reverse_algebraic_conic_order(order.order))
}

fn reverse_algebraic_conic_order(
    order: LineRationalQuadraticBezierAlgebraicBreakpointOrderClass,
) -> LineRationalQuadraticBezierAlgebraicBreakpointOrderClass {
    match order {
        LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Before => {
            LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::After
        }
        LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::After => {
            LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Before
        }
        LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Equal => {
            LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Equal
        }
        LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Unknown => {
            LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Unknown
        }
    }
}

fn algebraic_conic_source_spans(
    breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
    sequences: &[LineRationalQuadraticBezierAlgebraicBreakpointSequence],
) -> Vec<LineRationalQuadraticBezierAlgebraicSourceSpan> {
    let mut spans = Vec::new();
    for sequence in sequences {
        if sequence.class != LineRationalQuadraticBezierAlgebraicBreakpointSequenceClass::Ordered {
            continue;
        }
        let mut boundaries = Vec::with_capacity(sequence.breakpoints.len() + 2);
        boundaries.push(LineRationalQuadraticBezierAlgebraicSourceSpanBoundary::SourceStart);
        boundaries.extend(
            sequence
                .breakpoints
                .iter()
                .copied()
                .map(LineRationalQuadraticBezierAlgebraicSourceSpanBoundary::Breakpoint),
        );
        boundaries.push(LineRationalQuadraticBezierAlgebraicSourceSpanBoundary::SourceEnd);

        for pair in boundaries.windows(2) {
            let Some((parameter_lower, _)) =
                algebraic_conic_boundary_interval(sequence.source, pair[0], breakpoints)
            else {
                continue;
            };
            let Some((_, parameter_upper)) =
                algebraic_conic_boundary_interval(sequence.source, pair[1], breakpoints)
            else {
                continue;
            };
            spans.push(LineRationalQuadraticBezierAlgebraicSourceSpan {
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

fn algebraic_conic_boundary_interval(
    source: LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource,
    boundary: LineRationalQuadraticBezierAlgebraicSourceSpanBoundary,
    breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
) -> Option<(Real, Real)> {
    match boundary {
        LineRationalQuadraticBezierAlgebraicSourceSpanBoundary::SourceStart => {
            Some((Real::zero(), Real::zero()))
        }
        LineRationalQuadraticBezierAlgebraicSourceSpanBoundary::SourceEnd => {
            Some((Real::one(), Real::one()))
        }
        LineRationalQuadraticBezierAlgebraicSourceSpanBoundary::Breakpoint(index) => match source {
            LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Curve(_) => {
                let interval = &breakpoints.get(index)?.conic_parameter.interval;
                Some((interval.lower.clone(), interval.upper.clone()))
            }
            LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Line(_) => {
                let parameter = breakpoints.get(index)?.line_parameter.clone();
                Some((parameter.clone(), parameter))
            }
        },
    }
}

fn algebraic_conic_endpoint_envelopes(
    lines: &[LinePathSegment],
    curves: &[RationalQuadraticBezier],
    breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
    spans: &[LineRationalQuadraticBezierAlgebraicSourceSpan],
    policy: PredicatePolicy,
) -> Vec<LineRationalQuadraticBezierAlgebraicEndpointEnvelope> {
    spans
        .iter()
        .enumerate()
        .filter_map(|(span_index, span)| {
            let left = algebraic_conic_boundary_point_interval(
                span.source,
                span.left,
                lines,
                curves,
                breakpoints,
            )?;
            let right = algebraic_conic_boundary_point_interval(
                span.source,
                span.right,
                lines,
                curves,
                breakpoints,
            )?;
            let mut points = vec![left, right];
            if let LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Curve(
                curve_index,
            ) = span.source
            {
                points.extend(algebraic_conic_span_interior_extrema(
                    curves.get(curve_index)?,
                    span,
                    policy,
                )?);
            }
            let (x_lower, x_upper, y_lower, y_upper) =
                certified_conic_point_interval_bounds(&points, policy)?;
            Some(LineRationalQuadraticBezierAlgebraicEndpointEnvelope {
                span: span_index,
                x_lower,
                x_upper,
                y_lower,
                y_upper,
            })
        })
        .collect()
}

fn algebraic_conic_span_interior_extrema(
    curve: &RationalQuadraticBezier,
    span: &LineRationalQuadraticBezierAlgebraicSourceSpan,
    policy: PredicatePolicy,
) -> Option<Vec<ConicPointInterval>> {
    // A retained algebraic span is an interval certificate, not a materialized
    // sub-conic. We therefore admit quotient-derivative extrema only when
    // exact comparison proves the derivative root is inside the retained
    // source interval. Undecidable membership withholds the envelope instead
    // of approximating, matching Yap's EGC construction boundary.
    let mut extrema = Vec::new();
    for root in rational_quadratic_derivative_roots(curve, ConicCoordinate::X, policy)? {
        if real_in_closed_interval(&root, &span.parameter_lower, &span.parameter_upper, policy)? {
            extrema.push(affine_conic_point_exact_interval(curve, &root, policy)?);
        }
    }
    for root in rational_quadratic_derivative_roots(curve, ConicCoordinate::Y, policy)? {
        if real_in_closed_interval(&root, &span.parameter_lower, &span.parameter_upper, policy)? {
            extrema.push(affine_conic_point_exact_interval(curve, &root, policy)?);
        }
    }
    Some(extrema)
}

fn algebraic_conic_boundary_point_interval(
    source: LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource,
    boundary: LineRationalQuadraticBezierAlgebraicSourceSpanBoundary,
    lines: &[LinePathSegment],
    curves: &[RationalQuadraticBezier],
    breakpoints: &[LineRationalQuadraticBezierAlgebraicBreakpoint],
) -> Option<ConicPointInterval> {
    match boundary {
        LineRationalQuadraticBezierAlgebraicSourceSpanBoundary::SourceStart => match source {
            LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Line(line) => {
                conic_point_exact_interval(lines.get(line)?.start())
            }
            LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Curve(curve) => {
                conic_point_exact_interval(curves.get(curve)?.start())
            }
        },
        LineRationalQuadraticBezierAlgebraicSourceSpanBoundary::SourceEnd => match source {
            LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Line(line) => {
                conic_point_exact_interval(lines.get(line)?.end())
            }
            LineRationalQuadraticBezierAlgebraicBreakpointSequenceSource::Curve(curve) => {
                conic_point_exact_interval(curves.get(curve)?.end())
            }
        },
        LineRationalQuadraticBezierAlgebraicSourceSpanBoundary::Breakpoint(index) => {
            conic_point_exact_interval(&breakpoints.get(index)?.point)
        }
    }
}

#[derive(Clone, Debug)]
struct ConicPointInterval {
    x_lower: Real,
    x_upper: Real,
    y_lower: Real,
    y_upper: Real,
}

fn conic_point_exact_interval(point: &Point2) -> Option<ConicPointInterval> {
    Some(ConicPointInterval {
        x_lower: point.x.clone(),
        x_upper: point.x.clone(),
        y_lower: point.y.clone(),
        y_upper: point.y.clone(),
    })
}

fn affine_conic_point_exact_interval(
    curve: &RationalQuadraticBezier,
    parameter: &Real,
    policy: PredicatePolicy,
) -> Option<ConicPointInterval> {
    let homogeneous = homogeneous_eval_real(curve, parameter);
    match compare_reals_with_policy(&homogeneous.w, &Real::zero(), policy).value()? {
        Ordering::Equal => None,
        Ordering::Less | Ordering::Greater => {
            let x = (homogeneous.x / homogeneous.w.clone()).ok()?;
            let y = (homogeneous.y / homogeneous.w).ok()?;
            conic_point_exact_interval(&Point2::new(x, y))
        }
    }
}

fn certified_conic_point_interval_bounds(
    points: &[ConicPointInterval],
    policy: PredicatePolicy,
) -> Option<(Real, Real, Real, Real)> {
    let first = points.first()?;
    let mut x_lower = first.x_lower.clone();
    let mut x_upper = first.x_upper.clone();
    let mut y_lower = first.y_lower.clone();
    let mut y_upper = first.y_upper.clone();
    for point in points.iter().skip(1) {
        let x = certified_min_max(&x_lower, &x_upper, &point.x_lower, &point.x_upper, policy)?;
        x_lower = x.0;
        x_upper = x.1;
        let y = certified_min_max(&y_lower, &y_upper, &point.y_lower, &point.y_upper, policy)?;
        y_lower = y.0;
        y_upper = y.1;
    }
    Some((x_lower, x_upper, y_lower, y_upper))
}

fn certified_min_max(
    left_lower: &Real,
    left_upper: &Real,
    right_lower: &Real,
    right_upper: &Real,
    policy: PredicatePolicy,
) -> Option<(Real, Real)> {
    let lower = match compare_reals_with_policy(left_lower, right_lower, policy).value()? {
        Ordering::Less | Ordering::Equal => left_lower.clone(),
        Ordering::Greater => right_lower.clone(),
    };
    let upper = match compare_reals_with_policy(left_upper, right_upper, policy).value()? {
        Ordering::Less | Ordering::Equal => right_upper.clone(),
        Ordering::Greater => left_upper.clone(),
    };
    Some((lower, upper))
}

#[derive(Clone, Copy)]
enum ConicCoordinate {
    X,
    Y,
}

fn rational_quadratic_derivative_roots(
    curve: &RationalQuadraticBezier,
    coordinate: ConicCoordinate,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let numerator = rational_quadratic_coordinate_power_coefficients(curve, coordinate);
    let denominator = rational_quadratic_weight_power_coefficients(curve);
    let a = numerator[1].clone() * denominator[0].clone()
        - numerator[0].clone() * denominator[1].clone();
    let b = Real::from(2)
        * (numerator[2].clone() * denominator[0].clone()
            - numerator[0].clone() * denominator[2].clone());
    let c = numerator[2].clone() * denominator[1].clone()
        - numerator[1].clone() * denominator[2].clone();
    solve_conic_quadratic_or_linear_roots(c, b, a, policy)
}

fn rational_quadratic_coordinate_power_coefficients(
    curve: &RationalQuadraticBezier,
    coordinate: ConicCoordinate,
) -> [Real; 3] {
    let p0 = conic_coordinate(curve.start(), coordinate);
    let p1 = conic_coordinate(curve.control(), coordinate) * curve.control_weight().clone();
    let p2 = conic_coordinate(curve.end(), coordinate);
    [
        p0.clone(),
        Real::from(2) * (p1.clone() - p0.clone()),
        p0 - Real::from(2) * p1 + p2,
    ]
}

fn rational_quadratic_weight_power_coefficients(curve: &RationalQuadraticBezier) -> [Real; 3] {
    let w = curve.control_weight().clone();
    [
        Real::one(),
        Real::from(2) * (w.clone() - Real::one()),
        Real::one() - Real::from(2) * w + Real::one(),
    ]
}

fn conic_coordinate(point: &Point2, coordinate: ConicCoordinate) -> Real {
    match coordinate {
        ConicCoordinate::X => point.x.clone(),
        ConicCoordinate::Y => point.y.clone(),
    }
}

fn solve_conic_quadratic_or_linear_roots(
    a: Real,
    b: Real,
    c: Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    match compare_reals_with_policy(&a, &Real::zero(), policy).value()? {
        Ordering::Equal => solve_conic_linear_roots(b, c, policy),
        Ordering::Less | Ordering::Greater => solve_conic_quadratic_roots(a, b, c, policy),
    }
}

fn solve_conic_linear_roots(b: Real, c: Real, policy: PredicatePolicy) -> Option<Vec<Real>> {
    match compare_reals_with_policy(&b, &Real::zero(), policy).value()? {
        Ordering::Equal => Some(Vec::new()),
        Ordering::Less | Ordering::Greater => Some(vec![(-c / b).ok()?]),
    }
}

fn solve_conic_quadratic_roots(
    a: Real,
    b: Real,
    c: Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
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

fn real_in_closed_interval(
    value: &Real,
    lower: &Real,
    upper: &Real,
    policy: PredicatePolicy,
) -> Option<bool> {
    let lower_cmp = compare_reals_with_policy(value, lower, policy).value()?;
    let upper_cmp = compare_reals_with_policy(value, upper, policy).value()?;
    Some(
        matches!(lower_cmp, Ordering::Equal | Ordering::Greater)
            && matches!(upper_cmp, Ordering::Equal | Ordering::Less),
    )
}

fn compare_algebraic_conic_parameters(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> LineRationalQuadraticBezierAlgebraicBreakpointOrderClass {
    if let (Some(left_exact), Some(right_exact)) =
        (&left.interval.exact_root, &right.interval.exact_root)
    {
        return match compare_reals_with_policy(left_exact, right_exact, policy).value() {
            Some(Ordering::Less) => {
                LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Before
            }
            Some(Ordering::Equal) => {
                LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Equal
            }
            Some(Ordering::Greater) => {
                LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::After
            }
            None => LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Unknown,
        };
    }
    match compare_reals_with_policy(&left.interval.upper, &right.interval.lower, policy).value() {
        Some(Ordering::Less) => {
            return LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Before;
        }
        Some(Ordering::Equal | Ordering::Greater) | None => {}
    }
    match compare_reals_with_policy(&right.interval.upper, &left.interval.lower, policy).value() {
        Some(Ordering::Less) => LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::After,
        Some(Ordering::Equal | Ordering::Greater) | None => {
            LineRationalQuadraticBezierAlgebraicBreakpointOrderClass::Unknown
        }
    }
}

fn insert_line_breakpoint(
    breakpoints: &mut Vec<MixedConicLineArrangementBreakpoint>,
    line_index: usize,
    line: &LinePathSegment,
    point: Point2,
    _policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for existing in breakpoints.iter() {
        match point2_equal(&existing.point, &point).value() {
            Some(true) => return Ok(()),
            Some(false) => {}
            None => {
                return Err(LineRationalQuadraticBezierArrangementError::UndecidablePointEquality);
            }
        }
    }
    breakpoints.push(line_breakpoint(line_index, line, point));
    Ok(())
}

fn insert_conic_breakpoint(
    breakpoints: &mut Vec<RationalQuadraticBezierRealBreakpoint>,
    curve_index: usize,
    event: &LineRationalQuadraticBezierIntersection,
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    insert_exact_conic_breakpoint(
        breakpoints,
        curve_index,
        event.parameter.clone(),
        event.point.clone(),
        policy,
    )
}

fn insert_exact_conic_breakpoint(
    breakpoints: &mut Vec<RationalQuadraticBezierRealBreakpoint>,
    curve_index: usize,
    parameter: Real,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for existing in breakpoints.iter() {
        match compare_reals_with_policy(&existing.parameter, &parameter, policy).value() {
            Some(Ordering::Equal) => return Ok(()),
            Some(Ordering::Less | Ordering::Greater) => {}
            None => {
                return Err(
                    LineRationalQuadraticBezierArrangementError::UndecidableConicOrder {
                        curve: curve_index,
                    },
                );
            }
        }
    }
    breakpoints.push(RationalQuadraticBezierRealBreakpoint {
        curve: curve_index,
        parameter,
        point,
    });
    Ok(())
}

fn line_breakpoint(
    line_index: usize,
    line: &LinePathSegment,
    point: Point2,
) -> MixedConicLineArrangementBreakpoint {
    let dx = line.end().x.clone() - line.start().x.clone();
    let dy = line.end().y.clone() - line.start().y.clone();
    let px = point.x.clone() - line.start().x.clone();
    let py = point.y.clone() - line.start().y.clone();
    let parameter_numerator = px * dx.clone() + py * dy.clone();
    let parameter_denominator = dx.clone() * dx + dy.clone() * dy;
    MixedConicLineArrangementBreakpoint {
        line: line_index,
        point,
        parameter_numerator,
        parameter_denominator,
    }
}

fn sort_and_dedup_line_breakpoints(
    breakpoints: &mut [Vec<MixedConicLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for (line_index, points) in breakpoints.iter_mut().enumerate() {
        certify_line_orders(points, line_index, policy)?;
        points.sort_by(|left, right| {
            compare_line_parameters(left, right, policy)
                .expect("line breakpoint order was certified before sorting")
        });
        let mut deduped: Vec<MixedConicLineArrangementBreakpoint> = Vec::new();
        for point in points.drain(..) {
            if let Some(last) = deduped.last() {
                match point2_equal(&last.point, &point.point).value() {
                    Some(true) => continue,
                    Some(false) => {}
                    None => {
                        return Err(
                            LineRationalQuadraticBezierArrangementError::UndecidablePointEquality,
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

fn certify_line_orders(
    points: &[MixedConicLineArrangementBreakpoint],
    line_index: usize,
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for left in 0..points.len() {
        for right in (left + 1)..points.len() {
            compare_line_parameters(&points[left], &points[right], policy).ok_or(
                LineRationalQuadraticBezierArrangementError::UndecidableLineOrder {
                    line: line_index,
                },
            )?;
        }
    }
    Ok(())
}

fn compare_line_parameters(
    left: &MixedConicLineArrangementBreakpoint,
    right: &MixedConicLineArrangementBreakpoint,
    policy: PredicatePolicy,
) -> Option<Ordering> {
    compare_reals_with_policy(
        &(left.parameter_numerator.clone() * right.parameter_denominator.clone()),
        &(right.parameter_numerator.clone() * left.parameter_denominator.clone()),
        policy,
    )
    .value()
}

fn sort_and_dedup_conic_breakpoints(
    breakpoints: &mut [Vec<RationalQuadraticBezierRealBreakpoint>],
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for (curve_index, points) in breakpoints.iter_mut().enumerate() {
        certify_conic_orders(points, curve_index, policy)?;
        points.sort_by(|left, right| {
            compare_reals_with_policy(&left.parameter, &right.parameter, policy)
                .value()
                .expect("conic breakpoint order was certified before sorting")
        });
        let mut deduped: Vec<RationalQuadraticBezierRealBreakpoint> = Vec::new();
        for point in points.drain(..) {
            if let Some(last) = deduped.last() {
                match compare_reals_with_policy(&last.parameter, &point.parameter, policy).value() {
                    Some(Ordering::Equal) => continue,
                    Some(Ordering::Less | Ordering::Greater) => {}
                    None => {
                        return Err(
                            LineRationalQuadraticBezierArrangementError::UndecidableConicOrder {
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

fn certify_conic_orders(
    points: &[RationalQuadraticBezierRealBreakpoint],
    curve_index: usize,
    policy: PredicatePolicy,
) -> Result<(), LineRationalQuadraticBezierArrangementError> {
    for left in 0..points.len() {
        for right in (left + 1)..points.len() {
            compare_reals_with_policy(&points[left].parameter, &points[right].parameter, policy)
                .value()
                .ok_or(
                    LineRationalQuadraticBezierArrangementError::UndecidableConicOrder {
                        curve: curve_index,
                    },
                )?;
        }
    }
    Ok(())
}

fn build_line_fragments(
    breakpoints: &[Vec<MixedConicLineArrangementBreakpoint>],
    policy: PredicatePolicy,
) -> Result<Vec<MixedConicLineArrangementFragment>, LineRationalQuadraticBezierArrangementError> {
    let mut fragments = Vec::new();
    for points in breakpoints {
        for window in points.windows(2) {
            if compare_line_parameters(&window[0], &window[1], policy) == Some(Ordering::Equal) {
                continue;
            }
            fragments.push(MixedConicLineArrangementFragment {
                source_line: window[0].line,
                start: window[0].clone(),
                end: window[1].clone(),
                segment: LinePathSegment::new(window[0].point.clone(), window[1].point.clone()),
            });
        }
    }
    Ok(fragments)
}

fn build_conic_fragments(
    breakpoints: &[Vec<RationalQuadraticBezierRealBreakpoint>],
    curves: &[RationalQuadraticBezier],
    policy: PredicatePolicy,
) -> Result<Vec<RationalQuadraticBezierRealFragment>, LineRationalQuadraticBezierArrangementError> {
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
                        LineRationalQuadraticBezierArrangementError::UndecidableConicOrder {
                            curve: window[0].curve,
                        },
                    );
                }
            }
            let fragment =
                rational_quadratic_subcurve_real(&curves[window[0].curve], &window[0], &window[1]);
            fragments.push(fragment);
        }
    }
    Ok(fragments)
}

fn rational_quadratic_subcurve_real(
    curve: &RationalQuadraticBezier,
    start: &RationalQuadraticBezierRealBreakpoint,
    end: &RationalQuadraticBezierRealBreakpoint,
) -> RationalQuadraticBezierRealFragment {
    let start_control = homogeneous_eval_real(curve, &start.parameter);
    let end_control = homogeneous_eval_real(curve, &end.parameter);
    let delta = end.parameter.clone() - start.parameter.clone();
    let derivative = homogeneous_derivative_real(curve, &start.parameter);
    let half = Real::from(2);
    let control = HomogeneousPoint2 {
        x: start_control.x.clone()
            + (delta.clone() * derivative.x / half.clone()).expect("nonzero two"),
        y: start_control.y.clone()
            + (delta.clone() * derivative.y / half.clone()).expect("nonzero two"),
        w: start_control.w.clone() + (delta * derivative.w / half).expect("nonzero two"),
    };
    RationalQuadraticBezierRealFragment {
        source_curve: start.curve,
        start: start.clone(),
        end: end.clone(),
        start_control,
        control,
        end_control,
    }
}

fn homogeneous_eval_real(curve: &RationalQuadraticBezier, parameter: &Real) -> HomogeneousPoint2 {
    let one_minus_t = Real::one() - parameter.clone();
    let b0 = one_minus_t.clone() * one_minus_t.clone();
    let b1 = Real::from(2) * one_minus_t * parameter.clone();
    let b2 = parameter.clone() * parameter.clone();
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

fn homogeneous_derivative_real(
    curve: &RationalQuadraticBezier,
    parameter: &Real,
) -> HomogeneousPoint2 {
    let db0 = -Real::from(2) * (Real::one() - parameter.clone());
    let db1 = Real::from(2) * (Real::one() - Real::from(2) * parameter.clone());
    let db2 = Real::from(2) * parameter.clone();
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

fn input_exact_facts(
    lines: &[LinePathSegment],
    curves: &[RationalQuadraticBezier],
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
    for curve in curves {
        values.extend([
            &curve.start().x,
            &curve.start().y,
            &curve.control().x,
            &curve.control().y,
            &curve.end().x,
            &curve.end().y,
            curve.control_weight(),
        ]);
    }
    Real::exact_set_facts(values)
}

fn fragment_exact_facts(
    lines: &[MixedConicLineArrangementFragment],
    curves: &[RationalQuadraticBezierRealFragment],
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
