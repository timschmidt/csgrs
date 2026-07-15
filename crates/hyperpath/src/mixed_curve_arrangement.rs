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
//! Numerical proposal and exact object
//! construction are separated from topology acceptance. The polynomial
//! Bezier hodograph/convex-hull facts are the standard curve-carrier
//! discipline described by Farouki, *Pythagorean Hodograph Curves* (2008).
//! Explicit circular arcs use the same exact curve-object/predicate split as
//! circular-arc arrangement packages such as CGAL Arrangement_on_surface_2.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal};
use hyperreal::{Real, RealExactSetFacts};

use crate::arc::{ExplicitArcPointClassification, ExplicitCircularArc};
use crate::arrangement::{
    ExplicitArcArrangementFragment, LineArcArrangementEvent, LineArrangementBreakpoint,
    LineArrangementError, arrange_line_segments_with_explicit_arcs_and_provenance,
};
use crate::bezier::{BezierParameter, CubicBezier, QuadraticBezier, RationalQuadraticBezier};
use crate::curve_cell::{
    CurveArrangementCellError, CurveArrangementCellGraph, build_line_mixed_bezier_cell_graph,
};
use crate::mixed_bezier_arrangement::{
    LineQuadraticBezierArrangementError, LineQuadraticBezierArrangementEvent,
    MixedLineArrangementBreakpoint, MixedLineArrangementFragment, QuadraticBezierRealFragment,
    arrange_line_segments_with_quadratic_beziers_and_provenance,
};
use crate::mixed_conic_arrangement::{
    LineRationalQuadraticBezierAlgebraicBreakpoint,
    LineRationalQuadraticBezierAlgebraicBreakpointOrder,
    LineRationalQuadraticBezierAlgebraicBreakpointSequence,
    LineRationalQuadraticBezierAlgebraicEndpointEnvelope,
    LineRationalQuadraticBezierAlgebraicSourceSpan, LineRationalQuadraticBezierArrangementError,
    LineRationalQuadraticBezierArrangementEvent,
    LineRationalQuadraticBezierExactAlgebraicBreakpointPromotion,
    LineRationalQuadraticBezierSupportOverlapCandidate, MixedConicLineArrangementBreakpoint,
    RationalQuadraticBezierRealFragment,
    arrange_line_segments_with_rational_quadratic_beziers_and_provenance,
};
use crate::mixed_cubic_arrangement::{
    CubicBezierRealFragment, LineCubicBezierAlgebraicBreakpoint,
    LineCubicBezierAlgebraicBreakpointOrder, LineCubicBezierAlgebraicBreakpointSequence,
    LineCubicBezierAlgebraicEndpointEnvelope, LineCubicBezierAlgebraicOverlapBreakpoint,
    LineCubicBezierAlgebraicOverlapBreakpointOrder,
    LineCubicBezierAlgebraicOverlapBreakpointSequence,
    LineCubicBezierAlgebraicOverlapEndpointEnvelope, LineCubicBezierAlgebraicOverlapSourceSpan,
    LineCubicBezierAlgebraicSourceSpan, LineCubicBezierArrangementError,
    LineCubicBezierArrangementEvent, LineCubicBezierExactAlgebraicBreakpointPromotion,
    LineCubicBezierExactAlgebraicOverlapBreakpointPromotion,
    LineCubicBezierSupportOverlapCandidate, MixedCubicLineArrangementBreakpoint,
    arrange_line_segments_with_cubic_beziers_and_provenance,
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

/// Original non-line curve source for a mixed scheduler fragment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MixedCurveSourceRef {
    /// Input explicit circular arc index.
    ExplicitArc(usize),
    /// Input quadratic Bezier index.
    Quadratic(usize),
    /// Input cubic Bezier index.
    Cubic(usize),
    /// Input rational quadratic conic index.
    RationalQuadratic(usize),
}

/// Exact reason a pair of non-line fragments was accepted by the bounded mixed scheduler.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MixedCurveFragmentSeparationClass {
    /// Both fragments are exact sub-fragments of the same original curve.
    SameSourceSibling,
    /// Distinct-source boxes meet at one exact shared fragment endpoint.
    EndpointContact,
    /// The left fragment box is strictly before the right box on the x-axis.
    LeftBeforeRightX,
    /// The right fragment box is strictly before the left box on the x-axis.
    RightBeforeLeftX,
    /// The left fragment box is strictly below the right box on the y-axis.
    LeftBelowRightY,
    /// The right fragment box is strictly below the left box on the y-axis.
    RightBelowLeftY,
}

/// Endpoint selector for a retained mixed-curve fragment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MixedCurveFragmentEndpoint {
    /// Fragment start endpoint.
    Start,
    /// Fragment end endpoint.
    End,
}

/// Exact outgoing tangent orientation at an admitted endpoint contact.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MixedCurveEndpointTangentClass {
    /// The left outgoing tangent turns counter-clockwise to the right outgoing tangent.
    CounterClockwise,
    /// The left outgoing tangent turns clockwise to the right outgoing tangent.
    Clockwise,
    /// The outgoing tangent directions are collinear.
    Collinear,
}

/// Replay certificate for one accepted non-line fragment pair.
///
/// The bounded mixed scheduler still refuses general curve-curve topology.
/// This certificate records why a retained pair was allowed into the shared
/// graph: either both fragments are siblings emitted by one exact pairwise
/// split scheduler, an exact endpoint-corner contact was replayed, or an
/// exact axis-aligned hull inequality separates two distinct sources. Accepted
/// topology is accompanied by retained
/// predicate evidence, and unsupported topology remains explicit.
/// Same-source algebraic siblings arise from Collins-Loos isolated roots
/// promoted by cubic/conic sub-schedulers, while Bezier/conic hulls retain the
/// Farouki polynomial/rational curve-carrier discipline. Endpoint contacts are
/// deliberately narrower than curve-curve arrangement: both hull boxes must
/// meet at a single exact corner and the retained fragment endpoints at that
/// corner must compare equal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MixedCurveFragmentSeparation {
    /// Left non-line fragment in report fragment-index space.
    pub left: MixedCurveFragmentRef,
    /// Right non-line fragment in report fragment-index space.
    pub right: MixedCurveFragmentRef,
    /// Original curve source of `left`.
    pub left_source: MixedCurveSourceRef,
    /// Original curve source of `right`.
    pub right_source: MixedCurveSourceRef,
    /// Endpoint on `left` when `class == EndpointContact`.
    pub left_endpoint: Option<MixedCurveFragmentEndpoint>,
    /// Endpoint on `right` when `class == EndpointContact`.
    pub right_endpoint: Option<MixedCurveFragmentEndpoint>,
    /// Outgoing tangent orientation when `class == EndpointContact`.
    pub endpoint_tangent_class: Option<MixedCurveEndpointTangentClass>,
    /// Certified reason this pair was accepted.
    pub class: MixedCurveFragmentSeparationClass,
}

/// Exact coordinate envelope retained for one non-line mixed fragment.
///
/// This is the public counterpart of the scheduler's separation box. It is
/// emitted before the bounded mixed graph accepts topology, so callers can
/// replay the exact geometric facts used to admit or reject cross-source
/// curve pairs. That is the Yap boundary from "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): a numerical schedule
/// may propose candidates, but accepted topology is backed by exact retained
/// predicates and construction facts. Polynomial Bezier extrema are derived
/// from Bernstein hodographs as in Farouki, *Pythagorean Hodograph Curves*
/// (2008); rational-quadratic extrema use the denominator-cleared quotient
/// derivative `N'W - NW'`, again only after exact membership and nonzero
/// homogeneous-weight replay.
#[derive(Clone, Debug, PartialEq)]
pub struct MixedCurveFragmentEnvelope {
    /// Non-line fragment in report fragment-index space.
    pub fragment: MixedCurveFragmentRef,
    /// Original source curve that produced `fragment`.
    pub source: MixedCurveSourceRef,
    /// Exact affine start point of the retained fragment.
    pub start: Point2,
    /// Exact affine end point of the retained fragment.
    pub end: Point2,
    /// Minimum x-coordinate over the retained fragment.
    pub x_min: Real,
    /// Maximum x-coordinate over the retained fragment.
    pub x_max: Real,
    /// Minimum y-coordinate over the retained fragment.
    pub y_min: Real,
    /// Maximum y-coordinate over the retained fragment.
    pub y_max: Real,
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

/// Retained algebraic evidence copied from the line/cubic sub-scheduler.
///
/// The bounded mixed scheduler merges native line breakpoints and then builds
/// one concrete cell graph only from exact `Real` fragments. Cubic support
/// roots and overlap boundaries may still be represented algebraic objects, so
/// they are retained here as replay evidence instead of being discarded at the
/// family boundary. Exact objects and exact predicates
/// are preserved even when construction is not yet available. The represented
/// roots and order certificates use the Sturm/Collins-Loos model described by
/// Collins and Loos, "Real Zeros of Polynomials" (1982), while native cubic
/// fragments remain Bezier restrictions in the Farouki curve-carrier sense
/// (*Pythagorean Hodograph Curves*, 2008).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LineMixedCubicAlgebraicEvidence {
    /// Same-support line/cubic overlap candidates retained by the cubic scheduler.
    pub support_overlaps: Vec<LineCubicBezierSupportOverlapCandidate>,
    /// True line/cubic support roots retained as represented algebraic candidates.
    pub algebraic_breakpoints: Vec<LineCubicBezierAlgebraicBreakpoint>,
    /// Pairwise order evidence for retained true-support algebraic breakpoints.
    pub algebraic_breakpoint_orders: Vec<LineCubicBezierAlgebraicBreakpointOrder>,
    /// Per-line and per-cubic readiness sequences for true-support algebraic breakpoints.
    pub algebraic_breakpoint_sequences: Vec<LineCubicBezierAlgebraicBreakpointSequence>,
    /// Conservative source spans induced by ordered true-support algebraic breakpoints.
    pub algebraic_source_spans: Vec<LineCubicBezierAlgebraicSourceSpan>,
    /// Conservative endpoint coordinate envelopes for true-support algebraic spans.
    pub algebraic_endpoint_envelopes: Vec<LineCubicBezierAlgebraicEndpointEnvelope>,
    /// Same-support overlap boundaries retained as represented algebraic candidates.
    pub algebraic_overlap_breakpoints: Vec<LineCubicBezierAlgebraicOverlapBreakpoint>,
    /// Pairwise order evidence for retained overlap-boundary candidates.
    pub algebraic_overlap_breakpoint_orders: Vec<LineCubicBezierAlgebraicOverlapBreakpointOrder>,
    /// Per-line and per-cubic readiness sequences for retained overlap boundaries.
    pub algebraic_overlap_breakpoint_sequences:
        Vec<LineCubicBezierAlgebraicOverlapBreakpointSequence>,
    /// Conservative source spans induced by ordered overlap-boundary candidates.
    pub algebraic_overlap_source_spans: Vec<LineCubicBezierAlgebraicOverlapSourceSpan>,
    /// Conservative endpoint coordinate envelopes for overlap-boundary source spans.
    pub algebraic_overlap_endpoint_envelopes: Vec<LineCubicBezierAlgebraicOverlapEndpointEnvelope>,
    /// Exact rational true-support roots that were promoted into native split parameters.
    pub exact_algebraic_breakpoint_promotions:
        Vec<LineCubicBezierExactAlgebraicBreakpointPromotion>,
    /// Exact rational overlap-boundary roots that were promoted into native split parameters.
    pub exact_algebraic_overlap_breakpoint_promotions:
        Vec<LineCubicBezierExactAlgebraicOverlapBreakpointPromotion>,
}

/// Retained algebraic evidence copied from the line/rational-quadratic sub-scheduler.
///
/// Rational quadratic same-support overlap boundaries can be nonmonotone in
/// the affine line image. The conic sub-scheduler therefore retains
/// homogeneous boundary roots, exact order evidence, and source-span envelopes
/// without pretending every represented root is a native `Real` breakpoint.
/// The mixed scheduler keeps that evidence intact across the family merge.
/// This is the Yap (1997) exact-geometric-computation boundary: preserve
/// exact replay artifacts and leave unknown construction explicit. The root
/// isolation/order model follows Collins and Loos (1982), and the rational
/// curve remains in the homogeneous Farouki (2008) form until a certified
/// materializer consumes it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LineMixedRationalQuadraticAlgebraicEvidence {
    /// Same-support line/conic overlap candidates retained by the conic scheduler.
    pub support_overlaps: Vec<LineRationalQuadraticBezierSupportOverlapCandidate>,
    /// Nonmonotone overlap-boundary roots retained as represented algebraic candidates.
    pub algebraic_breakpoints: Vec<LineRationalQuadraticBezierAlgebraicBreakpoint>,
    /// Pairwise order evidence for retained conic algebraic breakpoints.
    pub algebraic_breakpoint_orders: Vec<LineRationalQuadraticBezierAlgebraicBreakpointOrder>,
    /// Per-line and per-conic readiness sequences for retained algebraic breakpoints.
    pub algebraic_breakpoint_sequences: Vec<LineRationalQuadraticBezierAlgebraicBreakpointSequence>,
    /// Conservative source spans induced by ordered retained conic breakpoints.
    pub algebraic_source_spans: Vec<LineRationalQuadraticBezierAlgebraicSourceSpan>,
    /// Conservative endpoint coordinate envelopes for retained conic algebraic spans.
    pub algebraic_endpoint_envelopes: Vec<LineRationalQuadraticBezierAlgebraicEndpointEnvelope>,
    /// Exact rational represented roots that were promoted into native conic split parameters.
    pub exact_algebraic_breakpoint_promotions:
        Vec<LineRationalQuadraticBezierExactAlgebraicBreakpointPromotion>,
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
    /// Retained algebraic evidence from the line/cubic sub-scheduler.
    pub cubic_algebraic_evidence: LineMixedCubicAlgebraicEvidence,
    /// Retained algebraic evidence from the line/conic sub-scheduler.
    pub rational_quadratic_algebraic_evidence: LineMixedRationalQuadraticAlgebraicEvidence,
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
    /// Exact coordinate envelopes for every retained non-line fragment.
    pub fragment_envelopes: Vec<MixedCurveFragmentEnvelope>,
    /// Certificates for every accepted non-line fragment pair.
    pub fragment_separations: Vec<MixedCurveFragmentSeparation>,
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
    let arc_report =
        arrange_line_segments_with_explicit_arcs_and_provenance(lines, arcs, policy, provenance)
            .map_err(LineMixedBezierArrangementError::Arc)?;
    let quadratic_report = arrange_line_segments_with_quadratic_beziers_and_provenance(
        lines,
        quadratic_curves,
        policy,
        provenance,
    )
    .map_err(LineMixedBezierArrangementError::Quadratic)?;
    let cubic_report = arrange_line_segments_with_cubic_beziers_and_provenance(
        lines,
        cubic_curves,
        policy,
        provenance,
    )
    .map_err(LineMixedBezierArrangementError::Cubic)?;
    let rational_quadratic_report =
        arrange_line_segments_with_rational_quadratic_beziers_and_provenance(
            lines,
            rational_quadratic_curves,
            policy,
            provenance,
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
    let fragment_boxes = build_curve_fragment_boxes(
        &arc_report.arc_fragments,
        &quadratic_report.bezier_fragments,
        &cubic_report.cubic_fragments,
        &rational_quadratic_report.conic_fragments,
        policy,
    )?;
    let fragment_envelopes = fragment_boxes
        .iter()
        .map(MixedCurveFragmentEnvelope::from_box)
        .collect::<Vec<_>>();
    let fragment_separations = validate_curve_fragment_separation(&fragment_boxes, policy)?;
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
        cubic_algebraic_evidence: LineMixedCubicAlgebraicEvidence {
            support_overlaps: cubic_report.support_overlaps,
            algebraic_breakpoints: cubic_report.algebraic_breakpoints,
            algebraic_breakpoint_orders: cubic_report.algebraic_breakpoint_orders,
            algebraic_breakpoint_sequences: cubic_report.algebraic_breakpoint_sequences,
            algebraic_source_spans: cubic_report.algebraic_source_spans,
            algebraic_endpoint_envelopes: cubic_report.algebraic_endpoint_envelopes,
            algebraic_overlap_breakpoints: cubic_report.algebraic_overlap_breakpoints,
            algebraic_overlap_breakpoint_orders: cubic_report.algebraic_overlap_breakpoint_orders,
            algebraic_overlap_breakpoint_sequences: cubic_report
                .algebraic_overlap_breakpoint_sequences,
            algebraic_overlap_source_spans: cubic_report.algebraic_overlap_source_spans,
            algebraic_overlap_endpoint_envelopes: cubic_report.algebraic_overlap_endpoint_envelopes,
            exact_algebraic_breakpoint_promotions: cubic_report
                .exact_algebraic_breakpoint_promotions,
            exact_algebraic_overlap_breakpoint_promotions: cubic_report
                .exact_algebraic_overlap_breakpoint_promotions,
        },
        rational_quadratic_algebraic_evidence: LineMixedRationalQuadraticAlgebraicEvidence {
            support_overlaps: rational_quadratic_report.support_overlaps,
            algebraic_breakpoints: rational_quadratic_report.algebraic_breakpoints,
            algebraic_breakpoint_orders: rational_quadratic_report.algebraic_breakpoint_orders,
            algebraic_breakpoint_sequences: rational_quadratic_report
                .algebraic_breakpoint_sequences,
            algebraic_source_spans: rational_quadratic_report.algebraic_source_spans,
            algebraic_endpoint_envelopes: rational_quadratic_report.algebraic_endpoint_envelopes,
            exact_algebraic_breakpoint_promotions: rational_quadratic_report
                .exact_algebraic_breakpoint_promotions,
        },
        line_breakpoints,
        line_fragments,
        arc_fragments: arc_report.arc_fragments,
        quadratic_fragments: quadratic_report.bezier_fragments,
        cubic_fragments: cubic_report.cubic_fragments,
        rational_quadratic_fragments: rational_quadratic_report.conic_fragments,
        fragment_envelopes,
        fragment_separations,
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
    _policy: PredicatePolicy,
) -> Result<(), LineMixedBezierArrangementError> {
    for existing in breakpoints.iter() {
        match point2_equal(&existing.point, &point.point).value() {
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
                match point2_equal(&last.point, &point.point).value() {
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
    owner: MixedCurveSourceRef,
    start: Point2,
    end: Point2,
    start_tangent: Point2,
    end_tangent: Point2,
    x_min: Real,
    x_max: Real,
    y_min: Real,
    y_max: Real,
}

impl FragmentBox {
    fn with_tangents(mut self, start_tangent: Point2, end_tangent: Point2) -> Self {
        self.start_tangent = start_tangent;
        self.end_tangent = end_tangent;
        self
    }
}

impl MixedCurveFragmentEnvelope {
    fn from_box(fragment_box: &FragmentBox) -> Self {
        Self {
            fragment: fragment_box.source,
            source: fragment_box.owner,
            start: fragment_box.start.clone(),
            end: fragment_box.end.clone(),
            x_min: fragment_box.x_min.clone(),
            x_max: fragment_box.x_max.clone(),
            y_min: fragment_box.y_min.clone(),
            y_max: fragment_box.y_max.clone(),
        }
    }
}

fn build_curve_fragment_boxes(
    arcs: &[ExplicitArcArrangementFragment],
    quadratics: &[QuadraticBezierRealFragment],
    cubics: &[CubicBezierRealFragment],
    conics: &[RationalQuadraticBezierRealFragment],
    policy: PredicatePolicy,
) -> Result<Vec<FragmentBox>, LineMixedBezierArrangementError> {
    let mut boxes = Vec::new();
    for (index, fragment) in arcs.iter().enumerate() {
        boxes.push(FragmentBox {
            source: MixedCurveFragmentRef::ExplicitArc(index),
            owner: MixedCurveSourceRef::ExplicitArc(fragment.source_arc),
            ..box_from_explicit_arc(fragment, policy)?
        });
    }
    for (index, fragment) in quadratics.iter().enumerate() {
        boxes.push(FragmentBox {
            source: MixedCurveFragmentRef::Quadratic(index),
            owner: MixedCurveSourceRef::Quadratic(fragment.source_curve),
            ..box_from_quadratic_fragment(fragment, policy)?
        });
    }
    for (index, fragment) in cubics.iter().enumerate() {
        boxes.push(FragmentBox {
            source: MixedCurveFragmentRef::Cubic(index),
            owner: MixedCurveSourceRef::Cubic(fragment.source_curve),
            ..box_from_cubic_fragment(fragment, policy)?
        });
    }
    for (index, fragment) in conics.iter().enumerate() {
        boxes.push(FragmentBox {
            source: MixedCurveFragmentRef::RationalQuadratic(index),
            owner: MixedCurveSourceRef::RationalQuadratic(fragment.source_curve),
            ..box_from_conic_fragment(fragment, policy)?
        });
    }
    Ok(boxes)
}

fn validate_curve_fragment_separation(
    boxes: &[FragmentBox],
    policy: PredicatePolicy,
) -> Result<Vec<MixedCurveFragmentSeparation>, LineMixedBezierArrangementError> {
    let mut separations = Vec::new();
    for left in 0..boxes.len() {
        for right in (left + 1)..boxes.len() {
            // Same-source siblings are already the output of one retained
            // pairwise split scheduler. Yap's exact-object boundary lets this
            // layer consume those constructed fragments as one source object;
            // only cross-source curve-curve topology needs a new certificate
            // before the bounded mixed graph may accept it. This distinction
            // is essential for exact algebraic root promotions: Collins-Loos
            // isolated roots can split a conic or cubic into multiple native
            // fragments whose convex hull boxes still overlap at certified
            // same-source boundaries.
            if boxes[left].owner == boxes[right].owner {
                separations.push(fragment_separation(
                    &boxes[left],
                    &boxes[right],
                    MixedCurveFragmentSeparationClass::SameSourceSibling,
                    None,
                    None,
                    None,
                ));
                continue;
            }
            if let Some(class) = boxes_separation_class(&boxes[left], &boxes[right], policy)? {
                separations.push(fragment_separation(
                    &boxes[left],
                    &boxes[right],
                    class,
                    None,
                    None,
                    None,
                ));
            } else if let Some((left_endpoint, right_endpoint)) =
                endpoint_corner_contact(&boxes[left], &boxes[right], policy)?
            {
                let tangent_class = endpoint_tangent_class(
                    &boxes[left],
                    left_endpoint,
                    &boxes[right],
                    right_endpoint,
                    policy,
                )?;
                separations.push(fragment_separation(
                    &boxes[left],
                    &boxes[right],
                    MixedCurveFragmentSeparationClass::EndpointContact,
                    Some(left_endpoint),
                    Some(right_endpoint),
                    Some(tangent_class),
                ));
            } else {
                return Err(
                    LineMixedBezierArrangementError::UnsupportedCurveCurveInteraction {
                        left: boxes[left].source,
                        right: boxes[right].source,
                    },
                );
            }
        }
    }
    Ok(separations)
}

fn fragment_separation(
    left: &FragmentBox,
    right: &FragmentBox,
    class: MixedCurveFragmentSeparationClass,
    left_endpoint: Option<MixedCurveFragmentEndpoint>,
    right_endpoint: Option<MixedCurveFragmentEndpoint>,
    endpoint_tangent_class: Option<MixedCurveEndpointTangentClass>,
) -> MixedCurveFragmentSeparation {
    MixedCurveFragmentSeparation {
        left: left.source,
        right: right.source,
        left_source: left.owner,
        right_source: right.owner,
        left_endpoint,
        right_endpoint,
        endpoint_tangent_class,
        class,
    }
}

/// Build a sweep-aware exact hull box for an explicit circular-arc fragment.
///
/// Topology is accepted only from
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
    hull.owner = MixedCurveSourceRef::ExplicitArc(usize::MAX);
    hull.start = arc.start().clone();
    hull.end = arc.end().clone();
    hull.start_tangent = arc.start_tangent();
    hull.end_tangent = arc.end_tangent();
    Ok(hull)
}

/// Build a certified coordinate-extrema box for a quadratic Bezier fragment.
///
/// The previous mixed scheduler used the quadratic control hull as a safe
/// rejection box. This routine keeps the same Yap boundary, "Towards Exact
/// Geometric Computation" (1997): a cross-curve pair is accepted only after an
/// exact predicate proves strict box separation. The box is now tighter because
/// polynomial coordinate extrema are admitted from exact derivative roots
/// inside `[0, 1]`; Farouki, *Pythagorean Hodograph Curves* (2008), describes
/// the Bernstein derivative carrier used here. No sampled point can shrink the
/// retained box.
fn box_from_quadratic_fragment(
    fragment: &QuadraticBezierRealFragment,
    policy: PredicatePolicy,
) -> Result<FragmentBox, LineMixedBezierArrangementError> {
    let curve = &fragment.curve;
    let mut hull = box_from_points([curve.start(), curve.end()], policy)?;
    for root in quadratic_coordinate_extrema_parameters(curve, Coordinate::X, policy)? {
        update_box_with_point(&mut hull, &eval_quadratic_real(curve, &root), policy)?;
    }
    for root in quadratic_coordinate_extrema_parameters(curve, Coordinate::Y, policy)? {
        update_box_with_point(&mut hull, &eval_quadratic_real(curve, &root), policy)?;
    }
    Ok(hull.with_tangents(
        quadratic_start_tangent(fragment),
        quadratic_end_tangent(fragment),
    ))
}

/// Build a certified coordinate-extrema box for a cubic Bezier fragment.
///
/// Cubic coordinate extrema occur at roots of the derivative quadratic. Roots
/// are constructed exactly as `Real` values and admitted only when exact
/// comparison certifies membership in the retained source interval `[0, 1]`.
/// If that proof is unavailable, the scheduler reports an undecidable exact
/// predicate rather than widening from samples. This follows Yap's exact
/// object/predicate split and the Bernstein hodograph construction described
/// by Farouki, *Pythagorean Hodograph Curves* (2008).
fn box_from_cubic_fragment(
    fragment: &CubicBezierRealFragment,
    policy: PredicatePolicy,
) -> Result<FragmentBox, LineMixedBezierArrangementError> {
    let curve = &fragment.curve;
    let mut hull = box_from_points([curve.start(), curve.end()], policy)?;
    for root in cubic_coordinate_extrema_parameters(curve, Coordinate::X, policy)? {
        update_box_with_point(&mut hull, &eval_cubic_real(curve, &root), policy)?;
    }
    for root in cubic_coordinate_extrema_parameters(curve, Coordinate::Y, policy)? {
        update_box_with_point(&mut hull, &eval_cubic_real(curve, &root), policy)?;
    }
    Ok(hull.with_tangents(cubic_start_tangent(fragment), cubic_end_tangent(fragment)))
}

/// Build a certified coordinate-extrema box for a rational-quadratic fragment.
///
/// Rational conic coordinates are quotient curves `R(t)=N(t)/W(t)`. Following
/// Yap's exact geometric-computation model, the mixed scheduler may use the
/// resulting box only when every extremum witness is an exact retained object:
/// derivative roots are solved from the denominator-cleared numerator
/// `N'W - NW'`, admitted by exact `[0, 1]` comparisons, and evaluated only
/// after proving the homogeneous weight is nonzero. The quotient-derivative
/// form is the standard rational Bezier hodograph relation described by
/// Farouki, *Pythagorean Hodograph Curves* (2008).
fn box_from_conic_fragment(
    fragment: &RationalQuadraticBezierRealFragment,
    policy: PredicatePolicy,
) -> Result<FragmentBox, LineMixedBezierArrangementError> {
    let start = affine_homogeneous_point(&fragment.start_control, policy)?;
    let end = affine_homogeneous_point(&fragment.end_control, policy)?;
    let mut hull = box_from_points([&start, &end], policy)?;
    for root in conic_coordinate_extrema_parameters(fragment, Coordinate::X, policy)? {
        update_box_with_point(
            &mut hull,
            &eval_conic_fragment_real(fragment, &root, policy)?,
            policy,
        )?;
    }
    for root in conic_coordinate_extrema_parameters(fragment, Coordinate::Y, policy)? {
        update_box_with_point(
            &mut hull,
            &eval_conic_fragment_real(fragment, &root, policy)?,
            policy,
        )?;
    }
    Ok(hull.with_tangents(
        homogeneous_endpoint_tangent(&fragment.start_control, &fragment.control),
        homogeneous_endpoint_tangent(&fragment.control, &fragment.end_control),
    ))
}

fn box_from_points<const N: usize>(
    points: [&Point2; N],
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
        owner: MixedCurveSourceRef::Quadratic(usize::MAX),
        start: points[0].clone(),
        end: points[N - 1].clone(),
        start_tangent: Point2::new(Real::zero(), Real::zero()),
        end_tangent: Point2::new(Real::zero(), Real::zero()),
        x_min,
        x_max,
        y_min,
        y_max,
    })
}

fn update_box_with_point(
    hull: &mut FragmentBox,
    point: &Point2,
    policy: PredicatePolicy,
) -> Result<(), LineMixedBezierArrangementError> {
    update_min_max(&mut hull.x_min, &mut hull.x_max, &point.x, policy)?;
    update_min_max(&mut hull.y_min, &mut hull.y_max, &point.y, policy)
}

#[derive(Clone, Copy)]
enum Coordinate {
    X,
    Y,
}

fn quadratic_coordinate_extrema_parameters(
    curve: &QuadraticBezier,
    coordinate: Coordinate,
    policy: PredicatePolicy,
) -> Result<Vec<Real>, LineMixedBezierArrangementError> {
    let p0 = coordinate_value(curve.start(), coordinate);
    let p1 = coordinate_value(curve.control(), coordinate);
    let p2 = coordinate_value(curve.end(), coordinate);
    let denominator = p0.clone() - Real::from(2) * p1.clone() + p2;
    if compare_required(&denominator, &Real::zero(), policy)? == Ordering::Equal {
        return Ok(Vec::new());
    }
    let root = ((p0 - p1) / denominator)
        .map_err(|_| LineMixedBezierArrangementError::UndecidablePointEquality)?;
    if real_in_unit_interval(&root, policy)? {
        Ok(vec![root])
    } else {
        Ok(Vec::new())
    }
}

fn cubic_coordinate_extrema_parameters(
    curve: &CubicBezier,
    coordinate: Coordinate,
    policy: PredicatePolicy,
) -> Result<Vec<Real>, LineMixedBezierArrangementError> {
    let p0 = coordinate_value(curve.start(), coordinate);
    let p1 = coordinate_value(curve.control0(), coordinate);
    let p2 = coordinate_value(curve.control1(), coordinate);
    let p3 = coordinate_value(curve.end(), coordinate);
    let a = -p0.clone() + Real::from(3) * p1.clone() - Real::from(3) * p2.clone() + p3;
    let b = Real::from(3) * p0.clone() - Real::from(6) * p1.clone() + Real::from(3) * p2;
    let c = Real::from(3) * (p1 - p0);
    let roots = solve_quadratic_or_linear_real(Real::from(3) * a, Real::from(2) * b, c, policy)?;
    let mut accepted = Vec::new();
    for root in roots {
        if real_in_unit_interval(&root, policy)? {
            accepted.push(root);
        }
    }
    Ok(accepted)
}

fn conic_coordinate_extrema_parameters(
    fragment: &RationalQuadraticBezierRealFragment,
    coordinate: Coordinate,
    policy: PredicatePolicy,
) -> Result<Vec<Real>, LineMixedBezierArrangementError> {
    let numerator = conic_coordinate_power_coefficients(fragment, coordinate);
    let denominator = conic_weight_power_coefficients(fragment);
    let constant = numerator[1].clone() * denominator[0].clone()
        - numerator[0].clone() * denominator[1].clone();
    let linear = Real::from(2)
        * (numerator[2].clone() * denominator[0].clone()
            - numerator[0].clone() * denominator[2].clone());
    let quadratic = numerator[2].clone() * denominator[1].clone()
        - numerator[1].clone() * denominator[2].clone();
    let roots = solve_quadratic_or_linear_real(quadratic, linear, constant, policy)?;
    let mut accepted = Vec::new();
    for root in roots {
        if real_in_unit_interval(&root, policy)? {
            accepted.push(root);
        }
    }
    Ok(accepted)
}

fn conic_coordinate_power_coefficients(
    fragment: &RationalQuadraticBezierRealFragment,
    coordinate: Coordinate,
) -> [Real; 3] {
    let p0 = homogeneous_coordinate(&fragment.start_control, coordinate);
    let p1 = homogeneous_coordinate(&fragment.control, coordinate);
    let p2 = homogeneous_coordinate(&fragment.end_control, coordinate);
    [
        p0.clone(),
        Real::from(2) * (p1.clone() - p0.clone()),
        p0 - Real::from(2) * p1 + p2,
    ]
}

fn conic_weight_power_coefficients(fragment: &RationalQuadraticBezierRealFragment) -> [Real; 3] {
    let w0 = fragment.start_control.w.clone();
    let w1 = fragment.control.w.clone();
    let w2 = fragment.end_control.w.clone();
    [
        w0.clone(),
        Real::from(2) * (w1.clone() - w0.clone()),
        w0 - Real::from(2) * w1 + w2,
    ]
}

fn solve_quadratic_or_linear_real(
    a: Real,
    b: Real,
    c: Real,
    policy: PredicatePolicy,
) -> Result<Vec<Real>, LineMixedBezierArrangementError> {
    match compare_required(&a, &Real::zero(), policy)? {
        Ordering::Equal => solve_linear_real(b, c, policy),
        Ordering::Less | Ordering::Greater => solve_quadratic_real(a, b, c, policy),
    }
}

fn solve_linear_real(
    b: Real,
    c: Real,
    policy: PredicatePolicy,
) -> Result<Vec<Real>, LineMixedBezierArrangementError> {
    match compare_required(&b, &Real::zero(), policy)? {
        Ordering::Equal => Ok(Vec::new()),
        Ordering::Less | Ordering::Greater => {
            Ok(vec![((-c) / b).map_err(|_| {
                LineMixedBezierArrangementError::UndecidablePointEquality
            })?])
        }
    }
}

fn solve_quadratic_real(
    a: Real,
    b: Real,
    c: Real,
    policy: PredicatePolicy,
) -> Result<Vec<Real>, LineMixedBezierArrangementError> {
    let discriminant = b.clone() * b.clone() - Real::from(4) * a.clone() * c;
    match compare_required(&discriminant, &Real::zero(), policy)? {
        Ordering::Less => Ok(Vec::new()),
        Ordering::Equal => {
            Ok(vec![((-b) / (Real::from(2) * a)).map_err(|_| {
                LineMixedBezierArrangementError::UndecidablePointEquality
            })?])
        }
        Ordering::Greater => {
            let root = discriminant
                .sqrt()
                .map_err(|_| LineMixedBezierArrangementError::UndecidablePointEquality)?;
            let denominator = Real::from(2) * a;
            Ok(vec![
                ((-b.clone() - root.clone()) / denominator.clone())
                    .map_err(|_| LineMixedBezierArrangementError::UndecidablePointEquality)?,
                ((-b + root) / denominator)
                    .map_err(|_| LineMixedBezierArrangementError::UndecidablePointEquality)?,
            ])
        }
    }
}

fn real_in_unit_interval(
    value: &Real,
    policy: PredicatePolicy,
) -> Result<bool, LineMixedBezierArrangementError> {
    let lower = compare_required(value, &Real::zero(), policy)?;
    let upper = compare_required(value, &Real::one(), policy)?;
    Ok(matches!(lower, Ordering::Equal | Ordering::Greater)
        && matches!(upper, Ordering::Equal | Ordering::Less))
}

fn compare_required(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> Result<Ordering, LineMixedBezierArrangementError> {
    compare_reals_with_policy(left, right, policy)
        .value()
        .ok_or(LineMixedBezierArrangementError::UndecidablePointEquality)
}

fn coordinate_value(point: &Point2, coordinate: Coordinate) -> Real {
    match coordinate {
        Coordinate::X => point.x.clone(),
        Coordinate::Y => point.y.clone(),
    }
}

fn homogeneous_coordinate(
    point: &crate::bezier_arrangement::HomogeneousPoint2,
    coordinate: Coordinate,
) -> Real {
    match coordinate {
        Coordinate::X => point.x.clone(),
        Coordinate::Y => point.y.clone(),
    }
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

fn eval_conic_fragment_real(
    fragment: &RationalQuadraticBezierRealFragment,
    parameter: &Real,
    policy: PredicatePolicy,
) -> Result<Point2, LineMixedBezierArrangementError> {
    let one_minus_t = Real::one() - parameter.clone();
    let start_weight = one_minus_t.clone() * one_minus_t.clone();
    let control_weight = Real::from(2) * one_minus_t * parameter.clone();
    let end_weight = parameter.clone() * parameter.clone();
    let homogeneous = crate::bezier_arrangement::HomogeneousPoint2 {
        x: fragment.start_control.x.clone() * start_weight.clone()
            + fragment.control.x.clone() * control_weight.clone()
            + fragment.end_control.x.clone() * end_weight.clone(),
        y: fragment.start_control.y.clone() * start_weight.clone()
            + fragment.control.y.clone() * control_weight.clone()
            + fragment.end_control.y.clone() * end_weight.clone(),
        w: fragment.start_control.w.clone() * start_weight
            + fragment.control.w.clone() * control_weight
            + fragment.end_control.w.clone() * end_weight,
    };
    affine_homogeneous_point(&homogeneous, policy)
}

fn eval_cubic_real(curve: &CubicBezier, parameter: &Real) -> Point2 {
    let one_minus_t = Real::one() - parameter.clone();
    let start_weight = one_minus_t.clone() * one_minus_t.clone() * one_minus_t.clone();
    let control0_weight =
        Real::from(3) * one_minus_t.clone() * one_minus_t.clone() * parameter.clone();
    let control1_weight = Real::from(3) * one_minus_t * parameter.clone() * parameter.clone();
    let end_weight = parameter.clone() * parameter.clone() * parameter.clone();
    Point2::new(
        curve.start().x.clone() * start_weight.clone()
            + curve.control0().x.clone() * control0_weight.clone()
            + curve.control1().x.clone() * control1_weight.clone()
            + curve.end().x.clone() * end_weight.clone(),
        curve.start().y.clone() * start_weight
            + curve.control0().y.clone() * control0_weight
            + curve.control1().y.clone() * control1_weight
            + curve.end().y.clone() * end_weight,
    )
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

fn boxes_separation_class(
    left: &FragmentBox,
    right: &FragmentBox,
    policy: PredicatePolicy,
) -> Result<Option<MixedCurveFragmentSeparationClass>, LineMixedBezierArrangementError> {
    if is_less(&left.x_max, &right.x_min, policy)? {
        Ok(Some(MixedCurveFragmentSeparationClass::LeftBeforeRightX))
    } else if is_less(&right.x_max, &left.x_min, policy)? {
        Ok(Some(MixedCurveFragmentSeparationClass::RightBeforeLeftX))
    } else if is_less(&left.y_max, &right.y_min, policy)? {
        Ok(Some(MixedCurveFragmentSeparationClass::LeftBelowRightY))
    } else if is_less(&right.y_max, &left.y_min, policy)? {
        Ok(Some(MixedCurveFragmentSeparationClass::RightBelowLeftY))
    } else {
        Ok(None)
    }
}

fn endpoint_corner_contact(
    left: &FragmentBox,
    right: &FragmentBox,
    policy: PredicatePolicy,
) -> Result<
    Option<(MixedCurveFragmentEndpoint, MixedCurveFragmentEndpoint)>,
    LineMixedBezierArrangementError,
> {
    let endpoint_pairs = [
        (
            MixedCurveFragmentEndpoint::Start,
            &left.start,
            MixedCurveFragmentEndpoint::Start,
            &right.start,
        ),
        (
            MixedCurveFragmentEndpoint::Start,
            &left.start,
            MixedCurveFragmentEndpoint::End,
            &right.end,
        ),
        (
            MixedCurveFragmentEndpoint::End,
            &left.end,
            MixedCurveFragmentEndpoint::Start,
            &right.start,
        ),
        (
            MixedCurveFragmentEndpoint::End,
            &left.end,
            MixedCurveFragmentEndpoint::End,
            &right.end,
        ),
    ];

    for (left_endpoint, left_point, right_endpoint, right_point) in endpoint_pairs {
        match point2_equal(left_point, right_point).value() {
            Some(true) => {
                if boxes_touch_only_at_corner(left, right, left_point, policy)? {
                    return Ok(Some((left_endpoint, right_endpoint)));
                }
            }
            Some(false) => {}
            None => return Err(LineMixedBezierArrangementError::UndecidablePointEquality),
        }
    }
    Ok(None)
}

fn endpoint_tangent_class(
    left: &FragmentBox,
    left_endpoint: MixedCurveFragmentEndpoint,
    right: &FragmentBox,
    right_endpoint: MixedCurveFragmentEndpoint,
    policy: PredicatePolicy,
) -> Result<MixedCurveEndpointTangentClass, LineMixedBezierArrangementError> {
    let left_tangent = outgoing_endpoint_tangent(left, left_endpoint);
    let right_tangent = outgoing_endpoint_tangent(right, right_endpoint);
    if vector_is_zero(&left_tangent, policy)? || vector_is_zero(&right_tangent, policy)? {
        return Err(LineMixedBezierArrangementError::UndecidablePointEquality);
    }
    let cross = left_tangent.x * right_tangent.y - left_tangent.y * right_tangent.x;
    match compare_reals_with_policy(&cross, &Real::zero(), policy).value() {
        Some(Ordering::Greater) => Ok(MixedCurveEndpointTangentClass::CounterClockwise),
        Some(Ordering::Less) => Ok(MixedCurveEndpointTangentClass::Clockwise),
        Some(Ordering::Equal) => Ok(MixedCurveEndpointTangentClass::Collinear),
        None => Err(LineMixedBezierArrangementError::UndecidablePointEquality),
    }
}

fn outgoing_endpoint_tangent(
    fragment: &FragmentBox,
    endpoint: MixedCurveFragmentEndpoint,
) -> Point2 {
    match endpoint {
        MixedCurveFragmentEndpoint::Start => fragment.start_tangent.clone(),
        MixedCurveFragmentEndpoint::End => Point2::new(
            -fragment.end_tangent.x.clone(),
            -fragment.end_tangent.y.clone(),
        ),
    }
}

fn boxes_touch_only_at_corner(
    left: &FragmentBox,
    right: &FragmentBox,
    point: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, LineMixedBezierArrangementError> {
    Ok((real_equal(&left.x_max, &point.x, policy)?
        && real_equal(&right.x_min, &point.x, policy)?
        && real_equal(&left.y_max, &point.y, policy)?
        && real_equal(&right.y_min, &point.y, policy)?)
        || (real_equal(&left.x_max, &point.x, policy)?
            && real_equal(&right.x_min, &point.x, policy)?
            && real_equal(&left.y_min, &point.y, policy)?
            && real_equal(&right.y_max, &point.y, policy)?)
        || (real_equal(&left.x_min, &point.x, policy)?
            && real_equal(&right.x_max, &point.x, policy)?
            && real_equal(&left.y_max, &point.y, policy)?
            && real_equal(&right.y_min, &point.y, policy)?)
        || (real_equal(&left.x_min, &point.x, policy)?
            && real_equal(&right.x_max, &point.x, policy)?
            && real_equal(&left.y_min, &point.y, policy)?
            && real_equal(&right.y_max, &point.y, policy)?))
}

fn quadratic_start_tangent(fragment: &QuadraticBezierRealFragment) -> Point2 {
    fragment.curve.derivative(BezierParameter {
        numerator: 0,
        denominator: 1,
    })
}

fn quadratic_end_tangent(fragment: &QuadraticBezierRealFragment) -> Point2 {
    fragment.curve.derivative(BezierParameter {
        numerator: 1,
        denominator: 1,
    })
}

fn cubic_start_tangent(fragment: &CubicBezierRealFragment) -> Point2 {
    fragment.curve.derivative(BezierParameter {
        numerator: 0,
        denominator: 1,
    })
}

fn cubic_end_tangent(fragment: &CubicBezierRealFragment) -> Point2 {
    fragment.curve.derivative(BezierParameter {
        numerator: 1,
        denominator: 1,
    })
}

fn homogeneous_endpoint_tangent(
    from: &crate::bezier_arrangement::HomogeneousPoint2,
    to: &crate::bezier_arrangement::HomogeneousPoint2,
) -> Point2 {
    Point2::new(
        from.w.clone() * to.x.clone() - to.w.clone() * from.x.clone(),
        from.w.clone() * to.y.clone() - to.w.clone() * from.y.clone(),
    )
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

fn vector_is_zero(
    vector: &Point2,
    policy: PredicatePolicy,
) -> Result<bool, LineMixedBezierArrangementError> {
    Ok(real_equal(&vector.x, &Real::zero(), policy)?
        && real_equal(&vector.y, &Real::zero(), policy)?)
}

fn real_equal(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> Result<bool, LineMixedBezierArrangementError> {
    match compare_reals_with_policy(left, right, policy).value() {
        Some(Ordering::Equal) => Ok(true),
        Some(Ordering::Less | Ordering::Greater) => Ok(false),
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
