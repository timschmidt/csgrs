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

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy, point2_equal};
use hyperreal::{Rational, Real, RealExactSetFacts};
use hypersolve::{
    AlgebraicRootPolynomialImageReport, AlgebraicRootPolynomialImageStatus,
    AlgebraicRootRepresentation, IsolatedRootRefinementStatus, RootIsolationConfig,
    refine_isolated_univariate_polynomial_interval, transform_algebraic_root_polynomial_image,
};

use crate::bezier::CubicBezier;
use crate::bezier_arrangement::{
    LineCubicAlgebraicPointDomain, LineCubicAlgebraicRootDomain,
    LineCubicBezierAlgebraicInverseRoot, LineCubicBezierAlgebraicPointImage,
    LineCubicBezierAlgebraicSupportRoot, LineCubicBezierIntersection,
    LineCubicBezierIntersectionClass, LineCubicBezierIntersectionReport,
    LineCubicBezierInverseBoundarySource, LineCubicBezierSupportOverlap,
    intersect_line_cubic_bezier,
};
use crate::curve_cell::{
    CurveArrangementCellError, CurveArrangementCellGraph, build_line_cubic_cell_graph,
};
use crate::provenance::PathProvenance;
use crate::segment::{Axis, LinePathSegment};

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
    /// Exact tangent ordering of incident mixed line/cubic fragments was undecidable.
    UndecidableCellOrder { vertex: usize },
    /// Exact polynomial Green-integral area replay was unavailable for a retained cell edge.
    UndecidableCellArea { edge: usize },
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

/// Retained same-support line/cubic overlap candidate.
///
/// These candidates are copied from the predicate report whenever a cubic
/// Bezier is certified to lie on an axis-aligned line support. They are
/// retained even when the event remains
/// [`LineCubicBezierIntersectionClass::Unknown`] because inverse-boundary
/// parameters are represented algebraic roots. The exact support, hodograph,
/// and inverse-root evidence stays available to later
/// cell-scheduling work instead of being replaced by sampled topology.
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierSupportOverlapCandidate {
    /// Line segment index.
    pub line: usize,
    /// Cubic Bezier index.
    pub curve: usize,
    /// Retained same-support overlap evidence.
    pub overlap: LineCubicBezierSupportOverlap,
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

/// Certified domain status for a retained algebraic cubic overlap-boundary candidate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicOverlapBreakpointDomain {
    /// Cubic parameter and line boundary source are certified inside the retained pair domains.
    InsideLineAndCurve,
    /// The retained cubic parameter is certified outside `[0, 1]`.
    OutsideCubic,
    /// Exact interval comparison did not decide.
    Unknown,
}

/// Retained algebraic breakpoint candidate for a line/cubic overlap boundary.
///
/// The predicate layer retains represented roots of `B_v(t) - value == 0`
/// for line-boundary values on a same-support cubic. This scheduler attaches
/// each represented root to the exact line endpoint that induced the boundary
/// value. The record is replay evidence, not concrete topology: it is kept
/// separate from [`CubicBezierRealBreakpoint`] until an algebraic
/// materialization pass can split cubic curves at represented parameters.
///
/// Exact algebraic objects remain explicit; the represented roots come from the
/// Sturm/Collins-Loos univariate root discipline used by `hypersolve`.
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierAlgebraicOverlapBreakpoint {
    /// Line segment index.
    pub line: usize,
    /// Cubic Bezier index.
    pub curve: usize,
    /// Line endpoint that supplied the retained boundary value.
    pub boundary_source: LineCubicBezierInverseBoundarySource,
    /// Exact varying-coordinate boundary value on the line support.
    pub boundary_value: Real,
    /// Exact point on the retained line support.
    pub point: Point2,
    /// Exact line parameter for the retained endpoint boundary (`0` or `1`).
    pub line_parameter: Real,
    /// Represented algebraic cubic parameter.
    pub cubic_parameter: AlgebraicRootRepresentation,
    /// Certified relation of the represented cubic parameter to `[0, 1]`.
    pub cubic_parameter_domain: LineCubicAlgebraicRootDomain,
    /// Certified relation of the candidate to both source domains.
    pub domain: LineCubicBezierAlgebraicOverlapBreakpointDomain,
}

/// Certified order relation between two retained cubic overlap-boundary roots.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicOverlapBreakpointOrderClass {
    /// The left retained boundary root is certified before the right root.
    Before,
    /// The retained roots are certified equal.
    Equal,
    /// The left retained boundary root is certified after the right root.
    After,
    /// The isolating intervals overlap or exact comparison did not decide.
    Unknown,
}

/// Pairwise ordering evidence for retained cubic overlap-boundary roots.
///
/// Same-support overlap roots carry two source parameters: the exact line
/// endpoint parameter (`0` or `1`) and the represented cubic inverse root.
/// This report records exact pairwise order on either source when both
/// candidates are certified in-domain. It deliberately does not mutate
/// [`LineCubicBezierArrangementReport::line_breakpoints`] or
/// [`LineCubicBezierArrangementReport::cubic_breakpoints`].
///
/// This is the Yap object/predicate boundary from "Towards Exact Geometric
/// Computation" (1997): the exact roots and their order certificates are kept
/// as replay evidence, while concrete topology waits for an algebraic split
/// materializer. Represented root comparisons follow the Sturm/Collins-Loos
/// isolating-interval model used by `hypersolve`.
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierAlgebraicOverlapBreakpointOrder {
    /// Index in [`LineCubicBezierArrangementReport::algebraic_overlap_breakpoints`].
    pub left: usize,
    /// Index in [`LineCubicBezierArrangementReport::algebraic_overlap_breakpoints`].
    pub right: usize,
    /// Same-cubic order, when both candidates came from the same cubic.
    pub cubic_order: Option<LineCubicBezierAlgebraicOverlapBreakpointOrderClass>,
    /// Same-line order, when both candidates came from the same line.
    pub line_order: Option<LineCubicBezierAlgebraicOverlapBreakpointOrderClass>,
}

/// Source parameter space for retained cubic overlap-boundary sequences.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicOverlapBreakpointSequenceSource {
    /// Breakpoints ordered by exact endpoint parameter on a retained line.
    Line(usize),
    /// Breakpoints ordered by represented source parameter on a retained cubic.
    Curve(usize),
}

/// Sequence readiness for retained cubic overlap-boundary roots.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicOverlapBreakpointSequenceClass {
    /// Every same-source pair had certified strict order, so `breakpoints` is sorted.
    Ordered,
    /// A pair was equal, missing, or undecidable; insertion order is retained.
    Ambiguous,
}

/// Exact blocker that prevents a retained cubic overlap-boundary sequence from being sorted.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicOverlapBreakpointSequenceBlocker {
    /// Same-source order evidence was not emitted for this pair.
    MissingOrder { left: usize, right: usize },
    /// Same-source represented intervals overlap or comparison did not decide.
    UnknownOrder { left: usize, right: usize },
    /// Distinct retained candidates have the same source parameter.
    EqualOrder { left: usize, right: usize },
}

/// Ordered retained cubic overlap-boundary breakpoint indices for one source.
///
/// The indices address
/// [`LineCubicBezierArrangementReport::algebraic_overlap_breakpoints`].
/// Only candidates certified
/// [`LineCubicBezierAlgebraicOverlapBreakpointDomain::InsideLineAndCurve`]
/// can produce an ordered sequence. Outside and unknown roots remain retained
/// on the report as exact evidence, but are omitted from readiness sequences
/// and never become span boundaries.
///
/// Exact objects are not discarded, but topological readiness is stated separately.
/// The represented cubic roots use Collins and Loos, "Real Zeros of
/// Polynomials" (1982), via `hypersolve` isolating intervals.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LineCubicBezierAlgebraicOverlapBreakpointSequence {
    /// Source whose parameter orders this sequence.
    pub source: LineCubicBezierAlgebraicOverlapBreakpointSequenceSource,
    /// Breakpoint indices, sorted only when `class == Ordered`.
    pub breakpoints: Vec<usize>,
    /// Whether this source sequence is ready for exact algebraic split construction.
    pub class: LineCubicBezierAlgebraicOverlapBreakpointSequenceClass,
    /// Exact reasons that prevented sorting.
    pub blockers: Vec<LineCubicBezierAlgebraicOverlapBreakpointSequenceBlocker>,
}

/// Boundary of a retained cubic overlap-boundary source span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineCubicBezierAlgebraicOverlapSourceSpanBoundary {
    /// The exact source parameter `0`.
    SourceStart,
    /// An index in [`LineCubicBezierArrangementReport::algebraic_overlap_breakpoints`].
    Breakpoint(usize),
    /// The exact source parameter `1`.
    SourceEnd,
}

/// Conservative source-parameter interval between ordered cubic overlap boundaries.
///
/// Spans are emitted only for ordered in-domain overlap-boundary sequences.
/// They are not line or cubic fragments; they are retained interval hulls that
/// later algebraic split construction can replay. Line spans use exact
/// endpoint parameters, and cubic spans use the retained Sturm isolating
/// intervals of the inverse-boundary roots.
///
/// This is the same retained-object discipline advocated by Yap, "Towards
/// Exact Geometric Computation" (1997): exact algebraic candidates advance
/// scheduling without being converted to sampled breakpoints. The root
/// intervals follow Collins and Loos, "Real Zeros of Polynomials" (1982).
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierAlgebraicOverlapSourceSpan {
    /// Source whose parameter space owns this span.
    pub source: LineCubicBezierAlgebraicOverlapBreakpointSequenceSource,
    /// Left adjacent boundary.
    pub left: LineCubicBezierAlgebraicOverlapSourceSpanBoundary,
    /// Right adjacent boundary.
    pub right: LineCubicBezierAlgebraicOverlapSourceSpanBoundary,
    /// Conservative lower source parameter bound.
    pub parameter_lower: Real,
    /// Conservative upper source parameter bound.
    pub parameter_upper: Real,
}

/// Conservative coordinate envelope for a cubic overlap source span.
///
/// The envelope is indexed by
/// [`LineCubicBezierArrangementReport::algebraic_overlap_source_spans`]. It
/// encloses the two span endpoints, exact source endpoints and retained
/// same-support inverse-boundary points, and certified interior coordinate
/// extrema for curve-owned spans. It is still a retained replay box rather
/// than a materialized algebraic subcurve: extrema are admitted only when exact
/// comparison proves that the derivative root lies inside the retained source
/// interval.
///
/// Represented roots and exact endpoint witnesses remain first-class retained objects
/// until a later construction pass can materialize algebraic subcurves. The
/// root intervals that make the source spans valid follow Collins and Loos,
/// "Real Zeros of Polynomials" (1982); the cubic extrema use the derivative
/// characterization of polynomial Bezier coordinates described by Farouki,
/// "The Bernstein Polynomial Basis: A Centennial Retrospective" (2012).
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierAlgebraicOverlapEndpointEnvelope {
    /// Index in [`LineCubicBezierArrangementReport::algebraic_overlap_source_spans`].
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

/// Exact native cubic breakpoint promoted from a retained overlap-boundary root.
///
/// Same-support line/cubic overlap boundaries are usually retained as
/// represented algebraic inverse roots. When the isolating interval carries an
/// exact rational witness, however, the mixed scheduler can replay that root
/// as an ordinary cubic `Real` split parameter while keeping the represented
/// source candidate available for audit.
///
/// This is the exact-construction boundary advocated by Yap, "Towards Exact
/// Geometric Computation" (1997): a represented object is materialized only
/// when the predicate layer has already certified an exact rational value and
/// domain membership. The root witness follows the Collins-Loos real-root
/// isolation model used by `hypersolve`; the emitted sub-curves are still
/// cubic Bezier restrictions built by de Casteljau subdivision, as described
/// in Farouki, *Pythagorean Hodograph Curves* (2008).
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierExactAlgebraicOverlapBreakpointPromotion {
    /// Index in [`LineCubicBezierArrangementReport::algebraic_overlap_breakpoints`].
    pub algebraic_overlap_breakpoint: usize,
    /// Cubic Bezier index.
    pub curve: usize,
    /// Exact promoted cubic source parameter.
    pub parameter: Real,
    /// Exact point attached to the retained overlap boundary.
    pub point: Point2,
}

/// Exact native line/cubic breakpoints promoted from a true cubic support root.
///
/// A retained line/cubic support root may be represented by a cubic polynomial
/// even when its isolator contains an exact rational witness. In that case the
/// mixed scheduler can replay the root as ordinary `Real` line and cubic split
/// parameters, while still retaining the original algebraic source candidate
/// for audit and for downstream exact cell construction.
///
/// This is a deliberately narrow materialization step: exact construction
/// happens only after the predicate layer supplies a valid rational root witness,
/// exact resultant point/line images, and certified domain membership. The
/// resultant images cite Sylvester (1853) and Collins and Loos, "Real Zeros
/// of Polynomials" (1982); the native cubic fragments remain de Casteljau
/// restrictions of the original Bezier curve, as in Farouki,
/// *Pythagorean Hodograph Curves* (2008).
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierExactAlgebraicBreakpointPromotion {
    /// Index in [`LineCubicBezierArrangementReport::algebraic_breakpoints`].
    pub algebraic_breakpoint: usize,
    /// Line segment index.
    pub line: usize,
    /// Cubic Bezier index.
    pub curve: usize,
    /// Exact promoted cubic source parameter.
    pub cubic_parameter: Real,
    /// Exact promoted normalized line parameter.
    pub line_parameter: Real,
    /// Exact point shared by the line and cubic at the promoted root.
    pub point: Point2,
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
/// Exact algebraic order evidence is retained as a report, but it does not mutate
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
/// Exact algebraic decisions are retained as first-class certificates and uncertain
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

/// Conservative coordinate envelope for an algebraic source span.
///
/// The envelope is indexed by
/// [`LineCubicBezierArrangementReport::algebraic_source_spans`]. It encloses
/// exact source endpoints, retained algebraic breakpoint point images, and any
/// certified cubic coordinate extrema whose exact derivative roots lie inside
/// a curve-owned retained span. It is still not a sampled approximation and
/// still does not materialize a nonlinear algebraic subcurve: extrema are
/// included only when exact root membership in the retained interval is
/// decidable.
///
/// Predicates and constructed objects remain separate, and unsupported construction keeps
/// exact certificates instead of floating approximations. The algebraic point
/// images use the Sylvester resultant construction of Sylvester (1853) and the
/// Sturm/Collins-Loos isolating-interval model from Collins and Loos, "Real
/// Zeros of Polynomials" (1982). The interior extrema replay exact polynomial
/// Bezier derivative roots, using the Bernstein/polynomial curve treatment
/// described by Farouki, *Pythagorean Hodograph Curves* (2008).
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierAlgebraicEndpointEnvelope {
    /// Index in [`LineCubicBezierArrangementReport::algebraic_source_spans`].
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

/// Retained mixed line/cubic-Bezier arrangement schedule and cell graph.
///
/// Certified events are replayed into sorted split parameters before fragments
/// are emitted. Unknown relations do not add breakpoints. Cubic fragments are
/// reconstructed from exact endpoint and derivative data on each retained
/// subinterval. Exact object construction and exact
/// predicate replay are separated, and unsupported roots remain report states.
/// The cubic restriction formula is de Casteljau's affine subdivision written
/// in endpoint/derivative form; see Farouki, *Pythagorean Hodograph Curves*
/// (2008), for the same retained polynomial-curve discipline. Certified
/// native line/cubic fragments also feed a retained curve cell graph whose
/// half-edges are ordered by exact endpoint tangents and whose nonzero face
/// walks replay exact polynomial Green-integral area; represented algebraic
/// roots stay in the retained evidence fields until exact materialization is
/// available.
#[derive(Clone, Debug, PartialEq)]
pub struct LineCubicBezierArrangementReport {
    /// Retained input line segments.
    pub lines: Vec<LinePathSegment>,
    /// Retained input cubic Beziers.
    pub curves: Vec<CubicBezier>,
    /// Certified or unknown pairwise events.
    pub events: Vec<LineCubicBezierArrangementEvent>,
    /// Retained same-support cubic overlap candidates.
    pub support_overlaps: Vec<LineCubicBezierSupportOverlapCandidate>,
    /// Algebraic breakpoint candidates retained from true cubic support roots.
    pub algebraic_breakpoints: Vec<LineCubicBezierAlgebraicBreakpoint>,
    /// Algebraic cubic breakpoint candidates retained from same-support overlap boundaries.
    pub algebraic_overlap_breakpoints: Vec<LineCubicBezierAlgebraicOverlapBreakpoint>,
    /// Pairwise exact order evidence for retained algebraic cubic overlap boundaries.
    pub algebraic_overlap_breakpoint_orders: Vec<LineCubicBezierAlgebraicOverlapBreakpointOrder>,
    /// Per-source retained cubic overlap-boundary sequences derived from exact order evidence.
    pub algebraic_overlap_breakpoint_sequences:
        Vec<LineCubicBezierAlgebraicOverlapBreakpointSequence>,
    /// Conservative source spans induced by certified cubic overlap-boundary sequences.
    pub algebraic_overlap_source_spans: Vec<LineCubicBezierAlgebraicOverlapSourceSpan>,
    /// Conservative endpoint coordinate envelopes for retained overlap source spans.
    pub algebraic_overlap_endpoint_envelopes: Vec<LineCubicBezierAlgebraicOverlapEndpointEnvelope>,
    /// Exact rational overlap-boundary roots promoted into native cubic split parameters.
    pub exact_algebraic_overlap_breakpoint_promotions:
        Vec<LineCubicBezierExactAlgebraicOverlapBreakpointPromotion>,
    /// Exact rational true-cubic support roots promoted into native split parameters.
    pub exact_algebraic_breakpoint_promotions:
        Vec<LineCubicBezierExactAlgebraicBreakpointPromotion>,
    /// Pairwise exact order evidence for retained algebraic breakpoints.
    pub algebraic_breakpoint_orders: Vec<LineCubicBezierAlgebraicBreakpointOrder>,
    /// Per-source retained algebraic breakpoint sequences derived from exact order evidence.
    pub algebraic_breakpoint_sequences: Vec<LineCubicBezierAlgebraicBreakpointSequence>,
    /// Conservative source spans induced by certified algebraic breakpoint sequences.
    pub algebraic_source_spans: Vec<LineCubicBezierAlgebraicSourceSpan>,
    /// Conservative endpoint coordinate envelopes for retained algebraic source spans.
    pub algebraic_endpoint_envelopes: Vec<LineCubicBezierAlgebraicEndpointEnvelope>,
    /// Sorted line breakpoints induced by line endpoints and certified events.
    pub line_breakpoints: Vec<Vec<MixedCubicLineArrangementBreakpoint>>,
    /// Sorted cubic breakpoints induced by curve endpoints and certified events.
    pub cubic_breakpoints: Vec<Vec<CubicBezierRealBreakpoint>>,
    /// Positive-length line fragments.
    pub line_fragments: Vec<MixedCubicLineArrangementFragment>,
    /// Positive-length cubic Bezier fragments.
    pub cubic_fragments: Vec<CubicBezierRealFragment>,
    /// Exact retained curve cell graph induced by native line and cubic fragments.
    pub cell_graph: CurveArrangementCellGraph,
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
    let mut support_overlaps = Vec::new();
    let mut algebraic_breakpoints = Vec::new();
    let mut algebraic_overlap_breakpoints = Vec::new();

    for (line_index, line) in lines.iter().enumerate() {
        for (curve_index, curve) in curves.iter().enumerate() {
            let intersection = intersect_line_cubic_bezier(line, curve, policy);
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
            if let Some(overlap) = &intersection.support_overlap {
                algebraic_overlap_breakpoints.extend(retained_algebraic_cubic_overlap_breakpoints(
                    line_index,
                    curve_index,
                    overlap,
                ));
                support_overlaps.push(LineCubicBezierSupportOverlapCandidate {
                    line: line_index,
                    curve: curve_index,
                    overlap: overlap.clone(),
                });
            }
            events.push(LineCubicBezierArrangementEvent {
                line: line_index,
                curve: curve_index,
                class: intersection.class,
                intersection,
            });
        }
    }

    let exact_algebraic_overlap_breakpoint_promotions =
        promote_exact_algebraic_cubic_overlap_breakpoints(
            &mut cubic_breakpoints,
            &algebraic_overlap_breakpoints,
            policy,
        )?;
    let exact_algebraic_breakpoint_promotions = promote_exact_algebraic_cubic_breakpoints(
        &mut line_breakpoints,
        &mut cubic_breakpoints,
        lines,
        curves,
        &algebraic_breakpoints,
        policy,
    )?;
    sort_and_dedup_line_breakpoints(&mut line_breakpoints, policy)?;
    sort_and_dedup_cubic_breakpoints(&mut cubic_breakpoints, policy)?;
    let algebraic_breakpoint_orders =
        algebraic_cubic_breakpoint_orders(&algebraic_breakpoints, policy);
    let algebraic_breakpoint_sequences =
        algebraic_cubic_breakpoint_sequences(&algebraic_breakpoints, &algebraic_breakpoint_orders);
    let algebraic_source_spans =
        algebraic_cubic_source_spans(&algebraic_breakpoints, &algebraic_breakpoint_sequences);
    let algebraic_overlap_breakpoint_orders =
        algebraic_cubic_overlap_breakpoint_orders(&algebraic_overlap_breakpoints, policy);
    let algebraic_overlap_breakpoint_sequences = algebraic_cubic_overlap_breakpoint_sequences(
        &algebraic_overlap_breakpoints,
        &algebraic_overlap_breakpoint_orders,
    );
    let algebraic_overlap_source_spans = algebraic_cubic_overlap_source_spans(
        &algebraic_overlap_breakpoints,
        &algebraic_overlap_breakpoint_sequences,
    );
    let algebraic_overlap_endpoint_envelopes = algebraic_cubic_overlap_endpoint_envelopes(
        lines,
        curves,
        &algebraic_overlap_breakpoints,
        &algebraic_overlap_source_spans,
        policy,
    );
    let algebraic_endpoint_envelopes = algebraic_cubic_endpoint_envelopes(
        lines,
        curves,
        &algebraic_breakpoints,
        &algebraic_source_spans,
        policy,
    );
    let line_fragments = build_line_fragments(&line_breakpoints, policy)?;
    let cubic_fragments = build_cubic_fragments(&cubic_breakpoints, curves, policy)?;
    let cell_graph = build_line_cubic_cell_graph(&line_fragments, &cubic_fragments, policy)
        .map_err(line_cubic_error_from_curve_cell_error)?;
    let facts = LineCubicBezierArrangementFacts {
        input_exact: input_exact_facts(lines, curves),
        fragment_exact: fragment_exact_facts(&line_fragments, &cubic_fragments),
        provenance,
    };

    Ok(LineCubicBezierArrangementReport {
        lines: lines.to_vec(),
        curves: curves.to_vec(),
        events,
        support_overlaps,
        algebraic_breakpoints,
        algebraic_overlap_breakpoints,
        algebraic_overlap_breakpoint_orders,
        algebraic_overlap_breakpoint_sequences,
        algebraic_overlap_source_spans,
        algebraic_overlap_endpoint_envelopes,
        exact_algebraic_overlap_breakpoint_promotions,
        exact_algebraic_breakpoint_promotions,
        algebraic_breakpoint_orders,
        algebraic_breakpoint_sequences,
        algebraic_source_spans,
        algebraic_endpoint_envelopes,
        line_breakpoints,
        cubic_breakpoints,
        line_fragments,
        cubic_fragments,
        cell_graph,
        facts,
    })
}

fn line_cubic_error_from_curve_cell_error(
    error: CurveArrangementCellError,
) -> LineCubicBezierArrangementError {
    match error {
        CurveArrangementCellError::UndecidablePointEquality => {
            LineCubicBezierArrangementError::UndecidablePointEquality
        }
        CurveArrangementCellError::UndecidableCellOrder { vertex } => {
            LineCubicBezierArrangementError::UndecidableCellOrder { vertex }
        }
        CurveArrangementCellError::UndecidableCellArea { edge } => {
            LineCubicBezierArrangementError::UndecidableCellArea { edge }
        }
    }
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

fn retained_algebraic_cubic_overlap_breakpoints(
    line_index: usize,
    curve_index: usize,
    overlap: &LineCubicBezierSupportOverlap,
) -> Vec<LineCubicBezierAlgebraicOverlapBreakpoint> {
    let mut retained = Vec::new();
    for boundary in &overlap.inverse_boundary_roots {
        let point = point_from_axis(overlap.axis, overlap.fixed.clone(), boundary.value.clone());
        let line_parameter = match boundary.source {
            LineCubicBezierInverseBoundarySource::SegmentStart => Real::zero(),
            LineCubicBezierInverseBoundarySource::SegmentEnd => Real::one(),
        };
        for root in &boundary.roots {
            retained.push(LineCubicBezierAlgebraicOverlapBreakpoint {
                line: line_index,
                curve: curve_index,
                boundary_source: boundary.source,
                boundary_value: boundary.value.clone(),
                point: point.clone(),
                line_parameter: line_parameter.clone(),
                cubic_parameter: root.parameter.clone(),
                cubic_parameter_domain: root.parameter_domain,
                domain: classify_algebraic_cubic_overlap_breakpoint_domain(root),
            });
        }
    }
    retained
}

fn classify_algebraic_cubic_overlap_breakpoint_domain(
    root: &LineCubicBezierAlgebraicInverseRoot,
) -> LineCubicBezierAlgebraicOverlapBreakpointDomain {
    match root.parameter_domain {
        LineCubicAlgebraicRootDomain::InsideUnitInterval => {
            LineCubicBezierAlgebraicOverlapBreakpointDomain::InsideLineAndCurve
        }
        LineCubicAlgebraicRootDomain::OutsideUnitInterval => {
            LineCubicBezierAlgebraicOverlapBreakpointDomain::OutsideCubic
        }
        LineCubicAlgebraicRootDomain::Unknown => {
            LineCubicBezierAlgebraicOverlapBreakpointDomain::Unknown
        }
    }
}

fn point_from_axis(axis: Axis, fixed: Real, varying: Real) -> Point2 {
    match axis {
        Axis::X => Point2::new(varying, fixed),
        Axis::Y => Point2::new(fixed, varying),
    }
}

fn promote_exact_algebraic_cubic_overlap_breakpoints(
    cubic_breakpoints: &mut [Vec<CubicBezierRealBreakpoint>],
    algebraic_overlap_breakpoints: &[LineCubicBezierAlgebraicOverlapBreakpoint],
    policy: PredicatePolicy,
) -> Result<
    Vec<LineCubicBezierExactAlgebraicOverlapBreakpointPromotion>,
    LineCubicBezierArrangementError,
> {
    let mut promotions = Vec::new();
    for (index, breakpoint) in algebraic_overlap_breakpoints.iter().enumerate() {
        if breakpoint.domain != LineCubicBezierAlgebraicOverlapBreakpointDomain::InsideLineAndCurve
        {
            continue;
        }
        let Some(parameter) = breakpoint.cubic_parameter.interval.exact_root.clone() else {
            continue;
        };
        insert_exact_cubic_breakpoint(
            &mut cubic_breakpoints[breakpoint.curve],
            breakpoint.curve,
            parameter.clone(),
            breakpoint.point.clone(),
            policy,
        )?;
        promotions.push(LineCubicBezierExactAlgebraicOverlapBreakpointPromotion {
            algebraic_overlap_breakpoint: index,
            curve: breakpoint.curve,
            parameter,
            point: breakpoint.point.clone(),
        });
    }
    Ok(promotions)
}

fn promote_exact_algebraic_cubic_breakpoints(
    line_breakpoints: &mut [Vec<MixedCubicLineArrangementBreakpoint>],
    cubic_breakpoints: &mut [Vec<CubicBezierRealBreakpoint>],
    lines: &[LinePathSegment],
    curves: &[CubicBezier],
    algebraic_breakpoints: &[LineCubicBezierAlgebraicBreakpoint],
    policy: PredicatePolicy,
) -> Result<Vec<LineCubicBezierExactAlgebraicBreakpointPromotion>, LineCubicBezierArrangementError>
{
    let mut promotions = Vec::new();
    for (index, breakpoint) in algebraic_breakpoints.iter().enumerate() {
        if breakpoint.domain != LineCubicBezierAlgebraicBreakpointDomain::InsideLineAndCurve {
            continue;
        }
        let Some(cubic_parameter) =
            exact_or_refined_cubic_root(&breakpoint.cubic_parameter, policy)
        else {
            continue;
        };
        let Some(line_parameter) = exact_cubic_line_parameter(
            &lines[breakpoint.line],
            &curves[breakpoint.curve],
            &cubic_parameter,
        ) else {
            continue;
        };
        if !exact_value_inside_transformed_image(
            &breakpoint.line_parameter,
            &line_parameter,
            policy,
        ) {
            continue;
        }
        let point = eval_cubic_real(&curves[breakpoint.curve], &cubic_parameter);
        if !exact_point_inside_algebraic_image(&breakpoint.point_image, &point, policy) {
            continue;
        }

        insert_line_breakpoint(
            &mut line_breakpoints[breakpoint.line],
            breakpoint.line,
            &lines[breakpoint.line],
            point.clone(),
            policy,
        )?;
        insert_exact_cubic_breakpoint(
            &mut cubic_breakpoints[breakpoint.curve],
            breakpoint.curve,
            cubic_parameter.clone(),
            point.clone(),
            policy,
        )?;
        promotions.push(LineCubicBezierExactAlgebraicBreakpointPromotion {
            algebraic_breakpoint: index,
            line: breakpoint.line,
            curve: breakpoint.curve,
            cubic_parameter,
            line_parameter,
            point,
        });
    }
    Ok(promotions)
}

fn exact_or_refined_cubic_root(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<Real> {
    if let Some(exact) = &root.interval.exact_root {
        return Some(exact.clone());
    }
    if let Some(witness) = exact_rational_grid_witness(root, policy) {
        return Some(witness);
    }
    let refinement = refine_isolated_univariate_polynomial_interval(
        &root.polynomial_coefficients,
        &root.interval,
        RootIsolationConfig {
            policy,
            max_interval_width: None,
            max_refinement_steps: 256,
        },
    );
    if refinement.status == IsolatedRootRefinementStatus::ExactRoot {
        return refinement
            .refined_interval
            .and_then(|interval| interval.exact_root);
    }
    None
}

fn exact_rational_grid_witness(
    root: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> Option<Real> {
    // A retained Sturm interval may be too narrow to hit a rational root by
    // bisection if the interval endpoints are not aligned with that rational.
    // For the native materialization boundary we therefore replay small exact
    // rational candidates against the support polynomial and the isolating
    // interval. This is still Yap-style exact construction: no approximation
    // is admitted, and every accepted candidate is an exact polynomial root
    // inside the existing Collins-Loos isolator.
    for denominator in 1_u64..=64 {
        for numerator in 0_i64..=(denominator as i64) {
            let candidate = Real::new(Rational::fraction(numerator, denominator).ok()?);
            if !exact_value_inside_interval(
                &candidate,
                &root.interval.lower,
                &root.interval.upper,
                policy,
            ) {
                continue;
            }
            if compare_reals_with_policy(
                &evaluate_real_polynomial(&root.polynomial_coefficients, &candidate),
                &Real::zero(),
                policy,
            )
            .value()
                == Some(Ordering::Equal)
            {
                return Some(candidate);
            }
        }
    }
    None
}

fn exact_cubic_line_parameter(
    line: &LinePathSegment,
    curve: &CubicBezier,
    parameter: &Real,
) -> Option<Real> {
    let coefficients = cubic_line_parameter_polynomial(line, curve)?;
    Some(evaluate_real_polynomial(&coefficients, parameter))
}

fn evaluate_real_polynomial(coefficients: &[Real], value: &Real) -> Real {
    coefficients
        .iter()
        .rev()
        .fold(Real::zero(), |accumulator, coefficient| {
            accumulator * value.clone() + coefficient.clone()
        })
}

fn exact_point_inside_algebraic_image(
    point_image: &LineCubicBezierAlgebraicPointImage,
    point: &Point2,
    policy: PredicatePolicy,
) -> bool {
    exact_value_inside_transformed_image(&point_image.x, &point.x, policy)
        && exact_value_inside_transformed_image(&point_image.y, &point.y, policy)
}

fn exact_value_inside_transformed_image(
    image: &AlgebraicRootPolynomialImageReport,
    value: &Real,
    policy: PredicatePolicy,
) -> bool {
    let Some(representation) = transformed_image_representation(image) else {
        return false;
    };
    exact_value_inside_interval(
        value,
        &representation.interval.lower,
        &representation.interval.upper,
        policy,
    )
}

fn exact_value_inside_interval(
    value: &Real,
    lower_bound: &Real,
    upper_bound: &Real,
    policy: PredicatePolicy,
) -> bool {
    let lower = compare_reals_with_policy(value, lower_bound, policy).value();
    let upper = compare_reals_with_policy(value, upper_bound, policy).value();
    matches!(lower, Some(Ordering::Equal | Ordering::Greater))
        && matches!(upper, Some(Ordering::Equal | Ordering::Less))
}

fn algebraic_cubic_overlap_breakpoint_orders(
    breakpoints: &[LineCubicBezierAlgebraicOverlapBreakpoint],
    policy: PredicatePolicy,
) -> Vec<LineCubicBezierAlgebraicOverlapBreakpointOrder> {
    let mut orders = Vec::new();
    for left in 0..breakpoints.len() {
        for right in (left + 1)..breakpoints.len() {
            if !algebraic_cubic_overlap_breakpoint_is_in_domain(&breakpoints[left])
                || !algebraic_cubic_overlap_breakpoint_is_in_domain(&breakpoints[right])
            {
                continue;
            }
            let cubic_order = (breakpoints[left].curve == breakpoints[right].curve).then(|| {
                compare_algebraic_cubic_overlap_parameters(
                    &breakpoints[left].cubic_parameter,
                    &breakpoints[right].cubic_parameter,
                    policy,
                )
            });
            let line_order = (breakpoints[left].line == breakpoints[right].line).then(|| {
                compare_exact_cubic_overlap_line_parameters(
                    &breakpoints[left].line_parameter,
                    &breakpoints[right].line_parameter,
                    policy,
                )
            });
            if cubic_order.is_some() || line_order.is_some() {
                orders.push(LineCubicBezierAlgebraicOverlapBreakpointOrder {
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

fn algebraic_cubic_overlap_breakpoint_is_in_domain(
    breakpoint: &LineCubicBezierAlgebraicOverlapBreakpoint,
) -> bool {
    breakpoint.domain == LineCubicBezierAlgebraicOverlapBreakpointDomain::InsideLineAndCurve
}

fn algebraic_cubic_overlap_breakpoint_sequences(
    breakpoints: &[LineCubicBezierAlgebraicOverlapBreakpoint],
    orders: &[LineCubicBezierAlgebraicOverlapBreakpointOrder],
) -> Vec<LineCubicBezierAlgebraicOverlapBreakpointSequence> {
    let mut curve_breakpoints: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    let mut line_breakpoints: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (index, breakpoint) in breakpoints.iter().enumerate() {
        if !algebraic_cubic_overlap_breakpoint_is_in_domain(breakpoint) {
            continue;
        }
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
        sequences.push(algebraic_cubic_overlap_breakpoint_sequence_for_source(
            LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Curve(curve),
            indices,
            orders,
        ));
    }
    for (line, indices) in line_breakpoints {
        sequences.push(algebraic_cubic_overlap_breakpoint_sequence_for_source(
            LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Line(line),
            indices,
            orders,
        ));
    }
    sequences
}

fn algebraic_cubic_overlap_breakpoint_sequence_for_source(
    source: LineCubicBezierAlgebraicOverlapBreakpointSequenceSource,
    mut indices: Vec<usize>,
    orders: &[LineCubicBezierAlgebraicOverlapBreakpointOrder],
) -> LineCubicBezierAlgebraicOverlapBreakpointSequence {
    let mut blockers = Vec::new();
    for left_index in 0..indices.len() {
        for right_index in (left_index + 1)..indices.len() {
            let left = indices[left_index];
            let right = indices[right_index];
            match algebraic_cubic_overlap_order_between(source, left, right, orders) {
                Some(LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Before)
                | Some(LineCubicBezierAlgebraicOverlapBreakpointOrderClass::After) => {}
                Some(LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Equal) => {
                    blockers.push(
                        LineCubicBezierAlgebraicOverlapBreakpointSequenceBlocker::EqualOrder {
                            left,
                            right,
                        },
                    );
                }
                Some(LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Unknown) => {
                    blockers.push(
                        LineCubicBezierAlgebraicOverlapBreakpointSequenceBlocker::UnknownOrder {
                            left,
                            right,
                        },
                    );
                }
                None => {
                    blockers.push(
                        LineCubicBezierAlgebraicOverlapBreakpointSequenceBlocker::MissingOrder {
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
            algebraic_cubic_overlap_ordering_for_sort(source, *left, *right, orders)
                .expect("algebraic overlap source order was certified before sorting")
        });
        LineCubicBezierAlgebraicOverlapBreakpointSequenceClass::Ordered
    } else {
        LineCubicBezierAlgebraicOverlapBreakpointSequenceClass::Ambiguous
    };

    LineCubicBezierAlgebraicOverlapBreakpointSequence {
        source,
        breakpoints: indices,
        class,
        blockers,
    }
}

fn algebraic_cubic_overlap_ordering_for_sort(
    source: LineCubicBezierAlgebraicOverlapBreakpointSequenceSource,
    left: usize,
    right: usize,
    orders: &[LineCubicBezierAlgebraicOverlapBreakpointOrder],
) -> Option<Ordering> {
    if left == right {
        return Some(Ordering::Equal);
    }
    match algebraic_cubic_overlap_order_between(source, left, right, orders)? {
        LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Before => Some(Ordering::Less),
        LineCubicBezierAlgebraicOverlapBreakpointOrderClass::After => Some(Ordering::Greater),
        LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Equal => Some(Ordering::Equal),
        LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Unknown => None,
    }
}

fn algebraic_cubic_overlap_order_between(
    source: LineCubicBezierAlgebraicOverlapBreakpointSequenceSource,
    left: usize,
    right: usize,
    orders: &[LineCubicBezierAlgebraicOverlapBreakpointOrder],
) -> Option<LineCubicBezierAlgebraicOverlapBreakpointOrderClass> {
    let direct = orders
        .iter()
        .find(|order| order.left == left && order.right == right)
        .and_then(|order| algebraic_cubic_overlap_order_for_source(source, order));
    if direct.is_some() {
        return direct;
    }
    orders
        .iter()
        .find(|order| order.left == right && order.right == left)
        .and_then(|order| algebraic_cubic_overlap_order_for_source(source, order))
        .map(reverse_algebraic_cubic_overlap_order)
}

fn algebraic_cubic_overlap_order_for_source(
    source: LineCubicBezierAlgebraicOverlapBreakpointSequenceSource,
    order: &LineCubicBezierAlgebraicOverlapBreakpointOrder,
) -> Option<LineCubicBezierAlgebraicOverlapBreakpointOrderClass> {
    match source {
        LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Line(_) => order.line_order,
        LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Curve(_) => order.cubic_order,
    }
}

fn reverse_algebraic_cubic_overlap_order(
    order: LineCubicBezierAlgebraicOverlapBreakpointOrderClass,
) -> LineCubicBezierAlgebraicOverlapBreakpointOrderClass {
    match order {
        LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Before => {
            LineCubicBezierAlgebraicOverlapBreakpointOrderClass::After
        }
        LineCubicBezierAlgebraicOverlapBreakpointOrderClass::After => {
            LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Before
        }
        LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Equal => {
            LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Equal
        }
        LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Unknown => {
            LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Unknown
        }
    }
}

fn algebraic_cubic_overlap_source_spans(
    breakpoints: &[LineCubicBezierAlgebraicOverlapBreakpoint],
    sequences: &[LineCubicBezierAlgebraicOverlapBreakpointSequence],
) -> Vec<LineCubicBezierAlgebraicOverlapSourceSpan> {
    let mut spans = Vec::new();
    for sequence in sequences {
        if sequence.class != LineCubicBezierAlgebraicOverlapBreakpointSequenceClass::Ordered {
            continue;
        }
        let mut boundaries = Vec::with_capacity(sequence.breakpoints.len() + 2);
        boundaries.push(LineCubicBezierAlgebraicOverlapSourceSpanBoundary::SourceStart);
        boundaries.extend(
            sequence
                .breakpoints
                .iter()
                .copied()
                .map(LineCubicBezierAlgebraicOverlapSourceSpanBoundary::Breakpoint),
        );
        boundaries.push(LineCubicBezierAlgebraicOverlapSourceSpanBoundary::SourceEnd);

        for pair in boundaries.windows(2) {
            let Some((parameter_lower, _)) =
                algebraic_cubic_overlap_boundary_interval(sequence.source, pair[0], breakpoints)
            else {
                continue;
            };
            let Some((_, parameter_upper)) =
                algebraic_cubic_overlap_boundary_interval(sequence.source, pair[1], breakpoints)
            else {
                continue;
            };
            spans.push(LineCubicBezierAlgebraicOverlapSourceSpan {
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

fn algebraic_cubic_overlap_boundary_interval(
    source: LineCubicBezierAlgebraicOverlapBreakpointSequenceSource,
    boundary: LineCubicBezierAlgebraicOverlapSourceSpanBoundary,
    breakpoints: &[LineCubicBezierAlgebraicOverlapBreakpoint],
) -> Option<(Real, Real)> {
    match boundary {
        LineCubicBezierAlgebraicOverlapSourceSpanBoundary::SourceStart => {
            Some((Real::zero(), Real::zero()))
        }
        LineCubicBezierAlgebraicOverlapSourceSpanBoundary::SourceEnd => {
            Some((Real::one(), Real::one()))
        }
        LineCubicBezierAlgebraicOverlapSourceSpanBoundary::Breakpoint(index) => match source {
            LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Curve(_) => {
                let interval = &breakpoints.get(index)?.cubic_parameter.interval;
                Some((interval.lower.clone(), interval.upper.clone()))
            }
            LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Line(_) => {
                let parameter = &breakpoints.get(index)?.line_parameter;
                Some((parameter.clone(), parameter.clone()))
            }
        },
    }
}

fn algebraic_cubic_overlap_endpoint_envelopes(
    lines: &[LinePathSegment],
    curves: &[CubicBezier],
    breakpoints: &[LineCubicBezierAlgebraicOverlapBreakpoint],
    spans: &[LineCubicBezierAlgebraicOverlapSourceSpan],
    policy: PredicatePolicy,
) -> Vec<LineCubicBezierAlgebraicOverlapEndpointEnvelope> {
    spans
        .iter()
        .enumerate()
        .filter_map(|(span_index, span)| {
            let left = algebraic_cubic_overlap_boundary_point_interval(
                span.source,
                span.left,
                lines,
                curves,
                breakpoints,
            )?;
            let right = algebraic_cubic_overlap_boundary_point_interval(
                span.source,
                span.right,
                lines,
                curves,
                breakpoints,
            )?;
            let mut points = vec![left, right];
            if let LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Curve(curve_index) =
                span.source
            {
                points.extend(algebraic_cubic_interval_interior_extrema(
                    curves.get(curve_index)?,
                    &span.parameter_lower,
                    &span.parameter_upper,
                    policy,
                )?);
            }
            let (x_lower, x_upper, y_lower, y_upper) =
                certified_point_interval_bounds(&points, policy)?;
            Some(LineCubicBezierAlgebraicOverlapEndpointEnvelope {
                span: span_index,
                x_lower,
                x_upper,
                y_lower,
                y_upper,
            })
        })
        .collect()
}

fn algebraic_cubic_overlap_boundary_point_interval(
    source: LineCubicBezierAlgebraicOverlapBreakpointSequenceSource,
    boundary: LineCubicBezierAlgebraicOverlapSourceSpanBoundary,
    lines: &[LinePathSegment],
    curves: &[CubicBezier],
    breakpoints: &[LineCubicBezierAlgebraicOverlapBreakpoint],
) -> Option<CubicPointInterval> {
    match boundary {
        LineCubicBezierAlgebraicOverlapSourceSpanBoundary::SourceStart => match source {
            LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Line(line) => {
                point_exact_interval(lines.get(line)?.start())
            }
            LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Curve(curve) => {
                point_exact_interval(curves.get(curve)?.start())
            }
        },
        LineCubicBezierAlgebraicOverlapSourceSpanBoundary::SourceEnd => match source {
            LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Line(line) => {
                point_exact_interval(lines.get(line)?.end())
            }
            LineCubicBezierAlgebraicOverlapBreakpointSequenceSource::Curve(curve) => {
                point_exact_interval(curves.get(curve)?.end())
            }
        },
        LineCubicBezierAlgebraicOverlapSourceSpanBoundary::Breakpoint(index) => {
            point_exact_interval(&breakpoints.get(index)?.point)
        }
    }
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

fn algebraic_cubic_endpoint_envelopes(
    lines: &[LinePathSegment],
    curves: &[CubicBezier],
    breakpoints: &[LineCubicBezierAlgebraicBreakpoint],
    spans: &[LineCubicBezierAlgebraicSourceSpan],
    policy: PredicatePolicy,
) -> Vec<LineCubicBezierAlgebraicEndpointEnvelope> {
    spans
        .iter()
        .enumerate()
        .filter_map(|(span_index, span)| {
            let left = algebraic_cubic_boundary_point_interval(
                span.source,
                span.left,
                lines,
                curves,
                breakpoints,
            )?;
            let right = algebraic_cubic_boundary_point_interval(
                span.source,
                span.right,
                lines,
                curves,
                breakpoints,
            )?;
            let mut points = vec![left, right];
            if let LineCubicBezierAlgebraicBreakpointSequenceSource::Curve(curve_index) =
                span.source
            {
                points.extend(algebraic_cubic_span_interior_extrema(
                    curves.get(curve_index)?,
                    span,
                    policy,
                )?);
            }
            let (x_lower, x_upper, y_lower, y_upper) =
                certified_point_interval_bounds(&points, policy)?;
            Some(LineCubicBezierAlgebraicEndpointEnvelope {
                span: span_index,
                x_lower,
                x_upper,
                y_lower,
                y_upper,
            })
        })
        .collect()
}

fn algebraic_cubic_span_interior_extrema(
    curve: &CubicBezier,
    span: &LineCubicBezierAlgebraicSourceSpan,
    policy: PredicatePolicy,
) -> Option<Vec<CubicPointInterval>> {
    algebraic_cubic_interval_interior_extrema(
        curve,
        &span.parameter_lower,
        &span.parameter_upper,
        policy,
    )
}

fn algebraic_cubic_interval_interior_extrema(
    curve: &CubicBezier,
    parameter_lower: &Real,
    parameter_upper: &Real,
    policy: PredicatePolicy,
) -> Option<Vec<CubicPointInterval>> {
    // Retained algebraic spans use interval endpoints, so an extrema root is
    // admitted only when exact comparison proves it lies inside the retained
    // parameter interval. If membership is undecidable, the whole envelope is
    // withheld instead of publishing a possibly incomplete box. This is the
    // conservative construction boundary required by Yap (1997).
    let mut extrema = Vec::new();
    for root in cubic_derivative_roots(curve, CubicCoordinate::X, policy)? {
        if real_in_closed_interval(&root, parameter_lower, parameter_upper, policy)? {
            extrema.push(point_exact_interval(&eval_cubic_real(curve, &root))?);
        }
    }
    for root in cubic_derivative_roots(curve, CubicCoordinate::Y, policy)? {
        if real_in_closed_interval(&root, parameter_lower, parameter_upper, policy)? {
            extrema.push(point_exact_interval(&eval_cubic_real(curve, &root))?);
        }
    }
    Some(extrema)
}

#[derive(Clone, Debug)]
struct CubicPointInterval {
    x_lower: Real,
    x_upper: Real,
    y_lower: Real,
    y_upper: Real,
}

fn algebraic_cubic_boundary_point_interval(
    source: LineCubicBezierAlgebraicBreakpointSequenceSource,
    boundary: LineCubicBezierAlgebraicSourceSpanBoundary,
    lines: &[LinePathSegment],
    curves: &[CubicBezier],
    breakpoints: &[LineCubicBezierAlgebraicBreakpoint],
) -> Option<CubicPointInterval> {
    match boundary {
        LineCubicBezierAlgebraicSourceSpanBoundary::SourceStart => match source {
            LineCubicBezierAlgebraicBreakpointSequenceSource::Line(line) => {
                point_exact_interval(lines.get(line)?.start())
            }
            LineCubicBezierAlgebraicBreakpointSequenceSource::Curve(curve) => {
                point_exact_interval(curves.get(curve)?.start())
            }
        },
        LineCubicBezierAlgebraicSourceSpanBoundary::SourceEnd => match source {
            LineCubicBezierAlgebraicBreakpointSequenceSource::Line(line) => {
                point_exact_interval(lines.get(line)?.end())
            }
            LineCubicBezierAlgebraicBreakpointSequenceSource::Curve(curve) => {
                point_exact_interval(curves.get(curve)?.end())
            }
        },
        LineCubicBezierAlgebraicSourceSpanBoundary::Breakpoint(index) => {
            let point_image = &breakpoints.get(index)?.point_image;
            let x = transformed_image_representation(&point_image.x)?;
            let y = transformed_image_representation(&point_image.y)?;
            Some(CubicPointInterval {
                x_lower: x.interval.lower.clone(),
                x_upper: x.interval.upper.clone(),
                y_lower: y.interval.lower.clone(),
                y_upper: y.interval.upper.clone(),
            })
        }
    }
}

fn point_exact_interval(point: &Point2) -> Option<CubicPointInterval> {
    Some(CubicPointInterval {
        x_lower: point.x.clone(),
        x_upper: point.x.clone(),
        y_lower: point.y.clone(),
        y_upper: point.y.clone(),
    })
}

fn certified_point_interval_bounds(
    points: &[CubicPointInterval],
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
enum CubicCoordinate {
    X,
    Y,
}

fn cubic_derivative_roots(
    curve: &CubicBezier,
    coordinate: CubicCoordinate,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    let (a, b, c, _) = cubic_extrema_coordinate_power_coefficients(curve, coordinate);
    let qa = Real::from(3) * a;
    let qb = Real::from(2) * b;
    solve_quadratic_or_linear_roots(qa, qb, c, policy)
}

fn cubic_extrema_coordinate_power_coefficients(
    curve: &CubicBezier,
    coordinate: CubicCoordinate,
) -> (Real, Real, Real, Real) {
    let p0 = cubic_coordinate(curve.start(), coordinate);
    let p1 = cubic_coordinate(curve.control0(), coordinate);
    let p2 = cubic_coordinate(curve.control1(), coordinate);
    let p3 = cubic_coordinate(curve.end(), coordinate);
    let a = -p0.clone() + Real::from(3) * p1.clone() - Real::from(3) * p2.clone() + p3;
    let b = Real::from(3) * p0.clone() - Real::from(6) * p1.clone() + Real::from(3) * p2;
    let c = Real::from(3) * (p1 - p0.clone());
    (a, b, c, p0)
}

fn cubic_coordinate(point: &Point2, coordinate: CubicCoordinate) -> Real {
    match coordinate {
        CubicCoordinate::X => point.x.clone(),
        CubicCoordinate::Y => point.y.clone(),
    }
}

fn solve_quadratic_or_linear_roots(
    a: Real,
    b: Real,
    c: Real,
    policy: PredicatePolicy,
) -> Option<Vec<Real>> {
    match compare_reals_with_policy(&a, &Real::zero(), policy).value()? {
        Ordering::Equal => solve_linear_roots(b, c, policy),
        Ordering::Less | Ordering::Greater => solve_quadratic_roots(a, b, c, policy),
    }
}

fn solve_linear_roots(b: Real, c: Real, policy: PredicatePolicy) -> Option<Vec<Real>> {
    match compare_reals_with_policy(&b, &Real::zero(), policy).value()? {
        Ordering::Equal => Some(Vec::new()),
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

fn compare_algebraic_cubic_overlap_parameters(
    left: &AlgebraicRootRepresentation,
    right: &AlgebraicRootRepresentation,
    policy: PredicatePolicy,
) -> LineCubicBezierAlgebraicOverlapBreakpointOrderClass {
    match compare_algebraic_cubic_parameters(left, right, policy) {
        LineCubicBezierAlgebraicBreakpointOrderClass::Before => {
            LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Before
        }
        LineCubicBezierAlgebraicBreakpointOrderClass::Equal => {
            LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Equal
        }
        LineCubicBezierAlgebraicBreakpointOrderClass::After => {
            LineCubicBezierAlgebraicOverlapBreakpointOrderClass::After
        }
        LineCubicBezierAlgebraicBreakpointOrderClass::Unknown => {
            LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Unknown
        }
    }
}

fn compare_exact_cubic_overlap_line_parameters(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> LineCubicBezierAlgebraicOverlapBreakpointOrderClass {
    match compare_reals_with_policy(left, right, policy).value() {
        Some(Ordering::Less) => LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Before,
        Some(Ordering::Equal) => LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Equal,
        Some(Ordering::Greater) => LineCubicBezierAlgebraicOverlapBreakpointOrderClass::After,
        None => LineCubicBezierAlgebraicOverlapBreakpointOrderClass::Unknown,
    }
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
    _policy: PredicatePolicy,
) -> Result<(), LineCubicBezierArrangementError> {
    for existing in breakpoints.iter() {
        match point2_equal(&existing.point, &point).value() {
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
    insert_exact_cubic_breakpoint(
        breakpoints,
        curve_index,
        event.parameter.clone(),
        event.point.clone(),
        policy,
    )
}

fn insert_exact_cubic_breakpoint(
    breakpoints: &mut Vec<CubicBezierRealBreakpoint>,
    curve_index: usize,
    parameter: Real,
    point: Point2,
    policy: PredicatePolicy,
) -> Result<(), LineCubicBezierArrangementError> {
    for existing in breakpoints.iter() {
        match compare_reals_with_policy(&existing.parameter, &parameter, policy).value() {
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
        parameter,
        point,
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
                match point2_equal(&last.point, &point.point).value() {
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
