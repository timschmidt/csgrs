//! Boolean-topology handoff reports for Bezier curve relations.
//!
//! Bezier curve/curve predicates are only useful to path booleans after their
//! outputs are normalized into split events or explicit blockers. This module
//! provides that normalization layer without pretending that unresolved
//! algebraic regions are topology. The separation follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): exact
//! predicates either provide certified combinatorial data or retain an
//! auditable uncertainty object. The split/arrangement view follows the
//! intersection-insertion stage used by Greiner and Hormann, "Efficient
//! clipping of arbitrary polygons," *ACM Transactions on Graphics* 17(2),
//! 71-83 (1998), and Martinez, Rueda, and Feito, "A new algorithm for
//! computing Boolean operations on polygons," *Computers & Geosciences* 35(6),
//! 1177-1185 (2009).

use crate::{
    BezierCurveIntersectionRegion, BezierCurveRelation, BezierGraphContact,
    BezierIntersectionRegionIsolationCertificate, BezierIntersectionRegionShape,
    BezierIntersectionRegionSummary, BezierLineContactKind, BezierMonotoneGraphContactOrder,
    BezierMonotoneGraphOrder, BezierMonotoneSpan, BooleanFragmentAction, BooleanOp, Classification,
    CubicBezier2, CurvePolicy, IntersectionKind, LineLineIntersection, ParamRange, Point2,
    QuadraticBezier2, RationalQuadraticBezier2, UncertaintyReason,
};
use hyperreal::Real;
use hypersolve::{
    AlgebraicRootComparisonStatus, AlgebraicRootRefinementComparisonConfig,
    AlgebraicRootRefinementComparisonReport, AlgebraicRootRepresentation,
    AlgebraicRootRepresentationReport, AlgebraicRootRepresentationStatus,
    BernsteinSubdivisionIntervalStatus, BernsteinSubdivisionReport, BernsteinSubdivisionStatus,
    RootIsolationStatus, UnivariateRootIsolationReport,
    compare_algebraic_root_representations_with_refinement,
};
use std::{cmp::Ordering, collections::HashSet};

/// Boolean-readiness state of a Bezier curve/curve relation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanHandoffStatus {
    /// The relation certifies that no split events are required.
    NoEvents,
    /// Every retained event has exact parameters and can feed split insertion.
    SplitEventsReady,
    /// Point geometry is certified, but the curve parameters still need recovery.
    NeedsParameterRecovery,
    /// Same-image or finite-overlap geometry needs an overlap-aware resolver.
    NeedsOverlapResolver,
    /// Retained parameter regions still need algebraic isolation/refinement.
    NeedsRegionIsolation,
    /// The relation is not resolved enough for boolean topology.
    Unresolved,
    /// A lower-level primitive reported explicit predicate uncertainty.
    Uncertain,
}

/// Parameterized point event ready for future Bezier split insertion.
///
/// These events carry exact parameters on both source curves. A future Bezier
/// contour segment can evaluate the point from either curve at split time; the
/// optional point is retained when the source predicate already produced one.
/// Keeping point geometry separate from split parameters follows Yap's
/// predicate/construction boundary (1997) and avoids promoting isolated
/// regions that are not yet represented roots.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanPointEvent2 {
    /// Exact parameter on the first curve.
    pub first_param: Real,
    /// Exact parameter on the second curve.
    pub second_param: Real,
    /// Optional certified point supplied by the source predicate.
    pub point: Option<Point2>,
    /// Local contact kind when known.
    pub kind: Option<IntersectionKind>,
}

/// Parameterized overlap event that still needs boolean overlap policy.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanOverlapEvent2 {
    /// Parameter range on the first curve.
    pub first_range: ParamRange,
    /// Parameter range on the second curve.
    pub second_range: ParamRange,
}

/// Overlap-resolution readiness for Bezier/conic boolean handoff events.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanOverlapResolutionStatus {
    /// No overlap events or blockers were supplied.
    Empty,
    /// Every finite overlap range has exact unit-domain boundary parameters.
    Ready,
    /// At least one overlap boundary parameter lies outside `[0, 1]`.
    InvalidParameterDomain,
    /// A non-overlap handoff blocker prevents trusted overlap resolution.
    Blocked,
}

/// Resolved finite-overlap event with exact split-boundary parameters.
///
/// The original ranges are retained verbatim, including reversed orientation
/// on either operand. Separate sorted boundary lists are provided for split
/// insertion. This mirrors the degenerate-overlap handling described by
/// Foster, Hormann, and Popa, "Clipping simple polygons with degenerate
/// intersections," *Computers & Graphics: X* 2 (2019), while preserving Yap's
/// rule that topology construction consumes certified object facts rather than
/// tolerance-collapsed intervals.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanResolvedOverlapEvent2 {
    /// Parameter range on the first curve, preserving source orientation.
    pub first_range: ParamRange,
    /// Parameter range on the second curve, preserving source orientation.
    pub second_range: ParamRange,
    /// Sorted unique first-operand split-boundary parameters.
    pub first_boundary_parameters: Vec<Real>,
    /// Sorted unique second-operand split-boundary parameters.
    pub second_boundary_parameters: Vec<Real>,
}

/// Exact finite-overlap resolution report for Bezier/conic boolean handoff.
///
/// Relation handoffs intentionally classify finite overlaps as blockers until
/// a degenerate-aware resolver has decided which split boundaries must be
/// inserted. This report performs that normalization only: it validates exact
/// unit-domain range endpoints, preserves each overlap interval, and exposes
/// sorted per-operand boundary parameters for fragment construction. It does
/// not decide fill ownership or traversal across coincident arcs/curves.
/// Greiner-Hormann (1998) and Martinez-Rueda-Feito (2009) require
/// intersection insertion before traversal; Yap (1997) requires that this
/// insertion be backed by certified combinatorial data or explicit blockers.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanOverlapResolutionReport2 {
    /// Coarse overlap-resolution status.
    pub status: BezierBooleanOverlapResolutionStatus,
    /// Number of overlap events consumed.
    pub overlap_event_count: usize,
    /// Exact resolved overlap events.
    pub resolved_events: Vec<BezierBooleanResolvedOverlapEvent2>,
    /// Sorted unique first-operand split-boundary parameters.
    pub first_curve_boundary_parameters: Vec<Real>,
    /// Sorted unique second-operand split-boundary parameters.
    pub second_curve_boundary_parameters: Vec<Real>,
    /// Number of overlap events with an out-of-domain endpoint.
    pub invalid_range_count: usize,
    /// Number of non-overlap handoff blockers seen in the input batch.
    pub blocker_count: usize,
    /// First explicit uncertainty reason retained by a blocking handoff.
    pub uncertainty_reason: Option<UncertaintyReason>,
}

/// Arrangement-traversal readiness for split Bezier/conic boolean operands.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanArrangementReadinessStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// Both operands are unsplit no-op fragments and no overlap traversal remains.
    NoInteriorSplits,
    /// Both operands have fragments and any finite overlaps have split boundaries.
    Ready,
    /// The first operand did not produce a usable fragment list.
    MissingFirstFragments,
    /// The second operand did not produce a usable fragment list.
    MissingSecondFragments,
    /// A finite-overlap resolver or same-image overlap blocker is still blocking.
    OverlapBlocked,
    /// A parameter-domain invariant failed before arrangement traversal.
    InvalidParameterDomain,
    /// A lower scheduler, fragment, or predicate stage blocked construction.
    Blocked,
}

/// Boolean arrangement-readiness certificate for higher-order curve fragments.
///
/// This report is the handoff from exact split construction into future
/// path-arrangement traversal. It intentionally does not choose fill ownership
/// or emit a boolean result. Instead, it certifies the preconditions that
/// Greiner-Hormann (1998) and Martinez-Rueda-Feito (2009) assume after
/// intersection insertion: both operands have fragment sequences, and any
/// finite coincident ranges have exact split-boundary parameters. Degenerate
/// overlap handling follows Foster-Hormann-Popa (2019), and the explicit
/// ready/blocked distinction follows Yap's "Towards Exact Geometric
/// Computation" (1997): uncertain or incomplete combinatorics remain data, not
/// topology.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanArrangementReadinessReport2 {
    /// Coarse arrangement-readiness status.
    pub status: BezierBooleanArrangementReadinessStatus,
    /// First operand fragment-construction status.
    pub first_fragment_status: BezierBooleanFragmentConstructionStatus,
    /// Second operand fragment-construction status.
    pub second_fragment_status: BezierBooleanFragmentConstructionStatus,
    /// Finite-overlap resolution status.
    pub overlap_status: BezierBooleanOverlapResolutionStatus,
    /// Number of fragments available for the first operand.
    pub first_fragment_count: usize,
    /// Number of fragments available for the second operand.
    pub second_fragment_count: usize,
    /// Number of resolved finite overlap events.
    pub resolved_overlap_count: usize,
    /// Total unique overlap boundary parameters across both operands.
    pub overlap_boundary_parameter_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Fragment-chain audit status before Bezier/conic arrangement traversal.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanTraversalPreconditionStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Fragment chains are nonempty, continuous, and ready for traversal.
    Ready,
    /// Arrangement readiness still has an explicit blocker.
    ReadinessBlocked,
    /// The first operand has no fragments to traverse.
    MissingFirstFragments,
    /// The second operand has no fragments to traverse.
    MissingSecondFragments,
    /// At least one adjacent first-operand fragment pair is discontinuous.
    FirstChainDiscontinuous,
    /// At least one adjacent second-operand fragment pair is discontinuous.
    SecondChainDiscontinuous,
}

/// Audit of split Bezier/conic fragment chains before arrangement traversal.
///
/// Arrangement algorithms such as Greiner-Hormann (1998) and
/// Martinez-Rueda-Feito (2009) assume that intersection insertion has produced
/// walkable operand chains. This report certifies only that precondition:
/// readiness blockers are preserved, both operands have fragment sequences,
/// and adjacent fragment endpoints match exactly. It does not decide
/// inside/outside ownership or emit a boolean result. Yap, "Towards Exact
/// Geometric Computation" (1997), is the model here: a malformed construction
/// frontier is explicit data, not a tolerance-repaired topology.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanTraversalPreconditionReport2 {
    /// Coarse traversal-precondition status.
    pub status: BezierBooleanTraversalPreconditionStatus,
    /// Arrangement-readiness status that fed this audit.
    pub readiness_status: BezierBooleanArrangementReadinessStatus,
    /// Number of first-operand fragments audited.
    pub first_fragment_count: usize,
    /// Number of second-operand fragments audited.
    pub second_fragment_count: usize,
    /// Number of adjacent first-operand fragment gaps.
    pub first_chain_gap_count: usize,
    /// Number of adjacent second-operand fragment gaps.
    pub second_chain_gap_count: usize,
    /// Number of resolved finite overlap events retained for traversal policy.
    pub resolved_overlap_count: usize,
    /// Total overlap boundary parameters retained for traversal policy.
    pub overlap_boundary_parameter_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Operand identity for a Bezier/conic boolean traversal step.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanTraversalOperand {
    /// A fragment from the first boolean operand.
    First,
    /// A fragment from the second boolean operand.
    Second,
}

/// One fragment visit in a Bezier/conic boolean traversal worklist.
///
/// The step only names the source operand and fragment index. It does not
/// claim that the fragment is emitted, reversed, inside, outside, or shared;
/// those are fill-ownership decisions for a later traversal stage.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BezierBooleanTraversalStep2 {
    /// Source operand for this fragment visit.
    pub operand: BezierBooleanTraversalOperand,
    /// Fragment index in that operand's split-fragment chain.
    pub fragment_index: usize,
}

/// Traversal-schedule status for split Bezier/conic fragments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanTraversalScheduleStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// The fragment worklist is ready for future fill-ownership traversal.
    Ready,
    /// A precondition audit blocked traversal scheduling.
    PreconditionBlocked,
}

/// Explicit fragment worklist for future Bezier/conic boolean traversal.
///
/// This report is the first traversal-facing object after split construction.
/// It converts a successful [`BezierBooleanTraversalPreconditionReport2`] into
/// a deterministic list of operand/index visits and carries finite-overlap
/// counts forward for later degenerate-overlap policy. It intentionally does
/// not classify fragments by fill ownership. Greiner-Hormann (1998) and
/// Martinez-Rueda-Feito (2009) separate insertion from traversal; this report
/// makes that seam explicit. Yap, "Towards Exact Geometric Computation"
/// (1997), motivates keeping blocked preconditions as report data rather than
/// repairing the schedule with tolerance heuristics.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanTraversalScheduleReport2 {
    /// Coarse traversal-schedule status.
    pub status: BezierBooleanTraversalScheduleStatus,
    /// Precondition status used to derive this schedule.
    pub precondition_status: BezierBooleanTraversalPreconditionStatus,
    /// Number of first-operand fragments represented.
    pub first_fragment_count: usize,
    /// Number of second-operand fragments represented.
    pub second_fragment_count: usize,
    /// Deterministic operand/index traversal worklist.
    pub steps: Vec<BezierBooleanTraversalStep2>,
    /// Number of resolved finite overlap events retained for traversal policy.
    pub resolved_overlap_count: usize,
    /// Total overlap boundary parameters retained for traversal policy.
    pub overlap_boundary_parameter_count: usize,
    /// Number of blocking preconditions retained by this schedule.
    pub blocker_count: usize,
}

/// Certified location of a Bezier/conic fragment representative in the other operand.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanFragmentOwnershipLocation {
    /// The fragment representative is outside the other operand's filled area.
    Outside,
    /// The fragment representative is inside the other operand's filled area.
    Inside,
    /// The fragment lies on the other operand boundary and needs overlap policy.
    Boundary,
}

/// One externally certified opposite-operand ownership fact.
///
/// The fact is keyed by the exact traversal step that it classifies. Keeping
/// the key alongside the location prevents callers from accidentally feeding a
/// permuted location list into boolean selection. This is the report boundary
/// recommended by Yap, "Towards Exact Geometric Computation" (1997): an exact
/// point-in-region locator may provide a certified combinatorial fact, but the
/// boolean topology layer must validate that the fact matches the construction
/// object it is about to consume.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BezierBooleanOwnershipFact2 {
    /// Scheduled fragment classified by an external exact locator.
    pub step: BezierBooleanTraversalStep2,
    /// Certified location of the fragment representative in the opposite operand.
    pub opposite_location: BezierBooleanFragmentOwnershipLocation,
}

/// Status for expanding operand-level locator facts into fragment facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanUniformOwnershipFactStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every scheduled fragment received a keyed fact from operand-level locations.
    Ready,
    /// Traversal scheduling is blocked.
    ScheduleBlocked,
    /// A uniform operand location lies on a boundary and needs overlap policy.
    BoundaryNeedsResolution,
}

/// Expanded ownership facts from uniform opposite-operand locator certificates.
///
/// This report is the first built-in ownership-fact producer for Bezier/conic
/// booleans. It is intentionally conservative: it only expands a stronger fact
/// supplied by an exact locator, namely that every first-operand fragment has
/// the same location in the second operand and every second-operand fragment
/// has the same location in the first operand. This covers separated and
/// whole-component containment cases without requiring callers to hand-write
/// one fact per scheduled fragment. Boundary locations remain blockers because
/// degenerate overlap traversal must decide them explicitly; see Foster,
/// Hormann, and Popa, "Clipping simple polygons with degenerate
/// intersections" (*Computers & Graphics: X* 2, 2019). The certificate
/// expansion follows Yap, "Towards Exact Geometric Computation" (1997):
/// combinatorial facts are named, replayable data, not tolerance-derived
/// midpoint samples.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanUniformOwnershipFactReport2 {
    /// Coarse uniform ownership-fact status.
    pub status: BezierBooleanUniformOwnershipFactStatus,
    /// Traversal-schedule status used to derive this report.
    pub schedule_status: BezierBooleanTraversalScheduleStatus,
    /// Certified location for first-operand fragments in the second operand.
    pub first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
    /// Certified location for second-operand fragments in the first operand.
    pub second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
    /// Number of scheduled first-operand fragments.
    pub first_fragment_count: usize,
    /// Number of scheduled second-operand fragments.
    pub second_fragment_count: usize,
    /// Expanded keyed ownership facts in schedule order.
    pub facts: Vec<BezierBooleanOwnershipFact2>,
    /// Number of expanded facts whose location is boundary.
    pub boundary_fact_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Status for generating exact per-fragment locator inputs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanFragmentLocatorInputStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every scheduled fragment has an exact representative point for a locator.
    Ready,
    /// Traversal scheduling is blocked.
    ScheduleBlocked,
    /// At least one scheduled first-operand index does not name a retained fragment.
    MissingFirstFragment,
    /// At least one scheduled second-operand index does not name a retained fragment.
    MissingSecondFragment,
    /// A rational/conic representative point could not be certified.
    PointEvaluationBlocked,
}

/// One exact representative-point query for opposite-operand ownership.
///
/// The `step` key identifies the scheduled fragment being classified, while
/// `representative_point` is an exact point on that fragment. The point is only
/// input to a later exact point/region locator; it is not an ownership fact.
/// This mirrors Yap, "Towards Exact Geometric Computation" (1997): exact
/// construction objects may be prepared here, but topology-changing
/// inside/outside decisions must return their own certificates or explicit
/// blockers. The staged ownership handoff follows Vatti (1992),
/// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanFragmentLocatorInput2 {
    /// Scheduled fragment to classify.
    pub step: BezierBooleanTraversalStep2,
    /// Exact point on the scheduled fragment.
    pub representative_point: Point2,
}

/// Exact representative-point handoff for non-uniform Bezier/conic ownership.
///
/// This report fills the gap between a traversal schedule and the caller's
/// point/region locator. It validates that every scheduled step still names a
/// retained fragment, then emits one exact representative point per step in
/// schedule order. It deliberately does not classify the point as inside,
/// outside, or boundary: callers must replay locator answers through
/// [`BezierBooleanOperandOwnershipLocationReport2`] or
/// [`BezierBooleanOwnershipFactReport2`] before boolean selection. This keeps
/// the predicate/construction boundary explicit in the sense of Yap (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanFragmentLocatorInputReport2 {
    /// Coarse locator-input status.
    pub status: BezierBooleanFragmentLocatorInputStatus,
    /// Traversal-schedule status used to derive this report.
    pub schedule_status: BezierBooleanTraversalScheduleStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Keyed representative points in schedule order.
    pub inputs: Vec<BezierBooleanFragmentLocatorInput2>,
    /// Number of schedule steps inspected.
    pub scheduled_step_count: usize,
    /// Number of representative inputs emitted.
    pub input_count: usize,
    /// Number of first-operand fragment indices that were out of range.
    pub missing_first_fragment_count: usize,
    /// Number of second-operand fragment indices that were out of range.
    pub missing_second_fragment_count: usize,
    /// Number of representative points that could not be certified.
    pub point_evaluation_blocker_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

fn half_real() -> Real {
    (Real::one() / Real::from(2_i8)).expect("2 is a nonzero exact rational denominator")
}

/// Status for expanding per-operand locator vectors into keyed ownership facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanOperandOwnershipLocationStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every scheduled fragment received one non-boundary location.
    Ready,
    /// Traversal scheduling is blocked.
    ScheduleBlocked,
    /// The first-operand location vector is shorter than the first fragment list.
    MissingFirstLocations,
    /// The second-operand location vector is shorter than the second fragment list.
    MissingSecondLocations,
    /// The first-operand location vector is longer than the first fragment list.
    ExtraFirstLocations,
    /// The second-operand location vector is longer than the second fragment list.
    ExtraSecondLocations,
    /// At least one supplied location lies on the opposite boundary and needs overlap policy.
    BoundaryNeedsResolution,
}

/// Expanded ownership facts from per-operand exact locator outputs.
///
/// A point/loop locator for a non-uniform Bezier/conic arrangement naturally
/// produces one location for each first-operand fragment in the second operand
/// and one location for each second-operand fragment in the first operand. This
/// report validates those vector lengths against the traversal schedule and
/// expands them into keyed [`BezierBooleanOwnershipFact2`] values in schedule
/// order. It is intentionally only a certificate adapter: it does not choose a
/// sample point, perturb boundary cases, or repair missing values. That is the
/// predicate/construction separation required by Yap, "Towards Exact Geometric
/// Computation" (1997). The explicit fill-state handoff matches the staged
/// clipping models of Vatti (1992), Greiner-Hormann (1998), and
/// Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanOperandOwnershipLocationReport2 {
    /// Coarse operand-location expansion status.
    pub status: BezierBooleanOperandOwnershipLocationStatus,
    /// Traversal-schedule status used to derive this report.
    pub schedule_status: BezierBooleanTraversalScheduleStatus,
    /// Number of scheduled first-operand fragments.
    pub first_fragment_count: usize,
    /// Number of scheduled second-operand fragments.
    pub second_fragment_count: usize,
    /// Number of first-operand locations supplied by the caller.
    pub supplied_first_location_count: usize,
    /// Number of second-operand locations supplied by the caller.
    pub supplied_second_location_count: usize,
    /// Expanded keyed ownership facts in schedule order.
    pub facts: Vec<BezierBooleanOwnershipFact2>,
    /// Number of missing first-operand locations.
    pub missing_first_location_count: usize,
    /// Number of missing second-operand locations.
    pub missing_second_location_count: usize,
    /// Number of extra first-operand locations.
    pub extra_first_location_count: usize,
    /// Number of extra second-operand locations.
    pub extra_second_location_count: usize,
    /// Number of expanded facts whose location is boundary.
    pub boundary_fact_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Certification status for opposite-operand ownership facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanOwnershipFactStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every scheduled fragment has a keyed non-boundary ownership fact.
    Ready,
    /// Traversal scheduling is blocked.
    ScheduleBlocked,
    /// At least one scheduled fragment has no supplied fact.
    MissingOwnershipFacts,
    /// More facts were supplied than scheduled fragments.
    ExtraOwnershipFacts,
    /// A supplied fact is not keyed to the corresponding scheduled fragment.
    StepMismatch,
    /// At least one fact lies on the opposite boundary and needs overlap policy.
    BoundaryNeedsResolution,
}

/// Validated opposite-operand ownership facts for Bezier/conic traversal.
///
/// This report is the bridge from a future exact locator into
/// [`BezierBooleanOwnershipClassificationReport2`]. It accepts only keyed facts
/// whose operand/index references match the deterministic traversal schedule,
/// preserves missing/extra/mismatched facts as blockers, and refuses to turn a
/// boundary fact into inside/outside by tolerance. The separation follows
/// Greiner-Hormann (1998), Vatti, "A generic solution to polygon clipping"
/// (*Communications of the ACM* 35(7), 56-63, 1992), and
/// Martinez-Rueda-Feito (2009): fill-state facts are established before
/// boolean selection. Yap (1997) is canonical here: certified facts are
/// consumed as exact data, while incomplete facts remain auditable blockers.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanOwnershipFactReport2 {
    /// Coarse ownership-fact certification status.
    pub status: BezierBooleanOwnershipFactStatus,
    /// Traversal-schedule status used to derive this report.
    pub schedule_status: BezierBooleanTraversalScheduleStatus,
    /// Number of schedule steps that require ownership facts.
    pub scheduled_step_count: usize,
    /// Number of facts supplied by the caller.
    pub supplied_fact_count: usize,
    /// Facts accepted in schedule order.
    pub facts: Vec<BezierBooleanOwnershipFact2>,
    /// Locations accepted in schedule order for the classification layer.
    pub locations: Vec<BezierBooleanFragmentOwnershipLocation>,
    /// Number of missing ownership facts.
    pub missing_fact_count: usize,
    /// Number of extra ownership facts.
    pub extra_fact_count: usize,
    /// Number of keyed facts that do not match the scheduled step.
    pub step_mismatch_count: usize,
    /// Number of boundary facts needing degenerate-overlap policy.
    pub boundary_fact_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Boolean action assigned to one scheduled Bezier/conic fragment.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BezierBooleanOwnedTraversalStep2 {
    /// Original scheduled fragment reference.
    pub step: BezierBooleanTraversalStep2,
    /// Certified location of this fragment relative to the opposite operand.
    pub opposite_location: BezierBooleanFragmentOwnershipLocation,
    /// Boolean selection action under the requested operation.
    pub action: BooleanFragmentAction,
}

/// Fill-ownership classification status for scheduled Bezier/conic fragments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanOwnershipClassificationStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every scheduled fragment has a non-boundary ownership classification.
    Ready,
    /// Traversal scheduling is blocked.
    ScheduleBlocked,
    /// Ownership facts were not supplied for every scheduled step.
    MissingOwnershipFacts,
    /// At least one scheduled fragment is on the opposite boundary.
    BoundaryNeedsResolution,
}

/// Report-only fill-ownership classification for Bezier/conic boolean traversal.
///
/// This is the first Bezier/conic layer that applies a boolean operation to
/// scheduled fragments, but it still does not construct output loops. Callers
/// must provide certified opposite-operand locations for every scheduled
/// fragment; missing or boundary locations remain explicit blockers. The
/// action table mirrors the material-contour fragment selection used by the
/// line/arc boolean layer. Greiner-Hormann (1998), Vatti, "A generic solution
/// to polygon clipping" (*Communications of the ACM* 35(7), 56-63, 1992), and
/// Martinez-Rueda-Feito (2009) all separate fill-state classification from
/// boundary assembly. Yap (1997) is the governing exactness rule: no midpoint
/// sample or tolerance may silently replace a certified ownership fact.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanOwnershipClassificationReport2 {
    /// Coarse ownership-classification status.
    pub status: BezierBooleanOwnershipClassificationStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Traversal-schedule status used to derive this report.
    pub schedule_status: BezierBooleanTraversalScheduleStatus,
    /// Number of schedule steps consumed.
    pub scheduled_step_count: usize,
    /// Number of ownership facts supplied by the caller.
    pub supplied_ownership_count: usize,
    /// Classified traversal steps with boolean actions.
    pub owned_steps: Vec<BezierBooleanOwnedTraversalStep2>,
    /// Number of fragments emitted in source direction.
    pub keep_source_count: usize,
    /// Number of fragments emitted reversed.
    pub keep_reversed_count: usize,
    /// Number of fragments discarded.
    pub discard_count: usize,
    /// Number of boundary fragments needing overlap/degenerate policy.
    pub boundary_blocker_count: usize,
    /// Number of missing ownership facts.
    pub missing_ownership_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Emission-plan status for ownership-classified Bezier/conic fragments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanEmissionPlanStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// At least one fragment is ready to feed future loop assembly.
    Ready,
    /// Ownership classification is blocked by missing facts or boundary policy.
    OwnershipBlocked,
    /// Ownership was certified but every fragment is discarded by the operation.
    NoEmittedFragments,
}

/// Explicit emission plan before higher-order Bezier/conic loop assembly.
///
/// This report consumes [`BezierBooleanOwnershipClassificationReport2`] and
/// separates fragments that should be emitted from fragments that should be
/// discarded. It still does not assemble cycles, orient closed loops, or
/// resolve coincident-boundary traversal. That separation matches
/// Greiner-Hormann (1998), Vatti (1992), and Martinez-Rueda-Feito (2009):
/// fill classification selects candidate boundary pieces before graph/loop
/// construction. Yap, "Towards Exact Geometric Computation" (1997), motivates
/// retaining ownership blockers rather than fabricating an output boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanEmissionPlanReport2 {
    /// Coarse emission-plan status.
    pub status: BezierBooleanEmissionPlanStatus,
    /// Ownership status used to derive this plan.
    pub ownership_status: BezierBooleanOwnershipClassificationStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Fragments selected for later loop assembly.
    pub emitted_steps: Vec<BezierBooleanOwnedTraversalStep2>,
    /// Fragments discarded by the boolean operation.
    pub discarded_steps: Vec<BezierBooleanOwnedTraversalStep2>,
    /// Number of fragments emitted in source direction.
    pub keep_source_count: usize,
    /// Number of fragments emitted reversed.
    pub keep_reversed_count: usize,
    /// Number of fragments discarded.
    pub discard_count: usize,
    /// Number of unresolved boundary fragments retained by ownership.
    pub boundary_blocker_count: usize,
    /// Number of missing ownership facts retained by ownership.
    pub missing_ownership_count: usize,
    /// Number of blocking preconditions retained by this plan.
    pub blocker_count: usize,
}

/// Loop-assembly readiness for emitted Bezier/conic boolean fragments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanAssemblyReadinessStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Emitted fragment references are valid and can feed future loop assembly.
    Ready,
    /// The emission plan is blocked by an earlier stage.
    EmissionBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// At least one emitted fragment index is outside its operand fragment list.
    InvalidFragmentReference,
}

/// Readiness audit before higher-order Bezier/conic output-loop assembly.
///
/// This report validates only the structural references emitted by
/// [`BezierBooleanEmissionPlanReport2`]. It verifies that each emitted
/// operand/index pair is inside the supplied operand fragment counts and
/// preserves no-output and blocked states separately. It does not order emitted
/// fragments into cycles or decide overlap traversal. Greiner-Hormann (1998),
/// Vatti (1992), and Martinez-Rueda-Feito (2009) all require a boundary
/// assembly phase after fragment selection; Yap, "Towards Exact Geometric
/// Computation" (1997), requires that this phase consume certified
/// combinatorial references rather than stale or tolerance-repaired indices.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanAssemblyReadinessReport2 {
    /// Coarse output-assembly readiness status.
    pub status: BezierBooleanAssemblyReadinessStatus,
    /// Emission-plan status used to derive this audit.
    pub emission_status: BezierBooleanEmissionPlanStatus,
    /// Number of first-operand fragments available to assembly.
    pub first_fragment_count: usize,
    /// Number of second-operand fragments available to assembly.
    pub second_fragment_count: usize,
    /// Number of emitted references checked.
    pub emitted_step_count: usize,
    /// Number of emitted first-operand references.
    pub first_emitted_count: usize,
    /// Number of emitted second-operand references.
    pub second_emitted_count: usize,
    /// Number of out-of-range emitted references.
    pub invalid_reference_count: usize,
    /// Number of emitted fragments in source direction.
    pub keep_source_count: usize,
    /// Number of emitted fragments reversed.
    pub keep_reversed_count: usize,
    /// Number of blocking preconditions retained by this audit.
    pub blocker_count: usize,
}

/// Output-loop assembly plan status for Bezier/conic boolean fragments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopAssemblyPlanStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Emitted fragment references are packaged for future loop construction.
    Ready,
    /// Assembly readiness is blocked or contains invalid emitted references.
    AssemblyBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
}

/// Report-only output-loop assembly plan for Bezier/conic booleans.
///
/// This is the last report-only handoff before an implementation that actually
/// links higher-order fragments into output cycles. It consumes
/// [`BezierBooleanAssemblyReadinessReport2`] plus the corresponding
/// [`BezierBooleanEmissionPlanReport2`] and packages only readiness-certified
/// emitted references. It does not infer adjacency, close loops, choose nesting
/// roles, or resolve coincident traversal. The staged boundary construction
/// follows Greiner-Hormann (1998), Vatti (1992), and Martinez-Rueda-Feito
/// (2009); the refusal to assemble from invalid references follows Yap,
/// "Towards Exact Geometric Computation" (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopAssemblyPlanReport2 {
    /// Coarse loop-assembly plan status.
    pub status: BezierBooleanLoopAssemblyPlanStatus,
    /// Assembly-readiness status used to derive this plan.
    pub assembly_status: BezierBooleanAssemblyReadinessStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Emitted fragment references to feed future loop construction.
    pub emitted_steps: Vec<BezierBooleanOwnedTraversalStep2>,
    /// Number of emitted first-operand references.
    pub first_emitted_count: usize,
    /// Number of emitted second-operand references.
    pub second_emitted_count: usize,
    /// Number of emitted fragments in source direction.
    pub keep_source_count: usize,
    /// Number of emitted fragments reversed.
    pub keep_reversed_count: usize,
    /// Number of invalid references retained by assembly readiness.
    pub invalid_reference_count: usize,
    /// Number of blocking preconditions retained by this plan.
    pub blocker_count: usize,
}

/// Externally certified graph facts for a Bezier/conic loop-assembly plan.
///
/// The fact is keyed by emitted-step count so a graph traversal certificate
/// cannot accidentally be reused for a different emission plan. This is a
/// small but important exact-computation boundary from Yap, "Towards Exact
/// Geometric Computation" (1997): combinatorial topology supplied by another
/// predicate must name the object it certifies before later boolean stages use
/// it. Branch vertices and resolved-overlap obligations follow the traversal
/// phase separation used by Vatti (1992), Greiner-Hormann (1998), and
/// Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BezierBooleanLoopGraphFacts2 {
    /// Number of emitted steps in the plan these graph facts certify.
    pub emitted_step_count: usize,
    /// Number of arrangement vertices that still require graph traversal.
    pub branch_vertex_count: usize,
    /// Number of resolved overlap ranges that still require traversal policy.
    pub resolved_overlap_count: usize,
}

/// One certified successor edge in an emitted-fragment traversal graph.
///
/// The indices name entries in
/// [`BezierBooleanLoopAssemblyPlanReport2::emitted_steps`]. A graph walker for
/// branch vertices or resolved overlaps can provide these exact successor
/// facts instead of a pre-linearized permutation. This keeps the graph topology
/// as replayable combinatorial evidence, matching Yap, "Towards Exact
/// Geometric Computation" (1997). The one-successor/one-predecessor boundary
/// walk model is the traversal phase used by Vatti (1992), Greiner-Hormann
/// (1998), and Martinez-Rueda-Feito (2009); degenerate overlap obligations
/// follow Foster-Hormann-Popa (2019).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BezierBooleanLoopGraphSuccessorFact2 {
    /// Emitted-step index whose successor is being certified.
    pub from_step_index: usize,
    /// Emitted-step index reached next in the certified boundary walk.
    pub to_step_index: usize,
}

/// Validation status for keyed Bezier/conic loop-graph facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopGraphFactStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// The graph facts certify a linear walk may be consumed.
    Ready,
    /// Loop assembly was blocked before graph facts could be used.
    PlanBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// The graph facts are keyed to a different emitted-step count.
    EmittedStepCountMismatch,
    /// Branch vertices require a separate graph traversal certificate.
    BranchPointsNeedTraversal,
    /// Resolved overlaps require a separate degenerate traversal certificate.
    ResolvedOverlapsNeedTraversal,
}

/// Validated graph facts for Bezier/conic loop traversal.
///
/// This report turns raw graph counts into a checked certificate tied to a
/// concrete [`BezierBooleanLoopAssemblyPlanReport2`]. It deliberately does not
/// discover graph topology. A future arrangement traversal stage supplies the
/// facts; this report validates their key and preserves branch/overlap
/// obligations as blockers. That keeps the boolean pipeline certificate-based,
/// matching Yap (1997), and preserves the explicit traversal/fill separation
/// described by Vatti (1992), Greiner-Hormann (1998), and
/// Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopGraphFactReport2 {
    /// Coarse graph-fact validation status.
    pub status: BezierBooleanLoopGraphFactStatus,
    /// Loop-assembly status used to derive this report.
    pub plan_status: BezierBooleanLoopAssemblyPlanStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Number of emitted steps in the plan.
    pub emitted_step_count: usize,
    /// Number of emitted steps claimed by the supplied fact.
    pub supplied_emitted_step_count: usize,
    /// Number of branch vertices supplied by the graph certificate.
    pub branch_vertex_count: usize,
    /// Number of resolved-overlap traversal obligations supplied by the graph certificate.
    pub resolved_overlap_count: usize,
    /// Number of emitted-step key mismatches.
    pub emitted_step_mismatch_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Graph-traversal readiness for emitted Bezier/conic boolean fragments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopGraphTraversalStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// The emitted order has no graph blockers and may feed linear closure.
    Ready,
    /// Loop assembly was blocked before graph traversal could be audited.
    PlanBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// One or more branch vertices require a real graph traversal/reorder.
    BranchPointsNeedTraversal,
    /// Resolved overlaps still require degenerate-overlap traversal policy.
    ResolvedOverlapsNeedTraversal,
}

/// Audit of graph traversal obligations before Bezier/conic loop closure.
///
/// The current higher-order boolean pipeline can close fragments only when the
/// emitted sequence is already a certified boundary walk. This report makes
/// that assumption explicit: callers provide counts for branch vertices and
/// resolved-overlap traversal obligations, and any nonzero count blocks linear
/// closure until a real arrangement graph walk has ordered the fragments.
/// Degenerate overlap handling follows Foster, Hormann, and Popa, "Clipping
/// simple polygons with degenerate intersections" (*Computers & Graphics: X*
/// 2, 2019). The insertion/traversal separation follows Vatti (1992),
/// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009). Yap, "Towards
/// Exact Geometric Computation" (1997), is the rule for this API: unsupported
/// graph branches are explicit blockers, not tolerance-reordered topology.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopGraphTraversalReport2 {
    /// Coarse graph-traversal readiness status.
    pub status: BezierBooleanLoopGraphTraversalStatus,
    /// Loop-assembly-plan status used to derive this audit.
    pub plan_status: BezierBooleanLoopAssemblyPlanStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Number of emitted references in the loop-assembly plan.
    pub emitted_step_count: usize,
    /// Number of branch vertices that require graph traversal.
    pub branch_vertex_count: usize,
    /// Number of resolved overlap ranges still requiring traversal policy.
    pub resolved_overlap_count: usize,
    /// Number of blocking graph preconditions retained by this audit.
    pub blocker_count: usize,
}

/// Status for a certified Bezier/conic loop graph walk order.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopGraphWalkStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// A certified walk order covers every emitted fragment exactly once.
    Ready,
    /// Graph traversal readiness is blocked.
    TraversalBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// The supplied walk omits one or more emitted fragments.
    MissingWalkSteps,
    /// The supplied walk contains more entries than emitted fragments.
    ExtraWalkSteps,
    /// The supplied walk names an emitted-fragment index outside the plan.
    OutOfRangeWalkStep,
    /// The supplied walk names the same emitted-fragment index more than once.
    DuplicateWalkStep,
}

/// Certified graph-walk order for emitted Bezier/conic boolean fragments.
///
/// This report is the constructive counterpart to
/// [`BezierBooleanLoopGraphTraversalReport2`]. A future arrangement graph walk
/// can supply a permutation of emitted-fragment indices; this report validates
/// that the walk covers every emitted reference exactly once and exposes the
/// reordered fragment payload for exact endpoint closure. The algorithmic seam
/// follows Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito
/// (2009): traversal order is a graph result, not an incidental vector order.
/// Degenerate overlap policy follows Foster-Hormann-Popa (2019). Yap (1997)
/// is the exactness contract: incomplete, duplicated, or stale walk indices
/// remain auditable blockers rather than being repaired by sorting or snapping.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopGraphWalkReport2 {
    /// Coarse graph-walk status.
    pub status: BezierBooleanLoopGraphWalkStatus,
    /// Graph-traversal readiness status used to derive this report.
    pub traversal_status: BezierBooleanLoopGraphTraversalStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Number of emitted references in the loop-assembly plan.
    pub emitted_step_count: usize,
    /// Number of walk indices supplied by the graph traversal stage.
    pub supplied_walk_step_count: usize,
    /// Certified emitted-fragment indices in walk order.
    pub walk_indices: Vec<usize>,
    /// Emitted references reordered into graph-walk order.
    pub ordered_steps: Vec<BezierBooleanOwnedTraversalStep2>,
    /// Number of omitted emitted references.
    pub missing_walk_step_count: usize,
    /// Number of extra walk entries.
    pub extra_walk_step_count: usize,
    /// Number of out-of-range walk entries.
    pub out_of_range_walk_step_count: usize,
    /// Number of duplicate walk entries.
    pub duplicate_walk_step_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Status for deriving a Bezier/conic graph walk from successor facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopGraphSuccessorWalkStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Successor facts define one closed walk over every emitted fragment.
    Ready,
    /// Graph traversal readiness is blocked.
    TraversalBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// Fewer successor facts than emitted fragments were supplied.
    MissingSuccessorFacts,
    /// More successor facts than emitted fragments were supplied.
    ExtraSuccessorFacts,
    /// At least one successor fact names an emitted-fragment index outside the plan.
    OutOfRangeSuccessorStep,
    /// At least one emitted fragment has more than one supplied successor.
    DuplicateSuccessorSource,
    /// At least one emitted fragment has more than one supplied predecessor.
    DuplicateSuccessorTarget,
    /// Successor facts do not form one closed walk covering all emitted fragments.
    OpenOrDisconnectedSuccessorCycle,
}

/// Certified graph walk derived from keyed successor facts.
///
/// This report is a small graph-traversal result object. It validates that the
/// supplied successor relation is a single closed cycle over the emitted
/// fragments and then exposes the corresponding walk permutation. It does not
/// choose the successor facts; a geometric arrangement walker must produce
/// them. The validation boundary follows Yap (1997): graph topology is trusted
/// only when replayed as exact, keyed evidence. The traversal contract follows
/// Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009), with
/// resolved-overlap obligations kept explicit as in Foster-Hormann-Popa (2019).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopGraphSuccessorWalkReport2 {
    /// Coarse successor-walk status.
    pub status: BezierBooleanLoopGraphSuccessorWalkStatus,
    /// Graph-traversal readiness status used to derive this report.
    pub traversal_status: BezierBooleanLoopGraphTraversalStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Number of emitted references in the loop-assembly plan.
    pub emitted_step_count: usize,
    /// Number of successor facts supplied by the graph traversal stage.
    pub supplied_successor_count: usize,
    /// Canonical walk start index used for deterministic reporting.
    pub start_step_index: Option<usize>,
    /// Certified emitted-fragment indices in successor-walk order.
    pub walk_indices: Vec<usize>,
    /// Emitted references reordered into successor-walk order.
    pub ordered_steps: Vec<BezierBooleanOwnedTraversalStep2>,
    /// Number of missing successor facts.
    pub missing_successor_count: usize,
    /// Number of extra successor facts.
    pub extra_successor_count: usize,
    /// Number of out-of-range source or target indices.
    pub out_of_range_successor_count: usize,
    /// Number of duplicate successor sources.
    pub duplicate_source_count: usize,
    /// Number of duplicate successor targets.
    pub duplicate_target_count: usize,
    /// Number of graph-cycle closure/connectivity blockers.
    pub cycle_blocker_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Status for deriving one or more Bezier/conic graph-walk cycles from successor facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopGraphMultiCycleWalkStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Successor facts define one or more closed walks over every emitted fragment.
    Ready,
    /// Graph traversal readiness is blocked.
    TraversalBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// Fewer successor facts than emitted fragments were supplied.
    MissingSuccessorFacts,
    /// More successor facts than emitted fragments were supplied.
    ExtraSuccessorFacts,
    /// At least one successor fact names an emitted-fragment index outside the plan.
    OutOfRangeSuccessorStep,
    /// At least one emitted fragment has more than one supplied successor.
    DuplicateSuccessorSource,
    /// At least one emitted fragment has more than one supplied predecessor.
    DuplicateSuccessorTarget,
    /// Successor facts do not form closed walks covering all emitted fragments.
    OpenSuccessorCycle,
}

/// Certified multi-cycle graph walk derived from keyed successor facts.
///
/// [`BezierBooleanLoopGraphSuccessorWalkReport2`] is deliberately strict: it
/// accepts only one closed successor cycle. This report is the explicit
/// multi-loop counterpart for arrangements whose graph traversal produces
/// several disjoint closed output loops. It still validates exactly one
/// successor and one predecessor per emitted fragment and then concatenates
/// cycles in deterministic smallest-unvisited-index order. The resulting walk
/// can feed the existing closure/output-loop reports because each cycle is
/// already closed before the next begins. This keeps traversal evidence
/// replayable under Yap, "Towards Exact Geometric Computation" (1997), while
/// preserving the Vatti (1992), Greiner-Hormann (1998), and
/// Martinez-Rueda-Feito (2009) separation between graph traversal and fill.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopGraphMultiCycleWalkReport2 {
    /// Coarse multi-cycle successor-walk status.
    pub status: BezierBooleanLoopGraphMultiCycleWalkStatus,
    /// Graph-traversal readiness status used to derive this report.
    pub traversal_status: BezierBooleanLoopGraphTraversalStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Number of emitted references in the loop-assembly plan.
    pub emitted_step_count: usize,
    /// Number of successor facts supplied by the graph traversal stage.
    pub supplied_successor_count: usize,
    /// Canonical start index of each accepted cycle in concatenated order.
    pub cycle_start_indices: Vec<usize>,
    /// Start offset of each accepted cycle in [`Self::walk_indices`].
    pub cycle_walk_start_offsets: Vec<usize>,
    /// Number of emitted fragments in each accepted cycle.
    pub cycle_step_counts: Vec<usize>,
    /// Number of accepted closed cycles.
    pub cycle_count: usize,
    /// Certified emitted-fragment indices in concatenated cycle order.
    pub walk_indices: Vec<usize>,
    /// Emitted references reordered into concatenated cycle order.
    pub ordered_steps: Vec<BezierBooleanOwnedTraversalStep2>,
    /// Number of missing successor facts.
    pub missing_successor_count: usize,
    /// Number of extra successor facts.
    pub extra_successor_count: usize,
    /// Number of out-of-range source or target indices.
    pub out_of_range_successor_count: usize,
    /// Number of duplicate successor sources.
    pub duplicate_source_count: usize,
    /// Number of duplicate successor targets.
    pub duplicate_target_count: usize,
    /// Number of graph-cycle closure/connectivity blockers.
    pub cycle_blocker_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Loop-closure status for emitted Bezier/conic boolean fragments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopClosureStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// The emitted sequence closes into one or more exact output loops.
    Closed,
    /// Loop assembly was blocked before endpoint closure could be audited.
    PlanBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// At least one emitted reference does not resolve to a fragment endpoint.
    InvalidFragmentReference,
    /// Emitted fragments contain one or more exact endpoint gaps.
    OpenChains,
}

/// Directed Bezier/conic fragment endpoint payload used during loop closure.
///
/// The source fragment endpoints are reversed when the selected boolean action
/// asks for reversed emission. This is the first place where the fragment
/// direction chosen by fill classification is converted into boundary-walk
/// geometry. The conversion remains exact and object-level: endpoint equality
/// is structural [`Point2`] equality over [`Real`], with no primitive-float
/// tolerance. This follows Yap, "Towards Exact Geometric Computation"
/// (1997), while the directed-boundary assembly seam follows Vatti, "A generic
/// solution to polygon clipping" (1992), Greiner-Hormann (1998), and
/// Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanDirectedLoopFragment2 {
    /// Source operand for this emitted fragment.
    pub operand: BezierBooleanTraversalOperand,
    /// Fragment index within the source operand.
    pub fragment_index: usize,
    /// Boolean action that selected the emitted direction.
    pub action: BooleanFragmentAction,
    /// Directed start point after applying the emission action.
    pub start: Point2,
    /// Directed end point after applying the emission action.
    pub end: Point2,
}

/// Exact loop-closure audit for emitted Bezier/conic boolean fragments.
///
/// This report consumes a [`BezierBooleanLoopAssemblyPlanReport2`] and concrete
/// fragment endpoint lists. It resolves each emitted reference to directed
/// endpoints, checks consecutive endpoint equality, and counts exact closed
/// loops in the supplied order. It intentionally does not reorder fragments,
/// infer missing intersections, or assign material/hole nesting. Those are
/// separate certified arrangement tasks. Yap (1997) is the governing rule:
/// open chains and stale references remain explicit blockers rather than being
/// repaired by tolerance snapping. The staged loop construction mirrors Vatti
/// (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopClosureReport2 {
    /// Coarse loop-closure status.
    pub status: BezierBooleanLoopClosureStatus,
    /// Loop-assembly-plan status used to derive this audit.
    pub plan_status: BezierBooleanLoopAssemblyPlanStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Directed endpoint payloads resolved from emitted references.
    pub directed_fragments: Vec<BezierBooleanDirectedLoopFragment2>,
    /// Number of emitted references consumed.
    pub emitted_step_count: usize,
    /// Number of references that did not resolve to a fragment endpoint.
    pub invalid_reference_count: usize,
    /// Number of exact gaps between adjacent emitted fragments.
    pub adjacency_gap_count: usize,
    /// Number of open chains left after scanning the emitted sequence.
    pub open_chain_count: usize,
    /// Number of exact closed loops found in emitted order.
    pub closed_loop_count: usize,
    /// Number of blocking preconditions retained by this audit.
    pub blocker_count: usize,
}

/// Output-loop packaging status for closed Bezier/conic boolean boundaries.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanOutputLoopStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Exactly closed directed fragments were packaged into output loops.
    Ready,
    /// Loop closure was blocked before output loops could be packaged.
    ClosureBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// A closed closure report did not contain a reconstructable loop range.
    MalformedClosedLoops,
}

/// One exact directed output loop over Bezier/conic fragments.
///
/// The loop names a contiguous range of [`BezierBooleanDirectedLoopFragment2`]
/// values inside [`BezierBooleanOutputLoopReport2::directed_fragments`]. It is
/// deliberately a topology carrier, not a region contour: material/hole role,
/// winding, and containment nesting are later certified stages. This keeps the
/// boundary-construction seam advocated by Vatti (1992), Greiner-Hormann
/// (1998), and Martinez-Rueda-Feito (2009), while following Yap's exact
/// geometric-computation rule that combinatorial topology must be certified
/// before it becomes output geometry.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanOutputLoop2 {
    /// Index of the first directed fragment in this loop.
    pub first_directed_fragment_index: usize,
    /// Number of directed fragments in this loop.
    pub directed_fragment_count: usize,
    /// Exact start/end point of the closed loop.
    pub anchor: Point2,
}

/// Report-bearing output-loop package for Bezier/conic booleans.
///
/// This consumes [`BezierBooleanLoopClosureReport2`] after exact endpoint
/// closure has succeeded. It records closed loop ranges over directed
/// fragments and preserves all non-ready states as blockers. It intentionally
/// does not infer nesting, regularize branch points, or convert loops into a
/// filled region. Yap, "Towards Exact Geometric Computation" (1997), is the
/// contract: closed loops are accepted only from certified exact closure, and
/// all other states remain auditable report data.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanOutputLoopReport2 {
    /// Coarse output-loop packaging status.
    pub status: BezierBooleanOutputLoopStatus,
    /// Loop-closure status used to derive this report.
    pub closure_status: BezierBooleanLoopClosureStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Directed endpoint payloads retained from closure.
    pub directed_fragments: Vec<BezierBooleanDirectedLoopFragment2>,
    /// Closed output loops over `directed_fragments`.
    pub loops: Vec<BezierBooleanOutputLoop2>,
    /// Number of exact closed loops reported by closure.
    pub closed_loop_count: usize,
    /// Number of directed fragments retained.
    pub directed_fragment_count: usize,
    /// Number of open chains retained from closure.
    pub open_chain_count: usize,
    /// Number of adjacency gaps retained from closure.
    pub adjacency_gap_count: usize,
    /// Number of invalid references retained from closure.
    pub invalid_reference_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Status for generating exact loop-locator input points.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopLocatorInputStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every output loop has a keyed exact representative point.
    Ready,
    /// Output-loop packaging was blocked.
    OutputLoopBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// At least one output loop range does not reference retained fragments.
    MalformedLoopRange,
}

/// One exact representative point for downstream loop-location predicates.
///
/// The point is keyed by output-loop index and copied from the exact anchor of
/// [`BezierBooleanOutputLoop2`]. It is not a containment decision. A later
/// exact point/loop locator may use the point as a query handle, then return
/// certified containment or nesting facts. This preserves Yap's "Towards Exact
/// Geometric Computation" (1997) boundary: constructing a representative
/// object is separate from deciding topology with it.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopLocatorInput2 {
    /// Index into [`BezierBooleanOutputLoopReport2::loops`].
    pub loop_index: usize,
    /// First directed fragment used as the representative source.
    pub directed_fragment_index: usize,
    /// Exact representative point for locator queries.
    pub representative_point: Point2,
}

/// Exact representative-point handoff for Bezier/conic loop locators.
///
/// This report prepares inputs for the remaining non-uniform arrangement
/// locators without performing containment. It validates that every packaged
/// output loop names a nonempty in-range directed-fragment span, then emits a
/// keyed exact representative point. Point-in-loop and loop-in-loop decisions
/// remain external certified predicates. The staging follows Vatti (1992),
/// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009): closed boundary
/// construction precedes containment classification. Yap (1997) is the
/// exactness rule: malformed ranges stay blockers and representative points
/// never become sampled topology evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopLocatorInputReport2 {
    /// Coarse locator-input status.
    pub status: BezierBooleanLoopLocatorInputStatus,
    /// Output-loop packaging status used to derive this report.
    pub output_status: BezierBooleanOutputLoopStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Keyed exact representative points, one per accepted output loop.
    pub inputs: Vec<BezierBooleanLoopLocatorInput2>,
    /// Number of output loops inspected.
    pub output_loop_count: usize,
    /// Number of representative inputs emitted.
    pub input_count: usize,
    /// Number of malformed loop ranges found.
    pub malformed_loop_range_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Status for generating exact loop-containment query work.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopContainmentQueryStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every ordered loop pair has a keyed containment query.
    Ready,
    /// Locator-input generation was blocked.
    LocatorInputBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// Fewer than two loops exist, so no pairwise containment queries are needed.
    NotEnoughLoops,
}

/// One exact representative-point containment query between output loops.
///
/// `query_loop_index` names the loop whose representative point is being
/// classified against `candidate_container_loop_index`. The point is copied
/// from [`BezierBooleanLoopLocatorInput2`] and remains exact [`Real`]-backed
/// geometry. This is a query object, not a result: a later exact loop locator
/// must return [`BezierBooleanLoopContainmentFact2`] or an explicit blocker.
/// Keeping the query separate from the predicate result follows Yap,
/// "Towards Exact Geometric Computation" (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopContainmentQuery2 {
    /// Loop whose representative point is being located.
    pub query_loop_index: usize,
    /// Candidate loop that may contain `query_loop_index`.
    pub candidate_container_loop_index: usize,
    /// Exact representative point for the queried loop.
    pub representative_point: Point2,
}

/// Pairwise exact locator-query worklist for Bezier/conic output loops.
///
/// This report consumes validated locator inputs and emits one ordered query
/// for every distinct `(query_loop, candidate_container)` pair. It deliberately
/// does not derive nesting or containment from representative points. A future
/// point/loop locator must certify each query before
/// [`BezierBooleanLoopContainmentFactReport2`] can derive depths. This staged
/// arrangement/containment split follows Vatti (1992), Greiner-Hormann
/// (1998), and Martinez-Rueda-Feito (2009); Yap (1997) is the exactness rule
/// that query geometry and topological decisions remain separate data.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopContainmentQueryReport2 {
    /// Coarse containment-query status.
    pub status: BezierBooleanLoopContainmentQueryStatus,
    /// Locator-input status used to derive this report.
    pub locator_status: BezierBooleanLoopLocatorInputStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Ordered exact query worklist for downstream locators.
    pub queries: Vec<BezierBooleanLoopContainmentQuery2>,
    /// Number of loops available for containment classification.
    pub loop_count: usize,
    /// Number of ordered loop-pair queries emitted.
    pub query_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Exact locator result for one Bezier/conic loop containment query.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopContainmentQueryResult {
    /// The candidate loop strictly contains the queried loop representative.
    Contains,
    /// The candidate loop does not contain the queried loop representative.
    Outside,
    /// The representative lies on the candidate boundary and needs overlap policy.
    Boundary,
    /// The locator could not certify the query.
    Unknown,
}

/// One keyed result returned by an exact point/loop locator.
///
/// The loop indices must match a query generated by
/// [`BezierBooleanLoopContainmentQueryReport2`]. This key replay prevents a
/// locator result for one loop pair from being reused for another pair. Yap,
/// "Towards Exact Geometric Computation" (1997), is the contract: predicate
/// results become construction facts only after their object identity and
/// decision status are validated.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BezierBooleanLoopContainmentQueryResult2 {
    /// Loop whose representative point was classified.
    pub query_loop_index: usize,
    /// Candidate loop tested as a possible container.
    pub candidate_container_loop_index: usize,
    /// Certified containment classification for the ordered pair.
    pub result: BezierBooleanLoopContainmentQueryResult,
}

/// Status for replaying containment-query results into containment facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopContainmentQueryResultStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Certified query results were replayed into containment facts.
    Ready,
    /// Query generation was blocked.
    QueryBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// Fewer than two loops exist, so containment facts are vacuously empty.
    NotEnoughLoops,
    /// Fewer query results than query work items were supplied.
    MissingQueryResults,
    /// More query results than query work items were supplied.
    ExtraQueryResults,
    /// At least one result is keyed to a different query pair.
    QueryKeyMismatch,
    /// At least one result classified a representative on a boundary.
    BoundaryNeedsResolution,
    /// At least one result remained uncertified.
    UnknownNeedsResolution,
}

/// Status for certifying loop containment from output loops and locator results.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopContainmentCertificationStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Output-loop containment and nesting depths were certified.
    Ready,
    /// Locator-input generation was blocked by malformed output loops.
    LocatorInputBlocked,
    /// Query-result replay was blocked by missing, stale, boundary, or unknown answers.
    QueryResultBlocked,
    /// Replayed containment facts were not a valid laminar containment certificate.
    ContainmentFactBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
}

/// End-to-end containment certificate for Bezier/conic output loops.
///
/// This report is the safe handoff from an exact point/loop locator into the
/// boolean fill stages. It derives locator inputs and the ordered containment
/// query worklist from a concrete [`BezierBooleanOutputLoopReport2`], replays
/// caller-supplied locator answers through
/// [`BezierBooleanLoopContainmentQueryResultReport2`], then validates the
/// resulting containment facts through
/// [`BezierBooleanLoopContainmentFactReport2`]. Callers therefore cannot
/// accidentally pair locator results with the wrong output-loop package or
/// treat boundary/unknown decisions as "outside." That is the exactness
/// contract of Yap, "Towards Exact Geometric Computation" (1997): predicate
/// answers become construction data only after object identity and uncertainty
/// are replayed. The boundary, containment, and fill split follows Vatti
/// (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopContainmentCertificationReport2 {
    /// Coarse certification status.
    pub status: BezierBooleanLoopContainmentCertificationStatus,
    /// Locator-input generation status.
    pub locator_status: BezierBooleanLoopLocatorInputStatus,
    /// Query-worklist status.
    pub query_status: BezierBooleanLoopContainmentQueryStatus,
    /// Query-result replay status.
    pub query_result_status: BezierBooleanLoopContainmentQueryResultStatus,
    /// Containment-fact validation status.
    pub fact_status: BezierBooleanLoopContainmentFactStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Validated strict containment facts.
    pub containment_facts: Vec<BezierBooleanLoopContainmentFact2>,
    /// Derived keyed nesting-depth facts.
    pub depth_facts: Vec<BezierBooleanLoopNestingDepthFact2>,
    /// Number of output loops inspected.
    pub output_loop_count: usize,
    /// Number of locator inputs emitted.
    pub locator_input_count: usize,
    /// Number of pairwise containment queries emitted.
    pub query_count: usize,
    /// Number of locator answers supplied.
    pub supplied_result_count: usize,
    /// Number of containment facts accepted.
    pub containment_fact_count: usize,
    /// Number of depth facts derived.
    pub depth_fact_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Validated replay of exact containment-query results.
///
/// This report is the bridge from a future exact point/loop locator to
/// [`BezierBooleanLoopContainmentFactReport2`]. It validates one result per
/// query, rejects stale keys, boundary, and unknown decisions, and lowers only
/// strict `Contains` results into [`BezierBooleanLoopContainmentFact2`].
/// `Outside` results are retained in counts but do not become facts. The
/// separation follows Yap (1997): predicate certificates are replayed before
/// they are used by construction. The boundary/containment stage split follows
/// Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopContainmentQueryResultReport2 {
    /// Coarse query-result replay status.
    pub status: BezierBooleanLoopContainmentQueryResultStatus,
    /// Query worklist status used to derive this report.
    pub query_status: BezierBooleanLoopContainmentQueryStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Strict containment facts accepted for nesting-depth derivation.
    pub containment_facts: Vec<BezierBooleanLoopContainmentFact2>,
    /// Number of query work items expected.
    pub query_count: usize,
    /// Number of query results supplied.
    pub supplied_result_count: usize,
    /// Number of strict containment facts emitted.
    pub containment_fact_count: usize,
    /// Number of missing query results.
    pub missing_result_count: usize,
    /// Number of extra query results.
    pub extra_result_count: usize,
    /// Number of result keys that did not match the query order.
    pub key_mismatch_count: usize,
    /// Number of strict contains decisions.
    pub contains_count: usize,
    /// Number of certified outside decisions.
    pub outside_count: usize,
    /// Number of boundary decisions that still need resolution.
    pub boundary_count: usize,
    /// Number of unknown decisions that still need a stronger predicate.
    pub unknown_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// One externally certified nesting-depth fact for a Bezier/conic output loop.
///
/// The fact is keyed by output-loop index so a future containment/nesting
/// stage cannot accidentally hand the boolean region builder a permuted depth
/// list. This is the same certified-combinatorics boundary advocated by Yap,
/// "Towards Exact Geometric Computation" (1997): exact containment predicates
/// may produce a loop-depth fact, but topology assembly validates that the fact
/// names the loop it classifies before using it.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BezierBooleanLoopNestingDepthFact2 {
    /// Index into [`BezierBooleanOutputLoopReport2::loops`].
    pub loop_index: usize,
    /// Certified number of containing output loops.
    pub nesting_depth: usize,
}

/// One externally certified containment relation between output loops.
///
/// `container_loop_index` names the loop certified to contain
/// `contained_loop_index`. The fact is deliberately index-keyed so a future
/// exact containment predicate can hand topology to the boolean stack without
/// relying on loop vector order or sample points. This follows Yap,
/// "Towards Exact Geometric Computation" (1997): containment is a certified
/// predicate result, and construction validates its object keys before use.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BezierBooleanLoopContainmentFact2 {
    /// Index of the containing loop in [`BezierBooleanOutputLoopReport2::loops`].
    pub container_loop_index: usize,
    /// Index of the contained loop in [`BezierBooleanOutputLoopReport2::loops`].
    pub contained_loop_index: usize,
}

/// Validation status for Bezier/conic loop containment facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopContainmentFactStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every containment fact is keyed to existing, distinct output loops.
    Ready,
    /// Output-loop packaging was blocked.
    OutputLoopBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// A supplied containment fact names a missing output loop.
    OutOfRangeLoopIndex,
    /// A supplied containment fact claims a loop contains itself.
    SelfContainment,
    /// The same ordered containment relation was supplied more than once.
    DuplicateContainmentFact,
    /// The supplied containment graph contains a directed cycle.
    CyclicContainmentFacts,
    /// Two certified containers of the same loop are not nested with each other.
    NonLaminarContainmentFacts,
}

/// Validated containment facts and derived nesting depths for output loops.
///
/// This is a certificate-validation layer, not a geometric containment solver.
/// A future exact loop-containment predicate supplies pair facts; this report
/// validates their loop indices, rejects self-containment, duplicate ordered
/// pairs, directed cycles, and non-laminar shared containers, and derives one
/// nesting-depth fact per output loop by counting transitive certified
/// containers. A cycle would claim that distinct Jordan loops mutually contain
/// each other, which is impossible for exact containment. If two loops both
/// contain a third loop, exact Jordan containment requires those two containers
/// to be nested with each other; otherwise the certificate is describing
/// crossing/overlapping filled interiors that should have been resolved before
/// nesting. Transitive depth derivation lets an exact locator provide
/// immediate-parent facts without also enumerating every ancestor relation. The
/// separation matches the nesting/fill phases of Vatti (1992),
/// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009). Yap (1997) is the
/// exactness rule: invalid or stale containment facts block construction
/// instead of being repaired with orientation or bounding-box guesses.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopContainmentFactReport2 {
    /// Coarse containment-fact validation status.
    pub status: BezierBooleanLoopContainmentFactStatus,
    /// Output-loop status used to derive this report.
    pub output_status: BezierBooleanOutputLoopStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Number of output loops requiring nesting depths.
    pub output_loop_count: usize,
    /// Number of containment facts supplied by the caller.
    pub supplied_fact_count: usize,
    /// Validated containment facts.
    pub facts: Vec<BezierBooleanLoopContainmentFact2>,
    /// Derived keyed nesting-depth facts in output-loop order.
    pub depth_facts: Vec<BezierBooleanLoopNestingDepthFact2>,
    /// Number of out-of-range loop references.
    pub out_of_range_fact_count: usize,
    /// Number of self-containment facts.
    pub self_containment_count: usize,
    /// Number of duplicate ordered containment facts.
    pub duplicate_fact_count: usize,
    /// Number of detected directed containment cycles.
    pub cyclic_fact_count: usize,
    /// Number of non-laminar container-pair conflicts.
    pub non_laminar_fact_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Certification status for keyed Bezier/conic loop nesting-depth facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopNestingDepthFactStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every output loop has a keyed nesting-depth fact.
    Ready,
    /// Output-loop packaging was blocked.
    OutputLoopBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// At least one output loop has no supplied depth fact.
    MissingNestingDepthFacts,
    /// More depth facts were supplied than output loops.
    ExtraNestingDepthFacts,
    /// A supplied depth fact is not keyed to the corresponding output loop.
    LoopIndexMismatch,
}

/// Validated nesting-depth facts for Bezier/conic output loops.
///
/// This report is the fact-validation seam before
/// [`BezierBooleanLoopNestingRoleReport2`]. It accepts externally certified
/// containment output only when each fact is keyed to the deterministic output
/// loop index, then exposes an ordered depth vector for parity-based
/// material/hole assignment. Vatti (1992), Greiner-Hormann (1998), and
/// Martinez-Rueda-Feito (2009) all separate contour assembly from fill/nesting
/// classification. Yap (1997) supplies the exactness rule used here: missing,
/// extra, or permuted nesting facts are blockers, not opportunities for
/// orientation or sample-point inference.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopNestingDepthFactReport2 {
    /// Coarse nesting-depth fact certification status.
    pub status: BezierBooleanLoopNestingDepthFactStatus,
    /// Output-loop status used to derive this report.
    pub output_status: BezierBooleanOutputLoopStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Number of output loops requiring depth facts.
    pub output_loop_count: usize,
    /// Number of depth facts supplied by the caller.
    pub supplied_fact_count: usize,
    /// Facts accepted in output-loop order.
    pub facts: Vec<BezierBooleanLoopNestingDepthFact2>,
    /// Nesting depths accepted in output-loop order.
    pub depths: Vec<usize>,
    /// Number of missing nesting-depth facts.
    pub missing_fact_count: usize,
    /// Number of extra nesting-depth facts.
    pub extra_fact_count: usize,
    /// Number of keyed facts that do not match output-loop order.
    pub loop_index_mismatch_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Status for generating Bezier/conic output-loop roles from nesting-depth facts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopNestingRoleStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every output loop received a parity role from a certified nesting depth.
    Ready,
    /// Output-loop packaging was blocked before role generation.
    OutputLoopBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// Fewer nesting-depth facts than output loops were supplied.
    MissingNestingDepthFacts,
    /// More nesting-depth facts than output loops were supplied.
    ExtraNestingDepthFacts,
}

/// Generated material/hole roles from certified loop nesting depths.
///
/// This report does not compute containment. Instead, it consumes nesting
/// depths supplied by a certified containment/nesting stage and maps even
/// depth to material, odd depth to hole under the usual alternating
/// material/hole convention. This mirrors the contour-nesting phase used after
/// boundary construction in Vatti (1992), Greiner-Hormann (1998), and
/// Martinez-Rueda-Feito (2009). Following Yap, "Towards Exact Geometric
/// Computation" (1997), missing or extra depth facts are blockers rather than
/// an invitation to infer roles from orientation, bounding boxes, or samples.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopNestingRoleReport2 {
    /// Coarse nesting-role generation status.
    pub status: BezierBooleanLoopNestingRoleStatus,
    /// Output-loop packaging status used to derive this report.
    pub output_status: BezierBooleanOutputLoopStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Roles generated from nesting-depth parity.
    pub roles: Vec<BezierBooleanOutputLoopRole>,
    /// Number of output loops requiring depth facts.
    pub output_loop_count: usize,
    /// Number of nesting-depth facts supplied by the caller.
    pub supplied_depth_count: usize,
    /// Number of material loops generated.
    pub material_loop_count: usize,
    /// Number of hole loops generated.
    pub hole_loop_count: usize,
    /// Number of missing depth facts.
    pub missing_depth_count: usize,
    /// Number of extra depth facts.
    pub extra_depth_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Certified fill role for a closed Bezier/conic output loop.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanOutputLoopRole {
    /// The loop bounds filled material.
    Material,
    /// The loop bounds a hole inside material.
    Hole,
    /// The nesting/containment stage has not certified this loop role.
    Unknown,
}

/// Role-assignment status for closed Bezier/conic output loops.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanLoopRoleAssignmentStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Every closed output loop has a certified material or hole role.
    Ready,
    /// Output-loop packaging was blocked before role assignment.
    OutputLoopBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// Fewer role facts than output loops were supplied.
    MissingRoleFacts,
    /// More role facts than output loops were supplied.
    ExtraRoleFacts,
    /// At least one supplied role fact is explicit unknown.
    UnknownRole,
    /// The keyed nesting-depth facts needed to certify role parity were blocked.
    NestingDepthFactBlocked,
    /// At least one supplied role disagrees with certified nesting-depth parity.
    RoleParityMismatch,
}

/// One closed Bezier/conic output loop with a certified material/hole role.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanAssignedOutputLoop2 {
    /// Closed output-loop range over directed fragments.
    pub output_loop: BezierBooleanOutputLoop2,
    /// Certified role assigned by a separate nesting/containment stage.
    pub role: BezierBooleanOutputLoopRole,
}

/// Report-bearing material/hole role assignment for Bezier/conic output loops.
///
/// This report consumes [`BezierBooleanOutputLoopReport2`] and externally
/// certified loop-role facts. It does not infer roles from orientation,
/// bounding boxes, or sampled points. That is intentional: Vatti (1992),
/// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009) all require a
/// topological interpretation phase after boundary construction, and Yap,
/// "Towards Exact Geometric Computation" (1997), requires that phase to supply
/// certified combinatorial facts or explicit uncertainty. Missing, extra, and
/// unknown role facts therefore remain blockers instead of becoming guessed
/// filled-region topology.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanLoopRoleAssignmentReport2 {
    /// Coarse role-assignment status.
    pub status: BezierBooleanLoopRoleAssignmentStatus,
    /// Output-loop packaging status used to derive this report.
    pub output_status: BezierBooleanOutputLoopStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Directed endpoint payloads retained from output-loop packaging.
    pub directed_fragments: Vec<BezierBooleanDirectedLoopFragment2>,
    /// Output loops paired with certified roles.
    pub assigned_loops: Vec<BezierBooleanAssignedOutputLoop2>,
    /// Number of output loops requiring role facts.
    pub output_loop_count: usize,
    /// Number of role facts supplied by the caller.
    pub supplied_role_count: usize,
    /// Number of material loops.
    pub material_loop_count: usize,
    /// Number of hole loops.
    pub hole_loop_count: usize,
    /// Number of explicit unknown role facts.
    pub unknown_role_count: usize,
    /// Number of roles that disagree with certified nesting-depth parity.
    pub role_parity_mismatch_count: usize,
    /// Number of missing role facts.
    pub missing_role_count: usize,
    /// Number of extra role facts.
    pub extra_role_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Region-assembly status for role-assigned Bezier/conic output loops.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanRegionAssemblyStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Role-assigned loops are packaged for a future higher-order region object.
    Ready,
    /// Loop role assignment was blocked.
    RoleAssignmentBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
    /// Hole loops exist without a certified material loop to contain them.
    HoleWithoutMaterial,
}

/// Report-bearing region-assembly handoff for Bezier/conic booleans.
///
/// This is the final report-only packaging layer before a future higher-order
/// region type can own polynomial Bezier and rational conic contours. It
/// consumes [`BezierBooleanLoopRoleAssignmentReport2`] and exposes material and
/// hole loop indices over the role-assigned loop array. It deliberately does
/// not attach holes to material owners or infer containment. That certification
/// belongs to a separate nesting stage, as in Vatti (1992), Greiner-Hormann
/// (1998), and Martinez-Rueda-Feito (2009). Yap, "Towards Exact Geometric
/// Computation" (1997), is the exactness contract: a hole-only result is a
/// blocker, not a guessed filled region.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanRegionAssemblyReport2 {
    /// Coarse region-assembly status.
    pub status: BezierBooleanRegionAssemblyStatus,
    /// Role-assignment status used to derive this report.
    pub role_status: BezierBooleanLoopRoleAssignmentStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Directed endpoint payloads retained from role assignment.
    pub directed_fragments: Vec<BezierBooleanDirectedLoopFragment2>,
    /// Role-assigned output loops retained for future region materialization.
    pub assigned_loops: Vec<BezierBooleanAssignedOutputLoop2>,
    /// Indices of material loops in `assigned_loops`.
    pub material_loop_indices: Vec<usize>,
    /// Indices of hole loops in `assigned_loops`.
    pub hole_loop_indices: Vec<usize>,
    /// Number of assigned loops retained.
    pub assigned_loop_count: usize,
    /// Number of material loops.
    pub material_loop_count: usize,
    /// Number of hole loops.
    pub hole_loop_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Accepted-result status for Bezier/conic boolean output artifacts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanResultStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// A role-assigned higher-order loop artifact is accepted.
    Ready,
    /// Region assembly is blocked.
    RegionAssemblyBlocked,
    /// Ownership was certified but no fragments are emitted.
    NoEmittedFragments,
}

/// Final report-bearing accepted artifact for Bezier/conic booleans.
///
/// This report is intentionally not a [`Region2`](crate::Region2): the current
/// concrete region type owns line/arc contours, while this artifact retains
/// higher-order Bezier/conic loop topology. It is the exact acceptance boundary
/// for the Bezier/conic boolean stack: split/ownership/loop/role facts have
/// been certified, material and hole loop index sets are present, and blockers
/// remain explicit. The shape follows Yap, "Towards Exact Geometric
/// Computation" (1997): exact combinatorial output is accepted only with
/// certified prerequisites. The phase separation follows Vatti (1992),
/// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanResultReport2 {
    /// Coarse accepted-result status.
    pub status: BezierBooleanResultStatus,
    /// Region-assembly status used to derive this report.
    pub assembly_status: BezierBooleanRegionAssemblyStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Directed endpoint payloads retained from region assembly.
    pub directed_fragments: Vec<BezierBooleanDirectedLoopFragment2>,
    /// Role-assigned output loops retained as the higher-order artifact.
    pub assigned_loops: Vec<BezierBooleanAssignedOutputLoop2>,
    /// Indices of material loops in `assigned_loops`.
    pub material_loop_indices: Vec<usize>,
    /// Indices of hole loops in `assigned_loops`.
    pub hole_loop_indices: Vec<usize>,
    /// Number of assigned loops retained.
    pub assigned_loop_count: usize,
    /// Number of material loops.
    pub material_loop_count: usize,
    /// Number of hole loops.
    pub hole_loop_count: usize,
    /// Number of directed fragments retained.
    pub directed_fragment_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Materialization-readiness audit status for accepted Bezier/conic results.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanMaterializationAuditStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// The accepted result is internally consistent and can feed a future region carrier.
    Ready,
    /// The result was blocked before materialization-readiness auditing.
    ResultBlocked,
    /// The result's public count fields do not match the retained payloads.
    CountMismatch,
    /// Material or hole loop index sets are stale, duplicate, or out of range.
    RoleIndexMismatch,
    /// A role-index set points at a loop whose certified role disagrees.
    LoopRoleMismatch,
    /// At least one output-loop range does not fit inside the retained directed fragments.
    FragmentRangeMismatch,
}

/// Audit report for future Bezier/conic region materialization.
///
/// This report is the current safe endpoint for "actual region" work while
/// `Region2` remains a line/arc contour carrier. It consumes an accepted
/// [`BezierBooleanResultReport2`] and replays its internal invariants before a
/// future higher-order Bezier/conic region type may trust the payload: public
/// counts must match retained vectors, material/hole index sets must be keyed
/// to existing assigned loops, indexed loops must have matching certified
/// roles, and every loop range must be contained in the retained directed
/// fragments. It performs no containment, winding, or orientation inference.
/// That preserves Yap's "Towards Exact Geometric Computation" (1997)
/// predicate/construction boundary, and keeps the post-boundary fill stage
/// explicit in the style of Vatti (1992), Greiner-Hormann (1998), and
/// Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanMaterializationAuditReport2 {
    /// Coarse materialization-readiness status.
    pub status: BezierBooleanMaterializationAuditStatus,
    /// Result status used to derive this audit.
    pub result_status: BezierBooleanResultStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Number of assigned loops retained by the result payload.
    pub assigned_loop_count: usize,
    /// Number of material loops retained by the result payload.
    pub material_loop_count: usize,
    /// Number of hole loops retained by the result payload.
    pub hole_loop_count: usize,
    /// Number of directed fragments retained by the result payload.
    pub directed_fragment_count: usize,
    /// Number of assigned loops whose range was checked.
    pub audited_loop_count: usize,
    /// Number of public count fields that disagree with retained payload sizes.
    pub count_mismatch_count: usize,
    /// Number of stale, duplicate, or out-of-range material/hole indices.
    pub role_index_mismatch_count: usize,
    /// Number of material/hole indices whose loop role disagrees with the index set.
    pub loop_role_mismatch_count: usize,
    /// Number of assigned-loop fragment ranges that do not fit the directed payload.
    pub fragment_range_mismatch_count: usize,
    /// Number of blocking preconditions retained by this audit.
    pub blocker_count: usize,
}

/// Materialized higher-order Bezier/conic region status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanMaterializedRegionStatus {
    /// No fragment or overlap work was supplied.
    Empty,
    /// The arrangement is a certified endpoint-only no-op.
    NoInteriorSplits,
    /// Material loops and their certified hole children are grouped.
    Ready,
    /// The accepted-result audit blocked materialization.
    AuditBlocked,
    /// At least one hole loop lacks a certified material container.
    MissingHoleContainment,
    /// At least one hole loop has more than one certified material container.
    AmbiguousHoleContainment,
    /// A containment fact is duplicate, self-containing, or names a missing loop.
    StaleContainmentFact,
    /// A containment fact does not connect a material loop to a hole loop.
    RoleIncompatibleContainment,
}

/// One material loop plus the hole loops certified to belong to it.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanMaterializedComponent2 {
    /// Index of the material loop in [`BezierBooleanResultReport2::assigned_loops`].
    pub material_loop_index: usize,
    /// Hole-loop indices in [`BezierBooleanResultReport2::assigned_loops`].
    pub hole_loop_indices: Vec<usize>,
}

/// Report-bearing higher-order Bezier/conic region materialization.
///
/// This is a topology carrier for polynomial Bezier and rational conic boolean
/// output, not a conversion to [`Region2`](crate::Region2). It first audits the
/// accepted result with [`BezierBooleanMaterializationAuditReport2`], then
/// consumes explicit containment facts to attach each certified hole loop to
/// exactly one certified material loop. It does not infer containment from
/// orientation, winding, bounding boxes, or sample points. That is the
/// object-level exactness boundary required by Yap, "Towards Exact Geometric
/// Computation" (1997). The material/hole attachment stage mirrors the
/// post-boundary fill interpretation used by Vatti (1992), Greiner-Hormann
/// (1998), and Martinez-Rueda-Feito (2009).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanMaterializedRegionReport2 {
    /// Coarse materialized-region status.
    pub status: BezierBooleanMaterializedRegionStatus,
    /// Audit status used before containment attachment.
    pub audit_status: BezierBooleanMaterializationAuditStatus,
    /// Requested boolean operation.
    pub operation: BooleanOp,
    /// Directed endpoint payloads retained from the accepted result.
    pub directed_fragments: Vec<BezierBooleanDirectedLoopFragment2>,
    /// Role-assigned output loops retained from the accepted result.
    pub assigned_loops: Vec<BezierBooleanAssignedOutputLoop2>,
    /// Material components with certified child holes.
    pub components: Vec<BezierBooleanMaterializedComponent2>,
    /// Number of material components emitted.
    pub component_count: usize,
    /// Number of containment facts supplied.
    pub supplied_containment_count: usize,
    /// Number of hole loops not attached to a material loop.
    pub missing_hole_containment_count: usize,
    /// Number of hole loops attached to more than one material loop.
    pub ambiguous_hole_containment_count: usize,
    /// Number of duplicate, self-containing, or out-of-range containment facts.
    pub stale_containment_count: usize,
    /// Number of containment facts whose endpoint roles are incompatible.
    pub role_incompatible_containment_count: usize,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Machine-readable handoff from Bezier intersection predicates to booleans.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanHandoffReport2 {
    /// Coarse readiness state.
    pub status: BezierBooleanHandoffStatus,
    /// Exact split-ready point events.
    pub point_events: Vec<BezierBooleanPointEvent2>,
    /// Exact overlap ranges that require an overlap resolver.
    pub overlap_events: Vec<BezierBooleanOverlapEvent2>,
    /// Retained regions summarized for algebraic isolation.
    pub region_summary: Option<BezierIntersectionRegionSummary>,
    /// Optional retained isolation certificate used to classify a region frontier.
    pub isolation_certificate: Option<BezierIntersectionRegionIsolationCertificate>,
    /// Certified point witnesses that lack exact curve parameters.
    pub point_witnesses_needing_parameters: usize,
    /// Count of overlapping/same-image relation cases.
    pub overlap_relations_needing_resolution: usize,
    /// Count of unresolved predicate branches.
    pub unresolved_relations: usize,
    /// Count of lower-level uncertain primitive branches.
    pub uncertain_relations: usize,
    /// Explicit primitive uncertainty reason, when one was retained.
    pub uncertainty_reason: Option<UncertaintyReason>,
}

/// Aggregate boolean-readiness state for Bezier relation handoff reports.
///
/// A contour/path boolean receives many curve-pair relation reports after
/// broad-phase pruning. This status folds those reports into the conservative
/// scheduling state required by an arrangement builder. The precedence keeps
/// Yap's exact-computation contract intact: explicit uncertainty and unresolved
/// predicates outrank split-ready events, and overlap/region/parameter blockers
/// remain visible instead of being interpreted by tolerance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanBatchHandoffStatus {
    /// No relation reports were supplied.
    Empty,
    /// Every relation certifies that no split events are required.
    NoEvents,
    /// Every required split event has exact parameters and no blocker remains.
    SplitEventsReady,
    /// At least one certified point witness still needs parameter recovery.
    NeedsParameterRecovery,
    /// At least one overlap or same-image relation needs an overlap resolver.
    NeedsOverlapResolver,
    /// At least one retained positive-width region needs algebraic isolation.
    NeedsRegionIsolation,
    /// At least one relation is explicitly unresolved.
    Unresolved,
    /// At least one lower predicate reported explicit uncertainty.
    Uncertain,
}

/// Batch handoff from Bezier relation predicates to a boolean arrangement.
///
/// This report performs no new geometry. It aggregates
/// [`BezierBooleanHandoffReport2`] values produced by curve/curve predicates,
/// retaining exact split events and exact overlap ranges while counting every
/// blocker category. Greiner-Hormann (1998) and Martinez-Rueda-Feito (2009)
/// both rely on an intersection-insertion stage before traversal; this report
/// is the Bezier/conic-facing scheduler contract for that stage. Sederberg and
/// Nishita's Bezier clipping regions remain isolation obligations until they
/// produce represented parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanBatchHandoffReport2 {
    /// Coarse batch readiness state.
    pub status: BezierBooleanBatchHandoffStatus,
    /// Number of relation reports consumed.
    pub relation_count: usize,
    /// Number of reports that certified no events.
    pub no_event_relation_count: usize,
    /// Number of reports that were individually split-ready.
    pub split_ready_relation_count: usize,
    /// Exact point events retained from split-ready reports.
    pub point_events: Vec<BezierBooleanPointEvent2>,
    /// Exact overlap ranges retained from overlap reports.
    pub overlap_events: Vec<BezierBooleanOverlapEvent2>,
    /// Total certified point witnesses that still need parameter recovery.
    pub point_witnesses_needing_parameters: usize,
    /// Total overlap/same-image relation obligations.
    pub overlap_relations_needing_resolution: usize,
    /// Number of relation reports that retained positive-width regions.
    pub region_isolation_relation_count: usize,
    /// Number of unresolved relation reports.
    pub unresolved_relations: usize,
    /// Number of relation reports with primitive uncertainty.
    pub uncertain_relations: usize,
    /// First explicit primitive uncertainty reason retained by the batch.
    pub uncertainty_reason: Option<UncertaintyReason>,
}

/// Root-isolation handoff status for Bezier/conic boolean predicates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanRootIsolationHandoffStatus {
    /// No relation reports were supplied.
    Empty,
    /// No root-isolation work is required.
    NotNeeded,
    /// The supplied frontier already has represented split events.
    SplitEventsReady,
    /// Retained algebraic regions are ready to be delegated to exact root isolation.
    ReadyForHypersolve,
    /// Exact curve parameters must be recovered before root isolation can run.
    BlockedByParameterRecovery,
    /// Same-image or overlap relations must be resolved before root isolation.
    BlockedByOverlapResolver,
    /// The relation is still unresolved and has no retained isolation frontier.
    BlockedByUnresolved,
    /// Represented roots still need crossing/tangency classification.
    BlockedByContactClassification,
    /// A monotone range still needs decomposition or a certified graph axis.
    BlockedByMonotoneDecomposition,
    /// A lower predicate reported explicit uncertainty.
    BlockedByUncertainty,
}

/// Report that hands retained Bezier/conic regions to exact root isolation.
///
/// This report is the seam between `hypercurve` path booleans and the exact
/// root-isolation machinery now available in `hypersolve`. It does not call a
/// solver and it never upgrades an isolating box into topology. Instead, it
/// audits whether the current boolean handoff contains retained algebraic
/// regions or an isolation certificate that can be delegated to a Sturm/
/// Collins-Loos style exact real-root isolator. This preserves Yap's "Towards
/// Exact Geometric Computation" (1997) predicate/construction boundary:
/// isolation may propose certified roots, but split insertion is accepted only
/// after represented split events or exact-point certificates are returned.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanRootIsolationHandoffReport2 {
    /// Coarse root-isolation handoff status.
    pub status: BezierBooleanRootIsolationHandoffStatus,
    /// Number of relation handoff reports consumed.
    pub relation_count: usize,
    /// Number of relations already split-ready.
    pub split_ready_relation_count: usize,
    /// Number of relations with retained region-isolation obligations.
    pub region_isolation_relation_count: usize,
    /// Number of retained isolation certificates.
    pub isolation_certificate_count: usize,
    /// Number of retained certificates whose terminal cells are exact points.
    pub exact_point_certificate_count: usize,
    /// Number of terminal region cells that still need exact root isolation.
    pub terminal_region_count: usize,
    /// Number of monotone-range isolating spans that still need exact root isolation.
    pub range_isolating_span_count: usize,
    /// Number of point witnesses that still need parameter recovery.
    pub point_witnesses_needing_parameters: usize,
    /// Number of represented contact parameters still needing classification.
    pub unclassified_parameter_count: usize,
    /// Number of overlap relations that must be resolved first.
    pub overlap_relations_needing_resolution: usize,
    /// Number of monotone ranges still needing decomposition or axis proof.
    pub not_shared_monotone_range_count: usize,
    /// Number of unresolved relation blockers.
    pub unresolved_relations: usize,
    /// Number of uncertain relation blockers.
    pub uncertain_relations: usize,
    /// First explicit primitive uncertainty reason retained by the handoff.
    pub uncertainty_reason: Option<UncertaintyReason>,
    /// Number of blocking preconditions retained by this report.
    pub blocker_count: usize,
}

/// Combined Bezier path-boolean scheduler status.
///
/// Bezier path booleans need both relation-level intersection facts and
/// monotone-range ordering facts before they can safely enter split insertion
/// and traversal. This status combines those two report families without
/// inventing topology. Yap's exact-geometric-computation model requires the
/// whole geometric object to expose certified combinatorics or explicit
/// uncertainty; a split-ready relation event is therefore not sufficient when a
/// monotone range still has an overlap, isolation, or uncertainty blocker.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanPathSchedulerStatus {
    /// No relation or monotone-range work was supplied.
    Empty,
    /// Both batches certify that no split insertion is needed.
    NoEvents,
    /// All required split parameters are represented and no blocker remains.
    SplitEventsReady,
    /// A point witness still needs exact curve parameters.
    NeedsParameterRecovery,
    /// A represented contact still needs crossing/tangency classification.
    NeedsContactClassification,
    /// A retained algebraic region or monotone isolating span needs refinement.
    NeedsRegionIsolation,
    /// A same-image, finite-overlap, or coincident monotone range needs an
    /// overlap resolver.
    NeedsOverlapResolver,
    /// A range still needs monotone decomposition or a certified graph axis.
    NeedsMonotoneDecomposition,
    /// A relation predicate was explicitly unresolved.
    Unresolved,
    /// A lower predicate reported explicit uncertainty.
    Uncertain,
}

/// Combined scheduler report for Bezier/conic path boolean construction.
///
/// This report joins [`BezierBooleanBatchHandoffReport2`] and
/// [`BezierPathRangeBatchReport2`] into the final boolean-facing scheduling
/// layer before split insertion. It intentionally runs no new predicate: it
/// preserves exact relation split events, exact monotone contact parameters,
/// and the first uncertainty reason while deriving a conservative global
/// status. Greiner-Hormann (1998) and Martinez-Rueda-Feito (2009) both assume
/// classified intersections are inserted before traversal; this report states
/// whether the Bezier/conic predicate frontier has enough certified data for
/// that stage. Foster-Hormann-Popa (2019) motivates keeping overlap blockers
/// explicit instead of treating coincident ranges as ordinary crossings.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanPathSchedulerReport2 {
    /// Coarse global scheduler status.
    pub status: BezierBooleanPathSchedulerStatus,
    /// Relation-level aggregate report.
    pub relation_batch: BezierBooleanBatchHandoffReport2,
    /// Monotone-range aggregate report.
    pub range_batch: BezierPathRangeBatchReport2,
    /// Exact relation point events ready for split insertion.
    pub relation_point_events: Vec<BezierBooleanPointEvent2>,
    /// Exact monotone contact parameters ready for split insertion.
    pub range_split_parameters: Vec<Real>,
    /// Total split insertion candidates represented exactly.
    pub represented_split_event_count: usize,
    /// First explicit primitive uncertainty reason retained by either batch.
    pub uncertainty_reason: Option<UncertaintyReason>,
}

/// Split-insertion readiness for a scheduled Bezier path-boolean frontier.
///
/// This is the last report-only stage before a future contour implementation
/// mutates curve fragments. It follows Yap's separation between certified
/// predicate facts and geometric construction: only a globally
/// [`BezierBooleanPathSchedulerStatus::SplitEventsReady`] scheduler can produce
/// a ready split plan. All other scheduler states remain explicit blockers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanSplitPlanStatus {
    /// No split work exists.
    Empty,
    /// Exact split parameters are represented and may be inserted.
    Ready,
    /// The scheduler still has a blocker, so no insertion plan is trusted.
    Blocked,
}

/// Exact split-parameter plan for future Bezier/conic boolean fragments.
///
/// The plan extracts exact parameters from a certified
/// [`BezierBooleanPathSchedulerReport2`] without evaluating curves or resolving
/// overlaps. Relation point events contribute per-operand parameters, while
/// monotone graph contacts contribute shared range parameters that future
/// range-fragment code can map into local curve spans. Greiner-Hormann (1998)
/// and Martinez-Rueda-Feito (2009) both require intersection insertion before
/// boolean traversal; this struct is the Bezier/conic handoff for that
/// insertion stage. Overlaps and unresolved Bezier-clipping regions remain
/// blockers as required by Yap (1997) and by Foster-Hormann-Popa's degenerate
/// clipping analysis (2019).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanSplitPlanReport2 {
    /// Split-plan readiness.
    pub status: BezierBooleanSplitPlanStatus,
    /// Scheduler status used to derive this plan.
    pub scheduler_status: BezierBooleanPathSchedulerStatus,
    /// Exact parameters to insert on the first relation operand.
    pub first_curve_parameters: Vec<Real>,
    /// Exact parameters to insert on the second relation operand.
    pub second_curve_parameters: Vec<Real>,
    /// Exact shared monotone-range parameters from graph contacts.
    pub shared_range_parameters: Vec<Real>,
    /// Number of relation point events represented in the plan.
    pub relation_event_count: usize,
    /// Number of shared-range split parameters represented in the plan.
    pub range_event_count: usize,
    /// First explicit primitive uncertainty reason retained by the scheduler.
    pub uncertainty_reason: Option<UncertaintyReason>,
}

/// Curve-side role for a represented algebraic split parameter.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanAlgebraicParameterRole {
    /// Parameter belongs to the first relation operand.
    FirstCurve,
    /// Parameter belongs to the second relation operand.
    SecondCurve,
    /// Parameter belongs to a shared monotone range.
    SharedRange,
}

/// Exact algebraic parameter evidence for future Bezier/conic split insertion.
///
/// This carrier is the first persistent non-rational parameter handoff for the
/// boolean pipeline. It stores the complete `hypersolve` represented root
/// object, including the defining polynomial and certified isolating interval,
/// instead of forcing the root through a primitive approximation. The current
/// fragment mutators still accept only exact rational `Real` parameters, but
/// later ordering, overlap, containment, and loop assembly reports can consume
/// this object directly. This follows Yap, "Towards Exact Geometric
/// Computation" (1997), and the Collins-Loos real-root representation model.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanAlgebraicParameterEvent2 {
    /// Which split-parameter lane this root belongs to.
    pub role: BezierBooleanAlgebraicParameterRole,
    /// Source algebraic-root report ordinal.
    pub report_index: usize,
    /// Source root ordinal inside the algebraic-root report.
    pub root_index: usize,
    /// Persistent exact algebraic-root evidence from `hypersolve`.
    pub root: AlgebraicRootRepresentation,
}

/// Algebraic-parameter handoff status for Bezier/conic boolean construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanAlgebraicParameterHandoffStatus {
    /// No scheduler or algebraic parameter work exists.
    Empty,
    /// The scheduler does not require algebraic root parameters.
    NotNeeded,
    /// The scheduler already has exact rational split events.
    RationalSplitEventsReady,
    /// Algebraic parameter objects are available for future split/order stages.
    Ready,
    /// An earlier scheduler or root-isolation handoff blocker remains.
    HandoffBlocked,
    /// Not enough represented roots were supplied for the retained frontier.
    MissingAlgebraicRoots,
    /// A represented root failed validation or came from an unsupported row.
    InvalidAlgebraicEvidence,
    /// At least one represented root is outside the Bezier unit domain.
    InvalidParameterDomain,
}

/// Report that promotes represented algebraic roots into curve-parameter evidence.
///
/// Unlike [`BezierBooleanRootIsolationReplayReport2`], this report does not
/// require exact rational witnesses. It accepts valid non-rational isolating
/// intervals as first-class parameter evidence, provided the entire isolating
/// interval is certified inside the Bezier unit domain. It deliberately does
/// not feed today's rational-only fragment splitters. The point is to create
/// the missing exact object required by Yap's EGC contract before future
/// algebraic ordering and splitting layers are allowed to construct topology.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanAlgebraicParameterHandoffReport2 {
    /// Coarse algebraic-parameter handoff status.
    pub status: BezierBooleanAlgebraicParameterHandoffStatus,
    /// Root-isolation handoff status that authorized or blocked this handoff.
    pub handoff_status: BezierBooleanRootIsolationHandoffStatus,
    /// Scheduler status before algebraic-parameter replay.
    pub scheduler_status: BezierBooleanPathSchedulerStatus,
    /// Algebraic parameter events retained for future exact split/order stages.
    pub events: Vec<BezierBooleanAlgebraicParameterEvent2>,
    /// Number of algebraic root obligations retained by the handoff.
    pub required_algebraic_parameter_count: usize,
    /// Number of represented algebraic roots supplied.
    pub supplied_algebraic_parameter_count: usize,
    /// Number of exact rational roots among retained events.
    pub exact_rational_parameter_count: usize,
    /// Number of non-rational interval roots among retained events.
    pub interval_parameter_count: usize,
    /// Number of still-missing represented algebraic roots.
    pub missing_algebraic_parameter_count: usize,
    /// Number of invalid/unsupported algebraic reports or roots.
    pub invalid_algebraic_evidence_count: usize,
    /// Number of represented roots outside `[0, 1]`.
    pub out_of_range_parameter_count: usize,
    /// Number of retained blocking preconditions.
    pub blocker_count: usize,
    /// First explicit primitive uncertainty reason retained by the scheduler.
    pub uncertainty_reason: Option<UncertaintyReason>,
}

/// Audit status for represented algebraic Bezier parameters.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanAlgebraicParameterAuditStatus {
    /// No scheduler or algebraic parameter work exists.
    Empty,
    /// The scheduler did not need algebraic parameter evidence.
    NotNeeded,
    /// Existing rational split events already satisfy the scheduler.
    RationalSplitEventsReady,
    /// Every retained algebraic event is structurally valid for later exact stages.
    Valid,
    /// The handoff was blocked before algebraic parameter auditing.
    HandoffBlocked,
    /// The handoff's public counts disagree with retained events.
    CountMismatch,
    /// An event role is not supported by the current univariate handoff.
    UnsupportedRole,
    /// An event carries invalid algebraic-root evidence.
    InvalidAlgebraicEvidence,
    /// An event's isolating interval is outside the Bezier unit domain.
    InvalidParameterDomain,
}

/// Structural audit for algebraic parameter events.
///
/// This report is the algebraic counterpart to
/// [`BezierBooleanSplitPlanAuditReport2`]. It validates that a ready
/// [`BezierBooleanAlgebraicParameterHandoffReport2`] has internally consistent
/// counts and only unit-domain, validated represented roots before later
/// ordering, overlap, containment, or split layers consume those events. It
/// does not compare algebraic roots to each other; algebraic ordering remains
/// a later `hypersolve`/`hyperlimit` certificate. Yap (1997) is the governing
/// rule: exact algebraic evidence is preserved as an object and audited before
/// construction, not sampled into primitive floats.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BezierBooleanAlgebraicParameterAuditReport2 {
    /// Final audit status.
    pub status: BezierBooleanAlgebraicParameterAuditStatus,
    /// Handoff status used to derive this audit.
    pub handoff_status: BezierBooleanAlgebraicParameterHandoffStatus,
    /// Number of retained algebraic parameter events checked.
    pub checked_event_count: usize,
    /// Number of public count mismatches found.
    pub count_mismatch_count: usize,
    /// Number of event roles unsupported by the current univariate handoff.
    pub unsupported_role_count: usize,
    /// Number of invalid represented-root payloads.
    pub invalid_algebraic_evidence_count: usize,
    /// Number of represented-root intervals outside `[0, 1]`.
    pub out_of_range_parameter_count: usize,
    /// Number of retained blocking preconditions.
    pub blocker_count: usize,
}

/// Readiness status for consuming audited algebraic parameter events.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanAlgebraicParameterReadinessStatus {
    /// No scheduler or algebraic parameter work exists.
    Empty,
    /// The scheduler did not need algebraic parameter evidence.
    NotNeeded,
    /// Existing rational split events already satisfy the scheduler.
    RationalSplitEventsReady,
    /// Audited algebraic parameter events are packaged for later exact stages.
    Ready,
    /// Algebraic-parameter audit blocked consumption.
    AuditBlocked,
}

/// Readiness package for future algebraic ordering and splitting.
///
/// This report is deliberately still non-mutating. It consumes an audited
/// [`BezierBooleanAlgebraicParameterHandoffReport2`] and separates exact
/// rational witnesses from represented interval roots while preserving the
/// original event objects. Exact rational witnesses may be bridged into the
/// existing rational split path by callers, but interval roots require a later
/// algebraic ordering/splitting certificate. This is the object-level staging
/// required by Yap (1997): exact evidence becomes explicit combinatorial data
/// before construction, and non-rational roots are never approximated.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanAlgebraicParameterReadinessReport2 {
    /// Coarse readiness status.
    pub status: BezierBooleanAlgebraicParameterReadinessStatus,
    /// Audit status used to derive readiness.
    pub audit_status: BezierBooleanAlgebraicParameterAuditStatus,
    /// Retained exact algebraic parameter events.
    pub events: Vec<BezierBooleanAlgebraicParameterEvent2>,
    /// Events with exact rational witnesses.
    pub exact_rational_events: Vec<BezierBooleanAlgebraicParameterEvent2>,
    /// Events represented only by isolating intervals.
    pub interval_events: Vec<BezierBooleanAlgebraicParameterEvent2>,
    /// Number of retained events.
    pub event_count: usize,
    /// Number of exact rational witness events.
    pub exact_rational_event_count: usize,
    /// Number of interval-only algebraic events.
    pub interval_event_count: usize,
    /// Number of retained blockers.
    pub blocker_count: usize,
}

/// Algebraic-parameter ordering status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanAlgebraicParameterOrderingStatus {
    /// No scheduler or algebraic parameter work exists.
    Empty,
    /// The scheduler did not need algebraic parameter evidence.
    NotNeeded,
    /// Existing rational split events already satisfy the scheduler.
    RationalSplitEventsReady,
    /// All retained algebraic parameters were exactly ordered.
    Ready,
    /// Algebraic-parameter readiness blocked ordering.
    ReadinessBlocked,
    /// At least one represented-root comparison did not certify an order.
    ComparisonBlocked,
}

/// One certified ordering comparison between algebraic parameter events.
///
/// The comparison is delegated to `hypersolve` represented-root comparison and
/// optional interval refinement. It records only the event indices and the
/// proof status; callers must continue to use the original event payloads for
/// construction. This follows Yap, "Towards Exact Geometric Computation"
/// (1997): ordering is a certified predicate over retained algebraic objects,
/// not an approximation of their primitive-float values. The interval
/// refinement model follows Sturm (1835) and Collins-Loos real-root isolation.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanAlgebraicParameterOrderingComparison2 {
    /// Left event index in [`BezierBooleanAlgebraicParameterReadinessReport2::events`].
    pub left_event_index: usize,
    /// Right event index in [`BezierBooleanAlgebraicParameterReadinessReport2::events`].
    pub right_event_index: usize,
    /// Final `hypersolve` comparison status.
    pub comparison_status: AlgebraicRootComparisonStatus,
    /// Certified ordering when available.
    pub ordering: Option<Ordering>,
    /// Number of alternating refinement rounds used by `hypersolve`.
    pub refinement_rounds: usize,
}

/// Certified order for retained algebraic Bezier/conic split parameters.
///
/// This report consumes algebraic parameter readiness and asks `hypersolve` to
/// compare represented roots with exact rational witnesses, disjoint isolating
/// intervals, or bounded Sturm-style interval refinement. It returns a stable
/// permutation of event indices plus pairwise comparison evidence. If any
/// comparison remains overlapping, undecided, or structurally invalid, ordering
/// blocks explicitly; no sampled parameter values enter the topology layer.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanAlgebraicParameterOrderingReport2 {
    /// Coarse ordering status.
    pub status: BezierBooleanAlgebraicParameterOrderingStatus,
    /// Readiness status used to derive ordering.
    pub readiness_status: BezierBooleanAlgebraicParameterReadinessStatus,
    /// Retained algebraic parameter events.
    pub events: Vec<BezierBooleanAlgebraicParameterEvent2>,
    /// Event indices sorted by certified parameter order.
    pub sorted_event_indices: Vec<usize>,
    /// Events sorted by certified parameter order.
    pub sorted_events: Vec<BezierBooleanAlgebraicParameterEvent2>,
    /// Pairwise comparisons used by the insertion sort.
    pub comparisons: Vec<BezierBooleanAlgebraicParameterOrderingComparison2>,
    /// Number of retained events.
    pub event_count: usize,
    /// Number of successful comparisons retained.
    pub comparison_count: usize,
    /// Number of comparisons that failed to certify an order.
    pub blocked_comparison_count: usize,
    /// Total `hypersolve` refinement rounds used.
    pub refinement_round_count: usize,
    /// Number of retained blocking preconditions.
    pub blocker_count: usize,
}

/// Status for bridging ordered algebraic parameters into today's split plan.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanAlgebraicSplitBridgeStatus {
    /// No scheduler or algebraic parameter work exists.
    Empty,
    /// The scheduler did not need algebraic parameter evidence.
    NotNeeded,
    /// Existing rational split events already satisfy the scheduler.
    RationalSplitEventsReady,
    /// Exact rational algebraic witnesses were converted into a split plan.
    Ready,
    /// Algebraic ordering blocked the bridge.
    OrderingBlocked,
    /// A retained algebraic event used an unsupported split lane.
    UnsupportedRole,
    /// At least one ordered algebraic event has no exact rational witness.
    NonRationalParameter,
}

/// Bridge from ordered represented roots to the rational split-insertion path.
///
/// Current Bezier/conic fragment splitters accept exact [`Real`] parameters.
/// This report consumes a certified algebraic ordering and lowers only exact
/// rational witnesses into [`BezierBooleanSplitPlanReport2`]. Interval-only
/// roots remain blockers until a future algebraic fragment splitter can carry
/// represented algebraic parameters directly. This is exactly the
/// predicate/construction separation Yap requires: represented roots are not
/// approximated just because a downstream construction currently accepts
/// rationals. Greiner-Hormann (1998) and Martinez-Rueda-Feito (2009) still
/// require split insertion before traversal; this bridge states when today's
/// insertion API can be used safely.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanAlgebraicSplitBridgeReport2 {
    /// Coarse bridge status.
    pub status: BezierBooleanAlgebraicSplitBridgeStatus,
    /// Algebraic ordering status used to derive this bridge.
    pub ordering_status: BezierBooleanAlgebraicParameterOrderingStatus,
    /// Split plan built from exact rational algebraic witnesses.
    pub split_plan: BezierBooleanSplitPlanReport2,
    /// Split insertion report derived from `split_plan`.
    pub insertion: BezierBooleanSplitInsertionReport2,
    /// Number of ordered algebraic events consumed.
    pub ordered_event_count: usize,
    /// Number of exact rational witnesses lowered to `Real` parameters.
    pub exact_rational_parameter_count: usize,
    /// Number of interval-only algebraic roots that blocked lowering.
    pub non_rational_parameter_count: usize,
    /// Number of unsupported event roles.
    pub unsupported_role_count: usize,
    /// Number of retained blocking preconditions.
    pub blocker_count: usize,
}

/// Replay status for exact roots returned by `hypersolve`.
///
/// Root isolation is only a proposal stage for boolean topology. `hypersolve`
/// may certify that roots are isolated, but `hypercurve` still requires exact
/// represented parameters before split insertion. This status follows Yap,
/// "Towards Exact Geometric Computation" (1997): algebraic work may advance a
/// construction only when its combinatorial preconditions become explicit
/// object facts, not primitive-float samples.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanRootIsolationReplayStatus {
    /// No scheduler or isolation work exists.
    Empty,
    /// No root-isolation replay was needed.
    NotNeeded,
    /// The handoff still has an earlier blocker.
    HandoffBlocked,
    /// `hypersolve` returned unsupported or undecided root-isolation evidence.
    HypersolveBlocked,
    /// `hypersolve` did not supply enough represented roots for the frontier.
    MissingIsolatedRoots,
    /// At least one supplied represented root lies outside the Bezier unit domain.
    InvalidParameterDomain,
    /// Exact represented roots can now feed split insertion.
    ReadyForSplitEvents,
}

/// Audit report for consuming exact roots returned by `hypersolve`.
///
/// This is the dependency-free replay seam between `hypersolve` root isolation
/// and `hypercurve` boolean construction. The report consumes a
/// [`BezierBooleanPathSchedulerReport2`] plus exact represented parameters
/// recovered from isolation. It then builds a split plan only if every retained
/// positive-width Bezier region and monotone-range isolating span has a
/// represented root in `[0, 1]`. The algorithmic contract is the
/// Sturm/Collins-Loos exact real-root isolation model: intervals or
/// multiplicity certificates are solver evidence, but boolean split insertion
/// receives exact parameters. This preserves Yap's predicate/construction
/// boundary and keeps Greiner-Hormann/Martinez-Rueda-Feito traversal stages
/// from seeing unrepresented algebraic intervals.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanRootIsolationReplayReport2 {
    /// Coarse replay status.
    pub status: BezierBooleanRootIsolationReplayStatus,
    /// Handoff status that authorized or blocked the replay.
    pub handoff_status: BezierBooleanRootIsolationHandoffStatus,
    /// Scheduler status before replay.
    pub scheduler_status: BezierBooleanPathSchedulerStatus,
    /// Split plan assembled from existing represented events plus isolated roots.
    pub split_plan: BezierBooleanSplitPlanReport2,
    /// Number of root-isolation obligations retained by the handoff.
    pub required_isolation_count: usize,
    /// Number of represented roots supplied by the caller.
    pub supplied_isolation_count: usize,
    /// Number of `hypersolve` root-isolation reports consumed.
    pub hypersolve_report_count: usize,
    /// Number of `hypersolve` isolating intervals inspected.
    pub hypersolve_interval_count: usize,
    /// Number of exact rational root witnesses recovered from `hypersolve`.
    pub hypersolve_exact_root_count: usize,
    /// Number of unsupported, undecided, or non-witness solver reports/intervals.
    pub hypersolve_unusable_count: usize,
    /// Number of `hypersolve` Bernstein subdivision reports consumed.
    pub hypersolve_bernstein_report_count: usize,
    /// Number of terminal Bernstein intervals certified empty.
    pub hypersolve_bernstein_empty_interval_count: usize,
    /// Number of exact Bernstein endpoint-root witnesses recovered.
    pub hypersolve_bernstein_endpoint_root_count: usize,
    /// Number of non-rational Bernstein isolating intervals retained as blockers.
    pub hypersolve_bernstein_isolating_interval_count: usize,
    /// Number of unsupported, undecided, depth-limited, or non-witness Bernstein rows.
    pub hypersolve_bernstein_unusable_count: usize,
    /// Number of `hypersolve` algebraic-root representation reports consumed.
    pub hypersolve_algebraic_report_count: usize,
    /// Number of represented algebraic roots inspected.
    pub hypersolve_algebraic_root_count: usize,
    /// Number of exact rational represented roots recovered.
    pub hypersolve_algebraic_exact_root_count: usize,
    /// Number of non-rational represented roots retained as interval blockers.
    pub hypersolve_algebraic_interval_root_count: usize,
    /// Number of invalid, unsupported, or non-witness algebraic-root rows.
    pub hypersolve_algebraic_unusable_count: usize,
    /// Number of still-missing represented roots.
    pub missing_isolation_count: usize,
    /// Number of supplied represented roots outside `[0, 1]`.
    pub out_of_range_parameter_count: usize,
    /// Number of blocking preconditions retained by this replay.
    pub blocker_count: usize,
    /// First explicit primitive uncertainty reason retained by the scheduler.
    pub uncertainty_reason: Option<UncertaintyReason>,
}

/// End-to-end status after replaying `hypersolve` roots into construction readiness.
///
/// This status keeps the algebraic solver boundary and the boolean
/// construction boundary visible as one report. Yap (1997) requires that
/// exact algebraic evidence become explicit object facts before construction;
/// Greiner-Hormann (1998) and Martinez-Rueda-Feito (2009) require those facts
/// to enter split insertion before traversal.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanRootIsolationConstructionStatus {
    /// No scheduler, replay, or construction work exists.
    Empty,
    /// The replay path completed and no interior splits are required.
    NoInteriorSplits,
    /// Replayed exact roots reached construction-ready interior split parameters.
    Ready,
    /// Root replay or an earlier exact stage blocked construction.
    ReplayBlocked,
    /// A replayed parameter violated the Bezier unit-domain invariant.
    InvalidParameterDomain,
}

/// Combined root-isolation replay and construction-readiness certificate.
///
/// This report is the compact `hypersolve`-to-boolean handoff: it consumes
/// exact root-isolation evidence, audits it as a
/// [`BezierBooleanRootIsolationReplayReport2`], then immediately runs the
/// standard split-plan audit and insertion classification. It is still a
/// report-only surface. It does not split curves, traverse arrangements, or
/// treat non-rational isolating intervals as topology.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanRootIsolationConstructionReport2 {
    /// Coarse chained status.
    pub status: BezierBooleanRootIsolationConstructionStatus,
    /// Replay audit over exact `hypersolve` witnesses.
    pub replay: BezierBooleanRootIsolationReplayReport2,
    /// Construction-readiness certificate derived from replay.
    pub readiness: BezierBooleanConstructionReadinessReport2,
    /// Number of blocking preconditions retained by replay/readiness.
    pub blocker_count: usize,
}

/// Audit status for a Bezier boolean split plan.
///
/// This audit is deliberately small: it certifies only the API-level invariant
/// that ready split plans contain unit-interval parameters. It does not sort or
/// deduplicate parameters and does not claim that a curve fragment has already
/// been split. That separation is Yap's predicate/construction boundary in
/// report form.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanSplitPlanAuditStatus {
    /// The plan has no split work.
    Empty,
    /// The plan is blocked by an earlier scheduler state.
    Blocked,
    /// Every emitted parameter was certified in the closed unit interval.
    Valid,
    /// At least one emitted parameter lies outside the closed unit interval.
    ParameterOutOfRange,
}

/// Unit-interval audit for a Bezier boolean split plan.
///
/// Bezier parameters are local curve coordinates, so split insertion is only
/// valid for values in `[0, 1]`. This report certifies that invariant through
/// exact `Real` ordering before a future fragment mutator consumes the plan.
/// As with the other boolean handoff reports, uncertain ordering is returned as
/// [`Classification::Uncertain`] instead of being decided by primitive-float
/// tolerance, following Yap, "Towards Exact Geometric Computation" (1997).
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BezierBooleanSplitPlanAuditReport2 {
    /// Final audit status.
    pub status: BezierBooleanSplitPlanAuditStatus,
    /// Total parameters checked across first, second, and shared range lists.
    pub checked_parameter_count: usize,
    /// Number of parameters certified outside `[0, 1]`.
    pub out_of_range_parameter_count: usize,
}

/// Location of a split parameter in a Bezier unit interval.
///
/// Future fragment mutation should only split at interior parameters. Endpoint
/// parameters are still valid boolean facts, but they are no-op boundaries for
/// a local curve segment. Keeping that distinction explicit prevents a boolean
/// implementation from manufacturing zero-length fragments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanSplitParameterLocation {
    /// The parameter is exactly `0` or exactly `1`.
    Endpoint,
    /// The parameter is strictly inside `(0, 1)`.
    Interior,
    /// The parameter is outside `[0, 1]`.
    OutOfRange,
}

/// Insertion-readiness status for an audited Bezier split plan.
///
/// This status refines [`BezierBooleanSplitPlanAuditStatus`] for the actual
/// fragment-insertion step. Greiner-Hormann and Martinez-Rueda-Feito boolean
/// pipelines insert intersections before traversal, but endpoint-only events
/// do not require a new local fragment. Yap's exact-computation discipline
/// requires that distinction to be decided by exact parameter ordering rather
/// than by a tolerance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanSplitInsertionStatus {
    /// No split work exists.
    Empty,
    /// An earlier scheduler state blocks insertion.
    Blocked,
    /// The plan contains out-of-range parameters.
    InvalidParameterDomain,
    /// Parameters are valid but all are endpoints, so no fragment mutation is needed.
    NoInteriorSplits,
    /// At least one valid interior parameter can be inserted.
    Ready,
}

/// Exact insertion-work report for a Bezier boolean split plan.
///
/// The report classifies already-audited split parameters into endpoint
/// no-ops and interior insertion candidates, preserving separate lists for the
/// first relation operand, second relation operand, and shared monotone-range
/// parameters. It performs no curve evaluation and no mutation. This is the
/// final report-only handoff before future Bezier/conic fragment splitting.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanSplitInsertionReport2 {
    /// Insertion readiness.
    pub status: BezierBooleanSplitInsertionStatus,
    /// Exact interior parameters for the first relation operand.
    pub first_curve_interior_parameters: Vec<Real>,
    /// Exact interior parameters for the second relation operand.
    pub second_curve_interior_parameters: Vec<Real>,
    /// Exact interior parameters for shared monotone ranges.
    pub shared_range_interior_parameters: Vec<Real>,
    /// Number of parameters classified as endpoints.
    pub endpoint_parameter_count: usize,
    /// Number of parameters classified as interior.
    pub interior_parameter_count: usize,
    /// Number of parameters classified outside `[0, 1]`.
    pub out_of_range_parameter_count: usize,
}

/// End-to-end report-only readiness status for Bezier boolean construction.
///
/// This status compacts the scheduler, split-plan, audit, and insertion reports
/// into the decision a future Bezier/conic boolean builder needs before it
/// mutates fragments. It does not claim that boolean traversal has happened;
/// it only certifies whether the exact intersection/frontier facts have reached
/// a safe insertion state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanConstructionReadinessStatus {
    /// No relation/range work exists.
    Empty,
    /// All facts are decided, but all split parameters are endpoint no-ops.
    NoInteriorSplits,
    /// Interior split parameters are ready for fragment insertion.
    Ready,
    /// A scheduler blocker prevents construction.
    Blocked,
    /// A split parameter violated the unit-interval invariant.
    InvalidParameterDomain,
}

/// End-to-end certificate for the report-only Bezier boolean handoff.
///
/// The report chains [`BezierBooleanPathSchedulerReport2`],
/// [`BezierBooleanSplitPlanReport2`],
/// [`BezierBooleanSplitPlanAuditReport2`], and
/// [`BezierBooleanSplitInsertionReport2`] into one object. This preserves Yap's
/// requirement that each object-level construction precondition be explicit:
/// scheduler blockers, invalid split domains, endpoint-only no-op plans, and
/// ready interior split sets are distinct states. The split/classify/traverse
/// framing follows Greiner-Hormann (1998) and Martinez-Rueda-Feito (2009);
/// degenerate overlap blockers remain explicit as in Foster-Hormann-Popa
/// (2019).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanConstructionReadinessReport2 {
    /// Coarse construction-readiness status.
    pub status: BezierBooleanConstructionReadinessStatus,
    /// Global scheduler report.
    pub scheduler: BezierBooleanPathSchedulerReport2,
    /// Split-plan payload derived from the scheduler.
    pub split_plan: BezierBooleanSplitPlanReport2,
    /// Exact unit-interval audit over split-plan parameters.
    pub split_plan_audit: BezierBooleanSplitPlanAuditReport2,
    /// Exact interior/endpoint insertion classification.
    pub insertion: BezierBooleanSplitInsertionReport2,
}

/// Fragment-construction status for one Bezier/conic boolean operand.
///
/// This is the first construction stage after
/// [`BezierBooleanConstructionReadinessReport2`]. It still does not perform
/// boolean traversal or inside/outside classification. It only says whether a
/// single source curve was actually split at exact interior parameters. The
/// split operation itself uses de Casteljau subdivision, preserving the affine
/// construction advocated by Yap, "Towards Exact Geometric Computation"
/// (1997), and the intersection-insertion phase used by Greiner-Hormann
/// (1998) and Martinez-Rueda-Feito (2009).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierBooleanFragmentConstructionStatus {
    /// No scheduler/range/relation work was supplied.
    Empty,
    /// An earlier exact stage blocked construction.
    Blocked,
    /// A split parameter violated the unit-interval invariant.
    InvalidParameterDomain,
    /// All valid parameters were endpoints, so the source curve is unchanged.
    NoInteriorSplits,
    /// The source curve was split into exact Bezier/conic fragments.
    Ready,
}

/// Exact split fragments for a quadratic Bezier boolean operand.
///
/// The report consumes already-certified split parameters and materializes the
/// local curve fragments needed by a later path-boolean arrangement. It sorts
/// and deduplicates exact interior parameters before subdivision so repeated
/// contacts do not manufacture zero-length fragments. It intentionally keeps
/// blocked and endpoint-only states visible instead of falling back to sampled
/// topology, following Yap's certified-or-explicit-unknown contract.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanQuadraticFragmentReport2 {
    /// Coarse fragment-construction status.
    pub status: BezierBooleanFragmentConstructionStatus,
    /// Construction-readiness state that fed this report.
    pub readiness_status: BezierBooleanConstructionReadinessStatus,
    /// Number of candidate parameters supplied for this operand.
    pub source_parameter_count: usize,
    /// Number of endpoint parameters ignored as no-op split boundaries.
    pub endpoint_parameter_count: usize,
    /// Number of out-of-range parameters rejected before construction.
    pub out_of_range_parameter_count: usize,
    /// Number of unique exact interior parameters inserted.
    pub inserted_parameter_count: usize,
    /// Sorted unique exact interior parameters used for construction.
    pub inserted_parameters: Vec<Real>,
    /// Exact quadratic Bezier fragments in source traversal order.
    pub fragments: Vec<QuadraticBezier2>,
}

/// Exact split fragments for a cubic Bezier boolean operand.
///
/// This is the cubic analogue of
/// [`BezierBooleanQuadraticFragmentReport2`]. The subdivision is exact
/// de Casteljau construction; no flattening or tolerance path is introduced
/// into the topology layer.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanCubicFragmentReport2 {
    /// Coarse fragment-construction status.
    pub status: BezierBooleanFragmentConstructionStatus,
    /// Construction-readiness state that fed this report.
    pub readiness_status: BezierBooleanConstructionReadinessStatus,
    /// Number of candidate parameters supplied for this operand.
    pub source_parameter_count: usize,
    /// Number of endpoint parameters ignored as no-op split boundaries.
    pub endpoint_parameter_count: usize,
    /// Number of out-of-range parameters rejected before construction.
    pub out_of_range_parameter_count: usize,
    /// Number of unique exact interior parameters inserted.
    pub inserted_parameter_count: usize,
    /// Sorted unique exact interior parameters used for construction.
    pub inserted_parameters: Vec<Real>,
    /// Exact cubic Bezier fragments in source traversal order.
    pub fragments: Vec<CubicBezier2>,
}

/// Exact split fragments for a rational quadratic Bezier/conic boolean operand.
///
/// Rational conics are split in homogeneous coordinates and then projected back
/// to affine control points only after each intermediate weight is certified
/// nonzero. This follows Yap's object-preserving exact-computation model
/// (1997) and Farin's rational Bezier construction: denominator/projective
/// boundaries are explicit uncertainty, not sampled topology.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierBooleanRationalQuadraticFragmentReport2 {
    /// Coarse fragment-construction status.
    pub status: BezierBooleanFragmentConstructionStatus,
    /// Construction-readiness state that fed this report.
    pub readiness_status: BezierBooleanConstructionReadinessStatus,
    /// Number of candidate parameters supplied for this operand.
    pub source_parameter_count: usize,
    /// Number of endpoint parameters ignored as no-op split boundaries.
    pub endpoint_parameter_count: usize,
    /// Number of out-of-range parameters rejected before construction.
    pub out_of_range_parameter_count: usize,
    /// Number of unique exact interior parameters inserted.
    pub inserted_parameter_count: usize,
    /// Sorted unique exact interior parameters used for construction.
    pub inserted_parameters: Vec<Real>,
    /// Exact rational quadratic fragments in source traversal order.
    pub fragments: Vec<RationalQuadraticBezier2>,
}

impl BezierBooleanQuadraticFragmentReport2 {
    /// Splits `curve` at first-operand parameters from a readiness certificate.
    pub fn from_first_curve_readiness(
        curve: &QuadraticBezier2,
        readiness: &BezierBooleanConstructionReadinessReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_readiness_parameters(
            curve,
            readiness,
            &readiness.insertion.first_curve_interior_parameters,
            policy,
        )
    }

    /// Splits `curve` at second-operand parameters from a readiness certificate.
    pub fn from_second_curve_readiness(
        curve: &QuadraticBezier2,
        readiness: &BezierBooleanConstructionReadinessReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_readiness_parameters(
            curve,
            readiness,
            &readiness.insertion.second_curve_interior_parameters,
            policy,
        )
    }

    /// Splits `curve` at caller-supplied exact parameters.
    ///
    /// This lower-level entry point is useful for tests and for future
    /// ownership-specific boolean builders that already selected one operand's
    /// parameter list. It performs the same exact domain, ordering, and
    /// deduplication checks as the readiness-based constructors.
    pub fn from_parameters(
        curve: &QuadraticBezier2,
        parameters: &[Real],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_parameters_with_status(
            curve,
            parameters,
            BezierBooleanConstructionReadinessStatus::Ready,
            policy,
        )
    }

    fn from_readiness_parameters(
        curve: &QuadraticBezier2,
        readiness: &BezierBooleanConstructionReadinessReport2,
        parameters: &[Real],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        match readiness.status {
            BezierBooleanConstructionReadinessStatus::Empty => {
                Classification::Decided(quadratic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::Empty,
                    readiness.status,
                    parameters.len(),
                    0,
                    0,
                    Vec::new(),
                    vec![curve.clone()],
                ))
            }
            BezierBooleanConstructionReadinessStatus::Blocked => {
                Classification::Decided(quadratic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::Blocked,
                    readiness.status,
                    parameters.len(),
                    0,
                    0,
                    Vec::new(),
                    Vec::new(),
                ))
            }
            BezierBooleanConstructionReadinessStatus::InvalidParameterDomain => {
                Classification::Decided(quadratic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::InvalidParameterDomain,
                    readiness.status,
                    parameters.len(),
                    0,
                    readiness.insertion.out_of_range_parameter_count,
                    Vec::new(),
                    Vec::new(),
                ))
            }
            BezierBooleanConstructionReadinessStatus::NoInteriorSplits => {
                Classification::Decided(quadratic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
                    readiness.status,
                    parameters.len(),
                    readiness.insertion.endpoint_parameter_count,
                    0,
                    Vec::new(),
                    vec![curve.clone()],
                ))
            }
            BezierBooleanConstructionReadinessStatus::Ready => {
                Self::from_parameters_with_status(curve, parameters, readiness.status, policy)
            }
        }
    }

    fn from_parameters_with_status(
        curve: &QuadraticBezier2,
        parameters: &[Real],
        readiness_status: BezierBooleanConstructionReadinessStatus,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let normalized = match normalize_split_parameters(parameters, policy) {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        if normalized.out_of_range_parameter_count > 0 {
            return Classification::Decided(quadratic_fragment_report(
                BezierBooleanFragmentConstructionStatus::InvalidParameterDomain,
                readiness_status,
                parameters.len(),
                normalized.endpoint_parameter_count,
                normalized.out_of_range_parameter_count,
                Vec::new(),
                Vec::new(),
            ));
        }
        if normalized.interior_parameters.is_empty() {
            return Classification::Decided(quadratic_fragment_report(
                BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
                readiness_status,
                parameters.len(),
                normalized.endpoint_parameter_count,
                0,
                Vec::new(),
                vec![curve.clone()],
            ));
        }

        let fragments =
            split_quadratic_at_sorted_parameters(curve, &normalized.interior_parameters);
        Classification::Decided(quadratic_fragment_report(
            BezierBooleanFragmentConstructionStatus::Ready,
            readiness_status,
            parameters.len(),
            normalized.endpoint_parameter_count,
            0,
            normalized.interior_parameters,
            fragments,
        ))
    }
}

impl BezierBooleanRationalQuadraticFragmentReport2 {
    /// Splits `curve` at first-operand parameters from a readiness certificate.
    pub fn from_first_curve_readiness(
        curve: &RationalQuadraticBezier2,
        readiness: &BezierBooleanConstructionReadinessReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_readiness_parameters(
            curve,
            readiness,
            &readiness.insertion.first_curve_interior_parameters,
            policy,
        )
    }

    /// Splits `curve` at second-operand parameters from a readiness certificate.
    pub fn from_second_curve_readiness(
        curve: &RationalQuadraticBezier2,
        readiness: &BezierBooleanConstructionReadinessReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_readiness_parameters(
            curve,
            readiness,
            &readiness.insertion.second_curve_interior_parameters,
            policy,
        )
    }

    /// Splits `curve` at caller-supplied exact parameters.
    pub fn from_parameters(
        curve: &RationalQuadraticBezier2,
        parameters: &[Real],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_parameters_with_status(
            curve,
            parameters,
            BezierBooleanConstructionReadinessStatus::Ready,
            policy,
        )
    }

    fn from_readiness_parameters(
        curve: &RationalQuadraticBezier2,
        readiness: &BezierBooleanConstructionReadinessReport2,
        parameters: &[Real],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        match readiness.status {
            BezierBooleanConstructionReadinessStatus::Empty => {
                Classification::Decided(rational_quadratic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::Empty,
                    readiness.status,
                    parameters.len(),
                    0,
                    0,
                    Vec::new(),
                    vec![curve.clone()],
                ))
            }
            BezierBooleanConstructionReadinessStatus::Blocked => {
                Classification::Decided(rational_quadratic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::Blocked,
                    readiness.status,
                    parameters.len(),
                    0,
                    0,
                    Vec::new(),
                    Vec::new(),
                ))
            }
            BezierBooleanConstructionReadinessStatus::InvalidParameterDomain => {
                Classification::Decided(rational_quadratic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::InvalidParameterDomain,
                    readiness.status,
                    parameters.len(),
                    0,
                    readiness.insertion.out_of_range_parameter_count,
                    Vec::new(),
                    Vec::new(),
                ))
            }
            BezierBooleanConstructionReadinessStatus::NoInteriorSplits => {
                Classification::Decided(rational_quadratic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
                    readiness.status,
                    parameters.len(),
                    readiness.insertion.endpoint_parameter_count,
                    0,
                    Vec::new(),
                    vec![curve.clone()],
                ))
            }
            BezierBooleanConstructionReadinessStatus::Ready => {
                Self::from_parameters_with_status(curve, parameters, readiness.status, policy)
            }
        }
    }

    fn from_parameters_with_status(
        curve: &RationalQuadraticBezier2,
        parameters: &[Real],
        readiness_status: BezierBooleanConstructionReadinessStatus,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let normalized = match normalize_split_parameters(parameters, policy) {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        if normalized.out_of_range_parameter_count > 0 {
            return Classification::Decided(rational_quadratic_fragment_report(
                BezierBooleanFragmentConstructionStatus::InvalidParameterDomain,
                readiness_status,
                parameters.len(),
                normalized.endpoint_parameter_count,
                normalized.out_of_range_parameter_count,
                Vec::new(),
                Vec::new(),
            ));
        }
        if normalized.interior_parameters.is_empty() {
            return Classification::Decided(rational_quadratic_fragment_report(
                BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
                readiness_status,
                parameters.len(),
                normalized.endpoint_parameter_count,
                0,
                Vec::new(),
                vec![curve.clone()],
            ));
        }

        let fragments = match split_rational_quadratic_at_sorted_parameters(
            curve,
            &normalized.interior_parameters,
            policy,
        ) {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        Classification::Decided(rational_quadratic_fragment_report(
            BezierBooleanFragmentConstructionStatus::Ready,
            readiness_status,
            parameters.len(),
            normalized.endpoint_parameter_count,
            0,
            normalized.interior_parameters,
            fragments,
        ))
    }
}

impl BezierBooleanOverlapResolutionReport2 {
    /// Resolves overlap events retained by a batch of handoff reports.
    ///
    /// Non-overlap blockers remain blockers. Split-ready point events and
    /// no-event reports do not block finite-overlap normalization because they
    /// can be inserted independently by the ordinary split-plan path.
    pub fn from_handoff_reports(
        reports: &[BezierBooleanHandoffReport2],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut overlap_events = Vec::new();
        let mut blocker_count = 0;
        let mut uncertainty_reason = None;

        for report in reports {
            overlap_events.extend(report.overlap_events.iter().cloned());
            match report.status {
                BezierBooleanHandoffStatus::NoEvents
                | BezierBooleanHandoffStatus::SplitEventsReady => {}
                BezierBooleanHandoffStatus::NeedsOverlapResolver => {
                    if report.overlap_events.is_empty() {
                        blocker_count += 1;
                    }
                }
                BezierBooleanHandoffStatus::NeedsParameterRecovery
                | BezierBooleanHandoffStatus::NeedsRegionIsolation
                | BezierBooleanHandoffStatus::Unresolved
                | BezierBooleanHandoffStatus::Uncertain => {
                    blocker_count += 1;
                    uncertainty_reason = uncertainty_reason.or(report.uncertainty_reason);
                }
            }
        }

        Self::from_overlap_events_with_blockers(
            &overlap_events,
            blocker_count,
            uncertainty_reason,
            policy,
        )
    }

    /// Resolves a standalone list of exact overlap events.
    pub fn from_overlap_events(
        events: &[BezierBooleanOverlapEvent2],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_overlap_events_with_blockers(events, 0, None, policy)
    }

    /// Returns true when finite overlaps produced exact split boundaries.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanOverlapResolutionStatus::Ready
    }

    /// Returns true when overlap resolution is blocked or invalid.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanOverlapResolutionStatus::InvalidParameterDomain
                | BezierBooleanOverlapResolutionStatus::Blocked
        )
    }

    fn from_overlap_events_with_blockers(
        events: &[BezierBooleanOverlapEvent2],
        blocker_count: usize,
        uncertainty_reason: Option<UncertaintyReason>,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        if blocker_count > 0 {
            return Classification::Decided(Self {
                status: BezierBooleanOverlapResolutionStatus::Blocked,
                overlap_event_count: events.len(),
                resolved_events: Vec::new(),
                first_curve_boundary_parameters: Vec::new(),
                second_curve_boundary_parameters: Vec::new(),
                invalid_range_count: 0,
                blocker_count,
                uncertainty_reason,
            });
        }

        if events.is_empty() {
            return Classification::Decided(Self {
                status: BezierBooleanOverlapResolutionStatus::Empty,
                overlap_event_count: 0,
                resolved_events: Vec::new(),
                first_curve_boundary_parameters: Vec::new(),
                second_curve_boundary_parameters: Vec::new(),
                invalid_range_count: 0,
                blocker_count: 0,
                uncertainty_reason: None,
            });
        }

        let mut resolved_events = Vec::with_capacity(events.len());
        let mut first_curve_boundary_parameters = Vec::new();
        let mut second_curve_boundary_parameters = Vec::new();
        let mut invalid_range_count = 0;

        for event in events {
            let first = match sorted_unit_range_boundaries(&event.first_range, policy) {
                Classification::Decided(Some(parameters)) => parameters,
                Classification::Decided(None) => {
                    invalid_range_count += 1;
                    continue;
                }
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
            let second = match sorted_unit_range_boundaries(&event.second_range, policy) {
                Classification::Decided(Some(parameters)) => parameters,
                Classification::Decided(None) => {
                    invalid_range_count += 1;
                    continue;
                }
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };

            for parameter in &first {
                if insert_unique_sorted_parameter(
                    &mut first_curve_boundary_parameters,
                    parameter,
                    policy,
                )
                .is_none()
                {
                    return Classification::Uncertain(UncertaintyReason::Ordering);
                }
            }
            for parameter in &second {
                if insert_unique_sorted_parameter(
                    &mut second_curve_boundary_parameters,
                    parameter,
                    policy,
                )
                .is_none()
                {
                    return Classification::Uncertain(UncertaintyReason::Ordering);
                }
            }

            resolved_events.push(BezierBooleanResolvedOverlapEvent2 {
                first_range: event.first_range.clone(),
                second_range: event.second_range.clone(),
                first_boundary_parameters: first,
                second_boundary_parameters: second,
            });
        }

        Classification::Decided(Self {
            status: if invalid_range_count == 0 {
                BezierBooleanOverlapResolutionStatus::Ready
            } else {
                BezierBooleanOverlapResolutionStatus::InvalidParameterDomain
            },
            overlap_event_count: events.len(),
            resolved_events,
            first_curve_boundary_parameters,
            second_curve_boundary_parameters,
            invalid_range_count,
            blocker_count: 0,
            uncertainty_reason: None,
        })
    }
}

impl BezierBooleanArrangementReadinessReport2 {
    /// Builds arrangement readiness for two quadratic Bezier operand reports.
    pub fn from_quadratic_fragments(
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        overlaps: &BezierBooleanOverlapResolutionReport2,
    ) -> Self {
        Self::from_parts(
            first.status,
            first.fragments.len(),
            second.status,
            second.fragments.len(),
            overlaps,
        )
    }

    /// Builds arrangement readiness for two cubic Bezier operand reports.
    pub fn from_cubic_fragments(
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        overlaps: &BezierBooleanOverlapResolutionReport2,
    ) -> Self {
        Self::from_parts(
            first.status,
            first.fragments.len(),
            second.status,
            second.fragments.len(),
            overlaps,
        )
    }

    /// Builds arrangement readiness for two rational quadratic/conic operand reports.
    pub fn from_rational_quadratic_fragments(
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        overlaps: &BezierBooleanOverlapResolutionReport2,
    ) -> Self {
        Self::from_parts(
            first.status,
            first.fragments.len(),
            second.status,
            second.fragments.len(),
            overlaps,
        )
    }

    /// Builds arrangement readiness from generic fragment statuses and counts.
    ///
    /// This constructor is useful when a future path owns a heterogeneous
    /// Bezier/conic segment list and has already normalized each operand into a
    /// fragment count. It preserves the same blocker precedence as the typed
    /// constructors.
    pub fn from_parts(
        first_status: BezierBooleanFragmentConstructionStatus,
        first_fragment_count: usize,
        second_status: BezierBooleanFragmentConstructionStatus,
        second_fragment_count: usize,
        overlaps: &BezierBooleanOverlapResolutionReport2,
    ) -> Self {
        let mut blocker_count = 0;
        let mut status = BezierBooleanArrangementReadinessStatus::Ready;

        if first_status == BezierBooleanFragmentConstructionStatus::InvalidParameterDomain
            || second_status == BezierBooleanFragmentConstructionStatus::InvalidParameterDomain
            || overlaps.status == BezierBooleanOverlapResolutionStatus::InvalidParameterDomain
        {
            blocker_count += 1;
            status = BezierBooleanArrangementReadinessStatus::InvalidParameterDomain;
        } else if first_status == BezierBooleanFragmentConstructionStatus::Blocked
            || second_status == BezierBooleanFragmentConstructionStatus::Blocked
        {
            blocker_count += 1;
            status = BezierBooleanArrangementReadinessStatus::Blocked;
        } else if overlaps.status == BezierBooleanOverlapResolutionStatus::Blocked {
            blocker_count += 1;
            status = BezierBooleanArrangementReadinessStatus::OverlapBlocked;
        } else if first_status == BezierBooleanFragmentConstructionStatus::Empty
            && second_status == BezierBooleanFragmentConstructionStatus::Empty
            && overlaps.status == BezierBooleanOverlapResolutionStatus::Empty
        {
            status = BezierBooleanArrangementReadinessStatus::Empty;
        } else if first_fragment_count == 0 {
            blocker_count += 1;
            status = BezierBooleanArrangementReadinessStatus::MissingFirstFragments;
        } else if second_fragment_count == 0 {
            blocker_count += 1;
            status = BezierBooleanArrangementReadinessStatus::MissingSecondFragments;
        } else if first_status == BezierBooleanFragmentConstructionStatus::NoInteriorSplits
            && second_status == BezierBooleanFragmentConstructionStatus::NoInteriorSplits
            && overlaps.status == BezierBooleanOverlapResolutionStatus::Empty
        {
            status = BezierBooleanArrangementReadinessStatus::NoInteriorSplits;
        }

        Self {
            status,
            first_fragment_status: first_status,
            second_fragment_status: second_status,
            overlap_status: overlaps.status,
            first_fragment_count,
            second_fragment_count,
            resolved_overlap_count: overlaps.resolved_events.len(),
            overlap_boundary_parameter_count: overlaps.first_curve_boundary_parameters.len()
                + overlaps.second_curve_boundary_parameters.len(),
            blocker_count,
        }
    }

    /// Returns true when a future arrangement traversal has its split inputs.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanArrangementReadinessStatus::Ready
    }

    /// Returns true when a blocker prevents arrangement traversal.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanArrangementReadinessStatus::MissingFirstFragments
                | BezierBooleanArrangementReadinessStatus::MissingSecondFragments
                | BezierBooleanArrangementReadinessStatus::OverlapBlocked
                | BezierBooleanArrangementReadinessStatus::InvalidParameterDomain
                | BezierBooleanArrangementReadinessStatus::Blocked
        )
    }
}

impl BezierBooleanTraversalPreconditionReport2 {
    /// Audits quadratic Bezier fragment chains for traversal.
    pub fn from_quadratic_fragments(
        readiness: &BezierBooleanArrangementReadinessReport2,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
    ) -> Self {
        Self::from_endpoint_chains(
            readiness,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
        )
    }

    /// Audits cubic Bezier fragment chains for traversal.
    pub fn from_cubic_fragments(
        readiness: &BezierBooleanArrangementReadinessReport2,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
    ) -> Self {
        Self::from_endpoint_chains(
            readiness,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
        )
    }

    /// Audits rational quadratic/conic fragment chains for traversal.
    pub fn from_rational_quadratic_fragments(
        readiness: &BezierBooleanArrangementReadinessReport2,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
    ) -> Self {
        Self::from_endpoint_chains(
            readiness,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
        )
    }

    /// Builds a traversal-precondition audit from generic fragment endpoints.
    ///
    /// Each endpoint pair is `(fragment_start, fragment_end)`. This constructor
    /// supports future heterogeneous Bezier/conic paths without forcing them
    /// through a single concrete curve enum.
    pub fn from_endpoint_chains(
        readiness: &BezierBooleanArrangementReadinessReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
    ) -> Self {
        let first_gap_count = fragment_chain_gap_count(first_endpoints);
        let second_gap_count = fragment_chain_gap_count(second_endpoints);
        let mut blocker_count = 0;
        let status = match readiness.status {
            BezierBooleanArrangementReadinessStatus::Empty => {
                BezierBooleanTraversalPreconditionStatus::Empty
            }
            BezierBooleanArrangementReadinessStatus::NoInteriorSplits => {
                if first_endpoints.is_empty() {
                    blocker_count += 1;
                    BezierBooleanTraversalPreconditionStatus::MissingFirstFragments
                } else if second_endpoints.is_empty() {
                    blocker_count += 1;
                    BezierBooleanTraversalPreconditionStatus::MissingSecondFragments
                } else if first_gap_count > 0 {
                    blocker_count += first_gap_count;
                    BezierBooleanTraversalPreconditionStatus::FirstChainDiscontinuous
                } else if second_gap_count > 0 {
                    blocker_count += second_gap_count;
                    BezierBooleanTraversalPreconditionStatus::SecondChainDiscontinuous
                } else {
                    BezierBooleanTraversalPreconditionStatus::NoInteriorSplits
                }
            }
            BezierBooleanArrangementReadinessStatus::Ready => {
                if first_endpoints.is_empty() {
                    blocker_count += 1;
                    BezierBooleanTraversalPreconditionStatus::MissingFirstFragments
                } else if second_endpoints.is_empty() {
                    blocker_count += 1;
                    BezierBooleanTraversalPreconditionStatus::MissingSecondFragments
                } else if first_gap_count > 0 {
                    blocker_count += first_gap_count;
                    BezierBooleanTraversalPreconditionStatus::FirstChainDiscontinuous
                } else if second_gap_count > 0 {
                    blocker_count += second_gap_count;
                    BezierBooleanTraversalPreconditionStatus::SecondChainDiscontinuous
                } else {
                    BezierBooleanTraversalPreconditionStatus::Ready
                }
            }
            BezierBooleanArrangementReadinessStatus::MissingFirstFragments => {
                blocker_count += 1;
                BezierBooleanTraversalPreconditionStatus::MissingFirstFragments
            }
            BezierBooleanArrangementReadinessStatus::MissingSecondFragments => {
                blocker_count += 1;
                BezierBooleanTraversalPreconditionStatus::MissingSecondFragments
            }
            BezierBooleanArrangementReadinessStatus::OverlapBlocked
            | BezierBooleanArrangementReadinessStatus::InvalidParameterDomain
            | BezierBooleanArrangementReadinessStatus::Blocked => {
                blocker_count += readiness.blocker_count.max(1);
                BezierBooleanTraversalPreconditionStatus::ReadinessBlocked
            }
        };

        Self {
            status,
            readiness_status: readiness.status,
            first_fragment_count: first_endpoints.len(),
            second_fragment_count: second_endpoints.len(),
            first_chain_gap_count: first_gap_count,
            second_chain_gap_count: second_gap_count,
            resolved_overlap_count: readiness.resolved_overlap_count,
            overlap_boundary_parameter_count: readiness.overlap_boundary_parameter_count,
            blocker_count,
        }
    }

    /// Returns true when fragment chains are safe for future traversal.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanTraversalPreconditionStatus::Ready
    }

    /// Returns true when a malformed or blocked frontier prevents traversal.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanTraversalPreconditionStatus::ReadinessBlocked
                | BezierBooleanTraversalPreconditionStatus::MissingFirstFragments
                | BezierBooleanTraversalPreconditionStatus::MissingSecondFragments
                | BezierBooleanTraversalPreconditionStatus::FirstChainDiscontinuous
                | BezierBooleanTraversalPreconditionStatus::SecondChainDiscontinuous
        )
    }
}

impl BezierBooleanTraversalScheduleReport2 {
    /// Builds a traversal schedule from quadratic Bezier fragment reports.
    pub fn from_quadratic_fragments(
        preconditions: &BezierBooleanTraversalPreconditionReport2,
        _first: &BezierBooleanQuadraticFragmentReport2,
        _second: &BezierBooleanQuadraticFragmentReport2,
    ) -> Self {
        Self::from_preconditions(preconditions)
    }

    /// Builds a traversal schedule from cubic Bezier fragment reports.
    pub fn from_cubic_fragments(
        preconditions: &BezierBooleanTraversalPreconditionReport2,
        _first: &BezierBooleanCubicFragmentReport2,
        _second: &BezierBooleanCubicFragmentReport2,
    ) -> Self {
        Self::from_preconditions(preconditions)
    }

    /// Builds a traversal schedule from rational quadratic/conic fragment reports.
    pub fn from_rational_quadratic_fragments(
        preconditions: &BezierBooleanTraversalPreconditionReport2,
        _first: &BezierBooleanRationalQuadraticFragmentReport2,
        _second: &BezierBooleanRationalQuadraticFragmentReport2,
    ) -> Self {
        Self::from_preconditions(preconditions)
    }

    /// Builds a deterministic traversal worklist from audited preconditions.
    ///
    /// Ready preconditions produce first-operand visits followed by
    /// second-operand visits. That stable order is not a fill rule; it is only
    /// an auditable worklist for a later ownership classifier.
    pub fn from_preconditions(preconditions: &BezierBooleanTraversalPreconditionReport2) -> Self {
        let (status, blocker_count, steps) = match preconditions.status {
            BezierBooleanTraversalPreconditionStatus::Empty => {
                (BezierBooleanTraversalScheduleStatus::Empty, 0, Vec::new())
            }
            BezierBooleanTraversalPreconditionStatus::NoInteriorSplits => (
                BezierBooleanTraversalScheduleStatus::NoInteriorSplits,
                0,
                Vec::new(),
            ),
            BezierBooleanTraversalPreconditionStatus::Ready => {
                let mut steps = Vec::with_capacity(
                    preconditions.first_fragment_count + preconditions.second_fragment_count,
                );
                for fragment_index in 0..preconditions.first_fragment_count {
                    steps.push(BezierBooleanTraversalStep2 {
                        operand: BezierBooleanTraversalOperand::First,
                        fragment_index,
                    });
                }
                for fragment_index in 0..preconditions.second_fragment_count {
                    steps.push(BezierBooleanTraversalStep2 {
                        operand: BezierBooleanTraversalOperand::Second,
                        fragment_index,
                    });
                }
                (BezierBooleanTraversalScheduleStatus::Ready, 0, steps)
            }
            BezierBooleanTraversalPreconditionStatus::ReadinessBlocked
            | BezierBooleanTraversalPreconditionStatus::MissingFirstFragments
            | BezierBooleanTraversalPreconditionStatus::MissingSecondFragments
            | BezierBooleanTraversalPreconditionStatus::FirstChainDiscontinuous
            | BezierBooleanTraversalPreconditionStatus::SecondChainDiscontinuous => (
                BezierBooleanTraversalScheduleStatus::PreconditionBlocked,
                preconditions.blocker_count.max(1),
                Vec::new(),
            ),
        };

        Self {
            status,
            precondition_status: preconditions.status,
            first_fragment_count: preconditions.first_fragment_count,
            second_fragment_count: preconditions.second_fragment_count,
            steps,
            resolved_overlap_count: preconditions.resolved_overlap_count,
            overlap_boundary_parameter_count: preconditions.overlap_boundary_parameter_count,
            blocker_count,
        }
    }

    /// Returns true when the fragment worklist can feed future traversal.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanTraversalScheduleStatus::Ready
    }

    /// Returns true when traversal scheduling preserved a blocker.
    pub fn has_blockers(&self) -> bool {
        self.status == BezierBooleanTraversalScheduleStatus::PreconditionBlocked
    }
}

impl BezierBooleanOwnershipFactReport2 {
    /// Expands per-fragment operand locator vectors and validates the facts.
    ///
    /// This is the non-uniform counterpart to
    /// [`Self::from_uniform_operand_locations`]. A future exact locator can
    /// classify each first-operand fragment against the second operand and
    /// each second-operand fragment against the first operand, then hand those
    /// vectors here without constructing keyed facts manually. Count and
    /// boundary blockers remain explicit before boolean selection.
    pub fn from_operand_locations(
        schedule: &BezierBooleanTraversalScheduleReport2,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
    ) -> Self {
        let locations = BezierBooleanOperandOwnershipLocationReport2::from_schedule_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        Self::from_schedule_facts(schedule, &locations.facts)
    }

    /// Expands uniform operand-level locator results and validates the facts.
    ///
    /// This is a convenience constructor over
    /// [`BezierBooleanUniformOwnershipFactReport2`] followed by
    /// [`Self::from_schedule_facts`]. Use it when a certified arrangement or
    /// containment pass has proved one relation for all first fragments and one
    /// relation for all second fragments. It deliberately does not sample a
    /// representative point or infer containment from bounding boxes.
    pub fn from_uniform_operand_locations(
        schedule: &BezierBooleanTraversalScheduleReport2,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
    ) -> Self {
        let uniform = BezierBooleanUniformOwnershipFactReport2::from_schedule_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        Self::from_schedule_facts(schedule, &uniform.facts)
    }

    /// Validates externally certified ownership facts against a traversal schedule.
    ///
    /// The supplied facts must be in the same deterministic order as
    /// [`BezierBooleanTraversalScheduleReport2::steps`] and each fact must
    /// repeat the exact operand/index key it classifies. Boundary facts are
    /// retained but block boolean selection until a degenerate-overlap policy
    /// has certified their side. This directly applies Yap's exact-computation
    /// model (1997): the report consumes certified combinatorial facts and
    /// exposes missing, extra, mismatched, or boundary facts as data instead of
    /// replacing them with midpoint samples.
    pub fn from_schedule_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        facts: &[BezierBooleanOwnershipFact2],
    ) -> Self {
        match schedule.status {
            BezierBooleanTraversalScheduleStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanOwnershipFactStatus::Empty,
                    schedule,
                    facts.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                );
            }
            BezierBooleanTraversalScheduleStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanOwnershipFactStatus::NoInteriorSplits,
                    schedule,
                    facts.len(),
                    0,
                    facts.len(),
                    0,
                    0,
                    facts.len(),
                );
            }
            BezierBooleanTraversalScheduleStatus::PreconditionBlocked => {
                return Self::empty_like(
                    BezierBooleanOwnershipFactStatus::ScheduleBlocked,
                    schedule,
                    facts.len(),
                    0,
                    0,
                    0,
                    0,
                    schedule.blocker_count.max(1),
                );
            }
            BezierBooleanTraversalScheduleStatus::Ready => {}
        }

        if facts.len() < schedule.steps.len() {
            let missing = schedule.steps.len() - facts.len();
            return Self::empty_like(
                BezierBooleanOwnershipFactStatus::MissingOwnershipFacts,
                schedule,
                facts.len(),
                missing,
                0,
                0,
                0,
                missing.max(1),
            );
        }

        if facts.len() > schedule.steps.len() {
            let extra = facts.len() - schedule.steps.len();
            return Self::empty_like(
                BezierBooleanOwnershipFactStatus::ExtraOwnershipFacts,
                schedule,
                facts.len(),
                0,
                extra,
                0,
                0,
                extra.max(1),
            );
        }

        let step_mismatch_count = schedule
            .steps
            .iter()
            .zip(facts.iter())
            .filter(|(step, fact)| **step != fact.step)
            .count();

        if step_mismatch_count > 0 {
            return Self::empty_like(
                BezierBooleanOwnershipFactStatus::StepMismatch,
                schedule,
                facts.len(),
                0,
                0,
                step_mismatch_count,
                0,
                step_mismatch_count,
            );
        }

        let boundary_fact_count = facts
            .iter()
            .filter(|fact| {
                fact.opposite_location == BezierBooleanFragmentOwnershipLocation::Boundary
            })
            .count();
        let status = if boundary_fact_count == 0 {
            BezierBooleanOwnershipFactStatus::Ready
        } else {
            BezierBooleanOwnershipFactStatus::BoundaryNeedsResolution
        };

        Self {
            status,
            schedule_status: schedule.status,
            scheduled_step_count: schedule.steps.len(),
            supplied_fact_count: facts.len(),
            facts: facts.to_vec(),
            locations: facts
                .iter()
                .map(|fact| fact.opposite_location)
                .collect::<Vec<_>>(),
            missing_fact_count: 0,
            extra_fact_count: 0,
            step_mismatch_count: 0,
            boundary_fact_count,
            blocker_count: boundary_fact_count,
        }
    }

    fn empty_like(
        status: BezierBooleanOwnershipFactStatus,
        schedule: &BezierBooleanTraversalScheduleReport2,
        supplied_fact_count: usize,
        missing_fact_count: usize,
        extra_fact_count: usize,
        step_mismatch_count: usize,
        boundary_fact_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            schedule_status: schedule.status,
            scheduled_step_count: schedule.steps.len(),
            supplied_fact_count,
            facts: Vec::new(),
            locations: Vec::new(),
            missing_fact_count,
            extra_fact_count,
            step_mismatch_count,
            boundary_fact_count,
            blocker_count,
        }
    }

    /// Applies boolean selection to validated facts.
    pub fn classify(
        &self,
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
    ) -> BezierBooleanOwnershipClassificationReport2 {
        BezierBooleanOwnershipClassificationReport2::from_schedule(
            schedule,
            operation,
            &self.locations,
        )
    }

    /// Returns true when every scheduled fragment has a non-boundary fact.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanOwnershipFactStatus::Ready
    }

    /// Returns true when more exact ownership or overlap facts are required.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanOwnershipFactStatus::ScheduleBlocked
                | BezierBooleanOwnershipFactStatus::MissingOwnershipFacts
                | BezierBooleanOwnershipFactStatus::ExtraOwnershipFacts
                | BezierBooleanOwnershipFactStatus::StepMismatch
                | BezierBooleanOwnershipFactStatus::BoundaryNeedsResolution
        )
    }
}

impl BezierBooleanFragmentLocatorInputReport2 {
    /// Builds locator inputs from already computed exact representative points.
    ///
    /// The representative arrays are indexed by fragment index for each source
    /// operand. This low-level constructor is useful for heterogeneous future
    /// paths whose retained fragment type is not yet represented by one of the
    /// typed Bezier/conic report structs. It validates only object identity and
    /// availability; locator decisions still have to be replayed as certified
    /// ownership facts before boolean selection.
    pub fn from_representative_points(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragment_points: &[Point2],
        second_fragment_points: &[Point2],
    ) -> Self {
        match schedule.status {
            BezierBooleanTraversalScheduleStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanFragmentLocatorInputStatus::Empty,
                    schedule,
                    operation,
                    0,
                );
            }
            BezierBooleanTraversalScheduleStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanFragmentLocatorInputStatus::NoInteriorSplits,
                    schedule,
                    operation,
                    0,
                );
            }
            BezierBooleanTraversalScheduleStatus::PreconditionBlocked => {
                return Self::empty_like(
                    BezierBooleanFragmentLocatorInputStatus::ScheduleBlocked,
                    schedule,
                    operation,
                    schedule.blocker_count.max(1),
                );
            }
            BezierBooleanTraversalScheduleStatus::Ready => {}
        }

        let mut missing_first_fragment_count = 0;
        let mut missing_second_fragment_count = 0;
        let mut inputs = Vec::with_capacity(schedule.steps.len());
        for step in &schedule.steps {
            let representative_point = match step.operand {
                BezierBooleanTraversalOperand::First => {
                    match first_fragment_points.get(step.fragment_index) {
                        Some(point) => point.clone(),
                        None => {
                            missing_first_fragment_count += 1;
                            continue;
                        }
                    }
                }
                BezierBooleanTraversalOperand::Second => {
                    match second_fragment_points.get(step.fragment_index) {
                        Some(point) => point.clone(),
                        None => {
                            missing_second_fragment_count += 1;
                            continue;
                        }
                    }
                }
            };
            inputs.push(BezierBooleanFragmentLocatorInput2 {
                step: step.clone(),
                representative_point,
            });
        }

        if missing_first_fragment_count > 0 {
            return Self::blocked_like(
                BezierBooleanFragmentLocatorInputStatus::MissingFirstFragment,
                schedule,
                operation,
                missing_first_fragment_count,
                missing_second_fragment_count,
                0,
            );
        }
        if missing_second_fragment_count > 0 {
            return Self::blocked_like(
                BezierBooleanFragmentLocatorInputStatus::MissingSecondFragment,
                schedule,
                operation,
                missing_first_fragment_count,
                missing_second_fragment_count,
                0,
            );
        }

        Self {
            status: BezierBooleanFragmentLocatorInputStatus::Ready,
            schedule_status: schedule.status,
            operation,
            input_count: inputs.len(),
            scheduled_step_count: schedule.steps.len(),
            inputs,
            missing_first_fragment_count: 0,
            missing_second_fragment_count: 0,
            point_evaluation_blocker_count: 0,
            blocker_count: 0,
        }
    }

    /// Builds locator inputs from quadratic Bezier fragments at `t = 1/2`.
    ///
    /// The midpoint is evaluated by exact de Casteljau construction, preserving
    /// the curve object until the representative point is needed. This is a
    /// query-handoff object, not a fill-state decision.
    pub fn from_quadratic_schedule_midpoints(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
    ) -> Self {
        let half = half_real();
        let first_points: Vec<_> = first
            .fragments
            .iter()
            .map(|fragment| fragment.point_at(half.clone()))
            .collect();
        let second_points: Vec<_> = second
            .fragments
            .iter()
            .map(|fragment| fragment.point_at(half.clone()))
            .collect();
        Self::from_representative_points(schedule, operation, &first_points, &second_points)
    }

    /// Builds locator inputs from cubic Bezier fragments at `t = 1/2`.
    ///
    /// Cubic midpoint evaluation uses exact de Casteljau subdivision, matching
    /// Farin's Bezier construction and Yap's object-preserving exact
    /// computation boundary.
    pub fn from_cubic_schedule_midpoints(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
    ) -> Self {
        let half = half_real();
        let first_points: Vec<_> = first
            .fragments
            .iter()
            .map(|fragment| fragment.point_at(half.clone()))
            .collect();
        let second_points: Vec<_> = second
            .fragments
            .iter()
            .map(|fragment| fragment.point_at(half.clone()))
            .collect();
        Self::from_representative_points(schedule, operation, &first_points, &second_points)
    }

    /// Builds locator inputs from rational quadratic/conic fragments at `t = 1/2`.
    ///
    /// Rational midpoint evaluation is projective: the homogeneous denominator
    /// must be certified before an affine point is emitted. If any midpoint is
    /// uncertain, the report blocks instead of manufacturing a sampled locator
    /// point. This follows Yap (1997) and Farin's rational Bezier model.
    pub fn from_rational_quadratic_schedule_midpoints(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        policy: &CurvePolicy,
    ) -> Self {
        let half = half_real();
        let mut point_evaluation_blocker_count = 0;
        let mut first_points = Vec::with_capacity(first.fragments.len());
        let mut second_points = Vec::with_capacity(second.fragments.len());
        for fragment in &first.fragments {
            match fragment.point_at(half.clone(), policy) {
                Classification::Decided(point) => first_points.push(point),
                Classification::Uncertain(_) => point_evaluation_blocker_count += 1,
            }
        }
        for fragment in &second.fragments {
            match fragment.point_at(half.clone(), policy) {
                Classification::Decided(point) => second_points.push(point),
                Classification::Uncertain(_) => point_evaluation_blocker_count += 1,
            }
        }
        if point_evaluation_blocker_count > 0 {
            return Self::blocked_like(
                BezierBooleanFragmentLocatorInputStatus::PointEvaluationBlocked,
                schedule,
                operation,
                0,
                0,
                point_evaluation_blocker_count,
            );
        }
        Self::from_representative_points(schedule, operation, &first_points, &second_points)
    }

    fn empty_like(
        status: BezierBooleanFragmentLocatorInputStatus,
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            schedule_status: schedule.status,
            operation,
            inputs: Vec::new(),
            scheduled_step_count: schedule.steps.len(),
            input_count: 0,
            missing_first_fragment_count: 0,
            missing_second_fragment_count: 0,
            point_evaluation_blocker_count: 0,
            blocker_count,
        }
    }

    fn blocked_like(
        status: BezierBooleanFragmentLocatorInputStatus,
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        missing_first_fragment_count: usize,
        missing_second_fragment_count: usize,
        point_evaluation_blocker_count: usize,
    ) -> Self {
        Self {
            status,
            schedule_status: schedule.status,
            operation,
            inputs: Vec::new(),
            scheduled_step_count: schedule.steps.len(),
            input_count: 0,
            missing_first_fragment_count,
            missing_second_fragment_count,
            point_evaluation_blocker_count,
            blocker_count: missing_first_fragment_count
                + missing_second_fragment_count
                + point_evaluation_blocker_count,
        }
    }

    /// Returns true when every scheduled fragment has a representative point.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanFragmentLocatorInputStatus::Ready
    }

    /// Returns true when no representative-point query is needed.
    pub fn is_vacuously_complete(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanFragmentLocatorInputStatus::Empty
                | BezierBooleanFragmentLocatorInputStatus::NoInteriorSplits
        )
    }

    /// Returns true when schedule or representative-point generation blocked.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanFragmentLocatorInputStatus::ScheduleBlocked
                | BezierBooleanFragmentLocatorInputStatus::MissingFirstFragment
                | BezierBooleanFragmentLocatorInputStatus::MissingSecondFragment
                | BezierBooleanFragmentLocatorInputStatus::PointEvaluationBlocked
        )
    }
}

impl BezierBooleanOperandOwnershipLocationReport2 {
    /// Expands exact per-operand locator outputs into keyed per-fragment facts.
    pub fn from_schedule_locations(
        schedule: &BezierBooleanTraversalScheduleReport2,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
    ) -> Self {
        match schedule.status {
            BezierBooleanTraversalScheduleStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanOperandOwnershipLocationStatus::Empty,
                    schedule,
                    first_fragments_in_second.len(),
                    second_fragments_in_first.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                );
            }
            BezierBooleanTraversalScheduleStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanOperandOwnershipLocationStatus::NoInteriorSplits,
                    schedule,
                    first_fragments_in_second.len(),
                    second_fragments_in_first.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                );
            }
            BezierBooleanTraversalScheduleStatus::PreconditionBlocked => {
                return Self::empty_like(
                    BezierBooleanOperandOwnershipLocationStatus::ScheduleBlocked,
                    schedule,
                    first_fragments_in_second.len(),
                    second_fragments_in_first.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    schedule.blocker_count.max(1),
                );
            }
            BezierBooleanTraversalScheduleStatus::Ready => {}
        }

        if first_fragments_in_second.len() < schedule.first_fragment_count {
            let missing = schedule.first_fragment_count - first_fragments_in_second.len();
            return Self::empty_like(
                BezierBooleanOperandOwnershipLocationStatus::MissingFirstLocations,
                schedule,
                first_fragments_in_second.len(),
                second_fragments_in_first.len(),
                missing,
                0,
                0,
                0,
                0,
                missing.max(1),
            );
        }
        if second_fragments_in_first.len() < schedule.second_fragment_count {
            let missing = schedule.second_fragment_count - second_fragments_in_first.len();
            return Self::empty_like(
                BezierBooleanOperandOwnershipLocationStatus::MissingSecondLocations,
                schedule,
                first_fragments_in_second.len(),
                second_fragments_in_first.len(),
                0,
                missing,
                0,
                0,
                0,
                missing.max(1),
            );
        }
        if first_fragments_in_second.len() > schedule.first_fragment_count {
            let extra = first_fragments_in_second.len() - schedule.first_fragment_count;
            return Self::empty_like(
                BezierBooleanOperandOwnershipLocationStatus::ExtraFirstLocations,
                schedule,
                first_fragments_in_second.len(),
                second_fragments_in_first.len(),
                0,
                0,
                extra,
                0,
                0,
                extra.max(1),
            );
        }
        if second_fragments_in_first.len() > schedule.second_fragment_count {
            let extra = second_fragments_in_first.len() - schedule.second_fragment_count;
            return Self::empty_like(
                BezierBooleanOperandOwnershipLocationStatus::ExtraSecondLocations,
                schedule,
                first_fragments_in_second.len(),
                second_fragments_in_first.len(),
                0,
                0,
                0,
                extra,
                0,
                extra.max(1),
            );
        }

        let mut facts = Vec::with_capacity(schedule.steps.len());
        let mut boundary_fact_count = 0;
        for step in &schedule.steps {
            let opposite_location = match step.operand {
                BezierBooleanTraversalOperand::First => {
                    let Some(location) = first_fragments_in_second.get(step.fragment_index) else {
                        return Self::empty_like(
                            BezierBooleanOperandOwnershipLocationStatus::MissingFirstLocations,
                            schedule,
                            first_fragments_in_second.len(),
                            second_fragments_in_first.len(),
                            1,
                            0,
                            0,
                            0,
                            0,
                            1,
                        );
                    };
                    *location
                }
                BezierBooleanTraversalOperand::Second => {
                    let Some(location) = second_fragments_in_first.get(step.fragment_index) else {
                        return Self::empty_like(
                            BezierBooleanOperandOwnershipLocationStatus::MissingSecondLocations,
                            schedule,
                            first_fragments_in_second.len(),
                            second_fragments_in_first.len(),
                            0,
                            1,
                            0,
                            0,
                            0,
                            1,
                        );
                    };
                    *location
                }
            };
            if opposite_location == BezierBooleanFragmentOwnershipLocation::Boundary {
                boundary_fact_count += 1;
            }
            facts.push(BezierBooleanOwnershipFact2 {
                step: step.clone(),
                opposite_location,
            });
        }

        Self {
            status: if boundary_fact_count == 0 {
                BezierBooleanOperandOwnershipLocationStatus::Ready
            } else {
                BezierBooleanOperandOwnershipLocationStatus::BoundaryNeedsResolution
            },
            schedule_status: schedule.status,
            first_fragment_count: schedule.first_fragment_count,
            second_fragment_count: schedule.second_fragment_count,
            supplied_first_location_count: first_fragments_in_second.len(),
            supplied_second_location_count: second_fragments_in_first.len(),
            facts,
            missing_first_location_count: 0,
            missing_second_location_count: 0,
            extra_first_location_count: 0,
            extra_second_location_count: 0,
            boundary_fact_count,
            blocker_count: boundary_fact_count,
        }
    }

    fn empty_like(
        status: BezierBooleanOperandOwnershipLocationStatus,
        schedule: &BezierBooleanTraversalScheduleReport2,
        supplied_first_location_count: usize,
        supplied_second_location_count: usize,
        missing_first_location_count: usize,
        missing_second_location_count: usize,
        extra_first_location_count: usize,
        extra_second_location_count: usize,
        boundary_fact_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            schedule_status: schedule.status,
            first_fragment_count: schedule.first_fragment_count,
            second_fragment_count: schedule.second_fragment_count,
            supplied_first_location_count,
            supplied_second_location_count,
            facts: Vec::new(),
            missing_first_location_count,
            missing_second_location_count,
            extra_first_location_count,
            extra_second_location_count,
            boundary_fact_count,
            blocker_count,
        }
    }

    /// Applies ordinary keyed fact validation to the expanded facts.
    pub fn validate(
        &self,
        schedule: &BezierBooleanTraversalScheduleReport2,
    ) -> BezierBooleanOwnershipFactReport2 {
        BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, &self.facts)
    }

    /// Returns true when all generated facts are present and non-boundary.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanOperandOwnershipLocationStatus::Ready
    }

    /// Returns true when schedule, counts, or boundary policy prevents fact use.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanOperandOwnershipLocationStatus::ScheduleBlocked
                | BezierBooleanOperandOwnershipLocationStatus::MissingFirstLocations
                | BezierBooleanOperandOwnershipLocationStatus::MissingSecondLocations
                | BezierBooleanOperandOwnershipLocationStatus::ExtraFirstLocations
                | BezierBooleanOperandOwnershipLocationStatus::ExtraSecondLocations
                | BezierBooleanOperandOwnershipLocationStatus::BoundaryNeedsResolution
        )
    }
}

impl BezierBooleanUniformOwnershipFactReport2 {
    /// Expands two operand-level locator facts into keyed per-fragment facts.
    pub fn from_schedule_locations(
        schedule: &BezierBooleanTraversalScheduleReport2,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
    ) -> Self {
        match schedule.status {
            BezierBooleanTraversalScheduleStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanUniformOwnershipFactStatus::Empty,
                    schedule,
                    first_fragments_in_second,
                    second_fragments_in_first,
                    0,
                    0,
                );
            }
            BezierBooleanTraversalScheduleStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanUniformOwnershipFactStatus::NoInteriorSplits,
                    schedule,
                    first_fragments_in_second,
                    second_fragments_in_first,
                    0,
                    0,
                );
            }
            BezierBooleanTraversalScheduleStatus::PreconditionBlocked => {
                return Self::empty_like(
                    BezierBooleanUniformOwnershipFactStatus::ScheduleBlocked,
                    schedule,
                    first_fragments_in_second,
                    second_fragments_in_first,
                    0,
                    schedule.blocker_count.max(1),
                );
            }
            BezierBooleanTraversalScheduleStatus::Ready => {}
        }

        let mut facts = Vec::with_capacity(schedule.steps.len());
        let mut boundary_fact_count = 0;
        for step in &schedule.steps {
            let opposite_location = match step.operand {
                BezierBooleanTraversalOperand::First => first_fragments_in_second,
                BezierBooleanTraversalOperand::Second => second_fragments_in_first,
            };
            if opposite_location == BezierBooleanFragmentOwnershipLocation::Boundary {
                boundary_fact_count += 1;
            }
            facts.push(BezierBooleanOwnershipFact2 {
                step: step.clone(),
                opposite_location,
            });
        }

        Self {
            status: if boundary_fact_count == 0 {
                BezierBooleanUniformOwnershipFactStatus::Ready
            } else {
                BezierBooleanUniformOwnershipFactStatus::BoundaryNeedsResolution
            },
            schedule_status: schedule.status,
            first_fragments_in_second,
            second_fragments_in_first,
            first_fragment_count: schedule.first_fragment_count,
            second_fragment_count: schedule.second_fragment_count,
            facts,
            boundary_fact_count,
            blocker_count: boundary_fact_count,
        }
    }

    fn empty_like(
        status: BezierBooleanUniformOwnershipFactStatus,
        schedule: &BezierBooleanTraversalScheduleReport2,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        boundary_fact_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            schedule_status: schedule.status,
            first_fragments_in_second,
            second_fragments_in_first,
            first_fragment_count: schedule.first_fragment_count,
            second_fragment_count: schedule.second_fragment_count,
            facts: Vec::new(),
            boundary_fact_count,
            blocker_count,
        }
    }

    /// Applies ordinary keyed fact validation to the expanded facts.
    pub fn validate(
        &self,
        schedule: &BezierBooleanTraversalScheduleReport2,
    ) -> BezierBooleanOwnershipFactReport2 {
        BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, &self.facts)
    }

    /// Returns true when all generated facts are non-boundary facts.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanUniformOwnershipFactStatus::Ready
    }

    /// Returns true when schedule or boundary policy prevents fact use.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanUniformOwnershipFactStatus::ScheduleBlocked
                | BezierBooleanUniformOwnershipFactStatus::BoundaryNeedsResolution
        )
    }
}

impl BezierBooleanOwnershipClassificationReport2 {
    /// Classifies scheduled fragments using caller-certified ownership facts.
    pub fn from_schedule(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownerships: &[BezierBooleanFragmentOwnershipLocation],
    ) -> Self {
        match schedule.status {
            BezierBooleanTraversalScheduleStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanOwnershipClassificationStatus::Empty,
                    operation,
                    schedule,
                    ownerships.len(),
                    0,
                    0,
                );
            }
            BezierBooleanTraversalScheduleStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanOwnershipClassificationStatus::NoInteriorSplits,
                    operation,
                    schedule,
                    ownerships.len(),
                    0,
                    0,
                );
            }
            BezierBooleanTraversalScheduleStatus::PreconditionBlocked => {
                return Self::empty_like(
                    BezierBooleanOwnershipClassificationStatus::ScheduleBlocked,
                    operation,
                    schedule,
                    ownerships.len(),
                    0,
                    schedule.blocker_count.max(1),
                );
            }
            BezierBooleanTraversalScheduleStatus::Ready => {}
        }

        if ownerships.len() != schedule.steps.len() {
            let missing = schedule.steps.len().saturating_sub(ownerships.len());
            return Self::empty_like(
                BezierBooleanOwnershipClassificationStatus::MissingOwnershipFacts,
                operation,
                schedule,
                ownerships.len(),
                missing,
                missing.max(1),
            );
        }

        let mut owned_steps = Vec::with_capacity(schedule.steps.len());
        let mut keep_source_count = 0;
        let mut keep_reversed_count = 0;
        let mut discard_count = 0;
        let mut boundary_blocker_count = 0;

        for (step, opposite_location) in schedule.steps.iter().zip(ownerships.iter().copied()) {
            let action =
                material_action_for_bezier_step(operation, step.operand, opposite_location);
            match action {
                BooleanFragmentAction::KeepSourceDirection => keep_source_count += 1,
                BooleanFragmentAction::KeepReversed => keep_reversed_count += 1,
                BooleanFragmentAction::Discard => discard_count += 1,
                BooleanFragmentAction::BoundaryNeedsResolution => boundary_blocker_count += 1,
            }
            owned_steps.push(BezierBooleanOwnedTraversalStep2 {
                step: step.clone(),
                opposite_location,
                action,
            });
        }

        let status = if boundary_blocker_count == 0 {
            BezierBooleanOwnershipClassificationStatus::Ready
        } else {
            BezierBooleanOwnershipClassificationStatus::BoundaryNeedsResolution
        };

        Self {
            status,
            operation,
            schedule_status: schedule.status,
            scheduled_step_count: schedule.steps.len(),
            supplied_ownership_count: ownerships.len(),
            owned_steps,
            keep_source_count,
            keep_reversed_count,
            discard_count,
            boundary_blocker_count,
            missing_ownership_count: 0,
            blocker_count: boundary_blocker_count,
        }
    }

    fn empty_like(
        status: BezierBooleanOwnershipClassificationStatus,
        operation: BooleanOp,
        schedule: &BezierBooleanTraversalScheduleReport2,
        supplied_ownership_count: usize,
        missing_ownership_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            operation,
            schedule_status: schedule.status,
            scheduled_step_count: schedule.steps.len(),
            supplied_ownership_count,
            owned_steps: Vec::new(),
            keep_source_count: 0,
            keep_reversed_count: 0,
            discard_count: 0,
            boundary_blocker_count: 0,
            missing_ownership_count,
            blocker_count,
        }
    }

    /// Returns true when all scheduled fragments have usable ownership actions.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanOwnershipClassificationStatus::Ready
    }

    /// Returns true when ownership classification needs more certified facts.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanOwnershipClassificationStatus::ScheduleBlocked
                | BezierBooleanOwnershipClassificationStatus::MissingOwnershipFacts
                | BezierBooleanOwnershipClassificationStatus::BoundaryNeedsResolution
        )
    }
}

impl BezierBooleanEmissionPlanReport2 {
    /// Builds an emission plan from ownership-classified traversal steps.
    pub fn from_ownership(ownership: &BezierBooleanOwnershipClassificationReport2) -> Self {
        match ownership.status {
            BezierBooleanOwnershipClassificationStatus::Empty => {
                return Self::empty_like(BezierBooleanEmissionPlanStatus::Empty, ownership, 0);
            }
            BezierBooleanOwnershipClassificationStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanEmissionPlanStatus::NoInteriorSplits,
                    ownership,
                    0,
                );
            }
            BezierBooleanOwnershipClassificationStatus::ScheduleBlocked
            | BezierBooleanOwnershipClassificationStatus::MissingOwnershipFacts
            | BezierBooleanOwnershipClassificationStatus::BoundaryNeedsResolution => {
                return Self::empty_like(
                    BezierBooleanEmissionPlanStatus::OwnershipBlocked,
                    ownership,
                    ownership.blocker_count.max(1),
                );
            }
            BezierBooleanOwnershipClassificationStatus::Ready => {}
        }

        let mut emitted_steps = Vec::new();
        let mut discarded_steps = Vec::new();
        for step in &ownership.owned_steps {
            if step.action.emits_fragment() {
                emitted_steps.push(step.clone());
            } else {
                discarded_steps.push(step.clone());
            }
        }

        let status = if emitted_steps.is_empty() {
            BezierBooleanEmissionPlanStatus::NoEmittedFragments
        } else {
            BezierBooleanEmissionPlanStatus::Ready
        };

        Self {
            status,
            ownership_status: ownership.status,
            operation: ownership.operation,
            emitted_steps,
            discarded_steps,
            keep_source_count: ownership.keep_source_count,
            keep_reversed_count: ownership.keep_reversed_count,
            discard_count: ownership.discard_count,
            boundary_blocker_count: ownership.boundary_blocker_count,
            missing_ownership_count: ownership.missing_ownership_count,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanEmissionPlanStatus,
        ownership: &BezierBooleanOwnershipClassificationReport2,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            ownership_status: ownership.status,
            operation: ownership.operation,
            emitted_steps: Vec::new(),
            discarded_steps: Vec::new(),
            keep_source_count: 0,
            keep_reversed_count: 0,
            discard_count: ownership.discard_count,
            boundary_blocker_count: ownership.boundary_blocker_count,
            missing_ownership_count: ownership.missing_ownership_count,
            blocker_count,
        }
    }

    /// Returns true when at least one selected fragment can feed loop assembly.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanEmissionPlanStatus::Ready
    }

    /// Returns true when blockers prevent trusted emission.
    pub fn has_blockers(&self) -> bool {
        self.status == BezierBooleanEmissionPlanStatus::OwnershipBlocked
    }
}

impl BezierBooleanAssemblyReadinessReport2 {
    /// Audits an emission plan against quadratic Bezier fragment reports.
    pub fn from_quadratic_fragments(
        plan: &BezierBooleanEmissionPlanReport2,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
    ) -> Self {
        Self::from_fragment_counts(plan, first.fragments.len(), second.fragments.len())
    }

    /// Audits an emission plan against cubic Bezier fragment reports.
    pub fn from_cubic_fragments(
        plan: &BezierBooleanEmissionPlanReport2,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
    ) -> Self {
        Self::from_fragment_counts(plan, first.fragments.len(), second.fragments.len())
    }

    /// Audits an emission plan against rational quadratic/conic fragment reports.
    pub fn from_rational_quadratic_fragments(
        plan: &BezierBooleanEmissionPlanReport2,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
    ) -> Self {
        Self::from_fragment_counts(plan, first.fragments.len(), second.fragments.len())
    }

    /// Audits an emission plan against generic operand fragment counts.
    pub fn from_fragment_counts(
        plan: &BezierBooleanEmissionPlanReport2,
        first_fragment_count: usize,
        second_fragment_count: usize,
    ) -> Self {
        match plan.status {
            BezierBooleanEmissionPlanStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanAssemblyReadinessStatus::Empty,
                    plan,
                    first_fragment_count,
                    second_fragment_count,
                    0,
                );
            }
            BezierBooleanEmissionPlanStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanAssemblyReadinessStatus::NoInteriorSplits,
                    plan,
                    first_fragment_count,
                    second_fragment_count,
                    0,
                );
            }
            BezierBooleanEmissionPlanStatus::OwnershipBlocked => {
                return Self::empty_like(
                    BezierBooleanAssemblyReadinessStatus::EmissionBlocked,
                    plan,
                    first_fragment_count,
                    second_fragment_count,
                    plan.blocker_count.max(1),
                );
            }
            BezierBooleanEmissionPlanStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanAssemblyReadinessStatus::NoEmittedFragments,
                    plan,
                    first_fragment_count,
                    second_fragment_count,
                    0,
                );
            }
            BezierBooleanEmissionPlanStatus::Ready => {}
        }

        let mut first_emitted_count = 0;
        let mut second_emitted_count = 0;
        let mut invalid_reference_count = 0;
        for emitted in &plan.emitted_steps {
            match emitted.step.operand {
                BezierBooleanTraversalOperand::First => {
                    first_emitted_count += 1;
                    if emitted.step.fragment_index >= first_fragment_count {
                        invalid_reference_count += 1;
                    }
                }
                BezierBooleanTraversalOperand::Second => {
                    second_emitted_count += 1;
                    if emitted.step.fragment_index >= second_fragment_count {
                        invalid_reference_count += 1;
                    }
                }
            }
        }

        let status = if invalid_reference_count == 0 {
            BezierBooleanAssemblyReadinessStatus::Ready
        } else {
            BezierBooleanAssemblyReadinessStatus::InvalidFragmentReference
        };

        Self {
            status,
            emission_status: plan.status,
            first_fragment_count,
            second_fragment_count,
            emitted_step_count: plan.emitted_steps.len(),
            first_emitted_count,
            second_emitted_count,
            invalid_reference_count,
            keep_source_count: plan.keep_source_count,
            keep_reversed_count: plan.keep_reversed_count,
            blocker_count: invalid_reference_count,
        }
    }

    fn empty_like(
        status: BezierBooleanAssemblyReadinessStatus,
        plan: &BezierBooleanEmissionPlanReport2,
        first_fragment_count: usize,
        second_fragment_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            emission_status: plan.status,
            first_fragment_count,
            second_fragment_count,
            emitted_step_count: 0,
            first_emitted_count: 0,
            second_emitted_count: 0,
            invalid_reference_count: 0,
            keep_source_count: 0,
            keep_reversed_count: 0,
            blocker_count,
        }
    }

    /// Returns true when emitted references can feed future loop assembly.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanAssemblyReadinessStatus::Ready
    }

    /// Returns true when blocked or invalid references prevent loop assembly.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanAssemblyReadinessStatus::EmissionBlocked
                | BezierBooleanAssemblyReadinessStatus::InvalidFragmentReference
        )
    }
}

impl BezierBooleanLoopAssemblyPlanReport2 {
    /// Packages assembly-ready emitted references for future loop construction.
    pub fn from_assembly_readiness(
        readiness: &BezierBooleanAssemblyReadinessReport2,
        emission: &BezierBooleanEmissionPlanReport2,
    ) -> Self {
        match readiness.status {
            BezierBooleanAssemblyReadinessStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopAssemblyPlanStatus::Empty,
                    readiness,
                    emission,
                    0,
                );
            }
            BezierBooleanAssemblyReadinessStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopAssemblyPlanStatus::NoInteriorSplits,
                    readiness,
                    emission,
                    0,
                );
            }
            BezierBooleanAssemblyReadinessStatus::EmissionBlocked
            | BezierBooleanAssemblyReadinessStatus::InvalidFragmentReference => {
                return Self::empty_like(
                    BezierBooleanLoopAssemblyPlanStatus::AssemblyBlocked,
                    readiness,
                    emission,
                    readiness.blocker_count.max(1),
                );
            }
            BezierBooleanAssemblyReadinessStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopAssemblyPlanStatus::NoEmittedFragments,
                    readiness,
                    emission,
                    0,
                );
            }
            BezierBooleanAssemblyReadinessStatus::Ready => {}
        }

        Self {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: readiness.status,
            operation: emission.operation,
            emitted_steps: emission.emitted_steps.clone(),
            first_emitted_count: readiness.first_emitted_count,
            second_emitted_count: readiness.second_emitted_count,
            keep_source_count: readiness.keep_source_count,
            keep_reversed_count: readiness.keep_reversed_count,
            invalid_reference_count: 0,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopAssemblyPlanStatus,
        readiness: &BezierBooleanAssemblyReadinessReport2,
        emission: &BezierBooleanEmissionPlanReport2,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            assembly_status: readiness.status,
            operation: emission.operation,
            emitted_steps: Vec::new(),
            first_emitted_count: 0,
            second_emitted_count: 0,
            keep_source_count: 0,
            keep_reversed_count: 0,
            invalid_reference_count: readiness.invalid_reference_count,
            blocker_count,
        }
    }

    /// Returns true when emitted references are ready for loop construction.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopAssemblyPlanStatus::Ready
    }

    /// Returns true when invalid or blocked assembly preconditions remain.
    pub fn has_blockers(&self) -> bool {
        self.status == BezierBooleanLoopAssemblyPlanStatus::AssemblyBlocked
    }
}

impl BezierBooleanLoopGraphFactReport2 {
    /// Validates graph facts against a loop-assembly plan.
    ///
    /// The supplied fact must be keyed to the plan's emitted-step count before
    /// branch or resolved-overlap counts are trusted. Nonzero graph obligations
    /// remain explicit blockers so a later certified traversal can resolve
    /// them without this layer guessing a walk order.
    pub fn from_plan_facts(
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        facts: &BezierBooleanLoopGraphFacts2,
    ) -> Self {
        match plan.status {
            BezierBooleanLoopAssemblyPlanStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopGraphFactStatus::Empty,
                    plan,
                    facts,
                    0,
                    0,
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopGraphFactStatus::NoInteriorSplits,
                    plan,
                    facts,
                    0,
                    0,
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::AssemblyBlocked => {
                return Self::empty_like(
                    BezierBooleanLoopGraphFactStatus::PlanBlocked,
                    plan,
                    facts,
                    0,
                    plan.blocker_count.max(1),
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopGraphFactStatus::NoEmittedFragments,
                    plan,
                    facts,
                    0,
                    0,
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::Ready => {}
        }

        if facts.emitted_step_count != plan.emitted_steps.len() {
            return Self::empty_like(
                BezierBooleanLoopGraphFactStatus::EmittedStepCountMismatch,
                plan,
                facts,
                1,
                1,
            );
        }

        if facts.branch_vertex_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphFactStatus::BranchPointsNeedTraversal,
                plan,
                facts,
                0,
                facts.branch_vertex_count,
            );
        }

        if facts.resolved_overlap_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphFactStatus::ResolvedOverlapsNeedTraversal,
                plan,
                facts,
                0,
                facts.resolved_overlap_count,
            );
        }

        Self {
            status: BezierBooleanLoopGraphFactStatus::Ready,
            plan_status: plan.status,
            operation: plan.operation,
            emitted_step_count: plan.emitted_steps.len(),
            supplied_emitted_step_count: facts.emitted_step_count,
            branch_vertex_count: facts.branch_vertex_count,
            resolved_overlap_count: facts.resolved_overlap_count,
            emitted_step_mismatch_count: 0,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopGraphFactStatus,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        facts: &BezierBooleanLoopGraphFacts2,
        emitted_step_mismatch_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            plan_status: plan.status,
            operation: plan.operation,
            emitted_step_count: plan.emitted_steps.len(),
            supplied_emitted_step_count: facts.emitted_step_count,
            branch_vertex_count: facts.branch_vertex_count,
            resolved_overlap_count: facts.resolved_overlap_count,
            emitted_step_mismatch_count,
            blocker_count,
        }
    }

    /// Converts validated graph facts into the existing traversal audit report.
    pub fn to_traversal_report(
        &self,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
    ) -> BezierBooleanLoopGraphTraversalReport2 {
        if self.status == BezierBooleanLoopGraphFactStatus::Ready {
            BezierBooleanLoopGraphTraversalReport2::from_plan_graph_facts(
                plan,
                self.branch_vertex_count,
                self.resolved_overlap_count,
            )
        } else {
            BezierBooleanLoopGraphTraversalReport2 {
                status: match self.status {
                    BezierBooleanLoopGraphFactStatus::Empty => {
                        BezierBooleanLoopGraphTraversalStatus::Empty
                    }
                    BezierBooleanLoopGraphFactStatus::NoInteriorSplits => {
                        BezierBooleanLoopGraphTraversalStatus::NoInteriorSplits
                    }
                    BezierBooleanLoopGraphFactStatus::NoEmittedFragments => {
                        BezierBooleanLoopGraphTraversalStatus::NoEmittedFragments
                    }
                    BezierBooleanLoopGraphFactStatus::BranchPointsNeedTraversal => {
                        BezierBooleanLoopGraphTraversalStatus::BranchPointsNeedTraversal
                    }
                    BezierBooleanLoopGraphFactStatus::ResolvedOverlapsNeedTraversal => {
                        BezierBooleanLoopGraphTraversalStatus::ResolvedOverlapsNeedTraversal
                    }
                    BezierBooleanLoopGraphFactStatus::Ready => {
                        BezierBooleanLoopGraphTraversalStatus::Ready
                    }
                    BezierBooleanLoopGraphFactStatus::PlanBlocked
                    | BezierBooleanLoopGraphFactStatus::EmittedStepCountMismatch => {
                        BezierBooleanLoopGraphTraversalStatus::PlanBlocked
                    }
                },
                plan_status: plan.status,
                operation: self.operation,
                emitted_step_count: plan.emitted_steps.len(),
                branch_vertex_count: self.branch_vertex_count,
                resolved_overlap_count: self.resolved_overlap_count,
                blocker_count: self.blocker_count.max(usize::from(self.has_blockers())),
            }
        }
    }

    /// Converts keyed graph facts into a traversal report for an explicit walk.
    ///
    /// [`Self::to_traversal_report`] preserves branch vertices and resolved
    /// overlaps as blockers because identity closure has no graph walk to
    /// justify a reorder. This variant is used only by constructors that also
    /// validate a caller-supplied walk permutation. In that setting nonzero
    /// branch/overlap counts are certified obligations already handled by the
    /// external graph walker, so the traversal report is ready once the graph
    /// fact is keyed to the emitted plan. Emitted-step mismatches and blocked
    /// plans remain blockers. This is the Yap (1997) predicate/construction
    /// boundary: topology may advance only when the exact graph-walk
    /// certificate is supplied as data. The traversal role follows Vatti
    /// (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn to_certified_walk_traversal_report(
        &self,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
    ) -> BezierBooleanLoopGraphTraversalReport2 {
        match self.status {
            BezierBooleanLoopGraphFactStatus::Ready
            | BezierBooleanLoopGraphFactStatus::BranchPointsNeedTraversal
            | BezierBooleanLoopGraphFactStatus::ResolvedOverlapsNeedTraversal => {
                BezierBooleanLoopGraphTraversalReport2 {
                    status: BezierBooleanLoopGraphTraversalStatus::Ready,
                    plan_status: plan.status,
                    operation: self.operation,
                    emitted_step_count: plan.emitted_steps.len(),
                    branch_vertex_count: self.branch_vertex_count,
                    resolved_overlap_count: self.resolved_overlap_count,
                    blocker_count: 0,
                }
            }
            BezierBooleanLoopGraphFactStatus::Empty => BezierBooleanLoopGraphTraversalReport2 {
                status: BezierBooleanLoopGraphTraversalStatus::Empty,
                plan_status: plan.status,
                operation: self.operation,
                emitted_step_count: plan.emitted_steps.len(),
                branch_vertex_count: self.branch_vertex_count,
                resolved_overlap_count: self.resolved_overlap_count,
                blocker_count: 0,
            },
            BezierBooleanLoopGraphFactStatus::NoInteriorSplits => {
                BezierBooleanLoopGraphTraversalReport2 {
                    status: BezierBooleanLoopGraphTraversalStatus::NoInteriorSplits,
                    plan_status: plan.status,
                    operation: self.operation,
                    emitted_step_count: plan.emitted_steps.len(),
                    branch_vertex_count: self.branch_vertex_count,
                    resolved_overlap_count: self.resolved_overlap_count,
                    blocker_count: 0,
                }
            }
            BezierBooleanLoopGraphFactStatus::NoEmittedFragments => {
                BezierBooleanLoopGraphTraversalReport2 {
                    status: BezierBooleanLoopGraphTraversalStatus::NoEmittedFragments,
                    plan_status: plan.status,
                    operation: self.operation,
                    emitted_step_count: plan.emitted_steps.len(),
                    branch_vertex_count: self.branch_vertex_count,
                    resolved_overlap_count: self.resolved_overlap_count,
                    blocker_count: 0,
                }
            }
            BezierBooleanLoopGraphFactStatus::PlanBlocked
            | BezierBooleanLoopGraphFactStatus::EmittedStepCountMismatch => {
                BezierBooleanLoopGraphTraversalReport2 {
                    status: BezierBooleanLoopGraphTraversalStatus::PlanBlocked,
                    plan_status: plan.status,
                    operation: self.operation,
                    emitted_step_count: plan.emitted_steps.len(),
                    branch_vertex_count: self.branch_vertex_count,
                    resolved_overlap_count: self.resolved_overlap_count,
                    blocker_count: self.blocker_count.max(1),
                }
            }
        }
    }

    /// Returns true when graph facts allow linear endpoint closure.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopGraphFactStatus::Ready
    }

    /// Returns true when graph facts are stale or require traversal work.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopGraphFactStatus::PlanBlocked
                | BezierBooleanLoopGraphFactStatus::EmittedStepCountMismatch
                | BezierBooleanLoopGraphFactStatus::BranchPointsNeedTraversal
                | BezierBooleanLoopGraphFactStatus::ResolvedOverlapsNeedTraversal
        )
    }
}

impl BezierBooleanLoopGraphTraversalReport2 {
    /// Audits whether an emitted plan can use linear endpoint closure.
    ///
    /// `branch_vertex_count` counts arrangement vertices with more than the
    /// ordinary in/out degree for a single boundary walk. `resolved_overlap_count`
    /// counts coincident ranges that have split boundaries but still need
    /// degenerate-overlap traversal policy. The report does not discover those
    /// facts; it validates externally certified graph facts before the current
    /// closure layer consumes emitted order as topology.
    pub fn from_plan_graph_facts(
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
    ) -> Self {
        match plan.status {
            BezierBooleanLoopAssemblyPlanStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopGraphTraversalStatus::Empty,
                    plan,
                    branch_vertex_count,
                    resolved_overlap_count,
                    0,
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopGraphTraversalStatus::NoInteriorSplits,
                    plan,
                    branch_vertex_count,
                    resolved_overlap_count,
                    0,
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::AssemblyBlocked => {
                return Self::empty_like(
                    BezierBooleanLoopGraphTraversalStatus::PlanBlocked,
                    plan,
                    branch_vertex_count,
                    resolved_overlap_count,
                    plan.blocker_count.max(1),
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopGraphTraversalStatus::NoEmittedFragments,
                    plan,
                    branch_vertex_count,
                    resolved_overlap_count,
                    0,
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::Ready => {}
        }

        if branch_vertex_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphTraversalStatus::BranchPointsNeedTraversal,
                plan,
                branch_vertex_count,
                resolved_overlap_count,
                branch_vertex_count,
            );
        }

        if resolved_overlap_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphTraversalStatus::ResolvedOverlapsNeedTraversal,
                plan,
                branch_vertex_count,
                resolved_overlap_count,
                resolved_overlap_count,
            );
        }

        Self {
            status: BezierBooleanLoopGraphTraversalStatus::Ready,
            plan_status: plan.status,
            operation: plan.operation,
            emitted_step_count: plan.emitted_steps.len(),
            branch_vertex_count,
            resolved_overlap_count,
            blocker_count: 0,
        }
    }

    /// Builds traversal readiness for a caller-certified explicit graph walk.
    ///
    /// [`Self::from_plan_graph_facts`] deliberately blocks nonzero branch
    /// vertices and resolved overlaps because the emitted order alone is not a
    /// proof of boundary traversal. This constructor is the counterpart for
    /// APIs that also validate a caller-supplied walk permutation. In that
    /// context the branch/overlap counts are retained as audited graph facts,
    /// but they no longer block traversal once the plan itself is ready.
    /// Degenerate overlap policy remains an external certificate producer, as
    /// in Foster, Hormann, and Popa (2019); the exact-computation contract is
    /// Yap, "Towards Exact Geometric Computation" (1997). The staged traversal
    /// model follows Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009).
    pub fn from_certified_walk_graph_facts(
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
    ) -> Self {
        match plan.status {
            BezierBooleanLoopAssemblyPlanStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopGraphTraversalStatus::Empty,
                    plan,
                    branch_vertex_count,
                    resolved_overlap_count,
                    0,
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopGraphTraversalStatus::NoInteriorSplits,
                    plan,
                    branch_vertex_count,
                    resolved_overlap_count,
                    0,
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::AssemblyBlocked => {
                return Self::empty_like(
                    BezierBooleanLoopGraphTraversalStatus::PlanBlocked,
                    plan,
                    branch_vertex_count,
                    resolved_overlap_count,
                    plan.blocker_count.max(1),
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopGraphTraversalStatus::NoEmittedFragments,
                    plan,
                    branch_vertex_count,
                    resolved_overlap_count,
                    0,
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::Ready => {}
        }

        Self {
            status: BezierBooleanLoopGraphTraversalStatus::Ready,
            plan_status: plan.status,
            operation: plan.operation,
            emitted_step_count: plan.emitted_steps.len(),
            branch_vertex_count,
            resolved_overlap_count,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopGraphTraversalStatus,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            plan_status: plan.status,
            operation: plan.operation,
            emitted_step_count: plan.emitted_steps.len(),
            branch_vertex_count,
            resolved_overlap_count,
            blocker_count,
        }
    }

    /// Returns true when emitted order can feed exact endpoint closure.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopGraphTraversalStatus::Ready
    }

    /// Returns true when branch or overlap graph traversal is still required.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopGraphTraversalStatus::PlanBlocked
                | BezierBooleanLoopGraphTraversalStatus::BranchPointsNeedTraversal
                | BezierBooleanLoopGraphTraversalStatus::ResolvedOverlapsNeedTraversal
        )
    }
}

impl BezierBooleanLoopGraphWalkReport2 {
    /// Builds the certified identity walk for a ready linear traversal.
    ///
    /// This constructor is the built-in graph-walk producer for the simplest
    /// arrangement case: the graph facts have already certified that there are
    /// no branch vertices and no resolved-overlap traversal obligations, so the
    /// emitted order is itself the boundary walk. It is still report-bearing:
    /// blocked traversal states are preserved through
    /// [`Self::from_traversal_order`] rather than converted into an empty walk.
    /// This follows Yap, "Towards Exact Geometric Computation" (1997), by
    /// making the combinatorial walk an explicit certificate. The phase split
    /// matches Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009): only a certified traversal result may feed
    /// output-boundary closure.
    pub fn from_identity_traversal(
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
    ) -> Self {
        let walk_indices = if traversal.status == BezierBooleanLoopGraphTraversalStatus::Ready {
            (0..plan.emitted_steps.len()).collect::<Vec<_>>()
        } else {
            Vec::new()
        };
        Self::from_traversal_order(traversal, plan, &walk_indices)
    }

    /// Validates a certified graph-walk order against an emitted-fragment plan.
    ///
    /// `walk_indices` names entries in
    /// [`BezierBooleanLoopAssemblyPlanReport2::emitted_steps`]. A ready report
    /// requires a complete permutation, because the current closure layer
    /// treats the resulting order as a boundary walk. This keeps traversal as
    /// certified graph data rather than a tolerance-derived reorder.
    pub fn from_traversal_order(
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        walk_indices: &[usize],
    ) -> Self {
        match traversal.status {
            BezierBooleanLoopGraphTraversalStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopGraphWalkStatus::Empty,
                    traversal,
                    plan,
                    walk_indices.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                );
            }
            BezierBooleanLoopGraphTraversalStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopGraphWalkStatus::NoInteriorSplits,
                    traversal,
                    plan,
                    walk_indices.len(),
                    0,
                    walk_indices.len(),
                    0,
                    0,
                    walk_indices.len(),
                );
            }
            BezierBooleanLoopGraphTraversalStatus::PlanBlocked
            | BezierBooleanLoopGraphTraversalStatus::BranchPointsNeedTraversal
            | BezierBooleanLoopGraphTraversalStatus::ResolvedOverlapsNeedTraversal => {
                return Self::empty_like(
                    BezierBooleanLoopGraphWalkStatus::TraversalBlocked,
                    traversal,
                    plan,
                    walk_indices.len(),
                    0,
                    0,
                    0,
                    0,
                    traversal.blocker_count.max(1),
                );
            }
            BezierBooleanLoopGraphTraversalStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopGraphWalkStatus::NoEmittedFragments,
                    traversal,
                    plan,
                    walk_indices.len(),
                    0,
                    walk_indices.len(),
                    0,
                    0,
                    walk_indices.len(),
                );
            }
            BezierBooleanLoopGraphTraversalStatus::Ready => {}
        }

        if walk_indices.len() < plan.emitted_steps.len() {
            let missing = plan.emitted_steps.len() - walk_indices.len();
            return Self::empty_like(
                BezierBooleanLoopGraphWalkStatus::MissingWalkSteps,
                traversal,
                plan,
                walk_indices.len(),
                missing,
                0,
                0,
                0,
                missing.max(1),
            );
        }

        if walk_indices.len() > plan.emitted_steps.len() {
            let extra = walk_indices.len() - plan.emitted_steps.len();
            return Self::empty_like(
                BezierBooleanLoopGraphWalkStatus::ExtraWalkSteps,
                traversal,
                plan,
                walk_indices.len(),
                0,
                extra,
                0,
                0,
                extra.max(1),
            );
        }

        let out_of_range_walk_step_count = walk_indices
            .iter()
            .filter(|index| **index >= plan.emitted_steps.len())
            .count();
        if out_of_range_walk_step_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphWalkStatus::OutOfRangeWalkStep,
                traversal,
                plan,
                walk_indices.len(),
                0,
                0,
                out_of_range_walk_step_count,
                0,
                out_of_range_walk_step_count,
            );
        }

        let mut seen = vec![false; plan.emitted_steps.len()];
        let mut duplicate_walk_step_count = 0;
        for index in walk_indices {
            if seen[*index] {
                duplicate_walk_step_count += 1;
            }
            seen[*index] = true;
        }
        if duplicate_walk_step_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphWalkStatus::DuplicateWalkStep,
                traversal,
                plan,
                walk_indices.len(),
                0,
                0,
                0,
                duplicate_walk_step_count,
                duplicate_walk_step_count,
            );
        }

        Self {
            status: BezierBooleanLoopGraphWalkStatus::Ready,
            traversal_status: traversal.status,
            operation: traversal.operation,
            emitted_step_count: plan.emitted_steps.len(),
            supplied_walk_step_count: walk_indices.len(),
            walk_indices: walk_indices.to_vec(),
            ordered_steps: walk_indices
                .iter()
                .map(|index| plan.emitted_steps[*index].clone())
                .collect(),
            missing_walk_step_count: 0,
            extra_walk_step_count: 0,
            out_of_range_walk_step_count: 0,
            duplicate_walk_step_count: 0,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopGraphWalkStatus,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        supplied_walk_step_count: usize,
        missing_walk_step_count: usize,
        extra_walk_step_count: usize,
        out_of_range_walk_step_count: usize,
        duplicate_walk_step_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            traversal_status: traversal.status,
            operation: traversal.operation,
            emitted_step_count: plan.emitted_steps.len(),
            supplied_walk_step_count,
            walk_indices: Vec::new(),
            ordered_steps: Vec::new(),
            missing_walk_step_count,
            extra_walk_step_count,
            out_of_range_walk_step_count,
            duplicate_walk_step_count,
            blocker_count,
        }
    }

    /// Repackages the validated walk order as a loop-assembly plan.
    pub fn to_loop_assembly_plan(
        &self,
        source: &BezierBooleanLoopAssemblyPlanReport2,
    ) -> BezierBooleanLoopAssemblyPlanReport2 {
        if self.status != BezierBooleanLoopGraphWalkStatus::Ready {
            return BezierBooleanLoopAssemblyPlanReport2 {
                status: BezierBooleanLoopAssemblyPlanStatus::AssemblyBlocked,
                assembly_status: source.assembly_status,
                operation: self.operation,
                emitted_steps: Vec::new(),
                first_emitted_count: 0,
                second_emitted_count: 0,
                keep_source_count: 0,
                keep_reversed_count: 0,
                invalid_reference_count: source.invalid_reference_count,
                blocker_count: self.blocker_count.max(1),
            };
        }

        BezierBooleanLoopAssemblyPlanReport2 {
            status: BezierBooleanLoopAssemblyPlanStatus::Ready,
            assembly_status: source.assembly_status,
            operation: self.operation,
            emitted_steps: self.ordered_steps.clone(),
            first_emitted_count: source.first_emitted_count,
            second_emitted_count: source.second_emitted_count,
            keep_source_count: source.keep_source_count,
            keep_reversed_count: source.keep_reversed_count,
            invalid_reference_count: source.invalid_reference_count,
            blocker_count: 0,
        }
    }

    /// Returns true when the supplied walk is a complete emitted-step permutation.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopGraphWalkStatus::Ready
    }

    /// Returns true when a certified graph walk is missing or malformed.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopGraphWalkStatus::TraversalBlocked
                | BezierBooleanLoopGraphWalkStatus::MissingWalkSteps
                | BezierBooleanLoopGraphWalkStatus::ExtraWalkSteps
                | BezierBooleanLoopGraphWalkStatus::OutOfRangeWalkStep
                | BezierBooleanLoopGraphWalkStatus::DuplicateWalkStep
        )
    }
}

impl BezierBooleanLoopGraphSuccessorWalkReport2 {
    /// Derives a certified walk order from exact successor facts.
    ///
    /// A ready report requires exactly one successor and one predecessor for
    /// every emitted fragment and one closed cycle reachable from the canonical
    /// start index `0`. The method does not repair disconnected cycles by
    /// sorting or splicing them. A disconnected or open relation is retained as
    /// an explicit blocker because, under Yap's exact-computation model
    /// (1997), loop topology cannot be inferred from an incomplete graph.
    pub fn from_successor_facts(
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
    ) -> Self {
        match traversal.status {
            BezierBooleanLoopGraphTraversalStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopGraphSuccessorWalkStatus::Empty,
                    traversal,
                    plan,
                    successor_facts.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                );
            }
            BezierBooleanLoopGraphTraversalStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopGraphSuccessorWalkStatus::NoInteriorSplits,
                    traversal,
                    plan,
                    successor_facts.len(),
                    0,
                    successor_facts.len(),
                    0,
                    0,
                    0,
                    0,
                    successor_facts.len(),
                );
            }
            BezierBooleanLoopGraphTraversalStatus::PlanBlocked
            | BezierBooleanLoopGraphTraversalStatus::BranchPointsNeedTraversal
            | BezierBooleanLoopGraphTraversalStatus::ResolvedOverlapsNeedTraversal => {
                return Self::empty_like(
                    BezierBooleanLoopGraphSuccessorWalkStatus::TraversalBlocked,
                    traversal,
                    plan,
                    successor_facts.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    traversal.blocker_count.max(1),
                );
            }
            BezierBooleanLoopGraphTraversalStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopGraphSuccessorWalkStatus::NoEmittedFragments,
                    traversal,
                    plan,
                    successor_facts.len(),
                    0,
                    successor_facts.len(),
                    0,
                    0,
                    0,
                    0,
                    successor_facts.len(),
                );
            }
            BezierBooleanLoopGraphTraversalStatus::Ready => {}
        }

        let emitted_step_count = plan.emitted_steps.len();
        if successor_facts.len() < emitted_step_count {
            let missing = emitted_step_count - successor_facts.len();
            return Self::empty_like(
                BezierBooleanLoopGraphSuccessorWalkStatus::MissingSuccessorFacts,
                traversal,
                plan,
                successor_facts.len(),
                missing,
                0,
                0,
                0,
                0,
                0,
                missing.max(1),
            );
        }
        if successor_facts.len() > emitted_step_count {
            let extra = successor_facts.len() - emitted_step_count;
            return Self::empty_like(
                BezierBooleanLoopGraphSuccessorWalkStatus::ExtraSuccessorFacts,
                traversal,
                plan,
                successor_facts.len(),
                0,
                extra,
                0,
                0,
                0,
                0,
                extra.max(1),
            );
        }

        let out_of_range_successor_count: usize = successor_facts
            .iter()
            .map(|fact| {
                usize::from(fact.from_step_index >= emitted_step_count)
                    + usize::from(fact.to_step_index >= emitted_step_count)
            })
            .sum();
        if out_of_range_successor_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphSuccessorWalkStatus::OutOfRangeSuccessorStep,
                traversal,
                plan,
                successor_facts.len(),
                0,
                0,
                out_of_range_successor_count,
                0,
                0,
                0,
                out_of_range_successor_count,
            );
        }

        let mut successor_by_source = vec![None; emitted_step_count];
        let mut predecessor_seen = vec![false; emitted_step_count];
        let mut duplicate_source_count = 0;
        let mut duplicate_target_count = 0;
        for fact in successor_facts {
            if successor_by_source[fact.from_step_index].is_some() {
                duplicate_source_count += 1;
            }
            successor_by_source[fact.from_step_index] = Some(fact.to_step_index);
            if predecessor_seen[fact.to_step_index] {
                duplicate_target_count += 1;
            }
            predecessor_seen[fact.to_step_index] = true;
        }
        if duplicate_source_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphSuccessorWalkStatus::DuplicateSuccessorSource,
                traversal,
                plan,
                successor_facts.len(),
                0,
                0,
                0,
                duplicate_source_count,
                0,
                0,
                duplicate_source_count,
            );
        }
        if duplicate_target_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphSuccessorWalkStatus::DuplicateSuccessorTarget,
                traversal,
                plan,
                successor_facts.len(),
                0,
                0,
                0,
                0,
                duplicate_target_count,
                0,
                duplicate_target_count,
            );
        }

        let start_step_index = 0;
        let mut walk_indices = Vec::with_capacity(emitted_step_count);
        let mut visited = vec![false; emitted_step_count];
        let mut current = start_step_index;
        for _ in 0..emitted_step_count {
            if visited[current] {
                return Self::empty_like(
                    BezierBooleanLoopGraphSuccessorWalkStatus::OpenOrDisconnectedSuccessorCycle,
                    traversal,
                    plan,
                    successor_facts.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    1,
                    1,
                );
            }
            visited[current] = true;
            walk_indices.push(current);
            match successor_by_source[current] {
                Some(next) => current = next,
                None => {
                    return Self::empty_like(
                        BezierBooleanLoopGraphSuccessorWalkStatus::OpenOrDisconnectedSuccessorCycle,
                        traversal,
                        plan,
                        successor_facts.len(),
                        0,
                        0,
                        0,
                        0,
                        0,
                        1,
                        1,
                    );
                }
            }
        }

        if current != start_step_index || visited.iter().any(|seen| !*seen) {
            return Self::empty_like(
                BezierBooleanLoopGraphSuccessorWalkStatus::OpenOrDisconnectedSuccessorCycle,
                traversal,
                plan,
                successor_facts.len(),
                0,
                0,
                0,
                0,
                0,
                1,
                1,
            );
        }

        Self {
            status: BezierBooleanLoopGraphSuccessorWalkStatus::Ready,
            traversal_status: traversal.status,
            operation: traversal.operation,
            emitted_step_count,
            supplied_successor_count: successor_facts.len(),
            start_step_index: Some(start_step_index),
            ordered_steps: walk_indices
                .iter()
                .map(|index| plan.emitted_steps[*index].clone())
                .collect(),
            walk_indices,
            missing_successor_count: 0,
            extra_successor_count: 0,
            out_of_range_successor_count: 0,
            duplicate_source_count: 0,
            duplicate_target_count: 0,
            cycle_blocker_count: 0,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopGraphSuccessorWalkStatus,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        supplied_successor_count: usize,
        missing_successor_count: usize,
        extra_successor_count: usize,
        out_of_range_successor_count: usize,
        duplicate_source_count: usize,
        duplicate_target_count: usize,
        cycle_blocker_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            traversal_status: traversal.status,
            operation: traversal.operation,
            emitted_step_count: plan.emitted_steps.len(),
            supplied_successor_count,
            start_step_index: None,
            walk_indices: Vec::new(),
            ordered_steps: Vec::new(),
            missing_successor_count,
            extra_successor_count,
            out_of_range_successor_count,
            duplicate_source_count,
            duplicate_target_count,
            cycle_blocker_count,
            blocker_count,
        }
    }

    /// Converts a ready successor-walk report into the existing walk report.
    pub fn to_graph_walk_report(
        &self,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
    ) -> BezierBooleanLoopGraphWalkReport2 {
        if self.status == BezierBooleanLoopGraphSuccessorWalkStatus::Ready {
            BezierBooleanLoopGraphWalkReport2::from_traversal_order(
                traversal,
                plan,
                &self.walk_indices,
            )
        } else {
            BezierBooleanLoopGraphWalkReport2 {
                status: BezierBooleanLoopGraphWalkStatus::TraversalBlocked,
                traversal_status: traversal.status,
                operation: self.operation,
                emitted_step_count: plan.emitted_steps.len(),
                supplied_walk_step_count: 0,
                walk_indices: Vec::new(),
                ordered_steps: Vec::new(),
                missing_walk_step_count: 0,
                extra_walk_step_count: 0,
                out_of_range_walk_step_count: 0,
                duplicate_walk_step_count: 0,
                blocker_count: self.blocker_count.max(1),
            }
        }
    }

    /// Returns true when successor facts define one closed emitted-step cycle.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopGraphSuccessorWalkStatus::Ready
    }

    /// Returns true when successor evidence is missing, stale, or disconnected.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopGraphSuccessorWalkStatus::TraversalBlocked
                | BezierBooleanLoopGraphSuccessorWalkStatus::MissingSuccessorFacts
                | BezierBooleanLoopGraphSuccessorWalkStatus::ExtraSuccessorFacts
                | BezierBooleanLoopGraphSuccessorWalkStatus::OutOfRangeSuccessorStep
                | BezierBooleanLoopGraphSuccessorWalkStatus::DuplicateSuccessorSource
                | BezierBooleanLoopGraphSuccessorWalkStatus::DuplicateSuccessorTarget
                | BezierBooleanLoopGraphSuccessorWalkStatus::OpenOrDisconnectedSuccessorCycle
        )
    }
}

impl BezierBooleanLoopGraphMultiCycleWalkReport2 {
    /// Derives deterministic closed cycles from exact successor facts.
    ///
    /// This accepts disjoint closed cycles, unlike
    /// [`BezierBooleanLoopGraphSuccessorWalkReport2::from_successor_facts`],
    /// but it does not relax the exact graph certificate: every emitted step
    /// must have exactly one successor and one predecessor. Cycles are emitted
    /// in smallest-unvisited-start order so the report is deterministic and
    /// replayable. Yap (1997) is the governing exactness rule; Vatti (1992),
    /// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009) provide the
    /// staged traversal/fill model.
    pub fn from_successor_facts(
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
    ) -> Self {
        match traversal.status {
            BezierBooleanLoopGraphTraversalStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopGraphMultiCycleWalkStatus::Empty,
                    traversal,
                    plan,
                    successor_facts.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                );
            }
            BezierBooleanLoopGraphTraversalStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopGraphMultiCycleWalkStatus::NoInteriorSplits,
                    traversal,
                    plan,
                    successor_facts.len(),
                    0,
                    successor_facts.len(),
                    0,
                    0,
                    0,
                    0,
                    successor_facts.len(),
                );
            }
            BezierBooleanLoopGraphTraversalStatus::PlanBlocked
            | BezierBooleanLoopGraphTraversalStatus::BranchPointsNeedTraversal
            | BezierBooleanLoopGraphTraversalStatus::ResolvedOverlapsNeedTraversal => {
                return Self::empty_like(
                    BezierBooleanLoopGraphMultiCycleWalkStatus::TraversalBlocked,
                    traversal,
                    plan,
                    successor_facts.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    traversal.blocker_count.max(1),
                );
            }
            BezierBooleanLoopGraphTraversalStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopGraphMultiCycleWalkStatus::NoEmittedFragments,
                    traversal,
                    plan,
                    successor_facts.len(),
                    0,
                    successor_facts.len(),
                    0,
                    0,
                    0,
                    0,
                    successor_facts.len(),
                );
            }
            BezierBooleanLoopGraphTraversalStatus::Ready => {}
        }

        let emitted_step_count = plan.emitted_steps.len();
        if emitted_step_count == 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphMultiCycleWalkStatus::NoEmittedFragments,
                traversal,
                plan,
                successor_facts.len(),
                0,
                successor_facts.len(),
                0,
                0,
                0,
                0,
                successor_facts.len(),
            );
        }
        if successor_facts.len() < emitted_step_count {
            let missing = emitted_step_count - successor_facts.len();
            return Self::empty_like(
                BezierBooleanLoopGraphMultiCycleWalkStatus::MissingSuccessorFacts,
                traversal,
                plan,
                successor_facts.len(),
                missing,
                0,
                0,
                0,
                0,
                0,
                missing.max(1),
            );
        }
        if successor_facts.len() > emitted_step_count {
            let extra = successor_facts.len() - emitted_step_count;
            return Self::empty_like(
                BezierBooleanLoopGraphMultiCycleWalkStatus::ExtraSuccessorFacts,
                traversal,
                plan,
                successor_facts.len(),
                0,
                extra,
                0,
                0,
                0,
                0,
                extra.max(1),
            );
        }

        let out_of_range_successor_count: usize = successor_facts
            .iter()
            .map(|fact| {
                usize::from(fact.from_step_index >= emitted_step_count)
                    + usize::from(fact.to_step_index >= emitted_step_count)
            })
            .sum();
        if out_of_range_successor_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphMultiCycleWalkStatus::OutOfRangeSuccessorStep,
                traversal,
                plan,
                successor_facts.len(),
                0,
                0,
                out_of_range_successor_count,
                0,
                0,
                0,
                out_of_range_successor_count,
            );
        }

        let mut successor_by_source = vec![None; emitted_step_count];
        let mut predecessor_seen = vec![false; emitted_step_count];
        let mut duplicate_source_count = 0;
        let mut duplicate_target_count = 0;
        for fact in successor_facts {
            if successor_by_source[fact.from_step_index].is_some() {
                duplicate_source_count += 1;
            }
            successor_by_source[fact.from_step_index] = Some(fact.to_step_index);
            if predecessor_seen[fact.to_step_index] {
                duplicate_target_count += 1;
            }
            predecessor_seen[fact.to_step_index] = true;
        }
        if duplicate_source_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphMultiCycleWalkStatus::DuplicateSuccessorSource,
                traversal,
                plan,
                successor_facts.len(),
                0,
                0,
                0,
                duplicate_source_count,
                0,
                0,
                duplicate_source_count,
            );
        }
        if duplicate_target_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopGraphMultiCycleWalkStatus::DuplicateSuccessorTarget,
                traversal,
                plan,
                successor_facts.len(),
                0,
                0,
                0,
                0,
                duplicate_target_count,
                0,
                duplicate_target_count,
            );
        }

        let mut visited = vec![false; emitted_step_count];
        let mut cycle_start_indices = Vec::new();
        let mut cycle_walk_start_offsets = Vec::new();
        let mut cycle_step_counts = Vec::new();
        let mut walk_indices = Vec::with_capacity(emitted_step_count);
        for start_step_index in 0..emitted_step_count {
            if visited[start_step_index] {
                continue;
            }
            cycle_start_indices.push(start_step_index);
            let cycle_walk_start_offset = walk_indices.len();
            cycle_walk_start_offsets.push(cycle_walk_start_offset);
            let mut current = start_step_index;
            loop {
                if visited[current] {
                    if current == start_step_index {
                        cycle_step_counts.push(walk_indices.len() - cycle_walk_start_offset);
                        break;
                    }
                    return Self::empty_like(
                        BezierBooleanLoopGraphMultiCycleWalkStatus::OpenSuccessorCycle,
                        traversal,
                        plan,
                        successor_facts.len(),
                        0,
                        0,
                        0,
                        0,
                        0,
                        1,
                        1,
                    );
                }
                visited[current] = true;
                walk_indices.push(current);
                let Some(next) = successor_by_source[current] else {
                    return Self::empty_like(
                        BezierBooleanLoopGraphMultiCycleWalkStatus::OpenSuccessorCycle,
                        traversal,
                        plan,
                        successor_facts.len(),
                        0,
                        0,
                        0,
                        0,
                        0,
                        1,
                        1,
                    );
                };
                current = next;
            }
        }

        let cycle_count = cycle_start_indices.len();
        Self {
            status: BezierBooleanLoopGraphMultiCycleWalkStatus::Ready,
            traversal_status: traversal.status,
            operation: traversal.operation,
            emitted_step_count,
            supplied_successor_count: successor_facts.len(),
            cycle_start_indices,
            cycle_walk_start_offsets,
            cycle_step_counts,
            cycle_count,
            ordered_steps: walk_indices
                .iter()
                .map(|index| plan.emitted_steps[*index].clone())
                .collect(),
            walk_indices,
            missing_successor_count: 0,
            extra_successor_count: 0,
            out_of_range_successor_count: 0,
            duplicate_source_count: 0,
            duplicate_target_count: 0,
            cycle_blocker_count: 0,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopGraphMultiCycleWalkStatus,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        supplied_successor_count: usize,
        missing_successor_count: usize,
        extra_successor_count: usize,
        out_of_range_successor_count: usize,
        duplicate_source_count: usize,
        duplicate_target_count: usize,
        cycle_blocker_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            traversal_status: traversal.status,
            operation: traversal.operation,
            emitted_step_count: plan.emitted_steps.len(),
            supplied_successor_count,
            cycle_start_indices: Vec::new(),
            cycle_walk_start_offsets: Vec::new(),
            cycle_step_counts: Vec::new(),
            cycle_count: 0,
            walk_indices: Vec::new(),
            ordered_steps: Vec::new(),
            missing_successor_count,
            extra_successor_count,
            out_of_range_successor_count,
            duplicate_source_count,
            duplicate_target_count,
            cycle_blocker_count,
            blocker_count,
        }
    }

    /// Converts a ready multi-cycle report into the existing walk report.
    pub fn to_graph_walk_report(
        &self,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
    ) -> BezierBooleanLoopGraphWalkReport2 {
        if self.status == BezierBooleanLoopGraphMultiCycleWalkStatus::Ready {
            BezierBooleanLoopGraphWalkReport2::from_traversal_order(
                traversal,
                plan,
                &self.walk_indices,
            )
        } else {
            BezierBooleanLoopGraphWalkReport2 {
                status: BezierBooleanLoopGraphWalkStatus::TraversalBlocked,
                traversal_status: traversal.status,
                operation: self.operation,
                emitted_step_count: plan.emitted_steps.len(),
                supplied_walk_step_count: 0,
                walk_indices: Vec::new(),
                ordered_steps: Vec::new(),
                missing_walk_step_count: 0,
                extra_walk_step_count: 0,
                out_of_range_walk_step_count: 0,
                duplicate_walk_step_count: 0,
                blocker_count: self.blocker_count.max(1),
            }
        }
    }

    /// Returns true when successor facts define closed cycles over all emitted steps.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopGraphMultiCycleWalkStatus::Ready
    }

    /// Returns true when successor evidence is missing, stale, or open.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopGraphMultiCycleWalkStatus::TraversalBlocked
                | BezierBooleanLoopGraphMultiCycleWalkStatus::MissingSuccessorFacts
                | BezierBooleanLoopGraphMultiCycleWalkStatus::ExtraSuccessorFacts
                | BezierBooleanLoopGraphMultiCycleWalkStatus::OutOfRangeSuccessorStep
                | BezierBooleanLoopGraphMultiCycleWalkStatus::DuplicateSuccessorSource
                | BezierBooleanLoopGraphMultiCycleWalkStatus::DuplicateSuccessorTarget
                | BezierBooleanLoopGraphMultiCycleWalkStatus::OpenSuccessorCycle
        )
    }
}

impl BezierBooleanLoopLocatorInputReport2 {
    /// Generates exact representative points from packaged output loops.
    ///
    /// The generated inputs are only query handles for a later exact locator.
    /// This method validates loop ranges against the retained directed-fragment
    /// array and never derives containment from the representative points. That
    /// separation is the Yap (1997) contract: a construction object can be
    /// prepared here, but the predicate that classifies it must return its own
    /// certificate or explicit blocker.
    pub fn from_output_loops(output: &BezierBooleanOutputLoopReport2) -> Self {
        match output.status {
            BezierBooleanOutputLoopStatus::Empty => {
                return Self::empty_like(BezierBooleanLoopLocatorInputStatus::Empty, output, 0, 0);
            }
            BezierBooleanOutputLoopStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopLocatorInputStatus::NoInteriorSplits,
                    output,
                    0,
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::ClosureBlocked
            | BezierBooleanOutputLoopStatus::MalformedClosedLoops => {
                return Self::empty_like(
                    BezierBooleanLoopLocatorInputStatus::OutputLoopBlocked,
                    output,
                    0,
                    output.blocker_count.max(1),
                );
            }
            BezierBooleanOutputLoopStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopLocatorInputStatus::NoEmittedFragments,
                    output,
                    0,
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::Ready => {}
        }

        let mut malformed_loop_range_count = 0;
        let mut inputs = Vec::with_capacity(output.loops.len());
        for (loop_index, output_loop) in output.loops.iter().enumerate() {
            let Some(end) = output_loop
                .first_directed_fragment_index
                .checked_add(output_loop.directed_fragment_count)
            else {
                malformed_loop_range_count += 1;
                continue;
            };
            if output_loop.directed_fragment_count == 0 || end > output.directed_fragments.len() {
                malformed_loop_range_count += 1;
                continue;
            }
            inputs.push(BezierBooleanLoopLocatorInput2 {
                loop_index,
                directed_fragment_index: output_loop.first_directed_fragment_index,
                representative_point: output_loop.anchor.clone(),
            });
        }

        if malformed_loop_range_count > 0 {
            return Self {
                status: BezierBooleanLoopLocatorInputStatus::MalformedLoopRange,
                output_status: output.status,
                operation: output.operation,
                inputs: Vec::new(),
                output_loop_count: output.loops.len(),
                input_count: 0,
                malformed_loop_range_count,
                blocker_count: malformed_loop_range_count,
            };
        }

        Self {
            status: BezierBooleanLoopLocatorInputStatus::Ready,
            output_status: output.status,
            operation: output.operation,
            input_count: inputs.len(),
            output_loop_count: output.loops.len(),
            inputs,
            malformed_loop_range_count: 0,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopLocatorInputStatus,
        output: &BezierBooleanOutputLoopReport2,
        malformed_loop_range_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            output_status: output.status,
            operation: output.operation,
            inputs: Vec::new(),
            output_loop_count: output.loops.len(),
            input_count: 0,
            malformed_loop_range_count,
            blocker_count,
        }
    }

    /// Returns true when every output loop has a keyed representative point.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopLocatorInputStatus::Ready
    }

    /// Returns true when output-loop packaging or range validation blocked the handoff.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopLocatorInputStatus::OutputLoopBlocked
                | BezierBooleanLoopLocatorInputStatus::MalformedLoopRange
        )
    }
}

impl BezierBooleanLoopContainmentQueryReport2 {
    /// Builds ordered representative-point containment queries.
    ///
    /// A ready report has `n * (n - 1)` queries for `n` output loops. Loops
    /// are never queried against themselves, and no containment fact is
    /// synthesized here. This keeps the future point/loop locator as the only
    /// stage allowed to decide containment, consistent with Yap's
    /// predicate-before-construction discipline.
    pub fn from_locator_inputs(locator: &BezierBooleanLoopLocatorInputReport2) -> Self {
        match locator.status {
            BezierBooleanLoopLocatorInputStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentQueryStatus::Empty,
                    locator,
                    0,
                );
            }
            BezierBooleanLoopLocatorInputStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentQueryStatus::NoInteriorSplits,
                    locator,
                    0,
                );
            }
            BezierBooleanLoopLocatorInputStatus::OutputLoopBlocked
            | BezierBooleanLoopLocatorInputStatus::MalformedLoopRange => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentQueryStatus::LocatorInputBlocked,
                    locator,
                    locator.blocker_count.max(1),
                );
            }
            BezierBooleanLoopLocatorInputStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentQueryStatus::NoEmittedFragments,
                    locator,
                    0,
                );
            }
            BezierBooleanLoopLocatorInputStatus::Ready => {}
        }

        if locator.inputs.len() < 2 {
            return Self::empty_like(
                BezierBooleanLoopContainmentQueryStatus::NotEnoughLoops,
                locator,
                0,
            );
        }

        let mut queries = Vec::with_capacity(locator.inputs.len() * (locator.inputs.len() - 1));
        for query in &locator.inputs {
            for candidate in &locator.inputs {
                if query.loop_index == candidate.loop_index {
                    continue;
                }
                queries.push(BezierBooleanLoopContainmentQuery2 {
                    query_loop_index: query.loop_index,
                    candidate_container_loop_index: candidate.loop_index,
                    representative_point: query.representative_point.clone(),
                });
            }
        }

        Self {
            status: BezierBooleanLoopContainmentQueryStatus::Ready,
            locator_status: locator.status,
            operation: locator.operation,
            query_count: queries.len(),
            loop_count: locator.inputs.len(),
            queries,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopContainmentQueryStatus,
        locator: &BezierBooleanLoopLocatorInputReport2,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            locator_status: locator.status,
            operation: locator.operation,
            queries: Vec::new(),
            loop_count: locator.inputs.len(),
            query_count: 0,
            blocker_count,
        }
    }

    /// Returns true when pairwise containment locator work was generated.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopContainmentQueryStatus::Ready
    }

    /// Returns true when no pairwise query is necessary for nesting.
    pub fn is_vacuously_complete(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopContainmentQueryStatus::NoInteriorSplits
                | BezierBooleanLoopContainmentQueryStatus::NoEmittedFragments
                | BezierBooleanLoopContainmentQueryStatus::NotEnoughLoops
        )
    }

    /// Returns true when locator-input generation blocked query construction.
    pub fn has_blockers(&self) -> bool {
        self.status == BezierBooleanLoopContainmentQueryStatus::LocatorInputBlocked
    }
}

impl BezierBooleanLoopContainmentQueryResultReport2 {
    /// Replays exact point/loop locator results into keyed containment facts.
    ///
    /// Each supplied result must match the query at the same index. The method
    /// intentionally does not sort or repair stale results because that would
    /// hide object-identity mistakes at the predicate/construction boundary.
    /// Only strict `Contains` decisions become containment facts; `Outside`
    /// decisions are certified misses, and boundary/unknown decisions remain
    /// blockers for later overlap or stronger locator policy.
    pub fn from_query_results(
        queries: &BezierBooleanLoopContainmentQueryReport2,
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        match queries.status {
            BezierBooleanLoopContainmentQueryStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentQueryResultStatus::Empty,
                    queries,
                    results.len(),
                    0,
                );
            }
            BezierBooleanLoopContainmentQueryStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentQueryResultStatus::NoInteriorSplits,
                    queries,
                    results.len(),
                    0,
                );
            }
            BezierBooleanLoopContainmentQueryStatus::LocatorInputBlocked => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentQueryResultStatus::QueryBlocked,
                    queries,
                    results.len(),
                    queries.blocker_count.max(1),
                );
            }
            BezierBooleanLoopContainmentQueryStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentQueryResultStatus::NoEmittedFragments,
                    queries,
                    results.len(),
                    0,
                );
            }
            BezierBooleanLoopContainmentQueryStatus::NotEnoughLoops => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentQueryResultStatus::NotEnoughLoops,
                    queries,
                    results.len(),
                    0,
                );
            }
            BezierBooleanLoopContainmentQueryStatus::Ready => {}
        }

        if results.len() < queries.queries.len() {
            let missing = queries.queries.len() - results.len();
            return Self::blocked_like(
                BezierBooleanLoopContainmentQueryResultStatus::MissingQueryResults,
                queries,
                results.len(),
                missing,
                0,
                0,
                0,
                0,
                0,
                missing.max(1),
            );
        }
        if results.len() > queries.queries.len() {
            let extra = results.len() - queries.queries.len();
            return Self::blocked_like(
                BezierBooleanLoopContainmentQueryResultStatus::ExtraQueryResults,
                queries,
                results.len(),
                0,
                extra,
                0,
                0,
                0,
                0,
                extra.max(1),
            );
        }

        let mut key_mismatch_count = 0;
        let mut contains_count = 0;
        let mut outside_count = 0;
        let mut boundary_count = 0;
        let mut unknown_count = 0;
        let mut containment_facts = Vec::new();
        for (query, result) in queries.queries.iter().zip(results.iter()) {
            if query.query_loop_index != result.query_loop_index
                || query.candidate_container_loop_index != result.candidate_container_loop_index
            {
                key_mismatch_count += 1;
                continue;
            }
            match result.result {
                BezierBooleanLoopContainmentQueryResult::Contains => {
                    contains_count += 1;
                    containment_facts.push(BezierBooleanLoopContainmentFact2 {
                        container_loop_index: query.candidate_container_loop_index,
                        contained_loop_index: query.query_loop_index,
                    });
                }
                BezierBooleanLoopContainmentQueryResult::Outside => outside_count += 1,
                BezierBooleanLoopContainmentQueryResult::Boundary => boundary_count += 1,
                BezierBooleanLoopContainmentQueryResult::Unknown => unknown_count += 1,
            }
        }

        if key_mismatch_count > 0 {
            return Self::blocked_like(
                BezierBooleanLoopContainmentQueryResultStatus::QueryKeyMismatch,
                queries,
                results.len(),
                0,
                0,
                key_mismatch_count,
                boundary_count,
                unknown_count,
                0,
                key_mismatch_count,
            );
        }
        if boundary_count > 0 {
            return Self::blocked_like(
                BezierBooleanLoopContainmentQueryResultStatus::BoundaryNeedsResolution,
                queries,
                results.len(),
                0,
                0,
                0,
                boundary_count,
                unknown_count,
                0,
                boundary_count,
            );
        }
        if unknown_count > 0 {
            return Self::blocked_like(
                BezierBooleanLoopContainmentQueryResultStatus::UnknownNeedsResolution,
                queries,
                results.len(),
                0,
                0,
                0,
                0,
                unknown_count,
                0,
                unknown_count,
            );
        }

        Self {
            status: BezierBooleanLoopContainmentQueryResultStatus::Ready,
            query_status: queries.status,
            operation: queries.operation,
            containment_fact_count: containment_facts.len(),
            containment_facts,
            query_count: queries.queries.len(),
            supplied_result_count: results.len(),
            missing_result_count: 0,
            extra_result_count: 0,
            key_mismatch_count: 0,
            contains_count,
            outside_count,
            boundary_count: 0,
            unknown_count: 0,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopContainmentQueryResultStatus,
        queries: &BezierBooleanLoopContainmentQueryReport2,
        supplied_result_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self::blocked_like(
            status,
            queries,
            supplied_result_count,
            0,
            0,
            0,
            0,
            0,
            0,
            blocker_count,
        )
    }

    fn blocked_like(
        status: BezierBooleanLoopContainmentQueryResultStatus,
        queries: &BezierBooleanLoopContainmentQueryReport2,
        supplied_result_count: usize,
        missing_result_count: usize,
        extra_result_count: usize,
        key_mismatch_count: usize,
        boundary_count: usize,
        unknown_count: usize,
        contains_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            query_status: queries.status,
            operation: queries.operation,
            containment_facts: Vec::new(),
            query_count: queries.queries.len(),
            supplied_result_count,
            containment_fact_count: 0,
            missing_result_count,
            extra_result_count,
            key_mismatch_count,
            contains_count,
            outside_count: 0,
            boundary_count,
            unknown_count,
            blocker_count,
        }
    }

    /// Returns true when all locator results were replayed without blockers.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopContainmentQueryResultStatus::Ready
    }

    /// Returns true when no containment facts are needed for nesting.
    pub fn is_vacuously_complete(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopContainmentQueryResultStatus::NoInteriorSplits
                | BezierBooleanLoopContainmentQueryResultStatus::NoEmittedFragments
                | BezierBooleanLoopContainmentQueryResultStatus::NotEnoughLoops
        )
    }

    /// Returns true when stale or uncertified locator results block containment facts.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopContainmentQueryResultStatus::QueryBlocked
                | BezierBooleanLoopContainmentQueryResultStatus::MissingQueryResults
                | BezierBooleanLoopContainmentQueryResultStatus::ExtraQueryResults
                | BezierBooleanLoopContainmentQueryResultStatus::QueryKeyMismatch
                | BezierBooleanLoopContainmentQueryResultStatus::BoundaryNeedsResolution
                | BezierBooleanLoopContainmentQueryResultStatus::UnknownNeedsResolution
        )
    }
}

impl BezierBooleanLoopContainmentCertificationReport2 {
    /// Certifies output-loop containment from exact locator answers.
    ///
    /// `results` must answer the ordered queries derived from `output` by this
    /// constructor. The method intentionally does not accept a caller-supplied
    /// query report: the query worklist is part of the output-loop object
    /// identity, so it is rebuilt here before replay. Boundary and unknown
    /// locator answers, stale result keys, malformed output-loop ranges, and
    /// non-laminar containment facts all remain explicit blockers. This is the
    /// Yap (1997) exact-computation model applied to loop nesting: every
    /// topological fact is replayed against the object it names before it can
    /// feed construction.
    pub fn from_output_loop_query_results(
        output: &BezierBooleanOutputLoopReport2,
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        let locator = BezierBooleanLoopLocatorInputReport2::from_output_loops(output);
        let queries = BezierBooleanLoopContainmentQueryReport2::from_locator_inputs(&locator);
        let replay =
            BezierBooleanLoopContainmentQueryResultReport2::from_query_results(&queries, results);

        if locator.has_blockers() {
            return Self::blocked_like(
                BezierBooleanLoopContainmentCertificationStatus::LocatorInputBlocked,
                output,
                &locator,
                &queries,
                &replay,
                BezierBooleanLoopContainmentFactStatus::OutputLoopBlocked,
                locator.blocker_count.max(1),
            );
        }

        if replay.has_blockers() {
            return Self::blocked_like(
                BezierBooleanLoopContainmentCertificationStatus::QueryResultBlocked,
                output,
                &locator,
                &queries,
                &replay,
                BezierBooleanLoopContainmentFactStatus::OutputLoopBlocked,
                replay.blocker_count.max(1),
            );
        }

        let facts = BezierBooleanLoopContainmentFactReport2::from_output_loop_containment_facts(
            output,
            &replay.containment_facts,
        );
        let status = match facts.status {
            BezierBooleanLoopContainmentFactStatus::Ready => {
                BezierBooleanLoopContainmentCertificationStatus::Ready
            }
            BezierBooleanLoopContainmentFactStatus::Empty => {
                BezierBooleanLoopContainmentCertificationStatus::Empty
            }
            BezierBooleanLoopContainmentFactStatus::NoInteriorSplits => {
                BezierBooleanLoopContainmentCertificationStatus::NoInteriorSplits
            }
            BezierBooleanLoopContainmentFactStatus::NoEmittedFragments => {
                BezierBooleanLoopContainmentCertificationStatus::NoEmittedFragments
            }
            BezierBooleanLoopContainmentFactStatus::OutputLoopBlocked => {
                BezierBooleanLoopContainmentCertificationStatus::LocatorInputBlocked
            }
            BezierBooleanLoopContainmentFactStatus::OutOfRangeLoopIndex
            | BezierBooleanLoopContainmentFactStatus::SelfContainment
            | BezierBooleanLoopContainmentFactStatus::DuplicateContainmentFact
            | BezierBooleanLoopContainmentFactStatus::CyclicContainmentFacts
            | BezierBooleanLoopContainmentFactStatus::NonLaminarContainmentFacts => {
                BezierBooleanLoopContainmentCertificationStatus::ContainmentFactBlocked
            }
        };
        let blocker_count = if status == BezierBooleanLoopContainmentCertificationStatus::Ready
            || matches!(
                status,
                BezierBooleanLoopContainmentCertificationStatus::Empty
                    | BezierBooleanLoopContainmentCertificationStatus::NoInteriorSplits
                    | BezierBooleanLoopContainmentCertificationStatus::NoEmittedFragments
            ) {
            0
        } else {
            facts.blocker_count.max(1)
        };

        let depth_fact_count = facts.depth_facts.len();

        Self {
            status,
            locator_status: locator.status,
            query_status: queries.status,
            query_result_status: replay.status,
            fact_status: facts.status,
            operation: output.operation,
            containment_facts: facts.facts,
            depth_facts: facts.depth_facts,
            output_loop_count: output.loops.len(),
            locator_input_count: locator.input_count,
            query_count: queries.query_count,
            supplied_result_count: results.len(),
            containment_fact_count: replay.containment_fact_count,
            depth_fact_count,
            blocker_count,
        }
    }

    /// Certifies scheduled graph-walk output loops from exact locator answers.
    ///
    /// This is the atomic scheduled counterpart to
    /// [`Self::from_output_loop_query_results`]. It derives ownership,
    /// emission, loop assembly, certified graph traversal, graph-walk order,
    /// exact closure, output loops, locator inputs, ordered containment
    /// queries, strict locator replay, and laminar containment depths in one
    /// report-bearing chain. A caller supplies only the same exact locator
    /// answers it would have returned for the derived query worklist. Missing,
    /// extra, stale, boundary, and unknown answers stay blockers; malformed
    /// graph walks and output loops also stay blockers. This follows Yap's
    /// "Towards Exact Geometric Computation" (1997): predicate results are
    /// replayed against the constructed object they classify before they can
    /// affect topology. The staged boundary/nesting split follows Vatti
    /// (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_graph_walk_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        let output = BezierBooleanOutputLoopReport2::from_graph_walk_endpoints(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_query_results(&output, results)
    }

    /// Certifies multi-cycle-successor output loops from exact locator answers.
    ///
    /// This is the successor-certificate counterpart to
    /// [`Self::from_schedule_graph_walk_query_results`] without the scheduled
    /// ownership prefix. The multi-cycle report supplies exact keyed successor
    /// evidence, endpoint closure packages the certified cycles as output
    /// loops, and this method then rebuilds locator inputs and replay queries
    /// before accepting any containment fact. Boundary, unknown, stale-key,
    /// malformed-cycle, and malformed-output evidence therefore remain
    /// report-bearing blockers. This follows Yap's "Towards Exact Geometric
    /// Computation" (1997): exact predicates are replayed against the object
    /// identity they classify before they feed construction.
    pub fn from_multi_cycle_successor_query_results(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_multi_cycle_successor_endpoints(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_query_results(&output, results)
    }

    /// Certifies scheduled multi-cycle-successor output loops from locator answers.
    ///
    /// This is the atomic scheduled successor route from exact point/loop
    /// locator answers to containment-depth facts. It composes keyed ownership,
    /// emission, assembly readiness, certified graph obligations, exact
    /// successor edges, cycle-preserving output-loop closure, locator input
    /// generation, query replay, and laminar containment validation. The
    /// staged construction mirrors the boundary/nesting split in Vatti (1992),
    /// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009), while keeping
    /// Yap's (1997) predicate/construction boundary explicit.
    pub fn from_schedule_multi_cycle_successor_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let multi_cycle = BezierBooleanLoopGraphMultiCycleWalkReport2::from_successor_facts(
            &traversal,
            &plan,
            successor_facts,
        );
        Self::from_multi_cycle_successor_query_results(
            &multi_cycle,
            &traversal,
            &plan,
            first_endpoints,
            second_endpoints,
            results,
        )
    }

    /// Certifies quadratic Bezier multi-cycle successor output loops from locator answers.
    ///
    /// This typed wrapper mirrors
    /// [`Self::from_schedule_multi_cycle_successor_query_results`] while
    /// deriving endpoint carriers from exact quadratic split fragments. It
    /// keeps the Vatti/Greiner-Hormann/Martinez-Rueda-Feito staged boolean
    /// handoff intact and preserves Yap's (1997) requirement that locator
    /// answers be replayed against the exact output loops they classify.
    pub fn from_quadratic_schedule_multi_cycle_successor_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_query_results(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            results,
        )
    }

    /// Certifies cubic Bezier multi-cycle successor output loops from locator answers.
    pub fn from_cubic_schedule_multi_cycle_successor_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_query_results(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            results,
        )
    }

    /// Certifies rational quadratic/conic multi-cycle successor output loops from locator answers.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_query_results(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            results,
        )
    }

    fn blocked_like(
        status: BezierBooleanLoopContainmentCertificationStatus,
        output: &BezierBooleanOutputLoopReport2,
        locator: &BezierBooleanLoopLocatorInputReport2,
        queries: &BezierBooleanLoopContainmentQueryReport2,
        replay: &BezierBooleanLoopContainmentQueryResultReport2,
        fact_status: BezierBooleanLoopContainmentFactStatus,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            locator_status: locator.status,
            query_status: queries.status,
            query_result_status: replay.status,
            fact_status,
            operation: output.operation,
            containment_facts: Vec::new(),
            depth_facts: Vec::new(),
            output_loop_count: output.loops.len(),
            locator_input_count: locator.input_count,
            query_count: queries.query_count,
            supplied_result_count: replay.supplied_result_count,
            containment_fact_count: 0,
            depth_fact_count: 0,
            blocker_count,
        }
    }

    /// Returns true when containment and nesting depths are certified.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopContainmentCertificationStatus::Ready
    }

    /// Returns true when no materializing containment work is present.
    pub fn is_vacuously_complete(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopContainmentCertificationStatus::Empty
                | BezierBooleanLoopContainmentCertificationStatus::NoInteriorSplits
                | BezierBooleanLoopContainmentCertificationStatus::NoEmittedFragments
        )
    }

    /// Returns true when locator replay or containment validation blocked certification.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopContainmentCertificationStatus::LocatorInputBlocked
                | BezierBooleanLoopContainmentCertificationStatus::QueryResultBlocked
                | BezierBooleanLoopContainmentCertificationStatus::ContainmentFactBlocked
        )
    }
}

impl BezierBooleanLoopClosureReport2 {
    /// Audits closure after a certified graph walk over quadratic fragments.
    ///
    /// This constructor makes graph-walk validation part of the closure
    /// precondition instead of asking callers to manually repackage a walk
    /// order. A malformed [`BezierBooleanLoopGraphWalkReport2`] is preserved as
    /// a plan blocker; a ready walk is converted into the certified emitted
    /// order before exact endpoint closure is checked. This follows Yap's
    /// "Towards Exact Geometric Computation" (1997): a graph traversal result
    /// is consumed only as certified combinatorial data, never inferred from
    /// vector order or tolerance snapping.
    pub fn from_quadratic_graph_walk(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
    ) -> Self {
        Self::from_graph_walk_endpoints(
            walk,
            plan,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
        )
    }

    /// Audits closure after a certified graph walk over cubic fragments.
    pub fn from_cubic_graph_walk(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
    ) -> Self {
        Self::from_graph_walk_endpoints(
            walk,
            plan,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
        )
    }

    /// Audits closure after a certified graph walk over rational quadratic fragments.
    pub fn from_rational_quadratic_graph_walk(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
    ) -> Self {
        Self::from_graph_walk_endpoints(
            walk,
            plan,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
        )
    }

    /// Audits closure after applying a certified graph walk to generic endpoints.
    ///
    /// Ready graph walks are converted into an ordered loop-assembly plan before
    /// closure. Blocked graph walks produce a plan-blocked closure report with
    /// the graph-walk blocker count retained, making the dependency on the
    /// graph traversal certificate visible in downstream output-loop reports.
    pub fn from_graph_walk_endpoints(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
    ) -> Self {
        let ordered_plan = walk.to_loop_assembly_plan(plan);
        Self::from_fragment_endpoints(&ordered_plan, first_endpoints, second_endpoints)
    }

    /// Audits closure after applying a certified multi-cycle successor walk.
    ///
    /// This is the closure-stage counterpart to
    /// [`BezierBooleanLoopGraphMultiCycleWalkReport2`]. The successor report
    /// remains the only source of traversal topology: this method simply
    /// replays its exact emitted-step order through the graph-walk closure
    /// path, preserving blockers when the successor certificate is not ready.
    /// Yap, "Towards Exact Geometric Computation" (1997), is the contract: no
    /// endpoint tolerance or vector-order guess may substitute for the keyed
    /// successor certificate.
    pub fn from_multi_cycle_successor_endpoints(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
    ) -> Self {
        let walk = multi_cycle.to_graph_walk_report(traversal, plan);
        Self::from_graph_walk_endpoints(&walk, plan, first_endpoints, second_endpoints)
    }

    /// Audits loop closure against quadratic Bezier fragments.
    pub fn from_quadratic_fragments(
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
    ) -> Self {
        Self::from_fragment_endpoints(
            plan,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
        )
    }

    /// Audits loop closure against cubic Bezier fragments.
    pub fn from_cubic_fragments(
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
    ) -> Self {
        Self::from_fragment_endpoints(
            plan,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
        )
    }

    /// Audits loop closure against rational quadratic/conic fragments.
    pub fn from_rational_quadratic_fragments(
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
    ) -> Self {
        Self::from_fragment_endpoints(
            plan,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
        )
    }

    /// Audits loop closure from generic operand fragment endpoints.
    ///
    /// Each endpoint pair is `(fragment_start, fragment_end)` in source
    /// direction. Reversed boolean emissions swap those endpoints before
    /// closure is checked.
    pub fn from_fragment_endpoints(
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
    ) -> Self {
        match plan.status {
            BezierBooleanLoopAssemblyPlanStatus::Empty => {
                return Self::empty_like(BezierBooleanLoopClosureStatus::Empty, plan, 0);
            }
            BezierBooleanLoopAssemblyPlanStatus::NoInteriorSplits => {
                return Self::empty_like(BezierBooleanLoopClosureStatus::NoInteriorSplits, plan, 0);
            }
            BezierBooleanLoopAssemblyPlanStatus::AssemblyBlocked => {
                return Self::empty_like(
                    BezierBooleanLoopClosureStatus::PlanBlocked,
                    plan,
                    plan.blocker_count.max(1),
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopClosureStatus::NoEmittedFragments,
                    plan,
                    0,
                );
            }
            BezierBooleanLoopAssemblyPlanStatus::Ready => {}
        }

        let mut directed_fragments = Vec::with_capacity(plan.emitted_steps.len());
        let mut invalid_reference_count = 0;
        for emitted in &plan.emitted_steps {
            let endpoints = match emitted.step.operand {
                BezierBooleanTraversalOperand::First => {
                    first_endpoints.get(emitted.step.fragment_index)
                }
                BezierBooleanTraversalOperand::Second => {
                    second_endpoints.get(emitted.step.fragment_index)
                }
            };

            let Some((source_start, source_end)) = endpoints else {
                invalid_reference_count += 1;
                continue;
            };

            let (start, end) = match emitted.action {
                BooleanFragmentAction::KeepSourceDirection => {
                    (source_start.clone(), source_end.clone())
                }
                BooleanFragmentAction::KeepReversed => (source_end.clone(), source_start.clone()),
                BooleanFragmentAction::Discard | BooleanFragmentAction::BoundaryNeedsResolution => {
                    invalid_reference_count += 1;
                    continue;
                }
            };
            directed_fragments.push(BezierBooleanDirectedLoopFragment2 {
                operand: emitted.step.operand,
                fragment_index: emitted.step.fragment_index,
                action: emitted.action,
                start,
                end,
            });
        }

        if invalid_reference_count > 0 {
            return Self {
                status: BezierBooleanLoopClosureStatus::InvalidFragmentReference,
                plan_status: plan.status,
                operation: plan.operation,
                directed_fragments,
                emitted_step_count: plan.emitted_steps.len(),
                invalid_reference_count,
                adjacency_gap_count: 0,
                open_chain_count: 0,
                closed_loop_count: 0,
                blocker_count: invalid_reference_count,
            };
        }

        let (closed_loop_count, open_chain_count, adjacency_gap_count) =
            count_directed_loop_closure(&directed_fragments);
        let blocker_count = adjacency_gap_count + open_chain_count;
        let status = if blocker_count == 0 && closed_loop_count > 0 {
            BezierBooleanLoopClosureStatus::Closed
        } else {
            BezierBooleanLoopClosureStatus::OpenChains
        };

        Self {
            status,
            plan_status: plan.status,
            operation: plan.operation,
            directed_fragments,
            emitted_step_count: plan.emitted_steps.len(),
            invalid_reference_count: 0,
            adjacency_gap_count,
            open_chain_count,
            closed_loop_count,
            blocker_count,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopClosureStatus,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            plan_status: plan.status,
            operation: plan.operation,
            directed_fragments: Vec::new(),
            emitted_step_count: 0,
            invalid_reference_count: plan.invalid_reference_count,
            adjacency_gap_count: 0,
            open_chain_count: 0,
            closed_loop_count: 0,
            blocker_count,
        }
    }

    /// Returns true when emitted fragments form exact closed loops.
    pub fn is_closed(&self) -> bool {
        self.status == BezierBooleanLoopClosureStatus::Closed
    }

    /// Returns true when stale references or open chains prevent loop output.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopClosureStatus::PlanBlocked
                | BezierBooleanLoopClosureStatus::InvalidFragmentReference
                | BezierBooleanLoopClosureStatus::OpenChains
        )
    }
}

impl BezierBooleanOutputLoopReport2 {
    /// Packages output loops after graph-walk-aware quadratic closure.
    ///
    /// This constructor threads the certified graph walk through
    /// [`BezierBooleanLoopClosureReport2::from_quadratic_graph_walk`] before
    /// packaging loops. It exists to make the graph-walk certificate a visible
    /// output-loop precondition. Following Yap, "Towards Exact Geometric
    /// Computation" (1997), malformed graph walks remain blockers; following
    /// Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009),
    /// only a certified traversal order can become output boundary topology.
    pub fn from_quadratic_graph_walk(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
    ) -> Self {
        let closure =
            BezierBooleanLoopClosureReport2::from_quadratic_graph_walk(walk, plan, first, second);
        Self::from_loop_closure(&closure)
    }

    /// Packages output loops after graph-walk-aware cubic closure.
    pub fn from_cubic_graph_walk(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
    ) -> Self {
        let closure =
            BezierBooleanLoopClosureReport2::from_cubic_graph_walk(walk, plan, first, second);
        Self::from_loop_closure(&closure)
    }

    /// Packages output loops after graph-walk-aware rational quadratic closure.
    pub fn from_rational_quadratic_graph_walk(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
    ) -> Self {
        let closure = BezierBooleanLoopClosureReport2::from_rational_quadratic_graph_walk(
            walk, plan, first, second,
        );
        Self::from_loop_closure(&closure)
    }

    /// Packages output loops from a certified graph walk and generic endpoints.
    ///
    /// Ready graph walks are converted into exact directed fragments by the
    /// closure layer; malformed graph walks become `ClosureBlocked` outputs.
    /// The method does not infer loop order, roles, or nesting.
    pub fn from_graph_walk_endpoints(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
    ) -> Self {
        let closure = BezierBooleanLoopClosureReport2::from_graph_walk_endpoints(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_loop_closure(&closure)
    }

    /// Packages output loops from a certified multi-cycle successor walk.
    ///
    /// The multi-cycle successor report supplies both the exact emitted-step
    /// order and the cycle boundaries. After endpoint closure succeeds, this
    /// constructor checks that packaged output-loop ranges match those
    /// certified cycle boundaries. A mismatch is reported as malformed closed
    /// loops rather than silently accepting endpoint evidence that merged or
    /// split graph cycles. This follows Yap (1997): every construction must be
    /// justified by the certificate at the boundary where it is consumed.
    pub fn from_multi_cycle_successor_endpoints(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
    ) -> Self {
        let closure = BezierBooleanLoopClosureReport2::from_multi_cycle_successor_endpoints(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
        );
        let mut output = Self::from_loop_closure(&closure);
        if output.status == BezierBooleanOutputLoopStatus::Ready
            && !output_loop_ranges_match_multi_cycle_walk(&output, multi_cycle)
        {
            output.status = BezierBooleanOutputLoopStatus::MalformedClosedLoops;
            output.blocker_count = output.blocker_count.max(1);
        }
        output
    }

    /// Packages exactly closed directed fragments into output-loop records.
    pub fn from_loop_closure(closure: &BezierBooleanLoopClosureReport2) -> Self {
        match closure.status {
            BezierBooleanLoopClosureStatus::Empty => {
                return Self::empty_like(BezierBooleanOutputLoopStatus::Empty, closure, 0);
            }
            BezierBooleanLoopClosureStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanOutputLoopStatus::NoInteriorSplits,
                    closure,
                    0,
                );
            }
            BezierBooleanLoopClosureStatus::PlanBlocked
            | BezierBooleanLoopClosureStatus::InvalidFragmentReference
            | BezierBooleanLoopClosureStatus::OpenChains => {
                return Self::empty_like(
                    BezierBooleanOutputLoopStatus::ClosureBlocked,
                    closure,
                    closure.blocker_count.max(1),
                );
            }
            BezierBooleanLoopClosureStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanOutputLoopStatus::NoEmittedFragments,
                    closure,
                    0,
                );
            }
            BezierBooleanLoopClosureStatus::Closed => {}
        }

        let loops = collect_output_loop_ranges(&closure.directed_fragments);
        let status = if loops.len() == closure.closed_loop_count && !loops.is_empty() {
            BezierBooleanOutputLoopStatus::Ready
        } else {
            BezierBooleanOutputLoopStatus::MalformedClosedLoops
        };
        let blocker_count =
            usize::from(status == BezierBooleanOutputLoopStatus::MalformedClosedLoops);

        Self {
            status,
            closure_status: closure.status,
            operation: closure.operation,
            directed_fragments: closure.directed_fragments.clone(),
            directed_fragment_count: closure.directed_fragments.len(),
            loops,
            closed_loop_count: closure.closed_loop_count,
            open_chain_count: closure.open_chain_count,
            adjacency_gap_count: closure.adjacency_gap_count,
            invalid_reference_count: closure.invalid_reference_count,
            blocker_count,
        }
    }

    fn empty_like(
        status: BezierBooleanOutputLoopStatus,
        closure: &BezierBooleanLoopClosureReport2,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            closure_status: closure.status,
            operation: closure.operation,
            directed_fragments: Vec::new(),
            loops: Vec::new(),
            closed_loop_count: closure.closed_loop_count,
            directed_fragment_count: 0,
            open_chain_count: closure.open_chain_count,
            adjacency_gap_count: closure.adjacency_gap_count,
            invalid_reference_count: closure.invalid_reference_count,
            blocker_count,
        }
    }

    /// Returns true when closed Bezier/conic loops are packaged for later nesting.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanOutputLoopStatus::Ready
    }

    /// Returns true when loop output still has explicit blockers.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanOutputLoopStatus::ClosureBlocked
                | BezierBooleanOutputLoopStatus::MalformedClosedLoops
        )
    }
}

impl BezierBooleanLoopContainmentFactReport2 {
    /// Validates containment facts and derives keyed nesting-depth facts.
    ///
    /// A valid report has one derived depth fact per output loop. The depth is
    /// the number of certified containers for that loop. This is exactly the
    /// parity input consumed by [`BezierBooleanLoopNestingRoleReport2`], while
    /// keeping containment itself as an external exact predicate/certificate.
    pub fn from_output_loop_containment_facts(
        output: &BezierBooleanOutputLoopReport2,
        facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        match output.status {
            BezierBooleanOutputLoopStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentFactStatus::Empty,
                    output,
                    facts.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentFactStatus::NoInteriorSplits,
                    output,
                    facts.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::ClosureBlocked
            | BezierBooleanOutputLoopStatus::MalformedClosedLoops => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentFactStatus::OutputLoopBlocked,
                    output,
                    facts.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    output.blocker_count.max(1),
                );
            }
            BezierBooleanOutputLoopStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopContainmentFactStatus::NoEmittedFragments,
                    output,
                    facts.len(),
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::Ready => {}
        }

        let loop_count = output.loops.len();
        let out_of_range_fact_count = facts
            .iter()
            .filter(|fact| {
                fact.container_loop_index >= loop_count || fact.contained_loop_index >= loop_count
            })
            .count();
        if out_of_range_fact_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopContainmentFactStatus::OutOfRangeLoopIndex,
                output,
                facts.len(),
                out_of_range_fact_count,
                0,
                0,
                0,
                0,
                out_of_range_fact_count,
            );
        }

        let self_containment_count = facts
            .iter()
            .filter(|fact| fact.container_loop_index == fact.contained_loop_index)
            .count();
        if self_containment_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopContainmentFactStatus::SelfContainment,
                output,
                facts.len(),
                0,
                self_containment_count,
                0,
                0,
                0,
                self_containment_count,
            );
        }

        let mut seen = std::collections::BTreeSet::new();
        let mut duplicate_fact_count = 0;
        for fact in facts {
            if !seen.insert((fact.container_loop_index, fact.contained_loop_index)) {
                duplicate_fact_count += 1;
            }
        }
        if duplicate_fact_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopContainmentFactStatus::DuplicateContainmentFact,
                output,
                facts.len(),
                0,
                0,
                duplicate_fact_count,
                0,
                0,
                duplicate_fact_count,
            );
        }

        let mut adjacency = vec![Vec::new(); loop_count];
        for fact in facts {
            adjacency[fact.container_loop_index].push(fact.contained_loop_index);
        }
        let mut states = vec![0_u8; loop_count];
        let mut cyclic_fact_count = 0_usize;
        for loop_index in 0..loop_count {
            if states[loop_index] == 0 && containment_has_cycle(loop_index, &adjacency, &mut states)
            {
                cyclic_fact_count += 1;
            }
        }
        if cyclic_fact_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopContainmentFactStatus::CyclicContainmentFacts,
                output,
                facts.len(),
                0,
                0,
                0,
                cyclic_fact_count,
                0,
                cyclic_fact_count,
            );
        }

        let mut non_laminar_fact_count = 0_usize;
        for contained_loop_index in 0..loop_count {
            let containers = (0..loop_count)
                .filter(|&candidate_container_index| {
                    candidate_container_index != contained_loop_index && {
                        let mut visited = vec![false; loop_count];
                        containment_reaches_loop(
                            candidate_container_index,
                            contained_loop_index,
                            &adjacency,
                            &mut visited,
                        )
                    }
                })
                .collect::<Vec<_>>();
            for first_container_position in 0..containers.len() {
                for second_container_position in first_container_position + 1..containers.len() {
                    let first_container_index = containers[first_container_position];
                    let second_container_index = containers[second_container_position];
                    let mut first_visited = vec![false; loop_count];
                    let first_contains_second = containment_reaches_loop(
                        first_container_index,
                        second_container_index,
                        &adjacency,
                        &mut first_visited,
                    );
                    let mut second_visited = vec![false; loop_count];
                    let second_contains_first = containment_reaches_loop(
                        second_container_index,
                        first_container_index,
                        &adjacency,
                        &mut second_visited,
                    );
                    if !first_contains_second && !second_contains_first {
                        non_laminar_fact_count += 1;
                    }
                }
            }
        }
        if non_laminar_fact_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopContainmentFactStatus::NonLaminarContainmentFacts,
                output,
                facts.len(),
                0,
                0,
                0,
                0,
                non_laminar_fact_count,
                non_laminar_fact_count,
            );
        }

        let mut depths = vec![0_usize; loop_count];
        for contained_loop_index in 0..loop_count {
            for candidate_container_index in 0..loop_count {
                if candidate_container_index == contained_loop_index {
                    continue;
                }
                let mut visited = vec![false; loop_count];
                if containment_reaches_loop(
                    candidate_container_index,
                    contained_loop_index,
                    &adjacency,
                    &mut visited,
                ) {
                    depths[contained_loop_index] += 1;
                }
            }
        }
        let depth_facts = depths
            .into_iter()
            .enumerate()
            .map(
                |(loop_index, nesting_depth)| BezierBooleanLoopNestingDepthFact2 {
                    loop_index,
                    nesting_depth,
                },
            )
            .collect::<Vec<_>>();

        Self {
            status: BezierBooleanLoopContainmentFactStatus::Ready,
            output_status: output.status,
            operation: output.operation,
            output_loop_count: loop_count,
            supplied_fact_count: facts.len(),
            facts: facts.to_vec(),
            depth_facts,
            out_of_range_fact_count: 0,
            self_containment_count: 0,
            duplicate_fact_count: 0,
            cyclic_fact_count: 0,
            non_laminar_fact_count: 0,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopContainmentFactStatus,
        output: &BezierBooleanOutputLoopReport2,
        supplied_fact_count: usize,
        out_of_range_fact_count: usize,
        self_containment_count: usize,
        duplicate_fact_count: usize,
        cyclic_fact_count: usize,
        non_laminar_fact_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            output_status: output.status,
            operation: output.operation,
            output_loop_count: output.loops.len(),
            supplied_fact_count,
            facts: Vec::new(),
            depth_facts: Vec::new(),
            out_of_range_fact_count,
            self_containment_count,
            duplicate_fact_count,
            cyclic_fact_count,
            non_laminar_fact_count,
            blocker_count,
        }
    }

    /// Returns true when containment facts derive one depth per output loop.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopContainmentFactStatus::Ready
    }

    /// Returns true when containment facts are stale or invalid.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopContainmentFactStatus::OutputLoopBlocked
                | BezierBooleanLoopContainmentFactStatus::OutOfRangeLoopIndex
                | BezierBooleanLoopContainmentFactStatus::SelfContainment
                | BezierBooleanLoopContainmentFactStatus::DuplicateContainmentFact
                | BezierBooleanLoopContainmentFactStatus::CyclicContainmentFacts
                | BezierBooleanLoopContainmentFactStatus::NonLaminarContainmentFacts
        )
    }
}

fn containment_has_cycle(loop_index: usize, adjacency: &[Vec<usize>], states: &mut [u8]) -> bool {
    states[loop_index] = 1;
    for &contained_loop_index in &adjacency[loop_index] {
        if states[contained_loop_index] == 1 {
            return true;
        }
        if states[contained_loop_index] == 0
            && containment_has_cycle(contained_loop_index, adjacency, states)
        {
            return true;
        }
    }
    states[loop_index] = 2;
    false
}

fn containment_reaches_loop(
    current_loop_index: usize,
    target_loop_index: usize,
    adjacency: &[Vec<usize>],
    visited: &mut [bool],
) -> bool {
    if current_loop_index == target_loop_index {
        return true;
    }
    if visited[current_loop_index] {
        return false;
    }
    visited[current_loop_index] = true;
    adjacency[current_loop_index]
        .iter()
        .any(|&next_loop_index| {
            containment_reaches_loop(next_loop_index, target_loop_index, adjacency, visited)
        })
}

impl BezierBooleanLoopNestingDepthFactReport2 {
    /// Validates keyed nesting-depth facts against packaged output loops.
    ///
    /// Facts must be supplied in output-loop order and each fact must repeat
    /// the exact loop index it classifies. The report does not compute
    /// containment and does not infer material/hole roles from orientation.
    /// It only validates certified loop-depth facts produced by an external
    /// exact nesting stage, preserving Yap's predicate/construction boundary
    /// from "Towards Exact Geometric Computation" (1997).
    pub fn from_output_loop_facts(
        output: &BezierBooleanOutputLoopReport2,
        facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        match output.status {
            BezierBooleanOutputLoopStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopNestingDepthFactStatus::Empty,
                    output,
                    facts.len(),
                    0,
                    0,
                    0,
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopNestingDepthFactStatus::NoInteriorSplits,
                    output,
                    facts.len(),
                    0,
                    facts.len(),
                    0,
                    facts.len(),
                );
            }
            BezierBooleanOutputLoopStatus::ClosureBlocked
            | BezierBooleanOutputLoopStatus::MalformedClosedLoops => {
                return Self::empty_like(
                    BezierBooleanLoopNestingDepthFactStatus::OutputLoopBlocked,
                    output,
                    facts.len(),
                    0,
                    0,
                    0,
                    output.blocker_count.max(1),
                );
            }
            BezierBooleanOutputLoopStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopNestingDepthFactStatus::NoEmittedFragments,
                    output,
                    facts.len(),
                    0,
                    facts.len(),
                    0,
                    facts.len(),
                );
            }
            BezierBooleanOutputLoopStatus::Ready => {}
        }

        if facts.len() < output.loops.len() {
            let missing = output.loops.len() - facts.len();
            return Self::empty_like(
                BezierBooleanLoopNestingDepthFactStatus::MissingNestingDepthFacts,
                output,
                facts.len(),
                missing,
                0,
                0,
                missing.max(1),
            );
        }

        if facts.len() > output.loops.len() {
            let extra = facts.len() - output.loops.len();
            return Self::empty_like(
                BezierBooleanLoopNestingDepthFactStatus::ExtraNestingDepthFacts,
                output,
                facts.len(),
                0,
                extra,
                0,
                extra.max(1),
            );
        }

        let loop_index_mismatch_count = facts
            .iter()
            .enumerate()
            .filter(|(expected, fact)| *expected != fact.loop_index)
            .count();

        if loop_index_mismatch_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopNestingDepthFactStatus::LoopIndexMismatch,
                output,
                facts.len(),
                0,
                0,
                loop_index_mismatch_count,
                loop_index_mismatch_count,
            );
        }

        Self {
            status: BezierBooleanLoopNestingDepthFactStatus::Ready,
            output_status: output.status,
            operation: output.operation,
            output_loop_count: output.loops.len(),
            supplied_fact_count: facts.len(),
            facts: facts.to_vec(),
            depths: facts.iter().map(|fact| fact.nesting_depth).collect(),
            missing_fact_count: 0,
            extra_fact_count: 0,
            loop_index_mismatch_count: 0,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopNestingDepthFactStatus,
        output: &BezierBooleanOutputLoopReport2,
        supplied_fact_count: usize,
        missing_fact_count: usize,
        extra_fact_count: usize,
        loop_index_mismatch_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            output_status: output.status,
            operation: output.operation,
            output_loop_count: output.loops.len(),
            supplied_fact_count,
            facts: Vec::new(),
            depths: Vec::new(),
            missing_fact_count,
            extra_fact_count,
            loop_index_mismatch_count,
            blocker_count,
        }
    }

    /// Generates material/hole roles from the validated depth facts.
    pub fn generate_roles(
        &self,
        output: &BezierBooleanOutputLoopReport2,
    ) -> BezierBooleanLoopNestingRoleReport2 {
        BezierBooleanLoopNestingRoleReport2::from_output_loop_depths(output, &self.depths)
    }

    /// Returns true when every output loop has a keyed nesting depth.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopNestingDepthFactStatus::Ready
    }

    /// Returns true when more exact nesting facts are required.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopNestingDepthFactStatus::OutputLoopBlocked
                | BezierBooleanLoopNestingDepthFactStatus::MissingNestingDepthFacts
                | BezierBooleanLoopNestingDepthFactStatus::ExtraNestingDepthFacts
                | BezierBooleanLoopNestingDepthFactStatus::LoopIndexMismatch
        )
    }
}

impl BezierBooleanLoopNestingRoleReport2 {
    /// Generates material/hole roles from externally certified nesting depths.
    pub fn from_output_loop_depths(
        output: &BezierBooleanOutputLoopReport2,
        nesting_depths: &[usize],
    ) -> Self {
        match output.status {
            BezierBooleanOutputLoopStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopNestingRoleStatus::Empty,
                    output,
                    nesting_depths.len(),
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopNestingRoleStatus::NoInteriorSplits,
                    output,
                    nesting_depths.len(),
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::ClosureBlocked
            | BezierBooleanOutputLoopStatus::MalformedClosedLoops => {
                return Self::empty_like(
                    BezierBooleanLoopNestingRoleStatus::OutputLoopBlocked,
                    output,
                    nesting_depths.len(),
                    output.blocker_count.max(1),
                );
            }
            BezierBooleanOutputLoopStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopNestingRoleStatus::NoEmittedFragments,
                    output,
                    nesting_depths.len(),
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::Ready => {}
        }

        let missing_depth_count = output.loops.len().saturating_sub(nesting_depths.len());
        let extra_depth_count = nesting_depths.len().saturating_sub(output.loops.len());
        if missing_depth_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopNestingRoleStatus::MissingNestingDepthFacts,
                output,
                nesting_depths.len(),
                missing_depth_count,
            );
        }
        if extra_depth_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopNestingRoleStatus::ExtraNestingDepthFacts,
                output,
                nesting_depths.len(),
                extra_depth_count,
            );
        }

        let mut roles = Vec::with_capacity(nesting_depths.len());
        let mut material_loop_count = 0;
        let mut hole_loop_count = 0;
        for depth in nesting_depths {
            let role = if depth % 2 == 0 {
                material_loop_count += 1;
                BezierBooleanOutputLoopRole::Material
            } else {
                hole_loop_count += 1;
                BezierBooleanOutputLoopRole::Hole
            };
            roles.push(role);
        }

        Self {
            status: BezierBooleanLoopNestingRoleStatus::Ready,
            output_status: output.status,
            operation: output.operation,
            roles,
            output_loop_count: output.loops.len(),
            supplied_depth_count: nesting_depths.len(),
            material_loop_count,
            hole_loop_count,
            missing_depth_count: 0,
            extra_depth_count: 0,
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanLoopNestingRoleStatus,
        output: &BezierBooleanOutputLoopReport2,
        supplied_depth_count: usize,
        blocker_count: usize,
    ) -> Self {
        let missing_depth_count =
            if status == BezierBooleanLoopNestingRoleStatus::MissingNestingDepthFacts {
                output.loops.len().saturating_sub(supplied_depth_count)
            } else {
                0
            };
        let extra_depth_count =
            if status == BezierBooleanLoopNestingRoleStatus::ExtraNestingDepthFacts {
                supplied_depth_count.saturating_sub(output.loops.len())
            } else {
                0
            };
        Self {
            status,
            output_status: output.status,
            operation: output.operation,
            roles: Vec::new(),
            output_loop_count: output.loops.len(),
            supplied_depth_count,
            material_loop_count: 0,
            hole_loop_count: 0,
            missing_depth_count,
            extra_depth_count,
            blocker_count,
        }
    }

    /// Returns true when nesting depths generated one role per output loop.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopNestingRoleStatus::Ready
    }

    /// Returns true when role generation lacks certified nesting-depth facts.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopNestingRoleStatus::OutputLoopBlocked
                | BezierBooleanLoopNestingRoleStatus::MissingNestingDepthFacts
                | BezierBooleanLoopNestingRoleStatus::ExtraNestingDepthFacts
        )
    }
}

impl BezierBooleanLoopRoleAssignmentReport2 {
    /// Assigns externally certified material/hole roles to closed output loops.
    pub fn from_output_loops(
        output: &BezierBooleanOutputLoopReport2,
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        match output.status {
            BezierBooleanOutputLoopStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanLoopRoleAssignmentStatus::Empty,
                    output,
                    roles.len(),
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanLoopRoleAssignmentStatus::NoInteriorSplits,
                    output,
                    roles.len(),
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::ClosureBlocked
            | BezierBooleanOutputLoopStatus::MalformedClosedLoops => {
                return Self::empty_like(
                    BezierBooleanLoopRoleAssignmentStatus::OutputLoopBlocked,
                    output,
                    roles.len(),
                    output.blocker_count.max(1),
                );
            }
            BezierBooleanOutputLoopStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanLoopRoleAssignmentStatus::NoEmittedFragments,
                    output,
                    roles.len(),
                    0,
                );
            }
            BezierBooleanOutputLoopStatus::Ready => {}
        }

        let missing_role_count = output.loops.len().saturating_sub(roles.len());
        let extra_role_count = roles.len().saturating_sub(output.loops.len());
        if missing_role_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopRoleAssignmentStatus::MissingRoleFacts,
                output,
                roles.len(),
                missing_role_count,
            );
        }
        if extra_role_count > 0 {
            return Self::empty_like(
                BezierBooleanLoopRoleAssignmentStatus::ExtraRoleFacts,
                output,
                roles.len(),
                extra_role_count,
            );
        }

        let mut assigned_loops = Vec::with_capacity(output.loops.len());
        let mut material_loop_count = 0;
        let mut hole_loop_count = 0;
        let mut unknown_role_count = 0;
        for (output_loop, role) in output.loops.iter().cloned().zip(roles.iter().copied()) {
            match role {
                BezierBooleanOutputLoopRole::Material => material_loop_count += 1,
                BezierBooleanOutputLoopRole::Hole => hole_loop_count += 1,
                BezierBooleanOutputLoopRole::Unknown => unknown_role_count += 1,
            }
            assigned_loops.push(BezierBooleanAssignedOutputLoop2 { output_loop, role });
        }

        let status = if unknown_role_count == 0 {
            BezierBooleanLoopRoleAssignmentStatus::Ready
        } else {
            BezierBooleanLoopRoleAssignmentStatus::UnknownRole
        };

        Self {
            status,
            output_status: output.status,
            operation: output.operation,
            directed_fragments: output.directed_fragments.clone(),
            assigned_loops,
            output_loop_count: output.loops.len(),
            supplied_role_count: roles.len(),
            material_loop_count,
            hole_loop_count,
            unknown_role_count,
            role_parity_mismatch_count: 0,
            missing_role_count: 0,
            extra_role_count: 0,
            blocker_count: unknown_role_count,
        }
    }

    /// Assigns externally supplied roles only when they match keyed depth parity.
    ///
    /// This is the stricter role-ingestion path for boolean callers that have
    /// both nesting-depth certificates and explicit material/hole role facts.
    /// It first validates [`BezierBooleanLoopNestingDepthFact2`] keys, then
    /// validates role cardinality/unknowns, then checks the alternating
    /// material/hole parity implied by the certified depth of each output loop.
    /// The separation follows Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009), where boundary construction and nesting
    /// interpretation are distinct stages. Yap, "Towards Exact Geometric
    /// Computation" (1997), is the rule enforced here: a stale role certificate
    /// is a blocker, not a reason to infer topology from orientation, sampled
    /// points, or caller intent.
    pub fn from_output_loop_depth_role_facts(
        output: &BezierBooleanOutputLoopReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let depth_report =
            BezierBooleanLoopNestingDepthFactReport2::from_output_loop_facts(output, depth_facts);
        if !depth_report.is_ready() {
            return Self::empty_like(
                BezierBooleanLoopRoleAssignmentStatus::NestingDepthFactBlocked,
                output,
                roles.len(),
                depth_report.blocker_count.max(1),
            );
        }

        let mut assigned = Self::from_output_loops(output, roles);
        if !assigned.is_ready() {
            return assigned;
        }

        let role_parity_mismatch_count = depth_report
            .depths
            .iter()
            .zip(roles.iter())
            .filter(|(depth, role)| {
                let expected = if **depth % 2 == 0 {
                    BezierBooleanOutputLoopRole::Material
                } else {
                    BezierBooleanOutputLoopRole::Hole
                };
                **role != expected
            })
            .count();

        if role_parity_mismatch_count > 0 {
            assigned.status = BezierBooleanLoopRoleAssignmentStatus::RoleParityMismatch;
            assigned.role_parity_mismatch_count = role_parity_mismatch_count;
            assigned.blocker_count = role_parity_mismatch_count;
        }

        assigned
    }

    fn empty_like(
        status: BezierBooleanLoopRoleAssignmentStatus,
        output: &BezierBooleanOutputLoopReport2,
        supplied_role_count: usize,
        blocker_count: usize,
    ) -> Self {
        let missing_role_count =
            if status == BezierBooleanLoopRoleAssignmentStatus::MissingRoleFacts {
                output.loops.len().saturating_sub(supplied_role_count)
            } else {
                0
            };
        let extra_role_count = if status == BezierBooleanLoopRoleAssignmentStatus::ExtraRoleFacts {
            supplied_role_count.saturating_sub(output.loops.len())
        } else {
            0
        };
        Self {
            status,
            output_status: output.status,
            operation: output.operation,
            directed_fragments: Vec::new(),
            assigned_loops: Vec::new(),
            output_loop_count: output.loops.len(),
            supplied_role_count,
            material_loop_count: 0,
            hole_loop_count: 0,
            unknown_role_count: 0,
            role_parity_mismatch_count: 0,
            missing_role_count,
            extra_role_count,
            blocker_count,
        }
    }

    /// Returns true when every output loop has a certified material/hole role.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanLoopRoleAssignmentStatus::Ready
    }

    /// Returns true when missing, extra, unknown, or blocked role facts remain.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanLoopRoleAssignmentStatus::OutputLoopBlocked
                | BezierBooleanLoopRoleAssignmentStatus::MissingRoleFacts
                | BezierBooleanLoopRoleAssignmentStatus::ExtraRoleFacts
                | BezierBooleanLoopRoleAssignmentStatus::UnknownRole
                | BezierBooleanLoopRoleAssignmentStatus::NestingDepthFactBlocked
                | BezierBooleanLoopRoleAssignmentStatus::RoleParityMismatch
        )
    }
}

impl BezierBooleanRegionAssemblyReport2 {
    /// Packages output loops into a region artifact using containment facts.
    ///
    /// This composes containment-fact validation into keyed nesting-depth
    /// generation before role assignment. It does not test containment itself;
    /// the supplied pairs are exact predicate certificates that are validated
    /// before they affect material/hole parity.
    pub fn from_output_loop_containment_facts(
        output: &BezierBooleanOutputLoopReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let containment =
            BezierBooleanLoopContainmentFactReport2::from_output_loop_containment_facts(
                output,
                containment_facts,
            );
        Self::from_output_loop_depth_facts(output, &containment.depth_facts)
    }

    /// Packages output loops when explicit roles agree with containment-derived depths.
    ///
    /// This constructor validates exact containment-pair certificates, derives
    /// keyed nesting-depth facts, and then accepts explicit material/hole role
    /// facts only when they match the derived depth parity. It keeps
    /// containment, nesting, and fill-role interpretation as separate
    /// certificate stages, following Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009). Yap, "Towards Exact Geometric
    /// Computation" (1997), is the exactness rule: stale role facts block
    /// construction rather than overriding containment evidence.
    pub fn from_output_loop_containment_role_facts(
        output: &BezierBooleanOutputLoopReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let containment =
            BezierBooleanLoopContainmentFactReport2::from_output_loop_containment_facts(
                output,
                containment_facts,
            );
        Self::from_output_loop_depth_role_facts(output, &containment.depth_facts, roles)
    }

    /// Packages output loops from replayed exact containment-query results.
    ///
    /// This is the query-result counterpart to
    /// [`Self::from_output_loop_containment_facts`]. A point/loop locator first
    /// answers the ordered worklist from
    /// [`BezierBooleanLoopContainmentQueryReport2`], then
    /// [`BezierBooleanLoopContainmentQueryResultReport2`] validates the
    /// object keys and lowers only strict `Contains` answers to containment
    /// facts. This constructor consumes that replay report directly so a
    /// blocked boundary, unknown, stale, missing, or extra query result cannot
    /// be accidentally treated as "no containment." This is Yap's "Towards
    /// Exact Geometric Computation" (1997) predicate/construction boundary:
    /// unresolved predicates remain explicit blockers. The loop nesting/fill
    /// separation follows Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009).
    pub fn from_output_loop_containment_query_results(
        output: &BezierBooleanOutputLoopReport2,
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        if query_results.has_blockers() || query_results.operation != output.operation {
            return Self {
                status: BezierBooleanRegionAssemblyStatus::RoleAssignmentBlocked,
                role_status: BezierBooleanLoopRoleAssignmentStatus::NestingDepthFactBlocked,
                operation: output.operation,
                directed_fragments: Vec::new(),
                assigned_loops: Vec::new(),
                material_loop_indices: Vec::new(),
                hole_loop_indices: Vec::new(),
                assigned_loop_count: 0,
                material_loop_count: 0,
                hole_loop_count: 0,
                blocker_count: query_results
                    .blocker_count
                    .max(usize::from(query_results.operation != output.operation))
                    .max(1),
            };
        }

        Self::from_output_loop_containment_facts(output, &query_results.containment_facts)
    }

    /// Packages output loops from an end-to-end containment certificate.
    ///
    /// Unlike [`Self::from_output_loop_containment_query_results`], this
    /// constructor consumes the atomic certificate that rebuilt the query
    /// worklist from the same output-loop package. That prevents a result
    /// replay produced for one loop vector from being paired with another.
    /// This follows Yap (1997): certified construction consumes object-keyed
    /// predicate evidence, not detached samples or caller-maintained ordering.
    pub fn from_output_loop_containment_certification(
        output: &BezierBooleanOutputLoopReport2,
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        if certification.has_blockers() || certification.operation != output.operation {
            return Self {
                status: BezierBooleanRegionAssemblyStatus::RoleAssignmentBlocked,
                role_status: BezierBooleanLoopRoleAssignmentStatus::NestingDepthFactBlocked,
                operation: output.operation,
                directed_fragments: Vec::new(),
                assigned_loops: Vec::new(),
                material_loop_indices: Vec::new(),
                hole_loop_indices: Vec::new(),
                assigned_loop_count: 0,
                material_loop_count: 0,
                hole_loop_count: 0,
                blocker_count: certification
                    .blocker_count
                    .max(usize::from(certification.operation != output.operation))
                    .max(1),
            };
        }

        Self::from_output_loop_depth_facts(output, &certification.depth_facts)
    }

    /// Packages graph-walk-certified generic endpoints using containment facts.
    pub fn from_graph_walk_containment_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_graph_walk_endpoints(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_containment_facts(&output, containment_facts)
    }

    /// Packages graph-walk-certified endpoints from replayed containment queries.
    ///
    /// The graph walk certifies boundary order and closure, while the query
    /// replay certifies loop containment. Keeping those certificates separate
    /// follows the staged boolean model of Vatti (1992), Greiner-Hormann
    /// (1998), and Martinez-Rueda-Feito (2009); consuming the replay report as
    /// an all-or-blocking input follows Yap (1997).
    pub fn from_graph_walk_containment_query_results(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_graph_walk_endpoints(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_containment_query_results(&output, query_results)
    }

    /// Packages graph-walk-certified endpoints from an end-to-end containment certificate.
    pub fn from_graph_walk_containment_certification(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_graph_walk_endpoints(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_containment_certification(&output, certification)
    }

    /// Packages graph-walk-certified endpoints using containment and role facts.
    ///
    /// The graph walk supplies exact output-loop order, containment facts
    /// derive keyed depths, and explicit roles are accepted only when they
    /// match containment-derived parity.
    pub fn from_graph_walk_containment_role_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_graph_walk_endpoints(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_containment_role_facts(&output, containment_facts, roles)
    }

    /// Packages multi-cycle-successor-certified endpoints using containment facts.
    ///
    /// This constructor is the containment counterpart to
    /// [`Self::from_multi_cycle_successor_depth_facts`]. The successor
    /// certificate fixes the exact output-cycle topology, endpoint closure
    /// proves the directed boundary ranges, and containment facts are lowered
    /// into keyed nesting-depth facts only after those output loops exist.
    /// That preserves Yap's "Towards Exact Geometric Computation" (1997)
    /// predicate/construction boundary while following the traversal/fill
    /// separation of Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009).
    pub fn from_multi_cycle_successor_containment_facts(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_multi_cycle_successor_endpoints(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_containment_facts(&output, containment_facts)
    }

    /// Packages multi-cycle-successor-certified endpoints using containment and roles.
    ///
    /// Explicit material/hole roles are accepted only when the containment
    /// pairs derive matching nesting-depth parity. This prevents orientation or
    /// caller ordering from overriding exact containment evidence.
    pub fn from_multi_cycle_successor_containment_role_facts(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_multi_cycle_successor_endpoints(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_containment_role_facts(&output, containment_facts, roles)
    }

    /// Packages multi-cycle-successor-certified endpoints from replayed containment queries.
    ///
    /// The replay report must be ready and keyed to the same operation as the
    /// output loops produced by the successor certificate. Boundary, unknown,
    /// missing, extra, or stale query answers remain blockers instead of being
    /// treated as empty containment. This is Yap's (1997)
    /// predicate-before-construction rule applied after exact cycle-preserving
    /// output-loop construction.
    pub fn from_multi_cycle_successor_containment_query_results(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_multi_cycle_successor_endpoints(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_containment_query_results(&output, query_results)
    }

    /// Packages multi-cycle-successor-certified endpoints from an end-to-end containment certificate.
    ///
    /// The certificate carries query replay, containment facts, and derived
    /// depths for a concrete output-loop package. This constructor consumes it
    /// only after rebuilding the output loops from the same successor and
    /// endpoint certificates, so detached locator evidence cannot silently
    /// bypass cycle-boundary validation.
    pub fn from_multi_cycle_successor_containment_certification(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_multi_cycle_successor_endpoints(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_containment_certification(&output, certification)
    }

    /// Packages output loops into a higher-order region artifact using keyed depths.
    ///
    /// This constructor composes the certified post-closure stages:
    /// [`BezierBooleanLoopNestingDepthFactReport2`] validates loop-indexed
    /// nesting facts, [`BezierBooleanLoopNestingRoleReport2`] maps depth parity
    /// to material/hole roles, and [`BezierBooleanLoopRoleAssignmentReport2`]
    /// binds those roles back to exact output loops. The composition follows
    /// Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009):
    /// traversal, loop construction, nesting, and region packaging are separate
    /// certified stages. Yap, "Towards Exact Geometric Computation" (1997), is
    /// the exactness rule: missing, stale, or blocked depth facts propagate as
    /// report blockers instead of orientation/sample inference.
    pub fn from_output_loop_depth_facts(
        output: &BezierBooleanOutputLoopReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let depths =
            BezierBooleanLoopNestingDepthFactReport2::from_output_loop_facts(output, depth_facts);
        let roles = depths.generate_roles(output);
        let assigned =
            BezierBooleanLoopRoleAssignmentReport2::from_output_loops(output, &roles.roles);
        Self::from_role_assignment(&assigned)
    }

    /// Packages output loops when explicit roles agree with keyed depths.
    ///
    /// This constructor is useful when an upstream exact nesting stage emits
    /// both loop-depth and role facts. The roles are accepted only if
    /// [`BezierBooleanLoopRoleAssignmentReport2::from_output_loop_depth_role_facts`]
    /// proves that every role matches depth parity; otherwise region assembly
    /// receives a normal role-assignment blocker.
    pub fn from_output_loop_depth_role_facts(
        output: &BezierBooleanOutputLoopReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let assigned = BezierBooleanLoopRoleAssignmentReport2::from_output_loop_depth_role_facts(
            output,
            depth_facts,
            roles,
        );
        Self::from_role_assignment(&assigned)
    }

    /// Packages graph-walk-certified generic endpoints into a region artifact.
    ///
    /// A ready graph walk feeds output-loop packaging, keyed nesting-depth
    /// validation, parity role generation, and role assignment. Any malformed
    /// graph walk or nesting-depth fact remains a blocker in the returned
    /// region-assembly report.
    pub fn from_graph_walk_depth_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_graph_walk_endpoints(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_depth_facts(&output, depth_facts)
    }

    /// Packages multi-cycle-successor-certified endpoints into a region artifact.
    ///
    /// This is the region-assembly counterpart to
    /// [`BezierBooleanOutputLoopReport2::from_multi_cycle_successor_endpoints`].
    /// The successor report supplies the graph-topology certificate, endpoint
    /// closure supplies the exact boundary construction, and keyed depth facts
    /// supply material/hole parity. Each stage is consumed as report-bearing
    /// evidence rather than recomputed from samples or vector order, matching
    /// Yap, "Towards Exact Geometric Computation" (1997). The traversal,
    /// output-loop, and fill-role split follows Vatti (1992),
    /// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_multi_cycle_successor_depth_facts(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_multi_cycle_successor_endpoints(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_depth_facts(&output, depth_facts)
    }

    /// Packages multi-cycle-successor-certified endpoints with explicit roles.
    ///
    /// Explicit material/hole roles are accepted only after keyed depth facts
    /// prove the same parity. This keeps fill interpretation auditable at the
    /// exact graph/result boundary instead of deriving it from loop orientation.
    pub fn from_multi_cycle_successor_depth_role_facts(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_multi_cycle_successor_endpoints(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_depth_role_facts(&output, depth_facts, roles)
    }

    /// Packages graph-walk-certified endpoints when roles agree with depths.
    ///
    /// This is the graph-walk counterpart to
    /// [`Self::from_output_loop_depth_role_facts`]. The walk first certifies
    /// emitted fragment order and exact closure, keyed nesting-depth facts then
    /// certify loop parity, and explicit role facts are accepted only when they
    /// match that parity. This keeps the Vatti (1992), Greiner-Hormann (1998),
    /// and Martinez-Rueda-Feito (2009) boundary-construction/nesting split
    /// visible at the API surface. Following Yap, "Towards Exact Geometric
    /// Computation" (1997), stale roles remain blockers rather than being
    /// repaired by orientation or sample-point guesses.
    pub fn from_graph_walk_depth_role_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_graph_walk_endpoints(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
        );
        Self::from_output_loop_depth_role_facts(&output, depth_facts, roles)
    }

    /// Packages graph-walk-certified quadratic Bezier loops into a region artifact.
    pub fn from_quadratic_graph_walk_depth_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let output =
            BezierBooleanOutputLoopReport2::from_quadratic_graph_walk(walk, plan, first, second);
        Self::from_output_loop_depth_facts(&output, depth_facts)
    }

    /// Packages graph-walk-certified quadratic Bezier loops with depth-certified roles.
    pub fn from_quadratic_graph_walk_depth_role_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let output =
            BezierBooleanOutputLoopReport2::from_quadratic_graph_walk(walk, plan, first, second);
        Self::from_output_loop_depth_role_facts(&output, depth_facts, roles)
    }

    /// Packages graph-walk-certified cubic Bezier loops into a region artifact.
    pub fn from_cubic_graph_walk_depth_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let output =
            BezierBooleanOutputLoopReport2::from_cubic_graph_walk(walk, plan, first, second);
        Self::from_output_loop_depth_facts(&output, depth_facts)
    }

    /// Packages graph-walk-certified cubic Bezier loops with depth-certified roles.
    pub fn from_cubic_graph_walk_depth_role_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let output =
            BezierBooleanOutputLoopReport2::from_cubic_graph_walk(walk, plan, first, second);
        Self::from_output_loop_depth_role_facts(&output, depth_facts, roles)
    }

    /// Packages graph-walk-certified rational quadratic loops into a region artifact.
    pub fn from_rational_quadratic_graph_walk_depth_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_rational_quadratic_graph_walk(
            walk, plan, first, second,
        );
        Self::from_output_loop_depth_facts(&output, depth_facts)
    }

    /// Packages graph-walk-certified rational quadratic/conic loops with depth-certified roles.
    pub fn from_rational_quadratic_graph_walk_depth_role_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let output = BezierBooleanOutputLoopReport2::from_rational_quadratic_graph_walk(
            walk, plan, first, second,
        );
        Self::from_output_loop_depth_role_facts(&output, depth_facts, roles)
    }

    /// Packages role-assigned Bezier/conic loops for future region materialization.
    pub fn from_role_assignment(roles: &BezierBooleanLoopRoleAssignmentReport2) -> Self {
        match roles.status {
            BezierBooleanLoopRoleAssignmentStatus::Empty => {
                return Self::empty_like(BezierBooleanRegionAssemblyStatus::Empty, roles, 0);
            }
            BezierBooleanLoopRoleAssignmentStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanRegionAssemblyStatus::NoInteriorSplits,
                    roles,
                    0,
                );
            }
            BezierBooleanLoopRoleAssignmentStatus::OutputLoopBlocked
            | BezierBooleanLoopRoleAssignmentStatus::MissingRoleFacts
            | BezierBooleanLoopRoleAssignmentStatus::ExtraRoleFacts
            | BezierBooleanLoopRoleAssignmentStatus::UnknownRole
            | BezierBooleanLoopRoleAssignmentStatus::NestingDepthFactBlocked
            | BezierBooleanLoopRoleAssignmentStatus::RoleParityMismatch => {
                return Self::empty_like(
                    BezierBooleanRegionAssemblyStatus::RoleAssignmentBlocked,
                    roles,
                    roles.blocker_count.max(1),
                );
            }
            BezierBooleanLoopRoleAssignmentStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanRegionAssemblyStatus::NoEmittedFragments,
                    roles,
                    0,
                );
            }
            BezierBooleanLoopRoleAssignmentStatus::Ready => {}
        }

        let mut material_loop_indices = Vec::new();
        let mut hole_loop_indices = Vec::new();
        for (index, assigned) in roles.assigned_loops.iter().enumerate() {
            match assigned.role {
                BezierBooleanOutputLoopRole::Material => material_loop_indices.push(index),
                BezierBooleanOutputLoopRole::Hole => hole_loop_indices.push(index),
                BezierBooleanOutputLoopRole::Unknown => {}
            }
        }

        let status = if material_loop_indices.is_empty() && !hole_loop_indices.is_empty() {
            BezierBooleanRegionAssemblyStatus::HoleWithoutMaterial
        } else {
            BezierBooleanRegionAssemblyStatus::Ready
        };
        let blocker_count =
            usize::from(status == BezierBooleanRegionAssemblyStatus::HoleWithoutMaterial);

        Self {
            status,
            role_status: roles.status,
            operation: roles.operation,
            directed_fragments: roles.directed_fragments.clone(),
            assigned_loops: roles.assigned_loops.clone(),
            assigned_loop_count: roles.assigned_loops.len(),
            material_loop_count: material_loop_indices.len(),
            hole_loop_count: hole_loop_indices.len(),
            material_loop_indices,
            hole_loop_indices,
            blocker_count,
        }
    }

    fn empty_like(
        status: BezierBooleanRegionAssemblyStatus,
        roles: &BezierBooleanLoopRoleAssignmentReport2,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            role_status: roles.status,
            operation: roles.operation,
            directed_fragments: Vec::new(),
            assigned_loops: Vec::new(),
            material_loop_indices: Vec::new(),
            hole_loop_indices: Vec::new(),
            assigned_loop_count: 0,
            material_loop_count: 0,
            hole_loop_count: 0,
            blocker_count,
        }
    }

    /// Returns true when role-assigned loops can feed future region materialization.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanRegionAssemblyStatus::Ready
    }

    /// Returns true when role assignment or hole ownership still blocks region output.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanRegionAssemblyStatus::RoleAssignmentBlocked
                | BezierBooleanRegionAssemblyStatus::HoleWithoutMaterial
        )
    }
}

impl BezierBooleanResultReport2 {
    /// Accepts a simple certified linear result from uniform ownership and depth facts.
    ///
    /// This constructor is the keyed-depth counterpart to
    /// [`Self::from_schedule_uniform_linear_identity_containment_facts`]. It
    /// is useful when a caller already has exact output-loop nesting depths and
    /// does not need the containment-pair-to-depth derivation layer. The method
    /// still generates graph facts only for the linear no-branch case and
    /// copies resolved-overlap obligations from the traversal schedule, so
    /// overlap cases remain explicit blockers. This follows Yap, "Towards
    /// Exact Geometric Computation" (1997): exact combinatorial facts are
    /// replayed as data before construction, while unsupported cases stay
    /// report-bearing. The staged clipping model follows Vatti (1992),
    /// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_uniform_linear_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_uniform_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph_facts = BezierBooleanLoopGraphFacts2 {
            emitted_step_count: plan.emitted_steps.len(),
            branch_vertex_count: 0,
            resolved_overlap_count: schedule.resolved_overlap_count,
        };
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, &graph_facts);
        let traversal = graph.to_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
        Self::from_graph_walk_depth_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        )
    }

    /// Accepts a simple quadratic Bezier result using linear identity traversal and depth facts.
    pub fn from_quadratic_schedule_uniform_linear_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_uniform_linear_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            depth_facts,
        )
    }

    /// Accepts a simple cubic Bezier result using linear identity traversal and depth facts.
    pub fn from_cubic_schedule_uniform_linear_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_uniform_linear_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            depth_facts,
        )
    }

    /// Accepts a simple rational quadratic/conic result using linear identity traversal and depth facts.
    pub fn from_rational_quadratic_schedule_uniform_linear_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_uniform_linear_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            depth_facts,
        )
    }

    /// Accepts a simple certified linear result from uniform ownership facts.
    ///
    /// This is the most compact certified constructor for already-linear
    /// Bezier/conic arrangements: the caller is selecting the no-branch linear
    /// graph certificate, while any resolved-overlap obligations carried by
    /// the traversal schedule are still copied into graph facts and therefore
    /// block identity traversal. The constructor then expands uniform
    /// ownership, validates the generated graph facts against the emitted plan,
    /// produces an identity walk, checks exact closure, and derives nesting
    /// from keyed containment facts. This follows Yap, "Towards Exact
    /// Geometric Computation" (1997): each combinatorial claim is explicit and
    /// replayed before construction is accepted. The traversal/fill separation
    /// follows Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009).
    pub fn from_schedule_uniform_linear_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_uniform_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph_facts = BezierBooleanLoopGraphFacts2 {
            emitted_step_count: plan.emitted_steps.len(),
            branch_vertex_count: 0,
            resolved_overlap_count: schedule.resolved_overlap_count,
        };
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, &graph_facts);
        let traversal = graph.to_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
        Self::from_graph_walk_containment_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        )
    }

    /// Accepts a simple quadratic Bezier result using a linear identity graph certificate.
    pub fn from_quadratic_schedule_uniform_linear_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_uniform_linear_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            containment_facts,
        )
    }

    /// Accepts a simple cubic Bezier result using a linear identity graph certificate.
    pub fn from_cubic_schedule_uniform_linear_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_uniform_linear_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            containment_facts,
        )
    }

    /// Accepts a simple rational quadratic/conic result using a linear identity graph certificate.
    pub fn from_rational_quadratic_schedule_uniform_linear_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_uniform_linear_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            containment_facts,
        )
    }

    /// Accepts a simple certified result from uniform ownership and graph facts.
    ///
    /// This is the shortest fully certified path for separated or
    /// whole-component containment arrangements. A caller supplies two exact
    /// operand-level locator facts, keyed graph facts, and keyed containment
    /// facts. The constructor expands uniform ownership into per-fragment facts,
    /// validates graph facts against the emitted plan, produces the identity
    /// walk only when the graph is certified linear, then runs exact closure and
    /// containment-based nesting. It does not infer topology from samples or
    /// vector order. Yap, "Towards Exact Geometric Computation" (1997), is the
    /// exactness contract; the staged clipping/traversal model follows Vatti
    /// (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_uniform_graph_fact_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_uniform_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
        Self::from_graph_walk_containment_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        )
    }

    /// Accepts a simple certified result from uniform ownership, graph facts, and depth facts.
    ///
    /// This is the keyed-depth counterpart to
    /// [`Self::from_schedule_uniform_graph_fact_identity_containment_facts`].
    /// The caller supplies operand-level locator facts, graph facts for the
    /// emitted plan, and exact loop nesting depths. The constructor expands
    /// ownership, validates the graph certificate, accepts only the identity
    /// walk for no-branch/no-overlap traversal, checks endpoint closure, then
    /// assigns material and hole roles from keyed depths. Yap, "Towards Exact
    /// Geometric Computation" (1997), is the acceptance model here: every
    /// combinatorial fact is explicit input and any missing or stale fact
    /// remains a blocker. The boolean staging follows Vatti (1992),
    /// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_uniform_graph_fact_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_uniform_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
        Self::from_graph_walk_depth_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        )
    }

    /// Accepts a simple quadratic Bezier result from uniform ownership and graph facts.
    pub fn from_quadratic_schedule_uniform_graph_fact_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_uniform_graph_fact_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            containment_facts,
        )
    }

    /// Accepts a simple quadratic Bezier result from uniform ownership, graph facts, and depth facts.
    pub fn from_quadratic_schedule_uniform_graph_fact_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_uniform_graph_fact_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            depth_facts,
        )
    }

    /// Accepts a simple cubic Bezier result from uniform ownership and graph facts.
    pub fn from_cubic_schedule_uniform_graph_fact_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_uniform_graph_fact_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            containment_facts,
        )
    }

    /// Accepts a simple cubic Bezier result from uniform ownership, graph facts, and depth facts.
    pub fn from_cubic_schedule_uniform_graph_fact_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_uniform_graph_fact_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            depth_facts,
        )
    }

    /// Accepts a simple rational quadratic/conic result from uniform ownership and graph facts.
    pub fn from_rational_quadratic_schedule_uniform_graph_fact_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_uniform_graph_fact_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            containment_facts,
        )
    }

    /// Accepts a simple rational quadratic/conic result from uniform ownership, graph facts, and depth facts.
    pub fn from_rational_quadratic_schedule_uniform_graph_fact_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: BezierBooleanFragmentOwnershipLocation,
        second_fragments_in_first: BezierBooleanFragmentOwnershipLocation,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_uniform_graph_fact_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            depth_facts,
        )
    }

    /// Accepts a simple linear result from locator vectors and depth facts.
    ///
    /// This is the non-uniform locator counterpart to
    /// [`Self::from_schedule_uniform_linear_identity_depth_facts`]. The
    /// constructor expands exact per-fragment locator vectors into keyed
    /// ownership facts, validates emitted operand references, generates the
    /// no-branch linear graph certificate internally, copies any
    /// resolved-overlap obligations from the schedule so overlap cases still
    /// block, accepts only identity traversal, then validates keyed nesting
    /// depths. This follows Yap, "Towards Exact Geometric Computation" (1997):
    /// every topological claim is replayable certificate data and unsupported
    /// predicates remain blockers. The staged boolean split follows Vatti
    /// (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_operand_locations_linear_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph_facts = BezierBooleanLoopGraphFacts2 {
            emitted_step_count: plan.emitted_steps.len(),
            branch_vertex_count: 0,
            resolved_overlap_count: schedule.resolved_overlap_count,
        };
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, &graph_facts);
        let traversal = graph.to_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
        Self::from_graph_walk_depth_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        )
    }

    /// Accepts a simple linear locator result with depth-certified explicit roles.
    ///
    /// This is the explicit-role counterpart to
    /// [`Self::from_schedule_operand_locations_linear_identity_depth_facts`].
    /// Exact per-fragment locator outputs are expanded into keyed ownership
    /// facts, the no-branch identity graph certificate is generated, keyed
    /// depths are validated, and supplied roles must match depth parity before
    /// the result is accepted. Yap, "Towards Exact Geometric Computation"
    /// (1997), is the contract: locator, depth, and role facts are replayable
    /// certificates, and stale role parity remains a blocker. The staged
    /// boolean model follows Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009).
    pub fn from_schedule_operand_locations_linear_identity_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph_facts = BezierBooleanLoopGraphFacts2 {
            emitted_step_count: plan.emitted_steps.len(),
            branch_vertex_count: 0,
            resolved_overlap_count: schedule.resolved_overlap_count,
        };
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, &graph_facts);
        let traversal = graph.to_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
        Self::from_graph_walk_depth_role_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
            roles,
        )
    }

    /// Accepts a simple linear result from locator vectors and containment facts.
    ///
    /// This is the containment-pair counterpart to
    /// [`Self::from_schedule_operand_locations_linear_identity_depth_facts`].
    /// The method validates non-uniform locator outputs, generates only the
    /// linear no-branch graph certificate, preserves resolved-overlap blockers,
    /// then derives nesting depths from keyed containment pairs after output
    /// loops exist. Yap (1997) supplies the exact-computation contract; Vatti
    /// (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009) supply
    /// the staged clipping/traversal/fill model.
    pub fn from_schedule_operand_locations_linear_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph_facts = BezierBooleanLoopGraphFacts2 {
            emitted_step_count: plan.emitted_steps.len(),
            branch_vertex_count: 0,
            resolved_overlap_count: schedule.resolved_overlap_count,
        };
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, &graph_facts);
        let traversal = graph.to_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
        Self::from_graph_walk_containment_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        )
    }

    /// Accepts a simple quadratic Bezier result from locator vectors using linear identity traversal.
    pub fn from_quadratic_schedule_operand_locations_linear_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_linear_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            depth_facts,
        )
    }

    /// Accepts a simple quadratic Bezier locator result with depth-certified roles.
    pub fn from_quadratic_schedule_operand_locations_linear_identity_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_linear_identity_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            depth_facts,
            roles,
        )
    }

    /// Accepts a simple quadratic Bezier result from locator vectors and containment facts.
    pub fn from_quadratic_schedule_operand_locations_linear_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_linear_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            containment_facts,
        )
    }

    /// Accepts a simple cubic Bezier result from locator vectors using linear identity traversal.
    pub fn from_cubic_schedule_operand_locations_linear_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_linear_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            depth_facts,
        )
    }

    /// Accepts a simple cubic Bezier locator result with depth-certified roles.
    pub fn from_cubic_schedule_operand_locations_linear_identity_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_linear_identity_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            depth_facts,
            roles,
        )
    }

    /// Accepts a simple cubic Bezier result from locator vectors and containment facts.
    pub fn from_cubic_schedule_operand_locations_linear_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_linear_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            containment_facts,
        )
    }

    /// Accepts a simple rational quadratic/conic result from locator vectors using linear identity traversal.
    pub fn from_rational_quadratic_schedule_operand_locations_linear_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_linear_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            depth_facts,
        )
    }

    /// Accepts a simple rational quadratic/conic locator result with depth-certified roles.
    pub fn from_rational_quadratic_schedule_operand_locations_linear_identity_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_linear_identity_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            depth_facts,
            roles,
        )
    }

    /// Accepts a simple rational quadratic/conic result from locator vectors and containment facts.
    pub fn from_rational_quadratic_schedule_operand_locations_linear_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_linear_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            containment_facts,
        )
    }

    /// Accepts a simple certified result from per-fragment locator outputs.
    ///
    /// This is the non-uniform exact-locator counterpart to
    /// [`Self::from_schedule_uniform_graph_fact_identity_depth_facts`]. A
    /// caller supplies one exact location per first fragment in the second
    /// operand and one exact location per second fragment in the first operand,
    /// plus graph and nesting-depth certificates. The locations are expanded
    /// into keyed ownership facts before selection, so missing, extra, or
    /// boundary locator outputs remain blockers rather than tolerance-derived
    /// choices. This directly follows Yap, "Towards Exact Geometric
    /// Computation" (1997). The separation of locator, traversal, and fill
    /// stages follows Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009).
    pub fn from_schedule_operand_locations_graph_fact_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
        Self::from_graph_walk_depth_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        )
    }

    /// Accepts a graph-fact identity locator result with depth-certified roles.
    ///
    /// This mirrors
    /// [`Self::from_schedule_operand_locations_graph_fact_identity_depth_facts`]
    /// but requires caller-supplied roles to agree with keyed nesting-depth
    /// parity before result acceptance.
    pub fn from_schedule_operand_locations_graph_fact_identity_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
        Self::from_graph_walk_depth_role_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
            roles,
        )
    }

    /// Accepts a simple certified result from locator vectors and containment facts.
    ///
    /// This is the containment-pair counterpart to
    /// [`Self::from_schedule_operand_locations_graph_fact_identity_depth_facts`].
    /// Exact per-fragment locator vectors are expanded into keyed ownership
    /// facts, graph facts are validated against the emitted plan, identity
    /// traversal is accepted only for certified no-branch/no-overlap graphs,
    /// and containment pairs are validated after output-loop indices exist.
    /// This keeps containment as an exact certificate producer rather than a
    /// sampling heuristic, following Yap, "Towards Exact Geometric
    /// Computation" (1997). The staged ownership/traversal/fill split follows
    /// Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_operand_locations_graph_fact_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_identity_traversal(&traversal, &plan);
        Self::from_graph_walk_containment_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        )
    }

    /// Accepts a simple quadratic Bezier result from per-fragment locator outputs.
    pub fn from_quadratic_schedule_operand_locations_graph_fact_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            depth_facts,
        )
    }

    /// Accepts a quadratic Bezier locator identity result with graph facts and depth-certified roles.
    pub fn from_quadratic_schedule_operand_locations_graph_fact_identity_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_identity_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            depth_facts,
            roles,
        )
    }

    /// Accepts a simple quadratic Bezier result from locator vectors and containment facts.
    pub fn from_quadratic_schedule_operand_locations_graph_fact_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            containment_facts,
        )
    }

    /// Accepts a simple cubic Bezier result from per-fragment locator outputs.
    pub fn from_cubic_schedule_operand_locations_graph_fact_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            depth_facts,
        )
    }

    /// Accepts a cubic Bezier locator identity result with graph facts and depth-certified roles.
    pub fn from_cubic_schedule_operand_locations_graph_fact_identity_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_identity_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            depth_facts,
            roles,
        )
    }

    /// Accepts a simple cubic Bezier result from locator vectors and containment facts.
    pub fn from_cubic_schedule_operand_locations_graph_fact_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            containment_facts,
        )
    }

    /// Accepts a simple rational quadratic/conic result from per-fragment locator outputs.
    pub fn from_rational_quadratic_schedule_operand_locations_graph_fact_identity_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_identity_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            depth_facts,
        )
    }

    /// Accepts a rational quadratic/conic locator identity result with graph facts and depth-certified roles.
    pub fn from_rational_quadratic_schedule_operand_locations_graph_fact_identity_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_identity_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            depth_facts,
            roles,
        )
    }

    /// Accepts a simple rational quadratic/conic result from locator vectors and containment facts.
    pub fn from_rational_quadratic_schedule_operand_locations_graph_fact_identity_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_identity_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            containment_facts,
        )
    }

    /// Accepts a full generic endpoint result from keyed ownership and containment facts.
    ///
    /// This constructor validates ownership facts, graph traversal facts, walk
    /// order, exact closure, and containment facts before accepting a result.
    /// Containment pairs are converted into keyed nesting depths only after
    /// output-loop packaging succeeds, preserving the predicate/construction
    /// boundary required by Yap, "Towards Exact Geometric Computation" (1997).
    /// The staged traversal/fill handoff follows Vatti (1992),
    /// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_graph_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_containment_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        )
    }

    /// Accepts a scheduled endpoint result with containment-certified explicit roles.
    ///
    /// This is the role-certified counterpart to
    /// [`Self::from_schedule_graph_walk_containment_facts`]. Ownership facts,
    /// raw graph-obligation counts, graph-walk order, containment pairs, and
    /// explicit roles are all replayed before the result is accepted. The
    /// supplied roles must match the nesting-depth parity derived from
    /// containment pairs, preserving Yap's "Towards Exact Geometric
    /// Computation" (1997) predicate/construction boundary and the staged
    /// clipping/fill split of Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009).
    pub fn from_schedule_graph_walk_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_containment_role_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
            roles,
        )
    }

    /// Accepts a scheduled endpoint result from exact multi-cycle successors and containment facts.
    ///
    /// This is the containment-fact counterpart to
    /// [`Self::from_schedule_multi_cycle_successor_depth_facts`]. It validates
    /// keyed ownership facts, emits the boolean action plan, checks graph
    /// traversal obligations, validates successor edges as one or more exact
    /// cycles, preserves those cycle ranges through endpoint closure, lowers
    /// containment facts into nesting depths, and accepts the final result only
    /// if every certificate remains valid. Yap (1997) is the exactness rule;
    /// Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009)
    /// justify the staged traversal/fill composition.
    pub fn from_schedule_multi_cycle_successor_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let multi_cycle = BezierBooleanLoopGraphMultiCycleWalkReport2::from_successor_facts(
            &traversal,
            &plan,
            successor_facts,
        );
        Self::from_multi_cycle_successor_containment_facts(
            &multi_cycle,
            &traversal,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        )
    }

    /// Accepts a scheduled multi-cycle successor result using containment-certified roles.
    pub fn from_schedule_multi_cycle_successor_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let multi_cycle = BezierBooleanLoopGraphMultiCycleWalkReport2::from_successor_facts(
            &traversal,
            &plan,
            successor_facts,
        );
        Self::from_multi_cycle_successor_containment_role_facts(
            &multi_cycle,
            &traversal,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
            roles,
        )
    }

    /// Accepts a scheduled multi-cycle successor result from replayed containment queries.
    ///
    /// This is the replay-report counterpart to
    /// [`Self::from_schedule_multi_cycle_successor_containment_facts`]. It
    /// keeps exact traversal, output-loop construction, and point/loop locator
    /// replay as separate certificates: the successor relation must close into
    /// cycle-preserving output loops before the query replay can contribute
    /// containment facts. Unresolved locator answers therefore block the final
    /// result under Yap's (1997) EGC model.
    pub fn from_schedule_multi_cycle_successor_containment_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let multi_cycle = BezierBooleanLoopGraphMultiCycleWalkReport2::from_successor_facts(
            &traversal,
            &plan,
            successor_facts,
        );
        Self::from_multi_cycle_successor_containment_query_results(
            &multi_cycle,
            &traversal,
            &plan,
            first_endpoints,
            second_endpoints,
            query_results,
        )
    }

    /// Accepts a scheduled multi-cycle successor result from an end-to-end containment certificate.
    pub fn from_schedule_multi_cycle_successor_containment_certification(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let multi_cycle = BezierBooleanLoopGraphMultiCycleWalkReport2::from_successor_facts(
            &traversal,
            &plan,
            successor_facts,
        );
        Self::from_multi_cycle_successor_containment_certification(
            &multi_cycle,
            &traversal,
            &plan,
            first_endpoints,
            second_endpoints,
            certification,
        )
    }

    /// Accepts a scheduled multi-cycle successor result from exact locator answers.
    ///
    /// This is the compact successor-based point/loop locator route to an
    /// accepted Bezier/conic boolean artifact. It derives the containment
    /// certificate from the same scheduled successor graph and exact endpoints
    /// used by result construction, preventing stale locator answers from
    /// being paired with unrelated output loops. The replay discipline follows
    /// Yap's "Towards Exact Geometric Computation" (1997).
    pub fn from_schedule_multi_cycle_successor_locator_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        let certification =
            BezierBooleanLoopContainmentCertificationReport2::from_schedule_multi_cycle_successor_query_results(
                schedule,
                operation,
                ownership_facts,
                first_endpoints,
                second_endpoints,
                branch_vertex_count,
                resolved_overlap_count,
                successor_facts,
                results,
            );
        Self::from_schedule_multi_cycle_successor_containment_certification(
            schedule,
            operation,
            ownership_facts,
            first_endpoints,
            second_endpoints,
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            &certification,
        )
    }

    /// Accepts a quadratic Bezier multi-cycle successor result with containment facts.
    ///
    /// This typed containment wrapper keeps exact fragment endpoint extraction
    /// inside `hypercurve` while preserving the same certified successor and
    /// containment replay chain used by the generic endpoint constructor.
    pub fn from_quadratic_schedule_multi_cycle_successor_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            containment_facts,
        )
    }

    /// Accepts a quadratic Bezier multi-cycle successor result with containment-certified roles.
    pub fn from_quadratic_schedule_multi_cycle_successor_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            containment_facts,
            roles,
        )
    }

    /// Accepts a quadratic Bezier multi-cycle successor result from replayed containment queries.
    pub fn from_quadratic_schedule_multi_cycle_successor_containment_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_query_results(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            query_results,
        )
    }

    /// Accepts a quadratic Bezier multi-cycle successor result from a containment certificate.
    pub fn from_quadratic_schedule_multi_cycle_successor_containment_certification(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_certification(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            certification,
        )
    }

    /// Accepts a quadratic Bezier multi-cycle successor result from locator answers.
    ///
    /// This typed wrapper keeps exact quadratic fragment endpoint extraction
    /// inside the boolean API while preserving the same Yap (1997) replay
    /// discipline as [`Self::from_schedule_multi_cycle_successor_locator_results`].
    pub fn from_quadratic_schedule_multi_cycle_successor_locator_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_locator_results(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            results,
        )
    }

    /// Accepts a cubic Bezier multi-cycle successor result from locator answers.
    pub fn from_cubic_schedule_multi_cycle_successor_locator_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_locator_results(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            results,
        )
    }

    /// Accepts a rational quadratic/conic multi-cycle successor result from locator answers.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_locator_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_locator_results(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            results,
        )
    }

    /// Accepts a cubic Bezier multi-cycle successor result with containment facts.
    pub fn from_cubic_schedule_multi_cycle_successor_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            containment_facts,
        )
    }

    /// Accepts a cubic Bezier multi-cycle successor result with containment-certified roles.
    pub fn from_cubic_schedule_multi_cycle_successor_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            containment_facts,
            roles,
        )
    }

    /// Accepts a cubic Bezier multi-cycle successor result from replayed containment queries.
    pub fn from_cubic_schedule_multi_cycle_successor_containment_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_query_results(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            query_results,
        )
    }

    /// Accepts a cubic Bezier multi-cycle successor result from a containment certificate.
    pub fn from_cubic_schedule_multi_cycle_successor_containment_certification(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_certification(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            certification,
        )
    }

    /// Accepts a rational quadratic/conic multi-cycle successor result with containment facts.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            containment_facts,
        )
    }

    /// Accepts a rational quadratic/conic multi-cycle successor result with containment-certified roles.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            containment_facts,
            roles,
        )
    }

    /// Accepts a rational quadratic/conic multi-cycle successor result from replayed containment queries.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_containment_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_query_results(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            query_results,
        )
    }

    /// Accepts a rational quadratic/conic multi-cycle successor result from a containment certificate.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_containment_certification(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_containment_certification(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            certification,
        )
    }

    /// Accepts a full generic endpoint result using keyed graph and containment facts.
    ///
    /// This is the strict containment counterpart to
    /// [`Self::from_schedule_graph_fact_walk_depth_facts`]. The graph fact is
    /// keyed to the emitted plan and containment facts are keyed to output-loop
    /// indices, so stale topology certificates cannot be silently reused.
    pub fn from_schedule_graph_fact_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_certified_walk_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_containment_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        )
    }

    /// Accepts a scheduled endpoint result with keyed graph facts and containment-certified roles.
    pub fn from_schedule_graph_fact_walk_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_certified_walk_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_containment_role_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
            roles,
        )
    }

    /// Accepts a full quadratic Bezier result using keyed containment facts.
    pub fn from_quadratic_schedule_graph_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_graph_walk_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a quadratic Bezier result using containment facts and explicit roles.
    pub fn from_quadratic_schedule_graph_walk_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_walk_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
            roles,
        )
    }

    /// Accepts a full quadratic Bezier result using keyed graph and containment facts.
    pub fn from_quadratic_schedule_graph_fact_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a quadratic Bezier result using keyed graph facts, containment facts, and explicit roles.
    pub fn from_quadratic_schedule_graph_fact_walk_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
            roles,
        )
    }

    /// Accepts a full cubic Bezier result using keyed containment facts.
    pub fn from_cubic_schedule_graph_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_graph_walk_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a cubic Bezier result using containment facts and explicit roles.
    pub fn from_cubic_schedule_graph_walk_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_walk_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
            roles,
        )
    }

    /// Accepts a full cubic Bezier result using keyed graph and containment facts.
    pub fn from_cubic_schedule_graph_fact_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a cubic Bezier result using keyed graph facts, containment facts, and explicit roles.
    pub fn from_cubic_schedule_graph_fact_walk_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
            roles,
        )
    }

    /// Accepts a full rational quadratic/conic result using keyed containment facts.
    pub fn from_rational_quadratic_schedule_graph_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_graph_walk_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a rational quadratic/conic result using containment facts and explicit roles.
    pub fn from_rational_quadratic_schedule_graph_walk_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_walk_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
            roles,
        )
    }

    /// Accepts a full rational quadratic/conic result using keyed graph and containment facts.
    pub fn from_rational_quadratic_schedule_graph_fact_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a rational quadratic/conic result using keyed graph facts, containment facts, and explicit roles.
    pub fn from_rational_quadratic_schedule_graph_fact_walk_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
            roles,
        )
    }

    /// Accepts output loops plus certified containment facts as a boolean artifact.
    ///
    /// Containment pairs are first validated and converted into keyed
    /// nesting-depth facts, then the normal result acceptance path is used.
    /// This keeps containment generation as a separate exact predicate stage
    /// while letting callers avoid manually materializing depth facts.
    pub fn from_output_loop_containment_facts(
        output: &BezierBooleanOutputLoopReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_output_loop_containment_facts(
            output,
            containment_facts,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts output loops when roles match containment-derived depths.
    ///
    /// This is the result-level counterpart to
    /// [`BezierBooleanRegionAssemblyReport2::from_output_loop_containment_role_facts`].
    /// It lets callers carry explicit role certificates through containment
    /// result APIs without bypassing the exact parity evidence derived from
    /// validated containment pairs.
    pub fn from_output_loop_containment_role_facts(
        output: &BezierBooleanOutputLoopReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_output_loop_containment_role_facts(
            output,
            containment_facts,
            roles,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified endpoint result with depth-certified roles.
    ///
    /// This preserves the same result-boundary composition as
    /// [`Self::from_graph_walk_depth_facts`] while requiring caller-supplied
    /// material/hole role facts to agree with keyed nesting-depth parity before
    /// the higher-order boolean artifact is accepted.
    pub fn from_graph_walk_depth_role_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_graph_walk_depth_role_facts(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
            roles,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified generic endpoint result with containment facts.
    pub fn from_graph_walk_containment_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_graph_walk_containment_facts(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified endpoint result from replayed containment queries.
    ///
    /// This constructor closes the exact point/loop locator path for current
    /// higher-order boolean output. The supplied
    /// [`BezierBooleanLoopContainmentQueryResultReport2`] must be ready and
    /// keyed to the same boolean operation; otherwise the result is blocked
    /// before containment-derived depths can assign material/hole roles. This
    /// directly implements Yap's (1997) rule that predicate uncertainty is not
    /// construction evidence. The graph/containment/result split follows
    /// Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_graph_walk_containment_query_results(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_graph_walk_containment_query_results(
                walk,
                plan,
                first_endpoints,
                second_endpoints,
                query_results,
            );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified endpoint result from an end-to-end containment certificate.
    pub fn from_graph_walk_containment_certification(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_graph_walk_containment_certification(
                walk,
                plan,
                first_endpoints,
                second_endpoints,
                certification,
            );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a scheduled endpoint result from replayed containment queries.
    ///
    /// Ownership facts, graph obligations, graph-walk order, exact closure, and
    /// point/loop containment-query replay are all validated before a result is
    /// accepted. Boundary or unknown locator answers therefore block exactly
    /// like stale graph or role certificates instead of collapsing into an
    /// empty containment set. This is the Yap (1997) predicate replay contract
    /// applied to the Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009) clipping/fill pipeline.
    pub fn from_schedule_graph_walk_containment_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_containment_query_results(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            query_results,
        )
    }

    /// Accepts a scheduled endpoint result from an end-to-end containment certificate.
    ///
    /// The certificate must have been built from the output loops produced by
    /// the same ownership, graph-walk, and endpoint data. Downstream depth
    /// validation still checks loop keys, so stale or mismatched certificates
    /// block rather than being repaired. This makes the exact point/loop
    /// locator result a first-class boolean input under Yap's (1997) model.
    pub fn from_schedule_graph_walk_containment_certification(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_containment_certification(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            certification,
        )
    }

    /// Accepts a scheduled endpoint result from exact locator answers.
    ///
    /// This constructor is the most direct current point/loop locator route to
    /// an accepted Bezier/conic boolean artifact. It first builds
    /// [`BezierBooleanLoopContainmentCertificationReport2`] from the scheduled
    /// graph-walk output loops and supplied locator answers, then consumes that
    /// certificate through the normal result path. The caller therefore cannot
    /// pair a detached query report with a different output-loop package, and
    /// unresolved locator answers remain blockers. This is Yap's (1997)
    /// predicate replay rule applied to the Vatti (1992), Greiner-Hormann
    /// (1998), and Martinez-Rueda-Feito (2009) clipping/fill pipeline.
    pub fn from_schedule_graph_walk_locator_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        let certification =
            BezierBooleanLoopContainmentCertificationReport2::from_schedule_graph_walk_query_results(
                schedule,
                operation,
                ownership_facts,
                first_endpoints,
                second_endpoints,
                branch_vertex_count,
                resolved_overlap_count,
                walk_indices,
                results,
            );
        Self::from_schedule_graph_walk_containment_certification(
            schedule,
            operation,
            ownership_facts,
            first_endpoints,
            second_endpoints,
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            &certification,
        )
    }

    /// Accepts a graph-walk-certified endpoint result with containment-certified roles.
    pub fn from_graph_walk_containment_role_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_graph_walk_containment_role_facts(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
            roles,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a multi-cycle-successor-certified result using containment facts.
    ///
    /// This is the result-level counterpart to
    /// [`BezierBooleanRegionAssemblyReport2::from_multi_cycle_successor_containment_facts`].
    /// The exact successor certificate, endpoint closure, containment
    /// certificates, role derivation, and final result acceptance are composed
    /// without flattening cycle topology into an unkeyed walk vector.
    pub fn from_multi_cycle_successor_containment_facts(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_multi_cycle_successor_containment_facts(
                multi_cycle,
                traversal,
                plan,
                first_endpoints,
                second_endpoints,
                containment_facts,
            );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a multi-cycle-successor-certified result using containment-certified roles.
    pub fn from_multi_cycle_successor_containment_role_facts(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_multi_cycle_successor_containment_role_facts(
                multi_cycle,
                traversal,
                plan,
                first_endpoints,
                second_endpoints,
                containment_facts,
                roles,
            );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a multi-cycle-successor-certified result from replayed containment queries.
    ///
    /// This is the multi-cycle successor counterpart to
    /// [`Self::from_graph_walk_containment_query_results`]. The exact
    /// successor certificate and endpoint closure define the output loops;
    /// the replay report may then contribute containment facts only if it is
    /// ready for the same operation. This preserves Yap's (1997) proof
    /// boundary for point/loop locator answers.
    pub fn from_multi_cycle_successor_containment_query_results(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_multi_cycle_successor_containment_query_results(
                multi_cycle,
                traversal,
                plan,
                first_endpoints,
                second_endpoints,
                query_results,
            );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a multi-cycle-successor-certified result from an end-to-end containment certificate.
    pub fn from_multi_cycle_successor_containment_certification(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_multi_cycle_successor_containment_certification(
                multi_cycle,
                traversal,
                plan,
                first_endpoints,
                second_endpoints,
                certification,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a multi-cycle-successor result from exact locator answers.
    ///
    /// This is the prebuilt-certificate counterpart to
    /// [`Self::from_schedule_multi_cycle_successor_locator_results`]. The
    /// locator answers are replayed through output loops derived from the
    /// same multi-cycle successor certificate, so boundary/unknown/stale
    /// answers remain blockers instead of becoming inferred containment.
    pub fn from_multi_cycle_successor_locator_results(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        let certification =
            BezierBooleanLoopContainmentCertificationReport2::from_multi_cycle_successor_query_results(
                multi_cycle,
                traversal,
                plan,
                first_endpoints,
                second_endpoints,
                results,
            );
        Self::from_multi_cycle_successor_containment_certification(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
            &certification,
        )
    }

    /// Accepts a full generic endpoint result using a keyed graph-facts certificate.
    ///
    /// This is the strictest report-only result constructor: ownership facts
    /// are keyed to the traversal schedule, graph facts are keyed to the
    /// emitted plan, graph-walk indices are validated as a complete
    /// permutation, and nesting depths are keyed to the output loops. The
    /// constructor composes certificates rather than producing them, preserving
    /// Yap's exact-computation contract that unsupported predicates remain
    /// explicit blockers.
    pub fn from_schedule_graph_fact_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_certified_walk_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_depth_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        )
    }

    /// Accepts a full generic endpoint result using keyed graph facts and depth-certified roles.
    ///
    /// This is the explicit-role counterpart to
    /// [`Self::from_schedule_graph_fact_walk_depth_facts`]. Ownership, graph,
    /// walk, depth, and role facts are all supplied as replayable certificate
    /// data and are validated in order before the result is accepted. The
    /// graph certificate may describe branch or resolved-overlap obligations
    /// because the caller also supplies an explicit graph walk. Yap, "Towards
    /// Exact Geometric Computation" (1997), is the acceptance rule: stale role
    /// parity is a blocker, not a topology hint to repair. The stage split
    /// follows Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009).
    pub fn from_schedule_graph_fact_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_certified_walk_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_depth_role_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full generic endpoint result from locator vectors and graph-walk facts.
    ///
    /// This is the non-uniform locator counterpart to
    /// [`Self::from_schedule_graph_fact_walk_depth_facts`]. It composes exact
    /// per-fragment locator outputs, keyed graph facts, an explicit graph-walk
    /// permutation, and keyed nesting depths into the result boundary. Unlike
    /// the identity-walk constructors, this path can accept a certified
    /// reordering for branch vertices or resolved overlaps once an exact graph
    /// walker supplies that permutation. Yap, "Towards Exact Geometric
    /// Computation" (1997), is the rule: the constructor validates replayable
    /// facts and never infers missing topology from vector order. The staged
    /// boolean model follows Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009).
    pub fn from_schedule_operand_locations_graph_fact_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_certified_walk_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_depth_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        )
    }

    /// Accepts a full locator-vector result using keyed graph facts and depth-certified roles.
    ///
    /// This is the non-uniform locator counterpart to
    /// [`Self::from_schedule_graph_fact_walk_depth_role_facts`]. The caller
    /// supplies exact locator outputs, keyed graph facts, an explicit graph
    /// walk, keyed depths, and explicit roles. All facts are replayed before
    /// acceptance; stale role parity blocks the result under Yap's exact
    /// predicate/construction contract.
    pub fn from_schedule_operand_locations_graph_fact_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_certified_walk_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_depth_role_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full generic endpoint result from locator vectors and raw graph counts.
    ///
    /// This is the raw-count counterpart to
    /// [`Self::from_schedule_operand_locations_graph_fact_walk_depth_facts`].
    /// It is useful when a graph walker can provide audited branch/resolved
    /// overlap counts and a complete walk permutation without first packaging
    /// those counts as [`BezierBooleanLoopGraphFacts2`]. Nonzero graph
    /// obligations are accepted only because this constructor also validates
    /// the explicit walk permutation; identity constructors still block them.
    /// This preserves Yap's exact-computation contract that graph topology is
    /// certified data, not inferred order. The traversal/fill split follows
    /// Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009);
    /// degenerate overlap policy follows Foster-Hormann-Popa (2019).
    pub fn from_schedule_operand_locations_graph_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_depth_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        )
    }

    /// Accepts a full locator-vector result using raw graph counts and depth-certified roles.
    ///
    /// This raw-count variant accepts audited branch/resolved-overlap counts
    /// beside an explicit walk permutation and then validates keyed depths plus
    /// role parity. It is a certificate-composition API, not a graph walker.
    pub fn from_schedule_operand_locations_graph_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_depth_role_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full generic endpoint result from locator vectors and containment facts.
    ///
    /// This is the containment-pair counterpart to
    /// [`Self::from_schedule_operand_locations_graph_fact_walk_depth_facts`].
    /// It validates non-uniform locator vectors, keyed graph facts, an explicit
    /// graph-walk permutation, exact endpoint closure, and containment pairs
    /// before accepting the higher-order boolean artifact. This is the compact
    /// certificate-composition entry point expected from future exact
    /// point/loop locators and branch/overlap graph walkers. Yap, "Towards
    /// Exact Geometric Computation" (1997), is the acceptance contract; the
    /// clipping-stage separation follows Vatti (1992), Greiner-Hormann (1998),
    /// and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_operand_locations_graph_fact_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let graph = BezierBooleanLoopGraphFactReport2::from_plan_facts(&plan, graph_facts);
        let traversal = graph.to_certified_walk_traversal_report(&plan);
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_containment_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        )
    }

    /// Accepts a full generic endpoint result from locator vectors, raw graph counts, and containment facts.
    ///
    /// This is the containment-pair counterpart to
    /// [`Self::from_schedule_operand_locations_graph_walk_depth_facts`]. The
    /// caller supplies exact locator vectors, raw graph-obligation counts, a
    /// complete walk permutation, and keyed containment facts. Containment is
    /// validated only after closed output-loop indices exist, preserving Yap's
    /// predicate/construction boundary. The staged boolean model follows Vatti
    /// (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_operand_locations_graph_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let facts = BezierBooleanOwnershipFactReport2::from_operand_locations(
            schedule,
            first_fragments_in_second,
            second_fragments_in_first,
        );
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_containment_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        )
    }

    /// Accepts a full quadratic Bezier result using a keyed graph-facts certificate.
    pub fn from_quadratic_schedule_graph_fact_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_depth_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a full quadratic Bezier result using keyed graph facts and depth-certified roles.
    pub fn from_quadratic_schedule_graph_fact_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_depth_role_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full quadratic Bezier result from locator vectors and graph-walk facts.
    pub fn from_quadratic_schedule_operand_locations_graph_fact_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_walk_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a quadratic Bezier locator graph-walk result with keyed graph facts and depth-certified roles.
    pub fn from_quadratic_schedule_operand_locations_graph_fact_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_walk_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full quadratic Bezier result from locator vectors and raw graph counts.
    pub fn from_quadratic_schedule_operand_locations_graph_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_walk_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a quadratic Bezier locator graph-walk result with raw graph counts and depth-certified roles.
    pub fn from_quadratic_schedule_operand_locations_graph_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_walk_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full quadratic Bezier result from locator vectors and containment facts.
    pub fn from_quadratic_schedule_operand_locations_graph_fact_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_walk_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a full quadratic Bezier result from locator vectors, raw graph counts, and containment facts.
    pub fn from_quadratic_schedule_operand_locations_graph_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_walk_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a full cubic Bezier result using a keyed graph-facts certificate.
    pub fn from_cubic_schedule_graph_fact_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_depth_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a full cubic Bezier result using keyed graph facts and depth-certified roles.
    pub fn from_cubic_schedule_graph_fact_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_depth_role_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full cubic Bezier result from locator vectors and graph-walk facts.
    pub fn from_cubic_schedule_operand_locations_graph_fact_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_walk_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a cubic Bezier locator graph-walk result with keyed graph facts and depth-certified roles.
    pub fn from_cubic_schedule_operand_locations_graph_fact_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_walk_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full cubic Bezier result from locator vectors and raw graph counts.
    pub fn from_cubic_schedule_operand_locations_graph_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_walk_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a cubic Bezier locator graph-walk result with raw graph counts and depth-certified roles.
    pub fn from_cubic_schedule_operand_locations_graph_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_walk_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full cubic Bezier result from locator vectors and containment facts.
    pub fn from_cubic_schedule_operand_locations_graph_fact_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_walk_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a full cubic Bezier result from locator vectors, raw graph counts, and containment facts.
    pub fn from_cubic_schedule_operand_locations_graph_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_walk_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a full rational quadratic/conic result using keyed graph facts.
    pub fn from_rational_quadratic_schedule_graph_fact_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_depth_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a full rational quadratic/conic result using keyed graph facts and depth-certified roles.
    pub fn from_rational_quadratic_schedule_graph_fact_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_depth_role_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full rational quadratic/conic result from locator vectors and graph-walk facts.
    pub fn from_rational_quadratic_schedule_operand_locations_graph_fact_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_walk_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a rational quadratic/conic locator graph-walk result with keyed graph facts and depth-certified roles.
    pub fn from_rational_quadratic_schedule_operand_locations_graph_fact_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_walk_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full rational quadratic/conic result from locator vectors and raw graph counts.
    pub fn from_rational_quadratic_schedule_operand_locations_graph_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_walk_depth_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a rational quadratic/conic locator graph-walk result with raw graph counts and depth-certified roles.
    pub fn from_rational_quadratic_schedule_operand_locations_graph_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_walk_depth_role_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full rational quadratic/conic result from locator vectors and containment facts.
    pub fn from_rational_quadratic_schedule_operand_locations_graph_fact_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_fact_walk_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a full rational quadratic/conic result from locator vectors, raw graph counts, and containment facts.
    pub fn from_rational_quadratic_schedule_operand_locations_graph_walk_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        first_fragments_in_second: &[BezierBooleanFragmentOwnershipLocation],
        second_fragments_in_first: &[BezierBooleanFragmentOwnershipLocation],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_operand_locations_graph_walk_containment_facts(
            schedule,
            operation,
            first_fragments_in_second,
            second_fragments_in_first,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
        )
    }

    /// Accepts a full generic endpoint result from keyed ownership and depth facts.
    ///
    /// This constructor is a convenience composition for the certified Bezier
    /// boolean stack. It validates keyed opposite-operand ownership facts,
    /// applies the boolean operation action table, audits emitted references,
    /// validates graph-traversal preconditions, validates the supplied graph
    /// walk, closes exact endpoints, validates keyed nesting-depth facts,
    /// assigns material/hole roles, assembles the higher-order region artifact,
    /// and finally accepts the result. It intentionally does **not** compute
    /// ownership, graph traversal, or containment itself. Those remain separate
    /// exact predicates/certificates, following Yap, "Towards Exact Geometric
    /// Computation" (1997). The staged composition follows the boundary,
    /// traversal, and fill-phase separation in Vatti (1992),
    /// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_graph_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_depth_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        )
    }

    /// Accepts a full scheduled result from exact multi-cycle successor facts.
    ///
    /// This is the successor-fact counterpart to
    /// [`Self::from_schedule_graph_walk_depth_facts`]. It validates keyed
    /// ownership facts, emits the boolean action plan, checks graph-traversal
    /// obligations, derives a deterministic multi-cycle walk from successor
    /// edges, verifies endpoint closure without merging or splitting those
    /// cycles, validates keyed depth facts, and only then accepts the final
    /// result. The method deliberately composes certificates rather than
    /// computing predicates. That follows Yap, "Towards Exact Geometric
    /// Computation" (1997), while the boundary traversal/fill split follows
    /// Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_multi_cycle_successor_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let multi_cycle = BezierBooleanLoopGraphMultiCycleWalkReport2::from_successor_facts(
            &traversal,
            &plan,
            successor_facts,
        );
        Self::from_multi_cycle_successor_depth_facts(
            &multi_cycle,
            &traversal,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        )
    }

    /// Accepts a scheduled multi-cycle successor result with depth-certified roles.
    pub fn from_schedule_multi_cycle_successor_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let multi_cycle = BezierBooleanLoopGraphMultiCycleWalkReport2::from_successor_facts(
            &traversal,
            &plan,
            successor_facts,
        );
        Self::from_multi_cycle_successor_depth_role_facts(
            &multi_cycle,
            &traversal,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
            roles,
        )
    }

    /// Accepts a quadratic Bezier multi-cycle successor result with keyed depths.
    ///
    /// The quadratic fragment reports only provide exact endpoint carriers;
    /// ownership, successor, and depth facts remain external certificates
    /// replayed under Yap's (1997) exact-geometric-computation boundary.
    pub fn from_quadratic_schedule_multi_cycle_successor_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_depth_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            depth_facts,
        )
    }

    /// Accepts a quadratic Bezier multi-cycle successor result with depth-certified roles.
    pub fn from_quadratic_schedule_multi_cycle_successor_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_depth_role_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            depth_facts,
            roles,
        )
    }

    /// Accepts a cubic Bezier multi-cycle successor result with keyed depths.
    pub fn from_cubic_schedule_multi_cycle_successor_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_depth_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            depth_facts,
        )
    }

    /// Accepts a cubic Bezier multi-cycle successor result with depth-certified roles.
    pub fn from_cubic_schedule_multi_cycle_successor_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_depth_role_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            depth_facts,
            roles,
        )
    }

    /// Accepts a rational quadratic/conic multi-cycle successor result with keyed depths.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_depth_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            depth_facts,
        )
    }

    /// Accepts a rational quadratic/conic multi-cycle successor result with depth-certified roles.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_depth_role_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full generic endpoint result from keyed ownership and depth-certified roles.
    ///
    /// This is the raw graph-count counterpart to
    /// [`Self::from_schedule_graph_fact_walk_depth_role_facts`]. It is useful
    /// when the exact graph walker reports branch and overlap counts directly
    /// beside an explicit walk permutation. The constructor still validates
    /// ownership keys, emitted references, graph-walk permutation, exact
    /// endpoint closure, keyed depths, and explicit role parity before result
    /// acceptance.
    pub fn from_schedule_graph_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let facts =
            BezierBooleanOwnershipFactReport2::from_schedule_facts(schedule, ownership_facts);
        let ownership = facts.classify(schedule, operation);
        let emission = BezierBooleanEmissionPlanReport2::from_ownership(&ownership);
        let assembly = BezierBooleanAssemblyReadinessReport2::from_fragment_counts(
            &emission,
            first_endpoints.len(),
            second_endpoints.len(),
        );
        let plan =
            BezierBooleanLoopAssemblyPlanReport2::from_assembly_readiness(&assembly, &emission);
        let traversal = BezierBooleanLoopGraphTraversalReport2::from_certified_walk_graph_facts(
            &plan,
            branch_vertex_count,
            resolved_overlap_count,
        );
        let walk = BezierBooleanLoopGraphWalkReport2::from_traversal_order(
            &traversal,
            &plan,
            walk_indices,
        );
        Self::from_graph_walk_depth_role_facts(
            &walk,
            &plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full quadratic Bezier result from keyed ownership and depth facts.
    ///
    /// The quadratic fragments provide exact endpoint carriers and fragment
    /// counts; ownership facts, graph facts, graph walk order, and nesting
    /// depths remain external certificates that are validated before result
    /// acceptance.
    pub fn from_quadratic_schedule_graph_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_graph_walk_depth_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a full quadratic Bezier result from keyed ownership and depth-certified roles.
    pub fn from_quadratic_schedule_graph_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_walk_depth_role_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full cubic Bezier result from keyed ownership and depth facts.
    pub fn from_cubic_schedule_graph_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_graph_walk_depth_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a full cubic Bezier result from keyed ownership and depth-certified roles.
    pub fn from_cubic_schedule_graph_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_walk_depth_role_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts a full rational quadratic/conic result from keyed ownership and depth facts.
    pub fn from_rational_quadratic_schedule_graph_walk_depth_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        Self::from_schedule_graph_walk_depth_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
        )
    }

    /// Accepts a full rational quadratic/conic result from keyed ownership and depth-certified roles.
    pub fn from_rational_quadratic_schedule_graph_walk_depth_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_walk_depth_role_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            depth_facts,
            roles,
        )
    }

    /// Accepts output loops plus keyed nesting-depth facts as a boolean artifact.
    ///
    /// This is the result-level counterpart to
    /// [`BezierBooleanRegionAssemblyReport2::from_output_loop_depth_facts`].
    /// It composes keyed nesting-depth validation, parity role generation,
    /// role assignment, region assembly, and final result acceptance without
    /// allowing callers to bypass an intermediate blocker. This mirrors the
    /// stage separation used by Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009). Yap, "Towards Exact Geometric Computation"
    /// (1997), is the acceptance rule: exact output is admitted only when every
    /// combinatorial certificate remains valid at the result boundary.
    pub fn from_output_loop_depth_facts(
        output: &BezierBooleanOutputLoopReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_output_loop_depth_facts(output, depth_facts);
        Self::from_region_assembly(&assembly)
    }

    /// Accepts output loops when supplied roles are certified by keyed depths.
    ///
    /// This is the result-level counterpart to
    /// [`BezierBooleanLoopRoleAssignmentReport2::from_output_loop_depth_role_facts`].
    /// It lets callers carry explicit material/hole role facts through the
    /// boolean result API without bypassing the exact nesting-depth parity
    /// certificate required by Yap's predicate/construction discipline.
    pub fn from_output_loop_depth_role_facts(
        output: &BezierBooleanOutputLoopReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_output_loop_depth_role_facts(
            output,
            depth_facts,
            roles,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified generic endpoint result with keyed depths.
    ///
    /// A ready graph walk supplies the exact emitted order for closure, keyed
    /// nesting-depth facts supply material/hole parity, and every malformed
    /// walk, stale reference, or invalid depth fact is retained as a blocker.
    pub fn from_graph_walk_depth_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_graph_walk_depth_facts(
            walk,
            plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a multi-cycle-successor-certified generic endpoint result with keyed depths.
    ///
    /// This composes the exact successor graph certificate directly into the
    /// result boundary. The successor relation must be a validated
    /// multi-cycle permutation, endpoint closure must preserve those cycle
    /// ranges, and the supplied nesting depths must be keyed to the resulting
    /// output loops. No orientation, tolerance, or sample-point inference is
    /// used. That is the Yap (1997) EGC contract applied at the final boolean
    /// result handoff; Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009) provide the staged traversal/fill model.
    pub fn from_multi_cycle_successor_depth_facts(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_multi_cycle_successor_depth_facts(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
            depth_facts,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a multi-cycle-successor-certified result with depth-certified roles.
    pub fn from_multi_cycle_successor_depth_role_facts(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_multi_cycle_successor_depth_role_facts(
                multi_cycle,
                traversal,
                plan,
                first_endpoints,
                second_endpoints,
                depth_facts,
                roles,
            );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified quadratic Bezier result with keyed depths.
    pub fn from_quadratic_graph_walk_depth_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_quadratic_graph_walk_depth_facts(
            walk,
            plan,
            first,
            second,
            depth_facts,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified quadratic Bezier result with depth-certified roles.
    pub fn from_quadratic_graph_walk_depth_role_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_quadratic_graph_walk_depth_role_facts(
                walk,
                plan,
                first,
                second,
                depth_facts,
                roles,
            );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified cubic Bezier result with keyed depths.
    pub fn from_cubic_graph_walk_depth_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_cubic_graph_walk_depth_facts(
            walk,
            plan,
            first,
            second,
            depth_facts,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified cubic Bezier result with depth-certified roles.
    pub fn from_cubic_graph_walk_depth_role_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let assembly = BezierBooleanRegionAssemblyReport2::from_cubic_graph_walk_depth_role_facts(
            walk,
            plan,
            first,
            second,
            depth_facts,
            roles,
        );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified rational quadratic result with keyed depths.
    pub fn from_rational_quadratic_graph_walk_depth_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_rational_quadratic_graph_walk_depth_facts(
                walk,
                plan,
                first,
                second,
                depth_facts,
            );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a graph-walk-certified rational quadratic result with depth-certified roles.
    pub fn from_rational_quadratic_graph_walk_depth_role_facts(
        walk: &BezierBooleanLoopGraphWalkReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        depth_facts: &[BezierBooleanLoopNestingDepthFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let assembly =
            BezierBooleanRegionAssemblyReport2::from_rational_quadratic_graph_walk_depth_role_facts(
                walk,
                plan,
                first,
                second,
                depth_facts,
                roles,
            );
        Self::from_region_assembly(&assembly)
    }

    /// Accepts a ready higher-order Bezier/conic boolean artifact.
    pub fn from_region_assembly(assembly: &BezierBooleanRegionAssemblyReport2) -> Self {
        match assembly.status {
            BezierBooleanRegionAssemblyStatus::Empty => {
                return Self::empty_like(BezierBooleanResultStatus::Empty, assembly, 0);
            }
            BezierBooleanRegionAssemblyStatus::NoInteriorSplits => {
                return Self::empty_like(BezierBooleanResultStatus::NoInteriorSplits, assembly, 0);
            }
            BezierBooleanRegionAssemblyStatus::RoleAssignmentBlocked
            | BezierBooleanRegionAssemblyStatus::HoleWithoutMaterial => {
                return Self::empty_like(
                    BezierBooleanResultStatus::RegionAssemblyBlocked,
                    assembly,
                    assembly.blocker_count.max(1),
                );
            }
            BezierBooleanRegionAssemblyStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanResultStatus::NoEmittedFragments,
                    assembly,
                    0,
                );
            }
            BezierBooleanRegionAssemblyStatus::Ready => {}
        }

        Self {
            status: BezierBooleanResultStatus::Ready,
            assembly_status: assembly.status,
            operation: assembly.operation,
            directed_fragments: assembly.directed_fragments.clone(),
            assigned_loops: assembly.assigned_loops.clone(),
            material_loop_indices: assembly.material_loop_indices.clone(),
            hole_loop_indices: assembly.hole_loop_indices.clone(),
            assigned_loop_count: assembly.assigned_loop_count,
            material_loop_count: assembly.material_loop_count,
            hole_loop_count: assembly.hole_loop_count,
            directed_fragment_count: assembly.directed_fragments.len(),
            blocker_count: 0,
        }
    }

    fn empty_like(
        status: BezierBooleanResultStatus,
        assembly: &BezierBooleanRegionAssemblyReport2,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            assembly_status: assembly.status,
            operation: assembly.operation,
            directed_fragments: Vec::new(),
            assigned_loops: Vec::new(),
            material_loop_indices: Vec::new(),
            hole_loop_indices: Vec::new(),
            assigned_loop_count: 0,
            material_loop_count: 0,
            hole_loop_count: 0,
            directed_fragment_count: 0,
            blocker_count,
        }
    }

    /// Returns true when a higher-order Bezier/conic boolean artifact is accepted.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanResultStatus::Ready
    }

    /// Returns true when any prerequisite prevents accepted boolean output.
    pub fn has_blockers(&self) -> bool {
        self.status == BezierBooleanResultStatus::RegionAssemblyBlocked
    }
}

impl BezierBooleanMaterializationAuditReport2 {
    /// Audits whether an accepted Bezier/conic boolean artifact is materialization-ready.
    ///
    /// The audit replays only already-certified structure. It does not compute
    /// containment, orient loops, sample points, or repair stale indices. This
    /// is the Yap (1997) exact-geometric-computation contract applied to the
    /// final handoff boundary: future concrete region materialization may
    /// consume this report only when every retained count, role index, and
    /// output-loop range is self-consistent. The staged interpretation of
    /// boundary loops and material/hole roles follows Vatti (1992),
    /// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_result(result: &BezierBooleanResultReport2) -> Self {
        match result.status {
            BezierBooleanResultStatus::Empty => {
                return Self::empty_like(BezierBooleanMaterializationAuditStatus::Empty, result, 0);
            }
            BezierBooleanResultStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanMaterializationAuditStatus::NoInteriorSplits,
                    result,
                    0,
                );
            }
            BezierBooleanResultStatus::RegionAssemblyBlocked => {
                return Self::empty_like(
                    BezierBooleanMaterializationAuditStatus::ResultBlocked,
                    result,
                    result.blocker_count.max(1),
                );
            }
            BezierBooleanResultStatus::NoEmittedFragments => {
                return Self::empty_like(
                    BezierBooleanMaterializationAuditStatus::ResultBlocked,
                    result,
                    0,
                );
            }
            BezierBooleanResultStatus::Ready => {}
        }

        let count_mismatch_count =
            usize::from(result.assigned_loop_count != result.assigned_loops.len())
                + usize::from(result.material_loop_count != result.material_loop_indices.len())
                + usize::from(result.hole_loop_count != result.hole_loop_indices.len())
                + usize::from(result.directed_fragment_count != result.directed_fragments.len());

        let mut seen_roles = vec![false; result.assigned_loops.len()];
        let mut role_index_mismatch_count = 0;
        let mut loop_role_mismatch_count = 0;
        for index in result.material_loop_indices.iter().copied() {
            if index >= result.assigned_loops.len() || seen_roles[index] {
                role_index_mismatch_count += 1;
                continue;
            }
            seen_roles[index] = true;
            if result.assigned_loops[index].role != BezierBooleanOutputLoopRole::Material {
                loop_role_mismatch_count += 1;
            }
        }
        for index in result.hole_loop_indices.iter().copied() {
            if index >= result.assigned_loops.len() || seen_roles[index] {
                role_index_mismatch_count += 1;
                continue;
            }
            seen_roles[index] = true;
            if result.assigned_loops[index].role != BezierBooleanOutputLoopRole::Hole {
                loop_role_mismatch_count += 1;
            }
        }
        role_index_mismatch_count += seen_roles.iter().filter(|seen| !**seen).count();

        let fragment_range_mismatch_count = result
            .assigned_loops
            .iter()
            .filter(|assigned| {
                let start = assigned.output_loop.first_directed_fragment_index;
                let count = assigned.output_loop.directed_fragment_count;
                count == 0
                    || start
                        .checked_add(count)
                        .map_or(true, |end| end > result.directed_fragments.len())
            })
            .count();

        let blocker_count = count_mismatch_count
            + role_index_mismatch_count
            + loop_role_mismatch_count
            + fragment_range_mismatch_count;
        let status = if count_mismatch_count > 0 {
            BezierBooleanMaterializationAuditStatus::CountMismatch
        } else if role_index_mismatch_count > 0 {
            BezierBooleanMaterializationAuditStatus::RoleIndexMismatch
        } else if loop_role_mismatch_count > 0 {
            BezierBooleanMaterializationAuditStatus::LoopRoleMismatch
        } else if fragment_range_mismatch_count > 0 {
            BezierBooleanMaterializationAuditStatus::FragmentRangeMismatch
        } else {
            BezierBooleanMaterializationAuditStatus::Ready
        };

        Self {
            status,
            result_status: result.status,
            operation: result.operation,
            assigned_loop_count: result.assigned_loops.len(),
            material_loop_count: result.material_loop_indices.len(),
            hole_loop_count: result.hole_loop_indices.len(),
            directed_fragment_count: result.directed_fragments.len(),
            audited_loop_count: result.assigned_loops.len(),
            count_mismatch_count,
            role_index_mismatch_count,
            loop_role_mismatch_count,
            fragment_range_mismatch_count,
            blocker_count,
        }
    }

    fn empty_like(
        status: BezierBooleanMaterializationAuditStatus,
        result: &BezierBooleanResultReport2,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            result_status: result.status,
            operation: result.operation,
            assigned_loop_count: 0,
            material_loop_count: 0,
            hole_loop_count: 0,
            directed_fragment_count: 0,
            audited_loop_count: 0,
            count_mismatch_count: 0,
            role_index_mismatch_count: 0,
            loop_role_mismatch_count: 0,
            fragment_range_mismatch_count: 0,
            blocker_count,
        }
    }

    /// Returns true when the retained higher-order result can feed materialization.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanMaterializationAuditStatus::Ready
    }

    /// Returns true when stale result payload data blocks materialization.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanMaterializationAuditStatus::ResultBlocked
                | BezierBooleanMaterializationAuditStatus::CountMismatch
                | BezierBooleanMaterializationAuditStatus::RoleIndexMismatch
                | BezierBooleanMaterializationAuditStatus::LoopRoleMismatch
                | BezierBooleanMaterializationAuditStatus::FragmentRangeMismatch
        )
    }
}

impl BezierBooleanMaterializedRegionReport2 {
    /// Materializes a scheduled endpoint result with laminar containment facts.
    ///
    /// This is the single-call materialization handoff for callers that already
    /// have a traversal schedule, exact ownership facts, a certified graph
    /// walk, and exact loop-containment facts. The method first uses
    /// [`BezierBooleanResultReport2::from_schedule_graph_walk_containment_facts`]
    /// so ownership, emission, closure, output-loop extraction, and role parity
    /// are validated exactly as in the normal result path. It then runs
    /// [`Self::from_result_laminar_containment_facts`] to attach each hole to
    /// its nearest certified material ancestor. The staged replay follows
    /// Yap's "Towards Exact Geometric Computation" (1997) predicate/
    /// construction boundary and the boundary/nesting/fill split of Vatti
    /// (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_schedule_graph_walk_laminar_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let result = BezierBooleanResultReport2::from_schedule_graph_walk_containment_facts(
            schedule,
            operation,
            ownership_facts,
            first_endpoints,
            second_endpoints,
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
        );
        Self::from_result_laminar_containment_facts(&result, containment_facts)
    }

    /// Materializes a scheduled endpoint result from replayed containment queries.
    ///
    /// This is the materialization endpoint for the current exact point/loop
    /// locator path. Query replay must be ready before the result constructor
    /// can derive loop roles, and the same replayed containment facts are then
    /// used to attach holes to their nearest certified material ancestors.
    /// Boundary, unknown, missing, extra, or stale query answers remain
    /// blockers, preserving Yap's (1997) exact-geometric-computation contract
    /// through the Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009) boundary/nesting/fill phases.
    pub fn from_schedule_graph_walk_laminar_containment_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        let result = BezierBooleanResultReport2::from_schedule_graph_walk_containment_query_results(
            schedule,
            operation,
            ownership_facts,
            first_endpoints,
            second_endpoints,
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            query_results,
        );
        Self::from_result_laminar_containment_query_results(&result, query_results)
    }

    /// Materializes a scheduled endpoint result from an end-to-end containment certificate.
    ///
    /// This is the most compact current handoff from exact point/loop locator
    /// answers to a materialized Bezier/conic boolean carrier. The certificate
    /// owns output-derived query generation, result replay, containment-fact
    /// validation, and depth derivation; this constructor then replays the same
    /// certificate through accepted-result construction and laminar hole
    /// attachment. It preserves Yap's (1997) rule that uncertified predicates
    /// remain blockers, while keeping Vatti (1992), Greiner-Hormann (1998),
    /// and Martinez-Rueda-Feito (2009) boundary/nesting/fill phases explicit.
    pub fn from_schedule_graph_walk_laminar_containment_certification(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        let result = BezierBooleanResultReport2::from_schedule_graph_walk_containment_certification(
            schedule,
            operation,
            ownership_facts,
            first_endpoints,
            second_endpoints,
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            certification,
        );
        Self::from_result_laminar_containment_certification(&result, certification)
    }

    /// Materializes a scheduled endpoint result from exact locator answers.
    ///
    /// This is the direct materialization counterpart to
    /// [`BezierBooleanResultReport2::from_schedule_graph_walk_locator_results`].
    /// It derives the output-loop containment certificate internally and then
    /// replays the same certificate for laminar hole attachment. The method is
    /// still report-bearing: locator boundary/unknowns, stale keys, malformed
    /// output-loop ranges, and non-laminar containment facts all block rather
    /// than being normalized or guessed. This is the Yap (1997) exactness
    /// boundary carried through the Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009) construction stages.
    pub fn from_schedule_graph_walk_laminar_locator_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        let certification =
            BezierBooleanLoopContainmentCertificationReport2::from_schedule_graph_walk_query_results(
                schedule,
                operation,
                ownership_facts,
                first_endpoints,
                second_endpoints,
                branch_vertex_count,
                resolved_overlap_count,
                walk_indices,
                results,
            );
        Self::from_schedule_graph_walk_laminar_containment_certification(
            schedule,
            operation,
            ownership_facts,
            first_endpoints,
            second_endpoints,
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            &certification,
        )
    }

    /// Materializes a multi-cycle-successor result with laminar containment facts.
    ///
    /// This is the materialization-level handoff for exact successor graphs
    /// whose boundary consists of several disjoint cycles. It composes
    /// [`BezierBooleanResultReport2::from_multi_cycle_successor_containment_facts`]
    /// with [`Self::from_result_laminar_containment_facts`] so the same
    /// successor certificate validates output-loop closure and role parity
    /// before hole attachment. Following Yap's "Towards Exact Geometric
    /// Computation" (1997), the containment facts are consumed as certified
    /// predicates rather than tolerance fallbacks; the explicit
    /// boundary/nesting/fill handoff follows Vatti (1992),
    /// Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_multi_cycle_successor_laminar_containment_facts(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let result = BezierBooleanResultReport2::from_multi_cycle_successor_containment_facts(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
            containment_facts,
        );
        Self::from_result_laminar_containment_facts(&result, containment_facts)
    }

    /// Materializes a multi-cycle-successor result from replayed containment queries.
    ///
    /// Query replay is kept report-bearing all the way to materialization: a
    /// boundary, unknown, stale, missing, or extra locator answer blocks the
    /// result constructor and then blocks this materializer. Ready replayed
    /// facts are attached through the laminar nearest-material ancestor rule,
    /// preserving Yap's (1997) predicate/construction separation.
    pub fn from_multi_cycle_successor_laminar_containment_query_results(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        let result =
            BezierBooleanResultReport2::from_multi_cycle_successor_containment_query_results(
                multi_cycle,
                traversal,
                plan,
                first_endpoints,
                second_endpoints,
                query_results,
            );
        Self::from_result_laminar_containment_query_results(&result, query_results)
    }

    /// Materializes a multi-cycle-successor result from an end-to-end certificate.
    ///
    /// The containment certificate owns locator replay and containment-fact
    /// validation; this wrapper owns only the final accepted-result audit and
    /// laminar component construction. That separation mirrors the exact
    /// geometric-computation contract in Yap (1997) and keeps the
    /// Vatti/Greiner-Hormann/Martinez-Rueda-Feito boolean phases explicit.
    pub fn from_multi_cycle_successor_laminar_containment_certification(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        let result =
            BezierBooleanResultReport2::from_multi_cycle_successor_containment_certification(
                multi_cycle,
                traversal,
                plan,
                first_endpoints,
                second_endpoints,
                certification,
            );
        Self::from_result_laminar_containment_certification(&result, certification)
    }

    /// Materializes a multi-cycle-successor result from exact locator answers.
    ///
    /// The containment certificate is derived from the successor-preserved
    /// output loops before materialization, rather than accepted as a detached
    /// query replay. This keeps Yap's (1997) exact predicate replay boundary
    /// intact through final laminar component construction.
    pub fn from_multi_cycle_successor_laminar_locator_results(
        multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
        traversal: &BezierBooleanLoopGraphTraversalReport2,
        plan: &BezierBooleanLoopAssemblyPlanReport2,
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        let certification =
            BezierBooleanLoopContainmentCertificationReport2::from_multi_cycle_successor_query_results(
                multi_cycle,
                traversal,
                plan,
                first_endpoints,
                second_endpoints,
                results,
            );
        Self::from_multi_cycle_successor_laminar_containment_certification(
            multi_cycle,
            traversal,
            plan,
            first_endpoints,
            second_endpoints,
            &certification,
        )
    }

    /// Materializes a scheduled result from exact multi-cycle successor facts.
    ///
    /// This is the scheduled counterpart to
    /// [`Self::from_multi_cycle_successor_laminar_containment_facts`]. The
    /// schedule, ownership facts, emitted plan, certified successor graph,
    /// endpoint closure, containment parity, and laminar materialization are
    /// composed without flattening multiple cycles into a caller-maintained
    /// walk vector.
    pub fn from_schedule_multi_cycle_successor_laminar_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let result =
            BezierBooleanResultReport2::from_schedule_multi_cycle_successor_containment_facts(
                schedule,
                operation,
                ownership_facts,
                first_endpoints,
                second_endpoints,
                branch_vertex_count,
                resolved_overlap_count,
                successor_facts,
                containment_facts,
            );
        Self::from_result_laminar_containment_facts(&result, containment_facts)
    }

    /// Materializes a scheduled multi-cycle successor result from replayed queries.
    ///
    /// The replay report is threaded through both result acceptance and
    /// materialization so unresolved locator answers cannot be silently
    /// converted to missing containment facts.
    pub fn from_schedule_multi_cycle_successor_laminar_containment_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        let result = BezierBooleanResultReport2::
            from_schedule_multi_cycle_successor_containment_query_results(
                schedule,
                operation,
                ownership_facts,
                first_endpoints,
                second_endpoints,
                branch_vertex_count,
                resolved_overlap_count,
                successor_facts,
                query_results,
            );
        Self::from_result_laminar_containment_query_results(&result, query_results)
    }

    /// Materializes a scheduled multi-cycle successor result from a containment certificate.
    ///
    /// This is the compact endpoint for callers that already have exact
    /// successor facts and an output-loop containment certificate. It rejects
    /// stale certificates before laminar component construction, preserving
    /// Yap's (1997) exact predicate boundary.
    pub fn from_schedule_multi_cycle_successor_laminar_containment_certification(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        let result = BezierBooleanResultReport2::
            from_schedule_multi_cycle_successor_containment_certification(
                schedule,
                operation,
                ownership_facts,
                first_endpoints,
                second_endpoints,
                branch_vertex_count,
                resolved_overlap_count,
                successor_facts,
                certification,
        );
        Self::from_result_laminar_containment_certification(&result, certification)
    }

    /// Materializes a scheduled multi-cycle successor result from locator answers.
    ///
    /// This is the direct materialization endpoint for exact point/loop
    /// locator integrations that certify successor edges instead of a flat
    /// graph-walk permutation. It derives and replays the containment
    /// certificate internally, so malformed successor cycles, output-loop
    /// ranges, boundary locator hits, and stale query keys all block before
    /// hole attachment.
    pub fn from_schedule_multi_cycle_successor_laminar_locator_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        let certification =
            BezierBooleanLoopContainmentCertificationReport2::from_schedule_multi_cycle_successor_query_results(
                schedule,
                operation,
                ownership_facts,
                first_endpoints,
                second_endpoints,
                branch_vertex_count,
                resolved_overlap_count,
                successor_facts,
                results,
            );
        Self::from_schedule_multi_cycle_successor_laminar_containment_certification(
            schedule,
            operation,
            ownership_facts,
            first_endpoints,
            second_endpoints,
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            &certification,
        )
    }

    /// Materializes a quadratic Bezier multi-cycle successor result from containment facts.
    ///
    /// This typed wrapper keeps exact quadratic endpoint extraction inside the
    /// API while composing the same successor, containment, and laminar
    /// materialization certificates as the generic endpoint path.
    pub fn from_quadratic_schedule_multi_cycle_successor_laminar_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            containment_facts,
        )
    }

    /// Materializes a quadratic Bezier multi-cycle successor result from replayed containment queries.
    pub fn from_quadratic_schedule_multi_cycle_successor_laminar_containment_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_containment_query_results(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            query_results,
        )
    }

    /// Materializes a quadratic Bezier multi-cycle successor result from a containment certificate.
    pub fn from_quadratic_schedule_multi_cycle_successor_laminar_containment_certification(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_containment_certification(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            certification,
        )
    }

    /// Materializes a quadratic Bezier multi-cycle successor result from locator answers.
    ///
    /// The wrapper is intentionally thin: quadratic fragments only supply exact
    /// endpoint carriers, while successor facts and locator answers remain
    /// external certificates replayed under Yap's (1997) exact-computation
    /// boundary.
    pub fn from_quadratic_schedule_multi_cycle_successor_laminar_locator_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_locator_results(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            results,
        )
    }

    /// Materializes a cubic Bezier multi-cycle successor result from locator answers.
    pub fn from_cubic_schedule_multi_cycle_successor_laminar_locator_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_locator_results(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            results,
        )
    }

    /// Materializes a rational quadratic/conic multi-cycle successor result from locator answers.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_laminar_locator_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        results: &[BezierBooleanLoopContainmentQueryResult2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_locator_results(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            results,
        )
    }

    /// Materializes a cubic Bezier multi-cycle successor result from containment facts.
    pub fn from_cubic_schedule_multi_cycle_successor_laminar_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            containment_facts,
        )
    }

    /// Materializes a cubic Bezier multi-cycle successor result from replayed containment queries.
    pub fn from_cubic_schedule_multi_cycle_successor_laminar_containment_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_containment_query_results(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            query_results,
        )
    }

    /// Materializes a cubic Bezier multi-cycle successor result from a containment certificate.
    pub fn from_cubic_schedule_multi_cycle_successor_laminar_containment_certification(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_containment_certification(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            certification,
        )
    }

    /// Materializes a rational quadratic/conic multi-cycle successor result from containment facts.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_laminar_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_containment_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            containment_facts,
        )
    }

    /// Materializes a rational quadratic/conic multi-cycle successor result from replayed containment queries.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_laminar_containment_query_results(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_containment_query_results(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            query_results,
        )
    }

    /// Materializes a rational quadratic/conic multi-cycle successor result from a containment certificate.
    pub fn from_rational_quadratic_schedule_multi_cycle_successor_laminar_containment_certification(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        successor_facts: &[BezierBooleanLoopGraphSuccessorFact2],
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        Self::from_schedule_multi_cycle_successor_laminar_containment_certification(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            branch_vertex_count,
            resolved_overlap_count,
            successor_facts,
            certification,
        )
    }

    /// Materializes a scheduled endpoint result with graph facts and laminar containment.
    pub fn from_schedule_graph_fact_walk_laminar_containment_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let result = BezierBooleanResultReport2::from_schedule_graph_fact_walk_containment_facts(
            schedule,
            operation,
            ownership_facts,
            first_endpoints,
            second_endpoints,
            graph_facts,
            walk_indices,
            containment_facts,
        );
        Self::from_result_laminar_containment_facts(&result, containment_facts)
    }

    /// Materializes a scheduled endpoint result when explicit roles match containment parity.
    ///
    /// The explicit roles are not trusted independently: the result constructor
    /// first checks them against containment-derived depth parity, then this
    /// materializer attaches holes through the same laminar containment
    /// certificate. Stale role facts therefore block before materialization.
    pub fn from_schedule_graph_walk_laminar_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        branch_vertex_count: usize,
        resolved_overlap_count: usize,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let result = BezierBooleanResultReport2::from_schedule_graph_walk_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            first_endpoints,
            second_endpoints,
            branch_vertex_count,
            resolved_overlap_count,
            walk_indices,
            containment_facts,
            roles,
        );
        Self::from_result_laminar_containment_facts(&result, containment_facts)
    }

    /// Materializes a scheduled endpoint result with graph facts and containment-certified roles.
    pub fn from_schedule_graph_fact_walk_laminar_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first_endpoints: &[(Point2, Point2)],
        second_endpoints: &[(Point2, Point2)],
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        let result =
            BezierBooleanResultReport2::from_schedule_graph_fact_walk_containment_role_facts(
                schedule,
                operation,
                ownership_facts,
                first_endpoints,
                second_endpoints,
                graph_facts,
                walk_indices,
                containment_facts,
                roles,
            );
        Self::from_result_laminar_containment_facts(&result, containment_facts)
    }

    /// Materializes a quadratic Bezier result with graph facts and containment-certified roles.
    pub fn from_quadratic_schedule_graph_fact_walk_laminar_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanQuadraticFragmentReport2,
        second: &BezierBooleanQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_laminar_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &quadratic_fragment_endpoints(&first.fragments),
            &quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
            roles,
        )
    }

    /// Materializes a cubic Bezier result with graph facts and containment-certified roles.
    pub fn from_cubic_schedule_graph_fact_walk_laminar_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanCubicFragmentReport2,
        second: &BezierBooleanCubicFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_laminar_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &cubic_fragment_endpoints(&first.fragments),
            &cubic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
            roles,
        )
    }

    /// Materializes a rational quadratic/conic result with graph facts and containment-certified roles.
    pub fn from_rational_quadratic_schedule_graph_fact_walk_laminar_containment_role_facts(
        schedule: &BezierBooleanTraversalScheduleReport2,
        operation: BooleanOp,
        ownership_facts: &[BezierBooleanOwnershipFact2],
        first: &BezierBooleanRationalQuadraticFragmentReport2,
        second: &BezierBooleanRationalQuadraticFragmentReport2,
        graph_facts: &BezierBooleanLoopGraphFacts2,
        walk_indices: &[usize],
        containment_facts: &[BezierBooleanLoopContainmentFact2],
        roles: &[BezierBooleanOutputLoopRole],
    ) -> Self {
        Self::from_schedule_graph_fact_walk_laminar_containment_role_facts(
            schedule,
            operation,
            ownership_facts,
            &rational_quadratic_fragment_endpoints(&first.fragments),
            &rational_quadratic_fragment_endpoints(&second.fragments),
            graph_facts,
            walk_indices,
            containment_facts,
            roles,
        )
    }

    /// Materializes a higher-order region carrier from a result and containment facts.
    ///
    /// A ready result is not enough to attach holes: each hole must have a
    /// certified material container. This constructor validates that handoff
    /// without running a geometric locator. Duplicate, self-containing,
    /// out-of-range, material-to-material, hole-to-hole, and hole-containing-
    /// material facts remain blockers. Following Yap (1997), containment is a
    /// certified predicate fact consumed by construction, not a tolerance
    /// fallback. The staged boundary/containment/fill decomposition follows
    /// Vatti (1992), Greiner-Hormann (1998), and Martinez-Rueda-Feito (2009).
    pub fn from_result_containment_facts(
        result: &BezierBooleanResultReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let audit = BezierBooleanMaterializationAuditReport2::from_result(result);
        match audit.status {
            BezierBooleanMaterializationAuditStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanMaterializedRegionStatus::Empty,
                    result,
                    &audit,
                    containment_facts.len(),
                    0,
                );
            }
            BezierBooleanMaterializationAuditStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanMaterializedRegionStatus::NoInteriorSplits,
                    result,
                    &audit,
                    containment_facts.len(),
                    0,
                );
            }
            BezierBooleanMaterializationAuditStatus::Ready => {}
            BezierBooleanMaterializationAuditStatus::ResultBlocked
            | BezierBooleanMaterializationAuditStatus::CountMismatch
            | BezierBooleanMaterializationAuditStatus::RoleIndexMismatch
            | BezierBooleanMaterializationAuditStatus::LoopRoleMismatch
            | BezierBooleanMaterializationAuditStatus::FragmentRangeMismatch => {
                return Self::empty_like(
                    BezierBooleanMaterializedRegionStatus::AuditBlocked,
                    result,
                    &audit,
                    containment_facts.len(),
                    audit.blocker_count.max(1),
                );
            }
        }

        let mut material_position_by_loop = vec![None; result.assigned_loops.len()];
        let mut components = Vec::with_capacity(result.material_loop_indices.len());
        for material_loop_index in result.material_loop_indices.iter().copied() {
            material_position_by_loop[material_loop_index] = Some(components.len());
            components.push(BezierBooleanMaterializedComponent2 {
                material_loop_index,
                hole_loop_indices: Vec::new(),
            });
        }

        let mut hole_assignment_count = vec![0usize; result.assigned_loops.len()];
        let mut seen_pairs = HashSet::new();
        let mut stale_containment_count = 0;
        let mut role_incompatible_containment_count = 0;
        for fact in containment_facts {
            let pair = (fact.container_loop_index, fact.contained_loop_index);
            if fact.container_loop_index == fact.contained_loop_index
                || fact.container_loop_index >= result.assigned_loops.len()
                || fact.contained_loop_index >= result.assigned_loops.len()
                || !seen_pairs.insert(pair)
            {
                stale_containment_count += 1;
                continue;
            }

            let container = &result.assigned_loops[fact.container_loop_index];
            let contained = &result.assigned_loops[fact.contained_loop_index];
            if container.role != BezierBooleanOutputLoopRole::Material
                || contained.role != BezierBooleanOutputLoopRole::Hole
            {
                role_incompatible_containment_count += 1;
                continue;
            }

            if let Some(component_index) = material_position_by_loop[fact.container_loop_index] {
                components[component_index]
                    .hole_loop_indices
                    .push(fact.contained_loop_index);
                hole_assignment_count[fact.contained_loop_index] += 1;
            } else {
                role_incompatible_containment_count += 1;
            }
        }

        let missing_hole_containment_count = result
            .hole_loop_indices
            .iter()
            .filter(|index| hole_assignment_count[**index] == 0)
            .count();
        let ambiguous_hole_containment_count = result
            .hole_loop_indices
            .iter()
            .filter(|index| hole_assignment_count[**index] > 1)
            .count();
        let blocker_count = missing_hole_containment_count
            + ambiguous_hole_containment_count
            + stale_containment_count
            + role_incompatible_containment_count;
        let status = if stale_containment_count > 0 {
            BezierBooleanMaterializedRegionStatus::StaleContainmentFact
        } else if role_incompatible_containment_count > 0 {
            BezierBooleanMaterializedRegionStatus::RoleIncompatibleContainment
        } else if ambiguous_hole_containment_count > 0 {
            BezierBooleanMaterializedRegionStatus::AmbiguousHoleContainment
        } else if missing_hole_containment_count > 0 {
            BezierBooleanMaterializedRegionStatus::MissingHoleContainment
        } else {
            BezierBooleanMaterializedRegionStatus::Ready
        };

        Self {
            status,
            audit_status: audit.status,
            operation: result.operation,
            directed_fragments: result.directed_fragments.clone(),
            assigned_loops: result.assigned_loops.clone(),
            component_count: components.len(),
            components,
            supplied_containment_count: containment_facts.len(),
            missing_hole_containment_count,
            ambiguous_hole_containment_count,
            stale_containment_count,
            role_incompatible_containment_count,
            blocker_count,
        }
    }

    /// Materializes a region carrier from a full laminar containment certificate.
    ///
    /// Unlike [`Self::from_result_containment_facts`], this constructor accepts
    /// ancestor containment facts for nested islands. Each hole is attached to
    /// its nearest certified material ancestor, so an outer material loop and
    /// an inner island material loop may both contain the same deeper hole
    /// without being treated as ambiguous. The containment graph is still
    /// validated for stale indices, self-containment, duplicate facts, directed
    /// cycles, and non-laminar shared containers before it can affect
    /// materialization. This follows Yap (1997): containment is certified
    /// combinatorial evidence consumed by construction. The nearest-ancestor
    /// fill attachment is the explicit nesting/fill phase separated from
    /// boundary construction in Vatti (1992), Greiner-Hormann (1998), and
    /// Martinez-Rueda-Feito (2009).
    pub fn from_result_laminar_containment_facts(
        result: &BezierBooleanResultReport2,
        containment_facts: &[BezierBooleanLoopContainmentFact2],
    ) -> Self {
        let audit = BezierBooleanMaterializationAuditReport2::from_result(result);
        match audit.status {
            BezierBooleanMaterializationAuditStatus::Empty => {
                return Self::empty_like(
                    BezierBooleanMaterializedRegionStatus::Empty,
                    result,
                    &audit,
                    containment_facts.len(),
                    0,
                );
            }
            BezierBooleanMaterializationAuditStatus::NoInteriorSplits => {
                return Self::empty_like(
                    BezierBooleanMaterializedRegionStatus::NoInteriorSplits,
                    result,
                    &audit,
                    containment_facts.len(),
                    0,
                );
            }
            BezierBooleanMaterializationAuditStatus::Ready => {}
            BezierBooleanMaterializationAuditStatus::ResultBlocked
            | BezierBooleanMaterializationAuditStatus::CountMismatch
            | BezierBooleanMaterializationAuditStatus::RoleIndexMismatch
            | BezierBooleanMaterializationAuditStatus::LoopRoleMismatch
            | BezierBooleanMaterializationAuditStatus::FragmentRangeMismatch => {
                return Self::empty_like(
                    BezierBooleanMaterializedRegionStatus::AuditBlocked,
                    result,
                    &audit,
                    containment_facts.len(),
                    audit.blocker_count.max(1),
                );
            }
        }

        let loop_count = result.assigned_loops.len();
        let mut stale_containment_count = 0;
        let mut seen_pairs = HashSet::new();
        let mut contains = vec![vec![false; loop_count]; loop_count];
        for fact in containment_facts {
            if fact.container_loop_index == fact.contained_loop_index
                || fact.container_loop_index >= loop_count
                || fact.contained_loop_index >= loop_count
                || !seen_pairs.insert((fact.container_loop_index, fact.contained_loop_index))
            {
                stale_containment_count += 1;
                continue;
            }
            contains[fact.container_loop_index][fact.contained_loop_index] = true;
        }

        for pivot in 0..loop_count {
            for container in 0..loop_count {
                if contains[container][pivot] {
                    for contained in 0..loop_count {
                        if contains[pivot][contained] {
                            contains[container][contained] = true;
                        }
                    }
                }
            }
        }

        for index in 0..loop_count {
            if contains[index][index] {
                stale_containment_count += 1;
            }
        }
        for contained_loop_index in 0..loop_count {
            let containers = (0..loop_count)
                .filter(|container_loop_index| {
                    contains[*container_loop_index][contained_loop_index]
                })
                .collect::<Vec<_>>();
            for first_index in 0..containers.len() {
                for second_index in (first_index + 1)..containers.len() {
                    let first = containers[first_index];
                    let second = containers[second_index];
                    if !contains[first][second] && !contains[second][first] {
                        stale_containment_count += 1;
                    }
                }
            }
        }

        let mut material_position_by_loop = vec![None; loop_count];
        let mut components = Vec::with_capacity(result.material_loop_indices.len());
        for material_loop_index in result.material_loop_indices.iter().copied() {
            material_position_by_loop[material_loop_index] = Some(components.len());
            components.push(BezierBooleanMaterializedComponent2 {
                material_loop_index,
                hole_loop_indices: Vec::new(),
            });
        }

        let mut missing_hole_containment_count = 0;
        let mut ambiguous_hole_containment_count = 0;
        if stale_containment_count == 0 {
            for hole_loop_index in result.hole_loop_indices.iter().copied() {
                let material_ancestors = result
                    .material_loop_indices
                    .iter()
                    .copied()
                    .filter(|material_loop_index| contains[*material_loop_index][hole_loop_index])
                    .collect::<Vec<_>>();
                if material_ancestors.is_empty() {
                    missing_hole_containment_count += 1;
                    continue;
                }

                let nearest = material_ancestors
                    .iter()
                    .copied()
                    .filter(|candidate| {
                        !material_ancestors
                            .iter()
                            .any(|other| candidate != other && contains[*candidate][*other])
                    })
                    .collect::<Vec<_>>();
                if nearest.len() != 1 {
                    ambiguous_hole_containment_count += 1;
                    continue;
                }

                if let Some(component_index) = material_position_by_loop[nearest[0]] {
                    components[component_index]
                        .hole_loop_indices
                        .push(hole_loop_index);
                } else {
                    stale_containment_count += 1;
                }
            }
        }

        let blocker_count = missing_hole_containment_count
            + ambiguous_hole_containment_count
            + stale_containment_count;
        let status = if stale_containment_count > 0 {
            BezierBooleanMaterializedRegionStatus::StaleContainmentFact
        } else if ambiguous_hole_containment_count > 0 {
            BezierBooleanMaterializedRegionStatus::AmbiguousHoleContainment
        } else if missing_hole_containment_count > 0 {
            BezierBooleanMaterializedRegionStatus::MissingHoleContainment
        } else {
            BezierBooleanMaterializedRegionStatus::Ready
        };

        Self {
            status,
            audit_status: audit.status,
            operation: result.operation,
            directed_fragments: result.directed_fragments.clone(),
            assigned_loops: result.assigned_loops.clone(),
            component_count: components.len(),
            components,
            supplied_containment_count: containment_facts.len(),
            missing_hole_containment_count,
            ambiguous_hole_containment_count,
            stale_containment_count,
            role_incompatible_containment_count: 0,
            blocker_count,
        }
    }

    /// Materializes a result from replayed containment-query facts.
    ///
    /// This wrapper deliberately consumes
    /// [`BezierBooleanLoopContainmentQueryResultReport2`] rather than its raw
    /// fact vector so unresolved point/loop locator outcomes cannot be dropped
    /// at the materialization boundary. If replay is blocked, materialization
    /// is blocked before hole attachment. If replay is ready, the contained
    /// facts flow into [`Self::from_result_laminar_containment_facts`]. This
    /// is the final Yap (1997) replay check for the current Bezier/conic
    /// boolean containment path.
    pub fn from_result_laminar_containment_query_results(
        result: &BezierBooleanResultReport2,
        query_results: &BezierBooleanLoopContainmentQueryResultReport2,
    ) -> Self {
        if query_results.has_blockers() || query_results.operation != result.operation {
            let audit = BezierBooleanMaterializationAuditReport2::from_result(result);
            return Self::empty_like(
                BezierBooleanMaterializedRegionStatus::AuditBlocked,
                result,
                &audit,
                query_results.containment_fact_count,
                query_results
                    .blocker_count
                    .max(usize::from(query_results.operation != result.operation))
                    .max(1),
            );
        }

        Self::from_result_laminar_containment_facts(result, &query_results.containment_facts)
    }

    /// Materializes a result from an end-to-end containment certificate.
    ///
    /// The certificate's containment facts are used for nearest-material
    /// ancestor attachment only after its replay and fact-validation statuses
    /// are ready. This prevents a failed locator replay from reaching
    /// materialization as an empty fact list.
    pub fn from_result_laminar_containment_certification(
        result: &BezierBooleanResultReport2,
        certification: &BezierBooleanLoopContainmentCertificationReport2,
    ) -> Self {
        if certification.has_blockers() || certification.operation != result.operation {
            let audit = BezierBooleanMaterializationAuditReport2::from_result(result);
            return Self::empty_like(
                BezierBooleanMaterializedRegionStatus::AuditBlocked,
                result,
                &audit,
                certification.containment_fact_count,
                certification
                    .blocker_count
                    .max(usize::from(certification.operation != result.operation))
                    .max(1),
            );
        }

        Self::from_result_laminar_containment_facts(result, &certification.containment_facts)
    }

    fn empty_like(
        status: BezierBooleanMaterializedRegionStatus,
        result: &BezierBooleanResultReport2,
        audit: &BezierBooleanMaterializationAuditReport2,
        supplied_containment_count: usize,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            audit_status: audit.status,
            operation: result.operation,
            directed_fragments: Vec::new(),
            assigned_loops: Vec::new(),
            components: Vec::new(),
            component_count: 0,
            supplied_containment_count,
            missing_hole_containment_count: 0,
            ambiguous_hole_containment_count: 0,
            stale_containment_count: 0,
            role_incompatible_containment_count: 0,
            blocker_count,
        }
    }

    /// Returns true when material loops and certified holes form a region carrier.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanMaterializedRegionStatus::Ready
    }

    /// Returns true when audit or containment facts block materialization.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanMaterializedRegionStatus::AuditBlocked
                | BezierBooleanMaterializedRegionStatus::MissingHoleContainment
                | BezierBooleanMaterializedRegionStatus::AmbiguousHoleContainment
                | BezierBooleanMaterializedRegionStatus::StaleContainmentFact
                | BezierBooleanMaterializedRegionStatus::RoleIncompatibleContainment
        )
    }
}

fn material_action_for_bezier_step(
    operation: BooleanOp,
    operand: BezierBooleanTraversalOperand,
    opposite_location: BezierBooleanFragmentOwnershipLocation,
) -> BooleanFragmentAction {
    use BezierBooleanFragmentOwnershipLocation::{Boundary, Inside, Outside};
    use BezierBooleanTraversalOperand::First;
    use BooleanFragmentAction::{
        BoundaryNeedsResolution, Discard, KeepReversed, KeepSourceDirection,
    };

    match opposite_location {
        Boundary => BoundaryNeedsResolution,
        Outside => match operation {
            BooleanOp::Union | BooleanOp::Xor => KeepSourceDirection,
            BooleanOp::Intersection => Discard,
            BooleanOp::Difference => {
                if operand == First {
                    KeepSourceDirection
                } else {
                    Discard
                }
            }
        },
        Inside => match operation {
            BooleanOp::Intersection => KeepSourceDirection,
            BooleanOp::Union => Discard,
            BooleanOp::Xor => KeepReversed,
            BooleanOp::Difference => {
                if operand == First {
                    Discard
                } else {
                    KeepReversed
                }
            }
        },
    }
}

impl BezierBooleanCubicFragmentReport2 {
    /// Splits `curve` at first-operand parameters from a readiness certificate.
    pub fn from_first_curve_readiness(
        curve: &CubicBezier2,
        readiness: &BezierBooleanConstructionReadinessReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_readiness_parameters(
            curve,
            readiness,
            &readiness.insertion.first_curve_interior_parameters,
            policy,
        )
    }

    /// Splits `curve` at second-operand parameters from a readiness certificate.
    pub fn from_second_curve_readiness(
        curve: &CubicBezier2,
        readiness: &BezierBooleanConstructionReadinessReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_readiness_parameters(
            curve,
            readiness,
            &readiness.insertion.second_curve_interior_parameters,
            policy,
        )
    }

    /// Splits `curve` at caller-supplied exact parameters.
    pub fn from_parameters(
        curve: &CubicBezier2,
        parameters: &[Real],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_parameters_with_status(
            curve,
            parameters,
            BezierBooleanConstructionReadinessStatus::Ready,
            policy,
        )
    }

    fn from_readiness_parameters(
        curve: &CubicBezier2,
        readiness: &BezierBooleanConstructionReadinessReport2,
        parameters: &[Real],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        match readiness.status {
            BezierBooleanConstructionReadinessStatus::Empty => {
                Classification::Decided(cubic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::Empty,
                    readiness.status,
                    parameters.len(),
                    0,
                    0,
                    Vec::new(),
                    vec![curve.clone()],
                ))
            }
            BezierBooleanConstructionReadinessStatus::Blocked => {
                Classification::Decided(cubic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::Blocked,
                    readiness.status,
                    parameters.len(),
                    0,
                    0,
                    Vec::new(),
                    Vec::new(),
                ))
            }
            BezierBooleanConstructionReadinessStatus::InvalidParameterDomain => {
                Classification::Decided(cubic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::InvalidParameterDomain,
                    readiness.status,
                    parameters.len(),
                    0,
                    readiness.insertion.out_of_range_parameter_count,
                    Vec::new(),
                    Vec::new(),
                ))
            }
            BezierBooleanConstructionReadinessStatus::NoInteriorSplits => {
                Classification::Decided(cubic_fragment_report(
                    BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
                    readiness.status,
                    parameters.len(),
                    readiness.insertion.endpoint_parameter_count,
                    0,
                    Vec::new(),
                    vec![curve.clone()],
                ))
            }
            BezierBooleanConstructionReadinessStatus::Ready => {
                Self::from_parameters_with_status(curve, parameters, readiness.status, policy)
            }
        }
    }

    fn from_parameters_with_status(
        curve: &CubicBezier2,
        parameters: &[Real],
        readiness_status: BezierBooleanConstructionReadinessStatus,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let normalized = match normalize_split_parameters(parameters, policy) {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        if normalized.out_of_range_parameter_count > 0 {
            return Classification::Decided(cubic_fragment_report(
                BezierBooleanFragmentConstructionStatus::InvalidParameterDomain,
                readiness_status,
                parameters.len(),
                normalized.endpoint_parameter_count,
                normalized.out_of_range_parameter_count,
                Vec::new(),
                Vec::new(),
            ));
        }
        if normalized.interior_parameters.is_empty() {
            return Classification::Decided(cubic_fragment_report(
                BezierBooleanFragmentConstructionStatus::NoInteriorSplits,
                readiness_status,
                parameters.len(),
                normalized.endpoint_parameter_count,
                0,
                Vec::new(),
                vec![curve.clone()],
            ));
        }

        let fragments = split_cubic_at_sorted_parameters(curve, &normalized.interior_parameters);
        Classification::Decided(cubic_fragment_report(
            BezierBooleanFragmentConstructionStatus::Ready,
            readiness_status,
            parameters.len(),
            normalized.endpoint_parameter_count,
            0,
            normalized.interior_parameters,
            fragments,
        ))
    }
}

impl BezierBooleanConstructionReadinessReport2 {
    /// Builds the complete report-only Bezier boolean construction certificate.
    ///
    /// Any uncertain parameter ordering from the audit/insertion stage is
    /// propagated as [`Classification::Uncertain`] instead of being hidden
    /// inside a readiness state.
    pub fn from_scheduler(
        scheduler: BezierBooleanPathSchedulerReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let split_plan = BezierBooleanSplitPlanReport2::from_scheduler(&scheduler);
        Self::from_split_plan(scheduler, split_plan, policy)
    }

    /// Builds construction readiness from a root-isolation replay report.
    ///
    /// This is the boolean-facing continuation of `hypersolve` root isolation:
    /// [`BezierBooleanRootIsolationReplayReport2`] may contribute exact
    /// represented roots that were not present in the original scheduler, then
    /// this constructor runs the same unit-domain audit and insertion
    /// classification used by ordinary split-ready schedulers. The split plan
    /// remains blocker-marked unless replay reached
    /// [`BezierBooleanRootIsolationReplayStatus::ReadyForSplitEvents`].
    /// That preserves Yap's exact-geometric-computation boundary and the
    /// Greiner-Hormann/Martinez-Rueda-Feito split-before-traversal discipline:
    /// solver intervals never become topology unless they are replayed as exact
    /// Bezier parameters.
    pub fn from_root_isolation_replay(
        scheduler: BezierBooleanPathSchedulerReport2,
        replay: &BezierBooleanRootIsolationReplayReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let split_plan = if replay.can_feed_split_events()
            || matches!(
                replay.status,
                BezierBooleanRootIsolationReplayStatus::Empty
                    | BezierBooleanRootIsolationReplayStatus::NotNeeded
            ) {
            replay.split_plan.clone()
        } else {
            BezierBooleanSplitPlanReport2 {
                status: BezierBooleanSplitPlanStatus::Blocked,
                scheduler_status: scheduler.status,
                first_curve_parameters: replay.split_plan.first_curve_parameters.clone(),
                second_curve_parameters: replay.split_plan.second_curve_parameters.clone(),
                shared_range_parameters: replay.split_plan.shared_range_parameters.clone(),
                relation_event_count: replay.split_plan.relation_event_count,
                range_event_count: replay.split_plan.range_event_count,
                uncertainty_reason: replay.split_plan.uncertainty_reason,
            }
        };
        Self::from_split_plan(scheduler, split_plan, policy)
    }

    /// Builds construction readiness directly from relation and range reports.
    pub fn from_reports(
        relation_reports: &[BezierBooleanHandoffReport2],
        range_reports: &[BezierPathRangeOrderReport2],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        Self::from_scheduler(
            BezierBooleanPathSchedulerReport2::from_reports(relation_reports, range_reports),
            policy,
        )
    }

    fn from_split_plan(
        scheduler: BezierBooleanPathSchedulerReport2,
        split_plan: BezierBooleanSplitPlanReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let split_plan_audit = match split_plan.audit(policy) {
            Classification::Decided(audit) => audit,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let insertion = match split_plan.insertion_report(policy) {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let status = match insertion.status {
            BezierBooleanSplitInsertionStatus::Empty => {
                BezierBooleanConstructionReadinessStatus::Empty
            }
            BezierBooleanSplitInsertionStatus::NoInteriorSplits => {
                BezierBooleanConstructionReadinessStatus::NoInteriorSplits
            }
            BezierBooleanSplitInsertionStatus::Ready => {
                BezierBooleanConstructionReadinessStatus::Ready
            }
            BezierBooleanSplitInsertionStatus::Blocked => {
                BezierBooleanConstructionReadinessStatus::Blocked
            }
            BezierBooleanSplitInsertionStatus::InvalidParameterDomain => {
                BezierBooleanConstructionReadinessStatus::InvalidParameterDomain
            }
        };

        Classification::Decided(Self {
            status,
            scheduler,
            split_plan,
            split_plan_audit,
            insertion,
        })
    }

    /// Returns true when interior split parameters are ready for insertion.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanConstructionReadinessStatus::Ready
    }

    /// Returns true when construction is blocked by unresolved or invalid facts.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanConstructionReadinessStatus::Blocked
                | BezierBooleanConstructionReadinessStatus::InvalidParameterDomain
        )
    }
}

struct NormalizedSplitParameters {
    interior_parameters: Vec<Real>,
    endpoint_parameter_count: usize,
    out_of_range_parameter_count: usize,
}

fn normalize_split_parameters(
    parameters: &[Real],
    policy: &CurvePolicy,
) -> Classification<NormalizedSplitParameters> {
    let mut interior_parameters = Vec::new();
    let mut endpoint_parameter_count = 0;
    let mut out_of_range_parameter_count = 0;

    for parameter in parameters {
        match split_parameter_location(parameter, policy) {
            Some(BezierBooleanSplitParameterLocation::Interior) => {
                match insert_unique_sorted_parameter(&mut interior_parameters, parameter, policy) {
                    Some(()) => {}
                    None => return Classification::Uncertain(UncertaintyReason::Ordering),
                }
            }
            Some(BezierBooleanSplitParameterLocation::Endpoint) => {
                endpoint_parameter_count += 1;
            }
            Some(BezierBooleanSplitParameterLocation::OutOfRange) => {
                out_of_range_parameter_count += 1;
            }
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }

    Classification::Decided(NormalizedSplitParameters {
        interior_parameters,
        endpoint_parameter_count,
        out_of_range_parameter_count,
    })
}

fn sorted_unit_range_boundaries(
    range: &ParamRange,
    policy: &CurvePolicy,
) -> Classification<Option<Vec<Real>>> {
    if parameter_in_unit_interval(range.start(), policy) != Some(true)
        || parameter_in_unit_interval(range.end(), policy) != Some(true)
    {
        if parameter_in_unit_interval(range.start(), policy).is_none()
            || parameter_in_unit_interval(range.end(), policy).is_none()
        {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }
        return Classification::Decided(None);
    }

    let mut parameters = Vec::with_capacity(2);
    if insert_unique_sorted_parameter(&mut parameters, range.start(), policy).is_none()
        || insert_unique_sorted_parameter(&mut parameters, range.end(), policy).is_none()
    {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    }
    Classification::Decided(Some(parameters))
}

fn insert_unique_sorted_parameter(
    parameters: &mut Vec<Real>,
    parameter: &Real,
    policy: &CurvePolicy,
) -> Option<()> {
    for index in 0..parameters.len() {
        match crate::classify::compare_reals(parameter, &parameters[index], policy)? {
            Ordering::Less => {
                parameters.insert(index, parameter.clone());
                return Some(());
            }
            Ordering::Equal => return Some(()),
            Ordering::Greater => {}
        }
    }
    parameters.push(parameter.clone());
    Some(())
}

fn quadratic_fragment_report(
    status: BezierBooleanFragmentConstructionStatus,
    readiness_status: BezierBooleanConstructionReadinessStatus,
    source_parameter_count: usize,
    endpoint_parameter_count: usize,
    out_of_range_parameter_count: usize,
    inserted_parameters: Vec<Real>,
    fragments: Vec<QuadraticBezier2>,
) -> BezierBooleanQuadraticFragmentReport2 {
    BezierBooleanQuadraticFragmentReport2 {
        status,
        readiness_status,
        source_parameter_count,
        endpoint_parameter_count,
        out_of_range_parameter_count,
        inserted_parameter_count: inserted_parameters.len(),
        inserted_parameters,
        fragments,
    }
}

fn cubic_fragment_report(
    status: BezierBooleanFragmentConstructionStatus,
    readiness_status: BezierBooleanConstructionReadinessStatus,
    source_parameter_count: usize,
    endpoint_parameter_count: usize,
    out_of_range_parameter_count: usize,
    inserted_parameters: Vec<Real>,
    fragments: Vec<CubicBezier2>,
) -> BezierBooleanCubicFragmentReport2 {
    BezierBooleanCubicFragmentReport2 {
        status,
        readiness_status,
        source_parameter_count,
        endpoint_parameter_count,
        out_of_range_parameter_count,
        inserted_parameter_count: inserted_parameters.len(),
        inserted_parameters,
        fragments,
    }
}

fn rational_quadratic_fragment_report(
    status: BezierBooleanFragmentConstructionStatus,
    readiness_status: BezierBooleanConstructionReadinessStatus,
    source_parameter_count: usize,
    endpoint_parameter_count: usize,
    out_of_range_parameter_count: usize,
    inserted_parameters: Vec<Real>,
    fragments: Vec<RationalQuadraticBezier2>,
) -> BezierBooleanRationalQuadraticFragmentReport2 {
    BezierBooleanRationalQuadraticFragmentReport2 {
        status,
        readiness_status,
        source_parameter_count,
        endpoint_parameter_count,
        out_of_range_parameter_count,
        inserted_parameter_count: inserted_parameters.len(),
        inserted_parameters,
        fragments,
    }
}

fn quadratic_fragment_endpoints(fragments: &[QuadraticBezier2]) -> Vec<(Point2, Point2)> {
    fragments
        .iter()
        .map(|fragment| (fragment.start().clone(), fragment.end().clone()))
        .collect()
}

fn cubic_fragment_endpoints(fragments: &[CubicBezier2]) -> Vec<(Point2, Point2)> {
    fragments
        .iter()
        .map(|fragment| (fragment.start().clone(), fragment.end().clone()))
        .collect()
}

fn rational_quadratic_fragment_endpoints(
    fragments: &[RationalQuadraticBezier2],
) -> Vec<(Point2, Point2)> {
    fragments
        .iter()
        .map(|fragment| (fragment.start().clone(), fragment.end().clone()))
        .collect()
}

fn fragment_chain_gap_count(endpoints: &[(Point2, Point2)]) -> usize {
    endpoints
        .windows(2)
        .filter(|pair| pair[0].1 != pair[1].0)
        .count()
}

fn count_directed_loop_closure(
    fragments: &[BezierBooleanDirectedLoopFragment2],
) -> (usize, usize, usize) {
    let Some(first) = fragments.first() else {
        return (0, 0, 0);
    };

    let mut closed_loop_count = 0;
    let mut open_chain_count = 0;
    let mut adjacency_gap_count = 0;
    let mut chain_start = first.start.clone();
    let mut previous_end = first.end.clone();
    let mut chain_is_open = true;

    if previous_end == chain_start {
        closed_loop_count += 1;
        chain_is_open = false;
    }

    for fragment in fragments.iter().skip(1) {
        if chain_is_open && previous_end != fragment.start {
            adjacency_gap_count += 1;
            open_chain_count += 1;
            chain_start = fragment.start.clone();
        } else if !chain_is_open {
            chain_start = fragment.start.clone();
            chain_is_open = true;
        }

        previous_end = fragment.end.clone();
        if previous_end == chain_start {
            closed_loop_count += 1;
            chain_is_open = false;
        }
    }

    if chain_is_open {
        open_chain_count += 1;
    }

    (closed_loop_count, open_chain_count, adjacency_gap_count)
}

fn collect_output_loop_ranges(
    fragments: &[BezierBooleanDirectedLoopFragment2],
) -> Vec<BezierBooleanOutputLoop2> {
    let Some(first) = fragments.first() else {
        return Vec::new();
    };

    let mut loops = Vec::new();
    let mut loop_start_index = 0;
    let mut loop_anchor = first.start.clone();

    for (index, fragment) in fragments.iter().enumerate() {
        if fragment.end == loop_anchor {
            loops.push(BezierBooleanOutputLoop2 {
                first_directed_fragment_index: loop_start_index,
                directed_fragment_count: index + 1 - loop_start_index,
                anchor: loop_anchor.clone(),
            });
            loop_start_index = index + 1;
            if let Some(next) = fragments.get(loop_start_index) {
                loop_anchor = next.start.clone();
            }
        }
    }

    loops
}

fn output_loop_ranges_match_multi_cycle_walk(
    output: &BezierBooleanOutputLoopReport2,
    multi_cycle: &BezierBooleanLoopGraphMultiCycleWalkReport2,
) -> bool {
    if multi_cycle.status != BezierBooleanLoopGraphMultiCycleWalkStatus::Ready {
        return false;
    }
    if output.loops.len() != multi_cycle.cycle_count
        || multi_cycle.cycle_walk_start_offsets.len() != multi_cycle.cycle_count
        || multi_cycle.cycle_step_counts.len() != multi_cycle.cycle_count
    {
        return false;
    }

    output
        .loops
        .iter()
        .zip(
            multi_cycle
                .cycle_walk_start_offsets
                .iter()
                .zip(&multi_cycle.cycle_step_counts),
        )
        .all(|(output_loop, (start_offset, step_count))| {
            output_loop.first_directed_fragment_index == *start_offset
                && output_loop.directed_fragment_count == *step_count
        })
}

fn split_quadratic_at_sorted_parameters(
    curve: &QuadraticBezier2,
    parameters: &[Real],
) -> Vec<QuadraticBezier2> {
    let mut fragments = Vec::with_capacity(parameters.len() + 1);
    let mut current = curve.clone();
    let mut previous = Real::zero();
    let one = Real::one();

    for parameter in parameters {
        let denominator = &one - &previous;
        let local = ((parameter - &previous) / &denominator)
            .expect("interior sorted split parameters leave nonzero remaining domain");
        let (left, right) = split_quadratic_at_local_parameter(&current, local);
        fragments.push(left);
        current = right;
        previous = parameter.clone();
    }
    fragments.push(current);
    fragments
}

fn split_rational_quadratic_at_sorted_parameters(
    curve: &RationalQuadraticBezier2,
    parameters: &[Real],
    policy: &CurvePolicy,
) -> Classification<Vec<RationalQuadraticBezier2>> {
    let mut fragments = Vec::with_capacity(parameters.len() + 1);
    let mut current = curve.clone();
    let mut previous = Real::zero();
    let one = Real::one();

    for parameter in parameters {
        let denominator = &one - &previous;
        let local = ((parameter - &previous) / &denominator)
            .expect("interior sorted split parameters leave nonzero remaining domain");
        let (left, right) =
            match split_rational_quadratic_at_local_parameter(&current, local, policy) {
                Classification::Decided(pair) => pair,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
        fragments.push(left);
        current = right;
        previous = parameter.clone();
    }
    fragments.push(current);
    Classification::Decided(fragments)
}

fn split_cubic_at_sorted_parameters(
    curve: &CubicBezier2,
    parameters: &[Real],
) -> Vec<CubicBezier2> {
    let mut fragments = Vec::with_capacity(parameters.len() + 1);
    let mut current = curve.clone();
    let mut previous = Real::zero();
    let one = Real::one();

    for parameter in parameters {
        let denominator = &one - &previous;
        let local = ((parameter - &previous) / &denominator)
            .expect("interior sorted split parameters leave nonzero remaining domain");
        let (left, right) = split_cubic_at_local_parameter(&current, local);
        fragments.push(left);
        current = right;
        previous = parameter.clone();
    }
    fragments.push(current);
    fragments
}

#[derive(Clone)]
struct HomogeneousPoint2 {
    xw: Real,
    yw: Real,
    w: Real,
}

fn split_rational_quadratic_at_local_parameter(
    curve: &RationalQuadraticBezier2,
    parameter: Real,
    policy: &CurvePolicy,
) -> Classification<(RationalQuadraticBezier2, RationalQuadraticBezier2)> {
    let h0 = homogeneous_control(curve.start(), curve.start_weight());
    let h1 = homogeneous_control(curve.control(), curve.control_weight());
    let h2 = homogeneous_control(curve.end(), curve.end_weight());
    let h01 = homogeneous_lerp(&h0, &h1, parameter.clone());
    let h12 = homogeneous_lerp(&h1, &h2, parameter.clone());
    let h012 = homogeneous_lerp(&h01, &h12, parameter);

    let left = match rational_from_homogeneous([h0.clone(), h01.clone(), h012.clone()], policy) {
        Classification::Decided(curve) => curve,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let right = match rational_from_homogeneous([h012, h12, h2], policy) {
        Classification::Decided(curve) => curve,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    Classification::Decided((left, right))
}

fn homogeneous_control(point: &Point2, weight: &Real) -> HomogeneousPoint2 {
    HomogeneousPoint2 {
        xw: weight * point.x(),
        yw: weight * point.y(),
        w: weight.clone(),
    }
}

fn homogeneous_lerp(
    first: &HomogeneousPoint2,
    second: &HomogeneousPoint2,
    parameter: Real,
) -> HomogeneousPoint2 {
    let one_minus_parameter = Real::one() - &parameter;
    HomogeneousPoint2 {
        xw: &one_minus_parameter * &first.xw + &parameter * &second.xw,
        yw: &one_minus_parameter * &first.yw + &parameter * &second.yw,
        w: &one_minus_parameter * &first.w + parameter * &second.w,
    }
}

fn rational_from_homogeneous(
    controls: [HomogeneousPoint2; 3],
    policy: &CurvePolicy,
) -> Classification<RationalQuadraticBezier2> {
    let [start, control, end] = controls;
    let start = match affine_from_homogeneous(start, policy) {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let control = match affine_from_homogeneous(control, policy) {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let end = match affine_from_homogeneous(end, policy) {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };

    RationalQuadraticBezier2::try_new(start.0, control.0, end.0, start.1, control.1, end.1)
        .map(Classification::Decided)
        .unwrap_or(Classification::Uncertain(UncertaintyReason::Boundary))
}

fn affine_from_homogeneous(
    point: HomogeneousPoint2,
    policy: &CurvePolicy,
) -> Classification<(Point2, Real)> {
    match crate::classify::is_zero(&point.w, policy) {
        Some(true) => Classification::Uncertain(UncertaintyReason::Boundary),
        Some(false) => {
            let Ok(x) = point.xw / &point.w else {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            };
            let Ok(y) = point.yw / &point.w else {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            };
            Classification::Decided((Point2::new(x, y), point.w))
        }
        None => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn split_quadratic_at_local_parameter(
    curve: &QuadraticBezier2,
    parameter: Real,
) -> (QuadraticBezier2, QuadraticBezier2) {
    let p01 = curve.start().lerp(curve.control(), parameter.clone());
    let p12 = curve.control().lerp(curve.end(), parameter.clone());
    let p012 = p01.lerp(&p12, parameter);
    (
        QuadraticBezier2::new(curve.start().clone(), p01, p012.clone()),
        QuadraticBezier2::new(p012, p12, curve.end().clone()),
    )
}

fn split_cubic_at_local_parameter(
    curve: &CubicBezier2,
    parameter: Real,
) -> (CubicBezier2, CubicBezier2) {
    let p01 = curve.start().lerp(curve.control1(), parameter.clone());
    let p12 = curve.control1().lerp(curve.control2(), parameter.clone());
    let p23 = curve.control2().lerp(curve.end(), parameter.clone());
    let p012 = p01.lerp(&p12, parameter.clone());
    let p123 = p12.lerp(&p23, parameter.clone());
    let p0123 = p012.lerp(&p123, parameter);
    (
        CubicBezier2::new(curve.start().clone(), p01, p012, p0123.clone()),
        CubicBezier2::new(p0123, p123, p23, curve.end().clone()),
    )
}

impl BezierBooleanSplitInsertionReport2 {
    /// Returns true when at least one interior split may be inserted.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanSplitInsertionStatus::Ready
    }

    /// Returns true when the report preserves a blocker or invalid payload.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanSplitInsertionStatus::Blocked
                | BezierBooleanSplitInsertionStatus::InvalidParameterDomain
        )
    }
}

impl BezierBooleanSplitPlanAuditReport2 {
    /// Returns true when the split plan may be consumed by insertion code.
    pub fn is_valid(&self) -> bool {
        self.status == BezierBooleanSplitPlanAuditStatus::Valid
    }
}

impl BezierBooleanSplitPlanReport2 {
    /// Builds a split plan from a global path scheduler report.
    ///
    /// The method does not sort, deduplicate, or reinterpret parameters:
    /// preserving event multiplicity is important for later overlap/contact
    /// policy. A future fragment mutator may decide how to coalesce identical
    /// local split parameters after it knows curve ownership.
    pub fn from_scheduler(scheduler: &BezierBooleanPathSchedulerReport2) -> Self {
        if scheduler.status == BezierBooleanPathSchedulerStatus::Empty {
            return Self {
                status: BezierBooleanSplitPlanStatus::Empty,
                scheduler_status: scheduler.status,
                first_curve_parameters: Vec::new(),
                second_curve_parameters: Vec::new(),
                shared_range_parameters: Vec::new(),
                relation_event_count: 0,
                range_event_count: 0,
                uncertainty_reason: scheduler.uncertainty_reason,
            };
        }

        if scheduler.status != BezierBooleanPathSchedulerStatus::SplitEventsReady {
            return Self {
                status: BezierBooleanSplitPlanStatus::Blocked,
                scheduler_status: scheduler.status,
                first_curve_parameters: Vec::new(),
                second_curve_parameters: Vec::new(),
                shared_range_parameters: Vec::new(),
                relation_event_count: 0,
                range_event_count: 0,
                uncertainty_reason: scheduler.uncertainty_reason,
            };
        }

        Self {
            status: BezierBooleanSplitPlanStatus::Ready,
            scheduler_status: scheduler.status,
            first_curve_parameters: scheduler
                .relation_point_events
                .iter()
                .map(|event| event.first_param.clone())
                .collect(),
            second_curve_parameters: scheduler
                .relation_point_events
                .iter()
                .map(|event| event.second_param.clone())
                .collect(),
            shared_range_parameters: scheduler.range_split_parameters.clone(),
            relation_event_count: scheduler.relation_point_events.len(),
            range_event_count: scheduler.range_split_parameters.len(),
            uncertainty_reason: None,
        }
    }

    /// Returns true when exact split parameters can be inserted.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanSplitPlanStatus::Ready
    }

    /// Returns true when the plan preserves a scheduler blocker.
    pub fn has_blockers(&self) -> bool {
        self.status == BezierBooleanSplitPlanStatus::Blocked
    }

    /// Audits the split plan's parameter-domain invariant.
    ///
    /// Ready plans must contain only parameters in the closed unit interval.
    /// Empty and blocked plans are decided audit states, but they are not
    /// reported as valid insertion payloads.
    pub fn audit(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierBooleanSplitPlanAuditReport2> {
        match self.status {
            BezierBooleanSplitPlanStatus::Empty => {
                return Classification::Decided(BezierBooleanSplitPlanAuditReport2 {
                    status: BezierBooleanSplitPlanAuditStatus::Empty,
                    checked_parameter_count: 0,
                    out_of_range_parameter_count: 0,
                });
            }
            BezierBooleanSplitPlanStatus::Blocked => {
                return Classification::Decided(BezierBooleanSplitPlanAuditReport2 {
                    status: BezierBooleanSplitPlanAuditStatus::Blocked,
                    checked_parameter_count: 0,
                    out_of_range_parameter_count: 0,
                });
            }
            BezierBooleanSplitPlanStatus::Ready => {}
        }

        let mut checked = 0;
        let mut out_of_range = 0;
        for parameter in self
            .first_curve_parameters
            .iter()
            .chain(self.second_curve_parameters.iter())
            .chain(self.shared_range_parameters.iter())
        {
            checked += 1;
            match parameter_in_unit_interval(parameter, policy) {
                Some(true) => {}
                Some(false) => out_of_range += 1,
                None => return Classification::Uncertain(UncertaintyReason::Ordering),
            }
        }

        Classification::Decided(BezierBooleanSplitPlanAuditReport2 {
            status: if out_of_range == 0 {
                BezierBooleanSplitPlanAuditStatus::Valid
            } else {
                BezierBooleanSplitPlanAuditStatus::ParameterOutOfRange
            },
            checked_parameter_count: checked,
            out_of_range_parameter_count: out_of_range,
        })
    }

    /// Classifies audited split parameters into interior insertion work and endpoint no-ops.
    ///
    /// This is the last report-only step before fragment mutation. It accepts
    /// only plans whose parameter-domain audit is valid; blocked, empty, and
    /// invalid plans are preserved as non-insertion states.
    pub fn insertion_report(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierBooleanSplitInsertionReport2> {
        let audit = match self.audit(policy) {
            Classification::Decided(audit) => audit,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        match audit.status {
            BezierBooleanSplitPlanAuditStatus::Empty => {
                return Classification::Decided(BezierBooleanSplitInsertionReport2 {
                    status: BezierBooleanSplitInsertionStatus::Empty,
                    first_curve_interior_parameters: Vec::new(),
                    second_curve_interior_parameters: Vec::new(),
                    shared_range_interior_parameters: Vec::new(),
                    endpoint_parameter_count: 0,
                    interior_parameter_count: 0,
                    out_of_range_parameter_count: 0,
                });
            }
            BezierBooleanSplitPlanAuditStatus::Blocked => {
                return Classification::Decided(BezierBooleanSplitInsertionReport2 {
                    status: BezierBooleanSplitInsertionStatus::Blocked,
                    first_curve_interior_parameters: Vec::new(),
                    second_curve_interior_parameters: Vec::new(),
                    shared_range_interior_parameters: Vec::new(),
                    endpoint_parameter_count: 0,
                    interior_parameter_count: 0,
                    out_of_range_parameter_count: 0,
                });
            }
            BezierBooleanSplitPlanAuditStatus::ParameterOutOfRange => {
                return Classification::Decided(BezierBooleanSplitInsertionReport2 {
                    status: BezierBooleanSplitInsertionStatus::InvalidParameterDomain,
                    first_curve_interior_parameters: Vec::new(),
                    second_curve_interior_parameters: Vec::new(),
                    shared_range_interior_parameters: Vec::new(),
                    endpoint_parameter_count: 0,
                    interior_parameter_count: 0,
                    out_of_range_parameter_count: audit.out_of_range_parameter_count,
                });
            }
            BezierBooleanSplitPlanAuditStatus::Valid => {}
        }

        let mut endpoint_count = 0;
        let mut out_of_range_count = 0;
        let first = match collect_interior_parameters(
            &self.first_curve_parameters,
            policy,
            &mut endpoint_count,
            &mut out_of_range_count,
        ) {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let second = match collect_interior_parameters(
            &self.second_curve_parameters,
            policy,
            &mut endpoint_count,
            &mut out_of_range_count,
        ) {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let shared = match collect_interior_parameters(
            &self.shared_range_parameters,
            policy,
            &mut endpoint_count,
            &mut out_of_range_count,
        ) {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let interior_count = first.len() + second.len() + shared.len();

        Classification::Decided(BezierBooleanSplitInsertionReport2 {
            status: if interior_count == 0 {
                BezierBooleanSplitInsertionStatus::NoInteriorSplits
            } else {
                BezierBooleanSplitInsertionStatus::Ready
            },
            first_curve_interior_parameters: first,
            second_curve_interior_parameters: second,
            shared_range_interior_parameters: shared,
            endpoint_parameter_count: endpoint_count,
            interior_parameter_count: interior_count,
            out_of_range_parameter_count: out_of_range_count,
        })
    }
}

impl BezierBooleanAlgebraicParameterHandoffReport2 {
    /// Builds algebraic parameter events from represented `hypersolve` roots.
    ///
    /// The current implementation maps univariate represented roots to the
    /// shared monotone-range lane because relation-region roots still require
    /// two curve parameters plus contact metadata. That is intentional: this
    /// report introduces the exact algebraic parameter object without
    /// pretending that a one-variable solver row is already a full curve/curve
    /// event. Sederberg and Nishita's Bezier clipping cells ("Curve
    /// intersection using Bezier clipping," 1990) and future
    /// resultant/Krawczyk pair isolation can add first/second-curve roles once
    /// both parameters are represented.
    pub fn from_hypersolve_algebraic_root_reports(
        scheduler: &BezierBooleanPathSchedulerReport2,
        algebraic_reports: &[AlgebraicRootRepresentationReport],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let handoff = BezierBooleanRootIsolationHandoffReport2::from_path_scheduler(scheduler);
        let required = handoff.range_isolating_span_count;
        let mut report = Self {
            status: BezierBooleanAlgebraicParameterHandoffStatus::HandoffBlocked,
            handoff_status: handoff.status,
            scheduler_status: scheduler.status,
            events: Vec::new(),
            required_algebraic_parameter_count: required,
            supplied_algebraic_parameter_count: 0,
            exact_rational_parameter_count: 0,
            interval_parameter_count: 0,
            missing_algebraic_parameter_count: 0,
            invalid_algebraic_evidence_count: 0,
            out_of_range_parameter_count: 0,
            blocker_count: handoff.blocker_count,
            uncertainty_reason: scheduler.uncertainty_reason,
        };

        match handoff.status {
            BezierBooleanRootIsolationHandoffStatus::Empty => {
                report.status = BezierBooleanAlgebraicParameterHandoffStatus::Empty;
                report.blocker_count = 0;
                return Classification::Decided(report);
            }
            BezierBooleanRootIsolationHandoffStatus::NotNeeded => {
                report.status = BezierBooleanAlgebraicParameterHandoffStatus::NotNeeded;
                report.blocker_count = 0;
                return Classification::Decided(report);
            }
            BezierBooleanRootIsolationHandoffStatus::SplitEventsReady => {
                report.status =
                    BezierBooleanAlgebraicParameterHandoffStatus::RationalSplitEventsReady;
                report.blocker_count = 0;
                return Classification::Decided(report);
            }
            BezierBooleanRootIsolationHandoffStatus::ReadyForHypersolve => {}
            BezierBooleanRootIsolationHandoffStatus::BlockedByParameterRecovery
            | BezierBooleanRootIsolationHandoffStatus::BlockedByOverlapResolver
            | BezierBooleanRootIsolationHandoffStatus::BlockedByUnresolved
            | BezierBooleanRootIsolationHandoffStatus::BlockedByContactClassification
            | BezierBooleanRootIsolationHandoffStatus::BlockedByMonotoneDecomposition
            | BezierBooleanRootIsolationHandoffStatus::BlockedByUncertainty => {
                report.blocker_count = handoff.blocker_count.max(1);
                return Classification::Decided(report);
            }
        }

        for (report_index, algebraic_report) in algebraic_reports.iter().enumerate() {
            match algebraic_report.status {
                AlgebraicRootRepresentationStatus::Represented => {}
                AlgebraicRootRepresentationStatus::NoRealRoots => continue,
                AlgebraicRootRepresentationStatus::UnsupportedIsolationStatus
                | AlgebraicRootRepresentationStatus::MissingSymbol
                | AlgebraicRootRepresentationStatus::MissingPolynomial
                | AlgebraicRootRepresentationStatus::InvalidEvidence => {
                    report.invalid_algebraic_evidence_count += 1;
                    continue;
                }
            }

            for (root_index, root) in algebraic_report.roots.iter().enumerate() {
                report.supplied_algebraic_parameter_count += 1;
                if !root.is_valid() {
                    report.invalid_algebraic_evidence_count += 1;
                    continue;
                }
                match algebraic_root_interval_in_unit_domain(root, policy) {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        report.out_of_range_parameter_count += 1;
                        continue;
                    }
                    Classification::Uncertain(reason) => return Classification::Uncertain(reason),
                }
                if root.exact_rational_witness().is_some() {
                    report.exact_rational_parameter_count += 1;
                } else {
                    report.interval_parameter_count += 1;
                }
                report.events.push(BezierBooleanAlgebraicParameterEvent2 {
                    role: BezierBooleanAlgebraicParameterRole::SharedRange,
                    report_index,
                    root_index,
                    root: root.clone(),
                });
            }
        }

        report.missing_algebraic_parameter_count = required.saturating_sub(report.events.len());
        report.blocker_count = report.missing_algebraic_parameter_count
            + report.invalid_algebraic_evidence_count
            + report.out_of_range_parameter_count;
        report.status = if report.invalid_algebraic_evidence_count > 0 {
            BezierBooleanAlgebraicParameterHandoffStatus::InvalidAlgebraicEvidence
        } else if report.out_of_range_parameter_count > 0 {
            BezierBooleanAlgebraicParameterHandoffStatus::InvalidParameterDomain
        } else if report.missing_algebraic_parameter_count > 0 {
            BezierBooleanAlgebraicParameterHandoffStatus::MissingAlgebraicRoots
        } else {
            BezierBooleanAlgebraicParameterHandoffStatus::Ready
        };
        Classification::Decided(report)
    }

    /// Returns true when represented algebraic parameters can feed a future
    /// algebraic split/order layer.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanAlgebraicParameterHandoffStatus::Ready
    }

    /// Returns true when algebraic-parameter handoff retained blockers.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanAlgebraicParameterHandoffStatus::HandoffBlocked
                | BezierBooleanAlgebraicParameterHandoffStatus::MissingAlgebraicRoots
                | BezierBooleanAlgebraicParameterHandoffStatus::InvalidAlgebraicEvidence
                | BezierBooleanAlgebraicParameterHandoffStatus::InvalidParameterDomain
        )
    }

    /// Audits retained algebraic parameter events before future exact consumers.
    ///
    /// The current univariate algebraic-root handoff can only certify shared
    /// monotone-range parameters. First/second curve roles require paired
    /// curve-curve event evidence and therefore remain unsupported here. This
    /// check also validates count fields and replays the represented-root
    /// validation/unit-domain obligations so later layers do not trust stale or
    /// forged algebraic events.
    pub fn audit(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<BezierBooleanAlgebraicParameterAuditReport2> {
        let mut audit = BezierBooleanAlgebraicParameterAuditReport2 {
            status: BezierBooleanAlgebraicParameterAuditStatus::HandoffBlocked,
            handoff_status: self.status,
            checked_event_count: 0,
            count_mismatch_count: 0,
            unsupported_role_count: 0,
            invalid_algebraic_evidence_count: 0,
            out_of_range_parameter_count: 0,
            blocker_count: 0,
        };

        match self.status {
            BezierBooleanAlgebraicParameterHandoffStatus::Empty => {
                audit.status = BezierBooleanAlgebraicParameterAuditStatus::Empty;
                return Classification::Decided(audit);
            }
            BezierBooleanAlgebraicParameterHandoffStatus::NotNeeded => {
                audit.status = BezierBooleanAlgebraicParameterAuditStatus::NotNeeded;
                return Classification::Decided(audit);
            }
            BezierBooleanAlgebraicParameterHandoffStatus::RationalSplitEventsReady => {
                audit.status = BezierBooleanAlgebraicParameterAuditStatus::RationalSplitEventsReady;
                return Classification::Decided(audit);
            }
            BezierBooleanAlgebraicParameterHandoffStatus::Ready => {}
            BezierBooleanAlgebraicParameterHandoffStatus::HandoffBlocked
            | BezierBooleanAlgebraicParameterHandoffStatus::MissingAlgebraicRoots
            | BezierBooleanAlgebraicParameterHandoffStatus::InvalidAlgebraicEvidence
            | BezierBooleanAlgebraicParameterHandoffStatus::InvalidParameterDomain => {
                audit.blocker_count = self.blocker_count.max(1);
                return Classification::Decided(audit);
            }
        }

        audit.checked_event_count = self.events.len();
        if self.required_algebraic_parameter_count != self.events.len() {
            audit.count_mismatch_count += 1;
        }
        if self.supplied_algebraic_parameter_count < self.events.len() {
            audit.count_mismatch_count += 1;
        }
        let exact_count = self
            .events
            .iter()
            .filter(|event| event.root.exact_rational_witness().is_some())
            .count();
        let interval_count = self.events.len().saturating_sub(exact_count);
        if self.exact_rational_parameter_count != exact_count {
            audit.count_mismatch_count += 1;
        }
        if self.interval_parameter_count != interval_count {
            audit.count_mismatch_count += 1;
        }

        for event in &self.events {
            if event.role != BezierBooleanAlgebraicParameterRole::SharedRange {
                audit.unsupported_role_count += 1;
            }
            if !event.root.is_valid() {
                audit.invalid_algebraic_evidence_count += 1;
                continue;
            }
            match algebraic_root_interval_in_unit_domain(&event.root, policy) {
                Classification::Decided(true) => {}
                Classification::Decided(false) => audit.out_of_range_parameter_count += 1,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }

        audit.blocker_count = audit.count_mismatch_count
            + audit.unsupported_role_count
            + audit.invalid_algebraic_evidence_count
            + audit.out_of_range_parameter_count;
        audit.status = if audit.count_mismatch_count > 0 {
            BezierBooleanAlgebraicParameterAuditStatus::CountMismatch
        } else if audit.unsupported_role_count > 0 {
            BezierBooleanAlgebraicParameterAuditStatus::UnsupportedRole
        } else if audit.invalid_algebraic_evidence_count > 0 {
            BezierBooleanAlgebraicParameterAuditStatus::InvalidAlgebraicEvidence
        } else if audit.out_of_range_parameter_count > 0 {
            BezierBooleanAlgebraicParameterAuditStatus::InvalidParameterDomain
        } else {
            BezierBooleanAlgebraicParameterAuditStatus::Valid
        };

        Classification::Decided(audit)
    }
}

impl BezierBooleanAlgebraicParameterAuditReport2 {
    /// Returns true when retained algebraic parameter events passed audit.
    pub fn is_valid(&self) -> bool {
        self.status == BezierBooleanAlgebraicParameterAuditStatus::Valid
    }

    /// Returns true when the audit retained blockers or stale evidence.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanAlgebraicParameterAuditStatus::HandoffBlocked
                | BezierBooleanAlgebraicParameterAuditStatus::CountMismatch
                | BezierBooleanAlgebraicParameterAuditStatus::UnsupportedRole
                | BezierBooleanAlgebraicParameterAuditStatus::InvalidAlgebraicEvidence
                | BezierBooleanAlgebraicParameterAuditStatus::InvalidParameterDomain
        )
    }
}

impl BezierBooleanAlgebraicParameterReadinessReport2 {
    /// Packages an audited algebraic-parameter handoff for future exact consumers.
    pub fn from_handoff(
        handoff: &BezierBooleanAlgebraicParameterHandoffReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let audit = match handoff.audit(policy) {
            Classification::Decided(audit) => audit,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        match audit.status {
            BezierBooleanAlgebraicParameterAuditStatus::Empty => {
                return Classification::Decided(Self::blocked_or_empty(
                    BezierBooleanAlgebraicParameterReadinessStatus::Empty,
                    audit.status,
                    Vec::new(),
                    0,
                ));
            }
            BezierBooleanAlgebraicParameterAuditStatus::NotNeeded => {
                return Classification::Decided(Self::blocked_or_empty(
                    BezierBooleanAlgebraicParameterReadinessStatus::NotNeeded,
                    audit.status,
                    Vec::new(),
                    0,
                ));
            }
            BezierBooleanAlgebraicParameterAuditStatus::RationalSplitEventsReady => {
                return Classification::Decided(Self::blocked_or_empty(
                    BezierBooleanAlgebraicParameterReadinessStatus::RationalSplitEventsReady,
                    audit.status,
                    Vec::new(),
                    0,
                ));
            }
            BezierBooleanAlgebraicParameterAuditStatus::Valid => {}
            BezierBooleanAlgebraicParameterAuditStatus::HandoffBlocked
            | BezierBooleanAlgebraicParameterAuditStatus::CountMismatch
            | BezierBooleanAlgebraicParameterAuditStatus::UnsupportedRole
            | BezierBooleanAlgebraicParameterAuditStatus::InvalidAlgebraicEvidence
            | BezierBooleanAlgebraicParameterAuditStatus::InvalidParameterDomain => {
                return Classification::Decided(Self::blocked_or_empty(
                    BezierBooleanAlgebraicParameterReadinessStatus::AuditBlocked,
                    audit.status,
                    Vec::new(),
                    audit.blocker_count.max(1),
                ));
            }
        }

        let exact_rational_events = handoff
            .events
            .iter()
            .filter(|event| event.root.exact_rational_witness().is_some())
            .cloned()
            .collect::<Vec<_>>();
        let interval_events = handoff
            .events
            .iter()
            .filter(|event| event.root.exact_rational_witness().is_none())
            .cloned()
            .collect::<Vec<_>>();
        Classification::Decided(Self {
            status: BezierBooleanAlgebraicParameterReadinessStatus::Ready,
            audit_status: audit.status,
            events: handoff.events.clone(),
            event_count: handoff.events.len(),
            exact_rational_event_count: exact_rational_events.len(),
            interval_event_count: interval_events.len(),
            exact_rational_events,
            interval_events,
            blocker_count: 0,
        })
    }

    /// Returns true when algebraic parameter events are ready for future exact consumers.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanAlgebraicParameterReadinessStatus::Ready
    }

    /// Returns true when readiness retained audit blockers.
    pub fn has_blockers(&self) -> bool {
        self.status == BezierBooleanAlgebraicParameterReadinessStatus::AuditBlocked
    }

    fn blocked_or_empty(
        status: BezierBooleanAlgebraicParameterReadinessStatus,
        audit_status: BezierBooleanAlgebraicParameterAuditStatus,
        events: Vec<BezierBooleanAlgebraicParameterEvent2>,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            audit_status,
            events,
            exact_rational_events: Vec::new(),
            interval_events: Vec::new(),
            event_count: 0,
            exact_rational_event_count: 0,
            interval_event_count: 0,
            blocker_count,
        }
    }
}

impl BezierBooleanAlgebraicParameterOrderingReport2 {
    /// Orders retained algebraic parameter events with `hypersolve` certificates.
    ///
    /// The implementation uses a stable insertion sort because the expected
    /// algebraic frontier at this handoff is small and because each comparison
    /// is itself proof-bearing. Equal roots keep their original relative order.
    /// This is deliberately a report constructor, not a fragment mutator:
    /// comparison failures remain explicit blockers until stronger isolation,
    /// algebraic-number arithmetic, or curve-pair evidence is supplied.
    pub fn from_readiness(
        readiness: &BezierBooleanAlgebraicParameterReadinessReport2,
        mut config: AlgebraicRootRefinementComparisonConfig,
        policy: &CurvePolicy,
    ) -> Self {
        config.policy = policy.predicate_policy;
        match readiness.status {
            BezierBooleanAlgebraicParameterReadinessStatus::Empty => {
                return Self::blocked_or_empty(
                    BezierBooleanAlgebraicParameterOrderingStatus::Empty,
                    readiness,
                    0,
                );
            }
            BezierBooleanAlgebraicParameterReadinessStatus::NotNeeded => {
                return Self::blocked_or_empty(
                    BezierBooleanAlgebraicParameterOrderingStatus::NotNeeded,
                    readiness,
                    0,
                );
            }
            BezierBooleanAlgebraicParameterReadinessStatus::RationalSplitEventsReady => {
                return Self::blocked_or_empty(
                    BezierBooleanAlgebraicParameterOrderingStatus::RationalSplitEventsReady,
                    readiness,
                    0,
                );
            }
            BezierBooleanAlgebraicParameterReadinessStatus::AuditBlocked => {
                return Self::blocked_or_empty(
                    BezierBooleanAlgebraicParameterOrderingStatus::ReadinessBlocked,
                    readiness,
                    readiness.blocker_count.max(1),
                );
            }
            BezierBooleanAlgebraicParameterReadinessStatus::Ready => {}
        }

        let mut sorted_indices: Vec<usize> = Vec::with_capacity(readiness.events.len());
        let mut comparisons = Vec::new();
        let mut blocked_comparison_count = 0;
        let mut refinement_round_count = 0;

        'events: for event_index in 0..readiness.events.len() {
            for insertion_index in 0..sorted_indices.len() {
                let existing_index = sorted_indices[insertion_index];
                let comparison = compare_algebraic_root_representations_with_refinement(
                    &readiness.events[event_index].root,
                    &readiness.events[existing_index].root,
                    config.clone(),
                );
                refinement_round_count += comparison.refinement_rounds;
                let comparison_record = algebraic_parameter_ordering_comparison(
                    event_index,
                    existing_index,
                    &comparison,
                );
                let Some(ordering) = comparison_record.ordering else {
                    blocked_comparison_count += 1;
                    comparisons.push(comparison_record);
                    continue;
                };
                let status = comparison_record.comparison_status.clone();
                comparisons.push(comparison_record);
                if !algebraic_comparison_status_is_ordered(status) {
                    blocked_comparison_count += 1;
                    continue;
                }
                if ordering == Ordering::Less {
                    sorted_indices.insert(insertion_index, event_index);
                    continue 'events;
                }
            }
            sorted_indices.push(event_index);
        }

        if blocked_comparison_count > 0 {
            return Self {
                status: BezierBooleanAlgebraicParameterOrderingStatus::ComparisonBlocked,
                readiness_status: readiness.status,
                events: readiness.events.clone(),
                sorted_event_indices: Vec::new(),
                sorted_events: Vec::new(),
                comparison_count: comparisons.len(),
                event_count: readiness.events.len(),
                comparisons,
                blocked_comparison_count,
                refinement_round_count,
                blocker_count: blocked_comparison_count,
            };
        }

        let sorted_events = sorted_indices
            .iter()
            .map(|index| readiness.events[*index].clone())
            .collect::<Vec<_>>();
        Self {
            status: BezierBooleanAlgebraicParameterOrderingStatus::Ready,
            readiness_status: readiness.status,
            events: readiness.events.clone(),
            sorted_event_indices: sorted_indices,
            sorted_events,
            comparison_count: comparisons.len(),
            event_count: readiness.events.len(),
            comparisons,
            blocked_comparison_count: 0,
            refinement_round_count,
            blocker_count: 0,
        }
    }

    fn blocked_or_empty(
        status: BezierBooleanAlgebraicParameterOrderingStatus,
        readiness: &BezierBooleanAlgebraicParameterReadinessReport2,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            readiness_status: readiness.status,
            events: Vec::new(),
            sorted_event_indices: Vec::new(),
            sorted_events: Vec::new(),
            comparisons: Vec::new(),
            event_count: 0,
            comparison_count: 0,
            blocked_comparison_count: 0,
            refinement_round_count: 0,
            blocker_count,
        }
    }

    /// Returns true when all retained algebraic parameters have certified order.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanAlgebraicParameterOrderingStatus::Ready
    }

    /// Returns true when readiness or comparison evidence blocked ordering.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanAlgebraicParameterOrderingStatus::ReadinessBlocked
                | BezierBooleanAlgebraicParameterOrderingStatus::ComparisonBlocked
        )
    }
}

impl BezierBooleanAlgebraicSplitBridgeReport2 {
    /// Lowers ordered exact rational algebraic roots into a split plan.
    ///
    /// The method preserves the order certified by
    /// [`BezierBooleanAlgebraicParameterOrderingReport2`] and rejects
    /// interval-only roots rather than choosing a sample from their isolating
    /// intervals. At present, algebraic root handoff events only support the
    /// shared monotone-range lane; first/second curve roles remain blockers
    /// until paired curve-curve algebraic parameter evidence exists.
    pub fn from_ordering(
        ordering: &BezierBooleanAlgebraicParameterOrderingReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        match ordering.status {
            BezierBooleanAlgebraicParameterOrderingStatus::Empty => {
                return Classification::Decided(Self::blocked_or_empty(
                    BezierBooleanAlgebraicSplitBridgeStatus::Empty,
                    ordering,
                    0,
                ));
            }
            BezierBooleanAlgebraicParameterOrderingStatus::NotNeeded => {
                return Classification::Decided(Self::blocked_or_empty(
                    BezierBooleanAlgebraicSplitBridgeStatus::NotNeeded,
                    ordering,
                    0,
                ));
            }
            BezierBooleanAlgebraicParameterOrderingStatus::RationalSplitEventsReady => {
                return Classification::Decided(Self::blocked_or_empty(
                    BezierBooleanAlgebraicSplitBridgeStatus::RationalSplitEventsReady,
                    ordering,
                    0,
                ));
            }
            BezierBooleanAlgebraicParameterOrderingStatus::ReadinessBlocked
            | BezierBooleanAlgebraicParameterOrderingStatus::ComparisonBlocked => {
                return Classification::Decided(Self::blocked_or_empty(
                    BezierBooleanAlgebraicSplitBridgeStatus::OrderingBlocked,
                    ordering,
                    ordering.blocker_count.max(1),
                ));
            }
            BezierBooleanAlgebraicParameterOrderingStatus::Ready => {}
        }

        let mut shared_range_parameters = Vec::with_capacity(ordering.sorted_events.len());
        let mut exact_rational_parameter_count = 0;
        let mut non_rational_parameter_count = 0;
        let mut unsupported_role_count = 0;
        for event in &ordering.sorted_events {
            if event.role != BezierBooleanAlgebraicParameterRole::SharedRange {
                unsupported_role_count += 1;
                continue;
            }
            let Some(parameter) = event.root.exact_rational_witness() else {
                non_rational_parameter_count += 1;
                continue;
            };
            shared_range_parameters.push(parameter.clone());
            exact_rational_parameter_count += 1;
        }

        if unsupported_role_count > 0 {
            return Classification::Decided(Self::blocked(
                BezierBooleanAlgebraicSplitBridgeStatus::UnsupportedRole,
                ordering,
                exact_rational_parameter_count,
                non_rational_parameter_count,
                unsupported_role_count,
            ));
        }
        if non_rational_parameter_count > 0 {
            return Classification::Decided(Self::blocked(
                BezierBooleanAlgebraicSplitBridgeStatus::NonRationalParameter,
                ordering,
                exact_rational_parameter_count,
                non_rational_parameter_count,
                unsupported_role_count,
            ));
        }

        let split_plan = BezierBooleanSplitPlanReport2 {
            status: if shared_range_parameters.is_empty() {
                BezierBooleanSplitPlanStatus::Empty
            } else {
                BezierBooleanSplitPlanStatus::Ready
            },
            scheduler_status: BezierBooleanPathSchedulerStatus::NeedsRegionIsolation,
            first_curve_parameters: Vec::new(),
            second_curve_parameters: Vec::new(),
            relation_event_count: 0,
            range_event_count: shared_range_parameters.len(),
            shared_range_parameters,
            uncertainty_reason: None,
        };
        let insertion = match split_plan.insertion_report(policy) {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        Classification::Decided(Self {
            status: BezierBooleanAlgebraicSplitBridgeStatus::Ready,
            ordering_status: ordering.status,
            ordered_event_count: ordering.sorted_events.len(),
            exact_rational_parameter_count,
            non_rational_parameter_count: 0,
            unsupported_role_count: 0,
            blocker_count: 0,
            split_plan,
            insertion,
        })
    }

    fn blocked_or_empty(
        status: BezierBooleanAlgebraicSplitBridgeStatus,
        ordering: &BezierBooleanAlgebraicParameterOrderingReport2,
        blocker_count: usize,
    ) -> Self {
        Self {
            status,
            ordering_status: ordering.status,
            split_plan: algebraic_bridge_empty_split_plan(),
            insertion: algebraic_bridge_empty_insertion(),
            ordered_event_count: 0,
            exact_rational_parameter_count: 0,
            non_rational_parameter_count: 0,
            unsupported_role_count: 0,
            blocker_count,
        }
    }

    fn blocked(
        status: BezierBooleanAlgebraicSplitBridgeStatus,
        ordering: &BezierBooleanAlgebraicParameterOrderingReport2,
        exact_rational_parameter_count: usize,
        non_rational_parameter_count: usize,
        unsupported_role_count: usize,
    ) -> Self {
        Self {
            status,
            ordering_status: ordering.status,
            split_plan: algebraic_bridge_empty_split_plan(),
            insertion: algebraic_bridge_empty_insertion(),
            ordered_event_count: ordering.sorted_events.len(),
            exact_rational_parameter_count,
            non_rational_parameter_count,
            unsupported_role_count,
            blocker_count: non_rational_parameter_count + unsupported_role_count,
        }
    }

    /// Returns true when exact rational algebraic parameters entered split insertion.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanAlgebraicSplitBridgeStatus::Ready
    }

    /// Returns true when ordering, role, or interval-only evidence blocked lowering.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanAlgebraicSplitBridgeStatus::OrderingBlocked
                | BezierBooleanAlgebraicSplitBridgeStatus::UnsupportedRole
                | BezierBooleanAlgebraicSplitBridgeStatus::NonRationalParameter
        )
    }
}

fn algebraic_bridge_empty_split_plan() -> BezierBooleanSplitPlanReport2 {
    BezierBooleanSplitPlanReport2 {
        status: BezierBooleanSplitPlanStatus::Empty,
        scheduler_status: BezierBooleanPathSchedulerStatus::Empty,
        first_curve_parameters: Vec::new(),
        second_curve_parameters: Vec::new(),
        shared_range_parameters: Vec::new(),
        relation_event_count: 0,
        range_event_count: 0,
        uncertainty_reason: None,
    }
}

fn algebraic_bridge_empty_insertion() -> BezierBooleanSplitInsertionReport2 {
    BezierBooleanSplitInsertionReport2 {
        status: BezierBooleanSplitInsertionStatus::Empty,
        first_curve_interior_parameters: Vec::new(),
        second_curve_interior_parameters: Vec::new(),
        shared_range_interior_parameters: Vec::new(),
        endpoint_parameter_count: 0,
        interior_parameter_count: 0,
        out_of_range_parameter_count: 0,
    }
}

fn algebraic_parameter_ordering_comparison(
    left_event_index: usize,
    right_event_index: usize,
    comparison: &AlgebraicRootRefinementComparisonReport,
) -> BezierBooleanAlgebraicParameterOrderingComparison2 {
    BezierBooleanAlgebraicParameterOrderingComparison2 {
        left_event_index,
        right_event_index,
        comparison_status: comparison.comparison.status.clone(),
        ordering: comparison.comparison.ordering,
        refinement_rounds: comparison.refinement_rounds,
    }
}

fn algebraic_comparison_status_is_ordered(status: AlgebraicRootComparisonStatus) -> bool {
    matches!(
        status,
        AlgebraicRootComparisonStatus::Compared | AlgebraicRootComparisonStatus::SameRepresentation
    )
}

fn parameter_in_unit_interval(parameter: &Real, policy: &CurvePolicy) -> Option<bool> {
    let zero = Real::zero();
    let one = Real::one();
    let lower = crate::classify::compare_reals(parameter, &zero, policy)?;
    let upper = crate::classify::compare_reals(parameter, &one, policy)?;
    Some(
        matches!(lower, Ordering::Greater | Ordering::Equal)
            && matches!(upper, Ordering::Less | Ordering::Equal),
    )
}

fn algebraic_root_interval_in_unit_domain(
    root: &AlgebraicRootRepresentation,
    policy: &CurvePolicy,
) -> Classification<bool> {
    let zero = Real::zero();
    let one = Real::one();
    let Some(lower) = crate::classify::compare_reals(&root.interval.lower, &zero, policy) else {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    };
    let Some(upper) = crate::classify::compare_reals(&root.interval.upper, &one, policy) else {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    };
    Classification::Decided(
        matches!(lower, Ordering::Greater | Ordering::Equal)
            && matches!(upper, Ordering::Less | Ordering::Equal),
    )
}

fn split_parameter_location(
    parameter: &Real,
    policy: &CurvePolicy,
) -> Option<BezierBooleanSplitParameterLocation> {
    let zero = Real::zero();
    let one = Real::one();
    let lower = crate::classify::compare_reals(parameter, &zero, policy)?;
    let upper = crate::classify::compare_reals(parameter, &one, policy)?;

    if lower == Ordering::Less || upper == Ordering::Greater {
        return Some(BezierBooleanSplitParameterLocation::OutOfRange);
    }
    if lower == Ordering::Equal || upper == Ordering::Equal {
        return Some(BezierBooleanSplitParameterLocation::Endpoint);
    }
    Some(BezierBooleanSplitParameterLocation::Interior)
}

fn collect_interior_parameters(
    parameters: &[Real],
    policy: &CurvePolicy,
    endpoint_count: &mut usize,
    out_of_range_count: &mut usize,
) -> Classification<Vec<Real>> {
    let mut interior = Vec::new();
    for parameter in parameters {
        match split_parameter_location(parameter, policy) {
            Some(BezierBooleanSplitParameterLocation::Interior) => {
                interior.push(parameter.clone());
            }
            Some(BezierBooleanSplitParameterLocation::Endpoint) => {
                *endpoint_count += 1;
            }
            Some(BezierBooleanSplitParameterLocation::OutOfRange) => {
                *out_of_range_count += 1;
            }
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(interior)
}

impl BezierBooleanPathSchedulerReport2 {
    /// Builds a global scheduler report from relation and monotone-range batches.
    ///
    /// Status precedence is conservative across both inputs: uncertainty,
    /// unresolved relation predicates, monotone-decomposition gaps, overlap
    /// obligations, algebraic isolation, contact classification, and parameter
    /// recovery all block split insertion. Only when neither batch has a
    /// blocker are represented relation/range split candidates marked ready.
    pub fn from_batches(
        relation_batch: BezierBooleanBatchHandoffReport2,
        range_batch: BezierPathRangeBatchReport2,
    ) -> Self {
        let uncertainty_reason = relation_batch
            .uncertainty_reason
            .or(range_batch.uncertainty_reason);
        let represented_split_event_count =
            relation_batch.point_events.len() + range_batch.split_parameters.len();
        let status = scheduler_status_from_batches(&relation_batch, &range_batch);

        Self {
            status,
            relation_point_events: relation_batch.point_events.clone(),
            range_split_parameters: range_batch.split_parameters.clone(),
            represented_split_event_count,
            relation_batch,
            range_batch,
            uncertainty_reason,
        }
    }

    /// Builds a global scheduler report directly from report slices.
    pub fn from_reports(
        relation_reports: &[BezierBooleanHandoffReport2],
        range_reports: &[BezierPathRangeOrderReport2],
    ) -> Self {
        Self::from_batches(
            BezierBooleanBatchHandoffReport2::from_handoff_reports(relation_reports),
            BezierPathRangeBatchReport2::from_range_reports(range_reports),
        )
    }

    /// Returns true when all represented split candidates can be inserted.
    pub fn can_feed_split_events(&self) -> bool {
        self.status == BezierBooleanPathSchedulerStatus::SplitEventsReady
    }

    /// Returns true when another exact stage must run before boolean topology.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanPathSchedulerStatus::NeedsParameterRecovery
                | BezierBooleanPathSchedulerStatus::NeedsContactClassification
                | BezierBooleanPathSchedulerStatus::NeedsRegionIsolation
                | BezierBooleanPathSchedulerStatus::NeedsOverlapResolver
                | BezierBooleanPathSchedulerStatus::NeedsMonotoneDecomposition
                | BezierBooleanPathSchedulerStatus::Unresolved
                | BezierBooleanPathSchedulerStatus::Uncertain
        )
    }
}

fn scheduler_status_from_batches(
    relation_batch: &BezierBooleanBatchHandoffReport2,
    range_batch: &BezierPathRangeBatchReport2,
) -> BezierBooleanPathSchedulerStatus {
    if relation_batch.status == BezierBooleanBatchHandoffStatus::Empty
        && range_batch.status == BezierPathRangeBatchStatus::Empty
    {
        return BezierBooleanPathSchedulerStatus::Empty;
    }
    if relation_batch.status == BezierBooleanBatchHandoffStatus::Uncertain
        || range_batch.status == BezierPathRangeBatchStatus::Uncertain
    {
        return BezierBooleanPathSchedulerStatus::Uncertain;
    }
    if relation_batch.status == BezierBooleanBatchHandoffStatus::Unresolved {
        return BezierBooleanPathSchedulerStatus::Unresolved;
    }
    if range_batch.status == BezierPathRangeBatchStatus::NeedsMonotoneDecomposition {
        return BezierBooleanPathSchedulerStatus::NeedsMonotoneDecomposition;
    }
    if relation_batch.status == BezierBooleanBatchHandoffStatus::NeedsOverlapResolver
        || range_batch.status == BezierPathRangeBatchStatus::NeedsOverlapResolver
    {
        return BezierBooleanPathSchedulerStatus::NeedsOverlapResolver;
    }
    if relation_batch.status == BezierBooleanBatchHandoffStatus::NeedsRegionIsolation
        || range_batch.status == BezierPathRangeBatchStatus::NeedsRegionIsolation
    {
        return BezierBooleanPathSchedulerStatus::NeedsRegionIsolation;
    }
    if range_batch.status == BezierPathRangeBatchStatus::NeedsContactClassification {
        return BezierBooleanPathSchedulerStatus::NeedsContactClassification;
    }
    if relation_batch.status == BezierBooleanBatchHandoffStatus::NeedsParameterRecovery {
        return BezierBooleanPathSchedulerStatus::NeedsParameterRecovery;
    }
    if relation_batch.status == BezierBooleanBatchHandoffStatus::SplitEventsReady
        || range_batch.status == BezierPathRangeBatchStatus::SplitEventsReady
    {
        return BezierBooleanPathSchedulerStatus::SplitEventsReady;
    }
    BezierBooleanPathSchedulerStatus::NoEvents
}

impl BezierBooleanBatchHandoffReport2 {
    /// Aggregates relation-level boolean handoff reports.
    ///
    /// Status precedence is intentionally conservative: uncertainty, unresolved
    /// predicates, overlap obligations, region-isolation obligations, and
    /// parameter-recovery obligations all block the batch before split-ready
    /// events can be consumed. This prevents a future path boolean from
    /// inserting a subset of events while ignoring a higher-priority unresolved
    /// combinatorial fact.
    pub fn from_handoff_reports(reports: &[BezierBooleanHandoffReport2]) -> Self {
        let mut batch = Self {
            status: BezierBooleanBatchHandoffStatus::Empty,
            relation_count: reports.len(),
            no_event_relation_count: 0,
            split_ready_relation_count: 0,
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 0,
            region_isolation_relation_count: 0,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        };

        for report in reports {
            match report.status {
                BezierBooleanHandoffStatus::NoEvents => {
                    batch.no_event_relation_count += 1;
                }
                BezierBooleanHandoffStatus::SplitEventsReady => {
                    batch.split_ready_relation_count += 1;
                }
                BezierBooleanHandoffStatus::NeedsParameterRecovery => {}
                BezierBooleanHandoffStatus::NeedsOverlapResolver => {}
                BezierBooleanHandoffStatus::NeedsRegionIsolation => {
                    batch.region_isolation_relation_count += 1;
                }
                BezierBooleanHandoffStatus::Unresolved => {}
                BezierBooleanHandoffStatus::Uncertain => {
                    batch.uncertainty_reason =
                        batch.uncertainty_reason.or(report.uncertainty_reason);
                }
            }

            batch
                .point_events
                .extend(report.point_events.iter().cloned());
            batch
                .overlap_events
                .extend(report.overlap_events.iter().cloned());
            batch.point_witnesses_needing_parameters += report.point_witnesses_needing_parameters;
            batch.overlap_relations_needing_resolution +=
                report.overlap_relations_needing_resolution;
            batch.unresolved_relations += report.unresolved_relations;
            batch.uncertain_relations += report.uncertain_relations;
        }

        batch.status = batch.derived_status();
        batch
    }

    /// Builds a batch report directly from classified Bezier relations.
    pub fn from_classified_relations(relations: &[Classification<BezierCurveRelation>]) -> Self {
        let reports = relations
            .iter()
            .map(BezierBooleanHandoffReport2::from_classified_relation)
            .collect::<Vec<_>>();
        Self::from_handoff_reports(&reports)
    }

    /// Returns true when every required point event is represented and no
    /// relation-level blocker remains.
    pub fn can_feed_split_events(&self) -> bool {
        self.status == BezierBooleanBatchHandoffStatus::SplitEventsReady
    }

    /// Returns true when another exact stage must run before boolean topology.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanBatchHandoffStatus::NeedsParameterRecovery
                | BezierBooleanBatchHandoffStatus::NeedsOverlapResolver
                | BezierBooleanBatchHandoffStatus::NeedsRegionIsolation
                | BezierBooleanBatchHandoffStatus::Unresolved
                | BezierBooleanBatchHandoffStatus::Uncertain
        )
    }

    fn derived_status(&self) -> BezierBooleanBatchHandoffStatus {
        if self.relation_count == 0 {
            BezierBooleanBatchHandoffStatus::Empty
        } else if self.uncertain_relations > 0 {
            BezierBooleanBatchHandoffStatus::Uncertain
        } else if self.unresolved_relations > 0 {
            BezierBooleanBatchHandoffStatus::Unresolved
        } else if self.overlap_relations_needing_resolution > 0 {
            BezierBooleanBatchHandoffStatus::NeedsOverlapResolver
        } else if self.region_isolation_relation_count > 0 {
            BezierBooleanBatchHandoffStatus::NeedsRegionIsolation
        } else if self.point_witnesses_needing_parameters > 0 {
            BezierBooleanBatchHandoffStatus::NeedsParameterRecovery
        } else if !self.point_events.is_empty() {
            BezierBooleanBatchHandoffStatus::SplitEventsReady
        } else {
            BezierBooleanBatchHandoffStatus::NoEvents
        }
    }
}

/// Path-operation order status for one shared monotone Bezier range.
///
/// This is the boolean-facing form of the monotone graph predicate. It follows
/// the path-operation representation discussed by Raph Levien for robust curve
/// booleans: y-monotone curve ranges are reduced to left/right order, contact,
/// crossing, overlap, or an explicit ambiguous payload. The conversion remains
/// a report layer, not a new geometric solve, matching Yap's exact geometric
/// computation rule that topology branches must carry either certified
/// combinatorics or auditable uncertainty.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierPathRangeOrderStatus {
    /// The requested axis was not certified as a shared strictly monotone graph axis.
    NotSharedMonotoneRange,
    /// The first curve is certified before the second in the compared coordinate.
    FirstBeforeSecond,
    /// The first curve is certified after the second in the compared coordinate.
    FirstAfterSecond,
    /// The curve images are certified coincident on the range.
    Overlap,
    /// Represented exact contacts are all tangential.
    TangentContact,
    /// At least one represented exact contact is a crossing.
    CrossingContact,
    /// One or more contact roots are retained only as isolating spans.
    Ambiguous,
    /// A lower predicate reported explicit uncertainty.
    Uncertain,
}

/// Boolean-facing order report for one shared monotone Bezier range.
///
/// The report packages the exact graph-order predicates into the shape needed
/// by later path booleans: order-only ranges can be swept directly, coincident
/// ranges require an overlap resolver, represented contacts can become split
/// events, and isolating spans remain blockers until an algebraic isolator
/// produces exact event parameters. The separation mirrors Greiner-Hormann and
/// Martinez-Rueda-Feito split/classify/traverse pipelines while keeping Yap's
/// predicate/construction boundary explicit.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierPathRangeOrderReport2 {
    /// Coarse status for this monotone range.
    pub status: BezierPathRangeOrderStatus,
    /// Exact represented contact parameters with crossing/tangent labels.
    pub contacts: Vec<BezierGraphContact>,
    /// Exact represented parameters whose crossing/tangent kind is not known.
    pub unclassified_parameters: Vec<Real>,
    /// Isolating spans for contact roots not represented by the scalar root API.
    pub isolating_spans: Vec<BezierMonotoneSpan>,
    /// Explicit primitive uncertainty reason, if the source predicate was uncertain.
    pub uncertainty_reason: Option<UncertaintyReason>,
}

/// Aggregate boolean-readiness state for a sequence of monotone range reports.
///
/// A path boolean usually consumes many monotone curve ranges, not a single
/// pair. This status compacts those per-range facts into the scheduling shape
/// needed by an arrangement builder while preserving Yap's requirement that
/// undecided combinatorics stay visible as object facts. Represented
/// crossing/tangent contacts may feed split insertion; coincident ranges still
/// need an overlap resolver as in degenerate polygon clipping work by Foster,
/// Hormann, and Popa, "Clipping simple polygons with degenerate
/// intersections," *Computers & Graphics: X* 2 (2019).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierPathRangeBatchStatus {
    /// No monotone range reports were supplied.
    Empty,
    /// All ranges have certified strict order and no split events.
    OrderedOnly,
    /// At least one represented contact can feed split insertion and no range
    /// has a blocker.
    SplitEventsReady,
    /// Represented parameters exist, but crossing/tangency classification is
    /// still missing.
    NeedsContactClassification,
    /// One or more unrepresented roots remain as isolating spans.
    NeedsRegionIsolation,
    /// One or more coincident ranges require an overlap resolver.
    NeedsOverlapResolver,
    /// One or more ranges were not certified as shared monotone graph ranges.
    NeedsMonotoneDecomposition,
    /// A lower predicate reported explicit uncertainty.
    Uncertain,
}

/// Boolean-facing aggregate over monotone Bezier range-order reports.
///
/// The report performs no geometry. It only folds already-certified
/// [`BezierPathRangeOrderReport2`] values into a batch-level contract:
/// strict-order ranges can be swept without split insertion, represented
/// contacts provide exact split parameters, unclassified represented roots need
/// contact classification, unrepresented spans need more algebraic isolation,
/// and overlap/not-monotone/uncertain ranges remain explicit blockers. This is
/// the same split/classify/traverse staging used by Greiner-Hormann (1998) and
/// Martinez-Rueda-Feito (2009), specialized to Bezier monotone-range facts.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierPathRangeBatchReport2 {
    /// Coarse batch readiness status.
    pub status: BezierPathRangeBatchStatus,
    /// Number of range reports consumed.
    pub range_count: usize,
    /// Number of strict ordered ranges.
    pub ordered_range_count: usize,
    /// Number of ranges with represented contact payloads.
    pub contact_range_count: usize,
    /// Number of represented crossing contacts.
    pub crossing_contact_count: usize,
    /// Number of represented tangent contacts.
    pub tangent_contact_count: usize,
    /// Exact represented contact parameters ready for split insertion when
    /// `status` is [`BezierPathRangeBatchStatus::SplitEventsReady`].
    pub split_parameters: Vec<Real>,
    /// Exact represented parameters whose crossing/tangency kind is not known.
    pub unclassified_parameters: Vec<Real>,
    /// Isolating spans for roots not represented by the scalar root API.
    pub isolating_spans: Vec<BezierMonotoneSpan>,
    /// Number of coincident/overlap ranges.
    pub overlap_range_count: usize,
    /// Number of ranges that still need monotone decomposition or axis proof.
    pub not_shared_monotone_range_count: usize,
    /// Number of ranges that carried primitive uncertainty.
    pub uncertain_range_count: usize,
    /// First explicit primitive uncertainty reason retained by the batch.
    pub uncertainty_reason: Option<UncertaintyReason>,
}

impl BezierPathRangeBatchReport2 {
    /// Aggregates per-range path-operation reports into a boolean scheduler report.
    ///
    /// Status precedence is intentionally conservative: uncertainty outranks
    /// monotone-decomposition gaps, then overlap obligations, unrepresented
    /// isolating spans, unclassified represented parameters, represented split
    /// contacts, and finally strict ordering. This prevents a later boolean
    /// stage from treating one split-ready contact as proof that the whole path
    /// range batch is ready.
    pub fn from_range_reports(reports: &[BezierPathRangeOrderReport2]) -> Self {
        let mut batch = Self {
            status: BezierPathRangeBatchStatus::Empty,
            range_count: reports.len(),
            ordered_range_count: 0,
            contact_range_count: 0,
            crossing_contact_count: 0,
            tangent_contact_count: 0,
            split_parameters: Vec::new(),
            unclassified_parameters: Vec::new(),
            isolating_spans: Vec::new(),
            overlap_range_count: 0,
            not_shared_monotone_range_count: 0,
            uncertain_range_count: 0,
            uncertainty_reason: None,
        };

        for report in reports {
            match report.status {
                BezierPathRangeOrderStatus::FirstBeforeSecond
                | BezierPathRangeOrderStatus::FirstAfterSecond => {
                    batch.ordered_range_count += 1;
                }
                BezierPathRangeOrderStatus::CrossingContact
                | BezierPathRangeOrderStatus::TangentContact => {
                    batch.contact_range_count += 1;
                }
                BezierPathRangeOrderStatus::Overlap => {
                    batch.overlap_range_count += 1;
                }
                BezierPathRangeOrderStatus::NotSharedMonotoneRange => {
                    batch.not_shared_monotone_range_count += 1;
                }
                BezierPathRangeOrderStatus::Ambiguous => {}
                BezierPathRangeOrderStatus::Uncertain => {
                    batch.uncertain_range_count += 1;
                    batch.uncertainty_reason =
                        batch.uncertainty_reason.or(report.uncertainty_reason);
                }
            }

            for contact in &report.contacts {
                batch.split_parameters.push(contact.parameter().clone());
                match contact.kind() {
                    BezierLineContactKind::Crossing => batch.crossing_contact_count += 1,
                    BezierLineContactKind::Tangent => batch.tangent_contact_count += 1,
                }
            }
            batch
                .unclassified_parameters
                .extend(report.unclassified_parameters.iter().cloned());
            batch
                .isolating_spans
                .extend(report.isolating_spans.iter().cloned());
        }

        batch.status = batch.derived_status();
        batch
    }

    /// Returns true when every required split parameter is represented and no
    /// range-level blocker remains.
    pub fn can_feed_split_events(&self) -> bool {
        self.status == BezierPathRangeBatchStatus::SplitEventsReady
    }

    /// Returns true when another exact stage must run before boolean topology.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierPathRangeBatchStatus::NeedsContactClassification
                | BezierPathRangeBatchStatus::NeedsRegionIsolation
                | BezierPathRangeBatchStatus::NeedsOverlapResolver
                | BezierPathRangeBatchStatus::NeedsMonotoneDecomposition
                | BezierPathRangeBatchStatus::Uncertain
        )
    }

    fn derived_status(&self) -> BezierPathRangeBatchStatus {
        if self.range_count == 0 {
            BezierPathRangeBatchStatus::Empty
        } else if self.uncertain_range_count > 0 {
            BezierPathRangeBatchStatus::Uncertain
        } else if self.not_shared_monotone_range_count > 0 {
            BezierPathRangeBatchStatus::NeedsMonotoneDecomposition
        } else if self.overlap_range_count > 0 {
            BezierPathRangeBatchStatus::NeedsOverlapResolver
        } else if !self.isolating_spans.is_empty() {
            BezierPathRangeBatchStatus::NeedsRegionIsolation
        } else if !self.unclassified_parameters.is_empty() {
            BezierPathRangeBatchStatus::NeedsContactClassification
        } else if !self.split_parameters.is_empty() {
            BezierPathRangeBatchStatus::SplitEventsReady
        } else {
            BezierPathRangeBatchStatus::OrderedOnly
        }
    }
}

impl BezierBooleanRootIsolationHandoffReport2 {
    /// Builds a root-isolation handoff from one boolean relation report.
    ///
    /// A [`BezierBooleanHandoffReport2`] with retained positive-width regions
    /// is now actionable because `hypersolve` can isolate roots. This method
    /// marks such frontiers as [`BezierBooleanRootIsolationHandoffStatus::ReadyForHypersolve`]
    /// while preserving parameter-recovery, overlap, unresolved, and
    /// uncertainty blockers. A split-ready exact-point certificate stays
    /// split-ready and does not need root isolation. This follows Yap (1997):
    /// the handoff certifies what exact stage may run next, not the final
    /// topology. The delegated isolation model is the Sturm/Collins-Loos
    /// exact real-root isolation tradition now surfaced through `hypersolve`.
    pub fn from_handoff_report(report: &BezierBooleanHandoffReport2) -> Self {
        let mut handoff = Self::empty();
        handoff.relation_count = 1;
        handoff.apply_report(report);
        handoff.finalize()
    }

    /// Builds a root-isolation handoff from many boolean relation reports.
    pub fn from_handoff_reports(reports: &[BezierBooleanHandoffReport2]) -> Self {
        let mut handoff = Self::empty();
        handoff.relation_count = reports.len();
        for report in reports {
            handoff.apply_report(report);
        }
        handoff.finalize()
    }

    /// Builds a root-isolation handoff from an aggregate boolean batch report.
    ///
    /// Batch reports do not retain individual isolation certificates, so this
    /// method preserves only aggregate blocker and region-isolation counts.
    pub fn from_batch_handoff(batch: &BezierBooleanBatchHandoffReport2) -> Self {
        let blocker_count = batch.point_witnesses_needing_parameters
            + batch.overlap_relations_needing_resolution
            + batch.unresolved_relations
            + batch.uncertain_relations;
        let status = match batch.status {
            BezierBooleanBatchHandoffStatus::Empty => {
                BezierBooleanRootIsolationHandoffStatus::Empty
            }
            BezierBooleanBatchHandoffStatus::NoEvents => {
                BezierBooleanRootIsolationHandoffStatus::NotNeeded
            }
            BezierBooleanBatchHandoffStatus::SplitEventsReady => {
                BezierBooleanRootIsolationHandoffStatus::SplitEventsReady
            }
            BezierBooleanBatchHandoffStatus::NeedsRegionIsolation => {
                BezierBooleanRootIsolationHandoffStatus::ReadyForHypersolve
            }
            BezierBooleanBatchHandoffStatus::NeedsParameterRecovery => {
                BezierBooleanRootIsolationHandoffStatus::BlockedByParameterRecovery
            }
            BezierBooleanBatchHandoffStatus::NeedsOverlapResolver => {
                BezierBooleanRootIsolationHandoffStatus::BlockedByOverlapResolver
            }
            BezierBooleanBatchHandoffStatus::Unresolved => {
                BezierBooleanRootIsolationHandoffStatus::BlockedByUnresolved
            }
            BezierBooleanBatchHandoffStatus::Uncertain => {
                BezierBooleanRootIsolationHandoffStatus::BlockedByUncertainty
            }
        };
        Self {
            status,
            relation_count: batch.relation_count,
            split_ready_relation_count: batch.split_ready_relation_count,
            region_isolation_relation_count: batch.region_isolation_relation_count,
            isolation_certificate_count: 0,
            exact_point_certificate_count: 0,
            terminal_region_count: batch.region_isolation_relation_count,
            range_isolating_span_count: 0,
            point_witnesses_needing_parameters: batch.point_witnesses_needing_parameters,
            unclassified_parameter_count: 0,
            overlap_relations_needing_resolution: batch.overlap_relations_needing_resolution,
            not_shared_monotone_range_count: 0,
            unresolved_relations: batch.unresolved_relations,
            uncertain_relations: batch.uncertain_relations,
            uncertainty_reason: batch.uncertainty_reason,
            blocker_count,
        }
    }

    /// Builds a root-isolation handoff from a combined path scheduler.
    pub fn from_path_scheduler(scheduler: &BezierBooleanPathSchedulerReport2) -> Self {
        let mut handoff = Self::from_batch_handoff(&scheduler.relation_batch);
        handoff.range_isolating_span_count = scheduler.range_batch.isolating_spans.len();
        handoff.unclassified_parameter_count = scheduler.range_batch.unclassified_parameters.len();
        handoff.overlap_relations_needing_resolution += scheduler.range_batch.overlap_range_count;
        handoff.not_shared_monotone_range_count =
            scheduler.range_batch.not_shared_monotone_range_count;
        handoff.uncertain_relations += scheduler.range_batch.uncertain_range_count;
        if handoff.uncertainty_reason.is_none() {
            handoff.uncertainty_reason = scheduler.range_batch.uncertainty_reason;
        }
        handoff.finalize()
    }

    fn empty() -> Self {
        Self {
            status: BezierBooleanRootIsolationHandoffStatus::Empty,
            relation_count: 0,
            split_ready_relation_count: 0,
            region_isolation_relation_count: 0,
            isolation_certificate_count: 0,
            exact_point_certificate_count: 0,
            terminal_region_count: 0,
            range_isolating_span_count: 0,
            point_witnesses_needing_parameters: 0,
            unclassified_parameter_count: 0,
            overlap_relations_needing_resolution: 0,
            not_shared_monotone_range_count: 0,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
            blocker_count: 0,
        }
    }

    fn apply_report(&mut self, report: &BezierBooleanHandoffReport2) {
        match report.status {
            BezierBooleanHandoffStatus::NoEvents => {}
            BezierBooleanHandoffStatus::SplitEventsReady => {
                self.split_ready_relation_count += 1;
            }
            BezierBooleanHandoffStatus::NeedsRegionIsolation => {
                self.region_isolation_relation_count += 1;
                if let Some(certificate) = &report.isolation_certificate {
                    self.isolation_certificate_count += 1;
                    self.terminal_region_count += certificate.terminal_region_count;
                    if certificate.terminal_region_count > 0
                        && certificate.terminal_summary.exact_point_cells
                            == certificate.terminal_region_count
                        && certificate.terminal_summary.invalid_spans == 0
                        && certificate.terminal_summary.unknown_regions == 0
                    {
                        self.exact_point_certificate_count += 1;
                    }
                } else if let Some(summary) = &report.region_summary {
                    self.terminal_region_count += summary.region_count;
                } else {
                    self.terminal_region_count += 1;
                }
            }
            BezierBooleanHandoffStatus::NeedsParameterRecovery => {
                self.point_witnesses_needing_parameters +=
                    report.point_witnesses_needing_parameters.max(1);
            }
            BezierBooleanHandoffStatus::NeedsOverlapResolver => {
                self.overlap_relations_needing_resolution +=
                    report.overlap_relations_needing_resolution.max(1);
            }
            BezierBooleanHandoffStatus::Unresolved => {
                self.unresolved_relations += report.unresolved_relations.max(1);
            }
            BezierBooleanHandoffStatus::Uncertain => {
                self.uncertain_relations += report.uncertain_relations.max(1);
                if self.uncertainty_reason.is_none() {
                    self.uncertainty_reason = report.uncertainty_reason;
                }
            }
        }
    }

    fn finalize(mut self) -> Self {
        self.blocker_count = self.point_witnesses_needing_parameters
            + self.unclassified_parameter_count
            + self.overlap_relations_needing_resolution
            + self.not_shared_monotone_range_count
            + self.unresolved_relations
            + self.uncertain_relations;
        self.status = if self.relation_count == 0
            && self.range_isolating_span_count == 0
            && self.unclassified_parameter_count == 0
            && self.overlap_relations_needing_resolution == 0
            && self.not_shared_monotone_range_count == 0
            && self.uncertain_relations == 0
        {
            BezierBooleanRootIsolationHandoffStatus::Empty
        } else if self.point_witnesses_needing_parameters > 0 {
            BezierBooleanRootIsolationHandoffStatus::BlockedByParameterRecovery
        } else if self.overlap_relations_needing_resolution > 0 {
            BezierBooleanRootIsolationHandoffStatus::BlockedByOverlapResolver
        } else if self.unclassified_parameter_count > 0 {
            BezierBooleanRootIsolationHandoffStatus::BlockedByContactClassification
        } else if self.not_shared_monotone_range_count > 0 {
            BezierBooleanRootIsolationHandoffStatus::BlockedByMonotoneDecomposition
        } else if self.unresolved_relations > 0 {
            BezierBooleanRootIsolationHandoffStatus::BlockedByUnresolved
        } else if self.uncertain_relations > 0 {
            BezierBooleanRootIsolationHandoffStatus::BlockedByUncertainty
        } else if self.region_isolation_relation_count > 0 || self.range_isolating_span_count > 0 {
            BezierBooleanRootIsolationHandoffStatus::ReadyForHypersolve
        } else if self.split_ready_relation_count > 0 {
            BezierBooleanRootIsolationHandoffStatus::SplitEventsReady
        } else {
            BezierBooleanRootIsolationHandoffStatus::NotNeeded
        };
        self
    }

    /// Returns true when exact root isolation can run next.
    pub fn can_feed_hypersolve(&self) -> bool {
        self.status == BezierBooleanRootIsolationHandoffStatus::ReadyForHypersolve
    }

    /// Returns true when an earlier exact stage must run first.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanRootIsolationHandoffStatus::BlockedByParameterRecovery
                | BezierBooleanRootIsolationHandoffStatus::BlockedByOverlapResolver
                | BezierBooleanRootIsolationHandoffStatus::BlockedByContactClassification
                | BezierBooleanRootIsolationHandoffStatus::BlockedByMonotoneDecomposition
                | BezierBooleanRootIsolationHandoffStatus::BlockedByUnresolved
                | BezierBooleanRootIsolationHandoffStatus::BlockedByUncertainty
        )
    }
}

impl BezierBooleanRootIsolationReplayReport2 {
    /// Replays exact represented roots returned by `hypersolve`.
    ///
    /// The caller supplies exact relation point events and exact monotone-range
    /// parameters after running root isolation. This method validates only the
    /// boolean-facing obligations: every retained frontier has a represented
    /// root, every represented parameter is inside the Bezier unit interval,
    /// and earlier handoff blockers remain blockers. It intentionally does not
    /// call `hypersolve` directly; `hypercurve` owns topology, while
    /// `hypersolve` owns the Sturm/Collins-Loos-style isolation proof package.
    pub fn from_hypersolve_roots(
        scheduler: &BezierBooleanPathSchedulerReport2,
        isolated_relation_events: &[BezierBooleanPointEvent2],
        isolated_range_parameters: &[Real],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let handoff = BezierBooleanRootIsolationHandoffReport2::from_path_scheduler(scheduler);
        let required_isolation_count =
            handoff.terminal_region_count + handoff.range_isolating_span_count;
        let supplied_isolation_count =
            isolated_relation_events.len() + isolated_range_parameters.len();
        let split_plan = Self::split_plan_from_replay(
            scheduler,
            isolated_relation_events,
            isolated_range_parameters,
            BezierBooleanSplitPlanStatus::Blocked,
        );
        let mut report = Self {
            status: BezierBooleanRootIsolationReplayStatus::HandoffBlocked,
            handoff_status: handoff.status,
            scheduler_status: scheduler.status,
            split_plan,
            required_isolation_count,
            supplied_isolation_count,
            hypersolve_report_count: 0,
            hypersolve_interval_count: 0,
            hypersolve_exact_root_count: 0,
            hypersolve_unusable_count: 0,
            hypersolve_bernstein_report_count: 0,
            hypersolve_bernstein_empty_interval_count: 0,
            hypersolve_bernstein_endpoint_root_count: 0,
            hypersolve_bernstein_isolating_interval_count: 0,
            hypersolve_bernstein_unusable_count: 0,
            hypersolve_algebraic_report_count: 0,
            hypersolve_algebraic_root_count: 0,
            hypersolve_algebraic_exact_root_count: 0,
            hypersolve_algebraic_interval_root_count: 0,
            hypersolve_algebraic_unusable_count: 0,
            missing_isolation_count: required_isolation_count
                .saturating_sub(supplied_isolation_count),
            out_of_range_parameter_count: 0,
            blocker_count: handoff.blocker_count,
            uncertainty_reason: scheduler.uncertainty_reason,
        };

        match handoff.status {
            BezierBooleanRootIsolationHandoffStatus::Empty => {
                report.status = BezierBooleanRootIsolationReplayStatus::Empty;
                report.split_plan = BezierBooleanSplitPlanReport2::from_scheduler(scheduler);
                report.blocker_count = 0;
                return Classification::Decided(report);
            }
            BezierBooleanRootIsolationHandoffStatus::NotNeeded => {
                report.status = BezierBooleanRootIsolationReplayStatus::NotNeeded;
                report.split_plan = BezierBooleanSplitPlanReport2::from_scheduler(scheduler);
                report.blocker_count = 0;
                return Classification::Decided(report);
            }
            BezierBooleanRootIsolationHandoffStatus::SplitEventsReady => {
                report.status = BezierBooleanRootIsolationReplayStatus::ReadyForSplitEvents;
                report.split_plan = BezierBooleanSplitPlanReport2::from_scheduler(scheduler);
                report.blocker_count = 0;
                return Classification::Decided(report);
            }
            BezierBooleanRootIsolationHandoffStatus::ReadyForHypersolve => {}
            BezierBooleanRootIsolationHandoffStatus::BlockedByParameterRecovery
            | BezierBooleanRootIsolationHandoffStatus::BlockedByOverlapResolver
            | BezierBooleanRootIsolationHandoffStatus::BlockedByUnresolved
            | BezierBooleanRootIsolationHandoffStatus::BlockedByContactClassification
            | BezierBooleanRootIsolationHandoffStatus::BlockedByMonotoneDecomposition
            | BezierBooleanRootIsolationHandoffStatus::BlockedByUncertainty => {
                report.blocker_count = handoff.blocker_count.max(1);
                return Classification::Decided(report);
            }
        }

        if report.missing_isolation_count > 0 {
            report.status = BezierBooleanRootIsolationReplayStatus::MissingIsolatedRoots;
            report.blocker_count = report.missing_isolation_count;
            return Classification::Decided(report);
        }

        let out_of_range = match Self::count_out_of_range_parameters(
            isolated_relation_events,
            isolated_range_parameters,
            policy,
        ) {
            Classification::Decided(count) => count,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        report.split_plan = Self::split_plan_from_replay(
            scheduler,
            isolated_relation_events,
            isolated_range_parameters,
            BezierBooleanSplitPlanStatus::Ready,
        );
        report.out_of_range_parameter_count = out_of_range;
        if out_of_range > 0 {
            report.status = BezierBooleanRootIsolationReplayStatus::InvalidParameterDomain;
            report.blocker_count = out_of_range;
        } else {
            report.status = BezierBooleanRootIsolationReplayStatus::ReadyForSplitEvents;
            report.blocker_count = 0;
        }
        Classification::Decided(report)
    }

    /// Replays exact rational witnesses from `hypersolve` isolation reports.
    ///
    /// This is the direct `hypersolve` integration path for monotone-range
    /// roots. `hypersolve::UnivariateRootIsolationReport` carries certified
    /// Sturm/Collins-Loos isolating intervals; `hypercurve` accepts only the
    /// intervals that also contain an exact rational witness. Non-rational
    /// intervals, unsupported rows, undecided rows, and no-root rows stay
    /// explicit blockers because Bezier split insertion needs represented
    /// parameters, not interval approximations. Relation-region roots still
    /// enter as [`BezierBooleanPointEvent2`] values because a univariate solver
    /// report cannot encode both curve parameters and contact metadata.
    pub fn from_hypersolve_range_reports(
        scheduler: &BezierBooleanPathSchedulerReport2,
        isolated_relation_events: &[BezierBooleanPointEvent2],
        range_reports: &[UnivariateRootIsolationReport],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut range_parameters = Vec::new();
        let mut interval_count = 0;
        let mut exact_root_count = 0;
        let mut unusable_count = 0;

        for report in range_reports {
            match report.status {
                RootIsolationStatus::Isolated | RootIsolationStatus::MultipleRoot => {}
                RootIsolationStatus::NoRealRoots
                | RootIsolationStatus::UnsupportedCoefficient
                | RootIsolationStatus::Undecided => {
                    unusable_count += 1;
                    continue;
                }
            }

            for interval in &report.intervals {
                interval_count += 1;
                if interval.distinct_root_count != 1 {
                    unusable_count += 1;
                    continue;
                }
                if let Some(root) = &interval.exact_root {
                    exact_root_count += 1;
                    range_parameters.push(root.clone());
                } else {
                    unusable_count += 1;
                }
            }
        }

        let mut replay = match Self::from_hypersolve_roots(
            scheduler,
            isolated_relation_events,
            &range_parameters,
            policy,
        ) {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        replay.hypersolve_report_count = range_reports.len();
        replay.hypersolve_interval_count = interval_count;
        replay.hypersolve_exact_root_count = exact_root_count;
        replay.hypersolve_unusable_count = unusable_count;

        if unusable_count > 0 {
            replay.status = BezierBooleanRootIsolationReplayStatus::HypersolveBlocked;
            replay.blocker_count += unusable_count;
        }

        Classification::Decided(replay)
    }

    /// Replays exact endpoint roots from `hypersolve` Bernstein subdivisions.
    ///
    /// Bernstein subdivision is a finite-interval algebraic filter, not a
    /// topology constructor. Empty terminal intervals are counted as certified
    /// non-root evidence; endpoint-root intervals contribute represented
    /// parameters only when `hypersolve` supplies the exact rational witness;
    /// variation-one isolating intervals, depth limits, unsupported rows, and
    /// undecided rows remain blockers. This mirrors Farouki and Rajan's
    /// Bernstein-form subdivision filter ("Algorithms for Polynomials in
    /// Bernstein Form," 1988) while preserving Yap's 1997 exact-geometric
    /// computation boundary: Bezier boolean split insertion receives explicit
    /// parameters, never interval samples.
    pub fn from_hypersolve_bernstein_subdivision_reports(
        scheduler: &BezierBooleanPathSchedulerReport2,
        isolated_relation_events: &[BezierBooleanPointEvent2],
        subdivision_reports: &[BernsteinSubdivisionReport],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut range_parameters = Vec::new();
        let mut empty_interval_count = 0;
        let mut endpoint_root_count = 0;
        let mut isolating_interval_count = 0;
        let mut unusable_count = 0;

        for report in subdivision_reports {
            match report.status {
                BernsteinSubdivisionStatus::Completed => {}
                BernsteinSubdivisionStatus::DepthLimit
                | BernsteinSubdivisionStatus::InvalidInterval
                | BernsteinSubdivisionStatus::UnsupportedCoefficient
                | BernsteinSubdivisionStatus::Undecided => {
                    unusable_count += 1;
                    continue;
                }
            }

            for interval in &report.intervals {
                match interval.status {
                    BernsteinSubdivisionIntervalStatus::Empty => {
                        empty_interval_count += 1;
                    }
                    BernsteinSubdivisionIntervalStatus::EndpointRoot => {
                        if let Some(root) = &interval.exact_root {
                            endpoint_root_count += 1;
                            range_parameters.push(root.clone());
                        } else {
                            unusable_count += 1;
                        }
                    }
                    BernsteinSubdivisionIntervalStatus::Isolating => {
                        isolating_interval_count += 1;
                        unusable_count += 1;
                    }
                    BernsteinSubdivisionIntervalStatus::DepthLimit => {
                        unusable_count += 1;
                    }
                }
            }
        }

        let mut replay = match Self::from_hypersolve_roots(
            scheduler,
            isolated_relation_events,
            &range_parameters,
            policy,
        ) {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        replay.hypersolve_bernstein_report_count = subdivision_reports.len();
        replay.hypersolve_bernstein_empty_interval_count = empty_interval_count;
        replay.hypersolve_bernstein_endpoint_root_count = endpoint_root_count;
        replay.hypersolve_bernstein_isolating_interval_count = isolating_interval_count;
        replay.hypersolve_bernstein_unusable_count = unusable_count;

        if unusable_count > 0 {
            replay.status = BezierBooleanRootIsolationReplayStatus::HypersolveBlocked;
            replay.blocker_count += unusable_count;
        }

        Classification::Decided(replay)
    }

    /// Replays exact witnesses from `hypersolve` algebraic-root representations.
    ///
    /// `hypersolve::AlgebraicRootRepresentationReport` keeps the exact
    /// polynomial row together with each certified isolating interval. That is
    /// stronger evidence than a bare interval, but Bezier boolean split
    /// insertion still needs represented parameters in the current API.
    /// Therefore exact rational represented roots are accepted as split
    /// parameters, while valid non-rational algebraic interval roots are
    /// counted and retained as explicit blockers for a future algebraic
    /// parameter carrier. This follows Yap's exact-computation boundary and
    /// the Collins-Loos real-root representation model: algebraic objects are
    /// proof evidence, not primitive-float topology.
    pub fn from_hypersolve_algebraic_root_reports(
        scheduler: &BezierBooleanPathSchedulerReport2,
        isolated_relation_events: &[BezierBooleanPointEvent2],
        algebraic_reports: &[AlgebraicRootRepresentationReport],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut range_parameters = Vec::new();
        let mut algebraic_root_count = 0;
        let mut exact_root_count = 0;
        let mut interval_root_count = 0;
        let mut unusable_count = 0;

        for report in algebraic_reports {
            match report.status {
                AlgebraicRootRepresentationStatus::Represented => {}
                AlgebraicRootRepresentationStatus::NoRealRoots => {
                    continue;
                }
                AlgebraicRootRepresentationStatus::UnsupportedIsolationStatus
                | AlgebraicRootRepresentationStatus::MissingSymbol
                | AlgebraicRootRepresentationStatus::MissingPolynomial
                | AlgebraicRootRepresentationStatus::InvalidEvidence => {
                    unusable_count += 1;
                    continue;
                }
            }

            for root in &report.roots {
                algebraic_root_count += 1;
                if !root.is_valid() {
                    unusable_count += 1;
                    continue;
                }
                if let Some(witness) = root.exact_rational_witness() {
                    exact_root_count += 1;
                    range_parameters.push(witness.clone());
                } else {
                    interval_root_count += 1;
                    unusable_count += 1;
                }
            }
        }

        let mut replay = match Self::from_hypersolve_roots(
            scheduler,
            isolated_relation_events,
            &range_parameters,
            policy,
        ) {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        replay.hypersolve_algebraic_report_count = algebraic_reports.len();
        replay.hypersolve_algebraic_root_count = algebraic_root_count;
        replay.hypersolve_algebraic_exact_root_count = exact_root_count;
        replay.hypersolve_algebraic_interval_root_count = interval_root_count;
        replay.hypersolve_algebraic_unusable_count = unusable_count;

        if unusable_count > 0 {
            replay.status = BezierBooleanRootIsolationReplayStatus::HypersolveBlocked;
            replay.blocker_count += unusable_count;
        }

        Classification::Decided(replay)
    }

    /// Returns true when replay produced a ready split plan.
    pub fn can_feed_split_events(&self) -> bool {
        self.status == BezierBooleanRootIsolationReplayStatus::ReadyForSplitEvents
            && self.split_plan.is_ready()
    }

    /// Returns true when replay still needs an earlier exact stage.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanRootIsolationReplayStatus::HandoffBlocked
                | BezierBooleanRootIsolationReplayStatus::HypersolveBlocked
                | BezierBooleanRootIsolationReplayStatus::MissingIsolatedRoots
                | BezierBooleanRootIsolationReplayStatus::InvalidParameterDomain
        )
    }

    fn split_plan_from_replay(
        scheduler: &BezierBooleanPathSchedulerReport2,
        isolated_relation_events: &[BezierBooleanPointEvent2],
        isolated_range_parameters: &[Real],
        status: BezierBooleanSplitPlanStatus,
    ) -> BezierBooleanSplitPlanReport2 {
        let mut first_curve_parameters = scheduler
            .relation_point_events
            .iter()
            .map(|event| event.first_param.clone())
            .collect::<Vec<_>>();
        let mut second_curve_parameters = scheduler
            .relation_point_events
            .iter()
            .map(|event| event.second_param.clone())
            .collect::<Vec<_>>();
        let mut shared_range_parameters = scheduler.range_split_parameters.clone();

        first_curve_parameters.extend(
            isolated_relation_events
                .iter()
                .map(|event| event.first_param.clone()),
        );
        second_curve_parameters.extend(
            isolated_relation_events
                .iter()
                .map(|event| event.second_param.clone()),
        );
        shared_range_parameters.extend(isolated_range_parameters.iter().cloned());

        BezierBooleanSplitPlanReport2 {
            status,
            scheduler_status: scheduler.status,
            first_curve_parameters,
            second_curve_parameters,
            shared_range_parameters,
            relation_event_count: scheduler.relation_point_events.len()
                + isolated_relation_events.len(),
            range_event_count: scheduler.range_split_parameters.len()
                + isolated_range_parameters.len(),
            uncertainty_reason: scheduler.uncertainty_reason,
        }
    }

    fn count_out_of_range_parameters(
        isolated_relation_events: &[BezierBooleanPointEvent2],
        isolated_range_parameters: &[Real],
        policy: &CurvePolicy,
    ) -> Classification<usize> {
        let mut out_of_range = 0;
        for parameter in isolated_relation_events
            .iter()
            .flat_map(|event| [&event.first_param, &event.second_param])
            .chain(isolated_range_parameters.iter())
        {
            match parameter_in_unit_interval(parameter, policy) {
                Some(true) => {}
                Some(false) => out_of_range += 1,
                None => return Classification::Uncertain(UncertaintyReason::Ordering),
            }
        }
        Classification::Decided(out_of_range)
    }
}

impl BezierBooleanRootIsolationConstructionReport2 {
    /// Runs the full `hypersolve` root-replay to construction-readiness chain.
    ///
    /// The scheduler is retained in the readiness report, while the replay
    /// report retains the exact solver evidence counts. This gives callers one
    /// auditable object for the algebraic root-isolation handoff before they
    /// choose typed Bezier/conic fragment constructors.
    pub fn from_hypersolve_range_reports(
        scheduler: BezierBooleanPathSchedulerReport2,
        isolated_relation_events: &[BezierBooleanPointEvent2],
        range_reports: &[UnivariateRootIsolationReport],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let replay = match BezierBooleanRootIsolationReplayReport2::from_hypersolve_range_reports(
            &scheduler,
            isolated_relation_events,
            range_reports,
            policy,
        ) {
            Classification::Decided(replay) => replay,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        Self::from_replay(scheduler, replay, policy)
    }

    /// Runs the Bernstein-subdivision replay to construction-readiness chain.
    ///
    /// This is the subdivision-filter sibling of
    /// [`Self::from_hypersolve_range_reports`]. It accepts only exact endpoint
    /// roots as represented split parameters and keeps all non-rational
    /// isolating intervals report-bearing, matching Yap's construction/proof
    /// separation and the Bernstein subdivision evidence model of Farouki and
    /// Rajan (1988).
    pub fn from_hypersolve_bernstein_subdivision_reports(
        scheduler: BezierBooleanPathSchedulerReport2,
        isolated_relation_events: &[BezierBooleanPointEvent2],
        subdivision_reports: &[BernsteinSubdivisionReport],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let replay =
            match BezierBooleanRootIsolationReplayReport2::from_hypersolve_bernstein_subdivision_reports(
                &scheduler,
                isolated_relation_events,
                subdivision_reports,
                policy,
            ) {
                Classification::Decided(replay) => replay,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        Self::from_replay(scheduler, replay, policy)
    }

    /// Runs the algebraic-root representation replay to construction readiness.
    ///
    /// This consumes persistent exact root objects from `hypersolve`, accepts
    /// only exact rational witnesses as current split parameters, and preserves
    /// non-rational algebraic interval roots as blockers for future algebraic
    /// Bezier parameter carriers. That keeps the Yap/Collins-Loos distinction
    /// between represented algebraic evidence and topology construction
    /// explicit in the compact boolean handoff report.
    pub fn from_hypersolve_algebraic_root_reports(
        scheduler: BezierBooleanPathSchedulerReport2,
        isolated_relation_events: &[BezierBooleanPointEvent2],
        algebraic_reports: &[AlgebraicRootRepresentationReport],
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let replay =
            match BezierBooleanRootIsolationReplayReport2::from_hypersolve_algebraic_root_reports(
                &scheduler,
                isolated_relation_events,
                algebraic_reports,
                policy,
            ) {
                Classification::Decided(replay) => replay,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
        Self::from_replay(scheduler, replay, policy)
    }

    /// Chains an existing replay report into construction readiness.
    pub fn from_replay(
        scheduler: BezierBooleanPathSchedulerReport2,
        replay: BezierBooleanRootIsolationReplayReport2,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let readiness = match BezierBooleanConstructionReadinessReport2::from_root_isolation_replay(
            scheduler, &replay, policy,
        ) {
            Classification::Decided(readiness) => readiness,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let status = if replay.has_blockers() {
            BezierBooleanRootIsolationConstructionStatus::ReplayBlocked
        } else {
            match readiness.status {
                BezierBooleanConstructionReadinessStatus::Empty => {
                    BezierBooleanRootIsolationConstructionStatus::Empty
                }
                BezierBooleanConstructionReadinessStatus::NoInteriorSplits => {
                    BezierBooleanRootIsolationConstructionStatus::NoInteriorSplits
                }
                BezierBooleanConstructionReadinessStatus::Ready => {
                    BezierBooleanRootIsolationConstructionStatus::Ready
                }
                BezierBooleanConstructionReadinessStatus::Blocked => {
                    BezierBooleanRootIsolationConstructionStatus::ReplayBlocked
                }
                BezierBooleanConstructionReadinessStatus::InvalidParameterDomain => {
                    BezierBooleanRootIsolationConstructionStatus::InvalidParameterDomain
                }
            }
        };
        let blocker_count = replay.blocker_count
            + usize::from(readiness.status == BezierBooleanConstructionReadinessStatus::Blocked)
            + readiness.insertion.out_of_range_parameter_count;

        Classification::Decided(Self {
            status,
            replay,
            readiness,
            blocker_count,
        })
    }

    /// Returns true when replayed roots reached interior split construction.
    pub fn is_ready(&self) -> bool {
        self.status == BezierBooleanRootIsolationConstructionStatus::Ready
    }

    /// Returns true when replay or construction retained blockers.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanRootIsolationConstructionStatus::ReplayBlocked
                | BezierBooleanRootIsolationConstructionStatus::InvalidParameterDomain
        )
    }
}

impl BezierPathRangeOrderReport2 {
    /// Builds a path-operation range report from a contact-classifying graph order.
    ///
    /// This conversion performs no new geometry. It only normalizes the exact
    /// graph-order payload into the left/right/contact/overlap/ambiguous states
    /// expected by a future monotone-range boolean arrangement.
    pub fn from_graph_contact_order(order: &BezierMonotoneGraphContactOrder) -> Self {
        match order {
            BezierMonotoneGraphContactOrder::NotSharedStrictlyMonotone => {
                Self::simple(BezierPathRangeOrderStatus::NotSharedMonotoneRange)
            }
            BezierMonotoneGraphContactOrder::Coincident => {
                Self::simple(BezierPathRangeOrderStatus::Overlap)
            }
            BezierMonotoneGraphContactOrder::FirstLess => {
                Self::simple(BezierPathRangeOrderStatus::FirstBeforeSecond)
            }
            BezierMonotoneGraphContactOrder::FirstGreater => {
                Self::simple(BezierPathRangeOrderStatus::FirstAfterSecond)
            }
            BezierMonotoneGraphContactOrder::IntersectsOrTouches { contacts, spans } => {
                let status = contact_status(contacts, spans);
                Self {
                    status,
                    contacts: contacts.clone(),
                    unclassified_parameters: Vec::new(),
                    isolating_spans: spans.clone(),
                    uncertainty_reason: None,
                }
            }
        }
    }

    /// Builds a path-operation range report from a graph order without contact kinds.
    ///
    /// Represented parameters from [`BezierMonotoneGraphOrder`] are retained as
    /// unclassified exact parameters because crossing/tangency has not been
    /// certified. Boolean callers may split at those parameters, but they should
    /// not infer entry/exit polarity until a contact-classifying predicate is
    /// available.
    pub fn from_graph_order(order: &BezierMonotoneGraphOrder) -> Self {
        match order {
            BezierMonotoneGraphOrder::NotSharedStrictlyMonotone => {
                Self::simple(BezierPathRangeOrderStatus::NotSharedMonotoneRange)
            }
            BezierMonotoneGraphOrder::Coincident => {
                Self::simple(BezierPathRangeOrderStatus::Overlap)
            }
            BezierMonotoneGraphOrder::FirstLess => {
                Self::simple(BezierPathRangeOrderStatus::FirstBeforeSecond)
            }
            BezierMonotoneGraphOrder::FirstGreater => {
                Self::simple(BezierPathRangeOrderStatus::FirstAfterSecond)
            }
            BezierMonotoneGraphOrder::IntersectsOrTouches { parameters, spans } => Self {
                status: BezierPathRangeOrderStatus::Ambiguous,
                contacts: Vec::new(),
                unclassified_parameters: parameters.clone(),
                isolating_spans: spans.clone(),
                uncertainty_reason: None,
            },
        }
    }

    /// Builds a report from a classified contact-order predicate.
    pub fn from_classified_graph_contact_order(
        order: &Classification<BezierMonotoneGraphContactOrder>,
    ) -> Self {
        match order {
            Classification::Decided(order) => Self::from_graph_contact_order(order),
            Classification::Uncertain(reason) => Self::uncertain(*reason),
        }
    }

    /// Builds a report from a classified graph-order predicate.
    pub fn from_classified_graph_order(order: &Classification<BezierMonotoneGraphOrder>) -> Self {
        match order {
            Classification::Decided(order) => Self::from_graph_order(order),
            Classification::Uncertain(reason) => Self::uncertain(*reason),
        }
    }

    /// Returns true when this range has a certified strict order and no split event.
    pub fn is_ordered(&self) -> bool {
        matches!(
            self.status,
            BezierPathRangeOrderStatus::FirstBeforeSecond
                | BezierPathRangeOrderStatus::FirstAfterSecond
        )
    }

    /// Returns true when another exact stage must refine this range before topology.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierPathRangeOrderStatus::NotSharedMonotoneRange
                | BezierPathRangeOrderStatus::Overlap
                | BezierPathRangeOrderStatus::Ambiguous
                | BezierPathRangeOrderStatus::Uncertain
        )
    }

    fn simple(status: BezierPathRangeOrderStatus) -> Self {
        Self {
            status,
            contacts: Vec::new(),
            unclassified_parameters: Vec::new(),
            isolating_spans: Vec::new(),
            uncertainty_reason: None,
        }
    }

    fn uncertain(reason: UncertaintyReason) -> Self {
        Self {
            status: BezierPathRangeOrderStatus::Uncertain,
            contacts: Vec::new(),
            unclassified_parameters: Vec::new(),
            isolating_spans: Vec::new(),
            uncertainty_reason: Some(reason),
        }
    }
}

fn contact_status(
    contacts: &[BezierGraphContact],
    spans: &[BezierMonotoneSpan],
) -> BezierPathRangeOrderStatus {
    if !spans.is_empty() {
        return BezierPathRangeOrderStatus::Ambiguous;
    }
    if contacts
        .iter()
        .any(|contact| contact.kind() == BezierLineContactKind::Crossing)
    {
        return BezierPathRangeOrderStatus::CrossingContact;
    }
    if contacts
        .iter()
        .any(|contact| contact.kind() == BezierLineContactKind::Tangent)
    {
        return BezierPathRangeOrderStatus::TangentContact;
    }
    BezierPathRangeOrderStatus::Ambiguous
}

impl BezierBooleanHandoffReport2 {
    /// Builds a boolean handoff report directly from a Bezier relation.
    ///
    /// This report does not run new geometry. It converts an already-certified
    /// relation into the data shape required by split-and-traverse booleans:
    /// point split events, overlap obligations, retained region obligations, or
    /// explicit blockers. Sederberg and Nishita's Bezier clipping cells
    /// ("Curve intersection using Bezier clipping," 1990) remain region
    /// obligations until a later algebraic isolator certifies represented
    /// roots.
    pub fn from_relation(relation: &BezierCurveRelation) -> Self {
        match relation {
            BezierCurveRelation::BoundingBoxesDisjoint | BezierCurveRelation::NoIntersection => {
                Self::no_events()
            }
            BezierCurveRelation::SameControlPolygon | BezierCurveRelation::SameCurveImage => {
                Self::overlap_relation()
            }
            BezierCurveRelation::SharedEndpoint => Self::parameter_recovery(1),
            BezierCurveRelation::EndpointIntersections { points }
            | BezierCurveRelation::IntersectionPoints { points } => {
                Self::parameter_recovery(points.len())
            }
            BezierCurveRelation::LineSegmentIntersection { intersection } => {
                Self::from_line_segment_intersection(intersection)
            }
            BezierCurveRelation::IntersectionRegions { regions } => Self::from_regions(regions),
            BezierCurveRelation::Unresolved => Self::unresolved(),
        }
    }

    /// Builds a boolean handoff from a classified Bezier relation.
    ///
    /// This is the convenience entry point for predicate APIs that return
    /// [`Classification`]. A classified uncertainty is retained as a boolean
    /// blocker instead of being collapsed into [`BezierCurveRelation::Unresolved`],
    /// preserving the difference between "the predicate could not decide" and
    /// "the relation was decided to need more algebra."
    pub fn from_classified_relation(relation: &Classification<BezierCurveRelation>) -> Self {
        match relation {
            Classification::Decided(relation) => Self::from_relation(relation),
            Classification::Uncertain(reason) => Self {
                status: BezierBooleanHandoffStatus::Uncertain,
                point_events: Vec::new(),
                overlap_events: Vec::new(),
                region_summary: None,
                isolation_certificate: None,
                point_witnesses_needing_parameters: 0,
                overlap_relations_needing_resolution: 0,
                unresolved_relations: 0,
                uncertain_relations: 1,
                uncertainty_reason: Some(*reason),
            },
        }
    }

    /// Builds a report from a retained-region isolation certificate.
    ///
    /// A certificate is split-ready only when every terminal cell is an exact
    /// point cell. Target-width satisfaction alone is not enough for boolean
    /// topology: Yap's model requires a certified combinatorial object, not a
    /// small numeric box.
    pub fn from_isolation_certificate(
        certificate: &BezierIntersectionRegionIsolationCertificate,
    ) -> Self {
        let exact_cells = certificate.terminal_summary.exact_point_cells;
        let terminal_count = certificate.terminal_region_count;
        let split_ready = terminal_count > 0
            && exact_cells == terminal_count
            && certificate.terminal_summary.invalid_spans == 0
            && certificate.terminal_summary.unknown_regions == 0;

        let status = if terminal_count == 0 {
            BezierBooleanHandoffStatus::NoEvents
        } else if split_ready {
            BezierBooleanHandoffStatus::SplitEventsReady
        } else {
            BezierBooleanHandoffStatus::NeedsRegionIsolation
        };

        Self {
            status,
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            region_summary: Some(certificate.terminal_summary.clone()),
            isolation_certificate: Some(certificate.clone()),
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 0,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }

    /// Returns true when the report can feed a split-event insertion stage.
    pub fn can_feed_split_events(&self) -> bool {
        self.status == BezierBooleanHandoffStatus::SplitEventsReady
    }

    /// Returns true when a later exact stage must run before boolean topology.
    pub fn has_blockers(&self) -> bool {
        matches!(
            self.status,
            BezierBooleanHandoffStatus::NeedsParameterRecovery
                | BezierBooleanHandoffStatus::NeedsOverlapResolver
                | BezierBooleanHandoffStatus::NeedsRegionIsolation
                | BezierBooleanHandoffStatus::Unresolved
                | BezierBooleanHandoffStatus::Uncertain
        )
    }

    fn no_events() -> Self {
        Self {
            status: BezierBooleanHandoffStatus::NoEvents,
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            region_summary: None,
            isolation_certificate: None,
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 0,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }

    fn parameter_recovery(count: usize) -> Self {
        Self {
            status: if count == 0 {
                BezierBooleanHandoffStatus::NoEvents
            } else {
                BezierBooleanHandoffStatus::NeedsParameterRecovery
            },
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            region_summary: None,
            isolation_certificate: None,
            point_witnesses_needing_parameters: count,
            overlap_relations_needing_resolution: 0,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }

    fn overlap_relation() -> Self {
        Self {
            status: BezierBooleanHandoffStatus::NeedsOverlapResolver,
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            region_summary: None,
            isolation_certificate: None,
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 1,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }

    fn unresolved() -> Self {
        Self {
            status: BezierBooleanHandoffStatus::Unresolved,
            point_events: Vec::new(),
            overlap_events: Vec::new(),
            region_summary: None,
            isolation_certificate: None,
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 0,
            unresolved_relations: 1,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }

    fn from_line_segment_intersection(intersection: &LineLineIntersection) -> Self {
        match intersection {
            LineLineIntersection::None => Self::no_events(),
            LineLineIntersection::Point {
                point,
                a_param,
                b_param,
                kind,
            } => Self {
                status: BezierBooleanHandoffStatus::SplitEventsReady,
                point_events: vec![BezierBooleanPointEvent2 {
                    first_param: a_param.clone(),
                    second_param: b_param.clone(),
                    point: Some(point.clone()),
                    kind: Some(*kind),
                }],
                overlap_events: Vec::new(),
                region_summary: None,
                isolation_certificate: None,
                point_witnesses_needing_parameters: 0,
                overlap_relations_needing_resolution: 0,
                unresolved_relations: 0,
                uncertain_relations: 0,
                uncertainty_reason: None,
            },
            LineLineIntersection::Overlap {
                a_range, b_range, ..
            } => Self {
                status: BezierBooleanHandoffStatus::NeedsOverlapResolver,
                point_events: Vec::new(),
                overlap_events: vec![BezierBooleanOverlapEvent2 {
                    first_range: a_range.clone(),
                    second_range: b_range.clone(),
                }],
                region_summary: None,
                isolation_certificate: None,
                point_witnesses_needing_parameters: 0,
                overlap_relations_needing_resolution: 1,
                unresolved_relations: 0,
                uncertain_relations: 0,
                uncertainty_reason: None,
            },
            LineLineIntersection::Uncertain { reason } => Self {
                status: BezierBooleanHandoffStatus::Uncertain,
                point_events: Vec::new(),
                overlap_events: Vec::new(),
                region_summary: None,
                isolation_certificate: None,
                point_witnesses_needing_parameters: 0,
                overlap_relations_needing_resolution: 0,
                unresolved_relations: 0,
                uncertain_relations: 1,
                uncertainty_reason: Some(*reason),
            },
        }
    }

    fn from_regions(regions: &[BezierCurveIntersectionRegion]) -> Self {
        let summary = crate::summarize_bezier_intersection_regions(regions);
        let split_ready = !regions.is_empty()
            && summary.exact_point_cells == regions.len()
            && summary.invalid_spans == 0
            && summary.unknown_regions == 0;
        let status = if regions.is_empty() {
            BezierBooleanHandoffStatus::NoEvents
        } else if split_ready {
            BezierBooleanHandoffStatus::SplitEventsReady
        } else {
            BezierBooleanHandoffStatus::NeedsRegionIsolation
        };
        let point_events = if split_ready {
            regions
                .iter()
                .filter_map(exact_point_event_from_region)
                .collect()
        } else {
            Vec::new()
        };

        Self {
            status,
            point_events,
            overlap_events: Vec::new(),
            region_summary: Some(summary),
            isolation_certificate: None,
            point_witnesses_needing_parameters: 0,
            overlap_relations_needing_resolution: 0,
            unresolved_relations: 0,
            uncertain_relations: 0,
            uncertainty_reason: None,
        }
    }
}

fn exact_point_event_from_region(
    region: &BezierCurveIntersectionRegion,
) -> Option<BezierBooleanPointEvent2> {
    let facts = crate::bezier_intersection_region_facts(region);
    if facts.shape != BezierIntersectionRegionShape::ExactPointCell {
        return None;
    }
    Some(BezierBooleanPointEvent2 {
        first_param: region.first().start().clone(),
        second_param: region.second().start().clone(),
        point: None,
        kind: None,
    })
}
