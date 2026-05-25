//! Continuous routing parameters delegated to `hypersolve`.
//!
//! PCB autorouters can use graph search, maze routing, SAT, or network flow to
//! propose topology, but length tuning is a continuous parameter problem once a
//! topology is fixed. This module creates deliberately small `hypersolve`
//! problems for exact extra-length variables and exact rectangular detour
//! candidates. The resulting geometry still has to be checked by path
//! predicates before it can become a route, matching Yap's separation between
//! numeric proposals and certified geometric decisions. For the routing
//! background, see Yan, Ma, and Wong, "Advances in PCB Routing," *Journal of
//! Information Processing* 2014.

use std::cmp::Ordering;

use hyperlimit::{Point2, PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;
use hypersolve::{
    CandidateCertificationReport, Constraint, Expr, PreparedProblem, Problem, SymbolId, VariableId,
    certify_candidate, context_from_problem,
};

use crate::offset::{LineOffsetError, OffsetSide, offset_axis_aligned_segment};
use crate::segment::{Axis, LinePathSegment};
use crate::solve::{constant_feed_time_equation, differential_pair_skew_equation};

mod feed;
mod lookahead;
mod orthogonal_keepout;

pub use feed::{
    CornerLookaheadJoinClass, CornerLookaheadJoinReport, CornerLookaheadLimitReport,
    FeedPathElement, JerkLimitedFeedTimeReport, certify_acceleration_limited_feed_time_for_path,
    certify_constant_feed_time_for_path, certify_corner_lookahead_limits,
    certify_symmetric_jerk_limited_feed_time, certify_symmetric_jerk_limited_feed_time_for_path,
};
pub use lookahead::{
    LookaheadFeedSchedule, LookaheadFeedScheduleReport, LookaheadSpanTransitionReport,
    certify_lookahead_feed_schedule,
};
use orthogonal_keepout::{
    segment_intersects_orthogonal_keepout, validate_orthogonal_keepout_vertices,
};

/// Exact length-match solve model for one continuous extension parameter.
#[derive(Clone, Debug)]
pub struct LengthMatchProblem {
    /// Solver problem containing the extension variable.
    pub problem: Problem,
    /// Symbol used for the extra length variable.
    pub extra_length_symbol: SymbolId,
}

/// Build a one-variable exact residual `current + extra - target = 0`.
pub fn build_length_match_problem(
    current: Real,
    target: Real,
    initial_extra: Real,
) -> LengthMatchProblem {
    let mut problem = Problem::default();
    let variable = problem.add_variable("extra_length", initial_extra);
    if let Some(row) = problem.variables.get_mut(variable.0 as usize) {
        row.lower = Some(Real::zero());
    }
    let symbol = SymbolId(variable.0);
    let residual = Expr::real(current) + Expr::symbol(symbol, "extra_length") - Expr::real(target);
    problem.add_constraint(Constraint::equality("length match", residual));
    LengthMatchProblem {
        problem,
        extra_length_symbol: symbol,
    }
}

/// Certify the current extra-length candidate by exact residual replay.
pub fn certify_length_extension(model: &LengthMatchProblem) -> CandidateCertificationReport {
    let prepared = PreparedProblem::new(&model.problem);
    let context = context_from_problem(&model.problem);
    certify_candidate(&prepared, &context)
}

/// Exact single-detour meander candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct SingleDetourMeander {
    /// Source segment that was length-tuned.
    pub source: LinePathSegment,
    /// Exact extra length requested.
    pub extra_length: Real,
    /// Exact detour amplitude, equal to `extra_length / 2`.
    pub amplitude: Real,
    /// Path segments in traversal order.
    pub segments: Vec<LinePathSegment>,
}

/// Exact repeated rectangular-detour meander candidate.
///
/// This carrier keeps the bump count and amplitude as exact retained structure
/// instead of flattening the tuned route to an opaque polyline. Lee, "An
/// Algorithm for Path Connections and Its Applications" (1961), and Hightower,
/// "A Solution to Line-Routing Problems on the Continuous Plane" (1969), are
/// candidate-search precedents; this object stores the proposed continuous
/// tuning parameter so `hypersolve` can replay the exact length residual before
/// the path is accepted.
#[derive(Clone, Debug, PartialEq)]
pub struct MultiDetourMeander {
    /// Source segment that was length-tuned.
    pub source: LinePathSegment,
    /// Exact extra length requested.
    pub extra_length: Real,
    /// Number of rectangular detour bumps.
    pub bump_count: u64,
    /// Exact detour amplitude, equal to `extra_length / (2 * bump_count)`.
    pub amplitude: Real,
    /// Path segments in traversal order.
    pub segments: Vec<LinePathSegment>,
}

/// Exact rectangular-detour meander with retained per-bump amplitudes.
///
/// This carrier is useful when an autorouter has already chosen uneven channel
/// excursions around obstacles. It keeps every amplitude exact and recomputes
/// the total extra length as `2 * sum(amplitudes)`, so `hypersolve` still
/// certifies the final length by residual replay instead of trusting the
/// proposal. This follows Yap's separation between constructed candidates and
/// certified decisions.
#[derive(Clone, Debug, PartialEq)]
pub struct NonUniformDetourMeander {
    /// Source segment that was length-tuned.
    pub source: LinePathSegment,
    /// Exact extra length contributed by all bumps.
    pub extra_length: Real,
    /// Exact per-bump amplitudes in traversal order.
    pub amplitudes: Vec<Real>,
    /// Path segments in traversal order.
    pub segments: Vec<LinePathSegment>,
}

/// Exact axis-aligned keepout used by meander side selection.
///
/// This is deliberately a routing-scheduler obstacle, not a full arrangement
/// primitive. Lee-style and Hightower-style autorouters can produce candidate
/// channels cheaply; under Yap's exact geometric computation model, this type
/// only rejects a rectangular bump side when exact AABB comparisons certify an
/// intersection with a retained keepout. Rich pad/trace geometry should still
/// be certified by the PCB predicates before output.
#[derive(Clone, Debug, PartialEq)]
pub struct MeanderObstacle {
    /// Exact minimum keepout corner.
    pub min: Point2,
    /// Exact maximum keepout corner.
    pub max: Point2,
}

/// Exact retained keepout shape used by meander placement.
///
/// This is still a routing-scheduler object, not a full copper or stock
/// boolean. Rectangular keepouts preserve the legacy [`MeanderObstacle`] model;
/// circular keepouts add exact disc predicates for vias, drills, round pads,
/// and machine exclusion zones without polygonal approximation. Orthogonal
/// polygon keepouts retain notched route blockages by exact vertex loops and
/// classify segment hits through rectilinear point-in-polygon and edge
/// intersection predicates. Lee/Hightower style search may propose these
/// candidates, but Yap's exact-computation boundary is preserved by rejecting
/// undecidable comparisons before a route is committed.
#[derive(Clone, Debug, PartialEq)]
pub enum MeanderKeepout {
    /// Axis-aligned rectangular keepout.
    Rectangular(MeanderObstacle),
    /// Circular/disc keepout with exact center and radius.
    Circular {
        /// Exact disc center.
        center: Point2,
        /// Exact nonnegative disc radius.
        radius: Real,
    },
    /// Simple orthogonal polygon keepout.
    OrthogonalPolygon {
        /// Retained rectilinear loop vertices in winding order.
        vertices: Vec<Point2>,
    },
}

/// Exact obstacle-aware rectangular-detour meander candidate.
///
/// The carrier records the side selected for each bump so an autorouter can
/// audit why a path alternated around keepouts. The emitted geometry remains a
/// candidate: length is certified through `hypersolve`, while final trace,
/// pad, via, and board-edge clearance remains in exact PCB predicates.
#[derive(Clone, Debug, PartialEq)]
pub struct ObstacleAwareDetourMeander {
    /// Repeated-detour geometry produced after side scheduling.
    pub meander: MultiDetourMeander,
    /// Side selected for each bump in traversal order.
    pub selected_sides: Vec<OffsetSide>,
    /// Obstacles considered during side selection.
    pub obstacles: Vec<MeanderObstacle>,
}

/// Exact rectangular-detour meander routed against generalized keepouts.
///
/// This is the retained-shape counterpart of [`ObstacleAwareDetourMeander`].
/// It records the exact keepout shapes used for side selection so downstream
/// route review can audit whether rectangular or circular predicate decisions
/// drove the emitted candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct KeepoutAwareDetourMeander {
    /// Repeated-detour geometry produced after side scheduling.
    pub meander: MultiDetourMeander,
    /// Side selected for each bump in traversal order.
    pub selected_sides: Vec<OffsetSide>,
    /// Keepouts considered during side selection.
    pub keepouts: Vec<MeanderKeepout>,
}

/// Exact caller-supplied meander placement candidate.
///
/// Unlike [`classify_meander_placement_slots`], this carrier does not require
/// equal source windows or a shared amplitude. It lets a router search stage
/// retain arbitrary axis-aligned windows and exact per-window amplitudes, then
/// ask the same Yap-style exact predicate boundary to classify which side is
/// blocked before path construction.
#[derive(Clone, Debug, PartialEq)]
pub struct MeanderPlacementCandidate {
    /// Exact source subsegment for the candidate bump.
    pub base: LinePathSegment,
    /// Exact detour amplitude for this candidate.
    pub amplitude: Real,
}

/// Exact placement decision for one rectangular meander bump window.
///
/// Earlier obstacle-aware construction selected a side per equal source split.
/// This report makes that placement stage explicit: each retained base window
/// records whether the preferred and opposite sides intersect keepouts and
/// which side, if any, remains available. It follows Yap's object/predicate
/// boundary by surfacing the exact routing decision instead of hiding it inside
/// a generated polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct MeanderPlacementSlot {
    /// Zero-based candidate window index.
    pub index: u64,
    /// Exact source subsegment for this candidate bump.
    pub base: LinePathSegment,
    /// Exact detour amplitude considered for this slot.
    pub amplitude: Real,
    /// Preferred side requested by the caller.
    pub preferred_side: OffsetSide,
    /// Whether the preferred side is exactly blocked by retained obstacles.
    pub preferred_blocked: bool,
    /// Opposite side considered as fallback.
    pub opposite_side: OffsetSide,
    /// Whether the opposite side is exactly blocked by retained obstacles.
    pub opposite_blocked: bool,
    /// Selected side when at least one side is available.
    pub selected_side: Option<OffsetSide>,
}

/// Exact placement report for equal rectangular-detour candidates.
#[derive(Clone, Debug, PartialEq)]
pub struct MeanderPlacementReport {
    /// Source segment being scheduled.
    pub source: LinePathSegment,
    /// Exact detour amplitude considered for every slot.
    pub amplitude: Real,
    /// Obstacles considered during scheduling.
    pub obstacles: Vec<MeanderObstacle>,
    /// Per-window placement decisions.
    pub slots: Vec<MeanderPlacementSlot>,
}

/// Exact placement report for arbitrary caller-supplied meander candidates.
#[derive(Clone, Debug, PartialEq)]
pub struct MeanderCandidatePlacementReport {
    /// Obstacles considered during scheduling.
    pub obstacles: Vec<MeanderObstacle>,
    /// Per-candidate placement decisions.
    pub slots: Vec<MeanderPlacementSlot>,
}

/// Exact placement report for equal candidates against generalized keepouts.
#[derive(Clone, Debug, PartialEq)]
pub struct MeanderKeepoutPlacementReport {
    /// Source segment being scheduled.
    pub source: LinePathSegment,
    /// Exact detour amplitude considered for every slot.
    pub amplitude: Real,
    /// Keepouts considered during scheduling.
    pub keepouts: Vec<MeanderKeepout>,
    /// Per-window placement decisions.
    pub slots: Vec<MeanderPlacementSlot>,
}

/// Exact placement report for arbitrary candidates against generalized keepouts.
#[derive(Clone, Debug, PartialEq)]
pub struct MeanderKeepoutCandidatePlacementReport {
    /// Keepouts considered during scheduling.
    pub keepouts: Vec<MeanderKeepout>,
    /// Per-candidate placement decisions.
    pub slots: Vec<MeanderPlacementSlot>,
}

/// Exact differential-pair length-skew certification report.
///
/// The route geometry stays in `hyperpath`: this report only sums exact
/// axis-aligned segment lengths and replays the continuous skew residual
/// through `hypersolve`. That mirrors Yap's object/predicate split and the PCB
/// routing literature surveyed by Yan, Ma, and Wong: search proposes route
/// topology, while exact predicates and residual replay decide whether the
/// candidate can be trusted.
#[derive(Clone, Debug)]
pub struct DifferentialPairSkewReport {
    /// Exact total length of the positive/first route.
    pub first_length: Real,
    /// Exact total length of the negative/second route.
    pub second_length: Real,
    /// Exact retained skew, `first_length - second_length`.
    pub actual_skew: Real,
    /// Exact requested skew target.
    pub target_skew: Real,
    /// Exact solver replay report for `first_length - second_length - target = 0`.
    pub certification: CandidateCertificationReport,
}

/// Exact constant-feed time certification report.
///
/// This is the first `hyperpath` feed/speed bridge: retained path geometry
/// supplies an exact axis-aligned length, while `hypersolve` replays the
/// affine residual `path_length - feed_rate * time = 0`. Rich acceleration,
/// jerk, controller blending, and material-process policies remain separate
/// machine-domain models rather than hidden tolerances in path geometry.
#[derive(Clone, Debug)]
pub struct ConstantFeedTimeReport {
    /// Exact retained route length.
    pub path_length: Real,
    /// Exact feed rate supplied to the replay model.
    pub feed_rate: Real,
    /// Exact candidate traversal time.
    pub target_time: Real,
    /// Exact solver replay report for `path_length - feed_rate * time = 0`.
    pub certification: CandidateCertificationReport,
}

/// Exact acceleration-limited feed profile class.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AccelerationLimitedFeedProfileClass {
    /// The move is too short to reach the requested maximum feed rate.
    Triangular,
    /// The move reaches maximum feed rate for a positive-length cruise span.
    Trapezoidal,
    /// The move reaches maximum feed rate exactly at the accel/decel switch.
    Boundary,
    /// Exact comparison could not choose a retained profile equation.
    Unknown,
}

/// Exact acceleration-limited traversal time certification report.
///
/// This report models a symmetric rest-to-rest speed profile over retained
/// axis-aligned path geometry. It classifies the profile by the exact
/// comparison `acceleration * path_length` against `max_feed^2`; then it
/// replays a denominator-free residual through `hypersolve`. For a triangular
/// profile the residual is `a*t^2 - 4L = 0`; for a trapezoidal profile it is
/// `a*v*t - a*L - v^2 = 0`. Keeping the profile equation explicit follows
/// Yap, "Towards Exact Geometric Computation," by separating proposed process
/// parameters from certified path facts. The rest-to-rest speed model is the
/// CAM/feed-rate counterpart of the hodograph-first view in Farouki,
/// *Pythagorean Hodograph Curves* (2008): speed laws are certified algebraic
/// objects, not sampled controller traces.
#[derive(Clone, Debug)]
pub struct AccelerationLimitedFeedTimeReport {
    /// Exact retained route length.
    pub path_length: Real,
    /// Exact maximum feed rate.
    pub max_feed_rate: Real,
    /// Exact positive acceleration limit.
    pub acceleration: Real,
    /// Exact candidate traversal time.
    pub target_time: Real,
    /// Certified profile class used to build the replay residual.
    pub profile: AccelerationLimitedFeedProfileClass,
    /// Exact solver replay report for the selected feed profile equation.
    pub certification: CandidateCertificationReport,
}

impl SingleDetourMeander {
    /// Return the exact total axis-aligned path length.
    pub fn exact_axis_length(&self, policy: PredicatePolicy) -> Result<Real, MeanderError> {
        exact_axis_length(&self.segments, policy)
    }

    /// Certify this meander against an exact target length through `hypersolve`.
    ///
    /// The geometric length is computed by exact axis-aligned segment lengths;
    /// the residual `current + extra - target = 0` is then replayed by
    /// `hypersolve`. This is the route-tuning version of Yap's proposed-object
    /// then certified-decision boundary.
    pub fn certify_target_length(
        &self,
        target: Real,
        policy: PredicatePolicy,
    ) -> Result<CandidateCertificationReport, MeanderError> {
        let current = self
            .source
            .axis_length(policy)
            .ok_or(MeanderError::UnsupportedSourceGeometry)?;
        let model = build_length_match_problem(current, target, self.extra_length.clone());
        Ok(certify_length_extension(&model))
    }
}

impl MultiDetourMeander {
    /// Return the exact total axis-aligned path length.
    pub fn exact_axis_length(&self, policy: PredicatePolicy) -> Result<Real, MeanderError> {
        exact_axis_length(&self.segments, policy)
    }

    /// Certify this repeated meander against an exact target length.
    ///
    /// The total route length is recomputed from exact segment geometry, while
    /// the continuous length variable is certified through the same
    /// `hypersolve` residual replay used by the single-detour carrier.
    pub fn certify_target_length(
        &self,
        target: Real,
        policy: PredicatePolicy,
    ) -> Result<CandidateCertificationReport, MeanderError> {
        let current = self
            .source
            .axis_length(policy)
            .ok_or(MeanderError::UnsupportedSourceGeometry)?;
        let model = build_length_match_problem(current, target, self.extra_length.clone());
        Ok(certify_length_extension(&model))
    }
}

impl NonUniformDetourMeander {
    /// Return the exact total axis-aligned path length.
    pub fn exact_axis_length(&self, policy: PredicatePolicy) -> Result<Real, MeanderError> {
        exact_axis_length(&self.segments, policy)
    }

    /// Certify this non-uniform meander against an exact target length.
    pub fn certify_target_length(
        &self,
        target: Real,
        policy: PredicatePolicy,
    ) -> Result<CandidateCertificationReport, MeanderError> {
        let current = self
            .source
            .axis_length(policy)
            .ok_or(MeanderError::UnsupportedSourceGeometry)?;
        let model = build_length_match_problem(current, target, self.extra_length.clone());
        Ok(certify_length_extension(&model))
    }
}

/// Errors while constructing an exact rectangular meander.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeanderError {
    /// The source segment was not a nondegenerate certified axis-aligned line.
    UnsupportedSourceGeometry,
    /// Extra length was structurally negative.
    NegativeExtraLength,
    /// One retained per-bump amplitude was structurally negative.
    NegativeAmplitude,
    /// Requested zero detour bumps for a repeated meander.
    ZeroBumps,
    /// Bump count cannot be represented in the exact scalar constructor.
    BumpCountTooLarge,
    /// Exact division by two failed.
    UnsupportedDivision,
    /// Offset construction failed.
    Offset(LineOffsetError),
    /// Obstacle bounds were not exactly ordered.
    InvalidObstacleBounds,
    /// Circular keepout radius was structurally negative.
    NegativeObstacleRadius,
    /// Orthogonal polygon keepout vertices were not a simple rectilinear loop.
    InvalidObstaclePolygon,
    /// A bump could not be placed on either side without hitting a keepout.
    ObstacleConflict,
    /// Obstacle intersection could not be decided exactly.
    ObstacleDecisionUnknown,
}

/// Errors while certifying exact route-level continuous parameters.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RouteCertificationError {
    /// At least one route had no retained path segments to measure.
    EmptyRoute,
    /// A segment could not provide a certified axis-aligned length.
    UnsupportedRouteGeometry,
    /// Feed rate was structurally negative.
    NegativeFeedRate,
    /// Feed rate was structurally zero.
    ZeroFeedRate,
    /// Acceleration limit was structurally negative.
    NegativeAcceleration,
    /// Acceleration limit was structurally zero.
    ZeroAcceleration,
    /// Candidate traversal time was structurally negative.
    NegativeTime,
    /// Exact comparison could not choose the acceleration-limited profile.
    UnknownFeedProfile,
    /// Jerk limit was structurally negative.
    NegativeJerk,
    /// Jerk limit was structurally zero.
    ZeroJerk,
    /// Exact scalar division failed while deriving feed-profile facts.
    UnsupportedDivision,
    /// Corner radius was structurally negative.
    NegativeCornerRadius,
    /// Corner radius was structurally zero.
    ZeroCornerRadius,
    /// Lookahead schedule vectors do not match the retained route shape.
    ScheduleShapeMismatch,
}

/// Build a one-bump rectangular meander from an exact extra length.
///
/// For a nonzero extra length, this replaces one axis-aligned source segment by
/// three axis-aligned segments: connector, parallel offset run, connector. The
/// added path length is exactly `2 * amplitude`, so `amplitude = extra / 2`.
/// The topology is intentionally simple; clearance against other traces, pads,
/// and board edges must still be certified by the exact PCB predicates.
pub fn build_single_detour_meander(
    source: &LinePathSegment,
    extra_length: Real,
    side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<SingleDetourMeander, MeanderError> {
    match extra_length.structural_facts().sign {
        Some(hyperreal::RealSign::Negative) => return Err(MeanderError::NegativeExtraLength),
        Some(hyperreal::RealSign::Zero) => {
            return Ok(SingleDetourMeander {
                source: source.clone(),
                extra_length,
                amplitude: Real::zero(),
                segments: vec![source.clone()],
            });
        }
        _ => {}
    }

    source
        .axis_length(policy)
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let amplitude =
        (extra_length.clone() / Real::from(2)).map_err(|_| MeanderError::UnsupportedDivision)?;
    let offset = offset_axis_aligned_segment(source, amplitude.clone(), side, policy)
        .map_err(MeanderError::Offset)?;
    let first = LinePathSegment::with_provenance(
        source.start().clone(),
        offset.segment.start().clone(),
        source.provenance(),
    );
    let last = LinePathSegment::with_provenance(
        offset.segment.end().clone(),
        source.end().clone(),
        source.provenance(),
    );
    Ok(SingleDetourMeander {
        source: source.clone(),
        extra_length,
        amplitude,
        segments: vec![first, offset.segment, last],
    })
}

/// Build a repeated rectangular meander with exact equal bump amplitudes.
///
/// The source is split into `bump_count` equal axis-aligned subsegments. Each
/// subsegment is replaced by connector, parallel offset run, and connector
/// pieces, adding exactly `2 * amplitude` of length per bump. The construction
/// is deliberately a candidate generator only: clearance, no-short, and
/// board-edge predicates must still certify the emitted swept geometry before a
/// router commits it, following Yap's exact-geometric-computation boundary.
pub fn build_multi_detour_meander(
    source: &LinePathSegment,
    extra_length: Real,
    bump_count: u64,
    side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<MultiDetourMeander, MeanderError> {
    build_multi_detour_meander_with_side(source, extra_length, bump_count, policy, |_| side)
}

/// Build a repeated rectangular meander with exact alternating sides.
///
/// This is the first topology variant beyond same-side equal bumps. It still
/// uses equal amplitudes so the continuous parameter remains
/// `extra / (2 * bumps)`, but adjacent bumps alternate around the source
/// segment to reduce one-sided excursions. Lee/Hightower-style routing can
/// propose this topology, while exact path predicates must still certify
/// clearance before output, matching Yap's proposed-object/certified-decision
/// split.
pub fn build_alternating_detour_meander(
    source: &LinePathSegment,
    extra_length: Real,
    bump_count: u64,
    first_side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<MultiDetourMeander, MeanderError> {
    build_multi_detour_meander_with_side(source, extra_length, bump_count, policy, |index| {
        if index % 2 == 0 {
            first_side
        } else {
            opposite_side(first_side)
        }
    })
}

/// Build a rectangular meander with caller-supplied exact amplitudes.
///
/// The source is split into as many equal subsegments as there are amplitudes.
/// Each nonzero amplitude creates one rectangular detour of that exact height,
/// adding `2 * amplitude` to the path. This is a candidate-construction layer:
/// obstacle avoidance may choose the amplitudes, but exact length replay and
/// clearance predicates still decide whether the route can be accepted.
pub fn build_nonuniform_detour_meander(
    source: &LinePathSegment,
    amplitudes: Vec<Real>,
    side: OffsetSide,
    policy: PredicatePolicy,
) -> Result<NonUniformDetourMeander, MeanderError> {
    build_nonuniform_detour_meander_with_side(source, amplitudes, policy, |_| side)
}

/// Build a repeated rectangular meander while choosing sides around keepouts.
///
/// Each bump first tries `preferred_side`; if any of that bump's exact
/// connector/offset segments intersects a retained [`MeanderObstacle`], the
/// opposite side is tried. If both sides are blocked, or if any obstacle
/// comparison is undecidable, construction fails instead of committing a
/// topology from sampled clearance. This is the PCB routing analogue of Yap's
/// proposed-object/certified-decision split and follows the Lee/Hightower
/// tradition by keeping search heuristic and exact validation separate.
pub fn build_obstacle_aware_detour_meander(
    source: &LinePathSegment,
    extra_length: Real,
    bump_count: u64,
    preferred_side: OffsetSide,
    obstacles: Vec<MeanderObstacle>,
    policy: PredicatePolicy,
) -> Result<ObstacleAwareDetourMeander, MeanderError> {
    validate_obstacles(&obstacles, policy)?;
    if bump_count == 0 {
        return Err(MeanderError::ZeroBumps);
    }
    match extra_length.structural_facts().sign {
        Some(hyperreal::RealSign::Negative) => return Err(MeanderError::NegativeExtraLength),
        Some(hyperreal::RealSign::Zero) => {
            let meander = MultiDetourMeander {
                source: source.clone(),
                extra_length,
                bump_count,
                amplitude: Real::zero(),
                segments: vec![source.clone()],
            };
            return Ok(ObstacleAwareDetourMeander {
                meander,
                selected_sides: Vec::new(),
                obstacles,
            });
        }
        _ => {}
    }

    source
        .axis_length(policy)
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let axis = source
        .facts()
        .axis_aligned
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let bump_count_i64 = i64::try_from(bump_count).map_err(|_| MeanderError::BumpCountTooLarge)?;
    let divisor = Real::from(2) * Real::from(bump_count_i64);
    let amplitude =
        (extra_length.clone() / divisor).map_err(|_| MeanderError::UnsupportedDivision)?;
    let bump_divisor = Real::from(bump_count_i64);
    let step_x = match axis {
        Axis::X => ((source.end().x.clone() - source.start().x.clone()) / bump_divisor.clone())
            .map_err(|_| MeanderError::UnsupportedDivision)?,
        Axis::Y => Real::zero(),
    };
    let step_y = match axis {
        Axis::X => Real::zero(),
        Axis::Y => ((source.end().y.clone() - source.start().y.clone()) / bump_divisor)
            .map_err(|_| MeanderError::UnsupportedDivision)?,
    };

    let selected_sides = classify_meander_placement_slots_with_step(
        source,
        amplitude.clone(),
        bump_count,
        preferred_side,
        obstacles.clone(),
        step_x,
        step_y,
        policy,
    )?
    .slots
    .into_iter()
    .map(|slot| slot.selected_side.ok_or(MeanderError::ObstacleConflict))
    .collect::<Result<Vec<_>, _>>()?;

    let meander =
        build_multi_detour_meander_with_side(source, extra_length, bump_count, policy, |index| {
            selected_sides[index as usize]
        })?;
    Ok(ObstacleAwareDetourMeander {
        meander,
        selected_sides,
        obstacles,
    })
}

/// Build a repeated rectangular meander while choosing sides around keepouts.
///
/// This generalized variant accepts retained rectangular and circular keepouts.
/// Circular keepouts are tested by exact segment-to-disc distance predicates
/// for the three candidate bump legs. That extends obstacle-aware routing
/// beyond rectangular keepouts without moving copper clipping, board booleans,
/// or pad topology into `hyperpath`.
pub fn build_keepout_aware_detour_meander(
    source: &LinePathSegment,
    extra_length: Real,
    bump_count: u64,
    preferred_side: OffsetSide,
    keepouts: Vec<MeanderKeepout>,
    policy: PredicatePolicy,
) -> Result<KeepoutAwareDetourMeander, MeanderError> {
    validate_meander_keepouts(&keepouts, policy)?;
    if bump_count == 0 {
        return Err(MeanderError::ZeroBumps);
    }
    match extra_length.structural_facts().sign {
        Some(hyperreal::RealSign::Negative) => return Err(MeanderError::NegativeExtraLength),
        Some(hyperreal::RealSign::Zero) => {
            let meander = MultiDetourMeander {
                source: source.clone(),
                extra_length,
                bump_count,
                amplitude: Real::zero(),
                segments: vec![source.clone()],
            };
            return Ok(KeepoutAwareDetourMeander {
                meander,
                selected_sides: Vec::new(),
                keepouts,
            });
        }
        _ => {}
    }

    source
        .axis_length(policy)
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let axis = source
        .facts()
        .axis_aligned
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let bump_count_i64 = i64::try_from(bump_count).map_err(|_| MeanderError::BumpCountTooLarge)?;
    let divisor = Real::from(2) * Real::from(bump_count_i64);
    let amplitude =
        (extra_length.clone() / divisor).map_err(|_| MeanderError::UnsupportedDivision)?;
    let bump_divisor = Real::from(bump_count_i64);
    let step_x = match axis {
        Axis::X => ((source.end().x.clone() - source.start().x.clone()) / bump_divisor.clone())
            .map_err(|_| MeanderError::UnsupportedDivision)?,
        Axis::Y => Real::zero(),
    };
    let step_y = match axis {
        Axis::X => Real::zero(),
        Axis::Y => ((source.end().y.clone() - source.start().y.clone()) / bump_divisor)
            .map_err(|_| MeanderError::UnsupportedDivision)?,
    };

    let selected_sides = classify_meander_placement_slots_with_keepout_step(
        source,
        amplitude,
        bump_count,
        preferred_side,
        keepouts.clone(),
        step_x,
        step_y,
        policy,
    )?
    .slots
    .into_iter()
    .map(|slot| slot.selected_side.ok_or(MeanderError::ObstacleConflict))
    .collect::<Result<Vec<_>, _>>()?;

    let meander =
        build_multi_detour_meander_with_side(source, extra_length, bump_count, policy, |index| {
            selected_sides[index as usize]
        })?;
    Ok(KeepoutAwareDetourMeander {
        meander,
        selected_sides,
        keepouts,
    })
}

/// Classify equal meander placement windows against retained obstacles.
///
/// The function does not emit route geometry. It only splits the source into
/// exact equal windows and classifies both candidate sides for each window,
/// making obstacle-aware placement auditable before a meander topology is
/// committed.
pub fn classify_meander_placement_slots(
    source: &LinePathSegment,
    amplitude: Real,
    bump_count: u64,
    preferred_side: OffsetSide,
    obstacles: Vec<MeanderObstacle>,
    policy: PredicatePolicy,
) -> Result<MeanderPlacementReport, MeanderError> {
    validate_obstacles(&obstacles, policy)?;
    if bump_count == 0 {
        return Err(MeanderError::ZeroBumps);
    }
    if amplitude.structural_facts().sign == Some(hyperreal::RealSign::Negative) {
        return Err(MeanderError::NegativeAmplitude);
    }
    source
        .axis_length(policy)
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let axis = source
        .facts()
        .axis_aligned
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let bump_count_i64 = i64::try_from(bump_count).map_err(|_| MeanderError::BumpCountTooLarge)?;
    let bump_divisor = Real::from(bump_count_i64);
    let step_x = match axis {
        Axis::X => ((source.end().x.clone() - source.start().x.clone()) / bump_divisor.clone())
            .map_err(|_| MeanderError::UnsupportedDivision)?,
        Axis::Y => Real::zero(),
    };
    let step_y = match axis {
        Axis::X => Real::zero(),
        Axis::Y => ((source.end().y.clone() - source.start().y.clone()) / bump_divisor)
            .map_err(|_| MeanderError::UnsupportedDivision)?,
    };
    classify_meander_placement_slots_with_step(
        source,
        amplitude,
        bump_count,
        preferred_side,
        obstacles,
        step_x,
        step_y,
        policy,
    )
}

/// Classify arbitrary exact meander placement candidates against keepouts.
///
/// This is the general predicate stage behind equal-window placement. It does
/// not require a uniform pitch or amplitude: each caller-supplied candidate
/// keeps its own axis-aligned base window and exact bump amplitude. The
/// function follows Lee/Hightower routing as candidate generation only, then
/// applies Yap's exact decision boundary by rejecting unsupported geometry,
/// negative amplitudes, invalid keepouts, and exactly blocked sides before any
/// emitted route is trusted.
pub fn classify_meander_candidate_slots(
    candidates: Vec<MeanderPlacementCandidate>,
    preferred_side: OffsetSide,
    obstacles: Vec<MeanderObstacle>,
    policy: PredicatePolicy,
) -> Result<MeanderCandidatePlacementReport, MeanderError> {
    validate_obstacles(&obstacles, policy)?;
    if candidates.is_empty() {
        return Err(MeanderError::ZeroBumps);
    }
    let slots = classify_meander_candidates(&candidates, preferred_side, &obstacles, policy)?;
    Ok(MeanderCandidatePlacementReport { obstacles, slots })
}

/// Classify equal meander placement windows against generalized keepouts.
///
/// Rectangular keepouts are tested by exact AABB separation. Circular keepouts
/// are tested by exact squared segment-to-disc distance, so round obstacles can
/// drive side selection without flattening to a sampled polygon.
pub fn classify_meander_placement_slots_with_keepouts(
    source: &LinePathSegment,
    amplitude: Real,
    bump_count: u64,
    preferred_side: OffsetSide,
    keepouts: Vec<MeanderKeepout>,
    policy: PredicatePolicy,
) -> Result<MeanderKeepoutPlacementReport, MeanderError> {
    validate_meander_keepouts(&keepouts, policy)?;
    if bump_count == 0 {
        return Err(MeanderError::ZeroBumps);
    }
    if amplitude.structural_facts().sign == Some(hyperreal::RealSign::Negative) {
        return Err(MeanderError::NegativeAmplitude);
    }
    source
        .axis_length(policy)
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let axis = source
        .facts()
        .axis_aligned
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let bump_count_i64 = i64::try_from(bump_count).map_err(|_| MeanderError::BumpCountTooLarge)?;
    let bump_divisor = Real::from(bump_count_i64);
    let step_x = match axis {
        Axis::X => ((source.end().x.clone() - source.start().x.clone()) / bump_divisor.clone())
            .map_err(|_| MeanderError::UnsupportedDivision)?,
        Axis::Y => Real::zero(),
    };
    let step_y = match axis {
        Axis::X => Real::zero(),
        Axis::Y => ((source.end().y.clone() - source.start().y.clone()) / bump_divisor)
            .map_err(|_| MeanderError::UnsupportedDivision)?,
    };
    classify_meander_placement_slots_with_keepout_step(
        source,
        amplitude,
        bump_count,
        preferred_side,
        keepouts,
        step_x,
        step_y,
        policy,
    )
}

/// Classify arbitrary exact meander placement candidates against keepouts.
///
/// This is the generalized retained-shape predicate layer behind
/// [`classify_meander_candidate_slots`]. It accepts the same exact candidate
/// windows but tests them against rectangular and circular keepouts.
pub fn classify_meander_candidate_slots_with_keepouts(
    candidates: Vec<MeanderPlacementCandidate>,
    preferred_side: OffsetSide,
    keepouts: Vec<MeanderKeepout>,
    policy: PredicatePolicy,
) -> Result<MeanderKeepoutCandidatePlacementReport, MeanderError> {
    validate_meander_keepouts(&keepouts, policy)?;
    if candidates.is_empty() {
        return Err(MeanderError::ZeroBumps);
    }
    let slots =
        classify_meander_candidates_with_keepouts(&candidates, preferred_side, &keepouts, policy)?;
    Ok(MeanderKeepoutCandidatePlacementReport { keepouts, slots })
}

/// Certify exact differential-pair skew for retained axis-aligned routes.
///
/// The residual is `first_length - second_length - target_skew = 0`, built by
/// [`hypersolve::differential_pair_skew_equation`] and replayed through the
/// normal exact candidate-certification path. This does not certify no-short,
/// clearance, pad/via transitions, impedance, or board-edge rules; those stay
/// in the dedicated exact PCB predicates before any route is accepted.
pub fn certify_differential_pair_skew(
    first_route: &[LinePathSegment],
    second_route: &[LinePathSegment],
    target_skew: Real,
    policy: PredicatePolicy,
) -> Result<DifferentialPairSkewReport, RouteCertificationError> {
    if first_route.is_empty() || second_route.is_empty() {
        return Err(RouteCertificationError::EmptyRoute);
    }
    let first_length = route_axis_length(first_route, policy)
        .ok_or(RouteCertificationError::UnsupportedRouteGeometry)?;
    let second_length = route_axis_length(second_route, policy)
        .ok_or(RouteCertificationError::UnsupportedRouteGeometry)?;
    let actual_skew = first_length.clone() - second_length.clone();
    let mut problem = Problem::default();
    problem.add_constraint(differential_pair_skew_equation(
        "differential pair skew",
        Expr::real(first_length.clone()),
        Expr::real(second_length.clone()),
        target_skew.clone(),
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    Ok(DifferentialPairSkewReport {
        first_length,
        second_length,
        actual_skew,
        target_skew,
        certification: certify_candidate(&prepared, &context),
    })
}

/// Certify a constant-feed traversal time for retained axis-aligned geometry.
///
/// The route is measured exactly from retained segment structure, then replayed
/// through [`hypersolve::constant_feed_time_equation`]. This follows Yap's
/// exact-geometric-computation boundary: path objects provide exact facts,
/// solver rows certify continuous parameters, and downstream CAM/export code
/// still owns process-specific feed, acceleration, and controller constraints.
pub fn certify_constant_feed_time(
    route: &[LinePathSegment],
    feed_rate: Real,
    target_time: Real,
    policy: PredicatePolicy,
) -> Result<ConstantFeedTimeReport, RouteCertificationError> {
    if route.is_empty() {
        return Err(RouteCertificationError::EmptyRoute);
    }
    match feed_rate.structural_facts().sign {
        Some(hyperreal::RealSign::Negative) => {
            return Err(RouteCertificationError::NegativeFeedRate);
        }
        Some(hyperreal::RealSign::Zero) => return Err(RouteCertificationError::ZeroFeedRate),
        _ => {}
    }
    if target_time.structural_facts().sign == Some(hyperreal::RealSign::Negative) {
        return Err(RouteCertificationError::NegativeTime);
    }
    let path_length = route_axis_length(route, policy)
        .ok_or(RouteCertificationError::UnsupportedRouteGeometry)?;
    let mut problem = Problem::default();
    let time = problem.add_variable("time", target_time.clone());
    problem.add_constraint(constant_feed_time_equation(
        "constant feed time",
        path_length.clone(),
        feed_rate.clone(),
        time,
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    Ok(ConstantFeedTimeReport {
        path_length,
        feed_rate,
        target_time,
        certification: certify_candidate(&prepared, &context),
    })
}

/// Certify a symmetric acceleration-limited feed traversal time.
///
/// The retained route is measured as exact axis-aligned length. The profile
/// starts and ends at rest and uses a single exact acceleration limit in both
/// directions. If `acceleration * path_length < max_feed_rate^2`, the move is
/// triangular and never cruises. If the comparison is greater, the move has a
/// trapezoidal cruise plateau. Equality is retained as an explicit boundary
/// class. This is process-parameter replay only: controller blending,
/// lookahead, jerk limits, cutter engagement, and material constraints remain
/// separate exact reports before any CAM output is accepted.
pub fn certify_acceleration_limited_feed_time(
    route: &[LinePathSegment],
    max_feed_rate: Real,
    acceleration: Real,
    target_time: Real,
    policy: PredicatePolicy,
) -> Result<AccelerationLimitedFeedTimeReport, RouteCertificationError> {
    if route.is_empty() {
        return Err(RouteCertificationError::EmptyRoute);
    }
    match max_feed_rate.structural_facts().sign {
        Some(hyperreal::RealSign::Negative) => {
            return Err(RouteCertificationError::NegativeFeedRate);
        }
        Some(hyperreal::RealSign::Zero) => return Err(RouteCertificationError::ZeroFeedRate),
        _ => {}
    }
    match acceleration.structural_facts().sign {
        Some(hyperreal::RealSign::Negative) => {
            return Err(RouteCertificationError::NegativeAcceleration);
        }
        Some(hyperreal::RealSign::Zero) => {
            return Err(RouteCertificationError::ZeroAcceleration);
        }
        _ => {}
    }
    if target_time.structural_facts().sign == Some(hyperreal::RealSign::Negative) {
        return Err(RouteCertificationError::NegativeTime);
    }
    let path_length = route_axis_length(route, policy)
        .ok_or(RouteCertificationError::UnsupportedRouteGeometry)?;
    let profile =
        classify_acceleration_limited_profile(&path_length, &max_feed_rate, &acceleration, policy)
            .ok_or(RouteCertificationError::UnknownFeedProfile)?;
    let mut problem = Problem::default();
    let time = problem.add_variable("time", target_time.clone());
    problem.add_constraint(acceleration_limited_feed_time_equation(
        "acceleration limited feed time",
        path_length.clone(),
        max_feed_rate.clone(),
        acceleration.clone(),
        time,
        profile,
    ));
    let prepared = PreparedProblem::new(&problem);
    let context = context_from_problem(&problem);
    Ok(AccelerationLimitedFeedTimeReport {
        path_length,
        max_feed_rate,
        acceleration,
        target_time,
        profile,
        certification: certify_candidate(&prepared, &context),
    })
}

fn build_nonuniform_detour_meander_with_side(
    source: &LinePathSegment,
    amplitudes: Vec<Real>,
    policy: PredicatePolicy,
    side_for_index: impl Fn(u64) -> OffsetSide,
) -> Result<NonUniformDetourMeander, MeanderError> {
    if amplitudes.is_empty() {
        return Err(MeanderError::ZeroBumps);
    }
    if amplitudes
        .iter()
        .any(|amplitude| amplitude.structural_facts().sign == Some(hyperreal::RealSign::Negative))
    {
        return Err(MeanderError::NegativeAmplitude);
    }
    let extra_length = amplitudes
        .iter()
        .cloned()
        .fold(Real::zero(), |sum, amplitude| {
            sum + amplitude * Real::from(2)
        });
    if extra_length.structural_facts().sign == Some(hyperreal::RealSign::Zero) {
        return Ok(NonUniformDetourMeander {
            source: source.clone(),
            extra_length,
            amplitudes,
            segments: vec![source.clone()],
        });
    }

    source
        .axis_length(policy)
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let axis = source
        .facts()
        .axis_aligned
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let bump_count =
        u64::try_from(amplitudes.len()).map_err(|_| MeanderError::BumpCountTooLarge)?;
    let bump_count_i64 = i64::try_from(bump_count).map_err(|_| MeanderError::BumpCountTooLarge)?;
    let bump_divisor = Real::from(bump_count_i64);
    let step_x = match axis {
        Axis::X => ((source.end().x.clone() - source.start().x.clone()) / bump_divisor.clone())
            .map_err(|_| MeanderError::UnsupportedDivision)?,
        Axis::Y => Real::zero(),
    };
    let step_y = match axis {
        Axis::X => Real::zero(),
        Axis::Y => ((source.end().y.clone() - source.start().y.clone()) / bump_divisor)
            .map_err(|_| MeanderError::UnsupportedDivision)?,
    };

    let mut segments = Vec::with_capacity(amplitudes.len() * 3);
    for (index, amplitude) in amplitudes.iter().enumerate() {
        if amplitude.structural_facts().sign == Some(hyperreal::RealSign::Zero) {
            let index = u64::try_from(index).map_err(|_| MeanderError::BumpCountTooLarge)?;
            let start = meander_split_point(source.start(), &step_x, &step_y, index)?;
            let end = meander_split_point(source.start(), &step_x, &step_y, index + 1)?;
            segments.push(LinePathSegment::with_provenance(
                start,
                end,
                source.provenance(),
            ));
            continue;
        }
        let index = u64::try_from(index).map_err(|_| MeanderError::BumpCountTooLarge)?;
        let start = meander_split_point(source.start(), &step_x, &step_y, index)?;
        let end = meander_split_point(source.start(), &step_x, &step_y, index + 1)?;
        let base = LinePathSegment::with_provenance(start, end, source.provenance());
        let offset =
            offset_axis_aligned_segment(&base, amplitude.clone(), side_for_index(index), policy)
                .map_err(MeanderError::Offset)?;
        segments.push(LinePathSegment::with_provenance(
            base.start().clone(),
            offset.segment.start().clone(),
            source.provenance(),
        ));
        segments.push(offset.segment.clone());
        segments.push(LinePathSegment::with_provenance(
            offset.segment.end().clone(),
            base.end().clone(),
            source.provenance(),
        ));
    }

    Ok(NonUniformDetourMeander {
        source: source.clone(),
        extra_length,
        amplitudes,
        segments,
    })
}

fn build_multi_detour_meander_with_side(
    source: &LinePathSegment,
    extra_length: Real,
    bump_count: u64,
    policy: PredicatePolicy,
    side_for_index: impl Fn(u64) -> OffsetSide,
) -> Result<MultiDetourMeander, MeanderError> {
    if bump_count == 0 {
        return Err(MeanderError::ZeroBumps);
    }
    match extra_length.structural_facts().sign {
        Some(hyperreal::RealSign::Negative) => return Err(MeanderError::NegativeExtraLength),
        Some(hyperreal::RealSign::Zero) => {
            return Ok(MultiDetourMeander {
                source: source.clone(),
                extra_length,
                bump_count,
                amplitude: Real::zero(),
                segments: vec![source.clone()],
            });
        }
        _ => {}
    }

    source
        .axis_length(policy)
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let axis = source
        .facts()
        .axis_aligned
        .ok_or(MeanderError::UnsupportedSourceGeometry)?;
    let bump_count_i64 = i64::try_from(bump_count).map_err(|_| MeanderError::BumpCountTooLarge)?;
    let divisor = Real::from(2) * Real::from(bump_count_i64);
    let amplitude =
        (extra_length.clone() / divisor).map_err(|_| MeanderError::UnsupportedDivision)?;
    let bump_divisor = Real::from(bump_count_i64);
    let step_x = match axis {
        Axis::X => ((source.end().x.clone() - source.start().x.clone()) / bump_divisor.clone())
            .map_err(|_| MeanderError::UnsupportedDivision)?,
        Axis::Y => Real::zero(),
    };
    let step_y = match axis {
        Axis::X => Real::zero(),
        Axis::Y => ((source.end().y.clone() - source.start().y.clone()) / bump_divisor)
            .map_err(|_| MeanderError::UnsupportedDivision)?,
    };

    let mut segments = Vec::with_capacity(bump_count as usize * 3);
    for index in 0..bump_count {
        let start = meander_split_point(source.start(), &step_x, &step_y, index)?;
        let end = meander_split_point(source.start(), &step_x, &step_y, index + 1)?;
        let base = LinePathSegment::with_provenance(start, end, source.provenance());
        let offset =
            offset_axis_aligned_segment(&base, amplitude.clone(), side_for_index(index), policy)
                .map_err(MeanderError::Offset)?;
        segments.push(LinePathSegment::with_provenance(
            base.start().clone(),
            offset.segment.start().clone(),
            source.provenance(),
        ));
        segments.push(offset.segment.clone());
        segments.push(LinePathSegment::with_provenance(
            offset.segment.end().clone(),
            base.end().clone(),
            source.provenance(),
        ));
    }

    Ok(MultiDetourMeander {
        source: source.clone(),
        extra_length,
        bump_count,
        amplitude,
        segments,
    })
}

fn opposite_side(side: OffsetSide) -> OffsetSide {
    match side {
        OffsetSide::Left => OffsetSide::Right,
        OffsetSide::Right => OffsetSide::Left,
    }
}

fn classify_meander_placement_slots_with_step(
    source: &LinePathSegment,
    amplitude: Real,
    bump_count: u64,
    preferred_side: OffsetSide,
    obstacles: Vec<MeanderObstacle>,
    step_x: Real,
    step_y: Real,
    policy: PredicatePolicy,
) -> Result<MeanderPlacementReport, MeanderError> {
    let mut candidates = Vec::with_capacity(bump_count as usize);
    for index in 0..bump_count {
        let start = meander_split_point(source.start(), &step_x, &step_y, index)?;
        let end = meander_split_point(source.start(), &step_x, &step_y, index + 1)?;
        let base = LinePathSegment::with_provenance(start, end, source.provenance());
        candidates.push(MeanderPlacementCandidate {
            base,
            amplitude: amplitude.clone(),
        });
    }
    let slots = classify_meander_candidates(&candidates, preferred_side, &obstacles, policy)?;
    Ok(MeanderPlacementReport {
        source: source.clone(),
        amplitude,
        obstacles,
        slots,
    })
}

fn classify_meander_placement_slots_with_keepout_step(
    source: &LinePathSegment,
    amplitude: Real,
    bump_count: u64,
    preferred_side: OffsetSide,
    keepouts: Vec<MeanderKeepout>,
    step_x: Real,
    step_y: Real,
    policy: PredicatePolicy,
) -> Result<MeanderKeepoutPlacementReport, MeanderError> {
    let mut candidates = Vec::with_capacity(bump_count as usize);
    for index in 0..bump_count {
        let start = meander_split_point(source.start(), &step_x, &step_y, index)?;
        let end = meander_split_point(source.start(), &step_x, &step_y, index + 1)?;
        let base = LinePathSegment::with_provenance(start, end, source.provenance());
        candidates.push(MeanderPlacementCandidate {
            base,
            amplitude: amplitude.clone(),
        });
    }
    let slots =
        classify_meander_candidates_with_keepouts(&candidates, preferred_side, &keepouts, policy)?;
    Ok(MeanderKeepoutPlacementReport {
        source: source.clone(),
        amplitude,
        keepouts,
        slots,
    })
}

fn classify_meander_candidates(
    candidates: &[MeanderPlacementCandidate],
    preferred_side: OffsetSide,
    obstacles: &[MeanderObstacle],
    policy: PredicatePolicy,
) -> Result<Vec<MeanderPlacementSlot>, MeanderError> {
    let keepouts = rectangular_keepouts(obstacles);
    classify_meander_candidates_with_keepouts(candidates, preferred_side, &keepouts, policy)
}

fn classify_meander_candidates_with_keepouts(
    candidates: &[MeanderPlacementCandidate],
    preferred_side: OffsetSide,
    keepouts: &[MeanderKeepout],
    policy: PredicatePolicy,
) -> Result<Vec<MeanderPlacementSlot>, MeanderError> {
    let mut slots = Vec::with_capacity(candidates.len());
    let opposite = opposite_side(preferred_side);
    for (index, candidate) in candidates.iter().enumerate() {
        if candidate.amplitude.structural_facts().sign == Some(hyperreal::RealSign::Negative) {
            return Err(MeanderError::NegativeAmplitude);
        }
        candidate
            .base
            .axis_length(policy)
            .ok_or(MeanderError::UnsupportedSourceGeometry)?;
        candidate
            .base
            .facts()
            .axis_aligned
            .ok_or(MeanderError::UnsupportedSourceGeometry)?;
        let preferred_blocked = candidate_bump_blocked(
            &candidate.base,
            &candidate.amplitude,
            preferred_side,
            keepouts,
            policy,
        )?;
        let opposite_blocked = candidate_bump_blocked(
            &candidate.base,
            &candidate.amplitude,
            opposite,
            keepouts,
            policy,
        )?;
        let selected_side = if !preferred_blocked {
            Some(preferred_side)
        } else if !opposite_blocked {
            Some(opposite)
        } else {
            None
        };
        let index = u64::try_from(index).map_err(|_| MeanderError::BumpCountTooLarge)?;
        slots.push(MeanderPlacementSlot {
            index,
            base: candidate.base.clone(),
            amplitude: candidate.amplitude.clone(),
            preferred_side,
            preferred_blocked,
            opposite_side: opposite,
            opposite_blocked,
            selected_side,
        });
    }
    Ok(slots)
}

fn candidate_bump_blocked(
    base: &LinePathSegment,
    amplitude: &Real,
    side: OffsetSide,
    keepouts: &[MeanderKeepout],
    policy: PredicatePolicy,
) -> Result<bool, MeanderError> {
    let offset = offset_axis_aligned_segment(base, amplitude.clone(), side, policy)
        .map_err(MeanderError::Offset)?;
    let offset_segment = offset.segment;
    let candidate_segments = [
        LinePathSegment::with_provenance(
            base.start().clone(),
            offset_segment.start().clone(),
            base.provenance(),
        ),
        offset_segment.clone(),
        LinePathSegment::with_provenance(
            offset_segment.end().clone(),
            base.end().clone(),
            base.provenance(),
        ),
    ];
    candidate_segments
        .iter()
        .try_fold(false, |blocked, segment| {
            if blocked {
                Ok(true)
            } else {
                segment_intersects_any_keepout(segment, keepouts, policy)
            }
        })
}

fn rectangular_keepouts(obstacles: &[MeanderObstacle]) -> Vec<MeanderKeepout> {
    obstacles
        .iter()
        .cloned()
        .map(MeanderKeepout::Rectangular)
        .collect()
}

fn validate_obstacles(
    obstacles: &[MeanderObstacle],
    policy: PredicatePolicy,
) -> Result<(), MeanderError> {
    for obstacle in obstacles {
        let ordered_x = compare_reals_with_policy(&obstacle.min.x, &obstacle.max.x, policy).value();
        let ordered_y = compare_reals_with_policy(&obstacle.min.y, &obstacle.max.y, policy).value();
        if !matches!(
            ordered_x,
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
        ) || !matches!(
            ordered_y,
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
        ) {
            return Err(MeanderError::InvalidObstacleBounds);
        }
    }
    Ok(())
}

pub(crate) fn validate_meander_keepouts(
    keepouts: &[MeanderKeepout],
    policy: PredicatePolicy,
) -> Result<(), MeanderError> {
    for keepout in keepouts {
        match keepout {
            MeanderKeepout::Rectangular(obstacle) => {
                validate_obstacles(std::slice::from_ref(obstacle), policy)?;
            }
            MeanderKeepout::Circular { radius, .. } => {
                if radius.structural_facts().sign == Some(hyperreal::RealSign::Negative) {
                    return Err(MeanderError::NegativeObstacleRadius);
                }
            }
            MeanderKeepout::OrthogonalPolygon { vertices } => {
                validate_orthogonal_keepout_vertices(vertices, policy)?;
            }
        }
    }
    Ok(())
}

fn segment_intersects_any_keepout(
    segment: &LinePathSegment,
    keepouts: &[MeanderKeepout],
    policy: PredicatePolicy,
) -> Result<bool, MeanderError> {
    keepouts.iter().try_fold(false, |blocked, keepout| {
        if blocked {
            Ok(true)
        } else {
            segment_intersects_keepout(segment, keepout, policy)
        }
    })
}

fn segment_intersects_keepout(
    segment: &LinePathSegment,
    keepout: &MeanderKeepout,
    policy: PredicatePolicy,
) -> Result<bool, MeanderError> {
    match keepout {
        MeanderKeepout::Rectangular(obstacle) => {
            segment_intersects_obstacle(segment, obstacle, policy)
        }
        MeanderKeepout::Circular { center, radius } => {
            segment_intersects_circular_keepout(segment, center, radius, policy)
        }
        MeanderKeepout::OrthogonalPolygon { vertices } => {
            segment_intersects_orthogonal_keepout(segment, vertices, policy)
        }
    }
}

fn segment_intersects_obstacle(
    segment: &LinePathSegment,
    obstacle: &MeanderObstacle,
    policy: PredicatePolicy,
) -> Result<bool, MeanderError> {
    let separated_left = strict_less_with_policy(&segment.bounds_max().x, &obstacle.min.x, policy)?;
    let separated_right =
        strict_less_with_policy(&obstacle.max.x, &segment.bounds_min().x, policy)?;
    let separated_below =
        strict_less_with_policy(&segment.bounds_max().y, &obstacle.min.y, policy)?;
    let separated_above =
        strict_less_with_policy(&obstacle.max.y, &segment.bounds_min().y, policy)?;
    Ok(!(separated_left || separated_right || separated_below || separated_above))
}

fn segment_intersects_circular_keepout(
    segment: &LinePathSegment,
    center: &Point2,
    radius: &Real,
    policy: PredicatePolicy,
) -> Result<bool, MeanderError> {
    let dx = distance_to_interval(
        &center.x,
        &segment.bounds_min().x,
        &segment.bounds_max().x,
        policy,
    )?;
    let dy = distance_to_interval(
        &center.y,
        &segment.bounds_min().y,
        &segment.bounds_max().y,
        policy,
    )?;
    let distance_squared = dx.clone() * dx + dy.clone() * dy;
    let radius_squared = radius.clone() * radius.clone();
    match compare_reals_with_policy(&distance_squared, &radius_squared, policy).value() {
        Some(Ordering::Less | Ordering::Equal) => Ok(true),
        Some(Ordering::Greater) => Ok(false),
        None => Err(MeanderError::ObstacleDecisionUnknown),
    }
}

fn distance_to_interval(
    coordinate: &Real,
    min: &Real,
    max: &Real,
    policy: PredicatePolicy,
) -> Result<Real, MeanderError> {
    match compare_reals_with_policy(coordinate, min, policy).value() {
        Some(Ordering::Less) => return Ok(min.clone() - coordinate.clone()),
        Some(Ordering::Equal | Ordering::Greater) => {}
        None => return Err(MeanderError::ObstacleDecisionUnknown),
    }
    match compare_reals_with_policy(max, coordinate, policy).value() {
        Some(Ordering::Less) => Ok(coordinate.clone() - max.clone()),
        Some(Ordering::Equal | Ordering::Greater) => Ok(Real::zero()),
        None => Err(MeanderError::ObstacleDecisionUnknown),
    }
}

fn strict_less_with_policy(
    left: &Real,
    right: &Real,
    policy: PredicatePolicy,
) -> Result<bool, MeanderError> {
    match hyperlimit::compare_reals_with_policy(left, right, policy).value() {
        Some(std::cmp::Ordering::Less) => Ok(true),
        Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater) => Ok(false),
        None => Err(MeanderError::ObstacleDecisionUnknown),
    }
}

fn exact_axis_length(
    segments: &[LinePathSegment],
    policy: PredicatePolicy,
) -> Result<Real, MeanderError> {
    segments
        .iter()
        .map(|segment| {
            segment
                .axis_length(policy)
                .ok_or(MeanderError::UnsupportedSourceGeometry)
        })
        .try_fold(Real::zero(), |sum, length| {
            length.map(|length| sum + length)
        })
}

fn route_axis_length(segments: &[LinePathSegment], policy: PredicatePolicy) -> Option<Real> {
    segments
        .iter()
        .map(|segment| segment.axis_length(policy))
        .try_fold(Real::zero(), |sum, length| {
            length.map(|length| sum + length)
        })
}

fn classify_acceleration_limited_profile(
    path_length: &Real,
    max_feed_rate: &Real,
    acceleration: &Real,
    policy: PredicatePolicy,
) -> Option<AccelerationLimitedFeedProfileClass> {
    let accel_distance_product = acceleration.clone() * path_length.clone();
    let feed_squared = max_feed_rate.clone() * max_feed_rate.clone();
    match compare_reals_with_policy(&accel_distance_product, &feed_squared, policy).value()? {
        Ordering::Less => Some(AccelerationLimitedFeedProfileClass::Triangular),
        Ordering::Equal => Some(AccelerationLimitedFeedProfileClass::Boundary),
        Ordering::Greater => Some(AccelerationLimitedFeedProfileClass::Trapezoidal),
    }
}

fn acceleration_limited_feed_time_equation(
    name: impl Into<String>,
    path_length: Real,
    max_feed_rate: Real,
    acceleration: Real,
    time: VariableId,
    profile: AccelerationLimitedFeedProfileClass,
) -> Constraint {
    let time_expr = Expr::symbol(time.into(), "time");
    match profile {
        AccelerationLimitedFeedProfileClass::Triangular => Constraint::equality(
            name,
            Expr::real(acceleration) * time_expr.clone() * time_expr
                - Expr::real(Real::from(4) * path_length),
        ),
        AccelerationLimitedFeedProfileClass::Boundary
        | AccelerationLimitedFeedProfileClass::Trapezoidal => Constraint::equality(
            name,
            Expr::real(acceleration.clone()) * Expr::real(max_feed_rate.clone()) * time_expr
                - Expr::real(acceleration * path_length)
                - Expr::real(max_feed_rate.clone() * max_feed_rate),
        ),
        AccelerationLimitedFeedProfileClass::Unknown => {
            Constraint::equality(name, Expr::real(Real::zero()))
        }
    }
}

fn meander_split_point(
    start: &Point2,
    step_x: &Real,
    step_y: &Real,
    index: u64,
) -> Result<Point2, MeanderError> {
    let index = i64::try_from(index).map_err(|_| MeanderError::BumpCountTooLarge)?;
    let scale = Real::from(index);
    Ok(Point2::new(
        start.x.clone() + step_x.clone() * scale.clone(),
        start.y.clone() + step_y.clone() * scale,
    ))
}
