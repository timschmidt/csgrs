//! Immediate exact Booleans over curved regions.

use crate::CurvePointData2;
use crate::curve_support::{CurveSupport2, retained_circular_support};
use std::cmp::Ordering;
use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

use crate::bezier_moment::RationalQuadraticAreaIntegralCache;
use crate::bezier_offset::{
    BezierAlgebraicChordAxisDirection2, BezierAlgebraicChordPairIntersections2,
    BezierAlgebraicChordParallelIntersections2, BezierAlgebraicChordParameter2,
    BezierAlgebraicChordRationalIntersections2, BezierAlgebraicChordRationalOverlap2,
    BezierAlgebraicCuspSemicircleRetainedChordContact2,
    BezierAlgebraicCuspSemicircleRetainedParallelContact2,
    BezierAlgebraicCuspSemicircleSelectedFiberContact2,
    BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2, BezierParallelRationalComponent2,
};
use crate::bezier_offset::{
    BezierAlgebraicCuspSemicirclePairIntersections2,
    BezierAlgebraicCuspSemicircleParallelIntersections2,
    BezierAlgebraicCuspSemicircleRationalIntersections2, BezierParameterComponentOverlap2,
};
use crate::bezier_split::{BezierSelectedFiberSource2, CurveParameterDomain2};
use crate::bezier_tangent_order::algebraic_endpoint_tangent_cross_sign;
use crate::classify::{compare_reals, real_sign};
use crate::curve_intersection::{
    CurveCircleOverlap2, CurveIntersectionBatchCache, CurveIntersectionContext,
    CurveOverlapCorrespondence2,
};
use crate::events::{MIN_AABB_SWEEP_PAIR_COUNT, visit_aabb_pair_candidates};
use crate::policy::resolve_certified_operation;
use crate::rational_bezier_general::{
    RationalBezierOverlapParameterCorrespondence2, exact_contact_point_evidence,
};
use crate::{
    Aabb2, ArcArcIntersection, Axis2, BezierArrangementFragment2, BezierArrangementGraph2,
    BezierEndpoint, BezierLineContactRelation, BezierLineCrossingDirection,
    BezierLineImageFitRelation, BezierParallel2, BezierParameter2, BezierParameterRange2,
    BezierSplitFragment2, BezierSubcurve2, BooleanOp, Classification, ContourPointLocation, Curve2,
    CurveContext, CurveError, CurveFamily2, CurveIntersectionContact2, CurveIntersectionOverlap2,
    CurveIntersectionPairBlocker2, CurveIntersectionPairBlockerKind2, CurveOperation2,
    CurveOutcome, CurveOverlapOrientation2, CurveParameter2, CurveParameterRange2, CurvePoint2,
    CurveRegion2, CurveRegionLoopRole, CurveResult, ExactCurveError, ExactCurveResult, FillRule,
    LineSeg2, LineSide, QuadraticBezier2, RationalBezier2, RationalBezierAlgebraicTangentImage2,
    RationalBezierIntersectionOverlap2, RationalBezierPointIncidence2, Real, RealSign,
    RegionPointLocation, Segment2, UncertaintyReason,
};

/// Region operand that owns one retained Boolean carrier.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveRegionBooleanOperand2 {
    /// Carrier originates in the first region.
    First,
    /// Carrier originates in the second region.
    Second,
}

/// An evaluable retained region carrier with its boundary provenance.
///
/// Parameters belong to [`Self::curve`]. Loop and fragment indices identify
/// the operation's prepared operand; region intersections regularize that
/// operand first. Preparation may choose a simpler chart.
/// All references to a carrier within one report share this exact curve.
#[derive(Clone, PartialEq)]
pub struct CurveRegionCarrier2 {
    data: Arc<CurveRegionCarrierData2>,
}

#[derive(PartialEq)]
struct CurveRegionCarrierData2 {
    curve: Curve2,
    carrier_index: usize,
    operand: CurveRegionBooleanOperand2,
    loop_index: usize,
    fragment_index: usize,
}

impl std::fmt::Debug for CurveRegionCarrier2 {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("CurveRegionCarrier2")
            .field("carrier_index", &self.carrier_index())
            .field("operand", &self.operand())
            .field("loop_index", &self.loop_index())
            .field("fragment_index", &self.fragment_index())
            .field("family", &self.curve().family())
            .finish_non_exhaustive()
    }
}

/// One exact contact between retained carriers from two curved regions.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionIntersectionContact2 {
    first: CurveRegionCarrier2,
    second: CurveRegionCarrier2,
    evidence: RegionPairContactEvidence,
}

/// One certified positive-length shared span between two curved regions.
///
/// The two ranges pair corresponding endpoints: both starts identify the same
/// point, as do both ends. Either range may descend in its source chart.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionIntersectionOverlap2 {
    first: CurveRegionCarrier2,
    second: CurveRegionCarrier2,
    overlap: CurveIntersectionOverlap2,
}

/// One incomplete retained carrier pair in a curved-region intersection result.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRegionIntersectionBlocker2 {
    first: CurveRegionCarrier2,
    second: CurveRegionCarrier2,
    blocker: RegionPairBlocker,
}

/// Clone-shared exact contact, overlap, and blocker result for two curved regions.
#[derive(Clone, Debug)]
pub struct CurveRegionIntersectionResult2 {
    data: Arc<CurveRegionIntersectionResultData>,
}

#[derive(Debug)]
struct CurveRegionIntersectionResultData {
    authored_carrier_pair_count: usize,
    candidate_carrier_pair_count: usize,
    contacts: Arc<[CurveRegionIntersectionContact2]>,
    overlaps: Arc<[CurveRegionIntersectionOverlap2]>,
    blockers: Arc<[CurveRegionIntersectionBlocker2]>,
}

/// The four exact regularized Boolean results for one region pair.
#[derive(Clone, Debug)]
pub struct CurveRegionBooleanResults2 {
    regions: Box<[CurveRegion2; 4]>,
    authored_carrier_pair_count: usize,
    candidate_carrier_pair_count: usize,
    topology_fragment_count: usize,
    topology_point_classification_count: usize,
}

#[derive(Debug)]
pub(crate) struct CurveRegionBooleanContext<'a> {
    data: CurveRegionBooleanContextData<'a>,
}

#[derive(Debug)]
struct CurveRegionBooleanContextData<'a> {
    first: &'a CurveRegion2,
    second: &'a CurveRegion2,
    policy: CurveContext,
    carriers: Vec<RegionCarrier>,
    first_carrier_count: usize,
    authored_carrier_pair_count: usize,
    pairs: Vec<RegionCarrierPair>,
    // Authored compound fills select from global signed winding. This rule
    // belongs to unary construction and is absent from published regions.
    regularization_fill_rule: Option<FillRule>,
    strict_line_image_only: OnceLock<bool>,
    operand_bounds: [OnceLock<Box<RegionOperandBounds>>; 2],
}

const CARRIER_BOUND_REFINEMENTS: [usize; 10] = [0, 2, 4, 8, 16, 32, 64, 128, 256, 512];

#[derive(Debug, Default)]
struct RegionOperandBounds {
    refinements: [OnceLock<Option<Aabb2>>; CARRIER_BOUND_REFINEMENTS.len()],
    /// Whether every carrier of this operand has an exact level-zero
    /// envelope that refinement cannot tighten.
    refinement_invariant: OnceLock<bool>,
}

/// An exactly straight carrier spanning its own exact endpoints (or exact
/// scalar parameters of an exact line image) has its endpoint hull as the
/// level-zero envelope; refinement cannot tighten it. Algebraic interior
/// endpoints keep conservative boxes and do not qualify.
fn carrier_bounds_refinement_invariant(carrier: &RegionCarrier) -> bool {
    match &carrier.geometry {
        CurveSupport2::Line(chord) => {
            let endpoint = |parameter: &CurveParameter2| {
                parameter
                    .as_algebraic_chord()
                    .and_then(|parameter| parameter.endpoint_of(chord))
            };
            chord.exact_line().is_some()
                && matches!(
                    (endpoint(&carrier.start), endpoint(&carrier.end)),
                    (Some(start), Some(end)) if start != end
                )
        }
        CurveSupport2::Bezier(BezierSubcurve2::Quadratic(curve)) => {
            carrier.start.scalar().is_some()
                && carrier.end.scalar().is_some()
                && curve.retained_exact_line_image().is_some()
        }
        _ => false,
    }
}

#[derive(Clone, Copy)]
enum RetainedPointProbeClassification {
    FilledRegion,
    LoopParity,
}

#[derive(Clone, Debug)]
struct RegionCarrier {
    operand: CurveRegionBooleanOperand2,
    loop_index: usize,
    fragment_index: usize,
    family: CurveFamily2,
    geometry: CurveSupport2,
    start: CurveParameter2,
    end: CurveParameter2,
    reversed: bool,
    filled_side_is_left: bool,
    selected_fiber_endpoint_points: Option<Arc<[CurvePoint2; 2]>>,
    image_is_injective: OnceLock<bool>,
    bounds: OnceLock<Classification<Aabb2>>,
    /// Refined envelopes, one per [`CARRIER_BOUND_REFINEMENTS`] level. Each
    /// is requested for every fragment of the carrier that the coarser
    /// levels could not separate.
    refined_bounds: [OnceLock<Classification<Aabb2>>; CARRIER_BOUND_REFINEMENTS.len()],
}

impl RegionCarrier {
    fn range(&self) -> CurveParameterRange2 {
        CurveParameterRange2::new_validated(self.start.clone(), self.end.clone())
    }
}

#[derive(Debug)]
struct RegionCarrierPair {
    first_carrier_index: usize,
    second_carrier_index: usize,
    context: RegionCarrierPairContext,
}

#[derive(Debug)]
enum RegionCarrierPairContext {
    Common(CurveIntersectionContext),
    ParallelRational {
        parallel_is_first: bool,
    },
    ParallelPair,
    ParallelSameImage,
    AlgebraicChordPair {
        endpoint_contact: Option<Box<RegionPairContactEvidence>>,
    },
    CuspChord {
        cusp_is_first: bool,
    },
    CuspRational {
        cusp_is_first: bool,
    },
    CuspParallel {
        cusp_is_first: bool,
    },
    CuspPair,
}

#[derive(Clone, Debug, PartialEq)]
struct RegionPairContactEvidence {
    first_parameter: CurveParameter2,
    second_parameter: CurveParameter2,
    point: Option<CurvePoint2>,
    certified_transverse: bool,
    tangent_cross_sign: Option<RealSign>,
    tangent_dot_sign: Option<RealSign>,
    second_side_of_first: Option<LineSide>,
}

#[derive(Clone, Debug, PartialEq)]
enum RegionPairBlocker {
    Common(CurveIntersectionPairBlocker2),
    Uncertain(UncertaintyReason),
    IncompleteReplay,
    PointImageParameterComponent,
}

#[derive(Clone, Debug)]
struct RegionPairResult {
    contacts: Vec<RegionPairContactEvidence>,
    overlaps: Vec<CurveIntersectionOverlap2>,
    blockers: Vec<RegionPairBlocker>,
}

impl RegionPairResult {
    fn empty() -> Self {
        Self {
            contacts: Vec::new(),
            overlaps: Vec::new(),
            blockers: Vec::new(),
        }
    }
}

#[derive(Clone, Debug)]
struct CarrierEvent {
    parameter: CurveParameter2,
    topology_vertex: Option<usize>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CarrierParameterLocation {
    Outside,
    Endpoint(BezierEndpoint),
    Interior,
}

#[derive(Clone, Debug)]
struct ContactVertex {
    point: Option<CurvePoint2>,
    topology_vertex: usize,
    carrier_indices: [usize; 2],
    parameters: [CurveParameter2; 2],
}

#[derive(Clone, Debug)]
struct CarrierOverlap {
    first_carrier_index: usize,
    second_carrier_index: usize,
    first_range: CurveParameterRange2,
    second_range: CurveParameterRange2,
    first_endpoint_vertices: [usize; 2],
    second_endpoint_vertices: [usize; 2],
    orientation: CurveOverlapOrientation2,
}

impl CarrierOverlap {
    /// Endpoint vertices of one overlap side. A self-overlap has distinct
    /// sides on the same carrier, so the side is explicit.
    fn endpoint_vertices(&self, second: bool) -> [usize; 2] {
        if second {
            self.second_endpoint_vertices
        } else {
            self.first_endpoint_vertices
        }
    }

    fn replace_topology_vertex(&mut self, from: usize, to: usize) {
        for vertex in self
            .first_endpoint_vertices
            .iter_mut()
            .chain(&mut self.second_endpoint_vertices)
        {
            if *vertex == from {
                *vertex = to;
            }
        }
    }
}

/// Returns the contiguous split-edge interval bounded by the exact overlap
/// endpoint vertices. A repeated vertex at more than one carrier boundary is
/// ambiguous (for example at a pinched self-contact), so callers retain their
/// scalar range fallback for that uncommon case.
fn carrier_overlap_split_interval(
    overlap: &CarrierOverlap,
    second: bool,
    splits: &[SplitCarrierFragment],
) -> Option<std::ops::Range<usize>> {
    let [first_vertex, second_vertex] = overlap.endpoint_vertices(second);
    let boundary_position = |vertex| {
        let mut position = None;
        for (split_index, split) in splits.iter().enumerate() {
            for (candidate, candidate_position) in [
                (split.start_topology_vertex, split_index),
                (split.end_topology_vertex, split_index + 1),
            ] {
                if candidate != Some(vertex) {
                    continue;
                }
                match position {
                    Some(existing) if existing != candidate_position => return None,
                    Some(_) => {}
                    None => position = Some(candidate_position),
                }
            }
        }
        position
    };
    let first = boundary_position(first_vertex)?;
    let second = boundary_position(second_vertex)?;
    (first != second).then(|| first.min(second)..first.max(second))
}

#[derive(Clone, Debug)]
struct TransitionContactCandidate {
    first_carrier: usize,
    second_carrier: usize,
    /// Both contact parameters lie strictly inside their individual carrier
    /// domains. Only these contacts can seed per-carrier Boolean locations;
    /// endpoint contacts instead require the actual incident branch order in
    /// the regularization face kernel.
    interior_on_both_carriers: bool,
    certified_transverse: bool,
    cross_is_positive: Option<bool>,
    tangent_dot_is_positive: Option<bool>,
    second_side_of_first: Option<LineSide>,
    self_parameters: Option<[CurveParameter2; 2]>,
}

#[derive(Clone, Debug)]
struct SplitCarrierFragment {
    fragment: BezierSplitFragment2,
    start_topology_vertex: Option<usize>,
    end_topology_vertex: Option<usize>,
}

#[derive(Clone, Debug)]
struct ClassifiedSplitCarrierFragment {
    split: SplitCarrierFragment,
    location: Option<RegionPointLocation>,
}

#[derive(Clone, Copy, Debug)]
struct BooleanArrangementFragmentDirection {
    carrier_index: usize,
    follows_carrier: bool,
    start_contact_branch: Option<TransitionContactBranch>,
    end_contact_branch: Option<TransitionContactBranch>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TransitionContactBranch {
    First,
    Second,
}

#[derive(Clone, Copy, Debug)]
struct CertifiedContactDirection {
    branch: TransitionContactBranch,
    follows_carrier: bool,
}

#[derive(Clone, Debug)]
struct CurveRegionBooleanTopology {
    split_fragments: Vec<Vec<ClassifiedSplitCarrierFragment>>,
    overlaps: Vec<CarrierOverlap>,
    transverse_contacts: HashMap<usize, TransitionContactCandidate>,
    point_classification_count: usize,
}

#[derive(Clone, Debug)]
struct CurveRegionSplitTopology {
    split_fragments: Vec<Vec<SplitCarrierFragment>>,
    overlaps: Vec<CarrierOverlap>,
    contact_candidates: HashMap<usize, TransitionContactCandidate>,
    transverse_contacts: HashMap<usize, TransitionContactCandidate>,
    transverse_vertices: Vec<bool>,
    reclassification_vertices: Vec<bool>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RegionFragmentAction {
    Discard,
    Keep,
    KeepReversed,
}

#[derive(Clone, Debug)]
struct RegularizedFragmentDecision {
    action: RegionFragmentAction,
    side_windings: [Vec<i32>; 2],
}

const NO_REGULARIZED_EDGE: usize = usize::MAX;
const AMBIGUOUS_REGULARIZED_EDGE: usize = usize::MAX - 1;

#[derive(Debug)]
struct RegularizedFragmentSelection {
    actions: Vec<Vec<RegionFragmentAction>>,
    /// Unique retained successors proved by the same local face-sector links
    /// that decided the actions, indexed by the flattened source edge.
    successor_edge_ids: Vec<usize>,
}

impl RegularizedFragmentDecision {
    fn from_classified_sides(
        left: (Vec<i32>, RegionPointLocation),
        right: (Vec<i32>, RegionPointLocation),
    ) -> Self {
        Self {
            action: action_from_result_sides(
                left.1 == RegionPointLocation::Inside,
                right.1 == RegionPointLocation::Inside,
            ),
            side_windings: [left.0, right.0],
        }
    }
}

fn record_regularized_vertex_sectors(
    vertex_sector_links: &mut Vec<(usize, usize, usize)>,
    vertex: usize,
    pairs: &[(usize, usize)],
) {
    vertex_sector_links.extend(pairs.iter().map(|&(first, second)| (vertex, first, second)));
}

/// Orders two co-directed rays leaving one vertex by exact signed curvature
/// `kappa = (T x A) / |T|^3`, in the clockwise sector convention: the ray
/// departing further left (larger signed curvature) comes first. Returns
/// whether `first` precedes `second`, or `None` when the curvatures are equal
/// or unavailable. Only materialized Bezier rays and exact chords carry the
/// needed derivatives; other carriers decline.
fn co_directed_ray_precedes(
    first: &BezierSplitFragment2,
    second: &BezierSplitFragment2,
    policy: &CurveContext,
) -> Option<bool> {
    let derivatives = |ray: &BezierSplitFragment2| -> Option<[(Real, Real); 2]> {
        match ray {
            BezierSplitFragment2::Materialized { curve, .. } => {
                let curve = RationalBezier2::try_from_subcurve(curve).ok()?;
                let Classification::Decided(values) =
                    curve.derivatives_at_classified(&Real::zero(), 2, policy)
                else {
                    return None;
                };
                let [first, second] = values.as_slice() else {
                    return None;
                };
                Some([
                    (first.dx().clone(), first.dy().clone()),
                    (second.dx().clone(), second.dy().clone()),
                ])
            }
            BezierSplitFragment2::AlgebraicChord(chord) => {
                let line = chord.exact_line()?;
                Some([
                    (
                        line.end().x() - line.start().x(),
                        line.end().y() - line.start().y(),
                    ),
                    (Real::zero(), Real::zero()),
                ])
            }
            _ => None,
        }
    };
    let curvature_parts = |[(tx, ty), (ax, ay)]: [(Real, Real); 2]| {
        let cross = &tx * &ay - &ty * &ax;
        let speed_squared = &tx * &tx + &ty * &ty;
        (cross, speed_squared)
    };
    let (first_cross, first_speed) = curvature_parts(derivatives(first)?);
    let (second_cross, second_speed) = curvature_parts(derivatives(second)?);
    if real_sign(&first_speed, policy)? != RealSign::Positive
        || real_sign(&second_speed, policy)? != RealSign::Positive
    {
        return None;
    }
    let first_sign = real_sign(&first_cross, policy)?;
    let second_sign = real_sign(&second_cross, policy)?;
    let rank = |sign: RealSign| match sign {
        RealSign::Negative => 0,
        RealSign::Zero => 1,
        RealSign::Positive => 2,
    };
    if first_sign != second_sign {
        return Some(rank(first_sign) > rank(second_sign));
    }
    if first_sign == RealSign::Zero {
        return None;
    }
    // Same side: |kappa_1| vs |kappa_2| through
    // cross_1^2 |T_2|^6 vs cross_2^2 |T_1|^6.
    let cube = |value: &Real| value * value * value;
    let first_magnitude = &first_cross * &first_cross * cube(&second_speed);
    let second_magnitude = &second_cross * &second_cross * cube(&first_speed);
    let first_tighter = match real_sign(&(first_magnitude - second_magnitude), policy)? {
        RealSign::Positive => true,
        RealSign::Negative => false,
        RealSign::Zero => return None,
    };
    // A tighter left turn departs further left; a tighter right turn further right.
    Some(first_tighter == (first_sign == RealSign::Positive))
}

/// Face sectors at one vertex, plus side equalities of coincident rays.
struct RegularizedVertexSectors {
    /// Certified cyclic sectors, published only when no rays coincide.
    sectors: Option<Vec<(usize, usize)>>,
    /// Zero-jump face equations: every sector pair and, for coincident
    /// straight rays, their equal left and right sides.
    winding_links: Vec<(usize, usize)>,
}

/// Orders the actual rays at an authored corner. A support's crossing or
/// tangency certificate does not describe a different carrier joined to it.
/// Co-directed curved rays need higher-order contact evidence and decline
/// this path. Co-directed straight rays coincide near the vertex, so their
/// sides are the same local faces: they are merged into one ray and their
/// side equalities published as winding links only, leaving successor
/// selection at such a vertex to the other exact routes.
fn regularized_incident_ray_sectors(
    incident: &[(usize, usize, bool)],
    topology: &CurveRegionSplitTopology,
    edge_offsets: &[usize],
    policy: &CurveContext,
) -> Option<RegularizedVertexSectors> {
    use crate::bezier_region::CurveTangent2;

    policy.strict_predicate_pass(|| {
        let reference = CurveTangent2::RepresentedDirection((Real::one(), Real::zero()));
        let mut rays: Vec<(CurveTangent2, usize, usize, bool, BezierSplitFragment2)> =
            Vec::with_capacity(incident.len());
        let mut coincident = Vec::new();
        for &(carrier, split, outgoing) in incident {
            let fragment = &topology.split_fragments[carrier][split].fragment;
            let straight = split_fragment_is_affine_line(fragment);
            let ray = if outgoing {
                fragment.clone()
            } else {
                fragment.reversed().ok()?
            };
            let Classification::Decided(tangent) =
                CurveTangent2::at_boundary_endpoint(&ray, true, policy).ok()?
            else {
                return None;
            };
            let edge = edge_offsets[carrier] + split;
            let left = 2 * edge;
            let right = left + 1;
            let (left, right) = if outgoing {
                (left, right)
            } else {
                (right, left)
            };
            let mut position = rays.len();
            let mut merged = false;
            for (index, (other, other_left, other_right, other_straight, other_ray)) in
                rays.iter().enumerate()
            {
                match reference.compare_filled_left_turn(&tangent, other, policy) {
                    Classification::Decided(Ordering::Less) => {
                        position = index;
                        break;
                    }
                    Classification::Decided(Ordering::Greater) => {}
                    Classification::Decided(Ordering::Equal) if straight && *other_straight => {
                        coincident.push((left, *other_left));
                        coincident.push((right, *other_right));
                        merged = true;
                        break;
                    }
                    Classification::Decided(Ordering::Equal) => {
                        // Co-directed but not both straight: order by exact
                        // curvature (then third order) of the departing rays.
                        if co_directed_ray_precedes(&ray, other_ray, policy)? {
                            position = index;
                            break;
                        }
                    }
                    Classification::Uncertain(_) => return None,
                }
            }
            if !merged {
                rays.insert(position, (tangent, left, right, straight, ray));
            }
        }
        // The turn comparator orders clockwise. The sector between adjacent
        // rays is right of the first and left of the second, including the
        // wraparound sector. Publish only a completely certified ordering.
        let sectors = (0..rays.len())
            .map(|index| (rays[index].2, rays[(index + 1) % rays.len()].1))
            .collect::<Vec<_>>();
        let mut winding_links = sectors.clone();
        winding_links.extend(coincident.iter().copied());
        Some(RegularizedVertexSectors {
            sectors: coincident.is_empty().then_some(sectors),
            winding_links,
        })
    })
}

#[derive(Clone, Copy, Debug)]
struct RegularizedFaceAdjacency {
    face: usize,
    jump: usize,
    direction: i8,
}

#[derive(Clone, Debug)]
enum RegularizedWindingJump {
    Zero,
    Single(usize),
    Aggregate(Box<[(usize, i32)]>),
}

/// Per-loop winding numbers of one arrangement face, stored sparsely.
///
/// A face lies inside few of a region's loops, while dense vectors cost the
/// loop count per face for every clone and comparison during propagation.
/// Entries are nonzero and sorted by loop, so equality is structural.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct LoopWindings {
    loop_count: usize,
    entries: Vec<(usize, i32)>,
}

impl LoopWindings {
    pub(crate) fn from_dense(windings: &[i32]) -> Self {
        Self {
            loop_count: windings.len(),
            entries: windings
                .iter()
                .copied()
                .enumerate()
                .filter(|(_, winding)| *winding != 0)
                .collect(),
        }
    }

    #[cfg(test)]
    fn to_dense(&self) -> Vec<i32> {
        let mut dense = vec![0; self.loop_count];
        for &(loop_index, winding) in &self.entries {
            dense[loop_index] = winding;
        }
        dense
    }

    pub(crate) const fn loop_count(&self) -> usize {
        self.loop_count
    }

    /// Nonzero `(loop, winding)` entries in loop order.
    pub(crate) fn entries(&self) -> &[(usize, i32)] {
        &self.entries
    }

    fn add(&mut self, loop_index: usize, delta: i32) -> Result<(), CurveError> {
        if loop_index >= self.loop_count {
            return Err(CurveError::Topology(
                "regularized arrangement edge references a missing loop".into(),
            ));
        }
        match self
            .entries
            .binary_search_by_key(&loop_index, |&(index, _)| index)
        {
            Ok(position) => {
                let winding = self.entries[position].1.checked_add(delta).ok_or_else(|| {
                    CurveError::Topology("regularized arrangement winding overflowed i32".into())
                })?;
                if winding == 0 {
                    self.entries.remove(position);
                } else {
                    self.entries[position].1 = winding;
                }
            }
            Err(position) => {
                if delta != 0 {
                    self.entries.insert(position, (loop_index, delta));
                }
            }
        }
        Ok(())
    }
}

fn propagate_regularized_face_windings(
    face_windings: &mut [Option<LoopWindings>],
    equation_faces_valid: &[bool],
    adjacency: &[Vec<RegularizedFaceAdjacency>],
    jumps: &[RegularizedWindingJump],
    seeds: impl IntoIterator<Item = (usize, LoopWindings)>,
) -> Result<Option<usize>, CurveError> {
    // Stage a complete propagation before publishing it. These sparse
    // equations are an accelerator over the authoritative geometric side
    // classifier, and contact-sector graphs can conservatively join distinct
    // local faces. A contradictory component must therefore decline the
    // acceleration without leaving a partially assigned winding graph.
    if equation_faces_valid.len() != face_windings.len() || adjacency.len() != face_windings.len() {
        return Err(CurveError::Topology(
            "regularized arrangement face storage is inconsistent".into(),
        ));
    }
    // Stage only the faces this propagation reaches; the arrangement can hold
    // far more faces than one seed's component.
    let mut staged = HashMap::<usize, LoopWindings>::new();
    let mut queue = std::collections::VecDeque::new();
    for (face, winding) in seeds {
        let Some(published) = face_windings.get(face) else {
            return Err(CurveError::Topology(
                "regularized arrangement referenced a missing face".into(),
            ));
        };
        if !equation_faces_valid[face] {
            continue;
        }
        match staged.get(&face).or(published.as_ref()) {
            Some(existing) if existing != &winding => return Ok(Some(face)),
            Some(_) => {}
            None => {
                staged.insert(face, winding);
                queue.push_back(face);
            }
        }
    }
    while let Some(face) = queue.pop_front() {
        let source = staged
            .get(&face)
            .or(face_windings[face].as_ref())
            .expect("queued regularized face has a staged winding vector")
            .clone();
        for edge in &adjacency[face] {
            if edge.face >= face_windings.len() {
                return Err(CurveError::Topology(
                    "regularized arrangement referenced a missing face".into(),
                ));
            }
            if !equation_faces_valid[edge.face] {
                continue;
            }
            let mut target = source.clone();
            let jump = jumps.get(edge.jump).ok_or_else(|| {
                CurveError::Topology("regularized arrangement references a missing jump".into())
            })?;
            let mut apply = |loop_index: usize, delta: i32| {
                target.add(loop_index, delta.saturating_mul(i32::from(edge.direction)))
            };
            match jump {
                RegularizedWindingJump::Zero => {}
                RegularizedWindingJump::Single(loop_index) => apply(*loop_index, 1)?,
                RegularizedWindingJump::Aggregate(components) => {
                    for &(loop_index, delta) in components.as_ref() {
                        apply(loop_index, delta)?;
                    }
                }
            }
            match staged.get(&edge.face).or(face_windings[edge.face].as_ref()) {
                Some(existing) if existing != &target => return Ok(Some(edge.face)),
                Some(_) => {}
                None => {
                    staged.insert(edge.face, target);
                    queue.push_back(edge.face);
                }
            }
        }
    }
    for (face, staged) in staged {
        let published = &mut face_windings[face];
        if published.is_none() {
            *published = Some(staged);
        }
    }
    Ok(None)
}

fn seed_regularized_face_windings(
    face_windings: &mut [Option<LoopWindings>],
    equation_faces_valid: &mut [bool],
    adjacency: &[Vec<RegularizedFaceAdjacency>],
    jumps: &[RegularizedWindingJump],
    seeds: impl IntoIterator<Item = (usize, Vec<i32>)>,
) -> Result<bool, CurveError> {
    let seeds = seeds
        .into_iter()
        .map(|(face, windings)| (face, LoopWindings::from_dense(&windings)))
        .collect::<Vec<_>>();
    let mut disabled_component = false;
    while let Some(conflicting_face) = propagate_regularized_face_windings(
        face_windings,
        equation_faces_valid,
        adjacency,
        jumps,
        seeds.iter().cloned(),
    )? {
        // Quarantine only the contradictory equation components. Independent
        // face components remain useful exact accelerators, and subsequent
        // geometric seeds stay authoritative for every quarantined face.
        disabled_component = true;
        if conflicting_face >= face_windings.len() {
            return Err(CurveError::Topology(
                "regularized arrangement referenced a missing face".into(),
            ));
        }
        let mut queue = std::collections::VecDeque::from([conflicting_face]);
        equation_faces_valid[conflicting_face] = false;
        face_windings[conflicting_face] = None;
        while let Some(face) = queue.pop_front() {
            for edge in &adjacency[face] {
                if edge.face >= face_windings.len() {
                    return Err(CurveError::Topology(
                        "regularized arrangement referenced a missing face".into(),
                    ));
                }
                if equation_faces_valid[edge.face] {
                    equation_faces_valid[edge.face] = false;
                    face_windings[edge.face] = None;
                    queue.push_back(edge.face);
                }
            }
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "regularization-face-winding",
            "contradictory-component-disabled",
        );
    }
    Ok(disabled_component)
}

#[cfg(test)]
mod regularized_face_winding_tests {
    use super::{
        LoopWindings, RegularizedFaceAdjacency, RegularizedWindingJump,
        propagate_regularized_face_windings, seed_regularized_face_windings,
    };

    fn sparse(windings: Vec<Option<Vec<i32>>>) -> Vec<Option<LoopWindings>> {
        windings
            .into_iter()
            .map(|windings| windings.map(|windings| LoopWindings::from_dense(&windings)))
            .collect()
    }

    fn dense(windings: &[Option<LoopWindings>]) -> Vec<Option<Vec<i32>>> {
        windings
            .iter()
            .map(|windings| windings.as_ref().map(LoopWindings::to_dense))
            .collect()
    }

    fn one_boundary_adjacency() -> (
        Vec<Vec<RegularizedFaceAdjacency>>,
        Vec<RegularizedWindingJump>,
    ) {
        (
            vec![
                vec![RegularizedFaceAdjacency {
                    face: 1,
                    jump: 0,
                    direction: -1,
                }],
                vec![RegularizedFaceAdjacency {
                    face: 0,
                    jump: 0,
                    direction: 1,
                }],
            ],
            vec![RegularizedWindingJump::Single(0)],
        )
    }

    #[test]
    fn winding_propagation_publishes_only_a_consistent_component() {
        let (adjacency, jumps) = one_boundary_adjacency();
        let mut windings = sparse(vec![None, None]);
        let valid = vec![true; 2];
        assert_eq!(
            propagate_regularized_face_windings(
                &mut windings,
                &valid,
                &adjacency,
                &jumps,
                [(1, LoopWindings::from_dense(&[0]))],
            )
            .unwrap(),
            None,
        );
        assert_eq!(dense(&windings), vec![Some(vec![1]), Some(vec![0])]);
    }

    #[test]
    fn contradictory_winding_propagation_is_transactional() {
        let (adjacency, jumps) = one_boundary_adjacency();
        let mut windings = sparse(vec![None, None]);
        let valid = vec![true; 2];
        assert_eq!(
            propagate_regularized_face_windings(
                &mut windings,
                &valid,
                &adjacency,
                &jumps,
                [
                    (0, LoopWindings::from_dense(&[0])),
                    (1, LoopWindings::from_dense(&[0])),
                ],
            )
            .unwrap(),
            Some(1),
        );
        assert_eq!(dense(&windings), vec![None, None]);
    }

    #[test]
    fn contradictory_winding_accelerator_is_disabled() {
        let (adjacency, jumps) = one_boundary_adjacency();
        let mut windings = sparse(vec![Some(vec![1]), Some(vec![0])]);
        let mut valid = vec![true; 2];
        assert!(
            seed_regularized_face_windings(
                &mut windings,
                &mut valid,
                &adjacency,
                &jumps,
                [(0, vec![0])],
            )
            .unwrap()
        );
        assert_eq!(valid, vec![false, false]);
        assert_eq!(dense(&windings), vec![None, None]);
    }

    #[test]
    fn contradiction_preserves_independent_winding_components() {
        let (mut adjacency, jumps) = one_boundary_adjacency();
        adjacency.extend([
            vec![RegularizedFaceAdjacency {
                face: 3,
                jump: 0,
                direction: -1,
            }],
            vec![RegularizedFaceAdjacency {
                face: 2,
                jump: 0,
                direction: 1,
            }],
        ]);
        let mut windings = sparse(vec![Some(vec![1]), Some(vec![0]), None, None]);
        let mut valid = vec![true; 4];
        assert!(
            seed_regularized_face_windings(
                &mut windings,
                &mut valid,
                &adjacency,
                &jumps,
                [(0, vec![0]), (3, vec![3])],
            )
            .unwrap()
        );
        assert_eq!(valid, vec![false, false, true, true]);
        assert_eq!(
            dense(&windings),
            vec![None, None, Some(vec![4]), Some(vec![3])]
        );
    }
}

fn retained_probe_outer_bounds(
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let mut last_reason = UncertaintyReason::Unsupported;
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let mut accumulated = None::<Aabb2>;
        let mut complete = true;
        for carrier in carriers {
            let bounds = match carrier.geometry.certified_outer_bounds(
                &carrier.range(),
                refinement_steps,
                policy,
            ) {
                Classification::Decided(bounds) => bounds,
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                    complete = false;
                    break;
                }
            };
            let bounds = bounds
                .certified_rational_outer_envelope(refinement_steps)
                .unwrap_or(bounds);
            accumulated = Some(match accumulated {
                None => bounds,
                Some(ref accumulated) => match accumulated.union(&bounds) {
                    Classification::Decided(bounds) => bounds,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        complete = false;
                        break;
                    }
                },
            });
        }
        if complete {
            return accumulated.map_or(
                Classification::Uncertain(UncertaintyReason::Unsupported),
                Classification::Decided,
            );
        }
    }
    Classification::Uncertain(last_reason)
}

fn retained_probe_exterior_candidate(bounds: &Aabb2, index: usize) -> Option<crate::Point2> {
    let one = Real::one();
    Some(match index {
        0 => crate::Point2::new(bounds.min().x() - &one, bounds.min().y() - &one),
        1 => crate::Point2::new(bounds.max().x() + &one, bounds.min().y() - &one),
        2 => crate::Point2::new(bounds.max().x() + &one, bounds.max().y() + &one),
        3 => crate::Point2::new(bounds.min().x() - &one, bounds.max().y() + &one),
        index => {
            let offset = u64::try_from(index.checked_sub(2)?).ok()?;
            crate::Point2::new(
                bounds.min().x() - &one,
                bounds.min().y() - Real::from(offset),
            )
        }
    })
}

impl CurveRegionCarrier2 {
    /// Returns the exact curve in the chart used by this report's parameters.
    pub fn curve(&self) -> &Curve2 {
        &self.data.curve
    }

    /// Returns the flattened carrier index within this intersection report.
    pub fn carrier_index(&self) -> usize {
        self.data.carrier_index
    }

    /// Returns the region operand that owns this carrier.
    pub fn operand(&self) -> CurveRegionBooleanOperand2 {
        self.data.operand
    }

    /// Returns the boundary-loop index in the operation's prepared operand.
    pub fn loop_index(&self) -> usize {
        self.data.loop_index
    }

    /// Returns the fragment index in the prepared boundary loop. Its source
    /// chart may differ from the retained [`Self::curve`] chart.
    pub fn fragment_index(&self) -> usize {
        self.data.fragment_index
    }
}

impl CurveRegionIntersectionContact2 {
    /// Returns the first-region carrier identity.
    pub const fn first(&self) -> &CurveRegionCarrier2 {
        &self.first
    }

    /// Returns the second-region carrier identity.
    pub const fn second(&self) -> &CurveRegionCarrier2 {
        &self.second
    }

    /// Returns the exact parameter on the first retained carrier.
    pub const fn first_parameter(&self) -> &CurveParameter2 {
        self.evidence.first_parameter()
    }

    /// Returns the exact parameter on the second retained carrier.
    pub const fn second_parameter(&self) -> &CurveParameter2 {
        self.evidence.second_parameter()
    }

    /// Returns retained affine point evidence when the pair kernel constructs it.
    ///
    /// Analytic-parallel pairs deliberately retain the two exact parameters as
    /// the point construction and therefore return `None` without demoting the
    /// contact to rounded coordinates.
    pub const fn point(&self) -> Option<&CurvePoint2> {
        self.evidence.point()
    }

    /// Returns whether exact tangent evidence certifies a transverse crossing.
    pub const fn is_certified_transverse(&self) -> bool {
        self.evidence.is_certified_transverse()
    }
}

impl CurveRegionIntersectionOverlap2 {
    /// Returns the first-region carrier identity.
    pub const fn first(&self) -> &CurveRegionCarrier2 {
        &self.first
    }

    /// Returns the second-region carrier identity.
    pub const fn second(&self) -> &CurveRegionCarrier2 {
        &self.second
    }

    /// Returns the paired ranges, endpoint ownership and reusable correspondence.
    /// The local parameters belong to the two published carrier curves.
    pub const fn overlap(&self) -> &CurveIntersectionOverlap2 {
        &self.overlap
    }
}

impl CurveRegionIntersectionBlocker2 {
    /// Returns the first-region carrier identity.
    pub const fn first(&self) -> &CurveRegionCarrier2 {
        &self.first
    }

    /// Returns the second-region carrier identity.
    pub const fn second(&self) -> &CurveRegionCarrier2 {
        &self.second
    }

    /// Returns common curve-pair blocker evidence, when applicable.
    pub const fn pair_blocker(&self) -> Option<&CurveIntersectionPairBlocker2> {
        match &self.blocker {
            RegionPairBlocker::Common(blocker) => Some(blocker),
            RegionPairBlocker::Uncertain(_)
            | RegionPairBlocker::IncompleteReplay
            | RegionPairBlocker::PointImageParameterComponent => None,
        }
    }

    /// Returns the terminal uncertainty reason when the exact carrier kernel was undecided.
    pub const fn uncertainty_reason(&self) -> Option<UncertaintyReason> {
        match &self.blocker {
            RegionPairBlocker::Uncertain(reason) => Some(*reason),
            RegionPairBlocker::Common(blocker) => match blocker.kind() {
                CurveIntersectionPairBlockerKind2::Uncertain(reason) => Some(*reason),
                _ => None,
            },
            RegionPairBlocker::IncompleteReplay
            | RegionPairBlocker::PointImageParameterComponent => None,
        }
    }

    /// Returns true when exact replay retained candidates it could not complete.
    pub const fn is_incomplete_replay(&self) -> bool {
        match &self.blocker {
            RegionPairBlocker::IncompleteReplay => true,
            RegionPairBlocker::Common(blocker) => matches!(
                blocker.kind(),
                CurveIntersectionPairBlockerKind2::IncompleteReplay
            ),
            _ => false,
        }
    }

    /// Returns true for a positive-dimensional parameter component with point image.
    pub const fn is_point_image_parameter_component(&self) -> bool {
        matches!(
            self.blocker,
            RegionPairBlocker::PointImageParameterComponent
        )
    }
}

impl RegionPairContactEvidence {
    fn from_intersection(contact: &CurveIntersectionContact2) -> Self {
        Self {
            first_parameter: contact.first().local_parameter().clone(),
            second_parameter: contact.second().local_parameter().clone(),
            point: Some(contact.point().clone()),
            certified_transverse: contact.is_certified_transverse(),
            tangent_cross_sign: contact.tangent_cross_sign(),
            tangent_dot_sign: None,
            second_side_of_first: None,
        }
    }

    fn direct_bezier(
        first_parameter: BezierParameter2,
        second_parameter: BezierParameter2,
        point: Option<CurvePoint2>,
        certified_transverse: bool,
        tangent_cross_sign: Option<RealSign>,
    ) -> Self {
        Self::direct(
            CurveParameter2::from(first_parameter),
            CurveParameter2::from(second_parameter),
            point,
            certified_transverse,
            tangent_cross_sign,
        )
    }

    fn direct(
        first_parameter: CurveParameter2,
        second_parameter: CurveParameter2,
        point: Option<CurvePoint2>,
        certified_transverse: bool,
        tangent_cross_sign: Option<RealSign>,
    ) -> Self {
        Self {
            first_parameter,
            second_parameter,
            point,
            certified_transverse,
            tangent_cross_sign,
            tangent_dot_sign: None,
            second_side_of_first: None,
        }
    }

    fn with_tangent_topology(
        mut self,
        tangent_dot_sign: RealSign,
        second_side_of_first: LineSide,
    ) -> Self {
        self.tangent_dot_sign = Some(tangent_dot_sign);
        self.second_side_of_first = Some(second_side_of_first);
        self
    }

    const fn first_parameter(&self) -> &CurveParameter2 {
        &self.first_parameter
    }

    const fn second_parameter(&self) -> &CurveParameter2 {
        &self.second_parameter
    }

    const fn point(&self) -> Option<&CurvePoint2> {
        self.point.as_ref()
    }

    const fn is_certified_transverse(&self) -> bool {
        self.certified_transverse
    }

    const fn tangent_cross_is_positive(&self) -> Option<bool> {
        match self.tangent_cross_sign {
            Some(RealSign::Positive) => Some(true),
            Some(RealSign::Negative) => Some(false),
            Some(RealSign::Zero) | None => None,
        }
    }
}

impl CurveRegionIntersectionResult2 {
    /// Returns the full Cartesian carrier-pair count before broad-phase pruning.
    pub fn authored_carrier_pair_count(&self) -> usize {
        self.data.authored_carrier_pair_count
    }

    /// Returns the carrier-pair count retained after certified broad-phase pruning.
    pub fn candidate_carrier_pair_count(&self) -> usize {
        self.data.candidate_carrier_pair_count
    }

    /// Returns exact contacts clipped to both retained carrier ranges.
    pub fn contacts(&self) -> &[CurveRegionIntersectionContact2] {
        &self.data.contacts
    }

    /// Returns exact positive-length overlaps clipped to both carrier ranges.
    pub fn overlaps(&self) -> &[CurveRegionIntersectionOverlap2] {
        &self.data.overlaps
    }

    /// Returns incomplete carrier pairs with retained exact evidence.
    pub fn blockers(&self) -> &[CurveRegionIntersectionBlocker2] {
        &self.data.blockers
    }

    /// Returns true when every retained carrier pair completed exact replay.
    pub fn is_complete(&self) -> bool {
        self.data.blockers.is_empty()
    }

    /// Returns true when complete replay found no contact or overlap.
    pub fn is_disjoint(&self) -> bool {
        self.is_complete() && self.data.contacts.is_empty() && self.data.overlaps.is_empty()
    }
}

impl CurveRegionBooleanResults2 {
    /// Returns the exact result for one Boolean operation.
    pub fn region(&self, operation: BooleanOp) -> &CurveRegion2 {
        &self.regions[boolean_operation_index(operation)]
    }

    /// Returns the exact union.
    pub const fn union(&self) -> &CurveRegion2 {
        &self.regions[0]
    }

    /// Returns the exact intersection.
    pub const fn intersection(&self) -> &CurveRegion2 {
        &self.regions[1]
    }

    /// Returns the exact first-minus-second difference.
    pub const fn difference(&self) -> &CurveRegion2 {
        &self.regions[2]
    }

    /// Returns the exact symmetric difference.
    pub const fn xor(&self) -> &CurveRegion2 {
        &self.regions[3]
    }

    /// Returns the Cartesian carrier-pair count before certified broad-phase filtering.
    pub const fn authored_carrier_pair_count(&self) -> usize {
        self.authored_carrier_pair_count
    }

    /// Returns the number of general cross-region pairs retained by the
    /// certified broad phase, or zero when native topology completed the batch.
    pub const fn candidate_carrier_pair_count(&self) -> usize {
        self.candidate_carrier_pair_count
    }

    /// Returns the number of split fragments shared by all four operations.
    pub const fn topology_fragment_count(&self) -> usize {
        self.topology_fragment_count
    }

    /// Returns the number of exact representative-point classifications shared
    /// by all four operations.
    pub const fn topology_point_classification_count(&self) -> usize {
        self.topology_point_classification_count
    }
}

impl CurveRegion2 {
    /// Computes one exact regularized Boolean immediately.
    pub fn boolean_region(
        &self,
        other: &Self,
        operation: BooleanOp,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            self.boolean_region_raw(other, operation, attempt)
        })
    }

    pub(crate) fn boolean_region_raw(
        &self,
        other: &Self,
        operation: BooleanOp,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if let Some(region) = boolean_trivial_region(self, other, operation)? {
            return region
                .regularized_region_raw(policy)
                .map_err(|error| error.with_operation(CurveOperation2::Boolean));
        }
        let (first, second) = self.regularized_pair(other, policy)?;
        if let Some(region) = boolean_trivial_region(&first, &second, operation)? {
            return Ok(region);
        }
        CurveRegionBooleanContext::try_new(&first, &second, policy)?
            .build_boolean_region(operation, None)
    }

    /// Computes all four exact regularized Booleans immediately while sharing
    /// intersection and split-topology work within this call.
    pub fn boolean_regions(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveRegionBooleanResults2>> {
        resolve_certified_operation(policy, |attempt| self.boolean_regions_raw(other, attempt))
    }

    pub(crate) fn boolean_regions_raw(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveRegionBooleanResults2> {
        let (first, second) = self.regularized_pair(other, policy)?;
        let operations = [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ];
        // Every nontrivial batch builds one authoritative arrangement. Pair
        // dispatch retains the affine, circular-conic, and general-curve fast
        // paths inside that topology instead of rebuilding a native region
        // Boolean four times. Empty and structurally identical operands need
        // no arrangement at all.
        if first.is_empty() || second.is_empty() || first == second {
            let immediate = [
                boolean_trivial_region(&first, &second, operations[0])?,
                boolean_trivial_region(&first, &second, operations[1])?,
                boolean_trivial_region(&first, &second, operations[2])?,
                boolean_trivial_region(&first, &second, operations[3])?,
            ];
            return Ok(CurveRegionBooleanResults2 {
                regions: Box::new(
                    immediate
                        .map(|region| region.expect("all immediate Boolean results were checked")),
                ),
                authored_carrier_pair_count: region_carrier_count(self)
                    .saturating_mul(region_carrier_count(other)),
                candidate_carrier_pair_count: 0,
                topology_fragment_count: 0,
                topology_point_classification_count: 0,
            });
        }
        CurveRegionBooleanContext::try_new(&first, &second, policy)?.build_boolean_regions()
    }

    /// Establishes filled-side ownership once per operand. Authored loops
    /// may intersect each other or have no represented signed area; their
    /// unary arrangement must precede a cross-operand-only arrangement.
    /// Completed boundaries and identical operands share their certificates.
    fn regularized_pair(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<(Self, Self)> {
        let first = self
            .regularized_region_raw(policy)
            .map_err(|error| error.with_operation(CurveOperation2::Boolean))?;
        let second = if self == other {
            first.clone()
        } else {
            other
                .regularized_region_raw(policy)
                .map_err(|error| error.with_operation(CurveOperation2::Boolean))?
        };
        Ok((first, second))
    }

    /// Regularizes this region's authored loops through the authoritative exact arrangement.
    ///
    /// Every retained carrier pair is intersected and split before the filled
    /// state on both local sides of each open fragment is classified. Fragments
    /// separating equal filled states are discarded; the remaining boundary is
    /// oriented with material on its left and traversed into closed loops.
    pub fn regularized_region(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| self.regularized_region_raw(attempt))
    }

    pub(crate) fn regularized_region_raw(&self, policy: &CurveContext) -> ExactCurveResult<Self> {
        if self.is_empty() {
            return Ok(self.clone());
        }
        // An authoritative filled-left face walk or an independent exact
        // convex-boundary certificate is already a canonical regularization
        // proof. Rebuilding its arrangement wastes work and, for compact
        // correlated chord cuts, would throw away the topology evidence that
        // deliberately replaces coordinate materialization.
        if self.has_regularized_filled_left_topology(policy) {
            return Ok(self.clone());
        }
        self.resolve_regularization(policy, || {
            let context = CurveRegionBooleanContext::try_new_unary(self, policy)?;
            context
                .build_regularized_region()
                .map_err(|error| error.with_operation(CurveOperation2::Arrangement))
        })
    }

    /// Applies an authored compound fill before publishing a normalized set.
    /// No raw-region regularization cache can be reused under another fill rule.
    pub(crate) fn regularize_boundary_paths_raw(
        paths: &[crate::CurvePath2],
        fill_rule: FillRule,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let raw = Self::try_from_boundary_paths_raw(paths, policy)?;
        let mut context = CurveRegionBooleanContext::try_new_unary(&raw, policy)?;
        context.data.regularization_fill_rule = Some(fill_rule);
        context.build_regularized_region()
    }

    /// Collects exact contacts and overlaps between regularized region boundaries.
    /// Authored winding and canceled seams are resolved before intersection.
    pub fn intersect_region(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveRegionIntersectionResult2>> {
        resolve_certified_operation(policy, |attempt| self.intersect_region_raw(other, attempt))
    }

    pub(crate) fn intersect_region_raw(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveRegionIntersectionResult2> {
        let (first, second) = self.regularized_pair(other, policy)?;
        CurveRegionBooleanContext::try_new(&first, &second, policy)?.build_intersection_evidence()
    }
}

/// Classifies retained multi-field point evidence against one certified simple
/// loop without materializing its coordinates.
///
/// The Boolean carrier-pair authority intersects an exterior probe with only
/// the selected loop. Crossing parity is independent of authored orientation
/// and therefore supplies the geometric nesting predicate needed to assign
/// material, hole, and nested-island roles.
pub(crate) fn classify_retained_point_evidence_against_loop_by_probe(
    region: &CurveRegion2,
    loop_index: usize,
    point: CurvePoint2,
    policy: &CurveContext,
) -> CurveResult<Classification<ContourPointLocation>> {
    let context = match CurveRegionBooleanContext::try_new_retained_loop(region, loop_index, policy)
    {
        Ok(context) => context,
        Err(ExactCurveError::Blocked(blocker)) => {
            return Ok(Classification::Uncertain(blocker.reason()));
        }
        Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
    };
    let classification = match context.classify_retained_point_off_boundary_by_probe(
        0,
        point,
        CurveRegionBooleanOperand2::First,
        RetainedPointProbeClassification::LoopParity,
    ) {
        Ok(classification) => classification,
        Err(ExactCurveError::Blocked(blocker)) => {
            return Ok(Classification::Uncertain(blocker.reason()));
        }
        Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
    };
    Ok(classification.map(|location| match location {
        RegionPointLocation::Outside => ContourPointLocation::Outside,
        RegionPointLocation::Boundary => ContourPointLocation::Boundary,
        RegionPointLocation::Inside => ContourPointLocation::Inside,
    }))
}

/// Classifies an exact point against the published filled boundary. A batch
/// retains successful boundary preparation within its current policy attempt.
pub(crate) fn classify_retained_point_evidence_against_region_by_probe<'a>(
    region: &'a CurveRegion2,
    point: CurvePoint2,
    policy: &CurveContext,
    prepared: &mut Option<CurveRegionBooleanContext<'a>>,
) -> CurveResult<Classification<RegionPointLocation>> {
    if prepared.is_none() {
        let context = match CurveRegionBooleanContext::try_new_curve_boundary(&[], region, policy) {
            Ok(context) => context,
            Err(ExactCurveError::Blocked(blocker)) => {
                return Ok(Classification::Uncertain(blocker.reason()));
            }
            Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
        };
        *prepared = Some(context);
    }
    let context = prepared
        .as_ref()
        .expect("successful point query preparation");
    match context.classify_retained_point_off_boundary_by_probe(
        0,
        point,
        CurveRegionBooleanOperand2::Second,
        RetainedPointProbeClassification::FilledRegion,
    ) {
        Ok(classification) => Ok(classification),
        Err(ExactCurveError::Blocked(blocker)) => Ok(Classification::Uncertain(blocker.reason())),
        Err(ExactCurveError::Invalid { cause, .. }) => Err(cause),
    }
}

mod fragment_location;
mod pair_kernels;
mod regularization;

impl<'a> CurveRegionBooleanContext<'a> {
    fn try_new_retained_loop(
        region: &'a CurveRegion2,
        loop_index: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let Some(boundary_loop) = region.boundary_loops().get(loop_index) else {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Classification,
                CurveFamily2::Line,
                CurveError::Topology("retained loop classification index is out of bounds".into()),
            ));
        };
        if boundary_loop.is_empty() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Classification,
                CurveFamily2::Line,
                CurveError::Topology("retained loop classification requires a boundary".into()),
            ));
        }
        let mut carriers = Vec::with_capacity(boundary_loop.len());
        for (fragment_index, fragment) in boundary_loop.fragments().iter().enumerate() {
            carriers.push(build_region_carrier(
                fragment,
                CurveRegionBooleanOperand2::First,
                loop_index,
                fragment_index,
                false,
                policy,
            )?);
        }
        let carrier_count = carriers.len();
        Ok(Self {
            data: CurveRegionBooleanContextData {
                first: region,
                second: region,
                policy: *policy,
                carriers,
                first_carrier_count: carrier_count,
                authored_carrier_pair_count: 0,
                pairs: Vec::new(),
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        })
    }

    fn try_new(
        first: &'a CurveRegion2,
        second: &'a CurveRegion2,
        policy: &'a CurveContext,
    ) -> ExactCurveResult<Self> {
        let mut rational_quadratic_area_cache = RationalQuadraticAreaIntegralCache::default();
        let first_carriers = build_region_carriers(
            first,
            CurveRegionBooleanOperand2::First,
            policy,
            &mut rational_quadratic_area_cache,
            true,
        )?;
        let first_carrier_count = first_carriers.len();
        let mut carriers = first_carriers;
        carriers.extend(build_region_carriers(
            second,
            CurveRegionBooleanOperand2::Second,
            policy,
            &mut rational_quadratic_area_cache,
            true,
        )?);

        let authored_carrier_pair_count =
            first_carrier_count.saturating_mul(carriers.len() - first_carrier_count);
        let pairs = build_cross_operand_carrier_pairs(&carriers, first_carrier_count, policy)?;

        Ok(Self {
            data: CurveRegionBooleanContextData {
                first,
                second,
                policy: *policy,
                carriers,
                first_carrier_count,
                authored_carrier_pair_count,
                pairs,
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        })
    }

    pub(crate) fn try_new_curve_boundary(
        source_spans: &[(usize, &crate::curve::CurveSourceSpan2)],
        region: &'a CurveRegion2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let mut carriers = Vec::with_capacity(
            source_spans
                .len()
                .saturating_add(region_carrier_count(region)),
        );
        for &(fragment_index, span) in source_spans {
            carriers.push(build_parameterized_carrier(
                &span.fragment,
                CurveRegionBooleanOperand2::First,
                0,
                fragment_index,
                false,
            ));
        }
        let first_carrier_count = carriers.len();
        let mut rational_quadratic_area_cache = RationalQuadraticAreaIntegralCache::default();
        carriers.extend(build_region_carriers(
            region,
            CurveRegionBooleanOperand2::Second,
            policy,
            &mut rational_quadratic_area_cache,
            false,
        )?);

        let authored_carrier_pair_count =
            first_carrier_count.saturating_mul(carriers.len() - first_carrier_count);
        let pairs = build_cross_operand_carrier_pairs(&carriers, first_carrier_count, policy)?;

        Ok(Self {
            data: CurveRegionBooleanContextData {
                // Cross-operand curve/boundary pairs never consult authored
                // adjacency, so the retained region safely supplies both
                // topology metadata slots without fabricating a source loop.
                first: region,
                second: region,
                policy: *policy,
                carriers,
                first_carrier_count,
                authored_carrier_pair_count,
                pairs,
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        })
    }

    fn try_new_unary(region: &'a CurveRegion2, policy: &'a CurveContext) -> ExactCurveResult<Self> {
        let mut rational_quadratic_area_cache = RationalQuadraticAreaIntegralCache::default();
        let mut carriers = build_region_carriers(
            region,
            CurveRegionBooleanOperand2::First,
            policy,
            &mut rational_quadratic_area_cache,
            false,
        )?;
        // A finite traversal equal to its own reverse has zero winding
        // contribution. Remove it before asking a self-intersection solver
        // to enumerate its positive-dimensional retracing relation.
        carriers.retain(|carrier| !carrier_is_symmetric_zero_chain(carrier, policy));
        let carrier_count = carriers.len();
        let authored_carrier_pair_count =
            carrier_count.saturating_mul(carrier_count.saturating_add(1)) / 2;
        let pairs = build_unary_carrier_pairs(&carriers, policy)?;
        Ok(Self {
            data: CurveRegionBooleanContextData {
                first: region,
                second: region,
                policy: *policy,
                carriers,
                first_carrier_count: carrier_count,
                authored_carrier_pair_count,
                pairs,
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        })
    }

    fn intersect_algebraic_probe_boundary(
        &self,
        probe: crate::BezierAlgebraicChord2,
        endpoint_incidence: Option<(usize, &CurveParameter2)>,
    ) -> ExactCurveResult<CurveRegionIntersectionResult2> {
        self.intersect_algebraic_probe_carriers(
            probe,
            self.data.first,
            &self.data.carriers,
            endpoint_incidence,
        )
    }

    fn intersect_algebraic_probe_carriers(
        &self,
        probe: crate::BezierAlgebraicChord2,
        boundary_region: &CurveRegion2,
        boundary_carriers: &[RegionCarrier],
        endpoint_incidence: Option<(usize, &CurveParameter2)>,
    ) -> ExactCurveResult<CurveRegionIntersectionResult2> {
        let start = probe.start_parameter();
        let end = probe.end_parameter();
        // The caller evaluated this source chart to construct the probe end.
        // Retain that incidence; a strict transverse derivative makes it a
        // simple factor, so replay can solve the residual contacts separately.
        let mut endpoint_incidence_contact = endpoint_incidence.and_then(|(index, parameter)| {
            let carrier = boundary_carriers.get(index)?;
            let CurveSupport2::Bezier(_) = &carrier.geometry else {
                return None;
            };
            let scalar = parameter.as_bezier_parameter()?.scalar()?;
            let line = probe.exact_line()?;
            let Classification::Decided(derivative) = carrier
                .geometry
                .derivative_at(scalar, &CurveContext::STRICT)
                .ok()?
            else {
                return None;
            };
            let (dx, dy) = line.delta();
            let cross = dx * derivative.dy() - dy * derivative.dx();
            let sign = real_sign(&cross, &CurveContext::STRICT)?;
            if sign == RealSign::Zero {
                return None;
            }
            Some((
                index + 1,
                Box::new(RegionPairContactEvidence::direct(
                    CurveParameter2::from_algebraic_chord(end.clone()),
                    parameter.clone(),
                    Some(probe.end().clone()),
                    true,
                    Some(sign),
                )),
            ))
        });
        let mut carriers = Vec::with_capacity(boundary_carriers.len().saturating_add(1));
        carriers.push(RegionCarrier {
            operand: CurveRegionBooleanOperand2::First,
            loop_index: 0,
            fragment_index: 0,
            family: CurveFamily2::Line,
            geometry: CurveSupport2::Line(probe),
            start: CurveParameter2::from_algebraic_chord(start),
            end: CurveParameter2::from_algebraic_chord(end),
            reversed: false,
            filled_side_is_left: false,
            selected_fiber_endpoint_points: None,
            image_is_injective: OnceLock::new(),
            bounds: OnceLock::new(),
            refined_bounds: Default::default(),
        });
        carriers.extend(boundary_carriers.iter().cloned().map(|mut carrier| {
            carrier.operand = CurveRegionBooleanOperand2::Second;
            carrier
        }));

        // Every pair contains the chord probe, so no Bezier/Bezier context
        // consumes a prepared curve. Keep the positional table empty.
        let curves = vec![None; carriers.len()];
        let mut pairs = Vec::with_capacity(boundary_carriers.len());
        let mut intersection_cache = CurveIntersectionBatchCache::default();
        for second_carrier_index in 1..carriers.len() {
            if let Some(mut pair) = build_candidate_carrier_pair(
                &carriers,
                &curves,
                0,
                second_carrier_index,
                &self.data.policy,
                &mut intersection_cache,
            )? {
                if let Some((index, _)) = &endpoint_incidence_contact
                    && *index == second_carrier_index
                    && let RegionCarrierPairContext::AlgebraicChordPair { endpoint_contact } =
                        &mut pair.context
                {
                    *endpoint_contact = Some(
                        endpoint_incidence_contact
                            .take()
                            .expect("the matching probe incidence is present")
                            .1,
                    );
                }
                pairs.push(pair);
            }
        }
        CurveRegionBooleanContext {
            data: CurveRegionBooleanContextData {
                first: boundary_region,
                second: boundary_region,
                policy: self.data.policy,
                carriers,
                first_carrier_count: 1,
                authored_carrier_pair_count: boundary_carriers.len(),
                pairs,
                regularization_fill_rule: None,
                strict_line_image_only: OnceLock::new(),
                operand_bounds: std::array::from_fn(|_| OnceLock::new()),
            },
        }
        .build_intersection_evidence()
    }

    pub(crate) fn build_intersection_evidence(
        &self,
    ) -> ExactCurveResult<CurveRegionIntersectionResult2> {
        let mut contacts = Vec::new();
        let mut overlaps = Vec::new();
        let mut blockers = Vec::new();
        let mut carriers: Vec<Option<CurveRegionCarrier2>> = vec![None; self.data.carriers.len()];
        let mut publish = |index: usize| -> ExactCurveResult<CurveRegionCarrier2> {
            if let Some(carrier) = &carriers[index] {
                return Ok(carrier.clone());
            }
            let carrier = &self.data.carriers[index];
            let fragment = carrier
                .geometry
                .restrict_certified(
                    carrier.range(),
                    carrier.selected_fiber_endpoint_points.as_deref().cloned(),
                    carrier.reversed,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(index, cause))?;
            let published = CurveRegionCarrier2 {
                data: Arc::new(CurveRegionCarrierData2 {
                    curve: Curve2::from_retained_fragment(fragment),
                    carrier_index: index,
                    operand: carrier.operand,
                    loop_index: carrier.loop_index,
                    fragment_index: carrier.fragment_index,
                }),
            };
            carriers[index] = Some(published.clone());
            Ok(published)
        };
        for pair in &self.data.pairs {
            let result = self.pair_result(pair)?;
            for blocker in result.blockers {
                blockers.push(CurveRegionIntersectionBlocker2 {
                    first: publish(pair.first_carrier_index)?,
                    second: publish(pair.second_carrier_index)?,
                    blocker,
                });
            }
            for contact in result.contacts {
                if parameter_in_carrier(
                    contact.first_parameter(),
                    &self.data.carriers[pair.first_carrier_index],
                    &self.data.policy,
                )? && parameter_in_carrier(
                    contact.second_parameter(),
                    &self.data.carriers[pair.second_carrier_index],
                    &self.data.policy,
                )? {
                    contacts.push(CurveRegionIntersectionContact2 {
                        first: publish(pair.first_carrier_index)?,
                        second: publish(pair.second_carrier_index)?,
                        evidence: contact,
                    });
                }
            }
            for overlap in result.overlaps {
                let Some(ranges) = self.clipped_overlap_ranges(pair, &overlap)? else {
                    continue;
                };
                let (first_range, second_range) =
                    self.paired_overlap_ranges(pair, overlap.orientation, ranges)?;
                overlaps.push(CurveRegionIntersectionOverlap2 {
                    first: publish(pair.first_carrier_index)?,
                    second: publish(pair.second_carrier_index)?,
                    overlap: match overlap
                        .with_paired_ranges(first_range, second_range, &self.data.policy)
                        .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                    {
                        Classification::Decided(overlap) => overlap,
                        Classification::Uncertain(reason) => {
                            return Err(self.blocked(pair.first_carrier_index, reason));
                        }
                    },
                });
            }
        }
        Ok(CurveRegionIntersectionResult2 {
            data: Arc::new(CurveRegionIntersectionResultData {
                authored_carrier_pair_count: self.data.authored_carrier_pair_count,
                candidate_carrier_pair_count: self.data.pairs.len(),
                contacts: contacts.into(),
                overlaps: overlaps.into(),
                blockers: blockers.into(),
            }),
        })
    }

    fn build_split_topology(&self) -> ExactCurveResult<CurveRegionSplitTopology> {
        let mut events = vec![Vec::new(); self.data.carriers.len()];
        let mut contact_points = Vec::<ContactVertex>::new();
        let mut contact_lookup = ContactPointIndex::default();
        let mut deferred_contact_matches = Vec::<(usize, usize, UncertaintyReason)>::new();
        let mut merge_vertices = Vec::new();
        let mut uncertain_contact_matches = Vec::new();
        let mut deferred_event_ordering = false;
        let mut next_topology_vertex = 0_usize;
        let mut contact_vertex_counts = Vec::<usize>::new();
        let mut transition_candidates = Vec::<Option<TransitionContactCandidate>>::new();
        let mut reclassification_vertices = Vec::<bool>::new();
        seed_loop_topology_vertices(&self.data.carriers, &mut events, &mut next_topology_vertex);
        for (carrier_index, carrier) in self.data.carriers.iter().enumerate() {
            let CurveSupport2::Parallel(parallel) = &carrier.geometry else {
                continue;
            };
            if real_sign(parallel.distance(), &self.data.policy) == Some(RealSign::Zero) {
                continue;
            }
            let analysis = match parallel
                .singularity_analysis(&carrier.range(), &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(analysis) => analysis,
                Classification::Uncertain(reason) => {
                    return Err(self.blocked(carrier_index, reason));
                }
            };
            for cusp in analysis.parallel_cusps() {
                let parameter = CurveParameter2::from(cusp.clone());
                if parameter_location_in_carrier(&parameter, carrier, &self.data.policy)?
                    != CarrierParameterLocation::Interior
                {
                    continue;
                }
                // A cusp is a branch boundary even when no other carrier
                // meets it. Later normal offsets and fillet center loci need
                // one derivative orientation on each open arrangement edge.
                events[carrier_index].push(CarrierEvent {
                    parameter,
                    topology_vertex: Some(next_topology_vertex),
                });
                next_topology_vertex += 1;
            }
        }
        contact_vertex_counts.resize(next_topology_vertex, 0);
        transition_candidates.resize(next_topology_vertex, None);
        reclassification_vertices.resize(next_topology_vertex, false);
        let mut overlaps = Vec::<CarrierOverlap>::new();
        for pair in &self.data.pairs {
            let result = self.pair_result(pair)?;
            if let Some(blocker) = result.blockers.first() {
                #[cfg(test)]
                if std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some() {
                    eprintln!(
                        "regularization pair blocker carriers=({}, {}) loops=({}, {}) fragments=({}, {}) context={:?} blocker={:?} adjacent={} selects-approximate={} permits-approximate={}",
                        pair.first_carrier_index,
                        pair.second_carrier_index,
                        self.data.carriers[pair.first_carrier_index].loop_index,
                        self.data.carriers[pair.second_carrier_index].loop_index,
                        self.data.carriers[pair.first_carrier_index].fragment_index,
                        self.data.carriers[pair.second_carrier_index].fragment_index,
                        pair.context,
                        blocker,
                        self.authored_carriers_are_adjacent(pair),
                        self.data.policy.selects_approximate_512(),
                        self.data.policy.permits_approximate_512(),
                    );
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-regularization-pair-blocker",
                    match &pair.context {
                        RegionCarrierPairContext::Common(_) => "common-pair",
                        RegionCarrierPairContext::ParallelRational {
                            parallel_is_first: true,
                        } => "parallel-rational",
                        RegionCarrierPairContext::ParallelRational {
                            parallel_is_first: false,
                        } => "rational-parallel",
                        RegionCarrierPairContext::ParallelPair => "parallel-pair",
                        RegionCarrierPairContext::ParallelSameImage => "parallel-same-image",
                        RegionCarrierPairContext::AlgebraicChordPair { .. } => {
                            "algebraic-chord-pair"
                        }
                        RegionCarrierPairContext::CuspChord {
                            cusp_is_first: true,
                        } => "cusp-chord",
                        RegionCarrierPairContext::CuspChord {
                            cusp_is_first: false,
                        } => "chord-cusp",
                        RegionCarrierPairContext::CuspRational {
                            cusp_is_first: true,
                        } => "cusp-rational",
                        RegionCarrierPairContext::CuspRational {
                            cusp_is_first: false,
                        } => "rational-cusp",
                        RegionCarrierPairContext::CuspParallel {
                            cusp_is_first: true,
                        } => "cusp-parallel",
                        RegionCarrierPairContext::CuspParallel {
                            cusp_is_first: false,
                        } => "parallel-cusp",
                        RegionCarrierPairContext::CuspPair => "cusp-pair",
                    },
                );
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-regularization-pair-blocker-adjacency",
                    if self.authored_carriers_are_adjacent(pair) {
                        "adjacent"
                    } else {
                        "nonadjacent"
                    },
                );
                let reason = match blocker {
                    RegionPairBlocker::Common(blocker) => match blocker.kind() {
                        crate::CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
                        crate::CurveIntersectionPairBlockerKind2::IncompleteReplay => {
                            UncertaintyReason::Predicate
                        }
                        crate::CurveIntersectionPairBlockerKind2::SharedComponent => {
                            UncertaintyReason::Boundary
                        }
                    },
                    RegionPairBlocker::Uncertain(reason) => *reason,
                    RegionPairBlocker::IncompleteReplay => UncertaintyReason::Predicate,
                    RegionPairBlocker::PointImageParameterComponent => UncertaintyReason::Boundary,
                };
                return Err(self.blocked(pair.first_carrier_index, reason));
            }

            for contact in &result.contacts {
                let first_parameter = contact.first_parameter();
                let second_parameter = contact.second_parameter();
                let first_location = parameter_location_in_carrier(
                    first_parameter,
                    &self.data.carriers[pair.first_carrier_index],
                    &self.data.policy,
                )?;
                let second_location = parameter_location_in_carrier(
                    second_parameter,
                    &self.data.carriers[pair.second_carrier_index],
                    &self.data.policy,
                )?;
                if first_location == CarrierParameterLocation::Outside
                    || second_location == CarrierParameterLocation::Outside
                {
                    continue;
                }
                // Range classification already owns endpoint equality. Reuse
                // that parameter and its seeded topology vertex instead of
                // asking an optional bounded lookup to prove the same fact.
                let canonical_parameter = |parameter, location, carrier_index: usize| {
                    let carrier = &self.data.carriers[carrier_index];
                    match location {
                        CarrierParameterLocation::Endpoint(BezierEndpoint::Start) => &carrier.start,
                        CarrierParameterLocation::Endpoint(BezierEndpoint::End) => &carrier.end,
                        CarrierParameterLocation::Interior | CarrierParameterLocation::Outside => {
                            parameter
                        }
                    }
                };
                let first_parameter =
                    canonical_parameter(first_parameter, first_location, pair.first_carrier_index);
                let second_parameter = canonical_parameter(
                    second_parameter,
                    second_location,
                    pair.second_carrier_index,
                );
                let first_existing = existing_contact_event_vertex_if_decided(
                    &events[pair.first_carrier_index],
                    first_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
                let second_existing = existing_contact_event_vertex_if_decided(
                    &events[pair.second_carrier_index],
                    second_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(pair.second_carrier_index, cause))?;
                let mut topology_vertex = first_existing.or(second_existing);
                merge_vertices.clear();
                if let (Some(first_vertex), Some(second_vertex)) = (first_existing, second_existing)
                    && first_vertex != second_vertex
                {
                    merge_vertices.push(second_vertex);
                }
                uncertain_contact_matches.clear();
                let mut matching_contact_index = None;
                // Distinctness from the current endpoint vertex depends only on
                // the existing contact's vertex. Group incidences by vertex once
                // and decide each vertex pair once, instead of rescanning every
                // contact for every contact.
                let mut distinct_vertices: Vec<Option<bool>> = Vec::new();
                let mut distinct_current_vertex = None;
                let candidates = contact_lookup.candidates(contact.point());
                let candidates: Box<dyn Iterator<Item = usize>> = match &candidates {
                    Some(candidates) => Box::new(candidates.iter().copied()),
                    None => Box::new(0..contact_points.len()),
                };
                for existing_index in candidates {
                    let existing = &contact_points[existing_index];
                    if topology_vertex == Some(existing.topology_vertex) {
                        matching_contact_index.get_or_insert(existing_index);
                        continue;
                    }
                    if let Some(current_vertex) = topology_vertex
                        && {
                            if distinct_current_vertex != Some(current_vertex) {
                                distinct_current_vertex = Some(current_vertex);
                                distinct_vertices.clear();
                                distinct_vertices.resize(next_topology_vertex, None);
                            }
                            let vertex = existing.topology_vertex;
                            *distinct_vertices[vertex].get_or_insert_with(|| {
                                contact_lookup.incidences(vertex).iter().any(|&index| {
                                    contact_decided_distinct_from_carrier_endpoint_vertex(
                                        &contact_points[index],
                                        current_vertex,
                                        &events,
                                        &self.data.carriers,
                                        &self.data.policy,
                                    )
                                })
                            })
                        }
                    {
                        continue;
                    }
                    if contacts_decided_same_from_shared_parallel(
                        existing,
                        [pair.first_carrier_index, pair.second_carrier_index],
                        [first_parameter, second_parameter],
                        &self.data.carriers,
                        &self.data.policy,
                    )? {
                        if let Some(vertex) = topology_vertex {
                            if vertex != existing.topology_vertex
                                && !merge_vertices.contains(&existing.topology_vertex)
                            {
                                merge_vertices.push(existing.topology_vertex);
                            }
                        } else {
                            topology_vertex = Some(existing.topology_vertex);
                        }
                        matching_contact_index.get_or_insert(existing_index);
                        continue;
                    }
                    let distinct = contacts_decided_distinct_from_carriers(
                        existing,
                        [pair.first_carrier_index, pair.second_carrier_index],
                        [first_parameter, second_parameter],
                        &self.data.carriers,
                        &self.data.policy,
                    )?;
                    if distinct {
                        continue;
                    }
                    match contacts_decided_same_from_circular_carriers(
                        existing,
                        [pair.first_carrier_index, pair.second_carrier_index],
                        [first_parameter, second_parameter],
                        &self.data.carriers,
                        &self.data.policy,
                    ) {
                        Classification::Decided(true) => {
                            if let Some(vertex) = topology_vertex {
                                if vertex != existing.topology_vertex
                                    && !merge_vertices.contains(&existing.topology_vertex)
                                {
                                    merge_vertices.push(existing.topology_vertex);
                                }
                            } else {
                                topology_vertex = Some(existing.topology_vertex);
                            }
                            matching_contact_index.get_or_insert(existing_index);
                            continue;
                        }
                        Classification::Decided(false) => continue,
                        Classification::Uncertain(_) => {}
                    }
                    if let (Some(existing_point), Some(point)) =
                        (existing.point.as_ref(), contact.point())
                    {
                        let exact_against_existing = match (existing_point, point) {
                            (
                                CurvePoint2(CurvePointData2::Algebraic(_)),
                                CurvePoint2(CurvePointData2::Exact(exact)),
                            ) => Some(exact),
                            _ => None,
                        };
                        if let Some(exact) = exact_against_existing
                            && existing
                                .carrier_indices
                                .iter()
                                .copied()
                                .any(|carrier_index| {
                                    exact_point_decided_outside_carrier(
                                        exact,
                                        &self.data.carriers[carrier_index],
                                        &self.data.policy,
                                    )
                                })
                        {
                            continue;
                        }
                        let same = if let Some(exact) = exact_against_existing {
                            match exact_point_matches_existing_contact_parameter(
                                exact,
                                existing,
                                &self.data.carriers,
                                &self.data.policy,
                            ) {
                                Classification::Decided(equal) => Classification::Decided(equal),
                                Classification::Uncertain(_) => {
                                    existing_point.same_point(point, &self.data.policy)
                                }
                            }
                        } else {
                            existing_point.same_point(point, &self.data.policy)
                        };
                        match same {
                            Classification::Decided(true) => {
                                if let Some(vertex) = topology_vertex {
                                    if vertex != existing.topology_vertex
                                        && !merge_vertices.contains(&existing.topology_vertex)
                                    {
                                        merge_vertices.push(existing.topology_vertex);
                                    }
                                } else {
                                    topology_vertex = Some(existing.topology_vertex);
                                }
                                matching_contact_index.get_or_insert(existing_index);
                            }
                            Classification::Decided(false) => {}
                            Classification::Uncertain(reason) => {
                                uncertain_contact_matches.push((existing_index, reason));
                            }
                        }
                    }
                }
                let topology_vertex = topology_vertex.unwrap_or_else(|| {
                    let vertex = next_topology_vertex;
                    next_topology_vertex += 1;
                    vertex
                });
                for previous_vertex in merge_vertices
                    .iter()
                    .copied()
                    .filter(|previous| *previous != topology_vertex)
                {
                    replace_topology_vertex(
                        &mut events,
                        &mut contact_points,
                        previous_vertex,
                        topology_vertex,
                    );
                    contact_lookup.merge_vertex(previous_vertex, topology_vertex);
                    for overlap in &mut overlaps {
                        overlap.replace_topology_vertex(previous_vertex, topology_vertex);
                    }
                    contact_vertex_counts[topology_vertex] +=
                        contact_vertex_counts[previous_vertex];
                    contact_vertex_counts[previous_vertex] = 0;
                    reclassification_vertices[topology_vertex] |=
                        reclassification_vertices[previous_vertex];
                    reclassification_vertices[previous_vertex] = false;
                    transition_candidates[topology_vertex] = None;
                    transition_candidates[previous_vertex] = None;
                }
                let contact_index = contact_points.len();
                let point = if let Some(existing_index) = matching_contact_index {
                    if contact_points[existing_index].point.is_none()
                        || matches!(
                            contact_points[existing_index].point,
                            Some(CurvePoint2(CurvePointData2::Algebraic(_)))
                        ) && matches!(
                            contact.point(),
                            Some(CurvePoint2(CurvePointData2::Exact(_)))
                        )
                    {
                        contact_points[existing_index].point = contact.point().cloned();
                    }
                    // The point representative above owns geometric evidence;
                    // this record only needs the additional carrier incidence.
                    None
                } else {
                    contact.point().cloned()
                };
                let mut retained_parameters = [first_parameter.clone(), second_parameter.clone()];
                if let Some(existing_index) = matching_contact_index
                    && let Some(representative) = contact_points[existing_index].point.clone()
                {
                    for (slot, (carrier_index, location)) in [
                        (pair.first_carrier_index, first_location),
                        (pair.second_carrier_index, second_location),
                    ]
                    .into_iter()
                    .enumerate()
                    {
                        let CurveSupport2::Line(chord) =
                            &self.data.carriers[carrier_index].geometry
                        else {
                            continue;
                        };
                        if location == CarrierParameterLocation::Interior
                            && retained_parameters[slot].as_algebraic_chord().is_some_and(
                                |parameter| parameter.is_certified_strict_interior_of(chord),
                            )
                        {
                            retained_parameters[slot] = CurveParameter2::from_algebraic_chord(
                                chord.parameter_at_certified_interior_point(representative.clone()),
                            );
                        }
                    }
                }
                contact_points.push(ContactVertex {
                    point,
                    topology_vertex,
                    carrier_indices: [pair.first_carrier_index, pair.second_carrier_index],
                    parameters: retained_parameters.clone(),
                });
                // A matched record stores no point, but the contact's own
                // point is the same certified point and supplies its box.
                contact_lookup.push(contact_index, contact.point(), topology_vertex);
                for &(existing_index, reason) in &uncertain_contact_matches {
                    deferred_contact_matches.push((existing_index, contact_index, reason));
                }
                if contact_vertex_counts.len() <= topology_vertex {
                    contact_vertex_counts.resize(topology_vertex + 1, 0);
                    transition_candidates.resize(topology_vertex + 1, None);
                    reclassification_vertices.resize(topology_vertex + 1, false);
                }
                contact_vertex_counts[topology_vertex] += 1;
                reclassification_vertices[topology_vertex] = true;
                transition_candidates[topology_vertex] = if contact_vertex_counts[topology_vertex]
                    == 1
                {
                    Some(TransitionContactCandidate {
                        first_carrier: pair.first_carrier_index,
                        second_carrier: pair.second_carrier_index,
                        interior_on_both_carriers: first_location
                            == CarrierParameterLocation::Interior
                            && second_location == CarrierParameterLocation::Interior,
                        certified_transverse: contact.is_certified_transverse(),
                        cross_is_positive: contact.tangent_cross_is_positive(),
                        tangent_dot_is_positive: match contact.tangent_dot_sign {
                            Some(RealSign::Positive) => Some(true),
                            Some(RealSign::Negative) => Some(false),
                            Some(RealSign::Zero) | None => None,
                        },
                        second_side_of_first: contact.second_side_of_first,
                        self_parameters: (pair.first_carrier_index == pair.second_carrier_index)
                            .then(|| retained_parameters.clone()),
                    })
                } else {
                    None
                };
                deferred_event_ordering |= push_contact_carrier_event(
                    &mut events[pair.first_carrier_index],
                    retained_parameters[0].clone(),
                    Some(topology_vertex),
                    &self.data.carriers[pair.first_carrier_index],
                    &self.data.policy,
                )?;
                deferred_event_ordering |= push_contact_carrier_event(
                    &mut events[pair.second_carrier_index],
                    retained_parameters[1].clone(),
                    Some(topology_vertex),
                    &self.data.carriers[pair.second_carrier_index],
                    &self.data.policy,
                )?;
            }

            for overlap in &result.overlaps {
                let Some(ranges) = self.clipped_overlap_ranges(pair, overlap)? else {
                    continue;
                };
                let (mut first_range, mut second_range) =
                    self.paired_overlap_ranges(pair, overlap.orientation, ranges)?;
                // A certified overlap is also exact endpoint-incidence
                // evidence.  Give each corresponding endpoint pair one
                // topology vertex even when neither carrier can materialize
                // the shared Cartesian point.  If regularization later
                // cancels both coincident spans, their neighboring unique
                // fragments still reconnect through these vertices.
                let mut first_parameters = [first_range.start().clone(), first_range.end().clone()];
                let mut second_parameters =
                    [second_range.start().clone(), second_range.end().clone()];
                let mut first_endpoint_vertices = [usize::MAX; 2];
                let mut second_endpoint_vertices = [usize::MAX; 2];
                for index in [0_usize, 1] {
                    let first_existing = existing_event_vertex_if_decided(
                        &events[pair.first_carrier_index],
                        &first_parameters[index],
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
                    let second_existing = existing_event_vertex_if_decided(
                        &events[pair.second_carrier_index],
                        &second_parameters[index],
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(pair.second_carrier_index, cause))?;
                    let topology_vertex = first_existing.or(second_existing).unwrap_or_else(|| {
                        let vertex = next_topology_vertex;
                        next_topology_vertex += 1;
                        vertex
                    });
                    if let Some(previous_vertex) = second_existing
                        && previous_vertex != topology_vertex
                    {
                        replace_topology_vertex(
                            &mut events,
                            &mut contact_points,
                            previous_vertex,
                            topology_vertex,
                        );
                        contact_lookup.merge_vertex(previous_vertex, topology_vertex);
                        for retained_overlap in &mut overlaps {
                            retained_overlap
                                .replace_topology_vertex(previous_vertex, topology_vertex);
                        }
                        contact_vertex_counts[topology_vertex] +=
                            contact_vertex_counts[previous_vertex];
                        contact_vertex_counts[previous_vertex] = 0;
                        reclassification_vertices[topology_vertex] |=
                            reclassification_vertices[previous_vertex];
                        reclassification_vertices[previous_vertex] = false;
                        transition_candidates[topology_vertex] = None;
                        transition_candidates[previous_vertex] = None;
                    }
                    if contact_vertex_counts.len() <= topology_vertex {
                        contact_vertex_counts.resize(topology_vertex + 1, 0);
                        transition_candidates.resize(topology_vertex + 1, None);
                        reclassification_vertices.resize(topology_vertex + 1, false);
                    }
                    // An overlap endpoint may reuse the vertex of one
                    // already-certified point contact. Keep that contact:
                    // the face kernel has exact sector formulas for a branch
                    // entering or leaving a coincident range. Only the
                    // distinct-vertex merge above invalidates the unique
                    // four-branch contact authority.
                    reclassification_vertices[topology_vertex] = true;
                    first_endpoint_vertices[index] = topology_vertex;
                    second_endpoint_vertices[index] = topology_vertex;
                    first_parameters[index] = push_canonical_carrier_event(
                        &mut events[pair.first_carrier_index],
                        first_parameters[index].clone(),
                        Some(topology_vertex),
                        &self.data.carriers[pair.first_carrier_index],
                        &self.data.policy,
                    )?;
                    second_parameters[index] = push_canonical_carrier_event(
                        &mut events[pair.second_carrier_index],
                        second_parameters[index].clone(),
                        Some(topology_vertex),
                        &self.data.carriers[pair.second_carrier_index],
                        &self.data.policy,
                    )?;
                    if let CurveOverlapCorrespondence2::Circle {
                        source: CurveCircleOverlap2::Selected(source),
                        ..
                    } = &overlap.parameter_correspondence
                    {
                        let selected_parameter = first_parameters[index]
                            .as_selected_fiber()
                            .or_else(|| second_parameters[index].as_selected_fiber());
                        if let Some(selected_parameter) = selected_parameter {
                            let point = match source
                                .point_evidence_for_other(selected_parameter, &self.data.policy)
                                .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                            {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(reason) => {
                                    return Err(self.blocked(pair.first_carrier_index, reason));
                                }
                            };
                            contact_lookup.push(
                                contact_points.len(),
                                Some(&point),
                                topology_vertex,
                            );
                            contact_points.push(ContactVertex {
                                point: Some(point),
                                topology_vertex,
                                carrier_indices: [
                                    pair.first_carrier_index,
                                    pair.second_carrier_index,
                                ],
                                parameters: [
                                    first_parameters[index].clone(),
                                    second_parameters[index].clone(),
                                ],
                            });
                        }
                    }
                }
                first_range = CurveParameterRange2::new_validated(
                    first_parameters[0].clone(),
                    first_parameters[1].clone(),
                );
                second_range = CurveParameterRange2::new_validated(
                    second_parameters[0].clone(),
                    second_parameters[1].clone(),
                );
                overlaps.push(CarrierOverlap {
                    first_carrier_index: pair.first_carrier_index,
                    second_carrier_index: pair.second_carrier_index,
                    first_range,
                    second_range,
                    first_endpoint_vertices,
                    second_endpoint_vertices,
                    orientation: overlap.orientation,
                });
            }
        }
        if deferred_event_ordering {
            canonicalize_injective_topology_events(
                &mut events,
                &self.data.carriers,
                &self.data.policy,
            );
        }
        for (first_index, second_index, reason) in deferred_contact_matches {
            let first = &contact_points[first_index];
            let second = &contact_points[second_index];
            if first.topology_vertex != second.topology_vertex {
                return Err(self.blocked(second.carrier_indices[0], reason));
            }
        }
        for overlap in &overlaps {
            // Clipping published these exact endpoint vertices when it
            // inserted the overlap events. Reuse that topology authority
            // instead of re-comparing independently retained parameters.
            for vertex in overlap
                .first_endpoint_vertices
                .into_iter()
                .chain(overlap.second_endpoint_vertices)
            {
                if transition_candidates.get(vertex).is_some() {
                    reclassification_vertices[vertex] = true;
                }
            }
        }
        if deferred_event_ordering {
            validate_carrier_event_separation(&events, &self.data.carriers, &self.data.policy)?;
        }

        let mut exact_contact_point_index_by_vertex = vec![usize::MAX; next_topology_vertex];
        for (contact_index, contact) in contact_points.iter().enumerate() {
            if matches!(contact.point, Some(CurvePoint2(CurvePointData2::Exact(_)))) {
                exact_contact_point_index_by_vertex[contact.topology_vertex] = contact_index;
            }
        }
        let split_fragments = self
            .data
            .carriers
            .iter()
            .enumerate()
            .map(|(carrier_index, carrier)| {
                split_carrier(
                    carrier,
                    &events[carrier_index],
                    &contact_points,
                    &exact_contact_point_index_by_vertex,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))
            })
            .collect::<ExactCurveResult<Vec<_>>>()?;
        let transverse_vertices = certified_transverse_contact_vertices(
            &split_fragments,
            &mut transition_candidates,
            &self.data.policy,
        );
        let contact_candidates = transition_candidates
            .iter()
            .enumerate()
            .filter_map(|(vertex, candidate)| {
                candidate.clone().map(|candidate| (vertex, candidate))
            })
            .collect();
        let transverse_contacts = transition_candidates
            .into_iter()
            .zip(&transverse_vertices)
            .enumerate()
            .filter_map(|(vertex, (candidate, transverse))| {
                if *transverse {
                    candidate.map(|candidate| (vertex, candidate))
                } else {
                    None
                }
            })
            .collect();
        Ok(CurveRegionSplitTopology {
            split_fragments,
            overlaps,
            contact_candidates,
            transverse_contacts,
            transverse_vertices,
            reclassification_vertices,
        })
    }

    fn build_boolean_topology(&self) -> ExactCurveResult<CurveRegionBooleanTopology> {
        let CurveRegionSplitTopology {
            split_fragments,
            overlaps,
            contact_candidates: _,
            transverse_contacts: all_transverse_contacts,
            transverse_vertices: all_transverse_vertices,
            mut reclassification_vertices,
        } = self.build_split_topology()?;
        let mut transverse_vertices = vec![false; all_transverse_vertices.len()];
        let transverse_contacts = all_transverse_contacts
            .into_iter()
            .filter_map(|(vertex, contact)| {
                if contact.interior_on_both_carriers {
                    transverse_vertices[vertex] = true;
                    Some((vertex, contact))
                } else {
                    // At a carrier endpoint the adjacent authored branch, not
                    // this individual carrier tangent, determines which side
                    // crosses. Stop run propagation and classify the next
                    // open fragment directly instead of silently carrying the
                    // pre-contact face label through a certified crossing.
                    reclassification_vertices[vertex] = true;
                    None
                }
            })
            .collect();
        let mut classified_split_fragments = split_fragments
            .into_iter()
            .map(|fragments| {
                fragments
                    .into_iter()
                    .map(|split| ClassifiedSplitCarrierFragment {
                        split,
                        location: None,
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        let mut point_classification_count = 0_usize;
        for (carrier_index, fragments) in classified_split_fragments.iter_mut().enumerate() {
            for classified in fragments {
                let range = classified.split.fragment.curve_region_parameter_range();
                let (start, end) = (range.start(), range.end());
                for overlap in &overlaps {
                    let overlap_range = if overlap.first_carrier_index == carrier_index {
                        Some(&overlap.first_range)
                    } else if overlap.second_carrier_index == carrier_index {
                        Some(&overlap.second_range)
                    } else {
                        None
                    };
                    if let Some(overlap_range) = overlap_range
                        && range_contains_fragment(overlap_range, start, end, &self.data.policy)?
                    {
                        classified.location = Some(RegionPointLocation::Boundary);
                        break;
                    }
                }
            }
        }
        self.seed_transverse_boolean_locations(
            &mut classified_split_fragments,
            &transverse_contacts,
        )?;

        let mut loop_start = 0_usize;
        while loop_start < self.data.carriers.len() {
            let first = &self.data.carriers[loop_start];
            let mut loop_end = loop_start + 1;
            while loop_end < self.data.carriers.len()
                && self.data.carriers[loop_end].operand == first.operand
                && self.data.carriers[loop_end].loop_index == first.loop_index
            {
                loop_end += 1;
            }
            let loop_range = loop_start..loop_end;
            for carrier_index in loop_range.clone() {
                for split_index in 0..classified_split_fragments[carrier_index].len() {
                    if classified_split_fragments[carrier_index][split_index]
                        .location
                        .is_some()
                        && !propagate_boolean_locations_from_seed(
                            &mut classified_split_fragments,
                            loop_range.clone(),
                            (carrier_index, split_index),
                            &transverse_vertices,
                            &reclassification_vertices,
                        )
                    {
                        return Err(self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "Boolean topology produced inconsistent face labels".into(),
                            ),
                        ));
                    }
                }
            }

            // Prefer ordinary and analytic carriers. Their Cartesian
            // representatives are cheaper and can classify a whole run that
            // contains retained algebraic cusp fragments in either direction.
            // A cusp representative remains the exact final seed when a run
            // contains no other carrier.
            for cusp_pass in [false, true] {
                for carrier_index in loop_range.clone() {
                    for split_index in 0..classified_split_fragments[carrier_index].len() {
                        let classified = &classified_split_fragments[carrier_index][split_index];
                        if classified.location.is_some()
                            || matches!(
                                classified.split.fragment,
                                BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                            ) != cusp_pass
                        {
                            continue;
                        }
                        let location = match self
                            .fragment_location(carrier_index, &classified.split.fragment)
                        {
                            Ok(location) => location,
                            Err(ExactCurveError::Blocked(_)) => continue,
                            Err(error) => return Err(error),
                        };
                        classified_split_fragments[carrier_index][split_index].location =
                            Some(location);
                        point_classification_count += 1;
                        if !propagate_boolean_locations_from_seed(
                            &mut classified_split_fragments,
                            loop_range.clone(),
                            (carrier_index, split_index),
                            &transverse_vertices,
                            &reclassification_vertices,
                        ) {
                            return Err(self.invalid(
                                carrier_index,
                                CurveError::Topology(
                                    "Boolean topology produced inconsistent face labels".into(),
                                ),
                            ));
                        }
                    }
                }
            }

            if let Some((carrier_index, split_index)) =
                loop_range.clone().find_map(|carrier_index| {
                    classified_split_fragments[carrier_index]
                        .iter()
                        .position(|classified| classified.location.is_none())
                        .map(|split_index| (carrier_index, split_index))
                })
            {
                // Replay one still-unclassified representative so the public
                // blocker reports the actual remaining mathematical path,
                // rather than an earlier candidate whose run was later
                // classified from a different seed.
                self.fragment_location(
                    carrier_index,
                    &classified_split_fragments[carrier_index][split_index]
                        .split
                        .fragment,
                )?;
                return Err(self.blocked(carrier_index, UncertaintyReason::Predicate));
            }
            loop_start = loop_end;
        }
        Ok(CurveRegionBooleanTopology {
            split_fragments: classified_split_fragments,
            overlaps,
            transverse_contacts,
            point_classification_count,
        })
    }

    fn seed_transverse_boolean_locations(
        &self,
        fragments: &mut [Vec<ClassifiedSplitCarrierFragment>],
        contacts: &HashMap<usize, TransitionContactCandidate>,
    ) -> ExactCurveResult<()> {
        for (&vertex, contact) in contacts {
            let Some(source_cross_is_positive) = contact.cross_is_positive else {
                continue;
            };
            let first = &self.data.carriers[contact.first_carrier];
            let second = &self.data.carriers[contact.second_carrier];
            if first.operand == second.operand {
                continue;
            }
            let traversal_cross_is_positive =
                source_cross_is_positive ^ first.reversed ^ second.reversed;
            let first_before_inside = traversal_cross_is_positive == second.filled_side_is_left;
            let second_before_inside = traversal_cross_is_positive != first.filled_side_is_left;
            for (carrier_index, before_inside) in [
                (contact.first_carrier, first_before_inside),
                (contact.second_carrier, second_before_inside),
            ] {
                if !seed_transverse_carrier_locations(
                    fragments,
                    carrier_index,
                    vertex,
                    before_inside,
                ) {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "transverse Boolean contact produced inconsistent face labels".into(),
                        ),
                    ));
                }
            }
        }
        Ok(())
    }

    fn build_boolean_regions(&self) -> ExactCurveResult<CurveRegionBooleanResults2> {
        let topology = self.build_boolean_topology()?;
        let union = self.build_boolean_region_from_topology(BooleanOp::Union, &topology)?;
        let intersection =
            self.build_boolean_region_from_topology(BooleanOp::Intersection, &topology)?;
        let difference =
            self.build_boolean_region_from_topology(BooleanOp::Difference, &topology)?;
        let xor = match self.build_boolean_region_from_topology(BooleanOp::Xor, &topology) {
            Ok(region) => region,
            Err(ExactCurveError::Blocked(_)) => {
                self.compose_xor_from_exact_regions(&union, &intersection)?
            }
            Err(error) => return Err(error),
        };
        let regions = [union, intersection, difference, xor];
        let topology_fragment_count = topology.split_fragments.iter().map(Vec::len).sum();
        let topology_point_classification_count = topology.point_classification_count;
        Ok(CurveRegionBooleanResults2 {
            regions: Box::new(regions),
            authored_carrier_pair_count: self.data.authored_carrier_pair_count,
            candidate_carrier_pair_count: self.data.pairs.len(),
            topology_fragment_count,
            topology_point_classification_count,
        })
    }

    fn build_boolean_region(
        &self,
        operation: BooleanOp,
        topology: Option<&CurveRegionBooleanTopology>,
    ) -> ExactCurveResult<CurveRegion2> {
        let topology_storage;
        let topology = match topology {
            Some(topology) => topology,
            None => {
                topology_storage = self.build_boolean_topology()?;
                &topology_storage
            }
        };
        match self.build_boolean_region_from_topology(operation, topology) {
            Ok(region) => Ok(region),
            Err(ExactCurveError::Blocked(_)) if operation == BooleanOp::Xor => {
                self.build_xor_from_exact_set_identity()
            }
            Err(error) => Err(error),
        }
    }

    fn transition_contact_branch(
        &self,
        topology: &CurveRegionSplitTopology,
        carrier_index: usize,
        vertex: Option<usize>,
        parameter: &CurveParameter2,
    ) -> ExactCurveResult<Option<TransitionContactBranch>> {
        let Some(contact) = vertex.and_then(|vertex| topology.transverse_contacts.get(&vertex))
        else {
            return Ok(None);
        };
        let Some([first, second]) = contact.self_parameters.as_ref() else {
            return Ok(None);
        };
        for (candidate, branch) in [
            (first, TransitionContactBranch::First),
            (second, TransitionContactBranch::Second),
        ] {
            match parameter
                .same_value(candidate, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(true) => return Ok(Some(branch)),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Err(self.blocked(carrier_index, reason));
                }
            }
        }
        Err(self.blocked(carrier_index, UncertaintyReason::Predicate))
    }

    fn build_regularized_region(&self) -> ExactCurveResult<CurveRegion2> {
        if self.data.carriers.is_empty() {
            return Ok(CurveRegion2::empty());
        }
        let topology = match self.build_split_topology() {
            Ok(topology) => topology,
            Err(error) => {
                return Err(error);
            }
        };
        let simple_loop_filled_side = self.certified_simple_single_loop_filled_side(&topology);
        let fragment_selection =
            match self.regularized_fragment_actions(&topology, simple_loop_filled_side) {
                Ok(selection) => selection,
                Err(error) => {
                    return Err(error);
                }
            };
        let mut arrangement_fragments = Vec::new();
        let mut arrangement_directions = Vec::new();
        let mut arrangement_source_edge_ids = Vec::new();
        let mut source_edge_index = 0_usize;
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for (split_fragment_index, split) in splits.iter().enumerate() {
                let current_source_edge = source_edge_index;
                source_edge_index += 1;
                let source_range = split.fragment.curve_region_parameter_range();
                // Split ranges retain the carrier's source chart, while
                // topology vertices follow the fragment's traversal.
                let (source_start, source_end) = if self.data.carriers[carrier_index].reversed {
                    (source_range.end(), source_range.start())
                } else {
                    (source_range.start(), source_range.end())
                };
                let source_start_branch = self.transition_contact_branch(
                    &topology,
                    carrier_index,
                    split.start_topology_vertex,
                    source_start,
                )?;
                let source_end_branch = self.transition_contact_branch(
                    &topology,
                    carrier_index,
                    split.end_topology_vertex,
                    source_end,
                )?;
                let action = fragment_selection.actions[carrier_index][split_fragment_index];
                if action == RegionFragmentAction::Discard {
                    continue;
                }
                let fragment = match action {
                    RegionFragmentAction::Keep => split.fragment.clone(),
                    RegionFragmentAction::KeepReversed => split
                        .fragment
                        .reversed()
                        .map_err(|cause| self.invalid(carrier_index, cause))?,
                    RegionFragmentAction::Discard => unreachable!(),
                };
                let (start_topology_vertex, end_topology_vertex) = match action {
                    RegionFragmentAction::Keep => {
                        (split.start_topology_vertex, split.end_topology_vertex)
                    }
                    RegionFragmentAction::KeepReversed => {
                        (split.end_topology_vertex, split.start_topology_vertex)
                    }
                    RegionFragmentAction::Discard => unreachable!(),
                };
                arrangement_directions.push(BooleanArrangementFragmentDirection {
                    carrier_index,
                    follows_carrier: action == RegionFragmentAction::Keep,
                    start_contact_branch: match action {
                        RegionFragmentAction::Keep => source_start_branch,
                        RegionFragmentAction::KeepReversed => source_end_branch,
                        RegionFragmentAction::Discard => unreachable!(),
                    },
                    end_contact_branch: match action {
                        RegionFragmentAction::Keep => source_end_branch,
                        RegionFragmentAction::KeepReversed => source_start_branch,
                        RegionFragmentAction::Discard => unreachable!(),
                    },
                });
                arrangement_source_edge_ids.push(current_source_edge);
                arrangement_fragments.push(
                    BezierArrangementFragment2::new(carrier_index, split_fragment_index, fragment)
                        .with_topology_vertices(start_topology_vertex, end_topology_vertex),
                );
            }
        }
        if arrangement_fragments.is_empty() {
            return Ok(CurveRegion2::default());
        }
        let affine_line_output = arrangement_fragments
            .iter()
            .all(|fragment| split_fragment_is_affine_line(fragment.fragment()));
        let graph = BezierArrangementGraph2::from_certified_fragments(arrangement_fragments);
        let mut arrangement_index_by_source_edge =
            vec![NO_REGULARIZED_EDGE; fragment_selection.successor_edge_ids.len()];
        for (arrangement_index, source_edge) in
            arrangement_source_edge_ids.iter().copied().enumerate()
        {
            arrangement_index_by_source_edge[source_edge] = arrangement_index;
        }
        let face_sector_successors = arrangement_source_edge_ids
            .iter()
            .map(|source_edge| {
                let successor = fragment_selection.successor_edge_ids[*source_edge];
                arrangement_index_by_source_edge
                    .get(successor)
                    .copied()
                    .filter(|successor| *successor != NO_REGULARIZED_EDGE)
            })
            .collect::<Vec<_>>();
        let certified_successors = certified_regularization_successors(
            &graph,
            &arrangement_directions,
            &face_sector_successors,
            &topology,
            &self.data.carriers,
            &self.data.policy,
        );
        let traversal = match graph.traverse_retained_filled_left_faces_with_certified_successors(
            &certified_successors,
            &self.data.policy,
        ) {
            Classification::Decided(traversal) => traversal,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(0, reason));
            }
        };
        let mut region = match CurveRegion2::from_certified_arrangement_traversal(
            &graph,
            &traversal,
            &self.data.policy,
        ) {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(0, reason));
            }
        }
        .with_regularized_filled_left_topology(&self.data.policy)
        .map_err(|cause| self.invalid(0, cause))?;
        region = region.with_pairwise_disjoint_material_loop_roles(&self.data.policy);
        if affine_line_output || self.strict_line_image_only() {
            return self.compact_line_image_result_or_retain(region);
        }
        if simple_loop_filled_side.is_some() {
            if traversal.chains().len() != 1 {
                return Err(self.invalid(
                    0,
                    CurveError::Topology(
                        "a certified simple material loop produced multiple retained chains".into(),
                    ),
                ));
            }
            region = region
                .with_certified_loop_roles(vec![CurveRegionLoopRole::Material])
                .map_err(|cause| self.invalid(0, cause))?;
        }
        Ok(region)
    }

    fn location_from_loop_windings(
        &self,
        region: &CurveRegion2,
        windings: &LoopWindings,
    ) -> CurveResult<RegionPointLocation> {
        let Some(fill_rule) = self.data.regularization_fill_rule else {
            return region.region_location_from_loop_winding_entries(
                windings.loop_count(),
                windings.entries().iter().copied(),
            );
        };
        if windings.loop_count() != region.boundary_loops().len() {
            return Err(CurveError::Topology(
                "compound winding vector is inconsistent with boundary loops".into(),
            ));
        }
        let winding = windings
            .entries()
            .iter()
            .try_fold(0_i64, |sum, &(_, value)| {
                sum.checked_add(i64::from(value))
                    .ok_or_else(|| CurveError::Topology("compound winding overflowed i64".into()))
            })?;
        let inside = match fill_rule {
            FillRule::NonZero => winding != 0,
            FillRule::EvenOdd => winding.rem_euclid(2) != 0,
        };
        Ok(if inside {
            RegionPointLocation::Inside
        } else {
            RegionPointLocation::Outside
        })
    }

    fn location_from_windings(
        &self,
        region: &CurveRegion2,
        windings: &[i32],
    ) -> CurveResult<RegionPointLocation> {
        let Some(fill_rule) = self.data.regularization_fill_rule else {
            return region.region_location_from_loop_windings(windings);
        };
        if windings.len() != region.boundary_loops().len() {
            return Err(CurveError::Topology(
                "compound winding vector is inconsistent with boundary loops".into(),
            ));
        }
        let winding = windings.iter().try_fold(0_i64, |sum, &value| {
            sum.checked_add(i64::from(value))
                .ok_or_else(|| CurveError::Topology("compound winding overflowed i64".into()))
        })?;
        let inside = match fill_rule {
            FillRule::NonZero => winding != 0,
            FillRule::EvenOdd => winding.rem_euclid(2) != 0,
        };
        Ok(if inside {
            RegionPointLocation::Inside
        } else {
            RegionPointLocation::Outside
        })
    }

    fn region_for_carrier(&self, carrier_index: usize) -> &CurveRegion2 {
        match self.data.carriers[carrier_index].operand {
            CurveRegionBooleanOperand2::First => self.data.first,
            CurveRegionBooleanOperand2::Second => self.data.second,
        }
    }

    fn build_xor_from_exact_set_identity(&self) -> ExactCurveResult<CurveRegion2> {
        let union = self.data.first.boolean_region_raw(
            self.data.second,
            BooleanOp::Union,
            &self.data.policy,
        )?;
        let intersection = self.data.first.boolean_region_raw(
            self.data.second,
            BooleanOp::Intersection,
            &self.data.policy,
        )?;
        if let Ok(xor) =
            union.boolean_region_raw(&intersection, BooleanOp::Difference, &self.data.policy)
        {
            return Ok(xor);
        }
        self.compose_xor_from_exact_regions(&union, &intersection)
    }

    fn compose_xor_from_exact_regions(
        &self,
        union: &CurveRegion2,
        intersection: &CurveRegion2,
    ) -> ExactCurveResult<CurveRegion2> {
        let mut filled_sides = match union.filled_side_is_left_raw(&self.data.policy) {
            Ok(Classification::Decided(sides)) => sides.to_vec(),
            Ok(Classification::Uncertain(reason)) => return Err(self.blocked(0, reason)),
            Err(cause) => return Err(self.invalid(0, cause)),
        };
        let intersection_filled_sides =
            match intersection.filled_side_is_left_raw(&self.data.policy) {
                Ok(Classification::Decided(sides)) => sides,
                Ok(Classification::Uncertain(reason)) => return Err(self.blocked(0, reason)),
                Err(cause) => return Err(self.invalid(0, cause)),
            };
        filled_sides.extend(intersection_filled_sides.iter().map(|side| !side));
        // XOR is the union with the intersection removed. Concatenating those
        // boundaries preserves the set, including loops whose filled side was
        // flipped, but shared edges are not yet a normalized boundary.
        let mut union_loops = union
            .boundary_loops()
            .iter()
            .cloned()
            // Both derived regions reuse the operands' source records. Strip
            // those records before combining them into one independent region.
            .map(crate::CurveRegionBoundaryLoop2::without_arrangement_sources)
            .collect::<Vec<_>>();
        union_loops.extend(
            intersection
                .boundary_loops()
                .iter()
                .cloned()
                .map(crate::CurveRegionBoundaryLoop2::without_arrangement_sources),
        );
        let union_count = union.boundary_loops().len();
        let mut loops = union_loops;
        let mut sides = filled_sides;
        let intersection_loops = loops.split_off(union_count);
        let mut intersection_sides = sides.split_off(union_count);
        // A structurally repeated loop contributes even-odd twice and is not
        // admissible region evidence. Cancel those pairs before publication;
        // partial edge coincidence still goes through unary regularization.
        for (intersection_loop, intersection_side) in intersection_loops
            .into_iter()
            .zip(intersection_sides.drain(..))
        {
            if let Some(index) = loops
                .iter()
                .position(|union_loop| union_loop.fragments() == intersection_loop.fragments())
            {
                loops.remove(index);
                sides.remove(index);
            } else {
                loops.push(intersection_loop);
                sides.push(intersection_side);
            }
        }
        if loops.is_empty() {
            return Ok(CurveRegion2::empty());
        }
        let region = CurveRegion2::new(loops)
            .and_then(|region| region.with_certified_filled_side_is_left(sides))
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Boolean,
                    CurveFamily2::RationalBezier,
                    cause,
                )
            })?;
        // Unary regularization cancels coincident seams. It does not call XOR,
        // so this fallback cannot recurse into itself.
        region
            .regularized_region_raw(&self.data.policy)
            .map_err(|error| error.with_operation(CurveOperation2::Boolean))
    }

    fn build_boolean_region_from_topology(
        &self,
        operation: BooleanOp,
        topology: &CurveRegionBooleanTopology,
    ) -> ExactCurveResult<CurveRegion2> {
        let mut arrangement_fragments = Vec::new();
        let mut arrangement_directions = Vec::new();
        for carrier_index in 0..self.data.carriers.len() {
            for (split_fragment_index, classified) in
                topology.split_fragments[carrier_index].iter().enumerate()
            {
                let split = &classified.split;
                let action = self.fragment_action(
                    carrier_index,
                    &split.fragment,
                    classified
                        .location
                        .expect("Boolean topology classifies every split fragment"),
                    &topology.overlaps,
                    operation,
                )?;
                if action == RegionFragmentAction::Discard {
                    continue;
                }
                let fragment = match action {
                    RegionFragmentAction::Keep => split.fragment.clone(),
                    RegionFragmentAction::KeepReversed => split
                        .fragment
                        .reversed()
                        .map_err(|cause| self.invalid(carrier_index, cause))?,
                    RegionFragmentAction::Discard => unreachable!(),
                };
                let (start_topology_vertex, end_topology_vertex) = match action {
                    RegionFragmentAction::Keep => {
                        (split.start_topology_vertex, split.end_topology_vertex)
                    }
                    RegionFragmentAction::KeepReversed => {
                        (split.end_topology_vertex, split.start_topology_vertex)
                    }
                    RegionFragmentAction::Discard => unreachable!(),
                };
                arrangement_directions.push(BooleanArrangementFragmentDirection {
                    carrier_index,
                    follows_carrier: action == RegionFragmentAction::Keep,
                    start_contact_branch: None,
                    end_contact_branch: None,
                });
                arrangement_fragments.push(
                    BezierArrangementFragment2::new(carrier_index, split_fragment_index, fragment)
                        .with_topology_vertices(start_topology_vertex, end_topology_vertex),
                );
            }
        }

        let affine_line_output = !arrangement_fragments.is_empty()
            && arrangement_fragments
                .iter()
                .all(|fragment| split_fragment_is_affine_line(fragment.fragment()));
        let graph = BezierArrangementGraph2::from_certified_fragments(arrangement_fragments);
        let certified_successors = certified_boolean_successors(
            &graph,
            &arrangement_directions,
            topology,
            &self.data.carriers,
            &self.data.policy,
        );
        let primary = graph
            .traverse_retained_with_certified_successors(&certified_successors, &self.data.policy);
        // Coincident or multi-valent retained boundaries can make the
        // smallest-turn walk ambiguous even when result-side evidence is
        // complete. Retry with the same certified successor set interpreted
        // as filled-left face half-edges for every operation.
        let traversal = match primary {
            Classification::Decided(traversal) => traversal,
            Classification::Uncertain(_) => {
                match graph.traverse_retained_filled_left_faces_with_certified_successors(
                    &certified_successors,
                    &self.data.policy,
                ) {
                    Classification::Decided(traversal) => traversal,
                    Classification::Uncertain(_) => {
                        match graph.traverse_retained_with_tangent_order(&self.data.policy) {
                            Classification::Decided(traversal) => traversal,
                            Classification::Uncertain(reason) => {
                                return Err(self.blocked(0, reason));
                            }
                        }
                    }
                }
            }
        };
        let mut region = match CurveRegion2::from_certified_arrangement_traversal(
            &graph,
            &traversal,
            &self.data.policy,
        ) {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(0, reason));
            }
        };
        region = region
            .with_regularized_filled_left_topology(&self.data.policy)
            .map_err(|cause| self.invalid(0, cause))?;
        region = region.with_pairwise_disjoint_material_loop_roles(&self.data.policy);
        region = self.coalesce_certified_boolean_line_runs(region)?;
        if affine_line_output || self.strict_line_image_only() {
            self.compact_line_image_result_or_retain(region)
        } else {
            Ok(region)
        }
    }

    fn strict_line_image_only(&self) -> bool {
        *self.data.strict_line_image_only.get_or_init(|| {
            self.data
                .carriers
                .iter()
                .all(|carrier| match &carrier.geometry {
                    CurveSupport2::Bezier(curve) => subcurve_is_strict_line_image(curve),
                    CurveSupport2::Line(chord) => chord.exact_line().is_some(),
                    CurveSupport2::Parallel(_) | CurveSupport2::Circle(_) => false,
                })
        })
    }

    fn merge_certified_boolean_line_fragments(
        &self,
        first: &BezierSplitFragment2,
        second: &BezierSplitFragment2,
    ) -> ExactCurveResult<Option<BezierSplitFragment2>> {
        match (first, second) {
            (
                BezierSplitFragment2::Materialized { .. },
                BezierSplitFragment2::Materialized { .. },
            ) => {
                let line = |fragment| match crate::bezier_region::retained_line_fragment_segment(
                    fragment,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(0, cause))?
                {
                    Classification::Decided(line) => Ok(Some(line)),
                    Classification::Uncertain(_) => Ok(None),
                };
                let (Some(first), Some(second)) = (line(first)?, line(second)?) else {
                    return Ok(None);
                };
                let merged = match crate::curve_string::merge_adjacent_line_segments(
                    &crate::Segment2::Line(first),
                    &crate::Segment2::Line(second),
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(0, cause))?
                {
                    Classification::Decided(merged) => merged,
                    Classification::Uncertain(_) => None,
                };
                Ok(merged.map(|line| BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(crate::QuadraticBezier2::from_line_segment(
                        line,
                    )),
                }))
            }
            (
                BezierSplitFragment2::AlgebraicChord(first),
                BezierSplitFragment2::AlgebraicChord(second),
            ) => {
                if first
                    .tangent_cross_sign(second, &self.data.policy)
                    .map_err(|cause| self.invalid(0, cause))?
                    != Classification::Decided(RealSign::Zero)
                    || first
                        .tangent_dot_sign(second, &self.data.policy)
                        .map_err(|cause| self.invalid(0, cause))?
                        != Classification::Decided(RealSign::Positive)
                {
                    return Ok(None);
                }
                let merged = if let Some(chord) = first
                    .merge_certified_collinear_forward(second, &self.data.policy)
                    .map_err(|cause| self.invalid(0, cause))?
                {
                    Classification::Decided(chord)
                } else {
                    crate::BezierAlgebraicChord2::try_new_from_certified_distinct_endpoints(
                        first.start().clone(),
                        second.end().clone(),
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(0, cause))?
                };
                match merged {
                    Classification::Decided(chord) => {
                        Ok(Some(BezierSplitFragment2::AlgebraicChord(chord)))
                    }
                    Classification::Uncertain(_) => Ok(None),
                }
            }
            _ => Ok(None),
        }
    }

    fn coalesce_certified_boolean_line_runs(
        &self,
        region: CurveRegion2,
    ) -> ExactCurveResult<CurveRegion2> {
        let mut changed = false;
        let mut boundaries = Vec::with_capacity(region.boundary_loops().len());
        let mut next_arrangement_fragment = region
            .boundary_loops()
            .iter()
            .filter_map(|boundary| boundary.arrangement_sources())
            .flatten()
            .map(|source| source.arrangement_fragment_index())
            .max()
            .map_or(0, |index| index.saturating_add(1));
        for boundary in region.boundary_loops() {
            let mut boundary_changed = false;
            let mut source = boundary.fragments().iter().cloned();
            let Some(mut current) = source.next() else {
                return Err(self.invalid(
                    0,
                    CurveError::Topology("a retained Boolean loop was empty".into()),
                ));
            };
            let mut fragments = Vec::with_capacity(boundary.len());
            for next in source {
                if let Some(merged) =
                    self.merge_certified_boolean_line_fragments(&current, &next)?
                {
                    current = merged;
                    changed = true;
                    boundary_changed = true;
                } else {
                    fragments.push(current);
                    current = next;
                }
            }
            fragments.push(current);
            if fragments.len() > 1
                && let Some(merged) = self.merge_certified_boolean_line_fragments(
                    fragments.last().expect("nonempty coalesced loop"),
                    &fragments[0],
                )?
            {
                fragments.pop();
                fragments[0] = merged;
                changed = true;
                boundary_changed = true;
            }
            for fragment in &mut fragments {
                if matches!(fragment, BezierSplitFragment2::Materialized { .. }) {
                    continue;
                }
                let line = match crate::bezier_region::retained_line_fragment_segment(
                    fragment,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(0, cause))?
                {
                    Classification::Decided(line) => line,
                    Classification::Uncertain(_) => continue,
                };
                *fragment = BezierSplitFragment2::Materialized {
                    start: BezierParameter2::Exact(Real::zero()),
                    end: BezierParameter2::Exact(Real::one()),
                    curve: BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(line)),
                };
                changed = true;
                boundary_changed = true;
            }
            if !boundary_changed {
                boundaries.push(boundary.clone());
                continue;
            }
            let arrangement_sources = (0..fragments.len())
                .map(|source_fragment_index| {
                    let arrangement_fragment_index = next_arrangement_fragment;
                    next_arrangement_fragment += 1;
                    crate::CurveRegionFragmentSource2::new(
                        arrangement_fragment_index,
                        arrangement_fragment_index,
                        source_fragment_index,
                    )
                })
                .collect();
            boundaries.push(
                crate::CurveRegionBoundaryLoop2::try_new_from_certified_connected_chain(
                    fragments,
                    Some(arrangement_sources),
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(0, cause))?,
            );
        }
        if !changed {
            return Ok(region);
        }
        CurveRegion2::new(boundaries)
            .and_then(|region| region.with_regularized_filled_left_topology(&self.data.policy))
            .map_err(|cause| self.invalid(0, cause))
    }

    fn compact_line_image_result_or_retain(
        &self,
        mut region: CurveRegion2,
    ) -> ExactCurveResult<CurveRegion2> {
        match self.compact_line_image_result(&mut region) {
            Ok(Some(compacted)) => Ok(compacted),
            Ok(None) | Err(ExactCurveError::Blocked(_)) => Ok(region),
            Err(error) => Err(error),
        }
    }

    fn compact_line_image_result(
        &self,
        region: &mut CurveRegion2,
    ) -> ExactCurveResult<Option<CurveRegion2>> {
        if region.is_empty() {
            return Ok(None);
        }
        let mut material = Vec::new();
        let mut holes = Vec::new();
        let mut mixed_roles = None::<Vec<crate::CurveRegionLoopRole>>;
        let mut reduced_fragment_count = false;
        for (loop_index, boundary) in region.boundary_loops().iter().enumerate() {
            let segments = boundary
                .fragments()
                .iter()
                .map(|fragment| {
                    if let BezierSplitFragment2::Materialized {
                        curve: BezierSubcurve2::Quadratic(curve),
                        ..
                    } = fragment
                        && let Some(line) = curve.retained_exact_line_image()
                    {
                        return Ok(crate::Segment2::Line(line.clone()));
                    }
                    match crate::bezier_region::retained_line_fragment_segment(
                        fragment,
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(0, cause))?
                    {
                        Classification::Decided(line) => Ok(crate::Segment2::Line(line)),
                        Classification::Uncertain(reason) => Err(self.blocked(0, reason)),
                    }
                })
                .collect::<ExactCurveResult<Vec<_>>>()?;
            let contour =
                crate::Contour2::from_validated_closed_segments(segments, FillRule::NonZero);
            let contour = match contour
                .merge_adjacent_collinear_lines(&self.data.policy)
                .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(contour) => contour,
                Classification::Uncertain(reason) => return Err(self.blocked(0, reason)),
            };
            reduced_fragment_count |= contour.len() < boundary.len();
            let area = contour
                .signed_area()
                .map_err(|cause| self.invalid(0, cause))?
                .expect("line contours always have an exact signed area");
            match crate::classify::compare_reals(&area, &crate::Real::zero(), &self.data.policy) {
                Some(Ordering::Greater) => {
                    if let Some(roles) = &mut mixed_roles {
                        roles.push(crate::CurveRegionLoopRole::Material);
                    }
                    material.push(contour);
                }
                Some(Ordering::Less) => {
                    mixed_roles
                        .get_or_insert_with(|| {
                            vec![crate::CurveRegionLoopRole::Material; loop_index]
                        })
                        .push(crate::CurveRegionLoopRole::Hole);
                    holes.push(contour);
                }
                Some(Ordering::Equal) => {
                    if affine_contour_is_exact_zero_chain(&contour) {
                        reduced_fragment_count = true;
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-boolean",
                            "discard-exact-zero-affine-chain",
                        );
                        continue;
                    }
                    return Err(self.invalid(
                        0,
                        CurveError::Topology(
                            "regularized Boolean emitted a zero-area affine line loop".into(),
                        ),
                    ));
                }
                None => return Err(self.blocked(0, UncertaintyReason::RealSign)),
            }
        }
        if !reduced_fragment_count {
            let loop_count = region.len();
            let region = std::mem::take(region);
            return match mixed_roles {
                Some(roles) => region.with_certified_loop_roles(roles),
                None => region.with_certified_all_material_loop_roles(loop_count),
            }
            .map(Some)
            .map_err(|cause| self.invalid(0, cause));
        }
        CurveRegion2::from_certified_oriented_line_contours(material, holes, &self.data.policy)
            .map(Some)
            .map_err(|cause| self.invalid(0, cause))
    }

    fn invalid(&self, carrier_index: usize, cause: CurveError) -> ExactCurveError {
        let carrier = &self.data.carriers[carrier_index];
        ExactCurveError::invalid(CurveOperation2::Boolean, carrier.family, cause)
    }

    fn blocked(&self, carrier_index: usize, reason: UncertaintyReason) -> ExactCurveError {
        let carrier = &self.data.carriers[carrier_index];
        ExactCurveError::blocked(CurveOperation2::Boolean, carrier.family, reason)
    }
}

/// Returns true only when every oriented affine segment is cancelled by one
/// exactly reversed mate.
///
/// A regularized arrangement can retain this lower-dimensional zero chain
/// when two equivalent boundaries use different carrier partitions. Signed
/// area alone is not a sufficient deletion certificate: a self-intersecting
/// contour can also have zero area. Pairwise reverse incidence proves the
/// stronger statement that the complete oriented boundary chain is zero.
fn affine_contour_is_exact_zero_chain(contour: &crate::Contour2) -> bool {
    let segments = contour.segments();
    if !segments.len().is_multiple_of(2) {
        return false;
    }
    let exact_point_equal = |first: &crate::Point2, second: &crate::Point2| {
        compare_reals(first.x(), second.x(), &CurveContext::STRICT) == Some(Ordering::Equal)
            && compare_reals(first.y(), second.y(), &CurveContext::STRICT) == Some(Ordering::Equal)
    };
    let mut paired = vec![false; segments.len()];
    for first_index in 0..segments.len() {
        if paired[first_index] {
            continue;
        }
        let crate::Segment2::Line(first) = &segments[first_index] else {
            return false;
        };
        let Some(second_index) = ((first_index + 1)..segments.len()).find(|second_index| {
            if paired[*second_index] {
                return false;
            }
            let crate::Segment2::Line(second) = &segments[*second_index] else {
                return false;
            };
            exact_point_equal(first.start(), second.end())
                && exact_point_equal(first.end(), second.start())
        }) else {
            return false;
        };
        paired[first_index] = true;
        paired[second_index] = true;
    }
    true
}

fn region_carrier_count(region: &CurveRegion2) -> usize {
    region
        .boundary_loops()
        .iter()
        .map(|boundary| boundary.fragments().len())
        .sum()
}

/// Prepares pair operands in their original support charts. A native unit
/// carrier stays native; retained ranges keep endpoint images and admission
/// evidence for the common finite-domain intersection kernel.
fn prepare_carrier_curve(
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<Curve2> {
    carrier
        .geometry
        .restrict_certified(
            carrier.range(),
            carrier.selected_fiber_endpoint_points.as_deref().cloned(),
            // Arrangement traversal is applied by the owning carrier. Pair
            // parameters and tangent signs use increasing source order.
            false,
            policy,
        )
        .map(Curve2::from_retained_fragment)
        .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Boolean, carrier.family, cause))
}

fn prepare_bezier_carrier_curves(
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> ExactCurveResult<Vec<Option<Curve2>>> {
    carriers
        .iter()
        .map(|carrier| {
            matches!(carrier.geometry, CurveSupport2::Bezier(_))
                .then(|| prepare_carrier_curve(carrier, policy))
                .transpose()
        })
        .collect()
}

fn carrier_scheduling_bounds(
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> Vec<Option<Aabb2>> {
    policy.strict_predicate_pass(|| {
        carriers
            .iter()
            .map(
                |carrier| match carrier_optional_outer_bounds_refined(carrier, 0, policy) {
                    Classification::Decided(bounds) => Some(bounds),
                    Classification::Uncertain(_) => None,
                },
            )
            .collect()
    })
}

fn build_unary_carrier_pairs(
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> ExactCurveResult<Vec<RegionCarrierPair>> {
    let count = carriers.len();
    let curves = prepare_bezier_carrier_curves(carriers, policy)?;
    let mut pairs = Vec::with_capacity(count.saturating_mul(2));
    let mut intersection_cache = CurveIntersectionBatchCache::default();
    let mut visit = |first, second, _| -> ExactCurveResult<()> {
        if first < second
            && let Some(pair) = build_candidate_carrier_pair(
                carriers,
                &curves,
                first,
                second,
                policy,
                &mut intersection_cache,
            )?
        {
            pairs.push(pair);
        }
        Ok(())
    };
    // Unary normalization has the same certified rejection authority as
    // binary Booleans. Query the same envelopes on both sides and visit each
    // unordered pair once. Unknown envelopes retain the complete fallback.
    let scheduled = if count.saturating_mul(count) >= MIN_AABB_SWEEP_PAIR_COUNT {
        let bounds = carrier_scheduling_bounds(carriers, policy);
        visit_aabb_pair_candidates(
            &bounds,
            &bounds,
            count,
            count,
            None,
            &CurveContext::STRICT,
            &mut visit,
        )
    } else {
        None
    };
    match scheduled {
        Some(result) => result?,
        None => {
            for first in 0..count {
                for second in first + 1..count {
                    visit(first, second, false)?;
                }
            }
        }
    }
    let pair_count = pairs.len();
    for (index, carrier) in carriers.iter().enumerate() {
        if !carrier_has_certified_injective_image(carrier, policy) {
            pairs.push(RegionCarrierPair {
                first_carrier_index: index,
                second_carrier_index: index,
                context: RegionCarrierPairContext::Common(CurveIntersectionContext::new_self(
                    &match &curves[index] {
                        Some(curve) => curve.clone(),
                        None => prepare_carrier_curve(carrier, policy)?,
                    },
                    policy,
                    &mut intersection_cache,
                )),
            });
        }
    }
    if pairs.len() != pair_count {
        // Keep the original authored order: cross contacts followed by the
        // source's own self contacts, before proceeding to the next carrier.
        pairs.sort_unstable_by_key(|pair| {
            (
                pair.first_carrier_index,
                if pair.first_carrier_index == pair.second_carrier_index {
                    count
                } else {
                    pair.second_carrier_index
                },
            )
        });
    }
    Ok(pairs)
}

fn build_cross_operand_carrier_pairs(
    carriers: &[RegionCarrier],
    first_carrier_count: usize,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<RegionCarrierPair>> {
    let second_carrier_count = carriers.len() - first_carrier_count;
    let cartesian_pair_count = first_carrier_count.saturating_mul(second_carrier_count);
    let curves = prepare_bezier_carrier_curves(carriers, policy)?;
    let mut pairs = Vec::with_capacity(carriers.len().min(cartesian_pair_count));
    let mut intersection_cache = CurveIntersectionBatchCache::default();
    let mut visit = |first_index, second_index, _| -> ExactCurveResult<()> {
        if let Some(pair) = build_candidate_carrier_pair(
            carriers,
            &curves,
            first_index,
            first_carrier_count + second_index,
            policy,
            &mut intersection_cache,
        )? {
            pairs.push(pair);
        }
        Ok(())
    };
    if cartesian_pair_count >= MIN_AABB_SWEEP_PAIR_COUNT {
        // Retain one optional envelope per carrier, not one per Cartesian
        // pair. Only the rejection proof suppresses terminal approximation;
        // surviving pairs still enter their kernel with the original policy.
        let bounds = carrier_scheduling_bounds(carriers, policy);
        let (first_bounds, second_bounds) = bounds.split_at(first_carrier_count);
        if let Some(result) = visit_aabb_pair_candidates(
            first_bounds,
            second_bounds,
            first_carrier_count,
            second_carrier_count,
            None,
            &CurveContext::STRICT,
            &mut visit,
        ) {
            result?;
            return Ok(pairs);
        }
    }
    for first_index in 0..first_carrier_count {
        for second_index in 0..second_carrier_count {
            visit(first_index, second_index, false)?;
        }
    }
    Ok(pairs)
}

fn build_candidate_carrier_pair(
    carriers: &[RegionCarrier],
    curves: &[Option<Curve2>],
    first_carrier_index: usize,
    second_carrier_index: usize,
    policy: &CurveContext,
    intersection_cache: &mut CurveIntersectionBatchCache,
) -> ExactCurveResult<Option<RegionCarrierPair>> {
    let first_carrier = &carriers[first_carrier_index];
    let second_carrier = &carriers[second_carrier_index];
    // Consecutive authored fragments share their loop vertex by construction.
    // They therefore cannot be rejected by bounds, and asking an algebraic
    // endpoint for a box here can be much more expensive than admitting the
    // pair to the authoritative exact kernel.
    if !authored_carriers_are_adjacent_in_set(carriers, first_carrier, second_carrier)
        && carrier_bounds_decided_disjoint(first_carrier, second_carrier, policy)
    {
        return Ok(None);
    }
    let context = match (&first_carrier.geometry, &second_carrier.geometry) {
        (CurveSupport2::Circle(_), CurveSupport2::Line(_)) => RegionCarrierPairContext::CuspChord {
            cusp_is_first: true,
        },
        (CurveSupport2::Line(_), CurveSupport2::Circle(_)) => RegionCarrierPairContext::CuspChord {
            cusp_is_first: false,
        },
        (CurveSupport2::Line(_), _) | (_, CurveSupport2::Line(_)) => {
            RegionCarrierPairContext::AlgebraicChordPair {
                endpoint_contact: None,
            }
        }
        (CurveSupport2::Bezier(_), CurveSupport2::Bezier(_)) => {
            let first = curves[first_carrier_index]
                .as_ref()
                .expect("Bezier carrier has a top-level curve");
            let second = curves[second_carrier_index]
                .as_ref()
                .expect("Bezier carrier has a top-level curve");
            let context = CurveIntersectionContext::try_new_with_batch_cache(
                first,
                second,
                policy,
                intersection_cache,
            )?;
            RegionCarrierPairContext::Common(context)
        }
        (CurveSupport2::Parallel(_), CurveSupport2::Bezier(_)) => {
            RegionCarrierPairContext::ParallelRational {
                parallel_is_first: true,
            }
        }
        (CurveSupport2::Bezier(_), CurveSupport2::Parallel(_)) => {
            RegionCarrierPairContext::ParallelRational {
                parallel_is_first: false,
            }
        }
        (CurveSupport2::Parallel(first), CurveSupport2::Parallel(second)) => {
            if first == second {
                RegionCarrierPairContext::ParallelSameImage
            } else {
                RegionCarrierPairContext::ParallelPair
            }
        }
        (CurveSupport2::Circle(_), CurveSupport2::Bezier(_)) => {
            RegionCarrierPairContext::CuspRational {
                cusp_is_first: true,
            }
        }
        (CurveSupport2::Bezier(_), CurveSupport2::Circle(_)) => {
            RegionCarrierPairContext::CuspRational {
                cusp_is_first: false,
            }
        }
        (CurveSupport2::Circle(_), CurveSupport2::Parallel(_)) => {
            RegionCarrierPairContext::CuspParallel {
                cusp_is_first: true,
            }
        }
        (CurveSupport2::Parallel(_), CurveSupport2::Circle(_)) => {
            RegionCarrierPairContext::CuspParallel {
                cusp_is_first: false,
            }
        }
        (CurveSupport2::Circle(_), CurveSupport2::Circle(_)) => RegionCarrierPairContext::CuspPair,
    };
    Ok(Some(RegionCarrierPair {
        first_carrier_index,
        second_carrier_index,
        context,
    }))
}

fn authored_carriers_are_adjacent_in_set(
    carriers: &[RegionCarrier],
    first: &RegionCarrier,
    second: &RegionCarrier,
) -> bool {
    if first.operand != second.operand || first.loop_index != second.loop_index {
        return false;
    }
    if first.fragment_index.abs_diff(second.fragment_index) == 1 {
        return true;
    }
    let last_fragment_index = carriers
        .iter()
        .filter(|carrier| {
            carrier.operand == first.operand && carrier.loop_index == first.loop_index
        })
        .map(|carrier| carrier.fragment_index)
        .max();
    matches!(
        last_fragment_index,
        Some(last)
            if (first.fragment_index == 0 && second.fragment_index == last)
                || (second.fragment_index == 0 && first.fragment_index == last)
    )
}

/// A reversal identity certifies a zero oriented boundary chain, even when
/// the image is curved. This is a filled-set reduction, not a curve-image
/// simplification: open curve queries must retain every parameter visit.
fn carrier_is_symmetric_zero_chain(carrier: &RegionCarrier, policy: &CurveContext) -> bool {
    if carrier.start.scalar() != Some(&Real::zero()) || carrier.end.scalar() != Some(&Real::one()) {
        return false;
    }
    let CurveSupport2::Bezier(curve) = &carrier.geometry else {
        return false;
    };
    let equal =
        |first: &Real, second: &Real| compare_reals(first, second, policy) == Some(Ordering::Equal);
    let same_point = |first: &crate::Point2, second: &crate::Point2| {
        equal(first.x(), second.x()) && equal(first.y(), second.y())
    };
    match curve {
        BezierSubcurve2::Quadratic(curve) => same_point(curve.start(), curve.end()),
        BezierSubcurve2::Cubic(curve) => {
            same_point(curve.start(), curve.end()) && same_point(curve.control1(), curve.control2())
        }
        BezierSubcurve2::RationalQuadratic(curve) => {
            same_point(curve.start(), curve.end())
                && equal(curve.start_weight(), curve.end_weight())
                && matches!(
                    RationalBezier2::from(curve.clone())
                        .denominator_sign(&CurveParameterRange2::unit()),
                    Classification::Decided(RealSign::Positive | RealSign::Negative)
                )
        }
        BezierSubcurve2::Rational(curve) => {
            curve
                .homogeneous_controls()
                .iter()
                .zip(curve.homogeneous_controls().iter().rev())
                .all(|(first, second)| {
                    equal(first.x(), second.x())
                        && equal(first.y(), second.y())
                        && equal(first.weight(), second.weight())
                })
                && matches!(
                    curve.denominator_sign(&CurveParameterRange2::unit()),
                    Classification::Decided(RealSign::Positive | RealSign::Negative)
                )
        }
    }
}

fn split_fragment_is_affine_line(fragment: &BezierSplitFragment2) -> bool {
    match fragment {
        BezierSplitFragment2::Materialized { curve, .. } => subcurve_is_strict_line_image(curve),
        BezierSplitFragment2::AlgebraicChord(chord) => chord.exact_line().is_some(),
        _ => false,
    }
}

fn subcurve_is_strict_line_image(curve: &BezierSubcurve2) -> bool {
    let fit = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.fit_exact_line_image(&CurveContext::STRICT),
        BezierSubcurve2::Cubic(curve) => curve.fit_exact_line_image(&CurveContext::STRICT),
        BezierSubcurve2::RationalQuadratic(curve) => {
            curve.fit_exact_line_image(&CurveContext::STRICT)
        }
        BezierSubcurve2::Rational(curve) => curve.fit_exact_line_image(&CurveContext::STRICT),
    };
    matches!(
        fit,
        Ok(Classification::Decided(BezierLineImageFitRelation::Fit(_)))
    )
}

fn exact_axis_aligned_line_direction(
    line: &LineSeg2,
) -> Option<BezierAlgebraicChordAxisDirection2> {
    let strict = CurveContext::STRICT;
    let x_order = compare_reals(line.start().x(), line.end().x(), &strict)?;
    let y_order = compare_reals(line.start().y(), line.end().y(), &strict)?;
    match (x_order, y_order) {
        (Ordering::Less, Ordering::Equal) => Some(BezierAlgebraicChordAxisDirection2::PositiveX),
        (Ordering::Greater, Ordering::Equal) => Some(BezierAlgebraicChordAxisDirection2::NegativeX),
        (Ordering::Equal, Ordering::Less) => Some(BezierAlgebraicChordAxisDirection2::PositiveY),
        (Ordering::Equal, Ordering::Greater) => Some(BezierAlgebraicChordAxisDirection2::NegativeY),
        (Ordering::Less | Ordering::Equal | Ordering::Greater, _) => None,
    }
}

fn retained_axis_aligned_line_chord(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
) -> Option<crate::BezierAlgebraicChord2> {
    let BezierSubcurve2::Quadratic(curve) = curve else {
        return None;
    };
    let line = curve.retained_exact_line_image()?;
    let direction = exact_axis_aligned_line_direction(line)?;
    Some(
        crate::BezierAlgebraicChord2::from_certified_axis_aligned_endpoints(
            CurvePoint2::from(line.start().clone()),
            CurvePoint2::from(line.end().clone()),
            direction,
            policy,
        ),
    )
}

fn boolean_trivial_region(
    first: &CurveRegion2,
    second: &CurveRegion2,
    operation: BooleanOp,
) -> ExactCurveResult<Option<CurveRegion2>> {
    if first.is_empty() || second.is_empty() {
        return empty_operand_result(first, second, operation).map(Some);
    }
    if first == second {
        return identical_operand_result(first, operation).map(Some);
    }
    Ok(None)
}

fn carrier_bounds_decided_disjoint(
    first: &RegionCarrier,
    second: &RegionCarrier,
    policy: &CurveContext,
) -> bool {
    // Bounds are an optional rejection proof. They must never consume the
    // APPROXIMATE_512 terminal or weaken the certainty of an operation whose
    // authoritative carrier kernel can decide the pair exactly.
    policy.strict_predicate_pass(|| {
        for refinement_steps in [0, 2] {
            let first_bounds =
                carrier_optional_outer_bounds_refined(first, refinement_steps, policy);
            let second_bounds =
                carrier_optional_outer_bounds_refined(second, refinement_steps, policy);
            if let (Classification::Decided(first), Classification::Decided(second)) =
                (first_bounds, second_bounds)
                && first.overlaps(&second, policy) == Classification::Decided(false)
            {
                return true;
            }
        }
        false
    })
}

/// Returns an optional rejection box without forcing a nonmaterialized chord
/// to join its retained endpoint fields. Rational outer envelopes make boxes
/// from independent fields directly comparable while remaining conservative.
fn carrier_optional_outer_bounds_refined(
    carrier: &RegionCarrier,
    refinement_steps: usize,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let compute = || carrier_optional_outer_bounds_uncached(carrier, refinement_steps, policy);
    // Level zero already shares `carrier.bounds`; cache only the refinements.
    match CARRIER_BOUND_REFINEMENTS
        .iter()
        .position(|&steps| steps == refinement_steps)
    {
        Some(level) if refinement_steps != 0 => {
            carrier.refined_bounds[level].get_or_init(compute).clone()
        }
        _ => compute(),
    }
}

fn carrier_optional_outer_bounds_uncached(
    carrier: &RegionCarrier,
    refinement_steps: usize,
    policy: &CurveContext,
) -> Classification<Aabb2> {
    let bounds = match &carrier.geometry {
        CurveSupport2::Line(chord) if chord.exact_line().is_none() => chord
            .conservative_local_bounds_refined(refinement_steps, policy)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported)),
        _ if refinement_steps == 0 => carrier
            .bounds
            .get_or_init(|| {
                carrier
                    .geometry
                    .certified_outer_bounds(&carrier.range(), 0, policy)
            })
            .clone(),
        _ => carrier
            .geometry
            .certified_outer_bounds(&carrier.range(), refinement_steps, policy),
    };
    bounds.map(|bounds| {
        bounds
            .certified_rational_outer_envelope(refinement_steps)
            .unwrap_or(bounds)
    })
}

fn build_region_carriers(
    region: &CurveRegion2,
    operand: CurveRegionBooleanOperand2,
    policy: &CurveContext,
    rational_quadratic_area_cache: &mut RationalQuadraticAreaIntegralCache,
    require_filled_sides: bool,
) -> ExactCurveResult<Vec<RegionCarrier>> {
    if region.is_empty() {
        return Ok(Vec::new());
    }
    let filled_sides = if require_filled_sides {
        match region
            .filled_side_is_left_with_area_cache(policy, rational_quadratic_area_cache)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, cause)
            })? {
            Classification::Decided(sides) => sides.to_vec(),
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Boolean,
                    CurveFamily2::Line,
                    reason,
                ));
            }
        }
    } else {
        vec![false; region.boundary_loops().len()]
    };
    let mut carriers = Vec::new();
    for (loop_index, boundary_loop) in region.boundary_loops().iter().enumerate() {
        for (fragment_index, fragment) in boundary_loop.fragments().iter().enumerate() {
            carriers.push(build_region_carrier(
                fragment,
                operand,
                loop_index,
                fragment_index,
                filled_sides[loop_index],
                policy,
            )?);
        }
    }
    Ok(carriers)
}

fn build_parameterized_carrier(
    fragment: &BezierSplitFragment2,
    operand: CurveRegionBooleanOperand2,
    loop_index: usize,
    fragment_index: usize,
    filled_side_is_left: bool,
) -> RegionCarrier {
    let selected_fiber_endpoint_points = match fragment {
        BezierSplitFragment2::SelectedFiber(fragment) => {
            let points = if fragment.is_reversed() {
                [fragment.end_point().clone(), fragment.start_point().clone()]
            } else {
                [fragment.start_point().clone(), fragment.end_point().clone()]
            };
            Some(Arc::new(points))
        }
        _ => None,
    };
    let geometry = CurveSupport2::from_fragment(fragment);
    let range = if matches!(fragment, BezierSplitFragment2::Materialized { .. }) {
        CurveParameterRange2::unit()
    } else {
        fragment.curve_region_parameter_range()
    };
    RegionCarrier {
        operand,
        loop_index,
        fragment_index,
        family: geometry.family(),
        geometry,
        start: range.start().clone(),
        end: range.end().clone(),
        reversed: fragment.source_is_reversed(),
        filled_side_is_left,
        selected_fiber_endpoint_points,
        image_is_injective: OnceLock::new(),
        bounds: OnceLock::new(),
        refined_bounds: Default::default(),
    }
}

fn build_region_carrier(
    fragment: &BezierSplitFragment2,
    operand: CurveRegionBooleanOperand2,
    loop_index: usize,
    fragment_index: usize,
    filled_side_is_left: bool,
    policy: &CurveContext,
) -> ExactCurveResult<RegionCarrier> {
    let mut carrier = build_parameterized_carrier(
        fragment,
        operand,
        loop_index,
        fragment_index,
        filled_side_is_left,
    );
    // Region boundaries may choose a simpler parameter chart. Open curve
    // queries retain the authored chart through build_parameterized_carrier.
    if let BezierSplitFragment2::Materialized { curve, .. } = fragment
        && let Some(chord) = retained_axis_aligned_line_chord(curve, policy)
    {
        carrier.start = CurveParameter2::from_algebraic_chord(chord.start_parameter());
        carrier.end = CurveParameter2::from_algebraic_chord(chord.end_parameter());
        carrier.geometry = CurveSupport2::Line(chord);
    }
    if matches!(fragment, BezierSplitFragment2::RetainedBezier { .. })
        && let Ok(Classification::Decided(line)) =
            crate::bezier_region::retained_line_fragment_segment(fragment, policy)
    {
        carrier.geometry = CurveSupport2::Bezier(BezierSubcurve2::Quadratic(
            QuadraticBezier2::from_line_segment(line),
        ));
        carrier.start = Real::zero().into();
        carrier.end = Real::one().into();
        carrier.reversed = false;
    }
    carrier.family = carrier.geometry.family();
    Ok(carrier)
}

fn split_carrier(
    carrier: &RegionCarrier,
    events: &[CarrierEvent],
    contact_points: &[ContactVertex],
    exact_contact_point_index_by_vertex: &[usize],
    policy: &CurveContext,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
    if carrier.selected_fiber_endpoint_points.is_some()
        || events
            .iter()
            .any(|event| event.parameter.is_retained_scalar())
    {
        return split_selected_fiber_carrier(carrier, events, contact_points, policy);
    }
    if let CurveSupport2::Line(chord) = &carrier.geometry {
        return split_algebraic_chord_carrier(carrier, chord, events, policy);
    }
    if let CurveSupport2::Circle(fragment) = &carrier.geometry {
        return split_algebraic_cusp_carrier(carrier, fragment, events, contact_points, policy);
    }
    // Most retained events need very little isolator separation. Preserve the
    // former eight-step proof budget for close roots or endpoint images whose
    // complete topology replay needs a narrower interval.
    for max_refinement_steps in [0, 1, 2, 4] {
        if let Ok(fragments) = split_carrier_with_refinement(
            carrier,
            events,
            contact_points,
            exact_contact_point_index_by_vertex,
            max_refinement_steps,
            policy,
        ) {
            return Ok(fragments);
        }
    }
    split_carrier_with_refinement(
        carrier,
        events,
        contact_points,
        exact_contact_point_index_by_vertex,
        8,
        policy,
    )
}

fn split_carrier_with_refinement(
    carrier: &RegionCarrier,
    events: &[CarrierEvent],
    contact_points: &[ContactVertex],
    exact_contact_point_index_by_vertex: &[usize],
    max_refinement_steps: usize,
    policy: &CurveContext,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
    // Specialized event splitters receive only authored endpoints, contacts
    // admitted by `parameter_in_carrier`, and clipped overlap endpoints.
    // Their sorted event windows are therefore already within the finite
    // carrier. Bezier materialization uses that same certified range.
    if matches!(carrier.geometry, CurveSupport2::Parallel(_)) {
        return split_analytic_carrier(carrier, events, max_refinement_steps, policy);
    }
    let parameters = events
        .iter()
        .map(|event| {
            event
                .parameter
                .as_bezier_parameter()
                .ok_or_else(|| {
                    CurveError::Topology("algebraic cusp cut reached the Bezier split path".into())
                })
                .cloned()
        })
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .map(|parameter| parameter.refined_isolating_interval(max_refinement_steps, policy))
        .collect::<Vec<_>>();
    let range = BezierParameterRange2::new_validated(
        carrier
            .start
            .as_bezier_parameter()
            .ok_or(CurveError::InvalidCurveParameter)?
            .clone(),
        carrier
            .end
            .as_bezier_parameter()
            .ok_or(CurveError::InvalidCurveParameter)?
            .clone(),
    );
    let materialization =
        match carrier
            .geometry
            .bezier()
            .split_at_parameters_refined(&range, &parameters, policy)?
        {
            Classification::Decided(materialization) => materialization,
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "retained curved-region split remained uncertain: {reason:?}"
                )));
            }
        };
    let mut output = Vec::new();
    for fragment in materialization.fragments() {
        let Some((start, end)) = fragment_range(fragment) else {
            return Err(CurveError::Topology(
                "algebraic cusp carrier reached the Bezier split path".into(),
            ));
        };
        let start_parameter = CurveParameter2::from(start.clone());
        let end_parameter = CurveParameter2::from(end.clone());
        let start_topology_vertex = event_vertex(events, &start_parameter, policy)?;
        let end_topology_vertex = event_vertex(events, &end_parameter, policy)?;
        let fragment = compact_retained_circular_fragment(
            fragment,
            carrier,
            start_topology_vertex,
            end_topology_vertex,
            contact_points,
            exact_contact_point_index_by_vertex,
            policy,
        );
        output.push(SplitCarrierFragment {
            fragment: if carrier.reversed {
                fragment.reversed()?
            } else {
                fragment
            },
            start_topology_vertex,
            end_topology_vertex,
        });
    }
    if carrier.reversed {
        output.reverse();
        for fragment in &mut output {
            std::mem::swap(
                &mut fragment.start_topology_vertex,
                &mut fragment.end_topology_vertex,
            );
        }
    }
    Ok(output)
}

fn selected_fiber_event_point(
    event: &CarrierEvent,
    carrier: &RegionCarrier,
    source: &BezierSelectedFiberSource2,
    contact_points: &[ContactVertex],
    policy: &CurveContext,
) -> Result<CurvePoint2, CurveError> {
    if let Some(points) = &carrier.selected_fiber_endpoint_points {
        if event.parameter == carrier.start {
            return Ok(points[0].clone());
        }
        if event.parameter == carrier.end {
            return Ok(points[1].clone());
        }
    }
    if let Some(vertex) = event.topology_vertex
        && let Some(point) = contact_points
            .iter()
            .filter(|contact| contact.topology_vertex == vertex)
            .find_map(|contact| contact.point.clone())
    {
        return Ok(point);
    }
    let parameter = if let Some(parameter) = event.parameter.as_bezier_parameter() {
        parameter.clone()
    } else {
        match policy
            .strict_predicate_pass(|| event.parameter.promoted_bezier_parameter_complete(policy))?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "a retained-scalar boundary lost its exact point evidence: {reason:?}"
                )));
            }
        }
    };
    match source {
        BezierSelectedFiberSource2::Rational(curve) => {
            match exact_contact_point_evidence(curve, &parameter, policy)? {
                Classification::Decided(point) => Ok(point),
                Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                    "a selected-fiber rational boundary could not retain its exact point: {reason:?}"
                ))),
            }
        }
        BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
            if let Some(parameter) = parameter.scalar() {
                return match parallel.point_at(parameter, policy)? {
                    Classification::Decided(point) => Ok(CurvePoint2::from(point)),
                    Classification::Uncertain(reason) => Err(CurveError::Topology(format!(
                        "selected-fiber analytic endpoint evaluation remained uncertain: {reason:?}"
                    ))),
                };
            }
            Ok(CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new(
                parallel.clone(),
                parameter,
                policy,
            )))
        }
    }
}

fn split_selected_fiber_carrier(
    carrier: &RegionCarrier,
    events: &[CarrierEvent],
    contact_points: &[ContactVertex],
    policy: &CurveContext,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
    let source = match &carrier.geometry {
        CurveSupport2::Bezier(curve) => {
            BezierSelectedFiberSource2::Rational(RationalBezier2::try_from_subcurve(curve)?)
        }
        CurveSupport2::Parallel(parallel) => {
            BezierSelectedFiberSource2::AnalyticParallel(parallel.clone())
        }
        CurveSupport2::Line(_) | CurveSupport2::Circle(_) => {
            return Err(CurveError::Topology(
                "a selected-fiber boundary reached an incompatible carrier".into(),
            ));
        }
    };
    let support = match &source {
        BezierSelectedFiberSource2::Rational(curve) => {
            CurveSupport2::Bezier(BezierSubcurve2::Rational(curve.clone()))
        }
        BezierSelectedFiberSource2::AnalyticParallel(parallel) => {
            CurveSupport2::Parallel(parallel.clone())
        }
    };
    let mut boundaries = events.to_vec();
    for index in 1..boundaries.len() {
        let mut cursor = index;
        while cursor > 0 {
            let order = match boundaries[cursor]
                .parameter
                .cmp_by_refinement(&boundaries[cursor - 1].parameter, policy)?
            {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Err(CurveError::Topology(format!(
                        "selected-fiber split ordering remained uncertain: {reason:?}"
                    )));
                }
            };
            if order != Ordering::Less {
                break;
            }
            boundaries.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }

    let mut output = Vec::with_capacity(boundaries.len().saturating_sub(1));
    for pair in boundaries.windows(2) {
        match pair[0]
            .parameter
            .cmp_by_refinement(&pair[1].parameter, policy)?
        {
            Classification::Decided(Ordering::Less) => {}
            Classification::Decided(Ordering::Equal) => continue,
            Classification::Decided(Ordering::Greater) => {
                return Err(CurveError::Topology(
                    "selected-fiber split boundaries are not increasing".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "selected-fiber split interval remained uncertain: {reason:?}"
                )));
            }
        }
        let start_point =
            selected_fiber_event_point(&pair[0], carrier, &source, contact_points, policy)?;
        let end_point =
            selected_fiber_event_point(&pair[1], carrier, &source, contact_points, policy)?;
        output.push(SplitCarrierFragment {
            fragment: support.restrict_certified(
                CurveParameterRange2::new_validated(
                    pair[0].parameter.clone(),
                    pair[1].parameter.clone(),
                ),
                Some([start_point, end_point]),
                false,
                policy,
            )?,
            start_topology_vertex: pair[0].topology_vertex,
            end_topology_vertex: pair[1].topology_vertex,
        });
    }
    if carrier.reversed {
        output.reverse();
        for split in &mut output {
            split.fragment = split.fragment.reversed()?;
            std::mem::swap(
                &mut split.start_topology_vertex,
                &mut split.end_topology_vertex,
            );
        }
    }
    Ok(output)
}

fn split_algebraic_chord_carrier(
    carrier: &RegionCarrier,
    chord: &crate::BezierAlgebraicChord2,
    events: &[CarrierEvent],
    policy: &CurveContext,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
    chord.validate_policy(policy)?;
    // The common no-contact path already carries the two authenticated domain
    // endpoints. Preserve the original chord instead of reordering its
    // algebraic fields and reconstructing identical endpoint evidence.
    if events.len() == 2 {
        let start = events.iter().find(|event| event.parameter == carrier.start);
        let end = events.iter().find(|event| event.parameter == carrier.end);
        if let (Some(start), Some(end)) = (start, end) {
            let (fragment, start_topology_vertex, end_topology_vertex) = if carrier.reversed {
                (
                    BezierSplitFragment2::AlgebraicChord(chord.reversed()),
                    end.topology_vertex,
                    start.topology_vertex,
                )
            } else {
                (
                    BezierSplitFragment2::AlgebraicChord(chord.clone()),
                    start.topology_vertex,
                    end.topology_vertex,
                )
            };
            return Ok(vec![SplitCarrierFragment {
                fragment,
                start_topology_vertex,
                end_topology_vertex,
            }]);
        }
    }
    let mut boundaries = (0..events.len()).collect::<Vec<_>>();
    let mut comparisons = HashMap::new();
    let mut compare =
        |first_index: usize, second_index: usize| -> CurveResult<Classification<Ordering>> {
            if let Some(order) = comparisons.get(&(first_index, second_index)).copied() {
                return Ok(Classification::Decided(order));
            }
            let result = algebraic_chord_carrier_parameter_cmp(
                chord,
                &events[first_index].parameter,
                &events[second_index].parameter,
                policy,
            )?;
            if let Classification::Decided(order) = result {
                comparisons.insert((first_index, second_index), order);
                comparisons.insert((second_index, first_index), order.reverse());
            }
            Ok(result)
        };
    for index in 1..boundaries.len() {
        let mut cursor = index;
        while cursor > 0 {
            let order = match compare(boundaries[cursor], boundaries[cursor - 1])? {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Err(CurveError::Topology(format!(
                        "algebraic chord split ordering remained uncertain: {reason:?}"
                    )));
                }
            };
            if order != Ordering::Less {
                break;
            }
            boundaries.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }

    let mut output = Vec::with_capacity(boundaries.len().saturating_sub(1));
    for pair in boundaries.windows(2) {
        match compare(pair[0], pair[1])? {
            Classification::Decided(Ordering::Less) => {}
            Classification::Decided(Ordering::Equal) => continue,
            Classification::Decided(Ordering::Greater) => {
                return Err(CurveError::Topology(
                    "algebraic chord split boundaries are not increasing".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "algebraic chord split interval remained uncertain: {reason:?}"
                )));
            }
        }
        let Some(start) = events[pair[0]].parameter.as_algebraic_chord() else {
            return Err(CurveError::Topology(
                "non-chord cut reached an algebraic chord carrier".into(),
            ));
        };
        let Some(end) = events[pair[1]].parameter.as_algebraic_chord() else {
            return Err(CurveError::Topology(
                "non-chord cut reached an algebraic chord carrier".into(),
            ));
        };
        output.push(SplitCarrierFragment {
            fragment: carrier.geometry.restrict_certified(
                CurveParameterRange2::new_validated(
                    CurveParameter2::from_algebraic_chord(start.clone()),
                    CurveParameter2::from_algebraic_chord(end.clone()),
                ),
                None,
                false,
                policy,
            )?,
            start_topology_vertex: events[pair[0]].topology_vertex,
            end_topology_vertex: events[pair[1]].topology_vertex,
        });
    }
    if carrier.reversed {
        output.reverse();
        for split in &mut output {
            split.fragment = split.fragment.reversed()?;
            std::mem::swap(
                &mut split.start_topology_vertex,
                &mut split.end_topology_vertex,
            );
        }
    }
    Ok(output)
}

/// Orders every split event in the finite carrier's one certified monotone
/// coordinate. Contact kernels may construct equivalent parameters through
/// different retained chord domains; comparing those domains pairwise is not
/// a valid total order even when all represented points share this support.
fn algebraic_chord_carrier_parameter_cmp(
    chord: &crate::BezierAlgebraicChord2,
    first_parameter: &CurveParameter2,
    second_parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Ordering>> {
    let (Some(first), Some(second)) = (
        first_parameter.as_algebraic_chord(),
        second_parameter.as_algebraic_chord(),
    ) else {
        return Err(CurveError::Topology(
            "non-chord event reached an algebraic chord carrier".into(),
        ));
    };
    if first == second {
        return Ok(Classification::Decided(Ordering::Equal));
    }
    let certified_rank = |parameter: &BezierAlgebraicChordParameter2| {
        if parameter.is_endpoint_of(chord, true) {
            Some(0_u8)
        } else if parameter.is_certified_strict_interior_of(chord) {
            Some(1)
        } else if parameter.is_endpoint_of(chord, false) {
            Some(2)
        } else {
            None
        }
    };
    let first_rank = certified_rank(first);
    let second_rank = certified_rank(second);
    if let (Some(first_rank), Some(second_rank)) = (first_rank, second_rank)
        && (first_rank != second_rank || first_rank != 1)
    {
        // Endpoint identity and pair-kernel strict-containment certificates
        // already provide the carrier's exact local order. Reconstructing a
        // Cartesian comparison here can join otherwise independent endpoint
        // fields merely to rediscover start < interior < end.
        return Ok(Classification::Decided(first_rank.cmp(&second_rank)));
    }
    let (axis, increasing) = [Axis2::X, Axis2::Y]
        .into_iter()
        .find_map(|axis| match chord.certified_tangent_axis_sign(axis) {
            Some(RealSign::Positive) => Some((axis, true)),
            Some(RealSign::Negative) => Some((axis, false)),
            Some(RealSign::Zero) | None => None,
        })
        .ok_or_else(|| {
            CurveError::Topology("an algebraic chord lost its monotone parameter axis".into())
        })?;
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
        let first_bounds = crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(
            first.point(),
            refinement_steps,
            policy,
        );
        let second_bounds = crate::bezier_offset::algebraic_chord_endpoint_bounds_refined(
            second.point(),
            refinement_steps,
            policy,
        );
        let (Classification::Decided(first_bounds), Classification::Decided(second_bounds)) =
            (first_bounds, second_bounds)
        else {
            continue;
        };
        let (Some(first_bounds), Some(second_bounds)) = (
            first_bounds.certified_rational_outer_envelope(refinement_steps),
            second_bounds.certified_rational_outer_envelope(refinement_steps),
        ) else {
            continue;
        };
        let (first_lower, first_upper, second_lower, second_upper) = match axis {
            Axis2::X => (
                first_bounds.min().x(),
                first_bounds.max().x(),
                second_bounds.min().x(),
                second_bounds.max().x(),
            ),
            Axis2::Y => (
                first_bounds.min().y(),
                first_bounds.max().y(),
                second_bounds.min().y(),
                second_bounds.max().y(),
            ),
        };
        let coordinate_order = if compare_reals(first_upper, second_lower, &CurveContext::STRICT)
            == Some(Ordering::Less)
        {
            Some(Ordering::Less)
        } else if compare_reals(first_lower, second_upper, &CurveContext::STRICT)
            == Some(Ordering::Greater)
        {
            Some(Ordering::Greater)
        } else {
            None
        };
        if let Some(order) = coordinate_order {
            return Ok(Classification::Decided(if increasing {
                order
            } else {
                order.reverse()
            }));
        }
    }
    if matches!(
        (first.point(), second.point()),
        (
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)),
            _,
        ) | (
            _,
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)),
        )
    ) && let Classification::Decided(order) = first.cmp_by_refinement(second, policy)?
    {
        // Correlated chord-pair and selected-circle contacts retain support
        // side/branch certificates that are stronger than reconstructing a
        // common Cartesian projective coordinate.
        return Ok(Classification::Decided(order));
    }
    let order = crate::BezierAlgebraicChord2::point_axis_order(
        first.point(),
        second.point(),
        axis,
        policy,
    )?;
    Ok(if increasing {
        order
    } else {
        order.map(Ordering::reverse)
    })
}

fn split_algebraic_cusp_carrier(
    carrier: &RegionCarrier,
    fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    events: &[CarrierEvent],
    contact_points: &[ContactVertex],
    policy: &CurveContext,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
    let endpoint_indices = || {
        let start = events
            .iter()
            .position(|event| event.parameter == carrier.start)?;
        let end = events
            .iter()
            .position(|event| event.parameter == carrier.end)?;
        (start != end).then_some([start, end])
    };
    if events.len() == 2
        && let Some([start, end]) = endpoint_indices()
    {
        let [start_topology_vertex, end_topology_vertex] = if carrier.reversed {
            [events[end].topology_vertex, events[start].topology_vertex]
        } else {
            [events[start].topology_vertex, events[end].topology_vertex]
        };
        return Ok(vec![SplitCarrierFragment {
            fragment: BezierSplitFragment2::AlgebraicCuspSemicircle(fragment.clone()),
            start_topology_vertex,
            end_topology_vertex,
        }]);
    }

    let boundaries = if events.len() == 3
        && let Some([start, end]) = endpoint_indices()
        && let Some(interior) = (0..events.len()).find(|index| *index != start && *index != end)
        && let Some(vertex) = events[interior].topology_vertex
        && let Some(point) = contact_points
            .iter()
            .filter(|contact| contact.topology_vertex == vertex)
            .find_map(|contact| contact.point.as_ref())
        && fragment.certified_incident_point_evidence_is_strict_interior(point, policy)?
            == Classification::Decided(true)
    {
        // The pair kernel has already certified the only nonendpoint event
        // as strict interior. Its finite-domain order is therefore exactly
        // start, cut, end; rebuilding two independent angular comparisons
        // would discard that stronger incidence certificate.
        vec![
            events[start].clone(),
            events[interior].clone(),
            events[end].clone(),
        ]
    } else {
        let mut boundaries = events.to_vec();
        for index in 1..boundaries.len() {
            let mut cursor = index;
            while cursor > 0 {
                let order = match boundaries[cursor]
                    .parameter
                    .cmp_by_refinement(&boundaries[cursor - 1].parameter, policy)?
                {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        return Err(CurveError::Topology(format!(
                            "algebraic cusp split ordering remained uncertain: {reason:?}"
                        )));
                    }
                };
                if order != Ordering::Less {
                    break;
                }
                boundaries.swap(cursor, cursor - 1);
                cursor -= 1;
            }
        }
        boundaries
    };

    let mut output = Vec::with_capacity(boundaries.len().saturating_sub(1));
    for pair in boundaries.windows(2) {
        match pair[0]
            .parameter
            .cmp_by_refinement(&pair[1].parameter, policy)?
        {
            Classification::Decided(Ordering::Less) => {}
            Classification::Decided(Ordering::Equal) => continue,
            Classification::Decided(Ordering::Greater) => {
                return Err(CurveError::Topology(
                    "algebraic cusp split boundaries are not increasing".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "algebraic cusp split interval remained uncertain: {reason:?}"
                )));
            }
        }
        let Some(start) = pair[0].parameter.as_algebraic_cusp() else {
            return Err(CurveError::Topology(
                "Bezier cut reached an algebraic cusp carrier".into(),
            ));
        };
        let Some(end) = pair[1].parameter.as_algebraic_cusp() else {
            return Err(CurveError::Topology(
                "Bezier cut reached an algebraic cusp carrier".into(),
            ));
        };
        output.push(SplitCarrierFragment {
            fragment: carrier.geometry.restrict_certified(
                CurveParameterRange2::new_validated(
                    CurveParameter2::from_algebraic_cusp(start.clone()),
                    CurveParameter2::from_algebraic_cusp(end.clone()),
                ),
                None,
                false,
                policy,
            )?,
            start_topology_vertex: pair[0].topology_vertex,
            end_topology_vertex: pair[1].topology_vertex,
        });
    }
    if carrier.reversed {
        output.reverse();
        for split in &mut output {
            split.fragment = split.fragment.reversed()?;
            std::mem::swap(
                &mut split.start_topology_vertex,
                &mut split.end_topology_vertex,
            );
        }
    }
    Ok(output)
}

fn split_analytic_carrier(
    carrier: &RegionCarrier,
    events: &[CarrierEvent],
    max_refinement_steps: usize,
    policy: &CurveContext,
) -> Result<Vec<SplitCarrierFragment>, CurveError> {
    let mut boundaries = events
        .iter()
        .map(|event| {
            let parameter = event.parameter.as_bezier_parameter().ok_or_else(|| {
                CurveError::Topology("algebraic cusp cut reached an analytic carrier".into())
            })?;
            Ok(CarrierEvent {
                parameter: CurveParameter2::from(
                    parameter
                        .clone()
                        .refined_isolating_interval(max_refinement_steps, policy),
                ),
                topology_vertex: event.topology_vertex,
            })
        })
        .collect::<Result<Vec<_>, CurveError>>()?;
    for index in 1..boundaries.len() {
        let mut cursor = index;
        while cursor > 0 {
            let order = match boundaries[cursor]
                .parameter
                .cmp_by_refinement(&boundaries[cursor - 1].parameter, policy)?
            {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Err(CurveError::Topology(format!(
                        "analytic parallel split ordering remained uncertain: {reason:?}"
                    )));
                }
            };
            if order != Ordering::Less {
                break;
            }
            boundaries.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }

    let mut output = Vec::with_capacity(boundaries.len().saturating_sub(1));
    for pair in boundaries.windows(2) {
        let Some(start) = pair[0].parameter.as_bezier_parameter().cloned() else {
            return Err(CurveError::Topology(
                "algebraic cusp cut reached an analytic carrier".into(),
            ));
        };
        let Some(end) = pair[1].parameter.as_bezier_parameter().cloned() else {
            return Err(CurveError::Topology(
                "algebraic cusp cut reached an analytic carrier".into(),
            ));
        };
        match start.cmp_by_refinement(&end, policy)? {
            Classification::Decided(Ordering::Less) => {}
            Classification::Decided(Ordering::Equal) => continue,
            Classification::Decided(Ordering::Greater) => {
                return Err(CurveError::Topology(
                    "analytic parallel split boundaries are not increasing".into(),
                ));
            }
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "analytic parallel split interval remained uncertain: {reason:?}"
                )));
            }
        }
        output.push(SplitCarrierFragment {
            fragment: carrier.geometry.restrict_certified(
                CurveParameterRange2::new_validated(start.into(), end.into()),
                None,
                false,
                policy,
            )?,
            start_topology_vertex: pair[0].topology_vertex,
            end_topology_vertex: pair[1].topology_vertex,
        });
    }
    if carrier.reversed {
        output.reverse();
        for fragment in &mut output {
            fragment.fragment = fragment.fragment.reversed()?;
            std::mem::swap(
                &mut fragment.start_topology_vertex,
                &mut fragment.end_topology_vertex,
            );
        }
    }
    Ok(output)
}

fn compact_retained_circular_fragment(
    fragment: &BezierSplitFragment2,
    carrier: &RegionCarrier,
    start_topology_vertex: Option<usize>,
    end_topology_vertex: Option<usize>,
    contact_points: &[ContactVertex],
    exact_contact_point_index_by_vertex: &[usize],
    policy: &CurveContext,
) -> BezierSplitFragment2 {
    if let BezierSplitFragment2::Materialized {
        start,
        end,
        curve: BezierSubcurve2::Rational(curve),
    } = fragment
        && curve.retained_circular_conic().is_some()
        && let Some(curve) = retained_circular_quadratic(curve, policy)
    {
        return BezierSplitFragment2::Materialized {
            start: start.clone(),
            end: end.clone(),
            curve: BezierSubcurve2::RationalQuadratic(curve),
        };
    }
    let BezierSplitFragment2::RetainedBezier { start, end, .. } = fragment else {
        return fragment.clone();
    };
    let CurveSupport2::Bezier(carrier_curve) = &carrier.geometry else {
        return fragment.clone();
    };
    let Some((implicit_conic, circular_conic)) = retained_circular_support(carrier_curve) else {
        return fragment.clone();
    };
    // The native circular span may certify a minor arc while an exterior
    // restriction traverses its major complement. Endpoint coordinates alone
    // cannot choose that branch; keep the retained chart outside the native
    // domain instead of replacing it with an unrelated minor arc.
    if !matches!(
        CurveParameterDomain2::new(&CurveParameterRange2::unit(), None).contains_finite_range(
            &CurveParameterRange2::new_validated(start.clone().into(), end.clone().into()),
            &policy.strict_counterpart(),
        ),
        Ok(Classification::Decided(true))
    ) {
        return fragment.clone();
    }
    let Some(start_point) = exact_split_endpoint_point(
        start,
        start_topology_vertex,
        carrier,
        contact_points,
        exact_contact_point_index_by_vertex,
        policy,
    ) else {
        return fragment.clone();
    };
    let Some(end_point) = exact_split_endpoint_point(
        end,
        end_topology_vertex,
        carrier,
        contact_points,
        exact_contact_point_index_by_vertex,
        policy,
    ) else {
        return fragment.clone();
    };
    let endpoints = [start_point, end_point];
    let Ok(curve) =
        crate::arc_bezier::rational_minor_arc_span(implicit_conic, circular_conic, &endpoints)
    else {
        return fragment.clone();
    };
    BezierSplitFragment2::Materialized {
        start: start.clone(),
        end: end.clone(),
        curve: BezierSubcurve2::RationalQuadratic(curve),
    }
}

fn retained_circular_quadratic(
    curve: &RationalBezier2,
    policy: &CurveContext,
) -> Option<crate::RationalQuadraticBezier2> {
    let curve = match curve.materialized_quadratic_representative(policy).ok()? {
        Classification::Decided(Some(curve)) => curve,
        Classification::Decided(None) | Classification::Uncertain(_) => return None,
    };
    (curve.retained_implicit_quadratic_conic().is_some()
        && curve.retained_circular_conic().is_some())
    .then_some(curve)
}

fn adjacent_axis_algebraic_chord_circular_curve_is_endpoint_only(
    chord: &crate::BezierAlgebraicChord2,
    chord_carrier: &RegionCarrier,
    curve: &BezierSubcurve2,
    curve_carrier: &RegionCarrier,
    chord_precedes_curve: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    // This optional shortcut may decline to the complete contact kernel.
    // Its reconstructed scalar probes cannot consume terminal approximation.
    let policy = &policy.strict_counterpart();
    if !carrier_has_certified_injective_image(curve_carrier, policy) {
        return Ok(Classification::Decided(false));
    }
    let Some((_, circle)) = retained_circular_support(curve) else {
        return Ok(Classification::Decided(false));
    };
    if real_sign(&circle.radius_squared, policy) != Some(RealSign::Positive) {
        return Ok(Classification::Decided(false));
    }
    let chord_parameter = if chord_precedes_curve {
        carrier_traversal_end(chord_carrier)
    } else {
        carrier_traversal_start(chord_carrier)
    };
    let Some(chord_point) = chord_parameter
        .as_algebraic_chord()
        .and_then(|parameter| parameter.point().coordinates())
    else {
        return Ok(Classification::Decided(false));
    };
    let curve_parameter = if chord_precedes_curve {
        carrier_traversal_start(curve_carrier)
    } else {
        carrier_traversal_end(curve_carrier)
    };
    let Some(curve_point) = exact_carrier_point(curve_carrier, curve_parameter, policy) else {
        return Ok(Classification::Decided(false));
    };
    if real_sign(&chord_point.distance_squared(&curve_point), policy) != Some(RealSign::Zero)
        || real_sign(
            &(curve_point.distance_squared(&circle.center) - &circle.radius_squared),
            policy,
        ) != Some(RealSign::Zero)
    {
        return Ok(Classification::Decided(false));
    }
    let axis = match chord.axis_direction(policy)? {
        Classification::Decided(Some(direction)) => direction.axis(),
        Classification::Decided(None) | Classification::Uncertain(_) => {
            return Ok(Classification::Decided(false));
        }
    };
    let tangent_residual = match axis {
        Axis2::X => curve_point.x() - circle.center.x(),
        Axis2::Y => curve_point.y() - circle.center.y(),
    };
    Ok(Classification::Decided(
        real_sign(&tangent_residual, policy) == Some(RealSign::Zero),
    ))
}

fn exact_split_endpoint_point(
    parameter: &BezierParameter2,
    topology_vertex: Option<usize>,
    carrier: &RegionCarrier,
    contact_points: &[ContactVertex],
    exact_contact_point_index_by_vertex: &[usize],
    policy: &CurveContext,
) -> Option<crate::Point2> {
    if let Some(contact_index) = topology_vertex
        .and_then(|vertex| exact_contact_point_index_by_vertex.get(vertex))
        .copied()
        .filter(|index| *index != usize::MAX)
        && let Some(CurvePoint2(CurvePointData2::Exact(point))) =
            &contact_points.get(contact_index)?.point
    {
        return Some(point.clone());
    }
    let parameter = parameter.scalar()?;
    match carrier.geometry.point_at(parameter, policy).ok()? {
        Classification::Decided(point) => Some(point),
        Classification::Uncertain(_) => None,
    }
}

fn certified_boolean_successors(
    graph: &BezierArrangementGraph2,
    directions: &[BooleanArrangementFragmentDirection],
    topology: &CurveRegionBooleanTopology,
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> Vec<Option<usize>> {
    let starts_by_vertex = arrangement_starts_by_vertex(graph, None);
    let mut successors = certified_transverse_successors(
        graph,
        directions,
        &topology.transverse_contacts,
        &starts_by_vertex,
        false,
        |contact, vertex| transverse_carrier_cross_is_positive(topology, contact, vertex, carriers),
    );
    certify_nontransverse_authored_continuity(
        &mut successors,
        graph,
        directions,
        topology,
        carriers,
        &starts_by_vertex,
    );
    // A crossing at an authored subdivision can involve more than the two
    // carriers named by one contact certificate. Reuse the exact retained
    // tangent authority before asking traversal to reconstruct endpoint
    // derivatives or a derived Boolean to rebuild this same arrangement.
    let mut ends_by_vertex = HashMap::<usize, Vec<usize>>::new();
    for (index, fragment) in graph.fragments().iter().enumerate() {
        if let Some(vertex) = fragment.end_topology_vertex()
            && starts_by_vertex
                .get(&vertex)
                .is_some_and(|outgoing| outgoing.len() > 1)
        {
            ends_by_vertex.entry(vertex).or_default().push(index);
        }
    }
    for (vertex, incoming) in ends_by_vertex {
        let Some(outgoing) = starts_by_vertex.get(&vertex) else {
            continue;
        };
        if incoming.iter().any(|&edge| successors[edge].is_none()) {
            policy.strict_predicate_pass(|| {
                certify_curve_tangent_successors(
                    &mut successors,
                    &incoming,
                    outgoing,
                    graph,
                    policy,
                )
            });
        }
    }
    successors
}

/// Certifies the original-loop successor at a nontransverse contact.
///
/// When both adjacent retained pieces have the same decided non-boundary
/// location relative to the other operand, their source boundary does not
/// cross that operand at the shared vertex. If Boolean classification retained the
/// two pieces with the same traversal orientation, continuing along the
/// authored loop is therefore an exact face continuation. This resolves an
/// external point touch without asking tangent order to distinguish coincident
/// first- and higher-order jets.
fn certify_nontransverse_authored_continuity(
    successors: &mut [Option<usize>],
    graph: &BezierArrangementGraph2,
    directions: &[BooleanArrangementFragmentDirection],
    topology: &CurveRegionBooleanTopology,
    carriers: &[RegionCarrier],
    starts_by_vertex: &HashMap<usize, Vec<usize>>,
) {
    for (current_index, current) in graph.fragments().iter().enumerate() {
        if successors
            .get(current_index)
            .is_none_or(|successor| successor.is_some())
        {
            continue;
        }
        let Some(candidates) = current
            .end_topology_vertex()
            .and_then(|vertex| starts_by_vertex.get(&vertex))
            .filter(|candidates| candidates.len() > 1)
        else {
            continue;
        };
        let mut certified = candidates.iter().copied().filter(|&candidate_index| {
            authored_boolean_fragments_are_continuous(
                current_index,
                candidate_index,
                graph,
                directions,
                topology,
                carriers,
            )
        });
        let Some(candidate_index) = certified.next() else {
            continue;
        };
        if certified.next().is_some() {
            continue;
        }
        successors[current_index] = Some(candidate_index);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "boolean-successor",
            "nontransverse-authored-continuity",
        );
    }
}

fn authored_boolean_fragments_are_continuous(
    current_index: usize,
    candidate_index: usize,
    graph: &BezierArrangementGraph2,
    directions: &[BooleanArrangementFragmentDirection],
    topology: &CurveRegionBooleanTopology,
    carriers: &[RegionCarrier],
) -> bool {
    let (Some(current), Some(candidate), Some(current_direction), Some(candidate_direction)) = (
        graph.fragments().get(current_index),
        graph.fragments().get(candidate_index),
        directions.get(current_index),
        directions.get(candidate_index),
    ) else {
        return false;
    };
    if current_direction.follows_carrier != candidate_direction.follows_carrier {
        return false;
    }
    let (current_carrier_index, candidate_carrier_index) =
        (current.source_curve_index(), candidate.source_curve_index());
    let (Some(current_carrier), Some(candidate_carrier)) = (
        carriers.get(current_carrier_index),
        carriers.get(candidate_carrier_index),
    ) else {
        return false;
    };
    if current_carrier.operand != candidate_carrier.operand
        || current_carrier.loop_index != candidate_carrier.loop_index
    {
        return false;
    }
    let current_location = topology
        .split_fragments
        .get(current_carrier_index)
        .and_then(|fragments| fragments.get(current.source_fragment_index()))
        .and_then(|fragment| fragment.location);
    let candidate_location = topology
        .split_fragments
        .get(candidate_carrier_index)
        .and_then(|fragments| fragments.get(candidate.source_fragment_index()))
        .and_then(|fragment| fragment.location);
    if current_location != candidate_location
        || !matches!(
            current_location,
            Some(RegionPointLocation::Inside | RegionPointLocation::Outside)
        )
    {
        return false;
    }
    if current_direction.follows_carrier {
        authored_split_fragment_is_successor(current, candidate, topology, carriers)
    } else {
        authored_split_fragment_is_successor(candidate, current, topology, carriers)
    }
}

fn authored_split_fragment_is_successor(
    current: &BezierArrangementFragment2,
    candidate: &BezierArrangementFragment2,
    topology: &CurveRegionBooleanTopology,
    carriers: &[RegionCarrier],
) -> bool {
    let (current_carrier_index, candidate_carrier_index) =
        (current.source_curve_index(), candidate.source_curve_index());
    let current_split_index = current.source_fragment_index();
    let candidate_split_index = candidate.source_fragment_index();
    let Some(current_split_count) = topology
        .split_fragments
        .get(current_carrier_index)
        .map(Vec::len)
    else {
        return false;
    };
    authored_split_source_is_successor(
        current_carrier_index,
        current_split_index,
        current_split_count,
        candidate_carrier_index,
        candidate_split_index,
        carriers,
    )
}

fn authored_split_source_is_successor(
    current_carrier_index: usize,
    current_split_index: usize,
    current_split_count: usize,
    candidate_carrier_index: usize,
    candidate_split_index: usize,
    carriers: &[RegionCarrier],
) -> bool {
    if let Some(next_split_index) = current_split_index
        .checked_add(1)
        .filter(|&index| index < current_split_count)
    {
        return current_carrier_index == candidate_carrier_index
            && candidate_split_index == next_split_index;
    }
    if candidate_split_index != 0 {
        return false;
    }
    let (Some(current_carrier), Some(candidate_carrier)) = (
        carriers.get(current_carrier_index),
        carriers.get(candidate_carrier_index),
    ) else {
        return false;
    };
    if current_carrier.operand != candidate_carrier.operand
        || current_carrier.loop_index != candidate_carrier.loop_index
    {
        return false;
    }
    if current_carrier.fragment_index.checked_add(1) == Some(candidate_carrier.fragment_index) {
        return true;
    }
    candidate_carrier.fragment_index == 0
        && current_carrier_index
            .checked_add(1)
            .and_then(|index| carriers.get(index))
            .is_none_or(|next| {
                next.operand != current_carrier.operand
                    || next.loop_index != current_carrier.loop_index
            })
}

fn certified_regularization_successors(
    graph: &BezierArrangementGraph2,
    directions: &[BooleanArrangementFragmentDirection],
    face_sector_successors: &[Option<usize>],
    topology: &CurveRegionSplitTopology,
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> Vec<Option<usize>> {
    let starts_by_vertex = arrangement_starts_by_vertex(graph, None);
    let transverse_successors = certified_transverse_successors(
        graph,
        directions,
        &topology.transverse_contacts,
        &starts_by_vertex,
        true,
        |contact, _| {
            Some(
                contact.cross_is_positive?
                    ^ carriers.get(contact.first_carrier)?.reversed
                    ^ carriers.get(contact.second_carrier)?.reversed,
            )
        },
    );
    let mut authored_successors = vec![None; graph.len()];
    certify_nontransverse_regularization_authored_continuity(
        &mut authored_successors,
        graph,
        directions,
        topology,
        carriers,
        &starts_by_vertex,
    );
    let mut ends_by_vertex = HashMap::<usize, Vec<usize>>::new();
    for (index, fragment) in graph.fragments().iter().enumerate() {
        if let Some(vertex) = fragment.end_topology_vertex() {
            ends_by_vertex.entry(vertex).or_default().push(index);
        }
    }
    let mut successors = vec![None; graph.len()];
    for (vertex, incoming) in ends_by_vertex {
        let Some(outgoing) = starts_by_vertex.get(&vertex) else {
            continue;
        };
        certify_regularization_vertex_successors(
            &mut successors,
            &incoming,
            outgoing,
            face_sector_successors,
            &transverse_successors,
            &authored_successors,
        );
        if incoming.iter().any(|&edge| successors[edge].is_none()) && outgoing.len() > 1 {
            policy.strict_predicate_pass(|| {
                certify_curve_tangent_successors(
                    &mut successors,
                    &incoming,
                    outgoing,
                    graph,
                    policy,
                )
            });
        }
    }
    successors
}

/// Replays retained curve directions only where incidence and face-sector
/// certificates leave a branch unresolved. The common tangent authority also
/// serves offset joins, so selected curves need no Cartesian tangent image.
fn certify_curve_tangent_successors(
    successors: &mut [Option<usize>],
    incoming: &[usize],
    outgoing: &[usize],
    graph: &BezierArrangementGraph2,
    policy: &CurveContext,
) {
    use crate::bezier_region::CurveTangent2;
    if incoming.len() != outgoing.len() {
        return;
    }
    let tangent = |edge: usize, at_start| match CurveTangent2::at_boundary_endpoint(
        graph.fragments()[edge].fragment(),
        at_start,
        policy,
    ) {
        Ok(Classification::Decided(tangent)) => Some(tangent),
        Ok(Classification::Uncertain(_)) | Err(_) => None,
    };
    let Some(outgoing_tangents) = outgoing
        .iter()
        .map(|&edge| tangent(edge, true))
        .collect::<Option<Vec<_>>>()
    else {
        return;
    };
    let proposed = incoming
        .iter()
        .copied()
        .filter_map(|edge| {
            if successors[edge].is_some() {
                return None;
            }
            let base = tangent(edge, false)?;
            let mut best = 0;
            for candidate in 1..outgoing.len() {
                match base.compare_filled_left_turn(
                    &outgoing_tangents[candidate],
                    &outgoing_tangents[best],
                    policy,
                ) {
                    Classification::Decided(Ordering::Less) => best = candidate,
                    Classification::Decided(Ordering::Greater) => {}
                    Classification::Decided(Ordering::Equal) | Classification::Uncertain(_) => {
                        return None;
                    }
                }
            }
            Some((edge, outgoing[best]))
        })
        .collect::<Vec<_>>();
    for &(edge, target) in &proposed {
        if proposed
            .iter()
            .filter(|(_, candidate)| *candidate == target)
            .count()
            == 1
            && !incoming
                .iter()
                .any(|&other| successors[other] == Some(target))
        {
            successors[edge] = Some(target);
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "regularization-successor",
                "retained-curve-tangent",
            );
        }
    }
}

/// Combines two exact but independently conservative successor sources.
///
/// Local face-sector links take precedence, followed by transverse-contact
/// tangent order. Only unique target claims are published. If those exact
/// claims leave one incoming and one outgoing edge at the same topology
/// vertex, bijectivity of the oriented regularized boundary forces the final
/// pair without requiring either retained tangent to be materialized.
fn certify_regularization_vertex_successors(
    successors: &mut [Option<usize>],
    incoming: &[usize],
    outgoing: &[usize],
    face_sector_successors: &[Option<usize>],
    transverse_successors: &[Option<usize>],
    authored_successors: &[Option<usize>],
) {
    if incoming.len() != outgoing.len() {
        return;
    }
    let mut claimed = Vec::with_capacity(outgoing.len());
    for source in [
        face_sector_successors,
        transverse_successors,
        authored_successors,
    ] {
        let candidates = incoming
            .iter()
            .copied()
            .filter(|&edge| successors.get(edge).is_some_and(Option::is_none))
            .filter_map(|edge| {
                source
                    .get(edge)
                    .copied()
                    .flatten()
                    .filter(|target| outgoing.contains(target) && !claimed.contains(target))
                    .map(|target| (edge, target))
            })
            .collect::<Vec<_>>();
        for &(edge, target) in &candidates {
            if candidates
                .iter()
                .filter(|(_, candidate)| *candidate == target)
                .count()
                == 1
            {
                successors[edge] = Some(target);
                claimed.push(target);
            }
        }
    }
    let unmatched_incoming = incoming
        .iter()
        .copied()
        .filter(|&edge| successors.get(edge).is_some_and(Option::is_none))
        .collect::<Vec<_>>();
    let unmatched_outgoing = outgoing
        .iter()
        .copied()
        .filter(|target| !claimed.contains(target))
        .collect::<Vec<_>>();
    if let ([incoming], [outgoing]) = (unmatched_incoming.as_slice(), unmatched_outgoing.as_slice())
    {
        successors[*incoming] = Some(*outgoing);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "regularization-successor",
            "forced-bijection",
        );
    }
}

#[cfg(test)]
mod regularization_successor_tests {
    use super::certify_regularization_vertex_successors;

    #[test]
    fn face_sector_claim_forces_the_unmaterialized_complement() {
        let mut successors = vec![None; 4];
        certify_regularization_vertex_successors(
            &mut successors,
            &[0, 1],
            &[2, 3],
            &[Some(2), None, None, None],
            &[Some(2), Some(2), None, None],
            &[None; 4],
        );
        assert_eq!(successors, vec![Some(2), Some(3), None, None]);
    }

    #[test]
    fn colliding_contact_claims_remain_unselected() {
        let mut successors = vec![None; 4];
        certify_regularization_vertex_successors(
            &mut successors,
            &[0, 1],
            &[2, 3],
            &[None; 4],
            &[Some(2), Some(2), None, None],
            &[None; 4],
        );
        assert_eq!(successors, vec![None; 4]);
    }
}

/// Certifies a retained authored walk through a nontransverse unary contact.
///
/// Regularization has already discarded edges whose two sides have equal
/// fill and oriented every retained edge with the filled result face on its
/// left. At a vertex with no transverse crossing, two retained pieces with
/// the same orientation that are consecutive in the authored loop therefore
/// carry the same exact result face through the contact. This is the unary
/// counterpart of `certify_nontransverse_authored_continuity` for Booleans.
fn certify_nontransverse_regularization_authored_continuity(
    successors: &mut [Option<usize>],
    graph: &BezierArrangementGraph2,
    directions: &[BooleanArrangementFragmentDirection],
    topology: &CurveRegionSplitTopology,
    carriers: &[RegionCarrier],
    starts_by_vertex: &HashMap<usize, Vec<usize>>,
) {
    for (current_index, current) in graph.fragments().iter().enumerate() {
        if successors
            .get(current_index)
            .is_none_or(|successor| successor.is_some())
        {
            continue;
        }
        let Some(vertex) = current.end_topology_vertex() else {
            continue;
        };
        if topology
            .transverse_vertices
            .get(vertex)
            .copied()
            .unwrap_or(false)
        {
            continue;
        }
        let Some(candidates) = starts_by_vertex
            .get(&vertex)
            .filter(|candidates| candidates.len() > 1)
        else {
            continue;
        };
        let Some(current_direction) = directions.get(current_index) else {
            continue;
        };
        let mut certified = candidates.iter().copied().filter(|&candidate_index| {
            let Some(candidate) = graph.fragments().get(candidate_index) else {
                return false;
            };
            let Some(candidate_direction) = directions.get(candidate_index) else {
                return false;
            };
            if current_direction.follows_carrier != candidate_direction.follows_carrier {
                return false;
            }
            let (source_current, source_candidate) = if current_direction.follows_carrier {
                (current, candidate)
            } else {
                (candidate, current)
            };
            let current_carrier_index = source_current.source_curve_index();
            let Some(current_split_count) = topology
                .split_fragments
                .get(current_carrier_index)
                .map(Vec::len)
            else {
                return false;
            };
            authored_split_source_is_successor(
                current_carrier_index,
                source_current.source_fragment_index(),
                current_split_count,
                source_candidate.source_curve_index(),
                source_candidate.source_fragment_index(),
                carriers,
            )
        });
        let Some(candidate_index) = certified.next() else {
            continue;
        };
        if certified.next().is_some() {
            continue;
        }
        successors[current_index] = Some(candidate_index);
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "regularization-successor",
            "nontransverse-authored-continuity",
        );
    }
}

fn arrangement_starts_by_vertex(
    graph: &BezierArrangementGraph2,
    selected_vertices: Option<&HashMap<usize, TransitionContactCandidate>>,
) -> HashMap<usize, Vec<usize>> {
    let mut starts_by_vertex = HashMap::<usize, Vec<usize>>::new();
    for (fragment_index, fragment) in graph.fragments().iter().enumerate() {
        if let Some(vertex) = fragment.start_topology_vertex()
            && selected_vertices.is_none_or(|vertices| vertices.contains_key(&vertex))
        {
            starts_by_vertex
                .entry(vertex)
                .or_default()
                .push(fragment_index);
        }
    }
    starts_by_vertex
}

fn certified_transverse_successors(
    graph: &BezierArrangementGraph2,
    directions: &[BooleanArrangementFragmentDirection],
    contacts: &HashMap<usize, TransitionContactCandidate>,
    starts_by_vertex: &HashMap<usize, Vec<usize>>,
    filled_left_faces: bool,
    mut crossing_is_positive: impl FnMut(&TransitionContactCandidate, usize) -> Option<bool>,
) -> Vec<Option<usize>> {
    graph
        .fragments()
        .iter()
        .enumerate()
        .map(|(current_index, current)| {
            let vertex = current.end_topology_vertex()?;
            let contact = contacts.get(&vertex)?;
            let crossing_is_positive = crossing_is_positive(contact, vertex)?;
            let retain_current = contact.first_carrier == contact.second_carrier;
            let mut candidates = starts_by_vertex
                .get(&vertex)?
                .iter()
                .copied()
                .filter(|candidate_index| retain_current || *candidate_index != current_index);
            let first_index = candidates.next()?;
            let second_index = candidates.next()?;
            if candidates.next().is_some() {
                return None;
            }
            let current =
                certified_contact_direction(*directions.get(current_index)?, false, contact)?;
            let first = certified_contact_direction(*directions.get(first_index)?, true, contact)?;
            let second =
                certified_contact_direction(*directions.get(second_index)?, true, contact)?;
            certified_turn_preference(
                current,
                first,
                second,
                crossing_is_positive,
                filled_left_faces,
            )
            .map(|first_before_second| {
                if first_before_second {
                    first_index
                } else {
                    second_index
                }
            })
        })
        .collect()
}

fn certified_contact_direction(
    direction: BooleanArrangementFragmentDirection,
    at_start: bool,
    contact: &TransitionContactCandidate,
) -> Option<CertifiedContactDirection> {
    let branch = if contact.first_carrier == contact.second_carrier {
        if at_start {
            direction.start_contact_branch
        } else {
            direction.end_contact_branch
        }?
    } else if direction.carrier_index == contact.first_carrier {
        TransitionContactBranch::First
    } else if direction.carrier_index == contact.second_carrier {
        TransitionContactBranch::Second
    } else {
        return None;
    };
    Some(CertifiedContactDirection {
        branch,
        follows_carrier: direction.follows_carrier,
    })
}

fn transverse_carrier_cross_is_positive(
    topology: &CurveRegionBooleanTopology,
    contact: &TransitionContactCandidate,
    vertex: usize,
    carriers: &[RegionCarrier],
) -> Option<bool> {
    let fragments = topology.split_fragments.get(contact.second_carrier)?;
    let before = fragments
        .iter()
        .find(|fragment| fragment.split.end_topology_vertex == Some(vertex))?
        .location?;
    let after = fragments
        .iter()
        .find(|fragment| fragment.split.start_topology_vertex == Some(vertex))?
        .location?;
    // For a regular crossing, whether the second oriented carrier enters the
    // first region determines the sign of cross(first tangent, second
    // tangent), after accounting for which side of the first carrier is
    // filled. This reuses the exact region classifications already retained
    // by topology construction.
    transverse_cross_from_locations(
        before,
        after,
        carriers.get(contact.first_carrier)?.filled_side_is_left,
    )
}

const fn transverse_cross_from_locations(
    before: RegionPointLocation,
    after: RegionPointLocation,
    first_filled_side_is_left: bool,
) -> Option<bool> {
    let enters_first_interior = match (before, after) {
        (RegionPointLocation::Outside, RegionPointLocation::Inside) => true,
        (RegionPointLocation::Inside, RegionPointLocation::Outside) => false,
        _ => return None,
    };
    Some(enters_first_interior == first_filled_side_is_left)
}

fn certified_turn_preference(
    base: CertifiedContactDirection,
    first: CertifiedContactDirection,
    second: CertifiedContactDirection,
    crossing_is_positive: bool,
    filled_left_faces: bool,
) -> Option<bool> {
    let first_half = certified_turn_half(base, first, crossing_is_positive)?;
    let second_half = certified_turn_half(base, second, crossing_is_positive)?;
    if first_half != second_half {
        return Some(first_half < second_half);
    }
    match certified_direction_cross(first, second, crossing_is_positive)? {
        1 => Some(!filled_left_faces),
        -1 => Some(filled_left_faces),
        _ => None,
    }
}

fn certified_turn_half(
    base: CertifiedContactDirection,
    candidate: CertifiedContactDirection,
    crossing_is_positive: bool,
) -> Option<u8> {
    if base.branch == candidate.branch {
        return Some(u8::from(base.follows_carrier != candidate.follows_carrier));
    }
    Some(
        if certified_direction_cross(base, candidate, crossing_is_positive)? > 0 {
            0
        } else {
            1
        },
    )
}

fn certified_direction_cross(
    first: CertifiedContactDirection,
    second: CertifiedContactDirection,
    crossing_is_positive: bool,
) -> Option<i8> {
    if first.branch == second.branch {
        return Some(0);
    }
    let source_cross = if first.branch == TransitionContactBranch::First
        && second.branch == TransitionContactBranch::Second
    {
        if crossing_is_positive { 1 } else { -1 }
    } else if first.branch == TransitionContactBranch::Second
        && second.branch == TransitionContactBranch::First
    {
        if crossing_is_positive { -1 } else { 1 }
    } else {
        return None;
    };
    let first_orientation = if first.follows_carrier { 1 } else { -1 };
    let second_orientation = if second.follows_carrier { 1 } else { -1 };
    Some(source_cross * first_orientation * second_orientation)
}

fn certified_transverse_contact_vertices(
    split_fragments: &[Vec<SplitCarrierFragment>],
    candidates: &mut [Option<TransitionContactCandidate>],
    policy: &CurveContext,
) -> Vec<bool> {
    candidates
        .iter_mut()
        .enumerate()
        .map(|(vertex, candidate)| {
            let Some(candidate) = candidate else {
                return false;
            };
            if candidate.cross_is_positive.is_some() {
                return true;
            }
            let Some(first) = algebraic_endpoint_tangent_at_vertex(
                &split_fragments[candidate.first_carrier],
                vertex,
            ) else {
                return candidate.certified_transverse;
            };
            let Some(second) = algebraic_endpoint_tangent_at_vertex(
                &split_fragments[candidate.second_carrier],
                vertex,
            ) else {
                return candidate.certified_transverse;
            };
            let cross = algebraic_endpoint_tangent_cross_sign(first, second, policy);
            match cross {
                Classification::Decided(RealSign::Positive) => {
                    candidate.cross_is_positive = Some(true);
                    true
                }
                Classification::Decided(RealSign::Negative) => {
                    candidate.cross_is_positive = Some(false);
                    true
                }
                Classification::Decided(RealSign::Zero) => false,
                Classification::Uncertain(_) => candidate.certified_transverse,
            }
        })
        .collect()
}

const fn toggled_region_location(location: RegionPointLocation) -> Option<RegionPointLocation> {
    match location {
        RegionPointLocation::Inside => Some(RegionPointLocation::Outside),
        RegionPointLocation::Outside => Some(RegionPointLocation::Inside),
        RegionPointLocation::Boundary => None,
    }
}

const fn boolean_location(inside: bool) -> RegionPointLocation {
    if inside {
        RegionPointLocation::Inside
    } else {
        RegionPointLocation::Outside
    }
}

fn seed_transverse_carrier_locations(
    fragments: &mut [Vec<ClassifiedSplitCarrierFragment>],
    carrier_index: usize,
    vertex: usize,
    before_inside: bool,
) -> bool {
    let Some(carrier_fragments) = fragments.get_mut(carrier_index) else {
        return false;
    };
    let before = carrier_fragments
        .iter()
        .position(|fragment| fragment.split.end_topology_vertex == Some(vertex));
    let after = carrier_fragments
        .iter()
        .position(|fragment| fragment.split.start_topology_vertex == Some(vertex));
    let (Some(before), Some(after)) = (before, after) else {
        return false;
    };
    for (fragment_index, location) in [
        (before, boolean_location(before_inside)),
        (after, boolean_location(!before_inside)),
    ] {
        match carrier_fragments[fragment_index].location {
            // Coincident-range ownership is authoritative for an overlap
            // fragment adjacent to a transverse overlap endpoint.  Seed only
            // the noncoincident open side; the overlap itself remains a
            // boundary fragment and is resolved by operation-side semantics.
            Some(RegionPointLocation::Boundary) => {}
            Some(existing) if existing != location => {
                return false;
            }
            Some(_) => {}
            None => carrier_fragments[fragment_index].location = Some(location),
        }
    }
    true
}

fn propagated_boolean_location(
    location: RegionPointLocation,
    before: &ClassifiedSplitCarrierFragment,
    after: &ClassifiedSplitCarrierFragment,
    transverse_vertices: &[bool],
    reclassification_vertices: &[bool],
) -> Option<RegionPointLocation> {
    let vertex = before.split.end_topology_vertex?;
    (after.split.start_topology_vertex == Some(vertex)).then_some(())?;
    if transverse_vertices.get(vertex).copied().unwrap_or(false) {
        toggled_region_location(location)
    } else if !reclassification_vertices
        .get(vertex)
        .copied()
        .unwrap_or(false)
    {
        Some(location)
    } else {
        None
    }
}

fn adjacent_boolean_loop_fragment(
    fragments: &[Vec<ClassifiedSplitCarrierFragment>],
    carrier_range: &std::ops::Range<usize>,
    current: (usize, usize),
    forward: bool,
) -> Option<(usize, usize)> {
    if forward {
        if current.1 + 1 < fragments[current.0].len() {
            return Some((current.0, current.1 + 1));
        }
        return (current.0 + 1..carrier_range.end)
            .chain(carrier_range.start..=current.0)
            .find_map(|carrier_index| {
                (!fragments[carrier_index].is_empty()).then_some((carrier_index, 0))
            });
    }
    if let Some(split_index) = current.1.checked_sub(1) {
        return Some((current.0, split_index));
    }
    (carrier_range.start..current.0)
        .rev()
        .chain((current.0..carrier_range.end).rev())
        .find_map(|carrier_index| {
            fragments[carrier_index]
                .len()
                .checked_sub(1)
                .map(|split_index| (carrier_index, split_index))
        })
}

fn propagate_boolean_locations_from_seed(
    fragments: &mut [Vec<ClassifiedSplitCarrierFragment>],
    carrier_range: std::ops::Range<usize>,
    seed: (usize, usize),
    transverse_vertices: &[bool],
    reclassification_vertices: &[bool],
) -> bool {
    for forward in [true, false] {
        let mut current = seed;
        while let Some(adjacent) =
            adjacent_boolean_loop_fragment(fragments, &carrier_range, current, forward)
        {
            let (before, after) = if forward {
                (current, adjacent)
            } else {
                (adjacent, current)
            };
            let Some(location) = fragments[current.0][current.1]
                .location
                .and_then(|location| {
                    propagated_boolean_location(
                        location,
                        &fragments[before.0][before.1],
                        &fragments[after.0][after.1],
                        transverse_vertices,
                        reclassification_vertices,
                    )
                })
            else {
                break;
            };
            if let Some(existing) = fragments[adjacent.0][adjacent.1].location {
                if existing != location {
                    return false;
                }
                break;
            }
            fragments[adjacent.0][adjacent.1].location = Some(location);
            current = adjacent;
        }
    }
    true
}

fn algebraic_endpoint_tangent_at_vertex(
    fragments: &[SplitCarrierFragment],
    vertex: usize,
) -> Option<&RationalBezierAlgebraicTangentImage2> {
    fragments.iter().find_map(|split| {
        let BezierSplitFragment2::RetainedBezier {
            reversed,
            start_image,
            end_image,
            ..
        } = &split.fragment
        else {
            return None;
        };
        if split.start_topology_vertex == Some(vertex) {
            return if *reversed { end_image } else { start_image }
                .as_ref()
                .and_then(|image| match image.tangent() {
                    Ok(Classification::Decided(tangent)) => Some(tangent),
                    _ => None,
                });
        }
        if split.end_topology_vertex == Some(vertex) {
            return if *reversed { start_image } else { end_image }
                .as_ref()
                .and_then(|image| match image.tangent() {
                    Ok(Classification::Decided(tangent)) => Some(tangent),
                    _ => None,
                });
        }
        None
    })
}

#[cfg(test)]
fn push_carrier_event(
    events: &mut Vec<CarrierEvent>,
    parameter: CurveParameter2,
    topology_vertex: Option<usize>,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    push_carrier_event_internal(events, parameter, topology_vertex, carrier, false, policy)
        .map(|_| ())
}

fn push_canonical_carrier_event(
    events: &mut Vec<CarrierEvent>,
    parameter: CurveParameter2,
    topology_vertex: Option<usize>,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<CurveParameter2> {
    let (_, event_index) =
        push_carrier_event_internal(events, parameter, topology_vertex, carrier, false, policy)?;
    Ok(events[event_index].parameter.clone())
}

fn push_contact_carrier_event(
    events: &mut Vec<CarrierEvent>,
    parameter: CurveParameter2,
    topology_vertex: Option<usize>,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    push_carrier_event_internal(events, parameter, topology_vertex, carrier, true, policy)
        .map(|(deferred, _)| deferred)
}

fn push_carrier_event_internal(
    events: &mut Vec<CarrierEvent>,
    parameter: CurveParameter2,
    topology_vertex: Option<usize>,
    carrier: &RegionCarrier,
    defer_unordered: bool,
    policy: &CurveContext,
) -> ExactCurveResult<(bool, usize)> {
    let mut deferred_ordering = false;
    for (event_index, event) in events.iter_mut().enumerate() {
        let same_topology_vertex =
            topology_vertex.is_some() && event.topology_vertex == topology_vertex;
        // A single-valued injective carrier cannot assign two parameters to
        // one exact topology vertex. Contact reconciliation has already
        // proved the vertex identity geometrically, so it is both stronger
        // and cheaper than rejoining two independently retained scalar
        // fields merely to rediscover parameter equality.
        if same_topology_vertex && carrier_has_certified_injective_image(carrier, policy) {
            return Ok((deferred_ordering, event_index));
        }
        let comparison = if defer_unordered {
            locally_decidable_contact_parameter_cmp(&parameter, &event.parameter, policy)
        } else {
            parameter.cmp_by_refinement(&event.parameter, policy)
        };
        match comparison.map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Boolean, carrier.family, cause)
        })? {
            Classification::Decided(Ordering::Equal) => {
                if event.topology_vertex.is_none() {
                    event.topology_vertex = topology_vertex;
                }
                return Ok((deferred_ordering, event_index));
            }
            Classification::Decided(_) => {}
            Classification::Uncertain(_) if defer_unordered => deferred_ordering = true,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Boolean,
                    carrier.family,
                    reason,
                ));
            }
        }
    }
    events.push(CarrierEvent {
        parameter,
        topology_vertex,
    });
    Ok((deferred_ordering, events.len() - 1))
}

fn seed_loop_topology_vertices(
    carriers: &[RegionCarrier],
    events: &mut [Vec<CarrierEvent>],
    next_topology_vertex: &mut usize,
) {
    let mut loop_start = 0_usize;
    while loop_start < carriers.len() {
        let operand = carriers[loop_start].operand;
        let loop_index = carriers[loop_start].loop_index;
        let mut loop_end = loop_start + 1;
        while loop_end < carriers.len()
            && carriers[loop_end].operand == operand
            && carriers[loop_end].loop_index == loop_index
        {
            loop_end += 1;
        }
        for current_index in loop_start..loop_end {
            let next_index = if current_index + 1 == loop_end {
                loop_start
            } else {
                current_index + 1
            };
            let vertex = *next_topology_vertex;
            *next_topology_vertex += 1;
            // Carrier construction has already certified a nonempty ordered
            // domain. Authored endpoint events therefore need no predicate;
            // later contact insertion still performs exact deduplication.
            events[current_index].push(CarrierEvent {
                parameter: carrier_traversal_end(&carriers[current_index]).clone(),
                topology_vertex: Some(vertex),
            });
            events[next_index].push(CarrierEvent {
                parameter: carrier_traversal_start(&carriers[next_index]).clone(),
                topology_vertex: Some(vertex),
            });
        }
        loop_start = loop_end;
    }
}

fn carrier_traversal_start(carrier: &RegionCarrier) -> &CurveParameter2 {
    if carrier.reversed {
        &carrier.end
    } else {
        &carrier.start
    }
}

fn carrier_traversal_end(carrier: &RegionCarrier) -> &CurveParameter2 {
    if carrier.reversed {
        &carrier.start
    } else {
        &carrier.end
    }
}

fn existing_event_vertex_if_decided(
    events: &[CarrierEvent],
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<usize>> {
    for event in events {
        match parameter.cmp_by_refinement(&event.parameter, policy)? {
            Classification::Decided(Ordering::Equal) => return Ok(event.topology_vertex),
            Classification::Decided(Ordering::Less | Ordering::Greater)
            | Classification::Uncertain(_) => {}
        }
    }
    Ok(None)
}

fn existing_contact_event_vertex_if_decided(
    events: &[CarrierEvent],
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<usize>> {
    for event in events {
        match locally_decidable_contact_parameter_cmp(parameter, &event.parameter, policy)? {
            Classification::Decided(Ordering::Equal) => return Ok(event.topology_vertex),
            Classification::Decided(Ordering::Less | Ordering::Greater)
            | Classification::Uncertain(_) => {}
        }
    }
    Ok(None)
}

fn locally_decidable_contact_parameter_cmp(
    first: &CurveParameter2,
    second: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Ordering>> {
    if first == second {
        return Ok(Classification::Decided(Ordering::Equal));
    }
    if let (Some(first), Some(second)) = (
        first.as_recursive_projective(),
        second.as_recursive_projective(),
    ) {
        return policy
            .bounded_exact_predicate_pass(|| first.cmp_by_native_refinement(second, policy));
    }
    if first.is_retained_scalar() && second.is_retained_scalar() {
        // Two independent retained authorities can require a joined recursive
        // field even when this caller only seeks an optional shortcut.  Their
        // exact Cartesian evidence is the authoritative contact identity.
        return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
    }
    // This is an optional contact lookup. Geometric incidence and endpoint
    // topology still follow, so it must neither exhaust a global scalar
    // reconstruction nor consume an approximate equality before those proofs.
    policy.bounded_exact_predicate_pass(|| first.cmp_by_refinement(second, policy))
}

fn carrier_has_certified_injective_image(carrier: &RegionCarrier, policy: &CurveContext) -> bool {
    if let Some(&injective) = carrier.image_is_injective.get() {
        return injective;
    }
    let injective = carrier
        .geometry
        .has_certified_injective_image(&carrier.range(), policy);
    let _ = carrier.image_is_injective.set(injective);
    injective
}

fn canonicalize_injective_topology_events(
    events: &mut [Vec<CarrierEvent>],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) {
    for (carrier_events, carrier) in events.iter_mut().zip(carriers) {
        if !carrier_has_certified_injective_image(carrier, policy) {
            continue;
        }
        let mut index = 0;
        while index < carrier_events.len() {
            let duplicate = carrier_events[index].topology_vertex.is_some()
                && carrier_events[..index].iter().any(|previous| {
                    previous.topology_vertex == carrier_events[index].topology_vertex
                });
            if duplicate {
                carrier_events.remove(index);
            } else {
                index += 1;
            }
        }
    }
}

fn validate_carrier_event_separation(
    events: &[Vec<CarrierEvent>],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    for (carrier_events, carrier) in events.iter().zip(carriers) {
        for (index, event) in carrier_events.iter().enumerate() {
            for other in &carrier_events[index + 1..] {
                if let Classification::Uncertain(reason) = event
                    .parameter
                    .cmp_by_refinement(&other.parameter, policy)
                    .map_err(|cause| {
                        ExactCurveError::invalid(CurveOperation2::Boolean, carrier.family, cause)
                    })?
                {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Boolean,
                        carrier.family,
                        reason,
                    ));
                }
            }
        }
    }
    Ok(())
}

/// Certified outward f64 box of a contact point: its exact rational
/// coordinates, or else its certified bounds when their corners are rational.
fn exact_contact_point_box(point: &CurvePoint2) -> Option<[f64; 4]> {
    use crate::bezier_region::certified_f64_enclosure;
    if let Some(point) = point.coordinates() {
        let [x_low, x_high] = certified_f64_enclosure(point.x())?;
        let [y_low, y_high] = certified_f64_enclosure(point.y())?;
        return Some([x_low, x_high, y_low, y_high]);
    }
    let Classification::Decided(bounds) = point.bounds(&CurveContext::STRICT).value else {
        return None;
    };
    Some([
        certified_f64_enclosure(bounds.min().x())?[0],
        certified_f64_enclosure(bounds.max().x())?[1],
        certified_f64_enclosure(bounds.min().y())?[0],
        certified_f64_enclosure(bounds.max().y())?[1],
    ])
}

/// Spatial candidates for contact deduplication.
///
/// Every branch of the contact matcher that changes state requires the two
/// contacts to be one point. Contacts whose certified boxes are disjoint are
/// therefore never observable there and are skipped. Contacts without a box
/// (selected or unrepresented points) are always candidates.
#[derive(Default)]
struct ContactPointIndex {
    by_x_low: std::collections::BTreeMap<u64, Vec<usize>>,
    boxes: Vec<Option<[f64; 4]>>,
    unboxed: Vec<usize>,
    max_width: f64,
    vertex_boxes: Vec<Option<[f64; 4]>>,
    vertex_incidences: Vec<Vec<usize>>,
}

impl ContactPointIndex {
    /// Order-preserving key for finite f64 values.
    fn key(value: f64) -> u64 {
        let bits = value.to_bits();
        if bits >> 63 == 0 {
            bits | (1 << 63)
        } else {
            !bits
        }
    }

    fn ensure_vertex(&mut self, vertex: usize) {
        if self.vertex_boxes.len() <= vertex {
            self.vertex_boxes.resize(vertex + 1, None);
            self.vertex_incidences.resize_with(vertex + 1, Vec::new);
        }
    }

    fn push(&mut self, index: usize, point: Option<&CurvePoint2>, vertex: usize) {
        debug_assert_eq!(index, self.boxes.len());
        self.ensure_vertex(vertex);
        // A contact without its own point shares its vertex representative.
        let bounds = point
            .and_then(exact_contact_point_box)
            .or(self.vertex_boxes[vertex]);
        self.boxes.push(bounds);
        self.vertex_incidences[vertex].push(index);
        match bounds {
            Some(bounds) => {
                self.by_x_low
                    .entry(Self::key(bounds[0]))
                    .or_default()
                    .push(index);
                self.max_width = self.max_width.max(bounds[1] - bounds[0]);
                if self.vertex_boxes[vertex].is_none() {
                    self.vertex_boxes[vertex] = Some(bounds);
                }
            }
            None => self.unboxed.push(index),
        }
    }

    fn merge_vertex(&mut self, from: usize, to: usize) {
        self.ensure_vertex(from.max(to));
        let moved = std::mem::take(&mut self.vertex_incidences[from]);
        self.vertex_incidences[to].extend(moved);
        if self.vertex_boxes[to].is_none() {
            self.vertex_boxes[to] = self.vertex_boxes[from];
        }
    }

    fn incidences(&self, vertex: usize) -> &[usize] {
        self.vertex_incidences
            .get(vertex)
            .map_or(&[], Vec::as_slice)
    }

    /// Ascending candidate indices, or `None` when every contact must be
    /// examined because the new contact has no exact box.
    fn candidates(&self, point: Option<&CurvePoint2>) -> Option<Vec<usize>> {
        let bounds = point.and_then(exact_contact_point_box)?;
        let low = Self::key(bounds[0] - self.max_width);
        let high = Self::key(bounds[1]);
        let mut candidates = self.unboxed.clone();
        for indices in self.by_x_low.range(low..=high).map(|(_, indices)| indices) {
            for &index in indices {
                let other = self.boxes[index].expect("indexed contacts have boxes");
                if other[1] >= bounds[0]
                    && other[0] <= bounds[1]
                    && other[3] >= bounds[2]
                    && other[2] <= bounds[3]
                {
                    candidates.push(index);
                }
            }
        }
        candidates.sort_unstable();
        Some(candidates)
    }
}

fn replace_topology_vertex(
    events: &mut [Vec<CarrierEvent>],
    contact_points: &mut [ContactVertex],
    from: usize,
    to: usize,
) {
    for event in events.iter_mut().flatten() {
        if event.topology_vertex == Some(from) {
            event.topology_vertex = Some(to);
        }
    }
    for contact in contact_points {
        if contact.topology_vertex == from {
            contact.topology_vertex = to;
        }
    }
}

fn contacts_decided_distinct_from_carriers(
    existing: &ContactVertex,
    carrier_indices: [usize; 2],
    parameters: [&CurveParameter2; 2],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    for (existing_shared_slot, existing_shared_carrier) in
        existing.carrier_indices.iter().copied().enumerate()
    {
        for (current_shared_slot, current_shared_carrier) in
            carrier_indices.iter().copied().enumerate()
        {
            if existing_shared_carrier != current_shared_carrier {
                continue;
            }
            let existing_chord_slot = 1 - existing_shared_slot;
            let current_chord_slot = 1 - current_shared_slot;
            let existing_chord_index = existing.carrier_indices[existing_chord_slot];
            let current_chord_index = carrier_indices[current_chord_slot];
            let (CurveSupport2::Line(existing_chord), CurveSupport2::Line(current_chord)) = (
                &carriers[existing_chord_index].geometry,
                &carriers[current_chord_index].geometry,
            ) else {
                continue;
            };
            let (Some(existing_parameter), Some(current_parameter)) = (
                existing.parameters[existing_chord_slot].as_algebraic_chord(),
                parameters[current_chord_slot].as_algebraic_chord(),
            ) else {
                continue;
            };
            if !existing_parameter.is_certified_strict_interior_of(existing_chord)
                || !current_parameter.is_certified_strict_interior_of(current_chord)
            {
                continue;
            }
            let shared_endpoint = [existing_chord.start(), existing_chord.end()]
                .into_iter()
                .enumerate()
                .find_map(|(existing_endpoint, first)| {
                    [current_chord.start(), current_chord.end()]
                        .into_iter()
                        .enumerate()
                        .find_map(|(current_endpoint, second)| {
                            first
                                .shares_storage(second)
                                .then_some((existing_endpoint, current_endpoint))
                        })
                });
            let Some((existing_endpoint, current_endpoint)) = shared_endpoint else {
                continue;
            };
            let Some(cross) =
                existing_chord.tangent_cross_sign_with_shared_endpoint(current_chord, policy)
            else {
                continue;
            };
            let cross = cross.map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Boolean,
                    carriers[existing_chord_index].family,
                    cause,
                )
            })?;
            match cross {
                Classification::Decided(RealSign::Positive | RealSign::Negative) => {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "contact-point-distinctness",
                        "shared-endpoint-chord-interiors",
                    );
                    return Ok(true);
                }
                Classification::Decided(RealSign::Zero) => {
                    let dot = existing_chord
                        .tangent_dot_sign(current_chord, policy)
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Boolean,
                                carriers[existing_chord_index].family,
                                cause,
                            )
                        })?;
                    let Classification::Decided(mut away_dot) = dot else {
                        continue;
                    };
                    if existing_endpoint != current_endpoint {
                        away_dot = match away_dot {
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => RealSign::Zero,
                            RealSign::Positive => RealSign::Negative,
                        };
                    }
                    if away_dot == RealSign::Negative {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "contact-point-distinctness",
                            "opposite-shared-endpoint-chord-interiors",
                        );
                        return Ok(true);
                    }
                }
                Classification::Uncertain(_) => {}
            }
        }
    }
    for (existing_slot, existing_carrier) in existing.carrier_indices.iter().copied().enumerate() {
        for (current_slot, current_carrier) in carrier_indices.iter().copied().enumerate() {
            // The per-carrier cache below already covers contacts on one
            // fragment. Separate fragments need a proof over their joining range.
            if existing_carrier == current_carrier {
                continue;
            }
            let (
                CurveSupport2::Parallel(existing_parallel),
                CurveSupport2::Parallel(current_parallel),
            ) = (
                &carriers[existing_carrier].geometry,
                &carriers[current_carrier].geometry,
            )
            else {
                continue;
            };
            if existing_parallel != current_parallel
                || !matches!(
                    locally_decidable_contact_parameter_cmp(
                        &existing.parameters[existing_slot],
                        parameters[current_slot],
                        policy,
                    )
                        .map_err(|cause| ExactCurveError::invalid(
                            CurveOperation2::Boolean,
                            carriers[existing_carrier].family,
                            cause,
                        ))?,
                    Classification::Decided(order) if order != Ordering::Equal
                )
            {
                continue;
            }
            let joining_range = CurveParameterRange2::new_validated(
                existing.parameters[existing_slot].clone(),
                parameters[current_slot].clone(),
            );
            if existing_parallel.range_has_certified_injective_axis(&joining_range, policy) {
                return Ok(true);
            }
        }
    }
    for existing_carrier in existing.carrier_indices {
        for current_carrier in carrier_indices {
            let existing_bounds = carriers[existing_carrier].bounds.get_or_init(|| {
                carriers[existing_carrier].geometry.certified_outer_bounds(
                    &carriers[existing_carrier].range(),
                    0,
                    policy,
                )
            });
            let current_bounds = carriers[current_carrier].bounds.get_or_init(|| {
                carriers[current_carrier].geometry.certified_outer_bounds(
                    &carriers[current_carrier].range(),
                    0,
                    policy,
                )
            });
            // Bounds are only an optional distinctness certificate. An
            // unresolved overlap must reach the exact point/parameter replay.
            if let (
                Classification::Decided(existing_bounds),
                Classification::Decided(current_bounds),
            ) = (existing_bounds, current_bounds)
                && existing_bounds.overlaps(current_bounds, &policy.strict_counterpart())
                    == Classification::Decided(false)
            {
                return Ok(true);
            }
        }
    }
    for (existing_slot, existing_carrier) in existing.carrier_indices.iter().copied().enumerate() {
        let carrier = &carriers[existing_carrier];
        if !carrier_has_certified_injective_image(carrier, policy) {
            continue;
        }
        for (current_slot, current_carrier) in carrier_indices.iter().copied().enumerate() {
            if existing_carrier == current_carrier
                && matches!(
                    locally_decidable_contact_parameter_cmp(
                        &existing.parameters[existing_slot],
                        parameters[current_slot],
                        policy,
                    )
                        .map_err(|cause| ExactCurveError::invalid(
                            CurveOperation2::Boolean,
                            carrier.family,
                            cause,
                        ))?,
                    Classification::Decided(order) if order != Ordering::Equal
                )
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Proves that an existing contact vertex cannot equal an already-authored
/// endpoint vertex of one of its incident carriers.
///
/// A contact record can omit other carrier incidences attached to the same
/// topology vertex, so reconciliation applies this predicate across the
/// complete vertex group. On an injective algebraic chord, a parameter
/// certified in the strict finite interior is necessarily distinct from both
/// authored endpoint parameters; no Cartesian point comparison is needed.
fn contact_decided_distinct_from_carrier_endpoint_vertex(
    existing: &ContactVertex,
    current_vertex: usize,
    events: &[Vec<CarrierEvent>],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> bool {
    existing
        .carrier_indices
        .iter()
        .copied()
        .enumerate()
        .any(|(slot, carrier_index)| {
            let carrier = &carriers[carrier_index];
            let CurveSupport2::Line(chord) = &carrier.geometry else {
                return false;
            };
            let Some(parameter) = existing.parameters[slot].as_algebraic_chord() else {
                return false;
            };
            parameter.is_certified_strict_interior_of(chord)
                && carrier_has_certified_injective_image(carrier, policy)
                && events[carrier_index].iter().any(|event| {
                    event.topology_vertex == Some(current_vertex)
                        && (event.parameter == carrier.start || event.parameter == carrier.end)
                })
        })
}

fn contacts_decided_same_from_shared_parallel(
    existing: &ContactVertex,
    carrier_indices: [usize; 2],
    parameters: [&CurveParameter2; 2],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    for (existing_slot, existing_carrier) in existing.carrier_indices.iter().copied().enumerate() {
        for (current_slot, current_carrier) in carrier_indices.iter().copied().enumerate() {
            let (
                CurveSupport2::Parallel(existing_parallel),
                CurveSupport2::Parallel(current_parallel),
            ) = (
                &carriers[existing_carrier].geometry,
                &carriers[current_carrier].geometry,
            )
            else {
                continue;
            };
            if existing_carrier == current_carrier {
                let existing_chord_slot = 1 - existing_slot;
                let current_chord_slot = 1 - current_slot;
                let existing_chord_index = existing.carrier_indices[existing_chord_slot];
                let current_chord_index = carrier_indices[current_chord_slot];
                if let (
                    CurveSupport2::Line(existing_chord),
                    CurveSupport2::Line(current_chord),
                    Some(existing_parallel_parameter),
                    Some(current_parallel_parameter),
                    Some(existing_chord_parameter),
                    Some(current_chord_parameter),
                ) = (
                    &carriers[existing_chord_index].geometry,
                    &carriers[current_chord_index].geometry,
                    existing.parameters[existing_slot].as_recursive_projective(),
                    parameters[current_slot].as_recursive_projective(),
                    existing.parameters[existing_chord_slot].as_algebraic_chord(),
                    parameters[current_chord_slot].as_algebraic_chord(),
                ) && existing_parallel_parameter
                    .certifies_monotone_chord_parallel_contact(existing_chord, existing_parallel)
                    && current_parallel_parameter
                        .certifies_monotone_chord_parallel_contact(current_chord, current_parallel)
                    && existing_chord_parameter.is_certified_strict_interior_of(existing_chord)
                    && current_chord_parameter.is_certified_strict_interior_of(current_chord)
                    && [existing_chord.start(), existing_chord.end()]
                        .into_iter()
                        .any(|first| {
                            [current_chord.start(), current_chord.end()]
                                .into_iter()
                                .any(|second| first.shares_storage(second))
                        })
                    && let Some(cross) = existing_chord
                        .tangent_cross_sign_with_shared_endpoint(current_chord, policy)
                {
                    match cross.map_err(|cause| {
                        ExactCurveError::invalid(
                            CurveOperation2::Boolean,
                            carriers[existing_chord_index].family,
                            cause,
                        )
                    })? {
                        Classification::Decided(RealSign::Zero) => {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "contact-point-equality",
                                "collinear-monotone-shared-carrier-root",
                            );
                            return Ok(true);
                        }
                        Classification::Decided(RealSign::Negative | RealSign::Positive)
                        | Classification::Uncertain(_) => {}
                    }
                }
            }
            if existing_parallel == current_parallel
                && matches!(
                    locally_decidable_contact_parameter_cmp(
                        &existing.parameters[existing_slot],
                        parameters[current_slot],
                        policy,
                    )
                    .map_err(|cause| ExactCurveError::invalid(
                        CurveOperation2::Boolean,
                        carriers[existing_carrier].family,
                        cause,
                    ))?,
                    Classification::Decided(Ordering::Equal)
                )
            {
                let normal_sheet_matches = match (
                    carriers[existing_carrier].start.as_bezier_parameter(),
                    carriers[existing_carrier].end.as_bezier_parameter(),
                    carriers[current_carrier].start.as_bezier_parameter(),
                    carriers[current_carrier].end.as_bezier_parameter(),
                ) {
                    (
                        Some(existing_start),
                        Some(existing_end),
                        Some(current_start),
                        Some(current_end),
                    ) => existing_parallel
                        .regular_source_ranges_share_normal_sheet(
                            &BezierParameterRange2::new_validated(
                                existing_start.clone(),
                                existing_end.clone(),
                            ),
                            &BezierParameterRange2::new_validated(
                                current_start.clone(),
                                current_end.clone(),
                            ),
                            policy,
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::Boolean,
                                carriers[existing_carrier].family,
                                cause,
                            )
                        })?,
                    _ => Classification::Uncertain(UncertaintyReason::Unsupported),
                };
                if !matches!(normal_sheet_matches, Classification::Decided(true)) {
                    continue;
                }
                let existing_endpoint = selected_fiber_endpoint_point_at_parameter(
                    &carriers[existing_carrier],
                    &existing.parameters[existing_slot],
                    policy,
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Boolean,
                        carriers[existing_carrier].family,
                        cause,
                    )
                })?;
                let current_endpoint = selected_fiber_endpoint_point_at_parameter(
                    &carriers[current_carrier],
                    parameters[current_slot],
                    policy,
                )
                .map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Boolean,
                        carriers[current_carrier].family,
                        cause,
                    )
                })?;
                if existing_endpoint.is_some() || current_endpoint.is_some() {
                    let (Some(existing_endpoint), Some(current_endpoint)) =
                        (existing_endpoint, current_endpoint)
                    else {
                        // At a source singularity the same procedural
                        // parallel and parameter can have two distinct
                        // one-sided normal limits.  Without both branch-owned
                        // endpoint images, parameter identity is not point
                        // identity; let the ordinary point-evidence matcher
                        // decide the contact instead.
                        continue;
                    };
                    let same = existing_endpoint.same_point(current_endpoint, policy);
                    if !matches!(same, Classification::Decided(true)) {
                        continue;
                    }
                }
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn selected_fiber_endpoint_point_at_parameter<'a>(
    carrier: &'a RegionCarrier,
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<&'a CurvePoint2>> {
    let Some(points) = carrier.selected_fiber_endpoint_points.as_deref() else {
        return Ok(None);
    };
    for (endpoint, point) in [(&carrier.start, &points[0]), (&carrier.end, &points[1])] {
        match endpoint.cmp_by_refinement(parameter, policy)? {
            Classification::Decided(Ordering::Equal) => return Ok(Some(point)),
            Classification::Decided(Ordering::Less | Ordering::Greater) => {}
            Classification::Uncertain(_) => return Ok(None),
        }
    }
    Ok(None)
}

fn parameter_matches_any(
    parameter: &CurveParameter2,
    candidates: &[BezierParameter2],
    policy: &CurveContext,
) -> Classification<bool> {
    let Some(parameter) = parameter.as_bezier_parameter() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    let mut uncertainty = None;
    for candidate in candidates {
        match parameter.same_value(candidate, policy) {
            Ok(Classification::Decided(true)) => return Classification::Decided(true),
            Ok(Classification::Decided(false)) => {}
            Ok(Classification::Uncertain(reason)) => {
                uncertainty.get_or_insert(reason);
            }
            Err(_) => {
                uncertainty.get_or_insert(UncertaintyReason::Unsupported);
            }
        }
    }
    uncertainty.map_or(Classification::Decided(false), Classification::Uncertain)
}

fn contacts_decided_same_from_circular_carriers(
    existing: &ContactVertex,
    carrier_indices: [usize; 2],
    parameters: [&CurveParameter2; 2],
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> Classification<bool> {
    let mut uncertainty = None;
    for (existing_slot, existing_carrier) in existing.carrier_indices.iter().copied().enumerate() {
        let CurveSupport2::Bezier(existing_subcurve) = &carriers[existing_carrier].geometry else {
            continue;
        };
        let Ok(existing_curve) = RationalBezier2::try_from_subcurve(existing_subcurve) else {
            continue;
        };
        if existing_curve.retained_circular_conic().is_none() {
            continue;
        }
        let Ok(Classification::Decided(Segment2::Arc(existing_arc))) =
            crate::bezier_region::materialized_native_subcurve_segment(existing_subcurve, policy)
        else {
            continue;
        };
        for (current_slot, current_carrier) in carrier_indices.iter().copied().enumerate() {
            let CurveSupport2::Bezier(current_subcurve) = &carriers[current_carrier].geometry
            else {
                continue;
            };
            let Ok(current_curve) = RationalBezier2::try_from_subcurve(current_subcurve) else {
                continue;
            };
            if current_curve.retained_circular_conic().is_none() {
                continue;
            }
            let Ok(Classification::Decided(Segment2::Arc(current_arc))) =
                crate::bezier_region::materialized_native_subcurve_segment(
                    current_subcurve,
                    policy,
                )
            else {
                continue;
            };
            let relation = match existing_arc.intersect_arc(&current_arc, policy) {
                Ok(relation) => relation,
                Err(_) => {
                    uncertainty.get_or_insert(UncertaintyReason::Unsupported);
                    continue;
                }
            };
            let points = match relation {
                ArcArcIntersection::None => return Classification::Decided(false),
                ArcArcIntersection::Point(hit) => vec![hit.point],
                ArcArcIntersection::TwoPoints { first, second } => {
                    vec![first.point, second.point]
                }
                ArcArcIntersection::Overlap { .. } => {
                    uncertainty.get_or_insert(UncertaintyReason::Boundary);
                    continue;
                }
                ArcArcIntersection::Uncertain { reason } => {
                    uncertainty.get_or_insert(reason);
                    continue;
                }
            };
            let mut pair_uncertainty = None;
            for point in points {
                let existing_parameters =
                    match existing_curve.retained_circle_point_parameters(&point, policy) {
                        Ok(Classification::Decided(parameters)) => parameters,
                        Ok(Classification::Uncertain(reason)) => {
                            pair_uncertainty.get_or_insert(reason);
                            continue;
                        }
                        Err(_) => {
                            pair_uncertainty.get_or_insert(UncertaintyReason::Unsupported);
                            continue;
                        }
                    };
                let current_parameters =
                    match current_curve.retained_circle_point_parameters(&point, policy) {
                        Ok(Classification::Decided(parameters)) => parameters,
                        Ok(Classification::Uncertain(reason)) => {
                            pair_uncertainty.get_or_insert(reason);
                            continue;
                        }
                        Err(_) => {
                            pair_uncertainty.get_or_insert(UncertaintyReason::Unsupported);
                            continue;
                        }
                    };
                match (
                    parameter_matches_any(
                        &existing.parameters[existing_slot],
                        &existing_parameters,
                        policy,
                    ),
                    parameter_matches_any(parameters[current_slot], &current_parameters, policy),
                ) {
                    (Classification::Decided(true), Classification::Decided(true)) => {
                        return Classification::Decided(true);
                    }
                    (Classification::Uncertain(reason), _)
                    | (_, Classification::Uncertain(reason)) => {
                        pair_uncertainty.get_or_insert(reason);
                    }
                    _ => {}
                }
            }
            if let Some(reason) = pair_uncertainty {
                uncertainty.get_or_insert(reason);
            } else {
                return Classification::Decided(false);
            }
        }
    }
    Classification::Uncertain(uncertainty.unwrap_or(UncertaintyReason::Unsupported))
}

fn exact_point_decided_outside_carrier(
    point: &crate::Point2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> bool {
    let CurveSupport2::Bezier(curve) = &carrier.geometry else {
        return false;
    };
    let Ok(Classification::Decided(segment)) =
        crate::bezier_region::materialized_native_subcurve_segment(curve, policy)
    else {
        return false;
    };
    match segment {
        Segment2::Line(line) => {
            line.contains_point(point, policy) == Classification::Decided(false)
        }
        Segment2::Arc(arc) => {
            arc.contains_sweep_point(point, policy) == Classification::Decided(false)
                || arc.contains_point(point, policy) == Classification::Decided(false)
        }
    }
}

fn exact_point_matches_existing_contact_parameter(
    point: &crate::Point2,
    existing: &ContactVertex,
    carriers: &[RegionCarrier],
    policy: &CurveContext,
) -> Classification<bool> {
    let mut uncertainty = None;
    for (slot, carrier_index) in existing.carrier_indices.iter().copied().enumerate() {
        let CurveSupport2::Bezier(curve) = &carriers[carrier_index].geometry else {
            continue;
        };
        let Ok(curve) = RationalBezier2::try_from_subcurve(curve) else {
            continue;
        };
        if curve.retained_circular_conic().is_none() {
            continue;
        }
        let parameters = match curve.retained_circle_point_parameters(point, policy) {
            Ok(Classification::Decided(parameters)) => parameters,
            Ok(Classification::Uncertain(reason)) => {
                uncertainty.get_or_insert(reason);
                continue;
            }
            Err(_) => {
                uncertainty.get_or_insert(UncertaintyReason::Unsupported);
                continue;
            }
        };
        if parameters.is_empty() {
            return Classification::Decided(false);
        }
        let mut parameter_uncertainty = None;
        for parameter in parameters {
            let Some(existing_parameter) = existing.parameters[slot].as_bezier_parameter() else {
                uncertainty.get_or_insert(UncertaintyReason::Unsupported);
                continue;
            };
            match existing_parameter.same_value(&parameter, policy) {
                Ok(Classification::Decided(true)) => return Classification::Decided(true),
                Ok(Classification::Decided(false)) => {}
                Ok(Classification::Uncertain(reason)) => {
                    parameter_uncertainty.get_or_insert(reason);
                }
                Err(_) => {
                    parameter_uncertainty.get_or_insert(UncertaintyReason::Unsupported);
                }
            }
        }
        if parameter_uncertainty.is_none() {
            return Classification::Decided(false);
        }
        uncertainty = parameter_uncertainty;
    }
    Classification::Uncertain(uncertainty.unwrap_or(UncertaintyReason::Unsupported))
}

fn event_vertex(
    events: &[CarrierEvent],
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> Result<Option<usize>, CurveError> {
    for event in events {
        match parameter.cmp_by_refinement(&event.parameter, policy)? {
            Classification::Decided(Ordering::Equal) => return Ok(event.topology_vertex),
            Classification::Decided(_) => {}
            Classification::Uncertain(reason) => {
                return Err(CurveError::Topology(format!(
                    "curved-region event ordering remained uncertain: {reason:?}"
                )));
            }
        }
    }
    Ok(None)
}

fn action_for_sides(
    operation: BooleanOp,
    operand: CurveRegionBooleanOperand2,
    own_left: bool,
    other_inside: bool,
) -> RegionFragmentAction {
    let (result_left, result_right) = match operand {
        CurveRegionBooleanOperand2::First => (
            operation.apply(own_left, other_inside),
            operation.apply(!own_left, other_inside),
        ),
        CurveRegionBooleanOperand2::Second => (
            operation.apply(other_inside, own_left),
            operation.apply(other_inside, !own_left),
        ),
    };
    action_from_result_sides(result_left, result_right)
}

const fn orient_tangent_cross_sign(sign: RealSign, source_is_first: bool) -> RealSign {
    if source_is_first {
        sign
    } else {
        match sign {
            RealSign::Positive => RealSign::Negative,
            RealSign::Negative => RealSign::Positive,
            RealSign::Zero => RealSign::Zero,
        }
    }
}

fn selected_fiber_cusp_result(
    contacts: Vec<BezierAlgebraicCuspSemicircleSelectedFiberContact2>,
    overlaps: Vec<BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2>,
    cusp_is_first: bool,
) -> RegionPairResult {
    let contacts = contacts
        .into_iter()
        .map(|contact| {
            let cusp_parameter = CurveParameter2::from_algebraic_cusp(contact.cusp_parameter());
            let other_parameter =
                CurveParameter2::from_selected_fiber(contact.other_parameter().clone());
            let tangent_cross_sign =
                orient_tangent_cross_sign(contact.tangent_cross_sign(), cusp_is_first);
            let (first_parameter, second_parameter) = if cusp_is_first {
                (cusp_parameter, other_parameter)
            } else {
                (other_parameter, cusp_parameter)
            };
            RegionPairContactEvidence::direct(
                first_parameter,
                second_parameter,
                Some(contact.point_evidence()),
                tangent_cross_sign != RealSign::Zero,
                Some(tangent_cross_sign),
            )
        })
        .collect();
    RegionPairResult {
        contacts,
        overlaps: overlaps
            .into_iter()
            .map(|source| {
                circle_overlap_evidence(CurveCircleOverlap2::Selected(source), cusp_is_first)
            })
            .collect(),
        blockers: Vec::new(),
    }
}

fn retained_cusp_parallel_contacts_result(
    contacts: Vec<BezierAlgebraicCuspSemicircleRetainedParallelContact2>,
    cusp_is_first: bool,
) -> RegionPairResult {
    let contacts = contacts
        .into_iter()
        .map(|contact| {
            let cusp_parameter = CurveParameter2::from_algebraic_cusp(contact.cusp_parameter());
            let other_parameter = contact.other_parameter().clone();
            let tangent_cross_sign =
                orient_tangent_cross_sign(contact.tangent_cross_sign(), cusp_is_first);
            let tangent_topology =
                contact
                    .tangent_topology()
                    .and_then(|(dot, circle_side_of_parallel)| {
                        let opposite = |side| match side {
                            LineSide::Left => LineSide::Right,
                            LineSide::Right => LineSide::Left,
                            LineSide::On => LineSide::On,
                        };
                        let second_side_of_first = if cusp_is_first {
                            if dot == RealSign::Positive {
                                opposite(circle_side_of_parallel)
                            } else {
                                circle_side_of_parallel
                            }
                        } else {
                            circle_side_of_parallel
                        };
                        (second_side_of_first != LineSide::On)
                            .then_some((dot, second_side_of_first))
                    });
            let (first_parameter, second_parameter) = if cusp_is_first {
                (cusp_parameter, other_parameter)
            } else {
                (other_parameter, cusp_parameter)
            };
            let evidence = RegionPairContactEvidence::direct(
                first_parameter,
                second_parameter,
                Some(contact.point_evidence()),
                tangent_cross_sign != RealSign::Zero,
                Some(tangent_cross_sign),
            );
            match tangent_topology {
                Some((dot, side)) => evidence.with_tangent_topology(dot, side),
                None => evidence,
            }
        })
        .collect();
    RegionPairResult {
        contacts,
        overlaps: Vec::new(),
        blockers: Vec::new(),
    }
}

fn circle_overlap_evidence(
    source: CurveCircleOverlap2,
    cusp_is_first: bool,
) -> CurveIntersectionOverlap2 {
    let (cusp_range, other_range) = source.parameter_ranges();
    let (first_range, second_range) = if cusp_is_first {
        (cusp_range, other_range)
    } else {
        (other_range, cusp_range)
    };
    CurveIntersectionOverlap2 {
        first_span_index: 0,
        second_span_index: 0,
        endpoint_inclusion: [true, true],
        orientation: source.orientation(),
        parameter_correspondence: CurveOverlapCorrespondence2::Circle {
            source,
            swapped: !cusp_is_first,
        },
        first_range,
        second_range,
    }
}

const fn action_from_result_sides(left: bool, right: bool) -> RegionFragmentAction {
    match (left, right) {
        (true, false) => RegionFragmentAction::Keep,
        (false, true) => RegionFragmentAction::KeepReversed,
        (false, false) | (true, true) => RegionFragmentAction::Discard,
    }
}

const fn carrier_traversal_start_parameter(carrier: &RegionCarrier) -> &CurveParameter2 {
    if carrier.reversed {
        &carrier.end
    } else {
        &carrier.start
    }
}

const fn carrier_traversal_end_parameter(carrier: &RegionCarrier) -> &CurveParameter2 {
    if carrier.reversed {
        &carrier.start
    } else {
        &carrier.end
    }
}

fn exact_carrier_point(
    carrier: &RegionCarrier,
    parameter: &CurveParameter2,
    policy: &CurveContext,
) -> Option<crate::Point2> {
    let parameter = parameter.scalar()?;
    match carrier.geometry.point_at(parameter, policy) {
        Ok(Classification::Decided(point)) => Some(point),
        Ok(Classification::Uncertain(_)) | Err(_) => None,
    }
}

fn point_coordinate(point: &crate::Point2, axis: Axis2) -> &Real {
    match axis {
        Axis2::X => point.x(),
        Axis2::Y => point.y(),
    }
}

fn ordered_axis_endpoint_points<'a>(
    first: &'a crate::Point2,
    second: &'a crate::Point2,
    axis: Axis2,
    policy: &CurveContext,
) -> Option<(&'a crate::Point2, &'a crate::Point2)> {
    match compare_reals(
        point_coordinate(first, axis),
        point_coordinate(second, axis),
        policy,
    ) {
        Some(Ordering::Less) => Some((first, second)),
        Some(Ordering::Greater) => Some((second, first)),
        Some(Ordering::Equal) | None => None,
    }
}

fn points_are_decided_distinct(
    first: &crate::Point2,
    second: &crate::Point2,
    policy: &CurveContext,
) -> bool {
    [Axis2::X, Axis2::Y].into_iter().any(|axis| {
        matches!(
            compare_reals(
                point_coordinate(first, axis),
                point_coordinate(second, axis),
                policy,
            ),
            Some(Ordering::Less | Ordering::Greater)
        )
    })
}

fn parameter_in_carrier(
    parameter: &CurveParameter2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    Ok(parameter_location_in_carrier(parameter, carrier, policy)?
        != CarrierParameterLocation::Outside)
}

fn parameter_location_in_carrier(
    parameter: &CurveParameter2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<CarrierParameterLocation> {
    if parameter == &carrier.start {
        return Ok(CarrierParameterLocation::Endpoint(BezierEndpoint::Start));
    }
    if parameter == &carrier.end {
        return Ok(CarrierParameterLocation::Endpoint(BezierEndpoint::End));
    }
    if let (Some(parameter), CurveSupport2::Line(chord)) =
        (parameter.as_algebraic_chord(), &carrier.geometry)
        && parameter.is_certified_strict_interior_of(chord)
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "curve-region-carrier-parameter-location",
            "authored-strict-interior",
        );
        return Ok(CarrierParameterLocation::Interior);
    }
    if let (Some(parameter), CurveSupport2::Circle(fragment)) =
        (parameter.as_algebraic_cusp(), &carrier.geometry)
        && let Some(Classification::Decided(true)) = fragment
            .translated_pair_parameter_is_strict_interior(parameter, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Boolean, carrier.family, cause)
            })?
    {
        return Ok(CarrierParameterLocation::Interior);
    }
    // Two independently selected fields need not admit a useful scalar
    // parameter comparison even when the pair kernel already certified their
    // common point on this supporting circle.  In that case the oriented
    // endpoint chord is the exact finite-arc predicate: its strict interior
    // side proves membership, its opposite side proves exclusion, and only a
    // chord contact needs the endpoint parameter comparison already provided
    // by `certified_incident_point_evidence_location`.
    if let (Some(parameter), CurveSupport2::Circle(fragment)) =
        (parameter.as_algebraic_cusp(), &carrier.geometry)
    {
        let point = parameter
            .coincident_point_evidence(fragment.semicircle(), policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Boolean, carrier.family, cause)
            })?;
        if let Classification::Decided(Some(point)) = point {
            use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::{
                End, Exterior, Interior, Start,
            };
            let location = fragment
                .certified_incident_point_evidence_location(parameter, &point, policy)
                .map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Boolean, carrier.family, cause)
                })?;
            match location {
                Classification::Decided(endpoint @ (Start | End)) => {
                    return Ok(CarrierParameterLocation::Endpoint(
                        if (endpoint == Start) ^ carrier.reversed {
                            BezierEndpoint::Start
                        } else {
                            BezierEndpoint::End
                        },
                    ));
                }
                Classification::Decided(Interior) => {
                    return Ok(CarrierParameterLocation::Interior);
                }
                Classification::Decided(Exterior) => {
                    return Ok(CarrierParameterLocation::Outside);
                }
                Classification::Uncertain(_) => {}
            }
        }
        use crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::{
            End, Exterior, Interior, Start,
        };
        return match fragment
            .parameter_location_by_order(parameter, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Boolean, carrier.family, cause)
            })? {
            Classification::Decided(endpoint @ (Start | End)) => Ok(
                CarrierParameterLocation::Endpoint(if (endpoint == Start) ^ carrier.reversed {
                    BezierEndpoint::Start
                } else {
                    BezierEndpoint::End
                }),
            ),
            Classification::Decided(Interior) => Ok(CarrierParameterLocation::Interior),
            Classification::Decided(Exterior) => Ok(CarrierParameterLocation::Outside),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Boolean,
                carrier.family,
                reason,
            )),
        };
    }
    let lower = decided_parameter_cmp(parameter, &carrier.start, policy)?;
    let upper = decided_parameter_cmp(parameter, &carrier.end, policy)?;
    Ok(if lower.is_lt() || upper.is_gt() {
        CarrierParameterLocation::Outside
    } else if lower == Ordering::Equal {
        CarrierParameterLocation::Endpoint(BezierEndpoint::Start)
    } else if upper == Ordering::Equal {
        CarrierParameterLocation::Endpoint(BezierEndpoint::End)
    } else {
        CarrierParameterLocation::Interior
    })
}

fn ranges_intersect(
    range: &CurveParameterRange2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (start, end) = ascending_range(range, policy)?;
    Ok(!decided_parameter_cmp(end, &carrier.start, policy)?.is_lt()
        && !decided_parameter_cmp(start, &carrier.end, policy)?.is_gt())
}

fn range_inside_carrier(
    range: &CurveParameterRange2,
    carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (start, end) = ascending_range(range, policy)?;
    Ok(
        !decided_parameter_cmp(start, &carrier.start, policy)?.is_lt()
            && !decided_parameter_cmp(end, &carrier.end, policy)?.is_gt(),
    )
}

#[cfg(test)]
fn clip_corresponding_parameter_overlap(
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    correspondence: &RationalBezierOverlapParameterCorrespondence2,
    first_carrier: &RegionCarrier,
    second_carrier: &RegionCarrier,
    policy: &CurveContext,
) -> ExactCurveResult<Option<(CurveParameterRange2, CurveParameterRange2)>> {
    let first_fragment =
        CurveParameterRange2::new_validated(first_carrier.start.clone(), first_carrier.end.clone());
    let second_fragment = CurveParameterRange2::new_validated(
        second_carrier.start.clone(),
        second_carrier.end.clone(),
    );
    match correspondence.clipped_ranges(
        first_range,
        second_range,
        &first_fragment,
        &second_fragment,
        policy,
    ) {
        Ok(Classification::Decided(ranges)) => Ok(ranges),
        Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
            CurveOperation2::Boolean,
            first_carrier.family,
            reason,
        )),
        Err(cause) => Err(ExactCurveError::invalid(
            CurveOperation2::Boolean,
            first_carrier.family,
            cause,
        )),
    }
}

fn range_contains_fragment(
    range: &CurveParameterRange2,
    fragment_start: &CurveParameter2,
    fragment_end: &CurveParameter2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (range_start, range_end) = ascending_range(range, policy)?;
    Ok(
        !decided_parameter_cmp(fragment_start, range_start, policy)?.is_lt()
            && !decided_parameter_cmp(fragment_end, range_end, policy)?.is_gt(),
    )
}

fn ascending_range<'a>(
    range: &'a CurveParameterRange2,
    policy: &CurveContext,
) -> ExactCurveResult<(&'a CurveParameter2, &'a CurveParameter2)> {
    match decided_parameter_cmp(range.start(), range.end(), policy)? {
        Ordering::Less => Ok((range.start(), range.end())),
        Ordering::Greater => Ok((range.end(), range.start())),
        Ordering::Equal => Err(ExactCurveError::invalid(
            CurveOperation2::Boolean,
            CurveFamily2::RationalBezier,
            CurveError::DegenerateOverlapRange,
        )),
    }
}

trait BooleanParameterOrder {
    fn boolean_cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>>;
}

impl BooleanParameterOrder for BezierParameter2 {
    fn boolean_cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        self.cmp_by_refinement(other, policy)
    }
}

impl BooleanParameterOrder for CurveParameter2 {
    fn boolean_cmp_by_refinement(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Ordering>> {
        self.cmp_by_refinement(other, policy)
    }
}

fn decided_parameter_cmp<P: BooleanParameterOrder>(
    first: &P,
    second: &P,
    policy: &CurveContext,
) -> ExactCurveResult<Ordering> {
    match first
        .boolean_cmp_by_refinement(second, policy)
        .map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Boolean,
                CurveFamily2::RationalBezier,
                cause,
            )
        })? {
        Classification::Decided(ordering) => Ok(ordering),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Boolean,
            CurveFamily2::RationalBezier,
            reason,
        )),
    }
}

fn fragment_range(
    fragment: &BezierSplitFragment2,
) -> Option<(&BezierParameter2, &BezierParameter2)> {
    match fragment {
        BezierSplitFragment2::Materialized { start, end, .. }
        | BezierSplitFragment2::RetainedBezier { start, end, .. } => Some((start, end)),
        BezierSplitFragment2::AnalyticParallel(fragment) => {
            Some((fragment.range().start(), fragment.range().end()))
        }
        BezierSplitFragment2::AlgebraicChord(_)
        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
        BezierSplitFragment2::SelectedFiber(_) => None,
    }
}

fn empty_operand_result(
    first: &CurveRegion2,
    second: &CurveRegion2,
    operation: BooleanOp,
) -> ExactCurveResult<CurveRegion2> {
    let result = match operation {
        BooleanOp::Union | BooleanOp::Xor => {
            if first.is_empty() {
                second.clone()
            } else {
                first.clone()
            }
        }
        BooleanOp::Intersection => CurveRegion2::new(Vec::new()).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, cause)
        })?,
        BooleanOp::Difference => first.clone(),
    };
    Ok(result)
}

fn identical_operand_result(
    region: &CurveRegion2,
    operation: BooleanOp,
) -> ExactCurveResult<CurveRegion2> {
    match operation {
        BooleanOp::Union | BooleanOp::Intersection => Ok(region.clone()),
        BooleanOp::Difference | BooleanOp::Xor => CurveRegion2::new(Vec::new()).map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Boolean, CurveFamily2::Line, cause)
        }),
    }
}

const fn boolean_operation_index(operation: BooleanOp) -> usize {
    match operation {
        BooleanOp::Union => 0,
        BooleanOp::Intersection => 1,
        BooleanOp::Difference => 2,
        BooleanOp::Xor => 3,
    }
}

#[cfg(test)]
mod certified_successor_tests;
