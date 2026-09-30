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
        let mut rays: Vec<(CurveTangent2, usize, usize, bool)> = Vec::with_capacity(incident.len());
        let mut coincident = Vec::new();
        for &(carrier, split, outgoing) in incident {
            let fragment = &topology.split_fragments[carrier][split].fragment;
            let straight = split_fragment_is_affine_line(fragment);
            let reversed;
            let ray = if outgoing {
                fragment
            } else {
                reversed = fragment.reversed().ok()?;
                &reversed
            };
            let Classification::Decided(tangent) =
                CurveTangent2::at_boundary_endpoint(ray, true, policy).ok()?
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
            for (index, (other, other_left, other_right, other_straight)) in rays.iter().enumerate()
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
                    Classification::Decided(Ordering::Equal) | Classification::Uncertain(_) => {
                        return None;
                    }
                }
            }
            if !merged {
                rays.insert(position, (tangent, left, right, straight));
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

    fn authored_parallel_support_contact(
        &self,
        pair: &RegionCarrierPair,
        parallel: &BezierParallel2,
        parallel_index: usize,
        direction_x: &Real,
        direction_y: &Real,
        regular_range: Option<&CurveParameterRange2>,
    ) -> ExactCurveResult<Option<(Real, RealSign)>> {
        let Some((first_at_start, second_at_start)) = self
            .authored_carrier_shared_endpoints(pair.first_carrier_index, pair.second_carrier_index)
        else {
            return Ok(None);
        };
        let parallel_at_start = if parallel_index == pair.first_carrier_index {
            first_at_start
        } else {
            second_at_start
        };
        let Some(parameter) = (if parallel_at_start {
            carrier_traversal_start(&self.data.carriers[parallel_index])
        } else {
            carrier_traversal_end(&self.data.carriers[parallel_index])
        })
        .scalar()
        .cloned() else {
            return Ok(None);
        };
        let tangent_relation = match regular_range {
            Some(range) => parallel.vector_tangent_cross_and_dot_signs_on_regular_range(
                &parameter.clone().into(),
                direction_x,
                direction_y,
                range,
                &self.data.policy,
            ),
            None => parallel.vector_tangent_cross_and_dot_signs(
                &parameter.clone().into(),
                direction_x,
                direction_y,
                &self.data.policy,
            ),
        }
        .map_err(|cause| self.invalid(parallel_index, cause))?;
        let Classification::Decided((cross, dot)) = tangent_relation else {
            return Ok(None);
        };
        Ok((cross != RealSign::Zero || dot != RealSign::Zero).then_some((parameter, cross)))
    }

    fn parallel_line_pair_result(
        &self,
        pair: &RegionCarrierPair,
        parallel: &BezierParallel2,
        parallel_index: usize,
        curve: &BezierSubcurve2,
        parallel_is_first: bool,
        regular_range: Option<&CurveParameterRange2>,
    ) -> ExactCurveResult<Classification<Option<RegionPairResult>>> {
        let retained = BezierSplitFragment2::Materialized {
            start: BezierParameter2::Exact(Real::zero()),
            end: BezierParameter2::Exact(Real::one()),
            curve: curve.clone(),
        };
        let line = match crate::bezier_region::retained_line_fragment_segment(
            &retained,
            &self.data.policy,
        )
        .map_err(|cause| self.invalid(0, cause))?
        {
            Classification::Decided(line) => line,
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let certified_tangent_contacts = match curve {
            BezierSubcurve2::Quadratic(curve) => curve
                .retained_parallel_line_tangent_contacts()
                .iter()
                .filter(|contact| contact.parallel() == parallel)
                .collect::<Vec<_>>(),
            BezierSubcurve2::Cubic(_)
            | BezierSubcurve2::RationalQuadratic(_)
            | BezierSubcurve2::Rational(_) => Vec::new(),
        };
        // The loop topology already owns its shared adjacent endpoint. Feed
        // that authored root and its exact first-order kind to the univariate
        // support kernel, which can divide it before isolating every residual
        // contact. This avoids asking independently materialized endpoint
        // expressions to rediscover their equality while preserving complete
        // detection of any later crossing of the finite line segment.
        let (direction_x, direction_y) = line.delta();
        let authored_contact = self.authored_parallel_support_contact(
            pair,
            parallel,
            parallel_index,
            &direction_x,
            &direction_y,
            regular_range,
        )?;
        let certified_crossing = authored_contact.as_ref().and_then(|(parameter, cross)| {
            let direction = match cross {
                RealSign::Positive => BezierLineCrossingDirection::NegativeToPositive,
                RealSign::Negative => BezierLineCrossingDirection::PositiveToNegative,
                RealSign::Zero => return None,
            };
            Some((parameter, direction))
        });
        let mut certified_tangent_parameters = certified_tangent_contacts
            .iter()
            .map(|contact| contact.parameter().clone())
            .collect::<Vec<_>>();
        if let Some((parameter, RealSign::Zero)) = &authored_contact
            && !certified_tangent_parameters.contains(parameter)
        {
            certified_tangent_parameters.push(parameter.clone());
        }
        let relation = match match regular_range {
            Some(range) => parallel
                .relation_to_supporting_line_on_regular_range_with_certified_contacts(
                    &line,
                    range,
                    certified_crossing,
                    &certified_tangent_parameters,
                    false,
                    &self.data.policy,
                ),
            None => parallel.relation_to_supporting_line_with_direction_and_certified_contacts(
                &line,
                &direction_x,
                &direction_y,
                certified_crossing,
                &certified_tangent_parameters,
                false,
                &self.data.policy,
            ),
        }
        .map_err(|cause| self.invalid(0, cause))?
        {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let contacts = match relation {
            BezierLineContactRelation::ControlHullDisjoint { .. }
            | BezierLineContactRelation::NoContact => {
                return Ok(Classification::Decided(Some(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: Vec::new(),
                })));
            }
            BezierLineContactRelation::OnSupportingLine => {
                return Ok(Classification::Decided(None));
            }
            BezierLineContactRelation::Contacts { contacts } => contacts,
        };
        let reversed_line = LineSeg2::try_new(line.end().clone(), line.start().clone())
            .map_err(|cause| self.invalid(0, cause))?;
        let mut retained_parameters = Vec::with_capacity(contacts.len());
        let mut retained_certified_tangencies = Vec::new();
        for contact in contacts {
            if contact.parameter().scalar().is_some_and(|parameter| {
                authored_contact
                    .as_ref()
                    .is_some_and(|(authored, _)| parameter == authored)
            }) {
                continue;
            }
            if let Some(certified) = contact.parameter().scalar().and_then(|parameter| {
                certified_tangent_contacts
                    .iter()
                    .find(|certified| certified.parameter() == parameter)
            }) {
                retained_certified_tangencies.push(*certified);
                continue;
            }
            let from_start = match match regular_range {
                Some(range) => parallel.supporting_line_parameter_order_on_regular_range(
                    contact.parameter(),
                    &line,
                    range,
                    &self.data.policy,
                ),
                None => parallel.supporting_line_parameter_order(
                    contact.parameter(),
                    &line,
                    &self.data.policy,
                ),
            }
            .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let from_end = match match regular_range {
                Some(range) => parallel.supporting_line_parameter_order_on_regular_range(
                    contact.parameter(),
                    &reversed_line,
                    range,
                    &self.data.policy,
                ),
                None => parallel.supporting_line_parameter_order(
                    contact.parameter(),
                    &reversed_line,
                    &self.data.policy,
                ),
            }
            .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(order) => order,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            if from_start == Ordering::Less || from_end == Ordering::Less {
                continue;
            }
            retained_parameters.push(contact.parameter().clone());
        }
        let mut result = match self.parallel_exact_parameter_pair_result(
            parallel,
            curve,
            retained_parameters,
            parallel_is_first,
        )? {
            Classification::Decided(Some(result)) => result,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for certified in retained_certified_tangencies {
            let (line_parameter, point) = match certified.line_endpoint() {
                BezierEndpoint::Start => (Real::zero(), line.start().clone()),
                BezierEndpoint::End => (Real::one(), line.end().clone()),
            };
            let parallel_parameter = BezierParameter2::Exact(certified.parameter().clone());
            let line_parameter = BezierParameter2::Exact(line_parameter);
            let (first_parameter, second_parameter) = if parallel_is_first {
                (parallel_parameter, line_parameter)
            } else {
                (line_parameter, parallel_parameter)
            };
            result
                .contacts
                .push(RegionPairContactEvidence::direct_bezier(
                    first_parameter,
                    second_parameter,
                    Some(CurvePoint2::from(point)),
                    false,
                    None,
                ));
        }
        Ok(Classification::Decided(Some(result)))
    }

    fn parallel_arc_pair_result(
        &self,
        parallel: &BezierParallel2,
        range: &CurveParameterRange2,
        curve: &BezierSubcurve2,
        parallel_is_first: bool,
    ) -> ExactCurveResult<Classification<Option<RegionPairResult>>> {
        let segment = match crate::bezier_region::materialized_native_subcurve_segment(
            curve,
            &self.data.policy,
        )
        .map_err(|cause| self.invalid(0, cause))?
        {
            Classification::Decided(segment) => segment,
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                return Ok(Classification::Decided(None));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Segment2::Arc(arc) = segment else {
            return Ok(Classification::Decided(None));
        };
        let certified_tangent_contacts = match curve {
            BezierSubcurve2::RationalQuadratic(curve) => curve.retained_circular_conic(),
            BezierSubcurve2::Rational(curve) => curve.retained_circular_conic(),
            BezierSubcurve2::Quadratic(_) | BezierSubcurve2::Cubic(_) => None,
        }
        .and_then(|circle| circle.tangent_contacts.as_deref())
        .into_iter()
        .flatten()
        .filter_map(|contact| match contact {
            crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(contact)
                if contact.parallel == *parallel =>
            {
                Some(contact)
            }
            crate::rational_bezier::RationalQuadraticCircleTangentContact2::Parallel(_)
            | crate::rational_bezier::RationalQuadraticCircleTangentContact2::Line { .. } => None,
        })
        .collect::<Vec<_>>();
        let certified_tangent_parameters = certified_tangent_contacts
            .iter()
            .map(|contact| {
                (
                    contact.parameter.clone(),
                    contact.eliminant_root_multiplicity,
                )
            })
            .collect::<Vec<_>>();
        let incidence = match parallel
            .circle_incidence(
                arc.center(),
                arc.radius_squared_ref(),
                range,
                &certified_tangent_parameters,
                &self.data.policy,
            )
            .map_err(|cause| self.invalid(0, cause))?
        {
            Classification::Decided(incidence) => incidence,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut parameters = Vec::with_capacity(incidence.len());
        for (parameter, crossing) in incidence {
            if let Some(contact) = parameter.scalar().and_then(|parameter| {
                certified_tangent_contacts
                    .iter()
                    .find(|contact| contact.parameter == *parameter)
            }) && self.data.policy.bounded_exact_predicate_pass(|| {
                arc.contains_sweep_point(&contact.point, &self.data.policy)
            }) == Classification::Decided(false)
            {
                // Circle incidence is already certified. A finite-sweep
                // exclusion needs neither another radius proof nor a conic
                // inverse at its possible affine infinity.
                continue;
            }
            parameters.push((parameter, crossing));
        }
        // The incidence result already owns radial crossing evidence, including
        // tangency. Use the same conic inverse for represented and selected
        // parameters instead of rebuilding scalar point/tangent intersections.
        let rational =
            RationalBezier2::try_from_subcurve(curve).map_err(|cause| self.invalid(0, cause))?;
        if matches!(
            rational
                .quadratic_homogeneous_controls(&self.data.policy)
                .map_err(|cause| self.invalid(0, cause))?,
            Classification::Decided(Some(_))
        ) {
            let mut contacts = Vec::with_capacity(parameters.len());
            for (parallel_parameter, radial_crossing_sign) in &parameters {
                let certified_contact = parallel_parameter.scalar().and_then(|parameter| {
                    certified_tangent_contacts
                        .iter()
                        .find(|contact| contact.parameter == *parameter)
                });
                if certified_contact.is_none()
                    && let Some(exact) = parallel_parameter.scalar()
                {
                    let point = parallel
                        .point_at(exact, &self.data.policy)
                        .map_err(|cause| self.invalid(0, cause))?;
                    // A finite-arc rejection is optional. Unresolved scalar
                    // coordinates retain the selected point's exact field.
                    if let Classification::Decided(point) = point
                        && self.data.policy.bounded_exact_predicate_pass(|| {
                            arc.contains_sweep_point(&point, &self.data.policy)
                        }) == Classification::Decided(false)
                    {
                        continue;
                    }
                }
                let point = certified_contact.map_or_else(
                    || {
                        CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new(
                            parallel.clone(),
                            parallel_parameter.clone(),
                            &self.data.policy,
                        ))
                    },
                    |contact| CurvePoint2::from(contact.point.clone()),
                );
                let other_parameter = if let Some(contact) = certified_contact
                    && contact.point == *rational.start()
                {
                    CurveParameter2::from(Real::zero())
                } else if let Some(contact) = certified_contact
                    && contact.point == *rational.end()
                {
                    CurveParameter2::from(Real::one())
                } else {
                    // Circle incidence already proves that this point lies on
                    // the conic. Its homogeneous inverse stays in the retained
                    // point field and decides the original closed unit chart;
                    // no independent image polynomial is needed for the cut.
                    match crate::bezier_offset::quadratic_conic_parameter_at_incident_point(
                        &point,
                        &rational,
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(0, cause))?
                    {
                        Classification::Decided(Some(parameter)) => parameter,
                        Classification::Decided(None) => continue,
                        Classification::Uncertain(_) => {
                            return Ok(Classification::Decided(None));
                        }
                    }
                };
                let parallel_parameter = CurveParameter2::from(parallel_parameter.clone());
                let (first_parameter, second_parameter) = if parallel_is_first {
                    (parallel_parameter, other_parameter)
                } else {
                    (other_parameter, parallel_parameter)
                };
                let tangent_cross_sign = radial_crossing_sign.map(|sign| {
                    if arc.is_clockwise() ^ !parallel_is_first {
                        match sign {
                            RealSign::Positive => RealSign::Negative,
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => RealSign::Zero,
                        }
                    } else {
                        sign
                    }
                });
                contacts.push(RegionPairContactEvidence::direct(
                    first_parameter,
                    second_parameter,
                    Some(point),
                    matches!(
                        tangent_cross_sign,
                        Some(RealSign::Positive | RealSign::Negative)
                    ),
                    tangent_cross_sign,
                ));
            }
            return Ok(Classification::Decided(Some(RegionPairResult {
                contacts,
                overlaps: Vec::new(),
                blockers: Vec::new(),
            })));
        }
        let mut retained_parameters = Vec::with_capacity(parameters.len());
        for (parameter, _) in parameters {
            if let Some(contact) = parameter.scalar().and_then(|parameter| {
                certified_tangent_contacts
                    .iter()
                    .find(|contact| contact.parameter == *parameter)
            }) {
                match arc.contains_point(&contact.point, &self.data.policy) {
                    Classification::Decided(true) => retained_parameters.push(parameter),
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                continue;
            }
            let Some(exact) = parameter.scalar() else {
                return Ok(Classification::Decided(None));
            };
            let point = match parallel
                .point_at(exact, &self.data.policy)
                .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match arc.contains_point(&point, &self.data.policy) {
                Classification::Decided(true) => retained_parameters.push(parameter),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        self.parallel_exact_parameter_pair_result(
            parallel,
            curve,
            retained_parameters,
            parallel_is_first,
        )
    }

    fn parallel_exact_parameter_pair_result(
        &self,
        parallel: &BezierParallel2,
        curve: &BezierSubcurve2,
        parallel_parameters: Vec<BezierParameter2>,
        parallel_is_first: bool,
    ) -> ExactCurveResult<Classification<Option<RegionPairResult>>> {
        let rational =
            RationalBezier2::try_from_subcurve(curve).map_err(|cause| self.invalid(0, cause))?;
        let mut result_contacts = Vec::with_capacity(parallel_parameters.len());
        for parallel_parameter in parallel_parameters {
            let Some(parallel_parameter_exact) = parallel_parameter.scalar() else {
                return Ok(Classification::Decided(None));
            };
            let point = match parallel.point_at(parallel_parameter_exact, &self.data.policy) {
                Ok(Classification::Decided(point)) => point,
                Ok(Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(reason));
                }
                Err(cause) => return Err(self.invalid(0, cause)),
            };
            let other_parameters = match rational
                .point_incidence_on_range(
                    &point,
                    &crate::CurveParameterRange2::unit(),
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(RationalBezierPointIncidence2::Parameters(parameters)) => {
                    parameters
                }
                Classification::Decided(RationalBezierPointIncidence2::EntireCurve) => {
                    return Ok(Classification::Decided(None));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let parallel_derivative = match parallel
                .derivative_at(parallel_parameter_exact, &self.data.policy)
                .map_err(|cause| self.invalid(0, cause))?
            {
                Classification::Decided(derivative) => derivative,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            for other_parameter in other_parameters {
                let Some(other_parameter_exact) = other_parameter.scalar() else {
                    return Ok(Classification::Decided(None));
                };
                let other_derivative = match rational
                    .derivative_at_classified(other_parameter_exact, &self.data.policy)
                {
                    Classification::Decided(derivative) => derivative,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let cross = parallel_derivative.dx() * other_derivative.dy()
                    - parallel_derivative.dy() * other_derivative.dx();
                // This is an optional transverse hint. An unresolved exact
                // zero already means no hint; consuming an approximate zero
                // would unnecessarily weaken all later retained contacts.
                let parallel_cross_other = self
                    .data
                    .policy
                    .bounded_exact_predicate_pass(|| real_sign(&cross, &self.data.policy));
                let tangent_cross_sign = parallel_cross_other.and_then(|sign| match sign {
                    RealSign::Positive | RealSign::Negative => Some(if parallel_is_first {
                        sign
                    } else {
                        match sign {
                            RealSign::Positive => RealSign::Negative,
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => unreachable!(),
                        }
                    }),
                    RealSign::Zero => None,
                });
                let (first_parameter, second_parameter) = if parallel_is_first {
                    (parallel_parameter.clone(), other_parameter)
                } else {
                    (other_parameter, parallel_parameter.clone())
                };
                result_contacts.push(RegionPairContactEvidence::direct_bezier(
                    first_parameter,
                    second_parameter,
                    Some(CurvePoint2::from(point.clone())),
                    tangent_cross_sign.is_some(),
                    tangent_cross_sign,
                ));
            }
        }
        Ok(Classification::Decided(Some(RegionPairResult {
            contacts: result_contacts,
            overlaps: Vec::new(),
            blockers: Vec::new(),
        })))
    }

    fn authored_carriers_are_adjacent(&self, pair: &RegionCarrierPair) -> bool {
        self.authored_carrier_shared_endpoints(pair.first_carrier_index, pair.second_carrier_index)
            .is_some()
    }

    /// Returns the authored endpoint shared by each carrier. `true` names its
    /// traversal start and `false` its traversal end.
    fn authored_carrier_shared_endpoints(
        &self,
        first: usize,
        second: usize,
    ) -> Option<(bool, bool)> {
        let first = &self.data.carriers[first];
        let second = &self.data.carriers[second];
        if first.operand != second.operand || first.loop_index != second.loop_index {
            return None;
        }
        let region = match first.operand {
            CurveRegionBooleanOperand2::First => self.data.first,
            CurveRegionBooleanOperand2::Second => self.data.second,
        };
        let fragment_count = region
            .boundary_loops()
            .get(first.loop_index)
            .map(|boundary| boundary.fragments().len())?;
        let first_precedes_second = first.fragment_index.checked_add(1)
            == Some(second.fragment_index)
            || (second.fragment_index == 0
                && first.fragment_index.checked_add(1) == Some(fragment_count));
        let second_precedes_first = second.fragment_index.checked_add(1)
            == Some(first.fragment_index)
            || (first.fragment_index == 0
                && second.fragment_index.checked_add(1) == Some(fragment_count));
        if first_precedes_second {
            Some((false, true))
        } else if second_precedes_first {
            Some((true, false))
        } else {
            None
        }
    }

    /// Recovers a full-circle endpoint contact from an adjacent sibling
    /// chart. Long selected arcs are stored as consecutive half-circle
    /// fragments; a curve adjacent to one half meets the same complete circle
    /// represented by the other half. Retain the sibling chart and both
    /// endpoint identities, with any additional contact proof: the authored
    /// angular parameter decides half-chart ownership without a new solve.
    fn authored_supporting_circle_endpoint(
        &self,
        cusp_index: usize,
        other_index: usize,
        qualifies: impl Fn(&crate::BezierAlgebraicCuspSemicircleFragment2, bool) -> bool,
    ) -> Option<(usize, bool, bool)> {
        let cusp = match &self.data.carriers.get(cusp_index)?.geometry {
            CurveSupport2::Circle(cusp) => cusp,
            _ => return None,
        };
        let other_carrier = self.data.carriers.get(other_index)?;
        let mut certified = None;
        for (candidate_index, candidate) in self.data.carriers.iter().enumerate() {
            let CurveSupport2::Circle(candidate_cusp) = &candidate.geometry else {
                continue;
            };
            if candidate.operand != other_carrier.operand
                || candidate.loop_index != other_carrier.loop_index
                || !cusp
                    .semicircle()
                    .shares_structural_supporting_circle(candidate_cusp.semicircle())
            {
                continue;
            }
            let Some((candidate_at_start, other_at_start)) =
                self.authored_carrier_shared_endpoints(candidate_index, other_index)
            else {
                continue;
            };
            if !qualifies(candidate_cusp, candidate_at_start) {
                continue;
            }
            match certified {
                Some((_, _, previous)) if previous != other_at_start => return None,
                Some(_) => {}
                None => certified = Some((candidate_index, candidate_at_start, other_at_start)),
            }
        }
        certified
    }

    fn algebraic_chord_linear_bezier_pair_result(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        curve: &BezierSubcurve2,
        curve_index: usize,
    ) -> ExactCurveResult<Option<RegionPairResult>> {
        let Some(chord_line) = chord.exact_line() else {
            return Ok(None);
        };
        let rational = RationalBezier2::try_from_subcurve(curve)
            .map_err(|cause| self.invalid(curve_index, cause))?;
        let Some(curve_line) = rational.exact_linear_parameterization_line() else {
            return Ok(None);
        };
        let relation = chord_line
            .intersect_line(&curve_line, &self.data.policy)
            .map_err(|cause| self.invalid(chord_index, cause))?;
        let blocker = |reason| RegionPairResult {
            contacts: Vec::new(),
            overlaps: Vec::new(),
            blockers: vec![RegionPairBlocker::Uncertain(reason)],
        };
        let chord_parameter = |point: &crate::Point2| match chord
            .parameter_at_certified_point(CurvePoint2::from(point.clone()), &self.data.policy)
            .map_err(|cause| self.invalid(chord_index, cause))?
        {
            Classification::Decided(Some(parameter)) => Ok(Classification::Decided(
                CurveParameter2::from_algebraic_chord(parameter),
            )),
            Classification::Decided(None) => Err(self.invalid(
                chord_index,
                CurveError::Topology(
                    "an exact chord support contact was outside its finite chord".into(),
                ),
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        };
        let chord_is_first = chord_index == pair.first_carrier_index;
        let result = match relation {
            crate::LineLineIntersection::None => RegionPairResult::empty(),
            crate::LineLineIntersection::Uncertain { reason } => blocker(reason),
            crate::LineLineIntersection::Point { point, b_param, .. } => {
                if self.authored_carriers_are_adjacent(pair) {
                    RegionPairResult::empty()
                } else {
                    let chord_parameter = match chord_parameter(&point)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => return Ok(Some(blocker(reason))),
                    };
                    let curve_parameter = CurveParameter2::from(BezierParameter2::Exact(b_param));
                    let (chord_dx, chord_dy) = chord_line.delta();
                    let (curve_dx, curve_dy) = curve_line.delta();
                    let cross = Real::diff_of_products(&chord_dx, &curve_dy, &chord_dy, &curve_dx);
                    let Some(cross_sign) = real_sign(&cross, &self.data.policy) else {
                        return Ok(Some(blocker(UncertaintyReason::RealSign)));
                    };
                    let cross_sign = orient_tangent_cross_sign(cross_sign, chord_is_first);
                    let (first_parameter, second_parameter) = if chord_is_first {
                        (chord_parameter, curve_parameter)
                    } else {
                        (curve_parameter, chord_parameter)
                    };
                    RegionPairResult {
                        contacts: vec![RegionPairContactEvidence::direct(
                            first_parameter,
                            second_parameter,
                            Some(CurvePoint2::from(point)),
                            cross_sign != RealSign::Zero,
                            Some(cross_sign),
                        )],
                        overlaps: Vec::new(),
                        blockers: Vec::new(),
                    }
                }
            }
            crate::LineLineIntersection::Overlap {
                segment, b_range, ..
            } => {
                let chord_start = match chord_parameter(segment.start())? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => return Ok(Some(blocker(reason))),
                };
                let chord_end = match chord_parameter(segment.end())? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => return Ok(Some(blocker(reason))),
                };
                let chord_range = CurveParameterRange2::new_validated(chord_start, chord_end);
                let (curve_start, curve_end, orientation) =
                    match compare_reals(b_range.start(), b_range.end(), &self.data.policy) {
                        Some(Ordering::Less) => (
                            b_range.start().clone(),
                            b_range.end().clone(),
                            CurveOverlapOrientation2::Same,
                        ),
                        Some(Ordering::Greater) => (
                            b_range.end().clone(),
                            b_range.start().clone(),
                            CurveOverlapOrientation2::Reversed,
                        ),
                        Some(Ordering::Equal) => {
                            return Err(self.invalid(
                                curve_index,
                                CurveError::Topology(
                                    "a positive-length exact line overlap had zero parameter range"
                                        .into(),
                                ),
                            ));
                        }
                        None => return Ok(Some(blocker(UncertaintyReason::Ordering))),
                    };
                let curve_range = CurveParameterRange2::new_validated(
                    CurveParameter2::from(BezierParameter2::Exact(curve_start)),
                    CurveParameter2::from(BezierParameter2::Exact(curve_end)),
                );
                let correspondence = CurveOverlapCorrespondence2::ChordRational {
                    source: Arc::new(BezierAlgebraicChordRationalOverlap2::from_certified_ranges(
                        chord.clone(),
                        rational.clone(),
                        [chord_range.start(), chord_range.end()].map(|p| {
                            p.as_algebraic_chord()
                                .expect("certified chord range")
                                .clone()
                        }),
                        CurveParameterRange2::new_validated(
                            b_range.start().clone().into(),
                            b_range.end().clone().into(),
                        ),
                        orientation,
                    )),
                    chord_first: chord_is_first,
                };
                let (first_range, second_range) = if chord_is_first {
                    (chord_range, curve_range)
                } else {
                    (curve_range, chord_range)
                };
                RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: vec![CurveIntersectionOverlap2 {
                        first_span_index: 0,
                        second_span_index: 0,
                        endpoint_inclusion: [true, true],
                        first_range,
                        second_range,
                        orientation,
                        parameter_correspondence: correspondence,
                    }],
                    blockers: Vec::new(),
                }
            }
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair",
            "exact-linear-bezier",
        );
        Ok(Some(result))
    }

    /// Replays finite chord contacts through an authored adjacent Bezier whose
    /// image is shared with the other carrier.
    ///
    /// A retained chord endpoint and its adjacent boundary-carrier endpoint
    /// are already the same exact topology vertex.  Mapping that Bezier
    /// parameter through a certified rational-image overlap therefore gives
    /// an exact parameter for the chord endpoint on the other carrier without
    /// comparing independently adjoined point-coordinate fields.  The
    /// supporting-line kernel remains the completeness authority: this path
    /// succeeds only when its complete finite contact set matches those
    /// transported endpoints one-to-one.
    fn algebraic_chord_shared_image_endpoint_pair_result(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        target: &RationalBezier2,
        target_index: usize,
    ) -> ExactCurveResult<Option<RegionPairResult>> {
        let chord_carrier = &self.data.carriers[chord_index];
        let target_carrier = &self.data.carriers[target_index];
        if chord_carrier.operand == target_carrier.operand {
            return Ok(None);
        }
        let region = match chord_carrier.operand {
            CurveRegionBooleanOperand2::First => self.data.first,
            CurveRegionBooleanOperand2::Second => self.data.second,
        };
        let Some(fragment_count) = region
            .boundary_loops()
            .get(chord_carrier.loop_index)
            .map(|boundary| boundary.fragments().len())
        else {
            return Ok(None);
        };
        if fragment_count < 2 {
            return Ok(None);
        }
        let predecessor_fragment = if chord_carrier.fragment_index == 0 {
            fragment_count - 1
        } else {
            chord_carrier.fragment_index - 1
        };
        let successor_fragment = (chord_carrier.fragment_index + 1) % fragment_count;
        let adjacent_carrier = |fragment_index| {
            self.data.carriers.iter().enumerate().find(|(_, carrier)| {
                carrier.operand == chord_carrier.operand
                    && carrier.loop_index == chord_carrier.loop_index
                    && carrier.fragment_index == fragment_index
            })
        };

        let mut mapped_endpoints = Vec::with_capacity(2);
        for (fragment_index, source_parameter, chord_parameter, point) in [
            (
                predecessor_fragment,
                true,
                carrier_traversal_start(chord_carrier),
                chord.start(),
            ),
            (
                successor_fragment,
                false,
                carrier_traversal_end(chord_carrier),
                chord.end(),
            ),
        ] {
            let Some((source_index, source_carrier)) = adjacent_carrier(fragment_index) else {
                continue;
            };
            let CurveSupport2::Bezier(source_curve) = &source_carrier.geometry else {
                continue;
            };
            let source_parameter = if source_parameter {
                carrier_traversal_end(source_carrier)
            } else {
                carrier_traversal_start(source_carrier)
            };
            let Some(source_parameter) = source_parameter.as_bezier_parameter() else {
                continue;
            };
            let source = RationalBezier2::try_from_subcurve(source_curve)
                .map_err(|cause| self.invalid(source_index, cause))?;
            let target_parameter =
                match RationalBezierOverlapParameterCorrespondence2::map_parameter_between_curves(
                    &source,
                    target,
                    source_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(source_index, cause))?
                {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) | Classification::Uncertain(_) => continue,
                };
            let target_region_parameter = CurveParameter2::from(target_parameter.clone());
            match parameter_in_carrier(&target_region_parameter, target_carrier, &self.data.policy)
            {
                Ok(true) => mapped_endpoints.push((
                    target_parameter,
                    chord_parameter.clone(),
                    point.clone(),
                )),
                Ok(false) | Err(ExactCurveError::Blocked(_)) => {}
                Err(error) => return Err(error),
            }
        }
        if mapped_endpoints.is_empty() {
            return Ok(None);
        }

        let Some(support_line) = chord
            .exact_line()
            .or_else(|| chord.strict_provenance_support_line(&self.data.policy))
        else {
            return Ok(None);
        };
        let line_contacts = match target
            .relation_to_line_with_contacts(&support_line, &self.data.policy)
        {
            Classification::Decided(
                BezierLineContactRelation::ControlHullDisjoint { .. }
                | BezierLineContactRelation::NoContact,
            ) => Vec::new(),
            Classification::Decided(BezierLineContactRelation::Contacts { contacts }) => contacts,
            Classification::Decided(BezierLineContactRelation::OnSupportingLine)
            | Classification::Uncertain(_) => return Ok(None),
        };
        let mut finite_contacts = Vec::with_capacity(line_contacts.len());
        for contact in line_contacts {
            let parameter = CurveParameter2::from(contact.parameter().clone());
            match parameter_in_carrier(&parameter, target_carrier, &self.data.policy) {
                Ok(true) => finite_contacts.push(contact),
                Ok(false) => {}
                Err(ExactCurveError::Blocked(_)) => return Ok(None),
                Err(error) => return Err(error),
            }
        }
        if finite_contacts.len() != mapped_endpoints.len() {
            return Ok(None);
        }

        let chord_is_first = chord_index == pair.first_carrier_index;
        let mut matched = vec![false; mapped_endpoints.len()];
        let mut contacts = Vec::with_capacity(finite_contacts.len());
        for contact in finite_contacts {
            let mut match_index = None;
            for (index, (parameter, _, _)) in mapped_endpoints.iter().enumerate() {
                if matched[index] {
                    continue;
                }
                match parameter
                    .same_value(contact.parameter(), &self.data.policy)
                    .map_err(|cause| self.invalid(target_index, cause))?
                {
                    Classification::Decided(true) if match_index.is_none() => {
                        match_index = Some(index);
                    }
                    Classification::Decided(true) | Classification::Uncertain(_) => {
                        return Ok(None);
                    }
                    Classification::Decided(false) => {}
                }
            }
            let Some(match_index) = match_index else {
                return Ok(None);
            };
            matched[match_index] = true;
            let (_, chord_parameter, point) = &mapped_endpoints[match_index];
            let chord_cross_target = match contact.crossing_direction() {
                Some(BezierLineCrossingDirection::NegativeToPositive) => RealSign::Positive,
                Some(BezierLineCrossingDirection::PositiveToNegative) => RealSign::Negative,
                None => RealSign::Zero,
            };
            let tangent_cross_sign = orient_tangent_cross_sign(chord_cross_target, chord_is_first);
            let target_parameter = CurveParameter2::from(contact.parameter().clone());
            let (first_parameter, second_parameter) = if chord_is_first {
                (chord_parameter.clone(), target_parameter)
            } else {
                (target_parameter, chord_parameter.clone())
            };
            contacts.push(RegionPairContactEvidence::direct(
                first_parameter,
                second_parameter,
                Some(point.clone()),
                tangent_cross_sign != RealSign::Zero,
                Some(tangent_cross_sign),
            ));
        }
        if matched.iter().any(|matched| !matched) {
            return Ok(None);
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair",
            "shared-image-endpoints",
        );
        Ok(Some(RegionPairResult {
            contacts,
            overlaps: Vec::new(),
            blockers: Vec::new(),
        }))
    }

    fn algebraic_chord_rational_pair_result(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        rational: &RationalBezier2,
        regular_component: Option<&BezierParallelRationalComponent2>,
        shared_source_parameter: Option<&CurveParameter2>,
    ) -> ExactCurveResult<Option<RegionPairResult>> {
        let other_index = if chord_index == pair.first_carrier_index {
            pair.second_carrier_index
        } else {
            pair.first_carrier_index
        };
        let collinear_support = if let Some(line) =
            regular_component.and_then(BezierParallelRationalComponent2::support_line)
        {
            matches!(
                self.data
                    .policy
                    .strict_predicate_pass(|| {
                        chord.has_non_collinear_support_with_exact_line(line, &self.data.policy)
                    })
                    .map_err(|cause| self.invalid(other_index, cause))?,
                Classification::Decided(false),
            )
        } else {
            false
        };
        let mut linear_intersections = None;
        if !collinear_support && let Some(component) = regular_component {
            // Unit-chart projection is complete only when it covers the
            // retained range. Exterior ranges use the common domain replay.
            let unit = CurveParameterRange2::unit();
            let domain = CurveParameterDomain2::new(&unit, None);
            for endpoint in [
                component.regular_range().start(),
                component.regular_range().end(),
            ] {
                if domain
                    .contains_finite_parameter(endpoint, &self.data.policy)
                    .map_err(|cause| self.invalid(other_index, cause))?
                    != Classification::Decided(true)
                {
                    return Ok(None);
                }
            }
            linear_intersections = chord
                .exact_linear_rational_intersections(rational, &self.data.policy)
                .map_err(|cause| self.invalid(other_index, cause))?;
        }
        let intersections = if collinear_support {
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair",
                "certified-rational-support-collinear",
            );
            let component =
                regular_component.expect("the regular component certified its line support");
            chord
                .collinear_rational_intersections_on_regular_component(
                    component,
                    shared_source_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(other_index, cause))?
        } else if let Some(mut intersections) = linear_intersections {
            // Two finite straight supports have at most the owned seam as
            // an isolated contact when their authored carriers are adjacent.
            // Positive overlaps retain their full correspondence.
            if self.authored_carriers_are_adjacent(pair)
                && let BezierAlgebraicChordRationalIntersections2::Contacts(contacts) =
                    &mut intersections
            {
                contacts.clear();
            }
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair",
                "exact-linear-parallel-chord-authority",
            );
            Classification::Decided(intersections)
        } else {
            chord
                .rational_intersections(
                    rational,
                    &CurveParameterRange2::new_validated(
                        self.data.carriers[other_index].start.clone(),
                        self.data.carriers[other_index].end.clone(),
                    ),
                    shared_source_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(other_index, cause))?
        };
        let complete = match intersections {
            Classification::Decided(BezierAlgebraicChordRationalIntersections2::Contacts(
                contacts,
            )) => Some((contacts, Vec::new())),
            Classification::Decided(BezierAlgebraicChordRationalIntersections2::Overlaps(
                overlaps,
            )) => Some((Vec::new(), overlaps)),
            Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::ContactsAndOverlaps {
                    contacts,
                    overlaps,
                },
            ) => Some((contacts, overlaps)),
            Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::DegenerateProjection,
            ) => {
                return Ok(Some(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: vec![RegionPairBlocker::Uncertain(UncertaintyReason::Boundary)],
                }));
            }
            Classification::Decided(
                BezierAlgebraicChordRationalIntersections2::NotSourceRelated,
            ) => None,
            Classification::Uncertain(reason) => {
                return Ok(Some(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: vec![RegionPairBlocker::Uncertain(reason)],
                }));
            }
        };
        let Some((contacts, overlaps)) = complete else {
            return Ok(None);
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair",
            if overlaps.is_empty() {
                if self.authored_carriers_are_adjacent(pair) {
                    "adjacent-source-complete"
                } else {
                    "source-complete"
                }
            } else {
                "collinear-overlap-complete"
            },
        );
        let chord_is_first = chord_index == pair.first_carrier_index;
        let contacts = contacts
            .into_iter()
            .map(|contact| {
                let tangent_cross_sign = if chord_is_first {
                    contact.tangent_cross_sign()
                } else {
                    match contact.tangent_cross_sign() {
                        RealSign::Positive => RealSign::Negative,
                        RealSign::Negative => RealSign::Positive,
                        RealSign::Zero => RealSign::Zero,
                    }
                };
                let chord_parameter =
                    CurveParameter2::from_algebraic_chord(contact.chord_parameter().clone());
                let other_parameter = contact.other_parameter().clone();
                let (first_parameter, second_parameter) = if chord_is_first {
                    (chord_parameter, other_parameter)
                } else {
                    (other_parameter, chord_parameter)
                };
                RegionPairContactEvidence::direct(
                    first_parameter,
                    second_parameter,
                    Some(contact.point().clone()),
                    tangent_cross_sign != RealSign::Zero,
                    Some(tangent_cross_sign),
                )
            })
            .collect();
        let overlaps = overlaps
            .into_iter()
            .map(|overlap| {
                let [chord_start, chord_end] = overlap.chord_range();
                let chord_range = CurveParameterRange2::new_validated(
                    CurveParameter2::from_algebraic_chord(chord_start.clone()),
                    CurveParameter2::from_algebraic_chord(chord_end.clone()),
                );
                let source_range = overlap.source_range().clone();
                let orientation = overlap.orientation();
                let (first_range, second_range) = if chord_is_first {
                    (chord_range, source_range)
                } else {
                    (source_range, chord_range)
                };
                CurveIntersectionOverlap2 {
                    first_span_index: 0,
                    second_span_index: 0,
                    endpoint_inclusion: [true, true],
                    parameter_correspondence: CurveOverlapCorrespondence2::ChordRational {
                        source: Arc::new(overlap),
                        chord_first: chord_is_first,
                    },
                    first_range,
                    second_range,
                    orientation,
                }
            })
            .collect();
        Ok(Some(RegionPairResult {
            contacts,
            overlaps,
            blockers: Vec::new(),
        }))
    }

    fn algebraic_chord_parallel_pair_result(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        parallel: &BezierParallel2,
        parallel_index: usize,
    ) -> ExactCurveResult<RegionPairResult> {
        let parallel_carrier = &self.data.carriers[parallel_index];
        let retained_range = CurveParameterRange2::new_validated(
            parallel_carrier.start.clone(),
            parallel_carrier.end.clone(),
        );
        let retained_contact_result = |contacts: Vec<
            crate::bezier_offset::BezierAlgebraicChordParallelContact2,
        >| {
            let chord_is_first = chord_index == pair.first_carrier_index;
            let contacts = contacts
                .into_iter()
                .map(|contact| {
                    let tangent_cross_sign =
                        orient_tangent_cross_sign(contact.tangent_cross_sign(), chord_is_first);
                    let chord_parameter =
                        CurveParameter2::from_algebraic_chord(contact.chord_parameter().clone());
                    let parallel_parameter = contact.parallel_parameter().clone();
                    let (first_parameter, second_parameter) = if chord_is_first {
                        (chord_parameter, parallel_parameter)
                    } else {
                        (parallel_parameter, chord_parameter)
                    };
                    RegionPairContactEvidence::direct(
                        first_parameter,
                        second_parameter,
                        Some(contact.point().clone()),
                        tangent_cross_sign != RealSign::Zero,
                        Some(tangent_cross_sign),
                    )
                })
                .collect();
            RegionPairResult {
                contacts,
                overlaps: Vec::new(),
                blockers: Vec::new(),
            }
        };
        let retained_monotone_contact_result =
            |contact: crate::bezier_offset::BezierAlgebraicChordRetainedParallelContact2| {
                let chord_is_first = chord_index == pair.first_carrier_index;
                let tangent_cross_sign =
                    orient_tangent_cross_sign(contact.tangent_cross_sign(), chord_is_first);
                let chord_parameter =
                    CurveParameter2::from_algebraic_chord(contact.chord_parameter().clone());
                let parallel_parameter = contact.parallel_parameter().clone();
                let (first_parameter, second_parameter) = if chord_is_first {
                    (chord_parameter, parallel_parameter)
                } else {
                    (parallel_parameter, chord_parameter)
                };
                RegionPairResult {
                    contacts: vec![RegionPairContactEvidence::direct(
                        first_parameter,
                        second_parameter,
                        Some(contact.point().clone()),
                        true,
                        Some(tangent_cross_sign),
                    )],
                    overlaps: Vec::new(),
                    blockers: Vec::new(),
                }
            };
        // Nonadjacent carriers have no authored endpoint to discharge.  Ask
        // the authoritative retained-support kernel first; monotonicity is a
        // completeness fallback for a support projection that stays blocked,
        // not a reason to spend exponential endpoint-refinement work before a
        // complete support answer that is already available.
        let mut authoritative_support_result = if self.authored_carriers_are_adjacent(pair) {
            None
        } else {
            Some(self.algebraic_chord_parallel_support_pair_result(
                pair,
                chord,
                chord_index,
                parallel,
                parallel_index,
            )?)
        };
        if authoritative_support_result
            .as_ref()
            .is_some_and(|result| result.blockers.is_empty())
        {
            return Ok(authoritative_support_result
                .take()
                .expect("the complete support result was retained above"));
        }
        {
            let monotonic = chord
                .parallel_tangent_cross_sign_on_region_range(
                    parallel,
                    &retained_range,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(parallel_index, cause))?;
            if let Classification::Decided(
                monotonic_sign @ (RealSign::Positive | RealSign::Negative),
            ) = monotonic
            {
                if self.authored_carriers_are_adjacent(pair) {
                    // The authored chain already owns one common endpoint. A
                    // strict support-incidence derivative over the complete
                    // retained span proves that this is its only contact.
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair",
                        "adjacent-parallel-monotone-complete",
                    );
                    return Ok(RegionPairResult::empty());
                }
                let endpoint_side = |parameter: &CurveParameter2| {
                    self.data.policy.strict_predicate_pass(|| {
                        let point = match parallel.point_evidence_on_regular_range(
                            parameter,
                            &retained_range,
                            &self.data.policy,
                        )? {
                            Classification::Decided(point) => point,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        #[cfg(test)]
                        let debug_kind = |point: &CurvePoint2| {
                            match point {
                                CurvePoint2(CurvePointData2::Exact(_)) => "exact",
                                CurvePoint2(CurvePointData2::Algebraic(_)) => {
                                    "algebraic"
                                }
                                CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => {
                                    "pair"
                                }
                                CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => {
                                    "cusp"
                                }
                                CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => {
                                    "derived"
                                }
                                CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => {
                                    "parallel"
                                }
                                CurvePoint2(CurvePointData2::AnalyticParallel(_)) => {
                                    "analytic"
                                }
                                CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                                    "similarity"
                                }
                            }
                        };
                        let retained_side =
                            chord.retained_procedural_point_side(&point, &self.data.policy)?;
                        #[cfg(test)]
                        if std::env::var_os("HYPERCURVE_DEBUG_PARALLEL_ENDPOINT_SIDE").is_some() {
                            eprintln!(
                                "parallel endpoint side point={} chord=({},{}) retained={retained_side:?}",
                                debug_kind(&point),
                                debug_kind(chord.start()),
                                debug_kind(chord.end()),
                            );
                        }
                        if let Some(side) = retained_side {
                            return Ok(Classification::Decided(side));
                        }
                        let interval = chord.strict_oriented_side_by_local_interval_refinement(
                            &point,
                            &self.data.policy,
                        )?;
                        #[cfg(test)]
                        if std::env::var_os("HYPERCURVE_DEBUG_PARALLEL_ENDPOINT_SIDE").is_some() {
                            eprintln!("parallel endpoint local interval={interval:?}");
                        }
                        if matches!(interval, Classification::Decided(_)) {
                            return Ok(interval);
                        }
                        if chord.certified_unit_tangent().is_some() {
                            let certified = chord.certified_tangent_side(&point, &self.data.policy);
                            if matches!(certified, Classification::Decided(_)) {
                                return Ok(certified);
                            }
                        }
                        Ok(Classification::Uncertain(UncertaintyReason::Predicate))
                    })
                };
                let sides = [
                    endpoint_side(retained_range.start()),
                    endpoint_side(retained_range.end()),
                ];
                let mut sides = match sides {
                    [Ok(first), Ok(second)] => [first, second],
                    [Err(cause), _] | [_, Err(cause)] => {
                        return Err(self.invalid(chord_index, cause));
                    }
                };
                // A strict derivative sign orders the two endpoint
                // incidences. If one retained endpoint has a nonzero side and
                // moving to the unknown endpoint changes incidence in that
                // same direction, the unknown endpoint has the same side.
                // This consumes only monotonicity and one compact contact
                // certificate; no endpoint coordinate is materialized.
                for known_index in 0..2 {
                    let unknown_index = 1 - known_index;
                    let Classification::Decided(known_side) = &sides[known_index] else {
                        continue;
                    };
                    let known_side = *known_side;
                    if matches!(sides[unknown_index], Classification::Decided(_)) {
                        continue;
                    }
                    let parameter_order = match retained_range
                        .start()
                        .cmp_by_refinement(retained_range.end(), &self.data.policy)
                        .map_err(|cause| self.invalid(parallel_index, cause))?
                    {
                        Classification::Decided(
                            order @ (std::cmp::Ordering::Less | std::cmp::Ordering::Greater),
                        ) => order,
                        Classification::Decided(std::cmp::Ordering::Equal)
                        | Classification::Uncertain(_) => continue,
                    };
                    let end_minus_start = if parameter_order == std::cmp::Ordering::Less {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    };
                    let unknown_minus_known = if unknown_index == 1 {
                        end_minus_start
                    } else {
                        match end_minus_start {
                            RealSign::Positive => RealSign::Negative,
                            RealSign::Negative => RealSign::Positive,
                            RealSign::Zero => unreachable!("a strict parameter order is nonzero"),
                        }
                    };
                    let difference_sign = if monotonic_sign == unknown_minus_known {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    };
                    let known_sign = match known_side {
                        LineSide::Left => RealSign::Positive,
                        LineSide::On => RealSign::Zero,
                        LineSide::Right => RealSign::Negative,
                    };
                    if known_sign == RealSign::Zero || known_sign == difference_sign {
                        sides[unknown_index] =
                            Classification::Decided(LineSide::from_real_sign(difference_sign));
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-pair",
                            "parallel-monotone-endpoint-order",
                        );
                    }
                }
                if matches!(
                    sides,
                    [
                        Classification::Decided(LineSide::Left),
                        Classification::Decided(LineSide::Left)
                    ] | [
                        Classification::Decided(LineSide::Right),
                        Classification::Decided(LineSide::Right)
                    ]
                ) {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair",
                        "parallel-monotone-one-sided",
                    );
                    return Ok(RegionPairResult::empty());
                }
                if let [
                    Classification::Decided(first @ (LineSide::Left | LineSide::Right)),
                    Classification::Decided(second @ (LineSide::Left | LineSide::Right)),
                ] = sides
                    && first != second
                {
                    match chord
                        .retained_monotone_parallel_contact_on_region_range(
                            parallel,
                            &retained_range,
                            [first, second],
                            monotonic_sign,
                            &self.data.policy,
                        )
                        .map_err(|cause| self.invalid(parallel_index, cause))?
                    {
                        Classification::Decided(Some(contact)) => {
                            return Ok(retained_monotone_contact_result(contact));
                        }
                        Classification::Decided(None) => {
                            return Ok(RegionPairResult::empty());
                        }
                        Classification::Uncertain(_) => {}
                    }
                }
            }
        }
        // Carrier representation is structural. Probe it under STRICT so
        // APPROXIMATE_512 remains terminal evidence rather than dispatch. A
        // source-stationary line can have a different exact rational component
        // on each regular side; that branch component preserves the authored
        // analytic parameter and therefore enters the same rational overlap
        // authority as an ordinary PH carrier.
        if let Classification::Decided(Some(rational)) = parallel
            .exact_rational_parallel_component_on_regular_range(
                &retained_range,
                &CurveContext::STRICT,
            )
            .map_err(|cause| self.invalid(parallel_index, cause))?
        {
            let shared_source_parameter = self
                .authored_carrier_shared_endpoints(
                    pair.first_carrier_index,
                    pair.second_carrier_index,
                )
                .map(|(first_at_start, second_at_start)| {
                    let parallel_at_start = if parallel_index == pair.first_carrier_index {
                        first_at_start
                    } else {
                        second_at_start
                    };
                    if parallel_at_start {
                        carrier_traversal_start(parallel_carrier)
                    } else {
                        carrier_traversal_end(parallel_carrier)
                    }
                });
            if let Some(result) = self.algebraic_chord_rational_pair_result(
                pair,
                chord,
                chord_index,
                rational.curve(),
                Some(&rational),
                shared_source_parameter,
            )? {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "analytic-parallel-strict-rational-component",
                );
                if result.blockers.is_empty() {
                    return Ok(result);
                }
                // A STRICT rational component is a representation fast path,
                // not a completeness boundary. Selected endpoint fields can
                // make its general rational replay inconclusive even though
                // the retained analytic support decides the same carrier.
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "rational-component-fallback",
                );
            }
        }
        let support_result = match authoritative_support_result.take() {
            Some(result) => result,
            None => self.algebraic_chord_parallel_support_pair_result(
                pair,
                chord,
                chord_index,
                parallel,
                parallel_index,
            )?,
        };
        if support_result.blockers.is_empty() {
            return Ok(support_result);
        }
        {
            let blocker = |reason| RegionPairResult {
                contacts: Vec::new(),
                overlaps: Vec::new(),
                blockers: vec![RegionPairBlocker::Uncertain(reason)],
            };
            let retained = chord
                .parallel_intersections_on_regular_range(
                    parallel,
                    &retained_range,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(parallel_index, cause))?;
            let contacts = match retained {
                Classification::Decided(BezierAlgebraicChordParallelIntersections2::Contacts(
                    contacts,
                )) => Some(contacts),
                Classification::Decided(
                    BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent {
                        ..
                    }
                    | BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                ) if chord.exact_line().is_none() => {
                    return Ok(blocker(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) if chord.exact_line().is_none() => {
                    return Ok(blocker(reason));
                }
                Classification::Decided(
                    BezierAlgebraicChordParallelIntersections2::CoincidentSupportComponent {
                        ..
                    }
                    | BezierAlgebraicChordParallelIntersections2::DegenerateProjection,
                )
                | Classification::Uncertain(_) => None,
            };
            if let Some(contacts) = contacts {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "analytic-parallel-retained-support",
                );
                return Ok(retained_contact_result(contacts));
            }
        }
        let Some(chord_line) = chord.exact_line() else {
            unreachable!("the retained-support path owns non-represented chords");
        };
        let line_curve =
            BezierSubcurve2::Quadratic(QuadraticBezier2::from_line_segment(chord_line.clone()));
        let chord_is_first = chord_index == pair.first_carrier_index;
        let parallel_is_first = parallel_index == pair.first_carrier_index;
        let blocker = |reason| RegionPairResult {
            contacts: Vec::new(),
            overlaps: Vec::new(),
            blockers: vec![RegionPairBlocker::Uncertain(reason)],
        };
        let chord_parameter = |point: CurvePoint2| match chord
            .parameter_at_certified_point(point, &self.data.policy)
            .map_err(|cause| self.invalid(chord_index, cause))?
        {
            Classification::Decided(Some(parameter)) => Ok(Classification::Decided(
                CurveParameter2::from_algebraic_chord(parameter),
            )),
            Classification::Decided(None) => Err(self.invalid(
                chord_index,
                CurveError::Topology(
                    "an analytic-parallel contact was outside its finite chord".into(),
                ),
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        };

        // Preserve the cheaper univariate supporting-line route whenever all
        // retained contacts have directly represented parallel parameters.
        let regular_range = parallel_carrier
            .start
            .as_bezier_parameter()
            .zip(parallel_carrier.end.as_bezier_parameter())
            .map(|_| {
                CurveParameterRange2::new_validated(
                    parallel_carrier.start.clone(),
                    parallel_carrier.end.clone(),
                )
            });
        match self.parallel_line_pair_result(
            pair,
            parallel,
            parallel_index,
            &line_curve,
            parallel_is_first,
            regular_range.as_ref(),
        )? {
            Classification::Decided(Some(mut result)) => {
                for contact in &mut result.contacts {
                    let Some(point) = contact.point.clone() else {
                        return Err(self.invalid(
                            parallel_index,
                            CurveError::Topology(
                                "a direct parallel/line contact lost its exact point evidence"
                                    .into(),
                            ),
                        ));
                    };
                    let parameter = match chord_parameter(point)? {
                        Classification::Decided(parameter) => parameter,
                        Classification::Uncertain(reason) => return Ok(blocker(reason)),
                    };
                    if chord_is_first {
                        contact.first_parameter = parameter;
                    } else {
                        contact.second_parameter = parameter;
                    }
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "analytic-parallel-line",
                );
                return Ok(result);
            }
            Classification::Decided(None) | Classification::Uncertain(_) => {}
        }

        let rational_line = RationalBezier2::try_from_subcurve(&line_curve)
            .map_err(|cause| self.invalid(chord_index, cause))?;
        let intersections = match parallel
            .intersections(&rational_line, &self.data.policy)
            .map_err(|cause| self.invalid(parallel_index, cause))?
        {
            Classification::Decided(intersections) => intersections,
            Classification::Uncertain(reason) => return Ok(blocker(reason)),
        };
        let mut contacts = Vec::with_capacity(intersections.contacts().len());
        for contact in intersections.contacts() {
            let chord_parameter = match chord_parameter(contact.point().clone())? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(blocker(reason)),
            };
            let parallel_parameter = CurveParameter2::from(contact.parallel_parameter().clone());
            let tangent_cross_sign = contact
                .tangent_cross_sign()
                .map(|sign| orient_tangent_cross_sign(sign, parallel_is_first));
            let (first_parameter, second_parameter) = if chord_is_first {
                (chord_parameter, parallel_parameter)
            } else {
                (parallel_parameter, chord_parameter)
            };
            contacts.push(RegionPairContactEvidence::direct(
                first_parameter,
                second_parameter,
                Some(contact.point().clone()),
                contact.is_certified_transverse(),
                tangent_cross_sign,
            ));
        }
        let mut overlaps = Vec::with_capacity(intersections.overlaps().len());
        for overlap in intersections.overlaps() {
            let chord_endpoint = |parameter: &BezierParameter2| {
                let point = match exact_contact_point_evidence(
                    &rational_line,
                    parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(chord_index, cause))?
                {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                chord_parameter(point)
            };
            let chord_start = match chord_endpoint(overlap.second_range().start())? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(blocker(reason)),
            };
            let chord_end = match chord_endpoint(overlap.second_range().end())? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Ok(blocker(reason)),
            };
            let chord_range = CurveParameterRange2::new_validated(chord_start, chord_end);
            let parallel_range =
                CurveParameterRange2::from_bezier_range(overlap.first_range().clone());
            let image = self
                .overlap_rational_image(parallel_index)?
                .ok_or_else(|| self.blocked(parallel_index, UncertaintyReason::Unsupported))?;
            let correspondence = CurveOverlapCorrespondence2::ChordRational {
                source: Arc::new(BezierAlgebraicChordRationalOverlap2::from_certified_ranges(
                    chord.clone(),
                    image,
                    [chord_range.start(), chord_range.end()].map(|p| {
                        p.as_algebraic_chord()
                            .expect("certified chord range")
                            .clone()
                    }),
                    parallel_range.clone(),
                    overlap.orientation(),
                )),
                chord_first: chord_is_first,
            };
            let (first_range, second_range) = if chord_is_first {
                (chord_range, parallel_range)
            } else {
                (parallel_range, chord_range)
            };
            overlaps.push(CurveIntersectionOverlap2 {
                first_span_index: 0,
                second_span_index: 0,
                endpoint_inclusion: [true, true],
                first_range,
                second_range,
                orientation: overlap.orientation(),
                parameter_correspondence: correspondence,
            });
        }
        let mut blockers = Vec::with_capacity(2);
        if !intersections.parameter_components().is_empty() {
            blockers.push(RegionPairBlocker::PointImageParameterComponent);
        }
        if !intersections.is_complete() {
            blockers.push(RegionPairBlocker::IncompleteReplay);
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair",
            "analytic-parallel-general",
        );
        Ok(RegionPairResult {
            contacts,
            overlaps,
            blockers,
        })
    }

    fn algebraic_chord_parallel_support_pair_result(
        &self,
        pair: &RegionCarrierPair,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        parallel: &BezierParallel2,
        parallel_index: usize,
    ) -> ExactCurveResult<RegionPairResult> {
        let blocker = |reason| RegionPairResult {
            contacts: Vec::new(),
            overlaps: Vec::new(),
            blockers: vec![RegionPairBlocker::Uncertain(reason)],
        };
        let parallel_carrier = &self.data.carriers[parallel_index];
        let regular_range = CurveParameterRange2::new_validated(
            parallel_carrier.start.clone(),
            parallel_carrier.end.clone(),
        );
        let Some(support_line) = chord
            .exact_line()
            .or_else(|| chord.strict_provenance_support_line(&self.data.policy))
        else {
            return Ok(blocker(UncertaintyReason::Unsupported));
        };
        let mut authored_direction = None;
        for contact in chord.parallel_tangent_contacts() {
            let tangent = match contact
                .parallel()
                .source_tangent_at(contact.parameter(), &self.data.policy)
                .map_err(|cause| self.invalid(chord_index, cause))?
            {
                Classification::Decided(tangent) => tangent,
                Classification::Uncertain(_) => continue,
            };
            authored_direction = Some(if contact.parallel_fragment_reversed() {
                (-tangent.0, -tangent.1)
            } else {
                tangent
            });
            break;
        }
        let (direction_x, direction_y) = if let Some(direction) =
            authored_direction.or_else(|| chord.certified_unit_tangent())
        {
            direction
        } else {
            // `strict_provenance_support_line` preserves the chord's
            // traversal orientation. Its exact nonzero delta is therefore
            // the direction authority for a canonicalized procedural
            // bevel even when no separately normalized unit tangent was
            // retained on the chord.
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "algebraic-chord-pair",
                "provenance-support-direction",
            );
            support_line.delta()
        };
        let directed_line = LineSeg2::try_new(
            support_line.start().clone(),
            support_line
                .start()
                .translated(direction_x.clone(), direction_y.clone()),
        )
        .map_err(|cause| self.invalid(chord_index, cause))?;
        let authored_contact = self.authored_parallel_support_contact(
            pair,
            parallel,
            parallel_index,
            &direction_x,
            &direction_y,
            Some(&regular_range),
        )?;
        let authored_crossing = authored_contact.as_ref().and_then(|(parameter, cross)| {
            let direction = match cross {
                RealSign::Positive => BezierLineCrossingDirection::NegativeToPositive,
                RealSign::Negative => BezierLineCrossingDirection::PositiveToNegative,
                RealSign::Zero => return None,
            };
            Some((parameter, direction))
        });
        let certified_tangent_contacts = chord
            .parallel_tangent_contacts()
            .iter()
            .filter(|contact| contact.parallel() == parallel)
            .collect::<Vec<_>>();
        let mut certified_tangent_parameters = certified_tangent_contacts
            .iter()
            .map(|contact| contact.parameter().clone())
            .collect::<Vec<_>>();
        if let Some((parameter, RealSign::Zero)) = &authored_contact
            && !certified_tangent_parameters.contains(parameter)
        {
            certified_tangent_parameters.push(parameter.clone());
        }
        let relation_on_retained_range = |deep_branch_refinement| {
            parallel.relation_to_supporting_line_on_regular_range_with_certified_contacts(
                &directed_line,
                &regular_range,
                authored_crossing,
                &certified_tangent_parameters,
                deep_branch_refinement,
                &self.data.policy,
            )
        };
        let relation = match relation_on_retained_range(false)
            .map_err(|cause| self.invalid(parallel_index, cause))?
        {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => {
                if reason != UncertaintyReason::RealSign {
                    return Ok(blocker(reason));
                }
                let unsigned = match parallel
                    .supporting_line_squared_incidence(
                        &directed_line,
                        &regular_range,
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(parallel_index, cause))?
                {
                    Classification::Decided(crate::BezierParallelIncidence2::Parameters(
                        parameters,
                    )) => parameters,
                    Classification::Decided(crate::BezierParallelIncidence2::EntireCurve)
                    | Classification::Uncertain(_) => return Ok(blocker(reason)),
                };
                let mut finite_candidate = false;
                for parameter in unsigned {
                    let mut disjoint = false;
                    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256] {
                        let (
                            Classification::Decided(point_bounds),
                            Classification::Decided(chord_bounds),
                        ) = (
                            parallel.point_bounds_at_parameter(
                                &parameter,
                                refinement_steps,
                                &self.data.policy,
                            ),
                            chord
                                .conservative_bounds_refined(refinement_steps, &self.data.policy)
                                .map_err(|cause| self.invalid(chord_index, cause))?,
                        )
                        else {
                            continue;
                        };
                        if point_bounds.overlaps(&chord_bounds, &self.data.policy)
                            == Classification::Decided(false)
                        {
                            disjoint = true;
                            break;
                        }
                    }
                    if !disjoint {
                        let selected_point =
                            crate::CurvePoint2::from(crate::BezierAnalyticParallelPoint2::new(
                                parallel.clone(),
                                parameter.clone(),
                                &self.data.policy,
                            ));
                        let selected_side = chord
                            .strict_oriented_side_by_fast_refinement(
                                &selected_point,
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(chord_index, cause))?;
                        if matches!(
                            selected_side,
                            Classification::Decided(
                                crate::classify::LineSide::Left | crate::classify::LineSide::Right
                            )
                        ) {
                            disjoint = true;
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "opposite-parallel-branch-by-retained-side",
                            );
                        }
                    }
                    if !disjoint {
                        finite_candidate = true;
                    }
                }
                if !finite_candidate {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-chord-pair",
                        "unsigned-support-candidates-outside-chord",
                    );
                    return Ok(RegionPairResult::empty());
                }
                match relation_on_retained_range(true)
                    .map_err(|cause| self.invalid(parallel_index, cause))?
                {
                    Classification::Decided(relation) => relation,
                    Classification::Uncertain(reason) => return Ok(blocker(reason)),
                }
            }
        };
        let line_contacts = match relation {
            BezierLineContactRelation::ControlHullDisjoint { .. }
            | BezierLineContactRelation::NoContact => return Ok(RegionPairResult::empty()),
            BezierLineContactRelation::OnSupportingLine => {
                return Ok(blocker(UncertaintyReason::Boundary));
            }
            BezierLineContactRelation::Contacts { contacts } => contacts,
        };
        let chord_is_first = chord_index == pair.first_carrier_index;
        let mut contacts = Vec::with_capacity(line_contacts.len());
        for contact in line_contacts {
            if contact.parameter().scalar().is_some_and(|parameter| {
                authored_contact
                    .as_ref()
                    .is_some_and(|(authored, _)| parameter == authored)
            }) {
                continue;
            }
            let certified = contact.parameter().scalar().and_then(|parameter| {
                certified_tangent_contacts
                    .iter()
                    .find(|certified| certified.parameter() == parameter)
            });
            let (point, chord_parameter) = if let Some(certified) = certified {
                match certified.line_endpoint() {
                    BezierEndpoint::Start => (chord.start().clone(), chord.start_parameter()),
                    BezierEndpoint::End => (chord.end().clone(), chord.end_parameter()),
                }
            } else {
                let point = match parallel
                    .point_evidence_on_regular_range(
                        &contact.parameter().clone().into(),
                        &regular_range,
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(parallel_index, cause))?
                {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => return Ok(blocker(reason)),
                };
                let chord_parameter = match chord
                    .parameter_at_certified_point(point.clone(), &self.data.policy)
                    .map_err(|cause| self.invalid(chord_index, cause))?
                {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(blocker(reason));
                    }
                };
                (point, chord_parameter)
            };
            let chord_cross_parallel = match contact.crossing_direction() {
                Some(BezierLineCrossingDirection::NegativeToPositive) => RealSign::Positive,
                Some(BezierLineCrossingDirection::PositiveToNegative) => RealSign::Negative,
                None => RealSign::Zero,
            };
            let tangent_cross_sign =
                orient_tangent_cross_sign(chord_cross_parallel, chord_is_first);
            let tangent_topology = if let Some(parallel_side_of_chord) = contact.tangent_side() {
                let tangent_relation = parallel
                    .vector_tangent_cross_and_dot_signs_on_regular_range(
                        &contact.parameter().clone().into(),
                        &direction_x,
                        &direction_y,
                        &regular_range,
                        &self.data.policy,
                    );
                let (cross, dot) =
                    match tangent_relation.map_err(|cause| self.invalid(parallel_index, cause))? {
                        Classification::Decided(signs) => signs,
                        Classification::Uncertain(reason) => return Ok(blocker(reason)),
                    };
                if cross != RealSign::Zero || dot == RealSign::Zero {
                    return Err(self.invalid(
                        parallel_index,
                        CurveError::Topology(
                            "supporting-line tangency disagreed with its tangent relation".into(),
                        ),
                    ));
                }
                let opposite = |side| match side {
                    LineSide::Left => LineSide::Right,
                    LineSide::Right => LineSide::Left,
                    LineSide::On => unreachable!("a tangent neighbor side is strict"),
                };
                let second_side_of_first = if chord_is_first {
                    parallel_side_of_chord
                } else if dot == RealSign::Positive {
                    opposite(parallel_side_of_chord)
                } else {
                    parallel_side_of_chord
                };
                Some((dot, second_side_of_first))
            } else {
                None
            };
            let chord_parameter = CurveParameter2::from_algebraic_chord(chord_parameter);
            let parallel_parameter = CurveParameter2::from(contact.parameter().clone());
            let (first_parameter, second_parameter) = if chord_is_first {
                (chord_parameter, parallel_parameter)
            } else {
                (parallel_parameter, chord_parameter)
            };
            let evidence = RegionPairContactEvidence::direct(
                first_parameter,
                second_parameter,
                Some(point),
                tangent_cross_sign != RealSign::Zero,
                Some(tangent_cross_sign),
            );
            contacts.push(match tangent_topology {
                Some((dot, side)) => evidence.with_tangent_topology(dot, side),
                None => evidence,
            });
        }
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "algebraic-chord-pair",
            "analytic-parallel-certified-support",
        );
        Ok(RegionPairResult {
            contacts,
            overlaps: Vec::new(),
            blockers: Vec::new(),
        })
    }

    fn algebraic_cusp_rational_pair_result(
        &self,
        pair: &RegionCarrierPair,
        cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
        rational: &RationalBezier2,
        cusp_is_first: bool,
    ) -> ExactCurveResult<RegionPairResult> {
        let other = &self.data.carriers[if cusp_is_first {
            pair.second_carrier_index
        } else {
            pair.first_carrier_index
        }];
        let range = CurveParameterRange2::new_validated(other.start.clone(), other.end.clone());
        let (intersections, parameter_map) = match cusp
            .semicircle()
            .rational_intersections_with_parameter_map(rational, &range, &self.data.policy)
            .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
        {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => {
                return Ok(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: vec![RegionPairBlocker::Uncertain(reason)],
                });
            }
        };
        match intersections {
            BezierAlgebraicCuspSemicircleRationalIntersections2::Mapped { contacts, overlaps } => {
                let mut retained = Vec::with_capacity(contacts.len());
                for contact in contacts {
                    let cusp_parameter =
                        contact.location.endpoint_parameter().unwrap_or_else(|| {
                            parameter_map
                                .as_ref()
                                .expect(
                                    "an interior cusp/rational contact retains its parameter map",
                                )
                                .contact_parameter(&contact)
                        });
                    let tangent_cross_sign =
                        orient_tangent_cross_sign(contact.tangent_cross_sign, cusp_is_first);
                    let (first_parameter, second_parameter) = if cusp_is_first {
                        (
                            CurveParameter2::from_algebraic_cusp(cusp_parameter),
                            contact.other_parameter,
                        )
                    } else {
                        (
                            contact.other_parameter,
                            CurveParameter2::from_algebraic_cusp(cusp_parameter),
                        )
                    };
                    retained.push(RegionPairContactEvidence::direct(
                        first_parameter,
                        second_parameter,
                        Some(contact.point),
                        tangent_cross_sign != RealSign::Zero,
                        Some(tangent_cross_sign),
                    ));
                }
                Ok(RegionPairResult {
                    contacts: retained,
                    overlaps: overlaps
                        .into_iter()
                        .map(|source| {
                            circle_overlap_evidence(
                                CurveCircleOverlap2::Mapped(source),
                                cusp_is_first,
                            )
                        })
                        .collect(),
                    blockers: Vec::new(),
                })
            }
            BezierAlgebraicCuspSemicircleRationalIntersections2::SelectedFiber {
                contacts,
                overlaps,
            } => Ok(selected_fiber_cusp_result(
                contacts,
                overlaps,
                cusp_is_first,
            )),
            BezierAlgebraicCuspSemicircleRationalIntersections2::DegenerateProjection => {
                Ok(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: vec![RegionPairBlocker::Uncertain(UncertaintyReason::Unsupported)],
                })
            }
        }
    }

    fn retained_cusp_chord_pair_result(
        &self,
        cusp: &crate::BezierAlgebraicCuspSemicircleFragment2,
        chord: &crate::BezierAlgebraicChord2,
        chord_index: usize,
        cusp_is_first: bool,
        contacts: Vec<crate::bezier_offset::BezierAlgebraicCuspSemicircleRetainedChordContact2>,
    ) -> ExactCurveResult<RegionPairResult> {
        let mut retained = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let tangent_cross_sign =
                orient_tangent_cross_sign(contact.tangent_cross_sign, cusp_is_first);
            let tangent_topology = if contact.tangent_cross_sign == RealSign::Zero {
                match contact
                    .tangent_topology(cusp.semicircle(), chord, &self.data.policy)
                    .map_err(|cause| self.invalid(chord_index, cause))?
                {
                    Classification::Decided(Some((dot, circle_side_of_chord))) => {
                        let opposite = |side| match side {
                            LineSide::Left => LineSide::Right,
                            LineSide::Right => LineSide::Left,
                            LineSide::On => LineSide::On,
                        };
                        let second_side_of_first = if cusp_is_first {
                            if dot == RealSign::Positive {
                                opposite(circle_side_of_chord)
                            } else {
                                circle_side_of_chord
                            }
                        } else {
                            circle_side_of_chord
                        };
                        (second_side_of_first != LineSide::On)
                            .then_some((dot, second_side_of_first))
                    }
                    Classification::Decided(None) | Classification::Uncertain(_) => None,
                }
            } else {
                None
            };
            let chord_parameter = CurveParameter2::from_algebraic_chord(contact.chord_parameter);
            let (first_parameter, second_parameter) = if cusp_is_first {
                (
                    CurveParameter2::from_algebraic_cusp(contact.cusp_parameter),
                    chord_parameter,
                )
            } else {
                (
                    chord_parameter,
                    CurveParameter2::from_algebraic_cusp(contact.cusp_parameter),
                )
            };
            let evidence = RegionPairContactEvidence::direct(
                first_parameter,
                second_parameter,
                Some(contact.point),
                tangent_cross_sign != RealSign::Zero,
                Some(tangent_cross_sign),
            );
            retained.push(match tangent_topology {
                Some((dot, side)) => evidence.with_tangent_topology(dot, side),
                None => evidence,
            });
        }
        Ok(RegionPairResult {
            contacts: retained,
            overlaps: Vec::new(),
            blockers: Vec::new(),
        })
    }

    fn overlap_rational_image(&self, index: usize) -> ExactCurveResult<Option<RationalBezier2>> {
        let carrier = &self.data.carriers[index];
        let image = match &carrier.geometry {
            CurveSupport2::Parallel(parallel) => parallel
                .exact_rational_parallel_component_on_regular_range(
                    &carrier.range(),
                    &self.data.policy.strict_counterpart(),
                )
                .map(|result| result.map(|image| image.map(|image| image.curve().clone()))),
            _ => carrier.geometry.exact_rational_component(&self.data.policy),
        }
        .map_err(|cause| self.invalid(index, cause))?;
        match image {
            Classification::Decided(image) => Ok(image),
            Classification::Uncertain(reason) => Err(self.blocked(index, reason)),
        }
    }

    fn analytic_component_overlaps(
        &self,
        pair: &RegionCarrierPair,
        components: &[BezierParameterComponentOverlap2],
        overlap: &RationalBezierIntersectionOverlap2,
        swapped: bool,
    ) -> ExactCurveResult<Vec<CurveIntersectionOverlap2>> {
        let (first_range, second_range) = if swapped {
            (overlap.second_range(), overlap.first_range())
        } else {
            (overlap.first_range(), overlap.second_range())
        };
        let mut sources = components
            .iter()
            .filter(|source| source.overlap() == overlap)
            .cloned()
            .map(|source| CurveOverlapCorrespondence2::ParameterComponent { source, swapped })
            .collect::<Vec<_>>();
        if sources.is_empty() {
            let first = self.overlap_rational_image(pair.first_carrier_index)?;
            let second = self.overlap_rational_image(pair.second_carrier_index)?;
            let (first, second) = match (first, second) {
                (Some(first), Some(second)) => (first, second),
                _ => {
                    // The analytic pair kernel emits raw overlaps only for
                    // source-image correspondences in unchanged source charts.
                    // Selected nonlinear image components carry their own map.
                    let (CurveSupport2::Parallel(first), CurveSupport2::Parallel(second)) = (
                        &self.data.carriers[pair.first_carrier_index].geometry,
                        &self.data.carriers[pair.second_carrier_index].geometry,
                    ) else {
                        return Err(
                            self.blocked(pair.first_carrier_index, UncertaintyReason::Unsupported)
                        );
                    };
                    (
                        first
                            .source()
                            .to_rational_bezier()
                            .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?,
                        second
                            .source()
                            .to_rational_bezier()
                            .map_err(|cause| self.invalid(pair.second_carrier_index, cause))?,
                    )
                }
            };
            sources.push(CurveOverlapCorrespondence2::for_rational_ranges(
                &first,
                &second,
                first_range,
                second_range,
                overlap.orientation(),
                &self.data.policy,
            ));
        }
        Ok(sources
            .into_iter()
            .map(|source| CurveIntersectionOverlap2 {
                first_span_index: 0,
                second_span_index: 0,
                first_range: CurveParameterRange2::from_bezier_range(first_range.clone()),
                second_range: CurveParameterRange2::from_bezier_range(second_range.clone()),
                orientation: overlap.orientation(),
                endpoint_inclusion: [overlap.includes_start(), overlap.includes_end()],
                parameter_correspondence: source,
            })
            .collect())
    }

    fn pair_result(&self, pair: &RegionCarrierPair) -> ExactCurveResult<RegionPairResult> {
        let first = &self.data.carriers[pair.first_carrier_index];
        let second = &self.data.carriers[pair.second_carrier_index];
        match &pair.context {
            RegionCarrierPairContext::Common(context) => {
                let result = context.result_view()?;
                Ok(RegionPairResult {
                    contacts: result
                        .contacts()
                        .iter()
                        .map(RegionPairContactEvidence::from_intersection)
                        .collect(),
                    overlaps: result.overlaps().to_vec(),
                    blockers: result
                        .blockers()
                        .iter()
                        .cloned()
                        .map(RegionPairBlocker::Common)
                        .chain(
                            (!result.parameter_components().is_empty())
                                .then_some(RegionPairBlocker::PointImageParameterComponent),
                        )
                        .collect(),
                })
            }
            RegionCarrierPairContext::ParallelRational { parallel_is_first } => {
                let (parallel_carrier, parallel, parallel_index, curve) = if *parallel_is_first {
                    (
                        first,
                        first.geometry.parallel(),
                        pair.first_carrier_index,
                        second.geometry.bezier(),
                    )
                } else {
                    (
                        second,
                        second.geometry.parallel(),
                        pair.second_carrier_index,
                        first.geometry.bezier(),
                    )
                };
                let regular_range = CurveParameterRange2::new_validated(
                    parallel_carrier.start.clone(),
                    parallel_carrier.end.clone(),
                );
                // Circle incidence proves source regularity on this range.
                // Retained endpoint storage is not a singularity certificate:
                // ordinary selected cuts use the same conic inverse, while
                // source-cusp limits continue through the regularized kernel.
                match self.parallel_arc_pair_result(
                    parallel,
                    &regular_range,
                    curve,
                    *parallel_is_first,
                )? {
                    Classification::Decided(Some(result)) => return Ok(result),
                    Classification::Decided(None) | Classification::Uncertain(_) => {}
                }
                let line = self.parallel_line_pair_result(
                    pair,
                    parallel,
                    parallel_index,
                    curve,
                    *parallel_is_first,
                    Some(&regular_range),
                )?;
                match line {
                    Classification::Decided(Some(result)) => return Ok(result),
                    Classification::Decided(None) | Classification::Uncertain(_) => {}
                }
                let rational = RationalBezier2::try_from_subcurve(curve)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
                let intersections = parallel.intersections_on_regular_range(
                    &rational,
                    &regular_range,
                    &self.data.policy,
                );
                let result = match intersections
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    Classification::Decided(result) => result,
                    Classification::Uncertain(reason) => {
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                            blockers: vec![RegionPairBlocker::Uncertain(reason)],
                        });
                    }
                };
                let contacts = result
                    .contacts()
                    .iter()
                    .map(|contact| {
                        let (first_parameter, second_parameter, tangent_cross_sign) =
                            if *parallel_is_first {
                                (
                                    contact.parallel_parameter().clone(),
                                    contact.other_parameter().clone(),
                                    contact.tangent_cross_sign(),
                                )
                            } else {
                                (
                                    contact.other_parameter().clone(),
                                    contact.parallel_parameter().clone(),
                                    contact.tangent_cross_sign().map(|sign| match sign {
                                        RealSign::Positive => RealSign::Negative,
                                        RealSign::Negative => RealSign::Positive,
                                        RealSign::Zero => RealSign::Zero,
                                    }),
                                )
                            };
                        RegionPairContactEvidence::direct_bezier(
                            first_parameter,
                            second_parameter,
                            Some(contact.point().clone()),
                            contact.is_certified_transverse(),
                            tangent_cross_sign,
                        )
                    })
                    .collect();
                let mut overlaps = Vec::new();
                for overlap in result.overlaps() {
                    overlaps.extend(self.analytic_component_overlaps(
                        pair,
                        result.component_overlaps(),
                        overlap,
                        !*parallel_is_first,
                    )?);
                }
                let mut blockers = Vec::with_capacity(2);
                if !result.parameter_components().is_empty() {
                    blockers.push(RegionPairBlocker::PointImageParameterComponent);
                }
                if !result.is_complete() {
                    blockers.push(RegionPairBlocker::IncompleteReplay);
                }
                Ok(RegionPairResult {
                    contacts,
                    overlaps,
                    blockers,
                })
            }
            RegionCarrierPairContext::ParallelPair
            | RegionCarrierPairContext::ParallelSameImage => {
                if self.parallel_pair_is_coordinate_disjoint(pair)
                    || self.adjacent_parallel_pair_is_endpoint_only(pair)
                {
                    // A shared strictly monotone coordinate either separates
                    // the complete images or reduces them to one already
                    // seeded adjacent loop vertex.  Neither case needs a
                    // bivariate resultant.
                    return Ok(RegionPairResult {
                        contacts: Vec::new(),
                        overlaps: Vec::new(),
                        blockers: Vec::new(),
                    });
                }
                let parallel = first.geometry.parallel();
                // Identity saturation and residual self contacts share the pair kernel.
                let intersection = parallel.parallel_intersections_on_regular_ranges(
                    second.geometry.parallel(),
                    &first.range(),
                    &second.range(),
                    &self.data.policy,
                );
                let result = match intersection
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    Classification::Decided(result) => result,
                    Classification::Uncertain(reason) => {
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                            blockers: vec![RegionPairBlocker::Uncertain(reason)],
                        });
                    }
                };
                let contacts = result
                    .contacts()
                    .iter()
                    .map(|contact| {
                        RegionPairContactEvidence::direct(
                            contact.first_parameter().clone(),
                            contact.second_parameter().clone(),
                            None,
                            contact.is_certified_transverse(),
                            contact.tangent_cross_sign(),
                        )
                    })
                    .collect();
                let mut overlaps = Vec::new();
                for overlap in result.overlaps() {
                    overlaps.extend(self.analytic_component_overlaps(
                        pair,
                        result.component_overlaps(),
                        overlap,
                        false,
                    )?);
                }
                let mut blockers = Vec::with_capacity(2);
                if !result.parameter_components().is_empty() {
                    blockers.push(RegionPairBlocker::PointImageParameterComponent);
                }
                if !result.is_complete() {
                    blockers.push(RegionPairBlocker::IncompleteReplay);
                }
                Ok(RegionPairResult {
                    contacts,
                    overlaps,
                    blockers,
                })
            }
            RegionCarrierPairContext::CuspChord { cusp_is_first } => {
                {
                    let (cusp, cusp_index, chord, chord_index) = if *cusp_is_first {
                        (
                            first.geometry.circle(),
                            pair.first_carrier_index,
                            match &second.geometry {
                                CurveSupport2::Line(chord) => chord,
                                _ => unreachable!("cusp/chord dispatch retained its chord"),
                            },
                            pair.second_carrier_index,
                        )
                    } else {
                        (
                            second.geometry.circle(),
                            pair.second_carrier_index,
                            match &first.geometry {
                                CurveSupport2::Line(chord) => chord,
                                _ => unreachable!("chord/cusp dispatch retained its chord"),
                            },
                            pair.first_carrier_index,
                        )
                    };
                    let mut certified_chord_endpoint_incidence = None;
                    if let Some((first_at_start, second_at_start)) = self
                        .authored_carrier_shared_endpoints(
                            pair.first_carrier_index,
                            pair.second_carrier_index,
                        )
                    {
                        let cusp_at_start = if *cusp_is_first {
                            first_at_start
                        } else {
                            second_at_start
                        };
                        if cusp.certified_tangent_endpoint(cusp_at_start)
                            && !cusp.selected_chord_normal_contact_endpoint(cusp_at_start)
                        {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-circle-chord-pair",
                                "adjacent-authored-tangent",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        let structural_endpoint_only = cusp
                            .authored_adjacent_chord_is_structurally_endpoint_only(
                                chord,
                                cusp_at_start,
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(chord_index, cause))?;
                        let endpoint_only = structural_endpoint_only
                            || self
                                .data
                                .policy
                                .strict_predicate_pass(|| {
                                    cusp.certified_adjacent_chord_is_endpoint_only(
                                        chord,
                                        cusp_at_start,
                                        &self.data.policy,
                                    )
                                })
                                .map_err(|cause| self.invalid(chord_index, cause))?
                                == Classification::Decided(true);
                        if endpoint_only {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-circle-chord-pair",
                                "authored-adjacent-endpoint-only",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        certified_chord_endpoint_incidence = Some(if *cusp_is_first {
                            second_at_start
                        } else {
                            first_at_start
                        });
                    }
                    if let Some((sibling_index, sibling_at_start, chord_at_start)) = self
                        .authored_supporting_circle_endpoint(
                            cusp_index,
                            chord_index,
                            |sibling, at_start| {
                                sibling.certified_tangent_endpoint(at_start)
                                    && !sibling.selected_chord_normal_contact_endpoint(at_start)
                            },
                        )
                        && certified_chord_endpoint_incidence
                            .is_none_or(|incident| incident == chord_at_start)
                    {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-circle-chord-pair",
                            "supporting-circle-sibling-endpoint-tangent",
                        );
                        let sibling = self.data.carriers[sibling_index].geometry.circle();
                        let mapped = self
                            .data
                            .policy
                            .strict_predicate_pass(|| {
                                cusp.parameter_of_shared_circle_endpoint(
                                    sibling,
                                    sibling_at_start,
                                    &self.data.policy,
                                )
                            })
                            .map_err(|cause| self.invalid(cusp_index, cause))?;
                        match mapped {
                            Classification::Decided(None) => return Ok(RegionPairResult::empty()),
                            Classification::Decided(Some(cusp_parameter)) => {
                                let (chord_parameter, point) = if chord_at_start {
                                    (chord.start_parameter(), chord.start().clone())
                                } else {
                                    (chord.end_parameter(), chord.end().clone())
                                };
                                return self.retained_cusp_chord_pair_result(
                                    cusp,
                                    chord,
                                    chord_index,
                                    *cusp_is_first,
                                    vec![BezierAlgebraicCuspSemicircleRetainedChordContact2 {
                                        cusp_parameter,
                                        chord_parameter,
                                        point,
                                        tangent_cross_sign: RealSign::Zero,
                                    }],
                                );
                            }
                            Classification::Uncertain(_) => {}
                        }
                        // A failed chart comparison does not erase the exact
                        // full-circle incidence or prove absence of a contact.
                        certified_chord_endpoint_incidence = Some(chord_at_start);
                    }
                    if certified_chord_endpoint_incidence.is_none()
                        && let Classification::Decided(Some(contact)) = cusp
                            .certified_chord_endpoint_contact(chord, &self.data.policy)
                            .map_err(|cause| self.invalid(chord_index, cause))?
                    {
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-circle-chord-pair",
                            "retained-nonadjacent-endpoint-contact",
                        );
                        return self.retained_cusp_chord_pair_result(
                            cusp,
                            chord,
                            chord_index,
                            *cusp_is_first,
                            vec![contact],
                        );
                    }
                    // Refined bounds are only a rejection accelerator. Keep
                    // their proof budget small and fall through to the exact
                    // circle/chord kernel when the boxes continue to overlap;
                    // policy-terminal refinement belongs in predicates that
                    // can decide the result, not in broad phase replay.
                    for refinement_steps in [0, 2] {
                        let circle_bounds = cusp
                            .semicircle()
                            .conservative_bounds_refined(refinement_steps, &self.data.policy)
                            .map_err(|cause| self.invalid(cusp_index, cause))?;
                        let chord_bounds = chord
                            .conservative_bounds_refined(refinement_steps, &self.data.policy)
                            .map_err(|cause| self.invalid(chord_index, cause))?;
                        let (
                            Classification::Decided(circle_bounds),
                            Classification::Decided(chord_bounds),
                        ) = (circle_bounds, chord_bounds)
                        else {
                            continue;
                        };
                        if circle_bounds.overlaps(&chord_bounds, &self.data.policy)
                            == Classification::Decided(false)
                        {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-circle-chord-pair",
                                "refined-bounds-disjoint",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                    }
                    let intersections = match certified_chord_endpoint_incidence {
                        Some(chord_at_start) => cusp
                            .semicircle()
                            .chord_intersections_with_certified_endpoint_incidence(
                                chord,
                                chord_at_start,
                                &self.data.policy,
                            ),
                        None => cusp
                            .semicircle()
                            .chord_intersections(chord, &self.data.policy),
                    }
                    .map_err(|cause| self.invalid(chord_index, cause))?;
                    let intersections = match intersections {
                        Classification::Decided(intersections) => intersections,
                        Classification::Uncertain(reason) => {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-circle-chord-pair",
                                match reason {
                                    UncertaintyReason::Unsupported => "kernel-unsupported",
                                    UncertaintyReason::Predicate => "kernel-predicate",
                                    UncertaintyReason::Ordering => "kernel-ordering",
                                    UncertaintyReason::RealSign => "kernel-real-sign",
                                    UncertaintyReason::Boundary => "kernel-boundary",
                                },
                            );
                            #[cfg(feature = "dispatch-trace")]
                            if reason == UncertaintyReason::Unsupported {
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-circle-chord-kernel-blocker",
                                    if chord.exact_line().is_some() {
                                        "exact-line"
                                    } else if chord.certified_unit_tangent().is_some() {
                                        "certified-tangent"
                                    } else {
                                        "general-retained"
                                    },
                                );
                            }
                            return Ok(RegionPairResult {
                                contacts: Vec::new(),
                                overlaps: Vec::new(),
                                blockers: vec![RegionPairBlocker::Uncertain(reason)],
                            });
                        }
                    };
                    let mut contacts = intersections;
                    if contacts.is_empty() {
                        return Ok(RegionPairResult::empty());
                    }
                    if let Some(chord_at_start) = certified_chord_endpoint_incidence {
                        // Boundary-loop seeding already owns this exact
                        // adjacent vertex. The circle/chord solve was still
                        // required because a line through one circle point can
                        // have a second finite contact; discard only the
                        // structurally identified endpoint and retain every
                        // other root.
                        contacts.retain(|contact| {
                            !contact
                                .chord_parameter
                                .is_endpoint_of(chord, chord_at_start)
                        });
                        if contacts.is_empty() {
                            return Ok(RegionPairResult::empty());
                        }
                    }
                    self.retained_cusp_chord_pair_result(
                        cusp,
                        chord,
                        chord_index,
                        *cusp_is_first,
                        contacts,
                    )
                }
            }
            RegionCarrierPairContext::AlgebraicChordPair { endpoint_contact } => {
                {
                    let (chord, chord_index, other, other_index) =
                        match (&first.geometry, &second.geometry) {
                            (CurveSupport2::Line(chord), other) => (
                                chord,
                                pair.first_carrier_index,
                                other,
                                pair.second_carrier_index,
                            ),
                            (other, CurveSupport2::Line(chord)) => (
                                chord,
                                pair.second_carrier_index,
                                other,
                                pair.first_carrier_index,
                            ),
                            _ => unreachable!("an algebraic-chord pair retains one chord"),
                        };
                    if let Some(contact) = endpoint_contact
                        && let CurveSupport2::Bezier(curve) = other
                    {
                        let parameter = if chord_index == pair.first_carrier_index {
                            &contact.second_parameter
                        } else {
                            &contact.first_parameter
                        };
                        let rational = RationalBezier2::try_from_subcurve(curve)
                            .map_err(|cause| self.invalid(other_index, cause))?;
                        if let Some(mut result) = self.algebraic_chord_rational_pair_result(
                            pair,
                            chord,
                            chord_index,
                            &rational,
                            None,
                            Some(parameter),
                        )? {
                            result.contacts.push((**contact).clone());
                            return Ok(result);
                        }
                    }
                    // Every retained-chord pairing below already owns a
                    // complete finite-domain kernel. Refining composite
                    // endpoints into an optional AABB duplicates those exact
                    // predicates and can expand a large shared scalar DAG
                    // before the authoritative carrier relation is consulted.
                    if let CurveSupport2::Line(other_chord) = other {
                        if self.authored_carriers_are_adjacent(pair) {
                            for (support, candidate) in [(chord, other_chord), (other_chord, chord)]
                            {
                                if support.certified_unit_tangent().is_none() {
                                    continue;
                                }
                                for endpoint in [candidate.start(), candidate.end()] {
                                    if matches!(
                                        support
                                            .certified_tangent_side(endpoint, &self.data.policy,),
                                        Classification::Decided(
                                            crate::classify::LineSide::Left
                                                | crate::classify::LineSide::Right
                                        )
                                    ) {
                                        // One endpoint off the retained line
                                        // proves the adjacent supports are
                                        // noncollinear. Their sole support
                                        // intersection is the authored vertex.
                                        #[cfg(feature = "dispatch-trace")]
                                        hyperreal::dispatch_trace::record(
                                            "hypercurve",
                                            "algebraic-chord-pair",
                                            "adjacent-certified-tangent-complete",
                                        );
                                        return Ok(RegionPairResult::empty());
                                    }
                                }
                            }
                        }
                        if self.authored_carriers_are_adjacent(pair)
                            && let (Some(first_tangent), Some(second_tangent)) = (
                                chord.certified_unit_tangent(),
                                other_chord.certified_unit_tangent(),
                            )
                        {
                            let tangent_cross = &first_tangent.0 * &second_tangent.1
                                - &first_tangent.1 * &second_tangent.0;
                            if matches!(
                                real_sign(&tangent_cross, &self.data.policy),
                                Some(RealSign::Positive | RealSign::Negative)
                            ) {
                                // Nonparallel straight supports meet exactly
                                // once. Authored adjacency already owns that
                                // endpoint, so there is no additional contact
                                // or overlap to add to the arrangement.
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-chord-pair",
                                    "adjacent-certified-nonparallel-complete",
                                );
                                return Ok(RegionPairResult::empty());
                            }
                        }
                        if self.authored_carriers_are_adjacent(pair) {
                            for (axis_chord, candidate) in
                                [(chord, other_chord), (other_chord, chord)]
                            {
                                let Some(direction) = axis_chord.certified_axis_direction() else {
                                    continue;
                                };
                                let constant_axis = match direction.axis() {
                                    Axis2::X => Axis2::Y,
                                    Axis2::Y => Axis2::X,
                                };
                                let mut certified_noncollinear = false;
                                for endpoint in [candidate.start(), candidate.end()] {
                                    match self
                                        .data
                                        .policy
                                        .strict_predicate_pass(|| {
                                            crate::BezierAlgebraicChord2::point_axis_order(
                                                axis_chord.start(),
                                                endpoint,
                                                constant_axis,
                                                &self.data.policy,
                                            )
                                        })
                                        .map_err(|cause| self.invalid(chord_index, cause))?
                                    {
                                        Classification::Decided(
                                            std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                                        ) => {
                                            certified_noncollinear = true;
                                            break;
                                        }
                                        Classification::Decided(std::cmp::Ordering::Equal)
                                        | Classification::Uncertain(_) => {}
                                    }
                                }
                                if certified_noncollinear {
                                    #[cfg(feature = "dispatch-trace")]
                                    hyperreal::dispatch_trace::record(
                                        "hypercurve",
                                        "algebraic-chord-pair",
                                        "adjacent-axis-noncollinear-complete",
                                    );
                                    return Ok(RegionPairResult::empty());
                                }
                            }
                        }
                        let strictly_one_sided = if let Some(line) = other_chord.exact_line() {
                            self.data
                                .policy
                                .strict_predicate_pass(|| {
                                    chord.is_strictly_one_sided_of_exact_line(
                                        &line,
                                        &self.data.policy,
                                    )
                                })
                                .map_err(|cause| self.invalid(chord_index, cause))?
                        } else if let Some(line) = chord.exact_line() {
                            self.data
                                .policy
                                .strict_predicate_pass(|| {
                                    other_chord.is_strictly_one_sided_of_exact_line(
                                        &line,
                                        &self.data.policy,
                                    )
                                })
                                .map_err(|cause| self.invalid(other_index, cause))?
                        } else {
                            Classification::Decided(false)
                        };
                        if strictly_one_sided == Classification::Decided(true) {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "exact-line-one-sided",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        let intersections = match chord
                            .chord_intersections(other_chord, &self.data.policy)
                            .map_err(|cause| self.invalid(chord_index, cause))?
                        {
                            Classification::Decided(intersections) => intersections,
                            Classification::Uncertain(reason) => {
                                return Ok(RegionPairResult {
                                    contacts: Vec::new(),
                                    overlaps: Vec::new(),
                                    blockers: vec![RegionPairBlocker::Uncertain(reason)],
                                });
                            }
                        };
                        let (mut contacts, overlaps) = match intersections {
                            BezierAlgebraicChordPairIntersections2::Contacts(contacts) => (
                                contacts
                                    .into_iter()
                                    .map(|contact| {
                                        RegionPairContactEvidence::direct(
                                            CurveParameter2::from_algebraic_chord(
                                                contact.first_parameter().clone(),
                                            ),
                                            CurveParameter2::from_algebraic_chord(
                                                contact.second_parameter().clone(),
                                            ),
                                            Some(contact.point().clone()),
                                            contact.tangent_cross_sign() != RealSign::Zero,
                                            Some(contact.tangent_cross_sign()),
                                        )
                                    })
                                    .collect(),
                                Vec::new(),
                            ),
                            BezierAlgebraicChordPairIntersections2::Overlaps(overlaps) => (
                                Vec::new(),
                                overlaps
                                    .into_iter()
                                    .map(|overlap| {
                                        let [first_start, first_end] = overlap.first_range();
                                        let [second_start, second_end] = overlap.second_range();
                                        CurveIntersectionOverlap2 {
                                            first_span_index: 0,
                                            second_span_index: 0,
                                            endpoint_inclusion: [true, true],
                                            parameter_correspondence:
                                                CurveOverlapCorrespondence2::Chords {
                                                    first: chord.clone(),
                                                    second: other_chord.clone(),
                                                    first_range:
                                                        CurveParameterRange2::new_validated(
                                                            CurveParameter2::from_algebraic_chord(
                                                                first_start.clone(),
                                                            ),
                                                            CurveParameter2::from_algebraic_chord(
                                                                first_end.clone(),
                                                            ),
                                                        ),
                                                    second_range:
                                                        CurveParameterRange2::new_validated(
                                                            CurveParameter2::from_algebraic_chord(
                                                                second_start.clone(),
                                                            ),
                                                            CurveParameter2::from_algebraic_chord(
                                                                second_end.clone(),
                                                            ),
                                                        ),
                                                },
                                            first_range: CurveParameterRange2::new_validated(
                                                CurveParameter2::from_algebraic_chord(
                                                    first_start.clone(),
                                                ),
                                                CurveParameter2::from_algebraic_chord(
                                                    first_end.clone(),
                                                ),
                                            ),
                                            second_range: CurveParameterRange2::new_validated(
                                                CurveParameter2::from_algebraic_chord(
                                                    second_start.clone(),
                                                ),
                                                CurveParameter2::from_algebraic_chord(
                                                    second_end.clone(),
                                                ),
                                            ),
                                            orientation: overlap.orientation(),
                                        }
                                    })
                                    .collect(),
                            ),
                        };
                        if self.authored_carriers_are_adjacent(pair) && overlaps.is_empty() {
                            // Adjacent straight chords have only their already
                            // seeded authored endpoint in common unless they
                            // overlap positively, which remains arrangement
                            // evidence.
                            contacts.clear();
                        }
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "algebraic-chord-pair",
                            if overlaps.is_empty() {
                                "chord-contact-complete"
                            } else {
                                "chord-overlap-complete"
                            },
                        );
                        return Ok(RegionPairResult {
                            contacts,
                            overlaps,
                            blockers: Vec::new(),
                        });
                    }
                    if let CurveSupport2::Bezier(curve) = other {
                        let other_carrier = &self.data.carriers[other_index];
                        let chord_carrier = &self.data.carriers[chord_index];
                        let authored_adjacent = self.authored_carriers_are_adjacent(pair);
                        let chord_precedes_other = authored_adjacent.then(|| {
                            let boundary = match chord_carrier.operand {
                                CurveRegionBooleanOperand2::First => self.data.first,
                                CurveRegionBooleanOperand2::Second => self.data.second,
                            }
                            .boundary_loops()
                            .get(chord_carrier.loop_index)
                            .expect("an admitted carrier retains its authored loop");
                            chord_carrier.fragment_index.checked_add(1)
                                == Some(other_carrier.fragment_index)
                                || (chord_carrier.fragment_index.checked_add(1)
                                    == Some(boundary.fragments().len())
                                    && other_carrier.fragment_index == 0)
                        });
                        if subcurve_is_strict_line_image(curve)
                            && let (Some(start), Some(end)) = (
                                exact_carrier_point(
                                    other_carrier,
                                    &other_carrier.start,
                                    &self.data.policy,
                                ),
                                exact_carrier_point(
                                    other_carrier,
                                    &other_carrier.end,
                                    &self.data.policy,
                                ),
                            )
                            && let Ok(line) = LineSeg2::try_new(start, end)
                            && chord
                                .is_strictly_one_sided_of_exact_line(&line, &self.data.policy)
                                .map_err(|cause| self.invalid(other_index, cause))?
                                == Classification::Decided(true)
                        {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "exact-line-one-sided",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        if authored_adjacent
                            && carrier_has_certified_injective_image(
                                other_carrier,
                                &self.data.policy,
                            )
                            && subcurve_is_strict_line_image(curve)
                            && let (Some(start), Some(end)) = (
                                exact_carrier_point(
                                    other_carrier,
                                    &other_carrier.start,
                                    &self.data.policy,
                                ),
                                exact_carrier_point(
                                    other_carrier,
                                    &other_carrier.end,
                                    &self.data.policy,
                                ),
                            )
                            && let Ok(line) = LineSeg2::try_new(start, end)
                        {
                            if let (
                                Classification::Decided(Some(chord_direction)),
                                Some(line_direction),
                            ) = (
                                chord
                                    .axis_direction(&self.data.policy)
                                    .map_err(|cause| self.invalid(chord_index, cause))?,
                                exact_axis_aligned_line_direction(&line),
                            ) && chord_direction.axis() != line_direction.axis()
                            {
                                #[cfg(feature = "dispatch-trace")]
                                hyperreal::dispatch_trace::record(
                                    "hypercurve",
                                    "algebraic-chord-pair",
                                    "adjacent-perpendicular-line-complete",
                                );
                                return Ok(RegionPairResult::empty());
                            }
                            match self
                                .data
                                .policy
                                .strict_predicate_pass(|| {
                                    chord.has_non_collinear_support_with_exact_line(
                                        &line,
                                        &self.data.policy,
                                    )
                                })
                                .map_err(|cause| self.invalid(other_index, cause))?
                            {
                                Classification::Decided(true) => {
                                    #[cfg(feature = "dispatch-trace")]
                                    hyperreal::dispatch_trace::record(
                                        "hypercurve",
                                        "algebraic-chord-pair",
                                        "adjacent-exact-line-complete",
                                    );
                                    return Ok(RegionPairResult::empty());
                                }
                                Classification::Decided(false) | Classification::Uncertain(_) => {}
                            }
                        }
                        if let Some(result) = self.algebraic_chord_linear_bezier_pair_result(
                            pair,
                            chord,
                            chord_index,
                            curve,
                            other_index,
                        )? {
                            return Ok(result);
                        }
                        if let Some((_, circle)) = retained_circular_support(curve)
                            && chord
                                .certifiably_disjoint_from_circle_bounds(
                                    &circle.center,
                                    &circle.radius_squared,
                                    &self.data.policy,
                                )
                                .map_err(|cause| self.invalid(other_index, cause))?
                                == Classification::Decided(true)
                        {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "retained-circle-bounds-disjoint",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        if let Some(chord_precedes_other) = chord_precedes_other
                            && adjacent_axis_algebraic_chord_circular_curve_is_endpoint_only(
                                chord,
                                chord_carrier,
                                curve,
                                other_carrier,
                                chord_precedes_other,
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(other_index, cause))?
                                == Classification::Decided(true)
                        {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "adjacent-circular-endpoint-only",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        let rational = RationalBezier2::try_from_subcurve(curve)
                            .map_err(|cause| self.invalid(other_index, cause))?;
                        // This optional adjacency shortcut must not consume
                        // approximation before the common exact pair kernel.
                        if let Some(result) = self.data.policy.strict_predicate_pass(|| {
                            self.algebraic_chord_shared_image_endpoint_pair_result(
                                pair,
                                chord,
                                chord_index,
                                &rational,
                                other_index,
                            )
                        })? {
                            return Ok(result);
                        }
                        let one_sided = chord
                            .rational_control_hull_is_strictly_one_sided(
                                &rational,
                                &other_carrier.range(),
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(other_index, cause))?;
                        if one_sided == Classification::Decided(true) {
                            #[cfg(feature = "dispatch-trace")]
                            hyperreal::dispatch_trace::record(
                                "hypercurve",
                                "algebraic-chord-pair",
                                "rational-control-hull-one-sided",
                            );
                            return Ok(RegionPairResult::empty());
                        }
                        let shared_source_parameter =
                            if let Some(chord_precedes_other) = chord_precedes_other {
                                let shared_parameter = if chord_precedes_other {
                                    carrier_traversal_start(other_carrier)
                                } else {
                                    carrier_traversal_end(other_carrier)
                                };
                                Some(shared_parameter)
                            } else {
                                None
                            };
                        if let Some(result) = self.algebraic_chord_rational_pair_result(
                            pair,
                            chord,
                            chord_index,
                            &rational,
                            None,
                            shared_source_parameter,
                        )? {
                            return Ok(result);
                        }
                    }
                    if let CurveSupport2::Parallel(parallel) = other {
                        return self.algebraic_chord_parallel_pair_result(
                            pair,
                            chord,
                            chord_index,
                            parallel,
                            other_index,
                        );
                    }
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-chord-pair",
                    "unsupported",
                );
                Ok(RegionPairResult {
                    contacts: Vec::new(),
                    overlaps: Vec::new(),
                    blockers: vec![RegionPairBlocker::Uncertain(UncertaintyReason::Unsupported)],
                })
            }
            RegionCarrierPairContext::CuspRational { cusp_is_first } => {
                let (cusp, curve, curve_carrier, curve_index) = if *cusp_is_first {
                    (
                        first.geometry.circle(),
                        second.geometry.bezier(),
                        second,
                        pair.second_carrier_index,
                    )
                } else {
                    (
                        second.geometry.circle(),
                        first.geometry.bezier(),
                        first,
                        pair.first_carrier_index,
                    )
                };
                if let Classification::Decided(bounds) = curve_carrier.bounds.get_or_init(|| {
                    curve_carrier.geometry.certified_outer_bounds(
                        &curve_carrier.range(),
                        0,
                        &self.data.policy,
                    )
                }) && cusp
                    .semicircle()
                    .certifiably_disjoint_from_bounds(bounds, &self.data.policy)
                    .map_err(|cause| self.invalid(curve_index, cause))?
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-rational-pair",
                        "bounds-disjoint",
                    );
                    return Ok(RegionPairResult::empty());
                }
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "algebraic-circle-rational-pair",
                    match (
                        self.authored_carriers_are_adjacent(pair),
                        subcurve_is_strict_line_image(curve),
                        retained_circular_support(curve).is_some(),
                    ) {
                        (true, true, _) => "adjacent-line",
                        (true, false, true) => "adjacent-circle",
                        (true, false, false) => "adjacent-general",
                        (false, true, _) => "nonadjacent-line",
                        (false, false, true) => "nonadjacent-circle",
                        (false, false, false) => "nonadjacent-general",
                    },
                );
                let rational = RationalBezier2::try_from_subcurve(curve)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?;
                let cusp_index = if *cusp_is_first {
                    pair.first_carrier_index
                } else {
                    pair.second_carrier_index
                };
                if let Some((sibling_index, sibling_at_start, curve_at_start)) =
                    self.authored_supporting_circle_endpoint(cusp_index, curve_index, |_, _| true)
                    && cusp
                        .semicircle()
                        .certifies_unique_rational_circle_contact(
                            &rational,
                            self.data.carriers[sibling_index]
                                .geometry
                                .circle()
                                .certified_tangent_endpoint(sibling_at_start),
                            &self.data.policy,
                        )
                        .map_err(|cause| self.invalid(curve_index, cause))?
                {
                    // Distinct tangent supporting circles share exactly one
                    // point. Boundary connectivity owns its parameter on an
                    // adjacent chart of this complete circle. Reuse that
                    // identity and transport it to the consumed half chart.
                    if sibling_index == cusp_index {
                        return Ok(RegionPairResult::empty());
                    }
                    let sibling = self.data.carriers[sibling_index].geometry.circle();
                    let mapped = self
                        .data
                        .policy
                        .strict_predicate_pass(|| {
                            cusp.parameter_of_shared_circle_endpoint(
                                sibling,
                                sibling_at_start,
                                &self.data.policy,
                            )
                        })
                        .map_err(|cause| self.invalid(cusp_index, cause))?;
                    match mapped {
                        Classification::Decided(None) => return Ok(RegionPairResult::empty()),
                        Classification::Decided(Some(parameter)) => {
                            let point = match sibling
                                .endpoint_point_evidence(sibling_at_start, &self.data.policy)
                                .map_err(|cause| self.invalid(cusp_index, cause))?
                            {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(_) => None,
                            };
                            let circle_parameter = CurveParameter2::from_algebraic_cusp(parameter);
                            let curve_parameter = if curve_at_start {
                                carrier_traversal_start_parameter(curve_carrier)
                            } else {
                                carrier_traversal_end_parameter(curve_carrier)
                            }
                            .clone();
                            let (first_parameter, second_parameter) = if *cusp_is_first {
                                (circle_parameter, curve_parameter)
                            } else {
                                (curve_parameter, circle_parameter)
                            };
                            return Ok(RegionPairResult {
                                contacts: vec![RegionPairContactEvidence::direct(
                                    first_parameter,
                                    second_parameter,
                                    point,
                                    false,
                                    Some(RealSign::Zero),
                                )],
                                overlaps: Vec::new(),
                                blockers: Vec::new(),
                            });
                        }
                        Classification::Uncertain(_) => {}
                    }
                }
                self.algebraic_cusp_rational_pair_result(pair, cusp, &rational, *cusp_is_first)
            }
            RegionCarrierPairContext::CuspParallel { cusp_is_first } => {
                let (cusp, parallel, parallel_carrier, parallel_index) = if *cusp_is_first {
                    (
                        first.geometry.circle(),
                        second.geometry.parallel(),
                        second,
                        pair.second_carrier_index,
                    )
                } else {
                    (
                        second.geometry.circle(),
                        first.geometry.parallel(),
                        first,
                        pair.first_carrier_index,
                    )
                };
                let parallel_range = CurveParameterRange2::new_validated(
                    parallel_carrier.start.clone(),
                    parallel_carrier.end.clone(),
                );
                if let Classification::Decided(Some(component)) = self.data.policy.bounded_exact_predicate_pass(|| parallel
                    .exact_rational_parallel_component_on_regular_range(&parallel_range, &self.data.policy))
                    .map_err(|cause| self.invalid(parallel_index, cause))?
                    // Exact affine lines are owned by the shared lower
                    // circle/parallel kernel, which delegates to the same
                    // circle/chord authority used by fillets and offsets.
                    && component.curve().exact_linear_parameterization_line().is_none()
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-parallel-pair",
                        "strict-rational-component",
                    );
                    let result = self.algebraic_cusp_rational_pair_result(
                        pair,
                        cusp,
                        component.curve(),
                        *cusp_is_first,
                    )?;
                    if result.blockers.is_empty() {
                        return Ok(result);
                    }
                    // The rationalized component is only a compact fast path.
                    // Recursive selected-circle frames retain a smaller exact
                    // circle/parallel authority that can decide the same
                    // finite range when global rational projection cannot.
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "algebraic-circle-parallel-pair",
                        "rational-component-fallback",
                    );
                }
                let intersections = match cusp
                    .semicircle()
                    .parallel_intersections(parallel, &parallel_range, None, &self.data.policy)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    Classification::Decided(result) => result,
                    Classification::Uncertain(reason) => {
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                            blockers: vec![RegionPairBlocker::Uncertain(reason)],
                        });
                    }
                };
                let (contacts, overlaps) = match intersections {
                    BezierAlgebraicCuspSemicircleParallelIntersections2::Mapped { contacts, overlaps } => (contacts, overlaps),
                    BezierAlgebraicCuspSemicircleParallelIntersections2::RetainedContacts(
                        contacts,
                    ) => {
                        return Ok(retained_cusp_parallel_contacts_result(
                            contacts,
                            *cusp_is_first,
                        ));
                    }
                    BezierAlgebraicCuspSemicircleParallelIntersections2::SelectedFiber { contacts, overlaps } => {
                        return Ok(selected_fiber_cusp_result(contacts, overlaps, *cusp_is_first));
                    }
                    BezierAlgebraicCuspSemicircleParallelIntersections2::CoincidentCircleComponent
                    | BezierAlgebraicCuspSemicircleParallelIntersections2::DegenerateProjection => {
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                            blockers: vec![RegionPairBlocker::Uncertain(
                                UncertaintyReason::Unsupported,
                            )],
                        });
                    }
                };
                let parameter_map = if contacts
                    .iter()
                    .any(|contact| contact.retained_cusp_parameter().is_none())
                {
                    match cusp
                        .semicircle()
                        .parallel_parameter_map(parallel, &self.data.policy)
                        .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                    {
                        Classification::Decided(map) => Some(map),
                        Classification::Uncertain(reason) => {
                            return Ok(RegionPairResult {
                                contacts: Vec::new(),
                                overlaps: Vec::new(),
                                blockers: vec![RegionPairBlocker::Uncertain(reason)],
                            });
                        }
                    }
                } else {
                    None
                };
                let mut retained = Vec::with_capacity(contacts.len());
                for contact in contacts {
                    let cusp_parameter = contact.retained_cusp_parameter().unwrap_or_else(|| {
                        parameter_map
                            .as_ref()
                            .expect("an interior cusp/parallel contact retains its parameter map")
                            .contact_parameter(&contact)
                    });
                    let tangent_cross_sign = contact
                        .tangent_cross_sign
                        .map(|sign| orient_tangent_cross_sign(sign, *cusp_is_first));
                    let tangent_topology = if tangent_cross_sign == Some(RealSign::Zero) {
                        match cusp
                            .semicircle()
                            .parallel_contact_endpoint_tangent_topology(
                                parallel,
                                &contact,
                                &self.data.policy,
                            )
                            .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                        {
                            Classification::Decided(Some((dot, circle_side)))
                                if dot != RealSign::Zero =>
                            {
                                let side = if *cusp_is_first {
                                    match parallel
                                        .tangent_side_at(
                                            &contact.parallel_parameter,
                                            &self.data.policy,
                                        )
                                        .map_err(|cause| {
                                            self.invalid(pair.first_carrier_index, cause)
                                        })? {
                                        Classification::Decided(LineSide::Left) => {
                                            Some(if dot == RealSign::Positive {
                                                LineSide::Left
                                            } else {
                                                LineSide::Right
                                            })
                                        }
                                        Classification::Decided(LineSide::Right) => {
                                            Some(if dot == RealSign::Positive {
                                                LineSide::Right
                                            } else {
                                                LineSide::Left
                                            })
                                        }
                                        Classification::Decided(LineSide::On)
                                        | Classification::Uncertain(_) => None,
                                    }
                                } else {
                                    Some(circle_side)
                                };
                                side.map(|side| (dot, side))
                            }
                            Classification::Decided(Some(_))
                            | Classification::Decided(None)
                            | Classification::Uncertain(_) => None,
                        }
                    } else {
                        None
                    };
                    let (first_parameter, second_parameter) = if *cusp_is_first {
                        (
                            CurveParameter2::from_algebraic_cusp(cusp_parameter),
                            CurveParameter2::from(contact.parallel_parameter),
                        )
                    } else {
                        (
                            CurveParameter2::from(contact.parallel_parameter),
                            CurveParameter2::from_algebraic_cusp(cusp_parameter),
                        )
                    };
                    let evidence = RegionPairContactEvidence::direct(
                        first_parameter,
                        second_parameter,
                        None,
                        matches!(
                            tangent_cross_sign,
                            Some(RealSign::Positive | RealSign::Negative)
                        ),
                        tangent_cross_sign,
                    );
                    retained.push(match tangent_topology {
                        Some((dot, side)) => evidence.with_tangent_topology(dot, side),
                        None => evidence,
                    });
                }
                Ok(RegionPairResult {
                    contacts: retained,
                    overlaps: overlaps
                        .into_iter()
                        .map(|source| {
                            circle_overlap_evidence(
                                CurveCircleOverlap2::Mapped(source),
                                *cusp_is_first,
                            )
                        })
                        .collect(),
                    blockers: Vec::new(),
                })
            }
            RegionCarrierPairContext::CuspPair => {
                let first_cusp = first.geometry.circle();
                let second_cusp = second.geometry.circle();
                if let Some((first_at_start, second_at_start)) = self
                    .authored_carrier_shared_endpoints(
                        pair.first_carrier_index,
                        pair.second_carrier_index,
                    )
                    && (first_cusp.certified_tangent_endpoint(first_at_start)
                        || second_cusp.certified_tangent_endpoint(second_at_start))
                {
                    // The round/fillet constructor certifies tangency to its
                    // authored boundary neighbor before the carriers become
                    // independent arrangement entries. Loop seeding already
                    // owns their shared vertex, so there is no extra contact
                    // event to reconstruct.
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-cusp-pair",
                        "adjacent-certified-tangent",
                    );
                    return Ok(RegionPairResult::empty());
                }
                if let Classification::Decided(Some((first_parameter, second_parameter))) =
                    first_cusp
                        .unique_shared_tangent_endpoint_contact(second_cusp, &self.data.policy)
                        .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    if self.authored_carriers_are_adjacent(pair) {
                        // Loop seeding already owns this exact vertex on both
                        // carrier domains. An authored tangent switch neither
                        // splits either injective carrier nor adds a second
                        // topology event, so replaying its mapped parameter
                        // range would duplicate the retained adjacency proof.
                        #[cfg(feature = "dispatch-trace")]
                        hyperreal::dispatch_trace::record(
                            "hypercurve",
                            "curve-region-cusp-pair",
                            "adjacent-retained-endpoint-tangency",
                        );
                        return Ok(RegionPairResult::empty());
                    }
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-cusp-pair",
                        "retained-endpoint-tangency",
                    );
                    return Ok(RegionPairResult {
                        contacts: vec![RegionPairContactEvidence::direct(
                            CurveParameter2::from_algebraic_cusp(first_parameter),
                            CurveParameter2::from_algebraic_cusp(second_parameter),
                            None,
                            false,
                            Some(RealSign::Zero),
                        )],
                        overlaps: Vec::new(),
                        blockers: Vec::new(),
                    });
                }
                let intersections = match first_cusp
                    .semicircle()
                    .pair_intersections(second_cusp.semicircle(), &self.data.policy)
                    .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
                {
                    Classification::Decided(result) => result,
                    Classification::Uncertain(reason) => {
                        return Ok(RegionPairResult {
                            contacts: Vec::new(),
                            overlaps: Vec::new(),
                            blockers: vec![RegionPairBlocker::Uncertain(reason)],
                        });
                    }
                };
                let mut retained = Vec::new();
                let mut overlaps = Vec::new();
                match intersections {
                    BezierAlgebraicCuspSemicirclePairIntersections2::NoContacts => {}
                    BezierAlgebraicCuspSemicirclePairIntersections2::Contacts {
                        contacts,
                        parameter_map,
                    } => {
                        retained.reserve(contacts.len());
                        for contact in contacts {
                            let tangent_cross_sign = contact.tangent_cross_sign;
                            retained.push(RegionPairContactEvidence::direct(
                                CurveParameter2::from_algebraic_cusp(
                                    parameter_map.first_contact_parameter(&contact),
                                ),
                                CurveParameter2::from_algebraic_cusp(
                                    parameter_map.second_contact_parameter(&contact),
                                ),
                                None,
                                tangent_cross_sign != RealSign::Zero,
                                Some(tangent_cross_sign),
                            ));
                        }
                    }
                    BezierAlgebraicCuspSemicirclePairIntersections2::EndpointContacts(contacts) => {
                        retained.reserve(contacts.len());
                        for contact in contacts {
                            let first_parameter = contact
                                .first_location
                                .endpoint_parameter()
                                .expect("an endpoint contact names a first cusp endpoint");
                            let second_parameter = contact
                                .second_location
                                .endpoint_parameter()
                                .expect("an endpoint contact names a second cusp endpoint");
                            retained.push(RegionPairContactEvidence::direct(
                                CurveParameter2::from_algebraic_cusp(first_parameter),
                                CurveParameter2::from_algebraic_cusp(second_parameter),
                                None,
                                false,
                                Some(RealSign::Zero),
                            ));
                        }
                    }
                    BezierAlgebraicCuspSemicirclePairIntersections2::Overlap(overlap) => {
                        overlaps.push(CurveIntersectionOverlap2 {
                            first_span_index: 0,
                            second_span_index: 0,
                            endpoint_inclusion: [true, true],
                            parameter_correspondence: CurveOverlapCorrespondence2::Circle {
                                source: CurveCircleOverlap2::Pair(overlap.clone()),
                                swapped: false,
                            },
                            first_range: CurveParameterRange2::new_validated(
                                CurveParameter2::from_algebraic_cusp(
                                    overlap.first_start_parameter(),
                                ),
                                CurveParameter2::from_algebraic_cusp(overlap.first_end_parameter()),
                            ),
                            second_range: CurveParameterRange2::new_validated(
                                CurveParameter2::from_algebraic_cusp(
                                    overlap.second_start_parameter(),
                                ),
                                CurveParameter2::from_algebraic_cusp(
                                    overlap.second_end_parameter(),
                                ),
                            ),
                            orientation: overlap.orientation(),
                        });
                    }
                }
                Ok(RegionPairResult {
                    contacts: retained,
                    overlaps,
                    blockers: Vec::new(),
                })
            }
        }
    }

    fn parallel_pair_is_coordinate_disjoint(&self, pair: &RegionCarrierPair) -> bool {
        let first = &self.data.carriers[pair.first_carrier_index];
        let second = &self.data.carriers[pair.second_carrier_index];
        let (CurveSupport2::Parallel(first_parallel), CurveSupport2::Parallel(second_parallel)) =
            (&first.geometry, &second.geometry)
        else {
            return false;
        };
        let Some(first_start) = exact_carrier_point(
            first,
            carrier_traversal_start_parameter(first),
            &self.data.policy,
        ) else {
            return false;
        };
        let Some(first_end) = exact_carrier_point(
            first,
            carrier_traversal_end_parameter(first),
            &self.data.policy,
        ) else {
            return false;
        };
        let Some(second_start) = exact_carrier_point(
            second,
            carrier_traversal_start_parameter(second),
            &self.data.policy,
        ) else {
            return false;
        };
        let Some(second_end) = exact_carrier_point(
            second,
            carrier_traversal_end_parameter(second),
            &self.data.policy,
        ) else {
            return false;
        };

        for axis in [Axis2::X, Axis2::Y] {
            if !first_parallel.range_has_certified_injective_axis_on(
                axis,
                &first.range(),
                &self.data.policy,
            ) || !second_parallel.range_has_certified_injective_axis_on(
                axis,
                &second.range(),
                &self.data.policy,
            ) {
                continue;
            }
            let Some((first_minimum, first_maximum)) =
                ordered_axis_endpoint_points(&first_start, &first_end, axis, &self.data.policy)
            else {
                continue;
            };
            let Some((second_minimum, second_maximum)) =
                ordered_axis_endpoint_points(&second_start, &second_end, axis, &self.data.policy)
            else {
                continue;
            };
            for (lower_maximum, upper_minimum) in [
                (first_maximum, second_minimum),
                (second_maximum, first_minimum),
            ] {
                match compare_reals(
                    point_coordinate(lower_maximum, axis),
                    point_coordinate(upper_minimum, axis),
                    &self.data.policy,
                ) {
                    Some(Ordering::Less) => return true,
                    Some(Ordering::Equal)
                        if points_are_decided_distinct(
                            lower_maximum,
                            upper_minimum,
                            &self.data.policy,
                        ) =>
                    {
                        // Strict coordinate monotonicity makes this boundary
                        // value unique on each carrier.  Distinct endpoint
                        // points therefore exclude even a tangential contact.
                        return true;
                    }
                    Some(Ordering::Equal | Ordering::Greater) | None => {}
                }
            }
        }
        false
    }

    fn adjacent_parallel_pair_is_endpoint_only(&self, pair: &RegionCarrierPair) -> bool {
        if pair.first_carrier_index == pair.second_carrier_index {
            return false;
        }
        let first = &self.data.carriers[pair.first_carrier_index];
        let second = &self.data.carriers[pair.second_carrier_index];
        if first.operand != second.operand || first.loop_index != second.loop_index {
            return false;
        }
        let (CurveSupport2::Parallel(first_parallel), CurveSupport2::Parallel(second_parallel)) =
            (&first.geometry, &second.geometry)
        else {
            return false;
        };
        let boundary = match first.operand {
            CurveRegionBooleanOperand2::First => self.data.first.boundary_loops(),
            CurveRegionBooleanOperand2::Second => self.data.second.boundary_loops(),
        }
        .get(first.loop_index);
        let Some(boundary) = boundary else {
            return false;
        };
        let fragment_count = boundary.fragments().len();
        let first_start = carrier_traversal_start_parameter(first);
        let first_end = carrier_traversal_end_parameter(first);
        let second_start = carrier_traversal_start_parameter(second);
        let second_end = carrier_traversal_end_parameter(second);
        let (first_other, first_shared, second_shared, second_other) =
            if first.fragment_index.checked_add(1) == Some(second.fragment_index) {
                (first_start, first_end, second_start, second_end)
            } else if first.fragment_index == 0
                && second.fragment_index.checked_add(1) == Some(fragment_count)
            {
                (first_end, first_start, second_end, second_start)
            } else {
                return false;
            };
        let Some(first_other) = exact_carrier_point(first, first_other, &self.data.policy) else {
            return false;
        };
        let Some(first_shared) = exact_carrier_point(first, first_shared, &self.data.policy) else {
            return false;
        };
        let Some(second_shared) = exact_carrier_point(second, second_shared, &self.data.policy)
        else {
            return false;
        };
        let Some(second_other) = exact_carrier_point(second, second_other, &self.data.policy)
        else {
            return false;
        };
        if compare_reals(first_shared.x(), second_shared.x(), &self.data.policy)
            != Some(Ordering::Equal)
            || compare_reals(first_shared.y(), second_shared.y(), &self.data.policy)
                != Some(Ordering::Equal)
        {
            return false;
        }

        for axis in [Axis2::X, Axis2::Y] {
            if !first_parallel.range_has_certified_injective_axis_on(
                axis,
                &first.range(),
                &self.data.policy,
            ) || !second_parallel.range_has_certified_injective_axis_on(
                axis,
                &second.range(),
                &self.data.policy,
            ) {
                continue;
            }
            let first_order = compare_reals(
                point_coordinate(&first_other, axis),
                point_coordinate(&first_shared, axis),
                &self.data.policy,
            );
            let second_order = compare_reals(
                point_coordinate(&second_other, axis),
                point_coordinate(&second_shared, axis),
                &self.data.policy,
            );
            if matches!(
                (first_order, second_order),
                (Some(Ordering::Less), Some(Ordering::Greater))
                    | (Some(Ordering::Greater), Some(Ordering::Less))
            ) {
                return true;
            }
        }
        false
    }

    /// Keeps overlap endpoint incidence independent of the pair kernel's range
    /// ordering. The orientation relates the two underlying source charts.
    fn paired_overlap_ranges(
        &self,
        pair: &RegionCarrierPair,
        orientation: CurveOverlapOrientation2,
        (first, second): (CurveParameterRange2, CurveParameterRange2),
    ) -> ExactCurveResult<(CurveParameterRange2, CurveParameterRange2)> {
        let first_direction = decided_parameter_cmp(first.start(), first.end(), &self.data.policy)?;
        let second_direction =
            decided_parameter_cmp(second.start(), second.end(), &self.data.policy)?;
        if first_direction == Ordering::Equal || second_direction == Ordering::Equal {
            return Err(self.invalid(pair.first_carrier_index, CurveError::DegenerateOverlapRange));
        }
        let corresponding = (first_direction == second_direction)
            == (orientation == CurveOverlapOrientation2::Same);
        let second = if corresponding {
            second
        } else {
            CurveParameterRange2::new_validated(second.end().clone(), second.start().clone())
        };
        Ok((first, second))
    }

    fn clipped_overlap_ranges(
        &self,
        pair: &RegionCarrierPair,
        overlap: &CurveIntersectionOverlap2,
    ) -> ExactCurveResult<Option<(CurveParameterRange2, CurveParameterRange2)>> {
        let first_carrier = &self.data.carriers[pair.first_carrier_index];
        let second_carrier = &self.data.carriers[pair.second_carrier_index];
        let first_intersects =
            ranges_intersect(&overlap.first_range, first_carrier, &self.data.policy)?;
        let second_intersects =
            ranges_intersect(&overlap.second_range, second_carrier, &self.data.policy)?;
        if !first_intersects || !second_intersects {
            return Ok(None);
        }
        let same_parameter_domain = |first: &CurveParameter2, second: &CurveParameter2| {
            (first.as_bezier_parameter().is_some() && second.as_bezier_parameter().is_some())
                || (first.as_selected_fiber().is_some() && second.as_selected_fiber().is_some())
                || (first.as_recursive_projective().is_some()
                    && second.as_recursive_projective().is_some())
                || (first.is_algebraic_chord() && second.is_algebraic_chord())
                || (first.is_algebraic_cusp() && second.is_algebraic_cusp())
        };
        let range_uses_carrier_domain = |range: &CurveParameterRange2, carrier: &RegionCarrier| {
            same_parameter_domain(range.start(), &carrier.start)
                && same_parameter_domain(range.end(), &carrier.end)
        };
        if range_inside_carrier(&overlap.first_range, first_carrier, &self.data.policy)?
            && range_inside_carrier(&overlap.second_range, second_carrier, &self.data.policy)?
            && range_uses_carrier_domain(&overlap.first_range, first_carrier)
            && range_uses_carrier_domain(&overlap.second_range, second_carrier)
        {
            return Ok(Some((
                overlap.first_range.clone(),
                overlap.second_range.clone(),
            )));
        }
        match overlap
            .restrict_raw(
                &first_carrier.range(),
                &second_carrier.range(),
                &self.data.policy,
            )
            .map_err(|cause| self.invalid(pair.first_carrier_index, cause))?
        {
            Classification::Decided(overlap) => {
                Ok(overlap.map(|overlap| (overlap.first_range, overlap.second_range)))
            }
            Classification::Uncertain(reason) => {
                Err(self.blocked(pair.first_carrier_index, reason))
            }
        }
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

    /// Resolves result-side actions after complete pair replay.
    ///
    /// A representative ray remains the cheapest seed for most fragments.
    /// Some exact carriers deliberately live in a larger selected field than
    /// any materialized `Real` point, however. At a degree-two authored
    /// continuation no boundary is crossed and the same two faces continue on
    /// either side of the vertex. Propagating a decided neighboring action is
    /// therefore an exact topological certificate and avoids manufacturing a
    /// second algebraic coordinate field solely for point classification.
    fn regularized_fragment_actions(
        &self,
        topology: &CurveRegionSplitTopology,
        simple_loop_filled_side: Option<bool>,
    ) -> ExactCurveResult<RegularizedFragmentSelection> {
        if let Some(filled_side_is_left) = simple_loop_filled_side {
            let action = if filled_side_is_left {
                RegionFragmentAction::Keep
            } else {
                RegionFragmentAction::KeepReversed
            };
            let actions = topology
                .split_fragments
                .iter()
                .map(|splits| vec![action; splits.len()])
                .collect::<Vec<_>>();
            let successor_edge_ids = topology.split_fragments.iter().map(Vec::len).sum::<usize>();
            return Ok(RegularizedFragmentSelection {
                actions,
                successor_edge_ids: vec![NO_REGULARIZED_EDGE; successor_edge_ids],
            });
        }

        let mut actions = topology
            .split_fragments
            .iter()
            .map(|splits| vec![None; splits.len()])
            .collect::<Vec<_>>();
        let mut blockers = topology
            .split_fragments
            .iter()
            .map(|splits| vec![None; splits.len()])
            .collect::<Vec<_>>();
        let mut incidents = Vec::<Vec<(usize, usize, bool)>>::new();
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for (split_index, split) in splits.iter().enumerate() {
                for (vertex, is_start) in [
                    (split.start_topology_vertex, true),
                    (split.end_topology_vertex, false),
                ] {
                    let Some(vertex) = vertex else {
                        continue;
                    };
                    if incidents.len() <= vertex {
                        incidents.resize_with(vertex + 1, Vec::new);
                    }
                    incidents[vertex].push((carrier_index, split_index, is_start));
                }
            }
        }
        let authored_successor = |incoming: (usize, usize), outgoing: (usize, usize)| {
            let (incoming_carrier, incoming_split) = incoming;
            let (outgoing_carrier, outgoing_split) = outgoing;
            if incoming_carrier == outgoing_carrier {
                return incoming_split.checked_add(1) == Some(outgoing_split);
            }
            if incoming_split.checked_add(1)
                != Some(topology.split_fragments[incoming_carrier].len())
                || outgoing_split != 0
            {
                return false;
            }
            let incoming = &self.data.carriers[incoming_carrier];
            let outgoing = &self.data.carriers[outgoing_carrier];
            if incoming.operand != outgoing.operand || incoming.loop_index != outgoing.loop_index {
                return false;
            }
            outgoing.fragment_index == incoming.fragment_index.saturating_add(1)
                || outgoing.fragment_index == 0
                    && !self.data.carriers.iter().any(|candidate| {
                        candidate.operand == incoming.operand
                            && candidate.loop_index == incoming.loop_index
                            && candidate.fragment_index > incoming.fragment_index
                    })
        };

        let mut edge_offsets = Vec::with_capacity(topology.split_fragments.len());
        let mut edge_count = 0_usize;
        for splits in &topology.split_fragments {
            edge_offsets.push(edge_count);
            edge_count = edge_count.saturating_add(splits.len());
        }
        let mut edge_sources = vec![(0_usize, 0_usize); edge_count];
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for split_index in 0..splits.len() {
                edge_sources[edge_offsets[carrier_index] + split_index] =
                    (carrier_index, split_index);
            }
        }
        let edge_index = |carrier_index: usize, split_index: usize| {
            edge_offsets[carrier_index].saturating_add(split_index)
        };
        let left_face = |edge: usize| edge.saturating_mul(2);
        let right_face = |edge: usize| edge.saturating_mul(2).saturating_add(1);
        let mut vertex_sector_links = Vec::new();
        let mut winding_sector_links = Vec::new();
        let mut transverse_winding_sector_links = Vec::new();

        // An authored continuation with no transverse event preserves both
        // local face sectors exactly.
        for (vertex, incident) in incidents.iter().enumerate() {
            // Several pair contacts can share one vertex. No single pair's
            // continuation theorem then owns the complete cyclic order.
            if incident.len() > 2
                && !topology.contact_candidates.contains_key(&vertex)
                && let Some(vertex_sectors) = regularized_incident_ray_sectors(
                    incident,
                    topology,
                    &edge_offsets,
                    &self.data.policy,
                )
            {
                if let Some(sectors) = &vertex_sectors.sectors {
                    record_regularized_vertex_sectors(&mut vertex_sector_links, vertex, sectors);
                }
                winding_sector_links.extend_from_slice(&vertex_sectors.winding_links);
                transverse_winding_sector_links.extend(vertex_sectors.winding_links);
            }
            if incident.len() != 2
                || topology
                    .transverse_vertices
                    .get(vertex)
                    .copied()
                    .unwrap_or(false)
            {
                continue;
            }
            let (incoming, outgoing) = match (incident[0], incident[1]) {
                ((carrier, split, false), (next_carrier, next_split, true)) => {
                    ((carrier, split), (next_carrier, next_split))
                }
                ((next_carrier, next_split, true), (carrier, split, false)) => {
                    ((carrier, split), (next_carrier, next_split))
                }
                _ => continue,
            };
            if !authored_successor(incoming, outgoing) {
                continue;
            }
            let incoming = edge_index(incoming.0, incoming.1);
            let outgoing = edge_index(outgoing.0, outgoing.1);
            winding_sector_links.push((left_face(incoming), left_face(outgoing)));
            winding_sector_links.push((right_face(incoming), right_face(outgoing)));
        }

        let overlap_orientation_between_edges =
            |first_edge: usize, second_edge: usize| -> ExactCurveResult<Option<bool>> {
                let (first_carrier_index, first_split_index) = edge_sources[first_edge];
                let (second_carrier_index, second_split_index) = edge_sources[second_edge];
                let first_fragment =
                    &topology.split_fragments[first_carrier_index][first_split_index].fragment;
                let second_fragment =
                    &topology.split_fragments[second_carrier_index][second_split_index].fragment;
                let first_range = first_fragment.curve_region_parameter_range();
                let second_range = second_fragment.curve_region_parameter_range();
                let mut relation = None;
                for overlap in &topology.overlaps {
                    let ranges = if overlap.first_carrier_index == first_carrier_index
                        && overlap.second_carrier_index == second_carrier_index
                    {
                        Some((&overlap.first_range, &overlap.second_range))
                    } else if overlap.second_carrier_index == first_carrier_index
                        && overlap.first_carrier_index == second_carrier_index
                    {
                        Some((&overlap.second_range, &overlap.first_range))
                    } else {
                        None
                    };
                    let Some((first_overlap, second_overlap)) = ranges else {
                        continue;
                    };
                    if !range_contains_fragment(
                        first_overlap,
                        first_range.start(),
                        first_range.end(),
                        &self.data.policy,
                    )? || !range_contains_fragment(
                        second_overlap,
                        second_range.start(),
                        second_range.end(),
                        &self.data.policy,
                    )? {
                        continue;
                    }
                    let reversed = (overlap.orientation == CurveOverlapOrientation2::Reversed)
                        ^ self.data.carriers[first_carrier_index].reversed
                        ^ self.data.carriers[second_carrier_index].reversed;
                    match relation {
                        Some(existing) if existing != reversed => {
                            return Err(self.invalid(
                                first_carrier_index,
                                CurveError::Topology(
                                    "overlap orientations disagree at a contact sector".into(),
                                ),
                            ));
                        }
                        Some(_) => {}
                        None => relation = Some(reversed),
                    }
                }
                Ok(relation)
            };

        // Interior crossing and tangent certificates fix the local face
        // sectors without materializing the contact coordinate. At authored
        // corners, order the actual branches before linking their faces.
        for (&vertex, contact) in &topology.contact_candidates {
            let Some(incident) = incidents.get(vertex) else {
                continue;
            };
            let mut branches = [None; 4];
            for &(carrier_index, split_index, is_start) in incident {
                let branch = if contact.first_carrier != contact.second_carrier {
                    if carrier_index == contact.first_carrier {
                        Some(TransitionContactBranch::First)
                    } else if carrier_index == contact.second_carrier {
                        Some(TransitionContactBranch::Second)
                    } else {
                        None
                    }
                } else {
                    let split = &topology.split_fragments[carrier_index][split_index];
                    let range = split.fragment.curve_region_parameter_range();
                    self.transition_contact_branch(
                        topology,
                        carrier_index,
                        Some(vertex),
                        if is_start ^ self.data.carriers[carrier_index].reversed {
                            range.start()
                        } else {
                            range.end()
                        },
                    )?
                };
                let Some(branch) = branch else {
                    continue;
                };
                let slot = match (branch, is_start) {
                    (TransitionContactBranch::First, false) => 0,
                    (TransitionContactBranch::First, true) => 1,
                    (TransitionContactBranch::Second, false) => 2,
                    (TransitionContactBranch::Second, true) => 3,
                };
                let edge = edge_index(carrier_index, split_index);
                if branches[slot].replace(edge).is_some() {
                    branches[slot] = None;
                }
            }
            if incident.len() != 4 || branches.iter().any(Option::is_none) {
                if let Some(vertex_sectors) = regularized_incident_ray_sectors(
                    incident,
                    topology,
                    &edge_offsets,
                    &self.data.policy,
                ) {
                    if let Some(sectors) = &vertex_sectors.sectors {
                        record_regularized_vertex_sectors(
                            &mut vertex_sector_links,
                            vertex,
                            sectors,
                        );
                    }
                    winding_sector_links.extend_from_slice(&vertex_sectors.winding_links);
                    transverse_winding_sector_links.extend(vertex_sectors.winding_links);
                }
                continue;
            }
            let [
                Some(first_in),
                Some(first_out),
                Some(second_in),
                Some(second_out),
            ] = branches
            else {
                continue;
            };
            if first_in == second_in && first_out == second_out {
                // The pair replay may report the common authored endpoint of
                // consecutive carriers. Both pair identities then resolve to
                // the same boundary walk; it is a join, not a second branch.
                continue;
            }
            let first = &self.data.carriers[contact.first_carrier];
            let second = &self.data.carriers[contact.second_carrier];
            if let Some(source_cross_is_positive) = contact.cross_is_positive {
                let cross_is_positive = source_cross_is_positive ^ first.reversed ^ second.reversed;
                if overlap_orientation_between_edges(first_out, second_in)? == Some(true) {
                    let sector_pairs = if cross_is_positive {
                        [
                            (left_face(first_in), left_face(second_out)),
                            (right_face(second_out), left_face(first_out)),
                            (right_face(first_out), right_face(first_in)),
                        ]
                    } else {
                        [
                            (left_face(first_in), left_face(first_out)),
                            (right_face(first_out), left_face(second_out)),
                            (right_face(second_out), right_face(first_in)),
                        ]
                    };
                    // Coincident endpoint sectors are exact local traversal
                    // links, but the same face can occupy several pinched
                    // sectors at the vertex. The overlap group's aggregate
                    // edge below is the sole global winding constraint.
                    record_regularized_vertex_sectors(
                        &mut vertex_sector_links,
                        vertex,
                        &sector_pairs,
                    );
                    continue;
                }
                if overlap_orientation_between_edges(first_in, second_out)? == Some(true) {
                    let sector_pairs = if cross_is_positive {
                        [
                            (right_face(first_out), right_face(second_in)),
                            (left_face(second_in), right_face(first_in)),
                            (left_face(first_in), left_face(first_out)),
                        ]
                    } else {
                        [
                            (right_face(first_out), right_face(first_in)),
                            (left_face(first_in), right_face(second_in)),
                            (left_face(second_in), left_face(first_out)),
                        ]
                    };
                    record_regularized_vertex_sectors(
                        &mut vertex_sector_links,
                        vertex,
                        &sector_pairs,
                    );
                    continue;
                }
                let sector_pairs = if cross_is_positive {
                    [
                        (left_face(first_out), right_face(second_out)),
                        (right_face(first_out), right_face(second_in)),
                        (left_face(first_in), left_face(second_out)),
                        (right_face(first_in), left_face(second_in)),
                    ]
                } else {
                    [
                        (left_face(first_out), left_face(second_in)),
                        (right_face(first_out), left_face(second_out)),
                        (left_face(first_in), right_face(second_in)),
                        (right_face(first_in), right_face(second_out)),
                    ]
                };
                record_regularized_vertex_sectors(&mut vertex_sector_links, vertex, &sector_pairs);
                winding_sector_links.extend(sector_pairs);
                transverse_winding_sector_links.extend(sector_pairs);
                continue;
            }
            let (Some(mut same_direction), Some(mut side)) = (
                contact.tangent_dot_is_positive,
                contact.second_side_of_first,
            ) else {
                continue;
            };
            same_direction ^= first.reversed ^ second.reversed;
            if first.reversed {
                side = match side {
                    LineSide::Left => LineSide::Right,
                    LineSide::Right => LineSide::Left,
                    LineSide::On => continue,
                };
            }
            let sector_pairs = match (same_direction, side) {
                (true, LineSide::Left) => [
                    (left_face(first_in), right_face(second_in)),
                    (right_face(first_out), right_face(first_in)),
                    (left_face(second_in), left_face(second_out)),
                    (right_face(second_out), left_face(first_out)),
                ],
                (true, LineSide::Right) => [
                    (left_face(first_in), left_face(first_out)),
                    (right_face(second_out), right_face(second_in)),
                    (left_face(second_in), right_face(first_in)),
                    (right_face(first_out), left_face(second_out)),
                ],
                (false, LineSide::Left) => [
                    (left_face(first_in), left_face(second_out)),
                    (right_face(first_out), right_face(first_in)),
                    (left_face(second_in), left_face(first_out)),
                    (right_face(second_out), right_face(second_in)),
                ],
                (false, LineSide::Right) => [
                    (left_face(first_in), left_face(first_out)),
                    (left_face(second_in), left_face(second_out)),
                    (right_face(second_out), right_face(first_in)),
                    (right_face(first_out), right_face(second_in)),
                ],
                (_, LineSide::On) => continue,
            };
            record_regularized_vertex_sectors(&mut vertex_sector_links, vertex, &sector_pairs);
            winding_sector_links.extend(sector_pairs);
        }

        let mut edge_overlapped = topology
            .split_fragments
            .iter()
            .map(|splits| vec![false; splits.len()])
            .collect::<Vec<_>>();
        let mut edge_owns_overlap = topology
            .split_fragments
            .iter()
            .map(|splits| vec![true; splits.len()])
            .collect::<Vec<_>>();
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for (split_index, split) in splits.iter().enumerate() {
                let range = split.fragment.curve_region_parameter_range();
                for overlap in &topology.overlaps {
                    for (second, side_carrier_index, own_range, other_carrier_index) in [
                        (
                            false,
                            overlap.first_carrier_index,
                            &overlap.first_range,
                            overlap.second_carrier_index,
                        ),
                        (
                            true,
                            overlap.second_carrier_index,
                            &overlap.second_range,
                            overlap.first_carrier_index,
                        ),
                    ] {
                        if side_carrier_index != carrier_index {
                            continue;
                        }
                        let contains = match carrier_overlap_split_interval(overlap, second, splits)
                        {
                            Some(interval) => interval.contains(&split_index),
                            None => range_contains_fragment(
                                own_range,
                                range.start(),
                                range.end(),
                                &self.data.policy,
                            )?,
                        };
                        // The lower carrier owns a shared span; within one
                        // retraced carrier, its first overlap side owns it.
                        if contains {
                            edge_overlapped[carrier_index][split_index] = true;
                            if other_carrier_index < carrier_index
                                || (second && other_carrier_index == carrier_index)
                            {
                                edge_owns_overlap[carrier_index][split_index] = false;
                            }
                        }
                    }
                }
                if !edge_owns_overlap[carrier_index][split_index] {
                    actions[carrier_index][split_index] = Some(RegionFragmentAction::Discard);
                }
            }
        }

        let mut overlap_links = vec![Vec::<(usize, bool)>::new(); edge_count];
        for overlap in &topology.overlaps {
            let collect_edges = |carrier_index: usize,
                                 second: bool,
                                 overlap_range: &CurveParameterRange2|
             -> ExactCurveResult<Vec<usize>> {
                let mut edges = Vec::new();
                for (split_index, split) in
                    topology.split_fragments[carrier_index].iter().enumerate()
                {
                    let range = split.fragment.curve_region_parameter_range();
                    let contains = match carrier_overlap_split_interval(
                        overlap,
                        second,
                        &topology.split_fragments[carrier_index],
                    ) {
                        Some(interval) => interval.contains(&split_index),
                        None => range_contains_fragment(
                            overlap_range,
                            range.start(),
                            range.end(),
                            &self.data.policy,
                        )?,
                    };
                    if contains {
                        edges.push(edge_index(carrier_index, split_index));
                    }
                }
                Ok(edges)
            };
            let first_edges =
                collect_edges(overlap.first_carrier_index, false, &overlap.first_range)?;
            let second_edges =
                collect_edges(overlap.second_carrier_index, true, &overlap.second_range)?;
            let first_carrier = &self.data.carriers[overlap.first_carrier_index];
            let second_carrier = &self.data.carriers[overlap.second_carrier_index];
            let reversed = (overlap.orientation == CurveOverlapOrientation2::Reversed)
                ^ first_carrier.reversed
                ^ second_carrier.reversed;

            let mut pairs = Vec::new();
            if first_edges.len() == second_edges.len() {
                for (index, &first_edge) in first_edges.iter().enumerate() {
                    let second_index = if reversed {
                        second_edges.len() - 1 - index
                    } else {
                        index
                    };
                    pairs.push((first_edge, second_edges[second_index], reversed));
                }
            } else {
                let mut used = vec![false; second_edges.len()];
                for &first_edge in &first_edges {
                    let (first_carrier_index, first_split_index) = edge_sources[first_edge];
                    let first_split =
                        &topology.split_fragments[first_carrier_index][first_split_index];
                    let mut matched = None;
                    for (second_index, &second_edge) in second_edges.iter().enumerate() {
                        if used[second_index] {
                            continue;
                        }
                        let (second_carrier_index, second_split_index) = edge_sources[second_edge];
                        let second_split =
                            &topology.split_fragments[second_carrier_index][second_split_index];
                        let endpoint_match = if reversed {
                            first_split.start_topology_vertex == second_split.end_topology_vertex
                                && first_split.end_topology_vertex
                                    == second_split.start_topology_vertex
                        } else {
                            first_split.start_topology_vertex == second_split.start_topology_vertex
                                && first_split.end_topology_vertex
                                    == second_split.end_topology_vertex
                        };
                        if endpoint_match
                            && first_split.start_topology_vertex.is_some()
                            && first_split.end_topology_vertex.is_some()
                        {
                            if matched.is_some() {
                                matched = None;
                                break;
                            }
                            matched = Some((second_index, second_edge));
                        }
                    }
                    if let Some((second_index, second_edge)) = matched {
                        used[second_index] = true;
                        pairs.push((first_edge, second_edge, reversed));
                    }
                }
            }

            for (first_edge, second_edge, reversed) in pairs {
                overlap_links[first_edge].push((second_edge, reversed));
                overlap_links[second_edge].push((first_edge, reversed));
            }
        }

        // Source winding is local to an open face sector. A global component
        // can touch itself at a point while carrying different winding values
        // in those sectors, so retain one node per oriented edge side and use
        // explicit zero-jump links for the certified local adjacencies.
        let face_roots = topology
            .split_fragments
            .iter()
            .enumerate()
            .map(|(carrier_index, splits)| {
                splits
                    .iter()
                    .enumerate()
                    .map(|(split_index, _)| {
                        let edge = edge_index(carrier_index, split_index);
                        [left_face(edge), right_face(edge)]
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();

        let mut face_adjacency = vec![Vec::new(); edge_count.saturating_mul(2)];
        let mut transverse_face_adjacency = vec![Vec::new(); edge_count.saturating_mul(2)];
        let mut winding_jumps = Vec::new();
        let mut edge_overlap_grouped = vec![false; edge_count];
        let mut overlap_orientation = vec![None; edge_count];
        let mut invalid_face_roots = vec![false; edge_count.saturating_mul(2)];
        for edge in 0..edge_count {
            if overlap_links[edge].is_empty() || overlap_orientation[edge].is_some() {
                continue;
            }
            overlap_orientation[edge] = Some(false);
            let mut queue = std::collections::VecDeque::from([edge]);
            let mut members = Vec::new();
            while let Some(member) = queue.pop_front() {
                let orientation =
                    overlap_orientation[member].expect("queued overlap member has an orientation");
                members.push((member, orientation));
                edge_overlap_grouped[member] = true;
                for &(neighbor, reversed) in &overlap_links[member] {
                    let neighbor_orientation = orientation ^ reversed;
                    match overlap_orientation[neighbor] {
                        Some(existing) if existing != neighbor_orientation => {
                            return Err(self.invalid(
                                edge_sources[member].0,
                                CurveError::Topology(
                                    "coincident carrier orientations are inconsistent".into(),
                                ),
                            ));
                        }
                        Some(_) => {}
                        None => {
                            overlap_orientation[neighbor] = Some(neighbor_orientation);
                            queue.push_back(neighbor);
                        }
                    }
                }
            }
            let mut components = vec![0_i32; self.data.first.boundary_loops().len()];
            for &(member, reversed) in &members {
                let carrier_index = edge_sources[member].0;
                let loop_index = self.data.carriers[carrier_index].loop_index;
                let Some(component) = components.get_mut(loop_index) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "coincident carrier references a missing source loop".into(),
                        ),
                    ));
                };
                *component += if reversed { -1 } else { 1 };
            }
            let components = components
                .into_iter()
                .enumerate()
                .filter_map(|(loop_index, delta)| (delta != 0).then_some((loop_index, delta)))
                .collect::<Vec<_>>()
                .into_boxed_slice();
            if components.is_empty() {
                for &(member, _) in &members {
                    let (carrier_index, split_index) = edge_sources[member];
                    actions[carrier_index][split_index] = Some(RegionFragmentAction::Discard);
                }
                // Crossing a cancelling coincident group changes no loop's
                // winding, so its two sides carry equal winding vectors. Link
                // them exactly; otherwise every shared internal edge splits the
                // face equations and asks for another geometric seed.
                let [left, right] =
                    face_roots[edge_sources[members[0].0].0][edge_sources[members[0].0].1];
                if left != right {
                    let jump = winding_jumps.len();
                    winding_jumps.push(RegularizedWindingJump::Zero);
                    face_adjacency[right].push(RegularizedFaceAdjacency {
                        face: left,
                        jump,
                        direction: 0,
                    });
                    face_adjacency[left].push(RegularizedFaceAdjacency {
                        face: right,
                        jump,
                        direction: 0,
                    });
                }
                continue;
            }
            let reference = members[0].0;
            let [left, right] = face_roots[edge_sources[reference].0][edge_sources[reference].1];
            if left == right {
                // A contact-sector union may conservatively collapse both
                // sides of a same-image span at a multi-branch endpoint. A
                // nonzero aggregate winding jump cannot be a self-loop. Mark
                // only this connected face component unusable; disjoint exact
                // sector equations remain authoritative.
                invalid_face_roots[left] = true;
                continue;
            }
            let jump = winding_jumps.len();
            winding_jumps.push(RegularizedWindingJump::Aggregate(components));
            face_adjacency[right].push(RegularizedFaceAdjacency {
                face: left,
                jump,
                direction: 1,
            });
            face_adjacency[left].push(RegularizedFaceAdjacency {
                face: right,
                jump,
                direction: -1,
            });
        }
        for (carrier_index, roots) in face_roots.iter().enumerate() {
            let loop_index = self.data.carriers[carrier_index].loop_index;
            for (split_index, &[left, right]) in roots.iter().enumerate() {
                // Coincident carriers require the aggregate jump of their
                // overlap group. Until that group is formed below, leaving
                // this edge disconnected is exact and merely asks for a seed.
                let edge = edge_index(carrier_index, split_index);
                if edge_overlapped[carrier_index][split_index] {
                    continue;
                }
                if left == right {
                    // Contact-sector unions can conservatively collapse the
                    // two local sides of a neighboring non-overlap edge as
                    // well as an overlap edge. Quarantine that component and
                    // leave its per-fragment geometric classifier authoritative.
                    invalid_face_roots[left] = true;
                    continue;
                }
                debug_assert!(!edge_overlap_grouped[edge]);
                let jump = winding_jumps.len();
                winding_jumps.push(RegularizedWindingJump::Single(loop_index));
                face_adjacency[right].push(RegularizedFaceAdjacency {
                    face: left,
                    jump,
                    direction: 1,
                });
                face_adjacency[left].push(RegularizedFaceAdjacency {
                    face: right,
                    jump,
                    direction: -1,
                });
                transverse_face_adjacency[right].push(RegularizedFaceAdjacency {
                    face: left,
                    jump,
                    direction: 1,
                });
                transverse_face_adjacency[left].push(RegularizedFaceAdjacency {
                    face: right,
                    jump,
                    direction: -1,
                });
            }
        }
        if !winding_sector_links.is_empty() || !transverse_winding_sector_links.is_empty() {
            let zero = winding_jumps.len();
            winding_jumps.push(RegularizedWindingJump::Zero);
            for (first, second) in winding_sector_links {
                face_adjacency[first].push(RegularizedFaceAdjacency {
                    face: second,
                    jump: zero,
                    direction: 0,
                });
                face_adjacency[second].push(RegularizedFaceAdjacency {
                    face: first,
                    jump: zero,
                    direction: 0,
                });
            }
            for (first, second) in transverse_winding_sector_links {
                transverse_face_adjacency[first].push(RegularizedFaceAdjacency {
                    face: second,
                    jump: zero,
                    direction: 0,
                });
                transverse_face_adjacency[second].push(RegularizedFaceAdjacency {
                    face: first,
                    jump: zero,
                    direction: 0,
                });
            }
        }
        // A root can be found invalid after another edge has already added an
        // equation to it. Remove both directions of every such equation while
        // preserving independent exact face components.
        for face in 0..face_adjacency.len() {
            if invalid_face_roots[face] {
                face_adjacency[face].clear();
            } else {
                face_adjacency[face]
                    .retain(|edge| !invalid_face_roots.get(edge.face).copied().unwrap_or(true));
            }
        }
        // Keep direct overlap and geometric classifications separate from
        // actions inferred through sparse face equations. If later exact
        // evidence disproves an equation component, inference is rebuilt from
        // these authoritative actions instead of retaining stale decisions.
        let mut authoritative_actions = actions.clone();
        let mut face_windings = vec![None; edge_count.saturating_mul(2)];
        let mut transverse_face_windings = vec![None; edge_count.saturating_mul(2)];
        let mut face_equation_faces_valid = invalid_face_roots
            .iter()
            .map(|invalid| !invalid)
            .collect::<Vec<_>>();
        let mut transverse_face_equation_faces_valid = vec![true; edge_count.saturating_mul(2)];
        let action_from_windings = |carrier_index: usize,
                                    split_index: usize,
                                    windings: &[Option<LoopWindings>],
                                    reject_invalid_roots: bool|
         -> ExactCurveResult<Option<RegionFragmentAction>> {
            let [left_face, right_face] = face_roots[carrier_index][split_index];
            if reject_invalid_roots
                && (invalid_face_roots[left_face] || invalid_face_roots[right_face])
            {
                return Ok(None);
            }
            let (Some(left), Some(right)) =
                (windings[left_face].as_ref(), windings[right_face].as_ref())
            else {
                return Ok(None);
            };
            let left = self
                .location_from_loop_windings(self.data.first, left)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            let right = self
                .location_from_loop_windings(self.data.first, right)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            Ok(Some(action_from_result_sides(
                left == RegionPointLocation::Inside,
                right == RegionPointLocation::Inside,
            )))
        };

        // Exact affine fragments are the cheapest authoritative local face
        // seeds. Evaluate them first and propagate their winding equations.
        // Skip the global exterior ray only when those exact seeds already
        // cover every edge; otherwise the historical probe remains the
        // complete fallback rather than merely the no-line fallback.
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for (split_index, split) in splits.iter().enumerate() {
                if actions[carrier_index][split_index].is_some()
                    || edge_overlapped[carrier_index][split_index]
                    || !match &split.fragment {
                        BezierSplitFragment2::AlgebraicChord(chord) => chord.exact_line().is_some(),
                        BezierSplitFragment2::Materialized { .. } => {
                            split_fragment_is_affine_line(&split.fragment)
                        }
                        BezierSplitFragment2::RetainedBezier { .. }
                        | BezierSplitFragment2::AnalyticParallel(_)
                        | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        | BezierSplitFragment2::SelectedFiber(_) => false,
                    }
                {
                    continue;
                }
                // Earlier seeds may already have propagated both side windings
                // to this fragment. Leave its action to the derivation passes
                // below, which rebuild derived actions if a later seed
                // disables a contradictory component; a geometric ray probe
                // here would only rediscover the same face action.
                if action_from_windings(
                    carrier_index,
                    split_index,
                    &transverse_face_windings,
                    false,
                )?
                .is_some()
                    || action_from_windings(carrier_index, split_index, &face_windings, true)?
                        .is_some()
                {
                    continue;
                }
                let decision = self.regularized_fragment_geometric_decision(
                    carrier_index,
                    &split.fragment,
                    true,
                );
                match decision {
                    Ok(decision) => {
                        actions[carrier_index][split_index] = Some(decision.action);
                        authoritative_actions[carrier_index][split_index] = Some(decision.action);
                        let [left, right] = decision.side_windings;
                        let roots = face_roots[carrier_index][split_index];
                        seed_regularized_face_windings(
                            &mut transverse_face_windings,
                            &mut transverse_face_equation_faces_valid,
                            &transverse_face_adjacency,
                            &winding_jumps,
                            [(roots[0], left.clone()), (roots[1], right.clone())],
                        )
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                        seed_regularized_face_windings(
                            &mut face_windings,
                            &mut face_equation_faces_valid,
                            &face_adjacency,
                            &winding_jumps,
                            [(roots[0], left), (roots[1], right)]
                                .into_iter()
                                .filter(|(face, _)| !invalid_face_roots[*face]),
                        )
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    }
                    Err(error @ ExactCurveError::Blocked(_)) => {
                        blockers[carrier_index][split_index] = Some(error);
                    }
                    Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
                }
            }
        }
        let exact_local_seeds_cover_arrangement =
            topology
                .split_fragments
                .iter()
                .enumerate()
                .all(|(carrier_index, splits)| {
                    splits.iter().enumerate().all(|(split_index, _)| {
                        actions[carrier_index][split_index].is_some() || {
                            let [left, right] = face_roots[carrier_index][split_index];
                            !invalid_face_roots[left]
                                && !invalid_face_roots[right]
                                && face_windings[left].is_some()
                                && face_windings[right].is_some()
                        }
                    })
                });
        if !exact_local_seeds_cover_arrangement {
            let mut retained_bounds = Vec::with_capacity(self.data.carriers.len());
            for carrier in &self.data.carriers {
                let cached = carrier.bounds.get_or_init(|| {
                    carrier
                        .geometry
                        .certified_outer_bounds(&carrier.range(), 0, &self.data.policy)
                });
                let bounds = match cached {
                    Classification::Decided(bounds) => Some(bounds.clone()),
                    Classification::Uncertain(_) => None,
                };
                retained_bounds.push(bounds);
            }
            if retained_bounds.len() == self.data.carriers.len() {
                let approximate_magnitude = retained_bounds
                    .iter()
                    .flatten()
                    .flat_map(|bounds| {
                        [
                            bounds.min().x(),
                            bounds.min().y(),
                            bounds.max().x(),
                            bounds.max().y(),
                        ]
                    })
                    .filter_map(|coordinate| coordinate.to_f64_lossy())
                    .map(f64::abs)
                    .fold(1.0_f64, f64::max);
                let initial_magnitude = if approximate_magnitude.is_finite()
                    && approximate_magnitude < (i64::MAX / 4) as f64
                {
                    approximate_magnitude.ceil() as i64 + 2
                } else {
                    1
                };
                let mut magnitude = Real::from(initial_magnitude);
                let mut exterior_coordinates = None;
                for _ in 0..64 {
                    let negative = -magnitude.clone();
                    let outside_axis = |axis: Axis2,
                                        value: &Real,
                                        negative_side: bool|
                     -> ExactCurveResult<bool> {
                        for (carrier_index, (carrier, bounds)) in
                            self.data.carriers.iter().zip(&retained_bounds).enumerate()
                        {
                            let outside = if let Some(bounds) = bounds {
                                let boundary = match (axis, negative_side) {
                                    (Axis2::X, true) => bounds.min().x(),
                                    (Axis2::X, false) => bounds.max().x(),
                                    (Axis2::Y, true) => bounds.min().y(),
                                    (Axis2::Y, false) => bounds.max().y(),
                                };
                                compare_reals(value, boundary, &self.data.policy)
                                    == Some(if negative_side {
                                        Ordering::Less
                                    } else {
                                        Ordering::Greater
                                    })
                            } else if let CurveSupport2::Line(chord) = &carrier.geometry {
                                let expected = if negative_side {
                                    Ordering::Greater
                                } else {
                                    Ordering::Less
                                };
                                match chord
                                    .endpoints_strict_axis_order_to_real(
                                        axis,
                                        value,
                                        expected,
                                        &self.data.policy,
                                    )
                                    .map_err(|cause| self.invalid(carrier_index, cause))?
                                {
                                    Classification::Decided(outside) => outside,
                                    Classification::Uncertain(_) => false,
                                }
                            } else {
                                false
                            };
                            if !outside {
                                return Ok(false);
                            }
                        }
                        Ok(true)
                    };
                    let left = outside_axis(Axis2::X, &negative, true)?;
                    let right = outside_axis(Axis2::X, &magnitude, false)?;
                    let below = outside_axis(Axis2::Y, &negative, true)?;
                    let above = outside_axis(Axis2::Y, &magnitude, false)?;
                    if left || right || below || above {
                        exterior_coordinates =
                            Some((negative, magnitude.clone(), [left, right, below, above]));
                        if left && right && below && above {
                            break;
                        }
                    }
                    magnitude *= Real::from(2_u8);
                }
                let half = (Real::one() / Real::from(2_u8))
                    .map_err(|cause| self.invalid(0, cause.into()))?;
                let targets = topology
                    .split_fragments
                    .iter()
                    .flat_map(|splits| splits.iter())
                    .filter_map(|split| match &split.fragment {
                        BezierSplitFragment2::AlgebraicChord(chord) => {
                            chord.exact_line().map(|line| line.point_at(half.clone()))
                        }
                        BezierSplitFragment2::Materialized { .. }
                        | BezierSplitFragment2::RetainedBezier { .. }
                        | BezierSplitFragment2::AnalyticParallel(_)
                        | BezierSplitFragment2::SelectedFiber(_)
                        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
                    })
                    .collect::<Vec<_>>();
                'probe: for target in targets {
                    let Some((negative, positive, certified)) = &exterior_coordinates else {
                        break;
                    };
                    let outside_points = [
                        (
                            certified[0]
                                .then(|| crate::Point2::new(negative.clone(), target.y().clone())),
                            Axis2::X,
                            true,
                        ),
                        (
                            certified[1]
                                .then(|| crate::Point2::new(positive.clone(), target.y().clone())),
                            Axis2::X,
                            false,
                        ),
                        (
                            certified[2]
                                .then(|| crate::Point2::new(target.x().clone(), negative.clone())),
                            Axis2::Y,
                            true,
                        ),
                        (
                            certified[3]
                                .then(|| crate::Point2::new(target.x().clone(), positive.clone())),
                            Axis2::Y,
                            false,
                        ),
                    ];
                    for (outside, probe_axis, coordinate_increases) in outside_points {
                        let Some(outside) = outside else {
                            continue;
                        };
                        let probe = match crate::BezierAlgebraicChord2::try_new(
                            CurvePoint2::from(outside),
                            CurvePoint2::from(target.clone()),
                            &self.data.policy,
                        )
                        .map_err(|cause| self.invalid(0, cause))?
                        {
                            Classification::Decided(probe) => probe,
                            Classification::Uncertain(_) => continue,
                        };
                        let evidence = match self.intersect_algebraic_probe_boundary(probe, None) {
                            Ok(evidence) if evidence.overlaps().is_empty() => evidence,
                            Ok(_) | Err(_) => continue,
                        };
                        let mut contacts = evidence
                            .contacts()
                            .iter()
                            .filter(|contact| contact.is_certified_transverse())
                            .collect::<Vec<_>>();
                        let mut ordered = true;
                        for index in 1..contacts.len() {
                            let mut cursor = index;
                            while cursor > 0 {
                                let order = match contacts[cursor]
                                    .first_parameter()
                                    .cmp_by_refinement(
                                        contacts[cursor - 1].first_parameter(),
                                        &self.data.policy,
                                    )
                                    .map_err(|cause| self.invalid(0, cause))?
                                {
                                    Classification::Decided(order) => order,
                                    Classification::Uncertain(_) => {
                                        ordered = false;
                                        break;
                                    }
                                };
                                if order != Ordering::Less {
                                    break;
                                }
                                contacts.swap(cursor, cursor - 1);
                                cursor -= 1;
                            }
                            if !ordered {
                                break;
                            }
                        }
                        if !ordered {
                            continue;
                        }
                        for contact in contacts {
                            let Some(probe_parameter) =
                                contact.first_parameter().as_algebraic_chord()
                            else {
                                continue;
                            };
                            let contact_point = probe_parameter.point();
                            let mut blockers_are_after = true;
                            for blocker in evidence.blockers() {
                                let Some(carrier_index) =
                                    blocker.second().carrier_index().checked_sub(1)
                                else {
                                    blockers_are_after = false;
                                    break;
                                };
                                let Some(carrier) = self.data.carriers.get(carrier_index) else {
                                    blockers_are_after = false;
                                    break;
                                };
                                let expected_endpoint_order = if coordinate_increases {
                                    Ordering::Greater
                                } else {
                                    Ordering::Less
                                };
                                let after = if let CurveSupport2::Line(chord) = &carrier.geometry {
                                    let mut after = true;
                                    for endpoint in [chord.start(), chord.end()] {
                                        match crate::BezierAlgebraicChord2::point_axis_order(
                                            endpoint,
                                            contact_point,
                                            probe_axis,
                                            &self.data.policy,
                                        )
                                        .map_err(|cause| self.invalid(carrier_index, cause))?
                                        {
                                            Classification::Decided(order)
                                                if order == expected_endpoint_order => {}
                                            Classification::Decided(_)
                                            | Classification::Uncertain(_) => {
                                                after = false;
                                                break;
                                            }
                                        }
                                    }
                                    after
                                } else if let Some(Some(bounds)) =
                                    retained_bounds.get(carrier_index)
                                {
                                    let boundary = match (probe_axis, coordinate_increases) {
                                        (Axis2::X, true) => bounds.min().x(),
                                        (Axis2::X, false) => bounds.max().x(),
                                        (Axis2::Y, true) => bounds.min().y(),
                                        (Axis2::Y, false) => bounds.max().y(),
                                    };
                                    let expected_contact_order = if coordinate_increases {
                                        Ordering::Less
                                    } else {
                                        Ordering::Greater
                                    };
                                    crate::BezierAlgebraicChord2::point_axis_order_to_real(
                                        contact_point,
                                        probe_axis,
                                        boundary,
                                        &self.data.policy,
                                    )
                                    .map_err(|cause| self.invalid(carrier_index, cause))?
                                        == Classification::Decided(expected_contact_order)
                                } else {
                                    false
                                };
                                if !after {
                                    blockers_are_after = false;
                                    break;
                                }
                            }
                            if !blockers_are_after {
                                continue;
                            }
                            let Some(carrier_index) =
                                self.data.carriers.iter().position(|carrier| {
                                    carrier.loop_index == contact.second().loop_index()
                                        && carrier.fragment_index
                                            == contact.second().fragment_index()
                                })
                            else {
                                continue;
                            };
                            let parameter = contact.second_parameter();
                            let mut containing_split = None;
                            for (split_index, split) in
                                topology.split_fragments[carrier_index].iter().enumerate()
                            {
                                let range = split.fragment.curve_region_parameter_range();
                                let start = parameter
                                    .cmp_by_refinement(range.start(), &self.data.policy)
                                    .map_err(|cause| self.invalid(carrier_index, cause))?;
                                let end = parameter
                                    .cmp_by_refinement(range.end(), &self.data.policy)
                                    .map_err(|cause| self.invalid(carrier_index, cause))?;
                                if start == Classification::Decided(Ordering::Greater)
                                    && end == Classification::Decided(Ordering::Less)
                                {
                                    containing_split = Some(split_index);
                                    break;
                                }
                            }
                            let Some(split_index) = containing_split else {
                                continue;
                            };
                            let Some(mut cross_is_positive) =
                                contact.evidence.tangent_cross_is_positive()
                            else {
                                continue;
                            };
                            cross_is_positive ^= self.data.carriers[carrier_index].reversed;
                            let roots = face_roots[carrier_index][split_index];
                            let exterior_face = if cross_is_positive {
                                roots[0]
                            } else {
                                roots[1]
                            };
                            if invalid_face_roots[exterior_face] {
                                continue;
                            }
                            seed_regularized_face_windings(
                                &mut face_windings,
                                &mut face_equation_faces_valid,
                                &face_adjacency,
                                &winding_jumps,
                                [(
                                    exterior_face,
                                    vec![0; self.data.first.boundary_loops().len()],
                                )],
                            )
                            .map_err(|cause| self.invalid(carrier_index, cause))?;
                            break 'probe;
                        }
                    }
                }
            }
        }

        let update_actions_from_faces = |actions: &mut Vec<Vec<Option<RegionFragmentAction>>>,
                                         face_windings: &[Option<LoopWindings>],
                                         transverse_face_windings: &[Option<LoopWindings>],
                                         _blockers: &[Vec<Option<ExactCurveError>>]|
         -> ExactCurveResult<()> {
            for (derived_carrier_index, roots) in face_roots.iter().enumerate() {
                for derived_split_index in 0..roots.len() {
                    if actions[derived_carrier_index][derived_split_index].is_some() {
                        continue;
                    }
                    let derived_edge = edge_index(derived_carrier_index, derived_split_index);
                    if edge_overlapped[derived_carrier_index][derived_split_index]
                        && (!edge_overlap_grouped[derived_edge]
                            || !edge_owns_overlap[derived_carrier_index][derived_split_index])
                    {
                        continue;
                    }
                    let derived = action_from_windings(
                        derived_carrier_index,
                        derived_split_index,
                        transverse_face_windings,
                        false,
                    )?;
                    let Some(derived) = derived.or(action_from_windings(
                        derived_carrier_index,
                        derived_split_index,
                        face_windings,
                        true,
                    )?) else {
                        continue;
                    };
                    match actions[derived_carrier_index][derived_split_index] {
                        Some(existing) if existing != derived => {
                            return Err(self.invalid(
                                derived_carrier_index,
                                CurveError::Topology(
                                    "regularized face winding disagrees with a side classification"
                                        .into(),
                                ),
                            ));
                        }
                        Some(_) => {}
                        None => {
                            actions[derived_carrier_index][derived_split_index] = Some(derived);
                        }
                    }
                }
            }
            Ok(())
        };
        let propagate_authored_actions = |actions: &mut Vec<Vec<Option<RegionFragmentAction>>>,
                                          blockers: &[Vec<Option<ExactCurveError>>]|
         -> ExactCurveResult<()> {
            let mut changed = true;
            while changed {
                changed = false;
                for (vertex, incident) in incidents.iter().enumerate() {
                    if incident.len() != 2
                        || topology
                            .transverse_vertices
                            .get(vertex)
                            .copied()
                            .unwrap_or(false)
                    {
                        continue;
                    }
                    let (incoming, outgoing) = match (incident[0], incident[1]) {
                        ((carrier, split, false), (next_carrier, next_split, true)) => {
                            ((carrier, split), (next_carrier, next_split))
                        }
                        ((next_carrier, next_split, true), (carrier, split, false)) => {
                            ((carrier, split), (next_carrier, next_split))
                        }
                        _ => continue,
                    };
                    if !authored_successor(incoming, outgoing) {
                        continue;
                    }
                    // A coincident edge that lost deterministic overlap
                    // ownership is discarded as a duplicate image.  That
                    // ownership action belongs only to the overlap cell;
                    // it cannot propagate through the authored endpoint
                    // into an adjacent unique edge.
                    if edge_overlapped[incoming.0][incoming.1]
                        && !edge_owns_overlap[incoming.0][incoming.1]
                        || edge_overlapped[outgoing.0][outgoing.1]
                            && !edge_owns_overlap[outgoing.0][outgoing.1]
                    {
                        continue;
                    }
                    let first = actions[incoming.0][incoming.1];
                    let second = actions[outgoing.0][outgoing.1];
                    match (first, second) {
                        (Some(first), Some(second)) if first != second => {
                            return Err(self.invalid(
                                incoming.0,
                                CurveError::Topology(format!(
                                    "a degree-two authored continuation changed regularized faces at vertex {vertex}: incoming {incoming:?} is {first:?}, outgoing {outgoing:?} is {second:?}",
                                )),
                            ));
                        }
                        (Some(action), None) if blockers[outgoing.0][outgoing.1].is_some() => {
                            actions[outgoing.0][outgoing.1] = Some(action);
                            changed = true;
                        }
                        (None, Some(action)) if blockers[incoming.0][incoming.1].is_some() => {
                            actions[incoming.0][incoming.1] = Some(action);
                            changed = true;
                        }
                        (Some(_), Some(_)) | (Some(_), None) | (None, Some(_)) | (None, None) => {}
                    }
                }
            }
            Ok(())
        };

        update_actions_from_faces(
            &mut actions,
            &face_windings,
            &transverse_face_windings,
            &blockers,
        )?;
        propagate_authored_actions(&mut actions, &blockers)?;

        let mut work = topology
            .split_fragments
            .iter()
            .enumerate()
            .flat_map(|(carrier_index, splits)| {
                splits.iter().enumerate().map(move |(split_index, split)| {
                    let rank = match &split.fragment {
                        BezierSplitFragment2::AlgebraicChord(chord)
                            if chord.exact_line().is_some() =>
                        {
                            0_u8
                        }
                        BezierSplitFragment2::Materialized { .. }
                            if split_fragment_is_affine_line(&split.fragment) =>
                        {
                            0
                        }
                        BezierSplitFragment2::AlgebraicChord(_) => 1,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_) => 2,
                        BezierSplitFragment2::Materialized { .. }
                        | BezierSplitFragment2::RetainedBezier { .. } => 3,
                        BezierSplitFragment2::AnalyticParallel(_)
                        | BezierSplitFragment2::SelectedFiber(_) => 4,
                    };
                    (rank, carrier_index, split_index)
                })
            })
            .collect::<Vec<_>>();
        work.sort_by_key(|&(rank, _, _)| rank);
        loop {
            let mut equation_component_changed = false;
            for &(_, carrier_index, split_index) in &work {
                if actions[carrier_index][split_index].is_some() {
                    continue;
                }
                // Each successful seed propagates exact winding values before
                // the next work item. Consume those values immediately: a
                // second geometric probe can introduce an unrelated algebraic
                // field merely to rediscover an already certified face action.
                // Coincident cells still obey their chosen overlap ownership.
                let edge = edge_index(carrier_index, split_index);
                if !edge_overlapped[carrier_index][split_index]
                    || edge_overlap_grouped[edge] && edge_owns_overlap[carrier_index][split_index]
                {
                    let derived = match action_from_windings(
                        carrier_index,
                        split_index,
                        &transverse_face_windings,
                        false,
                    )? {
                        Some(action) => Some(action),
                        None => {
                            action_from_windings(carrier_index, split_index, &face_windings, true)?
                        }
                    };
                    if let Some(action) = derived {
                        actions[carrier_index][split_index] = Some(action);
                        continue;
                    }
                }
                let split = &topology.split_fragments[carrier_index][split_index];
                let decision = self.regularized_fragment_geometric_decision(
                    carrier_index,
                    &split.fragment,
                    !edge_overlapped[carrier_index][split_index],
                );
                match decision {
                    Ok(decision) => {
                        actions[carrier_index][split_index] = Some(decision.action);
                        authoritative_actions[carrier_index][split_index] = Some(decision.action);
                        blockers[carrier_index][split_index] = None;
                        let [left, right] = decision.side_windings;
                        let roots = face_roots[carrier_index][split_index];
                        let transverse_changed = seed_regularized_face_windings(
                            &mut transverse_face_windings,
                            &mut transverse_face_equation_faces_valid,
                            &transverse_face_adjacency,
                            &winding_jumps,
                            [(roots[0], left.clone()), (roots[1], right.clone())],
                        )
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                        let face_changed = seed_regularized_face_windings(
                            &mut face_windings,
                            &mut face_equation_faces_valid,
                            &face_adjacency,
                            &winding_jumps,
                            [(roots[0], left), (roots[1], right)]
                                .into_iter()
                                .filter(|(face, _)| !invalid_face_roots[*face]),
                        )
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                        if transverse_changed || face_changed {
                            // A declined equation component may have supplied
                            // earlier actions. Rebuild every derived decision
                            // from the surviving equations and direct exact
                            // classifications before continuing the worklist.
                            actions.clone_from(&authoritative_actions);
                            update_actions_from_faces(
                                &mut actions,
                                &face_windings,
                                &transverse_face_windings,
                                &blockers,
                            )?;
                            propagate_authored_actions(&mut actions, &blockers)?;
                            equation_component_changed = true;
                            break;
                        }
                    }
                    Err(error @ ExactCurveError::Blocked(_)) => {
                        blockers[carrier_index][split_index] = Some(error);
                    }
                    Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
                }
            }
            if !equation_component_changed {
                break;
            }
        }

        propagate_authored_actions(&mut actions, &blockers)?;
        update_actions_from_faces(
            &mut actions,
            &face_windings,
            &transverse_face_windings,
            &blockers,
        )?;
        propagate_authored_actions(&mut actions, &blockers)?;

        let mut decided = Vec::with_capacity(actions.len());
        for (carrier_index, carrier_actions) in actions.into_iter().enumerate() {
            let mut carrier_decided = Vec::with_capacity(carrier_actions.len());
            for (split_index, action) in carrier_actions.into_iter().enumerate() {
                let Some(action) = action else {
                    let error = blockers[carrier_index][split_index]
                        .clone()
                        .unwrap_or_else(|| {
                            self.blocked(carrier_index, UncertaintyReason::Predicate)
                        });
                    return Err(error);
                };
                carrier_decided.push(action);
            }
            decided.push(carrier_decided);
        }
        let mut successor_edge_ids = vec![NO_REGULARIZED_EDGE; edge_count];
        let oriented_sector_edge = |vertex: usize, sector: usize| {
            let edge = sector / 2;
            let &(carrier_index, split_index) = edge_sources.get(edge)?;
            let action = decided[carrier_index][split_index];
            let filled_sector = match action {
                RegionFragmentAction::Keep => left_face(edge),
                RegionFragmentAction::KeepReversed => right_face(edge),
                RegionFragmentAction::Discard => return None,
            };
            if sector != filled_sector {
                return None;
            }
            let split = &topology.split_fragments[carrier_index][split_index];
            let (start, end) = match action {
                RegionFragmentAction::Keep => {
                    (split.start_topology_vertex, split.end_topology_vertex)
                }
                RegionFragmentAction::KeepReversed => {
                    (split.end_topology_vertex, split.start_topology_vertex)
                }
                RegionFragmentAction::Discard => unreachable!(),
            };
            if start == Some(vertex) {
                Some((edge, true))
            } else if end == Some(vertex) {
                Some((edge, false))
            } else {
                None
            }
        };
        for &(vertex, first_sector, second_sector) in &vertex_sector_links {
            let (Some(first), Some(second)) = (
                oriented_sector_edge(vertex, first_sector),
                oriented_sector_edge(vertex, second_sector),
            ) else {
                continue;
            };
            let (incoming, outgoing) = match (first, second) {
                ((incoming, false), (outgoing, true)) | ((outgoing, true), (incoming, false)) => {
                    (incoming, outgoing)
                }
                ((_, true), (_, true)) | ((_, false), (_, false)) => continue,
            };
            match successor_edge_ids[incoming] {
                NO_REGULARIZED_EDGE => {
                    successor_edge_ids[incoming] = outgoing;
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "regularization-successor",
                        "face-sector",
                    );
                }
                AMBIGUOUS_REGULARIZED_EDGE => {}
                existing if existing == outgoing => {}
                _ => successor_edge_ids[incoming] = AMBIGUOUS_REGULARIZED_EDGE,
            }
        }
        Ok(RegularizedFragmentSelection {
            actions: decided,
            successor_edge_ids,
        })
    }

    fn certified_simple_single_loop_filled_side(
        &self,
        topology: &CurveRegionSplitTopology,
    ) -> Option<bool> {
        if self.data.first.boundary_loops().len() != 1
            || self.data.carriers.is_empty()
            || topology.split_fragments.len() != self.data.carriers.len()
            || !topology.overlaps.is_empty()
            || self.data.carriers.iter().any(|carrier| {
                carrier.operand != CurveRegionBooleanOperand2::First
                    || carrier.loop_index != 0
                    || !carrier_has_certified_injective_image(carrier, &self.data.policy)
            })
        {
            return None;
        }
        let Ok(Classification::Decided(roles)) = self.data.first.loop_roles_raw(&self.data.policy)
        else {
            return None;
        };
        if roles.as_slice() != [CurveRegionLoopRole::Material] {
            return None;
        }
        let Ok(Classification::Decided(filled_sides)) =
            self.data.first.filled_side_is_left_raw(&self.data.policy)
        else {
            return None;
        };
        let [filled_side_is_left] = filled_sides else {
            return None;
        };

        // Complete pair replay and splitting have already run. A simple
        // authored loop therefore has one unsplit fragment per injective
        // carrier, each end joined only to the next authored start. Requiring
        // every authored start vertex to be distinct excludes nonadjacent
        // endpoint aliases and pinched walks without allocating a side sample
        // or materializing an algebraic carrier coordinate.
        for (index, splits) in topology.split_fragments.iter().enumerate() {
            let [split] = splits.as_slice() else {
                return None;
            };
            let start = split.start_topology_vertex?;
            let end = split.end_topology_vertex?;
            if start == end {
                return None;
            }
            let next = topology
                .split_fragments
                .get((index + 1) % topology.split_fragments.len())?;
            let [next] = next.as_slice() else {
                return None;
            };
            if next.start_topology_vertex != Some(end) {
                return None;
            }
            for previous in &topology.split_fragments[..index] {
                let [previous] = previous.as_slice() else {
                    return None;
                };
                if previous.start_topology_vertex == Some(start) {
                    return None;
                }
            }
        }
        Some(*filled_side_is_left)
    }

    fn regularized_fragment_geometric_decision(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        has_single_boundary_jump: bool,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        // A native side ray may fail an equality that the retained boundary
        // probe can replay from construction evidence. Exhaust both exact
        // routes before a local comparison may consume approximation.
        if self.data.policy.permits_approximate_512() {
            match self.data.policy.strict_predicate_pass(|| {
                self.regularized_fragment_geometric_decision(
                    carrier_index,
                    fragment,
                    has_single_boundary_jump,
                )
            }) {
                Err(ExactCurveError::Blocked(_)) => {}
                result => return result,
            }
        }
        if let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment {
            return self.regularized_algebraic_cusp_fragment_decision(carrier_index, fragment);
        }
        if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
            return self.regularized_algebraic_chord_fragment_decision(
                carrier_index,
                chord,
                has_single_boundary_jump,
            );
        }
        let carrier = &self.data.carriers[carrier_index];
        let max_representatives = match &carrier.geometry {
            CurveSupport2::Bezier(
                BezierSubcurve2::Quadratic(_) | BezierSubcurve2::RationalQuadratic(_),
            ) => 4,
            CurveSupport2::Bezier(BezierSubcurve2::Cubic(_)) => 6,
            CurveSupport2::Bezier(BezierSubcurve2::Rational(curve)) => {
                curve.degree().saturating_mul(2).max(2)
            }
            CurveSupport2::Parallel(_) => 4,
            CurveSupport2::Line(_) => 2,
            CurveSupport2::Circle(_) => 4,
        };
        let fragment_parameter_range = fragment.curve_region_parameter_range();
        let (start, end) = (
            fragment_parameter_range.start(),
            fragment_parameter_range.end(),
        );
        let mut upper = end.clone();
        let mut last_reason = UncertaintyReason::Boundary;
        // Retained circular-conic provenance is a construction certificate for
        // a proper rational parametrization of a nondegenerate minor arc.
        let retained_regular_circle = matches!(
            &carrier.geometry,
            CurveSupport2::Bezier(curve) if retained_circular_support(curve).is_some()
        );
        // General rational conversion is only a fallback certificate source.
        // Avoid rebuilding a circular quadratic whose native provenance has
        // already supplied everything this classifier needs.
        let rational_geometry = if retained_regular_circle {
            None
        } else {
            match &carrier.geometry {
                CurveSupport2::Bezier(curve) => RationalBezier2::try_from_subcurve(curve).ok(),
                CurveSupport2::Parallel(_) | CurveSupport2::Line(_) | CurveSupport2::Circle(_) => {
                    None
                }
            }
        };
        let isolator_touches =
            |parameter: &BezierParameter2, boundary: &Real, use_interval_start: bool| {
                match parameter.known_interval(&self.data.policy) {
                    Ok(Classification::Decided(interval)) => {
                        compare_reals(
                            if use_interval_start {
                                interval.start()
                            } else {
                                interval.end()
                            },
                            boundary,
                            &self.data.policy,
                        ) == Some(Ordering::Equal)
                    }
                    Ok(Classification::Uncertain(_)) | Err(_) => false,
                }
            };
        // An endpoint witness is only needed when the adjacent algebraic
        // isolator still touches that endpoint. Otherwise the represented
        // interior gap is cheaper and avoids an unnecessary boundary ray.
        let source_endpoint_witness = if retained_regular_circle
            && carrier.start == *start
            && start.scalar().is_some_and(|boundary| {
                end.as_bezier_parameter()
                    .is_some_and(|end| isolator_touches(end, boundary, true))
            }) {
            start.scalar().cloned()
        } else if retained_regular_circle
            && carrier.end == *end
            && end.scalar().is_some_and(|boundary| {
                start
                    .as_bezier_parameter()
                    .is_some_and(|start| isolator_touches(start, boundary, false))
            })
        {
            end.scalar().cloned()
        } else {
            None
        };
        let mut source_endpoint_witness_attempted = false;
        let mut boundary_probe_representative = None;
        for _ in 0..max_representatives {
            // Keep the original chart and its parameter as the winding
            // authority. Sampling a rematerialized circular fragment both
            // expands its coordinate expressions and discards that identity.
            let (parameter, representative, derivative) = {
                let parameter = if !source_endpoint_witness_attempted {
                    source_endpoint_witness_attempted = true;
                    source_endpoint_witness.clone()
                } else {
                    None
                };
                let parameter = match parameter {
                    Some(parameter) => parameter,
                    None => {
                        let parameter = match start
                            .strict_scalar_between_ordered(&upper, &self.data.policy)
                            .map_err(|cause| self.invalid(carrier_index, cause))?
                        {
                            Classification::Decided(parameter) => parameter,
                            Classification::Uncertain(reason) => {
                                last_reason = reason;
                                break;
                            }
                        };
                        upper = CurveParameter2::from(BezierParameter2::Exact(parameter.clone()));
                        parameter
                    }
                };
                let retained_endpoint = source_endpoint_witness
                    .as_ref()
                    .filter(|endpoint| *endpoint == &parameter)
                    .and_then(|endpoint| match &carrier.geometry {
                        CurveSupport2::Bezier(curve) if endpoint == &crate::Real::zero() => {
                            Some(curve.endpoint_refs().0.clone())
                        }
                        CurveSupport2::Bezier(curve) if endpoint == &crate::Real::one() => {
                            Some(curve.endpoint_refs().1.clone())
                        }
                        CurveSupport2::Bezier(_)
                        | CurveSupport2::Parallel(_)
                        | CurveSupport2::Line(_)
                        | CurveSupport2::Circle(_) => None,
                    });
                let representative = match retained_endpoint {
                    Some(point) => point,
                    None => match carrier
                        .geometry
                        .point_at(&parameter, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            continue;
                        }
                    },
                };
                let derivative = match carrier
                    .geometry
                    .derivative_at(&parameter, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?
                {
                    Classification::Decided(derivative) => derivative,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        continue;
                    }
                };
                (parameter, representative, derivative)
            };
            let tangent_squared =
                derivative.dx() * derivative.dx() + derivative.dy() * derivative.dy();
            let regular = match crate::classify::is_zero(&tangent_squared, &self.data.policy) {
                Some(false) => true,
                Some(true) => {
                    last_reason = UncertaintyReason::Boundary;
                    false
                }
                None if retained_regular_circle => true,
                None => {
                    match rational_geometry.as_ref().map(|curve| {
                        curve.derivative_is_certified_nonzero_at(&parameter, &self.data.policy)
                    }) {
                        Some(Ok(Classification::Decided(true))) => true,
                        Some(Ok(Classification::Uncertain(reason))) => {
                            last_reason = reason;
                            false
                        }
                        Some(Ok(Classification::Decided(false))) | None => {
                            last_reason = UncertaintyReason::RealSign;
                            false
                        }
                        Some(Err(cause)) => return Err(self.invalid(carrier_index, cause)),
                    }
                }
            };
            if !regular {
                continue;
            }
            let (mut tangent_x, mut tangent_y) = (derivative.dx().clone(), derivative.dy().clone());
            if carrier.reversed {
                tangent_x = -tangent_x;
                tangent_y = -tangent_y;
            }
            let source_parameter = CurveParameter2::from(BezierParameter2::Exact(parameter));
            boundary_probe_representative =
                Some((representative.clone(), source_parameter.clone()));
            let left = match self.fragment_side_classification(
                carrier_index,
                &representative,
                Some(&source_parameter),
                &tangent_x,
                &tangent_y,
                true,
            ) {
                Ok(location) => location,
                Err(ExactCurveError::Blocked(blocker)) => {
                    last_reason = blocker.reason();
                    continue;
                }
                Err(error) => return Err(error),
            };
            let right = match self.fragment_side_classification(
                carrier_index,
                &representative,
                Some(&source_parameter),
                &tangent_x,
                &tangent_y,
                false,
            ) {
                Ok(location) => location,
                Err(ExactCurveError::Blocked(blocker)) => {
                    last_reason = blocker.reason();
                    continue;
                }
                Err(error) => return Err(error),
            };
            return Ok(RegularizedFragmentDecision::from_classified_sides(
                left, right,
            ));
        }
        if last_reason == UncertaintyReason::Boundary
            && let Some((representative, parameter)) = boundary_probe_representative
        {
            return self.regularized_fragment_decision_by_boundary_probe(
                carrier_index,
                CurvePoint2::from(representative),
                Some(&parameter),
            );
        }
        Err(self.blocked(carrier_index, last_reason))
    }

    fn regularized_algebraic_cusp_fragment_decision(
        &self,
        carrier_index: usize,
        fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let start = CurveParameter2::from_algebraic_cusp(fragment.start_parameter().clone());
        let end = CurveParameter2::from_algebraic_cusp(fragment.end_parameter().clone());
        let parameter = match start
            .strict_scalar_between_ordered(&end, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Err(self.blocked(carrier_index, reason)),
        };
        let representative = match fragment
            .semicircle()
            .point_at(&parameter, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                return self.regularized_algebraic_cusp_fragment_decision_by_probe(
                    carrier_index,
                    fragment,
                    &parameter,
                );
            }
            Classification::Uncertain(reason) => return Err(self.blocked(carrier_index, reason)),
        };
        let tangent = match fragment
            .semicircle()
            .tangent_at(&parameter, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => return Err(self.blocked(carrier_index, reason)),
        };
        let (Some(representative_point), Some((mut tangent_x, mut tangent_y))) = (
            representative.exact_point(&self.data.policy),
            tangent.exact_vector(&self.data.policy),
        ) else {
            match self.regularized_algebraic_cusp_fragment_decision_in_selected_field(
                carrier_index,
                fragment,
                &representative,
                &tangent,
            ) {
                Ok(decision) => return Ok(decision),
                Err(ExactCurveError::Blocked(_)) => {
                    return self.regularized_algebraic_cusp_fragment_decision_by_probe(
                        carrier_index,
                        fragment,
                        &parameter,
                    );
                }
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            }
        };
        if fragment.is_reversed() {
            tangent_x = -tangent_x;
            tangent_y = -tangent_y;
        }
        let source_parameter = CurveParameter2::from_algebraic_cusp(
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(parameter),
        );
        let left = self.fragment_side_classification(
            carrier_index,
            &representative_point,
            Some(&source_parameter),
            &tangent_x,
            &tangent_y,
            true,
        )?;
        let right = self.fragment_side_classification(
            carrier_index,
            &representative_point,
            Some(&source_parameter),
            &tangent_x,
            &tangent_y,
            false,
        )?;
        Ok(RegularizedFragmentDecision::from_classified_sides(
            left, right,
        ))
    }

    /// Same-selected-field accelerator for unary cusp side classification.
    /// Any incomplete sign or ray predicate falls through to the general
    /// retained boundary probe instead of becoming an operation blocker.
    fn regularized_algebraic_cusp_fragment_decision_in_selected_field(
        &self,
        carrier_index: usize,
        fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
        representative: &crate::RationalBezierAlgebraicPointImage2,
        tangent: &crate::RationalBezierAlgebraicTangentImage2,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let tangent_x = tangent
            .coordinate_sign(true, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?;
        let tangent_y = tangent
            .coordinate_sign(false, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?;
        let reverse_sign = |sign| match sign {
            RealSign::Negative => RealSign::Positive,
            RealSign::Zero => RealSign::Zero,
            RealSign::Positive => RealSign::Negative,
        };
        let tangent_x = tangent_x.map(|sign| {
            if fragment.is_reversed() {
                reverse_sign(sign)
            } else {
                sign
            }
        });
        let tangent_y = tangent_y.map(|sign| {
            if fragment.is_reversed() {
                reverse_sign(sign)
            } else {
                sign
            }
        });
        let left = self.algebraic_fragment_side_classification(
            carrier_index,
            representative,
            tangent_x,
            tangent_y,
            true,
        )?;
        let right = self.algebraic_fragment_side_classification(
            carrier_index,
            representative,
            tangent_x,
            tangent_y,
            false,
        )?;
        Ok(RegularizedFragmentDecision::from_classified_sides(
            left, right,
        ))
    }

    fn regularized_algebraic_cusp_fragment_decision_by_probe(
        &self,
        carrier_index: usize,
        fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
        parameter: &Real,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let target = match fragment
            .semicircle()
            .point_evidence_at(parameter, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };

        let mut last_reason = UncertaintyReason::Unsupported;
        let mut outer_bounds = None;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let mut accumulated = None::<Aabb2>;
            let mut complete = true;
            for carrier in &self.data.carriers {
                let bounds = match carrier.geometry.certified_outer_bounds(
                    &carrier.range(),
                    refinement_steps,
                    &self.data.policy,
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
                outer_bounds = accumulated;
                break;
            }
        }
        let Some(outer_bounds) = outer_bounds else {
            return Err(self.blocked(carrier_index, last_reason));
        };
        let one = Real::one();
        let outside_points = [
            crate::Point2::new(outer_bounds.min().x() - &one, outer_bounds.min().y() - &one),
            crate::Point2::new(outer_bounds.max().x() + &one, outer_bounds.min().y() - &one),
            crate::Point2::new(outer_bounds.max().x() + &one, outer_bounds.max().y() + &one),
            crate::Point2::new(outer_bounds.min().x() - &one, outer_bounds.max().y() + &one),
        ];

        for outside in outside_points {
            let probe = match crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(outside),
                target.clone(),
                &self.data.policy,
            )
            .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(probe) => probe,
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                    continue;
                }
            };
            let probe_end = CurveParameter2::from_algebraic_chord(probe.end_parameter());
            let evidence = match self.intersect_algebraic_probe_boundary(probe, None) {
                Ok(evidence) => evidence,
                Err(ExactCurveError::Blocked(blocker)) => {
                    last_reason = blocker.reason();
                    continue;
                }
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            };
            if !evidence.overlaps().is_empty() {
                last_reason = UncertaintyReason::Boundary;
                continue;
            }
            if let Some(blocker) = evidence.blockers().first() {
                last_reason = blocker
                    .uncertainty_reason()
                    .unwrap_or(UncertaintyReason::Unsupported);
                continue;
            }

            let mut crossings = Vec::<(CurveParameter2, usize, bool)>::new();
            let mut target_cross = None;
            let mut ambiguous = false;
            for contact in evidence.contacts() {
                let order = match contact
                    .first_parameter()
                    .cmp_by_refinement(&probe_end, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?
                {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        ambiguous = true;
                        break;
                    }
                };
                if order == Ordering::Greater {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a regularization boundary probe retained a contact past its endpoint"
                                .into(),
                        ),
                    ));
                }
                let Some(boundary_index) = contact.second().carrier_index().checked_sub(1) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a regularization boundary probe lost its source carrier".into(),
                        ),
                    ));
                };
                let Some(boundary) = self.data.carriers.get(boundary_index) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a regularization boundary probe referenced an unknown carrier".into(),
                        ),
                    ));
                };
                let Some(mut cross_is_positive) = contact.evidence.tangent_cross_is_positive()
                else {
                    if order == Ordering::Equal {
                        last_reason = UncertaintyReason::Boundary;
                        ambiguous = true;
                        break;
                    }
                    continue;
                };
                cross_is_positive ^= boundary.reversed;
                if order == Ordering::Equal {
                    if boundary_index != carrier_index
                        || target_cross.replace(cross_is_positive).is_some()
                    {
                        last_reason = UncertaintyReason::Boundary;
                        ambiguous = true;
                        break;
                    }
                    continue;
                }
                if !contact.is_certified_transverse() {
                    continue;
                }
                for (parameter, _, _) in &crossings {
                    match contact
                        .first_parameter()
                        .cmp_by_refinement(parameter, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(Ordering::Equal) => {
                            last_reason = UncertaintyReason::Boundary;
                            ambiguous = true;
                            break;
                        }
                        Classification::Decided(Ordering::Less | Ordering::Greater) => {}
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            ambiguous = true;
                            break;
                        }
                    }
                }
                if ambiguous {
                    break;
                }
                crossings.push((
                    contact.first_parameter().clone(),
                    boundary_index,
                    cross_is_positive,
                ));
            }
            if ambiguous {
                continue;
            }
            let Some(target_cross) = target_cross else {
                last_reason = UncertaintyReason::Boundary;
                continue;
            };

            let mut incoming = vec![0_i32; self.data.first.boundary_loops().len()];
            for (_, boundary_index, cross_is_positive) in crossings {
                let loop_index = self.data.carriers[boundary_index].loop_index;
                let Some(winding) = incoming.get_mut(loop_index) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a regularization probe crossing lost its boundary loop".into(),
                        ),
                    ));
                };
                *winding += if cross_is_positive { -1 } else { 1 };
            }
            let mut outgoing = incoming.clone();
            outgoing[self.data.carriers[carrier_index].loop_index] +=
                if target_cross { -1 } else { 1 };
            let (left_windings, right_windings) = if target_cross {
                (incoming, outgoing)
            } else {
                (outgoing, incoming)
            };
            let left = self
                .location_from_windings(self.data.first, &left_windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            let right = self
                .location_from_windings(self.data.first, &right_windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-regularization-cusp-side",
                "exact-boundary-probe",
            );
            return Ok(RegularizedFragmentDecision::from_classified_sides(
                (left_windings, left),
                (right_windings, right),
            ));
        }
        Err(self.blocked(carrier_index, last_reason))
    }

    fn regularized_algebraic_chord_fragment_decision(
        &self,
        carrier_index: usize,
        chord: &crate::BezierAlgebraicChord2,
        has_single_boundary_jump: bool,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let representative = match chord
            .representative_point(&self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        // Complete pair replay has split every transverse contact away from
        // this open fragment, and the caller excludes coincident spans.  Its
        // two local sides therefore differ by exactly this oriented source
        // edge's unit winding.  Classify one side and derive the other instead
        // of intersecting a second, antiparallel boundary ray with the entire
        // region.
        let opposite_from_left = |left: &(Vec<i32>, RegionPointLocation)| {
            if !has_single_boundary_jump {
                return Ok(None);
            }
            let loop_index = self.data.carriers[carrier_index].loop_index;
            let mut windings = left.0.clone();
            let Some(winding) = windings.get_mut(loop_index) else {
                return Err(self.invalid(
                    carrier_index,
                    CurveError::Topology(
                        "a regularized fragment references a missing source loop".into(),
                    ),
                ));
            };
            *winding = winding.checked_sub(1).ok_or_else(|| {
                self.invalid(
                    carrier_index,
                    CurveError::Topology("a regularized winding underflowed i32".into()),
                )
            })?;
            let location = self
                .location_from_windings(self.region_for_carrier(carrier_index), &windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "regularization-fragment-side",
                "single-boundary-winding-jump",
            );
            Ok(Some((windings, location)))
        };
        if let CurvePoint2(CurvePointData2::Exact(representative)) = representative {
            let tangent = chord
                .exact_line()
                .map(|line| line.delta())
                .or_else(|| {
                    chord
                        .strict_provenance_support_line(&self.data.policy)
                        .map(|line| line.delta())
                })
                .or_else(|| chord.certified_unit_tangent());
            let Some((tangent_x, tangent_y)) = tangent else {
                return self.regularized_fragment_decision_by_boundary_probe(
                    carrier_index,
                    CurvePoint2::from(representative),
                    None,
                );
            };
            let source_chord = match &self.data.carriers[carrier_index].geometry {
                CurveSupport2::Line(source) => source,
                CurveSupport2::Bezier(_)
                | CurveSupport2::Parallel(_)
                | CurveSupport2::Circle(_) => chord,
            };
            // `chord` is an ordered nonempty split of `source_chord`, and its
            // representative is strictly interior to that split.  This
            // construction is a stronger finite-domain certificate than
            // independently reordering the transformed endpoint fields.
            let parameter = CurveParameter2::from_algebraic_chord(
                source_chord.parameter_at_certified_interior_point(CurvePoint2::from(
                    representative.clone(),
                )),
            );
            let classify = |left| {
                self.fragment_side_classification(
                    carrier_index,
                    &representative,
                    Some(&parameter),
                    &tangent_x,
                    &tangent_y,
                    left,
                )
            };
            let left = match classify(true) {
                Ok(classification) => classification,
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.reason() == UncertaintyReason::Boundary =>
                {
                    return self.regularized_fragment_decision_by_boundary_probe(
                        carrier_index,
                        CurvePoint2::from(representative),
                        None,
                    );
                }
                Err(error) => return Err(error),
            };
            let right = match opposite_from_left(&left)? {
                Some(classification) => classification,
                None => match classify(false) {
                    Ok(classification) => classification,
                    Err(ExactCurveError::Blocked(blocker))
                        if blocker.reason() == UncertaintyReason::Boundary =>
                    {
                        return self.regularized_fragment_decision_by_boundary_probe(
                            carrier_index,
                            CurvePoint2::from(representative),
                            None,
                        );
                    }
                    Err(error) => return Err(error),
                },
            };
            return Ok(RegularizedFragmentDecision::from_classified_sides(
                left, right,
            ));
        }
        let representative = match representative {
            CurvePoint2(CurvePointData2::Algebraic(representative)) => representative,
            representative => {
                return self.regularized_fragment_decision_by_boundary_probe(
                    carrier_index,
                    representative,
                    None,
                );
            }
        };
        let tangent = |axis| {
            chord
                .tangent_axis_sign(axis, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))
        };
        let [tangent_x, tangent_y] = [tangent(Axis2::X)?, tangent(Axis2::Y)?];
        let classify = |left| {
            self.algebraic_fragment_side_classification(
                carrier_index,
                &representative,
                tangent_x,
                tangent_y,
                left,
            )
        };
        let left = match classify(true) {
            Ok(classification) => classification,
            Err(ExactCurveError::Blocked(blocker))
                if blocker.reason() == UncertaintyReason::Boundary =>
            {
                return self.regularized_fragment_decision_by_boundary_probe(
                    carrier_index,
                    CurvePoint2::from(representative),
                    None,
                );
            }
            Err(error) => return Err(error),
        };
        let right = match opposite_from_left(&left)? {
            Some(classification) => classification,
            None => match classify(false) {
                Ok(classification) => classification,
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.reason() == UncertaintyReason::Boundary =>
                {
                    return self.regularized_fragment_decision_by_boundary_probe(
                        carrier_index,
                        CurvePoint2::from(representative),
                        None,
                    );
                }
                Err(error) => return Err(error),
            },
        };
        Ok(RegularizedFragmentDecision::from_classified_sides(
            left, right,
        ))
    }

    /// Seeds both local faces when a representative lies on a coincident
    /// boundary stack or lives in more than one selected field.
    ///
    /// A certified exterior rational point has zero winding in every source
    /// loop. Complete carrier-pair replay orders the crossings of the retained
    /// probe ending at the fragment representative. The transverse endpoint
    /// contacts identify the incoming face, and their aggregate loop-winding
    /// jump identifies the opposite face. No Cartesian coordinate or finite
    /// side displacement is manufactured.
    #[cold]
    fn regularized_fragment_decision_by_boundary_probe(
        &self,
        carrier_index: usize,
        representative: CurvePoint2,
        source_parameter: Option<&CurveParameter2>,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let outer_bounds = match retained_probe_outer_bounds(&self.data.carriers, &self.data.policy)
        {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        let loop_count = self.data.first.boundary_loops().len();
        let source_loop_index = self.data.carriers[carrier_index].loop_index;
        if source_loop_index >= loop_count {
            return Err(self.invalid(
                carrier_index,
                CurveError::Topology(
                    "a regularization boundary probe references a missing source loop".into(),
                ),
            ));
        }
        let retained_probe = match (&representative, &self.data.carriers[carrier_index].geometry) {
            (
                CurvePoint2(CurvePointData2::AlgebraicChordPair(point)),
                CurveSupport2::Line(boundary),
            ) => point
                .exterior_axis_probe_avoiding(boundary, &outer_bounds, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?,
            _ => None,
        };
        let retained_probe_count = usize::from(retained_probe.is_some());
        let candidate_count = self
            .data
            .carriers
            .len()
            .saturating_mul(2)
            .saturating_add(5)
            .saturating_add(retained_probe_count);
        let mut last_reason = UncertaintyReason::Unsupported;
        let authored_successor = |first_index: usize, second_index: usize| {
            let first = &self.data.carriers[first_index];
            let second = &self.data.carriers[second_index];
            if first.loop_index != second.loop_index {
                return false;
            }
            let fragment_count = self.data.first.boundary_loops()[first.loop_index].len();
            first.fragment_index.checked_add(1) == Some(second.fragment_index)
                || second.fragment_index == 0
                    && first.fragment_index.checked_add(1) == Some(fragment_count)
        };

        'candidate: for candidate_index in 0..candidate_count {
            let probe = if candidate_index < retained_probe_count {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-regularization-chord-side",
                    "retained-axis-probe",
                );
                retained_probe
                    .as_ref()
                    .expect("the retained probe count reflects its value")
                    .0
                    .clone()
            } else {
                let Some(outside) = retained_probe_exterior_candidate(
                    &outer_bounds,
                    candidate_index - retained_probe_count,
                ) else {
                    continue;
                };
                match crate::BezierAlgebraicChord2::try_new(
                    CurvePoint2::from(outside),
                    representative.clone(),
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))?
                {
                    Classification::Decided(probe) => probe,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        continue;
                    }
                }
            };
            let probe_end = CurveParameter2::from_algebraic_chord(probe.end_parameter());
            let evidence = match self.intersect_algebraic_probe_boundary(
                probe,
                source_parameter.map(|parameter| (carrier_index, parameter)),
            ) {
                Ok(evidence) => evidence,
                Err(ExactCurveError::Blocked(blocker)) => {
                    last_reason = blocker.reason();
                    continue;
                }
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            };
            if !evidence.overlaps().is_empty() {
                last_reason = UncertaintyReason::Boundary;
                continue;
            }
            if let Some(blocker) = evidence.blockers().first() {
                last_reason = blocker
                    .uncertainty_reason()
                    .unwrap_or(UncertaintyReason::Unsupported);
                continue;
            }

            let mut has_nontransverse_endpoint_contact = false;
            for contact in evidence.contacts() {
                let order = contact
                    .first_parameter()
                    .cmp_by_refinement(&probe_end, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?;
                match order {
                    Classification::Decided(Ordering::Less) => {}
                    Classification::Decided(Ordering::Equal) => {
                        has_nontransverse_endpoint_contact |= !contact.is_certified_transverse();
                    }
                    Classification::Decided(Ordering::Greater) => {
                        return Err(self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "a retained boundary-side probe kept a contact past its endpoint"
                                    .into(),
                            ),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        continue 'candidate;
                    }
                }
            }

            let mut crossings =
                Vec::<(&CurveParameter2, &CurveParameter2, usize, bool)>::with_capacity(
                    evidence.contacts().len(),
                );
            for contact in evidence
                .contacts()
                .iter()
                .filter(|contact| contact.is_certified_transverse())
            {
                let Some(mut cross_is_positive) = contact.evidence.tangent_cross_is_positive()
                else {
                    last_reason = UncertaintyReason::Predicate;
                    continue 'candidate;
                };
                let Some(boundary_index) = contact.second().carrier_index().checked_sub(1) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a retained boundary-side contact resolved to its probe carrier".into(),
                        ),
                    ));
                };
                let Some(boundary) = self.data.carriers.get(boundary_index) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a retained boundary-side contact lost its boundary carrier".into(),
                        ),
                    ));
                };
                cross_is_positive ^= boundary.reversed;
                crossings.push((
                    contact.first_parameter(),
                    contact.second_parameter(),
                    boundary_index,
                    cross_is_positive,
                ));
            }
            for index in 1..crossings.len() {
                let mut cursor = index;
                while cursor > 0 {
                    match crossings[cursor]
                        .0
                        .cmp_by_refinement(crossings[cursor - 1].0, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(Ordering::Less) => {
                            crossings.swap(cursor, cursor - 1);
                            cursor -= 1;
                        }
                        Classification::Decided(Ordering::Equal | Ordering::Greater) => break,
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            continue 'candidate;
                        }
                    }
                }
            }

            let mut windings = vec![0_i32; loop_count];
            let mut group_start = 0_usize;
            while group_start < crossings.len() {
                let mut group_end = group_start + 1;
                while group_end < crossings.len() {
                    match crossings[group_end]
                        .0
                        .cmp_by_refinement(crossings[group_start].0, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(Ordering::Equal) => group_end += 1,
                        Classification::Decided(Ordering::Greater) => break,
                        Classification::Decided(Ordering::Less) => {
                            return Err(self.invalid(
                                carrier_index,
                                CurveError::Topology(
                                    "retained boundary-side probe crossings lost exact order"
                                        .into(),
                                ),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            continue 'candidate;
                        }
                    }
                }
                let group = &crossings[group_start..group_end];
                let endpoint_order = match group[0]
                    .0
                    .cmp_by_refinement(&probe_end, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?
                {
                    Classification::Decided(order @ (Ordering::Equal | Ordering::Less)) => order,
                    Classification::Decided(Ordering::Greater) => {
                        return Err(self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "retained boundary-side crossing followed its probe endpoint"
                                    .into(),
                            ),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        continue 'candidate;
                    }
                };

                let source_cross_is_positive = if endpoint_order == Ordering::Equal {
                    let mut source_cross = None;
                    for &(_, _, boundary_index, cross_is_positive) in group {
                        if boundary_index != carrier_index {
                            continue;
                        }
                        if source_cross.replace(cross_is_positive).is_some() {
                            last_reason = UncertaintyReason::Boundary;
                            continue 'candidate;
                        }
                    }
                    let Some(source_cross) = source_cross else {
                        last_reason = UncertaintyReason::Predicate;
                        continue 'candidate;
                    };
                    Some(source_cross)
                } else {
                    None
                };

                let mut endpoint_roles = Vec::with_capacity(group.len());
                for &(_, parameter, boundary_index, _) in group {
                    let boundary = &self.data.carriers[boundary_index];
                    let (start, end) = if boundary.reversed {
                        (&boundary.end, &boundary.start)
                    } else {
                        (&boundary.start, &boundary.end)
                    };
                    let endpoint_role = |endpoint| {
                        parameter
                            .same_value(endpoint, &self.data.policy)
                            .map_err(|cause| self.invalid(carrier_index, cause))
                    };
                    let at_start = match endpoint_role(start)? {
                        Classification::Decided(at_start) => at_start,
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            continue 'candidate;
                        }
                    };
                    let at_end = match endpoint_role(end)? {
                        Classification::Decided(at_end) => at_end,
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            continue 'candidate;
                        }
                    };
                    endpoint_roles.push([at_start, at_end]);
                }
                let mut group_deltas = vec![0_i32; loop_count];
                let mut consumed = vec![false; group.len()];
                for index in 0..group.len() {
                    if consumed[index] {
                        continue;
                    }
                    let (_, _, boundary_index, cross_is_positive) = group[index];
                    let mut partner = None;
                    for candidate in index + 1..group.len() {
                        if consumed[candidate] {
                            continue;
                        }
                        let candidate_index = group[candidate].2;
                        let adjacent = authored_successor(boundary_index, candidate_index)
                            && endpoint_roles[index][1]
                            && endpoint_roles[candidate][0]
                            || authored_successor(candidate_index, boundary_index)
                                && endpoint_roles[candidate][1]
                                && endpoint_roles[index][0];
                        if !adjacent {
                            continue;
                        }
                        if partner.replace(candidate).is_some() {
                            last_reason = UncertaintyReason::Boundary;
                            continue 'candidate;
                        }
                    }
                    consumed[index] = true;
                    let delta = if let Some(partner) = partner {
                        consumed[partner] = true;
                        if cross_is_positive == group[partner].3 {
                            if cross_is_positive { -1 } else { 1 }
                        } else {
                            0
                        }
                    } else if cross_is_positive {
                        -1
                    } else {
                        1
                    };
                    let loop_index = self.data.carriers[boundary_index].loop_index;
                    let Some(winding) = group_deltas.get_mut(loop_index) else {
                        return Err(self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "a retained boundary-side contact references a missing loop".into(),
                            ),
                        ));
                    };
                    *winding = winding.checked_add(delta).ok_or_else(|| {
                        self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "retained boundary-side winding overflowed i32".into(),
                            ),
                        )
                    })?;
                }
                if let Some(source_cross_is_positive) = source_cross_is_positive {
                    let mut opposite = windings.clone();
                    for (winding, delta) in opposite.iter_mut().zip(group_deltas) {
                        *winding = winding.checked_add(delta).ok_or_else(|| {
                            self.invalid(
                                carrier_index,
                                CurveError::Topology(
                                    "retained boundary-side winding overflowed i32".into(),
                                ),
                            )
                        })?;
                    }
                    let (left_windings, right_windings) = if source_cross_is_positive {
                        (windings, opposite)
                    } else {
                        (opposite, windings)
                    };
                    let left = self
                        .location_from_windings(self.data.first, &left_windings)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    let right = self
                        .location_from_windings(self.data.first, &right_windings)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-regularization-chord-side",
                        "retained-endpoint-winding-probe",
                    );
                    return Ok(RegularizedFragmentDecision::from_classified_sides(
                        (left_windings, left),
                        (right_windings, right),
                    ));
                }
                for (winding, delta) in windings.iter_mut().zip(group_deltas) {
                    *winding = winding.checked_add(delta).ok_or_else(|| {
                        self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "retained boundary-side winding overflowed i32".into(),
                            ),
                        )
                    })?;
                }
                group_start = group_end;
            }
            if has_nontransverse_endpoint_contact {
                last_reason = UncertaintyReason::Boundary;
                continue;
            }
            let Some((_, source_cross_is_positive)) = retained_probe
                .as_ref()
                .filter(|_| candidate_index < retained_probe_count)
            else {
                continue;
            };
            // This retained probe ends at a point constructed strictly inside
            // the split source support. If generic finite-incidence replay did
            // not rediscover that endpoint, its defining-support cross still
            // owns the exact missing source jump. The representative cannot
            // be another event because complete pair replay selected it in an
            // open split fragment.
            let source_cross_is_positive =
                *source_cross_is_positive ^ self.data.carriers[carrier_index].reversed;
            let mut opposite = windings.clone();
            opposite[source_loop_index] = opposite[source_loop_index]
                .checked_add(if source_cross_is_positive { -1 } else { 1 })
                .ok_or_else(|| {
                    self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "retained boundary-side winding overflowed i32".into(),
                        ),
                    )
                })?;
            let (left_windings, right_windings) = if source_cross_is_positive {
                (windings, opposite)
            } else {
                (opposite, windings)
            };
            let left = self
                .location_from_windings(self.data.first, &left_windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            let right = self
                .location_from_windings(self.data.first, &right_windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-regularization-chord-side",
                "retained-constructed-endpoint-winding-probe",
            );
            return Ok(RegularizedFragmentDecision::from_classified_sides(
                (left_windings, left),
                (right_windings, right),
            ));
        }
        Err(self.blocked(carrier_index, last_reason))
    }

    fn algebraic_fragment_side_classification(
        &self,
        carrier_index: usize,
        representative: &crate::RationalBezierAlgebraicPointImage2,
        tangent_x: Classification<RealSign>,
        tangent_y: Classification<RealSign>,
        left: bool,
    ) -> ExactCurveResult<(Vec<i32>, RegionPointLocation)> {
        let carrier = &self.data.carriers[carrier_index];
        let reverse_sign = |sign| match sign {
            RealSign::Negative => RealSign::Positive,
            RealSign::Zero => RealSign::Zero,
            RealSign::Positive => RealSign::Negative,
        };
        let mut last_reason = UncertaintyReason::RealSign;
        let tangent_x = match tangent_x {
            Classification::Decided(sign) => Some(sign),
            Classification::Uncertain(reason) => {
                last_reason = reason;
                None
            }
        };
        let tangent_y = match tangent_y {
            Classification::Decided(sign) => Some(sign),
            Classification::Uncertain(reason) => {
                last_reason = reason;
                None
            }
        };
        let normal_x = tangent_y.map(reverse_sign);
        let normal_y = tangent_x;
        let normal_x = if left {
            normal_x
        } else {
            normal_x.map(reverse_sign)
        };
        let normal_y = if left {
            normal_y
        } else {
            normal_y.map(reverse_sign)
        };
        let unit = |sign: Option<RealSign>| match sign {
            Some(RealSign::Negative) => -1_i8,
            Some(RealSign::Positive) => 1_i8,
            Some(RealSign::Zero) | None => 0_i8,
        };
        let x = unit(normal_x);
        let y = unit(normal_y);
        if x == 0 && y == 0 {
            return Err(self.blocked(carrier_index, last_reason));
        }
        let directions = [
            (x, 0_i8),
            (0_i8, y),
            (x, y),
            (x.saturating_mul(2), y),
            (x, y.saturating_mul(2)),
            (x.saturating_mul(3), y),
            (x, y.saturating_mul(3)),
        ];
        for (direction_x, direction_y) in directions {
            if direction_x == 0 && direction_y == 0 {
                continue;
            }
            let classification = self
                .region_for_carrier(carrier_index)
                .algebraic_loop_windings_from_boundary_side_ray(
                    representative,
                    Real::from(direction_x),
                    Real::from(direction_y),
                    carrier.loop_index,
                    carrier.fragment_index,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            match classification {
                Classification::Decided(windings) => {
                    let location = self
                        .location_from_windings(self.region_for_carrier(carrier_index), &windings)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    return Ok((windings, location));
                }
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                }
            }
        }
        Err(self.blocked(carrier_index, last_reason))
    }

    fn fragment_side_classification(
        &self,
        carrier_index: usize,
        representative: &crate::Point2,
        source_parameter: Option<&CurveParameter2>,
        tangent_x: &crate::Real,
        tangent_y: &crate::Real,
        left: bool,
    ) -> ExactCurveResult<(Vec<i32>, RegionPointLocation)> {
        self.fragment_side_classification_with_reference_tangent(
            carrier_index,
            representative,
            source_parameter,
            tangent_x,
            tangent_y,
            left,
            true,
        )
    }

    fn fragment_side_classification_with_reference_tangent(
        &self,
        carrier_index: usize,
        representative: &crate::Point2,
        source_parameter: Option<&CurveParameter2>,
        tangent_x: &crate::Real,
        tangent_y: &crate::Real,
        left: bool,
        source_follows_reference_tangent: bool,
    ) -> ExactCurveResult<(Vec<i32>, RegionPointLocation)> {
        let carrier = &self.data.carriers[carrier_index];
        let normal_x = if left {
            -tangent_y.clone()
        } else {
            tangent_y.clone()
        };
        let normal_y = if left {
            tangent_x.clone()
        } else {
            -tangent_x.clone()
        };
        let x_axis = match crate::classify::compare_reals(
            &normal_x,
            &crate::Real::zero(),
            &self.data.policy,
        ) {
            Some(Ordering::Greater) => Some((crate::Real::one(), crate::Real::zero())),
            Some(Ordering::Less) => Some((-crate::Real::one(), crate::Real::zero())),
            Some(Ordering::Equal) | None => None,
        };
        let y_axis = match crate::classify::compare_reals(
            &normal_y,
            &crate::Real::zero(),
            &self.data.policy,
        ) {
            Some(Ordering::Greater) => Some((crate::Real::zero(), crate::Real::one())),
            Some(Ordering::Less) => Some((crate::Real::zero(), -crate::Real::one())),
            Some(Ordering::Equal) | None => None,
        };
        // Axis rays keep the analytic line-incidence coefficients smallest;
        // the tangent-derived directions remain exact fallbacks when an axis
        // contact lands on a harder algebraic ordering boundary.
        let directions = [
            x_axis,
            y_axis,
            Some((normal_x.clone(), normal_y.clone())),
            Some((&normal_x + tangent_x, &normal_y + tangent_y)),
            Some((&normal_x - tangent_x, &normal_y - tangent_y)),
            Some((
                &normal_x * crate::Real::from(2_u8) + tangent_x,
                &normal_y * crate::Real::from(2_u8) + tangent_y,
            )),
            Some((
                &normal_x * crate::Real::from(2_u8) - tangent_x,
                &normal_y * crate::Real::from(2_u8) - tangent_y,
            )),
        ];
        let mut last_reason = UncertaintyReason::Boundary;
        for (direction_x, direction_y) in directions.into_iter().flatten() {
            let result = self
                .region_for_carrier(carrier_index)
                .loop_windings_from_boundary_side_ray(
                    representative,
                    direction_x,
                    direction_y,
                    true,
                    if left == source_follows_reference_tangent {
                        BezierLineCrossingDirection::PositiveToNegative
                    } else {
                        BezierLineCrossingDirection::NegativeToPositive
                    },
                    carrier.loop_index,
                    carrier.fragment_index,
                    source_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            match result {
                Classification::Decided(windings) => {
                    let location = self
                        .location_from_windings(self.region_for_carrier(carrier_index), &windings)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    return Ok((windings, location));
                }
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                }
            }
        }
        Err(self.blocked(carrier_index, last_reason))
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

    /// Classifies an already split open-curve piece against the other operand.
    /// The local range belongs to its original prepared source span.
    pub(crate) fn trim_piece_location(
        &self,
        carrier_index: usize,
        curve: &Curve2,
        range: &CurveParameterRange2,
    ) -> ExactCurveResult<RegionPointLocation> {
        if let Some(fragment) = curve.retained_fragment() {
            return self.fragment_location(carrier_index, fragment);
        }
        if let Some(spans) =
            curve.restricted_source_spans(&self.data.policy, CurveOperation2::Subdivision)?
        {
            let [span] = spans else {
                return Err(self.invalid(
                    carrier_index,
                    CurveError::Topology(
                        "a prepared curve trim piece must occupy one source span".into(),
                    ),
                ));
            };
            return self.fragment_location(carrier_index, &span.fragment);
        }
        let spans = curve.native_bezier_fragments_for_operation(
            &self.data.policy,
            CurveOperation2::Subdivision,
        )?;
        let [span] = spans else {
            return Err(self.invalid(
                carrier_index,
                CurveError::Topology(
                    "a prepared curve trim piece must occupy one source span".into(),
                ),
            ));
        };
        let Some((start, end)) = range.as_bezier_parameters() else {
            return Err(self.blocked(carrier_index, UncertaintyReason::Unsupported));
        };
        self.fragment_location(
            carrier_index,
            &BezierSplitFragment2::Materialized {
                start: start.clone(),
                end: end.clone(),
                curve: span.native_curve().clone(),
            },
        )
    }

    fn fragment_location(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
    ) -> ExactCurveResult<RegionPointLocation> {
        let carrier = &self.data.carriers[carrier_index];
        let (other, other_operand) = match carrier.operand {
            CurveRegionBooleanOperand2::First => {
                (&self.data.second, CurveRegionBooleanOperand2::Second)
            }
            CurveRegionBooleanOperand2::Second => {
                (&self.data.first, CurveRegionBooleanOperand2::First)
            }
        };
        if self.carrier_bounds_are_outside_other_region(carrier_index) {
            return Ok(RegionPointLocation::Outside);
        }
        let classification = if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
            {
                // Complete pair replay guarantees that an open split fragment
                // cannot change faces. Its interior support point is the
                // authoritative face witness; endpoints can coincide with
                // contacts or retained overlaps and are only a fallback when
                // that interior representation is unavailable.
                let classify_interior = || {
                    let representative = chord
                        .representative_point(&self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    let classification = match representative {
                        Classification::Decided(point) => self
                            .classify_point_evidence_off_boundary(
                                carrier_index,
                                point,
                                other,
                                other_operand,
                            )?,
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    };
                    Ok(classification)
                };
                let interior_classification = classify_interior()?;
                if let Classification::Decided(
                    location @ (RegionPointLocation::Inside | RegionPointLocation::Outside),
                ) = interior_classification
                {
                    return Ok(location);
                }
                let endpoint_classification =
                    self.classify_chord_endpoint_off_other_boundary(carrier_index, chord, other)?;
                if let Some(
                    classification @ Classification::Decided(
                        RegionPointLocation::Inside | RegionPointLocation::Outside,
                    ),
                ) = endpoint_classification
                {
                    classification
                } else {
                    let endpoint_reason = match endpoint_classification {
                        Some(Classification::Uncertain(reason)) => Some(reason),
                        Some(Classification::Decided(RegionPointLocation::Boundary))
                        | Some(Classification::Decided(
                            RegionPointLocation::Inside | RegionPointLocation::Outside,
                        ))
                        | None => None,
                    };
                    match interior_classification {
                        Classification::Decided(
                            location @ (RegionPointLocation::Inside | RegionPointLocation::Outside),
                        ) => Classification::Decided(location),
                        Classification::Decided(RegionPointLocation::Boundary) => {
                            Classification::Uncertain(UncertaintyReason::Boundary)
                        }
                        Classification::Uncertain(UncertaintyReason::Unsupported) => {
                            Classification::Uncertain(
                                endpoint_reason.unwrap_or(UncertaintyReason::Unsupported),
                            )
                        }
                        Classification::Uncertain(reason) => Classification::Uncertain(reason),
                    }
                }
            }
        } else if let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment {
            let parameter = match fragment
                .representative_parameter()
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Err(self.blocked(carrier_index, reason));
                }
            };
            let point = match fragment
                .semicircle()
                .point_evidence_at(&parameter, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Err(self.blocked(carrier_index, reason));
                }
            };
            self.classify_point_evidence_off_boundary(carrier_index, point, other, other_operand)?
        } else if let BezierSplitFragment2::SelectedFiber(fragment) = fragment {
            let representative = match fragment
                .representative_point(&self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Err(self.blocked(carrier_index, reason));
                }
            };
            other
                .classify_point_raw(&representative, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?
        } else {
            let (parameter, representative) =
                self.fragment_representative(carrier_index, fragment)?;
            let classification = other
                .classify_point_raw(&representative, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            if matches!(
                classification,
                Classification::Decided(RegionPointLocation::Inside | RegionPointLocation::Outside)
            ) {
                classification
            } else {
                // A symmetric interior witness can land on a tangent or
                // shared-boundary event even though the open arrangement
                // fragment lies in one face. Complete pair replay guarantees
                // that its face cannot change between split events, so probe
                // exact scalar witnesses on both sides before propagating a
                // boundary ambiguity.
                let Some((start, end)) = fragment_range(fragment) else {
                    return Err(self.blocked(carrier_index, UncertaintyReason::Unsupported));
                };
                let middle = BezierParameter2::Exact(parameter);
                let mut decided = None;
                for (left, right) in [(start, &middle), (&middle, end)] {
                    let witness = match left
                        .strict_scalar_between_ordered(right, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(witness) => witness,
                        Classification::Uncertain(_) => continue,
                    };
                    let point = match carrier
                        .geometry
                        .point_at(&witness, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(_) => continue,
                    };
                    match other
                        .classify_point_raw(&point, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(
                            location @ (RegionPointLocation::Inside | RegionPointLocation::Outside),
                        ) => match decided {
                            Some(previous) if previous != location => {
                                return Err(self.invalid(
                                    carrier_index,
                                    CurveError::Topology(
                                        "one split carrier fragment crossed two Boolean faces"
                                            .into(),
                                    ),
                                ));
                            }
                            Some(_) => {}
                            None => decided = Some(location),
                        },
                        Classification::Decided(RegionPointLocation::Boundary)
                        | Classification::Uncertain(_) => {}
                    }
                }
                decided.map_or(classification, Classification::Decided)
            }
        };
        match classification {
            Classification::Decided(location) => Ok(location),
            Classification::Uncertain(reason) => Err(self.blocked(carrier_index, reason)),
        }
    }

    fn classify_chord_endpoint_off_other_boundary(
        &self,
        carrier_index: usize,
        chord: &crate::BezierAlgebraicChord2,
        other_region: &CurveRegion2,
    ) -> ExactCurveResult<Option<Classification<RegionPointLocation>>> {
        let mut last_reason = None;
        for endpoint in [chord.start(), chord.end()] {
            let direct = match endpoint {
                CurvePoint2(CurvePointData2::Exact(point)) => Some(
                    other_region
                        .classify_point_raw(point, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?,
                ),
                CurvePoint2(CurvePointData2::Algebraic(point)) => Some(
                    other_region
                        .classify_algebraic_point_raw(point, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?,
                ),
                CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
                | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
                | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
                | CurvePoint2(CurvePointData2::AnalyticParallel(_))
                | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                    None
                }
            };
            if let Some(classification) = direct {
                match classification {
                    Classification::Decided(
                        location @ (RegionPointLocation::Inside | RegionPointLocation::Outside),
                    ) => return Ok(Some(Classification::Decided(location))),
                    Classification::Decided(RegionPointLocation::Boundary) => {
                        last_reason = Some(UncertaintyReason::Boundary);
                    }
                    Classification::Uncertain(reason) => last_reason = Some(reason),
                }
                continue;
            }
            for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
                let bounds = match endpoint {
                    CurvePoint2(CurvePointData2::Endpoint(point)) => {
                        point.bounds(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::AlgebraicChordPair(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::AlgebraicCuspChord(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::AlgebraicChordParallel(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::AnalyticParallel(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::Similarity(point)) => {
                        point.conservative_bounds_refined(refinement_steps, &self.data.policy)
                    }
                    CurvePoint2(CurvePointData2::Exact(_))
                    | CurvePoint2(CurvePointData2::Algebraic(_)) => unreachable!(),
                };
                let Classification::Decided(bounds) = bounds else {
                    continue;
                };
                let mut separated_from_boundary = true;
                for boundary in &self.data.carriers {
                    if boundary.operand == self.data.carriers[carrier_index].operand {
                        continue;
                    }
                    let Classification::Decided(boundary_bounds) =
                        boundary.bounds.get_or_init(|| {
                            boundary.geometry.certified_outer_bounds(
                                &boundary.range(),
                                0,
                                &self.data.policy,
                            )
                        })
                    else {
                        separated_from_boundary = false;
                        break;
                    };
                    if bounds.overlaps(boundary_bounds, &self.data.policy)
                        != Classification::Decided(false)
                    {
                        separated_from_boundary = false;
                        break;
                    }
                }
                if !separated_from_boundary {
                    continue;
                }
                let two = Real::from(2_i8);
                let representative = crate::Point2::new(
                    ((bounds.min().x() + bounds.max().x()) / &two)
                        .map_err(|cause| self.invalid(carrier_index, cause.into()))?,
                    ((bounds.min().y() + bounds.max().y()) / &two)
                        .map_err(|cause| self.invalid(carrier_index, cause.into()))?,
                );
                let classification = other_region
                    .classify_point_raw(&representative, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?;
                match classification {
                    Classification::Decided(
                        location @ (RegionPointLocation::Inside | RegionPointLocation::Outside),
                    ) => return Ok(Some(Classification::Decided(location))),
                    Classification::Decided(RegionPointLocation::Boundary) => {
                        last_reason = Some(UncertaintyReason::Boundary);
                    }
                    Classification::Uncertain(reason) => last_reason = Some(reason),
                }
                break;
            }
        }
        Ok(last_reason.map(Classification::Uncertain))
    }

    fn classify_point_evidence_off_boundary(
        &self,
        owner_carrier_index: usize,
        point: CurvePoint2,
        boundary_region: &CurveRegion2,
        boundary_operand: CurveRegionBooleanOperand2,
    ) -> ExactCurveResult<Classification<RegionPointLocation>> {
        let direct = match &point {
            CurvePoint2(CurvePointData2::Exact(point)) => Some(
                boundary_region
                    .classify_point_raw(point, &self.data.policy)
                    .map_err(|cause| self.invalid(owner_carrier_index, cause))?,
            ),
            CurvePoint2(CurvePointData2::Algebraic(point)) => Some(
                boundary_region
                    .classify_algebraic_point_off_boundary_raw(point, &self.data.policy)
                    .map_err(|cause| self.invalid(owner_carrier_index, cause))?,
            ),
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => None,
        };
        match direct {
            Some(decided @ Classification::Decided(_)) => Ok(decided),
            Some(Classification::Uncertain(_)) | None => self
                .classify_retained_point_off_boundary_by_probe(
                    owner_carrier_index,
                    point,
                    boundary_operand,
                    RetainedPointProbeClassification::FilledRegion,
                ),
        }
    }

    /// Classifies retained multi-field point evidence without materializing a
    /// rounded coordinate pair.
    ///
    /// A rational point outside a certified outer box is joined to the target
    /// by one retained algebraic chord. Complete pair replay against the
    /// selected operand then identifies the last transverse boundary crossing.
    /// The boundary's certified filled side determines which face contains the
    /// target. Four exterior corners are the fast candidates. If all are
    /// degenerate, `2n+1` distinct points on one certified exterior line
    /// exclude every direction through the `n` loop vertices and every
    /// direction that can overlap one of the `n` retained line images.
    fn classify_retained_point_off_boundary_by_probe(
        &self,
        owner_carrier_index: usize,
        point: CurvePoint2,
        boundary_operand: CurveRegionBooleanOperand2,
        classification_kind: RetainedPointProbeClassification,
    ) -> ExactCurveResult<Classification<RegionPointLocation>> {
        // A terminal equality on one unlucky probe direction must not preempt
        // another direction that has a strict separation proof. Exhaust the
        // complete finite probe set with terminals suppressed, then replay it
        // under APPROXIMATE_512 only when every strict direction is blocked.
        if self.data.policy.permits_approximate_512() {
            match self.data.policy.strict_predicate_pass(|| {
                self.classify_retained_point_off_boundary_by_probe_once(
                    owner_carrier_index,
                    point.clone(),
                    boundary_operand,
                    classification_kind,
                )
            }) {
                Ok(decided @ Classification::Decided(_)) => return Ok(decided),
                Ok(Classification::Uncertain(_)) | Err(ExactCurveError::Blocked(_)) => {}
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            }
        }
        self.classify_retained_point_off_boundary_by_probe_once(
            owner_carrier_index,
            point,
            boundary_operand,
            classification_kind,
        )
    }

    fn classify_retained_point_off_boundary_by_probe_once(
        &self,
        owner_carrier_index: usize,
        point: CurvePoint2,
        boundary_operand: CurveRegionBooleanOperand2,
        classification_kind: RetainedPointProbeClassification,
    ) -> ExactCurveResult<Classification<RegionPointLocation>> {
        let boundary_region = match boundary_operand {
            CurveRegionBooleanOperand2::First => self.data.first,
            CurveRegionBooleanOperand2::Second => self.data.second,
        };
        // Every context stores the first operand's carriers before the second.
        // Borrow that existing partition so repeated queries also reuse its
        // retained bounds and injectivity facts.
        let boundary_carriers = match boundary_operand {
            CurveRegionBooleanOperand2::First => {
                &self.data.carriers[..self.data.first_carrier_count]
            }
            CurveRegionBooleanOperand2::Second => {
                &self.data.carriers[self.data.first_carrier_count..]
            }
        };
        if boundary_carriers.is_empty() {
            return Ok(Classification::Decided(RegionPointLocation::Outside));
        }

        let mut last_reason = UncertaintyReason::Unsupported;
        let outer_bounds = match retained_probe_outer_bounds(boundary_carriers, &self.data.policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let fallback_count = boundary_carriers.len().saturating_mul(2).saturating_add(1);
        for candidate_index in 0..fallback_count.saturating_add(4) {
            let Some(outside) = retained_probe_exterior_candidate(&outer_bounds, candidate_index)
            else {
                continue;
            };
            let probe = match crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(outside),
                point.clone(),
                &self.data.policy,
            ) {
                Ok(Classification::Decided(probe)) => probe,
                Ok(Classification::Uncertain(reason)) => {
                    last_reason = reason;
                    continue;
                }
                // This candidate lies strictly outside the certified boundary
                // enclosure. Coincidence with it already locates the target;
                // a zero-length probe needs no intersections or winding replay.
                Err(CurveError::ZeroLengthLine) => {
                    return Ok(Classification::Decided(RegionPointLocation::Outside));
                }
                Err(cause) => return Err(self.invalid(owner_carrier_index, cause)),
            };
            let probe_end = CurveParameter2::from_algebraic_chord(probe.end_parameter());
            let evidence = match self.intersect_algebraic_probe_carriers(
                probe,
                boundary_region,
                boundary_carriers,
                None,
            ) {
                Ok(evidence) => evidence,
                Err(ExactCurveError::Blocked(blocker)) => {
                    last_reason = blocker.reason();
                    continue;
                }
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            };
            if !evidence.overlaps().is_empty() {
                last_reason = UncertaintyReason::Boundary;
                continue;
            }
            if let Some(blocker) = evidence.blockers().first() {
                last_reason = blocker
                    .uncertainty_reason()
                    .unwrap_or(UncertaintyReason::Unsupported);
                continue;
            }

            // Any contact at the probe endpoint proves that the target itself
            // lies on the boundary, including a nontransverse endpoint touch.
            let mut endpoint_is_boundary = false;
            let mut comparison_blocked = false;
            for contact in evidence.contacts() {
                match contact
                    .first_parameter()
                    .cmp_by_refinement(&probe_end, &self.data.policy)
                    .map_err(|cause| self.invalid(owner_carrier_index, cause))?
                {
                    Classification::Decided(Ordering::Equal) => {
                        endpoint_is_boundary = true;
                        break;
                    }
                    Classification::Decided(Ordering::Less) => {}
                    Classification::Decided(Ordering::Greater) => {
                        return Err(self.invalid(
                            owner_carrier_index,
                            CurveError::Topology(
                                "an exterior classification probe retained a contact past its endpoint"
                                    .into(),
                            ),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        comparison_blocked = true;
                        break;
                    }
                }
            }
            if endpoint_is_boundary {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "retained-point-probe",
                    "endpoint-boundary",
                );
                return Ok(Classification::Decided(RegionPointLocation::Boundary));
            }
            if comparison_blocked {
                continue;
            }

            let mut crossings =
                Vec::<(&CurveParameter2, bool, usize)>::with_capacity(evidence.contacts().len());
            let mut ambiguous = false;
            for contact in evidence
                .contacts()
                .iter()
                .filter(|contact| contact.is_certified_transverse())
            {
                let Some(mut cross_is_positive) = contact.evidence.tangent_cross_is_positive()
                else {
                    last_reason = UncertaintyReason::Predicate;
                    ambiguous = true;
                    break;
                };
                let Some(boundary_index) = contact.second().carrier_index().checked_sub(1) else {
                    return Err(self.invalid(
                        owner_carrier_index,
                        CurveError::Topology(
                            "an exterior classification contact resolved to its probe carrier"
                                .into(),
                        ),
                    ));
                };
                let Some(boundary) = boundary_carriers.get(boundary_index) else {
                    return Err(self.invalid(
                        owner_carrier_index,
                        CurveError::Topology(
                            "an exterior classification contact lost its boundary carrier".into(),
                        ),
                    ));
                };
                cross_is_positive ^= boundary.reversed;
                crossings.push((
                    contact.first_parameter(),
                    cross_is_positive,
                    boundary.loop_index,
                ));
            }
            if ambiguous {
                continue;
            }
            match classification_kind {
                RetainedPointProbeClassification::FilledRegion => {
                    // Direct point classification, boundary incidence and an
                    // exterior probe with no crossings need no orientation
                    // theorem. Request cached filled sides only for this
                    // retained-point face decision.
                    let filled_sides = if crossings.is_empty() {
                        &[][..]
                    } else {
                        match boundary_region
                            .filled_side_is_left_raw(&self.data.policy)
                            .map_err(|cause| self.invalid(owner_carrier_index, cause))?
                        {
                            Classification::Decided(sides) => sides,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    };
                    let mut last_contact = None::<(&CurveParameter2, RegionPointLocation)>;
                    for (parameter, cross_is_positive, loop_index) in crossings {
                        let Some(&filled_side_is_left) = filled_sides.get(loop_index) else {
                            return Err(self.invalid(
                                owner_carrier_index,
                                CurveError::Topology(
                                    "an exterior classification contact lost its loop filled side"
                                        .into(),
                                ),
                            ));
                        };
                        let target_is_left_of_boundary = !cross_is_positive;
                        let location = if target_is_left_of_boundary == filled_side_is_left {
                            RegionPointLocation::Inside
                        } else {
                            RegionPointLocation::Outside
                        };
                        let replace = match last_contact {
                            None => true,
                            Some((previous_parameter, previous_location)) => match parameter
                                .cmp_by_refinement(previous_parameter, &self.data.policy)
                                .map_err(|cause| self.invalid(owner_carrier_index, cause))?
                            {
                                Classification::Decided(Ordering::Greater) => true,
                                Classification::Decided(Ordering::Less) => false,
                                Classification::Decided(Ordering::Equal)
                                    if previous_location == location =>
                                {
                                    false
                                }
                                Classification::Decided(Ordering::Equal) => {
                                    last_reason = UncertaintyReason::Boundary;
                                    ambiguous = true;
                                    break;
                                }
                                Classification::Uncertain(reason) => {
                                    last_reason = reason;
                                    ambiguous = true;
                                    break;
                                }
                            },
                        };
                        if replace {
                            last_contact = Some((parameter, location));
                        }
                    }
                    if ambiguous {
                        continue;
                    }
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "retained-point-probe",
                        match last_contact {
                            Some((_, RegionPointLocation::Inside)) => "inside",
                            Some((_, RegionPointLocation::Outside)) | None => "outside",
                            Some((_, RegionPointLocation::Boundary)) => unreachable!(),
                        },
                    );
                    return Ok(Classification::Decided(
                        last_contact.map_or(RegionPointLocation::Outside, |(_, location)| location),
                    ));
                }
                RetainedPointProbeClassification::LoopParity => {
                    for index in 1..crossings.len() {
                        let mut cursor = index;
                        while cursor > 0 {
                            match crossings[cursor]
                                .0
                                .cmp_by_refinement(crossings[cursor - 1].0, &self.data.policy)
                                .map_err(|cause| self.invalid(owner_carrier_index, cause))?
                            {
                                Classification::Decided(Ordering::Less) => {
                                    crossings.swap(cursor, cursor - 1);
                                    cursor -= 1;
                                }
                                Classification::Decided(Ordering::Equal | Ordering::Greater) => {
                                    break;
                                }
                                Classification::Uncertain(reason) => {
                                    last_reason = reason;
                                    ambiguous = true;
                                    break;
                                }
                            }
                        }
                        if ambiguous {
                            break;
                        }
                    }
                    if ambiguous {
                        continue;
                    }

                    let mut inside = false;
                    let mut group_start = 0_usize;
                    while group_start < crossings.len() {
                        let mut group_end = group_start + 1;
                        while group_end < crossings.len() {
                            match crossings[group_end]
                                .0
                                .cmp_by_refinement(crossings[group_start].0, &self.data.policy)
                                .map_err(|cause| self.invalid(owner_carrier_index, cause))?
                            {
                                Classification::Decided(Ordering::Equal) => group_end += 1,
                                Classification::Decided(Ordering::Greater) => break,
                                Classification::Decided(Ordering::Less) => {
                                    return Err(self.invalid(
                                        owner_carrier_index,
                                        CurveError::Topology(
                                            "retained loop probe crossings lost exact order".into(),
                                        ),
                                    ));
                                }
                                Classification::Uncertain(reason) => {
                                    last_reason = reason;
                                    ambiguous = true;
                                    break;
                                }
                            }
                        }
                        if ambiguous {
                            break;
                        }
                        let group = &crossings[group_start..group_end];
                        if group.len() > 2 {
                            last_reason = UncertaintyReason::Boundary;
                            ambiguous = true;
                            break;
                        }
                        let has_positive = group.iter().any(|(_, positive, _)| *positive);
                        let has_negative = group.iter().any(|(_, positive, _)| !*positive);
                        // A single transverse image, or the same oriented
                        // tangent on both sides of a split vertex, crosses the
                        // loop once. Opposite vertex tangents are a touch and
                        // leave parity unchanged.
                        if !(has_positive && has_negative) {
                            inside = !inside;
                        }
                        group_start = group_end;
                    }
                    if ambiguous {
                        continue;
                    }
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "retained-loop-parity-probe",
                        if inside { "inside" } else { "outside" },
                    );
                    return Ok(Classification::Decided(if inside {
                        RegionPointLocation::Inside
                    } else {
                        RegionPointLocation::Outside
                    }));
                }
            }
        }

        Ok(Classification::Uncertain(last_reason))
    }

    fn carrier_bounds_are_outside_other_region(&self, carrier_index: usize) -> bool {
        // This entire path is an optional fragment-classification shortcut.
        // If exact bounds cannot separate the operands, the authoritative
        // point/region classifier below must get the decision.
        self.data.policy.strict_predicate_pass(|| {
            let carrier = &self.data.carriers[carrier_index];
            let other_operand = match carrier.operand {
                CurveRegionBooleanOperand2::First => 1,
                CurveRegionBooleanOperand2::Second => 0,
            };
            // Each operand/refinement owns one lazy envelope. Rebuilding this
            // union for every fragment is another Cartesian scan, even after
            // the pair broad phase has discarded all distant components.
            let other_bounds = self.data.operand_bounds[other_operand].get_or_init(Box::default);
            // When both sides' envelopes are exact at level zero, refined
            // levels repeat the same boxes; this optional shortcut stops there.
            let invariant = carrier_bounds_refinement_invariant(carrier)
                && *other_bounds.refinement_invariant.get_or_init(|| {
                    self.data
                        .carriers
                        .iter()
                        .filter(|other| other.operand != carrier.operand)
                        .all(carrier_bounds_refinement_invariant)
                });
            // Set once a decided comparison has overlapped; refined levels of
            // invariant envelopes would repeat it.
            let mut decided_overlap = false;
            CARRIER_BOUND_REFINEMENTS
                .into_iter()
                .enumerate()
                .any(|(level, refinement_steps)| {
                    if invariant && decided_overlap {
                        return false;
                    }
                    let Classification::Decided(carrier_bounds) =
                        carrier_optional_outer_bounds_refined(
                            carrier,
                            refinement_steps,
                            &self.data.policy,
                        )
                    else {
                        return false;
                    };
                    let cell = &other_bounds.refinements[level];
                    let other_bounds = if let Some(bounds) = cell.get() {
                        bounds
                    } else {
                        let mut accumulated = None::<Aabb2>;
                        for other in &self.data.carriers {
                            if other.operand == carrier.operand {
                                continue;
                            }
                            let bounds = match carrier_optional_outer_bounds_refined(
                                other,
                                refinement_steps,
                                &self.data.policy,
                            ) {
                                Classification::Decided(bounds) => bounds,
                                Classification::Uncertain(_) => {
                                    // Unavailable evidence may become decidable
                                    // after another exact kernel refines it.
                                    return false;
                                }
                            };
                            accumulated = Some(match accumulated {
                                None => bounds,
                                Some(previous) => match previous.union(&bounds) {
                                    Classification::Decided(bounds) => bounds,
                                    Classification::Uncertain(_) => {
                                        return false;
                                    }
                                },
                            });
                        }
                        let _ = cell.set(accumulated);
                        cell.get().expect("the exact operand envelope was retained")
                    };
                    let disjoint = other_bounds.as_ref().is_none_or(|other_bounds| {
                        carrier_bounds.overlaps(other_bounds, &self.data.policy)
                            == Classification::Decided(false)
                    });
                    decided_overlap = !disjoint;
                    disjoint
                })
        })
    }

    fn fragment_representative(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
    ) -> ExactCurveResult<(crate::Real, crate::Point2)> {
        let carrier = &self.data.carriers[carrier_index];
        let Some((start, end)) = fragment_range(fragment) else {
            return Err(self.blocked(carrier_index, UncertaintyReason::Unsupported));
        };
        let parameter = match start
            .strict_scalar_between_ordered(end, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        let representative = match carrier
            .geometry
            .point_at(&parameter, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        Ok((parameter, representative))
    }

    fn fragment_action(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        location: RegionPointLocation,
        overlaps: &[CarrierOverlap],
        operation: BooleanOp,
    ) -> ExactCurveResult<RegionFragmentAction> {
        let carrier = &self.data.carriers[carrier_index];
        match location {
            RegionPointLocation::Inside => Ok(action_for_sides(
                operation,
                carrier.operand,
                carrier.filled_side_is_left,
                true,
            )),
            RegionPointLocation::Outside => Ok(action_for_sides(
                operation,
                carrier.operand,
                carrier.filled_side_is_left,
                false,
            )),
            RegionPointLocation::Boundary => {
                self.shared_fragment_action(carrier_index, fragment, overlaps, operation)
            }
        }
    }

    fn shared_fragment_action(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        overlaps: &[CarrierOverlap],
        operation: BooleanOp,
    ) -> ExactCurveResult<RegionFragmentAction> {
        let fragment_range = fragment.curve_region_parameter_range();
        let (start, end) = (fragment_range.start(), fragment_range.end());
        let mut matching_overlap = None;
        for overlap in overlaps {
            let range = if overlap.first_carrier_index == carrier_index {
                Some(&overlap.first_range)
            } else if overlap.second_carrier_index == carrier_index {
                Some(&overlap.second_range)
            } else {
                None
            };
            if let Some(range) = range
                && range_contains_fragment(range, start, end, &self.data.policy)?
            {
                matching_overlap = Some(overlap);
                break;
            }
        }
        let Some(overlap) = matching_overlap else {
            return Err(self.blocked(carrier_index, UncertaintyReason::Boundary));
        };
        if carrier_index >= self.data.first_carrier_count {
            return Ok(RegionFragmentAction::Discard);
        }
        let first = &self.data.carriers[overlap.first_carrier_index];
        let second = &self.data.carriers[overlap.second_carrier_index];
        let same_source_direction = overlap.orientation == CurveOverlapOrientation2::Same;
        let same_traversal = same_source_direction == (first.reversed == second.reversed);
        if let Some(action) =
            self.shared_algebraic_chord_action(overlap, fragment, same_traversal, operation)?
        {
            return Ok(action);
        }
        let second_left_in_first_direction = if same_traversal {
            second.filled_side_is_left
        } else {
            !second.filled_side_is_left
        };
        let left = operation.apply(first.filled_side_is_left, second_left_in_first_direction);
        let right = operation.apply(!first.filled_side_is_left, !second_left_in_first_direction);
        Ok(action_from_result_sides(left, right))
    }

    /// Decides coincident retained chords from the actual global occupancy on
    /// both sides of their shared image. Loop-local `filled_side_is_left`
    /// remains valid for a simple regularized boundary, but it cannot decide
    /// a span made internal by another loop of the same operand. The boundary
    /// side-ray kernel skips each owning source fragment at the common point,
    /// so no finite epsilon or inexact displacement is introduced.
    fn shared_algebraic_chord_action(
        &self,
        overlap: &CarrierOverlap,
        first_fragment: &BezierSplitFragment2,
        same_traversal: bool,
        operation: BooleanOp,
    ) -> ExactCurveResult<Option<RegionFragmentAction>> {
        let BezierSplitFragment2::AlgebraicChord(first_chord) = first_fragment else {
            return Ok(None);
        };
        if !matches!(
            self.data.carriers[overlap.second_carrier_index].geometry,
            CurveSupport2::Line(_)
        ) {
            return Ok(None);
        }
        let representative = match first_chord
            .representative_point(&self.data.policy)
            .map_err(|cause| self.invalid(overlap.first_carrier_index, cause))?
        {
            Classification::Decided(representative) => representative,
            Classification::Uncertain(first_reason) => {
                let CurveSupport2::Line(second_chord) =
                    &self.data.carriers[overlap.second_carrier_index].geometry
                else {
                    unreachable!("shared algebraic chord action validated its second carrier")
                };
                let (Some(second_start), Some(second_end)) = (
                    overlap.second_range.start().as_algebraic_chord(),
                    overlap.second_range.end().as_algebraic_chord(),
                ) else {
                    return Err(self.blocked(overlap.first_carrier_index, first_reason));
                };
                let second_fragment =
                    crate::BezierAlgebraicChord2::from_certified_ordered_parameter_range(
                        second_chord,
                        second_start,
                        second_end,
                        &self.data.policy,
                    )
                    .map_err(|cause| self.invalid(overlap.second_carrier_index, cause))?;
                match second_fragment
                    .representative_point(&self.data.policy)
                    .map_err(|cause| self.invalid(overlap.second_carrier_index, cause))?
                {
                    Classification::Decided(representative) => representative,
                    Classification::Uncertain(reason) => {
                        return Err(self.blocked(overlap.second_carrier_index, reason));
                    }
                }
            }
        };
        let location_is_inside = |carrier_index: usize,
                                  classification: (Vec<i32>, RegionPointLocation)|
         -> ExactCurveResult<bool> {
            match classification.1 {
                RegionPointLocation::Inside => Ok(true),
                RegionPointLocation::Outside => Ok(false),
                RegionPointLocation::Boundary => {
                    Err(self.blocked(carrier_index, UncertaintyReason::Boundary))
                }
            }
        };
        let certified_regularized_sides =
            |carrier_index: usize, source_follows_reference_tangent: bool| {
                let region = self.region_for_carrier(carrier_index);
                if !region.has_regularized_filled_left_topology(&self.data.policy) {
                    return None;
                }
                let source_left_is_inside = self.data.carriers[carrier_index].filled_side_is_left;
                let reference_left_is_inside = if source_follows_reference_tangent {
                    source_left_is_inside
                } else {
                    !source_left_is_inside
                };
                Some([reference_left_is_inside, !reference_left_is_inside])
            };
        let (first_sides, second_sides) = match representative {
            CurvePoint2(CurvePointData2::Exact(point)) => {
                let (tangent_x, tangent_y) = first_chord
                    .exact_line()
                    .map(|line| line.delta())
                    .or_else(|| {
                        first_chord
                            .strict_provenance_support_line(&self.data.policy)
                            .map(|line| line.delta())
                    })
                    .or_else(|| first_chord.certified_unit_tangent())
                    .ok_or_else(|| {
                        self.blocked(overlap.first_carrier_index, UncertaintyReason::Unsupported)
                    })?;
                let classify = |carrier_index,
                                source_follows_reference_tangent|
                 -> ExactCurveResult<[bool; 2]> {
                    if let Some(sides) =
                        certified_regularized_sides(carrier_index, source_follows_reference_tangent)
                    {
                        return Ok(sides);
                    }
                    let classify_side = |left| {
                        self.fragment_side_classification_with_reference_tangent(
                            carrier_index,
                            &point,
                            None,
                            &tangent_x,
                            &tangent_y,
                            left,
                            source_follows_reference_tangent,
                        )
                        .and_then(|classification| {
                            location_is_inside(carrier_index, classification)
                        })
                    };
                    Ok([classify_side(true)?, classify_side(false)?])
                };
                (
                    classify(overlap.first_carrier_index, true)?,
                    classify(overlap.second_carrier_index, same_traversal)?,
                )
            }
            CurvePoint2(CurvePointData2::Algebraic(point)) => {
                let tangent = |axis| {
                    first_chord
                        .tangent_axis_sign(axis, &self.data.policy)
                        .map_err(|cause| self.invalid(overlap.first_carrier_index, cause))
                };
                let [tangent_x, tangent_y] = [tangent(Axis2::X)?, tangent(Axis2::Y)?];
                let classify = |carrier_index,
                                source_follows_reference_tangent|
                 -> ExactCurveResult<[bool; 2]> {
                    if let Some(sides) =
                        certified_regularized_sides(carrier_index, source_follows_reference_tangent)
                    {
                        return Ok(sides);
                    }
                    let classify_side = |left| {
                        self.algebraic_fragment_side_classification(
                            carrier_index,
                            &point,
                            tangent_x,
                            tangent_y,
                            left,
                        )
                        .and_then(|classification| {
                            location_is_inside(carrier_index, classification)
                        })
                    };
                    let source_sides = [classify_side(true)?, classify_side(false)?];
                    Ok(if source_follows_reference_tangent {
                        source_sides
                    } else {
                        [source_sides[1], source_sides[0]]
                    })
                };
                (
                    classify(overlap.first_carrier_index, true)?,
                    classify(overlap.second_carrier_index, same_traversal)?,
                )
            }
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_))
            | CurvePoint2(CurvePointData2::AlgebraicChordParallel(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChord(_))
            | CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_))
            | CurvePoint2(CurvePointData2::AnalyticParallel(_))
            | CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => {
                return Ok(None);
            }
        };
        Ok(Some(action_from_result_sides(
            operation.apply(first_sides[0], second_sides[0]),
            operation.apply(first_sides[1], second_sides[1]),
        )))
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
